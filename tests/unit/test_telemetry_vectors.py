"""Check the telemetry codec against the shared vectors in tests/vectors/telemetry.json."""

import json
import struct
from pathlib import Path
from typing import Any

import pytest

from pocketsat.flight import Mode
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    BUS_V_SCALE,
    POINTING_SCALE,
    SOC_SCALE,
    TELEMETRY_FIELDS,
    TELEMETRY_FORMAT,
    TELEMETRY_FRAME_SIZE,
    TELEMETRY_PAYLOAD_SIZE,
    TEMPERATURE_SCALE,
    FlightComputerTelemetryState,
    Telemetry,
    TelemetryDecodeError,
    TelemetryFlags,
    decode_telemetry,
    encode_telemetry,
)
from pocketsat.spacecraft.controls import RadioMode
from pocketsat.spacecraft.snapshots import (
    AttitudeReadings,
    AttitudeState,
    CommsReadings,
    PayloadReadings,
    PayloadState,
    PowerReadings,
    ThermalReadings,
)

VECTORS: dict[str, Any] = json.loads(
    (Path(__file__).parents[1] / "vectors" / "telemetry.json").read_text()
)

VALID: list[dict[str, Any]] = VECTORS["valid_telemetry"]


def _ids(key: str) -> list[str]:
    return [v["name"] for v in VECTORS[key]]


def _float(value: float | str) -> float:
    return float(value)


def encode_vector_input(inp: dict[str, Any]) -> bytes:
    """Encode a vector's ``input``, filling readings fields that don't go on the wire."""
    power, thermal = inp["power"], inp["thermal"]
    attitude, payload, comms = inp["attitude"], inp["payload"], inp["comms"]
    return encode_telemetry(
        FlightComputerTelemetryState(
            uptime_ms=inp["uptime_ms"], mode=Mode[inp["mode"]], boot_count=inp["boot_count"]
        ),
        power=PowerReadings(
            bus_v=_float(power["bus_v"]),
            battery_current_a=-0.25,
            soc=_float(power["soc"]),
            low_battery=power["low_battery"],
            critical_battery=power["critical_battery"],
        ),
        thermal=ThermalReadings(
            battery_c=_float(thermal["battery_c"]),
            electronics_c=_float(thermal["electronics_c"]),
            over_temp=thermal["over_temp"],
            under_temp=thermal["under_temp"],
        ),
        attitude=AttitudeReadings(
            pointing_error_deg=_float(attitude["pointing_error_deg"]),
            rate_dps=0.1,
            state=AttitudeState[attitude["state"]],
        ),
        payload=PayloadReadings(
            state=PayloadState[payload["state"]],
            buffered_bytes=payload["buffered_bytes"],
            buffer_capacity_bytes=524_288,
            oldest_unreleased_chunk_id=0,
            next_chunk_id=64,
            total_produced_bytes=4096,
            total_released_bytes=0,
            power_w=0.5,
        ),
        comms=CommsReadings(
            radio_mode=RadioMode[comms["radio_mode"]],
            receiver_on=True,
            transmitter_on=True,
            transmit_capacity_bytes=120,
            previous_tick_sent_bytes=0,
            uplink_lost_count=0,
            outbound_suppressed_count=0,
            transmit_power_w=0.15,
        ),
    )


def test_sizes() -> None:
    assert VECTORS["payload_size"] == TELEMETRY_PAYLOAD_SIZE == 26
    assert VECTORS["frame_size"] == TELEMETRY_FRAME_SIZE == 36


def test_vectors_cover_each_flag_alone_all_and_none() -> None:
    seen = {v["fields"]["flags"] for v in VALID}
    singles = {int(flag) for flag in TelemetryFlags}
    every = 0
    for bit in singles:
        every |= bit
    assert {0, every} | singles <= seen


@pytest.mark.parametrize("vector", VALID, ids=_ids("valid_telemetry"))
def test_encode(vector: dict[str, Any]) -> None:
    assert encode_vector_input(vector["input"]).hex() == vector["payload_hex"]


@pytest.mark.parametrize("vector", VALID, ids=_ids("valid_telemetry"))
def test_wire_fields(vector: dict[str, Any]) -> None:
    raw = struct.unpack(TELEMETRY_FORMAT, bytes.fromhex(vector["payload_hex"]))
    names = [name for name, _ in TELEMETRY_FIELDS]
    assert dict(zip(names, raw, strict=True)) == vector["fields"]


@pytest.mark.parametrize("vector", VALID, ids=_ids("valid_telemetry"))
def test_decode(vector: dict[str, Any]) -> None:
    fields = vector["fields"]
    decoded = decode_telemetry(bytes.fromhex(vector["payload_hex"]))
    assert decoded == Telemetry(
        uptime_ms=fields["uptime_ms"],
        boot_count=fields["boot_count"],
        flags=TelemetryFlags(fields["flags"]),
        mode=Mode(fields["mode"]),
        radio_mode=list(RadioMode)[fields["radio_mode"]],
        attitude_state=list(AttitudeState)[fields["attitude_state"]],
        payload_state=list(PayloadState)[fields["payload_state"]],
        bus_v=fields["bus_mv"] / BUS_V_SCALE,
        soc=fields["soc_permille"] / SOC_SCALE,
        battery_c=fields["battery_centi_c"] / TEMPERATURE_SCALE,
        electronics_c=fields["electronics_centi_c"] / TEMPERATURE_SCALE,
        buffered_bytes=fields["buffered_bytes"],
        pointing_error_deg=fields["pointing_error_centi_deg"] / POINTING_SCALE,
    )


@pytest.mark.parametrize("vector", VALID, ids=_ids("valid_telemetry"))
def test_decode_then_reencode_is_byte_identical(vector: dict[str, Any]) -> None:
    # #101 changed the mode field to ``Mode``; the wire bytes must not change.
    payload = bytes.fromhex(vector["payload_hex"])
    t = decode_telemetry(payload)
    assert t.mode is Mode[vector["input"]["mode"]]
    again = encode_vector_input(
        {
            "uptime_ms": t.uptime_ms,
            "mode": t.mode.name,
            "boot_count": t.boot_count,
            "power": {
                "bus_v": t.bus_v,
                "soc": t.soc,
                "low_battery": TelemetryFlags.low_battery in t.flags,
                "critical_battery": TelemetryFlags.critical_battery in t.flags,
            },
            "thermal": {
                "battery_c": t.battery_c,
                "electronics_c": t.electronics_c,
                "over_temp": TelemetryFlags.over_temp in t.flags,
                "under_temp": TelemetryFlags.under_temp in t.flags,
            },
            "attitude": {
                "pointing_error_deg": t.pointing_error_deg,
                "state": t.attitude_state.name,
            },
            "payload": {"buffered_bytes": t.buffered_bytes, "state": t.payload_state.name},
            "comms": {"radio_mode": t.radio_mode.name},
        }
    )
    assert again == payload


@pytest.mark.parametrize("vector", VALID, ids=_ids("valid_telemetry"))
def test_frame(vector: dict[str, Any]) -> None:
    payload = bytes.fromhex(vector["payload_hex"])
    raw = encode_frame(Frame(FrameType.TELEMETRY, sequence=vector["sequence"], payload=payload))
    assert raw.hex() == vector["frame_hex"]
    assert len(raw) == TELEMETRY_FRAME_SIZE
    assert int.from_bytes(raw[-2:], "big") == vector["crc"]
    frame = decode_frame(raw)
    assert frame.frame_type is FrameType.TELEMETRY
    assert decode_telemetry(frame.payload) == decode_telemetry(payload)


@pytest.mark.parametrize("vector", VECTORS["decoder_ignores"], ids=_ids("decoder_ignores"))
def test_decoder_ignores_reserved_flag_bits(vector: dict[str, Any]) -> None:
    decoded = decode_telemetry(bytes.fromhex(vector["payload_hex"]))
    assert decoded.flags == vector["decoded_flags"]


@pytest.mark.parametrize("vector", VECTORS["invalid_payloads"], ids=_ids("invalid_payloads"))
def test_decode_invalid(vector: dict[str, Any]) -> None:
    reason = "bytes" if vector["error"] == "size" else vector["error"]
    with pytest.raises(TelemetryDecodeError, match=reason):
        decode_telemetry(bytes.fromhex(vector["payload_hex"]))
