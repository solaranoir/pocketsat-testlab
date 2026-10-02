"""Telemetry payload codec (#54): conversions, limits, round trips, flags, and docs."""

import dataclasses
import math
import random
import re
import struct
from decimal import ROUND_HALF_UP, Decimal
from pathlib import Path
from typing import Any

import pytest

from pocketsat.flight import Mode
from pocketsat.messages import (
    ATTITUDE_STATE_CODES,
    BUS_V_SCALE,
    INT16_MAX,
    INT16_MIN,
    KNOWN_FLAGS_MASK,
    PAYLOAD_STATE_CODES,
    POINTING_CENTI_DEG_MAX,
    POINTING_SCALE,
    RADIO_MODE_CODES,
    SOC_PERMILLE_MAX,
    SOC_SCALE,
    TELEMETRY_FIELDS,
    TELEMETRY_FLAG_GROUPS,
    TELEMETRY_FORMAT,
    TELEMETRY_FRAME_SIZE,
    TELEMETRY_PAYLOAD_SIZE,
    TEMPERATURE_SCALE,
    UINT16_MAX,
    UINT32_MAX,
    FlightComputerTelemetryState,
    Telemetry,
    TelemetryFlags,
    decode_telemetry,
    encode_telemetry,
    quantize,
)
from pocketsat.spacecraft.controls import RadioMode
from pocketsat.spacecraft.snapshots import (
    READINGS_FLAGS,
    AttitudeReadings,
    AttitudeState,
    CommsReadings,
    PayloadReadings,
    PayloadState,
    PowerReadings,
    PowerTruth,
    ThermalReadings,
)

PROTOCOL = (Path(__file__).parents[2] / "docs" / "protocol.md").read_text(encoding="utf-8")

FC = FlightComputerTelemetryState(uptime_ms=60_000, mode=Mode.NOMINAL, boot_count=2)
POWER = PowerReadings(
    bus_v=7.4, battery_current_a=0.5, soc=0.8, low_battery=False, critical_battery=False
)
THERMAL = ThermalReadings(battery_c=15.0, electronics_c=22.0, over_temp=False, under_temp=False)
ATTITUDE = AttitudeReadings(pointing_error_deg=2.0, rate_dps=0.05, state=AttitudeState.STABILIZED)
PAYLOAD = PayloadReadings(
    state=PayloadState.ACQUIRING,
    buffered_bytes=1000,
    buffer_capacity_bytes=524_288,
    oldest_unreleased_chunk_id=0,
    next_chunk_id=16,
    total_produced_bytes=1000,
    total_released_bytes=0,
    power_w=0.5,
)
COMMS = CommsReadings(
    radio_mode=RadioMode.RX_TX,
    receiver_on=True,
    transmitter_on=True,
    transmit_capacity_bytes=120,
    sent_bytes=0,
    uplink_lost_count=0,
    outbound_suppressed_count=0,
    transmit_power_w=0.15,
)


def encode(**changes: Any) -> bytes:
    """Encode the baseline inputs with fields replaced, for example ``bus_v=8.0``.

    ``attitude_state`` and ``payload_state`` set the ``state`` field of that record.
    """
    records: dict[str, Any] = {
        "power": POWER,
        "thermal": THERMAL,
        "attitude": ATTITUDE,
        "payload": PAYLOAD,
        "comms": COMMS,
    }
    fc = FC
    for name, value in changes.items():
        if name in {"attitude_state", "payload_state"}:
            owner = name.removesuffix("_state")
            records[owner] = dataclasses.replace(records[owner], state=value)
            continue
        if name in {f.name for f in dataclasses.fields(FC)}:
            fc = dataclasses.replace(fc, **{name: value})
            continue
        owners = [k for k, r in records.items() if name in {f.name for f in dataclasses.fields(r)}]
        assert len(owners) == 1, name
        records[owners[0]] = dataclasses.replace(records[owners[0]], **{name: value})
    return encode_telemetry(fc, **records)


def round_trip(**changes: Any) -> Telemetry:
    return decode_telemetry(encode(**changes))


# --- Sizes and layout ------------------------------------------------------------------


def test_payload_and_frame_size() -> None:
    assert TELEMETRY_PAYLOAD_SIZE == 26
    assert TELEMETRY_FRAME_SIZE == 36
    assert len(encode()) == TELEMETRY_PAYLOAD_SIZE


def test_fields_are_naturally_aligned_with_no_padding() -> None:
    offset = 0
    for name, code in TELEMETRY_FIELDS:
        size = struct.calcsize(">" + code)
        assert offset % size == 0, name
        offset += size
    assert offset == TELEMETRY_PAYLOAD_SIZE
    assert TELEMETRY_FORMAT.startswith(">")


# --- quantize --------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value", "scale", "minimum", "maximum", "expected"),
    [
        (7.4, 1000, 0, UINT16_MAX, 7400),
        (7.0005, 1000, 0, UINT16_MAX, 7001),  # product 7000.5: half rounds up
        (-0.005, 100, INT16_MIN, INT16_MAX, -1),  # -0.5: half rounds away from zero
        (-0.004, 100, INT16_MIN, INT16_MAX, 0),
        (2.675, 100, INT16_MIN, INT16_MAX, 268),  # double product is 267.5
        (1.005, 100, INT16_MIN, INT16_MAX, 100),  # double product is 100.4999...
        (65.535, 1000, 0, UINT16_MAX, UINT16_MAX),
        (65.5355, 1000, 0, UINT16_MAX, UINT16_MAX),
        (1e300, 1000, 0, UINT16_MAX, UINT16_MAX),
        (-1e300, 100, INT16_MIN, INT16_MAX, INT16_MIN),
        (math.inf, 100, INT16_MIN, INT16_MAX, INT16_MAX),
        (-math.inf, 100, INT16_MIN, INT16_MAX, INT16_MIN),
        (-0.0, 1000, 0, UINT16_MAX, 0),
        (5e-324, 1000, 0, UINT16_MAX, 0),
        (3, 1000, 0, UINT16_MAX, 3000),  # an int is scaled exactly
        (10**30, 1000, 0, UINT16_MAX, UINT16_MAX),
    ],
)
def test_quantize_examples(
    value: float, scale: int, minimum: int, maximum: int, expected: int
) -> None:
    assert quantize(value, scale, minimum, maximum) == expected


def test_quantize_rejects_nan() -> None:
    with pytest.raises(ValueError, match="NaN"):
        quantize(math.nan, 1000, 0, UINT16_MAX)


@pytest.mark.parametrize("value", [True, "1.0", None])
def test_quantize_rejects_non_numbers(value: Any) -> None:
    with pytest.raises(TypeError):
        quantize(value, 1000, 0, UINT16_MAX)


def _reference(value: float, scale: int, minimum: int, maximum: int) -> int:
    # Independent reference: exact decimal rounding of the IEEE double product.
    product = Decimal(value * scale)
    rounded = int(product.quantize(Decimal(1), rounding=ROUND_HALF_UP))
    return min(max(rounded, minimum), maximum)


@pytest.mark.parametrize(
    ("scale", "minimum", "maximum", "low", "high"),
    [
        (BUS_V_SCALE, 0, UINT16_MAX, -1.0, 70.0),
        (SOC_SCALE, 0, SOC_PERMILLE_MAX, -0.1, 1.1),
        (TEMPERATURE_SCALE, INT16_MIN, INT16_MAX, -340.0, 340.0),
        (POINTING_SCALE, 0, POINTING_CENTI_DEG_MAX, -1.0, 181.0),
    ],
)
def test_quantize_matches_decimal_reference(
    scale: int, minimum: int, maximum: int, low: float, high: float
) -> None:
    rng = random.Random(54)
    values = [rng.uniform(low, high) for _ in range(5000)]
    # Exact halves and values one ulp either side of them.
    for n in range(-200, 200):
        half = (n + 0.5) / scale
        values += [half, math.nextafter(half, math.inf), math.nextafter(half, -math.inf)]
    for value in values:
        assert quantize(value, scale, minimum, maximum) == _reference(
            value, scale, minimum, maximum
        ), value


@pytest.mark.parametrize(
    ("scale", "minimum", "maximum"),
    [
        (BUS_V_SCALE, 0, UINT16_MAX),
        (SOC_SCALE, 0, SOC_PERMILLE_MAX),
        (TEMPERATURE_SCALE, INT16_MIN, INT16_MAX),
        (POINTING_SCALE, 0, POINTING_CENTI_DEG_MAX),
    ],
)
def test_every_wire_value_survives_decode_and_reencode(
    scale: int, minimum: int, maximum: int
) -> None:
    for raw in range(minimum, maximum + 1):
        assert quantize(raw / scale, scale, minimum, maximum) == raw


# --- Round trips at and beyond the limits ----------------------------------------------


@pytest.mark.parametrize(
    ("field", "value", "decoded"),
    [
        # bus_v: 0 .. 65.535 V in 1 mV
        ("bus_v", 7.4, 7.4),
        ("bus_v", 7.40049, 7.4),
        ("bus_v", 0.0, 0.0),
        ("bus_v", 65.535, 65.535),
        ("bus_v", 65.536, 65.535),
        ("bus_v", 1000.0, 65.535),
        ("bus_v", -0.001, 0.0),
        ("bus_v", math.inf, 65.535),
        ("bus_v", -math.inf, 0.0),
        # soc: 0 .. 1 in 0.001
        ("soc", 0.8, 0.8),
        ("soc", 0.0, 0.0),
        ("soc", 1.0, 1.0),
        ("soc", 1.0004, 1.0),
        ("soc", 1.5, 1.0),
        ("soc", -0.2, 0.0),
        # temperatures: -327.68 .. 327.67 °C in 0.01 °C
        ("battery_c", -20.25, -20.25),
        ("battery_c", 327.67, 327.67),
        ("battery_c", 327.68, 327.67),
        ("battery_c", -327.68, -327.68),
        ("battery_c", -327.69, -327.68),
        ("battery_c", -math.inf, -327.68),
        ("electronics_c", 60.0, 60.0),
        ("electronics_c", 500.0, 327.67),
        ("electronics_c", -500.0, -327.68),
        ("electronics_c", math.inf, 327.67),
        # pointing error: 0 .. 180° in 0.01°
        ("pointing_error_deg", 12.34, 12.34),
        ("pointing_error_deg", 0.0, 0.0),
        ("pointing_error_deg", 180.0, 180.0),
        ("pointing_error_deg", 180.01, 180.0),
        ("pointing_error_deg", -0.01, 0.0),
    ],
)
def test_physical_round_trip(field: str, value: float, decoded: float) -> None:
    assert getattr(round_trip(**{field: value}), field) == decoded


@pytest.mark.parametrize(
    ("value", "decoded"),
    [(0, 0), (524_288, 524_288), (UINT32_MAX, UINT32_MAX), (UINT32_MAX + 1, UINT32_MAX)],
)
def test_buffered_bytes_round_trip_saturates(value: int, decoded: int) -> None:
    assert round_trip(buffered_bytes=value).buffered_bytes == decoded


@pytest.mark.parametrize(
    ("value", "decoded"),
    [(0, 0), (UINT32_MAX, UINT32_MAX), (UINT32_MAX + 1, 0), (UINT32_MAX + 1001, 1000)],
)
def test_uptime_wraps_modulo_2_32(value: int, decoded: int) -> None:
    assert round_trip(uptime_ms=value).uptime_ms == decoded


@pytest.mark.parametrize(
    ("value", "decoded"), [(0, 0), (UINT16_MAX, UINT16_MAX), (UINT16_MAX + 1, UINT16_MAX)]
)
def test_boot_count_saturates(value: int, decoded: int) -> None:
    assert round_trip(boot_count=value).boot_count == decoded


@pytest.mark.parametrize("mode", list(Mode))
def test_mode_round_trip(mode: Mode) -> None:
    assert round_trip(mode=mode).mode is mode


@pytest.mark.parametrize("radio_mode", list(RadioMode))
def test_radio_mode_round_trip(radio_mode: RadioMode) -> None:
    assert round_trip(radio_mode=radio_mode).radio_mode is radio_mode


@pytest.mark.parametrize("state", list(AttitudeState))
def test_attitude_state_round_trip(state: AttitudeState) -> None:
    assert round_trip(attitude_state=state).attitude_state is state


@pytest.mark.parametrize("state", list(PayloadState))
def test_payload_state_round_trip(state: PayloadState) -> None:
    assert round_trip(payload_state=state).payload_state is state


def test_decoded_telemetry_reencodes_to_the_same_bytes() -> None:
    payload = encode(bus_v=7.81234, soc=0.4567, battery_c=-3.333, pointing_error_deg=91.119)
    t = decode_telemetry(payload)
    again = encode_telemetry(
        FlightComputerTelemetryState(t.uptime_ms, t.mode, t.boot_count),
        power=dataclasses.replace(
            POWER,
            bus_v=t.bus_v,
            soc=t.soc,
            low_battery=TelemetryFlags.low_battery in t.flags,
            critical_battery=TelemetryFlags.critical_battery in t.flags,
        ),
        thermal=dataclasses.replace(
            THERMAL,
            battery_c=t.battery_c,
            electronics_c=t.electronics_c,
            over_temp=TelemetryFlags.over_temp in t.flags,
            under_temp=TelemetryFlags.under_temp in t.flags,
        ),
        attitude=dataclasses.replace(
            ATTITUDE, pointing_error_deg=t.pointing_error_deg, state=t.attitude_state
        ),
        payload=dataclasses.replace(
            PAYLOAD, buffered_bytes=t.buffered_bytes, state=t.payload_state
        ),
        comms=dataclasses.replace(COMMS, radio_mode=t.radio_mode),
    )
    assert again == payload


# --- Non-finite and invalid inputs -----------------------------------------------------


@pytest.mark.parametrize(
    "field", ["bus_v", "soc", "battery_c", "electronics_c", "pointing_error_deg"]
)
def test_nan_reading_is_rejected(field: str) -> None:
    with pytest.raises(ValueError, match="NaN"):
        encode(**{field: math.nan})


def test_truth_record_is_rejected_at_run_time() -> None:
    truth = PowerTruth(
        bus_v=7.4, battery_current_a=0.5, soc=0.8, generation_w=3.0, total_load_w=2.5
    )
    with pytest.raises(TypeError, match="PowerReadings"):
        encode_telemetry(
            FC,
            power=truth,  # type: ignore[arg-type]
            thermal=THERMAL,
            attitude=ATTITUDE,
            payload=PAYLOAD,
            comms=COMMS,
        )


@pytest.mark.parametrize(
    ("field", "value"),
    [("low_battery", 1), ("over_temp", None), ("buffered_bytes", 1.0), ("buffered_bytes", True)],
)
def test_wrongly_typed_reading_is_rejected(field: str, value: Any) -> None:
    with pytest.raises(TypeError, match=field):
        encode(**{field: value})


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"uptime_ms": -1}, ValueError),
        ({"boot_count": -1}, ValueError),
        ({"mode": 1}, TypeError),
        ({"mode": 6}, TypeError),
        ({"mode": True}, TypeError),
        ({"mode": "NOMINAL"}, TypeError),
        ({"uptime_ms": 1.0}, TypeError),
        ({"boot_count": "1"}, TypeError),
    ],
)
def test_flight_computer_state_is_validated(kwargs: dict[str, Any], error: type[Exception]) -> None:
    base: dict[str, Any] = {"uptime_ms": 0, "mode": Mode.BOOT, "boot_count": 0}
    with pytest.raises(error):
        FlightComputerTelemetryState(**(base | kwargs))


WIRE_MODE_VALUES = {"BOOT": 0, "NOMINAL": 1, "SCIENCE": 2, "DOWNLINK": 3, "SAFE": 4, "FAULT": 5}
"""The telemetry ``mode`` wire values (docs/protocol.md, #54), written out by hand so
that renumbering ``Mode`` fails here instead of silently changing the wire format."""


@pytest.mark.parametrize("mode", list(Mode), ids=lambda m: m.name)
def test_every_mode_value_is_its_telemetry_wire_value(mode: Mode) -> None:
    # #101: telemetry carries Mode itself, so the enum value and the wire byte can't drift.
    wire = struct.unpack(TELEMETRY_FORMAT, encode(mode=mode))[3]
    assert wire == mode.value == WIRE_MODE_VALUES[mode.name]
    assert decode_telemetry(encode(mode=mode)).mode is mode


def test_wire_mode_values_name_every_mode_once() -> None:
    assert {m.name for m in Mode} == set(WIRE_MODE_VALUES)
    assert sorted(WIRE_MODE_VALUES.values()) == list(range(len(Mode)))


# --- Flags -----------------------------------------------------------------------------


def test_flag_names_are_the_readings_flag_names() -> None:
    readings_flags = {name for names in READINGS_FLAGS.values() for name in names}
    assert {flag.name for flag in TelemetryFlags} == readings_flags


def test_each_flag_sits_in_its_subsystems_group() -> None:
    for subsystem, names in READINGS_FLAGS.items():
        for name in names:
            bit = TelemetryFlags[name].bit_length() - 1
            assert TelemetryFlags[name] == 1 << bit
            assert bit in TELEMETRY_FLAG_GROUPS[subsystem], name


def test_flag_groups_cover_the_uint16_once() -> None:
    bits = sorted({bit for group in TELEMETRY_FLAG_GROUPS.values() for bit in group})
    assert bits == list(range(16))
    assert TELEMETRY_FLAG_GROUPS["payload"] == TELEMETRY_FLAG_GROUPS["comms"]
    assert sum(int(flag) for flag in TelemetryFlags) == KNOWN_FLAGS_MASK


@pytest.mark.parametrize("flag", list(TelemetryFlags), ids=lambda f: str(f.name))
def test_each_readings_flag_sets_only_its_bit(flag: TelemetryFlags) -> None:
    payload = encode(**{str(flag.name): True})
    raw = struct.unpack(TELEMETRY_FORMAT, payload)[2]
    assert raw == flag
    assert decode_telemetry(payload).flags == flag


def test_all_and_no_flags() -> None:
    every = {str(flag.name): True for flag in TelemetryFlags}
    assert round_trip(**every).flags == KNOWN_FLAGS_MASK
    assert round_trip().flags == TelemetryFlags(0)


# --- docs/protocol.md agrees with the code ---------------------------------------------


def _table_after(heading: str) -> list[list[str]]:
    """Rows of the first Markdown table after ``heading``, header and rule excluded."""
    start = PROTOCOL.index(heading)
    rows: list[list[str]] = []
    for line in PROTOCOL[start:].splitlines()[1:]:
        if line.startswith("|"):
            rows.append([cell.strip() for cell in line.strip().strip("|").split("|")])
        elif rows:
            break
    return rows[2:]


def _code(cell: str) -> str:
    match = re.fullmatch(r"`([^`]+)`", cell)
    assert match, cell
    return match.group(1)


def test_docs_flag_table_matches_the_enum() -> None:
    rows = _table_after("### Flag bits")
    assert [int(row[0]) for row in rows] == list(range(16))
    documented: dict[str, int] = {}
    for bit, mask, _group, flag in rows:
        assert int(_code(mask), 16) == 1 << int(bit)
        if flag != "reserved":
            documented[_code(flag)] = int(bit)
    assert documented == {str(f.name): f.bit_length() - 1 for f in TelemetryFlags}


def test_docs_flag_groups_match() -> None:
    names = {"Power": "power", "Thermal": "thermal", "Attitude": "attitude"}
    for bit, _mask, group, _flag in _table_after("### Flag bits"):
        subsystem = names.get(group, "payload")
        assert int(bit) in TELEMETRY_FLAG_GROUPS[subsystem], (bit, group)


_TYPES = {"uint32": "I", "uint16": "H", "int16": "h", "uint8": "B"}

_SOURCES: dict[str, type[Any]] = {
    "FlightComputerTelemetryState": FlightComputerTelemetryState,
    "PowerReadings": PowerReadings,
    "ThermalReadings": ThermalReadings,
    "AttitudeReadings": AttitudeReadings,
    "PayloadReadings": PayloadReadings,
    "CommsReadings": CommsReadings,
}


def test_docs_mapping_table_matches_the_layout() -> None:
    rows = _table_after("### Layout and mapping")
    documented = [(_code(row[1]), _TYPES[row[2]]) for row in rows]
    assert documented == list(TELEMETRY_FIELDS)
    offsets = [int(row[0]) for row in rows]
    sizes = [struct.calcsize(">" + code) for _, code in TELEMETRY_FIELDS]
    assert offsets == [sum(sizes[:i]) for i in range(len(sizes))]


def test_docs_mapping_table_names_real_readings_fields() -> None:
    for row in _table_after("### Layout and mapping"):
        match = re.match(r"`(\w+)\.(\w+)`", row[3])
        if match is None:
            assert _code(row[1]) == "flags"
            continue
        record, field = match.groups()
        assert field in {f.name for f in dataclasses.fields(_SOURCES[record])}, row[3]


@pytest.mark.parametrize(
    ("heading", "codes"),
    [
        ("#### Mode", {m.name: m.value for m in Mode}),
        ("#### Radio mode", {m.name: v for m, v in RADIO_MODE_CODES.items()}),
        ("#### Attitude state", {s.name: v for s, v in ATTITUDE_STATE_CODES.items()}),
        ("#### Payload state", {s.name: v for s, v in PAYLOAD_STATE_CODES.items()}),
    ],
)
def test_docs_state_tables_match(heading: str, codes: dict[str, int]) -> None:
    documented = {_code(name): int(value) for value, name in _table_after(heading)}
    assert documented == dict(codes)


def test_state_codes_cover_every_member() -> None:
    assert set(RADIO_MODE_CODES) == set(RadioMode)
    assert set(ATTITUDE_STATE_CODES) == set(AttitudeState)
    assert set(PAYLOAD_STATE_CODES) == set(PayloadState)
    for codes in (RADIO_MODE_CODES, ATTITUDE_STATE_CODES, PAYLOAD_STATE_CODES):
        assert sorted(codes.values()) == list(range(len(codes)))
    assert sorted(m.value for m in Mode) == list(range(6))


def test_docs_state_the_sizes() -> None:
    assert f"**{TELEMETRY_PAYLOAD_SIZE} bytes**" in PROTOCOL
    assert f"**{TELEMETRY_FRAME_SIZE} bytes**" in PROTOCOL
