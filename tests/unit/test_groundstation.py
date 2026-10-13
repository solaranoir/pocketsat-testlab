"""Unit tests for the ground station framework (#152, ADR-0010 §1-§4)."""

import dataclasses
import typing

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.flight.modes import Mode, RejectReason
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    EVENT_TYPES,
    Aos,
    CommandAcked,
    CommandFailed,
    CommandNacked,
    CommandRef,
    CommandSent,
    CommandTimedOut,
    DataChunkReceived,
    FailureReason,
    FrameRejected,
    GroundStation,
    Los,
    RadioState,
    RadioStateChanged,
    RejectionReason,
    SequenceGap,
    SpacecraftRebooted,
    StationConfig,
    StationEvent,
    StationIdentified,
    TelemetryReceived,
    TelemetryStore,
    UnmatchedAck,
    Uplink,
)
from pocketsat.messages import (
    TELEMETRY_PAYLOAD_SIZE,
    Command,
    DecodeReason,
    Packet,
    decode_telemetry,
)

GEOMETRY = PassGeometry(
    azimuth_deg=120.0, elevation_deg=30.0, range_km=900.0, range_rate_km_per_s=-3.5
)

# --- Configuration ------------------------------------------------------------------


def test_default_config_is_the_documented_one() -> None:
    config = DEFAULT_STATION_CONFIG
    assert config == StationConfig()
    assert config.callsign == "N0CALL"
    assert (config.latitude_deg, config.longitude_deg, config.altitude_km) == (40.0, -105.0, 1.6)
    assert config.elevation_mask_deg == 10.0
    assert config.uplink_frequency_hz == 145_850_000
    assert config.downlink_frequency_hz == 435_500_000
    assert config.command_timeout_us == 2_000_000
    assert config.command_retry_count == 2
    assert config.identification_interval_us == 600_000_000


def test_config_is_frozen_and_replaceable() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        DEFAULT_STATION_CONFIG.callsign = "X"  # type: ignore[misc]
    changed = dataclasses.replace(DEFAULT_STATION_CONFIG, elevation_mask_deg=5.0)
    assert changed.elevation_mask_deg == 5.0
    assert DEFAULT_STATION_CONFIG.elevation_mask_deg == 10.0


@pytest.mark.parametrize("callsign", ["N0CALL", "AB1CDE/P", "K2X", "VE3ABC/MM"])
def test_valid_callsigns(callsign: str) -> None:
    assert StationConfig(callsign=callsign).callsign == callsign


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"callsign": "n0call"}, ValueError),
        ({"callsign": "AB"}, ValueError),
        ({"callsign": "A" * 17}, ValueError),
        ({"callsign": "N0 CALL"}, ValueError),
        ({"callsign": 1234}, TypeError),
        ({"latitude_deg": 90.5}, ValueError),
        ({"latitude_deg": -90.5}, ValueError),
        ({"latitude_deg": float("nan")}, ValueError),
        ({"latitude_deg": True}, TypeError),
        ({"longitude_deg": 180.5}, ValueError),
        ({"longitude_deg": float("inf")}, ValueError),
        ({"altitude_km": -1.0}, ValueError),
        ({"altitude_km": 9.5}, ValueError),
        ({"elevation_mask_deg": -1.0}, ValueError),
        ({"elevation_mask_deg": 90.0}, ValueError),
        ({"elevation_mask_deg": "10"}, TypeError),
        ({"uplink_frequency_hz": 0}, ValueError),
        ({"uplink_frequency_hz": 145.85e6}, TypeError),
        ({"downlink_frequency_hz": -1}, ValueError),
        ({"downlink_frequency_hz": True}, TypeError),
        ({"command_timeout_us": 0}, ValueError),
        ({"command_timeout_us": -1}, ValueError),
        ({"command_timeout_us": 2.0}, TypeError),
        ({"command_retry_count": -1}, ValueError),
        ({"command_retry_count": 11}, ValueError),
        ({"command_retry_count": False}, TypeError),
        ({"identification_interval_us": 0}, ValueError),
        ({"identification_interval_us": 600.0}, TypeError),
    ],
)
def test_config_validation(changes: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        StationConfig(**changes)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "changes",
    [
        {"latitude_deg": 90.0},
        {"latitude_deg": -90},
        {"longitude_deg": -180.0},
        {"longitude_deg": 180.0},
        {"altitude_km": -0.5},
        {"altitude_km": 9},
        {"elevation_mask_deg": 0.0},
        {"command_retry_count": 0},
        {"command_retry_count": 10},
    ],
)
def test_config_bounds_are_inclusive(changes: dict[str, object]) -> None:
    StationConfig(**changes)  # type: ignore[arg-type]


def test_check_tick_requires_two_ticks_of_timeout() -> None:
    DEFAULT_STATION_CONFIG.check_tick(100_000)
    DEFAULT_STATION_CONFIG.check_tick(1_000_000)  # exactly two ticks
    with pytest.raises(ValueError, match="two"):
        DEFAULT_STATION_CONFIG.check_tick(1_000_001)
    with pytest.raises(ValueError):
        DEFAULT_STATION_CONFIG.check_tick(0)
    with pytest.raises(TypeError):
        DEFAULT_STATION_CONFIG.check_tick(0.1)  # type: ignore[arg-type]


# --- Construction and stepping ------------------------------------------------------


def test_construction_takes_a_config() -> None:
    assert GroundStation().config is DEFAULT_STATION_CONFIG
    config = StationConfig(callsign="AB1CDE")
    assert GroundStation(config).config is config
    with pytest.raises(TypeError):
        GroundStation(config={"callsign": "AB1CDE"})  # type: ignore[arg-type]


def test_submit_assigns_references_in_order_from_zero() -> None:
    station = GroundStation()
    refs = [station.submit(Command.ping()), station.submit(Command.reset())]
    assert refs == [CommandRef(0), CommandRef(1)]
    with pytest.raises(TypeError):
        station.submit(b"\x01")  # type: ignore[arg-type]


def test_reset_restarts_references_and_time() -> None:
    station = GroundStation()
    station.submit(Command.ping())
    station.begin_tick(500_000, None)
    station.reset(RngFactory(42))
    assert station.submit(Command.ping()) == CommandRef(0)
    station.begin_tick(0, None)  # time 0 is valid again
    with pytest.raises(TypeError):
        station.reset(42)  # type: ignore[arg-type]


def test_stubs_are_no_ops_until_their_tickets() -> None:
    # Pass state (#161) and decoding (#156) are documented no-ops: with nothing
    # queued (uplink is #154, tests/unit/test_groundstation_uplink.py) the uplink is
    # empty, nothing is decoded, the store stays empty.
    station = GroundStation()
    assert station.begin_tick(0, GEOMETRY) == Uplink()
    assert station.begin_tick(0, None) == Uplink(packets=(), events=())
    # Decoding is #156 (test_station_downlink.py); the telemetry store stays empty until #157.
    downlink = [Packet(frame=b"\xa5\x5a\x01\x02\x00\x00\x00\x00\x00\x00")]
    assert [type(e) for e in station.end_tick(100_000, downlink)] == [FrameRejected]
    assert station.end_tick(100_000, []) == ()
    assert len(station.telemetry) == 0
    assert station.telemetry.latest is None
    assert station.telemetry.history == ()


def test_stepping_is_deterministic() -> None:
    def run() -> list[object]:
        station = GroundStation()
        station.reset(RngFactory(7))
        out: list[object] = []
        for tick in range(50):
            if tick % 10 == 0:
                out.append(station.submit(Command.set_mode(Mode.SCIENCE)))
            out.append(station.begin_tick(tick * 100_000, GEOMETRY))
            out.append(station.end_tick((tick + 1) * 100_000, []))
        return out

    assert run() == run()


@pytest.mark.parametrize("bad", [-1, 1.5, True])
def test_time_must_be_integer_microseconds(bad: object) -> None:
    station = GroundStation()
    with pytest.raises((TypeError, ValueError)):
        station.begin_tick(bad, None)  # type: ignore[arg-type]
    with pytest.raises((TypeError, ValueError)):
        station.end_tick(bad, [])  # type: ignore[arg-type]


def test_time_never_goes_back() -> None:
    station = GroundStation()
    station.begin_tick(100_000, None)
    station.end_tick(200_000, [])
    station.begin_tick(200_000, None)  # the next tick starts where the last ended
    with pytest.raises(ValueError, match="back"):
        station.end_tick(100_000, [])


def test_inputs_are_type_checked() -> None:
    station = GroundStation()
    with pytest.raises(TypeError):
        station.begin_tick(0, (120.0, 30.0))  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        station.end_tick(0, [b"\xa5\x5a"])  # type: ignore[list-item]


# --- Events -------------------------------------------------------------------------

TELEMETRY = decode_telemetry(bytes(TELEMETRY_PAYLOAD_SIZE))
REF = CommandRef(3)

EVENTS: list[StationEvent] = [
    Aos(0, pass_id=1, azimuth_deg=10.0, elevation_deg=10.0),
    Los(1, pass_id=1, azimuth_deg=200.0, elevation_deg=10.0, max_elevation_deg=45.0),
    CommandSent(2, command_ref=REF, command_id=1, sequence=0, retry_count=0),
    CommandAcked(3, command_ref=REF, sequence=0),
    CommandNacked(4, command_ref=REF, sequence=1, reason=RejectReason.BOOT_IN_PROGRESS),
    CommandTimedOut(5, command_ref=REF, retry_count=0, will_retry=True),
    CommandFailed(6, command_ref=REF, reason=FailureReason.NOT_RETRYABLE),
    UnmatchedAck(7, sequence=9, command_id=1),
    TelemetryReceived(8, sequence=4, telemetry=TELEMETRY),
    DataChunkReceived(9, sequence=5, chunk_id=12, content_ok=True),
    FrameRejected(10, reason=RejectionReason.WRONG_DIRECTION, frame=b"\xa5\x5a"),
    SequenceGap(11, sequence=8, missing_count=2),
    SpacecraftRebooted(12, sequence=0, boot_count=2),
    RadioStateChanged(13, state=RadioState.TRANSMIT),
    StationIdentified(14, callsign="N0CALL"),
]


def test_every_event_type_is_covered_and_in_the_union() -> None:
    assert [type(event) for event in EVENTS] == list(EVENT_TYPES)
    assert set(typing.get_args(StationEvent)) == set(EVENT_TYPES)


@pytest.mark.parametrize("event", EVENTS, ids=lambda e: type(e).__name__)
def test_events_are_immutable(event: StationEvent) -> None:
    for field in dataclasses.fields(event):
        with pytest.raises(dataclasses.FrozenInstanceError):
            setattr(event, field.name, None)


@pytest.mark.parametrize("event", EVENTS, ids=lambda e: type(e).__name__)
def test_events_carry_integer_microseconds(event: StationEvent) -> None:
    assert dataclasses.fields(event)[0].name == "time_us"
    kwargs = {f.name: getattr(event, f.name) for f in dataclasses.fields(event)}
    for bad, error in ((1.0, TypeError), (-1, ValueError), (True, TypeError)):
        with pytest.raises(error):
            type(event)(**{**kwargs, "time_us": bad})


def test_command_ref_validation() -> None:
    assert CommandRef(0).ref_id == 0
    with pytest.raises(ValueError):
        CommandRef(-1)
    with pytest.raises(TypeError):
        CommandRef(True)
    with pytest.raises(dataclasses.FrozenInstanceError):
        CommandRef(0).ref_id = 1  # type: ignore[misc]


def test_nack_reason_accepts_both_reason_enums() -> None:
    event = CommandNacked(0, command_ref=REF, sequence=0, reason=DecodeReason.UNKNOWN_COMMAND)
    assert event.reason is DecodeReason.UNKNOWN_COMMAND


def test_telemetry_store_starts_empty() -> None:
    store = TelemetryStore()
    assert len(store) == 0
    assert store.latest is None
