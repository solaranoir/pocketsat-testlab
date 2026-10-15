"""The mission console's text and JSON lines (#165).

Every event type has an exact expected text line, an event type with no formatter gets
the generic line, and the JSON lines have a fixed shape and key order.
"""

import io
import json
from dataclasses import dataclass
from enum import Enum

import pytest

from pocketsat.flight import Mode, RejectReason
from pocketsat.groundstation import (
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
    Los,
    RadioState,
    RadioStateChanged,
    RejectionReason,
    SequenceGap,
    SpacecraftRebooted,
    StationIdentified,
    TelemetryReceived,
    UnmatchedAck,
)
from pocketsat.groundstation.console import (
    ConsoleState,
    MissionConsole,
    event_json,
    event_record,
    format_event,
    format_time,
    formatted_types,
    generic_line,
    json_value,
)
from pocketsat.groundstation.events import _Event
from pocketsat.messages import CommandId, DecodeReason, Telemetry, TelemetryFlags
from pocketsat.spacecraft.controls import RadioMode
from pocketsat.spacecraft.snapshots import AttitudeState, PayloadState

T = 754_512_345  # T+00:12:34.5
REF = CommandRef(3)

TELEMETRY = Telemetry(
    uptime_ms=754_000,
    boot_count=2,
    flags=TelemetryFlags.low_battery | TelemetryFlags.over_temp,
    mode=Mode.SCIENCE,
    radio_mode=RadioMode.RX_TX,
    attitude_state=AttitudeState.STABILIZED,
    payload_state=PayloadState.ACQUIRING,
    bus_v=7.2,
    soc=0.5,
    battery_c=-3.5,
    electronics_c=21.07,
    buffered_bytes=4096,
    pointing_error_deg=1.25,
)


# --- Time ---------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("time_us", "text"),
    [
        (0, "T+00:00:00.0"),
        (99_999, "T+00:00:00.0"),
        (100_000, "T+00:00:00.1"),
        (754_512_345, "T+00:12:34.5"),
        (3_599_999_999, "T+00:59:59.9"),
        (3_600_000_000, "T+01:00:00.0"),
        (100 * 3_600_000_000 + 1, "T+100:00:00.0"),
    ],
)
def test_time_is_truncated_to_the_tenth(time_us: int, text: str) -> None:
    assert format_time(time_us) == text


def test_time_rejects_bad_values() -> None:
    with pytest.raises(ValueError, match="negative"):
        format_time(-1)
    with pytest.raises(TypeError):
        format_time(1.5)  # type: ignore[arg-type]


# --- One exact line per event type ------------------------------------------------------------

EXPECTED_LINES: list[tuple[object, str]] = [
    (
        Aos(T, pass_id=1, azimuth_deg=213.44, elevation_deg=10.0),
        "T+00:12:34.5  AOS    pass 1 az 213.4 deg el 10.0 deg",
    ),
    (
        Los(T, pass_id=1, azimuth_deg=45.25, elevation_deg=9.96, max_elevation_deg=62.31),
        "T+00:12:34.5  LOS    pass 1 az 45.2 deg el 10.0 deg max el 62.3 deg",
    ),
    (
        CommandSent(T, REF, CommandId.PING, sequence=12, retry_count=0),
        "T+00:12:34.5  CMD    #3 PING sent seq 12",
    ),
    (
        CommandSent(T, REF, CommandId.SET_MODE, sequence=14, retry_count=2),
        "T+00:12:34.5  CMD    #3 SET_MODE sent seq 14 retry 2",
    ),
    (
        CommandSent(T, REF, 0x7F, sequence=15, retry_count=0),
        "T+00:12:34.5  CMD    #3 0x7F sent seq 15",
    ),
    (CommandAcked(T, REF, sequence=12), "T+00:12:34.5  CMD    #3 ACK seq 12"),
    (
        CommandNacked(T, REF, sequence=12, reason=RejectReason.BOOT_IN_PROGRESS),
        "T+00:12:34.5  CMD    #3 NACK BOOT_IN_PROGRESS seq 12",
    ),
    (
        CommandNacked(T, REF, sequence=12, reason=DecodeReason.UNKNOWN_COMMAND),
        "T+00:12:34.5  CMD    #3 NACK UNKNOWN_COMMAND seq 12",
    ),
    (
        CommandTimedOut(T, REF, retry_count=0, will_retry=True),
        "T+00:12:34.5  CMD    #3 timed out attempt 1, retrying",
    ),
    (
        CommandTimedOut(T, REF, retry_count=2, will_retry=False),
        "T+00:12:34.5  CMD    #3 timed out attempt 3, no retry",
    ),
    (
        CommandFailed(T, REF, FailureReason.RETRIES_EXHAUSTED),
        "T+00:12:34.5  CMD    #3 FAILED RETRIES_EXHAUSTED",
    ),
    (
        CommandFailed(T, REF, FailureReason.EXPIRED),
        "T+00:12:34.5  CMD    #3 FAILED EXPIRED",
    ),
    (
        UnmatchedAck(T, sequence=9, command_id=CommandId.RESET),
        "T+00:12:34.5  CMD    unmatched ACK RESET seq 9",
    ),
    (
        UnmatchedAck(T, sequence=9, command_id=0x00),
        "T+00:12:34.5  CMD    unmatched ACK 0x00 seq 9",
    ),
    (
        TelemetryReceived(T, sequence=77, telemetry=TELEMETRY),
        "T+00:12:34.5  TLM    seq 77 SCIENCE boot 2 soc 0.500 bus 7.200 V batt -3.50 C"
        " elec 21.07 C radio RX_TX att STABILIZED point 1.25 deg payload ACQUIRING"
        " buf 4096 B flags low_battery,over_temp",
    ),
    (
        DataChunkReceived(T, sequence=345, chunk_id=12, content_ok=True),
        "T+00:12:34.5  DATA   chunk 12 seq 345 stored, 1 chunks stored",
    ),
    (
        DataChunkReceived(T, sequence=345, chunk_id=12, content_ok=True, duplicate=True),
        "T+00:12:34.5  DATA   chunk 12 seq 345 duplicate, 0 chunks stored",
    ),
    (
        DataChunkReceived(T, sequence=345, chunk_id=12, content_ok=False),
        "T+00:12:34.5  DATA   chunk 12 seq 345 CORRUPT, 0 chunks stored",
    ),
    (
        DataChunkReceived(T, sequence=345, chunk_id=12, content_ok=False, duplicate=True),
        "T+00:12:34.5  DATA   chunk 12 seq 345 CORRUPT duplicate, 0 chunks stored",
    ),
    (
        FrameRejected(T, RejectionReason.CRC, b"\xa5\x5a\x01\x02"),
        "T+00:12:34.5  FRAME  rejected CRC 4 bytes",
    ),
    (
        SequenceGap(T, sequence=2, missing_count=3),
        "T+00:12:34.5  GAP    seq 2 missing 3 (expected 65535)",
    ),
    (
        SpacecraftRebooted(T, sequence=0, boot_count=3),
        "T+00:12:34.5  REBOOT boot 3 seq 0",
    ),
    (RadioStateChanged(T, RadioState.TRANSMIT), "T+00:12:34.5  RADIO  TRANSMIT"),
    (StationIdentified(T, "W1AW/3"), "T+00:12:34.5  ID     W1AW/3"),
]


@pytest.mark.parametrize(("event", "line"), EXPECTED_LINES, ids=lambda v: type(v).__name__)
def test_each_event_type_has_its_line(event: object, line: str) -> None:
    assert format_event(event) == line


def test_every_station_event_type_has_a_formatter_and_an_expected_line() -> None:
    assert formatted_types() == set(EVENT_TYPES)
    assert {type(event) for event, _ in EXPECTED_LINES} == set(EVENT_TYPES)


def test_negative_zero_is_written_without_a_sign() -> None:
    event = Los(0, pass_id=1, azimuth_deg=-0.04, elevation_deg=-0.0, max_elevation_deg=5.0)
    assert format_event(event) == "T+00:00:00.0  LOS    pass 1 az 0.0 deg el 0.0 deg max el 5.0 deg"


def test_telemetry_without_flags_says_none() -> None:
    telemetry = Telemetry(**{**vars(TELEMETRY), "flags": TelemetryFlags(0)})
    assert format_event(TelemetryReceived(T, 1, telemetry)).endswith(" flags none")


# --- What the console remembers -------------------------------------------------------------


def test_command_lines_name_the_command_until_it_is_resolved() -> None:
    state = ConsoleState()
    lines = [
        format_event(event, state)
        for event in (
            CommandSent(0, CommandRef(0), CommandId.PING, 0, 0),
            CommandSent(0, CommandRef(1), CommandId.RESET, 1, 0),
            CommandSent(0, CommandRef(2), CommandId.SET_MODE, 2, 0),
            CommandTimedOut(2_000_000, CommandRef(1), 0, will_retry=False),
            CommandFailed(2_000_000, CommandRef(1), FailureReason.NOT_RETRYABLE),
            CommandAcked(2_100_000, CommandRef(0), 0),
            CommandNacked(2_100_000, CommandRef(2), 2, RejectReason.NOT_ALLOWED_IN_SAFE),
            CommandAcked(2_200_000, CommandRef(0), 0),
        )
    ]
    assert [line[14:] for line in lines] == [
        "CMD    #0 PING sent seq 0",
        "CMD    #1 RESET sent seq 1",
        "CMD    #2 SET_MODE sent seq 2",
        "CMD    #1 RESET timed out attempt 1, no retry",
        "CMD    #1 RESET FAILED NOT_RETRYABLE",
        "CMD    #0 PING ACK seq 0",
        "CMD    #2 SET_MODE NACK NOT_ALLOWED_IN_SAFE seq 2",
        # Resolved commands are forgotten, so the state stays bounded.
        "CMD    #0 ACK seq 0",
    ]
    assert state.command_ids == {}


def test_data_lines_count_the_chunks_stored() -> None:
    console = MissionConsole()
    details = [
        console.emit(DataChunkReceived(0, sequence, chunk_id, ok, duplicate))[14:]
        for sequence, chunk_id, ok, duplicate in (
            (1, 0, True, False),
            (2, 1, False, False),
            (3, 1, True, False),
            (4, 1, True, True),
            (5, 2, True, False),
        )
    ]
    assert details == [
        "DATA   chunk 0 seq 1 stored, 1 chunks stored",
        "DATA   chunk 1 seq 2 CORRUPT, 1 chunks stored",
        "DATA   chunk 1 seq 3 stored, 2 chunks stored",
        "DATA   chunk 1 seq 4 duplicate, 2 chunks stored",
        "DATA   chunk 2 seq 5 stored, 3 chunks stored",
    ]
    console.reset()
    assert console.state == ConsoleState()


# --- Event types with no formatter ------------------------------------------------------------


class _Polarization(Enum):
    RHCP = "rhcp"


@dataclass(frozen=True)
class _FutureEvent(_Event):
    """Stands in for an event type a later ticket adds (#162, #163, #164)."""

    doppler_hz: float
    snr_db: float
    locked: bool
    polarization: _Polarization
    command_ref: CommandRef
    raw: bytes
    note: str | None = None


FUTURE = _FutureEvent(
    T, -1234.5, 0.1, True, _Polarization.RHCP, CommandRef(4), b"\x01\xff", note=None
)


def test_an_event_type_without_a_formatter_gets_the_generic_line() -> None:
    assert format_event(FUTURE) == (
        "T+00:12:34.5  EVENT  _FutureEvent doppler_hz=-1234.5 snr_db=0.1 locked=true"
        " polarization=RHCP command_ref=#4 raw=01ff note=none"
    )
    assert generic_line(FUTURE) == format_event(FUTURE)[21:]


def test_an_event_type_without_a_formatter_gets_a_full_record() -> None:
    assert event_json(FUTURE) == (
        '{"event":"_FutureEvent","time_us":754512345,"doppler_hz":-1234.5,"snr_db":0.1,'
        '"locked":true,"polarization":"RHCP","command_ref":4,"raw":"01ff","note":null}'
    )


def test_generic_strings_stay_on_one_line() -> None:
    @dataclass(frozen=True)
    class Note(_Event):
        text: str

    assert generic_line(Note(0, "W1AW")) == "Note text=W1AW"
    assert generic_line(Note(0, "two words\n")) == 'Note text="two words\\n"'
    assert generic_line(Note(0, "")) == 'Note text=""'


def test_a_non_event_is_refused() -> None:
    @dataclass(frozen=True)
    class NoTime:
        pass_id: int

    for bad in (object(), Aos, NoTime(1)):
        with pytest.raises(TypeError):
            format_event(bad)
        with pytest.raises(TypeError):
            event_record(bad)


def test_values_without_a_deterministic_form_are_refused() -> None:
    @dataclass(frozen=True)
    class Odd(_Event):
        value: object

    with pytest.raises(TypeError, match="no deterministic text"):
        format_event(Odd(0, {1, 2}))
    with pytest.raises(TypeError, match="no deterministic JSON"):
        event_json(Odd(0, {1, 2}))
    with pytest.raises(ValueError, match="non-finite"):
        json_value(float("nan"))


# --- JSON lines -------------------------------------------------------------------------------


def test_json_records_have_the_type_then_the_fields_in_order() -> None:
    record = event_record(
        CommandNacked(T, REF, sequence=12, reason=RejectReason.SAFE_CONDITIONS_ACTIVE)
    )
    assert list(record.items()) == [
        ("event", "CommandNacked"),
        ("time_us", T),
        ("command_ref", 3),
        ("sequence", 12),
        ("reason", "SAFE_CONDITIONS_ACTIVE"),
    ]


def test_a_field_declared_int_is_a_number_even_for_an_assigned_id() -> None:
    assert event_record(CommandSent(T, REF, CommandId.SET_MODE, 1, 0))["command_id"] == 2
    assert event_record(UnmatchedAck(T, 1, 0x7F))["command_id"] == 0x7F


def test_telemetry_record_is_nested_in_engineering_units() -> None:
    assert event_json(TelemetryReceived(T, 77, TELEMETRY)) == (
        '{"event":"TelemetryReceived","time_us":754512345,"sequence":77,"telemetry":{'
        '"uptime_ms":754000,"boot_count":2,"flags":["low_battery","over_temp"],'
        '"mode":"SCIENCE","radio_mode":"RX_TX","attitude_state":"STABILIZED",'
        '"payload_state":"ACQUIRING","bus_v":7.2,"soc":0.5,"battery_c":-3.5,'
        '"electronics_c":21.07,"buffered_bytes":4096,"pointing_error_deg":1.25}}'
    )


def test_every_expected_event_has_a_json_line_that_round_trips() -> None:
    for event, _ in EXPECTED_LINES:
        line = event_json(event)
        assert "\n" not in line and " " not in line.replace("W1AW/3", "")
        assert json.loads(line) == event_record(event)


def test_frame_bytes_are_hex() -> None:
    record = event_record(FrameRejected(T, RejectionReason.WRONG_DIRECTION, b"\xa5\x5a"))
    assert record["frame"] == "a55a"
    assert record["reason"] == "WRONG_DIRECTION"


# --- The console writes both streams ------------------------------------------------------------


def test_console_writes_one_text_line_and_one_json_line_per_event() -> None:
    text, log = io.StringIO(), io.StringIO()
    console = MissionConsole(text, log)
    events = [event for event, _ in EXPECTED_LINES[:3]]
    console.emit_all(events)
    assert text.getvalue() == "".join(format_event(e) + "\n" for e in events)
    assert log.getvalue() == "".join(event_json(e) + "\n" for e in events)


def test_console_streams_are_optional() -> None:
    text = io.StringIO()
    assert MissionConsole(text).emit(StationIdentified(0, "N0CALL")) == (
        "T+00:00:00.0  ID     N0CALL"
    )
    assert text.getvalue() == "T+00:00:00.0  ID     N0CALL\n"
    log = io.StringIO()
    MissionConsole(log=log).emit(StationIdentified(0, "N0CALL"))
    assert log.getvalue() == '{"event":"StationIdentified","time_us":0,"callsign":"N0CALL"}\n'
