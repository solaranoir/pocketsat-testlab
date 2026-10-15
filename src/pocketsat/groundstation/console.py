"""The mission console: station events as text lines and JSON lines (#165, ADR-0010 §4).

The console is the station's presentation layer. It reads nothing but events
(:data:`~pocketsat.groundstation.events.StationEvent`) and writes two streams, both
injected and both a pure function of the events it was given:

- **Text**: one line per event, ``T+HH:MM:SS.t  TAG    detail``, the simulated time
  from the event's integer ``time_us`` (:func:`format_time`). For people.
- **JSON lines**: one object per event, ``{"event": <type name>, "time_us": ..., <the
  event's fields in declaration order>}`` (:func:`event_record`). For reporting
  (Phase 6) and assertions.

Nothing here reads a clock or draws a random number, so the same events give
byte-identical output on every platform (ADR-0003, ADR-0006): times use integer
arithmetic, text values are fixed-point at the telemetry's wire resolution
(:data:`~pocketsat.groundstation.telemetry.TELEMETRY_QUANTITIES`), and JSON floats are
Python's shortest round-trip ``repr``.

**Adding an event type** is one function here: write ``(event, state) -> (tag, detail)``
and decorate it with ``@_formats(NewEvent)``. An event type with no formatter still gets
a deterministic line (``EVENT  NewEvent field=value ...``, :func:`generic_line`) and a
full JSON record, so a new event is never lost from the console. The format is
documented in ``docs/groundstation.md`` ("Mission console").

The only state is :class:`ConsoleState`: the command IDs of commands sent and not yet
resolved (so an ACK line can name its command) and the DATA tally. Both come from
earlier events, so the output is still a function of the event sequence alone.
"""

import json
import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum, Flag
from fractions import Fraction
from typing import Any, Final, TextIO

from pocketsat.core.clock import check_us
from pocketsat.groundstation.events import (
    Aos,
    CommandAcked,
    CommandFailed,
    CommandNacked,
    CommandRef,
    CommandSent,
    CommandTimedOut,
    DataChunkReceived,
    FrameRejected,
    Los,
    RadioStateChanged,
    SequenceGap,
    SpacecraftRebooted,
    StationIdentified,
    TelemetryReceived,
    UnmatchedAck,
)
from pocketsat.groundstation.telemetry import QUANTITIES_BY_NAME
from pocketsat.messages import CommandId, Telemetry

US_PER_TENTH: Final = 100_000
"""Microseconds per tenth of a second, the text timestamp's resolution."""

TAG_WIDTH: Final = 6
"""Width the tag is padded to, so details line up."""

GENERIC_TAG: Final = "EVENT"
"""Tag of an event type with no formatter."""

type JsonValue = bool | int | float | str | list[JsonValue] | dict[str, JsonValue] | None
"""A value :func:`json.dumps` writes."""

Line = tuple[str, str]
"""A formatter's result: the tag and the detail."""


@dataclass
class ConsoleState:
    """What the console remembers from earlier events.

    Attributes:
        command_ids: Command ID of each command sent and not yet resolved, by
            ``CommandRef.ref_id``: set by ``CommandSent``, removed by ``CommandAcked``,
            ``CommandNacked`` and ``CommandFailed``.
        stored_chunk_count: DATA chunks stored (good and not duplicate) so far.
    """

    command_ids: dict[int, int] = field(default_factory=dict)
    stored_chunk_count: int = 0


Formatter = Callable[[Any, ConsoleState], Line]

_FORMATTERS: Final[dict[type, Formatter]] = {}
"""Formatter of each event type. Exact type match: a subclass is a new event type."""


def _formats[E](
    event_type: type[E],
) -> Callable[[Callable[[E, ConsoleState], Line]], Callable[[E, ConsoleState], Line]]:
    """Register the decorated function as ``event_type``'s formatter."""

    def register(function: Callable[[E, ConsoleState], Line]) -> Callable[[E, ConsoleState], Line]:
        if event_type in _FORMATTERS:
            raise ValueError(f"{event_type.__name__} already has a formatter")
        _FORMATTERS[event_type] = function
        return function

    return register


def formatted_types() -> frozenset[type]:
    """The event types with a dedicated formatter (the rest get :func:`generic_line`)."""
    return frozenset(_FORMATTERS)


# --- Value formatting -------------------------------------------------------------------


def format_time(time_us: int) -> str:
    """Simulated time as ``T+HH:MM:SS.t``, truncated to the tenth of a second.

    Integer arithmetic only. Hours are at least two digits and grow past 99.

    Raises:
        TypeError: ``time_us`` is not an int.
        ValueError: ``time_us`` is negative.
    """
    check_us("time_us", time_us)
    whole_seconds, tenth = divmod(time_us // US_PER_TENTH, 10)
    whole_minutes, seconds = divmod(whole_seconds, 60)
    hours, minutes = divmod(whole_minutes, 60)
    return f"T+{hours:02d}:{minutes:02d}:{seconds:02d}.{tenth}"


def _fixed(value: float, decimals: int) -> str:
    """``value`` to ``decimals`` places, with ``-0.0`` written as ``0.0``."""
    text = f"{value:.{decimals}f}"
    if text.startswith("-") and text.strip("-0.") == "":
        return text[1:]
    return text


def _decimals(resolution: Fraction) -> int:
    """Decimal places that show every multiple of ``resolution`` (1/10**n) exactly."""
    denominator, places = resolution.denominator, 0
    while denominator > 1:
        denominator, remainder = divmod(denominator, 10)
        if remainder:
            raise ValueError(f"resolution {resolution} is not a power of ten")
        places += 1
    return places


def _quantity(telemetry: Telemetry, name: str) -> str:
    """Telemetry field ``name`` at its wire resolution (``TELEMETRY_QUANTITIES``)."""
    resolution = QUANTITIES_BY_NAME[name].resolution
    value: float = getattr(telemetry, name)
    return _fixed(value, 0 if resolution is None else _decimals(resolution))


def _flag_names(flags: Flag) -> list[str]:
    """Names of the members set in ``flags``, in definition (bit) order."""
    return [str(member.name) for member in type(flags) if member in flags]


def _command_name(command_id: int) -> str:
    """The command's name, or its hex ID if it is not an assigned :class:`CommandId`."""
    try:
        return CommandId(command_id).name
    except ValueError:
        return f"0x{command_id:02X}"


def _command(state: ConsoleState, ref: CommandRef) -> str:
    """``#ref`` followed by the command's name, if its ``CommandSent`` was seen."""
    command_id = state.command_ids.get(ref.ref_id)
    return f"#{ref.ref_id}" if command_id is None else f"#{ref.ref_id} {_command_name(command_id)}"


def _text_value(value: object) -> str:
    """A field value for :func:`generic_line`: deterministic and on one line."""
    if value is None:
        return "none"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, Flag):
        return "|".join(_flag_names(value)) or "none"
    if isinstance(value, Enum):
        return str(value.name)
    if isinstance(value, CommandRef):
        return f"#{value.ref_id}"
    if isinstance(value, bytes):
        return value.hex()
    if isinstance(value, float):
        return repr(value)
    if is_dataclass(value) and not isinstance(value, type):
        inner = " ".join(f"{f.name}={_text_value(getattr(value, f.name))}" for f in fields(value))
        return "{" + inner + "}"
    if isinstance(value, (tuple, list)):
        return "[" + ",".join(_text_value(item) for item in value) + "]"
    if isinstance(value, str):
        # Quoted (JSON string syntax) if it could break the line or the name=value split.
        plain = value and value.isprintable() and not any(c.isspace() for c in value)
        return value if plain else json.dumps(value)
    if isinstance(value, int):
        return str(value)
    raise TypeError(f"no deterministic text for {type(value).__name__} value {value!r}")


def json_value(value: object) -> JsonValue:
    """A field value as JSON: enums by name, flags as a list of names in bit order,
    ``CommandRef`` as its number, bytes as lowercase hex, dataclasses as objects in field
    order, tuples as lists.

    Raises:
        TypeError: The value has no deterministic JSON form.
        ValueError: A float is not finite.
    """
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, Flag):
        names: list[JsonValue] = [name for name in _flag_names(value)]
        return names
    if isinstance(value, Enum):
        return str(value.name)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"no JSON for the non-finite float {value!r}")
        return value
    if isinstance(value, CommandRef):
        return value.ref_id
    if isinstance(value, bytes):
        return value.hex()
    if is_dataclass(value) and not isinstance(value, type):
        return _fields_json(value)
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    raise TypeError(f"no deterministic JSON for {type(value).__name__} value {value!r}")


def _fields_json(instance: Any) -> dict[str, JsonValue]:
    """A dataclass instance's fields as JSON, in declaration order. A field declared
    ``int`` is written as a number even when it holds an ``IntEnum`` member (for example
    ``CommandSent.command_id``), so a key always has one JSON type."""
    record: dict[str, JsonValue] = {}
    for f in fields(instance):
        value = getattr(instance, f.name)
        record[f.name] = int(value) if f.type in (int, "int") else json_value(value)
    return record


# --- Formatters, one per event type ------------------------------------------------------


@_formats(Aos)
def _aos(event: Aos, state: ConsoleState) -> Line:
    return "AOS", (
        f"pass {event.pass_id} az {_fixed(event.azimuth_deg, 1)} deg"
        f" el {_fixed(event.elevation_deg, 1)} deg"
    )


@_formats(Los)
def _los(event: Los, state: ConsoleState) -> Line:
    return "LOS", (
        f"pass {event.pass_id} az {_fixed(event.azimuth_deg, 1)} deg"
        f" el {_fixed(event.elevation_deg, 1)} deg"
        f" max el {_fixed(event.max_elevation_deg, 1)} deg"
    )


@_formats(CommandSent)
def _command_sent(event: CommandSent, state: ConsoleState) -> Line:
    state.command_ids[event.command_ref.ref_id] = event.command_id
    retry = f" retry {event.retry_count}" if event.retry_count else ""
    return "CMD", f"{_command(state, event.command_ref)} sent seq {event.sequence}{retry}"


@_formats(CommandAcked)
def _command_acked(event: CommandAcked, state: ConsoleState) -> Line:
    line = f"{_command(state, event.command_ref)} ACK seq {event.sequence}"
    state.command_ids.pop(event.command_ref.ref_id, None)
    return "CMD", line


@_formats(CommandNacked)
def _command_nacked(event: CommandNacked, state: ConsoleState) -> Line:
    line = f"{_command(state, event.command_ref)} NACK {event.reason.name} seq {event.sequence}"
    state.command_ids.pop(event.command_ref.ref_id, None)
    return "CMD", line


@_formats(CommandTimedOut)
def _command_timed_out(event: CommandTimedOut, state: ConsoleState) -> Line:
    then = "retrying" if event.will_retry else "no retry"
    attempt = event.retry_count + 1
    return "CMD", f"{_command(state, event.command_ref)} timed out attempt {attempt}, {then}"


@_formats(CommandFailed)
def _command_failed(event: CommandFailed, state: ConsoleState) -> Line:
    line = f"{_command(state, event.command_ref)} FAILED {event.reason.name}"
    state.command_ids.pop(event.command_ref.ref_id, None)
    return "CMD", line


@_formats(UnmatchedAck)
def _unmatched_ack(event: UnmatchedAck, state: ConsoleState) -> Line:
    return "CMD", f"unmatched ACK {_command_name(event.command_id)} seq {event.sequence}"


@_formats(TelemetryReceived)
def _telemetry(event: TelemetryReceived, state: ConsoleState) -> Line:
    t = event.telemetry
    flags = ",".join(_flag_names(t.flags)) if t.flags else "none"
    return "TLM", (
        f"seq {event.sequence} {t.mode.name} boot {t.boot_count}"
        f" soc {_quantity(t, 'soc')} bus {_quantity(t, 'bus_v')} V"
        f" batt {_quantity(t, 'battery_c')} C elec {_quantity(t, 'electronics_c')} C"
        f" radio {t.radio_mode.name} att {t.attitude_state.name}"
        f" point {_quantity(t, 'pointing_error_deg')} deg"
        f" payload {t.payload_state.name} buf {t.buffered_bytes} B flags {flags}"
    )


@_formats(DataChunkReceived)
def _data_chunk(event: DataChunkReceived, state: ConsoleState) -> Line:
    if not event.content_ok:
        status = "CORRUPT duplicate" if event.duplicate else "CORRUPT"
    elif event.duplicate:
        status = "duplicate"
    else:
        state.stored_chunk_count += 1
        status = "stored"
    stored = state.stored_chunk_count
    return "DATA", f"chunk {event.chunk_id} seq {event.sequence} {status}, {stored} chunks stored"


@_formats(FrameRejected)
def _frame_rejected(event: FrameRejected, state: ConsoleState) -> Line:
    return "FRAME", f"rejected {event.reason.name} {len(event.frame)} bytes"


@_formats(SequenceGap)
def _sequence_gap(event: SequenceGap, state: ConsoleState) -> Line:
    expected = (event.sequence - event.missing_count) % 65536
    return "GAP", f"seq {event.sequence} missing {event.missing_count} (expected {expected})"


@_formats(SpacecraftRebooted)
def _rebooted(event: SpacecraftRebooted, state: ConsoleState) -> Line:
    return "REBOOT", f"boot {event.boot_count} seq {event.sequence}"


@_formats(RadioStateChanged)
def _radio_state(event: RadioStateChanged, state: ConsoleState) -> Line:
    return "RADIO", event.state.name


@_formats(StationIdentified)
def _identified(event: StationIdentified, state: ConsoleState) -> Line:
    return "ID", event.callsign


# --- Lines and records -------------------------------------------------------------------


def _event_fields(event: object) -> list[tuple[str, object]]:
    """The event's fields in declaration order, ``time_us`` first.

    Raises:
        TypeError: ``event`` is not a dataclass instance whose first field is an int
            ``time_us``.
    """
    if not is_dataclass(event) or isinstance(event, type):
        raise TypeError(f"an event must be a dataclass instance, got {event!r}")
    pairs = [(f.name, getattr(event, f.name)) for f in fields(event)]
    if not pairs or pairs[0][0] != "time_us":
        raise TypeError(f"an event's first field must be time_us, got {event!r}")
    time_us = pairs[0][1]
    if isinstance(time_us, bool) or not isinstance(time_us, int):
        raise TypeError(f"an event's time_us must be an int, got {event!r}")
    return pairs


def generic_line(event: object) -> str:
    """The detail of an event type with no formatter: its type name and every field
    but ``time_us`` as ``name=value``, in declaration order.

    Raises:
        TypeError: ``event`` is not a dataclass with an int ``time_us``, or a field
            value has no deterministic text.
    """
    parts = [type(event).__name__]
    parts += [f"{name}={_text_value(value)}" for name, value in _event_fields(event)[1:]]
    return " ".join(parts)


def format_event(event: object, state: ConsoleState | None = None) -> str:
    """One console line for ``event``, without a newline: ``T+HH:MM:SS.t  TAG    detail``.

    Args:
        event: A :data:`~pocketsat.groundstation.events.StationEvent`, or any event
            dataclass with an int ``time_us`` (which gets :func:`generic_line`).
        state: What the console remembers from earlier events; updated in place. A
            fresh :class:`ConsoleState` if omitted.

    Raises:
        TypeError: ``event`` is not an event dataclass.
    """
    time_us = _event_fields(event)[0][1]
    assert isinstance(time_us, int)
    formatter = _FORMATTERS.get(type(event))
    if formatter is None:
        tag, detail = GENERIC_TAG, generic_line(event)
    else:
        tag, detail = formatter(event, ConsoleState() if state is None else state)
    return f"{format_time(time_us)}  {tag:<{TAG_WIDTH}} {detail}"


def event_record(event: object) -> dict[str, JsonValue]:
    """The structured record of ``event``: ``{"event": <type name>, "time_us": ...,
    <every other field in declaration order>}``, values by :func:`json_value`.

    Raises:
        TypeError: ``event`` is not an event dataclass, or a value has no JSON form.
    """
    _event_fields(event)
    return {"event": type(event).__name__} | _fields_json(event)


def event_json(event: object) -> str:
    """:func:`event_record` as one compact JSON line, without a newline: keys in record
    order, ASCII only, no spaces, non-finite floats refused."""
    return json.dumps(
        event_record(event), ensure_ascii=True, allow_nan=False, separators=(",", ":")
    )


class MissionConsole:
    """Writes each event as a text line and, optionally, a JSON line (#165).

    Every line ends with ``"\\n"``. Open file streams with ``newline="\\n"`` (and UTF-8)
    so the bytes are the same on every platform.

    Args:
        text: Stream for the text lines, or ``None`` for none.
        log: Stream for the JSON lines, or ``None`` for none.
    """

    def __init__(self, text: TextIO | None = None, log: TextIO | None = None) -> None:
        self._text = text
        self._log = log
        self._state = ConsoleState()

    @property
    def state(self) -> ConsoleState:
        """What the console remembers from the events so far."""
        return self._state

    def reset(self) -> None:
        """Forget earlier events, as after a station reset (command references restart)."""
        self._state = ConsoleState()

    def emit(self, event: object) -> str:
        """Write ``event`` to both streams and return its text line (without newline).

        Raises:
            TypeError: ``event`` is not an event dataclass.
        """
        line = format_event(event, self._state)
        if self._text is not None:
            self._text.write(line + "\n")
        if self._log is not None:
            self._log.write(event_json(event) + "\n")
        return line

    def emit_all(self, events: Iterable[object]) -> None:
        """:meth:`emit` each event, in order."""
        for event in events:
            self.emit(event)
