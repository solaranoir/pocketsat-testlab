"""Typed messages exchanged between components above the ``TestTarget`` boundary.

The target boundary itself carries raw frame bytes (see :mod:`pocketsat.frame`). These
types are deliberately minimal and are extended in later phases.

Telemetry (#54): :func:`encode_telemetry` packs the flight computer's state and the
subsystems' **readings** records into the fixed-size TELEMETRY payload, and
:func:`decode_telemetry` unpacks it into :class:`Telemetry`. The layout, the mapping from
readings fields to wire integers, and the flag bits are documented in
``docs/protocol.md`` ("Telemetry payload"); that document and
``tests/vectors/telemetry.json`` are the contract the Phase 7 firmware must match. True
values never go on the wire (ADR-0004 §7): the encoder accepts no truth record, so mypy
rejects one.

The telemetry ``mode`` field carries the flight computer's :class:`~pocketsat.flight.Mode`
(#47), whose values are the wire values, so there is no separate mode table to drift
from the enum (#101). This module imports ``Mode`` from :mod:`pocketsat.flight.modes`,
which depends only on :mod:`pocketsat.spacecraft.controls`; see :mod:`pocketsat.flight`
for how its modules import this one without a cycle.

Commands (#52): :func:`encode_command` builds a COMMAND payload from a :class:`Command`
(ground side), and :func:`decode_command` parses and validates one on the spacecraft,
returning a :class:`ParsedCommand` or a :class:`MalformedCommand` with a
:class:`DecodeReason`; it never raises. :func:`encode_ack` and :func:`decode_ack` handle
the ACK frame payload (:class:`CommandAck`), whose NACK reason is a :data:`NackReason`:
a :class:`DecodeReason` (0x01-0x0F) or the mode state machine's
:class:`~pocketsat.flight.RejectReason` (0x10-0x1F), reused rather than redefined. The
layouts and every code are in ``docs/protocol.md`` ("Commands"); the vectors are
``tests/vectors/commands.json``. The command dispatcher that uses them is #51.

DATA (#56): :func:`encode_data` packs one payload chunk (:class:`DataChunk`: its ID and
its content, ``pocketsat.spacecraft.payload.chunk_content``) into a DATA frame payload,
and :func:`decode_data` unpacks it on the ground. The layout is in ``docs/protocol.md``
("DATA payload"); the vectors are ``tests/vectors/data.json``. The flight computer's
downlink session (:mod:`pocketsat.flight.downlink`) sends them.
"""

import math
import struct
from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from types import MappingProxyType
from typing import Final, Self

from pocketsat.flight.modes import Mode, RejectReason
from pocketsat.frame import MAX_SEQUENCE, MIN_FRAME_SIZE
from pocketsat.spacecraft.config import MAX_CHUNK_SIZE_BYTES
from pocketsat.spacecraft.controls import RadioMode
from pocketsat.spacecraft.payload import MAX_CHUNK_ID
from pocketsat.spacecraft.snapshots import (
    AttitudeReadings,
    AttitudeState,
    CommsReadings,
    PayloadReadings,
    PayloadState,
    PowerReadings,
    ThermalReadings,
)


@dataclass(frozen=True)
class Packet:
    """A frame in transit, plus link-state attributes attached by the RF channel.

    Link-state attributes are ``None`` until the RF channel sets them.

    Attributes:
        frame: Encoded frame bytes. Corruption faults mutate these bytes.
        elevation_deg: Elevation of the spacecraft above the station horizon.
        range_km: Slant range between station and spacecraft.
        doppler_hz: Doppler shift applied to the carrier.
        snr_db: Signal-to-noise ratio.
        loss_probability: Probability, ``0..1``, that this packet is dropped.
        latency_s: One-way link latency in simulated seconds.
    """

    frame: bytes
    elevation_deg: float | None = None
    range_km: float | None = None
    doppler_hz: float | None = None
    snr_db: float | None = None
    loss_probability: float | None = None
    latency_s: float | None = None


# --- Commands and ACK/NACK (#52) -------------------------------------------------------


class CommandId(IntEnum):
    """Command identifier: the first byte of every COMMAND payload (#52).

    Values are uint8 wire codes and part of the firmware contract: a code is never
    renumbered or reused. ``0x00`` is reserved and never assigned, so an ACK for a
    COMMAND payload too short to carry an ID can name ``0x00``. The argument layout of
    each command is :data:`COMMAND_ARGUMENT_SIZES` and ``docs/protocol.md``.
    """

    PING = 0x01
    """No arguments. Answered with an ACK in every mode; never changes the mode (#51)."""

    SET_MODE = 0x02
    """One argument byte: the target :class:`~pocketsat.flight.Mode` value."""

    BEGIN_DOWNLINK = 0x03
    """No arguments. Starts a downlink session (#56)."""

    ENTER_SAFE_MODE = 0x04
    """No arguments. Enters SAFE."""

    RESET = 0x05
    """No arguments. Reboots the flight computer into BOOT (#49)."""


COMMAND_ARGUMENT_SIZES: Final[Mapping[CommandId, int]] = MappingProxyType(
    {
        CommandId.PING: 0,
        CommandId.SET_MODE: 1,
        CommandId.BEGIN_DOWNLINK: 0,
        CommandId.ENTER_SAFE_MODE: 0,
        CommandId.RESET: 0,
    }
)
"""Exact argument size of each command, bytes. A COMMAND payload is the command ID byte
followed by exactly this many argument bytes."""

COMMAND_ID_SIZE: Final = 1
"""Size of the command ID at the start of a COMMAND payload, bytes."""

RESERVED_COMMAND_ID: Final = 0x00
"""Command ID that is never assigned. An ACK names it when the COMMAND payload was
empty, so there was no ID to echo."""

UINT8_MAX: Final = 0xFF


class DecodeReason(IntEnum):
    """Why a COMMAND payload could not be decoded or its arguments are invalid (#52).

    NACK reason codes 0x01-0x0F. The mode state machine's
    :class:`~pocketsat.flight.RejectReason` holds 0x10-0x1F, so the two enums together
    (:data:`NackReason`) are every NACK reason, with no code in both. 0x00 is reserved:
    it is the ``reason`` byte of an ACK and never a NACK reason. Codes are never
    renumbered or reused.
    """

    UNKNOWN_COMMAND = 0x01
    """The command ID is not an assigned :class:`CommandId` (``0x00`` included). The
    arguments are not inspected."""

    PAYLOAD_TRUNCATED = 0x02
    """The COMMAND payload is shorter than its command's layout, including an empty
    payload with no command ID at all."""

    PAYLOAD_TOO_LONG = 0x03
    """The COMMAND payload has bytes beyond its command's layout."""

    INVALID_ARGUMENT = 0x04
    """An argument has a value with no meaning: for example a SET_MODE byte that is not
    a :class:`~pocketsat.flight.Mode` value. A real mode that cannot be commanded is the
    state machine's ``TARGET_NOT_COMMANDABLE`` (0x14), not this."""


type NackReason = DecodeReason | RejectReason
"""Every NACK reason: decoding and argument errors (:class:`DecodeReason`, 0x01-0x0F)
and mode rejections (:class:`~pocketsat.flight.RejectReason`, 0x10-0x1F)."""

NACK_REASONS: Final[Mapping[int, DecodeReason | RejectReason]] = MappingProxyType(
    {int(reason): reason for reason in (*DecodeReason, *RejectReason)}
)
"""Every NACK reason by wire code, ascending. ``docs/protocol.md`` lists exactly these."""


def _require_uint8(name: str, value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {value!r}")
    if not 0 <= value <= UINT8_MAX:
        raise ValueError(f"{name} out of range 0..{UINT8_MAX}: {value}")


@dataclass(frozen=True)
class Command:
    """A ground-originated request, encoded into a COMMAND payload by the ground station.

    ``command_id`` and ``payload`` are raw so a test can build a command the spacecraft
    must NACK (an unknown ID, a short or long payload, a bad argument). The class
    methods build the five valid commands.

    Attributes:
        command_id: Command identifier, a uint8. Assigned values are :class:`CommandId`
            (``docs/protocol.md``).
        payload: Argument bytes, which follow the command ID in the COMMAND payload.

    Raises:
        TypeError: ``command_id`` is not an ``int`` or ``payload`` is not ``bytes``.
        ValueError: ``command_id`` is outside 0..255.
    """

    command_id: int
    payload: bytes = b""

    def __post_init__(self) -> None:
        _require_uint8("command_id", self.command_id)
        if not isinstance(self.payload, bytes):
            raise TypeError(f"payload must be bytes, got {self.payload!r}")

    @classmethod
    def ping(cls) -> Self:
        """PING: no arguments."""
        return cls(CommandId.PING)

    @classmethod
    def set_mode(cls, target: Mode) -> Self:
        """SET_MODE to ``target``. Any mode is encodable; the spacecraft NACKs a mode
        that is not commandable (``TARGET_NOT_COMMANDABLE``)."""
        if not isinstance(target, Mode):
            raise TypeError(f"target must be a Mode, got {target!r}")
        return cls(CommandId.SET_MODE, bytes([target.value]))

    @classmethod
    def begin_downlink(cls) -> Self:
        """BEGIN_DOWNLINK: no arguments."""
        return cls(CommandId.BEGIN_DOWNLINK)

    @classmethod
    def enter_safe_mode(cls) -> Self:
        """ENTER_SAFE_MODE: no arguments."""
        return cls(CommandId.ENTER_SAFE_MODE)

    @classmethod
    def reset(cls) -> Self:
        """RESET: no arguments."""
        return cls(CommandId.RESET)


def encode_command(command: Command) -> bytes:
    """Encode a COMMAND payload: the command ID byte followed by the argument bytes.

    No validation beyond :class:`Command`'s own, so invalid commands can be sent on
    purpose; the frame codec limits the total size.

    Args:
        command: The command to encode.

    Returns:
        The COMMAND frame payload.
    """
    return bytes([command.command_id]) + command.payload


@dataclass(frozen=True)
class ParsedCommand:
    """A COMMAND payload that decoded and passed argument validation (#52).

    Whether the command is allowed in the current mode is not decided here: that is the
    state machine's (``pocketsat.flight.transition``), called by the dispatcher (#51).

    Attributes:
        command_id: The command.
        target: For SET_MODE, the requested mode (any :class:`~pocketsat.flight.Mode`,
            commandable or not); ``None`` for every other command.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``target`` is missing for SET_MODE or given for another command.
    """

    command_id: CommandId
    target: Mode | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.command_id, CommandId):
            raise TypeError(f"command_id must be a CommandId, got {self.command_id!r}")
        if self.target is not None and not isinstance(self.target, Mode):
            raise TypeError(f"target must be a Mode or None, got {self.target!r}")
        if (self.command_id is CommandId.SET_MODE) != (self.target is not None):
            raise ValueError(f"target is required for SET_MODE and only for it, got {self!r}")


@dataclass(frozen=True)
class MalformedCommand:
    """A COMMAND payload that could not be decoded or has an invalid argument (#52).

    The dispatcher (#51) answers it with a NACK carrying ``command_id`` and ``reason``.

    Attributes:
        command_id: The received command ID byte, echoed in the NACK, assigned or not;
            :data:`RESERVED_COMMAND_ID` (``0x00``) when the payload was empty.
        reason: Why it was rejected.
    """

    command_id: int
    reason: DecodeReason


def decode_command(payload: bytes) -> ParsedCommand | MalformedCommand:
    """Decode and validate a COMMAND payload (the frame's payload, not the whole frame).

    Never raises for any ``bytes``: every problem becomes a :class:`MalformedCommand`.
    Checks run in this order, so the reason is the first that applies:

    1. Empty payload: ``PAYLOAD_TRUNCATED``, command ID ``0x00``.
    2. Command ID not a :class:`CommandId`: ``UNKNOWN_COMMAND`` (arguments not read).
    3. Fewer argument bytes than the layout: ``PAYLOAD_TRUNCATED``.
    4. More argument bytes than the layout: ``PAYLOAD_TOO_LONG``.
    5. An argument value with no meaning (a SET_MODE byte that is not a mode):
       ``INVALID_ARGUMENT``.

    Args:
        payload: The COMMAND frame payload.

    Returns:
        The parsed command, or why it is malformed.
    """
    if not payload:
        return MalformedCommand(RESERVED_COMMAND_ID, DecodeReason.PAYLOAD_TRUNCATED)
    code = payload[0]
    try:
        command_id = CommandId(code)
    except ValueError:
        return MalformedCommand(code, DecodeReason.UNKNOWN_COMMAND)
    arguments = payload[COMMAND_ID_SIZE:]
    expected = COMMAND_ARGUMENT_SIZES[command_id]
    if len(arguments) < expected:
        return MalformedCommand(code, DecodeReason.PAYLOAD_TRUNCATED)
    if len(arguments) > expected:
        return MalformedCommand(code, DecodeReason.PAYLOAD_TOO_LONG)
    if command_id is CommandId.SET_MODE:
        try:
            target = Mode(arguments[0])
        except ValueError:
            return MalformedCommand(code, DecodeReason.INVALID_ARGUMENT)
        return ParsedCommand(command_id, target)
    return ParsedCommand(command_id)


ACK_FORMAT: Final = ">HBB"
"""``struct`` format of the ACK payload: sequence (uint16), command ID (uint8), reason
(uint8). Big-endian, no padding."""

ACK_PAYLOAD_SIZE: Final = struct.calcsize(ACK_FORMAT)
"""ACK payload size, bytes (4), for both ACK and NACK."""

ACK_FRAME_SIZE: Final = MIN_FRAME_SIZE + ACK_PAYLOAD_SIZE
"""Size of a whole ACK frame on the wire, bytes (14)."""

ACK_REASON: Final = 0x00
"""The ``reason`` byte of an ACK (accepted). Never a NACK reason."""


class AckDecodeError(ValueError):
    """An ACK payload has the wrong size or an unknown reason code."""


@dataclass(frozen=True)
class CommandAck:
    """The answer to one COMMAND frame, carried in an ACK frame (#52): an ACK, or a
    NACK with a reason.

    Built by the dispatcher (#51), decoded by the ground station. ACK and NACK share the
    ACK frame type (``0x03``) and one fixed 4-byte layout; ``reason`` tells them apart.

    Attributes:
        sequence: The sequence number of the COMMAND frame being answered (not the ACK
            frame's own downlink sequence), ``0..0xFFFF``.
        command_id: The command ID byte received, echoed whether assigned or not;
            ``0x00`` when the COMMAND payload was empty.
        reason: ``None`` for an ACK; the NACK reason otherwise.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``sequence`` or ``command_id`` is out of range.
    """

    sequence: int
    command_id: int
    reason: NackReason | None = None

    def __post_init__(self) -> None:
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise TypeError(f"sequence must be an int, got {self.sequence!r}")
        if not 0 <= self.sequence <= MAX_SEQUENCE:
            raise ValueError(f"sequence out of range 0..{MAX_SEQUENCE}: {self.sequence}")
        _require_uint8("command_id", self.command_id)
        if self.reason is not None and not isinstance(self.reason, DecodeReason | RejectReason):
            raise TypeError(
                f"reason must be a DecodeReason, a RejectReason, or None, got {self.reason!r}"
            )

    @property
    def accepted(self) -> bool:
        """True for an ACK, False for a NACK."""
        return self.reason is None


def encode_ack(ack: CommandAck) -> bytes:
    """Encode an ACK frame payload.

    Args:
        ack: The ACK or NACK.

    Returns:
        The :data:`ACK_PAYLOAD_SIZE`-byte payload: sequence, command ID, reason
        (:data:`ACK_REASON` for an ACK).
    """
    reason = ACK_REASON if ack.reason is None else int(ack.reason)
    return struct.pack(ACK_FORMAT, ack.sequence, ack.command_id, reason)


def decode_ack(payload: bytes) -> CommandAck:
    """Decode an ACK frame payload (the frame's payload, not the whole frame).

    Args:
        payload: Exactly :data:`ACK_PAYLOAD_SIZE` bytes.

    Returns:
        The ACK or NACK.

    Raises:
        AckDecodeError: The payload has the wrong size, or the reason code is neither
            ``0x00`` nor a code in :data:`NACK_REASONS`.
    """
    if len(payload) != ACK_PAYLOAD_SIZE:
        raise AckDecodeError(f"ACK payload must be {ACK_PAYLOAD_SIZE} bytes, got {len(payload)}")
    sequence, command_id, code = struct.unpack(ACK_FORMAT, payload)
    if code == ACK_REASON:
        return CommandAck(sequence, command_id)
    try:
        reason = NACK_REASONS[code]
    except KeyError:
        raise AckDecodeError(f"unknown NACK reason code 0x{code:02X}") from None
    return CommandAck(sequence, command_id, reason)


# --- Telemetry (#54) -------------------------------------------------------------------


class TelemetryFlags(IntFlag):
    """Bits of the telemetry ``flags`` field (uint16), grouped by subsystem (#54).

    The single source of truth for the flag bits: member names are the readings flag
    names (``pocketsat.spacecraft.snapshots.READINGS_FLAGS``), and the bit table in
    ``docs/protocol.md`` is checked against this enum. Groups: power bits 0-3, thermal
    4-7, attitude 8-11, payload and comms 12-15 (:data:`TELEMETRY_FLAG_GROUPS`).

    Wire-contract rules: a bit is never reassigned; a new flag takes a reserved bit in its
    subsystem's group (reserved bits are sent as 0 and receivers ignore unknown bits, so
    no protocol version change is needed); a removed flag leaves its bit reserved, never
    reused; reassigning or reusing a bit requires a protocol version change and an ADR.
    """

    low_battery = 1 << 0
    """Power: ``PowerReadings.low_battery``."""

    critical_battery = 1 << 1
    """Power: ``PowerReadings.critical_battery``."""

    over_temp = 1 << 4
    """Thermal: ``ThermalReadings.over_temp``."""

    under_temp = 1 << 5
    """Thermal: ``ThermalReadings.under_temp``."""


TELEMETRY_FLAG_GROUPS: Final[Mapping[str, range]] = MappingProxyType(
    {
        "power": range(0, 4),
        "thermal": range(4, 8),
        "attitude": range(8, 12),
        "payload": range(12, 16),
        "comms": range(12, 16),
    }
)
"""Flag bits reserved for each subsystem's readings flags. Payload and comms share one
group."""

KNOWN_FLAGS_MASK: Final = int(
    TelemetryFlags.low_battery
    | TelemetryFlags.critical_battery
    | TelemetryFlags.over_temp
    | TelemetryFlags.under_temp
)
"""Every assigned flag bit. The decoder ignores (clears) any other bit."""

RADIO_MODE_CODES: Final[Mapping[RadioMode, int]] = MappingProxyType(
    {RadioMode.OFF: 0, RadioMode.RX_ONLY: 1, RadioMode.RX_TX: 2}
)
"""Wire value of each radio mode (``CommsReadings.radio_mode``)."""

ATTITUDE_STATE_CODES: Final[Mapping[AttitudeState, int]] = MappingProxyType(
    {AttitudeState.TUMBLING: 0, AttitudeState.DETUMBLING: 1, AttitudeState.STABILIZED: 2}
)
"""Wire value of each attitude state (``AttitudeReadings.state``)."""

PAYLOAD_STATE_CODES: Final[Mapping[PayloadState, int]] = MappingProxyType(
    {PayloadState.OFF: 0, PayloadState.IDLE: 1, PayloadState.ACQUIRING: 2}
)
"""Wire value of each payload state (``PayloadReadings.state``)."""

TELEMETRY_FIELDS: Final[tuple[tuple[str, str], ...]] = (
    ("uptime_ms", "I"),
    ("boot_count", "H"),
    ("flags", "H"),
    ("mode", "B"),
    ("radio_mode", "B"),
    ("attitude_state", "B"),
    ("payload_state", "B"),
    ("bus_mv", "H"),
    ("soc_permille", "H"),
    ("battery_centi_c", "h"),
    ("electronics_centi_c", "h"),
    ("buffered_bytes", "I"),
    ("pointing_error_centi_deg", "H"),
)
"""Wire fields in payload order, with their ``struct`` codes (``I`` uint32, ``H``
uint16, ``h`` int16, ``B`` uint8). Every field sits at an offset that is a multiple of
its size."""

TELEMETRY_FORMAT: Final = ">" + "".join(code for _, code in TELEMETRY_FIELDS)
"""``struct`` format of the TELEMETRY payload: big-endian, no padding."""

TELEMETRY_PAYLOAD_SIZE: Final = struct.calcsize(TELEMETRY_FORMAT)
"""TELEMETRY payload size, bytes (26)."""

TELEMETRY_FRAME_SIZE: Final = MIN_FRAME_SIZE + TELEMETRY_PAYLOAD_SIZE
"""Size of a whole TELEMETRY frame on the wire, bytes (36): the payload plus the 10-byte
frame overhead."""

UINT16_MAX: Final = 0xFFFF
UINT32_MAX: Final = 0xFFFF_FFFF
INT16_MIN: Final = -0x8000
INT16_MAX: Final = 0x7FFF

BUS_V_SCALE: Final = 1000
"""``bus_v`` (volts) * 1000 → ``bus_mv`` (millivolts)."""

SOC_SCALE: Final = 1000
"""``soc`` (fraction 0..1) * 1000 → ``soc_permille`` (0.1 %)."""

SOC_PERMILLE_MAX: Final = 1000
"""Largest ``soc_permille`` value (100.0 %)."""

TEMPERATURE_SCALE: Final = 100
"""Temperature (°C) * 100 → 0.01 °C."""

POINTING_SCALE: Final = 100
"""``pointing_error_deg`` * 100 → 0.01°."""

POINTING_CENTI_DEG_MAX: Final = 18000
"""Largest ``pointing_error_centi_deg`` value (180.00°)."""


class TelemetryDecodeError(ValueError):
    """A TELEMETRY payload has the wrong size or an unknown enumerated value."""


@dataclass(frozen=True)
class FlightComputerTelemetryState:
    """The flight computer's own state carried in telemetry (#54).

    Supplied by the flight computer (:class:`~pocketsat.flight.FlightComputer`, #101):
    the uptime counter and boot count from #49 and the current mode from #47, assembled
    by the telemetry scheduler (#55). Validated on construction.

    Attributes:
        uptime_ms: Milliseconds since the last power-on or RESET (#49). Non-negative. The
            wire field is uint32 and wraps modulo 2**32 (about 49.7 days).
        mode: Current flight mode. Its value is the wire value: ``Mode`` is an
            ``IntEnum`` whose values are the telemetry encoding (#47).
        boot_count: Boots since first power-on; persists across RESET (#49).
            Non-negative. The wire field is uint16 and saturates at 65535.

    Raises:
        TypeError: ``uptime_ms`` or ``boot_count`` is not an ``int`` (``bool`` is
            rejected), or ``mode`` is not a :class:`~pocketsat.flight.Mode` (a plain
            ``int`` is rejected).
        ValueError: ``uptime_ms`` or ``boot_count`` is negative.
    """

    uptime_ms: int
    mode: Mode
    boot_count: int

    def __post_init__(self) -> None:
        for name in ("uptime_ms", "boot_count"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an int, got {value!r}")
        if not isinstance(self.mode, Mode):
            raise TypeError(f"mode must be a Mode, got {self.mode!r}")
        if self.uptime_ms < 0:
            raise ValueError(f"uptime_ms must be non-negative, got {self.uptime_ms}")
        if self.boot_count < 0:
            raise ValueError(f"boot_count must be non-negative, got {self.boot_count}")


@dataclass(frozen=True)
class Telemetry:
    """Spacecraft state decoded from a TELEMETRY payload by the ground station (#54).

    Every value is a reported value (ADR-0004 §7) at wire resolution: for example
    ``bus_v`` to 1 mV and temperatures to 0.01 °C. A value that saturated on encoding
    decodes as the field limit. Field names match the source readings fields.

    Attributes:
        uptime_ms: Flight computer uptime, milliseconds, modulo 2**32.
        boot_count: Boot count, saturated at 65535.
        flags: Readings flags; unknown bits are cleared.
        mode: Flight mode.
        radio_mode: ``CommsReadings.radio_mode``.
        attitude_state: ``AttitudeReadings.state``.
        payload_state: ``PayloadReadings.state``.
        bus_v: ``PowerReadings.bus_v``, volts, 0..65.535.
        soc: ``PowerReadings.soc``, fraction, 0..1 in steps of 0.001.
        battery_c: ``ThermalReadings.battery_c``, °C, -327.68..327.67.
        electronics_c: ``ThermalReadings.electronics_c``, °C, -327.68..327.67.
        buffered_bytes: ``PayloadReadings.buffered_bytes``, bytes, saturated at
            2**32 - 1.
        pointing_error_deg: ``AttitudeReadings.pointing_error_deg``, degrees, 0..180.
    """

    uptime_ms: int
    boot_count: int
    flags: TelemetryFlags
    mode: Mode
    radio_mode: RadioMode
    attitude_state: AttitudeState
    payload_state: PayloadState
    bus_v: float
    soc: float
    battery_c: float
    electronics_c: float
    buffered_bytes: int
    pointing_error_deg: float


def quantize(value: float, scale: int, minimum: int, maximum: int) -> int:
    """Convert a reported physical value to its wire integer (#54, ADR-0006).

    The rule, identical on every platform and in C:

    1. ``q = value * scale``: one IEEE 754 double multiplication, correctly rounded.
    2. Round ``q`` to the nearest integer, halves away from zero (C ``lround``):
       ``n = floor(|q|)``, plus 1 if ``|q| - n >= 0.5``, with the sign of ``q``.
    3. Saturate: clamp ``n`` to ``minimum..maximum``.

    ``+inf`` saturates to ``maximum`` and ``-inf`` to ``minimum``. NaN has no wire value
    and is rejected. An ``int`` value is scaled exactly and then saturated.

    Args:
        value: Reported value in its readings unit (for example volts).
        scale: Wire units per readings unit (for example 1000 for millivolts).
        minimum: Smallest wire value.
        maximum: Largest wire value.

    Returns:
        The wire integer, ``minimum..maximum``.

    Raises:
        TypeError: ``value`` is not an ``int`` or ``float`` (``bool`` is rejected).
        ValueError: ``value`` is NaN.
    """
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"value must be a float, got {value!r}")
    if isinstance(value, int):
        return min(max(value * scale, minimum), maximum)
    if math.isnan(value):
        raise ValueError("value is NaN, which has no telemetry encoding")
    scaled = value * scale
    if scaled >= maximum:
        return maximum
    if scaled <= minimum:
        return minimum
    magnitude = abs(scaled)
    whole = math.floor(magnitude)
    if magnitude - whole >= 0.5:
        whole += 1
    return whole if scaled >= 0 else -whole


def _saturate_count(name: str, value: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {value!r}")
    return min(max(value, 0), maximum)


def _flag(name: str, value: bool, flag: TelemetryFlags) -> TelemetryFlags:
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be a bool, got {value!r}")
    return flag if value else TelemetryFlags(0)


def _require_type(name: str, value: object, expected: type) -> None:
    # Static typing already rejects a truth record; this also catches untyped callers.
    if type(value) is not expected:
        raise TypeError(f"{name} must be a {expected.__name__}, got {type(value).__name__}")


def telemetry_flags(power: PowerReadings, thermal: ThermalReadings) -> TelemetryFlags:
    """Collect the readings flags into :class:`TelemetryFlags`.

    Args:
        power: Power readings (``low_battery``, ``critical_battery``).
        thermal: Thermal readings (``over_temp``, ``under_temp``).

    Returns:
        The set flags; reserved bits are 0.

    Raises:
        TypeError: A flag is not a ``bool``.
    """
    return (
        _flag("low_battery", power.low_battery, TelemetryFlags.low_battery)
        | _flag("critical_battery", power.critical_battery, TelemetryFlags.critical_battery)
        | _flag("over_temp", thermal.over_temp, TelemetryFlags.over_temp)
        | _flag("under_temp", thermal.under_temp, TelemetryFlags.under_temp)
    )


def encode_telemetry(
    flight_computer: FlightComputerTelemetryState,
    *,
    power: PowerReadings,
    thermal: ThermalReadings,
    attitude: AttitudeReadings,
    payload: PayloadReadings,
    comms: CommsReadings,
) -> bytes:
    """Encode a TELEMETRY payload from reported values only (#54, ADR-0004 §7).

    Only readings records are accepted, so passing a truth record (for example
    :class:`~pocketsat.spacecraft.snapshots.PowerTruth`) is a type error under mypy and
    a ``TypeError`` at run time. Each field is converted as documented in
    ``docs/protocol.md``: physical values with :func:`quantize` (round half away from
    zero, then saturate), ``uptime_ms`` modulo 2**32, counts saturated.

    Args:
        flight_computer: Uptime, mode, and boot count from the flight computer.
        power: Power readings: ``bus_v``, ``soc``, and the battery flags.
        thermal: Thermal readings: ``battery_c``, ``electronics_c``, and the
            temperature flags.
        attitude: Attitude readings: ``pointing_error_deg`` and ``state``.
        payload: Payload readings: ``buffered_bytes`` and ``state``.
        comms: Communications readings: ``radio_mode``.

    Returns:
        The :data:`TELEMETRY_PAYLOAD_SIZE`-byte payload, big-endian.

    Raises:
        TypeError: An argument is not the expected record type, or a field has the
            wrong type.
        ValueError: A physical value is NaN.
    """
    _require_type("flight_computer", flight_computer, FlightComputerTelemetryState)
    _require_type("power", power, PowerReadings)
    _require_type("thermal", thermal, ThermalReadings)
    _require_type("attitude", attitude, AttitudeReadings)
    _require_type("payload", payload, PayloadReadings)
    _require_type("comms", comms, CommsReadings)
    return struct.pack(
        TELEMETRY_FORMAT,
        flight_computer.uptime_ms & UINT32_MAX,
        min(flight_computer.boot_count, UINT16_MAX),
        telemetry_flags(power, thermal),
        flight_computer.mode.value,
        RADIO_MODE_CODES[comms.radio_mode],
        ATTITUDE_STATE_CODES[attitude.state],
        PAYLOAD_STATE_CODES[payload.state],
        quantize(power.bus_v, BUS_V_SCALE, 0, UINT16_MAX),
        quantize(power.soc, SOC_SCALE, 0, SOC_PERMILLE_MAX),
        quantize(thermal.battery_c, TEMPERATURE_SCALE, INT16_MIN, INT16_MAX),
        quantize(thermal.electronics_c, TEMPERATURE_SCALE, INT16_MIN, INT16_MAX),
        _saturate_count("buffered_bytes", payload.buffered_bytes, UINT32_MAX),
        quantize(attitude.pointing_error_deg, POINTING_SCALE, 0, POINTING_CENTI_DEG_MAX),
    )


def _lookup[E](name: str, codes: Mapping[E, int], code: int) -> E:
    for member, value in codes.items():
        if value == code:
            return member
    raise TelemetryDecodeError(f"unknown {name} value {code}")


def decode_telemetry(payload: bytes) -> Telemetry:
    """Decode a TELEMETRY payload (the frame's payload bytes, not the whole frame).

    Unknown flag bits are ignored (cleared). Physical values are divided by their scale
    (one IEEE 754 division), so they carry the wire resolution.

    Args:
        payload: Exactly :data:`TELEMETRY_PAYLOAD_SIZE` bytes.

    Returns:
        The decoded telemetry.

    Raises:
        TelemetryDecodeError: The payload has the wrong size, or the mode, radio mode,
            attitude state, or payload state value is unknown.
    """
    if len(payload) != TELEMETRY_PAYLOAD_SIZE:
        raise TelemetryDecodeError(
            f"telemetry payload must be {TELEMETRY_PAYLOAD_SIZE} bytes, got {len(payload)}"
        )
    (
        uptime_ms,
        boot_count,
        flags,
        mode,
        radio_mode,
        attitude_state,
        payload_state,
        bus_mv,
        soc_permille,
        battery_centi_c,
        electronics_centi_c,
        buffered_bytes,
        pointing_error_centi_deg,
    ) = struct.unpack(TELEMETRY_FORMAT, payload)
    try:
        decoded_mode = Mode(mode)
    except ValueError:
        raise TelemetryDecodeError(f"unknown mode value {mode}") from None
    return Telemetry(
        uptime_ms=uptime_ms,
        boot_count=boot_count,
        flags=TelemetryFlags(flags & KNOWN_FLAGS_MASK),
        mode=decoded_mode,
        radio_mode=_lookup("radio_mode", RADIO_MODE_CODES, radio_mode),
        attitude_state=_lookup("attitude_state", ATTITUDE_STATE_CODES, attitude_state),
        payload_state=_lookup("payload_state", PAYLOAD_STATE_CODES, payload_state),
        bus_v=bus_mv / BUS_V_SCALE,
        soc=soc_permille / SOC_SCALE,
        battery_c=battery_centi_c / TEMPERATURE_SCALE,
        electronics_c=electronics_centi_c / TEMPERATURE_SCALE,
        buffered_bytes=buffered_bytes,
        pointing_error_deg=pointing_error_centi_deg / POINTING_SCALE,
    )


# --- DATA (#56) ------------------------------------------------------------------------


DATA_CHUNK_ID_FORMAT: Final = ">I"
"""``struct`` format of the chunk ID at the start of a DATA payload: uint32,
big-endian."""

DATA_CHUNK_ID_SIZE: Final = struct.calcsize(DATA_CHUNK_ID_FORMAT)
"""Size of the chunk ID in a DATA payload, bytes (4). Equal to
``pocketsat.spacecraft.config.CHUNK_ID_SIZE_BYTES``, which sizes the largest chunk."""

MIN_DATA_PAYLOAD_SIZE: Final = DATA_CHUNK_ID_SIZE + 1
"""Smallest DATA payload, bytes (5): the chunk ID and at least one content byte
(``PayloadConfig.chunk_size_bytes`` is at least 1)."""


def data_frame_size(chunk_size_bytes: int) -> int:
    """Size on the wire of the DATA frame that carries a chunk of ``chunk_size_bytes``.

    The 10-byte frame overhead, the 4-byte chunk ID, and the content: 78 bytes for the
    default 64-byte chunk (``docs/protocol.md``, "DATA payload").
    """
    return MIN_FRAME_SIZE + DATA_CHUNK_ID_SIZE + chunk_size_bytes


class DataDecodeError(ValueError):
    """A DATA payload is too short to carry a chunk ID and content."""


@dataclass(frozen=True)
class DataChunk:
    """One payload data chunk as carried in a DATA frame (#56).

    Built by the flight computer's downlink session from the payload's chunk store
    (#43), decoded by the ground station. A receiver regenerates the expected content
    from the ID with ``pocketsat.spacecraft.payload.chunk_content`` and the chunk size,
    so it can detect corruption, loss, or duplication.

    Attributes:
        chunk_id: The chunk's ID, ``0..MAX_CHUNK_ID`` (uint32).
        content: The chunk's bytes, 1..``MAX_CHUNK_SIZE_BYTES`` long, so the whole
            payload fits a frame.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``chunk_id`` is out of range, or ``content`` is empty or too long.
    """

    chunk_id: int
    content: bytes

    def __post_init__(self) -> None:
        chunk_id = self.chunk_id
        if isinstance(chunk_id, bool) or not isinstance(chunk_id, int):
            raise TypeError(f"chunk_id must be an int, got {chunk_id!r}")
        if not 0 <= chunk_id <= MAX_CHUNK_ID:
            raise ValueError(f"chunk_id out of range 0..{MAX_CHUNK_ID}: {chunk_id}")
        if not isinstance(self.content, bytes):
            raise TypeError(f"content must be bytes, got {type(self.content).__name__}")
        if not 1 <= len(self.content) <= MAX_CHUNK_SIZE_BYTES:
            raise ValueError(
                f"content must be 1..{MAX_CHUNK_SIZE_BYTES} bytes, got {len(self.content)}"
            )


def encode_data(chunk: DataChunk) -> bytes:
    """Encode a DATA frame payload: the chunk ID (uint32, big-endian), then the content.

    Args:
        chunk: The chunk to send.

    Returns:
        ``DATA_CHUNK_ID_SIZE + len(chunk.content)`` bytes.
    """
    return struct.pack(DATA_CHUNK_ID_FORMAT, chunk.chunk_id) + chunk.content


def decode_data(payload: bytes) -> DataChunk:
    """Decode a DATA frame payload (the frame's payload, not the whole frame).

    The content is returned as received; checking it against the expected
    ``chunk_content`` is the receiver's job, because only it knows the chunk size.

    Args:
        payload: At least :data:`MIN_DATA_PAYLOAD_SIZE` bytes.

    Returns:
        The chunk ID and content.

    Raises:
        DataDecodeError: The payload is shorter than :data:`MIN_DATA_PAYLOAD_SIZE` (no
            room for a chunk ID and at least one content byte).
    """
    if len(payload) < MIN_DATA_PAYLOAD_SIZE:
        raise DataDecodeError(
            f"DATA payload must be at least {MIN_DATA_PAYLOAD_SIZE} bytes, got {len(payload)}"
        )
    (chunk_id,) = struct.unpack_from(DATA_CHUNK_ID_FORMAT, payload)
    return DataChunk(chunk_id, bytes(payload[DATA_CHUNK_ID_SIZE:]))
