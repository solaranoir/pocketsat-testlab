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
"""

import math
import struct
from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntFlag
from types import MappingProxyType
from typing import Final

from pocketsat.flight.modes import Mode
from pocketsat.frame import MIN_FRAME_SIZE
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


@dataclass(frozen=True)
class Command:
    """A ground-originated request, encoded into a COMMAND frame by the ground station.

    Attributes:
        command_id: Command identifier. Assigned values are listed in ``docs/protocol.md``.
        payload: Command-specific argument bytes.
    """

    command_id: int
    payload: bytes = b""


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
