"""Spacecraft configuration and starting state (ADR-0005).

Two separate collections, both made of per-subsystem frozen records:

- :class:`SpacecraftConfig` describes what the spacecraft *is*: capacities, array
  size, loads, thresholds, heater setpoints.
- :class:`SpacecraftInitialState` describes where it *starts*: battery charge,
  temperatures, attitude, buffer fill.

Both are supplied when a target is created (for example
``SilTarget(config=..., initial=...)``), and ``reset(seed)`` returns to exactly what
the target was built with. Each subsystem is built with its own settings record and
starting record, and ``reset(rng)`` returns to them; the ``Subsystem`` protocol does
not change.

Starting values are plain values, never random: campaigns generate varied values and
pass them in.
"""

import math
from dataclasses import dataclass, field
from typing import Final

from pocketsat.frame import MAX_PAYLOAD_SIZE
from pocketsat.targets.base import ABSOLUTE_ZERO_C

# --- Validation helpers ----------------------------------------------------------------


def _require_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return float(value)


def _require_in_range(name: str, value: object, low: float, high: float) -> None:
    number = _require_number(name, value)
    if not low <= number <= high:
        raise ValueError(f"{name} must be in {low}..{high}, got {number}")


def _require_records(owner: object, **kinds: type) -> None:
    for name, kind in kinds.items():
        value = getattr(owner, name)
        if not isinstance(value, kind):
            raise TypeError(f"{name} must be a {kind.__name__}, got {value!r}")


# --- Settings --------------------------------------------------------------------------


@dataclass(frozen=True)
class PowerConfig:
    """Power settings for the battery and solar model (#35).

    The power loads task (#36) adds its fields here. Every default is provisional: the
    power and thermal budget (#72) calibrates them. The defaults describe a small
    2S lithium-ion battery charged by a body-mounted array.

    Attributes:
        battery_capacity_wh: Usable battery energy from empty (SOC 0) to full (SOC 1),
            watt-hours. Positive. Default 20.0.
        solar_array_w: Solar array output at ideal pointing (pointing error 0°) in
            sunlight, watts. Non-negative. Default 8.0.
        base_load_w: Always-on electrical load (avionics, receiver), watts.
            Non-negative. Default 2.0. #36 adds the per-subsystem draws on top.
        battery_empty_v: Bus voltage at SOC 0, volts. Positive. Default 6.0.
        battery_full_v: Bus voltage at SOC 1, volts. Above ``battery_empty_v``.
            Default 8.4. Bus voltage is linear in SOC between the two.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite or is out of range.
    """

    battery_capacity_wh: float = 20.0
    solar_array_w: float = 8.0
    base_load_w: float = 2.0
    battery_empty_v: float = 6.0
    battery_full_v: float = 8.4

    def __post_init__(self) -> None:
        if _require_number("battery_capacity_wh", self.battery_capacity_wh) <= 0:
            raise ValueError(
                f"battery_capacity_wh must be positive, got {self.battery_capacity_wh}"
            )
        for name in ("solar_array_w", "base_load_w"):
            if _require_number(name, getattr(self, name)) < 0:
                raise ValueError(f"{name} must be non-negative, got {getattr(self, name)}")
        empty_v = _require_number("battery_empty_v", self.battery_empty_v)
        full_v = _require_number("battery_full_v", self.battery_full_v)
        if empty_v <= 0:
            raise ValueError(f"battery_empty_v must be positive, got {empty_v}")
        if full_v <= empty_v:
            raise ValueError(
                f"battery_full_v must be above battery_empty_v ({empty_v}), got {full_v}"
            )


@dataclass(frozen=True)
class ThermalConfig:
    """Thermal settings. Fields are added by the thermal model (#38) and the thermal
    limit flags task (#39)."""


@dataclass(frozen=True)
class AttitudeConfig:
    """Attitude settings. Fields are added by the attitude model (#41)."""


CHUNK_ID_SIZE_BYTES: Final = 4
"""Size of the chunk ID (uint32, big-endian) at the start of a DATA frame payload
(#56)."""

MAX_CHUNK_SIZE_BYTES: Final = MAX_PAYLOAD_SIZE - CHUNK_ID_SIZE_BYTES
"""Largest allowed chunk, bytes: a chunk's content must fit in one DATA frame payload
after the chunk ID. ``MAX_PAYLOAD_SIZE`` (65 535, the frame length field's limit,
:mod:`pocketsat.frame`) minus :data:`CHUNK_ID_SIZE_BYTES`, so 65 531."""


@dataclass(frozen=True)
class PayloadConfig:
    """Payload settings (#43).

    The defaults are provisional: illustrative values sized so a chunk's DATA frame
    fits in a tick's nominal transmit capacity. The power and thermal budget (#72)
    finalizes the power draws.

    Attributes:
        buffer_capacity_bytes: Buffer capacity, bytes. A positive whole number of
            chunks (a multiple of ``chunk_size_bytes``), so the buffer fills exactly
            on a chunk boundary.
        chunk_size_bytes: Size of every chunk, bytes. Positive and at most
            :data:`MAX_CHUNK_SIZE_BYTES`, so a chunk fits in one DATA frame (#56).
        data_rate_bytes_per_s: True acquisition data rate while acquiring, bytes per
            second. Non-negative.
        idle_power_w: Draw while commanded on but not acquiring, watts. Non-negative.
        acquiring_power_w: Draw while acquiring, watts. Non-negative.

    Raises:
        TypeError: A byte field is not an int, or a power field is not a number.
        ValueError: A field is out of range, or the capacity is not a whole number of
            chunks.
    """

    buffer_capacity_bytes: int = 65_536
    chunk_size_bytes: int = 64
    data_rate_bytes_per_s: int = 100
    idle_power_w: float = 0.5
    acquiring_power_w: float = 2.0

    def __post_init__(self) -> None:
        for name in ("buffer_capacity_bytes", "chunk_size_bytes", "data_rate_bytes_per_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an int, got {value!r}")
        if not 0 < self.chunk_size_bytes <= MAX_CHUNK_SIZE_BYTES:
            raise ValueError(
                f"chunk_size_bytes must be in 1..{MAX_CHUNK_SIZE_BYTES}, "
                f"got {self.chunk_size_bytes}"
            )
        if self.buffer_capacity_bytes <= 0 or self.buffer_capacity_bytes % self.chunk_size_bytes:
            raise ValueError(
                "buffer_capacity_bytes must be a positive multiple of chunk_size_bytes "
                f"({self.chunk_size_bytes}), got {self.buffer_capacity_bytes}"
            )
        if self.data_rate_bytes_per_s < 0:
            raise ValueError(
                f"data_rate_bytes_per_s must be non-negative, got {self.data_rate_bytes_per_s}"
            )
        for name in ("idle_power_w", "acquiring_power_w"):
            if _require_number(name, getattr(self, name)) < 0:
                raise ValueError(f"{name} must be non-negative, got {getattr(self, name)}")


@dataclass(frozen=True)
class CommsConfig:
    """Communications settings. Fields are added by the communications subsystem (#44)."""


@dataclass(frozen=True)
class SpacecraftConfig:
    """Every subsystem's settings: what the spacecraft is (ADR-0005).

    Each subsystem is built with, and reads, only its own record. The nominal set is
    :data:`NOMINAL_CONFIG`; the stressed set is added by the power and thermal budget
    (#72) once the records have fields.

    Attributes:
        power: Power settings.
        thermal: Thermal settings.
        attitude: Attitude settings.
        payload: Payload settings.
        comms: Communications settings.

    Raises:
        TypeError: A field is not the expected record type.
    """

    power: PowerConfig = field(default_factory=PowerConfig)
    thermal: ThermalConfig = field(default_factory=ThermalConfig)
    attitude: AttitudeConfig = field(default_factory=AttitudeConfig)
    payload: PayloadConfig = field(default_factory=PayloadConfig)
    comms: CommsConfig = field(default_factory=CommsConfig)

    def __post_init__(self) -> None:
        _require_records(
            self,
            power=PowerConfig,
            thermal=ThermalConfig,
            attitude=AttitudeConfig,
            payload=PayloadConfig,
            comms=CommsConfig,
        )


NOMINAL_CONFIG = SpacecraftConfig()
"""The nominal settings. The power and thermal budget (#72) calibrates them."""


# --- Starting state --------------------------------------------------------------------


@dataclass(frozen=True)
class PowerInitial:
    """Where the power subsystem starts.

    Attributes:
        soc: Battery state of charge, 0..1. The default is provisional; the power and
            thermal budget (#72) sets the final value.

    Raises:
        TypeError: ``soc`` is not a number.
        ValueError: ``soc`` is not finite or is outside 0..1.
    """

    soc: float = 0.8

    def __post_init__(self) -> None:
        _require_in_range("soc", self.soc, 0.0, 1.0)


@dataclass(frozen=True)
class ThermalInitial:
    """Where the thermal subsystem starts.

    Attributes:
        battery_c: Battery temperature, °C.
        electronics_c: Electronics temperature, °C.

    Both defaults equal the nominal sunlit ambient (20 °C) and are provisional; the
    power and thermal budget (#72) sets the final values.

    Raises:
        TypeError: A temperature is not a number.
        ValueError: A temperature is not finite or is not above absolute zero.
    """

    battery_c: float = 20.0
    electronics_c: float = 20.0

    def __post_init__(self) -> None:
        for name in ("battery_c", "electronics_c"):
            value = _require_number(name, getattr(self, name))
            if value <= ABSOLUTE_ZERO_C:
                raise ValueError(f"{name} must be above absolute zero, got {value}")


@dataclass(frozen=True)
class AttitudeInitial:
    """Where the attitude subsystem starts.

    Attributes:
        pointing_error_deg: Pointing error from the sun-optimal attitude, 0..180 degrees.
        rate_dps: Angular rate magnitude, degrees per second, non-negative.

    The defaults (stabilized, at rest) are provisional; the attitude model (#41)
    chooses and documents the final values.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite or is out of range.
    """

    pointing_error_deg: float = 0.0
    rate_dps: float = 0.0

    def __post_init__(self) -> None:
        _require_in_range("pointing_error_deg", self.pointing_error_deg, 0.0, 180.0)
        if _require_number("rate_dps", self.rate_dps) < 0:
            raise ValueError(f"rate_dps must be non-negative, got {self.rate_dps}")


@dataclass(frozen=True)
class PayloadInitial:
    """Where the payload subsystem starts.

    Attributes:
        buffer_fill: Fraction of buffer capacity already filled, 0..1. Defaults to
            empty. Chunk IDs always start at 0 (#43): a non-empty start holds
            ``floor(buffer_fill * buffer_capacity_bytes)`` bytes, stored as whole
            chunks with IDs from 0 plus any partial chunk, and counted as produced.

    Raises:
        TypeError: ``buffer_fill`` is not a number.
        ValueError: ``buffer_fill`` is not finite or is outside 0..1.
    """

    buffer_fill: float = 0.0

    def __post_init__(self) -> None:
        _require_in_range("buffer_fill", self.buffer_fill, 0.0, 1.0)


@dataclass(frozen=True)
class SpacecraftInitialState:
    """Every subsystem's starting state: where the spacecraft starts (ADR-0005).

    Covers physical state only; the flight computer always starts in BOOT.
    Communications has no starting record, because it holds no data.

    Attributes:
        power: Power starting state.
        thermal: Thermal starting state.
        attitude: Attitude starting state.
        payload: Payload starting state.

    Raises:
        TypeError: A field is not the expected record type.
    """

    power: PowerInitial = field(default_factory=PowerInitial)
    thermal: ThermalInitial = field(default_factory=ThermalInitial)
    attitude: AttitudeInitial = field(default_factory=AttitudeInitial)
    payload: PayloadInitial = field(default_factory=PayloadInitial)

    def __post_init__(self) -> None:
        _require_records(
            self,
            power=PowerInitial,
            thermal=ThermalInitial,
            attitude=AttitudeInitial,
            payload=PayloadInitial,
        )


DEFAULT_INITIAL_STATE = SpacecraftInitialState()
"""The default starting state."""
