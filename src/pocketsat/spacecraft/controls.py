"""Flight computer to subsystem controls (ADR-0004).

The flight computer produces one frozen :class:`SpacecraftControls` record per tick,
and every subsystem obeys it. Controls produced in tick N take effect in tick N+1.
Fault overrides are merged in by ``SilTarget`` with precedence fault > flight
computer, and act on the same tick they are injected.

Each subsystem reads only its own record (and the overrides that concern it).
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Final

SENSOR_SUBSYSTEMS: Final = frozenset({"power", "thermal", "attitude"})
"""Subsystems whose readings ``sensor_freeze`` can freeze (ADR-0004 §8)."""


class RadioMode(Enum):
    """Commanded radio mode. The radio is full duplex (ADR-0004 §10)."""

    OFF = "off"
    """Receiver and transmitter off."""

    RX_ONLY = "rx_only"
    """Receiver on, transmitter off."""

    RX_TX = "rx_tx"
    """Receiver and transmitter on; every normal flight mode uses this."""


@dataclass(frozen=True)
class PayloadControls:
    """Payload commands from the flight computer.

    Attributes:
        enabled: Acquisition is commanded on. Power, thermal, and attitude inhibits
            still apply (#43).
        release_through_chunk_id: Release (delete) every stored chunk up to and
            including this ID, or ``None`` to release nothing. The payload never
            deletes data on its own (ADR-0004 §13).

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``release_through_chunk_id`` is negative.
    """

    enabled: bool = False
    release_through_chunk_id: int | None = None

    def __post_init__(self) -> None:
        _require_bool("enabled", self.enabled)
        chunk = self.release_through_chunk_id
        if chunk is not None:
            if isinstance(chunk, bool) or not isinstance(chunk, int):
                raise TypeError(f"release_through_chunk_id must be an int or None, got {chunk!r}")
            if chunk < 0:
                raise ValueError(f"release_through_chunk_id must be non-negative, got {chunk}")


@dataclass(frozen=True)
class RadioControls:
    """Radio commands from the flight computer.

    Attributes:
        mode: Commanded radio mode.

    Raises:
        TypeError: ``mode`` is not a :class:`RadioMode`.
    """

    mode: RadioMode = RadioMode.RX_TX

    def __post_init__(self) -> None:
        if not isinstance(self.mode, RadioMode):
            raise TypeError(f"mode must be a RadioMode, got {self.mode!r}")


@dataclass(frozen=True)
class AttitudeControls:
    """Attitude control commands from the flight computer.

    Attributes:
        enabled: Attitude control is on. While off, the disturbance is not corrected.

    Raises:
        TypeError: ``enabled`` is not a bool.
    """

    enabled: bool = True

    def __post_init__(self) -> None:
        _require_bool("enabled", self.enabled)


@dataclass(frozen=True)
class SpacecraftControls:
    """Everything the subsystems obey for one tick (ADR-0004 §1).

    The defaults describe nominal operation: payload off, radio ``RX_TX``, attitude
    control on, no fault overrides. They are a test convenience; after a reset, the
    flight computer's BOOT controls apply to tick 0 (ADR-0004 §2).

    Attributes:
        payload: Payload commands.
        radio: Radio commands.
        attitude: Attitude control commands.
        frozen_sensors: Fault override from ``sensor_freeze``: subsystems whose
            readings are held at their last value. Subset of :data:`SENSOR_SUBSYSTEMS`.
        extra_load_w: Fault override from ``battery_drain``: extra load added to the
            power draw, in watts. Non-negative.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``frozen_sensors`` names a subsystem without sensors, or
            ``extra_load_w`` is negative or not finite.
    """

    payload: PayloadControls = field(default_factory=PayloadControls)
    radio: RadioControls = field(default_factory=RadioControls)
    attitude: AttitudeControls = field(default_factory=AttitudeControls)
    frozen_sensors: frozenset[str] = frozenset()
    extra_load_w: float = 0.0

    def __post_init__(self) -> None:
        for name, kind in (
            ("payload", PayloadControls),
            ("radio", RadioControls),
            ("attitude", AttitudeControls),
        ):
            if not isinstance(getattr(self, name), kind):
                raise TypeError(f"{name} must be a {kind.__name__}, got {getattr(self, name)!r}")
        if not isinstance(self.frozen_sensors, frozenset):
            raise TypeError(f"frozen_sensors must be a frozenset, got {self.frozen_sensors!r}")
        unknown = self.frozen_sensors - SENSOR_SUBSYSTEMS
        if unknown:
            raise ValueError(
                f"frozen_sensors may only name {sorted(SENSOR_SUBSYSTEMS)}, got {sorted(unknown)}"
            )
        load = self.extra_load_w
        if isinstance(load, bool) or not isinstance(load, int | float):
            raise TypeError(f"extra_load_w must be a number, got {load!r}")
        if not math.isfinite(load) or load < 0:
            raise ValueError(f"extra_load_w must be finite and non-negative, got {load}")


def _require_bool(name: str, value: object) -> None:
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be a bool, got {value!r}")
