"""Subsystem snapshot contracts (ADR-0004 §7, issue #76).

Every subsystem's snapshot holds two frozen records:

- a **truth** record (for example :class:`PowerTruth`): the true physical values.
  Physics, and coupling between subsystems, reads these.
- a **readings** record (for example :class:`PowerReadings`): the values the
  spacecraft's sensors report, possibly noisy or frozen, plus the limit flags.
  Decisions (inhibits, the flight computer) and telemetry read these.

Payload and communications have no sensors, so their readings always equal their
truth (:attr:`ContractSnapshot.mirrored`).

This module is the contract between subsystems: each subsystem ticket (#35, #36, #38,
#39, #41, #43, #44) implements its records exactly as defined here, and adds fields
only by extending this module in the same change. Field names follow the naming and
units convention in ``docs/architecture.md`` §4.1; the cross-subsystem reads are
documented in ``docs/spacecraft.md``.

The records do not validate their values: they are built every tick, and each
subsystem's own tests check the documented ranges and signs.
"""

from collections.abc import Callable, Mapping
from dataclasses import dataclass, fields
from enum import Enum
from operator import attrgetter
from types import MappingProxyType
from typing import Any, ClassVar, Final, Self

from pocketsat.spacecraft.base import SubsystemSnapshot
from pocketsat.spacecraft.controls import RadioMode

# --- States ----------------------------------------------------------------------------


class AttitudeState(Enum):
    """Attitude control state (#41). Decided with hysteresis on the angular rate."""

    TUMBLING = "tumbling"
    """Angular rate above the high threshold."""

    DETUMBLING = "detumbling"
    """Attitude control is reducing the rate."""

    STABILIZED = "stabilized"
    """Rate below the low threshold and pointing error within limits."""


class PayloadState(Enum):
    """Payload state (#43)."""

    OFF = "off"
    """Not commanded on (``controls.payload.enabled`` is false)."""

    IDLE = "idle"
    """Commanded on, but not acquiring (an inhibit applies or the buffer is full)."""

    ACQUIRING = "acquiring"
    """Acquiring data into the buffer."""


# --- Snapshot base ---------------------------------------------------------------------


@dataclass(frozen=True)
class ContractSnapshot[T, R](SubsystemSnapshot):
    """A subsystem snapshot made of a truth record and a readings record (ADR-0004 §7).

    Subclasses set :attr:`truth_type` and :attr:`readings_type`, and set
    :attr:`mirrored` for subsystems without sensors.

    Raises:
        TypeError: ``truth`` or ``readings`` is not the subsystem's record type.
        ValueError: The subsystem is mirrored and the readings differ from the truth.
    """

    truth: T
    """True values. Physics and cross-subsystem physical coupling read these."""

    readings: R
    """Reported values and flags. Decisions and telemetry read these."""

    truth_type: ClassVar[type[Any]]
    """The subsystem's truth record type."""

    readings_type: ClassVar[type[Any]]
    """The subsystem's readings record type."""

    mirrored: ClassVar[bool] = False
    """True for subsystems without sensors, whose readings always equal their truth."""

    def __post_init__(self) -> None:
        if type(self.truth) is not self.truth_type:
            raise TypeError(
                f"truth must be a {self.truth_type.__name__}, got {type(self.truth).__name__}"
            )
        if type(self.readings) is not self.readings_type:
            raise TypeError(
                f"readings must be a {self.readings_type.__name__}, "
                f"got {type(self.readings).__name__}"
            )
        if self.mirrored and _values(self.truth) != _values(self.readings):
            raise ValueError(f"{type(self).__name__} readings must equal its truth")

    @classmethod
    def from_truth(cls, truth: T) -> Self:
        """Build a mirrored snapshot whose readings equal ``truth``.

        Raises:
            TypeError: The subsystem has sensors, so its readings are not a copy of
                its truth.
        """
        if not cls.mirrored:
            raise TypeError(f"{cls.__name__} has sensors; supply its readings explicitly")
        readings = cls.readings_type(
            **{f.name: getattr(truth, f.name) for f in fields(cls.readings_type)}
        )
        return cls(truth=truth, readings=readings)


_VALUE_GETTERS: Final[dict[type, Callable[[Any], tuple[object, ...]]]] = {}
"""One getter per record type, made on first use: :func:`_values` runs twice in every
tick a mirrored snapshot is rebuilt, and ``dataclasses.fields`` dominated it (#121)."""


def _values(record: Any) -> tuple[object, ...]:
    """The record's field values, in field order."""
    getter = _VALUE_GETTERS.get(type(record))
    if getter is None:
        names = [f.name for f in fields(record)]
        if len(names) > 1:
            getter = attrgetter(*names)  # returns the tuple of values
        else:
            getter = lambda r: tuple(getattr(r, n) for n in names)  # noqa: E731
        _VALUE_GETTERS[type(record)] = getter
    return getter(record)


# --- Power -----------------------------------------------------------------------------


@dataclass(frozen=True)
class PowerTruth:
    """True power state (#35, #36). Net battery power is ``generation_w - total_load_w``."""

    bus_v: float
    """True bus voltage, volts."""

    battery_current_a: float
    """True battery current, amperes. Positive while charging, negative while
    discharging."""

    soc: float
    """True battery state of charge, fraction 0..1."""

    generation_w: float
    """Solar generation, watts. Never negative; zero in eclipse."""

    total_load_w: float
    """Total electrical load, watts: base load plus every switched-on draw (payload,
    attitude control, transmit, survival heater) and ``extra_load_w``. Never negative.
    Thermal reads it as the power dissipated as heat."""

    @property
    def net_power_w(self) -> float:
        """Net battery power, watts: generation minus total load. Positive while
        charging."""
        return self.generation_w - self.total_load_w


@dataclass(frozen=True)
class PowerReadings:
    """Reported power state and the low-battery flags (#36)."""

    bus_v: float
    """Reported bus voltage, volts. Noisy; held while ``power`` is in
    ``frozen_sensors``."""

    battery_current_a: float
    """Reported battery current, amperes, positive while charging. Noisy; held while
    frozen."""

    soc: float
    """State-of-charge estimate from the reported voltage, fraction 0..1. Follows the
    voltage reading's noise and freezes with it."""

    low_battery: bool
    """Flag: the SOC estimate is below the low-battery threshold (with hysteresis).
    ``TelemetryFlags.low_battery`` (#54)."""

    critical_battery: bool
    """Flag: the SOC estimate is below the critical-battery threshold (with
    hysteresis). ``TelemetryFlags.critical_battery`` (#54)."""


@dataclass(frozen=True)
class PowerSnapshot(ContractSnapshot[PowerTruth, PowerReadings]):
    """Power snapshot: :class:`PowerTruth` and :class:`PowerReadings`."""

    truth_type = PowerTruth
    readings_type = PowerReadings


# --- Thermal ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ThermalTruth:
    """True thermal state, including the hardwired survival heater (#38, ADR-0004
    §12)."""

    battery_c: float
    """True battery temperature, °C."""

    electronics_c: float
    """True electronics temperature, °C."""

    heater_on: bool
    """The battery survival heater is on. Switched by a mechanical thermostat on the
    true battery temperature; not commandable. A physical state, not a flag."""

    heater_power_w: float
    """Survival heater draw, watts. Zero while the heater is off; never negative.
    Power adds it to the total load."""


@dataclass(frozen=True)
class ThermalReadings:
    """Reported temperatures and the thermal limit flags (#39)."""

    battery_c: float
    """Reported battery temperature, °C. Noisy; held while ``thermal`` is in
    ``frozen_sensors``."""

    electronics_c: float
    """Reported electronics temperature, °C. Noisy; held while frozen."""

    over_temp: bool
    """Flag: a reported temperature is above its over-temperature threshold (with
    hysteresis). ``TelemetryFlags.over_temp`` (#54)."""

    under_temp: bool
    """Flag: a reported temperature is below its under-temperature threshold (with
    hysteresis). ``TelemetryFlags.under_temp`` (#54)."""


@dataclass(frozen=True)
class ThermalSnapshot(ContractSnapshot[ThermalTruth, ThermalReadings]):
    """Thermal snapshot: :class:`ThermalTruth` and :class:`ThermalReadings`."""

    truth_type = ThermalTruth
    readings_type = ThermalReadings


# --- Attitude --------------------------------------------------------------------------


@dataclass(frozen=True)
class AttitudeTruth:
    """True attitude state (#41)."""

    pointing_error_deg: float
    """True pointing error from the sun-optimal attitude, degrees, 0..180. Power uses
    it for solar generation."""

    rate_dps: float
    """True angular rate magnitude, degrees per second. Never negative."""

    state: AttitudeState
    """State decided from the true rate and pointing error."""

    control_power_w: float
    """Attitude-control draw, watts. Zero while attitude control is off; never
    negative. Power adds it to the total load."""


@dataclass(frozen=True)
class AttitudeReadings:
    """Reported attitude state (#41)."""

    pointing_error_deg: float
    """Reported pointing error, degrees, 0..180. Noisy; held while ``attitude`` is in
    ``frozen_sensors``."""

    rate_dps: float
    """Reported angular rate magnitude, degrees per second. Noisy; held while
    frozen."""

    state: AttitudeState
    """State decided from the reported values. The payload requires ``STABILIZED`` to
    acquire."""


@dataclass(frozen=True)
class AttitudeSnapshot(ContractSnapshot[AttitudeTruth, AttitudeReadings]):
    """Attitude snapshot: :class:`AttitudeTruth` and :class:`AttitudeReadings`."""

    truth_type = AttitudeTruth
    readings_type = AttitudeReadings


# --- Payload ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _PayloadRecord:
    """Payload fields, shared by the truth and readings records, which are equal."""

    state: PayloadState
    """Payload state."""

    buffered_bytes: int
    """Stored, unreleased data, bytes: 0..``buffer_capacity_bytes``. Equals
    ``total_produced_bytes - total_released_bytes``."""

    buffer_capacity_bytes: int
    """Buffer capacity, bytes, from ``PayloadConfig``. Positive."""

    oldest_unreleased_chunk_id: int
    """ID of the oldest chunk still stored. Equals ``next_chunk_id`` when no chunk is
    stored."""

    next_chunk_id: int
    """ID the next chunk created will get. Chunk IDs start at 0 after a reset and
    increase by one per chunk."""

    total_produced_bytes: int
    """Bytes produced since the last reset."""

    total_released_bytes: int
    """Bytes released through ``controls.payload.release_through_chunk_id`` since the
    last reset."""

    power_w: float
    """Payload draw, watts. Never negative. Power adds it to the total load."""

    @property
    def buffer_fill(self) -> float:
        """Fraction of the buffer filled, 0..1: ``buffered_bytes /
        buffer_capacity_bytes``."""
        return self.buffered_bytes / self.buffer_capacity_bytes


@dataclass(frozen=True)
class PayloadTruth(_PayloadRecord):
    """True payload state (#43). The payload owns stored data (ADR-0004 §13)."""


@dataclass(frozen=True)
class PayloadReadings(_PayloadRecord):
    """Reported payload state (#43). Always equal to :class:`PayloadTruth`: the payload
    has no sensors."""


@dataclass(frozen=True)
class PayloadSnapshot(ContractSnapshot[PayloadTruth, PayloadReadings]):
    """Payload snapshot. Build it with :meth:`from_truth`."""

    truth_type = PayloadTruth
    readings_type = PayloadReadings
    mirrored = True


# --- Communications --------------------------------------------------------------------


@dataclass(frozen=True)
class _CommsRecord:
    """Communications fields, shared by the truth and readings records, which are
    equal. Comms holds no data (ADR-0004 §13)."""

    radio_mode: RadioMode
    """Radio mode in force this tick (``controls.radio.mode``)."""

    receiver_on: bool
    """The uplink receiver is on. A physical state, not a flag."""

    transmitter_on: bool
    """The downlink transmitter is on (off in ``OFF``, in ``RX_ONLY``, and under the
    ``transmitter_off`` fault). A physical state, not a flag."""

    transmit_capacity_bytes: int
    """Bytes that may be sent this tick. Zero unless the transmitter is on."""

    previous_tick_sent_bytes: int
    """Wire bytes the flight computer sent in the **previous** tick (ADR-0007): every
    frame type, header, payload, and CRC. Per tick, not a running total. Bounded by the
    previous tick's capacity, not this tick's, so it can be non-zero while
    ``transmitter_on`` is false or exceed ``transmit_capacity_bytes`` (for example in
    the first tick of ``transmitter_off``)."""

    uplink_lost_count: int
    """Uplink frames lost because the receiver was off, since ``reset(seed)``. Running
    total, reported one tick late: a frame lost in tick N is counted from tick N+1
    (ADR-0007)."""

    outbound_suppressed_count: int
    """Outbound frames (ACK/NACK and telemetry) the flight computer suppressed for lack
    of transmit capacity, since ``reset(seed)``. Running total, reported one tick late
    (ADR-0007). DATA that does not fit stays buffered and is not counted."""

    transmit_power_w: float
    """Transmit draw this tick, watts: the idle draw of this tick's transmitter state
    (``transmitter_on_power_w`` while on, 0 while off) plus
    ``transmit_power_per_byte_w`` times ``previous_tick_sent_bytes`` (ADR-0007 §2).
    Never negative. Power adds it to the total load one tick later, so a byte's energy
    reaches the battery two ticks after it was sent."""


@dataclass(frozen=True)
class CommsTruth(_CommsRecord):
    """True communications state (#44)."""


@dataclass(frozen=True)
class CommsReadings(_CommsRecord):
    """Reported communications state (#44). Always equal to :class:`CommsTruth`:
    comms has no sensors."""


@dataclass(frozen=True)
class CommsSnapshot(ContractSnapshot[CommsTruth, CommsReadings]):
    """Communications snapshot. Build it with :meth:`from_truth`."""

    truth_type = CommsTruth
    readings_type = CommsReadings
    mirrored = True


# --- Registry --------------------------------------------------------------------------

SNAPSHOT_TYPES: Final[Mapping[str, type[ContractSnapshot[Any, Any]]]] = MappingProxyType(
    {
        "power": PowerSnapshot,
        "thermal": ThermalSnapshot,
        "attitude": AttitudeSnapshot,
        "payload": PayloadSnapshot,
        "comms": CommsSnapshot,
    }
)
"""Snapshot type of each subsystem, by name, in ``STEP_ORDER`` order."""

READINGS_FLAGS: Final[Mapping[str, tuple[str, ...]]] = MappingProxyType(
    {
        "power": ("low_battery", "critical_battery"),
        "thermal": ("over_temp", "under_temp"),
    }
)
"""Flag fields on each subsystem's readings record. Their names are the
``TelemetryFlags`` member names (#54); adding a flag reserves a bit in #54's flag table
in the same change."""
