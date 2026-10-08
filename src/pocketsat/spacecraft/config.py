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
    """Power settings: battery and solar model (#35), sensors and flags (#36).

    The defaults are calibrated as a set by the power and thermal budget (#72,
    ``docs/power-thermal-budget.md``, which gives each one's real-world range and
    rationale). They describe a small 2S lithium-ion battery charged by a body-mounted
    array.

    Attributes:
        battery_capacity_wh: Usable battery energy from empty (SOC 0) to full (SOC 1),
            watt-hours. Positive. Default 20.0.
        solar_array_w: Solar array output at ideal pointing (pointing error 0°) in
            sunlight, watts. Non-negative. Default 8.0.
        base_load_w: Always-on electrical load (avionics, receiver), watts.
            Non-negative. Default 1.8. The per-subsystem draws are added on top.
        battery_empty_v: Bus voltage at SOC 0, volts. Positive. Default 6.0.
        battery_full_v: Bus voltage at SOC 1, volts. Above ``battery_empty_v``.
            Default 8.4. Bus voltage is linear in SOC between the two.
        voltage_noise_v: Standard deviation of the reported bus voltage's noise at
            ``sensor_noise_scale`` 1.0, volts. Non-negative. Default 0.008, which puts
            the SOC estimate within ±0.02 of true SOC (the noise is bounded at 6 sigma,
            and 6 * 0.008 / 2.4 = 0.02).
        current_noise_a: Standard deviation of the reported battery current's noise at
            ``sensor_noise_scale`` 1.0, amperes. Non-negative. Default 0.01.
        low_battery_soc: ``low_battery`` sets when the SOC estimate falls below this
            fraction. Default 0.30.
        low_battery_clear_soc: ``low_battery`` clears when the SOC estimate rises
            above this fraction. Above ``low_battery_soc``; at most 1. Default 0.35.
        critical_battery_soc: ``critical_battery`` sets when the SOC estimate falls
            below this fraction. At most ``low_battery_soc``. Default 0.15.
        critical_battery_clear_soc: ``critical_battery`` clears when the SOC estimate
            rises above this fraction. Above ``critical_battery_soc`` and at most
            ``low_battery_clear_soc``. Default 0.20.

    The two orderings (critical at or below low, for both setting and clearing) mean
    ``critical_battery`` is only ever set while ``low_battery`` is set.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite or is out of range.
    """

    battery_capacity_wh: float = 20.0
    solar_array_w: float = 8.0
    base_load_w: float = 1.8
    battery_empty_v: float = 6.0
    battery_full_v: float = 8.4
    voltage_noise_v: float = 0.008
    current_noise_a: float = 0.01
    low_battery_soc: float = 0.30
    low_battery_clear_soc: float = 0.35
    critical_battery_soc: float = 0.15
    critical_battery_clear_soc: float = 0.20

    def __post_init__(self) -> None:
        if _require_number("battery_capacity_wh", self.battery_capacity_wh) <= 0:
            raise ValueError(
                f"battery_capacity_wh must be positive, got {self.battery_capacity_wh}"
            )
        for name in ("solar_array_w", "base_load_w", "voltage_noise_v", "current_noise_a"):
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
        for name in (
            "low_battery_soc",
            "low_battery_clear_soc",
            "critical_battery_soc",
            "critical_battery_clear_soc",
        ):
            _require_in_range(name, getattr(self, name), 0.0, 1.0)
        for flag in ("low_battery", "critical_battery"):
            set_soc = getattr(self, f"{flag}_soc")
            clear_soc = getattr(self, f"{flag}_clear_soc")
            if clear_soc <= set_soc:
                raise ValueError(
                    f"{flag}_clear_soc must be above {flag}_soc ({set_soc}), got {clear_soc}"
                )
        if self.critical_battery_soc > self.low_battery_soc:
            raise ValueError(
                f"critical_battery_soc must be at most low_battery_soc "
                f"({self.low_battery_soc}), got {self.critical_battery_soc}"
            )
        if self.critical_battery_clear_soc > self.low_battery_clear_soc:
            raise ValueError(
                f"critical_battery_clear_soc must be at most low_battery_clear_soc "
                f"({self.low_battery_clear_soc}), got {self.critical_battery_clear_soc}"
            )


@dataclass(frozen=True)
class ThermalConfig:
    """Thermal settings: the two-node lumped model and the battery survival heater (#38),
    and the sensors and limit flags (#39).

    The model is described in :mod:`pocketsat.spacecraft.thermal`. The defaults are
    calibrated by the power and thermal budget (#72, ``docs/power-thermal-budget.md``).
    They are chosen so that, with about 4 W of electrical load, the battery sits near
    29 °C and the electronics near 31 °C in the nominal 20 °C sunlit ambient, and so
    that the survival heater cycles in each nominal eclipse at the -20 °C eclipse
    ambient (#73): without the heater the battery would settle well below the ON
    setpoint there, with the heater on well above the OFF setpoint, so it switches
    between its setpoints.

    **Limit flags (#39).** Each node has its own thresholds, because the battery is
    heated and the electronics are not: the electronics fall well below the battery in
    eclipse. Each threshold pair has hysteresis: a node's under-temperature condition
    sets when its *reported* temperature falls below ``<node>_under_temp_c`` and clears
    when it rises above ``<node>_under_temp_clear_c``; its over-temperature condition
    sets above ``<node>_over_temp_c`` and clears below ``<node>_over_temp_clear_c``.
    ``ThermalReadings.under_temp`` is set while either node's under-temperature
    condition is set, and ``over_temp`` likewise (the flags are OR-ed across nodes).

    Attributes:
        battery_heat_capacity_j_per_c: Battery node heat capacity, joules per °C.
            Positive. Default 80.0 (roughly a 100 g lithium-ion pack).
        electronics_heat_capacity_j_per_c: Electronics node heat capacity, joules per
            °C. Positive. Default 300.0.
        battery_conductance_w_per_c: Thermal conductance from the battery node to the
            ambient, watts per °C. Positive, so the battery always cools toward the
            ambient. Default 0.12 (the battery sits inside the structure).
        electronics_conductance_w_per_c: Thermal conductance from the electronics node
            to the ambient, watts per °C. Positive. Default 0.25.
        coupling_conductance_w_per_c: Thermal conductance between the battery and the
            electronics nodes, watts per °C. Non-negative; 0 decouples them.
            Default 0.04.
        battery_dissipation_fraction: Fraction of the electrical dissipation (power's
            total load, less the survival heater, whose heat all goes to the battery)
            that heats the battery node, 0..1; the rest heats the electronics node.
            Default 0.25.
        heater_power_w: Survival heater power while on, watts. Non-negative. Default
            3.0.
        heater_on_setpoint_c: The heater switches on when the true battery temperature
            falls below this, °C. Default 1.0, so the battery, which dips just below
            the ON setpoint before the heater catches it, stays at least 5 °C above
            ``battery_under_temp_c`` (#72).
        heater_off_setpoint_c: The heater switches off when the true battery
            temperature rises above this, °C. Above ``heater_on_setpoint_c``.
            Default 5.0.
        temperature_noise_c: Standard deviation of the noise on each reported
            temperature at ``sensor_noise_scale`` 1.0, °C. Non-negative. Default 0.2,
            so the noise (bounded at 6 sigma) spans at most ±1.2 °C, less than every
            default hysteresis band (3 °C).
        battery_survival_limit_c: Lowest temperature the battery survives, °C. Not
            used by the model; it anchors the threshold order below. Default -10.0.
        battery_under_temp_c: The battery's under-temperature condition sets when its
            reported temperature falls below this, °C. Default -5.0.
        battery_under_temp_clear_c: ... and clears when it rises above this, °C.
            Default -2.0.
        battery_over_temp_c: The battery's over-temperature condition sets when its
            reported temperature rises above this, °C. Default 45.0.
        battery_over_temp_clear_c: ... and clears when it falls below this, °C.
            Default 42.0.
        electronics_under_temp_c: The electronics' under-temperature condition sets
            when their reported temperature falls below this, °C. Default -25.0.
        electronics_under_temp_clear_c: ... and clears when it rises above this, °C.
            Default -22.0.
        electronics_over_temp_c: The electronics' over-temperature condition sets when
            their reported temperature rises above this, °C. Default 60.0.
        electronics_over_temp_clear_c: ... and clears when it falls below this, °C.
            Default 57.0.

    Threshold order (validated):

    - Battery: ``battery_survival_limit_c < battery_under_temp_c < heater_on_setpoint_c
      < heater_off_setpoint_c``, with ``battery_under_temp_c`` at least 5 °C below
      ``heater_on_setpoint_c`` (the #72 margin rule), so the heater acts well before
      the flag.
    - Each node: ``under_temp_c < under_temp_clear_c < over_temp_clear_c <
      over_temp_c``.
    - The battery's ``battery_over_temp_clear_c`` is above ``heater_off_setpoint_c``,
      so the heater cannot drive the battery into over-temperature.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite or is out of range, or the setpoints or
            thresholds are out of order.
    """

    battery_heat_capacity_j_per_c: float = 80.0
    electronics_heat_capacity_j_per_c: float = 300.0
    battery_conductance_w_per_c: float = 0.12
    electronics_conductance_w_per_c: float = 0.25
    coupling_conductance_w_per_c: float = 0.04
    battery_dissipation_fraction: float = 0.25
    heater_power_w: float = 3.0
    heater_on_setpoint_c: float = 1.0
    heater_off_setpoint_c: float = 5.0
    temperature_noise_c: float = 0.2
    battery_survival_limit_c: float = -10.0
    battery_under_temp_c: float = -5.0
    battery_under_temp_clear_c: float = -2.0
    battery_over_temp_c: float = 45.0
    battery_over_temp_clear_c: float = 42.0
    electronics_under_temp_c: float = -25.0
    electronics_under_temp_clear_c: float = -22.0
    electronics_over_temp_c: float = 60.0
    electronics_over_temp_clear_c: float = 57.0

    def __post_init__(self) -> None:
        for name in (
            "battery_heat_capacity_j_per_c",
            "electronics_heat_capacity_j_per_c",
            "battery_conductance_w_per_c",
            "electronics_conductance_w_per_c",
        ):
            if _require_number(name, getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        for name in ("coupling_conductance_w_per_c", "heater_power_w", "temperature_noise_c"):
            if _require_number(name, getattr(self, name)) < 0:
                raise ValueError(f"{name} must be non-negative, got {getattr(self, name)}")
        _require_in_range("battery_dissipation_fraction", self.battery_dissipation_fraction, 0, 1)
        for name in (
            "heater_on_setpoint_c",
            "heater_off_setpoint_c",
            "battery_survival_limit_c",
            *(
                f"{node}_{kind}_c"
                for node in ("battery", "electronics")
                for kind in ("under_temp", "under_temp_clear", "over_temp", "over_temp_clear")
            ),
        ):
            if _require_number(name, getattr(self, name)) <= ABSOLUTE_ZERO_C:
                raise ValueError(f"{name} must be above absolute zero, got {getattr(self, name)}")

        def require_above(name: str, lower: str) -> None:
            value, bound = getattr(self, name), getattr(self, lower)
            if value <= bound:
                raise ValueError(f"{name} must be above {lower} ({bound}), got {value}")

        require_above("heater_off_setpoint_c", "heater_on_setpoint_c")
        # Battery threshold order (#39): survival < under_temp < heater ON < heater OFF.
        require_above("battery_under_temp_c", "battery_survival_limit_c")
        require_above("heater_on_setpoint_c", "battery_under_temp_c")
        margin_c = 5.0  # the #72 margin rule
        if self.heater_on_setpoint_c - self.battery_under_temp_c < margin_c:
            raise ValueError(
                f"battery_under_temp_c must be at least {margin_c} °C below "
                f"heater_on_setpoint_c ({self.heater_on_setpoint_c}), "
                f"got {self.battery_under_temp_c}"
            )
        for node in ("battery", "electronics"):
            require_above(f"{node}_under_temp_clear_c", f"{node}_under_temp_c")
            require_above(f"{node}_over_temp_clear_c", f"{node}_under_temp_clear_c")
            require_above(f"{node}_over_temp_c", f"{node}_over_temp_clear_c")
        require_above("battery_over_temp_clear_c", "heater_off_setpoint_c")


@dataclass(frozen=True)
class AttitudeConfig:
    """Attitude settings for the scalar attitude model (#41, ADR-0006).

    The model keeps two damped scalars, the pointing error and the angular rate
    magnitude (see :mod:`pocketsat.spacecraft.attitude`). The defaults are illustrative:
    they detumble the default starting state within a few minutes and hold the pointing
    error near 2° under the default disturbance. The power and thermal budget (#72)
    confirmed ``control_power_w`` (0.5 W).

    Attributes:
        rate_damping_per_s: Rate damping while attitude control is on, per second: each
            tick the rate shrinks by ``rate_damping_per_s * dt``. Non-negative.
            Default 0.05 (a 20 s time constant).
        pointing_gain_per_s: Pointing correction while attitude control is on, per
            second: each tick the pointing error shrinks by ``pointing_gain_per_s * dt``.
            Non-negative. Default 0.02 (a 50 s time constant).
        disturbance_mean_dps_per_s: Mean of the disturbance acceleration on the rate
            magnitude, degrees per second per second. Non-negative: with control off
            the rate drifts upward toward tumbling. Default 0.002.
        disturbance_sd_dps_per_s: Spread of the disturbance: the random part of each
            tick's rate change is ``disturbance_sd_dps_per_s * sqrt(dt) * n`` with
            ``n`` from ``portable_normal`` (a rate random walk, degrees per second per
            square-root second). Non-negative. Default 0.002.
        pointing_noise_deg: Standard deviation of the reported pointing error's noise
            at ``sensor_noise_scale`` 1.0, degrees. Non-negative. Default 0.5.
        rate_noise_dps: Standard deviation of the reported rate's noise at
            ``sensor_noise_scale`` 1.0, degrees per second. Non-negative. Default 0.01.
        tumbling_enter_rate_dps: Above this rate the state is ``TUMBLING``. Default 2.0.
        tumbling_exit_rate_dps: ``TUMBLING`` becomes ``DETUMBLING`` once the rate is at
            or below this with attitude control on. Default 1.5.
        stabilized_exit_rate_dps: ``STABILIZED`` becomes ``DETUMBLING`` above this rate.
            Default 0.4.
        stabilized_enter_rate_dps: ``DETUMBLING`` becomes ``STABILIZED`` below this rate
            (with the pointing error within ``stabilized_enter_pointing_error_deg``).
            Default 0.2. The rate thresholds must satisfy ``0 < stabilized_enter <
            stabilized_exit <= tumbling_exit < tumbling_enter``.
        stabilized_enter_pointing_error_deg: ``STABILIZED`` requires the pointing error
            at or below this on entry, degrees. Default 5.0.
        stabilized_exit_pointing_error_deg: ``STABILIZED`` becomes ``DETUMBLING`` above
            this pointing error, degrees. Default 10.0. The pointing thresholds must
            satisfy ``0 <= enter < exit <= 180``.
        control_power_w: Attitude-control draw while control is on, watts.
            Non-negative. Default 0.5.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite, is out of range, or the thresholds are out
            of order.
    """

    rate_damping_per_s: float = 0.05
    pointing_gain_per_s: float = 0.02
    disturbance_mean_dps_per_s: float = 0.002
    disturbance_sd_dps_per_s: float = 0.002
    pointing_noise_deg: float = 0.5
    rate_noise_dps: float = 0.01
    tumbling_enter_rate_dps: float = 2.0
    tumbling_exit_rate_dps: float = 1.5
    stabilized_exit_rate_dps: float = 0.4
    stabilized_enter_rate_dps: float = 0.2
    stabilized_enter_pointing_error_deg: float = 5.0
    stabilized_exit_pointing_error_deg: float = 10.0
    control_power_w: float = 0.5

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            if _require_number(name, getattr(self, name)) < 0:
                raise ValueError(f"{name} must be non-negative, got {getattr(self, name)}")
        if not (
            0
            < self.stabilized_enter_rate_dps
            < self.stabilized_exit_rate_dps
            <= self.tumbling_exit_rate_dps
            < self.tumbling_enter_rate_dps
        ):
            raise ValueError(
                "rate thresholds must satisfy 0 < stabilized_enter_rate_dps < "
                "stabilized_exit_rate_dps <= tumbling_exit_rate_dps < "
                f"tumbling_enter_rate_dps, got {self.stabilized_enter_rate_dps}, "
                f"{self.stabilized_exit_rate_dps}, {self.tumbling_exit_rate_dps}, "
                f"{self.tumbling_enter_rate_dps}"
            )
        if not (
            self.stabilized_enter_pointing_error_deg
            < self.stabilized_exit_pointing_error_deg
            <= 180.0
        ):
            raise ValueError(
                "pointing thresholds must satisfy 0 <= stabilized_enter_pointing_error_deg "
                "< stabilized_exit_pointing_error_deg <= 180, got "
                f"{self.stabilized_enter_pointing_error_deg}, "
                f"{self.stabilized_exit_pointing_error_deg}"
            )


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

    The defaults are calibrated by the power and thermal budget (#72,
    ``docs/power-thermal-budget.md``): a chunk's 78-byte DATA frame fits in one tick's
    nominal transmit capacity, one orbit of data (about 221 KB at 40 bytes/s) fits in
    one 10-minute DOWNLINK pass with more than 20% margin, and the buffer holds more
    than two orbits of data, so one missed pass loses nothing.

    Attributes:
        buffer_capacity_bytes: Buffer capacity, bytes. A positive whole number of
            chunks (a multiple of ``chunk_size_bytes``), so the buffer fills exactly
            on a chunk boundary. Default 524 288 (512 KiB).
        chunk_size_bytes: Size of every chunk, bytes. Positive and at most
            :data:`MAX_CHUNK_SIZE_BYTES`, so a chunk fits in one DATA frame (#56).
            Default 64.
        data_rate_bytes_per_s: True acquisition data rate while acquiring, bytes per
            second. Non-negative. Default 40.
        idle_power_w: Draw while commanded on but not acquiring, watts. Non-negative.
            Default 0.3.
        acquiring_power_w: Draw while acquiring, watts. Non-negative. Default 1.5.

    Raises:
        TypeError: A byte field is not an int, or a power field is not a number.
        ValueError: A field is out of range, or the capacity is not a whole number of
            chunks.
    """

    buffer_capacity_bytes: int = 524_288
    chunk_size_bytes: int = 64
    data_rate_bytes_per_s: int = 40
    idle_power_w: float = 0.3
    acquiring_power_w: float = 1.5

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


PER_BYTE_DRAW_TICK_US: Final = 100_000
"""The tick ``CommsConfig.transmit_power_per_byte_w`` is stated for: 100 ms (#72, #122).

The per-byte draw is the transmitter's extra draw, in watts, for each byte sent within
one tick of this length, so one byte costs ``transmit_power_per_byte_w * 0.1 s`` of
energy. Comms scales the draw to the actual tick, by ``PER_BYTE_DRAW_TICK_US / dt_us``,
so a byte costs the same energy at any tick length; at the 100 ms tick the factor is
exactly 1."""


@dataclass(frozen=True)
class CommsConfig:
    """Communications settings (#44).

    The transmit rate is a data rate, as on a real radio, so the downlink is 9600 bit/s
    at any tick length (#122). Comms converts it into each tick's
    ``transmit_capacity_bytes`` exactly, carrying the fraction of a byte (see
    :mod:`pocketsat.spacecraft.comms`). At the default 100 ms tick that is 120 bytes
    every tick, which fits one default payload chunk's DATA frame (64-byte chunk +
    4-byte ID + 10 bytes of frame header and CRC = 78 bytes) with room for an ACK.

    The draws follow the power and thermal budget's transmitter model (#72, "Option
    C"): the radio stays ``RX_TX`` in every normal mode, so the fixed draw is a small
    idle draw while the transmitter is enabled but not keyed, and energy follows
    airtime through the per-byte draw: full capacity (120 bytes per 100 ms tick) costs
    0.15 + 2.4 = 2.55 W keyed, typical of a small-satellite UHF transmitter.

    Attributes:
        transmit_rate_bytes_per_s: The transmitter's data rate while it is on, wire
            bytes per second, whatever the tick length. Non-negative int. Default 1200
            (9600 bit/s).
        transmitter_on_power_w: Fixed transmit draw while the transmitter is on (idle,
            not keyed), watts. Non-negative. Default 0.15.
        transmit_power_per_byte_w: Additional draw per byte sent within one 100 ms
            tick (:data:`PER_BYTE_DRAW_TICK_US`), watts per byte; comms scales it to
            the actual tick, so each byte costs the same energy (0.002 J by default)
            and full rate costs 2.4 W at any tick length (#122). Non-negative.
            Default 0.02.

    Raises:
        TypeError: ``transmit_rate_bytes_per_s`` is not an int, or a draw is not a
            number.
        ValueError: A field is negative or not finite.
    """

    transmit_rate_bytes_per_s: int = 1200
    transmitter_on_power_w: float = 0.15
    transmit_power_per_byte_w: float = 0.02

    def __post_init__(self) -> None:
        rate = self.transmit_rate_bytes_per_s
        if isinstance(rate, bool) or not isinstance(rate, int):
            raise TypeError(f"transmit_rate_bytes_per_s must be an int, got {rate!r}")
        if rate < 0:
            raise ValueError(f"transmit_rate_bytes_per_s must be non-negative, got {rate}")
        for name in ("transmitter_on_power_w", "transmit_power_per_byte_w"):
            if _require_number(name, getattr(self, name)) < 0:
                raise ValueError(f"{name} must be non-negative, got {getattr(self, name)}")


@dataclass(frozen=True)
class SpacecraftConfig:
    """Every subsystem's settings: what the spacecraft is (ADR-0005).

    Each subsystem is built with, and reads, only its own record. The nominal set is
    :data:`NOMINAL_CONFIG` and the stressed set :data:`STRESSED_CONFIG` (#72).

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
"""The nominal settings, calibrated by the power and thermal budget (#72,
``docs/power-thermal-budget.md``). Deliberately comfortable: scenarios about power or
thermal stress use :data:`STRESSED_CONFIG`, a different starting state, or
``EnvironmentState`` overrides, never retuned nominal defaults."""

STRESSED_SOLAR_ARRAY_W: Final = 6.0
"""Stressed solar array output, watts: 25% below nominal (end-of-life cell
degradation, higher array temperature, off-pointing)."""

STRESSED_BATTERY_CAPACITY_WH: Final = 14.0
"""Stressed battery capacity, watt-hours: 30% below nominal (an aged battery)."""

STRESSED_CONFIG = SpacecraftConfig(
    power=PowerConfig(
        battery_capacity_wh=STRESSED_BATTERY_CAPACITY_WH,
        solar_array_w=STRESSED_SOLAR_ARRAY_W,
        base_load_w=2.16,
    ),
    payload=PayloadConfig(idle_power_w=0.36, acquiring_power_w=1.8),
)
"""The stressed settings (#72): reduced solar output (-25%), reduced battery capacity
(-30%), and higher loads (base load and payload draws +20%); everything else nominal.
SCIENCE is energy-negative under it, so it is used by the budget's must-fail profiles
and is available to later phases (Phase 6 campaigns vary battery condition)."""


# --- Starting state --------------------------------------------------------------------


@dataclass(frozen=True)
class PowerInitial:
    """Where the power subsystem starts.

    Attributes:
        soc: Battery state of charge, 0..1. Default 0.5, set by the power and
            thermal budget (#72): a typical launch storage charge, which puts the
            reference profile's minimum SOC 20 percentage points above the
            ``low_battery`` threshold.

    Raises:
        TypeError: ``soc`` is not a number.
        ValueError: ``soc`` is not finite or is outside 0..1.
    """

    soc: float = 0.5

    def __post_init__(self) -> None:
        _require_in_range("soc", self.soc, 0.0, 1.0)


@dataclass(frozen=True)
class ThermalInitial:
    """Where the thermal subsystem starts.

    Attributes:
        battery_c: Battery temperature, °C.
        electronics_c: Electronics temperature, °C.

    Both defaults equal the nominal sunlit ambient (20 °C), confirmed by the power and
    thermal budget (#72): the nodes settle within an hour, long before the first
    eclipse, so the start barely affects the first orbit. The survival heater starts
    on if ``battery_c`` is below ``ThermalConfig.heater_on_setpoint_c`` and off
    otherwise (#38).

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

    The defaults (#41) describe the spacecraft after deployment tip-off has been
    partly damped: 1.0 °/s (below the default tumbling threshold, so it starts
    ``DETUMBLING``) and 45° off the sun-optimal attitude. With the default
    ``AttitudeConfig`` and control on, it reaches ``STABILIZED`` within a few minutes
    of simulated time, well inside story #40's three-orbit run.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite or is out of range.
    """

    pointing_error_deg: float = 45.0
    rate_dps: float = 1.0

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
