"""Power subsystem: battery and solar model (#35), loads, sensors, and flags (#36).

The power subsystem integrates the battery's state of charge (SOC) from solar
generation and electrical load, derives the bus voltage from the SOC, and reports
noisy sensor readings with the low-battery flags. It steps first in ``STEP_ORDER``.

Each tick:

1. **Generation.** In sunlight, ``solar_array_w`` times a pointing factor,
   ``portable_cos_deg`` of attitude's *true* pointing error (ADR-0004 §7, ADR-0006).
   Zero in eclipse, and zero at or beyond 90° of pointing error.
2. **Load.** The sum, in this order, of ``base_load_w``, the payload draw
   (``PayloadTruth.power_w``), the attitude-control draw
   (``AttitudeTruth.control_power_w``), the transmit draw
   (``CommsTruth.transmit_power_w``), the survival heater draw
   (``ThermalTruth.heater_power_w``), and ``controls.extra_load_w`` (the
   ``battery_drain`` fault override). Draws are read from the subsystems' *truth*
   records, because the load is physics. Every one of those subsystems steps after
   power, so each draw is the one published in the previous tick (#85, ADR-0004 §3).
   Power never reads the mode (ADR-0004 §4): a draw appears only when the subsystem
   that owns it reports it.
3. **Charge.** Net power (generation minus load) is integrated over ``dt_us``
   (integer microseconds, converted to seconds as ``dt_us / 1_000_000``) into the
   battery's energy, and the SOC is clamped to 0..1. Energy beyond full or below empty
   is discarded; regulation and brown-out are not modelled. Only true values feed the
   integration.
4. **Voltage and current.** Bus voltage is linear in SOC between ``battery_empty_v``
   and ``battery_full_v``. Battery current is net power divided by bus voltage:
   positive while charging, negative while discharging (architecture §4.1).
5. **Readings.** See below.

``EnvironmentState.battery_soc_override`` pins the SOC while set: the SOC equals the
override and the bus voltage derives from it, even while ``extra_load_w`` is set. The
override wins over the drain, but the drain still appears in the total load and the
battery current (ADR-0004 §11). When the override clears, integration resumes from the
last overridden value. The starting charge (``PowerInitial.soc``) is different: it only
sets where a run begins (ADR-0005).

**Readings** (:class:`PowerReadings`, ADR-0004 §6 to §8):

- Reported bus voltage and battery current are the true values plus noise drawn with
  :func:`~pocketsat.core.rng.portable_normal` from the stream
  ``spacecraft.power.noise``. The standard deviations are
  ``PowerConfig.voltage_noise_v`` and ``PowerConfig.current_noise_a`` times
  ``EnvironmentState.sensor_noise_scale``; at scale 0.0 the readings equal the truth
  exactly. Two draws (voltage, then current) are taken every tick, frozen or not, so
  the noise in a given tick does not depend on earlier freezes.
- The SOC estimate inverts the voltage curve on the *reported* voltage,
  ``(reported_v - battery_empty_v) / (battery_full_v - battery_empty_v)``, clamped to
  0..1, so noise and freezes carry through to it. The noise is bounded at 6 sigma, so at
  nominal noise the estimate is within ``6 * voltage_noise_v / (battery_full_v -
  battery_empty_v)`` of the true SOC (±0.02 with the defaults).
- ``low_battery`` sets when the estimate falls below ``low_battery_soc`` and clears
  when it rises above ``low_battery_clear_soc``; between the two it keeps its previous
  value. ``critical_battery`` works the same way with ``critical_battery_soc`` and
  ``critical_battery_clear_soc``. With the defaults the hysteresis band (0.05) exceeds
  the estimate's full noise spread at nominal noise (0.04), so a steady true SOC near
  a threshold cannot make a flag flap.
- While ``"power"`` is in ``controls.frozen_sensors``, the whole readings record
  (voltage, current, SOC estimate, and flags) holds exactly its last value before the
  freeze, without noise, while the truth keeps evolving. A real threshold crossing
  during a freeze is therefore hidden from the flags. On release, live readings
  resume and the flags continue from their pre-freeze state.

After a reset, the readings equal the true starting values exactly (no noise), and
each flag is set if the starting SOC is below its set threshold.
"""

from random import Random

from pocketsat.core.clock import check_us
from pocketsat.core.portable import portable_cos_deg
from pocketsat.core.rng import RngFactory, portable_normal
from pocketsat.spacecraft.base import SnapshotReader
from pocketsat.spacecraft.config import PowerConfig, PowerInitial
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    AttitudeSnapshot,
    CommsSnapshot,
    PayloadSnapshot,
    PowerReadings,
    PowerSnapshot,
    PowerTruth,
    ThermalSnapshot,
)
from pocketsat.targets.base import EnvironmentState

NOISE_STREAM = "spacecraft.power.noise"
"""Name of the random stream the power sensors' noise is drawn from."""

_US_PER_S = 1_000_000
_S_PER_H = 3600.0


class Power:
    """The power subsystem: battery, solar generation, loads, sensors, and flags.

    Implements the ``Subsystem`` protocol. Built with its settings and starting records
    (ADR-0005) and, optionally, a :class:`SnapshotReader` for the other subsystems'
    snapshots. See the module docstring for the model.

    Attributes:
        name: Always ``"power"``.
    """

    name = "power"

    def __init__(
        self,
        config: PowerConfig,
        initial: PowerInitial,
        reader: SnapshotReader | None = None,
    ) -> None:
        """Create the power subsystem, at its starting state. Call :meth:`reset` before
        stepping.

        Args:
            config: Power settings.
            initial: Starting state; :meth:`reset` returns to it.
            reader: Where to read the other subsystems' snapshots. If ``None``, pointing
                is taken as ideal (factor 1.0) and there are no per-subsystem draws
                (only ``base_load_w`` and ``extra_load_w``), which suits tests and
                stacks without the other subsystems. If a reader is given, power reads
                attitude (pointing and control draw), thermal (heater draw), payload,
                and comms (transmit draw) from it, so all four must be published on
                it, or :meth:`step` raises the reader's ``KeyError``.

        Raises:
            TypeError: ``config`` or ``initial`` is not the expected record type.
        """
        if not isinstance(config, PowerConfig):
            raise TypeError(f"config must be a PowerConfig, got {config!r}")
        if not isinstance(initial, PowerInitial):
            raise TypeError(f"initial must be a PowerInitial, got {initial!r}")
        self._config = config
        self._initial = initial
        self._reader = reader
        self._empty_v = config.battery_empty_v
        self._span_v = config.battery_full_v - config.battery_empty_v
        self._capacity_wh = config.battery_capacity_wh
        self._noise: Random | None = None
        self._restore()

    def _voltage(self, soc: float) -> float:
        return self._empty_v + soc * self._span_v

    def _restore(self) -> None:
        config = self._config
        soc = self._initial.soc
        bus_v = self._voltage(soc)
        self._soc = soc
        self._bus_v = bus_v
        self._current_a = 0.0
        self._generation_w = 0.0
        self._load_w = 0.0
        self._reported_v = bus_v
        self._reported_a = 0.0
        self._estimated_soc = soc
        self._low = soc < config.low_battery_soc
        self._critical = soc < config.critical_battery_soc
        self._readings: PowerReadings | None = None
        self._snapshot: PowerSnapshot | None = None

    def reset(self, rng: RngFactory) -> None:
        """Return to the starting state: SOC from ``PowerInitial``, nothing flowing.

        Until the first step, generation, load, and battery current are zero, and the
        readings equal the truth exactly. Requests the ``spacecraft.power.noise``
        stream.
        """
        self._noise = rng.stream(NOISE_STREAM)
        self._restore()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Advance the battery and the readings by ``dt_us`` microseconds.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
            RuntimeError: :meth:`reset` has not been called.
            KeyError: A reader was given but attitude, thermal, payload, or comms is
                not published on it.
        """
        check_us("dt_us", dt_us)
        noise = self._noise
        if noise is None:
            raise RuntimeError("Power.reset() must be called before step()")
        config = self._config
        reader = self._reader

        # Generation and load: physics, from truth records.
        if reader is None:
            factor = 1.0
            load_w = config.base_load_w
        else:
            attitude = reader.get("attitude", AttitudeSnapshot).truth
            factor = portable_cos_deg(attitude.pointing_error_deg)
            load_w = (
                config.base_load_w
                + reader.get("payload", PayloadSnapshot).truth.power_w
                + attitude.control_power_w
                + reader.get("comms", CommsSnapshot).truth.transmit_power_w
                + reader.get("thermal", ThermalSnapshot).truth.heater_power_w
            )
        load_w += controls.extra_load_w
        generation_w = config.solar_array_w * factor if env.sunlit else 0.0
        net_w = generation_w - load_w

        # Charge.
        override = env.battery_soc_override
        if override is not None:
            soc = float(override)
        else:
            energy_wh = net_w * (dt_us / _US_PER_S) / _S_PER_H
            soc = min(1.0, max(0.0, self._soc + energy_wh / self._capacity_wh))

        bus_v = self._voltage(soc)
        current_a = net_w / bus_v
        self._soc = soc
        self._bus_v = bus_v
        self._current_a = current_a
        self._generation_w = generation_w
        self._load_w = load_w
        self._snapshot = None

        # Readings: noise is drawn every tick, frozen or not.
        scale = env.sensor_noise_scale
        noise_v = portable_normal(noise, 0.0, config.voltage_noise_v * scale)
        noise_a = portable_normal(noise, 0.0, config.current_noise_a * scale)
        if "power" in controls.frozen_sensors:
            return
        reported_v = bus_v + noise_v
        estimate = min(1.0, max(0.0, (reported_v - self._empty_v) / self._span_v))
        if self._low:
            self._low = not estimate > config.low_battery_clear_soc
        else:
            self._low = estimate < config.low_battery_soc
        if self._critical:
            self._critical = not estimate > config.critical_battery_clear_soc
        else:
            self._critical = estimate < config.critical_battery_soc
        self._reported_v = reported_v
        self._reported_a = current_a + noise_a
        self._estimated_soc = estimate
        self._readings = None

    def snapshot(self) -> PowerSnapshot:
        """Return the power snapshot.

        The truth record holds the true values; the readings record holds the reported
        values and flags (see the module docstring). The snapshot is built once per
        step, and while the readings are frozen the same readings record is reused.
        """
        snap = self._snapshot
        if snap is None:
            readings = self._readings
            if readings is None:
                readings = PowerReadings(
                    bus_v=self._reported_v,
                    battery_current_a=self._reported_a,
                    soc=self._estimated_soc,
                    low_battery=self._low,
                    critical_battery=self._critical,
                )
                self._readings = readings
            snap = PowerSnapshot(
                truth=PowerTruth(
                    bus_v=self._bus_v,
                    battery_current_a=self._current_a,
                    soc=self._soc,
                    generation_w=self._generation_w,
                    total_load_w=self._load_w,
                ),
                readings=readings,
            )
            self._snapshot = snap
        return snap
