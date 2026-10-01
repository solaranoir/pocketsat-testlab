"""Battery and solar power model (#35).

The power subsystem integrates the battery's state of charge (SOC) from solar
generation and electrical load, and derives the bus voltage from the SOC. It steps
first in ``STEP_ORDER``.

Each tick:

1. **Generation.** In sunlight, ``solar_array_w`` times a pointing factor,
   ``portable_cos_deg`` of attitude's *true* pointing error (ADR-0004 §7, ADR-0006).
   Attitude steps after power, so this is the previous tick's pointing (#85). Zero in
   eclipse, and zero at or beyond 90° of pointing error.
2. **Load.** ``base_load_w`` plus ``controls.extra_load_w`` (the ``battery_drain``
   fault override). The per-subsystem draws are added by #36.
3. **Charge.** Net power (generation minus load) is integrated over ``dt_us``
   (integer microseconds, converted to seconds as ``dt_us / 1_000_000``) into the
   battery's energy, and the SOC is clamped to 0..1. Energy beyond full or below empty
   is discarded; regulation and brown-out are not modelled.
4. **Voltage and current.** Bus voltage is linear in SOC between ``battery_empty_v``
   and ``battery_full_v``. Battery current is net power divided by bus voltage:
   positive while charging, negative while discharging (architecture §4.1).

``EnvironmentState.battery_soc_override`` pins the SOC while set: the SOC equals the
override and the bus voltage derives from it, even while ``extra_load_w`` is set. The
override wins over the drain, but the drain still appears in the total load and the
battery current (ADR-0004 §11). When the override clears, integration resumes from the
last overridden value. The starting charge (``PowerInitial.soc``) is different: it only
sets where a run begins (ADR-0005).

Readings: until #36 adds sensor noise, the SOC estimate, ``frozen_sensors``, and the
low-battery flags, :class:`PowerReadings` reports the true bus voltage, current, and
SOC, with both flags ``False``. The model uses no randomness.
"""

from pocketsat.core.clock import check_us
from pocketsat.core.portable import portable_cos_deg
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.base import SnapshotReader
from pocketsat.spacecraft.config import PowerConfig, PowerInitial
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    AttitudeSnapshot,
    PowerReadings,
    PowerSnapshot,
    PowerTruth,
)
from pocketsat.targets.base import EnvironmentState

_US_PER_S = 1_000_000
_S_PER_H = 3600.0


class Power:
    """The power subsystem: battery state of charge, bus voltage, and solar generation.

    Implements the ``Subsystem`` protocol. Built with its settings and starting records
    (ADR-0005) and, optionally, a :class:`SnapshotReader` for attitude's pointing.

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
        """Create the power subsystem, at its starting state.

        Args:
            config: Power settings.
            initial: Starting state; :meth:`reset` returns to it.
            reader: Where to read attitude's snapshot for the pointing factor. If
                ``None``, pointing is taken as ideal (factor 1.0), which suits tests
                and stacks without an attitude subsystem. If a reader is given, attitude
                must be published on it, or :meth:`step` raises the reader's
                ``KeyError``.

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
        self._soc = initial.soc
        self._bus_v = self._voltage(initial.soc)
        self._current_a = 0.0
        self._generation_w = 0.0
        self._load_w = 0.0
        self._snapshot: PowerSnapshot | None = None

    def _voltage(self, soc: float) -> float:
        return self._empty_v + soc * self._span_v

    def reset(self, rng: RngFactory) -> None:
        """Return to the starting state: SOC from ``PowerInitial``, nothing flowing.

        Until the first step, generation, load, and battery current are zero. The model
        requests no random streams.
        """
        self._soc = self._initial.soc
        self._bus_v = self._voltage(self._soc)
        self._current_a = 0.0
        self._generation_w = 0.0
        self._load_w = 0.0
        self._snapshot = None

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Advance the battery by ``dt_us`` microseconds.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
            KeyError: A reader was given but no attitude snapshot is published on it.
        """
        check_us("dt_us", dt_us)
        if self._reader is None:
            factor = 1.0
        else:
            attitude = self._reader.get("attitude", AttitudeSnapshot)
            factor = portable_cos_deg(attitude.truth.pointing_error_deg)
        generation_w = self._config.solar_array_w * factor if env.sunlit else 0.0
        load_w = self._config.base_load_w + controls.extra_load_w
        net_w = generation_w - load_w

        override = env.battery_soc_override
        if override is not None:
            soc = float(override)
        else:
            energy_wh = net_w * (dt_us / _US_PER_S) / _S_PER_H
            soc = min(1.0, max(0.0, self._soc + energy_wh / self._capacity_wh))

        bus_v = self._voltage(soc)
        self._soc = soc
        self._bus_v = bus_v
        self._current_a = net_w / bus_v
        self._generation_w = generation_w
        self._load_w = load_w
        self._snapshot = None

    def snapshot(self) -> PowerSnapshot:
        """Return the power snapshot.

        The truth record holds the true values. Until #36, the readings record equals
        the true bus voltage, current, and SOC, with ``low_battery`` and
        ``critical_battery`` both ``False``. The snapshot is built once per step.
        """
        snap = self._snapshot
        if snap is None:
            snap = PowerSnapshot(
                truth=PowerTruth(
                    bus_v=self._bus_v,
                    battery_current_a=self._current_a,
                    soc=self._soc,
                    generation_w=self._generation_w,
                    total_load_w=self._load_w,
                ),
                readings=PowerReadings(
                    bus_v=self._bus_v,
                    battery_current_a=self._current_a,
                    soc=self._soc,
                    low_battery=False,
                    critical_battery=False,
                ),
            )
            self._snapshot = snap
        return snap
