"""Lumped thermal model (#38): battery and electronics nodes, and the survival heater.

Two temperature nodes, the battery (``B``) and the electronics (``E``), are integrated
step by step with explicit (forward Euler) integration and portable arithmetic only
(ADR-0006): no exponentials. Time steps are integer microseconds, converted to seconds
as ``dt = dt_us / 1_000_000``. Settings come from :class:`ThermalConfig`.

Each tick, with ``T_amb`` = ``EnvironmentState.ambient_temp_c``:

1. **Electrical dissipation** ``P``. All electrical load becomes heat (#76). It is
   read from power's *true* current-tick record, ``PowerTruth.total_load_w`` (physics
   reads truth, ADR-0004 §6; power steps before thermal, so the reader holds this
   tick's value, #85). Power's total load includes the survival heater draw thermal
   published in the previous tick, which is exactly the heater power applied in this
   tick (step 2), so thermal subtracts it: ``P = max(0, total_load_w - H)``. The
   heater's heat is therefore counted once, and goes into the battery node only.
   Without a reader there is no electrical heating (``P = 0``).
2. **Heater** ``H``: ``heater_power_w`` if the heater is on, else 0. The on/off state
   is the one the thermostat decided at the end of the previous tick (or at reset),
   which is also the one power read this tick.
3. **Integration.** With ``f`` = ``battery_dissipation_fraction``, ``G_B`` and
   ``G_E`` the conductances to the ambient, ``G_BE`` the coupling conductance, and
   ``C_B``, ``C_E`` the heat capacities::

       Q_BE = G_BE * (T_B - T_E)
       T_B += (f * P + H - G_B * (T_B - T_amb) - Q_BE) * dt / C_B
       T_E += ((1 - f) * P - G_E * (T_E - T_amb) + Q_BE) * dt / C_E

   Both nodes cool (or warm) toward the ambient and exchange heat through the
   coupling. The **environment** enters only through ``ambient_temp_c``: it is the
   effective sink temperature, and already includes solar heating (the nominal
   environment uses 20 °C in sunlight and -20 °C in eclipse, #73), so there is no
   separate solar input. Steady state is linear in ``T_amb``, so a change in ambient
   moves both steady-state temperatures by the same amount.
4. **Thermostat** (ADR-0004 §12). A mechanical thermostat on the battery node reads
   the battery's *true* temperature after integration: an off heater switches on when
   it is below ``heater_on_setpoint_c``, an on heater switches off when it is above
   ``heater_off_setpoint_c``, and between the setpoints it keeps its state
   (hysteresis, so it cannot chatter). It has no control and no mode can switch it
   off. Its power (``ThermalTruth.heater_power_w``) is heated into the battery in the
   next tick, the same tick power draws it.

**Step size.** Forward Euler is monotone (no overshoot or oscillation) while
``dt <= C / (sum of the node's conductances)`` for both nodes. :attr:`Thermal.max_step_us`
is that limit (500 s with the defaults); :meth:`Thermal.step` rejects a longer step.

**Readings** (:class:`ThermalReadings`, #39, ADR-0004 §6 to §8):

- The reported battery and electronics temperatures are the true values plus noise
  drawn with :func:`~pocketsat.core.rng.portable_normal` from the stream
  ``spacecraft.thermal.noise``, with standard deviation
  ``ThermalConfig.temperature_noise_c`` times ``EnvironmentState.sensor_noise_scale``;
  at scale 0.0 the readings equal the truth exactly. Two draws (battery, then
  electronics) are taken every tick, frozen or not, so the noise in a given tick does
  not depend on earlier freezes.
- Each node has its own under- and over-temperature thresholds, each with hysteresis
  (see :class:`ThermalConfig`), evaluated on that node's *reported* temperature: a
  condition sets beyond its set threshold, clears beyond its clear threshold, and
  between the two keeps its previous value. ``under_temp`` is set while the battery's
  or the electronics' under-temperature condition is set, ``over_temp`` likewise:
  the flags are OR-ed across the nodes.
- While ``"thermal"`` is in ``controls.frozen_sensors``, the whole readings record
  (both temperatures and both flags) holds exactly its last value before the freeze,
  without noise, while the truth keeps evolving. A real threshold crossing during a
  freeze is therefore hidden from the flags. On release, live readings resume and each
  node's conditions continue from their pre-freeze state.

The thermostat and the physics read only true values; the readings never feed back.
After a reset the readings equal the true starting temperatures exactly (no noise),
and each condition is set if its node's starting temperature is beyond its set
threshold. :meth:`Thermal.reset` must be called before the first step (it requests
the noise stream).
"""

import math
from random import Random
from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory, portable_normal
from pocketsat.spacecraft.base import SnapshotReader
from pocketsat.spacecraft.config import ThermalConfig, ThermalInitial
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    PowerSnapshot,
    ThermalReadings,
    ThermalSnapshot,
    ThermalTruth,
)
from pocketsat.targets.base import EnvironmentState

__all__ = ["NOISE_STREAM", "Thermal"]

NOISE_STREAM: Final = "spacecraft.thermal.noise"
"""Name of the random stream the thermal sensors' noise is drawn from."""

_US_PER_S: Final = 1_000_000


class Thermal:
    """The thermal subsystem (#38). Implements the ``Subsystem`` protocol.

    Built with its settings, its starting record (ADR-0005), and, optionally, a
    :class:`SnapshotReader` to read power's dissipation from. See the module docstring
    for the model.

    Attributes:
        name: Always ``"thermal"``.
    """

    name: Final = "thermal"

    def __init__(
        self,
        config: ThermalConfig,
        initial: ThermalInitial,
        reader: SnapshotReader | None = None,
    ) -> None:
        """Create the thermal subsystem, at its starting state. Call :meth:`reset`
        before stepping.

        Args:
            config: Thermal settings.
            initial: Starting temperatures; :meth:`reset` returns to them.
            reader: Where to read power's snapshot. If ``None``, there is no electrical
                heating (only the environment and the heater), which suits tests and
                stacks without power. If given, power must be published on it, or
                :meth:`step` raises the reader's ``KeyError``.

        Raises:
            TypeError: ``config`` or ``initial`` is not the expected record type.
        """
        if not isinstance(config, ThermalConfig):
            raise TypeError(f"config must be a ThermalConfig, got {config!r}")
        if not isinstance(initial, ThermalInitial):
            raise TypeError(f"initial must be a ThermalInitial, got {initial!r}")
        self._config = config
        self._initial = initial
        self._reader = reader
        self._inv_c_b = 1.0 / config.battery_heat_capacity_j_per_c
        self._inv_c_e = 1.0 / config.electronics_heat_capacity_j_per_c
        self._g_b = float(config.battery_conductance_w_per_c)
        self._g_e = float(config.electronics_conductance_w_per_c)
        self._g_be = float(config.coupling_conductance_w_per_c)
        self._f_b = float(config.battery_dissipation_fraction)
        self._f_e = 1.0 - self._f_b
        self._heater_w = float(config.heater_power_w)
        self._on_c = float(config.heater_on_setpoint_c)
        self._off_c = float(config.heater_off_setpoint_c)
        self._noise_c = float(config.temperature_noise_c)
        self._noise: Random | None = None
        limit_s = min(
            config.battery_heat_capacity_j_per_c / (self._g_b + self._g_be),
            config.electronics_heat_capacity_j_per_c / (self._g_e + self._g_be),
        )
        self._max_step_us = math.floor(limit_s * _US_PER_S)
        self._restore()

    @property
    def max_step_us(self) -> int:
        """Longest accepted time step, microseconds: the forward Euler monotonicity
        limit ``min(C_B / (G_B + G_BE), C_E / (G_E + G_BE))``."""
        return self._max_step_us

    def _restore(self) -> None:
        config = self._config
        battery_c = float(self._initial.battery_c)
        electronics_c = float(self._initial.electronics_c)
        self._battery_c = battery_c
        self._electronics_c = electronics_c
        self._heater_on = battery_c < self._on_c
        self._reported_battery_c = battery_c
        self._reported_electronics_c = electronics_c
        self._battery_under = battery_c < config.battery_under_temp_c
        self._battery_over = battery_c > config.battery_over_temp_c
        self._electronics_under = electronics_c < config.electronics_under_temp_c
        self._electronics_over = electronics_c > config.electronics_over_temp_c
        self._readings: ThermalReadings | None = None
        self._snapshot: ThermalSnapshot | None = None

    def reset(self, rng: RngFactory) -> None:
        """Return to the starting temperatures, with the readings equal to them.

        The heater starts on if the starting battery temperature is below the ON
        setpoint, and each limit condition is set if its node starts beyond its set
        threshold. Requests the ``spacecraft.thermal.noise`` stream.
        """
        self._noise = rng.stream(NOISE_STREAM)
        self._restore()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Advance both temperature nodes and the thermostat by ``dt_us`` microseconds.

        Only ``controls.frozen_sensors`` is read: thermal has no controls record,
        because the survival heater is hardwired (ADR-0004 §12).

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative or longer than :attr:`max_step_us`.
            RuntimeError: :meth:`reset` has not been called.
            KeyError: A reader was given but power is not published on it.
        """
        check_us("dt_us", dt_us)
        if dt_us > self._max_step_us:
            raise ValueError(
                f"dt_us must be at most {self._max_step_us} (thermal step limit), got {dt_us}"
            )
        noise = self._noise
        if noise is None:
            raise RuntimeError("Thermal.reset() must be called before step()")
        heater_w = self._heater_w if self._heater_on else 0.0
        reader = self._reader
        if reader is None:
            dissipation_w = 0.0
        else:
            total_w = reader.get("power", PowerSnapshot).truth.total_load_w
            dissipation_w = max(0.0, total_w - heater_w)

        dt_s = dt_us / _US_PER_S
        ambient_c = env.ambient_temp_c
        battery_c = self._battery_c
        electronics_c = self._electronics_c
        coupling_w = self._g_be * (battery_c - electronics_c)
        battery_c += (
            (
                self._f_b * dissipation_w
                + heater_w
                - self._g_b * (battery_c - ambient_c)
                - coupling_w
            )
            * dt_s
            * self._inv_c_b
        )
        electronics_c += (
            (self._f_e * dissipation_w - self._g_e * (electronics_c - ambient_c) + coupling_w)
            * dt_s
            * self._inv_c_e
        )
        self._battery_c = battery_c
        self._electronics_c = electronics_c

        # Thermostat on the true battery temperature, with hysteresis.
        if self._heater_on:
            self._heater_on = not battery_c > self._off_c
        else:
            self._heater_on = battery_c < self._on_c
        self._snapshot = None

        # Readings: noise is drawn every tick, frozen or not.
        sigma_c = self._noise_c * env.sensor_noise_scale
        noise_battery_c = portable_normal(noise, 0.0, sigma_c)
        noise_electronics_c = portable_normal(noise, 0.0, sigma_c)
        if "thermal" in controls.frozen_sensors:
            return
        config = self._config
        reported_battery_c = battery_c + noise_battery_c
        reported_electronics_c = electronics_c + noise_electronics_c
        # Per-node conditions with hysteresis, on the reported temperatures.
        if self._battery_under:
            self._battery_under = not reported_battery_c > config.battery_under_temp_clear_c
        else:
            self._battery_under = reported_battery_c < config.battery_under_temp_c
        if self._battery_over:
            self._battery_over = not reported_battery_c < config.battery_over_temp_clear_c
        else:
            self._battery_over = reported_battery_c > config.battery_over_temp_c
        if self._electronics_under:
            self._electronics_under = (
                not reported_electronics_c > config.electronics_under_temp_clear_c
            )
        else:
            self._electronics_under = reported_electronics_c < config.electronics_under_temp_c
        if self._electronics_over:
            self._electronics_over = (
                not reported_electronics_c < config.electronics_over_temp_clear_c
            )
        else:
            self._electronics_over = reported_electronics_c > config.electronics_over_temp_c
        self._reported_battery_c = reported_battery_c
        self._reported_electronics_c = reported_electronics_c
        self._readings = None

    def snapshot(self) -> ThermalSnapshot:
        """Return the thermal snapshot.

        The truth record holds the true values; the readings record holds the reported
        temperatures and the flags (see the module docstring). The snapshot is built
        once per step, and while the readings are frozen the same readings record is
        reused.
        """
        snap = self._snapshot
        if snap is None:
            readings = self._readings
            if readings is None:
                readings = ThermalReadings(
                    battery_c=self._reported_battery_c,
                    electronics_c=self._reported_electronics_c,
                    over_temp=self._battery_over or self._electronics_over,
                    under_temp=self._battery_under or self._electronics_under,
                )
                self._readings = readings
            heater_on = self._heater_on
            snap = ThermalSnapshot(
                truth=ThermalTruth(
                    battery_c=self._battery_c,
                    electronics_c=self._electronics_c,
                    heater_on=heater_on,
                    heater_power_w=self._heater_w if heater_on else 0.0,
                ),
                readings=readings,
            )
            self._snapshot = snap
        return snap
