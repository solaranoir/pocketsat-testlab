"""Scalar attitude model (#41): pointing error, angular rate, and control state.

Attitude is two damped scalars integrated step by step with portable arithmetic only
(ADR-0006): no trigonometry, no exponentials, and noise from
:func:`~pocketsat.core.rng.portable_normal`. Time steps are integer microseconds,
converted to seconds as ``dt_us / 1_000_000``.

**Pointing error** is measured against the sun-optimal attitude: at 0° the solar
arrays point at the sun. Power (#35) reads the *true* pointing error for its solar
pointing factor, and the payload (#43) acquires only while the *reported* state is
``STABILIZED``. Internally the model keeps a rotation phase ``0 <= phase < 360``
degrees; the pointing error is that phase folded into 0..180 (``phase`` up to 180,
``360 - phase`` above), so a tumbling spacecraft sweeps its error from 0° to 180° and
back.

Each tick, with ``dt`` in seconds and ``n`` a draw from the disturbance stream:

1. **Rate.** ``rate = |rate * damp + disturbance_mean_dps_per_s * dt +
   disturbance_sd_dps_per_s * sqrt(dt) * n|``, where ``damp`` is
   ``max(0, 1 - rate_damping_per_s * dt)`` while attitude control is on and 1 while it
   is off. The disturbance (stream ``spacecraft.attitude.disturbance``) is true
   dynamics and is *not* scaled by ``sensor_noise_scale``. Its mean is non-negative,
   so with control off nothing corrects it and the rate drifts upward toward
   ``TUMBLING``.
2. **Pointing.** The phase advances by ``rate * dt``. While control is on, the
   pointing error then shrinks by the factor ``max(0, 1 - pointing_gain_per_s * dt)``
   (toward 0° along the shorter way). With control on, the error settles near
   ``rate / pointing_gain_per_s``.
3. **State** (from the true rate and pointing error, with hysteresis, see
   :func:`next_state`).
4. **Control draw.** ``control_power_w`` from ``AttitudeConfig`` while control is on,
   0 while it is off. Power (#36) adds it to the total load.

**States** (``AttitudeState``), with thresholds from ``AttitudeConfig``:

- ``TUMBLING`` whenever the rate is above ``tumbling_enter_rate_dps``. It stays
  ``TUMBLING`` until control is on and the rate is at or below
  ``tumbling_exit_rate_dps``.
- ``DETUMBLING`` while control is reducing the rate: between leaving ``TUMBLING`` and
  reaching ``STABILIZED``, or after leaving ``STABILIZED``.
- ``STABILIZED`` once the rate is below ``stabilized_enter_rate_dps`` and the pointing
  error is at or below ``stabilized_enter_pointing_error_deg``. It is kept until the
  rate exceeds ``stabilized_exit_rate_dps`` or the error exceeds
  ``stabilized_exit_pointing_error_deg``.

The gaps between entry and exit thresholds are the hysteresis: a value hovering at one
threshold cannot make the state chatter.

**Causes of tumbling:** the starting rate (``AttitudeInitial``), attitude control
switched off (for example during BOOT, #49), and the seeded disturbance. An
attitude-control failure fault is a Phase 4 candidate, not part of Phase 1.

**Readings** report the pointing error and rate with noise from a separate stream
(``spacecraft.attitude.noise``), standard deviations ``pointing_noise_deg`` and
``rate_noise_dps`` scaled by ``EnvironmentState.sensor_noise_scale`` (0.0 gives the
exact true values). Reported values are clamped to their ranges (0..180°, rate
non-negative). The reported state is decided from the reported values with the same
rules, tracked separately from the true state. While ``attitude`` is in
``controls.frozen_sensors``, the whole readings record holds exactly its last
pre-freeze value (no noise is drawn) while the truth keeps evolving; live readings
resume on release. Before the first step the readings equal the truth.

Attitude reads nothing from other subsystems.

**Simplifications** (Phase 1):

- A single scalar pointing error and a single scalar rate magnitude, no 3-D attitude.
- No actuator saturation and no momentum build-up: control acts linearly and always.
- No environmental torques beyond the seeded disturbance.
- Antenna pointing does not affect the link in Phase 1; Phase 3's RF channel may use
  the true pointing error for antenna gain.
"""

import math
from random import Random
from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory, portable_normal
from pocketsat.spacecraft.config import AttitudeConfig, AttitudeInitial
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    AttitudeReadings,
    AttitudeSnapshot,
    AttitudeState,
    AttitudeTruth,
)
from pocketsat.targets.base import EnvironmentState

__all__ = [
    "DISTURBANCE_STREAM",
    "NOISE_STREAM",
    "Attitude",
    "next_state",
]

DISTURBANCE_STREAM: Final = "spacecraft.attitude.disturbance"
"""Random stream for the disturbance on the true rate."""

NOISE_STREAM: Final = "spacecraft.attitude.noise"
"""Random stream for the noise on the reported pointing error and rate."""

_US_PER_S: Final = 1_000_000
_FULL_TURN_DEG: Final = 360.0
_HALF_TURN_DEG: Final = 180.0

_TUMBLING: Final = AttitudeState.TUMBLING
_DETUMBLING: Final = AttitudeState.DETUMBLING
_STABILIZED: Final = AttitudeState.STABILIZED


def next_state(
    config: AttitudeConfig,
    previous: AttitudeState,
    rate_dps: float,
    pointing_error_deg: float,
    control_enabled: bool,
) -> AttitudeState:
    """Return the attitude state after ``previous`` given a rate and pointing error.

    Used for both the true state (true values) and the reported state (reported
    values). See the module docstring for the rules; the thresholds come from
    ``config``.

    Args:
        config: Attitude settings holding the thresholds.
        previous: The state before this decision.
        rate_dps: Angular rate magnitude, degrees per second.
        pointing_error_deg: Pointing error, degrees.
        control_enabled: Attitude control is on (``controls.attitude.enabled``).

    Returns:
        The new state.
    """
    if rate_dps > config.tumbling_enter_rate_dps:
        return _TUMBLING
    state = previous
    if state is _TUMBLING:
        if not control_enabled or rate_dps > config.tumbling_exit_rate_dps:
            return _TUMBLING
        state = _DETUMBLING
    if state is _STABILIZED:
        if (
            rate_dps > config.stabilized_exit_rate_dps
            or pointing_error_deg > config.stabilized_exit_pointing_error_deg
        ):
            return _DETUMBLING
        return _STABILIZED
    if (
        rate_dps < config.stabilized_enter_rate_dps
        and pointing_error_deg <= config.stabilized_enter_pointing_error_deg
    ):
        return _STABILIZED
    return _DETUMBLING


def _fold(phase_deg: float) -> float:
    """Pointing error, 0..180 degrees, of a phase in 0..360 degrees."""
    return phase_deg if phase_deg <= _HALF_TURN_DEG else _FULL_TURN_DEG - phase_deg


class Attitude:
    """The attitude subsystem: pointing error, angular rate, and control state.

    Implements the ``Subsystem`` protocol. Built with its settings and starting records
    (ADR-0005); it reads nothing from other subsystems.

    Attributes:
        name: Always ``"attitude"``.
    """

    name = "attitude"

    def __init__(self, config: AttitudeConfig, initial: AttitudeInitial) -> None:
        """Create the attitude subsystem, at its starting state.

        Call :meth:`reset` before :meth:`step`: the random streams come from the run's
        ``RngFactory``.

        Args:
            config: Attitude settings.
            initial: Starting state; :meth:`reset` returns to it.

        Raises:
            TypeError: ``config`` or ``initial`` is not the expected record type.
        """
        if not isinstance(config, AttitudeConfig):
            raise TypeError(f"config must be an AttitudeConfig, got {config!r}")
        if not isinstance(initial, AttitudeInitial):
            raise TypeError(f"initial must be an AttitudeInitial, got {initial!r}")
        self._config = config
        self._initial = initial
        self._disturbance: Random | None = None
        self._noise: Random | None = None
        self._sqrt_dt_us = -1
        self._sqrt_dt_s = 0.0
        self._start()

    def _start(self) -> None:
        initial = self._initial
        self._phase_deg = float(initial.pointing_error_deg)
        self._pointing_error_deg = float(initial.pointing_error_deg)
        self._rate_dps = float(initial.rate_dps)
        self._state = next_state(
            self._config, _DETUMBLING, self._rate_dps, self._pointing_error_deg, True
        )
        self._control_power_w = 0.0
        self._readings = AttitudeReadings(
            pointing_error_deg=self._pointing_error_deg,
            rate_dps=self._rate_dps,
            state=self._state,
        )
        self._snapshot: AttitudeSnapshot | None = None

    def reset(self, rng: RngFactory) -> None:
        """Return to the starting state from ``AttitudeInitial``.

        Requests the streams :data:`DISTURBANCE_STREAM` and :data:`NOISE_STREAM`. The
        starting state is decided from the starting values with no history (as if
        coming from ``DETUMBLING``). Until the first step the readings equal the truth
        and the control draw is zero.
        """
        self._disturbance = rng.stream(DISTURBANCE_STREAM)
        self._noise = rng.stream(NOISE_STREAM)
        self._start()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Advance attitude by ``dt_us`` microseconds.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
            RuntimeError: :meth:`reset` has not been called.
        """
        check_us("dt_us", dt_us)
        disturbance = self._disturbance
        noise = self._noise
        if disturbance is None or noise is None:
            raise RuntimeError("Attitude.reset(rng) must be called before step()")
        config = self._config
        enabled = controls.attitude.enabled
        dt_s = dt_us / _US_PER_S
        if dt_us != self._sqrt_dt_us:
            self._sqrt_dt_us = dt_us
            self._sqrt_dt_s = math.sqrt(dt_s)

        # 1. Rate: damped while control is on, plus the (unscaled) seeded disturbance.
        rate = self._rate_dps
        if enabled:
            rate *= max(0.0, 1.0 - config.rate_damping_per_s * dt_s)
        rate += config.disturbance_mean_dps_per_s * dt_s
        rate += config.disturbance_sd_dps_per_s * self._sqrt_dt_s * portable_normal(disturbance)
        rate = abs(rate)

        # 2. Pointing: the phase advances with the rate, then control shrinks the error.
        phase = self._phase_deg + rate * dt_s
        if phase >= _FULL_TURN_DEG:
            phase -= _FULL_TURN_DEG * math.floor(phase / _FULL_TURN_DEG)
            if phase >= _FULL_TURN_DEG or phase < 0.0:
                phase = 0.0
        error = _fold(phase)
        if enabled:
            error *= max(0.0, 1.0 - config.pointing_gain_per_s * dt_s)
            phase = error if phase <= _HALF_TURN_DEG else _FULL_TURN_DEG - error
            if phase >= _FULL_TURN_DEG:
                phase = 0.0

        # 3. True state, 4. control draw.
        self._rate_dps = rate
        self._phase_deg = phase
        self._pointing_error_deg = error
        self._state = next_state(config, self._state, rate, error, enabled)
        self._control_power_w = config.control_power_w if enabled else 0.0

        # Readings: held exactly while frozen, otherwise noisy and scaled.
        if "attitude" not in controls.frozen_sensors:
            scale = env.sensor_noise_scale
            if scale == 0.0:
                reported_error = error
                reported_rate = rate
            else:
                reported_error = error + portable_normal(
                    noise, 0.0, config.pointing_noise_deg * scale
                )
                reported_error = min(_HALF_TURN_DEG, max(0.0, reported_error))
                reported_rate = max(
                    0.0, rate + portable_normal(noise, 0.0, config.rate_noise_dps * scale)
                )
            self._readings = AttitudeReadings(
                pointing_error_deg=reported_error,
                rate_dps=reported_rate,
                state=next_state(
                    config, self._readings.state, reported_rate, reported_error, enabled
                ),
            )
        self._snapshot = None

    def snapshot(self) -> AttitudeSnapshot:
        """Return the attitude snapshot. Built once per step."""
        snap = self._snapshot
        if snap is None:
            snap = AttitudeSnapshot(
                truth=AttitudeTruth(
                    pointing_error_deg=self._pointing_error_deg,
                    rate_dps=self._rate_dps,
                    state=self._state,
                    control_power_w=self._control_power_w,
                ),
                readings=self._readings,
            )
            self._snapshot = snap
        return snap
