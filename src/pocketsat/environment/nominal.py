"""Nominal environment model: a repeatable sunlight/eclipse cycle.

The environment model runs on the orchestrator side and produces an
``EnvironmentState`` each tick (ADR-0002, ADR-0003). ``NominalEnvironment`` is a pure
function of simulated time, so the same time always gives the same state.
"""

from decimal import Decimal
from fractions import Fraction
from typing import Final, Protocol, runtime_checkable

from pocketsat.core.clock import SimClock, check_us
from pocketsat.targets.base import EnvironmentState

DEFAULT_ORBIT_PERIOD_US: Final = 92 * 60 * 1_000_000
"""Default orbit period: 92 minutes, typical of low Earth orbit."""

DEFAULT_ECLIPSE_FRACTION: Final = 0.35
"""Default fraction of each orbit spent in eclipse, typical of low Earth orbit."""


@runtime_checkable
class EnvironmentModel(Protocol):
    """Anything that produces the environment for a given simulated time."""

    def state_at(self, now_us: int) -> EnvironmentState:
        """Return the environment at simulated time ``now_us``."""
        ...


class NominalEnvironment:
    """A circular orbit with a fixed eclipse, and otherwise constant nominal conditions.

    Each orbit starts in sunlight at ``now_us = 0`` (mod the period) and ends in
    eclipse. With period ``P`` and eclipse length ``E``:

    - sunlit for ``0 <= now_us % P < P - E``
    - eclipse for ``P - E <= now_us % P < P``

    ``E`` is ``eclipse_fraction * P`` rounded to the nearest microsecond. It is computed
    once, exactly (floats are read via their decimal form), so the cycle has no
    floating-point time and never drifts.
    """

    def __init__(
        self,
        orbit_period_us: int = DEFAULT_ORBIT_PERIOD_US,
        eclipse_fraction: float | Fraction = DEFAULT_ECLIPSE_FRACTION,
        *,
        ambient_temp_c: float = EnvironmentState.ambient_temp_c,
        sensor_noise_scale: float = EnvironmentState.sensor_noise_scale,
    ) -> None:
        """Create the model.

        Args:
            orbit_period_us: Orbit period in microseconds (default 92 minutes).
            eclipse_fraction: Fraction of each orbit in eclipse, ``0`` (always sunlit)
                to ``1`` (always in eclipse). Default 0.35.
            ambient_temp_c: Ambient temperature reported in every state.
            sensor_noise_scale: Sensor noise scale reported in every state.

        Raises:
            TypeError: ``orbit_period_us`` is not an int, or ``eclipse_fraction`` is not
                a number.
            ValueError: A parameter is out of range.
        """
        check_us("orbit_period_us", orbit_period_us)
        if orbit_period_us == 0:
            raise ValueError("orbit_period_us must be positive")
        fraction = _exact_fraction(eclipse_fraction)
        if not 0 <= fraction <= 1:
            raise ValueError(f"eclipse_fraction must be in 0..1, got {eclipse_fraction}")

        self._period_us = orbit_period_us
        self._eclipse_us = round(fraction * orbit_period_us)
        self._sunlit = EnvironmentState(
            sunlit=True, ambient_temp_c=ambient_temp_c, sensor_noise_scale=sensor_noise_scale
        )
        self._eclipse = EnvironmentState(
            sunlit=False, ambient_temp_c=ambient_temp_c, sensor_noise_scale=sensor_noise_scale
        )

    @property
    def orbit_period_us(self) -> int:
        """Orbit period in microseconds."""
        return self._period_us

    @property
    def eclipse_us(self) -> int:
        """Eclipse duration per orbit in microseconds."""
        return self._eclipse_us

    @property
    def sunlit_us(self) -> int:
        """Sunlit duration per orbit in microseconds."""
        return self._period_us - self._eclipse_us

    def state_at(self, now_us: int) -> EnvironmentState:
        """Return the environment at simulated time ``now_us``.

        Raises:
            TypeError: ``now_us`` is not an int.
            ValueError: ``now_us`` is negative.
        """
        check_us("now_us", now_us)
        return self._sunlit if now_us % self._period_us < self.sunlit_us else self._eclipse

    def sample(self, clock: SimClock) -> EnvironmentState:
        """Return the environment at the clock's current time."""
        return self.state_at(clock.now_us)


def _exact_fraction(value: float | Fraction) -> Fraction:
    if isinstance(value, bool) or not isinstance(value, int | float | Fraction):
        raise TypeError(f"eclipse_fraction must be a number, got {value!r}")
    try:
        return Fraction(Decimal(str(value))) if isinstance(value, float) else Fraction(value)
    except (ValueError, OverflowError, ArithmeticError) as exc:
        raise ValueError(f"eclipse_fraction must be finite, got {value}") from exc
