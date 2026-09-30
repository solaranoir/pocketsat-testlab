"""Simulated clock (ADR-0003).

Simulated time is an integer count of microseconds, advanced one fixed tick at a
time. There is no floating-point time: ``now_us`` is always ``tick_index * tick_us``,
so it cannot drift however many ticks run.
"""

import math
from decimal import Decimal, InvalidOperation
from fractions import Fraction
from typing import Final

US_PER_S: Final = 1_000_000
DEFAULT_TICK_US: Final = 100_000
"""Default tick: 100 ms."""

Seconds = int | float | Decimal | Fraction | str
"""Accepted duration types. Floats are read via their shortest decimal form, so
``0.3`` means exactly 3/10 of a second, not the nearest binary double."""


def _exact_seconds(seconds: Seconds) -> Fraction:
    if isinstance(seconds, bool):
        raise TypeError("duration must be a number of seconds, not bool")
    try:
        if isinstance(seconds, float | str):
            value = Fraction(Decimal(str(seconds).strip()))
        else:
            value = Fraction(seconds)
    except (InvalidOperation, ValueError, OverflowError) as exc:
        raise ValueError(f"invalid duration: {seconds!r}") from exc
    if value < 0:
        raise ValueError(f"duration must be non-negative, got {seconds!r}")
    return value


def seconds_to_ticks(seconds: Seconds, tick_us: int = DEFAULT_TICK_US) -> int:
    """Quantize a duration to a whole number of ticks, rounding **up**.

    A duration that is an exact multiple of the tick maps to that many ticks. Any
    remainder, however small, adds one tick, so an event is never scheduled earlier
    than requested. For example, with 100 ms ticks: 0.3 s is 3 ticks, 0.30001 s is
    4 ticks, and 1 microsecond is 1 tick.

    Args:
        seconds: Non-negative duration. Floats are interpreted by their decimal repr.
        tick_us: Tick length in microseconds.

    Returns:
        The number of ticks.

    Raises:
        ValueError: The duration is negative, NaN, infinite, or unparseable, or
            ``tick_us`` is not positive.
        TypeError: The duration is a bool.
    """
    _check_tick_us(tick_us)
    return math.ceil(_exact_seconds(seconds) * US_PER_S / tick_us)


def _check_tick_us(tick_us: int) -> None:
    if isinstance(tick_us, bool) or not isinstance(tick_us, int):
        raise TypeError(f"tick_us must be an int, got {type(tick_us).__name__}")
    if tick_us <= 0:
        raise ValueError(f"tick_us must be positive, got {tick_us}")


class SimClock:
    """Fixed-tick simulated clock holding time as integer microseconds.

    Owned by the orchestrator. Simulation code reads it; only the run loop advances it.
    """

    def __init__(self, tick_us: int = DEFAULT_TICK_US) -> None:
        """Create a clock at time zero.

        Args:
            tick_us: Tick length in microseconds (default 100 ms).

        Raises:
            TypeError: ``tick_us`` is not an int.
            ValueError: ``tick_us`` is not positive.
        """
        _check_tick_us(tick_us)
        self._tick_us = tick_us
        self._tick_index = 0

    @property
    def tick_us(self) -> int:
        """Tick length in microseconds."""
        return self._tick_us

    @property
    def tick_index(self) -> int:
        """Number of ticks advanced since time zero."""
        return self._tick_index

    @property
    def now_us(self) -> int:
        """Current simulated time in microseconds (``tick_index * tick_us``)."""
        return self._tick_index * self._tick_us

    def advance_one_tick(self) -> None:
        """Advance simulated time by exactly one tick."""
        self._tick_index += 1

    def seconds_to_ticks(self, seconds: Seconds) -> int:
        """Quantize a duration to whole ticks of this clock, rounding up.

        See :func:`seconds_to_ticks`.
        """
        return seconds_to_ticks(seconds, self._tick_us)
