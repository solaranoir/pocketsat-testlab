"""Unit tests for pocketsat.core.clock."""

from decimal import Decimal
from fractions import Fraction

import pytest

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock, check_us, seconds_to_ticks


def test_starts_at_zero_with_default_tick() -> None:
    clock = SimClock()
    assert clock.tick_us == DEFAULT_TICK_US == 100_000
    assert clock.tick_index == 0
    assert clock.now_us == 0


def test_advance_is_monotonic_and_exact() -> None:
    clock = SimClock(tick_us=250)
    previous = clock.now_us
    for i in range(1, 1001):
        clock.advance_one_tick()
        assert clock.tick_index == i
        assert clock.now_us == previous + 250
        previous = clock.now_us


def test_no_drift_over_one_million_ticks() -> None:
    clock = SimClock()
    for _ in range(1_000_000):
        clock.advance_one_tick()
    assert clock.tick_index == 1_000_000
    assert clock.now_us == 100_000_000_000
    assert type(clock.now_us) is int


@pytest.mark.parametrize("tick_us", [0, -1])
def test_rejects_non_positive_tick(tick_us: int) -> None:
    with pytest.raises(ValueError, match="positive"):
        SimClock(tick_us=tick_us)


@pytest.mark.parametrize("tick_us", [0.1, 100_000.0, True])
def test_rejects_non_int_tick(tick_us: object) -> None:
    with pytest.raises(TypeError):
        SimClock(tick_us=tick_us)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("seconds", "ticks"),
    [
        (0, 0),
        (0.0, 0),
        (0.1, 1),  # exact multiple
        (0.3, 3),  # 0.3 is not exact in binary; must not become 4
        (0.7, 7),
        (1, 10),
        (2.5, 25),
        (0.30001, 4),  # any remainder rounds up
        (0.000001, 1),  # 1 microsecond
        (1e-9, 1),  # below microsecond resolution still rounds up
        (0.1 + 0.2, 4),  # 0.30000000000000004 really is over 0.3
        ("0.3", 3),
        (Decimal("0.3"), 3),
        (Fraction(1, 3), 4),
        (600, 6000),  # a 10-minute pass
    ],
)
def test_seconds_to_ticks_rounds_up(
    seconds: int | float | Decimal | Fraction | str, ticks: int
) -> None:
    assert seconds_to_ticks(seconds) == ticks
    assert SimClock().seconds_to_ticks(seconds) == ticks


def test_seconds_to_ticks_uses_clock_tick() -> None:
    assert SimClock(tick_us=1_000).seconds_to_ticks(0.0015) == 2
    assert seconds_to_ticks(0.0015, tick_us=500) == 3


@pytest.mark.parametrize("seconds", [-0.1, -1, "nan", float("nan"), float("inf"), "abc"])
def test_seconds_to_ticks_rejects_invalid(seconds: object) -> None:
    with pytest.raises(ValueError):
        seconds_to_ticks(seconds)  # type: ignore[arg-type]


def test_seconds_to_ticks_rejects_bool() -> None:
    with pytest.raises(TypeError):
        seconds_to_ticks(True)


@pytest.mark.parametrize("value", [0, 1, 100_000, 10**15])
def test_check_us_accepts_non_negative_ints(value: int) -> None:
    check_us("x", value)


@pytest.mark.parametrize(
    ("value", "error"),
    [(-1, ValueError), (0.1, TypeError), (1.0, TypeError), (True, TypeError), ("1", TypeError)],
)
def test_check_us_rejects_floats_bools_and_negatives(value: object, error: type[Exception]) -> None:
    with pytest.raises(error, match="dt_us"):
        check_us("dt_us", value)  # type: ignore[arg-type]
