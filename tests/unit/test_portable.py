"""Tests for portable arithmetic helpers (ADR-0006).

The pinned values are exact. The helpers use only + - * /, ``sqrt``, ``floor`` and
``frexp``, which IEEE 754 rounds
identically everywhere, so these assertions must hold bit-for-bit on every CI platform.
Tests may call ``math.cos`` as a reference; simulation code may not.
"""

import math

import pytest

from pocketsat.core import (
    PI,
    RngFactory,
    portable_atan2,
    portable_cos_deg,
    portable_db,
    portable_log2,
    portable_normal,
    portable_sincos_turns,
)
from pocketsat.core.rng import NORMAL_DRAWS

# --- portable_cos_deg -----------------------------------------------------------------


def test_pi_literal_matches_math_pi() -> None:
    assert math.pi == PI


@pytest.mark.parametrize(
    ("angle_deg", "expected"),
    [
        (0.0, 1.0),
        (30.0, 0.8648648648648649),
        (45.0, 0.7058823529411764),
        (60.0, 0.5000000000000001),
        (89.0, 0.01775749609384696),
        (90.0, 0.0),
        (-45.0, 0.7058823529411764),
    ],
)
def test_cos_pinned_values(angle_deg: float, expected: float) -> None:
    assert portable_cos_deg(angle_deg) == expected


def test_cos_max_error_against_math_cos() -> None:
    worst = max(
        abs(portable_cos_deg(hundredths / 100) - math.cos(math.radians(hundredths / 100)))
        for hundredths in range(-9000, 9001)
    )
    assert worst < 0.002


@pytest.mark.parametrize("angle_deg", [90.0, 90.0001, 135.0, 180.0, -100.0, 1e9])
def test_cos_clamped_to_zero_beyond_90(angle_deg: float) -> None:
    assert portable_cos_deg(angle_deg) == 0.0


def test_cos_is_even_and_bounded() -> None:
    for tenths in range(0, 901):
        value = portable_cos_deg(tenths / 10)
        assert value == portable_cos_deg(-tenths / 10)
        assert 0.0 <= value <= 1.0


@pytest.mark.parametrize("angle_deg", [float("nan"), float("inf"), -float("inf")])
def test_cos_rejects_non_finite(angle_deg: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        portable_cos_deg(angle_deg)


# --- portable_normal ------------------------------------------------------------------


def test_normal_pinned_values() -> None:
    rng = RngFactory(42).stream("test.noise")
    assert [portable_normal(rng) for _ in range(3)] == [
        -0.2982894991520144,
        1.6569846577973903,
        0.7749277867230422,
    ]


def test_normal_uses_exactly_twelve_draws() -> None:
    assert NORMAL_DRAWS == 12
    a = RngFactory(7).stream("s")
    b = RngFactory(7).stream("s")
    portable_normal(a)
    for _ in range(12):
        b.random()
    assert a.random() == b.random()


def test_normal_equals_the_documented_sum_bit_for_bit() -> None:
    # #121 unrolled the 12 draws; the sum must stay the loop's, added left to right.
    a = RngFactory(121).stream("s")
    b = RngFactory(121).stream("s")
    for n in range(20_000):
        mu, sigma = (n % 7) * 0.3 - 1.0, (n % 5) * 0.7
        total = 0.0
        for _ in range(NORMAL_DRAWS):
            total += b.random()
        assert portable_normal(a, mu, sigma) == mu + sigma * (total - 6.0)


def test_normal_mean_and_variance() -> None:
    rng = RngFactory(42).stream("test.noise")
    samples = [portable_normal(rng) for _ in range(100_000)]
    mean = sum(samples) / len(samples)
    variance = sum((x - mean) * (x - mean) for x in samples) / len(samples)
    assert abs(mean) < 0.02
    assert abs(variance - 1.0) < 0.02


def test_normal_scaling_and_bound() -> None:
    rng = RngFactory(3).stream("test.bound")
    samples = [portable_normal(rng, mu=10.0, sigma=0.5) for _ in range(50_000)]
    assert all(10.0 - 6 * 0.5 <= x <= 10.0 + 6 * 0.5 for x in samples)
    assert abs(sum(samples) / len(samples) - 10.0) < 0.02


def test_normal_zero_sigma_returns_mu() -> None:
    rng = RngFactory(1).stream("s")
    assert portable_normal(rng, mu=2.5, sigma=0.0) == 2.5


@pytest.mark.parametrize(
    ("mu", "sigma"), [(0.0, -1.0), (0.0, float("nan")), (float("inf"), 1.0), (0.0, float("inf"))]
)
def test_normal_rejects_invalid_parameters(mu: float, sigma: float) -> None:
    with pytest.raises(ValueError):
        portable_normal(RngFactory(1).stream("s"), mu=mu, sigma=sigma)


# --- portable_sincos_turns, portable_atan2, portable_log2, portable_db (ADR-0011) -------
#
# Accuracy bounds are ADR-0011 §5's: 1e-12 (sine, cosine, arctangent, log2) and 1e-11 dB.
# The pinned values make any reordering of operations a test failure on every platform.

SINCOS_BOUND = 1e-12
ATAN2_BOUND = 1e-12
LOG2_BOUND = 1e-12
DB_BOUND = 1e-11


@pytest.mark.parametrize(
    ("turns", "expected"),
    [
        (0.0, (0.0, 1.0)),
        (0.125, (0.7071067811865475, 0.7071067811865476)),
        (0.25, (1.0, 0.0)),
        (1 / 3, (0.8660254037844387, -0.49999999999999983)),
        (0.5, (0.0, -1.0)),
        (0.7, (-0.9510565162951535, -0.3090169943749477)),
        (-0.1, (-0.587785252292473, 0.8090169943749475)),
        (12345.678, (-0.8994052515660513, -0.43711576665159074)),
    ],
)
def test_sincos_pinned_values(turns: float, expected: tuple[float, float]) -> None:
    assert portable_sincos_turns(turns) == expected


def test_sincos_accuracy_over_three_turns() -> None:
    worst = 0.0
    for step in range(-15_000, 15_001):
        turns = step / 10_000 + 0.000_037  # off the quadrant boundaries too
        sin_value, cos_value = portable_sincos_turns(turns)
        angle = 2.0 * math.pi * turns
        worst = max(worst, abs(sin_value - math.sin(angle)), abs(cos_value - math.cos(angle)))
    assert worst < SINCOS_BOUND


@pytest.mark.parametrize("turns", [0.0, 0.25, 0.5, 0.75, 1.0, 2.0, -0.25, 1e6])
def test_sincos_exact_at_quadrant_boundaries(turns: float) -> None:
    sin_value, cos_value = portable_sincos_turns(turns)
    expected = {0.0: (0.0, 1.0), 0.25: (1.0, 0.0), 0.5: (0.0, -1.0), 0.75: (-1.0, 0.0)}
    assert (sin_value, cos_value) == expected[turns % 1.0]


def test_sincos_just_below_a_whole_turn() -> None:
    sin_value, cos_value = portable_sincos_turns(math.nextafter(1.0, 0.0))
    assert abs(sin_value) < 1e-15
    assert cos_value == 1.0


@pytest.mark.parametrize(
    ("y", "x", "expected"),
    [
        (1.0, 1.0, 0.7853981633974483),
        (1.0, -1.0, 2.356194490192345),
        (-1.0, -1.0, -2.356194490192345),
        (0.5, 2.0, 0.24497866312686412),
        (3.0, 0.2, 1.5042281630190728),
        (0.0, -1.0, 3.141592653589793),
        (-2.0, 0.0, -1.5707963267948966),
    ],
)
def test_atan2_pinned_values(y: float, x: float, expected: float) -> None:
    assert portable_atan2(y, x) == expected


def test_atan2_accuracy_over_a_grid() -> None:
    worst = 0.0
    for i in range(-300, 301):
        for j in range(-300, 301):
            y, x = i / 37.0, j / 41.0
            if (y, x) == (0.0, 0.0):
                continue
            worst = max(worst, abs(portable_atan2(y, x) - math.atan2(y, x)))
    assert worst < ATAN2_BOUND


def test_atan2_conventions() -> None:
    assert portable_atan2(0.0, 0.0) == 0.0
    assert portable_atan2(-0.0, -1.0) == PI  # the sign of a zero y is ignored
    assert portable_atan2(1.0, 0.0) == PI / 2
    assert portable_atan2(1e-300, 1e300) == 0.0
    assert portable_atan2(1e300, 1e-300) == PI / 2


@pytest.mark.parametrize(
    ("x", "log2", "db"),
    [
        (1.0, 0.0, 0.0),
        (2.0, 1.0, 3.010299956639812),
        (10.0, 3.321928094887362, 10.0),
        (0.3, -1.7369655941662063, -5.228787452803377),
        (1e-20, -66.43856189774725, -200.00000000000003),
        (6.02e23, 78.9941096693943, 237.79596491257823),
        (5e-324, -1074.0, -3233.062153431158),
    ],
)
def test_log2_and_db_pinned_values(x: float, log2: float, db: float) -> None:
    assert portable_log2(x) == log2
    assert portable_db(x) == db


def test_log2_and_db_accuracy_over_the_float_range() -> None:
    worst_log2 = worst_db = 0.0
    for step in range(-3_000, 3_001):
        x = 10.0 ** (step / 10.0) * 1.000_123
        worst_log2 = max(worst_log2, abs(portable_log2(x) - math.log2(x)))
        worst_db = max(worst_db, abs(portable_db(x) - 10.0 * math.log10(x)))
    assert worst_log2 < LOG2_BOUND
    assert worst_db < DB_BOUND


def test_log2_exact_for_powers_of_two() -> None:
    for exponent in range(-1074, 1024):
        assert portable_log2(2.0**exponent) == exponent


@pytest.mark.parametrize("bad", [0.0, -1.0, -0.0])
def test_log_rejects_non_positive(bad: float) -> None:
    with pytest.raises(ValueError, match="positive"):
        portable_log2(bad)
    with pytest.raises(ValueError, match="positive"):
        portable_db(bad)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_new_helpers_reject_non_finite(bad: float) -> None:
    for call in (
        lambda: portable_sincos_turns(bad),
        lambda: portable_atan2(bad, 1.0),
        lambda: portable_atan2(1.0, bad),
        lambda: portable_log2(bad),
        lambda: portable_db(bad),
    ):
        with pytest.raises(ValueError, match="finite"):
            call()
