"""Tests for portable arithmetic helpers (ADR-0006).

The pinned values are exact. The helpers use only + - * /, which IEEE 754 rounds
identically everywhere, so these assertions must hold bit-for-bit on every CI platform.
Tests may call ``math.cos`` as a reference; simulation code may not.
"""

import math

import pytest

from pocketsat.core import PI, RngFactory, portable_cos_deg, portable_normal
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
