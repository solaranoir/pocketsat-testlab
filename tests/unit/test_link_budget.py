"""Tests for the downlink link budget, the SNR indicator (#163, ADR-0011 §6).

The pinned values are exact: the budget uses only ``+ - * /`` and the portable,
``frexp``-based decibels (ADR-0006 as amended by ADR-0011 §6), so they must hold
bit-for-bit on every CI platform. The accuracy tests compare against the same budget
written with ``math.log10``, which tests may call and simulation code may not.
"""

import dataclasses
import itertools
import math

import pytest

from pocketsat.groundstation import DEFAULT_STATION_CONFIG
from pocketsat.link_budget import (
    DEFAULT_LINK_BUDGET,
    MIN_RANGE_KM,
    SPEED_OF_LIGHT_M_PER_S,
    THERMAL_NOISE_DENSITY_DBM_PER_HZ,
    LinkBudget,
)

SNR_TOLERANCE_DB = 1e-10
"""Documented accuracy of ``snr_db`` against the ``math.log10`` reference
(``docs/groundstation.md``, "SNR indicator"): two ``portable_db`` calls at construction
and one per call, each within 1e-11 dB, with headroom for the sums."""


def _reference_snr_db(budget: LinkBudget, range_km: float) -> float:
    """The same budget written with ``math.log10``: the reference."""
    distance_m = max(range_km, MIN_RANGE_KM) * 1000.0
    fspl = 20.0 * math.log10(4.0 * math.pi * distance_m * budget.frequency_hz / 299_792_458.0)
    noise = -174.0 + 10.0 * math.log10(budget.bandwidth_hz) + budget.noise_figure_db
    tx_power_dbm = 10.0 * math.log10(budget.tx_power_w * 1000.0)
    return (
        tx_power_dbm
        + budget.tx_antenna_gain_db
        + budget.rx_antenna_gain_db
        - fspl
        - budget.losses_db
        - noise
    )


# --- Defaults ---------------------------------------------------------------------------


def test_defaults_are_the_documented_amateur_uhf_link() -> None:
    assert (
        LinkBudget(
            frequency_hz=435_500_000,
            tx_power_w=1.0,
            tx_antenna_gain_db=0.0,
            rx_antenna_gain_db=12.0,
            losses_db=8.0,
            bandwidth_hz=15_000,
            noise_figure_db=3.0,
        )
        == DEFAULT_LINK_BUDGET
    )


def test_default_frequency_is_the_station_downlink_frequency() -> None:
    # The link does not depend on the station (ADR-0010 §5), so the two defaults are
    # kept equal by this test rather than by an import.
    assert DEFAULT_LINK_BUDGET.frequency_hz == DEFAULT_STATION_CONFIG.downlink_frequency_hz


def test_constants() -> None:
    assert SPEED_OF_LIGHT_M_PER_S == 299_792_458.0
    assert THERMAL_NOISE_DENSITY_DBM_PER_HZ == -174.0
    # kT₀ at 290 K in dBm/Hz is -173.98; the budget uses the conventional -174 (ADR-0011).
    assert 10.0 * math.log10(1.380649e-23 * 290.0 * 1000.0) == pytest.approx(-174.0, abs=0.03)
    assert MIN_RANGE_KM == 1.0


# --- Pinned values (portability) --------------------------------------------------------


def test_default_fixed_terms_pinned() -> None:
    assert DEFAULT_LINK_BUDGET.tx_power_dbm == 30.0
    assert DEFAULT_LINK_BUDGET.noise_floor_dbm == -129.2390874094432


@pytest.mark.parametrize(
    ("range_km", "snr_db", "path_loss_db"),
    [
        (1.0, 78.01154100068618, 85.22754640875701),
        (380.0, 26.415869068349977, 136.8232183410932),
        (500.0, 24.032140913965804, 139.20694649547738),
        (812.25, 19.81774660034577, 143.42134080909742),
        (1000.0, 18.01154100068618, 145.227546408757),
        (2000.0, 11.990941087406554, 151.24814632203663),
        (2500.0, 10.05274082724543, 153.18634658219776),
    ],
)
def test_default_budget_pinned_values(range_km: float, snr_db: float, path_loss_db: float) -> None:
    assert DEFAULT_LINK_BUDGET.snr_db(range_km) == snr_db
    assert DEFAULT_LINK_BUDGET.path_loss_db(range_km) == path_loss_db


# --- Accuracy against the math reference ------------------------------------------------


def test_snr_matches_the_math_reference_over_every_range() -> None:
    # 1 km to 5000 km in 1 km steps covers every range of a LEO pass several times over.
    worst = max(
        abs(DEFAULT_LINK_BUDGET.snr_db(float(r)) - _reference_snr_db(DEFAULT_LINK_BUDGET, r))
        for r in range(1, 5001)
    )
    assert worst < SNR_TOLERANCE_DB


@pytest.mark.parametrize(
    "budget",
    [
        LinkBudget(),
        LinkBudget(frequency_hz=145_850_000, bandwidth_hz=3_000, noise_figure_db=1.5),
        LinkBudget(tx_power_w=0.1, tx_antenna_gain_db=-3.0, rx_antenna_gain_db=18.5),
        LinkBudget(tx_power_w=5, losses_db=0.0, noise_figure_db=0.0, bandwidth_hz=1),
        LinkBudget(frequency_hz=2_400_000_000, bandwidth_hz=500_000, losses_db=12.25),
    ],
    ids=["default", "vhf-narrow", "low-power", "edge-zeroes", "s-band"],
)
@pytest.mark.parametrize("range_km", [0.5, 1.0, 400.0, 1234.5678, 2300.0, 40_000.0])
def test_other_budgets_match_the_math_reference(budget: LinkBudget, range_km: float) -> None:
    assert budget.snr_db(range_km) == pytest.approx(
        _reference_snr_db(budget, range_km), abs=SNR_TOLERANCE_DB
    )


def test_path_loss_matches_the_textbook_formula() -> None:
    # FSPL = 20 log₁₀(d / km) + 20 log₁₀(f / MHz) + 32.45 (the constant is 32.4478).
    for range_km in (400.0, 1000.0, 2200.0):
        textbook = 20.0 * math.log10(range_km) + 20.0 * math.log10(435.5) + 32.45
        assert DEFAULT_LINK_BUDGET.path_loss_db(range_km) == pytest.approx(textbook, abs=0.01)


def test_snr_plus_path_loss_is_independent_of_range() -> None:
    sums = {
        DEFAULT_LINK_BUDGET.snr_db(r) + DEFAULT_LINK_BUDGET.path_loss_db(r)
        for r in (1.0, 380.0, 2000.0)
    }
    assert max(sums) - min(sums) < 1e-12


def test_doubling_the_range_costs_six_db() -> None:
    drop = DEFAULT_LINK_BUDGET.snr_db(500.0) - DEFAULT_LINK_BUDGET.snr_db(1000.0)
    assert drop == pytest.approx(20.0 * math.log10(2.0), abs=1e-12)


def test_snr_falls_strictly_with_range() -> None:
    values = [DEFAULT_LINK_BUDGET.snr_db(r / 10) for r in range(10, 30_001, 7)]
    assert all(a > b for a, b in itertools.pairwise(values))


def test_each_term_moves_the_snr_by_its_own_decibels() -> None:
    base = DEFAULT_LINK_BUDGET.snr_db(1000.0)
    replace = dataclasses.replace
    assert replace(DEFAULT_LINK_BUDGET, tx_antenna_gain_db=2.5).snr_db(1000.0) == pytest.approx(
        base + 2.5, abs=1e-12
    )
    assert replace(DEFAULT_LINK_BUDGET, rx_antenna_gain_db=15.0).snr_db(1000.0) == pytest.approx(
        base + 3.0, abs=1e-12
    )
    assert replace(DEFAULT_LINK_BUDGET, losses_db=10.0).snr_db(1000.0) == pytest.approx(
        base - 2.0, abs=1e-12
    )
    assert replace(DEFAULT_LINK_BUDGET, noise_figure_db=4.0).snr_db(1000.0) == pytest.approx(
        base - 1.0, abs=1e-12
    )
    assert replace(DEFAULT_LINK_BUDGET, tx_power_w=2.0).snr_db(1000.0) == pytest.approx(
        base + 10.0 * math.log10(2.0), abs=1e-10
    )
    assert replace(DEFAULT_LINK_BUDGET, bandwidth_hz=30_000).snr_db(1000.0) == pytest.approx(
        base - 10.0 * math.log10(2.0), abs=1e-10
    )


# --- Short ranges -----------------------------------------------------------------------


@pytest.mark.parametrize("range_km", [0.0, 1e-300, 0.25, 0.999999])
def test_ranges_below_the_minimum_are_evaluated_at_the_minimum(range_km: float) -> None:
    assert DEFAULT_LINK_BUDGET.snr_db(range_km) == DEFAULT_LINK_BUDGET.snr_db(MIN_RANGE_KM)
    assert DEFAULT_LINK_BUDGET.path_loss_db(range_km) == DEFAULT_LINK_BUDGET.path_loss_db(
        MIN_RANGE_KM
    )


# --- Validation -------------------------------------------------------------------------


@pytest.mark.parametrize("range_km", [-1e-9, -1.0, math.inf, -math.inf, math.nan])
def test_invalid_ranges_are_rejected(range_km: float) -> None:
    with pytest.raises(ValueError, match="range_km"):
        DEFAULT_LINK_BUDGET.snr_db(range_km)
    with pytest.raises(ValueError, match="range_km"):
        DEFAULT_LINK_BUDGET.path_loss_db(range_km)


@pytest.mark.parametrize("range_km", [True, "400", None])
def test_non_numeric_ranges_are_rejected(range_km: object) -> None:
    with pytest.raises(TypeError, match="range_km"):
        DEFAULT_LINK_BUDGET.snr_db(range_km)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("frequency_hz", 0),
        ("frequency_hz", -435_500_000),
        ("bandwidth_hz", 0),
        ("tx_power_w", 0.0),
        ("tx_power_w", -1.0),
        ("tx_power_w", math.inf),
        ("tx_antenna_gain_db", math.nan),
        ("rx_antenna_gain_db", -math.inf),
        ("losses_db", -0.5),
        ("noise_figure_db", -0.1),
        ("noise_figure_db", math.nan),
    ],
)
def test_out_of_range_fields_are_rejected(field: str, value: float) -> None:
    with pytest.raises(ValueError, match=field):
        LinkBudget(**{field: value})


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("frequency_hz", 435.5e6),
        ("frequency_hz", True),
        ("bandwidth_hz", "15000"),
        ("tx_power_w", True),
        ("tx_power_w", "1"),
        ("losses_db", None),
    ],
)
def test_wrong_typed_fields_are_rejected(field: str, value: object) -> None:
    with pytest.raises(TypeError, match=field):
        LinkBudget(**{field: value})  # type: ignore[arg-type]


def test_negative_gains_are_allowed() -> None:
    # A spacecraft antenna null toward the station is a negative gain.
    budget = LinkBudget(tx_antenna_gain_db=-10.0, rx_antenna_gain_db=-1.0)
    assert budget.snr_db(1000.0) == pytest.approx(
        DEFAULT_LINK_BUDGET.snr_db(1000.0) - 23.0, abs=1e-12
    )


# --- Record behaviour -------------------------------------------------------------------


def test_budget_is_frozen() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        DEFAULT_LINK_BUDGET.tx_power_w = 2.0  # type: ignore[misc]


def test_replace_recomputes_the_constant_terms() -> None:
    budget = dataclasses.replace(DEFAULT_LINK_BUDGET, frequency_hz=145_850_000)
    expected = DEFAULT_LINK_BUDGET.snr_db(1000.0) + 20.0 * math.log10(435_500_000 / 145_850_000)
    assert budget.snr_db(1000.0) == pytest.approx(expected, abs=1e-10)


def test_equality_and_repr_show_only_the_configured_fields() -> None:
    assert LinkBudget() == DEFAULT_LINK_BUDGET
    assert hash(LinkBudget()) == hash(DEFAULT_LINK_BUDGET)
    assert "_snr" not in repr(DEFAULT_LINK_BUDGET)
    assert "tx_power_w=1.0" in repr(DEFAULT_LINK_BUDGET)
