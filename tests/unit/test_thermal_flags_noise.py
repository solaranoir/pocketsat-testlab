"""Tests for the thermal sensors and limit flags (#39): noise, freeze, flags, thresholds.

Most flag tests use :data:`EXACT`, a configuration in which one step of
:data:`LAND_US` takes both nodes exactly to the ambient (``dt * G / C = 1``, no
coupling, no heater, no reader), and :data:`HALF`, in which the battery lands on the
ambient and the electronics move exactly halfway. With ``sensor_noise_scale`` 0.0 the
reported temperatures then equal chosen values exactly, so thresholds can be tested at
their boundaries. All the chosen values are exact in binary.
"""

import dataclasses
import itertools
import statistics
from collections.abc import Sequence
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory, portable_normal
from pocketsat.spacecraft import (
    NOMINAL_CONFIG,
    SpacecraftControls,
    Thermal,
    ThermalConfig,
    ThermalInitial,
    ThermalReadings,
    ThermalSnapshot,
)
from pocketsat.spacecraft.thermal import NOISE_STREAM
from pocketsat.targets.base import EnvironmentState

SECOND_US = 1_000_000
TICK_US = 100_000
CONFIG = ThermalConfig()
CONTROLS = SpacecraftControls()
FROZEN = SpacecraftControls(frozen_sensors=frozenset({"thermal"}))

EXACT = dataclasses.replace(
    CONFIG,
    battery_heat_capacity_j_per_c=64.0,
    electronics_heat_capacity_j_per_c=64.0,
    battery_conductance_w_per_c=0.125,
    electronics_conductance_w_per_c=0.125,
    coupling_conductance_w_per_c=0.0,
    heater_power_w=0.0,
)
"""Both nodes reach the ambient exactly in one :data:`LAND_US` step."""

HALF = dataclasses.replace(EXACT, electronics_heat_capacity_j_per_c=128.0)
"""The battery reaches the ambient in one step; the electronics move halfway."""

LAND_US = 512 * SECOND_US
"""``C / G`` = 64 / 0.125 = 512 s."""

WIDE_BATTERY: dict[str, float] = {
    "battery_survival_limit_c": -200.0,
    "battery_under_temp_c": -150.0,
    "battery_under_temp_clear_c": -149.0,
    "battery_over_temp_clear_c": 149.0,
    "battery_over_temp_c": 150.0,
}
"""Battery thresholds far out of the way, to test the electronics alone."""

WIDE_ELECTRONICS: dict[str, float] = {
    "electronics_under_temp_c": -150.0,
    "electronics_under_temp_clear_c": -149.0,
    "electronics_over_temp_clear_c": 149.0,
    "electronics_over_temp_c": 150.0,
}
"""Electronics thresholds far out of the way, to test the battery alone."""


def _env(ambient_c: float, scale: float = 0.0) -> EnvironmentState:
    return EnvironmentState(ambient_temp_c=ambient_c, sensor_noise_scale=scale)


def _thermal(
    config: ThermalConfig = EXACT,
    initial: ThermalInitial | None = None,
    seed: int = 1,
) -> Thermal:
    thermal = Thermal(config, initial or ThermalInitial())
    thermal.reset(RngFactory(seed))
    return thermal


def _drive(
    thermal: Thermal,
    ambients: Sequence[float],
    controls: SpacecraftControls = CONTROLS,
    scale: float = 0.0,
    dt_us: int = LAND_US,
) -> list[ThermalSnapshot]:
    out = []
    for ambient_c in ambients:
        thermal.step(dt_us, _env(ambient_c, scale), controls)
        out.append(thermal.snapshot())
    return out


# --- Settings ---------------------------------------------------------------------------


def test_new_defaults_are_documented_values() -> None:
    config = NOMINAL_CONFIG.thermal
    assert config.temperature_noise_c == 0.2
    assert config.battery_survival_limit_c == -10.0
    assert (config.battery_under_temp_c, config.battery_under_temp_clear_c) == (-5.0, -2.0)
    assert (config.battery_over_temp_c, config.battery_over_temp_clear_c) == (45.0, 42.0)
    assert (config.electronics_under_temp_c, config.electronics_under_temp_clear_c) == (
        -25.0,
        -22.0,
    )
    assert (config.electronics_over_temp_c, config.electronics_over_temp_clear_c) == (
        60.0,
        57.0,
    )


def test_default_hysteresis_bands_exceed_the_noise_spread() -> None:
    # The noise is bounded at 6 sigma, so a steady temperature's readings span at most
    # 12 sigma (2.4 °C); every band is wider, so a steady temperature cannot chatter.
    spread_c = 12 * CONFIG.temperature_noise_c
    for node in ("battery", "electronics"):
        under = getattr(CONFIG, f"{node}_under_temp_clear_c") - getattr(
            CONFIG, f"{node}_under_temp_c"
        )
        over = getattr(CONFIG, f"{node}_over_temp_c") - getattr(CONFIG, f"{node}_over_temp_clear_c")
        assert under > spread_c and over > spread_c


BATTERY_ORDER = (
    "battery_survival_limit_c",
    "battery_under_temp_c",
    "heater_on_setpoint_c",
    "heater_off_setpoint_c",
)
ORDER_VALUES = (-20.0, -10.0, 0.0, 4.0)


@pytest.mark.parametrize("values", list(itertools.permutations(ORDER_VALUES)))
def test_battery_threshold_order_is_validated(values: tuple[float, ...]) -> None:
    # Every assignment of four increasing values to survival limit, under_temp, heater
    # ON, heater OFF is rejected except the increasing one. The under-temperature clear
    # threshold follows the set threshold so that only the order is under test.
    kwargs: dict[str, Any] = dict(zip(BATTERY_ORDER, values, strict=True))
    kwargs["battery_under_temp_clear_c"] = kwargs["battery_under_temp_c"] + 1.0
    if values == ORDER_VALUES:
        ThermalConfig(**kwargs)
    else:
        with pytest.raises(ValueError, match="must be above"):
            ThermalConfig(**kwargs)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        # Equal neighbours in the battery order.
        ({"battery_survival_limit_c": -5.0}, "battery_under_temp_c must be above"),
        (
            {"battery_under_temp_c": 1.0, "battery_under_temp_clear_c": 2.0},
            "heater_on_setpoint_c must be above",
        ),
        ({"heater_off_setpoint_c": 0.0}, "heater_off_setpoint_c must be above"),
        # The 5 °C margin between under_temp and the heater ON setpoint.
        ({"battery_under_temp_c": -3.0}, "at least 5.0 °C below"),
        ({"battery_under_temp_c": -4.5, "heater_on_setpoint_c": 0.0}, "at least 5.0 °C"),
        (
            {"heater_on_setpoint_c": 1.0, "battery_under_temp_c": -3.5},
            "at least 5.0 °C",
        ),
        # Hysteresis pairs and bands, per node.
        ({"battery_under_temp_clear_c": -5.0}, "battery_under_temp_clear_c must be above"),
        ({"battery_over_temp_clear_c": 45.0}, "battery_over_temp_c must be above"),
        (
            {"battery_over_temp_clear_c": -2.0, "battery_over_temp_c": 45.0},
            "battery_over_temp_clear_c must be above",
        ),
        ({"electronics_under_temp_clear_c": -26.0}, "electronics_under_temp_clear_c"),
        ({"electronics_over_temp_c": 57.0}, "electronics_over_temp_c must be above"),
        (
            {"electronics_over_temp_clear_c": -22.0},
            "electronics_over_temp_clear_c must be above",
        ),
        # The heater cannot drive the battery into over-temperature.
        (
            {"battery_over_temp_clear_c": 4.0, "battery_over_temp_c": 10.0},
            "battery_over_temp_clear_c must be above heater_off_setpoint_c",
        ),
        # Ranges.
        ({"temperature_noise_c": -0.1}, "non-negative"),
        ({"temperature_noise_c": float("nan")}, "finite"),
        ({"battery_survival_limit_c": -300.0}, "absolute zero"),
        ({"electronics_over_temp_c": float("inf")}, "finite"),
    ],
)
def test_invalid_thresholds_rejected(kwargs: dict[str, float], match: str) -> None:
    with pytest.raises(ValueError, match=match):
        ThermalConfig(**kwargs)


@pytest.mark.parametrize("name", ["temperature_noise_c", "battery_over_temp_c"])
def test_threshold_type_checked(name: str) -> None:
    with pytest.raises(TypeError):
        ThermalConfig(**{name: True})
    with pytest.raises(TypeError):
        # Deliberately the wrong type: the test checks the runtime type check.
        ThermalConfig(**{name: "1"})  # type: ignore[arg-type]


def test_margin_of_exactly_five_degrees_is_valid() -> None:
    ThermalConfig(heater_on_setpoint_c=2.0, battery_under_temp_c=-3.0)
    ThermalConfig(temperature_noise_c=0.0)


# --- Flags and hysteresis, per node -----------------------------------------------------


def _under_sequence(set_c: float, clear_c: float) -> list[tuple[float, bool]]:
    return [
        (clear_c + 1.5, False),
        (set_c + 0.5, False),
        (set_c, False),  # strictly below sets
        (set_c - 0.5, True),
        (set_c + 0.5, True),  # inside the band: holds
        (clear_c, True),  # strictly above clears
        (clear_c + 0.5, False),
        (set_c + 0.5, False),  # inside the band: holds
        (set_c - 0.5, True),
    ]


def _over_sequence(set_c: float, clear_c: float) -> list[tuple[float, bool]]:
    return [
        (clear_c - 1.5, False),
        (set_c - 0.5, False),
        (set_c, False),  # strictly above sets
        (set_c + 0.5, True),
        (set_c - 0.5, True),
        (clear_c, True),  # strictly below clears
        (clear_c - 0.5, False),
        (set_c - 0.5, False),
        (set_c + 0.5, True),
    ]


@pytest.mark.parametrize("node", ["battery", "electronics"])
@pytest.mark.parametrize("kind", ["under", "over"])
def test_flag_thresholds_and_hysteresis_per_node(node: str, kind: str) -> None:
    wide = WIDE_ELECTRONICS if node == "battery" else WIDE_BATTERY
    config = dataclasses.replace(EXACT, **wide)
    set_c = getattr(config, f"{node}_{kind}_temp_c")
    clear_c = getattr(config, f"{node}_{kind}_temp_clear_c")
    sequence = (_under_sequence if kind == "under" else _over_sequence)(set_c, clear_c)
    thermal = _thermal(config, ThermalInitial(battery_c=20.0, electronics_c=20.0))
    for ambient_c, expected in sequence:
        readings = _drive(thermal, [ambient_c])[-1].readings
        assert readings.battery_c == ambient_c and readings.electronics_c == ambient_c
        flag, other = (
            (readings.under_temp, readings.over_temp)
            if kind == "under"
            else (readings.over_temp, readings.under_temp)
        )
        assert flag is expected, ambient_c
        assert other is False


def test_flags_use_each_nodes_own_thresholds() -> None:
    # -10 °C is below the battery's under-temperature threshold (-5) but well above
    # the electronics' (-25); 50 °C is above the battery's over-temperature threshold
    # (45) but below the electronics' (60).
    battery_only = _thermal(dataclasses.replace(EXACT, **WIDE_ELECTRONICS))
    electronics_only = _thermal(dataclasses.replace(EXACT, **WIDE_BATTERY))
    assert _drive(battery_only, [-10.0])[-1].readings.under_temp
    assert not _drive(electronics_only, [-10.0])[-1].readings.under_temp
    assert _drive(battery_only, [50.0])[-1].readings.over_temp
    assert not _drive(electronics_only, [50.0])[-1].readings.over_temp


def test_flags_are_or_ed_across_nodes() -> None:
    # HALF: the battery lands on the ambient, the electronics move halfway.
    thermal = _thermal(HALF, ThermalInitial(battery_c=-6.0, electronics_c=-60.0))
    assert thermal.snapshot().readings.under_temp  # both nodes under at the start
    first, second = _drive(thermal, [0.0, 0.0])
    # Battery back at 0 (cleared, above -2); electronics at -30, still under: set.
    assert (first.readings.battery_c, first.readings.electronics_c) == (0.0, -30.0)
    assert first.readings.under_temp
    # Electronics at -15, above -22: both cleared.
    assert second.readings.electronics_c == -15.0
    assert not second.readings.under_temp

    thermal = _thermal(HALF, ThermalInitial(battery_c=20.0, electronics_c=20.0))
    hot, cooler = _drive(thermal, [50.0, 30.0])
    # Battery 50 (over 45), electronics 35: set by the battery alone.
    assert (hot.readings.battery_c, hot.readings.electronics_c) == (50.0, 35.0)
    assert hot.readings.over_temp
    # Battery 30 (below 42, cleared), electronics 32.5: cleared.
    assert not cooler.readings.over_temp

    # Both flags at once: a cold battery and hot electronics.
    thermal = _thermal(HALF, ThermalInitial(battery_c=-6.0, electronics_c=100.0))
    readings = thermal.snapshot().readings
    assert readings.under_temp and readings.over_temp
    readings = _drive(thermal, [20.0])[-1].readings  # battery 20, electronics 60
    assert not readings.under_temp and readings.over_temp  # 60 is not below 57
    readings = _drive(thermal, [20.0])[-1].readings  # electronics 40
    assert not readings.under_temp and not readings.over_temp


@pytest.mark.parametrize(
    ("battery_c", "electronics_c", "under", "over"),
    [
        (20.0, 20.0, False, False),
        (-5.0, -25.0, False, False),  # at the set thresholds: not beyond
        (-5.5, 20.0, True, False),
        (20.0, -25.5, True, False),
        (45.5, 20.0, False, True),
        (20.0, 60.5, False, True),
    ],
)
def test_flags_after_reset_follow_the_starting_temperatures(
    battery_c: float, electronics_c: float, under: bool, over: bool
) -> None:
    initial = ThermalInitial(battery_c=battery_c, electronics_c=electronics_c)
    thermal = _thermal(CONFIG, initial)
    assert thermal.snapshot().readings == ThermalReadings(
        battery_c=battery_c, electronics_c=electronics_c, over_temp=over, under_temp=under
    )


def _expected_conditions(
    config: ThermalConfig, state: dict[str, bool], readings: ThermalReadings
) -> tuple[bool, bool]:
    """The documented rule, applied to the reported values: per node and kind, set
    beyond the set threshold, clear beyond the clear threshold, otherwise hold; the
    flags OR the nodes."""
    for node, value in (("battery", readings.battery_c), ("electronics", readings.electronics_c)):
        under_set = getattr(config, f"{node}_under_temp_c")
        under_clear = getattr(config, f"{node}_under_temp_clear_c")
        over_set = getattr(config, f"{node}_over_temp_c")
        over_clear = getattr(config, f"{node}_over_temp_clear_c")
        key = f"{node}_under"
        state[key] = value <= under_clear if state[key] else value < under_set
        key = f"{node}_over"
        state[key] = value >= over_clear if state[key] else value > over_set
    return (
        state["battery_over"] or state["electronics_over"],
        state["battery_under"] or state["electronics_under"],
    )


def test_flags_follow_the_rule_on_noisy_reported_values() -> None:
    # Heavy noise (scale 10, sigma 2 °C) on temperatures sweeping across every
    # threshold: the flags must follow the reported values, not the truth.
    thermal = _thermal(HALF, ThermalInitial(battery_c=20.0, electronics_c=20.0), seed=5)
    state = dict.fromkeys(
        ("battery_under", "battery_over", "electronics_under", "electronics_over"), False
    )
    sweep = [*range(20, 80, 2), *range(80, -60, -2), *range(-60, 20, 2)] * 3
    differs_from_truth = 0
    seen = set()
    for snap in _drive(thermal, [float(a) for a in sweep], scale=10.0):
        readings = snap.readings
        expected = _expected_conditions(HALF, state, readings)
        assert (readings.over_temp, readings.under_temp) == expected
        seen.add(expected)
        differs_from_truth += readings.battery_c != snap.truth.battery_c
    assert seen == {(False, False), (True, False), (False, True)}
    assert differs_from_truth > 0


def test_steady_temperature_at_a_threshold_does_not_chatter() -> None:
    # Truth held exactly at a set threshold with nominal noise: the flag sets once and,
    # because the band is wider than the noise, never clears.
    for config, ambient_c, flag in (
        (dataclasses.replace(EXACT, **WIDE_ELECTRONICS), -5.0, "under_temp"),
        (dataclasses.replace(EXACT, **WIDE_ELECTRONICS), 45.0, "over_temp"),
        (dataclasses.replace(EXACT, **WIDE_BATTERY), -25.0, "under_temp"),
        (dataclasses.replace(EXACT, **WIDE_BATTERY), 60.0, "over_temp"),
    ):
        thermal = _thermal(config, ThermalInitial(battery_c=ambient_c, electronics_c=ambient_c))
        values = [getattr(s.readings, flag) for s in _drive(thermal, [ambient_c] * 2000, scale=1.0)]
        changes = sum(1 for a, b in itertools.pairwise([False, *values]) if a != b)
        assert changes == 1, (ambient_c, flag)


# --- Noise ------------------------------------------------------------------------------


def test_requests_the_noise_stream() -> None:
    rng = RngFactory(7)
    Thermal(CONFIG, ThermalInitial()).reset(rng)
    assert NOISE_STREAM == "spacecraft.thermal.noise"
    assert list(rng.stream_seeds) == [NOISE_STREAM]


def _noisy_run(
    seed: int, scale: float, ticks: int = 2000, freeze: range = range(0)
) -> list[ThermalSnapshot]:
    thermal = _thermal(CONFIG, ThermalInitial(battery_c=30.0, electronics_c=10.0), seed=seed)
    out = []
    for tick in range(ticks):
        controls = FROZEN if tick in freeze else CONTROLS
        thermal.step(SECOND_US, _env(20.0, scale), controls)
        out.append(thermal.snapshot())
    return out


def test_noise_is_portable_normal_on_its_stream() -> None:
    run = _noisy_run(seed=11, scale=1.0, ticks=200)
    rng = RngFactory(11).stream(NOISE_STREAM)
    sigma = CONFIG.temperature_noise_c
    for snap in run:
        expected_b = snap.truth.battery_c + portable_normal(rng, 0.0, sigma)
        expected_e = snap.truth.electronics_c + portable_normal(rng, 0.0, sigma)
        assert snap.readings.battery_c == expected_b
        assert snap.readings.electronics_c == expected_e


def test_noise_reproducible_for_a_seed() -> None:
    first = _noisy_run(seed=42, scale=1.0)
    assert first == _noisy_run(seed=42, scale=1.0)
    other = _noisy_run(seed=43, scale=1.0)
    assert [s.truth for s in other] == [s.truth for s in first]
    assert [s.readings for s in other] != [s.readings for s in first]


def test_noise_scale_zero_gives_exact_values() -> None:
    for snap in _noisy_run(seed=3, scale=0.0):
        assert snap.readings.battery_c == snap.truth.battery_c
        assert snap.readings.electronics_c == snap.truth.electronics_c


def _residuals(run: list[ThermalSnapshot]) -> list[float]:
    return [s.readings.battery_c - s.truth.battery_c for s in run] + [
        s.readings.electronics_c - s.truth.electronics_c for s in run
    ]


def test_noise_scale_two_doubles_the_noise() -> None:
    nominal = _residuals(_noisy_run(seed=8, scale=1.0))
    doubled = _residuals(_noisy_run(seed=8, scale=2.0))
    # Same draws, twice the standard deviation.
    assert doubled == pytest.approx([2.0 * r for r in nominal], rel=1e-9, abs=1e-12)
    sigma = 2.0 * CONFIG.temperature_noise_c
    assert statistics.pstdev(doubled) == pytest.approx(sigma, rel=0.1)
    assert abs(statistics.fmean(doubled)) < 0.1 * sigma
    assert max(abs(r) for r in doubled) <= 6 * sigma


# --- sensor_freeze -------------------------------------------------------------------------


def test_freeze_holds_readings_exactly_while_truth_changes() -> None:
    frozen = range(500, 1500)
    run = _noisy_run(seed=9, scale=1.0, freeze=frozen)
    held = run[frozen.start - 1].readings
    for tick in frozen:
        assert run[tick].readings is held  # the same record: exact, and no allocation
    truths = [run[tick].truth for tick in frozen]
    assert truths[0].battery_c != truths[-1].battery_c
    assert truths[0].electronics_c != truths[-1].electronics_c
    # Truth is unaffected by the freeze.
    live = _noisy_run(seed=9, scale=1.0)
    assert [s.truth for s in run] == [s.truth for s in live]
    # Live readings resume on release, with the same noise as if never frozen: the
    # noise is drawn every tick.
    assert run[frozen.stop].readings != held
    assert [s.readings for s in run[frozen.stop :]] == [s.readings for s in live[frozen.stop :]]


def test_freeze_from_the_first_tick_holds_the_reset_readings() -> None:
    thermal = _thermal(CONFIG, ThermalInitial(battery_c=30.0, electronics_c=10.0))
    start = thermal.snapshot().readings
    for snap in _drive(thermal, [20.0] * 50, FROZEN, scale=1.0, dt_us=SECOND_US):
        assert snap.readings == ThermalReadings(
            battery_c=30.0, electronics_c=10.0, over_temp=False, under_temp=False
        )
        assert snap.readings is start


@pytest.mark.parametrize(
    ("config", "cold_c", "flag"),
    [
        (dataclasses.replace(EXACT, **WIDE_ELECTRONICS), -10.0, "under_temp"),
        (dataclasses.replace(EXACT, **WIDE_ELECTRONICS), 50.0, "over_temp"),
        (dataclasses.replace(EXACT, **WIDE_BATTERY), -30.0, "under_temp"),
        (dataclasses.replace(EXACT, **WIDE_BATTERY), 65.0, "over_temp"),
    ],
)
def test_frozen_reading_hides_a_real_threshold_crossing(
    config: ThermalConfig, cold_c: float, flag: str
) -> None:
    thermal = _thermal(config, ThermalInitial(battery_c=20.0, electronics_c=20.0))
    before = _drive(thermal, [20.0])[-1].readings
    # Truth crosses the threshold while frozen: the flag does not see it.
    for snap in _drive(thermal, [cold_c] * 3, FROZEN):
        assert snap.truth.battery_c == cold_c and snap.truth.electronics_c == cold_c
        assert snap.readings == before
        assert not getattr(snap.readings, flag)
    # On release the live reading sets the flag.
    after = _drive(thermal, [cold_c])[-1].readings
    assert after.battery_c == cold_c
    assert getattr(after, flag)
    # And a recovery during a freeze is hidden too: the flag stays set until release.
    for snap in _drive(thermal, [20.0] * 3, FROZEN):
        assert getattr(snap.readings, flag)
    assert not getattr(_drive(thermal, [20.0])[-1].readings, flag)


def test_other_frozen_sensors_do_not_freeze_thermal() -> None:
    controls = SpacecraftControls(frozen_sensors=frozenset({"power", "attitude"}))
    thermal = _thermal(dataclasses.replace(EXACT, **WIDE_ELECTRONICS))
    assert _drive(thermal, [-10.0], controls)[-1].readings.under_temp


def test_heater_reads_true_temperature_not_the_readings() -> None:
    # A frozen warm reading does not stop the thermostat from switching on.
    thermal = _thermal(CONFIG, ThermalInitial(battery_c=10.0, electronics_c=10.0))
    snaps = _drive(thermal, [-20.0] * 600, FROZEN, scale=1.0, dt_us=SECOND_US)
    assert snaps[-1].readings.battery_c == 10.0
    assert any(s.truth.heater_on for s in snaps)
