"""Tests for the power subsystem: battery and solar model (#35), loads, sensors, and
flags (#36)."""

import dataclasses
import math
from collections.abc import Callable
from typing import Any

import pytest

from pocketsat.core.portable import portable_cos_deg
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import (
    NOMINAL_CONFIG,
    AttitudeSnapshot,
    Power,
    PowerConfig,
    PowerInitial,
    PowerSnapshot,
    PowerTruth,
    SnapshotBoard,
    SpacecraftControls,
    Subsystem,
    SubsystemStack,
)
from pocketsat.spacecraft.fakes import (
    FakeSubsystem,
    default_snapshot,
    fake_subsystems,
    replace_truth,
)
from pocketsat.targets.base import EnvironmentState

TICK_US = 100_000
SUN = EnvironmentState(sunlit=True)
ECLIPSE = EnvironmentState(sunlit=False)
CONTROLS = SpacecraftControls()
CONFIG = PowerConfig()
TICKS_PER_HOUR = 36_000


def _power(soc: float = 0.5, config: PowerConfig = CONFIG) -> Power:
    power = Power(config, PowerInitial(soc=soc))
    power.reset(RngFactory(1))
    return power


def _truth(power: Power) -> PowerTruth:
    return power.snapshot().truth


def _run(
    power: Power,
    ticks: int,
    env: EnvironmentState = SUN,
    controls: SpacecraftControls = CONTROLS,
) -> None:
    for _ in range(ticks):
        power.step(TICK_US, env, controls)


def _attitude(pointing_error_deg: float) -> AttitudeSnapshot:
    snap = replace_truth(default_snapshot("attitude"), pointing_error_deg=pointing_error_deg)
    assert isinstance(snap, AttitudeSnapshot)
    return snap


def _others(**overrides: FakeSubsystem[Any]) -> list[FakeSubsystem[Any]]:
    """Fakes of the four subsystems power reads, holding the defaults unless overridden."""
    return [f for f in fake_subsystems(overrides) if f.name != "power"]


def _stack(
    fakes: list[FakeSubsystem[Any]],
    soc: float = 0.5,
    config: PowerConfig = CONFIG,
    seed: int = 1,
) -> tuple[SubsystemStack, Power]:
    board = SnapshotBoard()
    power = Power(config, PowerInitial(soc=soc), reader=board)
    stack = SubsystemStack([power, *fakes], board=board)
    stack.reset(RngFactory(seed))
    return stack, power


def _stack_with_attitude(
    attitude: FakeSubsystem[AttitudeSnapshot], soc: float = 0.5
) -> tuple[SubsystemStack, Power]:
    return _stack(_others(attitude=attitude), soc=soc)


# --- Construction, configuration, reset ------------------------------------------------


def test_satisfies_subsystem_protocol() -> None:
    power = Power(CONFIG, PowerInitial())
    assert isinstance(power, Subsystem)
    assert power.name == "power"


def test_config_defaults_are_documented_values() -> None:
    assert NOMINAL_CONFIG.power == PowerConfig(
        battery_capacity_wh=20.0,
        solar_array_w=8.0,
        base_load_w=2.0,
        battery_empty_v=6.0,
        battery_full_v=8.4,
        voltage_noise_v=0.008,
        current_noise_a=0.01,
        low_battery_soc=0.30,
        low_battery_clear_soc=0.35,
        critical_battery_soc=0.15,
        critical_battery_clear_soc=0.20,
    )


@pytest.mark.parametrize(
    ("build", "error"),
    [
        (lambda: PowerConfig(battery_capacity_wh=0.0), ValueError),
        (lambda: PowerConfig(battery_capacity_wh=float("inf")), ValueError),
        (lambda: PowerConfig(solar_array_w=-1.0), ValueError),
        (lambda: PowerConfig(base_load_w=-0.1), ValueError),
        (lambda: PowerConfig(base_load_w=float("nan")), ValueError),
        (lambda: PowerConfig(battery_empty_v=0.0), ValueError),
        (lambda: PowerConfig(battery_full_v=6.0), ValueError),
        (lambda: PowerConfig(solar_array_w=True), TypeError),
        (lambda: PowerConfig(battery_capacity_wh="20"), TypeError),  # type: ignore[arg-type]
        (lambda: PowerConfig(voltage_noise_v=-0.001), ValueError),
        (lambda: PowerConfig(current_noise_a=float("inf")), ValueError),
        (lambda: PowerConfig(low_battery_soc=-0.1), ValueError),
        (lambda: PowerConfig(low_battery_clear_soc=1.1), ValueError),
        (lambda: PowerConfig(low_battery_clear_soc=0.30), ValueError),
        (lambda: PowerConfig(critical_battery_clear_soc=0.15), ValueError),
        (lambda: PowerConfig(critical_battery_soc=0.31), ValueError),
        (lambda: PowerConfig(critical_battery_clear_soc=0.36), ValueError),
        (lambda: PowerConfig(low_battery_soc=True), TypeError),
    ],
)
def test_invalid_config_rejected(build: Callable[[], object], error: type[Exception]) -> None:
    with pytest.raises(error):
        build()


def test_zero_array_and_load_are_valid() -> None:
    PowerConfig(solar_array_w=0.0, base_load_w=0.0)


def test_constructor_checks_record_types() -> None:
    with pytest.raises(TypeError):
        Power(PowerInitial(), PowerInitial())  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        Power(CONFIG, CONFIG)  # type: ignore[arg-type]


def test_starts_from_starting_record_with_nothing_flowing() -> None:
    truth = _truth(_power(soc=0.25))
    assert truth.soc == 0.25
    assert truth.bus_v == pytest.approx(6.0 + 0.25 * 2.4)
    assert truth.generation_w == 0.0
    assert truth.total_load_w == 0.0
    assert truth.battery_current_a == 0.0


def test_reset_returns_to_starting_state() -> None:
    power = _power(soc=0.5)
    before = power.snapshot()
    _run(power, 1000, ECLIPSE)
    assert _truth(power).soc < 0.5
    power.reset(RngFactory(1))
    assert power.snapshot() == before


def test_bus_voltage_is_linear_in_soc() -> None:
    for soc in (0.0, 0.5, 1.0):
        assert _truth(_power(soc=soc)).bus_v == pytest.approx(6.0 + soc * 2.4)


@pytest.mark.parametrize("dt_us", [-1, 1.5, True])
def test_step_rejects_bad_dt(dt_us: object) -> None:
    with pytest.raises((TypeError, ValueError)):
        _power().step(dt_us, SUN, CONTROLS)  # type: ignore[arg-type]


# --- Charge and discharge --------------------------------------------------------------


def test_discharges_through_eclipse() -> None:
    power = _power(soc=0.8)
    previous = 0.8
    for _ in range(100):
        power.step(TICK_US, ECLIPSE, CONTROLS)
        truth = _truth(power)
        assert truth.soc < previous
        assert truth.generation_w == 0.0
        assert truth.total_load_w == 2.0
        previous = truth.soc
    # One hour of eclipse at a 2 W base load drains 2 Wh of 20 Wh: SOC falls by 0.1.
    _run(power, TICKS_PER_HOUR - 100, ECLIPSE)
    assert _truth(power).soc == pytest.approx(0.7, abs=1e-9)


def test_charges_in_sunlight() -> None:
    power = _power(soc=0.5)
    _run(power, TICKS_PER_HOUR)
    truth = _truth(power)
    assert truth.generation_w == 8.0
    # Net 6 W for one hour is 6 Wh of 20 Wh: SOC rises by 0.3.
    assert truth.soc == pytest.approx(0.8, abs=1e-9)
    assert truth.net_power_w == 6.0


def test_integration_uses_integer_time() -> None:
    fine = _power(soc=0.5)
    _run(fine, 10, ECLIPSE)
    coarse = _power(soc=0.5)
    coarse.step(1_000_000, ECLIPSE, CONTROLS)
    assert _truth(fine).soc == pytest.approx(_truth(coarse).soc, abs=1e-12)
    idle = _power(soc=0.5)
    idle.step(0, ECLIPSE, CONTROLS)
    assert _truth(idle).soc == 0.5


def test_soc_clamped_at_full() -> None:
    power = _power(soc=0.99)
    _run(power, TICKS_PER_HOUR)
    assert _truth(power).soc == 1.0
    assert _truth(power).bus_v == pytest.approx(8.4)


def test_soc_clamped_at_empty() -> None:
    power = _power(soc=0.01)
    _run(power, TICKS_PER_HOUR, ECLIPSE, SpacecraftControls(extra_load_w=10.0))
    assert _truth(power).soc == 0.0
    assert _truth(power).bus_v == pytest.approx(6.0)


def test_soc_stays_in_range_under_mixed_conditions() -> None:
    power = _power(soc=0.5)
    drain = SpacecraftControls(extra_load_w=30.0)
    for i in range(20_000):
        env = SUN if (i // 3000) % 2 == 0 else ECLIPSE
        power.step(TICK_US * 10, env, drain if (i // 7000) % 2 else CONTROLS)
        assert 0.0 <= _truth(power).soc <= 1.0


def test_battery_current_positive_while_charging() -> None:
    power = _power(soc=0.5)
    power.step(TICK_US, SUN, CONTROLS)
    truth = _truth(power)
    assert truth.net_power_w > 0
    assert truth.battery_current_a > 0
    assert truth.battery_current_a == pytest.approx(truth.net_power_w / truth.bus_v)


def test_battery_current_negative_while_discharging() -> None:
    power = _power(soc=0.5)
    power.step(TICK_US, ECLIPSE, CONTROLS)
    truth = _truth(power)
    assert truth.net_power_w < 0
    assert truth.battery_current_a < 0
    assert truth.battery_current_a == pytest.approx(-2.0 / truth.bus_v)
    assert truth.generation_w >= 0
    assert truth.total_load_w >= 0


def test_extra_load_adds_to_total_load() -> None:
    power = _power()
    power.step(TICK_US, SUN, SpacecraftControls(extra_load_w=3.5))
    assert _truth(power).total_load_w == 5.5


def test_deterministic_over_10000_ticks() -> None:
    def run() -> list[PowerSnapshot]:
        attitude = FakeSubsystem(
            "attitude",
            _attitude(0.0),
            script=lambda tick, prev: _attitude(float(tick % 120)),
        )
        stack, power = _stack_with_attitude(attitude, soc=0.6)
        out = []
        for i in range(10_000):
            env = EnvironmentState(sunlit=(i // 600) % 3 != 2)
            controls = SpacecraftControls(extra_load_w=1.5 if (i // 1000) % 2 else 0.0)
            stack.step(TICK_US, env, controls)
            out.append(power.snapshot())
        return out

    first = run()
    assert first == run()
    assert len({s.truth.soc for s in first}) > 1000  # it actually moved


# --- Pointing factor -------------------------------------------------------------------


@pytest.mark.parametrize("pointing_error_deg", [0.0, 45.0, 90.0, 120.0, 180.0])
def test_pointing_factor(pointing_error_deg: float) -> None:
    attitude = FakeSubsystem("attitude", _attitude(pointing_error_deg))
    stack, power = _stack_with_attitude(attitude)
    stack.step(TICK_US, SUN, CONTROLS)
    generation_w = _truth(power).generation_w
    assert generation_w == 8.0 * portable_cos_deg(pointing_error_deg)
    expected = 8.0 * max(0.0, math.cos(math.radians(pointing_error_deg)))
    assert generation_w == pytest.approx(expected, abs=8.0 * 0.002)
    if pointing_error_deg == 0.0:
        assert generation_w == 8.0
    if pointing_error_deg >= 90.0:
        assert generation_w == 0.0


def test_pointing_beyond_90_degrees_discharges_in_sunlight() -> None:
    attitude = FakeSubsystem("attitude", _attitude(135.0))
    stack, power = _stack_with_attitude(attitude)
    stack.step(TICK_US, SUN, CONTROLS)
    assert _truth(power).generation_w == 0.0
    assert _truth(power).battery_current_a < 0


def test_reads_previous_tick_pointing() -> None:
    # Attitude steps after power, so power sees the pointing attitude published in
    # the previous tick (#85).
    attitude = FakeSubsystem("attitude", _attitude(0.0), script={1: _attitude(90.0)})
    stack, power = _stack_with_attitude(attitude)
    stack.step(TICK_US, SUN, CONTROLS)  # tick 0: attitude at 0°
    assert _truth(power).generation_w == 8.0
    stack.step(TICK_US, SUN, CONTROLS)  # tick 1: attitude moves to 90° after power
    assert _truth(power).generation_w == 8.0
    stack.step(TICK_US, SUN, CONTROLS)  # tick 2: power sees 90°
    assert _truth(power).generation_w == 0.0


def test_reads_true_pointing_not_readings() -> None:
    snap = dataclasses.replace(
        _attitude(0.0),
        readings=dataclasses.replace(_attitude(0.0).readings, pointing_error_deg=90.0),
    )
    stack, power = _stack_with_attitude(FakeSubsystem("attitude", snap))
    stack.step(TICK_US, SUN, CONTROLS)
    assert _truth(power).generation_w == 8.0


def test_no_generation_in_eclipse_whatever_the_pointing() -> None:
    stack, power = _stack_with_attitude(FakeSubsystem("attitude", _attitude(0.0)))
    stack.step(TICK_US, ECLIPSE, CONTROLS)
    assert _truth(power).generation_w == 0.0


def test_without_reader_pointing_is_ideal() -> None:
    power = _power()
    power.step(TICK_US, SUN, CONTROLS)
    assert _truth(power).generation_w == 8.0


@pytest.mark.parametrize("missing", ["thermal", "attitude", "payload", "comms"])
def test_reader_missing_a_subsystem_raises(missing: str) -> None:
    fakes = [f for f in _others() if f.name != missing]
    stack, _ = _stack(fakes)
    with pytest.raises(KeyError, match=missing):
        stack.step(TICK_US, SUN, CONTROLS)


# --- battery_soc_override --------------------------------------------------------------


def test_override_applied() -> None:
    power = _power(soc=0.8)
    power.step(TICK_US, EnvironmentState(battery_soc_override=0.1), CONTROLS)
    truth = _truth(power)
    assert truth.soc == 0.1
    assert truth.bus_v == pytest.approx(6.0 + 0.1 * 2.4)


def test_override_held_across_ticks() -> None:
    power = _power(soc=0.8)
    for env in (
        EnvironmentState(sunlit=True, battery_soc_override=0.3),
        EnvironmentState(sunlit=False, battery_soc_override=0.3),
    ):
        for _ in range(500):
            power.step(TICK_US, env, CONTROLS)
            assert _truth(power).soc == 0.3


def test_override_cleared_resumes_from_pinned_value() -> None:
    power = _power(soc=0.8)
    _run(power, 10, EnvironmentState(sunlit=False, battery_soc_override=0.3))
    power.step(TICK_US, ECLIPSE, CONTROLS)
    soc = _truth(power).soc
    assert soc < 0.3
    assert soc == pytest.approx(0.3 - 2.0 * 0.1 / 3600.0 / 20.0)


def test_override_beats_extra_load() -> None:
    power = _power(soc=0.8)
    drain = SpacecraftControls(extra_load_w=10.0)
    pinned = EnvironmentState(sunlit=False, battery_soc_override=0.6, sensor_noise_scale=0.0)
    for _ in range(100):
        power.step(TICK_US, pinned, drain)
        truth = _truth(power)
        assert truth.soc == 0.6
        assert truth.bus_v == pytest.approx(6.0 + 0.6 * 2.4)
        # The drain's current is still reported.
        assert truth.total_load_w == 12.0
        assert truth.battery_current_a == pytest.approx(-12.0 / truth.bus_v)
        assert power.snapshot().readings.battery_current_a == truth.battery_current_a
    # After the override clears, SOC falls from the pinned value with the drain active.
    power.step(TICK_US, ECLIPSE, drain)
    assert _truth(power).soc == pytest.approx(0.6 - 12.0 * 0.1 / 3600.0 / 20.0)


def test_starting_charge_is_not_an_override() -> None:
    power = _power(soc=0.6)
    power.step(TICK_US, ECLIPSE, CONTROLS)
    assert _truth(power).soc < 0.6


# --- Snapshot --------------------------------------------------------------------------


def test_snapshot_is_built_once_per_step() -> None:
    power = _power()
    power.step(TICK_US, SUN, CONTROLS)
    assert power.snapshot() is power.snapshot()
    first = power.snapshot()
    power.step(TICK_US, SUN, CONTROLS)
    assert power.snapshot() is not first
