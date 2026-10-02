"""Tests for the lumped thermal model and the battery survival heater (#38).

The sensors and limit flags (#39) are tested in ``test_thermal_flags_noise.py``.
"""

import dataclasses
import itertools
from collections.abc import Callable
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    AttitudeControls,
    PayloadControls,
    Power,
    PowerSnapshot,
    RadioControls,
    RadioMode,
    SnapshotBoard,
    SpacecraftControls,
    Subsystem,
    SubsystemStack,
    Thermal,
    ThermalConfig,
    ThermalInitial,
    ThermalReadings,
    ThermalSnapshot,
    ThermalTruth,
)
from pocketsat.spacecraft.fakes import (
    FakeSubsystem,
    default_snapshot,
    fake_subsystems,
    replace_truth,
)
from pocketsat.targets.base import EnvironmentState

TICK_US = 100_000
SECOND_US = 1_000_000
CONFIG = ThermalConfig()
CONTROLS = SpacecraftControls()
WARM = EnvironmentState(ambient_temp_c=20.0)
COLD = EnvironmentState(sunlit=False, ambient_temp_c=-20.0)
ON_C = CONFIG.heater_on_setpoint_c
OFF_C = CONFIG.heater_off_setpoint_c

HOLD_MARGIN_C = 2.0
"""The cold-ambient test asserts the battery stays at or above ``ON_C - HOLD_MARGIN_C``,
well above the battery's under-temperature threshold (#39, at least 5 °C below ON)."""


def _power_fake(
    board: SnapshotBoard,
    load_w: float,
    script: dict[int, float] | None = None,
    draw_heater: bool = True,
) -> FakeSubsystem[Any]:
    """A fake power whose total load is ``load_w`` (changing per ``script``), plus,
    like the real power subsystem, the heater draw thermal published last tick."""
    loads = dict(script or {})
    current = [load_w]

    def step(tick: int, previous: PowerSnapshot) -> PowerSnapshot:
        current[0] = loads.get(tick, current[0])
        heater_w = board.get("thermal", ThermalSnapshot).truth.heater_power_w
        return replace_truth(previous, total_load_w=current[0] + (heater_w if draw_heater else 0.0))

    initial = replace_truth(default_snapshot("power"), total_load_w=load_w)
    return FakeSubsystem("power", initial, script=step)


def _stack(
    load_w: float,
    initial: ThermalInitial = DEFAULT_INITIAL_STATE.thermal,
    config: ThermalConfig = CONFIG,
    script: dict[int, float] | None = None,
    draw_heater: bool = True,
) -> tuple[SubsystemStack, Thermal]:
    """Thermal with a scripted fake power on a shared board."""
    board = SnapshotBoard()
    thermal = Thermal(config, initial, reader=board)
    power = _power_fake(board, load_w, script, draw_heater)
    stack = SubsystemStack([power, thermal], board=board)
    stack.reset(RngFactory(1))
    return stack, thermal


def _run(
    stack: SubsystemStack,
    thermal: Thermal,
    ticks: int,
    env: EnvironmentState = WARM,
    dt_us: int = TICK_US,
) -> list[ThermalTruth]:
    out = []
    for _ in range(ticks):
        stack.step(dt_us, env, CONTROLS)
        out.append(thermal.snapshot().truth)
    return out


def _steady_state(
    ambient_c: float, load_w: float, heater_w: float = 0.0, c: ThermalConfig = CONFIG
) -> tuple[float, float]:
    """Analytic steady state of the two-node model (a 2x2 linear solve)."""
    f = c.battery_dissipation_fraction
    a11 = c.battery_conductance_w_per_c + c.coupling_conductance_w_per_c
    a22 = c.electronics_conductance_w_per_c + c.coupling_conductance_w_per_c
    a12 = -c.coupling_conductance_w_per_c
    b1 = f * load_w + heater_w + c.battery_conductance_w_per_c * ambient_c
    b2 = (1 - f) * load_w + c.electronics_conductance_w_per_c * ambient_c
    det = a11 * a22 - a12 * a12
    return (b1 * a22 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det


# --- Construction, configuration, reset ------------------------------------------------


def test_satisfies_subsystem_protocol() -> None:
    thermal = Thermal(CONFIG, ThermalInitial())
    assert isinstance(thermal, Subsystem)
    assert thermal.name == "thermal"


def test_config_defaults_are_documented_values() -> None:
    assert NOMINAL_CONFIG.thermal == ThermalConfig(
        battery_heat_capacity_j_per_c=80.0,
        electronics_heat_capacity_j_per_c=300.0,
        battery_conductance_w_per_c=0.12,
        electronics_conductance_w_per_c=0.25,
        coupling_conductance_w_per_c=0.04,
        battery_dissipation_fraction=0.25,
        heater_power_w=3.0,
        heater_on_setpoint_c=1.0,
        heater_off_setpoint_c=5.0,
    )
    assert DEFAULT_INITIAL_STATE.thermal == ThermalInitial(battery_c=20.0, electronics_c=20.0)


@pytest.mark.parametrize(
    ("build", "error"),
    [
        (lambda: ThermalConfig(battery_heat_capacity_j_per_c=0.0), ValueError),
        (lambda: ThermalConfig(electronics_heat_capacity_j_per_c=-1.0), ValueError),
        (lambda: ThermalConfig(battery_conductance_w_per_c=0.0), ValueError),
        (lambda: ThermalConfig(electronics_conductance_w_per_c=0.0), ValueError),
        (lambda: ThermalConfig(coupling_conductance_w_per_c=-0.01), ValueError),
        (lambda: ThermalConfig(battery_dissipation_fraction=-0.1), ValueError),
        (lambda: ThermalConfig(battery_dissipation_fraction=1.1), ValueError),
        (lambda: ThermalConfig(heater_power_w=-1.0), ValueError),
        (lambda: ThermalConfig(heater_power_w=float("inf")), ValueError),
        (lambda: ThermalConfig(heater_on_setpoint_c=float("nan")), ValueError),
        (
            lambda: ThermalConfig(heater_on_setpoint_c=-300.0, heater_off_setpoint_c=-280.0),
            ValueError,
        ),
        (lambda: ThermalConfig(heater_on_setpoint_c=4.0, heater_off_setpoint_c=4.0), ValueError),
        (lambda: ThermalConfig(heater_on_setpoint_c=5.0, heater_off_setpoint_c=4.0), ValueError),
        (lambda: ThermalConfig(heater_power_w=True), TypeError),
        (lambda: ThermalConfig(heater_power_w="3"), TypeError),  # type: ignore[arg-type]
    ],
)
def test_invalid_config_rejected(build: Callable[[], object], error: type[Exception]) -> None:
    with pytest.raises(error):
        build()


def test_edge_values_are_valid() -> None:
    ThermalConfig(
        coupling_conductance_w_per_c=0.0,
        battery_dissipation_fraction=0.0,
        heater_power_w=0.0,
    )
    ThermalConfig(battery_dissipation_fraction=1)


def test_constructor_checks_record_types() -> None:
    with pytest.raises(TypeError):
        Thermal(ThermalInitial(), ThermalInitial())  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        Thermal(CONFIG, CONFIG)  # type: ignore[arg-type]


def test_starts_from_starting_record() -> None:
    thermal = Thermal(CONFIG, ThermalInitial(battery_c=12.5, electronics_c=30.0))
    snap = thermal.snapshot()
    assert snap.truth == ThermalTruth(
        battery_c=12.5, electronics_c=30.0, heater_on=False, heater_power_w=0.0
    )
    assert snap.readings == ThermalReadings(
        battery_c=12.5, electronics_c=30.0, over_temp=False, under_temp=False
    )


def test_heater_starts_on_below_on_setpoint() -> None:
    cold = Thermal(CONFIG, ThermalInitial(battery_c=ON_C - 0.5)).snapshot().truth
    assert cold.heater_on and cold.heater_power_w == CONFIG.heater_power_w
    # Between the setpoints at the start: off.
    mid = Thermal(CONFIG, ThermalInitial(battery_c=(ON_C + OFF_C) / 2)).snapshot().truth
    assert not mid.heater_on and mid.heater_power_w == 0.0


def test_reset_returns_to_starting_state() -> None:
    initial = ThermalInitial(battery_c=-5.0, electronics_c=10.0)
    stack, thermal = _stack(4.0, initial)
    before = thermal.snapshot()
    _run(stack, thermal, 3000, WARM)
    assert thermal.snapshot().truth != before.truth
    thermal.reset(RngFactory(99))
    assert thermal.snapshot() == before


def test_max_step_is_the_euler_monotonicity_limit() -> None:
    thermal = Thermal(CONFIG, ThermalInitial())
    # min(80 / (0.12 + 0.04), 300 / (0.25 + 0.04)) = 500 s
    assert thermal.max_step_us == 500 * SECOND_US


@pytest.mark.parametrize(
    ("dt_us", "error"),
    [(-1, ValueError), (1.0, TypeError), (True, TypeError), (500 * SECOND_US + 1, ValueError)],
)
def test_step_rejects_bad_dt(dt_us: Any, error: type[Exception]) -> None:
    thermal = Thermal(CONFIG, ThermalInitial())
    with pytest.raises(error):
        thermal.step(dt_us, WARM, CONTROLS)


def test_longest_step_is_accepted_and_monotone() -> None:
    thermal = Thermal(CONFIG, ThermalInitial(battery_c=40.0, electronics_c=40.0))
    thermal.reset(RngFactory(1))
    thermal.step(thermal.max_step_us, WARM, CONTROLS)
    truth = thermal.snapshot().truth
    assert 20.0 <= truth.battery_c <= 40.0
    assert 20.0 <= truth.electronics_c <= 40.0


def test_zero_step_changes_nothing() -> None:
    stack, thermal = _stack(4.0)
    before = thermal.snapshot().truth
    stack.step(0, WARM, CONTROLS)
    assert thermal.snapshot().truth == before


# --- Heating and cooling ------------------------------------------------------------------


def test_heats_under_load() -> None:
    stack, thermal = _stack(5.0)
    truths = _run(stack, thermal, 6000, WARM)  # 10 minutes
    battery = [t.battery_c for t in truths]
    electronics = [t.electronics_c for t in truths]
    assert battery[-1] > 21.0
    assert electronics[-1] > 23.0
    # Monotone warming from ambient under a constant load.
    assert all(b2 >= b1 for b1, b2 in itertools.pairwise(battery))
    assert all(e2 >= e1 for e1, e2 in itertools.pairwise(electronics))


def test_cools_at_idle() -> None:
    stack, thermal = _stack(0.0, ThermalInitial(battery_c=40.0, electronics_c=45.0))
    truths = _run(stack, thermal, 6000, WARM)
    battery = [t.battery_c for t in truths]
    electronics = [t.electronics_c for t in truths]
    assert battery[-1] < 32.0
    assert electronics[-1] < 40.0
    assert all(b2 <= b1 for b1, b2 in itertools.pairwise(battery))
    assert all(e2 <= e1 for e1, e2 in itertools.pairwise(electronics))
    assert min(battery) > 20.0 and min(electronics) > 20.0  # toward ambient, not past it


def test_without_reader_there_is_no_electrical_heating() -> None:
    thermal = Thermal(CONFIG, ThermalInitial(battery_c=20.0, electronics_c=20.0))
    thermal.reset(RngFactory(1))
    for _ in range(1000):
        thermal.step(TICK_US, WARM, CONTROLS)
    truth = thermal.snapshot().truth
    assert truth.battery_c == 20.0
    assert truth.electronics_c == 20.0


def test_reader_without_power_raises() -> None:
    thermal = Thermal(CONFIG, ThermalInitial(), reader=SnapshotBoard())
    thermal.reset(RngFactory(1))
    with pytest.raises(KeyError):
        thermal.step(TICK_US, WARM, CONTROLS)


def test_step_before_reset_raises() -> None:
    thermal = Thermal(CONFIG, ThermalInitial())
    with pytest.raises(RuntimeError):
        thermal.step(TICK_US, WARM, CONTROLS)


def test_reaches_a_stable_steady_state() -> None:
    stack, thermal = _stack(4.0)
    truths = _run(stack, thermal, 3000, WARM, dt_us=10 * SECOND_US)  # 500 minutes
    expected_b, expected_e = _steady_state(20.0, 4.0)
    last = truths[-1]
    assert last.battery_c == pytest.approx(expected_b, abs=1e-6)
    assert last.electronics_c == pytest.approx(expected_e, abs=1e-6)
    # Stable: it stays there.
    for truth in _run(stack, thermal, 100, WARM, dt_us=10 * SECOND_US):
        assert truth.battery_c == pytest.approx(expected_b, abs=1e-6)
        assert truth.electronics_c == pytest.approx(expected_e, abs=1e-6)
    assert not last.heater_on


def test_steady_state_tracks_a_change_in_ambient() -> None:
    stack, thermal = _stack(4.0)
    first = _run(stack, thermal, 3000, WARM, dt_us=10 * SECOND_US)[-1]
    hot = EnvironmentState(ambient_temp_c=30.0)
    second = _run(stack, thermal, 3000, hot, dt_us=10 * SECOND_US)[-1]
    assert second.battery_c - first.battery_c == pytest.approx(10.0, abs=1e-6)
    assert second.electronics_c - first.electronics_c == pytest.approx(10.0, abs=1e-6)
    assert (second.battery_c, second.electronics_c) == pytest.approx(
        _steady_state(30.0, 4.0), abs=1e-6
    )


def test_responds_in_the_same_tick_as_a_power_load_change() -> None:
    # Power's load steps from 2 W to 6 W at tick 10; thermal reads it in tick 10.
    stepped, stepped_thermal = _stack(2.0, script={10: 6.0})
    steady, steady_thermal = _stack(2.0)
    for tick in range(12):
        stepped.step(TICK_US, WARM, CONTROLS)
        steady.step(TICK_US, WARM, CONTROLS)
        a = stepped_thermal.snapshot().truth
        b = steady_thermal.snapshot().truth
        if tick < 10:
            assert a == b
        else:
            assert a.battery_c > b.battery_c
            assert a.electronics_c > b.electronics_c


def test_one_step_matches_the_documented_equations() -> None:
    # Heater on (cold start); power's total load includes the 3 W heater draw it read.
    initial = ThermalInitial(battery_c=-5.0, electronics_c=10.0)
    stack, thermal = _stack(4.0, initial)
    dt_s = 10.0
    stack.step(10 * SECOND_US, COLD, CONTROLS)
    truth = thermal.snapshot().truth
    c = CONFIG
    p, h, amb = 4.0, c.heater_power_w, -20.0
    q_be = c.coupling_conductance_w_per_c * (-5.0 - 10.0)
    battery_rate = (
        c.battery_dissipation_fraction * p + h - c.battery_conductance_w_per_c * (-5.0 - amb) - q_be
    ) / c.battery_heat_capacity_j_per_c
    electronics_rate = (
        (1 - c.battery_dissipation_fraction) * p
        - c.electronics_conductance_w_per_c * (10.0 - amb)
        + q_be
    ) / c.electronics_heat_capacity_j_per_c
    assert truth.battery_c == pytest.approx(-5.0 + battery_rate * dt_s, rel=1e-12)
    assert truth.electronics_c == pytest.approx(10.0 + electronics_rate * dt_s, rel=1e-12)


def test_dissipation_split_between_nodes() -> None:
    all_battery = dataclasses.replace(
        CONFIG, battery_dissipation_fraction=1.0, coupling_conductance_w_per_c=0.0
    )
    stack, thermal = _stack(4.0, config=all_battery)
    truth = _run(stack, thermal, 100, WARM)[-1]
    assert truth.battery_c > 20.0
    assert truth.electronics_c == 20.0
    all_electronics = dataclasses.replace(all_battery, battery_dissipation_fraction=0.0)
    stack, thermal = _stack(4.0, config=all_electronics)
    truth = _run(stack, thermal, 100, WARM)[-1]
    assert truth.battery_c == 20.0
    assert truth.electronics_c > 20.0


def test_coupling_moves_heat_between_nodes() -> None:
    stack, thermal = _stack(0.0, ThermalInitial(battery_c=20.0, electronics_c=40.0))
    truth = _run(stack, thermal, 10, WARM)[-1]
    assert truth.battery_c > 20.0  # warmed by the electronics, though at ambient


# --- Survival heater ------------------------------------------------------------------


def test_heater_switches_at_its_setpoints_without_chatter() -> None:
    stack, thermal = _stack(4.0)
    truths = _run(stack, thermal, 3 * 3600, COLD, dt_us=SECOND_US)  # 3 hours, 1 s ticks
    switches = []
    previous = False
    for tick, truth in enumerate(truths):
        # Invariants: off only at or above ON, on only at or below OFF.
        if truth.heater_on:
            assert truth.battery_c <= OFF_C
        else:
            assert truth.battery_c >= ON_C
        if truth.heater_on != previous:
            switches.append((tick, truth.heater_on, truth.battery_c))
            previous = truth.heater_on
    assert len(switches) >= 6
    for _, on, battery_c in switches:
        if on:
            assert battery_c < ON_C
        else:
            assert battery_c > OFF_C
    # Alternates, and each on or off period lasts minutes, not ticks.
    assert [on for _, on, _ in switches] == [i % 2 == 0 for i in range(len(switches))]
    gaps = [b[0] - a[0] for a, b in itertools.pairwise(switches)]
    assert min(gaps) > 60


def test_heater_holds_battery_up_under_a_cold_ambient() -> None:
    stack, thermal = _stack(4.0)
    truths = _run(stack, thermal, 4 * 3600, COLD, dt_us=SECOND_US)
    assert min(t.battery_c for t in truths) >= ON_C - HOLD_MARGIN_C
    # Without the heater the same ambient takes the battery far below.
    stack, thermal = _stack(4.0, config=dataclasses.replace(CONFIG, heater_power_w=0.0))
    truths = _run(stack, thermal, 4 * 3600, COLD, dt_us=SECOND_US)
    assert truths[-1].battery_c < ON_C - 10.0


def test_heater_power_appears_in_truth() -> None:
    stack, thermal = _stack(4.0, ThermalInitial(battery_c=ON_C - 1.0, electronics_c=0.0))
    truth = thermal.snapshot().truth
    assert truth.heater_on and truth.heater_power_w == CONFIG.heater_power_w
    seen = {(t.heater_on, t.heater_power_w) for t in _run(stack, thermal, 3600, COLD, SECOND_US)}
    assert seen == {(True, CONFIG.heater_power_w), (False, 0.0)}


def test_heater_heat_goes_into_the_battery() -> None:
    initial = ThermalInitial(battery_c=ON_C - 1.0, electronics_c=ON_C - 1.0)
    with_heater, a = _stack(0.0, initial)
    without, b = _stack(0.0, initial, config=dataclasses.replace(CONFIG, heater_power_w=0.0))
    ta = _run(with_heater, a, 1, COLD)[-1]
    tb = _run(without, b, 1, COLD)[-1]
    assert ta.battery_c - tb.battery_c == pytest.approx(
        CONFIG.heater_power_w * 0.1 / CONFIG.battery_heat_capacity_j_per_c, rel=1e-9
    )
    assert ta.electronics_c == tb.electronics_c


def test_heater_draw_in_power_load_is_not_counted_twice() -> None:
    # Power's total load (4 W + the 3 W heater draw, 7 W) includes the heater; thermal
    # heats the battery with the heater once, and the electronics see only the 4 W.
    initial = ThermalInitial(battery_c=ON_C - 1.0, electronics_c=10.0)
    with_draw, a = _stack(4.0, initial)
    without_heater, b = _stack(4.0, initial, config=dataclasses.replace(CONFIG, heater_power_w=0.0))
    ta = _run(with_draw, a, 1, COLD)[-1]
    tb = _run(without_heater, b, 1, COLD)[-1]
    assert ta.electronics_c == tb.electronics_c


def test_heater_is_not_commandable() -> None:
    assert not any(
        "thermal" in f.name or "heater" in f.name for f in dataclasses.fields(SpacecraftControls)
    )
    everything_off = SpacecraftControls(
        payload=PayloadControls(enabled=False),
        radio=RadioControls(mode=RadioMode.OFF),
        attitude=AttitudeControls(enabled=False),
        frozen_sensors=frozenset({"thermal"}),
        extra_load_w=5.0,
    )
    stack_a, a = _stack(4.0)
    stack_b, b = _stack(4.0)
    for _ in range(3600):
        stack_a.step(SECOND_US, COLD, CONTROLS)
        stack_b.step(SECOND_US, COLD, everything_off)
        # The freeze holds b's readings (#39); the heater and the physics are unchanged.
        assert a.snapshot().truth == b.snapshot().truth


# --- Readings and snapshot ------------------------------------------------------------


def test_readings_without_noise_report_true_temperatures_without_flags() -> None:
    quiet_cold = EnvironmentState(sunlit=False, ambient_temp_c=-20.0, sensor_noise_scale=0.0)
    stack, thermal = _stack(4.0)
    for _ in range(3600):
        stack.step(SECOND_US, quiet_cold, CONTROLS)
        truth = thermal.snapshot().truth
        readings = thermal.snapshot().readings
        assert readings.battery_c == truth.battery_c
        assert readings.electronics_c == truth.electronics_c
        assert not readings.over_temp and not readings.under_temp


def test_snapshot_is_built_once_per_step() -> None:
    stack, thermal = _stack(4.0)
    first = thermal.snapshot()
    assert thermal.snapshot() is first
    stack.step(TICK_US, WARM, CONTROLS)
    second = thermal.snapshot()
    assert second is not first
    assert thermal.snapshot() is second
    assert isinstance(second, ThermalSnapshot)


def test_deterministic() -> None:
    def run() -> list[ThermalTruth]:
        stack, thermal = _stack(4.0, script={500: 1.0, 2000: 6.0})
        return _run(stack, thermal, 3000, COLD, SECOND_US)

    assert run() == run()


# --- With the real power subsystem and the nominal environment ------------------------


def _nominal_stack() -> tuple[SubsystemStack, Power, Thermal]:
    board = SnapshotBoard()
    power = Power(NOMINAL_CONFIG.power, DEFAULT_INITIAL_STATE.power, reader=board)
    thermal = Thermal(NOMINAL_CONFIG.thermal, DEFAULT_INITIAL_STATE.thermal, reader=board)
    others = [f for f in fake_subsystems() if f.name not in ("power", "thermal")]
    stack = SubsystemStack([power, thermal, *others], board=board)
    stack.reset(RngFactory(7))
    return stack, power, thermal


def _orbit(orbits: int) -> list[tuple[EnvironmentState, ThermalTruth, float]]:
    env_model = NominalEnvironment()
    stack, power, thermal = _nominal_stack()
    out = []
    now_us = 0
    while now_us < orbits * env_model.orbit_period_us:
        env = env_model.state_at(now_us)
        stack.step(SECOND_US, env, CONTROLS)
        now_us += SECOND_US
        out.append((env, thermal.snapshot().truth, power.snapshot().truth.total_load_w))
    return out


def test_power_draws_the_heater_one_tick_late() -> None:
    stack, power, thermal = _nominal_stack()
    previous_heater_w = thermal.snapshot().truth.heater_power_w
    for _ in range(3600):
        stack.step(SECOND_US, COLD, CONTROLS)
        load_w = power.snapshot().truth.total_load_w
        # Base 1.8 W + fake attitude 0.5 W + fake comms 1.0 W + last tick's heater.
        assert load_w == pytest.approx(3.3 + previous_heater_w)
        previous_heater_w = thermal.snapshot().truth.heater_power_w
    assert previous_heater_w in (0.0, CONFIG.heater_power_w)


def test_heater_cycles_during_a_nominal_eclipse() -> None:
    # One orbit of NominalEnvironment (sunlit 20 °C, eclipse -20 °C, #73) with the real
    # power subsystem and the default settings.
    run = _orbit(1)
    sunlit = [truth for env, truth, _ in run if env.sunlit]
    eclipse = [truth for env, truth, _ in run if not env.sunlit]
    assert not any(t.heater_on for t in sunlit)
    switches_on = sum(1 for a, b in itertools.pairwise(eclipse) if b.heater_on and not a.heater_on)
    assert switches_on >= 2
    assert min(t.battery_c for t in eclipse) >= ON_C - HOLD_MARGIN_C


@pytest.mark.slow
def test_heater_cycles_every_nominal_eclipse_over_three_orbits() -> None:
    run = _orbit(3)
    period = NominalEnvironment().orbit_period_us // SECOND_US
    for orbit in range(3):
        chunk = run[orbit * period : (orbit + 1) * period]
        sunlit = [truth for env, truth, _ in chunk if env.sunlit]
        eclipse = [truth for env, truth, _ in chunk if not env.sunlit]
        assert not any(t.heater_on for t in sunlit[600:])  # after leaving the eclipse
        cycles = sum(1 for a, b in itertools.pairwise(eclipse) if b.heater_on and not a.heater_on)
        assert cycles >= 2
        assert min(t.battery_c for t in eclipse) >= ON_C - HOLD_MARGIN_C
        assert max(t.electronics_c for t in sunlit) < 45.0
