"""Story-level integration test for thermal (story #37, delivered with #39).

The real ``Power`` and the real ``Thermal`` run in a ``SubsystemStack`` sharing a
``SnapshotBoard``, with the shared fakes for attitude, payload, and comms (#76),
driven by ``NominalEnvironment`` (92-minute orbit, 35% eclipse, 20 °C sunlit and
-20 °C eclipse ambient, #73) and a ``SimClock`` at the default 100 ms tick, for three
full orbits, with the default settings, starting state, and controls. The fakes'
draws are the shared defaults (attitude control 0.5 W, transmit 1.0 W, payload off),
so power's total load is 3.5 W plus the survival heater.

Three identical stacks step in lockstep: two with the same seed, which must produce
identical ``SpacecraftState`` values every tick, and one with a different seed, whose
truth must match and whose noisy readings must differ.

**Periodicity tolerance.** The run starts at 20 °C, so the first orbit is a transient.
The second and third orbits are compared tick by tick: every true temperature must
agree within :data:`PERIODIC_TOLERANCE_C`. The remaining difference comes from the
survival heater's switching instants, which still move slightly between orbits while
the pattern converges: with the defaults the largest difference is about 0.14 °C
between the second and third orbits, and about 0.002 °C between the third and a
fourth. The temperatures at the end of each orbit must agree within
:data:`DRIFT_TOLERANCE_C` (they differ by about 0.002 °C): no long-term drift.
"""

import itertools
from dataclasses import dataclass, field
from typing import Any

import pytest

from pocketsat.core.clock import SimClock
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    Power,
    PowerSnapshot,
    SnapshotBoard,
    SpacecraftControls,
    SubsystemStack,
    Thermal,
    ThermalSnapshot,
    ThermalTruth,
)
from pocketsat.spacecraft.fakes import FakeSubsystem, fake_subsystems

# Multi-orbit story run: skipped by a plain `pytest`, run with `pytest -m slow`.
pytestmark = pytest.mark.slow

ORBITS = 3
FAKE_LOAD_W = 3.5
"""Power's total load without the heater: base 2.0 W, attitude 0.5 W, transmit 1.0 W."""

PERIODIC_TOLERANCE_C = 0.25
"""Largest tick-by-tick difference between the second and third orbits, °C."""

DRIFT_TOLERANCE_C = 0.01
"""Largest difference between the second and third orbits' end temperatures, °C."""


def _stack(seed: int) -> SubsystemStack:
    board = SnapshotBoard()
    power = Power(NOMINAL_CONFIG.power, DEFAULT_INITIAL_STATE.power, reader=board)
    thermal = Thermal(NOMINAL_CONFIG.thermal, DEFAULT_INITIAL_STATE.thermal, reader=board)
    fakes: list[FakeSubsystem[Any]] = [
        f for f in fake_subsystems() if f.name not in ("power", "thermal")
    ]
    stack = SubsystemStack([power, thermal, *fakes], board=board)
    stack.reset(RngFactory(seed))
    return stack


@dataclass
class Period:
    """One sunlit or eclipse period: the thermal truth before its first tick and after
    each tick."""

    sunlit: bool
    start: ThermalTruth
    truths: list[ThermalTruth] = field(default_factory=list)


@dataclass
class Run:
    periods: list[Period] = field(default_factory=list)
    orbits: list[list[ThermalTruth]] = field(default_factory=list)
    flagged_ticks: int = 0
    readings_differ: bool = False
    loads_w: set[float] = field(default_factory=set)


def _run() -> Run:
    environment = NominalEnvironment()
    clock = SimClock()
    controls = SpacecraftControls()
    ticks_per_orbit = environment.orbit_period_us // clock.tick_us
    assert ticks_per_orbit * clock.tick_us == environment.orbit_period_us

    stacks = [_stack(seed) for seed in (1, 1, 2)]
    run = Run()
    first = stacks[0].snapshot().get("thermal", ThermalSnapshot)
    truth = first.truth
    previous_heater_w = truth.heater_power_w

    for tick in range(ORBITS * ticks_per_orbit):
        if tick % ticks_per_orbit == 0:
            run.orbits.append([])
        env = environment.sample(clock)
        if not run.periods or run.periods[-1].sunlit != env.sunlit:
            run.periods.append(Period(sunlit=env.sunlit, start=truth))
        for stack in stacks:
            stack.step(clock.tick_us, env, controls)
        clock.advance_one_tick()

        state, same, other = (stack.snapshot() for stack in stacks)
        assert state == same  # deterministic: same seed, identical SpacecraftState
        thermal = state.get("thermal", ThermalSnapshot)
        other_thermal = other.get("thermal", ThermalSnapshot)
        assert other_thermal.truth == thermal.truth  # a seed changes only the readings
        assert other.get("power", PowerSnapshot).truth == state.get("power", PowerSnapshot).truth
        run.readings_differ = run.readings_differ or other_thermal.readings != thermal.readings

        # The heater draw thermal published last tick is in power's load this tick.
        load_w = state.get("power", PowerSnapshot).truth.total_load_w
        assert load_w == pytest.approx(FAKE_LOAD_W + previous_heater_w)
        run.loads_w.add(load_w)
        previous_heater_w = thermal.truth.heater_power_w

        readings = thermal.readings
        run.flagged_ticks += readings.over_temp or readings.under_temp
        truth = thermal.truth
        run.periods[-1].truths.append(truth)
        run.orbits[-1].append(truth)
    return run


@pytest.fixture(scope="module")
def run() -> Run:
    return _run()


def test_temperatures_cycle_with_eclipse(run: Run) -> None:
    # Each orbit is one sunlit period followed by one eclipse.
    assert [p.sunlit for p in run.periods] == [True, False] * ORBITS
    for period in run.periods:
        end = period.truths[-1]
        if period.sunlit:
            assert end.battery_c > period.start.battery_c
            assert end.electronics_c > period.start.electronics_c
        else:
            assert end.battery_c < period.start.battery_c - 10.0
            assert end.electronics_c < period.start.electronics_c - 10.0
    for sunlit, eclipse in itertools.batched(run.periods, 2):
        assert max(t.battery_c for t in sunlit.truths) > max(t.battery_c for t in eclipse.truths)
        assert min(t.electronics_c for t in eclipse.truths) < min(
            t.electronics_c for t in sunlit.truths
        )


def test_settles_into_the_same_pattern_each_orbit(run: Run) -> None:
    assert len(run.orbits) == ORBITS
    second, third = run.orbits[1], run.orbits[2]
    assert len(second) == len(third)
    for a, b in zip(second, third, strict=True):
        assert abs(a.battery_c - b.battery_c) <= PERIODIC_TOLERANCE_C
        assert abs(a.electronics_c - b.electronics_c) <= PERIODIC_TOLERANCE_C
    # No long-term drift: each orbit ends where the previous one did.
    assert abs(second[-1].battery_c - third[-1].battery_c) <= DRIFT_TOLERANCE_C
    assert abs(second[-1].electronics_c - third[-1].electronics_c) <= DRIFT_TOLERANCE_C
    # The pattern converges: the orbits get closer, not further apart.
    gap_12 = max(abs(a.battery_c - b.battery_c) for a, b in zip(run.orbits[0], second, strict=True))
    gap_23 = max(abs(a.battery_c - b.battery_c) for a, b in zip(second, third, strict=True))
    assert gap_23 < gap_12


def test_no_limit_flag_in_nominal_orbits(run: Run) -> None:
    assert run.flagged_ticks == 0


def test_heater_cycles_during_every_eclipse(run: Run) -> None:
    for period in run.periods:
        truths = [period.start, *period.truths]
        on = sum(1 for a, b in itertools.pairwise(truths) if b.heater_on and not a.heater_on)
        off = sum(1 for a, b in itertools.pairwise(truths) if a.heater_on and not b.heater_on)
        if period.sunlit:
            assert on == 0  # never switches on in sunlight
        else:
            assert on >= 1 and off >= 1
    # Its draw appears in power's total load (checked one tick late, every tick).
    assert sorted(run.loads_w) == pytest.approx(
        [FAKE_LOAD_W, FAKE_LOAD_W + NOMINAL_CONFIG.thermal.heater_power_w]
    )


def test_a_different_seed_changes_the_noisy_readings(run: Run) -> None:
    assert run.readings_differ
