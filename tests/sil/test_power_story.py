"""Story-level integration test for power (story #34, delivered with #36).

Power runs in a ``SubsystemStack`` with the shared fakes for thermal, attitude,
payload, and comms (#76), driven by ``NominalEnvironment`` (92-minute orbit, 35%
eclipse) and a ``SimClock`` at the default 100 ms tick, for three full orbits, with
the default settings, starting state, and controls (payload off, radio ``RX_TX``,
attitude control on). The fakes' draws are the shared defaults: attitude control
0.5 W, transmit 1.0 W, payload and heater off.

The nominal scenario steps three identical stacks in lockstep: two with the same seed,
which must produce identical ``SpacecraftState`` values every tick, and one with a
different seed, whose truth must match and whose noisy readings must differ. The other
scenarios step one stack, to keep the suite fast.
"""

from dataclasses import dataclass, field
from itertools import pairwise
from typing import Any

import pytest

from pocketsat.core.clock import SimClock
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    Power,
    PowerInitial,
    PowerSnapshot,
    SnapshotBoard,
    SpacecraftControls,
    SubsystemStack,
)
from pocketsat.spacecraft.fakes import FakeSubsystem, fake_subsystems

# Multi-orbit story runs: skipped by a plain `pytest`, run with `pytest -m slow`.
pytestmark = pytest.mark.slow


ORBITS = 3
CONFIG = NOMINAL_CONFIG.power


def _stack(seed: int, initial: PowerInitial) -> SubsystemStack:
    board = SnapshotBoard()
    power = Power(CONFIG, initial, reader=board)
    fakes: list[FakeSubsystem[Any]] = [f for f in fake_subsystems() if f.name != "power"]
    stack = SubsystemStack([power, *fakes], board=board)
    stack.reset(RngFactory(seed))
    return stack


def _expected_flags(low: bool, critical: bool, estimate: float) -> tuple[bool, bool]:
    """The documented hysteresis rule: set below the set threshold, clear above the
    clear threshold, otherwise hold."""
    low_set, low_clear = CONFIG.low_battery_soc, CONFIG.low_battery_clear_soc
    critical_set, critical_clear = CONFIG.critical_battery_soc, CONFIG.critical_battery_clear_soc
    return (
        estimate <= low_clear if low else estimate < low_set,
        estimate <= critical_clear if critical else estimate < critical_set,
    )


@dataclass
class Period:
    """One sunlit or eclipse period: the SOC before its first tick and after each."""

    sunlit: bool
    start_soc: float
    socs: list[float] = field(default_factory=list)


@dataclass
class Run:
    periods: list[Period]
    orbit_energy_wh: list[float]
    orbit_start_soc: list[float]
    orbit_end_soc: list[float]
    flag_changes: list[tuple[bool, bool]]
    readings_differ: bool


def _run(initial: PowerInitial, controls: SpacecraftControls, *, lockstep: bool = True) -> Run:
    environment = NominalEnvironment()
    clock = SimClock()
    ticks_per_orbit = environment.orbit_period_us // clock.tick_us
    assert ticks_per_orbit * clock.tick_us == environment.orbit_period_us

    seeds = (1, 1, 2) if lockstep else (1,)
    stacks = [_stack(seed, initial) for seed in seeds]
    first = stacks[0].snapshot().subsystems["power"]
    assert isinstance(first, PowerSnapshot)
    soc = first.truth.soc
    low, critical = first.readings.low_battery, first.readings.critical_battery

    periods: list[Period] = []
    orbit_energy_wh: list[float] = []
    orbit_start_soc: list[float] = []
    orbit_end_soc: list[float] = []
    flag_changes: list[tuple[bool, bool]] = []
    readings_differ = False
    energy_wh = 0.0

    for tick in range(ORBITS * ticks_per_orbit):
        if tick % ticks_per_orbit == 0:
            orbit_start_soc.append(soc)
            energy_wh = 0.0
        env = environment.sample(clock)
        if not periods or periods[-1].sunlit != env.sunlit:
            periods.append(Period(sunlit=env.sunlit, start_soc=soc))
        for stack in stacks:
            stack.step(clock.tick_us, env, controls)
        clock.advance_one_tick()

        state = stacks[0].snapshot()
        power = state.get("power", PowerSnapshot)
        if lockstep:
            same, other = stacks[1].snapshot(), stacks[2].snapshot()
            assert state == same  # deterministic: same seed, identical state
            other_power = other.get("power", PowerSnapshot)
            assert other_power.truth == power.truth
            readings_differ = readings_differ or other_power.readings != power.readings

        soc = power.truth.soc
        periods[-1].socs.append(soc)
        energy_wh += power.truth.net_power_w * clock.tick_us / 1_000_000 / 3600.0

        readings = power.readings
        expected = _expected_flags(low, critical, readings.soc)
        assert (readings.low_battery, readings.critical_battery) == expected
        if expected != (low, critical):
            flag_changes.append(expected)
        low, critical = expected

        if (tick + 1) % ticks_per_orbit == 0:
            orbit_energy_wh.append(energy_wh)
            orbit_end_soc.append(soc)

    return Run(
        periods, orbit_energy_wh, orbit_start_soc, orbit_end_soc, flag_changes, readings_differ
    )


def _check_story(run: Run, *, lockstep: bool = True) -> None:
    # Each orbit is one sunlit period followed by one eclipse.
    assert [p.sunlit for p in run.periods] == [True, False] * ORBITS

    for period in run.periods:
        socs = [period.start_soc, *period.socs]
        steps = list(pairwise(socs))
        if period.sunlit:
            # SOC rises during every sunlit period (until the battery is full).
            assert period.socs[-1] > period.start_soc
            assert all(b > a or b == 1.0 for a, b in steps)
        else:
            # SOC falls during every eclipse, every tick.
            assert period.socs[-1] < period.start_soc
            assert all(b < a for a, b in steps)

    # Net energy over each orbit is non-negative: the battery does not trend to empty.
    assert len(run.orbit_energy_wh) == ORBITS
    assert all(e >= 0.0 for e in run.orbit_energy_wh)
    assert all(
        end >= start for start, end in zip(run.orbit_start_soc, run.orbit_end_soc, strict=True)
    )

    # A different seed changes the noisy readings (but not the truth).
    assert run.readings_differ or not lockstep


def test_story_nominal_start() -> None:
    run = _run(DEFAULT_INITIAL_STATE.power, SpacecraftControls())
    _check_story(run)
    # From the default 0.8 start the battery stays well above the low threshold.
    assert run.flag_changes == []


def test_story_low_start_flags_clear_once_without_flapping() -> None:
    # Starting at 0.2 the low flag is set. The first sunlit period lifts the estimate
    # above 0.35, which clears it; the following eclipses only reach about 0.33, inside
    # the hysteresis band, so it never sets again.
    run = _run(PowerInitial(soc=0.2), SpacecraftControls(), lockstep=False)
    _check_story(run, lockstep=False)
    assert run.flag_changes == [(False, False)]


def test_story_drain_sets_each_flag_once() -> None:
    # A 5 W battery_drain override makes the battery trend down over three orbits
    # (about -0.25 SOC per orbit): each flag sets exactly once, at its hysteresis
    # crossing, and never flaps while the SOC creeps past the thresholds.
    run = _run(DEFAULT_INITIAL_STATE.power, SpacecraftControls(extra_load_w=5.0), lockstep=False)
    assert run.flag_changes == [(True, False), (True, True)]
