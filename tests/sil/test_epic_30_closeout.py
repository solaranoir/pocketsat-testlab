"""Epic #30 close-out: all five subsystems together, deterministic for a given seed (#70).

The real ``Power``, ``Thermal``, ``Attitude``, ``Payload``, and ``Comms`` run in one
``SubsystemStack`` (default ``STEP_ORDER``) sharing a ``SnapshotBoard``, driven by
``NominalEnvironment`` and a ``SimClock`` at the default 100 ms tick, with
``NOMINAL_CONFIG`` and the default starting state, under #72's reference profile:
SCIENCE plus one 10-minute DOWNLINK pass per orbit. The profile driver and its traffic
and release stand-ins are #72's own (``_reference_profile.py``), not a copy.

**Determinism** (:func:`test_same_seed_gives_an_identical_state_sequence`,
:func:`test_a_different_seed_gives_different_noisy_readings`): three stacks run in
lockstep for :data:`ORBITS` orbits, two with seed :data:`SEED` and one with
:data:`OTHER_SEED`. The two same-seed stacks must produce an identical
``SpacecraftState`` at every tick. The other seed must give different reported (noisy)
values. Truth is *not* expected to match across seeds: the attitude disturbance is
seeded true dynamics, and payload acquisition follows reported values (ADR-0004 §6),
whose draw feeds back into the true SOC (see the #70 comments).

**Named-stream independence** (ADR-0003;
:func:`test_adding_or_removing_a_subsystem_leaves_other_streams_unchanged`,
:func:`test_a_subsystem_alone_draws_what_it_draws_in_the_full_stack`) is checked at two
levels, each run being one orbit of the reference profile:

1. *Raw stream outputs.* A recording ``RngFactory`` logs every ``random()`` value each
   named stream hands out. Against the full stack, each subsystem in turn is removed:
   replaced with a scripted fake from ``pocketsat.spacecraft.fakes``, which requests no
   streams, because the others read its snapshot and the stack can't run without it.
   And power, thermal, and attitude are each run on their own (built with no reader,
   next to only a fake payload for the profile driver to read, so nothing else draws
   from the factory), which is the "adding" direction. Every stream of every remaining
   subsystem must hand out exactly the same values as in the full stack, the removed
   subsystem's streams must not be requested, and no other stream may appear. This
   works because every draw is unconditional: power and thermal draw their noise every
   tick, frozen or not; attitude draws its disturbance every tick and its noise every
   tick while the noise scale is non-zero (it is 1 throughout). So the draw sequence
   cannot depend on physics, only on the stream's own seed, and any sharing or
   reordering of streams between subsystems would show up as a difference.
2. *What the subsystem reports.* With the removals chosen, physics does change (the
   fakes' draws, heater, and pointing differ from the real ones), so the check also
   asserts that some remaining subsystem's truth differs. Coupling therefore can't
   explain an *unchanged* noise sequence, and an unchanged noise is what is required:

   - power and thermal: the noise they add, ``readings - truth`` (bus voltage, battery
     current, both temperatures), matches the full stack to rounding
     (:data:`NOISE_TOLERANCE`), even though truth differs;
   - attitude reads no other subsystem, so its whole snapshot sequence, truth and
     readings, is bit-identical with or without the others.

   Payload and comms use no randomness, so they have nothing to compare; level 1
   confirms they request no streams.

Runtimes are printed (run with ``-s``) and recorded in the #70 PR as input to #63's CI
budget (under 10 s per orbit) and #78's suite budget.
"""

import time
from array import array
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from random import Random

import pytest
from _reference_profile import PASS_US, REFERENCE, ProfileDriver, real_stack, real_subsystems

from pocketsat.core.clock import SimClock
from pocketsat.core.rng import RngFactory, derive_seed
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    STEP_ORDER,
    Attitude,
    AttitudeSnapshot,
    CommsSnapshot,
    PayloadSnapshot,
    Power,
    PowerSnapshot,
    SnapshotBoard,
    SpacecraftState,
    Subsystem,
    SubsystemStack,
    Thermal,
    ThermalSnapshot,
)
from pocketsat.spacecraft import attitude as attitude_module
from pocketsat.spacecraft import power as power_module
from pocketsat.spacecraft import thermal as thermal_module
from pocketsat.spacecraft.fakes import FakeSubsystem, default_snapshot

# Multi-orbit integration run: skipped by a plain `pytest`, run with `pytest -m slow`.
pytestmark = pytest.mark.slow

ORBITS = 3
SEED = 1
OTHER_SEED = 2

STREAMS: dict[str, frozenset[str]] = {
    "power": frozenset({power_module.NOISE_STREAM}),
    "thermal": frozenset({thermal_module.NOISE_STREAM}),
    "attitude": frozenset({attitude_module.DISTURBANCE_STREAM, attitude_module.NOISE_STREAM}),
    "payload": frozenset(),
    "comms": frozenset(),
}
"""The named streams each subsystem requests (payload and comms use no randomness)."""

NOISE_TOLERANCE = 1e-12
"""``readings - truth`` recovers the drawn noise only to floating-point rounding of the
sum (about 1e-15 at these magnitudes), which depends on the truth value."""

MIN_DIFFERING_FRACTION = 0.99
"""With a different seed, each noisy reported value differs in at least this fraction
of ticks (measured: every tick, for every value in :data:`READINGS`)."""

ReadingsField = Callable[[SpacecraftState], float]

READINGS: dict[str, ReadingsField] = {
    "power.bus_v": lambda s: s.get("power", PowerSnapshot).readings.bus_v,
    "power.battery_current_a": lambda s: s.get("power", PowerSnapshot).readings.battery_current_a,
    "thermal.battery_c": lambda s: s.get("thermal", ThermalSnapshot).readings.battery_c,
    "thermal.electronics_c": lambda s: s.get("thermal", ThermalSnapshot).readings.electronics_c,
    "attitude.pointing_error_deg": (
        lambda s: s.get("attitude", AttitudeSnapshot).readings.pointing_error_deg
    ),
    "attitude.rate_dps": lambda s: s.get("attitude", AttitudeSnapshot).readings.rate_dps,
}


def _ticks_per_orbit(environment: NominalEnvironment, clock: SimClock) -> int:
    ticks, rest = divmod(environment.orbit_period_us, clock.tick_us)
    assert rest == 0
    return ticks


# --- Determinism: three stacks in lockstep ---------------------------------------------


@dataclass
class LockstepRun:
    ticks: int = 0
    pass_ticks: int = 0
    first_divergence: int | None = None
    """First tick (1-based) at which the two same-seed stacks differed, if any."""
    differing: dict[str, int] = field(default_factory=lambda: dict.fromkeys(READINGS, 0))
    """Ticks in which each reported value differs between the two seeds."""
    flags: set[str] = field(default_factory=set)
    released_per_orbit: list[int] = field(default_factory=list)
    names: tuple[str, ...] = ()
    final: SpacecraftState | None = None
    runtime_s: float = 0.0
    one_stack_s: float = 0.0
    """Time spent driving and stepping one stack, seconds."""


def _flags(state: SpacecraftState) -> set[str]:
    power = state.get("power", PowerSnapshot).readings
    thermal = state.get("thermal", ThermalSnapshot).readings
    return {
        name
        for name, on in (
            ("low_battery", power.low_battery),
            ("critical_battery", power.critical_battery),
            ("under_temp", thermal.under_temp),
            ("over_temp", thermal.over_temp),
        )
        if on
    }


def _lockstep_run() -> LockstepRun:
    started = time.perf_counter()
    environment = NominalEnvironment()
    clock = SimClock()
    ticks_per_orbit = _ticks_per_orbit(environment, clock)
    driver = ProfileDriver(REFERENCE, NOMINAL_CONFIG, environment.orbit_period_us)
    stacks = [real_stack(SEED), real_stack(SEED), real_stack(OTHER_SEED)]
    run = LockstepRun(names=stacks[0].names)
    previous = [stack.snapshot() for stack in stacks]
    assert previous[0] == previous[1]
    released_start = 0
    for tick in range(1, ORBITS * ticks_per_orbit + 1):
        now_us = clock.now_us
        env = environment.state_at(now_us)
        states = []
        for i, (stack, before) in enumerate(zip(stacks, previous, strict=True)):
            # Each stack is driven from its own state (its release follows its buffer).
            t0 = time.perf_counter()
            command = driver.command(now_us, before)
            stack.step(clock.tick_us, env, command.controls)
            states.append(stack.snapshot())
            if i == 0:
                run.one_stack_s += time.perf_counter() - t0
                run.pass_ticks += command.in_pass
        clock.advance_one_tick()
        state, same, other = states
        if run.first_divergence is None and state != same:
            run.first_divergence = tick
        for name, read in READINGS.items():
            run.differing[name] += read(state) != read(other)
        run.flags |= _flags(state)
        if tick % ticks_per_orbit == 0:
            released = state.get("payload", PayloadSnapshot).truth.total_released_bytes
            run.released_per_orbit.append(released - released_start)
            released_start = released
        previous = states
    run.ticks = ORBITS * ticks_per_orbit
    run.final = previous[0]
    run.runtime_s = time.perf_counter() - started
    print(
        f"\nepic #30 close-out: {ORBITS} orbits x 3 stacks in lockstep, {run.ticks} ticks"
        f" each: {run.runtime_s:.1f} s wall; one stack {run.one_stack_s:.1f} s"
        f" ({run.one_stack_s / ORBITS:.2f} s per orbit)"
    )
    print(
        "  ticks with different readings, seed"
        f" {SEED} vs {OTHER_SEED}: "
        + ", ".join(f"{k} {v / run.ticks:.1%}" for k, v in run.differing.items())
    )
    return run


@pytest.fixture(scope="module")
def lockstep() -> LockstepRun:
    return _lockstep_run()


def test_all_five_real_subsystems_run_the_reference_profile(lockstep: LockstepRun) -> None:
    assert lockstep.names == STEP_ORDER
    assert lockstep.final is not None
    for name, kind in (
        ("power", PowerSnapshot),
        ("thermal", ThermalSnapshot),
        ("attitude", AttitudeSnapshot),
        ("payload", PayloadSnapshot),
        ("comms", CommsSnapshot),
    ):
        lockstep.final.get(name, kind)
    clock = SimClock()
    assert clock.tick_us == 100_000  # the default tick, never coarsened
    orbit_ticks = NominalEnvironment().orbit_period_us // clock.tick_us
    assert lockstep.ticks == ORBITS * orbit_ticks
    # One 10-minute DOWNLINK pass per orbit, and each pass downlinked (released) data.
    assert lockstep.pass_ticks == ORBITS * PASS_US // clock.tick_us
    assert len(lockstep.released_per_orbit) == ORBITS
    assert all(released > 0 for released in lockstep.released_per_orbit)
    # The reference profile is the nominal case (#72): nothing is flagged.
    assert not lockstep.flags, f"flags raised in the reference profile: {lockstep.flags}"


def test_same_seed_gives_an_identical_state_sequence(lockstep: LockstepRun) -> None:
    assert lockstep.first_divergence is None, (
        f"same-seed stacks diverged at tick {lockstep.first_divergence}"
    )


def test_a_different_seed_gives_different_noisy_readings(lockstep: LockstepRun) -> None:
    # Only the reported values are asserted; truth may differ too (module docstring).
    minimum = MIN_DIFFERING_FRACTION
    for name in READINGS:
        fraction = lockstep.differing[name] / lockstep.ticks
        assert fraction >= minimum, (
            f"{name}: seeds {SEED} and {OTHER_SEED} differ in {fraction:.1%} of ticks;"
            f" required {minimum:.0%}"
        )


# --- Named-stream independence ---------------------------------------------------------


class _RecordingRandom(Random):
    """A ``Random`` that logs every ``random()`` value it returns."""

    def __init__(self, seed: int, log: array[float]) -> None:
        self._log = log
        super().__init__(seed)

    def random(self) -> float:
        value = super().random()
        self._log.append(value)
        return value


class _RecordingRngFactory(RngFactory):
    """An ``RngFactory`` whose streams log their outputs, by stream name.

    The streams are seeded exactly as ``RngFactory.stream`` seeds them, so the values
    are the ones the subsystem would see without recording.
    """

    def __init__(self, master_seed: int) -> None:
        super().__init__(master_seed)
        self.draws: dict[str, array[float]] = {}

    def stream(self, name: str) -> Random:
        super().stream(name)  # validates the name and records its seed
        log = self.draws.setdefault(name, array("d"))
        return _RecordingRandom(derive_seed(self.master_seed, name), log)


@dataclass
class Trace:
    """One orbit of one stack: stream outputs and what each subsystem reported."""

    draws: dict[str, array[float]]
    noise: dict[str, array[float]] = field(default_factory=dict)
    """``readings - truth`` per value, for the subsystems in the stack."""
    truth: dict[str, array[float]] = field(default_factory=dict)
    """A true value per noisy subsystem (power SOC, battery temperature)."""
    attitude: list[AttitudeSnapshot] = field(default_factory=list)
    runtime_s: float = 0.0


def _trace(subsystems: Iterable[Subsystem], board: SnapshotBoard, real: set[str]) -> Trace:
    """Run one orbit of the reference profile with a recording factory, tracing the
    subsystems named in ``real`` (the others are fakes)."""
    started = time.perf_counter()
    stack = SubsystemStack(subsystems, board=board)
    rng = _RecordingRngFactory(SEED)
    stack.reset(rng)
    environment = NominalEnvironment()
    clock = SimClock()
    driver = ProfileDriver(REFERENCE, NOMINAL_CONFIG, environment.orbit_period_us)
    trace = Trace(draws=rng.draws)
    names = real
    noise = trace.noise
    truth = trace.truth
    if "power" in names:
        for key in ("power.bus_v", "power.battery_current_a", "power.soc"):
            (truth if key == "power.soc" else noise)[key] = array("d")
    if "thermal" in names:
        for key in ("thermal.battery_c", "thermal.electronics_c"):
            noise[key] = array("d")
        truth["thermal.battery_c"] = array("d")
    previous = stack.snapshot()
    for _ in range(_ticks_per_orbit(environment, clock)):
        now_us = clock.now_us
        command = driver.command(now_us, previous)
        stack.step(clock.tick_us, environment.state_at(now_us), command.controls)
        clock.advance_one_tick()
        state = stack.snapshot()
        if "power" in names:
            power = state.get("power", PowerSnapshot)
            noise["power.bus_v"].append(power.readings.bus_v - power.truth.bus_v)
            noise["power.battery_current_a"].append(
                power.readings.battery_current_a - power.truth.battery_current_a
            )
            truth["power.soc"].append(power.truth.soc)
        if "thermal" in names:
            thermal = state.get("thermal", ThermalSnapshot)
            noise["thermal.battery_c"].append(thermal.readings.battery_c - thermal.truth.battery_c)
            noise["thermal.electronics_c"].append(
                thermal.readings.electronics_c - thermal.truth.electronics_c
            )
            truth["thermal.battery_c"].append(thermal.truth.battery_c)
        if "attitude" in names:
            trace.attitude.append(state.get("attitude", AttitudeSnapshot))
        previous = state
    trace.runtime_s = time.perf_counter() - started
    return trace


def _with_fakes(real: Iterable[str], board: SnapshotBoard) -> list[Subsystem]:
    """The real subsystems named in ``real``, and a default fake for every other one."""
    subsystems = real_subsystems(board)
    return [
        subsystems[name] if name in real else FakeSubsystem(name, default_snapshot(name))
        for name in STEP_ORDER
    ]


def _alone(name: str) -> tuple[list[Subsystem], SnapshotBoard]:
    """``name`` as the only subsystem drawing from the factory: built with no reader,
    plus a fake payload for the profile driver to read the buffer from."""
    board = SnapshotBoard()
    config, initial = NOMINAL_CONFIG, DEFAULT_INITIAL_STATE
    built: Subsystem
    if name == "power":
        built = Power(config.power, initial.power)
    elif name == "thermal":
        built = Thermal(config.thermal, initial.thermal)
    else:
        built = Attitude(config.attitude, initial.attitude)
    return [built, FakeSubsystem("payload", default_snapshot("payload"))], board


def _max_abs_difference(a: array[float], b: array[float]) -> float:
    assert len(a) == len(b)
    return max(abs(x - y) for x, y in zip(a, b, strict=True))


@pytest.fixture(scope="module")
def full_trace() -> Trace:
    board = SnapshotBoard()
    return _trace(real_subsystems(board).values(), board, set(STEP_ORDER))


def _check_against_full(
    full: Trace, trace: Trace, present: set[str], case: str, coupled: bool
) -> None:
    expected_streams = set().union(*(STREAMS[name] for name in present))
    # Level 1: the same streams, handing out exactly the same values.
    assert set(trace.draws) == expected_streams, f"{case}: requested {sorted(trace.draws)}"
    for stream in expected_streams:
        assert len(trace.draws[stream]) == len(full.draws[stream]), (
            f"{case}: {stream} handed out {len(trace.draws[stream])} values,"
            f" {len(full.draws[stream])} in the full stack"
        )
        assert trace.draws[stream] == full.draws[stream], f"{case}: {stream} values changed"
    # Level 2: the noise each subsystem reports is unchanged...
    for key, values in trace.noise.items():
        difference = _max_abs_difference(values, full.noise[key])
        assert difference <= NOISE_TOLERANCE, (
            f"{case}: {key} noise (readings - truth) differs by up to {difference:.3g}"
        )
    if "attitude" in present:
        assert trace.attitude == full.attitude, f"{case}: attitude snapshots changed"
    # ...even where the physics it measures changed, so coupling can't explain it.
    if coupled:
        assert any(values != full.truth[key] for key, values in trace.truth.items()), (
            f"{case}: no truth changed, so the case does not exercise coupling"
        )


@pytest.mark.parametrize("removed", STEP_ORDER)
def test_adding_or_removing_a_subsystem_leaves_other_streams_unchanged(
    removed: str, full_trace: Trace
) -> None:
    present = set(STEP_ORDER) - {removed}
    board = SnapshotBoard()
    trace = _trace(_with_fakes(present, board), board, present)
    print(f"\n  {removed} replaced with a fake: one orbit, {trace.runtime_s:.1f} s wall")
    # Power or thermal (or both) remains and reads the fake, so some truth changes.
    _check_against_full(full_trace, trace, present, f"{removed} removed", coupled=True)


@pytest.mark.parametrize("name", ["power", "thermal", "attitude"])
def test_a_subsystem_alone_draws_what_it_draws_in_the_full_stack(
    name: str, full_trace: Trace
) -> None:
    # The "adding" direction: from a stack where only `name` draws random values to the
    # full stack, its streams hand out the same values and its noise is unchanged.
    subsystems, board = _alone(name)
    trace = _trace(subsystems, board, {name})
    print(f"\n  {name} alone: one orbit, {trace.runtime_s:.1f} s wall")
    # Power and thermal alone read nothing, so their truth differs from the full stack;
    # attitude reads nothing in either case, so its truth is the same.
    _check_against_full(full_trace, trace, {name}, f"{name} alone", coupled=name != "attitude")
