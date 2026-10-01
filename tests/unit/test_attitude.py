"""Tests for the scalar attitude model (#41) and story #40's integration test."""

import dataclasses
import hashlib
from collections.abc import Callable

import pytest

from pocketsat.core.clock import SimClock
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    Attitude,
    AttitudeConfig,
    AttitudeControls,
    AttitudeInitial,
    AttitudeReadings,
    AttitudeSnapshot,
    AttitudeState,
    Power,
    PowerSnapshot,
    SnapshotBoard,
    SpacecraftControls,
    Subsystem,
    SubsystemStack,
)
from pocketsat.spacecraft.attitude import DISTURBANCE_STREAM, NOISE_STREAM, next_state
from pocketsat.spacecraft.fakes import FakeSubsystem, default_snapshot
from pocketsat.targets.base import EnvironmentState

TICK_US = 100_000
TICKS_PER_MINUTE = 600
ENV = EnvironmentState()
ON = SpacecraftControls()
OFF = SpacecraftControls(attitude=AttitudeControls(enabled=False))
FROZEN = SpacecraftControls(frozen_sensors=frozenset({"attitude"}))
CONFIG = AttitudeConfig()

TUMBLING = AttitudeState.TUMBLING
DETUMBLING = AttitudeState.DETUMBLING
STABILIZED = AttitudeState.STABILIZED


def _attitude(
    initial: AttitudeInitial = DEFAULT_INITIAL_STATE.attitude,
    config: AttitudeConfig = CONFIG,
    seed: int = 1,
) -> Attitude:
    attitude = Attitude(config, initial)
    attitude.reset(RngFactory(seed))
    return attitude


def _run(
    attitude: Attitude,
    ticks: int,
    env: EnvironmentState = ENV,
    controls: SpacecraftControls = ON,
) -> list[AttitudeSnapshot]:
    snaps = []
    for _ in range(ticks):
        attitude.step(TICK_US, env, controls)
        snaps.append(attitude.snapshot())
    return snaps


# --- Construction, configuration, reset ------------------------------------------------


def test_satisfies_subsystem_protocol() -> None:
    attitude = Attitude(CONFIG, AttitudeInitial())
    assert isinstance(attitude, Subsystem)
    assert attitude.name == "attitude"


def test_config_defaults_are_documented_values() -> None:
    assert NOMINAL_CONFIG.attitude == AttitudeConfig(
        rate_damping_per_s=0.05,
        pointing_gain_per_s=0.02,
        disturbance_mean_dps_per_s=0.002,
        disturbance_sd_dps_per_s=0.002,
        pointing_noise_deg=0.5,
        rate_noise_dps=0.01,
        tumbling_enter_rate_dps=2.0,
        tumbling_exit_rate_dps=1.5,
        stabilized_exit_rate_dps=0.4,
        stabilized_enter_rate_dps=0.2,
        stabilized_enter_pointing_error_deg=5.0,
        stabilized_exit_pointing_error_deg=10.0,
        control_power_w=0.5,
    )
    assert DEFAULT_INITIAL_STATE.attitude == AttitudeInitial(pointing_error_deg=45.0, rate_dps=1.0)


@pytest.mark.parametrize(
    ("build", "error"),
    [
        (lambda: AttitudeConfig(rate_damping_per_s=-0.1), ValueError),
        (lambda: AttitudeConfig(disturbance_sd_dps_per_s=float("nan")), ValueError),
        (lambda: AttitudeConfig(pointing_noise_deg=float("inf")), ValueError),
        (lambda: AttitudeConfig(control_power_w=-1.0), ValueError),
        (lambda: AttitudeConfig(stabilized_enter_rate_dps=0.0), ValueError),
        (lambda: AttitudeConfig(stabilized_enter_rate_dps=0.4), ValueError),
        (lambda: AttitudeConfig(stabilized_exit_rate_dps=1.6), ValueError),
        (lambda: AttitudeConfig(tumbling_exit_rate_dps=2.0), ValueError),
        (lambda: AttitudeConfig(stabilized_enter_pointing_error_deg=10.0), ValueError),
        (lambda: AttitudeConfig(stabilized_exit_pointing_error_deg=181.0), ValueError),
        (lambda: AttitudeConfig(rate_noise_dps=True), TypeError),
        (lambda: AttitudeConfig(control_power_w="0.5"), TypeError),  # type: ignore[arg-type]
    ],
)
def test_invalid_config_rejected(build: Callable[[], object], error: type[Exception]) -> None:
    with pytest.raises(error):
        build()


def test_edge_config_values_accepted() -> None:
    AttitudeConfig(
        rate_damping_per_s=0.0,
        pointing_gain_per_s=0.0,
        disturbance_mean_dps_per_s=0.0,
        disturbance_sd_dps_per_s=0.0,
        pointing_noise_deg=0.0,
        rate_noise_dps=0.0,
        stabilized_exit_rate_dps=1.5,
        stabilized_enter_pointing_error_deg=0.0,
        stabilized_exit_pointing_error_deg=180.0,
        control_power_w=0.0,
    )


def test_constructor_checks_record_types() -> None:
    with pytest.raises(TypeError):
        Attitude(AttitudeInitial(), AttitudeInitial())  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        Attitude(CONFIG, CONFIG)  # type: ignore[arg-type]


def test_step_before_reset_raises() -> None:
    with pytest.raises(RuntimeError):
        Attitude(CONFIG, AttitudeInitial()).step(TICK_US, ENV, ON)


@pytest.mark.parametrize("dt_us", [-1, 1.5, True])
def test_step_rejects_bad_dt(dt_us: object) -> None:
    with pytest.raises((TypeError, ValueError)):
        _attitude().step(dt_us, ENV, ON)  # type: ignore[arg-type]


def test_reset_requests_named_streams() -> None:
    rng = RngFactory(7)
    Attitude(CONFIG, AttitudeInitial()).reset(rng)
    assert list(rng.stream_seeds) == [DISTURBANCE_STREAM, NOISE_STREAM]
    assert DISTURBANCE_STREAM == "spacecraft.attitude.disturbance"


@pytest.mark.parametrize(
    ("initial", "state"),
    [
        (AttitudeInitial(pointing_error_deg=45.0, rate_dps=1.0), DETUMBLING),
        (AttitudeInitial(pointing_error_deg=10.0, rate_dps=5.0), TUMBLING),
        (AttitudeInitial(pointing_error_deg=1.0, rate_dps=0.0), STABILIZED),
        (AttitudeInitial(pointing_error_deg=30.0, rate_dps=0.0), DETUMBLING),
    ],
)
def test_starts_from_starting_record(initial: AttitudeInitial, state: AttitudeState) -> None:
    snap = _attitude(initial).snapshot()
    assert snap.truth.pointing_error_deg == initial.pointing_error_deg
    assert snap.truth.rate_dps == initial.rate_dps
    assert snap.truth.state is state
    assert snap.truth.control_power_w == 0.0
    assert snap.readings == AttitudeReadings(
        pointing_error_deg=initial.pointing_error_deg, rate_dps=initial.rate_dps, state=state
    )


def test_reset_returns_to_starting_state() -> None:
    attitude = _attitude()
    before = attitude.snapshot()
    first = _run(attitude, 100)
    assert attitude.snapshot() != before
    attitude.reset(RngFactory(1))
    assert attitude.snapshot() == before
    assert _run(attitude, 100) == first


def test_zero_dt_changes_nothing_physical() -> None:
    attitude = _attitude()
    before = attitude.snapshot().truth
    attitude.step(0, ENV, ON)
    after = attitude.snapshot().truth
    assert after.pointing_error_deg == before.pointing_error_deg
    assert after.rate_dps == before.rate_dps


def test_large_step_stays_in_range() -> None:
    attitude = _attitude(AttitudeInitial(pointing_error_deg=170.0, rate_dps=50.0))
    attitude.step(1_000_000_000, ENV, ON)
    truth = attitude.snapshot().truth
    assert 0.0 <= truth.pointing_error_deg <= 180.0
    assert truth.rate_dps >= 0.0
    attitude.step(1_000_000_000, ENV, OFF)
    assert 0.0 <= attitude.snapshot().truth.pointing_error_deg <= 180.0


# --- Dynamics --------------------------------------------------------------------------


def test_stabilizes_from_default_initial_rate() -> None:
    attitude = _attitude()
    assert attitude.snapshot().truth.state is DETUMBLING
    snaps = _run(attitude, 10 * TICKS_PER_MINUTE)
    first = next(i for i, s in enumerate(snaps) if s.truth.state is STABILIZED)
    assert first < 5 * TICKS_PER_MINUTE
    for snap in snaps[first:]:
        assert snap.truth.state is STABILIZED
        assert snap.readings.state is STABILIZED
        assert snap.truth.pointing_error_deg <= CONFIG.stabilized_exit_pointing_error_deg
        assert snap.truth.rate_dps < CONFIG.stabilized_exit_rate_dps


def test_deterministic_for_a_given_seed() -> None:
    assert _run(_attitude(seed=5), 2000) == _run(_attitude(seed=5), 2000)


def test_different_seed_changes_disturbance_and_noise() -> None:
    a = _run(_attitude(seed=5), 200)
    b = _run(_attitude(seed=6), 200)
    assert [s.truth.rate_dps for s in a] != [s.truth.rate_dps for s in b]
    assert [s.readings.pointing_error_deg for s in a] != [s.readings.pointing_error_deg for s in b]


def test_values_stay_in_range_while_tumbling() -> None:
    attitude = _attitude(AttitudeInitial(pointing_error_deg=179.0, rate_dps=30.0))
    errors = []
    for snap in _run(attitude, 3000, controls=OFF):
        for record in (snap.truth, snap.readings):
            assert 0.0 <= record.pointing_error_deg <= 180.0
            assert record.rate_dps >= 0.0
        errors.append(snap.truth.pointing_error_deg)
    # A tumbling spacecraft sweeps its pointing error across the whole range.
    assert min(errors) < 10.0
    assert max(errors) > 170.0


def test_tumbles_with_control_off() -> None:
    attitude = _attitude(AttitudeInitial(pointing_error_deg=0.0, rate_dps=0.0))
    assert attitude.snapshot().truth.state is STABILIZED
    snaps = _run(attitude, 30 * TICKS_PER_MINUTE, controls=OFF)
    states = [s.truth.state for s in snaps]
    assert states[-1] is TUMBLING
    # Without correction the rate drifts upward and the pointing error grows.
    assert snaps[-1].truth.rate_dps > snaps[len(snaps) // 2].truth.rate_dps
    assert max(s.truth.pointing_error_deg for s in snaps[:6000]) > 10.0
    # Once tumbling, it stays tumbling while control is off.
    first = states.index(TUMBLING)
    assert all(state is TUMBLING for state in states[first:])
    assert all(s.truth.control_power_w == 0.0 for s in snaps)


def test_detumbles_to_stabilized_once_control_is_switched_on() -> None:
    attitude = _attitude(AttitudeInitial(pointing_error_deg=120.0, rate_dps=5.0))
    tumbling = _run(attitude, 100, controls=OFF)
    assert all(s.truth.state is TUMBLING for s in tumbling)
    snaps = _run(attitude, 15 * TICKS_PER_MINUTE)
    states = [s.truth.state for s in snaps]
    # TUMBLING, then DETUMBLING, then STABILIZED for good: no other transitions.
    transitions = [b for a, b in zip([TUMBLING, *states], states, strict=False) if a is not b]
    assert transitions == [DETUMBLING, STABILIZED]
    detumbling = states.index(DETUMBLING)
    assert snaps[detumbling].truth.rate_dps <= CONFIG.tumbling_exit_rate_dps
    assert snaps[detumbling - 1].truth.rate_dps > CONFIG.tumbling_exit_rate_dps
    assert snaps[-1].readings.state is STABILIZED


def test_disturbance_is_not_scaled_by_sensor_noise_scale() -> None:
    truths = []
    for scale in (0.0, 1.0, 2.0):
        env = EnvironmentState(sensor_noise_scale=scale)
        truths.append([s.truth for s in _run(_attitude(seed=3), 500, env)])
    assert truths[0] == truths[1] == truths[2]
    rates = [t.rate_dps for t in truths[0]]
    assert len(set(rates)) == len(rates)  # the disturbance acts at every scale


# --- Control draw ----------------------------------------------------------------------


def test_control_draw_follows_controls() -> None:
    config = dataclasses.replace(CONFIG, control_power_w=1.25)
    attitude = _attitude(config=config)
    assert attitude.snapshot().truth.control_power_w == 0.0
    assert _run(attitude, 1)[0].truth.control_power_w == 1.25
    assert _run(attitude, 1, controls=OFF)[0].truth.control_power_w == 0.0
    assert _run(attitude, 1)[0].truth.control_power_w == 1.25


# --- Hysteresis ------------------------------------------------------------------------


def _states(
    start: AttitudeState,
    values: list[tuple[float, float]],
    control_enabled: bool = True,
) -> list[AttitudeState]:
    state = start
    out = []
    for rate, error in values:
        state = next_state(CONFIG, state, rate, error, control_enabled)
        out.append(state)
    return out


def test_state_rules() -> None:
    # Above the high threshold: always TUMBLING.
    for start in AttitudeState:
        assert next_state(CONFIG, start, 2.01, 0.0, True) is TUMBLING
    # TUMBLING leaves only with control on and the rate at or below the exit threshold.
    assert next_state(CONFIG, TUMBLING, 1.5, 90.0, True) is DETUMBLING
    assert next_state(CONFIG, TUMBLING, 1.51, 90.0, True) is TUMBLING
    assert next_state(CONFIG, TUMBLING, 1.0, 90.0, False) is TUMBLING
    assert next_state(CONFIG, TUMBLING, 0.1, 1.0, True) is STABILIZED
    # STABILIZED needs a low rate and a small pointing error.
    assert next_state(CONFIG, DETUMBLING, 0.19, 5.0, True) is STABILIZED
    assert next_state(CONFIG, DETUMBLING, 0.2, 1.0, True) is DETUMBLING
    assert next_state(CONFIG, DETUMBLING, 0.1, 5.01, True) is DETUMBLING
    # STABILIZED is left above the exit thresholds.
    assert next_state(CONFIG, STABILIZED, 0.4, 10.0, True) is STABILIZED
    assert next_state(CONFIG, STABILIZED, 0.41, 1.0, True) is DETUMBLING
    assert next_state(CONFIG, STABILIZED, 0.1, 10.01, True) is DETUMBLING


@pytest.mark.parametrize(
    ("start", "values", "expected"),
    [
        # Rate hovering at the tumbling threshold, after one excursion above it.
        (DETUMBLING, [(2.1, 50.0), (1.9, 50.0)] * 10, TUMBLING),
        # Rate hovering at the tumbling exit threshold.
        (TUMBLING, [(1.4, 50.0), (1.6, 50.0)] * 10, DETUMBLING),
        # Rate hovering at the stabilized entry threshold.
        (DETUMBLING, [(0.19, 1.0), (0.21, 1.0)] * 10, STABILIZED),
        # Rate hovering at the stabilized exit threshold.
        (STABILIZED, [(0.41, 1.0), (0.39, 1.0)] * 10, DETUMBLING),
        # Pointing error hovering at the entry threshold.
        (DETUMBLING, [(0.1, 4.9), (0.1, 5.1)] * 10, STABILIZED),
        # Pointing error hovering at the exit threshold.
        (STABILIZED, [(0.1, 10.1), (0.1, 9.9)] * 10, DETUMBLING),
    ],
)
def test_no_chatter_at_thresholds(
    start: AttitudeState, values: list[tuple[float, float]], expected: AttitudeState
) -> None:
    states = _states(start, values)
    assert states[0] is expected
    assert all(state is expected for state in states)


def test_readings_state_follows_reported_values() -> None:
    attitude = _attitude(AttitudeInitial(pointing_error_deg=4.0, rate_dps=0.1))
    for snap in _run(attitude, 200, EnvironmentState(sensor_noise_scale=0.0)):
        assert snap.readings.state is snap.truth.state
    # With large noise, the reported state can differ from the true state, and is
    # decided from the reported values.
    noisy = dataclasses.replace(CONFIG, pointing_noise_deg=4.0)
    attitude = _attitude(AttitudeInitial(pointing_error_deg=4.0, rate_dps=0.1), noisy)
    snaps = _run(attitude, 500)
    assert any(s.readings.state is not s.truth.state for s in snaps)
    for snap in snaps:
        assert snap.readings.state in (STABILIZED, DETUMBLING)


# --- Readings: noise and freeze --------------------------------------------------------


def test_noise_scale_zero_reports_exact_values() -> None:
    env = EnvironmentState(sensor_noise_scale=0.0)
    for snap in _run(_attitude(), 1000, env):
        assert snap.readings.pointing_error_deg == snap.truth.pointing_error_deg
        assert snap.readings.rate_dps == snap.truth.rate_dps
        assert snap.readings.state is snap.truth.state


def test_noise_scales_with_sensor_noise_scale() -> None:
    initial = AttitudeInitial(pointing_error_deg=90.0, rate_dps=1.0)
    nominal = _run(_attitude(initial, seed=9), 300, EnvironmentState(sensor_noise_scale=1.0))
    doubled = _run(_attitude(initial, seed=9), 300, EnvironmentState(sensor_noise_scale=2.0))
    pointing_1 = [s.readings.pointing_error_deg - s.truth.pointing_error_deg for s in nominal]
    pointing_2 = [s.readings.pointing_error_deg - s.truth.pointing_error_deg for s in doubled]
    rate_1 = [s.readings.rate_dps - s.truth.rate_dps for s in nominal]
    rate_2 = [s.readings.rate_dps - s.truth.rate_dps for s in doubled]
    # Same truth and the same noise draws: the noise is exactly twice as large.
    assert pointing_2 == pytest.approx([2.0 * x for x in pointing_1], abs=1e-9)
    assert rate_2 == pytest.approx([2.0 * x for x in rate_1], abs=1e-9)
    assert any(x != 0.0 for x in pointing_1)
    spread = sum(x * x for x in pointing_1) / len(pointing_1)
    assert 0.5 * 0.5 * 0.6 < spread < 0.5 * 0.5 * 1.5


def test_freeze_holds_readings_while_truth_evolves() -> None:
    attitude = _attitude(AttitudeInitial(pointing_error_deg=90.0, rate_dps=1.5))
    _run(attitude, 50)
    held = attitude.snapshot().readings
    frozen = _run(attitude, 200, controls=FROZEN)
    assert all(s.readings == held for s in frozen)
    truths = [s.truth for s in frozen]
    assert len({t.pointing_error_deg for t in truths}) == len(truths)
    assert truths[-1].state is not held.state or truths[-1].rate_dps != held.rate_dps
    # Live readings resume on release, tracking the evolved truth.
    live = _run(attitude, 20)
    assert all(s.readings != held for s in live)
    for snap in live:
        assert abs(snap.readings.pointing_error_deg - snap.truth.pointing_error_deg) <= 3.0
    zero = _run(attitude, 1, EnvironmentState(sensor_noise_scale=0.0))[0]
    assert zero.readings.pointing_error_deg == zero.truth.pointing_error_deg


def test_freeze_of_other_subsystems_does_not_hold_attitude() -> None:
    attitude = _attitude()
    _run(attitude, 5)
    held = attitude.snapshot().readings
    other = SpacecraftControls(frozen_sensors=frozenset({"power", "thermal"}))
    assert all(s.readings != held for s in _run(attitude, 5, controls=other))


# --- Story #40: attitude in a stack over three orbits ----------------------------------

STORY_ORBITS = 3


def _story_stack(
    initial: AttitudeInitial, seed: int, orbits: int
) -> tuple[SubsystemStack, NominalEnvironment, SimClock, int]:
    board = SnapshotBoard()
    power = Power(NOMINAL_CONFIG.power, DEFAULT_INITIAL_STATE.power, reader=board)
    attitude = Attitude(NOMINAL_CONFIG.attitude, initial)
    # Fakes for the subsystems power may read on the board (#36), so the stack holds a
    # snapshot for every subsystem.
    fakes = [
        FakeSubsystem(name, default_snapshot(name)) for name in ("thermal", "payload", "comms")
    ]
    stack = SubsystemStack([power, attitude, *fakes], board=board)
    stack.reset(RngFactory(seed))
    environment = NominalEnvironment()
    clock = SimClock(TICK_US)
    ticks = orbits * environment.orbit_period_us // TICK_US
    return stack, environment, clock, ticks


def _story_run(
    initial: AttitudeInitial, seed: int, orbits: int = STORY_ORBITS
) -> dict[str, object]:
    """Run the story stack for ``orbits`` orbits and summarize what the tests need."""
    stack, environment, clock, ticks = _story_stack(initial, seed, orbits)
    ticks_per_orbit = ticks // orbits
    reported_first_orbit = ""
    digest = hashlib.sha256()
    reported = hashlib.sha256()
    first_stabilized = None
    left_stabilized = False
    generation_tumbling: list[float] = []
    generation_stabilized_sunlit: list[float] = []
    previous_state = None
    for tick in range(ticks):
        env = environment.state_at(clock.now_us)
        stack.step(clock.tick_us, env, ON)
        clock.advance_one_tick()
        state = stack.snapshot()
        attitude = state.get("attitude", AttitudeSnapshot)
        power = state.get("power", PowerSnapshot)
        # Pointing quality is present in every snapshot for downstream subsystems.
        assert 0.0 <= attitude.truth.pointing_error_deg <= 180.0
        assert 0.0 <= attitude.readings.pointing_error_deg <= 180.0
        digest.update(repr(state).encode())
        reported.update(repr(attitude.readings).encode())
        stabilized = attitude.truth.state is STABILIZED and attitude.readings.state is STABILIZED
        if first_stabilized is None:
            if stabilized:
                first_stabilized = tick
            elif env.sunlit and previous_state is TUMBLING:
                # Power reads attitude of the previous tick (it steps first).
                generation_tumbling.append(power.truth.generation_w)
        else:
            if not stabilized:
                left_stabilized = True
            # Power reads attitude of the previous tick (it steps first).
            if env.sunlit and previous_state is STABILIZED:
                generation_stabilized_sunlit.append(power.truth.generation_w)
        previous_state = attitude.truth.state
        if tick + 1 == ticks_per_orbit:
            reported_first_orbit = reported.hexdigest()
    return {
        "ticks": ticks,
        "digest": digest.hexdigest(),
        "reported_first_orbit": reported_first_orbit,
        "first_stabilized": first_stabilized,
        "left_stabilized": left_stabilized,
        "generation_tumbling": generation_tumbling,
        "generation_stabilized_sunlit": generation_stabilized_sunlit,
    }


@pytest.fixture(scope="module")
def story_default() -> dict[str, object]:
    return _story_run(DEFAULT_INITIAL_STATE.attitude, seed=40)


@pytest.mark.slow
def test_story_reaches_and_stays_stabilized(story_default: dict[str, object]) -> None:
    assert story_default["ticks"] == STORY_ORBITS * 92 * TICKS_PER_MINUTE
    first = story_default["first_stabilized"]
    assert isinstance(first, int)
    assert first < 10 * TICKS_PER_MINUTE
    assert story_default["left_stabilized"] is False


@pytest.mark.slow
def test_story_is_deterministic(story_default: dict[str, object]) -> None:
    again = _story_run(DEFAULT_INITIAL_STATE.attitude, seed=40)
    assert again["digest"] == story_default["digest"]
    # A different seed changes the noisy reported values (checked over the first orbit
    # to keep the suite fast).
    other = _story_run(DEFAULT_INITIAL_STATE.attitude, seed=41, orbits=1)
    assert other["reported_first_orbit"] != story_default["reported_first_orbit"]


@pytest.mark.slow
def test_story_solar_generation_follows_pointing() -> None:
    # Start with a high rate and the arrays facing away from the sun, in sunlight. One
    # orbit is enough to see the drop and the recovery; the three-orbit run above
    # covers staying stabilized.
    run = _story_run(AttitudeInitial(pointing_error_deg=150.0, rate_dps=10.0), seed=40, orbits=1)
    solar_w = NOMINAL_CONFIG.power.solar_array_w
    tumbling = run["generation_tumbling"]
    stabilized = run["generation_stabilized_sunlit"]
    assert isinstance(tumbling, list)
    assert isinstance(stabilized, list)
    assert run["left_stabilized"] is False
    # While tumbling, the pointing error sweeps 0..180°: generation drops, to zero
    # beyond 90°.
    assert tumbling
    assert min(tumbling) == 0.0
    assert sum(tumbling) / len(tumbling) < 0.6 * solar_w
    # Once STABILIZED, generation recovers to near the full array output.
    assert len(stabilized) > 50 * TICKS_PER_MINUTE
    assert min(stabilized) > 0.97 * solar_w
