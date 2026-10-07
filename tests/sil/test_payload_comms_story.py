"""Story-level integration test for payload and comms (story #42, delivered with #44).

The real ``Power``, ``Thermal``, ``Attitude``, ``Payload``, and ``Comms`` run in one
``SubsystemStack`` sharing a ``SnapshotBoard``, driven by ``NominalEnvironment`` (its
defaults: 92-minute orbit, 35% eclipse) and a ``SimClock`` at the default 100 ms tick,
for three full orbits, with the default settings and starting state, the payload
commanded on (``controls.payload.enabled``), and the radio ``RX_TX``.

The stack runs without a flight computer, on purpose: the payload is commanded on for
all three orbits, which no flight computer mode does (DOWNLINK switches it off, #47), so
this run covers acquisition against its inhibits throughout. Two consequences:

- **Nothing is downlinked.** Comms sends nothing (``sent_bytes == 0`` every tick):
  moving chunks into DATA frames within the transmit capacity is the flight computer's
  downlink (#56). Story #42's "bytes sent minus bytes released is never more than one
  tick's worth" and the radio-transmit inhibits are verified with the real flight
  computer in ``test_downlink_story.py`` (three orbits of the reference profile through
  ``SilTarget``), ``test_downlink_sil.py``, and ``tests/unit/test_downlink.py``.
- **Releases are scripted.** Every :data:`RELEASE_EVERY_TICKS` ticks the test releases
  every whole chunk stored, so the buffer never fills and acquisition depends only on
  the inhibits. Released data is not sent anywhere; the release exercises the data
  accounting.

Verified here, at every tick:

- **Data accounting:** ``total_produced_bytes == buffered_bytes +
  total_released_bytes`` (comms holds no data, so nothing else can hold bytes).
- **Payload inhibits** with the real subsystems: the payload acquires exactly when no
  power or thermal flag is set and attitude reports ``STABILIZED`` (the readings it
  read this tick), and never while any inhibit holds.
- **Comms:** radio ``RX_TX``, receiver and transmitter on, the configured capacity,
  and nothing sent.
- **Power** sees the payload, attitude-control, transmit, and heater draws one tick
  late.
- **Determinism:** two stacks with the same seed produce identical ``SpacecraftState``
  values; a stack with a different seed reports different noisy readings.

A second scenario forces an inhibit: a ``battery_drain`` extra load (as ``SilTarget``
would merge it, #60) drives the SOC estimate below the low-battery threshold, the
payload stops acquiring, and acquisition resumes once the drain is removed and the flag
clears.
"""

from dataclasses import dataclass

import pytest

from pocketsat.core.clock import SimClock
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    Attitude,
    AttitudeSnapshot,
    AttitudeState,
    Comms,
    CommsSnapshot,
    Payload,
    PayloadControls,
    PayloadSnapshot,
    PayloadState,
    Power,
    PowerInitial,
    PowerSnapshot,
    RadioControls,
    RadioMode,
    SnapshotBoard,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    SubsystemStack,
    Thermal,
    ThermalSnapshot,
)

# Multi-orbit story run: skipped by a plain `pytest`, run with `pytest -m slow`.
pytestmark = pytest.mark.slow

ORBITS = 3
RELEASE_EVERY_TICKS = 3_000
"""Scripted release period, ticks (300 s): the buffer (65 536 bytes at 100 bytes/s)
would take about 655 s to fill."""

DRAIN_W = 10.0
"""``battery_drain`` extra load in the forced-inhibit scenario, watts."""


def _stack(seed: int, initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE) -> SubsystemStack:
    config = NOMINAL_CONFIG
    board = SnapshotBoard()
    stack = SubsystemStack(
        [
            Power(config.power, initial.power, reader=board),
            Thermal(config.thermal, initial.thermal, reader=board),
            Attitude(config.attitude, initial.attitude),
            Payload(config.payload, initial.payload, reader=board),
            Comms(config.comms),
        ],
        board=board,
    )
    stack.reset(RngFactory(seed))
    return stack


def _controls(release: int | None = None, extra_load_w: float = 0.0) -> SpacecraftControls:
    return SpacecraftControls(
        payload=PayloadControls(enabled=True, release_through_chunk_id=release),
        radio=RadioControls(mode=RadioMode.RX_TX),
        extra_load_w=extra_load_w,
    )


def _release(state: SpacecraftState) -> int | None:
    """Release every whole chunk stored, if any (the scripted stand-in for #56)."""
    payload = state.get("payload", PayloadSnapshot).truth
    if payload.next_chunk_id > payload.oldest_unreleased_chunk_id:
        return payload.next_chunk_id - 1
    return None


def _inhibited(state: SpacecraftState) -> bool:
    """The payload's documented inhibits, from this tick's readings (#43)."""
    power = state.get("power", PowerSnapshot).readings
    thermal = state.get("thermal", ThermalSnapshot).readings
    attitude = state.get("attitude", AttitudeSnapshot).readings
    return (
        power.low_battery
        or power.critical_battery
        or thermal.over_temp
        or thermal.under_temp
        or attitude.state is not AttitudeState.STABILIZED
    )


def _check_tick(previous: SpacecraftState, state: SpacecraftState, extra_load_w: float) -> bool:
    """Assert the per-tick story invariants; return whether the payload was inhibited."""
    payload = state.get("payload", PayloadSnapshot).truth
    capacity = payload.buffer_capacity_bytes

    # Data accounting: nothing lost or duplicated, and comms holds none.
    assert payload.total_produced_bytes == payload.buffered_bytes + payload.total_released_bytes

    # Acquisition exactly when the inhibits allow (the buffer never fills here).
    before = previous.get("payload", PayloadSnapshot).truth
    assert before.buffered_bytes < capacity
    inhibited = _inhibited(state)
    if inhibited:
        assert payload.state is PayloadState.IDLE
        assert payload.total_produced_bytes == before.total_produced_bytes
    else:
        assert payload.state is PayloadState.ACQUIRING

    # Comms: RX_TX, full capacity, nothing sent without a flight computer.
    comms = state.get("comms", CommsSnapshot).truth
    assert comms.radio_mode is RadioMode.RX_TX
    assert comms.receiver_on and comms.transmitter_on
    assert comms.transmit_capacity_bytes == NOMINAL_CONFIG.comms.transmit_capacity_bytes
    assert comms.sent_bytes == 0
    assert comms.uplink_lost_count == 0 and comms.outbound_suppressed_count == 0

    # Power sees every subsystem's draw one tick late.
    load_w = state.get("power", PowerSnapshot).truth.total_load_w
    assert load_w == pytest.approx(
        NOMINAL_CONFIG.power.base_load_w
        + before.power_w
        + previous.get("attitude", AttitudeSnapshot).truth.control_power_w
        + previous.get("comms", CommsSnapshot).truth.transmit_power_w
        + previous.get("thermal", ThermalSnapshot).truth.heater_power_w
        + extra_load_w
    )
    return inhibited


@dataclass
class Run:
    inhibited_ticks: int = 0
    acquiring_ticks: int = 0
    resumes: int = 0
    releases: int = 0
    readings_differ: bool = False
    final: SpacecraftState | None = None


def _nominal_run() -> Run:
    environment = NominalEnvironment()
    clock = SimClock()
    ticks_per_orbit = environment.orbit_period_us // clock.tick_us
    assert ticks_per_orbit * clock.tick_us == environment.orbit_period_us
    stacks = [_stack(seed) for seed in (1, 1, 2)]
    previous = [stack.snapshot() for stack in stacks]
    run = Run()
    was_inhibited = True

    for tick in range(ORBITS * ticks_per_orbit):
        env = environment.sample(clock)
        release_tick = tick > 0 and tick % RELEASE_EVERY_TICKS == 0
        for stack, before in zip(stacks, previous, strict=True):
            # Each stack's scripted release follows its own state (seeds may differ).
            release = _release(before) if release_tick else None
            stack.step(clock.tick_us, env, _controls(release=release))
        clock.advance_one_tick()
        states = [stack.snapshot() for stack in stacks]
        state, same, other = states

        assert state == same  # deterministic: same seed, identical SpacecraftState
        run.readings_differ = run.readings_differ or (
            other.get("power", PowerSnapshot).readings != state.get("power", PowerSnapshot).readings
        )
        inhibited = _check_tick(previous[0], state, 0.0)
        _check_tick(previous[2], other, 0.0)  # the invariants hold for any seed

        run.releases += release_tick
        run.inhibited_ticks += inhibited
        run.acquiring_ticks += not inhibited
        run.resumes += was_inhibited and not inhibited
        was_inhibited = inhibited
        previous = states
    run.final = previous[0]
    return run


@pytest.fixture(scope="module")
def nominal() -> Run:
    return _nominal_run()


def test_data_accounting_balances_and_data_was_released(nominal: Run) -> None:
    # The balance is asserted every tick in the run; here, that it was exercised.
    assert nominal.final is not None
    payload = nominal.final.get("payload", PayloadSnapshot).truth
    assert nominal.releases > 0
    assert payload.total_released_bytes > 0
    assert payload.total_produced_bytes == payload.buffered_bytes + payload.total_released_bytes


def test_payload_acquires_once_attitude_stabilizes(nominal: Run) -> None:
    # The run starts DETUMBLING (inhibited), then acquires for most of three orbits.
    assert nominal.inhibited_ticks > 0
    assert nominal.resumes >= 1
    assert nominal.acquiring_ticks > nominal.inhibited_ticks


def test_comms_sends_nothing_without_a_flight_computer(nominal: Run) -> None:
    # Checked every tick in the run; the final state confirms the counters.
    assert nominal.final is not None
    comms = nominal.final.get("comms", CommsSnapshot).truth
    assert comms.sent_bytes == 0
    assert comms.transmit_power_w == NOMINAL_CONFIG.comms.transmitter_on_power_w


def test_a_different_seed_changes_the_noisy_readings(nominal: Run) -> None:
    assert nominal.readings_differ


def test_battery_drain_inhibits_acquisition_until_the_flag_clears() -> None:
    environment = NominalEnvironment()
    clock = SimClock()
    ticks_per_orbit = environment.orbit_period_us // clock.tick_us
    initial = SpacecraftInitialState(power=PowerInitial(soc=0.4))
    stack = _stack(1, initial)
    previous = stack.snapshot()
    phases: list[str] = []
    phase = "settling"  # until attitude stabilizes and the payload acquires
    drain_ticks = 0

    for tick in range(3 * ticks_per_orbit):
        extra_load_w = DRAIN_W if phase == "draining" else 0.0
        release = _release(previous) if tick > 0 and tick % RELEASE_EVERY_TICKS == 0 else None
        controls = _controls(release=release, extra_load_w=extra_load_w)
        stack.step(clock.tick_us, environment.sample(clock), controls)
        clock.advance_one_tick()
        state = stack.snapshot()
        inhibited = _check_tick(previous, state, extra_load_w)
        low = state.get("power", PowerSnapshot).readings.low_battery
        acquiring = state.get("payload", PayloadSnapshot).truth.state is PayloadState.ACQUIRING
        if phase == "settling" and acquiring:
            phase = "draining"
        elif phase == "draining":
            drain_ticks += 1
            if low:
                assert inhibited and not acquiring  # stops in the tick the flag sets
                phase = "recovering"
        elif phase == "recovering":
            if low:
                assert not acquiring
            elif acquiring:
                phases.append("resumed")
                break
        if not phases or phases[-1] != phase:
            phases.append(phase)
        previous = state

    assert phases == ["settling", "draining", "recovering", "resumed"]
    assert drain_ticks > 0
