"""Tests for the communications subsystem (#44).

The ``transmitter_off`` fault is represented by ``SilTarget`` (#60) as a downgrade of
``controls.radio.mode`` from ``RX_TX`` to ``RX_ONLY``, so the transmitter-failure tests
step comms in ``RX_ONLY`` after ``RX_TX``.
"""

import dataclasses
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import (
    NO_RADIO_TRAFFIC,
    NOMINAL_CONFIG,
    Comms,
    CommsConfig,
    CommsSnapshot,
    CommsTruth,
    Power,
    PowerConfig,
    PowerInitial,
    PowerSnapshot,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SnapshotBoard,
    SpacecraftControls,
    Subsystem,
    SubsystemStack,
    transmit_draw_w,
)
from pocketsat.spacecraft.fakes import FakeSubsystem, default_snapshot
from pocketsat.targets.base import EnvironmentState

ENV = EnvironmentState()
TICK_US = 100_000
CONFIG = CommsConfig(
    transmit_capacity_bytes=200, transmitter_on_power_w=1.5, transmit_power_per_byte_w=0.01
)


def radio(mode: RadioMode) -> SpacecraftControls:
    return SpacecraftControls(radio=RadioControls(mode=mode))


def stepped(mode: RadioMode, config: CommsConfig = CONFIG) -> CommsSnapshot:
    comms = Comms(config)
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, radio(mode))
    return comms.snapshot()


# --- Construction and reset -----------------------------------------------------------


def test_implements_the_subsystem_protocol() -> None:
    comms = Comms(CONFIG)
    assert isinstance(comms, Subsystem)
    assert comms.name == "comms"


def test_rejects_a_wrong_config() -> None:
    with pytest.raises(TypeError, match="CommsConfig"):
        Comms(object())  # type: ignore[arg-type]


def test_reset_state_is_off_with_no_draw() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    truth = comms.snapshot().truth
    assert truth == CommsTruth(
        radio_mode=RadioMode.OFF,
        receiver_on=False,
        transmitter_on=False,
        transmit_capacity_bytes=0,
        previous_tick_sent_bytes=0,
        uplink_lost_count=0,
        outbound_suppressed_count=0,
        transmit_power_w=0.0,
    )


def test_reset_returns_to_the_initial_state() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    initial = comms.snapshot()
    comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    assert comms.snapshot() != initial
    comms.reset(RngFactory(2))  # the seed does not matter: no randomness
    assert comms.snapshot() == initial


def test_no_randomness_any_seed_gives_the_same_run() -> None:
    runs = []
    for seed in (1, 2, 3):
        comms = Comms(CONFIG)
        comms.reset(RngFactory(seed))
        run = []
        for mode in (RadioMode.RX_TX, RadioMode.OFF, RadioMode.RX_ONLY, RadioMode.RX_TX):
            comms.step(TICK_US, ENV, radio(mode))
            run.append(comms.snapshot())
        runs.append(run)
    assert runs[0] == runs[1] == runs[2]


def test_rejects_a_bad_dt() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    with pytest.raises(ValueError):
        comms.step(-1, ENV, radio(RadioMode.RX_TX))
    with pytest.raises(TypeError):
        comms.step(0.1, ENV, radio(RadioMode.RX_TX))  # type: ignore[arg-type]


# --- Radio modes ----------------------------------------------------------------------


@pytest.mark.parametrize(
    ("mode", "receiver", "transmitter", "capacity", "draw"),
    [
        (RadioMode.OFF, False, False, 0, 0.0),
        (RadioMode.RX_ONLY, True, False, 0, 0.0),
        (RadioMode.RX_TX, True, True, 200, 1.5),
    ],
)
def test_each_radio_mode(
    mode: RadioMode, receiver: bool, transmitter: bool, capacity: int, draw: float
) -> None:
    truth = stepped(mode).truth
    assert truth.radio_mode is mode
    assert truth.receiver_on is receiver
    assert truth.transmitter_on is transmitter
    assert truth.transmit_capacity_bytes == capacity
    assert truth.transmit_power_w == draw


def test_capacity_comes_from_the_config() -> None:
    config = dataclasses.replace(CONFIG, transmit_capacity_bytes=77)
    assert stepped(RadioMode.RX_TX, config).truth.transmit_capacity_bytes == 77


def test_capacity_does_not_scale_with_the_tick_length() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    for dt_us in (1_000, TICK_US, 1_000_000):
        comms.step(dt_us, ENV, radio(RadioMode.RX_TX))
        assert comms.snapshot().truth.transmit_capacity_bytes == 200


def test_follows_the_mode_every_tick() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    sequence = [RadioMode.RX_TX, RadioMode.RX_ONLY, RadioMode.OFF, RadioMode.RX_TX]
    for mode in sequence:
        comms.step(TICK_US, ENV, radio(mode))
        assert comms.snapshot().truth.radio_mode is mode


def test_snapshot_is_reused_while_the_mode_is_unchanged() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    first = comms.snapshot()
    comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    assert comms.snapshot() is first


@pytest.mark.parametrize("mode", list(RadioMode))
def test_readings_equal_truth(mode: RadioMode) -> None:
    snap = stepped(mode)
    assert isinstance(snap, CommsSnapshot)
    assert dataclasses.asdict(snap.readings) == dataclasses.asdict(snap.truth)


def test_default_controls_give_rx_tx_with_the_nominal_config() -> None:
    comms = Comms(NOMINAL_CONFIG.comms)
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, SpacecraftControls())
    truth = comms.snapshot().truth
    assert truth.radio_mode is RadioMode.RX_TX
    assert truth.transmit_capacity_bytes == 120
    assert truth.transmit_power_w == 0.15


# --- Transmitter failure (the transmitter_off fault, merged by #60) -------------------


def test_transmitter_failure_downgrade_keeps_the_receiver_on() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    assert comms.snapshot().truth.transmitter_on
    # SilTarget represents transmitter_off as RX_TX downgraded to RX_ONLY (#60).
    comms.step(TICK_US, ENV, radio(RadioMode.RX_ONLY))
    truth = comms.snapshot().truth
    assert truth.transmit_capacity_bytes == 0
    assert truth.transmitter_on is False
    assert truth.receiver_on is True
    assert truth.transmit_power_w == 0.0
    # Clearing the fault restores the transmitter.
    comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    assert comms.snapshot().truth.transmit_capacity_bytes == 200


# --- Transmit draw formula ------------------------------------------------------------


@pytest.mark.parametrize("sent", [0, 1, 50, 200])
def test_draw_is_fixed_plus_proportional_to_bytes_sent(sent: int) -> None:
    assert transmit_draw_w(CONFIG, True, sent) == pytest.approx(1.5 + 0.01 * sent)


def test_per_byte_term_is_linear_in_bytes_sent() -> None:
    base = transmit_draw_w(CONFIG, True, 0)
    per_byte = [transmit_draw_w(CONFIG, True, n) - base for n in (10, 20, 40, 80)]
    assert per_byte == pytest.approx([0.1, 0.2, 0.4, 0.8])


def test_draw_at_zero_bytes_is_exactly_the_fixed_draw() -> None:
    assert transmit_draw_w(CONFIG, True, 0) == CONFIG.transmitter_on_power_w


def test_no_draw_while_the_transmitter_is_off() -> None:
    assert transmit_draw_w(CONFIG, False, 0) == 0.0


def test_draw_rejects_impossible_traffic() -> None:
    with pytest.raises(ValueError, match="off"):
        transmit_draw_w(CONFIG, False, 1)
    with pytest.raises(ValueError):
        transmit_draw_w(CONFIG, True, -1)
    with pytest.raises(ValueError):
        transmit_draw_w(CONFIG, True, 201)
    with pytest.raises(TypeError):
        transmit_draw_w(CONFIG, True, 1.0)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        transmit_draw_w(CONFIG, True, True)


def test_step_uses_the_formula_with_no_bytes_sent() -> None:
    truth = stepped(RadioMode.RX_TX).truth
    assert truth.previous_tick_sent_bytes == 0
    assert truth.transmit_power_w == transmit_draw_w(CONFIG, True, 0)


# --- Radio traffic (ADR-0007, #98) ----------------------------------------------------


def traffic(
    mode: RadioMode, sent: int = 0, lost: int = 0, suppressed: int = 0
) -> SpacecraftControls:
    """Controls for ``mode`` carrying the previous tick's traffic, as SilTarget merges it."""
    return SpacecraftControls(
        radio=RadioControls(mode=mode),
        radio_traffic=RadioTraffic(
            sent_bytes=sent, uplink_lost_count=lost, outbound_suppressed_count=suppressed
        ),
    )


def run(*controls: SpacecraftControls) -> list[CommsTruth]:
    """A fresh comms stepped with each record in turn; the truth after each step."""
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    truths = []
    for record in controls:
        comms.step(TICK_US, ENV, record)
        truths.append(comms.snapshot().truth)
    return truths


def test_counters_stay_zero_without_traffic() -> None:
    # Default controls carry no traffic, so subsystem tests see none in any mode.
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    for mode in [*RadioMode, *RadioMode]:
        comms.step(TICK_US, ENV, radio(mode))
        truth = comms.snapshot().truth
        counters = (
            truth.previous_tick_sent_bytes,
            truth.uplink_lost_count,
            truth.outbound_suppressed_count,
        )
        assert counters == (0, 0, 0)


def test_controls_default_to_no_traffic() -> None:
    assert SpacecraftControls().radio_traffic == RadioTraffic(0, 0, 0)
    assert SpacecraftControls().radio_traffic is NO_RADIO_TRAFFIC


def test_bytes_are_reported_per_tick_and_charged_with_the_idle_draw() -> None:
    first, second, third = run(
        traffic(RadioMode.RX_TX), traffic(RadioMode.RX_TX, sent=120), traffic(RadioMode.RX_TX)
    )
    assert first.previous_tick_sent_bytes == 0
    assert second.previous_tick_sent_bytes == 120
    # Bit-for-bit transmit_draw_w while the transmitter stays on (ADR-0007 §2).
    assert second.transmit_power_w == transmit_draw_w(CONFIG, True, 120)
    assert second.transmit_power_w == 1.5 + 0.01 * 120
    # Per tick, not a running total: no bytes reported, only the idle draw.
    assert third.previous_tick_sent_bytes == 0
    assert third.transmit_power_w == 1.5


def test_bytes_sent_before_the_transmitter_switched_off_are_still_charged() -> None:
    # transmitter_off in tick N+1 (RX_ONLY): the capacity is 0 and the idle draw is 0,
    # but tick N's bytes were sent with the transmitter on and are reported and paid.
    on, off, after = run(
        traffic(RadioMode.RX_TX), traffic(RadioMode.RX_ONLY, sent=200), traffic(RadioMode.RX_ONLY)
    )
    assert on.transmitter_on
    assert not off.transmitter_on and off.transmit_capacity_bytes == 0
    assert off.previous_tick_sent_bytes == 200 > off.transmit_capacity_bytes
    assert off.transmit_power_w == 0.0 + 0.01 * 200
    assert after.previous_tick_sent_bytes == 0 and after.transmit_power_w == 0.0


def test_lost_and_suppressed_counts_are_running_totals_since_reset() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    steps = [
        traffic(RadioMode.OFF),
        traffic(RadioMode.OFF, lost=2),
        traffic(RadioMode.RX_TX, lost=1),
        traffic(RadioMode.RX_TX, suppressed=3),
        traffic(RadioMode.RX_TX),
        traffic(RadioMode.RX_TX, suppressed=1),
    ]
    totals = []
    for record in steps:
        comms.step(TICK_US, ENV, record)
        truth = comms.snapshot().truth
        totals.append((truth.uplink_lost_count, truth.outbound_suppressed_count))
    assert totals == [(0, 0), (2, 0), (3, 0), (3, 3), (3, 3), (3, 4)]
    comms.reset(RngFactory(1))
    truth = comms.snapshot().truth
    assert (truth.uplink_lost_count, truth.outbound_suppressed_count) == (0, 0)


def test_suppressed_frames_may_be_reported_in_any_mode() -> None:
    # Suppression happens because the capacity was too small, including 0 (transmitter
    # off, or radio OFF): nothing in comms' state contradicts it.
    truths = run(traffic(RadioMode.RX_ONLY), traffic(RadioMode.OFF, suppressed=4))
    assert truths[-1].outbound_suppressed_count == 4


@pytest.mark.parametrize(
    ("previous", "record", "match"),
    [
        (RadioMode.RX_TX, RadioTraffic(sent_bytes=201), "0..200"),
        (RadioMode.RX_ONLY, RadioTraffic(sent_bytes=1), "off"),
        (RadioMode.OFF, RadioTraffic(sent_bytes=13), "off"),
        (RadioMode.RX_TX, RadioTraffic(uplink_lost_count=1), "receiver was on"),
        (RadioMode.RX_ONLY, RadioTraffic(uplink_lost_count=2), "receiver was on"),
    ],
)
def test_traffic_impossible_in_the_previous_tick_raises(
    previous: RadioMode, record: RadioTraffic, match: str
) -> None:
    # Checked against comms' own previous tick (ADR-0007 §4), not the current one: here
    # the current mode would allow it.
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, radio(previous))
    before = comms.snapshot()
    current = RadioMode.OFF if record.uplink_lost_count else RadioMode.RX_TX
    with pytest.raises(ValueError, match=match):
        comms.step(
            TICK_US, ENV, SpacecraftControls(radio=RadioControls(current), radio_traffic=record)
        )
    assert comms.snapshot() is before  # nothing applied, nothing clipped


def test_the_first_step_after_reset_accepts_no_bytes() -> None:
    # Before the first step the radio is OFF, so nothing can have been sent.
    with pytest.raises(ValueError, match="off"):
        run(traffic(RadioMode.RX_TX, sent=1))
    assert run(traffic(RadioMode.RX_TX))[0].previous_tick_sent_bytes == 0


def test_bytes_at_the_previous_capacity_are_accepted_even_when_it_dropped() -> None:
    # The bound is the previous tick's capacity: 200 bytes after RX_TX, then OFF.
    truths = run(
        traffic(RadioMode.RX_TX),
        traffic(RadioMode.RX_TX, sent=200),
        traffic(RadioMode.OFF, sent=200),
    )
    assert [t.previous_tick_sent_bytes for t in truths] == [0, 200, 200]


def test_lost_uplink_is_accepted_while_the_receiver_was_off() -> None:
    truths = run(traffic(RadioMode.OFF), traffic(RadioMode.RX_TX, lost=5))
    assert truths[-1].uplink_lost_count == 5


def test_snapshot_is_rebuilt_when_traffic_arrives_and_reused_without_it() -> None:
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, traffic(RadioMode.RX_TX))
    idle = comms.snapshot()
    comms.step(TICK_US, ENV, traffic(RadioMode.RX_TX, sent=10))
    busy = comms.snapshot()
    assert busy is not idle and busy.truth.previous_tick_sent_bytes == 10
    comms.step(TICK_US, ENV, traffic(RadioMode.RX_TX))
    quiet = comms.snapshot()
    assert quiet == idle and quiet is not busy
    comms.step(TICK_US, ENV, traffic(RadioMode.RX_TX))
    assert comms.snapshot() is quiet


# --- Power sees the transmit draw one tick late ---------------------------------------


def _power_and_comms() -> tuple[SubsystemStack, SnapshotBoard, float]:
    board = SnapshotBoard()
    power_config = PowerConfig()
    power = Power(power_config, PowerInitial(), reader=board)
    comms = Comms(CONFIG)
    fakes: list[FakeSubsystem[Any]] = [
        FakeSubsystem(name, default_snapshot(name)) for name in ("thermal", "attitude", "payload")
    ]
    stack = SubsystemStack([power, *fakes, comms], board=board)
    stack.reset(RngFactory(1))
    other_w = power_config.base_load_w + 0.5  # fake attitude control; heater and payload off
    return stack, board, other_w


def test_power_sees_the_transmit_draw_one_tick_late() -> None:
    stack, board, other_w = _power_and_comms()
    modes = [RadioMode.RX_TX, RadioMode.RX_TX, RadioMode.RX_ONLY, RadioMode.RX_TX, RadioMode.OFF]
    previous_draw_w = board.get("comms", CommsSnapshot).truth.transmit_power_w
    assert previous_draw_w == 0.0
    loads = []
    for mode in modes:
        stack.step(TICK_US, ENV, radio(mode))
        load_w = board.get("power", PowerSnapshot).truth.total_load_w
        assert load_w == pytest.approx(other_w + previous_draw_w)
        loads.append(load_w - other_w)
        previous_draw_w = board.get("comms", CommsSnapshot).truth.transmit_power_w
    assert loads == pytest.approx([0.0, 1.5, 1.5, 0.0, 1.5])


def test_bytes_sent_in_tick_n_reach_comms_in_n_plus_1_and_power_in_n_plus_2() -> None:
    # Frames sent in tick N are handed to comms by the merge of tick N+1 (ADR-0007).
    stack, board, other_w = _power_and_comms()
    records = [
        traffic(RadioMode.RX_TX),  # tick N-1
        traffic(RadioMode.RX_TX),  # tick N: 100 bytes go out (in the flight computer)
        traffic(RadioMode.RX_TX, sent=100),  # tick N+1: comms is told
        traffic(RadioMode.RX_TX),  # tick N+2: power integrates it
        traffic(RadioMode.RX_TX),
    ]
    reported, power_extra = [], []
    for record in records:
        stack.step(TICK_US, ENV, record)
        reported.append(board.get("comms", CommsSnapshot).truth.previous_tick_sent_bytes)
        load_w = board.get("power", PowerSnapshot).truth.total_load_w
        power_extra.append(load_w - other_w)
    assert reported == [0, 0, 100, 0, 0]
    assert power_extra == pytest.approx([0.0, 1.5, 1.5, 1.5 + 0.01 * 100, 1.5])


def test_comms_reads_nothing_from_other_subsystems() -> None:
    # Alone in a stack, with nothing else published, comms steps normally.
    comms = Comms(CONFIG)
    stack = SubsystemStack([comms])
    stack.reset(RngFactory(1))
    stack.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    assert stack.snapshot().get("comms", CommsSnapshot).truth.transmitter_on


# --- Config ---------------------------------------------------------------------------


def test_config_defaults() -> None:
    config = CommsConfig()
    assert config.transmit_capacity_bytes == 120
    assert config.transmitter_on_power_w == 0.15
    assert config.transmit_power_per_byte_w == 0.02
    assert NOMINAL_CONFIG.comms == config


def test_config_accepts_zero() -> None:
    CommsConfig(transmit_capacity_bytes=0, transmitter_on_power_w=0, transmit_power_per_byte_w=0)


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"transmit_capacity_bytes": -1}, ValueError),
        ({"transmit_capacity_bytes": 1.5}, TypeError),
        ({"transmit_capacity_bytes": True}, TypeError),
        ({"transmitter_on_power_w": -0.1}, ValueError),
        ({"transmitter_on_power_w": float("nan")}, ValueError),
        ({"transmitter_on_power_w": "1"}, TypeError),
        ({"transmit_power_per_byte_w": -0.001}, ValueError),
        ({"transmit_power_per_byte_w": float("inf")}, ValueError),
        ({"transmit_power_per_byte_w": None}, TypeError),
    ],
)
def test_config_validation(changes: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        CommsConfig(**changes)  # type: ignore[arg-type]
