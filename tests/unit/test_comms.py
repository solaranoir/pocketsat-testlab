"""Tests for the communications subsystem (#44).

The ``transmitter_off`` fault is represented by ``SilTarget`` (#60) as a downgrade of
``controls.radio.mode`` from ``RX_TX`` to ``RX_ONLY``, so the transmitter-failure tests
step comms in ``RX_ONLY`` after ``RX_TX``.
"""

import copy
import dataclasses
import random
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
    transmit_rate_bytes_per_s=2000, transmitter_on_power_w=1.5, transmit_power_per_byte_w=0.01
)
"""2000 bytes/s: exactly 200 bytes in every 100 ms tick."""


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


def test_capacity_comes_from_the_rate_in_the_config() -> None:
    config = dataclasses.replace(CONFIG, transmit_rate_bytes_per_s=770)
    assert stepped(RadioMode.RX_TX, config).truth.transmit_capacity_bytes == 77


# --- Transmit rate (#122) -------------------------------------------------------------


def capacities(
    config: CommsConfig, dts_us: list[int], modes: list[RadioMode] | None = None
) -> list[int]:
    """A fresh comms stepped once per ``dts_us`` entry; the capacity after each step."""
    comms = Comms(config)
    comms.reset(RngFactory(1))
    out = []
    for n, dt_us in enumerate(dts_us):
        comms.step(dt_us, ENV, radio(modes[n] if modes else RadioMode.RX_TX))
        out.append(comms.snapshot().truth.transmit_capacity_bytes)
    return out


def test_default_rate_is_exactly_120_bytes_in_every_100_ms_tick() -> None:
    # No behaviour change at the default tick (#122): 1200 bytes/s divides exactly.
    assert set(capacities(NOMINAL_CONFIG.comms, [TICK_US] * 10_000)) == {120}


@pytest.mark.parametrize(
    "dt_us", [1_000, 30_000, 33_333, 70_001, TICK_US, 250_000, 1_000_000, 1_234_567]
)
@pytest.mark.parametrize("rate", [1_200, 1_234, 7, 1_000_003])
def test_long_run_rate_equals_the_configured_rate(rate: int, dt_us: int) -> None:
    # Exact at every tick length, including ticks that don't divide evenly: after n
    # ticks the capacities add up to floor(rate * elapsed), so nothing drifts.
    caps = capacities(CommsConfig(transmit_rate_bytes_per_s=rate), [dt_us] * 3_000)
    total = 0
    for n, cap in enumerate(caps, start=1):
        total += cap
        assert total == rate * dt_us * n // 1_000_000
    # Each tick holds the per-tick rate rounded down or up.
    assert set(caps) <= {rate * dt_us // 1_000_000, -(-rate * dt_us // 1_000_000)}


def test_uneven_ticks_carry_the_fraction_of_a_byte() -> None:
    # 1234 bytes/s over 30 ms is 37.02 bytes: 37 per tick, and the carried 0.02 adds a
    # byte every 50th tick.
    caps = capacities(CommsConfig(transmit_rate_bytes_per_s=1234), [30_000] * 100)
    assert [n for n, cap in enumerate(caps) if cap != 37] == [49, 99]
    assert caps[49] == caps[99] == 38
    # The default 1200 bytes/s at 30 ms and at the demo's 1 s tick: still 9600 bit/s.
    assert set(capacities(NOMINAL_CONFIG.comms, [30_000] * 100)) == {36}
    assert capacities(NOMINAL_CONFIG.comms, [1_000_000] * 3) == [1200] * 3


def test_capacity_follows_each_ticks_length() -> None:
    # SilTarget's tick is fixed, but comms does not assume it: with a different length
    # every step the total is still exact.
    dts = [100_000, 30_000, 1_000_000, 1, 70_001, 0, 999_999]
    caps = capacities(CommsConfig(transmit_rate_bytes_per_s=1234), dts)
    assert sum(caps) == 1234 * sum(dts) // 1_000_000
    assert caps[5] == 0  # a zero-length step has no capacity


def test_capacity_is_0_while_the_transmitter_is_off_and_the_carry_is_held() -> None:
    # 5 bytes/s over 100 ms is half a byte per tick. The carry neither grows while the
    # transmitter is off (else the first tick back would hold 3 bytes) nor is discarded
    # (else it would hold 0): the half byte carried from before is completed.
    config = CommsConfig(transmit_rate_bytes_per_s=5)
    on, rx, off = RadioMode.RX_TX, RadioMode.RX_ONLY, RadioMode.OFF
    modes = [on, rx, rx, off, off, rx, on, on, on]
    assert capacities(config, [TICK_US] * len(modes), modes) == [0, 0, 0, 0, 0, 0, 1, 0, 1]


def test_capacity_counts_transmitter_on_time_only() -> None:
    # Over any mix of modes and tick lengths, the capacities add up to the rate times
    # the time the transmitter was on, rounded down.
    rng = random.Random(122)
    comms = Comms(CommsConfig(transmit_rate_bytes_per_s=1234))
    comms.reset(RngFactory(1))
    on_us = total = 0
    for _ in range(2_000):
        dt_us = rng.choice([30_000, 70_001, TICK_US, 1_000_000, rng.randrange(1, 2_000_000)])
        comms.step(dt_us, ENV, radio(rng.choice(list(RadioMode))))
        truth = comms.snapshot().truth
        if truth.transmitter_on:
            on_us += dt_us
        else:
            assert truth.transmit_capacity_bytes == 0
        total += truth.transmit_capacity_bytes
        assert total == 1234 * on_us // 1_000_000


def test_reset_clears_the_carry() -> None:
    # Half a byte is carried after one tick; without the reset the next tick would
    # complete it and hold 1 byte.
    comms = Comms(CommsConfig(transmit_rate_bytes_per_s=5))
    comms.reset(RngFactory(1))
    comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
    assert comms.snapshot().truth.transmit_capacity_bytes == 0
    comms.reset(RngFactory(1))
    for expected in (0, 1, 0, 1):
        comms.step(TICK_US, ENV, radio(RadioMode.RX_TX))
        assert comms.snapshot().truth.transmit_capacity_bytes == expected


def test_a_zero_rate_never_allows_a_byte() -> None:
    config = CommsConfig(transmit_rate_bytes_per_s=0)
    assert set(capacities(config, [TICK_US, 1_000_000, 30_000])) == {0}
    truth = stepped(RadioMode.RX_TX, config).truth
    assert truth.transmitter_on and truth.transmit_power_w == 0.15


def test_traffic_is_checked_against_the_previous_ticks_varying_capacity() -> None:
    # ADR-0007 §4 at uneven ticks: 1234 bytes/s over 30 ms gives 37 or 38 bytes. Every
    # tick sends the previous tick's full capacity (accepted); one byte more is
    # rejected, also where this tick's own capacity (38) would allow it.
    dt_us = 30_000
    caps = capacities(CommsConfig(transmit_rate_bytes_per_s=1234), [dt_us] * 201)
    comms = Comms(CommsConfig(transmit_rate_bytes_per_s=1234))
    comms.reset(RngFactory(1))
    comms.step(dt_us, ENV, radio(RadioMode.RX_TX))
    rejected_within_this_ticks_capacity = 0
    for n in range(1, 201):
        previous = caps[n - 1]
        assert comms.snapshot().truth.transmit_capacity_bytes == previous
        trial = copy.deepcopy(comms)
        before = trial.snapshot()
        with pytest.raises(ValueError, match=f"0..{previous},"):
            trial.step(dt_us, ENV, traffic(RadioMode.RX_TX, sent=previous + 1))
        assert trial.snapshot() is before  # nothing applied
        rejected_within_this_ticks_capacity += previous + 1 <= caps[n]
        comms.step(dt_us, ENV, traffic(RadioMode.RX_TX, sent=previous))
        assert comms.snapshot().truth.previous_tick_sent_bytes == previous
        assert comms.snapshot().truth.transmit_capacity_bytes == caps[n]
    assert rejected_within_this_ticks_capacity == 4  # the four 38-byte ticks


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
    assert transmit_draw_w(CONFIG, True, sent, capacity_bytes=200) == pytest.approx(
        1.5 + 0.01 * sent
    )


def test_per_byte_term_is_linear_in_bytes_sent() -> None:
    base = transmit_draw_w(CONFIG, True, 0, capacity_bytes=200)
    per_byte = [
        transmit_draw_w(CONFIG, True, n, capacity_bytes=200) - base for n in (10, 20, 40, 80)
    ]
    assert per_byte == pytest.approx([0.1, 0.2, 0.4, 0.8])


def test_draw_at_zero_bytes_is_exactly_the_fixed_draw() -> None:
    assert transmit_draw_w(CONFIG, True, 0, capacity_bytes=200) == CONFIG.transmitter_on_power_w


def test_no_draw_while_the_transmitter_is_off() -> None:
    assert transmit_draw_w(CONFIG, False, 0, capacity_bytes=200) == 0.0


def test_draw_rejects_impossible_traffic() -> None:
    with pytest.raises(ValueError, match="off"):
        transmit_draw_w(CONFIG, False, 1, capacity_bytes=200)
    with pytest.raises(ValueError):
        transmit_draw_w(CONFIG, True, -1, capacity_bytes=200)
    with pytest.raises(ValueError):
        transmit_draw_w(CONFIG, True, 201, capacity_bytes=200)
    with pytest.raises(TypeError):
        transmit_draw_w(CONFIG, True, 1.0, capacity_bytes=200)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        transmit_draw_w(CONFIG, True, True, capacity_bytes=200)


def test_draw_at_the_100_ms_tick_is_bit_for_bit_the_unscaled_formula() -> None:
    # #122 scales the per-byte draw by 100 ms / dt; at 100 ms the factor is exactly 1,
    # so the draw, and every #72 figure, is unchanged.
    for sent in range(0, 201):
        expected = CONFIG.transmitter_on_power_w + CONFIG.transmit_power_per_byte_w * sent
        assert transmit_draw_w(CONFIG, True, sent, capacity_bytes=200) == expected
        assert transmit_draw_w(CONFIG, True, sent, capacity_bytes=200, dt_us=TICK_US) == expected


@pytest.mark.parametrize("dt_us", [30_000, 70_001, TICK_US, 1_000_000, 10_000_000])
def test_a_bytes_energy_does_not_depend_on_the_tick_length(dt_us: int) -> None:
    # Energy of the per-byte part = draw x tick: transmit_power_per_byte_w x 0.1 s per byte.
    sent = 37
    draw_w = transmit_draw_w(CONFIG, True, sent, capacity_bytes=sent, dt_us=dt_us)
    energy_j = (draw_w - CONFIG.transmitter_on_power_w) * dt_us / 1_000_000
    assert energy_j == pytest.approx(sent * CONFIG.transmit_power_per_byte_w * 0.1)


@pytest.mark.parametrize("dt_us", [30_000, 70_001, 1_000_000, 10_000_000])
def test_full_rate_costs_the_same_draw_at_any_tick_length(dt_us: int) -> None:
    # Sending at the full 2000 bytes/s costs 1.5 + 2.0 W, as at 100 ms (200 bytes).
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    comms.step(dt_us, ENV, radio(RadioMode.RX_TX))
    draws = []
    for _ in range(50):
        capacity = comms.snapshot().truth.transmit_capacity_bytes
        comms.step(dt_us, ENV, traffic(RadioMode.RX_TX, sent=capacity))
        draws.append(comms.snapshot().truth.transmit_power_w)
    # Per tick the capacity is rounded, so the draw is too; on average it is exact.
    assert sum(draws) / len(draws) == pytest.approx(1.5 + 2.0, rel=1e-3)
    assert max(draws) - min(draws) <= CONFIG.transmit_power_per_byte_w * TICK_US / dt_us + 1e-12


def test_bytes_are_charged_over_the_tick_they_were_sent_in() -> None:
    # 100 bytes sent in a 50 ms tick are reported in a 200 ms tick: the draw is scaled
    # by the 50 ms tick's length (twice the 100 ms draw), the airtime they took.
    comms = Comms(CONFIG)
    comms.reset(RngFactory(1))
    comms.step(50_000, ENV, radio(RadioMode.RX_TX))
    assert comms.snapshot().truth.transmit_capacity_bytes == 100
    comms.step(200_000, ENV, traffic(RadioMode.RX_TX, sent=100))
    assert comms.snapshot().truth.transmit_power_w == 1.5 + 0.01 * (100 * 2.0)


def test_draw_rejects_bytes_over_a_tick_of_no_length() -> None:
    with pytest.raises(ValueError, match="dt_us"):
        transmit_draw_w(CONFIG, True, 1, capacity_bytes=200, dt_us=0)
    assert transmit_draw_w(CONFIG, True, 0, capacity_bytes=0, dt_us=0) == 1.5


def test_step_uses_the_formula_with_no_bytes_sent() -> None:
    truth = stepped(RadioMode.RX_TX).truth
    assert truth.previous_tick_sent_bytes == 0
    assert truth.transmit_power_w == transmit_draw_w(CONFIG, True, 0, capacity_bytes=200)


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
    assert second.transmit_power_w == transmit_draw_w(CONFIG, True, 120, capacity_bytes=200)
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
    assert config.transmit_rate_bytes_per_s == 1200  # 9600 bit/s
    assert config.transmitter_on_power_w == 0.15
    assert config.transmit_power_per_byte_w == 0.02
    assert NOMINAL_CONFIG.comms == config


def test_config_accepts_zero() -> None:
    CommsConfig(transmit_rate_bytes_per_s=0, transmitter_on_power_w=0, transmit_power_per_byte_w=0)


def test_the_per_tick_capacity_setting_is_gone() -> None:
    # #122 replaced it with the rate; the per-tick value is only in the snapshot.
    with pytest.raises(TypeError):
        CommsConfig(transmit_capacity_bytes=120)  # type: ignore[call-arg]


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"transmit_rate_bytes_per_s": -1}, ValueError),
        ({"transmit_rate_bytes_per_s": 1200.0}, TypeError),
        ({"transmit_rate_bytes_per_s": 1.5}, TypeError),
        ({"transmit_rate_bytes_per_s": True}, TypeError),
        ({"transmit_rate_bytes_per_s": "1200"}, TypeError),
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
