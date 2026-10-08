"""Radio traffic reported to comms through ``SilTarget`` (#98, ADR-0007).

Each tick ``SilTarget`` hands the tick's traffic (the wire bytes of every downlink
frame, the uplink frames lost because the receiver was off, and the flight computer's
suppressed-frame count) to comms as ``controls.radio_traffic`` at step a of the next
tick. So, for traffic in tick N:

- comms reports it in tick N+1: ``previous_tick_sent_bytes`` and the increments of its
  running totals ``uplink_lost_count`` and ``outbound_suppressed_count``, and its
  ``transmit_power_w`` carries the bytes' per-byte draw;
- power, which reads comms one tick late, integrates that energy in tick N+2.

These are SIL timing properties (ADR-0007 §7): asserted here, never in the contract
suite. The ground side is ``_downlink_ground.Ground`` (the real flight computer, a 1 s
boot), which also checks the data accounting in every tick; the radio ``OFF`` case uses
the scripted flight computer, because no real command switches the radio off.
"""

import dataclasses
import itertools
from collections.abc import Iterable, Sequence

import pytest
from _downlink_ground import CHUNK, TICK, Ground, payload_with
from _scripted_flight_computer import ScriptedFlightComputer, ping, set_radio

from pocketsat.flight import FlightComputerOutput, Mode, SpacecraftReadings, controls_for_mode
from pocketsat.frame import MIN_FRAME_SIZE, Frame, FrameType, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    DATA_CHUNK_ID_SIZE,
    TELEMETRY_FRAME_SIZE,
    Command,
)
from pocketsat.spacecraft import (
    NO_RADIO_TRAFFIC,
    NOMINAL_CONFIG,
    AttitudeSnapshot,
    CommsSnapshot,
    CommsTruth,
    PayloadSnapshot,
    PowerSnapshot,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SpacecraftInitialState,
    ThermalSnapshot,
)
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import FORCED_RESET, TRANSMITTER_OFF, SilTarget, SilTick

COMMS = NOMINAL_CONFIG.comms
IDLE_W = COMMS.transmitter_on_power_w
PER_BYTE_W = COMMS.transmit_power_per_byte_w
DATA_FRAME_SIZE = MIN_FRAME_SIZE + DATA_CHUNK_ID_SIZE + CHUNK
"""One DATA frame on the wire: 78 bytes for a 64-byte chunk (#56)."""


def comms(tick: SilTick) -> CommsTruth:
    return tick.state.get("comms", CommsSnapshot).truth


def sent(tick: SilTick) -> int:
    """Wire bytes the flight computer sent in ``tick``."""
    return sum(len(frame) for frame in tick.downlink_frames)


def booted(chunks: int = 0) -> Ground:
    """The real flight computer in NOMINAL (1 s boot), ``chunks`` chunks stored."""
    ground = Ground(SpacecraftInitialState(payload=payload_with(chunks)))
    ground.until(Mode.NOMINAL)
    ground.tick()
    return ground


def check_handover(ticks: Sequence[SilTick]) -> None:
    """Every tick's traffic is in the next tick's controls and comms snapshot."""
    for before, after in itertools.pairwise(ticks):
        traffic = before.traffic
        assert after.controls.radio_traffic == RadioTraffic(
            traffic.sent_bytes, traffic.uplink_lost_count, traffic.outbound_suppressed_count
        )
        truth = comms(after)
        assert truth.previous_tick_sent_bytes == traffic.sent_bytes == sent(before)
        assert truth.uplink_lost_count == comms(before).uplink_lost_count + (
            traffic.uplink_lost_count
        )
        assert truth.outbound_suppressed_count == comms(before).outbound_suppressed_count + (
            traffic.outbound_suppressed_count
        )


# --- Counters per tick -----------------------------------------------------------------


def test_telemetry_only_is_reported_one_tick_later() -> None:
    ground = booted()
    start = len(ground.ticks)
    for _ in range(30):
        ground.tick()
    ticks = ground.ticks[start - 1 :]
    check_handover(ticks)
    telemetry_ticks = [n for n, t in enumerate(ticks) if t.downlink_frames]
    assert len(telemetry_ticks) == 3  # 1 Hz in NOMINAL (#55)
    for n, tick in enumerate(ticks[1:], start=1):
        expected = TELEMETRY_FRAME_SIZE if n - 1 in telemetry_ticks else 0
        assert comms(tick).previous_tick_sent_bytes == expected
        assert comms(tick).transmit_power_w == IDLE_W + PER_BYTE_W * expected
        assert (comms(tick).uplink_lost_count, comms(tick).outbound_suppressed_count) == (0, 0)


def test_data_during_a_pass_is_reported_one_tick_later_and_accounted() -> None:
    # Ground.tick checks the data accounting in every tick (produced = buffered +
    # released, and sent - released is exactly this tick's chunks).
    ground = booted(chunks=40)
    start = len(ground.ticks)
    ground.tick(Command.begin_downlink())
    ground.until(Mode.NOMINAL)
    ground.tick()
    ticks = ground.ticks[start:]
    check_handover(ticks)
    data_ticks = [t for t in ticks if any(len(f) == DATA_FRAME_SIZE for f in t.downlink_frames)]
    assert len(data_ticks) >= 40
    assert max(sent(t) for t in ticks) <= COMMS.transmit_capacity_bytes
    # The first pass tick carries the BEGIN_DOWNLINK ACK and DOWNLINK's first telemetry.
    assert sent(ticks[0]) >= ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE
    assert ground.order == list(range(40))


def test_uplink_lost_while_the_radio_is_off_is_a_running_total() -> None:
    computers: list[ScriptedFlightComputer] = []

    def factory() -> ScriptedFlightComputer:
        computers.append(ScriptedFlightComputer())
        return computers[-1]

    ticks: list[SilTick] = []
    target = SilTarget(flight_computer_factory=factory, tick_observer=ticks.append)
    target.reset(4)
    target.send(set_radio(RadioMode.OFF, sequence=1))
    target.advance(TICK)  # the radio is OFF from the next tick
    for burst in (2, 0, 3):
        for n in range(burst):
            target.send(ping(sequence=10 + n))
        target.advance(TICK)
    target.advance(TICK)
    check_handover(ticks)
    assert [t.traffic.uplink_lost_count for t in ticks] == [0, 2, 0, 3, 0]
    assert [comms(t).uplink_lost_count for t in ticks] == [0, 0, 2, 2, 5]
    # Running totals since reset(seed), and only reset(seed) clears them.
    target.reset(4)
    target.advance(TICK)
    assert comms(ticks[-1]).uplink_lost_count == 0


def test_outbound_suppressed_under_transmitter_off_is_a_running_total() -> None:
    ground = booted()
    ground.target.inject(TargetFault(TRANSMITTER_OFF, duration_us=25 * TICK))
    start = len(ground.ticks)
    ground.tick(Command.ping(), Command.ping())  # two ACKs, no capacity: suppressed
    for _ in range(30):
        ground.tick()
    ticks = ground.ticks[start - 1 :]
    check_handover(ticks)
    faulted = ticks[1:26]
    assert all(sent(t) == 0 for t in faulted)
    suppressed = [t.traffic.outbound_suppressed_count for t in faulted]
    assert suppressed[0] >= 2  # both ACKs (and telemetry, if due)
    assert sum(suppressed) == comms(ticks[26]).outbound_suppressed_count >= 4
    # The receiver kept working: nothing lost, and the pings were executed.
    assert comms(ticks[-1]).uplink_lost_count == 0
    # The count survives a reboot of the flight computer (forced_reset): comms owns it.
    total = comms(ticks[-1]).outbound_suppressed_count
    ground.target.inject(TargetFault(FORCED_RESET))
    ground.tick()
    ground.tick()
    assert comms(ground.ticks[-1]).outbound_suppressed_count == total


# --- Timing and energy -----------------------------------------------------------------


def test_transmit_energy_appears_in_comms_one_tick_and_in_power_two_ticks_later() -> None:
    # ADR-0007 §2 over a whole DOWNLINK pass: the per-byte energy of every frame sent
    # in tick N is in comms' transmit_power_w in tick N+1 and in power's load in N+2,
    # and over the pass it is exactly transmit_power_per_byte_w x the bytes sent.
    ground = booted(chunks=30)
    start = len(ground.ticks)
    ground.tick(Command.begin_downlink())
    ground.until(Mode.NOMINAL)
    for _ in range(3):  # two ticks past the last frame see all of its energy
        ground.tick()
    ticks = ground.ticks[start - 1 :]
    dt_h = TICK / 3_600e6

    for n in range(1, len(ticks)):
        frame_bytes = sent(ticks[n - 1])
        transmit_w = comms(ticks[n]).transmit_power_w
        idle_w = IDLE_W if comms(ticks[n]).transmitter_on else 0.0
        assert transmit_w == idle_w + PER_BYTE_W * frame_bytes  # exact (ADR-0006)
    for n in range(2, len(ticks)):
        previous = ticks[n - 1].state
        load_w = ticks[n].state.get("power", PowerSnapshot).truth.total_load_w
        assert load_w == pytest.approx(
            NOMINAL_CONFIG.power.base_load_w
            + previous.get("payload", PayloadSnapshot).truth.power_w
            + previous.get("attitude", AttitudeSnapshot).truth.control_power_w
            + comms(ticks[n - 1]).transmit_power_w  # tick n-2's bytes
            + previous.get("thermal", ThermalSnapshot).truth.heater_power_w,
            rel=1e-12,
        )

    total_bytes = sum(sent(t) for t in ticks[:-2])
    assert all(sent(t) == 0 for t in ticks[-2:])
    assert total_bytes >= 30 * DATA_FRAME_SIZE
    comms_wh = sum((comms(t).transmit_power_w - IDLE_W) * dt_h for t in ticks[1:])
    assert comms_wh == pytest.approx(PER_BYTE_W * total_bytes * dt_h, rel=1e-12)
    # Power sees the same per-byte energy, shifted by one more tick.
    power_wh = sum(
        (comms(ticks[n - 1]).transmit_power_w - IDLE_W) * dt_h for n in range(2, len(ticks))
    )
    assert power_wh == pytest.approx(comms_wh, rel=1e-12)


def test_transmitter_switching_off_with_bytes_in_flight_is_no_fault() -> None:
    # #48 (PR #110): in the first tick of transmitter_off, comms reports the previous
    # tick's bytes with a capacity of 0. The flight computer's check uses the previous
    # tick's capacity, so this raises no FAULT.
    ground = booted(chunks=30)
    ground.tick(Command.begin_downlink())
    ground.tick()
    assert sent(ground.last) > 0
    ground.target.inject(TargetFault(TRANSMITTER_OFF, duration_us=5 * TICK))
    ground.tick()
    first = comms(ground.last)
    assert first.transmit_capacity_bytes == 0 and not first.transmitter_on
    assert first.previous_tick_sent_bytes > first.transmit_capacity_bytes
    assert first.transmit_power_w == PER_BYTE_W * first.previous_tick_sent_bytes
    for _ in range(10):
        ground.tick()
        assert ground.mode is not Mode.FAULT
    assert ground.mode is Mode.DOWNLINK  # resumed once the transmitter came back


# --- Validation: comms rejects traffic the previous tick could not carry ---------------


class Overcommitted(ScriptedFlightComputer):
    """Sends one frame of ``size`` bytes every tick, ignoring the capacity: a broken
    flight computer, to show comms rejects it rather than clipping it."""

    def __init__(self, size: int, radio: RadioMode) -> None:
        boot = controls_for_mode(Mode.BOOT)
        super().__init__(dataclasses.replace(boot, radio=RadioControls(mode=radio)))
        self.size = size

    def step(
        self, uplink_frames: Iterable[bytes], readings: SpacecraftReadings, now_us: int
    ) -> FlightComputerOutput:
        output = super().step(uplink_frames, readings, now_us)
        payload = bytes(self.size - MIN_FRAME_SIZE)
        frame = encode_frame(Frame(FrameType.TELEMETRY, 0, payload))
        return dataclasses.replace(output, downlink_frames=(frame,))


@pytest.mark.parametrize(
    ("size", "radio", "match"),
    [(COMMS.transmit_capacity_bytes + 1, RadioMode.RX_TX, "0..120"), (20, RadioMode.OFF, "off")],
)
def test_traffic_over_the_previous_capacity_fails_the_next_tick(
    size: int, radio: RadioMode, match: str
) -> None:
    target = SilTarget(flight_computer_factory=lambda: Overcommitted(size, radio))
    target.reset(1)
    target.advance(TICK)  # tick 0: the broken frame goes out; nothing checks it yet
    with pytest.raises(ValueError, match=match):
        target.advance(TICK)  # tick 1: comms is told, and rejects it (ADR-0007 §4)


# --- Reset -----------------------------------------------------------------------------


def test_reset_zeroes_the_traffic_and_tick_zero_gets_none() -> None:
    ground = booted()
    for _ in range(12):
        ground.tick(Command.ping())
    assert comms(ground.last).outbound_suppressed_count == 0
    ground.target.reset(3)
    ground.target.advance(TICK)
    tick = ground.target.last_tick
    assert tick is not None
    assert tick.controls.radio_traffic is NO_RADIO_TRAFFIC
    truth = comms(tick)
    assert (truth.previous_tick_sent_bytes, truth.uplink_lost_count) == (0, 0)
    assert truth.outbound_suppressed_count == 0
