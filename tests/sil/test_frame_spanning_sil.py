"""Frames that span ticks, through ``SilTarget`` with the real flight computer (#132).

ADR-0008 makes the outbound queue a transmitter: at most one frame in transmission,
carried across ticks, so a tick shorter than one frame's airtime still sends at the
configured rate. These runs drive ``SilTarget`` at short ticks (30 ms and 50 ms at the
default 1200 bytes/s: 36 and 60 bytes per tick, below one 78-byte DATA frame, and an
uneven 1234 bytes/s at 30 ms: 37 or 38 bytes) and check, every tick (:class:`Ground`):

- the flight computer transmits at most the tick's capacity, and comms reports those
  bytes in the next tick (ADR-0007, validated by comms itself);
- each chunk is delivered once, in ID order, with its content, and released only after
  its frame was delivered; produced = buffered + released; delivered - released is the
  chunks delivered in the tick, at most one below 78 bytes per tick.

Then, per test: a pass completes, telemetry keeps its period, every tick of a pass is
full (the configured rate), energy follows the bytes, a RESET command reboots only once
its ACK is out (ADR-0008 §6a), and every interruption aborts and counts (§6). At the
default 100 ms tick nothing streams (ADR-0008 §7): ``test_behavior_digests.py`` and the
golden captures prove the bytes unchanged; here every 100 ms tick is checked to be a
whole-frame tick.
"""

import dataclasses
import itertools
import random

import pytest

from pocketsat.flight import FlightComputer, FlightComputerConfig, Mode, controls_for_mode
from pocketsat.flight.boot import BootConfig
from pocketsat.flight.safety import SafetyConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    Command,
    CommandAck,
    CommandId,
    Telemetry,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    NOMINAL_CONFIG,
    CommsSnapshot,
    CommsTruth,
    PayloadInitial,
    PayloadSnapshot,
    PowerSnapshot,
    SpacecraftInitialState,
    chunk_content,
)
from pocketsat.spacecraft.comms import transmit_draw_w
from pocketsat.spacecraft.config import PER_BYTE_DRAW_TICK_US
from pocketsat.targets.base import EnvironmentState, TargetFault
from pocketsat.targets.sil import FORCED_RESET, TRANSMITTER_OFF, SilTarget, SilTick

US_PER_S = 1_000_000
CHUNK = NOMINAL_CONFIG.payload.chunk_size_bytes
BUFFER = NOMINAL_CONFIG.payload.buffer_capacity_bytes
FRAME = 78
ACK = 14
BOOT_1S = FlightComputerConfig(boot=BootConfig(duration_us=US_PER_S))

SHORT_TICKS = [
    pytest.param(30_000, 1_200, id="30ms-36B"),
    pytest.param(50_000, 1_200, id="50ms-60B"),
    pytest.param(30_000, 1_234, id="30ms-37or38B"),
]


def comms(tick: SilTick) -> CommsTruth:
    return tick.state.get("comms", CommsSnapshot).truth


class Ground:
    """``SilTarget`` at any tick length, driven through ``TestTarget``, checked every
    tick (see the module docstring)."""

    def __init__(
        self,
        tick_us: int,
        *,
        rate: int = 1_200,
        chunks: int = 60,
        config: FlightComputerConfig = BOOT_1S,
        seed: int = 3,
    ) -> None:
        spacecraft = dataclasses.replace(
            NOMINAL_CONFIG,
            comms=dataclasses.replace(NOMINAL_CONFIG.comms, transmit_rate_bytes_per_s=rate),
        )
        initial = SpacecraftInitialState(
            payload=PayloadInitial(buffer_fill=(chunks * CHUNK + CHUNK // 2) / BUFFER)
        )
        self.computers: list[FlightComputer] = []

        def factory() -> FlightComputer:
            computer = FlightComputer(config)
            self.computers.append(computer)
            return computer

        self.tick_us = tick_us
        self.config = spacecraft
        self.target = SilTarget(
            config=spacecraft, initial=initial, tick_us=tick_us, flight_computer_factory=factory
        )
        self.target.reset(seed)
        self.sequence = 0
        self.ticks: list[SilTick] = []
        self.frames: list[list[Frame]] = []
        self.received: dict[int, int] = {}  # chunk ID -> tick delivered
        self.order: list[int] = []
        self.acks: list[tuple[int, CommandAck, int]] = []  # (tick, ACK, frame sequence)
        self.telemetry: list[tuple[int, Telemetry]] = []
        self.oldest = 0

    @property
    def fc(self) -> FlightComputer:
        return self.computers[-1]

    def mode(self) -> Mode:
        """The flight computer's mode (a call, so mypy does not narrow it across ticks)."""
        return self.fc.mode

    @property
    def last(self) -> SilTick:
        return self.ticks[-1]

    def tick(
        self,
        *commands: Command,
        faults: tuple[TargetFault, ...] = (),
        env: EnvironmentState | None = None,
    ) -> list[Frame]:
        for fault in faults:
            self.target.inject(fault)
        if env is not None:
            self.target.apply_environment(env)
        for command in commands:
            self.sequence += 1
            frame = Frame(FrameType.COMMAND, self.sequence, encode_command(command))
            self.target.send(encode_frame(frame))
        self.target.advance(self.tick_us)
        record = self.target.last_tick
        assert record is not None
        n = len(self.ticks)
        previous = self.ticks[-1] if self.ticks else None
        self.ticks.append(record)
        truth = comms(record)
        # The transmitter keeps within this tick's capacity; comms is told in the next.
        assert record.traffic.sent_bytes <= truth.transmit_capacity_bytes
        if previous is not None:
            assert truth.previous_tick_sent_bytes == previous.traffic.sent_bytes

        payload = record.state.get("payload", PayloadSnapshot).truth
        for chunk_id in range(self.oldest, payload.oldest_unreleased_chunk_id):
            assert self.received[chunk_id] < n, f"chunk {chunk_id} released before delivery"
        self.oldest = payload.oldest_unreleased_chunk_id

        frames = [decode_frame(raw) for raw in self.target.receive()]
        assert [encode_frame(f) for f in frames] == list(record.downlink_frames)
        self.frames.append(frames)
        delivered: list[int] = []
        for frame in frames:
            if frame.frame_type is FrameType.DATA:
                chunk = decode_data(frame.payload)
                assert chunk.content == chunk_content(chunk.chunk_id, CHUNK)
                assert chunk.chunk_id not in self.received, "a chunk went down twice"
                assert not self.order or chunk.chunk_id == self.order[-1] + 1, "out of order"
                self.received[chunk.chunk_id] = n
                self.order.append(chunk.chunk_id)
                delivered.append(chunk.chunk_id)
            elif frame.frame_type is FrameType.ACK:
                self.acks.append((n, decode_ack(frame.payload), frame.sequence))
            elif frame.frame_type is FrameType.TELEMETRY:
                self.telemetry.append((n, decode_telemetry(frame.payload)))
        assert payload.total_produced_bytes == payload.buffered_bytes + payload.total_released_bytes
        assert len(self.received) * CHUNK - payload.total_released_bytes == len(delivered) * CHUNK
        if truth.transmit_capacity_bytes < FRAME:
            assert len(delivered) <= 1
        return frames

    def run(self, count: int) -> None:
        for _ in range(count):
            self.tick()

    def until(self, mode: Mode, limit: int = 5_000) -> None:
        for _ in range(limit):
            if self.mode() is mode:
                return
            self.tick()
        raise AssertionError(f"never reached {mode.name}")

    def start_pass(self) -> None:
        self.until(Mode.NOMINAL)
        self.tick(Command.begin_downlink())
        assert self.fc.mode is Mode.DOWNLINK

    def until_data_in_transmission(self, remaining_at_least: int = 1) -> None:
        """Tick until a DATA frame is in transmission with at least this much left."""
        for _ in range(500):
            progress = self.last.transmitting if self.ticks else None
            if (
                progress is not None
                and progress.frame_type is FrameType.DATA
                and progress.size_bytes - progress.sent_bytes >= remaining_at_least
            ):
                return
            self.tick()
        raise AssertionError("no DATA frame in transmission")


def remaining(tick: SilTick) -> int:
    """What is left of the frame in transmission at the end of ``tick``."""
    progress = tick.transmitting
    return 0 if progress is None else progress.size_bytes - progress.sent_bytes


# --- A pass at short ticks ----------------------------------------------------------------


@pytest.mark.parametrize(("tick_us", "rate"), SHORT_TICKS)
def test_a_pass_completes_at_a_short_tick_at_the_configured_rate(tick_us: int, rate: int) -> None:
    ground = Ground(tick_us, rate=rate, config=dataclasses.replace(BOOT_1S))
    ground.start_pass()
    entry = len(ground.ticks) - 1
    pings = 0
    for n in itertools.count():
        if ground.mode() is not Mode.DOWNLINK:
            break
        if n % 7 == 3:
            pings += 1
            ground.tick(Command.ping())
        else:
            ground.tick()
        assert n < 5_000
    assert ground.mode() is Mode.NOMINAL  # DOWNLINK_COMPLETE
    assert ground.order == list(range(60))  # every chunk once, in order
    complete = len(ground.ticks) - 1
    ground.run(5)  # the last ACK and NOMINAL's first frame finish

    # Nothing was suppressed during the pass: every ACK came back, and every telemetry
    # frame. The one exception is ADR-0008 §5's replacement rule at the mode change:
    # DOWNLINK_COMPLETE makes NOMINAL's frame due at once, and if a periodic DOWNLINK
    # frame is still held behind DATA it is replaced (newer readings, the new mode).
    suppressed = [i for i, t in enumerate(ground.ticks) if t.traffic.outbound_suppressed_count]
    assert suppressed in ([], [complete])
    assert sum(t.traffic.outbound_suppressed_count for t in ground.ticks) == len(suppressed)
    assert ground.telemetry[-1][1].mode is Mode.NOMINAL
    assert sum(t.outbound_aborted_count for t in ground.ticks) == 0
    answered = [ack for _, ack, _ in ground.acks if ack.command_id == CommandId.PING]
    assert len(answered) == pings

    # Telemetry keeps its period (1 s, rounded up to whole ticks): one frame per period
    # in DOWNLINK, by uptime.
    period_ms = -(-US_PER_S // tick_us) * tick_us // 1000
    uptimes = [t.uptime_ms for _, t in ground.telemetry if t.mode is Mode.DOWNLINK]
    assert len(uptimes) >= 3
    assert {b - a for a, b in itertools.pairwise(uptimes)} == {period_ms}

    # The configured rate: every tick of the pass is full until the last chunk is out.
    last_chunk_tick = ground.received[59]
    pass_ticks = ground.ticks[entry:last_chunk_tick]
    for tick in pass_ticks:
        assert tick.traffic.sent_bytes == comms(tick).transmit_capacity_bytes
    tail = ground.ticks[last_chunk_tick]
    assert tail.traffic.sent_bytes > 0
    on_bytes = sum(
        comms(t).transmit_capacity_bytes for t in ground.ticks[entry : last_chunk_tick + 1]
    )
    sent = sum(t.traffic.sent_bytes for t in ground.ticks[entry : last_chunk_tick + 1])
    assert on_bytes - FRAME < sent <= on_bytes  # at most one frame of idle tail


@pytest.mark.parametrize(("tick_us", "rate"), SHORT_TICKS)
def test_energy_follows_the_bytes_transmitted(tick_us: int, rate: int) -> None:
    # Comms charges tick N's transmitted bytes in tick N+1, scaled to tick N's length,
    # so a byte costs 2 mJ at any tick length; the bytes of frames still in
    # transmission are charged as they go out, not when the frame is delivered.
    ground = Ground(tick_us, rate=rate)
    ground.start_pass()
    ground.run(200)
    config = ground.config.comms
    per_byte_j = 0.0
    total = 0
    for before, after in itertools.pairwise(ground.ticks):
        on = comms(after).transmitter_on
        if not (on and comms(before).transmitter_on):
            continue
        sent = before.traffic.sent_bytes
        expected = transmit_draw_w(
            config, on, sent, capacity_bytes=comms(before).transmit_capacity_bytes, dt_us=tick_us
        )
        assert comms(after).transmit_power_w == expected
        idle = transmit_draw_w(config, on, 0, capacity_bytes=0)
        per_byte_j += (comms(after).transmit_power_w - idle) * tick_us / US_PER_S
        total += sent
    assert total > 3_000
    assert per_byte_j == pytest.approx(
        total * config.transmit_power_per_byte_w * PER_BYTE_DRAW_TICK_US / US_PER_S
    )
    assert per_byte_j == pytest.approx(0.002 * total)  # 2 mJ per byte


def test_a_short_tick_run_is_deterministic() -> None:
    def run(seed: int) -> list[SilTick]:
        ground = Ground(30_000, seed=seed)
        ground.start_pass()
        for n in range(150):
            ground.tick(*([Command.ping()] if n % 11 == 5 else []))
        return ground.ticks

    first, second = run(7), run(7)
    assert first == second
    assert first != run(8)


# --- No change at the default 100 ms tick (ADR-0008 §7) ------------------------------------


def test_every_100ms_tick_is_a_whole_frame_tick() -> None:
    # A pass with ACKs, nine-command bursts (the ninth ACK is suppressed), RESETs in a
    # pass, transmitter_off and forced_reset: nothing is ever in transmission at a tick
    # boundary, held, aborted, or pending, and the bytes transmitted are the frames
    # delivered.
    rng = random.Random(132)
    ground = Ground(100_000, chunks=200, config=FlightComputerConfig())
    for n in range(1_500):
        commands: list[Command] = []
        faults: tuple[TargetFault, ...] = ()
        if n in (100, 600, 1_100):
            commands.append(Command.begin_downlink())
        if rng.random() < 0.03:
            commands += [Command.ping()] * rng.randint(1, 9)
        if n in (300, 750):
            commands += [Command.ping(), Command.reset()]
        if n == 400:
            faults = (TargetFault(fault_type=TRANSMITTER_OFF, params={}, duration_us=700_000),)
        if n in (500, 900):
            faults = (TargetFault(fault_type=FORCED_RESET, params={}, duration_us=200_000),)
        ground.tick(*commands, faults=faults)
        tick = ground.last
        assert tick.transmitting is None
        assert not tick.reboot_pending
        assert tick.outbound_aborted_count == 0
        assert tick.traffic.sent_bytes == sum(map(len, tick.downlink_frames))
        assert ground.fc.held_frame_count == 0
    assert ground.order  # DATA went down
    assert sum(t.traffic.outbound_suppressed_count for t in ground.ticks) > 0


@pytest.mark.parametrize("phase", range(10))
@pytest.mark.parametrize("before", [0, 1, 8], ids=["alone", "after-a-ping", "ninth"])
def test_at_100ms_a_reset_reboots_in_its_own_tick(phase: int, before: int) -> None:
    # In every phase of DOWNLINK's 10-tick telemetry period, alone, after a PING, and
    # as the ninth command (its ACK does not fit and is suppressed): the reboot is in
    # step d of the RESET tick, as before ADR-0008, and never pending.
    ground = Ground(100_000, chunks=200)
    ground.start_pass()
    ground.run(phase)
    boots = ground.fc.boot_count
    ground.tick(*[Command.ping()] * before, Command.reset())
    tick = ground.last
    assert ground.fc.boot_count == boots + 1 and ground.mode() is Mode.BOOT
    assert not tick.reboot_pending and tick.transmitting is None
    assert tick.next_controls == controls_for_mode(Mode.BOOT)
    resets = [a for a in ground.acks if a[1].command_id == CommandId.RESET]
    if before == 8:
        assert resets == [] and tick.traffic.outbound_suppressed_count >= 1
    else:
        [(n, _, _)] = resets
        assert n == len(ground.ticks) - 1


# --- RESET waits for its ACK (ADR-0008 §6a) ------------------------------------------------


def reset_mid_data(tick_us: int, remaining_at_least: int = 1, seed: int = 3) -> tuple[Ground, int]:
    """A pass with a DATA frame in transmission; then a RESET command. Returns the
    ground and the RESET tick's index."""
    ground = Ground(tick_us, seed=seed)
    ground.start_pass()
    ground.until_data_in_transmission(remaining_at_least)
    ground.tick(Command.reset())
    return ground, len(ground.ticks) - 1


def reboot_tick(ground: Ground, boots_before: int, limit: int = 10) -> int:
    """Tick until the flight computer reboots; return that tick's index."""
    for _ in range(limit):
        if ground.fc.boot_count > boots_before:
            return len(ground.ticks) - 1
        ground.tick()
    raise AssertionError("never rebooted")


@pytest.mark.parametrize(("tick_us", "most"), [(30_000, 2), (50_000, 1)])
@pytest.mark.parametrize("seed", range(12))
def test_the_reboot_waits_for_the_reset_ack_within_the_bound(
    tick_us: int, most: int, seed: int
) -> None:
    rng = random.Random(seed)
    ground = Ground(tick_us, seed=seed)
    ground.start_pass()
    ground.run(rng.randint(0, 40))  # a random phase of the frames in transmission
    boots = ground.fc.boot_count
    r = remaining(ground.last)
    last_sequence = max((f.sequence for frames in ground.frames for f in frames), default=0)
    ground.tick(Command.reset())
    n = len(ground.ticks) - 1
    m = reboot_tick(ground, boots)
    # M is the first tick with C_N + ... + C_M >= r + 14 (ADR-0008 §6a).
    capacities = [comms(t).transmit_capacity_bytes for t in ground.ticks[n:]]
    expected = next(d for d in range(len(capacities)) if sum(capacities[: d + 1]) >= r + ACK)
    assert m == n + expected and m - n <= most
    assert ground.fc.boot_count == boots + 1
    assert [t.reboot_pending for t in ground.ticks[n : m + 1]] == [True] * (m - n) + [False]
    # The RESET ACK is delivered in M, numbered from before the reboot.
    [(tick, ack, sequence)] = [a for a in ground.acks if a[1].command_id == CommandId.RESET]
    assert tick == m and ack.accepted
    frames_before = [f for frames in ground.frames[n : m + 1] for f in frames]
    assert [f.sequence for f in frames_before] == list(
        range(last_sequence + 1, last_sequence + 1 + len(frames_before))
    )
    assert sequence in [f.sequence for f in frames_before]  # pre-reboot numbering
    # BOOT's controls from M+1 (with the release of a chunk completed in M, ADR-0008
    # §6); the old mode's until then. Uptime counts from the end of M.
    completed = [c for c, t in ground.received.items() if t == m]
    boot = controls_for_mode(Mode.BOOT, release_through_chunk_id=max(completed, default=None))
    assert ground.ticks[m].next_controls == boot
    for t in ground.ticks[n:m]:
        assert t.next_controls.radio == controls_for_mode(Mode.DOWNLINK).radio
        assert t.next_controls.attitude == controls_for_mode(Mode.DOWNLINK).attitude
    assert ground.fc.uptime_us == 0
    ground.tick()
    assert ground.last.controls.payload == boot.payload
    ground.until(Mode.NOMINAL)
    beacon = next(f for frames in ground.frames[m + 1 :] for f in frames)
    assert beacon.sequence == 0  # the sequence restarted at the reboot
    beacon_tick = next(i for i in range(m + 1, len(ground.frames)) if ground.frames[i])
    assert decode_telemetry(beacon.payload).uptime_ms == (beacon_tick - m) * tick_us // 1000


def test_while_the_reboot_is_pending_uplink_is_ignored_and_nothing_new_is_sent() -> None:
    # At least 60 bytes of DATA ahead of the RESET ACK: 74 or more at 36 per tick, so
    # the reboot is pending in N and N+1 and happens in N+2.
    ground, n = reset_mid_data(30_000, remaining_at_least=60)
    assert remaining(ground.ticks[n - 1]) >= 60
    assert ground.last.reboot_pending
    ground.tick(Command.ping())  # N+1: pending
    pending = ground.last
    assert pending.reboot_pending
    assert pending.uplink_dropped_in_reset_count == 1  # heard, ignored, no reply
    assert pending.delivered_uplink != ()  # passed on to the flight computer
    assert ground.mode() is Mode.DOWNLINK  # no mode event applies yet
    ground.tick()  # N+2: the RESET ACK's last byte, then the reboot
    assert not ground.last.reboot_pending and ground.mode() is Mode.BOOT
    kinds = [f.frame_type for frames in ground.frames[n:] for f in frames]
    assert kinds == [FrameType.DATA, FrameType.ACK]  # no telemetry, no new DATA
    assert [a.command_id for _, a, _ in ground.acks[-1:]] == [CommandId.RESET]


def test_no_flag_rule_runs_while_the_reboot_is_pending() -> None:
    # Flags sustained for one tick enter SAFE; a battery forced to 0 % would do so at
    # once, but while the reboot is pending no rule runs. After the reboot the rules
    # start again from BOOT.
    config = FlightComputerConfig(boot=BootConfig(US_PER_S), safety=SafetyConfig(1))
    ground = Ground(30_000, config=config)
    ground.start_pass()
    ground.until_data_in_transmission(60)
    boots = ground.fc.boot_count
    ground.tick(Command.reset())
    flat = EnvironmentState(battery_soc_override=0.0)
    ground.tick(env=flat)
    assert ground.last.reboot_pending
    power = ground.last.state.get("power", PowerSnapshot).readings
    assert power.critical_battery  # the flag is there
    assert ground.mode() is Mode.DOWNLINK  # but not acted on
    ground.tick(env=flat)
    assert ground.fc.boot_count == boots + 1 and ground.mode() is Mode.BOOT
    ground.tick(env=flat)
    assert ground.mode() is Mode.SAFE  # from BOOT, once the rules run again


def test_a_data_frame_ahead_of_the_reset_ack_completes_and_is_released_once() -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission(40)
    chunk = len(ground.order)  # the chunk in transmission
    boots = ground.fc.boot_count
    ground.tick(Command.reset())
    n = len(ground.ticks) - 1
    m = reboot_tick(ground, boots)
    assert n <= ground.received[chunk] <= m  # delivered ahead of the RESET ACK
    ground.run(3)
    payload = ground.last.state.get("payload", PayloadSnapshot).truth
    assert payload.oldest_unreleased_chunk_id == chunk + 1  # released, once
    ground.until(Mode.NOMINAL)
    ground.tick(Command.begin_downlink())
    ground.run(10)
    assert ground.order[ground.order.index(chunk) + 1] == chunk + 1  # resumed after it


def test_the_reset_ack_overtakes_waiting_acks_which_keep_their_order() -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission(60)
    ground.tick(Command.ping(), Command.ping())  # held behind the DATA frame
    held = [ground.sequence - 1, ground.sequence]
    assert ground.fc.held_frame_count == 2
    boots = ground.fc.boot_count
    ground.tick(Command.ping(), Command.reset(), Command.ping())
    pings = [*held, ground.sequence - 2, ground.sequence]  # in command order
    reboot_tick(ground, boots)
    answered = [(a.command_id, a.sequence) for _, a, _ in ground.acks[1:]]  # after BEGIN
    # The RESET ACK went first; the PING ACKs behind it kept their order, and those
    # still waiting at the reboot were dropped and counted as suppressed.
    assert answered[0] == (CommandId.RESET, ground.sequence - 1)
    after = [sequence for _, sequence in answered[1:]]
    assert after == pings[: len(after)] and len(after) < 4
    assert sum(t.traffic.outbound_suppressed_count for t in ground.ticks) == 4 - len(after)


def test_two_resets_in_one_tick_reboot_twice_at_the_deferred_reboot() -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission(60)
    boots = ground.fc.boot_count
    ground.tick(Command.reset(), Command.reset())
    assert ground.last.reboot_pending
    reboot_tick(ground, boots)
    assert ground.fc.boot_count == boots + 2  # as today's update_mode applies them


def test_transmitter_off_in_the_reset_tick_reboots_at_once() -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission()
    boots = ground.fc.boot_count
    off = TargetFault(fault_type=TRANSMITTER_OFF, params={}, duration_us=30_000)
    ground.tick(Command.reset(), faults=(off,))
    tick = ground.last
    assert ground.fc.boot_count == boots + 1 and not tick.reboot_pending
    assert tick.traffic.sent_bytes == 0
    assert tick.outbound_aborted_count == 1  # the DATA frame
    assert tick.traffic.outbound_suppressed_count == 1  # the RESET ACK


def test_transmitter_off_during_a_pending_reboot_reboots_in_that_tick() -> None:
    ground, _ = reset_mid_data(30_000, remaining_at_least=60)
    boots = ground.fc.boot_count
    assert ground.last.reboot_pending
    off = TargetFault(fault_type=TRANSMITTER_OFF, params={}, duration_us=30_000)
    ground.tick(faults=(off,))
    tick = ground.last
    assert ground.fc.boot_count == boots + 1 and not tick.reboot_pending
    assert ground.mode() is Mode.BOOT
    # The RESET ACK, never delivered, is aborted (or dropped while held) and counted.
    assert tick.traffic.outbound_suppressed_count == 1
    assert tick.outbound_aborted_count >= 1
    assert not [a for a in ground.acks if a[1].command_id == CommandId.RESET]


@pytest.mark.parametrize("hold_us", [None, 90_000], ids=["pulse", "hold"])
def test_forced_reset_during_a_pending_reboot_reboots_once(hold_us: int | None) -> None:
    ground, _ = reset_mid_data(30_000, remaining_at_least=60)
    boots = ground.fc.boot_count
    assert ground.last.reboot_pending
    reset = TargetFault(fault_type=FORCED_RESET, params={}, duration_us=hold_us)
    ground.tick(faults=(reset,))
    while ground.last.flight_computer_held:
        assert ground.last.traffic.sent_bytes == 0  # nothing transmitted while held
        assert ground.last.transmitting is not None  # frozen in the flight computer
        ground.tick()
    rebooted = ground.last
    assert rebooted.flight_computer_rebooted and not rebooted.reboot_pending
    assert ground.fc.boot_count == boots + 1  # one reboot, not two
    assert rebooted.traffic.outbound_suppressed_count == 1  # the RESET ACK
    assert rebooted.outbound_aborted_count >= 1
    ground.run(5)
    assert ground.fc.boot_count == boots + 1
    assert not [a for a in ground.acks if a[1].command_id == CommandId.RESET]


# --- Other interruptions (ADR-0008 §6) -----------------------------------------------------


@pytest.mark.parametrize("hold_us", [None, 90_000], ids=["pulse", "hold"])
def test_forced_reset_mid_frame_aborts_it_and_the_chunk_is_resent(hold_us: int | None) -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission(30)
    chunk = len(ground.order)
    reset = TargetFault(fault_type=FORCED_RESET, params={}, duration_us=hold_us)
    ground.tick(faults=(reset,))
    while ground.last.flight_computer_held:
        assert ground.last.traffic.sent_bytes == 0
        ground.tick()
    assert ground.last.outbound_aborted_count == 1
    assert ground.last.traffic.outbound_suppressed_count == 0  # DATA is not counted
    ground.until(Mode.NOMINAL)
    ground.tick(Command.begin_downlink())
    ground.run(20)
    assert ground.order[chunk] == chunk  # resent first, once


def test_transmitter_off_mid_data_aborts_it_and_the_pass_completes() -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission(30)
    off = TargetFault(fault_type=TRANSMITTER_OFF, params={}, duration_us=300_000)
    ground.tick(faults=(off,))
    assert ground.last.outbound_aborted_count == 1
    ground.until(Mode.NOMINAL)
    assert ground.order == list(range(60))


@pytest.mark.parametrize(
    "leave", [Command.enter_safe_mode(), Command.set_mode(Mode.NOMINAL)], ids=["SAFE", "SET_MODE"]
)
def test_leaving_downlink_mid_data_aborts_it_and_the_next_pass_resends_it(leave: Command) -> None:
    ground = Ground(30_000)
    ground.start_pass()
    ground.until_data_in_transmission(60)
    chunk = len(ground.order)
    ground.tick(leave)
    assert ground.mode() is not Mode.DOWNLINK
    assert ground.last.outbound_aborted_count == 1
    assert ground.last.transmitting is None
    if ground.mode() is Mode.SAFE:
        ground.run(5)
        ground.tick(Command.set_mode(Mode.NOMINAL))
    ground.run(5)
    ground.tick(Command.begin_downlink())
    ground.until(Mode.NOMINAL)
    assert ground.order == list(range(60))
    assert ground.order[chunk] == chunk
