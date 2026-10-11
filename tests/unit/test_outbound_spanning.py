"""Tests for the outbound queue's transmitter: frames that span ticks (#132, ADR-0008).

The real :class:`~pocketsat.flight.FlightComputer` runs against the real
:class:`~pocketsat.spacecraft.Payload` (as in ``test_downlink.py``), with the transmit
capacity set per tick, so a release produced in tick N is applied in tick N+1. Every
tick, :class:`Rig` checks what ADR-0008 requires whatever the capacity:

- the flight computer transmits at most the tick's capacity (``sent_bytes``);
- each chunk is delivered once, in ID order, and released only after its whole frame
  was delivered, so produced = buffered + released and delivered - released is the
  chunks delivered in that tick (at most one when the capacity is below one frame);
- bytes are conserved: what was transmitted is what was delivered, plus the frame in
  transmission, plus what was aborted.

A whole-frame tick (a capacity of at least the largest frame, 78 bytes, with nothing in
transmission or held) follows #51's rule unchanged; those tests are in
``test_command_dispatcher.py``, ``test_telemetry_scheduler.py`` and
``test_downlink.py``. The streaming counterparts of their below-78-byte cases are here.
"""

import dataclasses
import itertools
import random
from collections.abc import Sequence

import pytest

from pocketsat import messages
from pocketsat.core.rng import RngFactory
from pocketsat.flight import (
    FlightComputer,
    FlightComputerConfig,
    FlightComputerOutput,
    Mode,
    ModeState,
    SpacecraftReadings,
    controls_for_mode,
)
from pocketsat.flight.boot import BootConfig
from pocketsat.flight.computer import OutboundClass, OutboundConfig, TickContext
from pocketsat.flight.downlink import DownlinkConfig
from pocketsat.flight.telemetry import TelemetryConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    TELEMETRY_FRAME_SIZE,
    Command,
    CommandId,
    data_frame_size,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    Payload,
    PayloadConfig,
    PayloadInitial,
    SnapshotBoard,
    SpacecraftControls,
    SubsystemStack,
    chunk_content,
)
from pocketsat.spacecraft.fakes import fake_stack, fake_subsystems
from pocketsat.targets.base import EnvironmentState

TICK_US = 100_000
CHUNK = 64
FRAME = data_frame_size(CHUNK)  # 78
NEVER = 10**12
"""A telemetry period no test reaches: only mode changes send telemetry."""

QUIET = FlightComputerConfig(
    boot=BootConfig(duration_us=TICK_US), telemetry=TelemetryConfig.uniform(NEVER)
)
"""Boot in one step; telemetry only at mode changes."""

BASE = SpacecraftReadings.from_state(fake_stack().snapshot())


def frame_for(command: Command, sequence: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


class Rig:
    """The real flight computer and the real payload, one tick at a time, with the
    transmit capacity set per tick (0 also turns the transmitter off)."""

    def __init__(self, chunks: int = 0, *, config: FlightComputerConfig = QUIET) -> None:
        buffer_bytes = CHUNK * 1000
        board = SnapshotBoard()
        fakes = [fake for fake in fake_subsystems() if fake.name != "payload"]
        SubsystemStack(fakes, board=board).reset(RngFactory(0))
        fill = (chunks * CHUNK + CHUNK // 2) / buffer_bytes
        self.payload = Payload(
            PayloadConfig(buffer_capacity_bytes=buffer_bytes, chunk_size_bytes=CHUNK),
            PayloadInitial(buffer_fill=fill),
            board,
        )
        self.payload.reset(RngFactory(0))
        self.fc = FlightComputer(config)
        self.controls: SpacecraftControls = self.fc.reset()
        self.now_us = 0
        self.sequence = 0
        self.outputs: list[FlightComputerOutput] = []
        self.delivered: list[Frame] = []
        self.per_tick: list[list[Frame]] = []
        self.chunks: list[int] = []  # DATA chunk IDs delivered, in order
        self.received_tick: dict[int, int] = {}
        self.transmitted = 0
        self.aborted_bytes = 0
        self.step(capacity=1000)  # boot (a whole-frame tick)
        assert self.fc.mode is Mode.NOMINAL

    def readings(self) -> messages.PayloadReadings:  # type: ignore[name-defined]
        return self.payload.snapshot().readings

    def step(
        self,
        *commands: Command,
        capacity: int = 120,
        flags: Sequence[str] = (),
        fault: bool = False,
        raw: Sequence[bytes] = (),
    ) -> list[Frame]:
        """Run one tick; return the frames delivered in it, decoded."""
        self.payload.step(TICK_US, EnvironmentState(), self.controls)
        payload = self.payload.snapshot().readings
        low = "low_battery" in flags or "critical_battery" in flags
        readings = dataclasses.replace(
            BASE,
            payload=payload,
            comms=dataclasses.replace(
                BASE.comms, transmit_capacity_bytes=capacity, transmitter_on=capacity > 0
            ),
            power=dataclasses.replace(
                BASE.power,
                # fault: critical without low fails a consistency check (FAULT, #48)
                low_battery=low and not fault,
                critical_battery="critical_battery" in flags or fault,
            ),
            thermal=dataclasses.replace(
                BASE.thermal, over_temp="over_temp" in flags, under_temp="under_temp" in flags
            ),
        )
        uplink = [*raw]
        for command in commands:
            self.sequence += 1
            uplink.append(frame_for(command, self.sequence))
        before = self.fc.transmitting
        self.now_us += TICK_US
        out = self.fc.step(uplink, readings, self.now_us)
        self.controls = out.controls
        self.outputs.append(out)
        n = len(self.outputs) - 1

        assert out.sent_bytes <= capacity
        frames = [decode_frame(f) for f in out.downlink_frames]
        self.delivered += frames
        self.per_tick.append(frames)
        # Bytes are conserved: transmitted = delivered + in transmission + aborted.
        self.transmitted += out.sent_bytes
        delivered_bytes = sum(map(len, out.downlink_frames))
        progress_before = before.sent_bytes if before is not None else 0
        progress_after = out.transmitting.sent_bytes if out.transmitting is not None else 0
        assert out.sent_bytes >= 0
        aborted = progress_before + out.sent_bytes - delivered_bytes - progress_after
        assert aborted >= 0
        self.aborted_bytes += aborted
        if aborted:
            assert out.outbound_aborted_count >= 1

        sent_now: list[int] = []
        for frame in frames:
            if frame.frame_type is FrameType.DATA:
                chunk = decode_data(frame.payload)
                assert chunk.content == chunk_content(chunk.chunk_id, CHUNK)
                assert chunk.chunk_id not in self.received_tick, "a chunk went down twice"
                assert not self.chunks or chunk.chunk_id == self.chunks[-1] + 1, "out of order"
                self.received_tick[chunk.chunk_id] = n
                self.chunks.append(chunk.chunk_id)
                sent_now.append(chunk.chunk_id)
        # Data accounting: produced = buffered + released; delivered - released is this
        # tick's chunks, and released only after delivery (#56, ADR-0008 §6).
        assert payload.total_produced_bytes == payload.buffered_bytes + payload.total_released_bytes
        assert (len(self.chunks) - len(sent_now)) * CHUNK == payload.total_released_bytes
        if capacity < FRAME:
            assert len(sent_now) <= 1
        release = out.controls.payload.release_through_chunk_id
        if sent_now:
            assert release == sent_now[-1]
        return frames

    def run(self, ticks: int, **kwargs: object) -> None:
        for _ in range(ticks):
            self.step(**kwargs)  # type: ignore[arg-type]

    def begin_downlink(self) -> None:
        """Enter DOWNLINK in a whole-frame tick with room for the ACK and the entry
        telemetry frame only, so no DATA goes out yet."""
        self.step(Command.begin_downlink(), capacity=ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE + 28)
        assert self.fc.mode is Mode.DOWNLINK
        assert not self.chunks

    @property
    def last(self) -> FlightComputerOutput:
        return self.outputs[-1]


def types(frames: Sequence[Frame]) -> list[FrameType]:
    return [f.frame_type for f in frames]


# --- Settings ----------------------------------------------------------------------------


def test_the_held_ack_bound_defaults_to_one_default_ticks_worth() -> None:
    assert OutboundConfig().max_held_ack_frames == 8 == 120 // ACK_FRAME_SIZE
    assert FlightComputerConfig().outbound == OutboundConfig()
    assert OutboundConfig(max_held_ack_frames=0).max_held_ack_frames == 0


@pytest.mark.parametrize(
    ("value", "error"), [(-1, ValueError), (1.0, TypeError), (True, TypeError), ("8", TypeError)]
)
def test_outbound_config_validates(value: object, error: type[Exception]) -> None:
    with pytest.raises(error):
        OutboundConfig(max_held_ack_frames=value)  # type: ignore[arg-type]


def test_flight_computer_config_checks_the_outbound_type() -> None:
    with pytest.raises(TypeError, match="outbound"):
        FlightComputerConfig(outbound=8)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("chunk_size", "largest"),
    [(64, 78), (16, TELEMETRY_FRAME_SIZE), (1, TELEMETRY_FRAME_SIZE), (200, 214)],
)
def test_the_largest_frame_follows_the_chunk_size(chunk_size: int, largest: int) -> None:
    fc = FlightComputer(FlightComputerConfig(downlink=DownlinkConfig(chunk_size)))
    assert fc.max_frame_size == largest


# --- The two kinds of tick (ADR-0008 §2) -------------------------------------------------


@pytest.mark.parametrize("capacity", [FRAME, FRAME + 1, 120])
def test_a_tick_of_at_least_one_frame_sends_whole_frames(capacity: int) -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    for n in range(5):
        frames = rig.step(capacity=capacity)
        assert [decode_data(f.payload).chunk_id for f in frames] == [n]
        assert rig.last.sent_bytes == FRAME
        assert rig.last.transmitting is None and rig.fc.held_frame_count == 0


def test_a_tick_below_one_frame_streams_it_across_ticks() -> None:
    # 77 bytes per tick: each DATA frame takes two ticks, and every tick is full.
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=FRAME - 1)
    assert rig.per_tick[-1] == []
    progress = rig.last.transmitting
    assert progress is not None
    assert (progress.frame_type, progress.size_bytes, progress.sent_bytes) == (
        FrameType.DATA,
        FRAME,
        FRAME - 1,
    )
    for _ in range(10):
        rig.step(capacity=FRAME - 1)
        assert rig.last.sent_bytes == FRAME - 1
    assert rig.chunks == list(range(10))  # 11 ticks x 77 bytes = 847 bytes: 10 frames
    assert sum(o.sent_bytes for o in rig.outputs[-11:]) == 11 * (FRAME - 1)


def test_the_worked_example_of_adr_0008_section_3() -> None:
    # 30 ms at the default rate: 36 bytes per tick, a pass with no telemetry due.
    rig = Rig(chunks=20)
    rig.begin_downlink()
    for _ in range(13):
        rig.step(capacity=36)
    delivered_in = {chunk: tick for chunk, tick in rig.received_tick.items()}
    first = len(rig.outputs) - 13  # the index of tick 1 of the table
    assert [delivered_in[c] - first + 1 for c in range(6)] == [3, 5, 7, 9, 11, 13]
    assert rig.chunks == list(range(6))  # 6 frames in 13 ticks
    assert sum(o.sent_bytes for o in rig.outputs[first:]) == 468 == 13 * 36
    # DATA 0 is released in tick 3's step f; the payload applies it in tick 4.
    assert rig.outputs[first + 2].controls.payload.release_through_chunk_id == 0
    assert rig.outputs[first + 1].controls.payload.release_through_chunk_id is None


# --- Order: continuation first, no interleaving, priority at commit ----------------------


def test_the_frame_in_transmission_continues_before_new_acks() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=36)  # DATA 0: 36 of 78
    rig.step(Command.ping(), Command.ping(), capacity=36)  # DATA 0: 72; ACKs held
    assert rig.per_tick[-1] == [] and rig.fc.held_frame_count == 2
    frames = rig.step(capacity=36)  # 6 + 14 + 14 = 34, then DATA 1 starts with 2
    assert types(frames) == [FrameType.DATA, FrameType.ACK, FrameType.ACK]
    sequences = [f.sequence for f in frames]
    assert sequences == list(range(sequences[0], sequences[0] + 3))
    assert rig.last.transmitting is not None
    assert rig.last.transmitting.sent_bytes == 2
    assert rig.last.transmitting.sequence == sequences[-1] + 1  # numbered at commit


def test_waiting_frames_are_committed_in_priority_order() -> None:
    # A DATA frame in transmission; a PING and a due telemetry frame wait behind it.
    # When it completes: the ACK, then telemetry, then the next DATA frame.
    config = dataclasses.replace(
        QUIET, telemetry=dataclasses.replace(QUIET.telemetry, downlink_period_us=2 * TICK_US)
    )
    rig = Rig(chunks=20, config=config)
    rig.begin_downlink()  # telemetry next due two steps later
    rig.step(capacity=1)  # DATA 0 committed: 1 byte
    rig.step(Command.ping(), capacity=1)  # DATA 0: 2 bytes; ACK and telemetry held
    assert rig.fc.held_frame_count == 2
    frames = rig.step(capacity=1000)
    assert types(frames)[:4] == [
        FrameType.DATA,
        FrameType.ACK,
        FrameType.TELEMETRY,
        FrameType.DATA,
    ]
    assert decode_data(frames[0].payload).chunk_id == 0
    assert decode_data(frames[3].payload).chunk_id == 1
    sequences = [f.sequence for f in frames]
    assert sequences == list(range(sequences[0], sequences[0] + len(frames)))


def test_frames_are_never_interleaved() -> None:
    # Whatever the capacities, the bytes on air are whole frames back to back: the
    # running total of transmitted bytes passes through every frame boundary, and the
    # frame in transmission's progress grows by exactly what the tick sent to it.
    rng = random.Random(132)
    rig = Rig(chunks=200)
    rig.begin_downlink()
    for _ in range(300):
        commands = [Command.ping()] * rng.choice([0, 0, 0, 1, 3])
        before = rig.fc.transmitting
        frames = rig.step(*commands, capacity=rng.randint(1, 77))
        if before is not None and not frames:
            now = rig.last.transmitting
            assert now is not None and now.sequence == before.sequence
            assert now.sent_bytes == before.sent_bytes + rig.last.sent_bytes
    assert rig.aborted_bytes == 0
    total = sum(len(encode_frame(f)) for f in rig.delivered)
    progress = rig.last.transmitting
    assert rig.transmitted == total + (progress.sent_bytes if progress else 0)


# --- ACK latency (ADR-0008 §5) ------------------------------------------------------------


@pytest.mark.parametrize("seed", range(5))
def test_ack_latency_follows_the_formula(seed: int) -> None:
    # The ACK of a command received in tick N is delivered in the first tick N + d with
    # C_N + ... + C_{N+d} >= r + A + 14 (r: what is left of the frame in transmission at
    # the start of tick N; A: the ACK bytes ahead of it).
    rng = random.Random(seed)
    rig = Rig(chunks=300)
    rig.begin_downlink()
    capacities: list[int] = []
    expected: dict[int, tuple[int, int]] = {}  # command sequence -> (tick, bytes ahead)
    for _ in range(400):
        n = len(rig.outputs)
        progress = rig.fc.transmitting
        r = progress.size_bytes - progress.sent_bytes if progress else 0
        ahead = r + ACK_FRAME_SIZE * (rig.fc.held_frame_count)
        commands = [Command.ping()] * rng.choice([0, 0, 0, 0, 1, 2])
        for k in range(len(commands)):
            expected[rig.sequence + k + 1] = (n, ahead + ACK_FRAME_SIZE * k)
        capacity = rng.randint(1, 77)
        capacities.append(capacity)
        rig.step(*commands, capacity=capacity)
    first = len(rig.outputs) - 400
    delivered: dict[int, int] = {}
    for tick, frames in enumerate(rig.per_tick):
        for frame in frames:
            if frame.frame_type is FrameType.ACK:
                delivered[decode_ack(frame.payload).sequence] = tick
    assert rig.last.outbound_suppressed_count == 0
    checked = 0
    for sequence, (n, ahead) in expected.items():
        need, total, d = ahead + ACK_FRAME_SIZE, 0, 0
        while True:
            if n - first + d >= len(capacities):
                break
            total += capacities[n - first + d]
            if total >= need:
                assert delivered[sequence] == n + d, (sequence, n, d)
                checked += 1
                break
            d += 1
    assert checked > 20


# --- Holding (ADR-0008 §5) ----------------------------------------------------------------


def nominal(config: FlightComputerConfig = QUIET) -> Rig:
    return Rig(config=config)


def test_held_acks_are_bounded_and_the_newest_is_suppressed() -> None:
    rig = nominal()
    rig.step(*[Command.ping()] * 10, capacity=1)
    # One ACK committed (1 byte), eight held, the tenth suppressed and counted.
    assert rig.fc.held_frame_count == 8
    assert rig.last.outbound_suppressed_count == 1
    for _ in range(20):
        rig.step(capacity=10)
    answered = [
        decode_ack(f.payload).sequence for f in rig.delivered if f.frame_type is FrameType.ACK
    ]
    assert answered == list(range(1, 10))  # in command order; never the tenth
    assert sum(o.outbound_suppressed_count for o in rig.outputs) == 1


@pytest.mark.parametrize("bound", [0, 1, 2, 8, 20])
def test_the_held_ack_bound_is_configurable(bound: int) -> None:
    rig = nominal(dataclasses.replace(QUIET, outbound=OutboundConfig(max_held_ack_frames=bound)))
    rig.step(*[Command.ping()] * 12, capacity=1)
    assert rig.fc.held_frame_count == min(bound, 11)
    assert rig.last.outbound_suppressed_count == 11 - min(bound, 11)


def test_held_acks_are_not_suppressed_in_a_streaming_tick() -> None:
    # The streaming counterpart of #51's 1-to-42-byte capacity boundaries: three ACKs
    # are never suppressed; each goes out when its bytes have been transmitted.
    for capacity in range(1, FRAME):
        rig = nominal()
        rig.step(*[Command.ping()] * 3, capacity=capacity)
        assert len(rig.per_tick[-1]) == min(3, capacity // ACK_FRAME_SIZE)
        assert rig.last.outbound_suppressed_count == 0
        assert rig.last.sent_bytes == min(capacity, 3 * ACK_FRAME_SIZE)
        while rig.fc.transmitting is not None or rig.fc.held_frame_count:
            rig.step(capacity=capacity)
        assert types(rig.delivered[-3:]) == [FrameType.ACK] * 3
        assert sum(o.sent_bytes for o in rig.outputs[1:]) == 3 * ACK_FRAME_SIZE
        assert len(rig.outputs) - 1 == -(-3 * ACK_FRAME_SIZE // capacity)  # no idle tick


@pytest.mark.parametrize("capacity", [35, 36, 63, 64])
def test_telemetry_waits_behind_acks_in_a_streaming_tick(capacity: int) -> None:
    # The streaming counterpart of #55's 35/36/63/64-byte cases: two ACKs and a mode
    # change's telemetry frame are all delivered, in priority order, none suppressed.
    rig = nominal()
    rig.step(Command.ping(), Command.set_mode(Mode.SCIENCE), capacity=capacity)
    first = len(rig.per_tick[-1])
    assert first == min(2, capacity // ACK_FRAME_SIZE) + (capacity >= 64)
    while rig.fc.transmitting is not None or rig.fc.held_frame_count:
        rig.step(capacity=capacity)
    assert types(rig.delivered[-3:]) == [FrameType.ACK, FrameType.ACK, FrameType.TELEMETRY]
    assert sum(o.outbound_suppressed_count for o in rig.outputs) == 0
    assert decode_telemetry(rig.delivered[-1].payload).mode is Mode.SCIENCE


def test_a_newer_telemetry_frame_replaces_a_held_one() -> None:
    every_tick = dataclasses.replace(QUIET, telemetry=TelemetryConfig.uniform(TICK_US))
    rig = nominal(every_tick)  # booted: the beacon went out at 1000 bytes
    rig.step(capacity=10)  # frame A committed: 10 of 36
    rig.step(capacity=10)  # A: 20; frame B held
    assert rig.fc.held_frame_count == 1 and rig.last.outbound_suppressed_count == 0
    rig.step(capacity=10)  # A: 30; frame C replaces B, which is counted
    assert rig.fc.held_frame_count == 1 and rig.last.outbound_suppressed_count == 1
    rig.step(capacity=10)  # A done (6), C committed (4); D held
    frames = rig.per_tick[-1]
    assert types(frames) == [FrameType.TELEMETRY]
    uptimes = [decode_telemetry(f.payload).uptime_ms for f in rig.delivered[-1:]]
    a_uptime = uptimes[0]
    rig.run(4, capacity=10)
    later = [decode_telemetry(f.payload).uptime_ms for f in rig.delivered]
    c_uptime = later[later.index(a_uptime) + 1]
    assert c_uptime == a_uptime + 200  # C, two ticks after A; B never went down


def test_with_the_default_periods_telemetry_keeps_its_period_while_streaming() -> None:
    # 36 bytes per tick through a busy pass with PINGs: a held telemetry frame always
    # goes out before the next is due, so every frame is created one period after the
    # last (its uptime) and none is suppressed. Delivery lags creation by at most the
    # frames ahead of it: up to 77 bytes of DATA and an ACK, three 36-byte ticks.
    config = dataclasses.replace(QUIET, telemetry=TelemetryConfig())
    rig = Rig(chunks=400, config=config)
    rig.begin_downlink()
    rng = random.Random(4)
    for _ in range(300):
        rig.step(*[Command.ping()] * rng.choice([0, 0, 0, 1]), capacity=36)
    assert sum(o.outbound_suppressed_count for o in rig.outputs) == 0
    delivered = [
        (n, decode_telemetry(f.payload).uptime_ms)
        for n, frames in enumerate(rig.per_tick)
        for f in frames
        if f.frame_type is FrameType.TELEMETRY
    ]
    uptimes = [uptime for _, uptime in delivered[1:]]  # after the boot-complete beacon
    assert len(uptimes) >= 29
    assert {b - a for a, b in itertools.pairwise(uptimes)} == {1000}
    for n, uptime in delivered[1:]:
        created = uptime // 100 - 1  # the step it was due in (step n ends at n+1 ticks)
        assert 0 <= n - created <= 3


# --- Sequence numbers and bytes (ADR-0008 §3) ---------------------------------------------


def test_sequence_numbers_follow_transmit_order_with_no_gaps() -> None:
    rng = random.Random(8)
    rig = Rig(chunks=200, config=dataclasses.replace(QUIET, telemetry=TelemetryConfig()))
    rig.begin_downlink()
    start = len(rig.delivered)
    for _ in range(300):
        rig.step(*[Command.ping()] * rng.choice([0, 0, 1, 2]), capacity=rng.randint(1, 200))
    sequences = [f.sequence for f in rig.delivered[start:]]
    assert sequences == list(range(sequences[0], sequences[0] + len(sequences)))


def test_sent_bytes_is_what_was_transmitted_not_what_was_delivered() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=50)
    assert rig.last.sent_bytes == 50 and rig.last.downlink_frames == ()
    rig.step(capacity=50)  # DATA 0 completes (28 bytes), DATA 1 starts (22)
    assert rig.last.sent_bytes == 50
    assert sum(map(len, rig.last.downlink_frames)) == FRAME > 50


# --- The streaming counterpart of #51's priority seam -------------------------------------

TELEMETRY_PAYLOAD = bytes(messages.TELEMETRY_PAYLOAD_SIZE)
DATA_PAYLOAD = messages.encode_data(messages.DataChunk(0, bytes(16)))


@pytest.mark.parametrize(
    ("capacity", "delivered", "telemetry_taken", "data_taken"),
    [
        (64, [FrameType.ACK, FrameType.ACK, FrameType.TELEMETRY], True, False),
        (63, [FrameType.ACK, FrameType.ACK], True, False),
        (28, [FrameType.ACK, FrameType.ACK], True, False),
        (14, [FrameType.ACK], True, False),
    ],
)
def test_frames_that_do_not_fit_a_streaming_tick_wait_rather_than_being_counted(
    capacity: int,
    delivered: list[FrameType],
    telemetry_taken: bool,
    data_taken: bool,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fc = FlightComputer()
    fc._mode_state = ModeState(Mode.NOMINAL)
    taken: list[bool] = []

    def emit_telemetry(tick: TickContext) -> None:
        taken.append(
            fc._queue_outbound(
                tick, FrameType.TELEMETRY, TELEMETRY_PAYLOAD, OutboundClass.TELEMETRY
            )
        )
        taken.append(fc._queue_outbound(tick, FrameType.DATA, DATA_PAYLOAD, OutboundClass.DATA))

    monkeypatch.setattr(fc, "_emit_telemetry", emit_telemetry)
    readings = dataclasses.replace(
        BASE, comms=dataclasses.replace(BASE.comms, transmit_capacity_bytes=capacity)
    )
    out = fc.step([frame_for(Command.ping(), n) for n in (1, 2)], readings, 0)
    assert [decode_frame(f).frame_type for f in out.downlink_frames] == delivered
    assert taken == [telemetry_taken, data_taken]  # telemetry held or committed
    assert out.outbound_suppressed_count == 0
    assert out.sent_bytes == capacity


# --- Interruptions (ADR-0008 §6) ----------------------------------------------------------


def test_transmitter_off_mid_data_aborts_the_frame_and_the_chunk_is_resent() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=36)
    rig.step(capacity=36)  # DATA 0: 72 of 78
    first_sequence = rig.fc.transmitting.sequence  # type: ignore[union-attr]
    rig.step(capacity=0)
    out = rig.last
    assert (out.outbound_aborted_count, out.outbound_suppressed_count) == (1, 0)
    assert out.transmitting is None and out.sent_bytes == 0
    assert rig.aborted_bytes == 72  # transmitted, charged, never delivered
    while not rig.chunks:
        rig.step(capacity=36)
    frame = next(f for f in rig.delivered if f.frame_type is FrameType.DATA)
    assert decode_data(frame.payload).chunk_id == 0
    assert frame.sequence == first_sequence + 1  # a new frame, a new number


def test_transmitter_off_mid_ack_and_mid_telemetry_suppresses_and_never_resends() -> None:
    rig = nominal()
    rig.step(Command.set_mode(Mode.SCIENCE), Command.ping(), capacity=5)
    # ACK 1 in transmission (5 of 14); ACK 2 and SCIENCE's telemetry frame held.
    assert rig.fc.held_frame_count == 2
    rig.step(Command.ping(), capacity=0)  # the new PING's ACK is suppressed too
    out = rig.last
    assert out.outbound_aborted_count == 3
    assert out.outbound_suppressed_count == 4
    assert rig.fc.held_frame_count == 0 and rig.fc.transmitting is None
    rig.run(5, capacity=120)
    assert all(frames == [] for frames in rig.per_tick[-6:])  # none is sent later
    assert rig.fc.mode is Mode.SCIENCE  # the command was executed all the same


def test_a_reboot_between_steps_aborts_and_counts_in_the_next_output() -> None:
    rig = nominal()
    rig.step(*[Command.ping()] * 3, capacity=5)  # one in transmission, two held
    rig.fc.reboot(now_us=rig.now_us)  # a forced_reset pulse
    assert rig.fc.transmitting is None and rig.fc.held_frame_count == 0
    rig.step(Command.ping(), capacity=120)
    out = rig.last
    assert (out.outbound_aborted_count, out.outbound_suppressed_count) == (3, 3)
    # A whole-frame tick again: the PING's ACK, then the one-tick boot's beacon.
    assert out.sent_bytes == ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE
    assert [decode_frame(f).sequence for f in out.downlink_frames] == [0, 1]


@pytest.mark.parametrize(
    "leave",
    [
        Command.enter_safe_mode(),
        Command.set_mode(Mode.NOMINAL),
    ],
    ids=["ENTER_SAFE_MODE", "SET_MODE"],
)
def test_leaving_downlink_mid_data_aborts_it_and_the_next_session_resends_the_chunk(
    leave: Command,
) -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=36)  # DATA 0: 36 of 78
    rig.step(leave, capacity=36)  # DATA 0: 72; the ACK waits; the mode leaves DOWNLINK
    out = rig.last
    assert rig.fc.mode is not Mode.DOWNLINK
    assert (out.outbound_aborted_count, out.outbound_suppressed_count) == (1, 0)
    assert out.transmitting is None
    rig.run(3, capacity=36)  # the ACK and the new mode's telemetry go out
    assert types(rig.delivered[-2:]) == [FrameType.ACK, FrameType.TELEMETRY]
    assert not rig.chunks
    if rig.fc.mode is Mode.SAFE:
        rig.step(Command.set_mode(Mode.NOMINAL), capacity=120)
    rig.step(Command.begin_downlink(), capacity=1000)
    assert rig.chunks[0] == 0  # the aborted chunk goes first, in a new frame


@pytest.mark.parametrize("cause", ["safe_flag", "fault"])
def test_safe_or_fault_entry_mid_data_aborts_it(cause: str) -> None:
    # One byte per tick, so a DATA frame is still in transmission when a flag sustained
    # for ten ticks enters SAFE, or a consistency check enters FAULT at once.
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=1)  # DATA 0 committed
    for _ in range(12):
        if cause == "fault":
            rig.step(capacity=1, fault=True)
        else:
            rig.step(capacity=1, flags=["critical_battery"])
        if rig.fc.mode is not Mode.DOWNLINK:
            break
    assert rig.fc.mode is (Mode.FAULT if cause == "fault" else Mode.SAFE)
    assert rig.last.outbound_aborted_count == 1
    assert rig.last.outbound_suppressed_count == 0
    assert not rig.chunks
    assert rig.readings().oldest_unreleased_chunk_id == 0  # nothing lost


def test_a_transmit_inhibit_lets_the_data_frame_in_transmission_finish() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=36)  # DATA 0: 36 of 78
    rig.run(4, capacity=36, flags=["low_battery"])  # DATA 0 finishes; nothing new
    assert rig.chunks == [0]
    assert rig.fc.transmitting is None and rig.fc.mode is Mode.DOWNLINK
    assert sum(o.sent_bytes for o in rig.outputs[-4:]) == FRAME - 36
    rig.run(3, capacity=36)  # the flag clears: the pass resumes at chunk 1
    assert rig.fc.transmitting is not None


def test_the_last_chunk_completing_with_downlink_complete_is_released() -> None:
    # The completion-release corner (ADR-0008 §6): the last chunk's frame completes at
    # the start of a tick whose step d raises DOWNLINK_COMPLETE; step f still releases
    # it, so it never goes down twice.
    rig = Rig(chunks=2)
    rig.begin_downlink()
    for _ in range(20):
        rig.step(capacity=36)
        if rig.fc.mode is not Mode.DOWNLINK:
            break
    assert rig.fc.mode is Mode.NOMINAL
    out = rig.last
    assert decode_data(rig.delivered[-1].payload).chunk_id == 1
    assert rig.received_tick[1] == len(rig.outputs) - 1  # completed in this very tick
    assert rig.fc.downlink_session is None
    assert out.controls.payload.release_through_chunk_id == 1
    assert out.controls == controls_for_mode(Mode.NOMINAL, release_through_chunk_id=1)
    rig.step(capacity=36)
    assert rig.readings().oldest_unreleased_chunk_id == 2  # both released, once each


def test_a_command_ending_the_pass_in_the_tick_a_chunk_completes_still_releases_it() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=60)  # DATA 0: 60 of 78
    rig.step(Command.set_mode(Mode.NOMINAL), capacity=18)  # DATA 0 completes; then the ACK
    assert rig.chunks == [0] and rig.fc.mode is Mode.NOMINAL
    assert rig.last.controls.payload.release_through_chunk_id == 0
    rig.step(capacity=120)
    assert rig.readings().oldest_unreleased_chunk_id == 1


# --- The RESET command waits for its ACK (ADR-0008 §6a), at the flight computer ----------


def test_a_reset_ack_overtakes_waiting_acks_and_the_reboot_waits_for_it() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=10)  # DATA 0: 10 of 78
    rig.step(Command.ping(), Command.ping(), capacity=10)  # DATA 0: 20; two ACKs held
    rig.step(Command.ping(), Command.reset(), Command.ping(), capacity=10)
    assert rig.fc.reboot_pending and rig.fc.mode is Mode.DOWNLINK
    assert rig.last.reboot_pending
    assert rig.last.controls == controls_for_mode(Mode.DOWNLINK)  # the old mode's
    boots = rig.fc.boot_count
    while rig.fc.reboot_pending:
        rig.step(Command.ping(), capacity=10)
        if rig.fc.reboot_pending:
            assert rig.last.uplink_ignored_count == 1  # ignored, not answered
    acks = [decode_ack(f.payload) for f in rig.delivered if f.frame_type is FrameType.ACK]
    # DATA 0 completes, then the RESET ACK, ahead of the four waiting PING ACKs.
    assert acks[1].command_id == CommandId.RESET  # after the BEGIN_DOWNLINK ACK
    assert rig.fc.boot_count == boots + 1 and rig.fc.mode is Mode.BOOT
    out = rig.last
    assert out.controls == controls_for_mode(Mode.BOOT)  # BOOT's, from the next tick
    # DATA 0 finished ahead of the ACK and was released in the step it completed.
    assert rig.chunks == [0]
    assert rig.outputs[rig.received_tick[0]].controls.payload.release_through_chunk_id == 0
    # The PING ACKs still waiting at the reboot are dropped and counted.
    assert out.outbound_suppressed_count >= 1
    assert rig.fc.uptime_us == 0


def test_two_resets_in_one_tick_reboot_twice_at_the_deferred_reboot() -> None:
    rig = Rig(chunks=20)
    rig.begin_downlink()
    rig.step(capacity=10)
    boots = rig.fc.boot_count
    rig.step(Command.reset(), Command.reset(), capacity=10)
    assert rig.fc.reboot_pending
    while rig.fc.reboot_pending:
        rig.step(capacity=10)
    # As today's update_mode applies two RESETs: two reboots, at the same time.
    assert rig.fc.boot_count == boots + 2
