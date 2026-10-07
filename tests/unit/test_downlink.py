"""Tests for the DOWNLINK session (#56): DATA frames, release, completion, abort, resume.

The real :class:`~pocketsat.flight.FlightComputer` runs against the real
:class:`~pocketsat.spacecraft.Payload` (its chunk store, #43), stepped with the flight
computer's own controls, so a release produced in tick N is applied by the payload in
tick N+1 exactly as in ``SilTarget`` (ADR-0004 §2). The other readings come from the
shared fakes (#76), with the transmit capacity and the power and thermal flags set per
tick. Every tick, :class:`Rig` checks the data accounting: produced = buffered +
released, every DATA frame carries the right content, chunks go out in order and once,
and sent - released is exactly the chunks sent in that tick, so never more than one
tick's worth (story #42's deferred criterion).
"""

import dataclasses
import random
from collections.abc import Sequence

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.flight import (
    EventKind,
    FlightComputer,
    FlightComputerConfig,
    FlightComputerOutput,
    Mode,
    ModeEvent,
    Outcome,
    SpacecraftReadings,
    controls_for_mode,
)
from pocketsat.flight.boot import BootConfig
from pocketsat.flight.computer import OutboundClass, TickContext
from pocketsat.flight.downlink import (
    DEFAULT_DOWNLINK_CONFIG,
    DownlinkConfig,
    DownlinkSession,
    release_through_chunk_id,
    transmit_inhibited,
    unsent_chunk_ids,
)
from pocketsat.frame import MAX_SEQUENCE, Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    TELEMETRY_FRAME_SIZE,
    Command,
    DataChunk,
    data_frame_size,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    MAX_CHUNK_SIZE_BYTES,
    NOMINAL_CONFIG,
    Payload,
    PayloadConfig,
    PayloadInitial,
    PayloadReadings,
    PayloadState,
    SnapshotBoard,
    SpacecraftControls,
    SubsystemStack,
    chunk_content,
)
from pocketsat.spacecraft.fakes import fake_stack, fake_subsystems
from pocketsat.targets.base import EnvironmentState
from pocketsat.targets.sil import SilTarget

TICK_US = 100_000
CHUNK = 64
FRAME = data_frame_size(CHUNK)  # 78
CAPACITY = NOMINAL_CONFIG.comms.transmit_capacity_bytes  # 120
FAST_BOOT = FlightComputerConfig(boot=BootConfig(duration_us=TICK_US))
"""Boot in one step, so a test reaches NOMINAL at once."""

BASE = SpacecraftReadings.from_state(fake_stack().snapshot())
"""The fakes' nominal readings: no flags, radio RX_TX, 120 bytes of capacity."""

FLAGS = ("low_battery", "critical_battery", "over_temp", "under_temp")
"""The power and thermal flags that inhibit DATA (#56)."""


def frame_for(command: Command, sequence: int = 1) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


def mode_of(rig: "Rig") -> Mode:
    """The flight computer's mode (a call, so mypy doesn't narrow it across steps)."""
    return rig.fc.mode


class Rig:
    """The real flight computer and the real payload, one tick at a time."""

    def __init__(
        self,
        chunks: int,
        *,
        chunk_size: int = CHUNK,
        config: FlightComputerConfig = FAST_BOOT,
        science: bool = False,
    ) -> None:
        capacity_bytes = chunk_size * 1000
        fill = (chunks * chunk_size + chunk_size // 2) / capacity_bytes  # + a partial chunk
        self.chunk_size = chunk_size
        # The payload reads its inhibits (power, thermal, attitude) only while enabled
        # (SCIENCE): the fakes' nominal snapshots, published on its board.
        board = SnapshotBoard()
        fakes = [fake for fake in fake_subsystems() if fake.name != "payload"]
        SubsystemStack(fakes, board=board).reset(RngFactory(0))
        self.payload = Payload(
            PayloadConfig(buffer_capacity_bytes=capacity_bytes, chunk_size_bytes=chunk_size),
            PayloadInitial(buffer_fill=fill),
            board,
        )
        self.payload.reset(RngFactory(0))
        assert self.readings().next_chunk_id == chunks
        self.fc = FlightComputer(config)
        self.controls: SpacecraftControls = self.fc.reset()
        self.now_us = 0
        self.sequence = 0
        self.sent: list[int] = []  # chunk IDs, in transmit order, over the whole run
        self.data_per_tick: list[list[int]] = []
        self.frames: list[tuple[bytes, ...]] = []
        self.outputs: list[FlightComputerOutput] = []
        self.step()  # boot
        assert mode_of(self) is Mode.NOMINAL
        if science:
            self.step(Command.set_mode(Mode.SCIENCE))
            assert mode_of(self) is Mode.SCIENCE

    def readings(self) -> PayloadReadings:
        return self.payload.snapshot().readings

    def step(
        self,
        *commands: Command,
        capacity: int = CAPACITY,
        flags: Sequence[str] = (),
        raw: Sequence[bytes] = (),
    ) -> list[int]:
        """Run one tick; return the chunk IDs sent in it."""
        self.payload.step(TICK_US, EnvironmentState(), self.controls)
        payload = self.readings()
        readings = dataclasses.replace(
            BASE,
            payload=payload,
            comms=dataclasses.replace(
                BASE.comms, transmit_capacity_bytes=capacity, transmitter_on=capacity > 0
            ),
            power=dataclasses.replace(
                BASE.power,
                low_battery="low_battery" in flags or "critical_battery" in flags,
                critical_battery="critical_battery" in flags,
            ),
            thermal=dataclasses.replace(
                BASE.thermal, over_temp="over_temp" in flags, under_temp="under_temp" in flags
            ),
        )
        uplink = [*raw]
        for command in commands:
            self.sequence += 1
            uplink.append(frame_for(command, self.sequence))
        self.now_us += TICK_US
        out = self.fc.step(uplink, readings, self.now_us)
        self.controls = out.controls
        self.frames.append(out.downlink_frames)
        self.outputs.append(out)
        assert out.sent_bytes <= capacity

        sent_now: list[int] = []
        for raw_frame in out.downlink_frames:
            frame = decode_frame(raw_frame)
            if frame.frame_type is FrameType.DATA:
                chunk = decode_data(frame.payload)
                assert chunk.content == chunk_content(chunk.chunk_id, self.chunk_size)
                sent_now.append(chunk.chunk_id)
        # DATA frames come last, in ID order, each chunk once, never skipping one.
        kinds = [decode_frame(f).frame_type for f in out.downlink_frames]
        assert kinds[len(kinds) - len(sent_now) :] == [FrameType.DATA] * len(sent_now)
        if sent_now:
            assert sent_now == list(range(sent_now[0], sent_now[-1] + 1))
            assert not self.sent or sent_now[0] > self.sent[-1]
        assert not set(sent_now) & set(self.sent)
        self.sent += sent_now
        self.data_per_tick.append(sent_now)

        # Data accounting, every tick: produced = buffered + released; nothing lost.
        assert payload.total_produced_bytes == payload.buffered_bytes + payload.total_released_bytes
        # Sent - released is exactly this tick's chunks: never more than one tick's worth.
        sent_bytes = len(self.sent) * self.chunk_size
        assert sent_bytes - payload.total_released_bytes == len(sent_now) * self.chunk_size
        # Every sent chunk is released through the next tick's controls, and nothing else.
        release = out.controls.payload.release_through_chunk_id
        if sent_now:
            assert release == sent_now[-1]
        elif release is not None:
            assert release < payload.oldest_unreleased_chunk_id  # a no-op repeat
        return sent_now

    def run(self, ticks: int, **kwargs: object) -> list[int]:
        sent: list[int] = []
        for _ in range(ticks):
            sent += self.step(**kwargs)  # type: ignore[arg-type]
        return sent

    def until_mode(self, mode: Mode, limit: int = 10_000) -> int:
        for n in range(limit):
            if mode_of(self) is mode:
                return n
            self.step()
        raise AssertionError(f"never reached {mode.name}")


# --- A whole session --------------------------------------------------------------------


@pytest.mark.parametrize("science", [False, True], ids=["from-NOMINAL", "from-SCIENCE"])
def test_a_session_sends_every_chunk_in_order_releases_them_and_returns(science: bool) -> None:
    rig = Rig(chunks=25, science=science)
    entry_mode = mode_of(rig)
    rig.step(Command.begin_downlink())
    assert mode_of(rig) is Mode.DOWNLINK
    assert rig.fc.downlink_session == DownlinkSession(start_chunk_id=0, next_chunk_id=0)
    # The entry tick carries the ACK and the mode-change telemetry frame (50 bytes): no
    # 78-byte DATA frame fits the 70 left.
    assert rig.data_per_tick[-1] == []

    ticks = rig.until_mode(entry_mode)
    # One DATA frame per tick (78 of 120 bytes, 114 with telemetry), then the step after
    # the last one raises DOWNLINK_COMPLETE.
    assert ticks == 26
    assert rig.sent == list(range(25))
    assert rig.data_per_tick[-26:-1] == [[n] for n in range(25)]
    assert rig.data_per_tick[-1] == []
    payload = rig.readings()
    assert payload.oldest_unreleased_chunk_id == payload.next_chunk_id == 25
    # Only the partial chunk is left (SCIENCE acquired a few bytes before DOWNLINK).
    assert payload.buffered_bytes < CHUNK
    assert payload.total_released_bytes == 25 * CHUNK
    assert rig.fc.downlink_session is None
    assert rig.controls == controls_for_mode(entry_mode)  # no release after the session
    # The completion step's telemetry frame shows the mode DOWNLINK returned to.
    [telemetry] = [
        decode_telemetry(decode_frame(f).payload)
        for f in rig.frames[-1]
        if decode_frame(f).frame_type is FrameType.TELEMETRY
    ]
    assert telemetry.mode is entry_mode


def test_downlink_complete_is_raised_only_once_no_unsent_chunk_is_left() -> None:
    rig = Rig(chunks=3)
    rig.step(Command.begin_downlink())
    events: list[list[EventKind]] = []
    original = rig.fc._update_mode

    def spy(tick: TickContext) -> None:
        events.append([e.kind for e in tick.mode_events])
        original(tick)

    rig.fc._update_mode = spy  # type: ignore[method-assign]
    rig.until_mode(Mode.NOMINAL)
    assert events == [[], [], [], [EventKind.DOWNLINK_COMPLETE]]


def test_an_empty_buffer_gives_a_one_step_session() -> None:
    # The session runs one step (the ground sees DOWNLINK in telemetry), then completes.
    rig = Rig(chunks=0)
    rig.step(Command.begin_downlink())
    assert mode_of(rig) is Mode.DOWNLINK
    rig.step()
    assert mode_of(rig) is Mode.NOMINAL
    assert rig.sent == []
    assert rig.readings().buffered_bytes == CHUNK // 2  # the partial chunk is never sent


def test_downlink_complete_is_not_raised_in_the_entry_step() -> None:
    rig = Rig(chunks=0)
    original = rig.fc._update_mode
    seen: list[list[EventKind]] = []

    def spy(tick: TickContext) -> None:
        seen.append([e.kind for e in tick.mode_events])
        original(tick)

    rig.fc._update_mode = spy  # type: ignore[method-assign]
    rig.step(Command.begin_downlink())
    assert seen == [[EventKind.BEGIN_DOWNLINK]]


def test_begin_downlink_during_a_session_does_not_restart_it() -> None:
    rig = Rig(chunks=10)
    rig.step(Command.begin_downlink())
    rig.run(4)
    before = rig.fc.downlink_session
    rig.step(Command.begin_downlink())  # ACK, no change (#47)
    assert rig.fc.downlink_session is not None
    assert before is not None
    assert rig.fc.downlink_session.start_chunk_id == before.start_chunk_id == 0
    rig.until_mode(Mode.NOMINAL)
    assert rig.sent == list(range(10))


# --- Session state (the hook for acknowledgement-based release) --------------------------


def test_session_state_records_what_was_sent() -> None:
    rig = Rig(chunks=40)
    rig.step(Command.begin_downlink())
    rig.run(7)
    assert rig.fc.downlink_session == DownlinkSession(
        start_chunk_id=0, next_chunk_id=7, sent_chunk_count=7, last_sent_chunk_id=6
    )


def test_the_release_rule_is_the_last_chunk_sent() -> None:
    assert release_through_chunk_id(None) is None
    assert release_through_chunk_id(DownlinkSession(3, 3)) is None
    assert release_through_chunk_id(DownlinkSession(3, 9, 6, 8)) == 8


def test_a_session_sends_from_its_own_next_chunk() -> None:
    # The session sends from its own record of what it sent, not only from the
    # payload's oldest unreleased chunk: with a release rule that waits (for an
    # acknowledgement, a later phase), nothing is sent twice.
    payload = PayloadReadings(PayloadState.OFF, 640, 6400, 0, 10, 640, 0, 0.0)
    assert unsent_chunk_ids(DownlinkSession(0, 4, 4, 3), payload) == range(4, 10)
    assert unsent_chunk_ids(DownlinkSession(0, 0), payload) == range(0, 10)
    # A payload that released more than was sent is never asked for a released chunk.
    released = dataclasses.replace(payload, oldest_unreleased_chunk_id=6, buffered_bytes=256)
    assert unsent_chunk_ids(DownlinkSession(0, 4, 4, 3), released) == range(6, 10)
    assert unsent_chunk_ids(DownlinkSession(0, 10, 10, 9), payload) == range(10, 10)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"start_chunk_id": -1, "next_chunk_id": 0},
        {"start_chunk_id": 5, "next_chunk_id": 4},
        {"start_chunk_id": 0, "next_chunk_id": 3, "sent_chunk_count": 3},
        {"start_chunk_id": 0, "next_chunk_id": 3, "last_sent_chunk_id": 2},
        {"start_chunk_id": 0, "next_chunk_id": 3, "sent_chunk_count": 3, "last_sent_chunk_id": 1},
    ],
)
def test_session_rejects_contradictory_state(kwargs: dict[str, int]) -> None:
    with pytest.raises(ValueError):
        DownlinkSession(**kwargs)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"start_chunk_id": 0.0, "next_chunk_id": 0},
        {"start_chunk_id": 0, "next_chunk_id": True},
        {"start_chunk_id": 0, "next_chunk_id": 1, "sent_chunk_count": 1, "last_sent_chunk_id": 0.0},
    ],
)
def test_session_rejects_wrong_types(kwargs: dict[str, object]) -> None:
    with pytest.raises(TypeError):
        DownlinkSession(**kwargs)  # type: ignore[arg-type]


# --- Rate limiting by transmit capacity -----------------------------------------------------


@pytest.mark.parametrize(
    ("capacity", "per_tick"),
    [(0, 0), (FRAME - 1, 0), (FRAME, 1), (2 * FRAME - 1, 1), (2 * FRAME, 2), (500, 6)],
)
def test_data_fills_the_capacity_with_whole_frames(capacity: int, per_tick: int) -> None:
    config = FlightComputerConfig(
        boot=BootConfig(duration_us=TICK_US),
        telemetry=dataclasses.replace(FAST_BOOT.telemetry, downlink_period_us=10**12),
    )
    rig = Rig(chunks=100, config=config)
    rig.step(Command.begin_downlink(), capacity=1000)  # entry tick: ACK, telemetry, DATA
    first = len(rig.sent)
    rig.run(5, capacity=capacity)  # no telemetry due in these ticks
    assert rig.data_per_tick[-5:] == [
        list(range(first + n * per_tick, first + (n + 1) * per_tick)) for n in range(5)
    ]


def test_data_stops_at_the_first_chunk_that_does_not_fit(monkeypatch: pytest.MonkeyPatch) -> None:
    # The outbound queue lets a later, smaller frame through (#51); the session offers
    # nothing after a chunk that does not fit, so chunks never go out of order.
    rig = Rig(chunks=20)
    rig.step(Command.begin_downlink())
    offered: list[OutboundClass] = []
    original = rig.fc._queue_outbound

    def spy(tick: TickContext, frame_type: FrameType, payload: bytes, cls: OutboundClass) -> bool:
        offered.append(cls)
        return original(tick, frame_type, payload, cls)

    monkeypatch.setattr(rig.fc, "_queue_outbound", spy)
    for capacity in (0, 77, 78, 155, 156, 240):
        offered.clear()
        sent = rig.step(capacity=capacity)
        assert offered.count(OutboundClass.DATA) == len(sent) == capacity // FRAME


def test_a_smaller_chunk_size_packs_more_frames() -> None:
    config = FlightComputerConfig(
        boot=BootConfig(duration_us=TICK_US), downlink=DownlinkConfig(chunk_size_bytes=16)
    )
    rig = Rig(chunks=50, chunk_size=16, config=config)
    rig.step(Command.begin_downlink())
    rig.step()
    assert len(rig.data_per_tick[-1]) == CAPACITY // data_frame_size(16) == 4


# --- Priority: ACK/NACK, then telemetry, then DATA ------------------------------------------


@pytest.mark.parametrize(
    ("capacity", "acks", "telemetry", "data"),
    [
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE + 2 * FRAME, 2, True, 2),
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE + FRAME, 2, True, 1),
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE + FRAME - 1, 2, True, 0),
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE, 2, True, 0),
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE - 1, 2, False, 0),
        (ACK_FRAME_SIZE, 1, False, 0),
        (0, 0, False, 0),
    ],
)
def test_acks_and_telemetry_go_ahead_of_data(
    capacity: int, acks: int, telemetry: bool, data: int
) -> None:
    # A tick with two PINGs and a telemetry frame due, in a pass with plenty to send.
    rig = Rig(chunks=50)
    rig.step(Command.begin_downlink())
    rig.run(9)  # the next telemetry frame is due 10 steps after the entry step
    sent = rig.step(Command.ping(), Command.ping(), capacity=capacity)
    kinds = [decode_frame(f).frame_type for f in rig.frames[-1]]
    expected = [FrameType.ACK] * acks + [FrameType.TELEMETRY] * telemetry
    assert kinds == expected + [FrameType.DATA] * data
    assert len(sent) == data
    # ACK/NACK and telemetry that don't fit are counted; DATA that doesn't fit is not.
    assert rig.outputs[-1].outbound_suppressed_count == (2 - acks) + (not telemetry)
    sequences = [decode_frame(f).sequence for f in rig.frames[-1]]
    assert sequences == sorted(sequences)  # one downlink counter for every frame type


def test_data_frames_take_the_shared_downlink_sequence() -> None:
    rig = Rig(chunks=10)
    rig.step(Command.begin_downlink())
    rig.until_mode(Mode.NOMINAL)
    sequences = [decode_frame(f).sequence for out in rig.frames for f in out]
    assert sequences == list(range(len(sequences)))
    assert sequences[-1] < MAX_SEQUENCE


# --- Radio-transmit inhibits --------------------------------------------------------------


@pytest.mark.parametrize("flag", FLAGS)
def test_a_flag_mid_pass_stops_data_in_that_tick_and_nothing_is_lost(flag: str) -> None:
    rig = Rig(chunks=30)
    rig.step(Command.begin_downlink())
    rig.run(5)
    assert len(rig.sent) == 5
    before = rig.readings()

    # The flag is set in this tick's readings: no DATA from this tick on, while ACKs and
    # telemetry still go out. Two ticks only, so SAFE (10 sustained ticks, #48) does not
    # interrupt the pass.
    assert rig.step(Command.ping(), flags=[flag]) == []
    assert [decode_frame(f).frame_type for f in rig.frames[-1]] == [FrameType.ACK]
    assert rig.step(flags=[flag]) == []
    assert mode_of(rig) is Mode.DOWNLINK
    # Everything sent before the flag was released (the last of it in the first flagged
    # tick); everything else stayed buffered.
    after = rig.readings()
    assert before.oldest_unreleased_chunk_id == 4
    assert after.oldest_unreleased_chunk_id == 5 == len(rig.sent)
    assert after.buffered_bytes == before.buffered_bytes - CHUNK
    assert after.total_produced_bytes == after.buffered_bytes + after.total_released_bytes

    # Cleared: the session resumes at the next chunk, with no gap and no repeat.
    rig.until_mode(Mode.NOMINAL)
    assert rig.sent == list(range(30))


@pytest.mark.parametrize("flag", FLAGS)
def test_transmit_inhibited_reads_each_power_and_thermal_flag(flag: str) -> None:
    assert not transmit_inhibited(BASE)
    if flag in ("low_battery", "critical_battery"):
        power = dataclasses.replace(BASE.power, **{flag: True})
        readings = dataclasses.replace(BASE, power=power)
    else:
        thermal = dataclasses.replace(BASE.thermal, **{flag: True})
        readings = dataclasses.replace(BASE, thermal=thermal)
    assert transmit_inhibited(readings)


def test_a_sustained_safe_flag_inhibits_data_then_safe_aborts_the_session() -> None:
    rig = Rig(chunks=50)
    rig.step(Command.begin_downlink())
    rig.run(3)
    for _ in range(9):  # sustained, not yet 10 ticks: still DOWNLINK, but no DATA
        assert rig.step(flags=["over_temp"]) == []
    assert mode_of(rig) is Mode.DOWNLINK
    rig.step(flags=["over_temp"])
    assert mode_of(rig) is Mode.SAFE
    assert rig.fc.downlink_session is None
    assert rig.sent == [0, 1, 2]
    assert rig.readings().oldest_unreleased_chunk_id == 3


# --- Abort and resume ------------------------------------------------------------------------


ABORTS: dict[str, tuple[Mode, dict[str, object]]] = {
    "ENTER_SAFE_MODE": (Mode.SAFE, {"commands": (Command.enter_safe_mode(),)}),
    "SET_MODE NOMINAL": (Mode.NOMINAL, {"commands": (Command.set_mode(Mode.NOMINAL),)}),
    "SET_MODE SCIENCE": (Mode.SCIENCE, {"commands": (Command.set_mode(Mode.SCIENCE),)}),
    "RESET": (Mode.BOOT, {"commands": (Command.reset(),)}),
}


@pytest.mark.parametrize("name", list(ABORTS))
def test_leaving_downlink_aborts_and_the_next_session_resumes(name: str) -> None:
    mode, kwargs = ABORTS[name]
    rig = Rig(chunks=30)
    rig.step(Command.begin_downlink())
    rig.run(10)
    assert rig.sent == list(range(10))

    commands = kwargs["commands"]
    assert isinstance(commands, tuple)
    sent = rig.step(*commands)
    assert mode_of(rig) is mode
    assert sent == []  # the mode after step d is not DOWNLINK: no DATA this tick
    assert rig.fc.downlink_session is None
    assert rig.controls.payload.release_through_chunk_id is None
    # Nothing lost: the 10 chunks sent were released, the other 20 are buffered.
    payload = rig.readings()
    assert payload.oldest_unreleased_chunk_id == 10
    assert payload.next_chunk_id == 30

    # Back to NOMINAL (SAFE needs SET_MODE NOMINAL; BOOT completes after one step).
    if mode is Mode.BOOT:
        rig.until_mode(Mode.NOMINAL)
    elif mode is not Mode.NOMINAL:
        rig.step(Command.set_mode(Mode.NOMINAL))
    assert mode_of(rig) is Mode.NOMINAL

    rig.step(Command.begin_downlink())
    session = rig.fc.downlink_session
    assert session is not None and session.start_chunk_id == 10  # the oldest unreleased
    rig.until_mode(Mode.NOMINAL)
    assert rig.sent == list(range(30))  # every chunk once, in order


def test_fault_detected_aborts_the_session() -> None:
    rig = Rig(chunks=30)
    rig.step(Command.begin_downlink())
    rig.run(4)
    # A non-finite reading is a consistency failure (#48): FAULT at once.
    rig.payload.step(TICK_US, EnvironmentState(), rig.controls)
    bad = dataclasses.replace(
        BASE,
        payload=rig.readings(),
        power=dataclasses.replace(BASE.power, bus_v=float("nan")),
    )
    rig.now_us += TICK_US
    out = rig.fc.step([], bad, rig.now_us)
    assert mode_of(rig) is Mode.FAULT
    assert rig.fc.downlink_session is None
    assert all(decode_frame(f).frame_type is not FrameType.DATA for f in out.downlink_frames)
    assert out.controls == controls_for_mode(Mode.FAULT)


def test_a_reboot_between_steps_aborts_the_session() -> None:
    # The forced_reset path (#60): FlightComputer.reboot() between steps.
    rig = Rig(chunks=30)
    rig.step(Command.begin_downlink())
    rig.run(6)
    rig.fc.reboot(now_us=rig.now_us)
    assert mode_of(rig) is Mode.BOOT
    assert rig.fc.downlink_session is None
    rig.until_mode(Mode.NOMINAL)
    rig.step(Command.begin_downlink())
    rig.until_mode(Mode.NOMINAL)
    assert rig.sent == list(range(30))


def test_downlink_complete_after_a_command_left_downlink_is_ignored() -> None:
    rig = Rig(chunks=2)
    rig.step(Command.begin_downlink())
    rig.run(2)  # both chunks sent; the next step would complete
    original = rig.fc._update_mode
    outcomes: list[list[Outcome]] = []

    def spy(tick: TickContext) -> None:
        original(tick)
        outcomes.append([t.outcome for t in tick.transitions])

    rig.fc._update_mode = spy  # type: ignore[method-assign]
    rig.step(Command.set_mode(Mode.SCIENCE))
    assert mode_of(rig) is Mode.SCIENCE
    assert outcomes == [[Outcome.TRANSITION, Outcome.IGNORED]]


# --- Controls ---------------------------------------------------------------------------


def test_controls_for_mode_carries_the_release() -> None:
    controls = controls_for_mode(Mode.DOWNLINK, release_through_chunk_id=41)
    assert controls.payload.release_through_chunk_id == 41
    assert dataclasses.replace(
        controls, payload=dataclasses.replace(controls.payload, release_through_chunk_id=None)
    ) == controls_for_mode(Mode.DOWNLINK)
    assert controls_for_mode(Mode.DOWNLINK) is controls_for_mode(Mode.DOWNLINK)
    with pytest.raises(ValueError):
        controls_for_mode(Mode.DOWNLINK, release_through_chunk_id=-1)


def test_the_release_is_in_the_controls_of_the_tick_that_sent() -> None:
    rig = Rig(chunks=5)
    rig.step(Command.begin_downlink())
    assert rig.controls.payload.release_through_chunk_id is None  # nothing sent yet
    sent = rig.step()
    assert rig.controls.payload.release_through_chunk_id == sent[-1] == 0
    assert rig.controls.payload.enabled is False  # DOWNLINK: payload off (#47)
    assert rig.readings().oldest_unreleased_chunk_id == 0  # released in the next tick
    rig.step()
    assert rig.readings().oldest_unreleased_chunk_id == 1


# --- Settings ---------------------------------------------------------------------------


def test_downlink_config_defaults_to_the_payload_chunk_size() -> None:
    assert DEFAULT_DOWNLINK_CONFIG.chunk_size_bytes == NOMINAL_CONFIG.payload.chunk_size_bytes
    assert FlightComputerConfig().downlink == DEFAULT_DOWNLINK_CONFIG


@pytest.mark.parametrize(
    ("size", "error"),
    [(0, ValueError), (MAX_CHUNK_SIZE_BYTES + 1, ValueError), (64.0, TypeError), (True, TypeError)],
)
def test_downlink_config_validates(size: object, error: type[Exception]) -> None:
    with pytest.raises(error):
        DownlinkConfig(chunk_size_bytes=size)  # type: ignore[arg-type]


def test_flight_computer_config_checks_the_downlink_type() -> None:
    with pytest.raises(TypeError, match="downlink"):
        FlightComputerConfig(downlink=64)  # type: ignore[arg-type]


def test_sil_target_rejects_a_flight_computer_built_for_another_chunk_size() -> None:
    config = FlightComputerConfig(downlink=DownlinkConfig(chunk_size_bytes=32))
    target = SilTarget(flight_computer_factory=lambda: FlightComputer(config))
    with pytest.raises(ValueError, match="chunk_size_bytes"):
        target.reset(1)


# --- Determinism -------------------------------------------------------------------------


def _random_pass(seed: int) -> list[tuple[bytes, ...]]:
    rng = random.Random(seed)
    rig = Rig(chunks=200)
    rig.step(Command.begin_downlink())
    for _ in range(150):
        flags = [rng.choice(FLAGS)] if rng.random() < 0.05 else []
        commands = [Command.ping()] if rng.random() < 0.2 else []
        rig.step(*commands, capacity=rng.choice([0, 78, 120, 200, 400]), flags=flags)
    return rig.frames


def test_a_pass_is_deterministic() -> None:
    assert _random_pass(5) == _random_pass(5)
    assert _random_pass(5) != _random_pass(6)


def test_data_frames_decode_to_data_chunks() -> None:
    rig = Rig(chunks=3)
    rig.step(Command.begin_downlink())
    rig.step()
    [frame] = [decode_frame(f) for f in rig.frames[-1]]
    assert decode_data(frame.payload) == DataChunk(0, chunk_content(0, CHUNK))
    # The entry tick before it: the BEGIN_DOWNLINK ACK, then the telemetry frame.
    ack = decode_frame(rig.frames[-2][0])
    assert decode_ack(ack.payload).accepted


def test_mode_event_for_completion_is_automatic() -> None:
    assert not EventKind.DOWNLINK_COMPLETE.is_command
    assert ModeEvent(EventKind.DOWNLINK_COMPLETE).target is None
