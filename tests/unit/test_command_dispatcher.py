"""Tests for the command dispatcher and the outbound queue (#51).

The dispatcher is the flight computer's decode-uplink and execute-commands phases: every
COMMAND frame gets one ACK frame back (an ACK, or a NACK with a reason), queued first in
the downlink within comms' transmit capacity. Expected answers come from #47's
``transition()`` (the transition table) and #52's ``decode_command``, so these tests
check the wiring between them rather than restating either table, plus a few explicit
rows from the issue.
"""

import dataclasses
import random
from collections.abc import Callable, Sequence

import pytest

from pocketsat import messages
from pocketsat.flight import (
    ALL_EVENTS,
    EventKind,
    FlightComputer,
    Mode,
    ModeEvent,
    ModeState,
    Outcome,
    RejectReason,
    SpacecraftReadings,
    Transition,
    controls_for_mode,
    transition,
)
from pocketsat.flight.computer import OutboundClass, TickContext
from pocketsat.frame import (
    MAX_SEQUENCE,
    Frame,
    FrameType,
    decode_frame,
    encode_frame,
)
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    Command,
    CommandAck,
    CommandId,
    DecodeReason,
    MalformedCommand,
    ParsedCommand,
    decode_ack,
    decode_command,
    encode_command,
)
from pocketsat.spacecraft.fakes import fake_stack

TICK_US = 100_000

_FAKE_READINGS = SpacecraftReadings.from_state(fake_stack().snapshot())
READINGS = dataclasses.replace(
    _FAKE_READINGS,
    payload=dataclasses.replace(
        _FAKE_READINGS.payload,
        buffered_bytes=64_000,
        next_chunk_id=1_000,
        total_produced_bytes=64_000,
    ),
)
"""Nominal readings: no flags, radio RX_TX, 120 bytes of transmit capacity, and a payload
backlog of 1000 chunks, so a DOWNLINK session (#56) has data to send and DOWNLINK
lasts."""

CAPACITY = READINGS.comms.transmit_capacity_bytes


def with_capacity(capacity: int) -> SpacecraftReadings:
    """``READINGS`` with comms' transmit capacity set; 0 also turns the transmitter off,
    as ``transmitter_off`` does (#60, ADR-0007 §5)."""
    comms = dataclasses.replace(
        READINGS.comms, transmit_capacity_bytes=capacity, transmitter_on=capacity > 0
    )
    return dataclasses.replace(READINGS, comms=comms)


FLAGGED = dataclasses.replace(
    READINGS,
    power=dataclasses.replace(READINGS.power, low_battery=True, critical_battery=True),
)
"""Readings with a safe-mode flag set (``critical_battery``), so leaving SAFE is refused."""

STATES: list[ModeState] = [
    ModeState(Mode.BOOT),
    ModeState(Mode.NOMINAL),
    ModeState(Mode.SCIENCE),
    ModeState(Mode.DOWNLINK, return_mode=Mode.NOMINAL),
    ModeState(Mode.DOWNLINK, return_mode=Mode.SCIENCE),
    ModeState(Mode.SAFE),
    ModeState(Mode.FAULT),
]
"""Every mode, DOWNLINK from both return modes."""

VALID_COMMANDS: list[Command] = [
    Command.ping(),
    *(Command.set_mode(mode) for mode in Mode),
    Command.begin_downlink(),
    Command.enter_safe_mode(),
    Command.reset(),
]
"""Every valid command, SET_MODE once per mode."""

MALFORMED_PAYLOADS: list[tuple[bytes, int, DecodeReason]] = [
    (b"", 0x00, DecodeReason.PAYLOAD_TRUNCATED),
    (b"\x00", 0x00, DecodeReason.UNKNOWN_COMMAND),
    (b"\x06", 0x06, DecodeReason.UNKNOWN_COMMAND),
    (b"\xff\x01\x02", 0xFF, DecodeReason.UNKNOWN_COMMAND),
    (b"\x02", 0x02, DecodeReason.PAYLOAD_TRUNCATED),
    (b"\x01\x00", 0x01, DecodeReason.PAYLOAD_TOO_LONG),
    (b"\x05\x05", 0x05, DecodeReason.PAYLOAD_TOO_LONG),
    (b"\x02\x01\x01", 0x02, DecodeReason.PAYLOAD_TOO_LONG),
    (b"\x02\x06", 0x02, DecodeReason.INVALID_ARGUMENT),
    (b"\x02\xff", 0x02, DecodeReason.INVALID_ARGUMENT),
]
"""COMMAND payloads malformed inside a valid frame: (payload, echoed ID, reason)."""


def command_frame(payload: bytes, sequence: int = 1) -> bytes:
    """A valid COMMAND frame carrying ``payload``."""
    return encode_frame(Frame(FrameType.COMMAND, sequence, payload))


def frame_for(command: Command, sequence: int = 1) -> bytes:
    """A valid COMMAND frame carrying ``command``."""
    return command_frame(encode_command(command), sequence)


class AckOnlyComputer(FlightComputer):
    """The flight computer with its emit-telemetry phase switched off.

    These tests check the dispatcher and the outbound queue through ACK/NACK frames
    alone; telemetry (#55) would add a frame at every mode change. Telemetry's place in
    the queue (after the ACKs, sharing the capacity and the sequence counter) is tested
    in ``test_telemetry_scheduler.py``.
    """

    def _emit_telemetry(self, tick: TickContext) -> None:
        pass


def in_state(state: ModeState) -> FlightComputer:
    """A flight computer in ``state``, with telemetry off (:class:`AckOnlyComputer`).

    Set directly, so each test reaches its mode in one step instead of waiting out
    the 5 s boot (#49) and commanding the mode first.
    """
    fc = AckOnlyComputer()
    fc._mode_state = state
    return fc


def answers(frames: Sequence[bytes]) -> list[CommandAck]:
    """Decode downlink frames that must all be ACK frames."""
    decoded = [decode_frame(frame) for frame in frames]
    assert all(frame.frame_type is FrameType.ACK for frame in decoded)
    return [decode_ack(frame.payload) for frame in decoded]


def expected_transition(
    state: ModeState, command: Command, *, safe_exit_allowed: bool = True
) -> Transition | None:
    """#47's answer for a valid command; ``None`` for PING, which raises no event."""
    parsed = decode_command(encode_command(command))
    assert isinstance(parsed, ParsedCommand)
    if parsed.command_id is CommandId.PING:
        return None
    event = ModeEvent(EventKind[parsed.command_id.name], parsed.target)
    return transition(state, event, safe_exit_allowed=safe_exit_allowed)


def capture_ticks(fc: FlightComputer, monkeypatch: pytest.MonkeyPatch) -> list[TickContext]:
    """Record each tick's context as the update-mode phase finishes with it."""
    seen: list[TickContext] = []
    original: Callable[[TickContext], None] = fc._update_mode

    def update_mode(tick: TickContext) -> None:
        original(tick)
        seen.append(tick)

    monkeypatch.setattr(fc, "_update_mode", update_mode)
    return seen


# --- Every command in every mode ----------------------------------------------------------


@pytest.mark.parametrize("state", STATES, ids=lambda s: f"{s.mode.name}-{s.return_mode}")
@pytest.mark.parametrize("command", VALID_COMMANDS, ids=repr)
def test_every_command_in_every_mode_follows_the_transition_table(
    state: ModeState, command: Command
) -> None:
    fc = in_state(state)
    out = fc.step([frame_for(command, sequence=0x1234)], READINGS, 0)

    expected = expected_transition(state, command)
    reason = None if expected is None else expected.reason
    assert answers(out.downlink_frames) == [CommandAck(0x1234, command.command_id, reason)]
    assert fc.mode_state == (state if expected is None else expected.state)
    assert out.controls == controls_for_mode(fc.mode)
    assert out.outbound_suppressed_count == 0
    assert out.sent_bytes == ACK_FRAME_SIZE


@pytest.mark.parametrize("command", VALID_COMMANDS, ids=repr)
def test_boot_nacks_everything_but_reset_and_ping(command: Command) -> None:
    out = FlightComputer().step([frame_for(command)], READINGS, 0)
    [ack] = answers(out.downlink_frames)
    if command.command_id in (CommandId.PING, CommandId.RESET):
        assert ack.accepted
    else:
        assert ack.reason is RejectReason.BOOT_IN_PROGRESS


@pytest.mark.parametrize("command", VALID_COMMANDS, ids=repr)
def test_fault_nacks_everything_but_reset_and_ping(command: Command) -> None:
    fc = in_state(ModeState(Mode.FAULT))
    [ack] = answers(fc.step([frame_for(command)], READINGS, 0).downlink_frames)
    if command.command_id in (CommandId.PING, CommandId.RESET):
        assert ack.accepted
    else:
        assert ack.reason is RejectReason.FAULT_REQUIRES_RESET


@pytest.mark.parametrize("state", STATES, ids=lambda s: f"{s.mode.name}-{s.return_mode}")
def test_ping_is_acked_in_every_mode_and_raises_no_event(
    state: ModeState, monkeypatch: pytest.MonkeyPatch
) -> None:
    fc = in_state(state)
    ticks = capture_ticks(fc, monkeypatch)
    out = fc.step([frame_for(Command.ping(), sequence=9)], READINGS, 0)
    assert answers(out.downlink_frames) == [CommandAck(9, CommandId.PING)]
    assert ticks[0].mode_events == []
    assert fc.mode_state == state


def test_reset_is_acked_and_raised_as_a_mode_event(monkeypatch: pytest.MonkeyPatch) -> None:
    # The dispatcher only raises the event; the update-mode phase applies it and
    # reboots (#49, tests/unit/test_boot_reset.py).
    fc = in_state(ModeState(Mode.SCIENCE))
    ticks = capture_ticks(fc, monkeypatch)
    out = fc.step([frame_for(Command.reset(), sequence=3)], READINGS, 0)
    assert answers(out.downlink_frames) == [CommandAck(3, CommandId.RESET)]
    assert ticks[0].mode_events == [ModeEvent(EventKind.RESET)]
    assert fc.mode is Mode.BOOT
    assert out.controls == controls_for_mode(Mode.BOOT)


def test_set_mode_nominal_leaves_safe_only_when_this_ticks_flags_are_clear() -> None:
    fc = in_state(ModeState(Mode.SAFE))
    out = fc.step([frame_for(Command.set_mode(Mode.NOMINAL), sequence=1)], FLAGGED, 0)
    assert answers(out.downlink_frames) == [
        CommandAck(1, CommandId.SET_MODE, RejectReason.SAFE_CONDITIONS_ACTIVE)
    ]
    assert fc.mode_state == ModeState(Mode.SAFE)

    out = fc.step([frame_for(Command.set_mode(Mode.NOMINAL), sequence=2)], READINGS, TICK_US)
    assert answers(out.downlink_frames) == [CommandAck(2, CommandId.SET_MODE)]
    assert fc.mode is Mode.NOMINAL


def test_command_ids_map_to_command_events() -> None:
    # Every command except PING raises the EventKind of the same name (docs/protocol.md).
    for command_id in CommandId:
        if command_id is CommandId.PING:
            assert command_id.name not in EventKind.__members__
        else:
            assert EventKind[command_id.name].is_command
    commands = {event.kind for event in ALL_EVENTS if event.kind.is_command}
    assert commands == {EventKind[c.name] for c in CommandId if c is not CommandId.PING}


# --- Malformed commands and check order ---------------------------------------------------


@pytest.mark.parametrize("state", STATES, ids=lambda s: f"{s.mode.name}-{s.return_mode}")
@pytest.mark.parametrize(("payload", "command_id", "reason"), MALFORMED_PAYLOADS)
def test_malformed_commands_are_nacked_with_the_decode_reason_in_every_mode(
    state: ModeState, payload: bytes, command_id: int, reason: DecodeReason
) -> None:
    assert decode_command(payload) == MalformedCommand(command_id, reason)  # the codec's view
    fc = in_state(state)
    out = fc.step([command_frame(payload, sequence=77)], READINGS, 0)
    assert answers(out.downlink_frames) == [CommandAck(77, command_id, reason)]
    assert fc.mode_state == state


@pytest.mark.parametrize(
    ("state", "payload", "reason"),
    [
        (ModeState(Mode.BOOT), b"\x09", DecodeReason.UNKNOWN_COMMAND),
        (ModeState(Mode.BOOT), b"\x02\x07", DecodeReason.INVALID_ARGUMENT),
        (ModeState(Mode.BOOT), b"\x03\x00", DecodeReason.PAYLOAD_TOO_LONG),
        (ModeState(Mode.FAULT), b"\x02", DecodeReason.PAYLOAD_TRUNCATED),
        (ModeState(Mode.SAFE), b"\x02\x10", DecodeReason.INVALID_ARGUMENT),
        (ModeState(Mode.NOMINAL), b"\x02\x08", DecodeReason.INVALID_ARGUMENT),
    ],
)
def test_decoding_reasons_come_before_mode_reasons(
    state: ModeState, payload: bytes, reason: DecodeReason
) -> None:
    # The same command, well formed, would get a mode reason (or an ACK) in that mode.
    [ack] = answers(in_state(state).step([command_frame(payload)], READINGS, 0).downlink_frames)
    assert ack.reason is reason


def test_a_valid_but_uncommandable_target_gets_the_mode_reason() -> None:
    # SET_MODE FAULT decodes (FAULT is a mode), so the state machine answers it.
    fc = in_state(ModeState(Mode.NOMINAL))
    out = fc.step([frame_for(Command.set_mode(Mode.FAULT))], READINGS, 0)
    [ack] = answers(out.downlink_frames)
    assert ack.reason is RejectReason.TARGET_NOT_COMMANDABLE


@pytest.mark.parametrize(
    "frame",
    [
        b"",
        b"\xa5\x5a",
        command_frame(b"\x01")[:-1],  # truncated
        command_frame(b"\x01")[:-2] + b"\x00\x00",  # bad CRC
        encode_frame(Frame(FrameType.TELEMETRY, 1, b"\x01")),  # wrong direction
        encode_frame(Frame(FrameType.ACK, 1, b"\x00\x01\x01\x00")),  # wrong direction
    ],
)
def test_frames_that_are_not_valid_command_frames_get_no_reply(frame: bytes) -> None:
    # SilTarget drops and counts undecodable frames before they get here (#59); the
    # flight computer still never raises on one, and never answers it.
    fc = in_state(ModeState(Mode.NOMINAL))
    out = fc.step([frame], READINGS, 0)
    assert out.downlink_frames == ()
    assert out.outbound_suppressed_count == 0
    assert fc.mode is Mode.NOMINAL


# --- Several commands in one tick -------------------------------------------------------------


def test_commands_in_one_tick_are_answered_in_arrival_order() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    frames = [
        frame_for(Command.set_mode(Mode.SCIENCE), sequence=10),
        frame_for(Command.ping(), sequence=11),
        command_frame(b"\x42", sequence=12),
        frame_for(Command.begin_downlink(), sequence=13),
    ]
    out = fc.step(frames, READINGS, 0)
    assert answers(out.downlink_frames) == [
        CommandAck(10, CommandId.SET_MODE),
        CommandAck(11, CommandId.PING),
        CommandAck(12, 0x42, DecodeReason.UNKNOWN_COMMAND),
        CommandAck(13, CommandId.BEGIN_DOWNLINK),
    ]
    assert fc.mode_state == ModeState(Mode.DOWNLINK, return_mode=Mode.SCIENCE)


def test_each_command_is_judged_after_the_ones_before_it() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    frames = [
        frame_for(Command.enter_safe_mode(), sequence=1),
        frame_for(Command.set_mode(Mode.SCIENCE), sequence=2),  # now in SAFE
        frame_for(Command.reset(), sequence=3),
        frame_for(Command.set_mode(Mode.NOMINAL), sequence=4),  # now in BOOT
        frame_for(Command.ping(), sequence=5),
    ]
    out = fc.step(frames, READINGS, 0)
    assert [a.reason for a in answers(out.downlink_frames)] == [
        None,
        RejectReason.NOT_ALLOWED_IN_SAFE,
        None,
        RejectReason.BOOT_IN_PROGRESS,
        None,
    ]
    assert fc.mode is Mode.BOOT


def test_the_same_command_twice_is_answered_twice() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    frame = frame_for(Command.set_mode(Mode.SCIENCE), sequence=8)
    out = fc.step([frame, frame], READINGS, 0)
    # The retry is a NO_CHANGE, so a ground retry after a lost ACK succeeds (#47).
    assert answers(out.downlink_frames) == [CommandAck(8, CommandId.SET_MODE)] * 2


@pytest.mark.parametrize("seed", range(20))
def test_acks_agree_with_the_transitions_the_update_mode_phase_applies(
    seed: int, monkeypatch: pytest.MonkeyPatch
) -> None:
    # The ACK is decided in step c; the event is applied in step d. Random command mixes
    # from random modes, flags on and off, must give the same result in both.
    rng = random.Random(seed)
    fc = in_state(rng.choice(STATES))
    ticks = capture_ticks(fc, monkeypatch)
    for n in range(20):
        commands = [rng.choice(VALID_COMMANDS) for _ in range(rng.randrange(0, 6))]
        readings = rng.choice([READINGS, FLAGGED])
        frames = [frame_for(c, sequence=i) for i, c in enumerate(commands)]
        out = fc.step(frames, readings, n * TICK_US)

        tick = ticks[-1]
        mode_answers = [a for a in answers(out.downlink_frames) if a.command_id != CommandId.PING]
        applied = tick.transitions[: len(mode_answers)]
        assert len(applied) == len(mode_answers)
        for ack, result in zip(mode_answers, applied, strict=True):
            assert ack.reason == result.reason
            assert ack.accepted == result.acknowledged
            assert result.outcome is not Outcome.IGNORED


# --- Downlink sequence numbers --------------------------------------------------------------


def test_ack_frames_carry_the_downlink_sequence_and_name_the_command_sequence() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    sequences: list[int] = []
    for n in range(3):
        out = fc.step(
            [frame_for(Command.ping(), sequence=500 + n), frame_for(Command.ping(), sequence=7)],
            READINGS,
            n * TICK_US,
        )
        sequences.extend(decode_frame(frame).sequence for frame in out.downlink_frames)
        assert [a.sequence for a in answers(out.downlink_frames)] == [500 + n, 7]
    assert sequences == [0, 1, 2, 3, 4, 5]


def test_the_downlink_sequence_wraps() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    fc._downlink_sequence = MAX_SEQUENCE
    out = fc.step([frame_for(Command.ping())] * 2, READINGS, 0)
    assert [decode_frame(f).sequence for f in out.downlink_frames] == [MAX_SEQUENCE, 0]


def test_the_downlink_sequence_restarts_on_reboot_and_power_on() -> None:
    fc = FlightComputer()
    fc.step([frame_for(Command.ping())] * 3, READINGS, 0)
    fc.reboot(now_us=TICK_US)
    out = fc.step([frame_for(Command.ping())], READINGS, TICK_US)
    assert decode_frame(out.downlink_frames[0]).sequence == 0
    fc.step([frame_for(Command.ping())] * 2, READINGS, 2 * TICK_US)
    fc.reset()
    out = fc.step([frame_for(Command.ping())], READINGS, 0)
    assert decode_frame(out.downlink_frames[0]).sequence == 0


def test_suppressed_frames_use_no_sequence_number() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    fc.step([frame_for(Command.ping())] * 3, with_capacity(0), 0)
    out = fc.step([frame_for(Command.ping())], READINGS, TICK_US)
    assert decode_frame(out.downlink_frames[0]).sequence == 0


# --- Transmit capacity and suppression ------------------------------------------------------


@pytest.mark.parametrize("count", [0, 1, 7, 8, 9, 20])
def test_acks_fill_the_capacity_in_order_and_the_rest_are_counted(count: int) -> None:
    # 120 bytes of capacity fit 8 ACK frames of 14 bytes.
    fits = CAPACITY // ACK_FRAME_SIZE
    assert fits == 8
    fc = in_state(ModeState(Mode.NOMINAL))
    frames = [frame_for(Command.ping(), sequence=n) for n in range(count)]
    out = fc.step(frames, READINGS, 0)
    sent = min(count, fits)
    assert [a.sequence for a in answers(out.downlink_frames)] == list(range(sent))
    assert out.outbound_suppressed_count == count - sent
    assert out.sent_bytes == sent * ACK_FRAME_SIZE == sum(map(len, out.downlink_frames))
    assert out.sent_bytes <= CAPACITY


@pytest.mark.parametrize(
    ("capacity", "sent"),
    [(0, 0), (1, 0), (13, 0), (14, 1), (27, 1), (28, 2), (41, 2), (42, 3), (1000, 3)],
)
def test_capacity_boundaries(capacity: int, sent: int) -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    out = fc.step([frame_for(Command.ping())] * 3, with_capacity(capacity), 0)
    assert len(out.downlink_frames) == sent
    assert out.outbound_suppressed_count == 3 - sent
    assert out.sent_bytes == sent * ACK_FRAME_SIZE


def test_with_the_transmitter_off_commands_execute_but_every_ack_is_suppressed() -> None:
    # ADR-0004 §10, ADR-0007 §5: the receiver works, so the spacecraft hears and executes
    # commands, but cannot reply; every ACK/NACK is counted as suppressed.
    fc = in_state(ModeState(Mode.NOMINAL))
    frames = [
        frame_for(Command.set_mode(Mode.SCIENCE)),
        frame_for(Command.set_mode(Mode.SAFE)),  # NACK, suppressed too
        command_frame(b""),  # malformed: NACK, suppressed too
        frame_for(Command.ping()),
    ]
    out = fc.step(frames, with_capacity(0), 0)
    assert out.downlink_frames == ()
    assert out.sent_bytes == 0
    assert out.outbound_suppressed_count == 4
    assert fc.mode is Mode.SCIENCE
    assert out.controls == controls_for_mode(Mode.SCIENCE)


def test_suppression_is_counted_per_tick_not_accumulated() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    assert (
        fc.step([frame_for(Command.ping())] * 2, with_capacity(0), 0).outbound_suppressed_count == 2
    )
    assert fc.step([], with_capacity(0), TICK_US).outbound_suppressed_count == 0
    assert (
        fc.step([frame_for(Command.ping())], READINGS, 2 * TICK_US).outbound_suppressed_count == 0
    )


def test_capacity_returning_lets_acks_out_again() -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    out = fc.step([frame_for(Command.ping(), sequence=1)], with_capacity(0), 0)
    assert out.downlink_frames == ()
    out = fc.step([frame_for(Command.ping(), sequence=2)], READINGS, TICK_US)
    # The suppressed ACK is not queued for later: only the new command is answered.
    assert answers(out.downlink_frames) == [CommandAck(2, CommandId.PING)]


# --- Priority: ACK/NACK before telemetry and DATA (the outbound seam) -----------------------
#
# These pin the seam itself (#51) with frames of chosen sizes, including a DATA frame
# smaller than a telemetry frame, which no real chunk size here produces. The real
# telemetry scheduler and downlink session through the same seam are tested in
# test_telemetry_scheduler.py (#55) and test_downlink.py (#56).

TELEMETRY_PAYLOAD = bytes(messages.TELEMETRY_PAYLOAD_SIZE)
DATA_PAYLOAD = messages.encode_data(messages.DataChunk(0, bytes(16)))
"""A 36-byte telemetry frame and a 30-byte DATA frame (a 16-byte chunk)."""


def emit_after_acks(fc: FlightComputer, monkeypatch: pytest.MonkeyPatch) -> list[bool]:
    """Queue one telemetry frame and one DATA frame in the emit-telemetry phase,
    through the outbound seam. Returns whether each was queued."""
    queued: list[bool] = []

    def emit_telemetry(tick: TickContext) -> None:
        queued.append(
            fc._queue_outbound(
                tick, FrameType.TELEMETRY, TELEMETRY_PAYLOAD, OutboundClass.TELEMETRY
            )
        )
        queued.append(fc._queue_outbound(tick, FrameType.DATA, DATA_PAYLOAD, OutboundClass.DATA))

    monkeypatch.setattr(fc, "_emit_telemetry", emit_telemetry)
    return queued


@pytest.mark.parametrize(
    ("capacity", "acks", "telemetry", "data", "suppressed"),
    [
        (120, 2, True, True, 0),  # 28 + 36 + 30 = 94 fit
        (64, 2, True, False, 0),  # ACKs and telemetry fit; DATA waits, not counted
        (63, 2, False, True, 1),  # telemetry doesn't fit; the smaller DATA frame does
        (28, 2, False, False, 1),  # only the ACKs
        (14, 1, False, False, 2),  # one ACK; the other ACK and telemetry are counted
        (0, 0, False, False, 3),  # transmitter off: everything suppressed but DATA
    ],
)
def test_acks_go_ahead_of_telemetry_and_data_when_capacity_is_tight(
    capacity: int,
    acks: int,
    telemetry: bool,
    data: bool,
    suppressed: int,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fc = in_state(ModeState(Mode.NOMINAL))
    queued = emit_after_acks(fc, monkeypatch)
    out = fc.step([frame_for(Command.ping())] * 2, with_capacity(capacity), 0)

    assert queued == [telemetry, data]
    kinds = [decode_frame(frame) for frame in out.downlink_frames]
    expected_sizes = (
        [ACK_FRAME_SIZE] * acks
        + ([messages.TELEMETRY_FRAME_SIZE] if telemetry else [])
        + ([10 + len(DATA_PAYLOAD)] if data else [])
    )
    assert [len(frame) for frame in out.downlink_frames] == expected_sizes
    assert all(k.frame_type is FrameType.ACK for k in kinds[:acks])
    assert [k.frame_type for k in kinds].count(FrameType.DATA) == data
    assert out.outbound_suppressed_count == suppressed
    assert out.sent_bytes == sum(expected_sizes) <= capacity
    assert [k.sequence for k in kinds] == list(range(len(kinds)))


def test_offering_a_higher_priority_frame_after_a_lower_one_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fc = FlightComputer()

    def emit_telemetry(tick: TickContext) -> None:
        fc._queue_outbound(tick, FrameType.TELEMETRY, TELEMETRY_PAYLOAD, OutboundClass.TELEMETRY)
        fc._queue_outbound(tick, FrameType.ACK, bytes(4), OutboundClass.ACK)

    monkeypatch.setattr(fc, "_emit_telemetry", emit_telemetry)
    with pytest.raises(RuntimeError, match="OutboundClass order"):
        fc.step([], READINGS, 0)


def test_only_ack_and_telemetry_count_as_suppressed() -> None:
    assert OutboundClass.ACK.counts_suppression
    assert OutboundClass.TELEMETRY.counts_suppression
    assert not OutboundClass.DATA.counts_suppression
    assert sorted(OutboundClass) == [OutboundClass.ACK, OutboundClass.TELEMETRY, OutboundClass.DATA]


def test_reserve_outbound_tracks_the_remaining_capacity() -> None:
    tick = TickContext(now_us=0, readings=with_capacity(30), uplink_frames=())
    assert tick.outbound_remaining_bytes == 30
    assert tick.reserve_outbound(14, OutboundClass.ACK)
    assert tick.reserve_outbound(14, OutboundClass.ACK)
    assert not tick.reserve_outbound(14, OutboundClass.ACK)
    assert tick.outbound_remaining_bytes == 2
    assert tick.outbound_suppressed_count == 1
    assert not tick.reserve_outbound(3, OutboundClass.DATA)
    assert tick.outbound_suppressed_count == 1
    assert tick.reserve_outbound(2, OutboundClass.DATA)
    assert tick.outbound_remaining_bytes == 0


# --- Determinism ------------------------------------------------------------------------------


def _random_session(seed: int) -> list[tuple[tuple[bytes, ...], int, Mode]]:
    rng = random.Random(seed)
    fc = in_state(rng.choice(STATES))
    history: list[tuple[tuple[bytes, ...], int, Mode]] = []
    for n in range(100):
        frames: list[bytes] = []
        for _ in range(rng.randrange(0, 5)):
            if rng.random() < 0.3:
                frames.append(
                    command_frame(rng.randbytes(rng.randrange(0, 4)), rng.randrange(1 << 16))
                )
            else:
                frames.append(frame_for(rng.choice(VALID_COMMANDS), rng.randrange(1 << 16)))
        readings = with_capacity(rng.choice([0, 14, 50, CAPACITY]))
        out = fc.step(frames, readings, n * TICK_US)
        history.append((out.downlink_frames, out.outbound_suppressed_count, fc.mode))
    return history


def test_dispatch_is_deterministic() -> None:
    assert _random_session(3) == _random_session(3)
    assert _random_session(3) != _random_session(4)
