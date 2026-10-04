"""End to end: COMMAND frames through ``SilTarget`` to the real flight computer (#51).

The ground's view only: frames go in with ``send()``, time passes with ``advance()``,
and the ACK frames come back from ``receive()``. Every test here runs within the first
5 s, so the real flight computer is still in BOOT (#49) and only PING and RESET are
accepted; every mode is covered in ``tests/unit/test_command_dispatcher.py``, and
commands after the boot in ``tests/sil/test_boot_reset_story.py``.
"""

from pocketsat.flight import Mode, RejectReason
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    Command,
    CommandAck,
    CommandId,
    DecodeReason,
    decode_ack,
    encode_command,
)
from pocketsat.spacecraft import CommsSnapshot
from pocketsat.targets.sil import SilTarget


def command_frame(command: Command, sequence: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


def acks(frames: list[bytes]) -> list[CommandAck]:
    decoded = [decode_frame(frame) for frame in frames]
    assert all(frame.frame_type is FrameType.ACK for frame in decoded)
    return [decode_ack(frame.payload) for frame in decoded]


def started(seed: int = 0) -> SilTarget:
    target = SilTarget()
    target.connect()
    target.reset(seed)
    return target


def test_ping_sent_through_sil_target_is_acked_in_the_same_tick() -> None:
    target = started()
    target.send(command_frame(Command.ping(), sequence=42))
    target.advance(target.tick_us)

    downlink = target.receive()
    assert acks(downlink) == [CommandAck(42, CommandId.PING)]
    assert decode_frame(downlink[0]).sequence == 0  # first downlink frame since power-on
    tick = target.last_tick
    assert tick is not None
    assert tick.downlink_frames == tuple(downlink)
    assert tick.traffic.sent_bytes == ACK_FRAME_SIZE
    assert tick.traffic.outbound_suppressed_count == 0
    assert target.receive() == []


def test_commands_in_boot_get_the_documented_answers() -> None:
    target = started()
    target.send(command_frame(Command.set_mode(Mode.NOMINAL), 100))
    target.send(command_frame(Command.enter_safe_mode(), 101))
    target.send(command_frame(Command(0x7F), 102))  # unknown command
    target.send(command_frame(Command.reset(), 103))
    target.send(command_frame(Command.ping(), 104))
    target.advance(target.tick_us)
    assert acks(target.receive()) == [
        CommandAck(100, CommandId.SET_MODE, RejectReason.BOOT_IN_PROGRESS),
        CommandAck(101, CommandId.ENTER_SAFE_MODE, RejectReason.BOOT_IN_PROGRESS),
        CommandAck(102, 0x7F, DecodeReason.UNKNOWN_COMMAND),
        CommandAck(103, CommandId.RESET),
        CommandAck(104, CommandId.PING),
    ]


def test_a_frame_that_fails_frame_decoding_gets_no_reply() -> None:
    target = started()
    good = command_frame(Command.ping(), sequence=1)
    target.send(good[:-1] + bytes([good[-1] ^ 0xFF]))  # bad CRC
    target.send(command_frame(Command.ping(), sequence=2))
    target.advance(target.tick_us)
    assert acks(target.receive()) == [CommandAck(2, CommandId.PING)]
    tick = target.last_tick
    assert tick is not None
    assert tick.undecodable_uplink_count == 1


def test_acks_beyond_the_transmit_capacity_are_suppressed_and_counted() -> None:
    target = started()
    for sequence in range(10):
        target.send(command_frame(Command.ping(), sequence))
    target.advance(target.tick_us)
    tick = target.last_tick
    assert tick is not None
    capacity = tick.state.get("comms", CommsSnapshot).readings.transmit_capacity_bytes
    fits = capacity // ACK_FRAME_SIZE
    assert fits < 10
    assert [a.sequence for a in acks(target.receive())] == list(range(fits))
    assert tick.traffic.sent_bytes == fits * ACK_FRAME_SIZE <= capacity
    assert tick.traffic.outbound_suppressed_count == 10 - fits


def test_uplink_in_a_later_tick_is_answered_in_that_tick() -> None:
    target = started()
    target.advance(5 * target.tick_us)
    assert target.receive() == []
    target.send(command_frame(Command.ping(), sequence=7))
    target.advance(target.tick_us)
    downlink = target.receive()
    assert acks(downlink) == [CommandAck(7, CommandId.PING)]
    target.advance(target.tick_us)
    assert target.receive() == []


def test_command_traffic_is_deterministic() -> None:
    def run(seed: int) -> list[bytes]:
        target = started(seed)
        downlink: list[bytes] = []
        for n in range(30):
            if n % 3 == 0:
                target.send(command_frame(Command.ping(), n))
            if n % 7 == 0:
                target.send(command_frame(Command.begin_downlink(), n))
            target.advance(target.tick_us)
            downlink.extend(target.receive())
        return downlink

    assert run(1) == run(1)
    assert run(1) == run(2)  # commands in BOOT don't depend on the sensor noise
