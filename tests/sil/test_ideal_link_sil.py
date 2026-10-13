"""The ideal link against ``SilTarget`` (#153, ADR-0010 §5).

A PING goes up through the link, the flight computer answers it, and the ACK comes back
down through the link byte for byte, using only the ``TestTarget`` interface.
"""

from pocketsat.environment.geometry import PassGeometry
from pocketsat.frame import FrameType, decode_frame, encode_frame_fields
from pocketsat.link import IdealLink
from pocketsat.messages import Command, CommandAck, CommandId, Packet, decode_ack, encode_command
from pocketsat.targets import base
from pocketsat.targets.sil import SilTarget

TICK_US = 100_000
PING_SEQUENCE = 7

ABOVE = PassGeometry(
    azimuth_deg=90.0, elevation_deg=42.0, range_km=700.0, range_rate_km_per_s=-1.25
)
BELOW = PassGeometry(
    azimuth_deg=10.0, elevation_deg=-3.0, range_km=3_000.0, range_rate_km_per_s=7.0
)


def _ping_frame() -> bytes:
    return encode_frame_fields(FrameType.COMMAND, PING_SEQUENCE, encode_command(Command.ping()))


def _tick(
    link: IdealLink, target: base.TestTarget, uplink: list[Packet], geometry: PassGeometry | None
) -> tuple[list[Packet], list[bytes]]:
    """Run loop steps 4, 6, 7 and 8 for one tick.

    Returns the packets delivered to the station and the raw downlink the target produced.
    """
    for frame in link.carry_uplink(uplink, 0, geometry):
        target.send(frame)
    target.advance(TICK_US)
    raw = target.receive()
    return link.carry_downlink(raw, TICK_US, geometry), raw


def _acks(packets: list[Packet]) -> list[CommandAck]:
    frames = [decode_frame(packet.frame) for packet in packets]
    return [decode_ack(f.payload) for f in frames if f.frame_type is FrameType.ACK]


def test_ping_through_the_link_is_acked_and_downlink_arrives_byte_for_byte() -> None:
    link = IdealLink()
    target = SilTarget()
    target.connect()
    target.reset(seed=42)

    packets, raw = _tick(link, target, [], ABOVE)
    packets_with_ack, raw_with_ack = _tick(link, target, [Packet(frame=_ping_frame())], ABOVE)

    assert [packet.frame for packet in packets] == raw
    assert [packet.frame for packet in packets_with_ack] == raw_with_ack
    assert all(packet.elevation_deg == 42.0 for packet in packets_with_ack)
    assert all(packet.range_km == 700.0 for packet in packets_with_ack)
    assert _acks(packets_with_ack) == [
        CommandAck(sequence=PING_SEQUENCE, command_id=CommandId.PING, reason=None)
    ]
    target.close()


def test_ping_below_the_horizon_never_reaches_sil_target() -> None:
    link = IdealLink()
    target = SilTarget()
    target.connect()
    target.reset(seed=42)

    _tick(link, target, [Packet(frame=_ping_frame())], BELOW)
    # Back above the horizon with nothing new sent: the dropped PING was never received,
    # so no ACK for it ever comes down.
    packets, _ = _tick(link, target, [], ABOVE)

    assert link.uplink_dropped_count == 1
    assert _acks(packets) == []
    target.close()
