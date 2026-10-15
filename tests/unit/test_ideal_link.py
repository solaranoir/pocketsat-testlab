"""Tests for the ideal link between the ground station and a TestTarget (#153, ADR-0010 §5)."""

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.frame import FrameType, encode_frame_fields
from pocketsat.link import IdealLink, Link
from pocketsat.messages import Command, Packet, encode_command
from pocketsat.targets.echo import EchoTarget

TICK_US = 100_000

ABOVE = PassGeometry(
    azimuth_deg=120.0, elevation_deg=35.5, range_km=812.25, range_rate_km_per_s=-3.5
)
HORIZON = PassGeometry(
    azimuth_deg=200.0, elevation_deg=0.0, range_km=2_500.0, range_rate_km_per_s=6.0
)
BELOW = PassGeometry(
    azimuth_deg=300.0, elevation_deg=-0.5, range_km=2_600.0, range_rate_km_per_s=6.5
)


def _command_frame(sequence: int) -> bytes:
    return encode_frame_fields(FrameType.COMMAND, sequence, encode_command(Command.ping()))


def _round_trip(
    link: IdealLink, target: EchoTarget, frames: list[bytes], geometry: PassGeometry | None
) -> list[Packet]:
    """One tick of the run loop's link steps: 4 (send), 6 (advance), 7 and 8 (receive)."""
    for frame in link.carry_uplink([Packet(frame=f) for f in frames], 0, geometry):
        target.send(frame)
    target.advance(TICK_US)
    return link.carry_downlink(target.receive(), TICK_US, geometry)


def test_ideal_link_satisfies_the_link_protocol() -> None:
    link: Link = IdealLink()
    assert isinstance(link, Link)


@pytest.mark.parametrize("geometry", [None, ABOVE, HORIZON], ids=["none", "above", "horizon"])
def test_frames_pass_byte_for_byte_and_in_order_through_echo_target(
    geometry: PassGeometry | None,
) -> None:
    link = IdealLink()
    target = EchoTarget()
    target.reset(seed=1)
    frames = [_command_frame(sequence) for sequence in range(5)]

    packets = _round_trip(link, target, frames, geometry)

    assert [packet.frame for packet in packets] == frames
    assert link.uplink_dropped_count == 0
    assert link.downlink_dropped_count == 0


def test_uplink_unwraps_packets_to_their_frames_unchanged() -> None:
    frames = [_command_frame(1), b"not a frame", b""]
    carried = IdealLink().carry_uplink([Packet(frame=f) for f in frames], 0, ABOVE)
    assert carried == frames


def test_downlink_fills_elevation_range_and_doppler_from_geometry_only() -> None:
    (packet,) = IdealLink().carry_downlink([_command_frame(3)], TICK_US, ABOVE)
    assert packet == Packet(
        frame=_command_frame(3),
        elevation_deg=35.5,
        range_km=812.25,
        doppler_hz=5084.350721057833,  # 435.5 MHz approaching at 3.5 km/s (#162)
    )
    assert packet.snr_db is None
    assert packet.loss_probability is None


def test_downlink_without_geometry_has_no_link_attributes() -> None:
    (packet,) = IdealLink().carry_downlink([_command_frame(3)], TICK_US, None)
    assert packet == Packet(frame=_command_frame(3))


def test_downlink_wraps_a_copy_of_mutable_frame_bytes() -> None:
    frame = bytearray(_command_frame(4))
    (packet,) = IdealLink().carry_downlink([frame], TICK_US, None)
    frame[0] = 0
    assert packet.frame == _command_frame(4)
    assert type(packet.frame) is bytes


def test_below_horizon_drops_and_counts_uplink() -> None:
    link = IdealLink()
    packets = [Packet(frame=_command_frame(sequence)) for sequence in range(3)]

    assert link.carry_uplink(packets, 0, BELOW) == []
    assert link.carry_uplink(packets[:1], 0, BELOW) == []

    assert link.uplink_dropped_count == 4
    assert link.downlink_dropped_count == 0


def test_below_horizon_drops_and_counts_downlink() -> None:
    link = IdealLink()

    assert link.carry_downlink([_command_frame(1), _command_frame(2)], TICK_US, BELOW) == []

    assert link.downlink_dropped_count == 2
    assert link.uplink_dropped_count == 0


def test_echo_round_trip_below_horizon_delivers_nothing_either_way() -> None:
    link = IdealLink()
    target = EchoTarget()
    target.reset(seed=1)

    assert _round_trip(link, target, [_command_frame(1)], BELOW) == []
    # The uplink never reached the target, so it had nothing to echo.
    assert link.uplink_dropped_count == 1
    assert link.downlink_dropped_count == 0


def test_echo_reply_is_dropped_when_the_spacecraft_sets_during_the_tick() -> None:
    link = IdealLink()
    target = EchoTarget()
    target.reset(seed=1)

    for frame in link.carry_uplink([Packet(frame=_command_frame(1))], 0, ABOVE):
        target.send(frame)
    target.advance(TICK_US)

    assert link.carry_downlink(target.receive(), TICK_US, BELOW) == []
    assert link.downlink_dropped_count == 1


def test_reset_clears_the_drop_counts() -> None:
    link = IdealLink()
    link.carry_uplink([Packet(frame=b"x")], 0, BELOW)
    link.carry_downlink([b"y"], 0, BELOW)

    link.reset(RngFactory(42))

    assert link.uplink_dropped_count == 0
    assert link.downlink_dropped_count == 0


def test_empty_input_delivers_and_counts_nothing() -> None:
    link = IdealLink()
    assert link.carry_uplink([], 0, BELOW) == []
    assert link.carry_downlink([], 0, BELOW) == []
    assert link.uplink_dropped_count == 0
    assert link.downlink_dropped_count == 0
