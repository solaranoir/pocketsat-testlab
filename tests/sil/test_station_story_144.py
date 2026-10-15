"""Story #144: the ground station through the ideal link to a ``TestTarget``.

The station, the ideal link (#153) and a target are wired in the run loop's step order
(``docs/architecture.md`` §8), using only the ``TestTarget`` interface: a frame goes up
and comes back from ``EchoTarget``, and ``SilTarget`` answers with downlink the station
decodes (#156). The uplink COMMAND packet is built by hand, outside the station's
uplink (#154), so the PING's ACK answers no command the station sent: correlation (#155)
reports it as an ``UnmatchedAck``, and it is decoded without rejection. The closed loop
through the station's own uplink is story #145 (``test_station_story_145.py``).
"""

from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.frame import FrameType, encode_frame_fields
from pocketsat.groundstation import (
    Aos,
    FrameRejected,
    GroundStation,
    RejectionReason,
    StationEvent,
    TelemetryReceived,
)
from pocketsat.link import IdealLink
from pocketsat.messages import Command, Packet, encode_command
from pocketsat.targets import base
from pocketsat.targets.echo import EchoTarget
from pocketsat.targets.sil import SilTarget

TICK_US = 100_000
SEED = 42
GEOMETRY = PassGeometry(
    azimuth_deg=180.0, elevation_deg=35.0, range_km=900.0, range_rate_km_per_s=-2.0
)


def _ping_packet(sequence: int) -> Packet:
    return Packet(
        frame=encode_frame_fields(FrameType.COMMAND, sequence, encode_command(Command.ping()))
    )


def _run(
    target: base.TestTarget, uplink_by_tick: dict[int, list[Packet]], ticks: int
) -> tuple[GroundStation, list[StationEvent]]:
    """Run ``ticks`` ticks of steps 3 to 9 and return the station and every event."""
    station = GroundStation()
    link = IdealLink()
    rngs = RngFactory(SEED)
    station.reset(rngs)
    link.reset(rngs)
    target.connect()
    target.reset(SEED)
    events: list[StationEvent] = []
    for tick in range(ticks):
        start_us, end_us = tick * TICK_US, (tick + 1) * TICK_US
        uplink = station.begin_tick(start_us, GEOMETRY)  # steps 3 and 4
        events.extend(uplink.events)
        packets = [*uplink.packets, *uplink_by_tick.get(tick, [])]
        for frame in link.carry_uplink(packets, start_us, GEOMETRY):
            target.send(frame)
        target.advance(TICK_US)  # step 6
        downlink = link.carry_downlink(target.receive(), end_us, GEOMETRY)  # steps 7, 8
        events.extend(station.end_tick(end_us, downlink))  # step 9
    target.close()
    return station, events


def test_frame_sent_to_echo_target_comes_back_to_the_station() -> None:
    ping = _ping_packet(sequence=1)
    station, events = _run(EchoTarget(), {0: [ping]}, ticks=2)
    # The geometry is above the mask, so the first tick starts a pass (#161). EchoTarget
    # sends the COMMAND frame back unchanged; the station sees an uplink type on the
    # downlink and rejects it with its bytes.
    assert events == [
        Aos(0, pass_id=1, azimuth_deg=180.0, elevation_deg=35.0),
        FrameRejected(TICK_US, RejectionReason.WRONG_DIRECTION, ping.frame),
    ]
    assert station.link_statistics.received_count == 1


def test_sil_target_answers_with_downlink_the_station_decodes() -> None:
    # Ten seconds: past BOOT (5 s) and several telemetry periods.
    station, events = _run(SilTarget(), {0: [_ping_packet(sequence=0)]}, ticks=100)
    telemetry = [e for e in events if isinstance(e, TelemetryReceived)]
    assert telemetry, "SilTarget sent no telemetry the station decoded"
    assert not any(isinstance(e, FrameRejected) for e in events)
    stats = station.link_statistics
    assert stats.rejected_count == 0
    assert stats.missing_count == 0
    # Every downlink frame decoded: the telemetry plus at least the PING's ACK.
    assert stats.received_count > len(telemetry)
    assert telemetry[0].telemetry.boot_count == telemetry[-1].telemetry.boot_count
