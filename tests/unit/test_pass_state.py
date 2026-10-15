"""Pass state: AOS and LOS against the elevation mask (#161)."""

import dataclasses

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.frame import FrameType, encode_frame_fields
from pocketsat.groundstation import (
    Aos,
    GroundStation,
    Los,
    PassState,
    StationConfig,
    StationEvent,
)
from pocketsat.groundstation.passes import PassTracker
from pocketsat.messages import Command, Packet

MASK_DEG = 10.0
TICK_US = 100_000


def _at(elevation_deg: float, azimuth_deg: float = 90.0) -> PassGeometry:
    return PassGeometry(
        azimuth_deg=azimuth_deg,
        elevation_deg=elevation_deg,
        range_km=1_000.0,
        range_rate_km_per_s=0.0,
    )


def _track(elevations: list[float | None]) -> tuple[PassTracker, list[StationEvent]]:
    tracker = PassTracker(MASK_DEG)
    events: list[StationEvent] = []
    for tick, elevation in enumerate(elevations):
        geometry = None if elevation is None else _at(elevation, azimuth_deg=float(tick))
        events.extend(tracker.update(tick * TICK_US, geometry))
    return tracker, events


def test_aos_and_los_on_the_first_tick_across_the_mask() -> None:
    tracker, events = _track([-5.0, 9.9, 10.5, 30.0, 45.0, 20.0, 9.0, 2.0])
    assert events == [
        Aos(2 * TICK_US, pass_id=1, azimuth_deg=2.0, elevation_deg=10.5),
        Los(6 * TICK_US, pass_id=1, azimuth_deg=6.0, elevation_deg=9.0, max_elevation_deg=45.0),
    ]
    assert tracker.state == PassState(in_pass=False, pass_id=1)


def test_exactly_at_the_mask_is_in_the_pass() -> None:
    _, events = _track([5.0, MASK_DEG, 5.0])
    assert [type(e) for e in events] == [Aos, Los]
    assert isinstance(events[1], Los)
    assert events[1].max_elevation_deg == MASK_DEG


def test_passes_are_numbered_from_one() -> None:
    _, events = _track([0.0, 20.0, 0.0, 0.0, 15.0, 0.0])
    assert [(type(e), e.pass_id) for e in events if isinstance(e, Aos | Los)] == [
        (Aos, 1),
        (Los, 1),
        (Aos, 2),
        (Los, 2),
    ]


def test_state_tracks_the_highest_sample_of_the_current_pass() -> None:
    tracker, _ = _track([12.0, 40.0, 35.0])
    assert tracker.state == PassState(in_pass=True, pass_id=1, max_elevation_deg=40.0)


def test_no_geometry_leaves_the_state_unchanged() -> None:
    tracker, events = _track([None, None])
    assert events == []
    assert tracker.state == PassState()
    tracker, events = _track([20.0, None, None])
    assert [type(e) for e in events] == [Aos]
    assert tracker.state.in_pass


def test_a_grazing_pass_is_reported_without_hysteresis() -> None:
    # One tick above the mask is a pass: no minimum duration (passes.py docstring).
    _, events = _track([9.0, 10.01, 9.99, 10.01, 9.0])
    assert [type(e) for e in events] == [Aos, Los, Aos, Los]


def test_pass_state_is_immutable() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        PassState().in_pass = True  # type: ignore[misc]


# --- In the station ----------------------------------------------------------------


def _decodable_frame(sequence: int) -> Packet:
    # An ACK frame with a bad payload: decoded as a frame, so it is counted as received.
    return Packet(frame=encode_frame_fields(FrameType.ACK, sequence, b""))


def test_station_reports_aos_and_los_in_begin_tick_events() -> None:
    station = GroundStation(StationConfig(elevation_mask_deg=MASK_DEG))
    assert station.begin_tick(0, _at(-1.0)).events == ()
    aos = station.begin_tick(TICK_US, _at(25.0)).events
    assert aos == (Aos(TICK_US, pass_id=1, azimuth_deg=90.0, elevation_deg=25.0),)
    assert station.pass_state == PassState(in_pass=True, pass_id=1, max_elevation_deg=25.0)
    los = station.begin_tick(2 * TICK_US, _at(3.0)).events
    assert los == (
        Los(
            2 * TICK_US,
            pass_id=1,
            azimuth_deg=90.0,
            elevation_deg=3.0,
            max_elevation_deg=25.0,
        ),
    )


def test_pass_events_come_before_uplink_events() -> None:
    station = GroundStation()
    station.submit(Command.ping())
    events = station.begin_tick(0, _at(45.0)).events
    assert type(events[0]) is Aos
    assert len(events) == 2


def test_per_pass_link_statistics_restart_at_aos() -> None:
    station = GroundStation()
    station.begin_tick(0, _at(45.0))
    station.end_tick(TICK_US, [_decodable_frame(0), _decodable_frame(1)])
    station.begin_tick(TICK_US, _at(-1.0))  # LOS: the pass's statistics remain
    assert station.pass_link_statistics.received_count == 2
    station.begin_tick(2 * TICK_US, _at(45.0))  # AOS of pass 2
    assert station.pass_link_statistics.received_count == 0
    assert station.link_statistics.received_count == 2


def test_reset_clears_the_pass_state() -> None:
    station = GroundStation()
    station.begin_tick(0, _at(45.0))
    station.reset(RngFactory(1))
    assert station.pass_state == PassState()
    assert station.begin_tick(0, _at(45.0)).events[0] == Aos(
        0, pass_id=1, azimuth_deg=90.0, elevation_deg=45.0
    )
