"""Story #147: the station knows when it is in a pass (#161).

Over one day of the default orbit and station (ADR-0011, #160), the station's AOS and LOS
fall within one tick of the geometry crossing the elevation mask, each pass's maximum
elevation matches the prediction, and frames are delivered only during passes.

The day-long runs use a 10 s tick in the fast suite (one day is 8,640 ticks); the slow
test repeats the AOS/LOS and maximum-elevation checks at the default 100 ms tick. The
pass state machine depends on the tick only through sampling, so the coarse tick tests
the same logic with looser, tick-sized bounds.

**Delivery.** The link delivers frames while the spacecraft is above the geometric
horizon (ADR-0010 §5), and the station's mask is its own policy, so "delivered only
during passes" is checked with the mask at 0°, where the two coincide. With a higher
mask, frames between the horizon and the mask still arrive; radio control (#164) stops
the station transmitting there.
"""

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.orbit import OrbitGeometry, PredictedPass
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    Aos,
    GroundStation,
    Los,
    StationConfig,
    StationEvent,
)
from pocketsat.link import IdealLink
from pocketsat.messages import Command
from pocketsat.targets.echo import EchoTarget

DAY_US = 86_400 * 1_000_000
COARSE_TICK_US = 10_000_000
DEFAULT_TICK_US = 100_000
PREDICTION_RESOLUTION_US = 1_000
"""AOS/LOS are predicted to 1 ms, far finer than either tick."""

MASK_DEG = DEFAULT_STATION_CONFIG.elevation_mask_deg


def _predicted_passes(geometry: OrbitGeometry, mask_deg: float) -> list[PredictedPass]:
    """Every pass with AOS and LOS inside the day."""
    passes: list[PredictedPass] = []
    after_us = 0
    while True:
        predicted = geometry.next_pass(
            after_us,
            min_elevation_deg=mask_deg,
            within_us=DAY_US - after_us,
            resolution_us=PREDICTION_RESOLUTION_US,
        )
        if predicted is None:
            return passes
        passes.append(predicted)
        after_us = predicted.los_us


def _station_events(tick_us: int) -> list[StationEvent]:
    """One day of ``begin_tick`` with the default orbit, station and mask."""
    geometry = OrbitGeometry()
    station = GroundStation()
    station.reset(RngFactory(1))
    events: list[StationEvent] = []
    for tick in range(DAY_US // tick_us):
        now_us = tick * tick_us
        events.extend(station.begin_tick(now_us, geometry.geometry_at(now_us)).events)
    return events


def _check_against_prediction(tick_us: int, max_elevation_tolerance_deg: float) -> None:
    events = _station_events(tick_us)
    predicted = _predicted_passes(OrbitGeometry(), MASK_DEG)
    aos = [e for e in events if isinstance(e, Aos)]
    los = [e for e in events if isinstance(e, Los)]

    assert len(predicted) >= 4, "the default orbit should give several passes a day"
    assert len(aos) == len(los) == len(predicted)
    for number, (expected, rise, fall) in enumerate(zip(predicted, aos, los, strict=True), 1):
        assert rise.pass_id == fall.pass_id == number
        # The station samples at tick starts, so it reports the first tick at or past
        # the crossing: never before it (less the prediction's resolution), and less than
        # one tick after.
        assert -PREDICTION_RESOLUTION_US <= rise.time_us - expected.aos_us <= tick_us
        assert -PREDICTION_RESOLUTION_US <= fall.time_us - expected.los_us <= tick_us
        assert rise.elevation_deg >= MASK_DEG > fall.elevation_deg
        # The highest sample is at most the true peak, and within the tolerance of it.
        shortfall_deg = expected.max_elevation_deg - fall.max_elevation_deg
        assert 0.0 <= shortfall_deg <= max_elevation_tolerance_deg


def test_aos_los_and_maximum_elevation_over_a_day_at_a_coarse_tick() -> None:
    # Measured worst case 0.031° at 10 s; the bound leaves margin.
    _check_against_prediction(COARSE_TICK_US, max_elevation_tolerance_deg=0.1)


@pytest.mark.slow
def test_aos_los_and_maximum_elevation_over_a_day_at_the_default_tick() -> None:
    # 864,000 ticks, about 9 s. Measured worst case 2.2e-5° at 100 ms.
    _check_against_prediction(DEFAULT_TICK_US, max_elevation_tolerance_deg=0.001)


def test_frames_are_delivered_only_during_passes() -> None:
    geometry = OrbitGeometry()
    station = GroundStation(StationConfig(elevation_mask_deg=0.0))
    link = IdealLink()
    target = EchoTarget()
    rngs = RngFactory(1)
    station.reset(rngs)
    link.reset(rngs)
    target.connect()
    target.reset(1)

    delivered_ticks = 0
    sent_outside_passes = 0
    for tick in range(DAY_US // COARSE_TICK_US):
        start_us, end_us = tick * COARSE_TICK_US, (tick + 1) * COARSE_TICK_US
        sample = geometry.geometry_at(start_us)
        station.submit(Command.ping())
        uplink = station.begin_tick(start_us, sample)
        for frame in link.carry_uplink(uplink.packets, start_us, sample):
            target.send(frame)
        target.advance(COARSE_TICK_US)
        downlink = link.carry_downlink(target.receive(), end_us, sample)
        station.end_tick(end_us, downlink)
        # A PING goes up every tick and EchoTarget returns it, so the station hears
        # something exactly in the ticks the link carries frames.
        assert bool(downlink) == station.pass_state.in_pass, f"tick {tick}"
        delivered_ticks += bool(downlink)
        if not downlink:
            sent_outside_passes += len(uplink.packets)
    target.close()

    assert station.pass_state.pass_id >= 4
    assert delivered_ticks > 0
    # EchoTarget never ACKs, so unanswered PINGs are retried (#155) and a tick can send
    # more than one frame: every frame sent outside a pass, and only those, is dropped.
    assert sent_outside_passes > 0
    assert link.uplink_dropped_count == sent_outside_passes
