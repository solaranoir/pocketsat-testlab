"""The SNR indicator through a pass (#163).

The ideal link fills ``Packet.snr_db`` from the link budget at the tick's range
(ADR-0010 §5, ADR-0011 §6), and the station carries it onto its telemetry and DATA
events. Over a pass the SNR rises with elevation to the peak and falls after it, falls
strictly with range, stays within the documented accuracy of the ``math`` reference, and
never drops a packet: Phase 3 maps SNR to loss.

Geometry is sampled at the start of each tick and given to both the station and the
link, as the run loop (#167) does.
"""

import itertools
import math

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.environment.orbit import OrbitGeometry, PredictedPass
from pocketsat.groundstation import GroundStation, StationEvent, TelemetryReceived
from pocketsat.link import IdealLink
from pocketsat.link_budget import DEFAULT_LINK_BUDGET
from pocketsat.targets.sil import SilTarget

DAY_US = 86_400 * 1_000_000
SAMPLE_US = 1_000_000
TICK_US = 100_000
SEED = 7
FRAME = b"\x00"
"""Any bytes: the ideal link wraps frames without reading them."""


def _first_pass(geometry: OrbitGeometry, within_us: int = DAY_US) -> PredictedPass:
    predicted = geometry.next_pass(
        0, min_elevation_deg=0.0, within_us=within_us, resolution_us=1_000
    )
    assert predicted is not None
    return predicted


def _reference_snr_db(range_km: float) -> float:
    """The default budget written with ``math.log10``."""
    fspl = 20.0 * math.log10(4.0 * math.pi * range_km * 1000.0 * 435_500_000 / 299_792_458.0)
    noise = -174.0 + 10.0 * math.log10(15_000) + 3.0
    return 30.0 + 0.0 + 12.0 - fspl - 8.0 - noise


def _link_samples() -> tuple[PredictedPass, list[tuple[int, PassGeometry, float]]]:
    """The default orbit's first pass, one frame through the ideal link each second."""
    geometry = OrbitGeometry()
    predicted = _first_pass(geometry)
    link = IdealLink()
    samples: list[tuple[int, PassGeometry, float]] = []
    first_us = (predicted.aos_us // SAMPLE_US + 1) * SAMPLE_US
    for now_us in range(first_us, predicted.los_us, SAMPLE_US):
        at = geometry.geometry_at(now_us)
        (packet,) = link.carry_downlink([FRAME], now_us, at)
        assert packet.snr_db is not None
        samples.append((now_us, at, packet.snr_db))
    assert link.downlink_dropped_count == 0
    return predicted, samples


def test_snr_rises_with_elevation_and_falls_after_the_peak() -> None:
    predicted, samples = _link_samples()
    rising = [snr for now_us, _, snr in samples if now_us <= predicted.max_elevation_us]
    setting = [snr for now_us, _, snr in samples if now_us >= predicted.max_elevation_us]
    assert len(rising) > 60 and len(setting) > 60
    assert all(a < b for a, b in itertools.pairwise(rising))
    assert all(a > b for a, b in itertools.pairwise(setting))


def test_snr_falls_strictly_with_range_and_rises_with_elevation() -> None:
    _, samples = _link_samples()
    by_range = sorted(samples, key=lambda sample: sample[1].range_km)
    snrs = [snr for _, _, snr in by_range]
    assert all(a > b for a, b in itertools.pairwise(snrs))
    # Over a circular orbit range falls as elevation rises, so the order is the same.
    by_elevation = sorted(samples, key=lambda sample: sample[1].elevation_deg)
    elevations_snr = [snr for _, _, snr in by_elevation]
    assert all(a < b for a, b in itertools.pairwise(elevations_snr))


def test_snr_over_the_default_pass_is_the_documented_range_and_matches_the_reference() -> None:
    predicted, samples = _link_samples()
    snrs = [snr for _, _, snr in samples]
    # docs/groundstation.md: about 11 dB at the horizon to about 25 dB at this pass's
    # 54° peak (26.4 dB is the zenith of the default orbit).
    assert 11.0 < min(snrs) < 11.5
    assert 24.5 < max(snrs) < 25.0
    assert predicted.max_elevation_deg == pytest.approx(54.39, abs=0.01)
    worst = max(abs(snr - _reference_snr_db(at.range_km)) for _, at, snr in samples)
    assert worst < 1e-10


def test_snr_over_the_default_pass_is_pinned() -> None:
    # Exact values: byte-identical on Linux and macOS (ADR-0006, ADR-0011 §6).
    predicted, samples = _link_samples()
    peak_us = (predicted.max_elevation_us // SAMPLE_US) * SAMPLE_US
    snr_at = {now_us: snr for now_us, _, snr in samples}
    first_us, _, first_snr = samples[0]
    last_us, _, last_snr = samples[-1]
    assert (first_us, first_snr) == (1_655_000_000, 11.069425709195997)
    assert (peak_us, snr_at[peak_us]) == (1_959_000_000, 24.782281055331765)
    assert (last_us, last_snr) == (2_264_000_000, 11.065173170093331)


# --- Through the station, against SilTarget ---------------------------------------------


def _station_run() -> tuple[list[StationEvent], list[tuple[int, float]], IdealLink]:
    """A 60° pass peaking at 400 s, with SilTarget sending telemetry every second."""
    geometry = OrbitGeometry.with_pass(60.0, 400_000_000)
    predicted = _first_pass(geometry, within_us=900_000_000)
    station, link, target = GroundStation(), IdealLink(), SilTarget()
    rngs = RngFactory(SEED)
    station.reset(rngs)
    link.reset(rngs)
    target.connect()
    target.reset(SEED)
    events: list[StationEvent] = []
    ranges: list[tuple[int, float]] = []
    for tick in range(predicted.los_us // TICK_US + 10):
        start_us, end_us = tick * TICK_US, (tick + 1) * TICK_US
        at = geometry.geometry_at(start_us)
        uplink = station.begin_tick(start_us, at)
        for frame in link.carry_uplink(uplink.packets, start_us, at):
            target.send(frame)
        target.advance(TICK_US)
        events.extend(station.end_tick(end_us, link.carry_downlink(target.receive(), end_us, at)))
        ranges.append((end_us, at.range_km))
    target.close()
    return events, ranges, link


def test_station_telemetry_events_carry_the_snr_of_their_tick() -> None:
    events, ranges, link = _station_run()
    range_at = dict(ranges)
    telemetry = [e for e in events if isinstance(e, TelemetryReceived)]
    # One frame a second over the whole pass, the weakest part included: nothing above
    # the horizon is dropped for its SNR.
    assert len(telemetry) > 600
    assert link.downlink_dropped_count > 0  # frames sent before AOS, below the horizon
    for event in telemetry:
        assert event.snr_db == DEFAULT_LINK_BUDGET.snr_db(range_at[event.time_us])
    snrs = [event.snr_db for event in telemetry if event.snr_db is not None]
    assert len(snrs) == len(telemetry)
    peak = snrs.index(max(snrs))
    assert all(a < b for a, b in itertools.pairwise(snrs[: peak + 1]))
    assert all(a > b for a, b in itertools.pairwise(snrs[peak:]))
    assert 11.0 < snrs[0] < 11.5 and 11.0 < snrs[-1] < 11.5
    assert 25.0 < max(snrs) < 25.5
