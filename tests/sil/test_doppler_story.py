"""Doppler shift and radio tuning through a pass (#162).

One pass of the default orbit, placed by the pass anchor (ADR-0011 §1) to peak at 60° at
t = 600 s, is run at the default 100 ms tick: the station's ``begin_tick`` with the
tick's geometry, and the ideal link's downlink with the same geometry, as the run loop
does (ADR-0010 §6). Checked:

- the Doppler shift is positive while the spacecraft approaches, crosses zero within a
  tick of closest approach and is negative while it recedes, with a magnitude bounded
  by the orbital speed and near it at AOS;
- with Doppler correction on, the station's tuning tracks the shift: the residual is at
  most half a tuning step, against the station's prediction and against the shift the
  link puts on the packets;
- with it off, the radio stays on the nominal frequencies and the residual is the full
  shift.
"""

import dataclasses
import math
from functools import cache
from itertools import pairwise

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.orbit import DEFAULT_ORBIT, OrbitGeometry, orbit_radius_km
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    GroundStation,
    RadioTuned,
    StationConfig,
)
from pocketsat.link import IdealLink
from pocketsat.messages import Packet

TICK_US = 100_000
PEAK_US = 600_000_000
PEAK_ELEVATION_DEG = 60.0
MARGIN_US = 10_000_000
UPLINK_HZ = DEFAULT_STATION_CONFIG.uplink_frequency_hz
DOWNLINK_HZ = DEFAULT_STATION_CONFIG.downlink_frequency_hz
TOLERANCE_HZ = DEFAULT_STATION_CONFIG.tuning_step_hz / 2
"""The documented tracking tolerance: half the tuning step (``docs/groundstation.md``)."""

C_KM_PER_S = 299_792.458
EARTH_ROTATION_KM_PER_S = 0.4651
"""Equatorial speed of the Earth's surface, the most the station's motion can add."""


@dataclasses.dataclass(frozen=True)
class Tick:
    time_us: int
    tuned: RadioTuned | None
    packet: Packet


@cache
def _geometry() -> OrbitGeometry:
    return OrbitGeometry.with_pass(PEAK_ELEVATION_DEG, PEAK_US)


@cache
def _run(config: StationConfig) -> tuple[Tick, ...]:
    """The pass, from 10 s before AOS to 10 s after LOS, one record per tick."""
    geometry = _geometry()
    predicted = geometry.next_pass(0, min_elevation_deg=config.elevation_mask_deg)
    assert predicted is not None
    station = GroundStation(config)
    link = IdealLink(downlink_frequency_hz=config.downlink_frequency_hz)
    rngs = RngFactory(1)
    station.reset(rngs)
    link.reset(rngs)
    ticks: list[Tick] = []
    start_us = (predicted.aos_us - MARGIN_US) // TICK_US * TICK_US
    for now_us in range(start_us, predicted.los_us + MARGIN_US, TICK_US):
        sample = geometry.geometry_at(now_us)
        events = station.begin_tick(now_us, sample).events
        tuned = [event for event in events if isinstance(event, RadioTuned)]
        (packet,) = link.carry_downlink([b"\xa5\x5a"], now_us + TICK_US, sample)
        ticks.append(Tick(now_us, tuned[0] if tuned else None, packet))
    return tuple(ticks)


def _tuned(config: StationConfig = DEFAULT_STATION_CONFIG) -> list[RadioTuned]:
    return [tick.tuned for tick in _run(config) if tick.tuned is not None]


def test_the_station_tunes_on_every_tick_of_the_pass_only() -> None:
    ticks = _run(DEFAULT_STATION_CONFIG)
    in_pass = [tick.tuned is not None for tick in ticks]
    # Not tuned in the margins, tuned on one contiguous run of ticks between them.
    assert not in_pass[0] and not in_pass[-1]
    first, last = in_pass.index(True), len(in_pass) - 1 - in_pass[::-1].index(True)
    assert all(in_pass[first : last + 1])
    # A 60° pass at 380 km lasts several minutes above the 10° mask.
    assert (last - first + 1) * TICK_US > 6 * 60_000_000


def test_doppler_sign_and_magnitude_through_the_pass() -> None:
    tuned = _tuned()
    downlink = [event.downlink_doppler_hz for event in tuned]

    # Positive approaching, negative receding, falling all the way through the pass.
    assert downlink[0] > 0.0 > downlink[-1]
    assert all(later <= earlier for earlier, later in pairwise(downlink))

    # Zero at closest approach: the sign changes within a tick of the peak.
    crossing = next(event.time_us for event in tuned if event.downlink_doppler_hz <= 0.0)
    assert abs(crossing - PEAK_US) <= TICK_US
    nearest = min(tuned, key=lambda event: abs(event.time_us - PEAK_US))
    assert abs(nearest.downlink_doppler_hz) < 20.0  # under one tick of Doppler rate

    # Magnitude: never more than the orbital speed plus the Earth's rotation allow, and
    # most of the orbital speed at AOS, where the spacecraft comes nearly straight in.
    orbital_km_per_s = (
        2
        * math.pi
        * orbit_radius_km(DEFAULT_ORBIT.orbit_period_us)
        / (DEFAULT_ORBIT.orbit_period_us / 1e6)
    )
    bound_hz = DOWNLINK_HZ * (orbital_km_per_s + EARTH_ROTATION_KM_PER_S) / C_KM_PER_S
    assert max(abs(shift) for shift in downlink) < bound_hz
    assert downlink[0] > 0.75 * DOWNLINK_HZ * orbital_km_per_s / C_KM_PER_S
    assert downlink[-1] < -0.75 * DOWNLINK_HZ * orbital_km_per_s / C_KM_PER_S

    # The uplink shift is the same physics on the lower carrier.
    for event in tuned:
        assert event.uplink_doppler_hz == pytest.approx(
            event.downlink_doppler_hz * UPLINK_HZ / DOWNLINK_HZ
        )


def test_corrected_tuning_tracks_within_half_a_step() -> None:
    for tick in _run(DEFAULT_STATION_CONFIG):
        assert tick.packet.doppler_hz is not None
        if tick.tuned is None:
            continue
        tuned = tick.tuned
        assert abs(tuned.uplink_residual_hz) <= TOLERANCE_HZ
        assert abs(tuned.downlink_residual_hz) <= TOLERANCE_HZ
        # Against the link's truth: the received downlink carrier is within the tolerance
        # of the tuned receive frequency.
        received_hz = DOWNLINK_HZ + tick.packet.doppler_hz
        assert abs(received_hz - tuned.downlink_frequency_hz) <= TOLERANCE_HZ
    # The radio moved: tuned frequencies follow the shift, not the nominal values.
    corrections = {event.downlink_frequency_hz - DOWNLINK_HZ for event in _tuned()}
    assert len(corrections) > 1_000


def test_disabled_correction_shows_the_full_shift() -> None:
    config = dataclasses.replace(DEFAULT_STATION_CONFIG, doppler_correction=False)
    ticks = [tick for tick in _run(config) if tick.tuned is not None]
    for tick in ticks:
        assert tick.tuned is not None
        assert tick.tuned.uplink_frequency_hz == UPLINK_HZ
        assert tick.tuned.downlink_frequency_hz == DOWNLINK_HZ
        assert tick.tuned.uplink_residual_hz == tick.tuned.uplink_doppler_hz
        assert tick.tuned.downlink_residual_hz == tick.tuned.downlink_doppler_hz
        assert tick.tuned.downlink_residual_hz == tick.packet.doppler_hz
    # The residual sweeps the whole shift: kilohertz, against 5 Hz with correction on.
    residuals = [tick.tuned.downlink_residual_hz for tick in ticks if tick.tuned is not None]
    assert max(residuals) > 8_000.0
    assert min(residuals) < -8_000.0
