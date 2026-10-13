"""Tests for the pass geometry provider (#160, ADR-0011).

Reference values come from an independent computation with ``math`` (tests may use the
maths library; simulation code may not). Pinned values make the geometry byte-identical
across platforms: CI runs them on Linux and macOS.
"""

import math
import time
from dataclasses import replace

import pytest

from pocketsat.core import SimClock
from pocketsat.environment import (
    DEFAULT_ORBIT,
    DEFAULT_ORBIT_PERIOD_US,
    DEFAULT_STATION_LOCATION,
    GeometryModel,
    NominalEnvironment,
    Orbit,
    OrbitGeometry,
    PredictedPass,
    StationLocation,
    orbit_radius_km,
)
from pocketsat.environment.geometry import PassGeometry
from pocketsat.environment.orbit import EARTH_RADIUS_KM, SIDEREAL_DAY_US
from pocketsat.groundstation import DEFAULT_STATION_CONFIG

US_PER_S = 1_000_000
DAY_US = 86_400 * US_PER_S
TICK_US = 100_000
GEOMETRY_GUIDANCE_US = 10.0
"""ADR-0011 §7: at most 10 µs per tick for ``geometry_at`` on CI Linux (guidance)."""


def reference_geometry(
    orbit: Orbit, station: StationLocation, now_us: int
) -> tuple[float, float, float, float]:
    """Azimuth, elevation (degrees), range (km) and range rate (km/s), with ``math``."""
    radius = orbit_radius_km(orbit.orbit_period_us)
    u = (
        math.radians(orbit.argument_of_latitude_deg)
        + 2 * math.pi * (now_us % orbit.orbit_period_us) / orbit.orbit_period_us
    )
    node = math.radians(orbit.raan_deg) - 2 * math.pi * (now_us % SIDEREAL_DAY_US) / (
        SIDEREAL_DAY_US
    )
    inc = math.radians(orbit.inclination_deg)

    def position(u: float, node: float) -> tuple[float, float, float]:
        return (
            radius * (math.cos(u) * math.cos(node) - math.sin(u) * math.cos(inc) * math.sin(node)),
            radius * (math.cos(u) * math.sin(node) + math.sin(u) * math.cos(inc) * math.cos(node)),
            radius * math.sin(u) * math.sin(inc),
        )

    lat = math.radians(station.latitude_deg)
    lon = math.radians(station.longitude_deg)
    rs = EARTH_RADIUS_KM + station.altitude_km
    s = (rs * math.cos(lat) * math.cos(lon), rs * math.cos(lat) * math.sin(lon), rs * math.sin(lat))
    p = position(u, node)
    d = [p[i] - s[i] for i in range(3)]
    east = -math.sin(lon) * d[0] + math.cos(lon) * d[1]
    north = (
        -math.sin(lat) * math.cos(lon) * d[0]
        - math.sin(lat) * math.sin(lon) * d[1]
        + math.cos(lat) * d[2]
    )
    up = (
        math.cos(lat) * math.cos(lon) * d[0]
        + math.cos(lat) * math.sin(lon) * d[1]
        + math.sin(lat) * d[2]
    )
    rng = math.sqrt(sum(c * c for c in d))
    # Range rate by a central difference of half a millisecond either side.
    h = 500e-6
    n = 2 * math.pi * US_PER_S / orbit.orbit_period_us
    w = 2 * math.pi * US_PER_S / SIDEREAL_DAY_US
    ahead = position(u + n * h, node - w * h)
    behind = position(u - n * h, node + w * h)
    rate = (math.dist(ahead, s) - math.dist(behind, s)) / (2 * h)
    azimuth = math.degrees(math.atan2(east, north)) % 360.0
    elevation = math.degrees(math.atan2(up, math.hypot(east, north)))
    return azimuth, elevation, rng, rate


# --- Configuration ----------------------------------------------------------------------


def test_default_orbit_radius_and_altitude() -> None:
    assert orbit_radius_km(DEFAULT_ORBIT_PERIOD_US) == 6750.750617574892
    radius = math.cbrt(398_600.4418 * (DEFAULT_ORBIT_PERIOD_US / 1e6 / (2 * math.pi)) ** 2)
    assert orbit_radius_km(DEFAULT_ORBIT_PERIOD_US) == pytest.approx(radius, rel=1e-14)
    assert 370.0 < orbit_radius_km(DEFAULT_ORBIT_PERIOD_US) - EARTH_RADIUS_KM < 390.0


def test_default_orbit_shares_the_environment_period() -> None:
    # ADR-0011 §4, D1: one orbit period for eclipse and geometry.
    assert DEFAULT_ORBIT.orbit_period_us == NominalEnvironment().orbit_period_us
    assert OrbitGeometry().orbit_period_us == DEFAULT_ORBIT_PERIOD_US


def test_default_station_matches_the_ground_station_default() -> None:
    assert DEFAULT_STATION_LOCATION.latitude_deg == DEFAULT_STATION_CONFIG.latitude_deg
    assert DEFAULT_STATION_LOCATION.longitude_deg == DEFAULT_STATION_CONFIG.longitude_deg
    assert DEFAULT_STATION_LOCATION.altitude_km == DEFAULT_STATION_CONFIG.altitude_km


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"orbit_period_us": 0}, ValueError),
        ({"orbit_period_us": 80 * 60 * US_PER_S}, ValueError),  # below 100 km altitude
        ({"orbit_period_us": 5.5e9}, TypeError),
        ({"inclination_deg": -1.0}, ValueError),
        ({"inclination_deg": 180.5}, ValueError),
        ({"raan_deg": float("nan")}, ValueError),
        ({"argument_of_latitude_deg": "0"}, TypeError),
    ],
)
def test_orbit_validation(changes: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        replace(DEFAULT_ORBIT, **changes)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"latitude_deg": 90.5}, ValueError),
        ({"longitude_deg": -180.5}, ValueError),
        ({"altitude_km": 10.0}, ValueError),
        ({"altitude_km": True}, TypeError),
    ],
)
def test_station_validation(changes: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        replace(DEFAULT_STATION_LOCATION, **changes)  # type: ignore[arg-type]


def test_constructor_rejects_wrong_types() -> None:
    with pytest.raises(TypeError):
        OrbitGeometry(orbit=DEFAULT_STATION_LOCATION)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        OrbitGeometry(station=DEFAULT_ORBIT)  # type: ignore[arg-type]


# --- Geometry ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("now_us", "expected"),
    [
        (
            0,
            PassGeometry(
                302.73240720961235, -60.498762306903735, 11523.377114534154, -3.4229383148594343
            ),
        ),
        (
            1_680_000_000,
            PassGeometry(
                302.7994752376225, 1.6594691406693873, 2050.7759261619767, -6.916485619992491
            ),
        ),
        (
            DAY_US,
            PassGeometry(
                124.71204639743354, -54.54438331493191, 10839.616130411001, 4.03713092830089
            ),
        ),
        (
            10**12,
            PassGeometry(
                70.89739349153402, -14.345281829409496, 4309.3904031614, 4.219710272169141
            ),
        ),
    ],
)
def test_geometry_pinned_values(now_us: int, expected: PassGeometry) -> None:
    assert OrbitGeometry().geometry_at(now_us) == expected


@pytest.mark.parametrize(
    "orbit",
    [
        DEFAULT_ORBIT,
        Orbit(inclination_deg=97.6, raan_deg=10.0, argument_of_latitude_deg=200.0),
        Orbit(orbit_period_us=100 * 60 * US_PER_S, inclination_deg=0.0),
    ],
)
def test_geometry_matches_a_math_reference(orbit: Orbit) -> None:
    geometry = OrbitGeometry(orbit)
    for step in range(0, 2 * DAY_US, 397 * US_PER_S):
        got = geometry.geometry_at(step)
        azimuth, elevation, rng, rate = reference_geometry(orbit, DEFAULT_STATION_LOCATION, step)
        assert got.elevation_deg == pytest.approx(elevation, abs=1e-9)
        assert got.range_km == pytest.approx(rng, abs=1e-8)
        assert got.range_rate_km_per_s == pytest.approx(rate, abs=1e-6)
        azimuth_error = abs(got.azimuth_deg - azimuth)
        assert min(azimuth_error, 360.0 - azimuth_error) < 1e-9
        assert 0.0 <= got.azimuth_deg < 360.0


def test_geometry_does_not_degrade_over_a_long_run() -> None:
    # The phases are reduced by integer modulo, so a time a year in is as exact as one
    # an orbit in.
    year_us = 365 * DAY_US + 12_345_678
    got = OrbitGeometry().geometry_at(year_us)
    _, elevation, rng, _ = reference_geometry(DEFAULT_ORBIT, DEFAULT_STATION_LOCATION, year_us)
    assert got.elevation_deg == pytest.approx(elevation, abs=1e-9)
    assert got.range_km == pytest.approx(rng, abs=1e-8)


def test_elevation_at_equals_geometry_elevation() -> None:
    geometry = OrbitGeometry()
    for now_us in range(0, DAY_US, 3_331 * US_PER_S):
        assert geometry.elevation_at(now_us) == geometry.geometry_at(now_us).elevation_deg


def test_sample_reads_the_clock_and_rejects_negative_time() -> None:
    geometry = OrbitGeometry()
    clock = SimClock()
    for _ in range(3):
        clock.advance_one_tick()
    assert geometry.sample(clock) == geometry.geometry_at(3 * TICK_US)
    with pytest.raises(ValueError):
        geometry.geometry_at(-1)
    with pytest.raises(TypeError):
        geometry.geometry_at(1.0)  # type: ignore[arg-type]


def test_is_a_geometry_model() -> None:
    model: GeometryModel = OrbitGeometry()
    assert isinstance(model, GeometryModel)


def test_geometry_cost_per_tick_is_printed() -> None:
    # Guidance only (ADR-0011 §7), never a failure: printed beside the #78 figures.
    geometry = OrbitGeometry()
    ticks = 20_000
    start = time.perf_counter()
    for tick in range(ticks):
        geometry.geometry_at(tick * TICK_US)
    per_tick_us = (time.perf_counter() - start) / ticks * 1e6
    note = " (over guidance)" if per_tick_us > GEOMETRY_GUIDANCE_US else ""
    print(f"\n[perf #160] geometry_at: {per_tick_us:.1f} µs per tick{note}")


# --- Pass prediction --------------------------------------------------------------------


def passes(geometry: OrbitGeometry, end_us: int, mask_deg: float) -> list[PredictedPass]:
    found: list[PredictedPass] = []
    t = 0
    while True:
        predicted = geometry.next_pass(t, min_elevation_deg=mask_deg, within_us=end_us)
        if predicted is None or predicted.aos_us >= end_us:
            return found
        found.append(predicted)
        t = predicted.los_us


def test_first_default_pass_is_pinned() -> None:
    assert OrbitGeometry().next_pass(0) == PredictedPass(
        aos_us=1_654_453_125,
        los_us=2_264_453_125,
        max_elevation_us=1_959_445_282,
        max_elevation_deg=54.39221907247257,
    )


def test_default_station_sees_several_passes_a_day_including_a_high_one() -> None:
    mask = DEFAULT_STATION_CONFIG.elevation_mask_deg
    for day in range(2):
        start_us = day * DAY_US
        geometry = OrbitGeometry()
        day_passes = [p for p in passes(geometry, start_us + DAY_US, mask) if p.aos_us >= start_us]
        assert len(day_passes) >= 4
        assert max(p.max_elevation_deg for p in day_passes) >= 45.0


def test_pass_boundaries_are_within_one_tick_of_the_mask_crossing() -> None:
    geometry = OrbitGeometry()
    mask = 10.0
    for predicted in passes(geometry, DAY_US, mask):
        assert geometry.elevation_at(predicted.aos_us) >= mask
        assert geometry.elevation_at(predicted.aos_us - TICK_US) < mask
        assert geometry.elevation_at(predicted.los_us) < mask
        assert geometry.elevation_at(predicted.los_us - TICK_US) >= mask
        assert predicted.aos_us < predicted.max_elevation_us < predicted.los_us
        # The reported maximum is the highest point: no sample is above it.
        samples = range(predicted.aos_us, predicted.los_us, TICK_US)
        assert max(geometry.elevation_at(t) for t in samples) <= predicted.max_elevation_deg


def test_next_pass_skips_a_pass_in_progress() -> None:
    geometry = OrbitGeometry()
    first = geometry.next_pass(0)
    assert first is not None
    during = geometry.next_pass(first.max_elevation_us)
    assert during is not None
    assert during.aos_us > first.los_us


def test_next_pass_returns_none_when_no_pass_fits_the_window() -> None:
    assert OrbitGeometry().next_pass(0, within_us=60 * US_PER_S) is None


def test_next_pass_validates_its_arguments() -> None:
    geometry = OrbitGeometry()
    with pytest.raises(ValueError):
        geometry.next_pass(0, min_elevation_deg=91.0)
    with pytest.raises(ValueError):
        geometry.next_pass(0, step_us=0)
    with pytest.raises(ValueError):
        geometry.next_pass(-1)


# --- The pass anchor (ADR-0011 §1, A3) --------------------------------------------------


def argument_of_latitude_deg(orbit: Orbit, now_us: int) -> float:
    phase = (now_us % orbit.orbit_period_us) / orbit.orbit_period_us * 360.0
    return (orbit.argument_of_latitude_deg + phase) % 360.0


@pytest.mark.parametrize(
    ("max_elevation_deg", "at_us", "ascending"),
    [
        (12.0, 300 * US_PER_S, True),
        (45.0, 3_600 * US_PER_S, False),
        (5.0, 10_000 * US_PER_S, True),
        (89.9, 7_000 * US_PER_S, False),
    ],
)
def test_with_pass_places_the_requested_pass(
    max_elevation_deg: float, at_us: int, ascending: bool
) -> None:
    geometry = OrbitGeometry.with_pass(max_elevation_deg, at_us, ascending=ascending)
    predicted = geometry.next_pass(max(0, at_us - 20 * 60 * US_PER_S))
    assert predicted is not None
    assert predicted.max_elevation_deg == pytest.approx(max_elevation_deg, abs=0.01)
    assert abs(predicted.max_elevation_us - at_us) <= TICK_US
    u = argument_of_latitude_deg(geometry.orbit, at_us)
    heading_north = u < 90.0 or u > 270.0
    assert heading_north is ascending
    assert geometry.orbit.orbit_period_us == DEFAULT_ORBIT.orbit_period_us
    assert geometry.orbit.inclination_deg == DEFAULT_ORBIT.inclination_deg


def test_with_pass_is_deterministic_and_pinned() -> None:
    first = OrbitGeometry.with_pass(12.0, 300 * US_PER_S).orbit
    assert first == OrbitGeometry.with_pass(12.0, 300 * US_PER_S).orbit
    assert first == Orbit(raan_deg=235.1845533710957, argument_of_latitude_deg=24.188589569465428)


def test_with_pass_rejects_an_unreachable_elevation_with_the_reachable_maximum() -> None:
    station = StationLocation(latitude_deg=70.0)  # beyond the 51.6° inclination
    with pytest.raises(ValueError, match=r"the highest is 0\.\d\d°"):
        OrbitGeometry.with_pass(30.0, 0, station=station)


@pytest.mark.parametrize("bad", [0.0, -5.0, 90.5])
def test_with_pass_rejects_elevations_out_of_range(bad: float) -> None:
    with pytest.raises(ValueError):
        OrbitGeometry.with_pass(bad, 0)
