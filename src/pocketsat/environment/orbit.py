"""Pass geometry from a circular orbit over a rotating Earth (ADR-0011, #160).

:class:`OrbitGeometry` is the geometry provider ADR-0010 §6 places on the orchestrator
side, beside :class:`~pocketsat.environment.NominalEnvironment`: a pure function of
simulated time that returns a :class:`~pocketsat.environment.geometry.PassGeometry`
(azimuth, elevation, range, range rate) of the spacecraft as seen from the ground
station. The run loop computes it once per tick and gives it to both the link and the
station.

The model (ADR-0011 §2): a two-body circular orbit (no J2, no drag) over a spherical
Earth turning once per sidereal day, with the station at a fixed latitude, longitude and
altitude. The orbit period is the input and the radius follows from Kepler's third law.
Angles are carried in turns, and the orbital phase and Earth rotation angle are reduced
by integer modulo of the simulated time before any floating-point operation, so the
geometry never loses accuracy over a long run. All arithmetic is portable (ADR-0006):
sines, cosines and arctangents come from :mod:`pocketsat.core.portable`.

Two ways to place the orbit (ADR-0011 §1, decision A3):

- by orbital elements: :class:`Orbit` (period, inclination, node, phase);
- by a pass anchor: :meth:`OrbitGeometry.with_pass` solves the node and phase so the
  station sees a pass with a given maximum elevation at a given time.

Pass prediction (:meth:`OrbitGeometry.next_pass`) steps the forward model and bisects on
integer microseconds; there is no closed-form inverse trigonometry.

Eclipse stays with ``NominalEnvironment`` (ADR-0011 §4, decision D1): the two share the
orbit period (:data:`~pocketsat.environment.DEFAULT_ORBIT_PERIOD_US` by default), and the
run loop (:class:`pocketsat.orchestrator.RunLoop`, #167) checks they are equal.
"""

import math
from dataclasses import dataclass, replace
from typing import Final, Protocol, runtime_checkable

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock, check_us
from pocketsat.core.portable import (
    PI,
    TWO_PI,
    portable_atan2,
    portable_sincos_turns,
)
from pocketsat.environment.geometry import PassGeometry
from pocketsat.environment.nominal import DEFAULT_ORBIT_PERIOD_US

EARTH_MU_KM3_PER_S2: Final = 398_600.4418
"""Earth's gravitational parameter GM, km³/s² (WGS-84)."""

EARTH_RADIUS_KM: Final = 6371.0
"""Radius of the spherical Earth, km (the mean radius)."""

SIDEREAL_DAY_US: Final = 86_164_090_500
"""One rotation of the Earth relative to the stars, µs (86 164.0905 s)."""

MIN_ORBIT_ALTITUDE_KM: Final = 100.0
"""Lowest orbit altitude accepted, km: below this the orbit would be inside the
atmosphere (the Kármán line). Periods shorter than about 86.5 minutes are rejected."""

_US_PER_S: Final = 1_000_000
_DEG_PER_RAD: Final = 180.0 / PI
_EARTH_RATE_RAD_PER_S: Final = TWO_PI * _US_PER_S / SIDEREAL_DAY_US
"""Earth's rotation rate, rad/s."""

_RADIUS_NEWTON_STEPS: Final = 200
"""Fixed Newton iteration count for the cube root in the orbit radius. Far more than
needed for any accepted period, so the result is the converged value."""

_PEAK_SCAN_STEP_US: Final = 30_000_000
"""Coarse step, µs, when scanning for the highest point of a pass in the solver."""

_NODE_SEARCH_DEG: Final = 20.0
"""Half-width of the node offset searched for the highest reachable pass, degrees."""

_NODE_SEARCH_STEPS: Final = 60
"""Fixed ternary-search iterations for the highest reachable pass."""

_NODE_BISECT_STEPS: Final = 40
"""Fixed bisection iterations for the node offset that gives the requested elevation."""

_NODE_OFFSET_MAX_DEG: Final = 90.0
"""Largest node offset tried by the solver, degrees east of the highest pass."""

_NODE_STEP_DEG: Final = 1.0
"""Step, degrees, when moving the node east to bracket the requested elevation."""


@runtime_checkable
class GeometryModel(Protocol):
    """Anything that produces the pass geometry for a given simulated time."""

    def geometry_at(self, now_us: int) -> PassGeometry:
        """Return the pass geometry at simulated time ``now_us``."""
        ...


def _number(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return float(value)


def orbit_radius_km(orbit_period_us: int) -> float:
    """Return the radius of a circular orbit with the given period, km.

    Kepler's third law, a = (μ (P / 2π)²)^(1/3), with the cube root by Newton's method
    and a fixed iteration count (arithmetic only, ADR-0006).

    Raises:
        TypeError: ``orbit_period_us`` is not an int.
        ValueError: ``orbit_period_us`` is not positive.
    """
    check_us("orbit_period_us", orbit_period_us)
    if orbit_period_us == 0:
        raise ValueError("orbit_period_us must be positive")
    seconds_per_radian = orbit_period_us / (TWO_PI * _US_PER_S)
    cube = EARTH_MU_KM3_PER_S2 * seconds_per_radian * seconds_per_radian
    radius = max(cube, 1.0)
    for _ in range(_RADIUS_NEWTON_STEPS):
        radius = (2.0 * radius + cube / (radius * radius)) / 3.0
    return radius


@dataclass(frozen=True)
class Orbit:
    """A circular orbit, by its period and orientation at simulated time 0.

    The defaults are the reference orbit (#160, ADR-0011 question 5): the 92-minute
    period ``NominalEnvironment`` uses (altitude about 380 km) at the inclination of the
    International Space Station, 51.6°, which carries the ground track over every
    latitude from 51.6° S to 51.6° N. The node (120°) and phase (0°) are chosen so the
    default station (:data:`DEFAULT_STATION_LOCATION`) sees a 54° pass in the first orbit
    (AOS about 28 minutes in), which keeps short pass tests short, and eight passes above
    the horizon on the first day (six above the default 10° mask), from 1° to 54°.

    Attributes:
        orbit_period_us: Orbit period, µs. Must give an altitude of at least
            :data:`MIN_ORBIT_ALTITUDE_KM`.
        inclination_deg: Inclination, ``0..180`` (above 90 is retrograde).
        raan_deg: Right ascension of the ascending node at time 0, degrees. The Earth's
            rotation angle is 0 at time 0, so this is also the node's longitude then.
        argument_of_latitude_deg: The spacecraft's angle from the ascending node along
            the orbit at time 0, degrees.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: A field is out of range.
    """

    orbit_period_us: int = DEFAULT_ORBIT_PERIOD_US
    inclination_deg: float = 51.6
    raan_deg: float = 120.0
    argument_of_latitude_deg: float = 0.0

    def __post_init__(self) -> None:
        check_us("orbit_period_us", self.orbit_period_us)
        if self.orbit_period_us == 0:
            raise ValueError("orbit_period_us must be positive")
        inclination = _number("inclination_deg", self.inclination_deg)
        if not 0.0 <= inclination <= 180.0:
            raise ValueError(f"inclination_deg must be in 0..180, got {inclination}")
        _number("raan_deg", self.raan_deg)
        _number("argument_of_latitude_deg", self.argument_of_latitude_deg)
        altitude = orbit_radius_km(self.orbit_period_us) - EARTH_RADIUS_KM
        if altitude < MIN_ORBIT_ALTITUDE_KM:
            raise ValueError(
                f"orbit_period_us {self.orbit_period_us} gives an altitude of"
                f" {altitude:.1f} km, below {MIN_ORBIT_ALTITUDE_KM} km"
            )


DEFAULT_ORBIT: Final = Orbit()
"""The reference orbit: see :class:`Orbit`."""


@dataclass(frozen=True)
class StationLocation:
    """Where the ground station is, on the spherical Earth.

    The defaults equal :class:`~pocketsat.groundstation.StationConfig`'s (a test checks
    they agree). The run loop builds this record from the station's configuration;
    ``pocketsat.environment`` does not import the ground station.

    Attributes:
        latitude_deg: Latitude, ``-90..90``, north positive. Default 40.0.
        longitude_deg: Longitude, ``-180..180``, east positive. Default -105.0.
        altitude_km: Altitude above the sphere, km. Default 1.6.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is out of range.
    """

    latitude_deg: float = 40.0
    longitude_deg: float = -105.0
    altitude_km: float = 1.6

    def __post_init__(self) -> None:
        latitude = _number("latitude_deg", self.latitude_deg)
        if not -90.0 <= latitude <= 90.0:
            raise ValueError(f"latitude_deg must be in -90..90, got {latitude}")
        longitude = _number("longitude_deg", self.longitude_deg)
        if not -180.0 <= longitude <= 180.0:
            raise ValueError(f"longitude_deg must be in -180..180, got {longitude}")
        altitude = _number("altitude_km", self.altitude_km)
        if not -0.5 <= altitude <= 9.0:
            raise ValueError(f"altitude_km must be in -0.5..9.0, got {altitude}")


DEFAULT_STATION_LOCATION: Final = StationLocation()
"""The default station: see :class:`StationLocation`."""


@dataclass(frozen=True)
class PredictedPass:
    """A pass predicted by :meth:`OrbitGeometry.next_pass`.

    Times are integer microseconds, found to within the requested resolution.

    Attributes:
        aos_us: First time the elevation is at or above the threshold.
        los_us: First time after AOS the elevation is below the threshold again.
        max_elevation_us: Time of the highest elevation.
        max_elevation_deg: The highest elevation, degrees.
    """

    aos_us: int
    los_us: int
    max_elevation_us: int
    max_elevation_deg: float


class OrbitGeometry:
    """The pass geometry of a circular orbit, seen from a ground station.

    A pure function of simulated time, with the shape of ``EnvironmentModel``:
    :meth:`geometry_at` and :meth:`sample`. About 5 µs per call (ADR-0011 §7: at most
    10 µs on CI Linux, guidance).

    Args:
        orbit: The orbit (default :data:`DEFAULT_ORBIT`).
        station: The station (default :data:`DEFAULT_STATION_LOCATION`).

    Raises:
        TypeError: ``orbit`` or ``station`` has the wrong type.
    """

    def __init__(
        self,
        orbit: Orbit = DEFAULT_ORBIT,
        station: StationLocation = DEFAULT_STATION_LOCATION,
    ) -> None:
        if not isinstance(orbit, Orbit):
            raise TypeError(f"orbit must be an Orbit, got {orbit!r}")
        if not isinstance(station, StationLocation):
            raise TypeError(f"station must be a StationLocation, got {station!r}")
        self._orbit = orbit
        self._station = station

        self._period_us = orbit.orbit_period_us
        self._radius_km = orbit_radius_km(orbit.orbit_period_us)
        self._mean_motion_rad_per_s = TWO_PI * _US_PER_S / orbit.orbit_period_us
        self._sin_incl, self._cos_incl = portable_sincos_turns(orbit.inclination_deg / 360.0)
        self._raan_turns = orbit.raan_deg / 360.0
        self._phase_turns = orbit.argument_of_latitude_deg / 360.0

        sin_lat, cos_lat = portable_sincos_turns(station.latitude_deg / 360.0)
        sin_lon, cos_lon = portable_sincos_turns(station.longitude_deg / 360.0)
        station_radius_km = EARTH_RADIUS_KM + station.altitude_km
        self._station_xyz = (
            station_radius_km * cos_lat * cos_lon,
            station_radius_km * cos_lat * sin_lon,
            station_radius_km * sin_lat,
        )
        self._east = (-sin_lon, cos_lon, 0.0)
        self._north = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat)
        self._up = (cos_lat * cos_lon, cos_lat * sin_lon, sin_lat)

    @property
    def orbit(self) -> Orbit:
        """The orbit, as configured or as solved by :meth:`with_pass`."""
        return self._orbit

    @property
    def station(self) -> StationLocation:
        """The station location."""
        return self._station

    @property
    def orbit_period_us(self) -> int:
        """Orbit period, µs. The run loop checks it equals ``NominalEnvironment``'s."""
        return self._period_us

    @property
    def orbit_radius_km(self) -> float:
        """Orbit radius from the Earth's centre, km."""
        return self._radius_km

    def geometry_at(self, now_us: int) -> PassGeometry:
        """Return the pass geometry at simulated time ``now_us``.

        Raises:
            TypeError: ``now_us`` is not an int.
            ValueError: ``now_us`` is negative.
        """
        check_us("now_us", now_us)
        return self._geometry(now_us)

    def sample(self, clock: SimClock) -> PassGeometry:
        """Return the pass geometry at the clock's current time."""
        return self.geometry_at(clock.now_us)

    def elevation_at(self, now_us: int) -> float:
        """Return only the elevation at ``now_us``, degrees (as :meth:`geometry_at`)."""
        check_us("now_us", now_us)
        return self._elevation(now_us)

    def next_pass(
        self,
        after_us: int,
        *,
        min_elevation_deg: float = 0.0,
        within_us: int = 2 * 86_400 * _US_PER_S,
        step_us: int = 10 * _US_PER_S,
        resolution_us: int = DEFAULT_TICK_US,
    ) -> PredictedPass | None:
        """Predict the next pass that begins after ``after_us``.

        Steps the model every ``step_us`` from ``after_us`` until the elevation reaches
        ``min_elevation_deg``, then bisects AOS and LOS on integer microseconds to
        within ``resolution_us``, and finds the highest point by ternary search. If the
        spacecraft is already above the threshold at ``after_us``, that pass is skipped
        and the next one is returned. A pass shorter than ``step_us`` above the
        threshold can be missed; the default 10 s step is far shorter than any pass of a
        LEO orbit above a useful mask.

        Args:
            after_us: Time to search from, µs.
            min_elevation_deg: Elevation threshold, degrees, ``-90..90`` (the station's
                mask, or 0 for the geometric horizon).
            within_us: How far ahead to search, µs.
            step_us: Coarse search step, µs.
            resolution_us: Resolution of AOS and LOS, µs (default one tick).

        Returns:
            The pass, or ``None`` if none begins and ends within the search window.

        Raises:
            TypeError: An argument has the wrong type.
            ValueError: An argument is out of range.
        """
        check_us("after_us", after_us)
        threshold = _number("min_elevation_deg", min_elevation_deg)
        if not -90.0 <= threshold <= 90.0:
            raise ValueError(f"min_elevation_deg must be in -90..90, got {threshold}")
        for name, value in (
            ("within_us", within_us),
            ("step_us", step_us),
            ("resolution_us", resolution_us),
        ):
            check_us(name, value)
            if value == 0:
                raise ValueError(f"{name} must be positive")
        end_us = after_us + within_us

        def above(t: int) -> bool:
            return self._elevation(t) >= threshold

        t = after_us
        while above(t):  # already in a pass: skip to its end
            t += step_us
            if t > end_us:
                return None
        while not above(t):
            last_below = t
            t += step_us
            if t > end_us:
                return None
        aos_us = _first_true(above, last_below, t, resolution_us)
        while above(t):
            last_above = t
            t += step_us
            if t > end_us:
                return None
        los_us = _first_true(lambda s: not above(s), last_above, t, resolution_us)
        peak_us = _ternary_peak(self._elevation, aos_us, los_us)
        return PredictedPass(
            aos_us=aos_us,
            los_us=los_us,
            max_elevation_us=peak_us,
            max_elevation_deg=self._elevation(peak_us),
        )

    @classmethod
    def with_pass(
        cls,
        max_elevation_deg: float,
        at_us: int,
        *,
        orbit: Orbit = DEFAULT_ORBIT,
        station: StationLocation = DEFAULT_STATION_LOCATION,
        ascending: bool = True,
    ) -> "OrbitGeometry":
        """Build a geometry whose station sees a pass peaking at ``max_elevation_deg``
        at ``at_us`` (ADR-0011 §1, the pass anchor).

        The orbit keeps ``orbit``'s period and inclination; its node and phase are
        solved. First the spacecraft is put over the station's latitude at ``at_us``
        on the ascending (or descending) half of the orbit, and the node offset that
        gives the highest pass is found (ternary search, fixed iterations). Then the
        node is moved east by bisection (fixed iterations) until the pass's highest
        elevation equals ``max_elevation_deg`` within 0.01°. Finally the whole
        trajectory is shifted in time, exactly, so the highest point is at ``at_us``
        (within one tick). Every step only runs the forward model, so the result is
        deterministic.

        Args:
            max_elevation_deg: The pass's highest elevation, degrees, above 0.
            at_us: Time of the highest elevation, µs.
            orbit: Period and inclination to keep (node and phase are replaced).
            station: The station.
            ascending: Whether the spacecraft is heading north at the pass.

        Returns:
            The geometry; its :attr:`orbit` holds the solved node and phase.

        Raises:
            TypeError: An argument has the wrong type.
            ValueError: ``max_elevation_deg`` is not above 0, or is higher than the
                orbit's inclination and the station's latitude allow; the message gives
                the highest reachable elevation.
        """
        target = _number("max_elevation_deg", max_elevation_deg)
        if not 0.0 < target <= 90.0:
            raise ValueError(f"max_elevation_deg must be in (0, 90], got {target}")
        check_us("at_us", at_us)
        if not isinstance(ascending, bool):
            raise TypeError(f"ascending must be a bool, got {ascending!r}")
        base = cls(orbit, station)
        window_us = orbit.orbit_period_us // 4

        # Argument of latitude at which the ground track crosses the station's latitude
        # (or comes closest to it, if the station is beyond the inclination).
        sin_lat, _ = portable_sincos_turns(station.latitude_deg / 360.0)
        ratio = sin_lat / base._sin_incl if base._sin_incl > 0.0 else 0.0
        ratio = min(1.0, max(-1.0, ratio))
        crossing_turns = portable_atan2(ratio, math.sqrt(1.0 - ratio * ratio)) / TWO_PI
        if not ascending:
            crossing_turns = 0.5 - crossing_turns
        # Longitude of the crossing point east of the node (in the Earth-fixed frame).
        sin_u, cos_u = portable_sincos_turns(crossing_turns)
        along_turns = portable_atan2(base._cos_incl * sin_u, cos_u) / TWO_PI
        earth_turns = (at_us % SIDEREAL_DAY_US) / SIDEREAL_DAY_US
        phase_turns = crossing_turns - (at_us % orbit.orbit_period_us) / orbit.orbit_period_us

        def placed(offset_deg: float) -> "OrbitGeometry":
            raan_turns = (
                station.longitude_deg / 360.0 + offset_deg / 360.0 - along_turns + earth_turns
            )
            solved = replace(
                orbit,
                raan_deg=_wrap_deg(raan_turns * 360.0),
                argument_of_latitude_deg=_wrap_deg(phase_turns * 360.0),
            )
            return cls(solved, station)

        def peak(offset_deg: float) -> tuple[int, float]:
            geometry = placed(offset_deg)
            peak_us = _scan_peak(geometry._elevation, at_us - window_us, at_us + window_us)
            return peak_us, geometry._elevation(peak_us)

        # The node offset of the highest pass (max elevation is unimodal near it).
        low, high = -_NODE_SEARCH_DEG, _NODE_SEARCH_DEG
        for _ in range(_NODE_SEARCH_STEPS):
            third = (high - low) / 3.0
            if peak(low + third)[1] < peak(high - third)[1]:
                low += third
            else:
                high -= third
        best_offset = (low + high) / 2.0
        reachable = peak(best_offset)[1]
        if target > reachable:
            raise ValueError(
                f"max_elevation_deg {target} is not reachable from this station with"
                f" inclination {orbit.inclination_deg}°; the highest is {reachable:.2f}°"
            )
        # Move the node east until the pass is as low as requested: step out a degree at a
        # time to the first offset that is too low (further out, the orbit's other half
        # can come over the station), then bisect between the last two steps.
        near = best_offset
        far = near + _NODE_STEP_DEG
        while peak(far)[1] >= target:
            near = far
            far += _NODE_STEP_DEG
            if far - best_offset > _NODE_OFFSET_MAX_DEG:
                raise ValueError(
                    f"max_elevation_deg {target} could not be bracketed within"
                    f" {_NODE_OFFSET_MAX_DEG}° of node offset"
                )
        for _ in range(_NODE_BISECT_STEPS):
            middle = (near + far) / 2.0
            if peak(middle)[1] >= target:
                near = middle
            else:
                far = middle
        offset = (near + far) / 2.0
        peak_us, _ = peak(offset)
        # Shift the trajectory in time so its highest point is at at_us: the phase moves
        # back along the orbit and the node moves with the Earth, so r'(t) = r(t - shift).
        shift_us = at_us - peak_us
        geometry = placed(offset)
        solved = replace(
            geometry.orbit,
            raan_deg=_wrap_deg(geometry.orbit.raan_deg + shift_us * 360.0 / SIDEREAL_DAY_US),
            argument_of_latitude_deg=_wrap_deg(
                geometry.orbit.argument_of_latitude_deg - shift_us * 360.0 / orbit.orbit_period_us
            ),
        )
        return cls(solved, station)

    def _vectors(
        self, now_us: int
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Return the spacecraft's Earth-fixed position (km) and velocity (km/s)."""
        u_turns = self._phase_turns + (now_us % self._period_us) / self._period_us
        node_turns = self._raan_turns - (now_us % SIDEREAL_DAY_US) / SIDEREAL_DAY_US
        sin_u, cos_u = portable_sincos_turns(u_turns)
        sin_n, cos_n = portable_sincos_turns(node_turns)
        a = self._radius_km
        ci = self._cos_incl
        x = a * (cos_u * cos_n - sin_u * ci * sin_n)
        y = a * (cos_u * sin_n + sin_u * ci * cos_n)
        z = a * sin_u * self._sin_incl
        # d/du of the position, times the mean motion, plus the Earth's rotation
        # (the node angle falls at the Earth rate): v = n ∂r/∂u + ω_E (y, -x, 0).
        n = self._mean_motion_rad_per_s
        w = _EARTH_RATE_RAD_PER_S
        vx = n * a * (-sin_u * cos_n - cos_u * ci * sin_n) + w * y
        vy = n * a * (-sin_u * sin_n + cos_u * ci * cos_n) - w * x
        vz = n * a * cos_u * self._sin_incl
        return (x, y, z), (vx, vy, vz)

    def _topocentric(self, now_us: int) -> tuple[float, float, float, float, float]:
        """Return east, north, up (km), range (km) and range rate (km/s)."""
        (x, y, z), (vx, vy, vz) = self._vectors(now_us)
        sx, sy, sz = self._station_xyz
        dx, dy, dz = x - sx, y - sy, z - sz
        east = dx * self._east[0] + dy * self._east[1]
        north = dx * self._north[0] + dy * self._north[1] + dz * self._north[2]
        up = dx * self._up[0] + dy * self._up[1] + dz * self._up[2]
        range_km = math.sqrt(dx * dx + dy * dy + dz * dz)
        range_rate = (dx * vx + dy * vy + dz * vz) / range_km
        return east, north, up, range_km, range_rate

    def _elevation(self, now_us: int) -> float:
        east, north, up, _, _ = self._topocentric(now_us)
        return portable_atan2(up, math.sqrt(east * east + north * north)) * _DEG_PER_RAD

    def _geometry(self, now_us: int) -> PassGeometry:
        east, north, up, range_km, range_rate = self._topocentric(now_us)
        elevation = portable_atan2(up, math.sqrt(east * east + north * north)) * _DEG_PER_RAD
        azimuth = portable_atan2(east, north) * _DEG_PER_RAD
        if azimuth < 0.0:
            azimuth += 360.0
        if azimuth >= 360.0:  # a tiny negative angle rounds up to exactly 360
            azimuth -= 360.0
        return PassGeometry(
            azimuth_deg=azimuth,
            elevation_deg=elevation,
            range_km=range_km,
            range_rate_km_per_s=range_rate,
        )


def _wrap_deg(angle_deg: float) -> float:
    """Return ``angle_deg`` reduced to [0, 360)."""
    wrapped = angle_deg - 360.0 * math.floor(angle_deg / 360.0)
    return 0.0 if wrapped >= 360.0 else wrapped


def _first_true(predicate: "_Predicate", false_at: int, true_at: int, resolution_us: int) -> int:
    """Bisect integer time between a false and a true point; return the true end."""
    while true_at - false_at > resolution_us:
        middle = (false_at + true_at) // 2
        if predicate(middle):
            true_at = middle
        else:
            false_at = middle
    return true_at


def _ternary_peak(elevation: "_Elevation", start_us: int, end_us: int) -> int:
    """Return the time of the highest elevation in [start_us, end_us] (unimodal)."""
    low, high = start_us, end_us
    while high - low > 2:
        third = (high - low) // 3
        if elevation(low + third) < elevation(high - third):
            low += third
        else:
            high -= third
    return max(range(low, high + 1), key=elevation)


def _scan_peak(elevation: "_Elevation", start_us: int, end_us: int) -> int:
    """Return the time of the highest elevation in [start_us, end_us]: a coarse scan,
    then a ternary search around the best sample."""
    best_us = start_us
    best = elevation(start_us)
    t = start_us + _PEAK_SCAN_STEP_US
    while t <= end_us:
        value = elevation(t)
        if value > best:
            best_us, best = t, value
        t += _PEAK_SCAN_STEP_US
    return _ternary_peak(
        elevation,
        max(start_us, best_us - _PEAK_SCAN_STEP_US),
        min(end_us, best_us + _PEAK_SCAN_STEP_US),
    )


class _Predicate(Protocol):
    def __call__(self, now_us: int, /) -> bool: ...


class _Elevation(Protocol):
    def __call__(self, now_us: int, /) -> float: ...
