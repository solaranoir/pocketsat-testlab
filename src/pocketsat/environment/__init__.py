"""Environment models: an EnvironmentState for each simulated tick, and the pass
geometry of the spacecraft seen from the ground station (ADR-0010 §6, ADR-0011)."""

from pocketsat.environment.nominal import (
    DEFAULT_ECLIPSE_AMBIENT_TEMP_C,
    DEFAULT_ECLIPSE_FRACTION,
    DEFAULT_ORBIT_PERIOD_US,
    EnvironmentModel,
    NominalEnvironment,
)
from pocketsat.environment.orbit import (
    DEFAULT_ORBIT,
    DEFAULT_STATION_LOCATION,
    GeometryModel,
    Orbit,
    OrbitGeometry,
    PredictedPass,
    StationLocation,
    orbit_radius_km,
)

__all__ = [
    "DEFAULT_ECLIPSE_AMBIENT_TEMP_C",
    "DEFAULT_ECLIPSE_FRACTION",
    "DEFAULT_ORBIT",
    "DEFAULT_ORBIT_PERIOD_US",
    "DEFAULT_STATION_LOCATION",
    "EnvironmentModel",
    "GeometryModel",
    "NominalEnvironment",
    "Orbit",
    "OrbitGeometry",
    "PredictedPass",
    "StationLocation",
    "orbit_radius_km",
]
