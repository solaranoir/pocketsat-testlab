"""Environment models: produce an EnvironmentState for each simulated tick."""

from pocketsat.environment.nominal import (
    DEFAULT_ECLIPSE_AMBIENT_TEMP_C,
    DEFAULT_ECLIPSE_FRACTION,
    DEFAULT_ORBIT_PERIOD_US,
    EnvironmentModel,
    NominalEnvironment,
)

__all__ = [
    "DEFAULT_ECLIPSE_AMBIENT_TEMP_C",
    "DEFAULT_ECLIPSE_FRACTION",
    "DEFAULT_ORBIT_PERIOD_US",
    "EnvironmentModel",
    "NominalEnvironment",
]
