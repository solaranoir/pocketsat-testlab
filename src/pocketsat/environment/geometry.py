"""Station-relative pass geometry: where the spacecraft is as seen from the ground station.

ADR-0010 §6: pass geometry is orchestrator-side world truth, computed beside the
environment model, and delivered as its own message. It is not part of
``EnvironmentState``, which crosses the ``TestTarget`` boundary; the target never sees
it. The run loop gives one record per tick to both the link (physical truth: visibility
and link attributes) and the ground station (its pass prediction). The model that
produces it is ADR-0011's decision (#159, #160); this module defines only the record.
"""

import math
from dataclasses import dataclass


def _finite(name: str, value: float) -> float:
    """Return ``value`` as a float, rejecting bools, non-numbers, NaN, and infinities."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return float(value)


@dataclass(frozen=True)
class PassGeometry:
    """The spacecraft's position relative to the ground station at one simulated time.

    Attributes:
        azimuth_deg: Azimuth from true north, clockwise, in ``[0, 360)``.
        elevation_deg: Elevation above the station's geometric horizon, in
            ``[-90, 90]``. Negative while the spacecraft is below the horizon.
        range_km: Slant range from the station to the spacecraft. Non-negative.
        range_rate_km_per_s: Rate of change of the slant range: positive while the
            spacecraft recedes, negative while it approaches.

    Raises:
        TypeError: A field is not a number.
        ValueError: A field is not finite or is out of range.
    """

    azimuth_deg: float
    elevation_deg: float
    range_km: float
    range_rate_km_per_s: float

    def __post_init__(self) -> None:
        azimuth = _finite("azimuth_deg", self.azimuth_deg)
        if not 0.0 <= azimuth < 360.0:
            raise ValueError(f"azimuth_deg must be in [0, 360), got {azimuth}")
        elevation = _finite("elevation_deg", self.elevation_deg)
        if not -90.0 <= elevation <= 90.0:
            raise ValueError(f"elevation_deg must be in [-90, 90], got {elevation}")
        if _finite("range_km", self.range_km) < 0.0:
            raise ValueError(f"range_km must be non-negative, got {self.range_km}")
        _finite("range_rate_km_per_s", self.range_rate_km_per_s)
