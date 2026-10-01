"""Portable arithmetic approximations (ADR-0006).

Simulation code may not call the platform maths library (``math.cos``, ``math.exp``,
and so on), because results can differ in the last bit between platforms. Where a
trigonometric function is genuinely needed, this module provides a documented
approximation built only from ``+ - * /``, which IEEE 754 makes identical everywhere.
"""

import math
from typing import Final

PI: Final = 3.141592653589793
"""π as a literal constant, so no library call is needed."""

PI_SQUARED: Final = PI * PI

_RAD_PER_DEG: Final = PI / 180.0


def portable_cos_deg(angle_deg: float) -> float:
    """Return a clamped cosine of ``angle_deg`` using Bhaskara's approximation.

    For |x| ≤ 90°, with x in radians: cos x ≈ (π² - 4x²) / (π² + x²). The maximum
    absolute error against the true cosine over ±90° is about 0.0016. Beyond ±90° the
    result is clamped to 0, which is what a projected-area factor (for example solar
    generation versus pointing error) needs; this is not a general-purpose cosine.

    Args:
        angle_deg: Angle in degrees. Any finite value.

    Returns:
        A value in [0, 1]: 1 at 0°, 0 at and beyond ±90°.

    Raises:
        ValueError: ``angle_deg`` is not finite.
    """
    if not math.isfinite(angle_deg):
        raise ValueError(f"angle_deg must be finite, got {angle_deg}")
    magnitude = abs(angle_deg)
    if magnitude >= 90.0:
        return 0.0
    x = magnitude * _RAD_PER_DEG
    x_squared = x * x
    return (PI_SQUARED - 4.0 * x_squared) / (PI_SQUARED + x_squared)
