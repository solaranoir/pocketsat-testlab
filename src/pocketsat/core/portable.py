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


# --- Full-circle trigonometry and logarithms (ADR-0011 §5, §6) --------------------------
#
# Built only from + - * /, math.sqrt, math.floor and math.frexp (ADR-0006 §1 as amended
# by ADR-0011 §6), so every result is identical on every IEEE 754 platform. Series
# coefficients are 1/n computed by one exactly rounded division at import.

HALF_PI: Final = PI / 2.0
"""π/2: an exact halving of :data:`PI`."""

QUARTER_PI: Final = PI / 4.0
"""π/4: an exact quartering of :data:`PI`."""

TWO_PI: Final = PI * 2.0
"""2π: an exact doubling of :data:`PI`."""

_INV_LN2: Final = 1.4426950408889634
"""1 / ln 2, the correctly rounded literal."""

_DB_PER_OCTAVE: Final = 3.010299956639812
"""10 log₁₀ 2, the correctly rounded literal: decibels per doubling."""

_SQRT_HALF: Final = 0.7071067811865476
"""√½, the correctly rounded literal."""

_TAN_PI_8: Final = 0.41421356237309503
"""tan(π/8) = √2 - 1, the correctly rounded literal."""

_S3: Final = -1.0 / 6
_S5: Final = 1.0 / 120
_S7: Final = -1.0 / 5040
_S9: Final = 1.0 / 362_880
_S11: Final = -1.0 / 39_916_800
_S13: Final = 1.0 / 6_227_020_800
_S15: Final = -1.0 / 1_307_674_368_000
"""Taylor coefficients of sin x, (-1)^k / (2k+1)!, each one exactly rounded division."""

_C2: Final = -1.0 / 2
_C4: Final = 1.0 / 24
_C6: Final = -1.0 / 720
_C8: Final = 1.0 / 40_320
_C10: Final = -1.0 / 3_628_800
_C12: Final = 1.0 / 479_001_600
_C14: Final = -1.0 / 87_178_291_200
_C16: Final = 1.0 / 20_922_789_888_000
"""Taylor coefficients of cos x, (-1)^k / (2k)!."""

_A3: Final = -1.0 / 3
_A5: Final = 1.0 / 5
_A7: Final = -1.0 / 7
_A9: Final = 1.0 / 9
_A11: Final = -1.0 / 11
_A13: Final = 1.0 / 13
_A15: Final = -1.0 / 15
_A17: Final = 1.0 / 17
_A19: Final = -1.0 / 19
_A21: Final = 1.0 / 21
_A23: Final = -1.0 / 23
"""Taylor coefficients of atan u, (-1)^k / (2k+1)."""

_ATANH_TERMS: Final = tuple(1.0 / (2 * k + 1) for k in range(13))
"""Series coefficients of atanh s: 1 / (2k+1), k = 0..12 (to s²⁵)."""


def _horner(terms: tuple[float, ...], x: float) -> float:
    """Evaluate ``terms[0] + terms[1] x + terms[2] x² + ...`` in Horner form."""
    total = 0.0
    for coefficient in reversed(terms):
        total = total * x + coefficient
    return total


def _sin_cos_octant(x: float) -> tuple[float, float]:
    """Return (sin x, cos x) for x in [0, π/4] by Taylor polynomials (to x¹⁵ and x¹⁶),
    in Horner form, written out for speed (this is the geometry's inner loop)."""
    q = x * x
    sin_x = x * (
        1.0 + q * (_S3 + q * (_S5 + q * (_S7 + q * (_S9 + q * (_S11 + q * (_S13 + q * _S15))))))
    )
    cos_x = 1.0 + q * (
        _C2 + q * (_C4 + q * (_C6 + q * (_C8 + q * (_C10 + q * (_C12 + q * (_C14 + q * _C16))))))
    )
    return sin_x, cos_x


def _finite(name: str, value: float) -> float:
    """Return ``value``, rejecting NaN and infinities (``math.isfinite`` rejects
    non-numbers with ``TypeError``), as cheaply as possible: these run every tick."""
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return value


def portable_sincos_turns(turns: float) -> tuple[float, float]:
    """Return (sine, cosine) of an angle given in turns (1 turn = 360° = 2π rad).

    Whole turns are removed with ``floor`` (exact), the remainder is reduced to a
    quadrant and then to [0, π/4], and sine and cosine there come from Taylor
    polynomials to the 15th and 16th power. The absolute error against ``math`` is
    below 1e-12 for every finite input (ADR-0011 §5). Callers that hold an angle as
    integer microseconds of a period should reduce it with integer modulo first, so
    ``turns`` stays small and exact.

    Args:
        turns: The angle in turns. Any finite value.

    Returns:
        ``(sin, cos)`` of the angle.

    Raises:
        TypeError: ``turns`` is not a number.
        ValueError: ``turns`` is not finite.
    """
    value = _finite("turns", turns)
    fraction = value - math.floor(value)  # exact: [0, 1)
    scaled = fraction * 4.0  # exact: a power of two
    quadrant = math.floor(scaled)
    within = scaled - quadrant  # exact: [0, 1), a fraction of the quadrant
    if within <= 0.5:
        sin_part, cos_part = _sin_cos_octant(within * HALF_PI)
    else:
        cos_part, sin_part = _sin_cos_octant((1.0 - within) * HALF_PI)
    quadrant %= 4  # a fraction just below 1 can scale to exactly 4.0
    if quadrant == 0:
        return sin_part, cos_part
    if quadrant == 1:
        return cos_part, -sin_part
    if quadrant == 2:
        return -sin_part, -cos_part
    return -cos_part, sin_part


def _atan_unit(z: float) -> float:
    """Return atan z for z in [0, 1]."""
    offset = 0.0
    if z > _TAN_PI_8:
        # atan z = π/4 + atan((z - 1) / (z + 1)), with the argument in (-0.18, 0].
        offset = QUARTER_PI
        z = (z - 1.0) / (z + 1.0)
    # Halve the angle once: atan z = 2 atan(z / (1 + √(1 + z²))), |argument| < 0.2.
    u = z / (1.0 + math.sqrt(1.0 + z * z))
    q = u * u
    series = 1.0 + q * (
        _A3
        + q
        * (
            _A5
            + q
            * (
                _A7
                + q
                * (
                    _A9
                    + q
                    * (
                        _A11
                        + q * (_A13 + q * (_A15 + q * (_A17 + q * (_A19 + q * (_A21 + q * _A23)))))
                    )
                )
            )
        )
    )
    return offset + 2.0 * u * series


def portable_atan2(y: float, x: float) -> float:
    """Return the angle of the point (x, y) in radians, in (-π, π].

    Like ``math.atan2``, except that ``atan2(0, 0)`` is 0 and the sign of a zero ``y``
    is ignored (a point on the negative x axis gives +π, never -π). The ratio of the
    smaller to the larger magnitude is reduced to [0, tan(π/8)], halved once with
    ``sqrt``, and evaluated by a Taylor series. The absolute error against ``math`` is
    below 1e-12 rad (ADR-0011 §5).

    Args:
        y: Ordinate. Any finite value.
        x: Abscissa. Any finite value.

    Returns:
        The angle in radians.

    Raises:
        TypeError: An argument is not a number.
        ValueError: An argument is not finite.
    """
    y_value = _finite("y", y)
    x_value = _finite("x", x)
    ay = abs(y_value)
    ax = abs(x_value)
    if ay == 0.0 and ax == 0.0:
        return 0.0
    angle = HALF_PI - _atan_unit(ax / ay) if ay > ax else _atan_unit(ay / ax)
    if x_value < 0.0:
        angle = PI - angle
    if y_value < 0.0:
        angle = -angle
    return angle


def portable_log2(x: float) -> float:
    """Return the base-2 logarithm of ``x``.

    ``math.frexp`` splits ``x`` exactly into m · 2^e (it reads the float's bits; it is
    allowed by ADR-0006 as amended by ADR-0011 §6). The mantissa is moved into
    [1/√2, √2), and ln m = 2 atanh((m - 1)/(m + 1)) by an odd series to the 25th power.
    The absolute error against ``math.log2`` is below 1e-12 for every positive finite
    input, subnormals included.

    Args:
        x: A positive, finite number.

    Returns:
        log₂ x.

    Raises:
        TypeError: ``x`` is not a number.
        ValueError: ``x`` is not finite or not positive.
    """
    value = _finite("x", x)
    if value <= 0.0:
        raise ValueError(f"x must be positive, got {value}")
    mantissa, exponent = math.frexp(value)
    if mantissa < _SQRT_HALF:
        mantissa *= 2.0  # exact
        exponent -= 1
    s = (mantissa - 1.0) / (mantissa + 1.0)
    ln_mantissa = 2.0 * s * _horner(_ATANH_TERMS, s * s)
    return exponent + ln_mantissa * _INV_LN2


def portable_db(ratio: float) -> float:
    """Return a power ratio in decibels, 10 log₁₀(ratio).

    Computed as 10 log₁₀2 · :func:`portable_log2`. The absolute error against
    ``10 * math.log10`` is below 1e-11 dB for every positive finite ratio
    (ADR-0011 §6).

    Args:
        ratio: A positive, finite power ratio.

    Returns:
        The ratio in decibels.

    Raises:
        TypeError: ``ratio`` is not a number.
        ValueError: ``ratio`` is not finite or not positive.
    """
    value = _finite("ratio", ratio)
    if value <= 0.0:
        raise ValueError(f"ratio must be positive, got {value}")
    return _DB_PER_OCTAVE * portable_log2(value)
