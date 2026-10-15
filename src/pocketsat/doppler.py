"""Doppler shift from the pass geometry's range rate (#162, ADR-0011 §6).

The first-order Doppler shift of a carrier of frequency ``f`` between the station and a
spacecraft whose slant range changes at ``ṙ`` is ``Δf = -f · ṙ / c``: positive while the
spacecraft approaches (``ṙ < 0``), zero at closest approach, negative while it recedes.
It is arithmetic only (ADR-0006), with the speed of light as a literal constant.

This is the Phase 2 Doppler indicator of ADR-0010 §5, shared by the two sides that need
it and depending on neither, as :mod:`pocketsat.link` is: the link computes the true
shift from its geometry and puts it on each downlink packet (``Packet.doppler_hz``),
and the ground station computes the shift it predicts from its own geometry to tune its
radio (:mod:`pocketsat.groundstation.tuning`). Phase 3's RF channel (``pocketsat.rf``)
reuses it.

The shift is the same in both directions for the same carrier: the uplink is shifted by
``doppler_shift_hz(uplink_frequency_hz, ṙ)`` on its way to the spacecraft, the downlink
by ``doppler_shift_hz(downlink_frequency_hz, ṙ)`` on its way to the station. The second
order term (``f · ṙ² / c²``, under 0.1 Hz at 435 MHz in LEO) is neglected.
"""

from typing import Final

SPEED_OF_LIGHT_KM_PER_S: Final = 299_792.458
"""The speed of light in vacuum, km/s (exact by the SI definition of the metre)."""


def doppler_shift_hz(frequency_hz: float, range_rate_km_per_s: float) -> float:
    """Return the first-order Doppler shift of a carrier, Hz.

    Computed as ``(-frequency_hz · range_rate_km_per_s) / SPEED_OF_LIGHT_KM_PER_S`` in
    that order, so the result is the same on every platform (ADR-0006).

    Args:
        frequency_hz: The transmitted carrier frequency, Hz.
        range_rate_km_per_s: Rate of change of the slant range
            (``PassGeometry.range_rate_km_per_s``): negative while approaching.

    Returns:
        The received frequency minus the transmitted one, Hz: positive while the
        spacecraft approaches, negative while it recedes.
    """
    return -frequency_hz * range_rate_km_per_s / SPEED_OF_LIGHT_KM_PER_S
