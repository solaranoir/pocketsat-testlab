"""The downlink SNR indicator: a deterministic link budget (#163, ADR-0011 §6).

The budget is additive in decibels::

    SNR = P_tx + G_tx + G_rx - FSPL(d) - L - N
    FSPL(d) = 20 log₁₀(4π d f / c)                (free-space path loss)
    N = -174 dBm/Hz + 10 log₁₀(B) + NF            (noise floor in the bandwidth)

Only the range term changes from tick to tick. Every other term, including
20 log₁₀(4π · 1 km · f / c) and 10 log₁₀(B), is folded into one constant at
construction, so an SNR costs one :func:`~pocketsat.core.portable.portable_db` call:
``SNR(d) = SNR(1 km) - 20 log₁₀(d / 1 km)``. Decibels come from the portable
``frexp``-based logarithm (ADR-0006 as amended by ADR-0011 §6), so the values are
byte-identical on every platform.

The budget is a property of the link, not of the station or the spacecraft: the link
(:class:`~pocketsat.link.IdealLink` now, Phase 3's RF channel later) fills
``Packet.snr_db`` from it (ADR-0010 §5). It reads nothing across the ``TestTarget``
boundary, so it is the same for SIL and HIL targets. It is an indicator only: in Phase 2
no packet is dropped for a low SNR; Phase 3 maps SNR to loss.

The defaults describe a typical amateur UHF small-satellite downlink; the reasoning for
each is in ``docs/groundstation.md`` ("SNR indicator").
"""

import math
from dataclasses import dataclass, field
from typing import Final

from pocketsat.core.portable import PI, portable_db

SPEED_OF_LIGHT_M_PER_S: Final = 299_792_458.0
"""The speed of light in vacuum, m/s (exact by the SI definition of the metre)."""

THERMAL_NOISE_DENSITY_DBM_PER_HZ: Final = -174.0
"""Thermal noise power density kT₀ at the reference temperature T₀ = 290 K, dBm/Hz
(10 log₁₀(1.380649e-23 · 290 · 1000) = -173.98, conventionally rounded to -174; the
value ADR-0011 §6 fixes)."""

MIN_RANGE_KM: Final = 1.0
"""Shortest range the budget evaluates, km. Free-space loss goes to -∞ as the range goes
to 0, and the far-field formula does not hold at a few wavelengths anyway; a shorter
range (only a hand-built :class:`~pocketsat.environment.geometry.PassGeometry` can give
one; a real orbit is hundreds of km away) is evaluated at this range."""

_M_PER_KM: Final = 1000.0
_MW_PER_W: Final = 1000.0


def _finite(name: str, value: float) -> float:
    """Return ``value`` as a float, rejecting bools, non-numbers, NaN, and infinities."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return float(value)


def _positive_int(name: str, value: int) -> None:
    """Reject anything but a positive ``int`` (bools are rejected too)."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {value!r}")
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value}")


@dataclass(frozen=True)
class LinkBudget:
    """A downlink budget from spacecraft transmitter to station receiver (#163).

    Frozen and validated like the other configurations (ADR-0005); change single values
    with ``dataclasses.replace()``. The values are plain values, never random. Defaults
    are those of ``docs/groundstation.md`` ("SNR indicator"); with them, a pass of the
    default orbit (#160) runs from about 11 dB at the horizon to about 26 dB overhead.

    Attributes:
        frequency_hz: Downlink carrier frequency, positive. Default 435 500 000, the
            station's default ``downlink_frequency_hz`` (70 cm amateur-satellite
            sub-band). Not read from ``StationConfig``: the link does not depend on the
            station (ADR-0010 §5), so a scenario that moves one moves the other.
        tx_power_w: Spacecraft transmitter output power, positive watts. Default 1.0
            (30 dBm).
        tx_antenna_gain_db: Spacecraft antenna gain toward the station, dB relative to
            an isotropic antenna (dBi). Default 0.0 (a deployable monopole or dipole,
            close to omnidirectional).
        rx_antenna_gain_db: Station antenna gain, dBi. Default 12.0 (a tracked 70 cm
            crossed Yagi).
        losses_db: Every other loss, in total, non-negative dB: polarisation mismatch,
            feed line, spacecraft antenna pattern and pointing. Default 8.0.
        bandwidth_hz: Receiver noise bandwidth, positive. Default 15 000 (the IF filter
            of a 9600 bit/s G3RUH FSK receiver; the spacecraft sends 1200 bytes/s).
        noise_figure_db: Receiver system noise figure, non-negative dB. Default 3.0 (a
            mast-mounted preamplifier plus feed-line loss and antenna noise).

    Raises:
        TypeError: A field has the wrong type.
        ValueError: A field is out of range or not finite.
    """

    frequency_hz: int = 435_500_000
    tx_power_w: float = 1.0
    tx_antenna_gain_db: float = 0.0
    rx_antenna_gain_db: float = 12.0
    losses_db: float = 8.0
    bandwidth_hz: int = 15_000
    noise_figure_db: float = 3.0
    _path_loss_at_1_km_db: float = field(init=False, repr=False, compare=False)
    _snr_at_1_km_db: float = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        _positive_int("frequency_hz", self.frequency_hz)
        _positive_int("bandwidth_hz", self.bandwidth_hz)
        tx_power_w = _finite("tx_power_w", self.tx_power_w)
        if tx_power_w <= 0.0:
            raise ValueError(f"tx_power_w must be positive, got {tx_power_w}")
        tx_gain = _finite("tx_antenna_gain_db", self.tx_antenna_gain_db)
        rx_gain = _finite("rx_antenna_gain_db", self.rx_antenna_gain_db)
        losses = _finite("losses_db", self.losses_db)
        if losses < 0.0:
            raise ValueError(f"losses_db must be non-negative, got {losses}")
        noise_figure = _finite("noise_figure_db", self.noise_figure_db)
        if noise_figure < 0.0:
            raise ValueError(f"noise_figure_db must be non-negative, got {noise_figure}")
        # The constant terms, once (ADR-0011 §6): P_tx in dBm, 20 log₁₀(4π · 1 km · f / c)
        # and N = -174 + 10 log₁₀(B) + NF.
        amplitude_ratio = 4.0 * PI * _M_PER_KM * self.frequency_hz / SPEED_OF_LIGHT_M_PER_S
        path_loss = 2.0 * portable_db(amplitude_ratio)
        snr_at_1_km = (
            self.tx_power_dbm + tx_gain + rx_gain - path_loss - losses - self.noise_floor_dbm
        )
        object.__setattr__(self, "_path_loss_at_1_km_db", path_loss)
        object.__setattr__(self, "_snr_at_1_km_db", snr_at_1_km)

    @property
    def tx_power_dbm(self) -> float:
        """The transmitter power in dBm, 10 log₁₀(P / 1 mW)."""
        return portable_db(self.tx_power_w * _MW_PER_W)

    @property
    def noise_floor_dbm(self) -> float:
        """The receiver noise power in the bandwidth, -174 + 10 log₁₀(B) + NF, dBm."""
        return (
            THERMAL_NOISE_DENSITY_DBM_PER_HZ
            + portable_db(self.bandwidth_hz)
            + float(self.noise_figure_db)
        )

    def path_loss_db(self, range_km: float) -> float:
        """Free-space path loss at ``range_km``, 20 log₁₀(4π d f / c), dB.

        Args:
            range_km: Slant range, km, non-negative and finite. Ranges below
                :data:`MIN_RANGE_KM` are evaluated at :data:`MIN_RANGE_KM`.

        Raises:
            TypeError: ``range_km`` is not a number.
            ValueError: ``range_km`` is negative or not finite.
        """
        return self._path_loss_at_1_km_db + self._range_term_db(range_km)

    def snr_db(self, range_km: float) -> float:
        """The downlink signal-to-noise ratio at ``range_km``, dB.

        One :func:`~pocketsat.core.portable.portable_db` call: the constant terms were
        computed at construction. Within 1e-10 dB of the same budget evaluated with
        ``math.log10`` (``docs/groundstation.md``, "SNR indicator").

        Args:
            range_km: Slant range, km, non-negative and finite. Ranges below
                :data:`MIN_RANGE_KM` are evaluated at :data:`MIN_RANGE_KM`.

        Raises:
            TypeError: ``range_km`` is not a number.
            ValueError: ``range_km`` is negative or not finite.
        """
        return self._snr_at_1_km_db - self._range_term_db(range_km)

    @staticmethod
    def _range_term_db(range_km: float) -> float:
        """20 log₁₀(d / 1 km), with d at least :data:`MIN_RANGE_KM`."""
        distance = _finite("range_km", range_km)
        if distance < 0.0:
            raise ValueError(f"range_km must be non-negative, got {distance}")
        return 2.0 * portable_db(max(distance, MIN_RANGE_KM))


DEFAULT_LINK_BUDGET: Final = LinkBudget()
"""The default budget: the defaults documented on :class:`LinkBudget`."""
