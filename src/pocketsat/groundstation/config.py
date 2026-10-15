"""Ground station configuration (ADR-0010 §3).

Like the spacecraft settings (ADR-0005), the station is built with one frozen, validated
:class:`StationConfig`, and :meth:`~pocketsat.groundstation.GroundStation.reset` returns
to it. Scenarios change single values with ``dataclasses.replace()``. The values are
plain values, never random.
"""

import math
import re
from dataclasses import dataclass
from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.groundstation.commands import (
    DEFAULT_DOWNLINK_RATE_BYTES_PER_S,
    max_answer_latency_us,
)
from pocketsat.spacecraft.config import MAX_CHUNK_SIZE_BYTES

CALLSIGN_PATTERN: Final = re.compile(r"[A-Z0-9/]{3,16}")
"""A callsign: 3 to 16 uppercase letters, digits and ``/`` (portable suffixes such as
``/P``). The station does not check that a callsign is issued; it is never on the wire
(ADR-0010 §9)."""

MIN_ALTITUDE_KM: Final = -0.5
"""Lowest station altitude accepted, km: below the Dead Sea shore (about -0.43 km)."""

MAX_ALTITUDE_KM: Final = 9.0
"""Highest station altitude accepted, km: above the summit of Everest (about 8.85 km)."""

MAX_COMMAND_RETRY_COUNT: Final = 10
"""Most retries a configuration may ask for. More only delays a command's failure."""

DEFAULT_MAX_UPLINK_COMMAND_COUNT: Final = 4
"""Default for :attr:`StationConfig.max_uplink_command_count`; see its documentation."""

DEFAULT_DATA_CHUNK_SIZE_BYTES: Final = 64
"""Default for :attr:`StationConfig.data_chunk_size_bytes`: the spacecraft's default
``PayloadConfig.chunk_size_bytes``."""
DEFAULT_TELEMETRY_RETENTION_COUNT: Final = 20_000
"""Default for :attr:`StationConfig.telemetry_retention_count`; see its documentation."""

DEFAULT_TUNING_STEP_HZ: Final = 10
"""Default for :attr:`StationConfig.tuning_step_hz`; see its documentation."""


def _number(name: str, value: float) -> float:
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
class StationConfig:
    """What the ground station is: identity, location, radio and command settings.

    The defaults describe a plausible amateur-satellite station. They are a test
    convenience, not a real station.

    Attributes:
        callsign: Station identity, matching :data:`CALLSIGN_PATTERN`. Used in the
            console, the run record and identification events, never on the wire
            (ADR-0010 §9). Default ``N0CALL``, the placeholder callsign amateur software
            uses, so no real station is named.
        latitude_deg: Geodetic latitude, ``-90..90``, north positive. Default 40.0, a
            mid-latitude station that sees several passes a day of a typical
            low-inclination or polar LEO orbit.
        longitude_deg: Longitude, ``-180..180`` (``-180`` and ``180`` are the same
            meridian), east positive. Default -105.0.
        altitude_km: Altitude above the reference ellipsoid, :data:`MIN_ALTITUDE_KM` to
            :data:`MAX_ALTITUDE_KM`. Default 1.6.
        elevation_mask_deg: Minimum elevation at which the station operates, ``0..90``
            (exclusive of 90): the AOS/LOS threshold (#161) and the transmit limit
            (#164). The link applies the geometric horizon separately (ADR-0010 §5).
            Default 10.0: below about 10° a pass is long-range, often blocked by terrain
            and buildings, and of little use to a small station.
        uplink_frequency_hz: Nominal uplink carrier before Doppler correction (#162).
            Default 145_850_000, in the 2 m amateur-satellite sub-band (145.8 to
            146.0 MHz).
        downlink_frequency_hz: Nominal downlink carrier before Doppler correction.
            Default 435_500_000, in the 70 cm amateur-satellite sub-band (435 to
            438 MHz).
        command_timeout_us: Simulated time from a command's transmission to its
            timeout (#155). Default 2_000_000 (2 s, 20 ticks at the default 100 ms
            tick): command to ACK is the same tick at the default tick (ADR-0004), and
            a RESET ACK can be a few ticks late at short ticks (ADR-0008 §6a), so 2 s
            leaves a wide margin without slowing a failed command's report much.
        command_retry_count: Retries after the first attempt, for commands whose policy
            allows them (ADR-0010 §2: RESET is never retried), ``0..``
            :data:`MAX_COMMAND_RETRY_COUNT`. Default 2.
        identification_interval_us: Station identification interval while
            transmitting (#164). Default 600_000_000 (10 minutes), the interval amateur
            rules commonly require (for example FCC §97.119(a)).
        max_uplink_command_count: Most COMMAND frames sent in one tick (#154); the rest
            wait for the next tick. Positive. Default
            :data:`DEFAULT_MAX_UPLINK_COMMAND_COUNT` (4): every command is answered by a
            14-byte ACK frame in the same tick, and the default downlink carries 120
            bytes per 100 ms tick (``OutboundConfig.max_held_ack_frames``), so four ACKs
            (56 bytes) still leave room for a 36-byte TELEMETRY frame in the tick.
            Sending more would push telemetry or ACKs into the next tick.
        command_expiry_us: How long after its due time a command that could not be
            sent (the transmit gate stayed closed) is dropped and reported as failed
            (#154), simulated µs, or ``None`` to keep it queued until it is sent.
            Default ``None``: a command waits for the next pass, as an operator's queue
            does, unless a scenario asks for an expiry.
        data_chunk_size_bytes: Size of every payload data chunk, bytes,
            ``1..MAX_CHUNK_SIZE_BYTES`` (#158). The chunk size is not on the wire: both
            ends are configured with it (ADR-0009 §6), so it must equal the spacecraft's
            ``PayloadConfig.chunk_size_bytes``, and the station checks every DATA
            chunk's content against ``chunk_content`` at this size. Default
            :data:`DEFAULT_DATA_CHUNK_SIZE_BYTES` (64), the spacecraft's default.
        telemetry_retention_count: Most telemetry records the telemetry store keeps
            (#157); when full, the oldest is dropped and counted. Positive. Default
            :data:`DEFAULT_TELEMETRY_RETENTION_COUNT` (20 000): at the default 1 s
            telemetry cadence that is about 5.5 hours of continuous reception, or
            several days of passes, which covers any Phase 2 scenario, in about 10 MB
            (about 520 bytes per record, measured), so a campaign of many runs stays
            bounded.
        doppler_correction: Whether the station tunes its radio for the predicted
            Doppler shift each tick (#162, :mod:`pocketsat.groundstation.tuning`). With
            it off the radio stays on the nominal frequencies and the residual tuning
            error is the full shift: the switch Phase 4's Doppler tracking error fault
            uses (``docs/architecture.md`` §9). Default ``True``, as a satellite station
            runs with its tracking software correcting Doppler.
        tuning_step_hz: The radio's tuning resolution, Hz, positive (#162): each
            Doppler correction is rounded to a whole number of steps, so with
            correction on the residual tuning error is at most half a step. Default
            :data:`DEFAULT_TUNING_STEP_HZ` (10), a common CAT frequency resolution of
            amateur transceivers; the 5 Hz worst-case residual is three orders of
            magnitude below the shift it corrects (about ±10 kHz at 435.5 MHz).

    Raises:
        TypeError: A field has the wrong type.
        ValueError: A field is out of range.
    """

    callsign: str = "N0CALL"
    latitude_deg: float = 40.0
    longitude_deg: float = -105.0
    altitude_km: float = 1.6
    elevation_mask_deg: float = 10.0
    uplink_frequency_hz: int = 145_850_000
    downlink_frequency_hz: int = 435_500_000
    command_timeout_us: int = 2_000_000
    command_retry_count: int = 2
    identification_interval_us: int = 600_000_000
    max_uplink_command_count: int = DEFAULT_MAX_UPLINK_COMMAND_COUNT
    command_expiry_us: int | None = None
    data_chunk_size_bytes: int = DEFAULT_DATA_CHUNK_SIZE_BYTES
    telemetry_retention_count: int = DEFAULT_TELEMETRY_RETENTION_COUNT
    doppler_correction: bool = True
    tuning_step_hz: int = DEFAULT_TUNING_STEP_HZ

    def __post_init__(self) -> None:
        if not isinstance(self.callsign, str):
            raise TypeError(f"callsign must be a str, got {self.callsign!r}")
        if CALLSIGN_PATTERN.fullmatch(self.callsign) is None:
            raise ValueError(
                f"callsign must be 3 to 16 uppercase letters, digits and '/', got {self.callsign!r}"
            )
        latitude = _number("latitude_deg", self.latitude_deg)
        if not -90.0 <= latitude <= 90.0:
            raise ValueError(f"latitude_deg must be in [-90, 90], got {latitude}")
        longitude = _number("longitude_deg", self.longitude_deg)
        if not -180.0 <= longitude <= 180.0:
            raise ValueError(f"longitude_deg must be in [-180, 180], got {longitude}")
        altitude = _number("altitude_km", self.altitude_km)
        if not MIN_ALTITUDE_KM <= altitude <= MAX_ALTITUDE_KM:
            raise ValueError(
                f"altitude_km must be in [{MIN_ALTITUDE_KM}, {MAX_ALTITUDE_KM}], got {altitude}"
            )
        mask = _number("elevation_mask_deg", self.elevation_mask_deg)
        if not 0.0 <= mask < 90.0:
            raise ValueError(f"elevation_mask_deg must be in [0, 90), got {mask}")
        _positive_int("uplink_frequency_hz", self.uplink_frequency_hz)
        _positive_int("downlink_frequency_hz", self.downlink_frequency_hz)
        check_us("command_timeout_us", self.command_timeout_us)
        if self.command_timeout_us == 0:
            raise ValueError("command_timeout_us must be positive, got 0")
        retries = self.command_retry_count
        if isinstance(retries, bool) or not isinstance(retries, int):
            raise TypeError(f"command_retry_count must be an int, got {retries!r}")
        if not 0 <= retries <= MAX_COMMAND_RETRY_COUNT:
            raise ValueError(
                f"command_retry_count must be in 0..{MAX_COMMAND_RETRY_COUNT}, got {retries}"
            )
        check_us("identification_interval_us", self.identification_interval_us)
        if self.identification_interval_us == 0:
            raise ValueError("identification_interval_us must be positive, got 0")
        _positive_int("max_uplink_command_count", self.max_uplink_command_count)
        if self.command_expiry_us is not None:
            check_us("command_expiry_us", self.command_expiry_us)
        _positive_int("data_chunk_size_bytes", self.data_chunk_size_bytes)
        if self.data_chunk_size_bytes > MAX_CHUNK_SIZE_BYTES:
            raise ValueError(
                f"data_chunk_size_bytes must be at most {MAX_CHUNK_SIZE_BYTES}, "
                f"got {self.data_chunk_size_bytes}"
            )
        _positive_int("telemetry_retention_count", self.telemetry_retention_count)
        if not isinstance(self.doppler_correction, bool):
            raise TypeError(f"doppler_correction must be a bool, got {self.doppler_correction!r}")
        _positive_int("tuning_step_hz", self.tuning_step_hz)

    def check_tick(
        self, tick_us: int, downlink_rate_bytes_per_s: int = DEFAULT_DOWNLINK_RATE_BYTES_PER_S
    ) -> None:
        """Check that this configuration can run at a tick of ``tick_us``.

        The command timeout must cover the longest legitimate answer
        (:func:`~pocketsat.groundstation.commands.max_answer_latency_us`, #155), so a
        command answered as late as the protocol allows never times out: one tick in
        every whole-frame tick (ADR-0004, "command → ACK, same tick"; 100 ms at the
        default tick), and up to the airtime of the frame in transmission, the held ACKs
        and the answer itself in streaming ticks (ADR-0008 §5), rounded up to whole
        ticks. The run loop calls this with the run's tick (#167).

        Args:
            tick_us: The run's tick length, simulated microseconds.
            downlink_rate_bytes_per_s: The spacecraft's transmit rate (default 1200).

        Raises:
            TypeError: An argument is not an int.
            ValueError: An argument is not positive, or ``command_timeout_us`` is
                shorter than the longest legitimate answer.
        """
        bound_us = max_answer_latency_us(tick_us, downlink_rate_bytes_per_s)
        if self.command_timeout_us < bound_us:
            raise ValueError(
                f"command_timeout_us ({self.command_timeout_us}) must be at least the "
                f"longest legitimate answer, {bound_us} µs at a {tick_us} µs tick"
            )


DEFAULT_STATION_CONFIG: Final = StationConfig()
"""The default station: the defaults documented on :class:`StationConfig`."""
