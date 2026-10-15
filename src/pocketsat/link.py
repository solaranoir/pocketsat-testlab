"""The link between the ground station and the spacecraft (ADR-0010 §5).

The run loop moves frames through a :class:`Link`: the station's uplink
:class:`~pocketsat.messages.Packet` objects go in at step 4 and come out as frame bytes
for ``TestTarget.send()``; the frames from ``TestTarget.receive()`` go in at step 8 and
come out as packets for the station (``docs/architecture.md`` §3, §8). Only frame bytes
cross the ``TestTarget`` boundary, so the link unwraps packets on the way up and wraps
frames in new packets on the way down.

The link never calls the target, and the station never sees the link. A link returns
the frames or packets due *now*; a link with latency (Phase 3's RF channel,
``pocketsat.rf``) queues them and returns them on a later call, behind the same
protocol, so replacing :class:`IdealLink` changes neither the station nor the run loop.

Visibility is physics, so it belongs to the link: while the spacecraft is below the
geometric horizon (``elevation_deg < 0``) nothing is delivered in either direction. The
station's elevation mask is its operating policy and is applied by the station (#164),
not here. Visibility comes from the :class:`~pocketsat.environment.geometry.PassGeometry`
the run loop passes on every call; this supersedes #153's original wording of an
injected visibility function (ADR-0010 §5).
"""

from collections.abc import Sequence
from typing import Final, Protocol, runtime_checkable

from pocketsat.core.rng import RngFactory
from pocketsat.doppler import doppler_shift_hz
from pocketsat.environment.geometry import PassGeometry
from pocketsat.messages import Packet

DEFAULT_DOWNLINK_FREQUENCY_HZ: Final = 435_500_000
"""Default downlink carrier of :class:`IdealLink`, Hz: the default
``StationConfig.downlink_frequency_hz``. The run loop passes the station's value."""


@runtime_checkable
class Link(Protocol):
    """Carries frames between the ground station and the target, one tick at a time.

    Implementations: :class:`IdealLink` (Phase 2) and Phase 3's RF channel. ``now_us``
    is the simulated time of the call (ADR-0003); ``geometry`` is the tick's pass
    geometry, or ``None`` when no geometry model is running (the spacecraft is then
    treated as always visible).
    """

    def reset(self, rngs: RngFactory) -> None:
        """Return to the initial state, drawing any randomness from ``rngs`` (``link.*``)."""
        ...

    def carry_uplink(
        self, packets: Sequence[Packet], now_us: int, geometry: PassGeometry | None
    ) -> list[bytes]:
        """Take the station's uplink packets; return the frames due at the target now."""
        ...

    def carry_downlink(
        self, frames: Sequence[bytes], now_us: int, geometry: PassGeometry | None
    ) -> list[Packet]:
        """Take the target's downlink frames; return the packets due at the station now."""
        ...


class IdealLink:
    """A lossless, instant link, gated only by the horizon.

    Every packet or frame is delivered in the same call, unchanged and in order, unless
    the spacecraft is below the horizon, when it is dropped and counted. Downlink
    packets carry ``elevation_deg``, ``range_km`` and ``doppler_hz`` from the geometry
    when there is one; every other link attribute stays ``None`` (the SNR indicator comes
    from #163, loss and latency from Phase 3). ``doppler_hz`` is the true shift of the
    downlink carrier, :func:`~pocketsat.doppler.doppler_shift_hz` of
    ``downlink_frequency_hz`` and the geometry's range rate (#162).

    Args:
        downlink_frequency_hz: The spacecraft's downlink carrier, Hz; the station's
            ``StationConfig.downlink_frequency_hz``. Default
            :data:`DEFAULT_DOWNLINK_FREQUENCY_HZ`.
    """

    def __init__(self, *, downlink_frequency_hz: int = DEFAULT_DOWNLINK_FREQUENCY_HZ) -> None:
        if isinstance(downlink_frequency_hz, bool) or not isinstance(downlink_frequency_hz, int):
            raise TypeError(f"downlink_frequency_hz must be an int, got {downlink_frequency_hz!r}")
        if downlink_frequency_hz <= 0:
            raise ValueError(f"downlink_frequency_hz must be positive, got {downlink_frequency_hz}")
        self._downlink_frequency_hz = downlink_frequency_hz
        self._uplink_dropped_count = 0
        self._downlink_dropped_count = 0

    @property
    def uplink_dropped_count(self) -> int:
        """Uplink frames dropped below the horizon since the last reset."""
        return self._uplink_dropped_count

    @property
    def downlink_dropped_count(self) -> int:
        """Downlink frames dropped below the horizon since the last reset."""
        return self._downlink_dropped_count

    def reset(self, rngs: RngFactory) -> None:
        """Clear the drop counts. The ideal link draws no random numbers."""
        self._uplink_dropped_count = 0
        self._downlink_dropped_count = 0

    def carry_uplink(
        self, packets: Sequence[Packet], now_us: int, geometry: PassGeometry | None
    ) -> list[bytes]:
        """Unwrap each packet to its frame, or drop them all below the horizon."""
        if not _visible(geometry):
            self._uplink_dropped_count += len(packets)
            return []
        return [packet.frame for packet in packets]

    def carry_downlink(
        self, frames: Sequence[bytes], now_us: int, geometry: PassGeometry | None
    ) -> list[Packet]:
        """Wrap each frame in a new packet, or drop them all below the horizon."""
        if not _visible(geometry):
            self._downlink_dropped_count += len(frames)
            return []
        if geometry is None:
            return [Packet(frame=bytes(frame)) for frame in frames]
        doppler_hz = doppler_shift_hz(self._downlink_frequency_hz, geometry.range_rate_km_per_s)
        return [
            Packet(
                frame=bytes(frame),
                elevation_deg=geometry.elevation_deg,
                range_km=geometry.range_km,
                doppler_hz=doppler_hz,
            )
            for frame in frames
        ]


def _visible(geometry: PassGeometry | None) -> bool:
    """True unless the spacecraft is below the geometric horizon; no geometry is visible."""
    return geometry is None or geometry.elevation_deg >= 0.0
