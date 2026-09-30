"""Typed messages exchanged between components above the ``TestTarget`` boundary.

The target boundary itself carries raw frame bytes (see :mod:`pocketsat.frame`). These
types are deliberately minimal and are extended in later phases.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    """A ground-originated request, encoded into a COMMAND frame by the ground station.

    Attributes:
        command_id: Command identifier. Assigned values are listed in ``docs/protocol.md``.
        payload: Command-specific argument bytes.
    """

    command_id: int
    payload: bytes = b""


@dataclass(frozen=True)
class Telemetry:
    """Spacecraft state decoded from a TELEMETRY frame by the ground station.

    Attributes:
        uptime_ms: Spacecraft uptime counter from the payload, in milliseconds.
        payload: Remaining telemetry bytes; the field layout is defined in a later phase.
    """

    uptime_ms: int
    payload: bytes = b""


@dataclass(frozen=True)
class Packet:
    """A frame in transit, plus link-state attributes attached by the RF channel.

    Link-state attributes are ``None`` until the RF channel sets them.

    Attributes:
        frame: Encoded frame bytes. Corruption faults mutate these bytes.
        elevation_deg: Elevation of the spacecraft above the station horizon.
        range_km: Slant range between station and spacecraft.
        doppler_hz: Doppler shift applied to the carrier.
        snr_db: Signal-to-noise ratio.
        loss_probability: Probability, ``0..1``, that this packet is dropped.
        latency_s: One-way link latency in simulated seconds.
    """

    frame: bytes
    elevation_deg: float | None = None
    range_km: float | None = None
    doppler_hz: float | None = None
    snr_db: float | None = None
    loss_probability: float | None = None
    latency_s: float | None = None
