"""The ground station (ADR-0010): pass state, radio control, command uplink, decoding.

Shared infrastructure above the ``TestTarget`` boundary. The station is a pure
component: it takes simulated time, pass geometry and downlink packets, and returns
uplink packets and events. The run loop moves packets between it, the link and the
target. See ``docs/groundstation.md``.
"""

from pocketsat.groundstation.commands import max_answer_latency_us
from pocketsat.groundstation.config import DEFAULT_STATION_CONFIG, StationConfig
from pocketsat.groundstation.downlink import LinkStatistics
from pocketsat.groundstation.events import (
    EVENT_TYPES,
    Aos,
    CommandAcked,
    CommandFailed,
    CommandNacked,
    CommandRef,
    CommandSent,
    CommandTimedOut,
    DataChunkReceived,
    FailureReason,
    FrameRejected,
    Los,
    RadioState,
    RadioStateChanged,
    RejectionReason,
    SequenceGap,
    SpacecraftRebooted,
    StationEvent,
    StationIdentified,
    TelemetryReceived,
    UnmatchedAck,
)
from pocketsat.groundstation.station import GroundStation, Uplink
from pocketsat.groundstation.telemetry import (
    TELEMETRY_QUANTITIES,
    TelemetryQuantity,
    TelemetryRecord,
    TelemetryStore,
)

__all__ = [
    "DEFAULT_STATION_CONFIG",
    "EVENT_TYPES",
    "TELEMETRY_QUANTITIES",
    "Aos",
    "CommandAcked",
    "CommandFailed",
    "CommandNacked",
    "CommandRef",
    "CommandSent",
    "CommandTimedOut",
    "DataChunkReceived",
    "FailureReason",
    "FrameRejected",
    "GroundStation",
    "LinkStatistics",
    "Los",
    "RadioState",
    "RadioStateChanged",
    "RejectionReason",
    "SequenceGap",
    "SpacecraftRebooted",
    "StationConfig",
    "StationEvent",
    "StationIdentified",
    "TelemetryQuantity",
    "TelemetryReceived",
    "TelemetryRecord",
    "TelemetryStore",
    "UnmatchedAck",
    "Uplink",
    "max_answer_latency_us",
]
