"""Ground station events (ADR-0010 §4).

Every observable thing the station does is an event: a frozen record carrying the
simulated time ``time_us`` at which it happened. Events are returned from
:meth:`~pocketsat.groundstation.GroundStation.begin_tick` and
:meth:`~pocketsat.groundstation.GroundStation.end_tick`; the station keeps no history of
them. The console (#165) formats them and assertions query them, so a new observable is
a new event type here, not a new station accessor.

Events defined here before the ticket that emits them exist so the whole stream is
settled at once; each names its ticket. Field names follow the units convention
(``docs/architecture.md`` §4.1).
"""

from dataclasses import dataclass
from enum import Enum

from pocketsat.core.clock import check_us
from pocketsat.flight.modes import RejectReason
from pocketsat.messages import DecodeReason, Telemetry


@dataclass(frozen=True)
class CommandRef:
    """Identifies one submitted command across all its attempts (ADR-0010 §2).

    Ground-local: assigned by :meth:`~pocketsat.groundstation.GroundStation.submit` in
    submission order from 0 after each reset, and never on the wire. Each attempt has
    its own uplink sequence number; events link them to the command through this
    reference.

    Attributes:
        ref_id: The reference number.
    """

    ref_id: int

    def __post_init__(self) -> None:
        if isinstance(self.ref_id, bool) or not isinstance(self.ref_id, int):
            raise TypeError(f"ref_id must be an int, got {self.ref_id!r}")
        if self.ref_id < 0:
            raise ValueError(f"ref_id must be non-negative, got {self.ref_id}")


class FailureReason(Enum):
    """Why a command failed without an ACK or NACK (#154, #155)."""

    RETRIES_EXHAUSTED = "retries_exhausted"
    """Every allowed attempt timed out."""

    NOT_RETRYABLE = "not_retryable"
    """The only attempt timed out and the command's policy forbids a retry (RESET)."""

    EXPIRED = "expired"
    """The command was never sent: it was still queued ``command_expiry_us`` after it
    became due (#154)."""


class RejectionReason(Enum):
    """Why a received frame was rejected (#156)."""

    SYNC = "sync"
    """The first two bytes are not the sync word."""

    LENGTH = "length"
    """The frame is shorter or longer than its header says."""

    CRC = "crc"
    """The CRC does not match."""

    TYPE = "type"
    """The frame type is not assigned."""

    PAYLOAD = "payload"
    """The frame is valid but its payload does not decode for its type."""

    WRONG_DIRECTION = "wrong_direction"
    """A valid frame of an uplink type (COMMAND) received on the downlink, as
    ``EchoTarget`` produces."""


class RadioState(Enum):
    """The station radio's state (#164). #164 may add states."""

    RECEIVE = "receive"
    TRANSMIT = "transmit"


@dataclass(frozen=True)
class _Event:
    """Base of every event: the simulated time it happened.

    Attributes:
        time_us: Simulated time, integer microseconds (ADR-0003).
    """

    time_us: int

    def __post_init__(self) -> None:
        check_us("time_us", self.time_us)


@dataclass(frozen=True)
class Aos(_Event):
    """Acquisition of signal: the spacecraft rose above the elevation mask (#161).

    Attributes:
        pass_id: Pass number, from 1 after each reset.
        azimuth_deg: Azimuth at AOS.
        elevation_deg: Elevation at AOS.
    """

    pass_id: int
    azimuth_deg: float
    elevation_deg: float


@dataclass(frozen=True)
class Los(_Event):
    """Loss of signal: the spacecraft set below the elevation mask (#161).

    Attributes:
        pass_id: Pass number, as at its AOS.
        azimuth_deg: Azimuth at LOS.
        elevation_deg: Elevation at LOS.
        max_elevation_deg: Highest elevation of the pass.
    """

    pass_id: int
    azimuth_deg: float
    elevation_deg: float
    max_elevation_deg: float


@dataclass(frozen=True)
class CommandSent(_Event):
    """A COMMAND frame was handed to the link (#154).

    Attributes:
        command_ref: The command.
        command_id: Its command ID byte.
        sequence: The frame's uplink header sequence.
        retry_count: Retries before this attempt: 0 for the first attempt.
    """

    command_ref: CommandRef
    command_id: int
    sequence: int
    retry_count: int


@dataclass(frozen=True)
class CommandAcked(_Event):
    """An ACK answered one of the command's attempts (#155).

    Attributes:
        command_ref: The command.
        sequence: The uplink sequence the ACK answered.
    """

    command_ref: CommandRef
    sequence: int


@dataclass(frozen=True)
class CommandNacked(_Event):
    """A NACK answered one of the command's attempts (#155).

    Attributes:
        command_ref: The command.
        sequence: The uplink sequence the NACK answered.
        reason: The NACK reason (ADR-0009 §4).
    """

    command_ref: CommandRef
    sequence: int
    reason: DecodeReason | RejectReason


@dataclass(frozen=True)
class CommandTimedOut(_Event):
    """An attempt got no answer within the configured timeout (#155).

    Attributes:
        command_ref: The command.
        retry_count: Retries before the attempt that timed out.
        will_retry: Whether another attempt follows.
    """

    command_ref: CommandRef
    retry_count: int
    will_retry: bool


@dataclass(frozen=True)
class CommandFailed(_Event):
    """The command will get no answer (#155).

    Attributes:
        command_ref: The command.
        reason: Why it failed.
    """

    command_ref: CommandRef
    reason: FailureReason


@dataclass(frozen=True)
class UnmatchedAck(_Event):
    """An ACK or NACK that answers no outstanding attempt (#155): an unknown or
    already-answered sequence. Counted and reported, not an error.

    Attributes:
        sequence: The uplink sequence the ACK names.
        command_id: The command ID it echoes.
    """

    sequence: int
    command_id: int


@dataclass(frozen=True)
class TelemetryReceived(_Event):
    """A TELEMETRY frame decoded (#156).

    Attributes:
        sequence: The frame's downlink header sequence.
        telemetry: The decoded payload.
    """

    sequence: int
    telemetry: Telemetry


@dataclass(frozen=True)
class DataChunkReceived(_Event):
    """A DATA frame decoded (#158).

    Attributes:
        sequence: The frame's downlink header sequence.
        chunk_id: The chunk's ID.
        content_ok: Whether the content matches ``chunk_content(chunk_id)`` at the
            configured chunk size. A chunk that does not is corrupted and not stored.
        duplicate: Whether a chunk with this ID was already stored; the stored copy is
            kept.
    """

    sequence: int
    chunk_id: int
    content_ok: bool
    duplicate: bool = False


@dataclass(frozen=True)
class FrameRejected(_Event):
    """A received frame was rejected (#156).

    Attributes:
        reason: Why.
        frame: The raw bytes received.
    """

    reason: RejectionReason
    frame: bytes


@dataclass(frozen=True)
class SequenceGap(_Event):
    """Downlink frames are missing before this one (#156, ADR-0009 §7).

    The expected sequence is ``(sequence - missing_count) mod 65536``.

    Attributes:
        sequence: The downlink sequence received.
        missing_count: Frames missing before it.
    """

    sequence: int
    missing_count: int


@dataclass(frozen=True)
class SpacecraftRebooted(_Event):
    """A downlink sequence restart paired with a higher boot count (#156, ADR-0009 §7).

    Attributes:
        sequence: The downlink sequence of the telemetry that showed it.
        boot_count: The new boot count.
    """

    sequence: int
    boot_count: int


@dataclass(frozen=True)
class RadioStateChanged(_Event):
    """The station radio changed state (#164).

    Attributes:
        state: The new state.
    """

    state: RadioState


@dataclass(frozen=True)
class StationIdentified(_Event):
    """The station identified itself (#164; off the wire, ADR-0010 §9).

    Attributes:
        callsign: The callsign.
    """

    callsign: str


@dataclass(frozen=True)
class RadioTuned(_Event):
    """The station tuned its radio for the predicted Doppler shift (#162).

    Emitted at the start of every tick of a pass (AOS included, LOS not), after the
    pass events. The shifts are the station's prediction from its geometry (ADR-0010
    §6); the residuals are what the tuning leaves uncorrected: at most half
    ``tuning_step_hz`` with ``doppler_correction`` on, the full shift with it off.

    Attributes:
        uplink_doppler_hz: Predicted shift of the uplink carrier at the spacecraft.
        downlink_doppler_hz: Predicted shift of the downlink carrier at the station.
        uplink_frequency_hz: Tuned transmit frequency, pre-compensated so the
            spacecraft hears the nominal uplink frequency.
        downlink_frequency_hz: Tuned receive frequency.
        uplink_residual_hz: Offset from the nominal uplink frequency at which the
            spacecraft hears the station.
        downlink_residual_hz: Offset of the received downlink carrier from the tuned
            receive frequency.
    """

    uplink_doppler_hz: float
    downlink_doppler_hz: float
    uplink_frequency_hz: int
    downlink_frequency_hz: int
    uplink_residual_hz: float
    downlink_residual_hz: float


StationEvent = (
    Aos
    | Los
    | CommandSent
    | CommandAcked
    | CommandNacked
    | CommandTimedOut
    | CommandFailed
    | UnmatchedAck
    | TelemetryReceived
    | DataChunkReceived
    | FrameRejected
    | SequenceGap
    | SpacecraftRebooted
    | RadioStateChanged
    | StationIdentified
    | RadioTuned
)
"""Every event the ground station emits (ADR-0010 §4)."""

EVENT_TYPES: tuple[type[StationEvent], ...] = (
    Aos,
    Los,
    CommandSent,
    CommandAcked,
    CommandNacked,
    CommandTimedOut,
    CommandFailed,
    UnmatchedAck,
    TelemetryReceived,
    DataChunkReceived,
    FrameRejected,
    SequenceGap,
    SpacecraftRebooted,
    RadioStateChanged,
    StationIdentified,
    RadioTuned,
)
"""The members of :data:`StationEvent`, in the order of ADR-0010 §4's table, then the
events later tickets added (``docs/groundstation.md``, "Events")."""
