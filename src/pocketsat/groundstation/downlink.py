"""Downlink decoding, sequence tracking and link statistics (#156, ADR-0010 §2, §4).

:class:`DownlinkDecoder` turns the packets the link delivers into events. For each
packet, in arrival order:

1. **Frame decoding** (``decode_frame``). A frame that fails (sync, length, CRC, type)
   becomes a :class:`FrameRejected` event and is counted by reason; its header cannot
   be trusted, so it takes no part in sequence tracking. A valid COMMAND frame is an
   uplink type and is rejected as ``WRONG_DIRECTION`` (``EchoTarget`` echoes uplink
   frames), also outside sequence tracking.
2. **Sequence tracking** for every valid downlink frame, all types sharing one counter
   (ADR-0009 §7). See :class:`SequenceTracker` for the gap, wrap and restart rules.
3. **Content**, routed by frame type: TELEMETRY is decoded into a
   :class:`TelemetryReceived` event, and a :class:`SpacecraftRebooted` event follows it
   when the boot count went up, and the telemetry is stored in the telemetry store
   (#157) when the decoder has one; an ACK/NACK is correlated with its command by the
   station's command tracker (#155), whose event follows; DATA is decoded and handed to
   the :class:`~pocketsat.groundstation.data.DataStore` (#158), which checks, stores and
   counts the chunk and returns its :class:`DataChunkReceived` event. A payload that
   does not decode for its type becomes ``FrameRejected`` with reason ``PAYLOAD``; its
   sequence was already tracked, because the CRC vouched for the header.

So per packet the events are frame-level first (``SequenceGap`` or ``FrameRejected``),
then content. Nothing raises for any received bytes.

Link statistics (:class:`LinkStatistics`) are kept in total and for the current pass.
Pass boundaries come from the pass state machine (#161), which calls
:meth:`DownlinkDecoder.start_pass`; until then the whole run is one pass.
"""

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Final

from pocketsat.frame import (
    MAX_SEQUENCE,
    Frame,
    FrameCrcError,
    FrameError,
    FrameLengthError,
    FrameSyncError,
    FrameType,
    decode_frame,
)
from pocketsat.groundstation.config import DEFAULT_DATA_CHUNK_SIZE_BYTES
from pocketsat.groundstation.data import DataStore
from pocketsat.groundstation.events import (
    FrameRejected,
    RejectionReason,
    SequenceGap,
    SpacecraftRebooted,
    StationEvent,
    TelemetryReceived,
)
from pocketsat.groundstation.telemetry import TelemetryRecord, TelemetryStore
from pocketsat.messages import (
    AckDecodeError,
    CommandAck,
    DataDecodeError,
    Packet,
    Telemetry,
    TelemetryDecodeError,
    decode_ack,
    decode_data,
    decode_telemetry,
)

SEQUENCE_MODULUS: Final = MAX_SEQUENCE + 1
"""Number of distinct header sequence values (65536)."""

BACKWARD_WINDOW: Final = SEQUENCE_MODULUS // 2
"""A sequence this far or further ahead of the expected one, modulo 65536, is read as
behind it: a restart, not a gap of 32768 or more frames."""


@dataclass(frozen=True)
class LinkStatistics:
    """Downlink counts over a span of the run: the total, or one pass.

    Attributes:
        received_count: Packets delivered to the station, whatever became of them.
        rejected_count: Packets rejected, every reason together.
        rejected_by_reason: Rejected packets per reason; reasons not seen are absent.
        missing_count: Downlink frames never received, from sequence gaps.
        restart_count: Downlink sequence restarts (reboots, ADR-0009 §7).
    """

    received_count: int = 0
    rejected_count: int = 0
    rejected_by_reason: Mapping[RejectionReason, int] = field(
        default_factory=lambda: MappingProxyType({})
    )
    missing_count: int = 0
    restart_count: int = 0


class _Counts:
    """Mutable counters behind one :class:`LinkStatistics` snapshot."""

    def __init__(self) -> None:
        self.received_count = 0
        self.rejected: dict[RejectionReason, int] = {}
        self.missing_count = 0
        self.restart_count = 0

    def reject(self, reason: RejectionReason) -> None:
        self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def snapshot(self) -> LinkStatistics:
        # Reasons in enum order, so equal statistics compare and print identically.
        by_reason = {r: self.rejected[r] for r in RejectionReason if r in self.rejected}
        return LinkStatistics(
            received_count=self.received_count,
            rejected_count=sum(by_reason.values()),
            rejected_by_reason=MappingProxyType(by_reason),
            missing_count=self.missing_count,
            restart_count=self.restart_count,
        )


@dataclass(frozen=True)
class SequenceResult:
    """What :meth:`SequenceTracker.observe` concluded about one frame's sequence.

    Attributes:
        missing_count: Frames missing before this one (0 if none).
        restarted: The counter restarted (ADR-0009 §7).
    """

    missing_count: int = 0
    restarted: bool = False


class SequenceTracker:
    """Tracks the downlink header sequence shared by every frame type (ADR-0009 §7).

    The spacecraft numbers every frame it commits to the transmitter from one counter
    that starts at 0 at power-on and after every reboot and wraps ``0xFFFF`` → ``0``.
    For a received sequence ``s`` and the expected next one ``e``, with
    ``d = (s - e) mod 65536``:

    - **First frame** after a reset of the station: accepted as is; nothing is missing
      (the station cannot know what came before it was listening).
    - ``d == 0``: in order. This includes the wrap, ``0xFFFF`` followed by ``0``.
    - ``s == 0`` and ``d != 0``: a **restart**, the spacecraft rebooted. Frames lost
      before the reboot are not counted: the counter restarted, so their number is
      unknowable. (A wrap that lost exactly the frames up to ``0xFFFF`` looks the same;
      it is read as a restart.)
    - ``0 < d < 32768``: a **gap** of ``d`` frames, wrap included.
    - ``d >= 32768``: the sequence went **backwards**, so the counter restarted and the
      frames ``0 .. s - 1`` of the new count were lost: a restart with ``s`` missing.

    Whether a restart was a reboot is confirmed from the content, not the header: the
    boot count in telemetry goes up (see :class:`DownlinkDecoder`).
    """

    def __init__(self) -> None:
        self._expected: int | None = None

    def observe(self, sequence: int) -> SequenceResult:
        """Record a received downlink sequence and classify it.

        Args:
            sequence: The frame's header sequence, ``0..0xFFFF``.

        Returns:
            How many frames are missing before it, and whether the counter restarted.
        """
        expected = self._expected
        self._expected = (sequence + 1) % SEQUENCE_MODULUS
        if expected is None:
            return SequenceResult()
        delta = (sequence - expected) % SEQUENCE_MODULUS
        if delta == 0:
            return SequenceResult()
        if sequence == 0:
            return SequenceResult(restarted=True)
        if delta < BACKWARD_WINDOW:
            return SequenceResult(missing_count=delta)
        return SequenceResult(missing_count=sequence, restarted=True)


_FRAME_ERROR_REASONS: Final[tuple[tuple[type[FrameError], RejectionReason], ...]] = (
    (FrameSyncError, RejectionReason.SYNC),
    (FrameLengthError, RejectionReason.LENGTH),
    (FrameCrcError, RejectionReason.CRC),
)
"""Frame errors other than the type error, by rejection reason."""


def _frame_error_reason(error: FrameError) -> RejectionReason:
    for error_type, reason in _FRAME_ERROR_REASONS:
        if isinstance(error, error_type):
            return reason
    return RejectionReason.TYPE


AckHandler = Callable[[int, CommandAck], StationEvent]
"""Correlates one decoded ACK or NACK at a simulated time and returns its event (#155)."""


class DownlinkDecoder:
    """Decodes downlink packets into events and keeps the link statistics (#156).

    Args:
        telemetry: The telemetry store decoded telemetry is kept in (#157); ``None``
            keeps none.
        ack_handler: Receives every decoded ACK/NACK (#155); the station passes its
            command tracker's ``answer``. ``None`` decodes ACKs and emits nothing for them.
        data: Where DATA chunks go (#158). Default: a new store at the default chunk
            size.
    """

    def __init__(
        self,
        telemetry: TelemetryStore | None = None,
        ack_handler: AckHandler | None = None,
        *,
        data: DataStore | None = None,
    ) -> None:
        self._telemetry_store = telemetry
        self._ack_handler = ack_handler
        self._data = DataStore(DEFAULT_DATA_CHUNK_SIZE_BYTES) if data is None else data
        self._pass_id = 0
        self._sequence = SequenceTracker()
        self._total = _Counts()
        self._pass = _Counts()
        self._boot_count: int | None = None

    @property
    def statistics(self) -> LinkStatistics:
        """Downlink statistics since the station's last reset."""
        return self._total.snapshot()

    @property
    def pass_statistics(self) -> LinkStatistics:
        """Downlink statistics since the current pass began (:meth:`start_pass`)."""
        return self._pass.snapshot()

    def start_pass(self) -> None:
        """Begin a new pass: the per-pass statistics start again from zero.

        Called by the pass state machine at AOS (#161). Until #161 nothing calls it, so
        the whole run is one pass and :attr:`pass_statistics` equals :attr:`statistics`.
        The DATA store's pass begins too.
        """
        self._pass = _Counts()
        self._data.start_pass()
        self._pass_id += 1

    @property
    def pass_id(self) -> int:
        """The current pass: 0 until the first :meth:`start_pass`, then one more at each.
        Recorded with each telemetry record (#157)."""
        return self._pass_id

    def decode(self, now_us: int, packet: Packet) -> list[StationEvent]:
        """Decode one packet and return its events, frame-level first.

        Never raises for any frame bytes.

        Args:
            now_us: Simulated time stamped on the events.
            packet: The packet the link delivered.

        Returns:
            The packet's events, in order.
        """
        self._count_received()
        try:
            frame = decode_frame(packet.frame)
        except FrameError as error:
            return [self._reject(now_us, _frame_error_reason(error), packet.frame)]
        if frame.frame_type is FrameType.COMMAND:
            return [self._reject(now_us, RejectionReason.WRONG_DIRECTION, packet.frame)]

        events: list[StationEvent] = []
        result = self._sequence.observe(frame.sequence)
        if result.restarted:
            self._total.restart_count += 1
            self._pass.restart_count += 1
        if result.missing_count:
            self._total.missing_count += result.missing_count
            self._pass.missing_count += result.missing_count
            events.append(SequenceGap(now_us, frame.sequence, result.missing_count))
        events.extend(self._content(now_us, frame, packet.frame, packet.snr_db))
        return events

    def _content(
        self, now_us: int, frame: Frame, raw: bytes, snr_db: float | None
    ) -> list[StationEvent]:
        """Decode a valid downlink frame's payload by type (step 3 of the module doc).

        Telemetry and DATA events carry the packet's ``snr_db`` (#163).
        """
        try:
            if frame.frame_type is FrameType.TELEMETRY:
                telemetry = decode_telemetry(frame.payload)
                return self._telemetry(now_us, frame.sequence, telemetry, snr_db)
            if frame.frame_type is FrameType.ACK:
                return self.on_ack(now_us, frame.sequence, decode_ack(frame.payload))
            chunk = decode_data(frame.payload)
            return [self._data.receive(now_us, frame.sequence, chunk, snr_db=snr_db)]
        except (TelemetryDecodeError, AckDecodeError, DataDecodeError):
            return [self._reject(now_us, RejectionReason.PAYLOAD, raw)]

    def _telemetry(
        self, now_us: int, sequence: int, telemetry: Telemetry, snr_db: float | None
    ) -> list[StationEvent]:
        """A TELEMETRY event, then a reboot event if the boot count went up."""
        events: list[StationEvent] = [TelemetryReceived(now_us, sequence, telemetry, snr_db)]
        if self._telemetry_store is not None:
            self._telemetry_store.append(
                TelemetryRecord(now_us, sequence, self._pass_id, telemetry)
            )
        self.on_telemetry(now_us, telemetry)
        previous, self._boot_count = self._boot_count, telemetry.boot_count
        if previous is not None and telemetry.boot_count > previous:
            events.append(SpacecraftRebooted(now_us, sequence, telemetry.boot_count))
        return events

    def on_telemetry(self, now_us: int, telemetry: Telemetry) -> None:
        """Hook for decoded telemetry, after it is stored in the telemetry store (#157).

        A no-op: an extension point for tests and later tickets.
        """

    def on_ack(self, now_us: int, sequence: int, ack: CommandAck) -> list[StationEvent]:
        """A decoded ACK or NACK, correlated with its command by the ACK handler (#155).

        Returns its ``CommandAcked``, ``CommandNacked`` or ``UnmatchedAck`` event, or
        nothing without a handler. ``sequence`` is the ACK frame's own downlink
        sequence; ``ack.sequence`` names the uplink frame it answers.
        """
        if self._ack_handler is None:
            return []
        return [self._ack_handler(now_us, ack)]

    def _count_received(self) -> None:
        self._total.received_count += 1
        self._pass.received_count += 1

    def _reject(self, now_us: int, reason: RejectionReason, raw: bytes) -> FrameRejected:
        self._total.reject(reason)
        self._pass.reject(reason)
        return FrameRejected(now_us, reason, bytes(raw))
