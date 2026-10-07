"""The DOWNLINK session: moving payload chunks into DATA frames (#56).

The payload owns stored data, the flight computer moves it, and comms only provides
transmit capacity (ADR-0004 §13). This module holds the session's settings, its state,
and its rules as pure functions; :class:`~pocketsat.flight.FlightComputer` calls them
from its phases. The whole state machine, in ``docs/spacecraft-modes.md`` ("Downlink
session"):

- **Start.** When the mode becomes DOWNLINK (update-mode phase, ADR-0004 §2 step d), a
  session starts at the payload's oldest unreleased chunk
  (``PayloadReadings.oldest_unreleased_chunk_id``), so an interrupted transfer resumes
  where it stopped and nothing is skipped.
- **Per tick** (emit-telemetry phase, step e, after ACK/NACK and telemetry): unless a
  radio-transmit inhibit holds (:func:`transmit_inhibited`), the session sends the
  oldest unsent chunks, one DATA frame each, in ID order, while their frames fit what is
  left of comms' transmit capacity. It stops at the first chunk that does not fit, so
  chunks are never sent out of order.
- **Release** (produce-controls phase, step f): :func:`release_through_chunk_id` gives
  ``PayloadControls.release_through_chunk_id`` for the next tick. Phase 1 rule: the last
  chunk sent, so the payload deletes every sent chunk in the next tick (ADR-0004 §13).
- **Complete** (evaluate-flags phase, step d): in a step that starts in DOWNLINK, when no
  unsent whole chunk remains (:func:`unsent_chunk_ids` is empty), ``DOWNLINK_COMPLETE``
  is raised and the mode returns to the one DOWNLINK was entered from (#47).
- **Abort.** Any other exit from DOWNLINK (SET_MODE, ENTER_SAFE_MODE, SAFE_CONDITION,
  FAULT_DETECTED, RESET, ``forced_reset``) ends the session at once. Chunks sent before
  were already released (the release always reaches the payload in the tick after the
  send), and every other chunk stays in the payload buffer: nothing is lost.

**Hook for acknowledgement-based release (later phases).** :class:`DownlinkSession`
keeps what the session has sent (where it started, the next chunk to send, how many it
sent, and the last one), separately from what the payload has released. Releasing only
once the ground acknowledges, and retransmitting what it doesn't, then means changing
:func:`release_through_chunk_id` and the session's next-chunk rule, nothing else.

Pure and deterministic: no randomness, no wall-clock time, integer arithmetic only.
Decisions read **readings** only (ADR-0004 §6, §7). Imports nothing from
``pocketsat.messages`` (see the import rule in :mod:`pocketsat.flight`).
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING, Final, Self

from pocketsat.spacecraft.config import MAX_CHUNK_SIZE_BYTES, PayloadConfig
from pocketsat.spacecraft.snapshots import PayloadReadings

if TYPE_CHECKING:
    from pocketsat.flight.computer import SpacecraftReadings

__all__ = [
    "DEFAULT_DOWNLINK_CONFIG",
    "DownlinkConfig",
    "DownlinkSession",
    "release_through_chunk_id",
    "transmit_inhibited",
    "unsent_chunk_ids",
]


@dataclass(frozen=True)
class DownlinkConfig:
    """Settings of the downlink session (#56).

    Attributes:
        chunk_size_bytes: Size of every payload chunk, bytes: the content length of each
            DATA frame. It must equal the payload's ``PayloadConfig.chunk_size_bytes``
            (``SilTarget`` checks this at ``reset``), as flight software is built for
            its payload. Default 64, the payload's default.

    Raises:
        TypeError: ``chunk_size_bytes`` is not an int.
        ValueError: ``chunk_size_bytes`` is not in 1..``MAX_CHUNK_SIZE_BYTES``.
    """

    chunk_size_bytes: int = PayloadConfig().chunk_size_bytes

    def __post_init__(self) -> None:
        size = self.chunk_size_bytes
        if isinstance(size, bool) or not isinstance(size, int):
            raise TypeError(f"chunk_size_bytes must be an int, got {size!r}")
        if not 1 <= size <= MAX_CHUNK_SIZE_BYTES:
            raise ValueError(f"chunk_size_bytes must be in 1..{MAX_CHUNK_SIZE_BYTES}, got {size}")


DEFAULT_DOWNLINK_CONFIG: Final = DownlinkConfig()
"""The default settings: 64-byte chunks, matching the default payload."""


@dataclass(frozen=True)
class DownlinkSession:
    """One DOWNLINK session's state: transient flight computer state (#56).

    Exists while the mode is DOWNLINK and only then; a reboot clears it with the rest of
    the transient state. Records what was **sent**, independently of what the payload
    has released, so acknowledgement-based release can be added later by changing only
    the release rule (see the module docstring).

    Attributes:
        start_chunk_id: The payload's oldest unreleased chunk when the session started:
            where it resumed.
        next_chunk_id: The next chunk to send. Every chunk from ``start_chunk_id`` up to
            here has been sent once, in order.
        sent_chunk_count: DATA frames sent in this session.
        last_sent_chunk_id: The last chunk sent, or ``None`` before the first.

    Raises:
        TypeError: A field is not an int (or ``None`` for ``last_sent_chunk_id``).
        ValueError: The fields contradict each other.
    """

    start_chunk_id: int
    next_chunk_id: int
    sent_chunk_count: int = 0
    last_sent_chunk_id: int | None = None

    def __post_init__(self) -> None:
        for name in ("start_chunk_id", "next_chunk_id", "sent_chunk_count"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an int, got {value!r}")
            if value < 0:
                raise ValueError(f"{name} must be non-negative, got {value}")
        last = self.last_sent_chunk_id
        if last is not None and (isinstance(last, bool) or not isinstance(last, int)):
            raise TypeError(f"last_sent_chunk_id must be an int or None, got {last!r}")
        if self.next_chunk_id < self.start_chunk_id:
            raise ValueError(f"next_chunk_id is before start_chunk_id: {self!r}")
        if (last is None) != (self.sent_chunk_count == 0) or (
            last is not None and last != self.next_chunk_id - 1
        ):
            raise ValueError(f"last_sent_chunk_id does not match the chunks sent: {self!r}")

    @classmethod
    def start(cls, payload: PayloadReadings) -> Self:
        """Start a session at the payload's oldest unreleased chunk.

        Args:
            payload: This tick's payload readings.

        Returns:
            A session that has sent nothing yet.
        """
        oldest = payload.oldest_unreleased_chunk_id
        return cls(start_chunk_id=oldest, next_chunk_id=oldest)

    def after_sending(self, first_chunk_id: int, last_chunk_id: int) -> Self:
        """The session after sending chunks ``first_chunk_id..last_chunk_id`` in order.

        Args:
            first_chunk_id: The first chunk sent this tick, the start of
                :func:`unsent_chunk_ids`.
            last_chunk_id: The last chunk sent this tick.

        Returns:
            The updated session.
        """
        return type(self)(
            start_chunk_id=self.start_chunk_id,
            next_chunk_id=last_chunk_id + 1,
            sent_chunk_count=self.sent_chunk_count + last_chunk_id - first_chunk_id + 1,
            last_sent_chunk_id=last_chunk_id,
        )


def unsent_chunk_ids(session: DownlinkSession, payload: PayloadReadings) -> range:
    """The chunks the session has still to send, oldest first.

    From the session's next chunk (or the payload's oldest unreleased chunk, if that is
    later) up to the payload's newest whole chunk. The partial chunk still accumulating
    is not a chunk yet and is never sent.

    Args:
        session: The current session.
        payload: This tick's payload readings.

    Returns:
        The chunk IDs, possibly empty.
    """
    start = max(session.next_chunk_id, payload.oldest_unreleased_chunk_id)
    return range(start, max(start, payload.next_chunk_id))


def transmit_inhibited(readings: "SpacecraftReadings") -> bool:
    """Whether the radio-transmit inhibits stop DATA this tick (story #42, #56).

    DATA is sent only while no power or thermal flag is set in this tick's **readings**
    (ADR-0004 §6): ``low_battery``, ``critical_battery``, ``over_temp``, or
    ``under_temp``, the same power and thermal flags that inhibit payload acquisition
    (#43). Comms reads nothing, so the inhibit lives here. It stops DATA only: ACK/NACK
    and telemetry still go out, so the ground sees why.

    Args:
        readings: This tick's readings.

    Returns:
        True if no DATA may be sent this tick.
    """
    power, thermal = readings.power, readings.thermal
    return power.low_battery or power.critical_battery or thermal.over_temp or thermal.under_temp


def release_through_chunk_id(session: DownlinkSession | None) -> int | None:
    """The release rule: ``PayloadControls.release_through_chunk_id`` for the next tick.

    Phase 1 releases a chunk once it is sent (ADR-0004 §13): the session's last sent
    chunk, so the payload deletes everything sent so far in the next tick, or ``None``
    when there is no session or nothing was sent. Releasing a chunk already released is
    a no-op for the payload, so the value may repeat from tick to tick. Acknowledgement-
    based release (a later phase) changes only this rule.

    Args:
        session: The current session, or ``None`` outside DOWNLINK.

    Returns:
        The chunk to release through, or ``None``.
    """
    if session is None:
        return None
    return session.last_sent_chunk_id
