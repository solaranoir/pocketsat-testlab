"""ACK/NACK correlation, timeouts and retries (#155, ADR-0010 §2).

:class:`CommandTracker` closes the loop on every command the station sends. The station
tells it about each attempt (a ``CommandSent`` from the uplink queue, #154), the
downlink decoder hands it every decoded ACK frame (#156), and at the start of each tick
it times out attempts that got no answer.

- **Correlation.** An ACK or NACK answers the attempt whose uplink sequence it names
  (``CommandAck.sequence``, ADR-0009 §4), provided it also echoes that attempt's
  command ID. The first answer to any attempt of a command resolves the command:
  ``CommandAcked``, or ``CommandNacked`` with the reason as a :class:`DecodeReason` or
  :class:`RejectReason` member, so it is reported by name.
- **Late answers.** A command's earlier attempts stay matchable until the command is
  resolved: an ACK for an attempt that already timed out, arriving while a retry is
  outstanding, still answers the command, because the spacecraft did execute it.
- **Unmatched answers.** An ACK naming no attempt of an unresolved command (an unknown
  sequence, a mismatched command ID, or a second answer to a resolved command) is an
  ``UnmatchedAck`` event and counted (:attr:`CommandTracker.unmatched_ack_count`). It
  is never an error: a duplicated frame or an answer after a timeout produces one.
- **Timeouts.** An attempt times out at the first ``begin_tick`` at or after
  ``command_timeout_us`` from the start of the tick it was sent in, unless an answer
  was decoded first (``end_tick`` runs before the next ``begin_tick`` at the same time).
  Only a command's latest attempt can time out.
- **Retries** follow ADR-0010 §2. A timed-out command is sent again, with a new uplink
  sequence and the next ``retry_count``, up to ``command_retry_count`` retries, then it
  fails with ``RETRIES_EXHAUSTED``. RESET is never retried automatically: a retried
  RESET whose first ACK was lost would reboot the spacecraft twice, so it fails with
  ``NOT_RETRYABLE`` after its first timeout. A retry is queued due at the timeout, so it
  goes out in the same ``begin_tick`` if the transmit gate is open; otherwise it waits
  in the queue like any command (and can expire, #154).

The timeout bound against the tick is :func:`max_answer_latency_us`.
"""

from dataclasses import dataclass
from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.groundstation.events import (
    CommandAcked,
    CommandFailed,
    CommandNacked,
    CommandRef,
    CommandTimedOut,
    FailureReason,
    StationEvent,
    UnmatchedAck,
)
from pocketsat.messages import Command, CommandAck, CommandId

NON_RETRYABLE_COMMANDS: Final = frozenset({CommandId.RESET})
"""Commands never retried automatically (ADR-0010 §2). Every other command is
idempotent on the spacecraft (ADR-0009 §4), so repeating it after a lost ACK is safe."""

ACK_FRAME_BYTES: Final = 14
"""Wire size of one ACK/NACK frame: 10 bytes of framing and a 4-byte payload
(``docs/protocol.md``)."""

MAX_FRAME_AHEAD_BYTES: Final = 77
"""Most bytes of a frame still in transmission when a command arrives: the largest
frame (a 78-byte DATA frame with 64-byte chunks) less the byte already sent
(ADR-0008 §5, ``r``)."""

MAX_HELD_ACK_FRAMES: Final = 8
"""Most ACK/NACK frames the flight computer holds ahead of a new one
(``OutboundConfig.max_held_ack_frames``, ADR-0008 §5). An ACK that would be the ninth is
suppressed and its command times out."""

WORST_CASE_ANSWER_BYTES: Final = (
    MAX_FRAME_AHEAD_BYTES + MAX_HELD_ACK_FRAMES * ACK_FRAME_BYTES + ACK_FRAME_BYTES
)
"""Downlink bytes that can go out before an answer's last byte (203): the rest of the
frame in transmission, every held ACK ahead of it, and the answer itself
(ADR-0008 §5, ``r + A + 14``)."""

DEFAULT_DOWNLINK_RATE_BYTES_PER_S: Final = 1200
"""The spacecraft radio's default transmit rate (``CommsConfig.transmit_rate_bytes_per_s``,
``docs/protocol.md``). The station cannot see the target's settings, so a run with
another rate passes it to :func:`max_answer_latency_us`."""


def max_answer_latency_us(
    tick_us: int, downlink_rate_bytes_per_s: int = DEFAULT_DOWNLINK_RATE_BYTES_PER_S
) -> int:
    """The longest a legitimate answer takes, from sending a command to decoding its ACK.

    A command sent in ``begin_tick`` at time ``t`` reaches the flight computer in the
    tick that starts at ``t`` (``SilTarget`` delivers uplink in the next tick it runs),
    and its answer is delivered in the tick its last byte is transmitted (ADR-0008 §5).
    In a whole-frame tick that is the same tick, so the answer is decoded in
    ``end_tick`` at ``t + tick_us``: the latency of ADR-0004's "command → ACK, same
    tick". In streaming ticks (shorter than one frame's airtime) the answer can wait
    behind :data:`WORST_CASE_ANSWER_BYTES` of downlink, and the tick capacity carries
    fractions of a byte (one byte of slack). The latency is that airtime rounded up to
    whole ticks, and never less than one tick.

    A RESET's ACK follows the same rule: its reboot waits for the ACK (ADR-0008 §6a),
    not the other way round, so the deferred reboot adds nothing to the ACK's latency.

    Args:
        tick_us: The run's tick length, simulated microseconds. Positive.
        downlink_rate_bytes_per_s: The spacecraft's transmit rate. Positive.

    Returns:
        The latency bound, simulated microseconds, a whole number of ticks.

    Raises:
        TypeError: An argument is not an int.
        ValueError: An argument is not positive.
    """
    check_us("tick_us", tick_us)
    if tick_us == 0:
        raise ValueError("tick_us must be positive, got 0")
    rate = downlink_rate_bytes_per_s
    if isinstance(rate, bool) or not isinstance(rate, int):
        raise TypeError(f"downlink_rate_bytes_per_s must be an int, got {rate!r}")
    if rate <= 0:
        raise ValueError(f"downlink_rate_bytes_per_s must be positive, got {rate}")
    airtime_us = -(-(WORST_CASE_ANSWER_BYTES + 1) * 1_000_000 // rate)
    ticks = max(1, -(-airtime_us // tick_us))
    return ticks * tick_us


@dataclass(frozen=True)
class Attempt:
    """One transmission of a command, waiting for its answer.

    Attributes:
        command_ref: The command.
        command: The command as submitted (a retry resends it).
        sequence: The attempt's uplink sequence.
        retry_count: Retries before this attempt: 0 for the first.
        sent_us: Simulated time of the ``begin_tick`` that sent it.
    """

    command_ref: CommandRef
    command: Command
    sequence: int
    retry_count: int
    sent_us: int


@dataclass(frozen=True)
class Retry:
    """A command to send again, returned by :meth:`CommandTracker.expire`.

    Attributes:
        command_ref: The command.
        command: The command as submitted.
        retry_count: The new attempt's retry count.
    """

    command_ref: CommandRef
    command: Command
    retry_count: int


class CommandTracker:
    """Outstanding commands: their attempts, answers, timeouts and retries (#155).

    Args:
        command_timeout_us: How long an attempt waits for its answer. Positive.
        command_retry_count: Retries allowed for a retryable command. Non-negative.
    """

    def __init__(self, command_timeout_us: int, command_retry_count: int) -> None:
        check_us("command_timeout_us", command_timeout_us)
        if command_timeout_us == 0:
            raise ValueError("command_timeout_us must be positive, got 0")
        if isinstance(command_retry_count, bool) or not isinstance(command_retry_count, int):
            raise TypeError(f"command_retry_count must be an int, got {command_retry_count!r}")
        if command_retry_count < 0:
            raise ValueError(f"command_retry_count must be non-negative, got {command_retry_count}")
        self._timeout_us = command_timeout_us
        self._retry_limit = command_retry_count
        self._by_sequence: dict[int, Attempt] = {}
        self._latest: dict[CommandRef, Attempt] = {}
        self._unmatched_ack_count = 0

    @property
    def outstanding(self) -> tuple[CommandRef, ...]:
        """Commands sent and not yet resolved, in reference order."""
        return tuple(sorted(self._latest, key=lambda ref: ref.ref_id))

    @property
    def unmatched_ack_count(self) -> int:
        """ACK/NACK frames that answered no outstanding attempt, since the last reset."""
        return self._unmatched_ack_count

    def sent(self, attempt: Attempt) -> None:
        """Record an attempt handed to the link."""
        # After 65536 sends the sequence wraps onto any attempt still matchable; the new
        # attempt replaces it, since an answer can no longer tell the two apart.
        self._by_sequence[attempt.sequence] = attempt
        self._latest[attempt.command_ref] = attempt

    def answer(self, now_us: int, ack: CommandAck) -> StationEvent:
        """Correlate one decoded ACK or NACK and return its event."""
        attempt = self._by_sequence.get(ack.sequence)
        if attempt is None or attempt.command.command_id != ack.command_id:
            self._unmatched_ack_count += 1
            return UnmatchedAck(now_us, ack.sequence, ack.command_id)
        self._resolve(attempt.command_ref)
        if ack.reason is None:
            return CommandAcked(now_us, attempt.command_ref, ack.sequence)
        return CommandNacked(now_us, attempt.command_ref, ack.sequence, ack.reason)

    def expire(self, now_us: int) -> tuple[tuple[StationEvent, ...], tuple[Retry, ...]]:
        """Time out every latest attempt sent ``command_timeout_us`` or more ago.

        Returns:
            The events, in command reference order (each ``CommandTimedOut``, followed
            by ``CommandFailed`` when no retry follows), and the commands to resend.
        """
        if not self._latest:  # nothing outstanding: the usual tick, kept cheap (#167)
            return (), ()
        events: list[StationEvent] = []
        retries: list[Retry] = []
        for ref in self.outstanding:
            attempt = self._latest[ref]
            if now_us - attempt.sent_us < self._timeout_us:
                continue
            retryable = attempt.command.command_id not in NON_RETRYABLE_COMMANDS
            will_retry = retryable and attempt.retry_count < self._retry_limit
            events.append(CommandTimedOut(now_us, ref, attempt.retry_count, will_retry))
            if will_retry:
                # Keep this attempt matchable for a late answer; the retry replaces it as
                # the latest attempt when it is sent.
                retries.append(Retry(ref, attempt.command, attempt.retry_count + 1))
                del self._latest[ref]
            else:
                reason = (
                    FailureReason.RETRIES_EXHAUSTED if retryable else FailureReason.NOT_RETRYABLE
                )
                events.append(CommandFailed(now_us, ref, reason))
                self._resolve(ref)
        return tuple(events), tuple(retries)

    def discard(self, ref: CommandRef) -> None:
        """Forget a command resolved elsewhere: a retry that expired in the queue (#154)."""
        self._resolve(ref)

    def _resolve(self, ref: CommandRef) -> None:
        """Forget every attempt of a resolved command: later answers are unmatched."""
        self._latest.pop(ref, None)
        for sequence in [s for s, a in self._by_sequence.items() if a.command_ref == ref]:
            del self._by_sequence[sequence]
