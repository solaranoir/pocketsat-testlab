"""Command queue, uplink sequencing and encoding (#154, ADR-0010 §2).

:class:`CommandQueue` holds submitted commands until they are due and the radio may
transmit, then encodes them into COMMAND frames (``encode_command``, ADR-0009 §3) with
the station's uplink sequence counter. :class:`~pocketsat.groundstation.GroundStation`
owns one and calls it from ``submit`` and ``begin_tick``.

- **Order.** Due commands go out oldest due time first, then in submission order.
  A command that is not yet due never holds back one that is.
- **Gate.** Commands are sent only while the transmit gate is open. Until radio control
  (#164) supplies one, the gate is :func:`always_open`. While it is closed, commands stay
  queued.
- **Expiry.** With ``StationConfig.command_expiry_us`` set, a command still queued that
  long after its due time is dropped and reported as
  :class:`~pocketsat.groundstation.CommandFailed` with
  :attr:`~pocketsat.groundstation.FailureReason.EXPIRED`. Expiry is checked whether or
  not the gate is open, so a command queued through a whole outage expires on time.
- **Per-tick limit.** At most ``StationConfig.max_uplink_command_count`` commands go out
  in one tick; the rest wait for the next.
- **Sequence.** Every COMMAND frame takes the next uplink sequence number, starting at 0
  after each reset and wrapping ``0xFFFF`` → ``0x0000`` like the downlink counter
  (``docs/protocol.md``). Retries (#155) will take new numbers the same way.
"""

from collections.abc import Callable
from dataclasses import dataclass

from pocketsat.core.clock import check_us
from pocketsat.frame import MAX_SEQUENCE, FrameType, encode_frame_fields
from pocketsat.groundstation.events import (
    CommandFailed,
    CommandRef,
    CommandSent,
    FailureReason,
    StationEvent,
)
from pocketsat.messages import Command, Packet, encode_command

TransmitGate = Callable[[int], bool]
"""Decides whether the station may transmit at a simulated time (``now_us``).

It must be a pure function of its input and the run's state, never of wall-clock time,
so runs stay deterministic (ADR-0003). Radio control (#164) replaces the default."""


def always_open(now_us: int) -> bool:
    """The default transmit gate: always open, until radio and pass state exist (#164)."""
    return True


@dataclass(frozen=True)
class QueuedCommand:
    """A submitted command waiting to be sent.

    Attributes:
        command_ref: The command's reference.
        command: The command.
        due_us: Simulated time from which it may be sent.
    """

    command_ref: CommandRef
    command: Command
    due_us: int


class CommandQueue:
    """Queued commands and the uplink sequence counter.

    Args:
        max_uplink_command_count: Most COMMAND frames sent in one tick. Positive.
        command_expiry_us: How long after its due time a queued command is dropped, or
            ``None`` to keep it until it is sent.

    Raises:
        TypeError: An argument has the wrong type.
        ValueError: ``max_uplink_command_count`` is not positive, or
            ``command_expiry_us`` is negative.
    """

    def __init__(self, max_uplink_command_count: int, command_expiry_us: int | None) -> None:
        if isinstance(max_uplink_command_count, bool) or not isinstance(
            max_uplink_command_count, int
        ):
            raise TypeError(
                f"max_uplink_command_count must be an int, got {max_uplink_command_count!r}"
            )
        if max_uplink_command_count <= 0:
            raise ValueError(
                f"max_uplink_command_count must be positive, got {max_uplink_command_count}"
            )
        if command_expiry_us is not None:
            check_us("command_expiry_us", command_expiry_us)
        self._limit = max_uplink_command_count
        self._expiry_us = command_expiry_us
        self._queued: list[QueuedCommand] = []
        self._next_sequence = 0

    @property
    def queued(self) -> tuple[QueuedCommand, ...]:
        """The commands waiting, in the order they will be considered for sending."""
        return tuple(self._ordered())

    @property
    def next_sequence(self) -> int:
        """The uplink sequence the next COMMAND frame will carry."""
        return self._next_sequence

    def reset(self) -> None:
        """Empty the queue and restart the uplink sequence at 0."""
        self._queued = []
        self._next_sequence = 0

    def add(self, command_ref: CommandRef, command: Command, due_us: int) -> None:
        """Queue a command, to be sent from ``due_us``.

        Raises:
            TypeError: ``due_us`` is not an int.
            ValueError: ``due_us`` is negative.
        """
        check_us("due_us", due_us)
        self._queued.append(QueuedCommand(command_ref, command, due_us))

    def transmit(
        self, now_us: int, gate_open: bool
    ) -> tuple[tuple[Packet, ...], tuple[StationEvent, ...]]:
        """Expire stale commands, then encode due ones while the gate is open.

        Args:
            now_us: Simulated time at the start of the tick.
            gate_open: Whether the station may transmit now.

        Returns:
            The uplink packets in transmission order, and the events: expiries first
            (in queue order), then one ``CommandSent`` per packet.
        """
        events: list[StationEvent] = []
        kept: list[QueuedCommand] = []
        for queued in self._ordered():
            if self._expiry_us is not None and now_us >= queued.due_us + self._expiry_us:
                events.append(CommandFailed(now_us, queued.command_ref, FailureReason.EXPIRED))
            else:
                kept.append(queued)

        packets: list[Packet] = []
        remaining: list[QueuedCommand] = []
        for queued in kept:
            if gate_open and queued.due_us <= now_us and len(packets) < self._limit:
                sequence = self._take_sequence()
                payload = encode_command(queued.command)
                packets.append(
                    Packet(frame=encode_frame_fields(FrameType.COMMAND, sequence, payload))
                )
                events.append(
                    CommandSent(
                        now_us,
                        queued.command_ref,
                        queued.command.command_id,
                        sequence,
                        retry_count=0,
                    )
                )
            else:
                remaining.append(queued)
        self._queued = remaining
        return tuple(packets), tuple(events)

    def _ordered(self) -> list[QueuedCommand]:
        """The queue by due time, then submission order (``CommandRef`` order)."""
        return sorted(self._queued, key=lambda q: (q.due_us, q.command_ref.ref_id))

    def _take_sequence(self) -> int:
        sequence = self._next_sequence
        self._next_sequence = 0 if sequence == MAX_SEQUENCE else sequence + 1
        return sequence
