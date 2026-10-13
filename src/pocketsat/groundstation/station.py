"""The ground station (ADR-0010 §1, §2).

:class:`GroundStation` is a pure component: it never calls the target, the link or a
clock. Each tick the run loop calls it twice, matching the tick order of
``docs/architecture.md`` §8: :meth:`~GroundStation.begin_tick` before the target
advances (steps 3 and 4: pass state, timeouts, uplink) and :meth:`~GroundStation.end_tick`
after it (steps 8 and 9: decoding). Commands are queued with
:meth:`~GroundStation.submit` at step 3.

This module is the framework (#152). The behaviour arrives ticket by ticket, and each
stub below names its ticket: uplink encoding (#154), ACK/NACK correlation, timeouts and
retries (#155), DATA (#158), pass
state (#161) and radio control (#164).
"""

from collections.abc import Sequence
from dataclasses import dataclass

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.groundstation.config import DEFAULT_STATION_CONFIG, StationConfig
from pocketsat.groundstation.downlink import DownlinkDecoder, LinkStatistics
from pocketsat.groundstation.events import CommandRef, StationEvent
from pocketsat.groundstation.telemetry import TelemetryStore
from pocketsat.groundstation.uplink import CommandQueue, TransmitGate, always_open
from pocketsat.messages import Command, Packet


@dataclass(frozen=True)
class Uplink:
    """What :meth:`GroundStation.begin_tick` returns.

    Attributes:
        packets: Uplink packets for the link, in transmission order.
        events: Events from the start of the tick (AOS/LOS, timeouts, commands sent).
    """

    packets: tuple[Packet, ...] = ()
    events: tuple[StationEvent, ...] = ()


class GroundStation:
    """An amateur-style ground station: messages in, messages out (ADR-0010 §1).

    Built ready to run, in the state :meth:`reset` returns to.

    Args:
        config: The station's configuration (ADR-0010 §3).
        transmit_gate: Whether the station may transmit at a simulated time (#154).
            Defaults to :func:`~pocketsat.groundstation.uplink.always_open` until radio
            control (#164) supplies one. Must be deterministic (ADR-0003).

    Raises:
        TypeError: ``config`` is not a :class:`StationConfig`, or ``transmit_gate`` is
            not callable.
    """

    def __init__(
        self,
        config: StationConfig = DEFAULT_STATION_CONFIG,
        *,
        transmit_gate: TransmitGate = always_open,
    ) -> None:
        if not isinstance(config, StationConfig):
            raise TypeError(f"config must be a StationConfig, got {config!r}")
        if not callable(transmit_gate):
            raise TypeError(f"transmit_gate must be callable, got {transmit_gate!r}")
        self._config = config
        self._transmit_gate = transmit_gate
        self._clear()

    def _clear(self) -> None:
        """Set every piece of run state to its initial value."""
        self._next_ref_id = 0
        self._uplink = CommandQueue(
            self._config.max_uplink_command_count, self._config.command_expiry_us
        )
        self._now_us = 0
        self._telemetry = TelemetryStore(self._config.telemetry_retention_count)
        self._downlink = DownlinkDecoder(self._telemetry)

    @property
    def config(self) -> StationConfig:
        """The configuration the station was built with."""
        return self._config

    @property
    def telemetry(self) -> TelemetryStore:
        """The decoded telemetry, bounded by ``config.telemetry_retention_count`` (#157).

        Read-only to callers: the downlink decoder fills it in :meth:`end_tick`, and
        :meth:`reset` empties it."""
        return self._telemetry

    @property
    def link_statistics(self) -> LinkStatistics:
        """Downlink statistics since the last reset (#156)."""
        return self._downlink.statistics

    @property
    def pass_link_statistics(self) -> LinkStatistics:
        """Downlink statistics for the current pass (#156). The whole run is one pass
        until the pass state machine (#161) starts a new one at each AOS."""
        return self._downlink.pass_statistics

    def reset(self, rngs: RngFactory) -> None:
        """Return to the configured initial state at simulated time 0.

        Args:
            rngs: The run's random streams. The station draws none in Phase 2; Phase 4's
                ground-station faults request streams named ``groundstation.*``.

        Raises:
            TypeError: ``rngs`` is not an :class:`RngFactory`.
        """
        if not isinstance(rngs, RngFactory):
            raise TypeError(f"rngs must be an RngFactory, got {rngs!r}")
        self._clear()

    def submit(self, command: Command, *, due_us: int | None = None) -> CommandRef:
        """Queue a command for uplink (step 3). Nothing is encoded until :meth:`begin_tick`.

        Args:
            command: The command to send.
            due_us: Simulated time from which it may be sent (#154). ``None`` (the
                default) means now: the time of the station's latest
                :meth:`begin_tick` or :meth:`end_tick`, so a command submitted at step 3
                goes out in this tick's :meth:`begin_tick` if the gate is open. A time in
                the past is due immediately.

        Returns:
            The reference that identifies the command in every later event.

        Raises:
            TypeError: ``command`` is not a :class:`Command`, or ``due_us`` is not an
                int or ``None``.
            ValueError: ``due_us`` is negative.
        """
        if not isinstance(command, Command):
            raise TypeError(f"command must be a Command, got {command!r}")
        due = self._now_us if due_us is None else due_us
        check_us("due_us", due)
        ref = CommandRef(self._next_ref_id)
        self._next_ref_id += 1
        self._uplink.add(ref, command, due)
        return ref

    def begin_tick(self, now_us: int, geometry: PassGeometry | None) -> Uplink:
        """Start a tick (steps 3 and 4): pass state, timeouts and retries, then uplink.

        Queued commands that are due are encoded into COMMAND frames while the transmit
        gate is open, at most ``config.max_uplink_command_count`` per tick, and stale
        ones expire (#154, :mod:`pocketsat.groundstation.uplink`). Stubs until their
        tickets land: pass state from ``geometry`` (#161), timeouts and retries (#155),
        and the radio's gate (#164).

        Args:
            now_us: Simulated time at the start of the tick.
            geometry: The tick's pass geometry, the station's pass prediction
                (ADR-0010 §6), or ``None`` when no geometry model runs.

        Returns:
            The uplink packets and the events of the start of the tick.

        Raises:
            TypeError: ``now_us`` is not an int, or ``geometry`` is not a
                :class:`PassGeometry` or ``None``.
            ValueError: ``now_us`` is negative or earlier than the previous call.
        """
        self._advance_to(now_us)
        if geometry is not None and not isinstance(geometry, PassGeometry):
            raise TypeError(f"geometry must be a PassGeometry or None, got {geometry!r}")
        packets, events = self._uplink.transmit(now_us, self._transmit_gate(now_us))
        return Uplink(packets, events)

    def end_tick(self, now_us: int, downlink: Sequence[Packet]) -> tuple[StationEvent, ...]:
        """End a tick (steps 8 and 9): decode the downlink, in arrival order.

        Each packet is decoded by :class:`~pocketsat.groundstation.downlink.DownlinkDecoder`
        (#156): frame-level events first (rejection, sequence gap), then the content
        (telemetry, and a reboot when the boot count went up). Nothing raises for any
        frame bytes. Decoded telemetry is stored in :attr:`telemetry` (#157). ACK
        correlation (#155) and DATA (#158) are hooks in the decoder until their tickets.

        Args:
            now_us: Simulated time at the end of the tick.
            downlink: Packets delivered by the link this tick, oldest first.

        Returns:
            The events, per packet: frame-level events first, then the content.

        Raises:
            TypeError: ``now_us`` is not an int, or an item of ``downlink`` is not a
                :class:`Packet`.
            ValueError: ``now_us`` is negative or earlier than the previous call.
        """
        self._advance_to(now_us)
        for packet in downlink:
            if not isinstance(packet, Packet):
                raise TypeError(f"downlink items must be Packets, got {packet!r}")
        events: list[StationEvent] = []
        for packet in downlink:
            events.extend(self._downlink.decode(now_us, packet))
        return tuple(events)

    def _advance_to(self, now_us: int) -> None:
        """Validate and record the simulated time of a call. Time never goes back."""
        check_us("now_us", now_us)
        if now_us < self._now_us:
            raise ValueError(f"now_us went back from {self._now_us} to {now_us}")
        self._now_us = now_us
