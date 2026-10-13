"""The ground station (ADR-0010 §1, §2).

:class:`GroundStation` is a pure component: it never calls the target, the link or a
clock. Each tick the run loop calls it twice, matching the tick order of
``docs/architecture.md`` §8: :meth:`~GroundStation.begin_tick` before the target
advances (steps 3 and 4: pass state, timeouts, uplink) and :meth:`~GroundStation.end_tick`
after it (steps 8 and 9: decoding). Commands are queued with
:meth:`~GroundStation.submit` at step 3.

This module is the framework (#152). The behaviour arrives ticket by ticket, and each
stub below names its ticket: uplink encoding (#154), ACK/NACK correlation, timeouts and
retries (#155), downlink decoding (#156), the telemetry store (#157), DATA (#158), pass
state (#161) and radio control (#164).
"""

from collections.abc import Sequence
from dataclasses import dataclass

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.groundstation.config import DEFAULT_STATION_CONFIG, StationConfig
from pocketsat.groundstation.events import CommandRef, StationEvent
from pocketsat.groundstation.telemetry import TelemetryStore
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

    Raises:
        TypeError: ``config`` is not a :class:`StationConfig`.
    """

    def __init__(self, config: StationConfig = DEFAULT_STATION_CONFIG) -> None:
        if not isinstance(config, StationConfig):
            raise TypeError(f"config must be a StationConfig, got {config!r}")
        self._config = config
        self._clear()

    def _clear(self) -> None:
        """Set every piece of run state to its initial value."""
        self._next_ref_id = 0
        self._queue: list[tuple[CommandRef, Command]] = []
        self._now_us = 0
        self._telemetry = TelemetryStore()

    @property
    def config(self) -> StationConfig:
        """The configuration the station was built with."""
        return self._config

    @property
    def telemetry(self) -> TelemetryStore:
        """The decoded telemetry (#157). Empty until #157."""
        return self._telemetry

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

    def submit(self, command: Command) -> CommandRef:
        """Queue a command for uplink (step 3). Nothing is encoded until :meth:`begin_tick`.

        Args:
            command: The command to send.

        Returns:
            The reference that identifies the command in every later event.

        Raises:
            TypeError: ``command`` is not a :class:`Command`.
        """
        if not isinstance(command, Command):
            raise TypeError(f"command must be a Command, got {command!r}")
        ref = CommandRef(self._next_ref_id)
        self._next_ref_id += 1
        self._queue.append((ref, command))
        return ref

    def begin_tick(self, now_us: int, geometry: PassGeometry | None) -> Uplink:
        """Start a tick (steps 3 and 4): pass state, timeouts and retries, then uplink.

        Stubs until their tickets land: pass state from ``geometry`` (#161), timeouts
        and retries (#155), and encoding queued commands into COMMAND frames while the
        radio may transmit (#154, #164). Until then queued commands stay queued and the
        result is empty.

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
        return Uplink()

    def end_tick(self, now_us: int, downlink: Sequence[Packet]) -> tuple[StationEvent, ...]:
        """End a tick (steps 8 and 9): decode the downlink, in arrival order.

        A stub until #156 (decoding), #155 (ACK correlation), #157 (telemetry store)
        and #158 (DATA): packets are accepted and nothing is decoded.

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
        return ()

    def _advance_to(self, now_us: int) -> None:
        """Validate and record the simulated time of a call. Time never goes back."""
        check_us("now_us", now_us)
        if now_us < self._now_us:
            raise ValueError(f"now_us went back from {self._now_us} to {now_us}")
        self._now_us = now_us
