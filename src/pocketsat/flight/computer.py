"""The flight computer component and its single ``step()`` entry point (#101).

:class:`FlightComputer` is the one place the flight software plugs into. Each tick,
after every subsystem has stepped, ``SilTarget`` (#59) calls :meth:`FlightComputer.step`
once with the uplink frames received that tick, the subsystems' **readings** (never their
truth, ADR-0004 §6, §7), and the simulated time. ``step()`` runs six fixed phases, which
are ADR-0004 §2 steps c to f:

1. ``decode_uplink`` (step c): decode COMMAND frames with #52's codec (#51).
2. ``execute_commands`` (step c): dispatch commands and queue ACK/NACK (#51). RESET
   only raises its event here; the reboot happens in ``update_mode`` (#49).
3. ``evaluate_flags`` (step d): raise SAFE_CONDITION, FAULT_DETECTED, and BOOT_COMPLETE.
   Filled by #48 and #49.
4. ``update_mode`` (step d): apply the mode events with #47's ``transition()``; an
   applied RESET reboots (#49).
5. ``emit_telemetry`` (step e): telemetry when due, then DATA in DOWNLINK. Filled by
   #55 and #56.
6. ``produce_controls`` (step f): the next tick's controls from #47's
   ``controls_for_mode()``, the single controls function (ADR-0004 §14). #56 adds the
   chunk release.

``decode_uplink`` and ``execute_commands`` are #51's command dispatcher;
``evaluate_flags`` applies #48's safe-mode and fault rules
(:mod:`pocketsat.flight.safety`) and #49's boot timing (:mod:`pocketsat.flight.boot`);
``update_mode`` and ``produce_controls`` call #47's
:func:`~pocketsat.flight.modes.transition` and
:func:`~pocketsat.flight.modes.controls_for_mode`. ``emit_telemetry`` is a documented
no-op until #55 and #56.

**Outbound frames (ADR-0004 §10, ADR-0007 §4).** Every downlink frame goes through
:meth:`FlightComputer._queue_outbound`, which gives it the next downlink sequence number
and appends it to the tick's frames only if it fits what is left of comms'
``transmit_capacity_bytes`` for this tick (read from the readings). Frames are offered
in :class:`OutboundClass` order, ACK/NACK (step c), then telemetry and DATA (step e), so
the order of the phases is the priority order; offering a higher class after a lower one
is a programming error and raises. A frame that does not fit is not queued; ACK/NACK
and telemetry that don't fit are counted in ``outbound_suppressed_count``, DATA is not
(it stays in the payload buffer, ADR-0007 §3). #55 and #56 add their frames through the
same method, with no change to this rule.

The flight computer is **not** part of ``STEP_ORDER``: it always runs after all
subsystems, and the controls it produces in tick N apply in tick N+1 (ADR-0004 §2). It
uses no randomness and no wall-clock time; simulated time is passed in by the caller
(integer microseconds from the run's ``SimClock``, ADR-0003).
"""

from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Final, Self

from pocketsat import messages
from pocketsat.core.clock import check_us
from pocketsat.flight.boot import DEFAULT_BOOT_CONFIG, BootConfig, boot_complete, uptime_ms
from pocketsat.flight.modes import (
    INITIAL_STATE,
    EventKind,
    Mode,
    ModeEvent,
    ModeState,
    Outcome,
    Transition,
    controls_for_mode,
    transition,
)
from pocketsat.flight.safety import (
    DEFAULT_SAFETY_CONFIG,
    INITIAL_SAFETY_STATE,
    SafetyConfig,
    SafetyState,
    SafetyVerdict,
    active_flags,
    evaluate,
)
from pocketsat.frame import (
    MAX_SEQUENCE,
    MIN_FRAME_SIZE,
    Frame,
    FrameError,
    FrameType,
    decode_frame,
    encode_frame,
)
from pocketsat.spacecraft.base import SpacecraftState
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    AttitudeReadings,
    AttitudeSnapshot,
    CommsReadings,
    CommsSnapshot,
    PayloadReadings,
    PayloadSnapshot,
    PowerReadings,
    PowerSnapshot,
    ThermalReadings,
    ThermalSnapshot,
)

PHASE_ORDER: Final[tuple[str, ...]] = (
    "decode_uplink",
    "execute_commands",
    "evaluate_flags",
    "update_mode",
    "emit_telemetry",
    "produce_controls",
)
"""The phases :meth:`FlightComputer.step` runs, in order. Each is the method
``FlightComputer._<name>``. The order is ADR-0004 §2 steps c to f and never changes
without a new ADR."""


@dataclass(frozen=True)
class SpacecraftReadings:
    """The subsystems' reported values for one tick: the flight computer's only view of
    the spacecraft (ADR-0004 §6, §7).

    The flight computer decides from readings, never from true values, so its input is
    typed to the readings records: mypy rejects a truth record, and construction checks
    the exact types at run time. Build it from the stack's :class:`SpacecraftState`
    with :meth:`from_state`.

    Attributes:
        power: Power readings (bus voltage, SOC estimate, battery flags).
        thermal: Thermal readings (temperatures, temperature flags).
        attitude: Attitude readings (pointing error, rate, state).
        payload: Payload readings (state, buffer, chunk IDs).
        comms: Communications readings (radio state, transmit capacity).

    Raises:
        TypeError: A field is not its subsystem's readings record.
    """

    power: PowerReadings
    thermal: ThermalReadings
    attitude: AttitudeReadings
    payload: PayloadReadings
    comms: CommsReadings

    def __post_init__(self) -> None:
        for name, kind in (
            ("power", PowerReadings),
            ("thermal", ThermalReadings),
            ("attitude", AttitudeReadings),
            ("payload", PayloadReadings),
            ("comms", CommsReadings),
        ):
            value = getattr(self, name)
            if type(value) is not kind:
                raise TypeError(f"{name} must be a {kind.__name__}, got {type(value).__name__}")

    @classmethod
    def from_state(cls, state: SpacecraftState) -> Self:
        """Take the readings records out of a :class:`SpacecraftState`.

        The truth records are left behind, so nothing downstream can reach them.

        Args:
            state: Every subsystem's snapshot after this tick's subsystem step.

        Returns:
            The readings of power, thermal, attitude, payload, and comms.

        Raises:
            KeyError: A subsystem is missing from ``state``.
            TypeError: A snapshot is not its subsystem's contract snapshot.
        """
        return cls(
            power=state.get("power", PowerSnapshot).readings,
            thermal=state.get("thermal", ThermalSnapshot).readings,
            attitude=state.get("attitude", AttitudeSnapshot).readings,
            payload=state.get("payload", PayloadSnapshot).readings,
            comms=state.get("comms", CommsSnapshot).readings,
        )


@dataclass(frozen=True)
class FlightComputerOutput:
    """What one :meth:`FlightComputer.step` hands back to ``SilTarget`` (#59).

    Attributes:
        downlink_frames: Encoded frames to transmit this tick, in transmit order
            (ACK/NACK, then telemetry, then DATA; ADR-0004 §10), together no longer
            than comms' ``transmit_capacity_bytes`` for the tick. ``SilTarget`` returns
            them from ``receive()``. ACK/NACK frames come from #51; telemetry (#55) and
            DATA (#56) are added later.
        controls: The :class:`SpacecraftControls` for the next tick (ADR-0004 §2 step
            f), from the single controls function. ``SilTarget`` merges fault overrides
            and the radio traffic into them at step a of the next tick (#59, #98).
        outbound_suppressed_count: ACK/NACK and telemetry frames suppressed this tick
            because they did not fit comms' transmit capacity (ADR-0004 §10,
            ADR-0007 §3). Per tick, not a running total; ``SilTarget`` passes it on in
            ``RadioTraffic.outbound_suppressed_count`` (#98). Counted for ACK/NACK
            (#51) and, later, telemetry (#55).

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``outbound_suppressed_count`` is negative.
    """

    downlink_frames: tuple[bytes, ...]
    controls: SpacecraftControls
    outbound_suppressed_count: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.downlink_frames, tuple) or not all(
            isinstance(frame, bytes) for frame in self.downlink_frames
        ):
            raise TypeError(
                f"downlink_frames must be a tuple of bytes, got {self.downlink_frames!r}"
            )
        if not isinstance(self.controls, SpacecraftControls):
            raise TypeError(
                f"controls must be SpacecraftControls, got {type(self.controls).__name__}"
            )
        count = self.outbound_suppressed_count
        if isinstance(count, bool) or not isinstance(count, int):
            raise TypeError(f"outbound_suppressed_count must be an int, got {count!r}")
        if count < 0:
            raise ValueError(f"outbound_suppressed_count must be non-negative, got {count}")

    @property
    def sent_bytes(self) -> int:
        """Wire bytes sent this tick: the total length of :attr:`downlink_frames`.

        This is ADR-0007's ``RadioTraffic.sent_bytes`` (header, payload, and CRC of
        every frame type), derived from the frames so the two can never disagree.
        ``SilTarget`` passes it on (#98). Never more than comms'
        ``transmit_capacity_bytes`` for the tick.
        """
        return sum(len(frame) for frame in self.downlink_frames)


class OutboundClass(IntEnum):
    """The priority class of a downlink frame (ADR-0004 §10): lower goes first.

    :meth:`FlightComputer._queue_outbound` takes frames in this order within a tick
    (the order of the phases that produce them guarantees it) and raises if a frame of
    a higher class is offered after a lower one, so a later ticket cannot reorder the
    downlink by accident.
    """

    ACK = 0
    """ACK and NACK frames (#51), queued in the execute-commands phase (step c)."""

    TELEMETRY = 1
    """TELEMETRY frames (#55), queued in the emit-telemetry phase (step e)."""

    DATA = 2
    """DATA frames (#56), queued in the emit-telemetry phase after telemetry."""

    @property
    def counts_suppression(self) -> bool:
        """Whether a frame of this class that does not fit is counted as suppressed.

        True for ACK/NACK and telemetry, which are not queued for later. False for
        DATA: a chunk that does not fit stays in the payload buffer and is sent in a
        later tick (ADR-0004 §10, ADR-0007 §3).
        """
        return self is not OutboundClass.DATA


@dataclass(frozen=True)
class UplinkCommand:
    """One COMMAND frame received this tick, decoded by the decode-uplink phase (#51).

    Attributes:
        sequence: The COMMAND frame's header sequence number, echoed in the ACK/NACK.
        command: The decoded payload (#52): a ``ParsedCommand``, or a
            ``MalformedCommand`` with the ``DecodeReason`` its NACK carries.
    """

    sequence: int
    command: "messages.ParsedCommand | messages.MalformedCommand"


@dataclass
class TickContext:
    """Working data for one :meth:`FlightComputer.step`, passed through every phase.

    Created at the start of ``step()`` and dropped at its end; nothing in it survives
    the tick. Phases read what earlier phases wrote and add their own results. Later
    tickets add the fields they need.

    Attributes:
        now_us: Simulated time of this tick, integer microseconds (``SimClock.now_us``).
        readings: The subsystems' readings after this tick's subsystem step.
        uplink_frames: Raw uplink frames received this tick, in arrival order.
        commands: The COMMAND frames among them, decoded, in arrival order (#51).
        mode_events: Mode events raised this tick (by #48, #49, #51, #56), applied in
            order by the update-mode phase.
        transitions: The result of each applied mode event, in order.
        safe_exit_allowed: Whether the flags that put the spacecraft in SAFE have
            cleared (#48); passed to ``transition()``. Set by the evaluate-flags phase
            from this tick's readings; False until then.
        safety: The safe-mode and fault rules' verdict for this tick (#48), set by the
            evaluate-flags phase.
        downlink_frames: Encoded frames queued for transmission this tick, in transmit
            order. Appended only by :meth:`FlightComputer._queue_outbound`.
        outbound_suppressed_count: ACK/NACK and telemetry frames suppressed this tick.
        outbound_remaining_bytes: What is left of comms' ``transmit_capacity_bytes``
            (from ``readings``) after the frames queued so far.
        outbound_class: The class of the last frame offered for transmission; frames
            must be offered in :class:`OutboundClass` order.
    """

    now_us: int
    readings: SpacecraftReadings
    uplink_frames: tuple[bytes, ...]
    mode_events: list[ModeEvent] = field(default_factory=list)
    transitions: list[Transition] = field(default_factory=list)
    safe_exit_allowed: bool = False
    safety: SafetyVerdict | None = None
    downlink_frames: list[bytes] = field(default_factory=list)
    outbound_suppressed_count: int = 0
    commands: list[UplinkCommand] = field(default_factory=list)
    outbound_remaining_bytes: int = field(init=False)
    outbound_class: OutboundClass = field(init=False, default=OutboundClass.ACK)

    def __post_init__(self) -> None:
        self.outbound_remaining_bytes = self.readings.comms.transmit_capacity_bytes

    def reserve_outbound(self, size: int, outbound_class: OutboundClass) -> bool:
        """Claim ``size`` bytes of this tick's transmit capacity for one frame.

        The capacity rule of ADR-0004 §10 and ADR-0007 §4, in one place. Frames are
        offered in priority order and each is taken if it fits what is left; one that
        doesn't is not queued (and counted as suppressed unless it is DATA). A frame
        that does not fit does not block a later, smaller one.

        Args:
            size: The frame's size on the wire, bytes (header, payload, and CRC).
            outbound_class: The frame's priority class.

        Returns:
            True if the frame fits and its bytes are now reserved; False if it is
            suppressed (or, for DATA, left for a later tick).

        Raises:
            RuntimeError: A frame of a higher class is offered after a lower one.
        """
        if outbound_class < self.outbound_class:
            raise RuntimeError(
                f"{outbound_class.name} frame offered after a {self.outbound_class.name} "
                "frame; outbound frames go in OutboundClass order (ADR-0004 §10)"
            )
        self.outbound_class = outbound_class
        if size <= self.outbound_remaining_bytes:
            self.outbound_remaining_bytes -= size
            return True
        if outbound_class.counts_suppression:
            self.outbound_suppressed_count += 1
        return False


class FlightComputer:
    """The flight computer: modes, commands, telemetry, and controls (epic #45).

    Not a subsystem and not part of ``STEP_ORDER``: it runs after every subsystem each
    tick, within ADR-0004 §2 steps c to f, and the controls it produces apply from the
    next tick (ADR-0004 §2). It reads only the subsystems' readings (ADR-0004 §6, §7).

    Lifecycle:

    - :meth:`reset` is power-on: BOOT (ADR-0005 §5), boot counter 0, and BOOT's
      controls returned for tick 0. A new flight computer starts in that state.
    - :meth:`step` runs once per tick (see :data:`PHASE_ORDER`).
    - :meth:`reboot` is the RESET path shared by the RESET command (#49, #51) and the
      ``forced_reset`` fault (#60, ADR-0004 §9).
    - BOOT lasts :attr:`BootConfig.duration_us` of uptime, then ``BOOT_COMPLETE``
      moves it to NOMINAL (#49); a safe condition or fault in BOOT leaves it earlier.

    Deterministic: the same calls with the same arguments always give the same outputs.
    No randomness, no wall-clock time; simulated time comes from the caller.
    """

    def __init__(
        self,
        *,
        safety: SafetyConfig = DEFAULT_SAFETY_CONFIG,
        boot: BootConfig = DEFAULT_BOOT_CONFIG,
    ) -> None:
        """Create a flight computer in its power-on state at time 0 (see :meth:`reset`).

        Args:
            safety: Settings of the safe-mode rules (#48), such as how many ticks a
                flag must be sustained.
            boot: Settings of the boot sequence (#49): how long BOOT lasts.

        Raises:
            TypeError: ``safety`` is not a :class:`SafetyConfig` or ``boot`` is not a
                :class:`BootConfig`.
        """
        if not isinstance(safety, SafetyConfig):
            raise TypeError(f"safety must be a SafetyConfig, got {type(safety).__name__}")
        if not isinstance(boot, BootConfig):
            raise TypeError(f"boot must be a BootConfig, got {type(boot).__name__}")
        self._safety_config = safety
        self._boot_config = boot
        self._safety_state: SafetyState = INITIAL_SAFETY_STATE
        self._mode_state: ModeState = INITIAL_STATE
        self._boot_count = 0
        self._boot_time_us = 0
        self._now_us = 0
        self._downlink_sequence = 0
        self.reset()

    # --- State ------------------------------------------------------------------------

    @property
    def mode(self) -> Mode:
        """The current mode."""
        return self._mode_state.mode

    @property
    def mode_state(self) -> ModeState:
        """The mode state machine's whole state (mode and DOWNLINK return mode)."""
        return self._mode_state

    @property
    def boot_count(self) -> int:
        """Reboots since power-on (#49): 0 after :meth:`reset`, +1 for every
        :meth:`reboot` (RESET command or ``forced_reset``), never cleared by one.

        Not bounded here; telemetry's uint16 field saturates at 65535.
        """
        return self._boot_count

    @property
    def uptime_us(self) -> int:
        """Simulated time since the flight software last started, integer microseconds,
        as of the last :meth:`reset`, :meth:`reboot`, or :meth:`step`.

        0 at power-on and at every reboot (#49).
        """
        return self._now_us - self._boot_time_us

    @property
    def uptime_ms(self) -> int:
        """:attr:`uptime_us` in whole milliseconds, for telemetry (#49, #55).

        Not wrapped here: the telemetry encoder sends it modulo 2**32
        (:func:`pocketsat.flight.boot.uptime_ms`).
        """
        return uptime_ms(self.uptime_us)

    # --- Lifecycle --------------------------------------------------------------------

    def reset(self, *, now_us: int = 0) -> SpacecraftControls:
        """Power on: enter BOOT and return BOOT's controls, which apply to tick 0.

        Clears all flight computer state, the boot counter included, and starts uptime
        at ``now_us``. ``SilTarget.reset(seed)`` calls this and applies the returned
        controls to tick 0 (ADR-0004 §2, ADR-0005 §5, #59).

        Args:
            now_us: Simulated time of power-on, integer microseconds (0 at the start of
                a run).

        Returns:
            BOOT's controls, from the single controls function.

        Raises:
            TypeError: ``now_us`` is not an int.
            ValueError: ``now_us`` is negative.
        """
        check_us("now_us", now_us)
        self._boot_count = 0
        self._now_us = now_us
        self._clear_transient_state()
        return self._produce_controls()

    def reboot(self, *, now_us: int) -> None:
        """Reboot through the RESET path: BOOT, transient state cleared, uptime reset,
        boot counter incremented. Takes effect immediately.

        The RESET command (applied in the update-mode phase, #49, #51) and the
        ``forced_reset`` fault (#60) both call this (ADR-0004 §9). Subsystem physical
        state is not touched: it is not the flight computer's. The next produce-controls
        phase (step f of this tick for RESET, of the next step for a call between
        steps) produces BOOT's controls, and the boot duration counts from ``now_us``.

        Held-in-reset period (ADR-0004 §9): not the flight computer's. It belongs to
        ``SilTarget``'s ``forced_reset`` (#60): the target doesn't step the flight
        computer during the hold and calls ``reboot(now_us=<end of the hold>)`` once, so
        uptime counts from leaving reset. A RESET command has no hold.

        Args:
            now_us: Simulated time of the reboot, integer microseconds. Uptime counts
                from here.

        Raises:
            TypeError: ``now_us`` is not an int.
            ValueError: ``now_us`` is negative or earlier than the last time seen.
        """
        self._advance_time(now_us)
        self._boot_count += 1
        self._clear_transient_state()

    def _clear_transient_state(self) -> None:
        """Forget everything that does not survive a reboot and restart uptime.

        Today that is the mode state (back to BOOT, :data:`INITIAL_STATE`), the
        safe-mode persistence counters (#48), so a flag still set after a reboot must
        be sustained again from BOOT, and the downlink sequence counter (#51), so the
        first frame after a reboot has sequence 0. Later tickets add their own
        transient state here (for example the telemetry schedule, #55, and the
        downlink session, #56). The boot counter is not transient.
        """
        self._mode_state = INITIAL_STATE
        self._safety_state = INITIAL_SAFETY_STATE
        self._boot_time_us = self._now_us
        self._downlink_sequence = 0

    def _advance_time(self, now_us: int) -> None:
        check_us("now_us", now_us)
        if now_us < self._now_us:
            raise ValueError(f"now_us went backwards: {now_us} < {self._now_us}")
        self._now_us = now_us

    # --- The tick -----------------------------------------------------------------------

    def step(
        self,
        uplink_frames: Iterable[bytes],
        readings: SpacecraftReadings,
        now_us: int,
    ) -> FlightComputerOutput:
        """Run one tick of the flight computer (ADR-0004 §2 steps c to f).

        The single entry point. Runs the phases in :data:`PHASE_ORDER`: decode uplink,
        execute commands, evaluate flags, update mode, emit telemetry, produce
        controls.

        Args:
            uplink_frames: Raw frames received this tick, in arrival order. ``SilTarget``
                passes only frames that arrived while the receiver was on and that
                decode as wire frames, and counts the rest (#59). Any frame given here
                that does not decode, or is not a COMMAND frame, is ignored without a
                reply and never raises.
            readings: The subsystems' readings after this tick's subsystem step
                (:meth:`SpacecraftReadings.from_state`).
            now_us: Simulated time of this tick, integer microseconds
                (``SimClock.now_us``). Never earlier than the last call.

        Returns:
            This tick's downlink frames, the controls for the next tick, and the
            suppressed-frame count.

        Raises:
            TypeError: An argument has the wrong type.
            ValueError: ``now_us`` is negative or earlier than the last time seen.
        """
        frames = tuple(uplink_frames)
        for frame in frames:
            if not isinstance(frame, bytes):
                raise TypeError(f"uplink frames must be bytes, got {type(frame).__name__}")
        if not isinstance(readings, SpacecraftReadings):
            raise TypeError(f"readings must be SpacecraftReadings, got {type(readings).__name__}")
        self._advance_time(now_us)

        tick = TickContext(now_us=now_us, readings=readings, uplink_frames=frames)
        self._decode_uplink(tick)
        self._execute_commands(tick)
        self._evaluate_flags(tick)
        self._update_mode(tick)
        self._emit_telemetry(tick)
        controls = self._produce_controls()
        return FlightComputerOutput(
            downlink_frames=tuple(tick.downlink_frames),
            controls=controls,
            outbound_suppressed_count=tick.outbound_suppressed_count,
        )

    # --- Phases, in PHASE_ORDER -------------------------------------------------------

    def _decode_uplink(self, tick: TickContext) -> None:
        """Phase 1 (ADR-0004 step c): decode the uplink frames into commands (#51).

        Each frame that decodes as a COMMAND frame is decoded with #52's
        ``decode_command`` into ``tick.commands``, in arrival order, with its header
        sequence number. A payload that is malformed inside a valid frame becomes a
        ``MalformedCommand`` and is NACKed by the next phase.

        Frames that fail frame decoding (sync, length, CRC, type) get no reply, because
        their sequence number cannot be trusted (``docs/protocol.md``). ``SilTarget``
        already drops and counts them (#59), so the flight computer only skips any it is
        given directly, without counting. Frames of another type (TELEMETRY or ACK,
        which only the spacecraft sends) are skipped the same way. Never raises.
        """
        for raw in tick.uplink_frames:
            try:
                frame = decode_frame(raw)
            except FrameError:
                continue
            if frame.frame_type is not FrameType.COMMAND:
                continue
            tick.commands.append(
                UplinkCommand(frame.sequence, messages.decode_command(frame.payload))
            )

    def _execute_commands(self, tick: TickContext) -> None:
        """Phase 2 (ADR-0004 step c): dispatch the decoded commands and queue ACK/NACK.

        Every command in ``tick.commands`` gets exactly one ACK frame (an ACK, or a NACK
        with a reason), naming its sequence number and command ID, queued with
        ``OutboundClass.ACK`` priority through :meth:`_queue_outbound`, so it goes
        out first and within comms' transmit capacity, or is suppressed and counted.
        In arrival order:

        1. A ``MalformedCommand`` is NACKed with its ``DecodeReason`` (0x01-0x04). The
           decoding reasons are checked before the mode (#52): an unknown command in
           BOOT is ``UNKNOWN_COMMAND``, not ``BOOT_IN_PROGRESS``.
        2. PING is ACKed in every mode and raises no event.
        3. SET_MODE, BEGIN_DOWNLINK, ENTER_SAFE_MODE, and RESET raise the mode event of
           the same name, appended to ``tick.mode_events``. The update-mode phase
           applies it in step d (``docs/spacecraft-modes.md``) and, for RESET, reboots
           there (#49).
           The ACK or NACK is the result of #47's ``transition()`` for that event: ACK
           for TRANSITION and NO_CHANGE, NACK with the ``RejectReason`` for REJECTED.

        **ACK timing.** The ACK is sent in the same tick the command arrives
        (ADR-0004 §2: command → ACK, same tick). To answer in step c for an event
        applied in step d, this phase runs ``transition()`` on a working copy of the
        mode state, from the current state through this tick's commands in order, with
        ``safe_exit_allowed`` from this tick's readings (:func:`active_flags`, the guard
        #48's ``evaluate`` reports). The update-mode phase then applies the same events
        to the same state with the same guard, before any automatic event, so it gets
        the same results: the ACK states what the command did to the mode this tick.
        An ACK means the command was accepted and the mode changed (or already was what
        was asked) by the end of this tick, and telemetry from this tick shows it; the
        new mode's controls take effect in the next tick (ADR-0004 §2), so the physical
        effect follows one tick after the ACK. An automatic event later in the same
        tick (SAFE_CONDITION, FAULT_DETECTED) can still override an ACKed change; the
        ACK reports the command, and telemetry the final mode. A RESET ACK goes out in
        the tick of the reset, numbered in the downlink sequence from before the
        reboot; commands after a RESET in the same tick are answered as in BOOT.
        """
        state = self._mode_state
        safe_exit_allowed = not active_flags(tick.readings)
        for received in tick.commands:
            command = received.command
            reason: messages.NackReason | None
            if isinstance(command, messages.MalformedCommand):
                reason = command.reason
            elif command.command_id is messages.CommandId.PING:
                reason = None
            else:
                event = _mode_event(command)
                result = transition(state, event, safe_exit_allowed=safe_exit_allowed)
                state = result.state
                tick.mode_events.append(event)
                reason = result.reason
            ack = messages.CommandAck(received.sequence, command.command_id, reason)
            self._queue_outbound(tick, FrameType.ACK, messages.encode_ack(ack), OutboundClass.ACK)

    def _evaluate_flags(self, tick: TickContext) -> None:
        """Phase 3 (ADR-0004 step d): turn readings flags and boot timing into events.

        Applies the safe-mode and fault rules (#48, :func:`pocketsat.flight.safety.evaluate`)
        to this tick's **readings**: appends FAULT_DETECTED when a consistency check
        fails and SAFE_CONDITION while a safe-mode flag is sustained, after any events
        the commands raised (so automatic events have the last word in a tick), and
        sets ``tick.safe_exit_allowed`` for the update-mode phase. Runs in every mode;
        the state machine ignores what doesn't apply.

        Then, in BOOT, appends BOOT_COMPLETE once the uptime has reached the boot
        duration (#49, :class:`BootConfig`). It comes last, so a SAFE_CONDITION or
        FAULT_DETECTED in the same tick wins and BOOT_COMPLETE is ignored. Once SAFE or
        FAULT has left BOOT, BOOT_COMPLETE is never raised: every boot step but the wait
        already ran at the reboot (``docs/spacecraft-modes.md``, "Boot sequence").
        """
        verdict = evaluate(self._safety_state, tick.readings, self._safety_config)
        self._safety_state = verdict.state
        tick.safety = verdict
        tick.safe_exit_allowed = verdict.safe_exit_allowed
        tick.mode_events.extend(verdict.events)
        if self._mode_state.mode is Mode.BOOT and boot_complete(self.uptime_us, self._boot_config):
            tick.mode_events.append(ModeEvent(EventKind.BOOT_COMPLETE))

    def _update_mode(self, tick: TickContext) -> None:
        """Phase 4 (ADR-0004 step d): apply this tick's mode events, in order.

        Each event in ``tick.mode_events`` goes through #47's ``transition()``; its
        result is recorded in ``tick.transitions``, one per event. With no events the
        mode stays as it is.

        An applied RESET reboots here (#49): :meth:`reboot` at this tick's time, so the
        boot counter goes up, uptime restarts, and transient state is cleared. Frames
        already queued this tick (the RESET's ACK, #51) still go out, and step f
        produces BOOT's controls. Automatic events after the RESET were raised from the
        state the reboot discarded, so they are recorded as IGNORED and not applied;
        later command events still go through the table (in BOOT: a NACK, or another
        RESET).
        """
        rebooted = False
        for event in tick.mode_events:
            if rebooted and not event.kind.is_command:
                tick.transitions.append(Transition(Outcome.IGNORED, self._mode_state))
                continue
            result = transition(self._mode_state, event, safe_exit_allowed=tick.safe_exit_allowed)
            self._mode_state = result.state
            tick.transitions.append(result)
            if event.kind is EventKind.RESET:
                self.reboot(now_us=tick.now_us)
                rebooted = True

    def _emit_telemetry(self, tick: TickContext) -> None:
        """Phase 5 (ADR-0004 step e): emit telemetry if due, and DATA in DOWNLINK.

        No-op for now. #55 adds the per-mode cadence and sends TELEMETRY frames with
        ``self._queue_outbound(tick, FrameType.TELEMETRY, payload,
        OutboundClass.TELEMETRY)``, which puts them after any ACK/NACK, within the
        remaining transmit capacity, and counts suppressed ones. #56 then fills what
        capacity is left with DATA frames the same way (``OutboundClass.DATA``, not
        counted when they don't fit).
        """

    def _produce_controls(self) -> SpacecraftControls:
        """Phase 6 (ADR-0004 step f): the controls for the next tick.

        The single controls function of ADR-0004 §14: every ``SpacecraftControls`` the
        flight computer produces comes from here, both from :meth:`step` and from
        :meth:`reset`. It returns #47's ``controls_for_mode()`` for the mode after
        step d. #56 adds ``release_through_chunk_id`` here.
        """
        return controls_for_mode(self._mode_state.mode)

    # --- Outbound -----------------------------------------------------------------------

    def _queue_outbound(
        self,
        tick: TickContext,
        frame_type: FrameType,
        payload: bytes,
        outbound_class: OutboundClass,
    ) -> bool:
        """Send one downlink frame this tick if it fits the transmit capacity.

        The single way any phase sends a frame (ACK/NACK #51, telemetry #55, DATA #56).
        The capacity and priority rule is :meth:`TickContext.reserve_outbound`. A frame
        that fits gets the next downlink sequence number (one counter for every frame
        type, wrapping from 0xFFFF to 0, ``docs/protocol.md``), is encoded, and is
        appended to ``tick.downlink_frames``. A frame that does not fit is never
        encoded and uses no sequence number, so a gap in the ground's sequence means a
        frame lost on the way down, not one the spacecraft never sent.

        Args:
            tick: This tick's working data.
            frame_type: The frame type.
            payload: The frame payload.
            outbound_class: Its priority class.

        Returns:
            True if the frame was queued, False if it did not fit.

        Raises:
            RuntimeError: Frames were offered out of :class:`OutboundClass` order.
        """
        if not tick.reserve_outbound(MIN_FRAME_SIZE + len(payload), outbound_class):
            return False
        frame = Frame(frame_type, self._downlink_sequence, payload)
        tick.downlink_frames.append(encode_frame(frame))
        self._downlink_sequence = (self._downlink_sequence + 1) & MAX_SEQUENCE
        return True


def _mode_event(command: "messages.ParsedCommand") -> ModeEvent:
    """The mode event a command raises: the :class:`EventKind` of the same name.

    Every :class:`~pocketsat.messages.CommandId` except PING names a command event
    (``docs/protocol.md``, "Command IDs"); SET_MODE carries its target mode.
    """
    return ModeEvent(EventKind[command.command_id.name], command.target)
