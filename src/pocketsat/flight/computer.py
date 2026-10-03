"""The flight computer component and its single ``step()`` entry point (#101).

:class:`FlightComputer` is the one place the flight software plugs into. Each tick,
after every subsystem has stepped, ``SilTarget`` (#59) calls :meth:`FlightComputer.step`
once with the uplink frames received that tick, the subsystems' **readings** (never their
truth, ADR-0004 §6, §7), and the simulated time. ``step()`` runs six fixed phases, which
are ADR-0004 §2 steps c to f:

1. ``decode_uplink`` (step c): decode COMMAND frames. Filled by #51 and #52.
2. ``execute_commands`` (step c): dispatch commands and queue ACK/NACK. Filled by #51
   (and #49 for RESET).
3. ``evaluate_flags`` (step d): raise SAFE_CONDITION, FAULT_DETECTED, and BOOT_COMPLETE.
   Filled by #48 and #49.
4. ``update_mode`` (step d): apply the mode events with #47's ``transition()``.
5. ``emit_telemetry`` (step e): telemetry when due, then DATA in DOWNLINK. Filled by
   #55 and #56.
6. ``produce_controls`` (step f): the next tick's controls from #47's
   ``controls_for_mode()``, the single controls function (ADR-0004 §14). #56 adds the
   chunk release.

``evaluate_flags`` applies #48's safe-mode and fault rules
(:mod:`pocketsat.flight.safety`); ``update_mode`` and ``produce_controls`` call #47's
:func:`~pocketsat.flight.modes.transition` and
:func:`~pocketsat.flight.modes.controls_for_mode`. The other phases are documented
no-ops. Later tickets fill in one phase each.

The flight computer is **not** part of ``STEP_ORDER``: it always runs after all
subsystems, and the controls it produces in tick N apply in tick N+1 (ADR-0004 §2). It
uses no randomness and no wall-clock time; simulated time is passed in by the caller
(integer microseconds from the run's ``SimClock``, ADR-0003).
"""

from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Final, Self

from pocketsat.core.clock import check_us
from pocketsat.flight.modes import (
    INITIAL_STATE,
    Mode,
    ModeEvent,
    ModeState,
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
    evaluate,
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
            (ACK/NACK, then telemetry, then DATA; ADR-0004 §10). ``SilTarget`` returns
            them from ``receive()``. Filled by #51 (ACK/NACK), #55 (telemetry), and #56
            (DATA); always empty today.
        controls: The :class:`SpacecraftControls` for the next tick (ADR-0004 §2 step
            f), from the single controls function. ``SilTarget`` merges fault overrides
            and the radio traffic into them at step a of the next tick (#59, #98).
        outbound_suppressed_count: ACK/NACK and telemetry frames suppressed this tick
            because they did not fit comms' transmit capacity (ADR-0004 §10,
            ADR-0007 §3). Per tick, not a running total; ``SilTarget`` passes it on in
            ``RadioTraffic.outbound_suppressed_count`` (#98). Counted by #51 and #55;
            always 0 today.

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
        ``SilTarget`` passes it on (#98). 0 until #51, #55, and #56 send frames.
        """
        return sum(len(frame) for frame in self.downlink_frames)


@dataclass
class TickContext:
    """Working data for one :meth:`FlightComputer.step`, passed through every phase.

    Created at the start of ``step()`` and dropped at its end; nothing in it survives
    the tick. Phases read what earlier phases wrote and add their own results. Later
    tickets add the fields they need (for example decoded commands, #51).

    Attributes:
        now_us: Simulated time of this tick, integer microseconds (``SimClock.now_us``).
        readings: The subsystems' readings after this tick's subsystem step.
        uplink_frames: Raw uplink frames received this tick, in arrival order.
        mode_events: Mode events raised this tick (by #48, #49, #51, #56), applied in
            order by the update-mode phase.
        transitions: The result of each applied mode event, in order.
        safe_exit_allowed: Whether the flags that put the spacecraft in SAFE have
            cleared (#48); passed to ``transition()``. Set by the evaluate-flags phase
            from this tick's readings; False until then.
        safety: The safe-mode and fault rules' verdict for this tick (#48), set by the
            evaluate-flags phase.
        downlink_frames: Encoded frames queued for transmission this tick.
        outbound_suppressed_count: ACK/NACK and telemetry frames suppressed this tick.
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

    Deterministic: the same calls with the same arguments always give the same outputs.
    No randomness, no wall-clock time; simulated time comes from the caller.
    """

    def __init__(self, *, safety: SafetyConfig = DEFAULT_SAFETY_CONFIG) -> None:
        """Create a flight computer in its power-on state at time 0 (see :meth:`reset`).

        Args:
            safety: Settings of the safe-mode rules (#48), such as how many ticks a
                flag must be sustained.

        Raises:
            TypeError: ``safety`` is not a :class:`SafetyConfig`.
        """
        if not isinstance(safety, SafetyConfig):
            raise TypeError(f"safety must be a SafetyConfig, got {type(safety).__name__}")
        self._safety_config = safety
        self._safety_state: SafetyState = INITIAL_SAFETY_STATE
        self._mode_state: ModeState = INITIAL_STATE
        self._boot_count = 0
        self._boot_time_us = 0
        self._now_us = 0
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
        """Reboots since power-on (:meth:`reset`); kept across :meth:`reboot` (#49)."""
        return self._boot_count

    @property
    def uptime_us(self) -> int:
        """Simulated time since the last power-on or reboot, integer microseconds, as of
        the last :meth:`reset`, :meth:`reboot`, or :meth:`step`. Telemetry carries it
        in milliseconds (#49, #55)."""
        return self._now_us - self._boot_time_us

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
        boot counter incremented.

        The RESET command (#49, #51) and the ``forced_reset`` fault (#60) both call this
        (ADR-0004 §9). Subsystem physical state is not touched: it is not the flight
        computer's. BOOT's controls are produced by the next produce-controls phase,
        at step f of the tick the reboot completes in (ADR-0004 §9). The held-in-reset
        period is #49's and #60's to add.

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

        Today that is the mode state (back to BOOT, :data:`INITIAL_STATE`) and the
        safe-mode persistence counters (#48), so a flag still set after a reboot must
        be sustained again from BOOT. Later tickets add their own transient state here
        (for example the telemetry schedule and sequence counters, #55, and the
        downlink session, #56). The boot counter is not transient.
        """
        self._mode_state = INITIAL_STATE
        self._safety_state = INITIAL_SAFETY_STATE
        self._boot_time_us = self._now_us

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
                passes only frames that arrived while the receiver was on (#59).
                Undecodable frames are the decode phase's to handle and never raise.
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
        """Phase 1 (ADR-0004 step c): decode the uplink frames into commands.

        No-op for now: ``tick.uplink_frames`` are passed through untouched. #51 decodes
        COMMAND frames with the frame codec and #52 validates their payloads;
        undecodable frames are counted and never raise.
        """

    def _execute_commands(self, tick: TickContext) -> None:
        """Phase 2 (ADR-0004 step c): dispatch decoded commands and queue ACK/NACK.

        No-op for now. #51 adds the dispatcher: PING, and the command mode events
        (SET_MODE, BEGIN_DOWNLINK, ENTER_SAFE_MODE, RESET) for the mode machine, with
        ACK/NACK frames (reason codes from ``RejectReason`` and #52) queued first in
        ``tick.downlink_frames`` within comms' transmit capacity, counting what is
        suppressed. RESET reboots through :meth:`reboot` (#49).
        """

    def _evaluate_flags(self, tick: TickContext) -> None:
        """Phase 3 (ADR-0004 step d): turn readings flags and boot timing into events.

        Applies the safe-mode and fault rules (#48, :func:`pocketsat.flight.safety.evaluate`)
        to this tick's **readings**: appends FAULT_DETECTED when a consistency check
        fails and SAFE_CONDITION while a safe-mode flag is sustained, after any events
        the commands raised (so automatic events have the last word in a tick), and
        sets ``tick.safe_exit_allowed`` for the update-mode phase. Runs in every mode;
        the state machine ignores what doesn't apply. #49 adds BOOT_COMPLETE once the
        boot duration has elapsed, and decides which boot steps still run on the
        BOOT → SAFE path (#107).
        """
        verdict = evaluate(self._safety_state, tick.readings, self._safety_config)
        self._safety_state = verdict.state
        tick.safety = verdict
        tick.safe_exit_allowed = verdict.safe_exit_allowed
        tick.mode_events.extend(verdict.events)

    def _update_mode(self, tick: TickContext) -> None:
        """Phase 4 (ADR-0004 step d): apply this tick's mode events, in order.

        Each event in ``tick.mode_events`` goes through #47's ``transition()``; its
        result is recorded in ``tick.transitions``. With no events (all that the
        earlier phases raise today) the mode stays as it is.
        """
        for event in tick.mode_events:
            result = transition(self._mode_state, event, safe_exit_allowed=tick.safe_exit_allowed)
            self._mode_state = result.state
            tick.transitions.append(result)

    def _emit_telemetry(self, tick: TickContext) -> None:
        """Phase 5 (ADR-0004 step e): emit telemetry if due, and DATA in DOWNLINK.

        No-op for now. #55 adds the per-mode cadence and appends TELEMETRY frames
        after any ACK/NACK, within the remaining transmit capacity, counting
        suppressed ones in ``tick.outbound_suppressed_count``. #56 then fills what
        capacity is left with DATA frames.
        """

    def _produce_controls(self) -> SpacecraftControls:
        """Phase 6 (ADR-0004 step f): the controls for the next tick.

        The single controls function of ADR-0004 §14: every ``SpacecraftControls`` the
        flight computer produces comes from here, both from :meth:`step` and from
        :meth:`reset`. It returns #47's ``controls_for_mode()`` for the mode after
        step d. #56 adds ``release_through_chunk_id`` here.
        """
        return controls_for_mode(self._mode_state.mode)
