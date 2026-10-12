"""The flight computer component and its single ``step()`` entry point (#101).

:class:`FlightComputer` is the one place the flight software plugs into. Each tick,
after every subsystem has stepped, ``SilTarget`` (#59) calls :meth:`FlightComputer.step`
once with the uplink frames received that tick, the subsystems' **readings** (never their
truth, ADR-0004 §6, §7), and the simulated time. ``step()`` runs six fixed phases, which
are ADR-0004 §2 steps c to f:

1. ``decode_uplink`` (step c): decode COMMAND frames with #52's codec (#51).
2. ``execute_commands`` (step c): dispatch commands and queue ACK/NACK (#51). RESET
   only raises its event here; the reboot happens in ``update_mode`` (#49).
3. ``evaluate_flags`` (step d): raise SAFE_CONDITION, FAULT_DETECTED, BOOT_COMPLETE, and
   DOWNLINK_COMPLETE. Filled by #48, #49, and #56.
4. ``update_mode`` (step d): apply the mode events with #47's ``transition()``; an
   applied RESET reboots (#49). Starts and ends the downlink session (#56).
5. ``emit_telemetry`` (step e): telemetry when due (#55), then DATA in DOWNLINK (#56).
6. ``produce_controls`` (step f): the next tick's controls from #47's
   ``controls_for_mode()``, the single controls function (ADR-0004 §14), with the
   downlink session's chunk release (#56).

``decode_uplink`` and ``execute_commands`` are #51's command dispatcher;
``evaluate_flags`` applies #48's safe-mode and fault rules
(:mod:`pocketsat.flight.safety`) and #49's boot timing (:mod:`pocketsat.flight.boot`);
``update_mode`` and ``produce_controls`` call #47's
:func:`~pocketsat.flight.modes.transition` and
:func:`~pocketsat.flight.modes.controls_for_mode`; ``emit_telemetry`` follows #55's
per-mode cadence (:mod:`pocketsat.flight.telemetry`) and encodes frames with #54's
codec. DATA and the downlink session are #56 (:mod:`pocketsat.flight.downlink`).

**Settings.** One :class:`FlightComputerConfig` holds every setting: #48's
:class:`~pocketsat.flight.safety.SafetyConfig`, #49's
:class:`~pocketsat.flight.boot.BootConfig`, #55's
:class:`~pocketsat.flight.telemetry.TelemetryConfig`, #56's
:class:`~pocketsat.flight.downlink.DownlinkConfig`, and #132's :class:`OutboundConfig`.

**Outbound frames (ADR-0004 §10, ADR-0007 §4, ADR-0008).** Every downlink frame goes
through :meth:`FlightComputer._queue_outbound`. Frames are offered in
:class:`OutboundClass` order, ACK/NACK (step c), then telemetry and DATA (step e), so the
order of the phases is the priority order; offering a higher class after a lower one is
a programming error and raises. The flight computer models the radio driver's transmit
queue (ADR-0008 §1): at most one frame **in transmission**, carried from tick to tick,
and the ACK/NACK and telemetry frames **held** behind it. Each tick spends comms'
``transmit_capacity_bytes`` (read from the readings) in one of two ways (ADR-0008 §2):

- **A whole-frame tick** (the capacity is at least :attr:`FlightComputer.max_frame_size`,
  nothing is in transmission and nothing is held; every tick at the default 100 ms):
  each frame is sent whole if it fits what is left, gets the next downlink sequence
  number and is delivered at once. A frame that does not fit is not queued; ACK/NACK
  and telemetry that don't fit are counted in ``outbound_suppressed_count``, DATA is not
  (it stays in the payload buffer, ADR-0007 §3), and the downlink session stops at the
  first DATA frame that does not fit, so chunks never go out of order.
- **A streaming tick** (any other tick): the frame in transmission continues first,
  then waiting frames are **committed** one after another in priority order, each
  numbered when its first byte goes out and spanning ticks as needed, never preempted
  or interleaved. A frame is delivered in the tick its last byte is transmitted.
  ACK/NACK and telemetry that cannot be committed are held (at most
  :attr:`OutboundConfig.max_held_ack_frames` ACK/NACK, oldest first, and one telemetry
  frame, a newer one replacing it), and DATA is committed one chunk at a time when
  nothing else waits.

``FlightComputerOutput.sent_bytes`` is the bytes transmitted in the tick, never more
than the capacity. A tick with the transmitter off aborts the frame in transmission and
suppresses every held and new ACK/NACK and telemetry frame; a reboot aborts and drops
them too; leaving DOWNLINK aborts a DATA frame in transmission, whose chunk stays
buffered (ADR-0008 §6). A RESET command reboots only once its ACK has been transmitted
(ADR-0008 §6a): until then the reboot is **pending**.

The flight computer is **not** part of ``STEP_ORDER``: it always runs after all
subsystems, and the controls it produces in tick N apply in tick N+1 (ADR-0004 §2). It
uses no randomness and no wall-clock time; simulated time is passed in by the caller
(integer microseconds from the run's ``SimClock``, ADR-0003).
"""

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Final, Self

from pocketsat import messages
from pocketsat.core.clock import check_us
from pocketsat.flight.boot import DEFAULT_BOOT_CONFIG, BootConfig, boot_complete, uptime_ms
from pocketsat.flight.downlink import (
    DEFAULT_DOWNLINK_CONFIG,
    DownlinkConfig,
    DownlinkSession,
    release_through_chunk_id,
    transmit_inhibited,
    unsent_chunk_ids,
)
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
from pocketsat.flight.telemetry import (
    DEFAULT_TELEMETRY_CONFIG,
    INITIAL_TELEMETRY_SCHEDULE,
    TelemetryConfig,
    TelemetrySchedule,
    telemetry_due,
)
from pocketsat.frame import (
    MAX_SEQUENCE,
    MIN_FRAME_SIZE,
    FrameError,
    FrameType,
    decode_frame,
    encode_frame_fields,
)
from pocketsat.spacecraft.base import SpacecraftState
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.payload import chunk_content
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
class OutboundConfig:
    """Settings of the outbound queue's transmitter (#132, ADR-0008 §5).

    Attributes:
        max_held_ack_frames: How many ACK/NACK frames may wait behind a frame in
            transmission in a streaming tick (ADR-0008 §2). One more is suppressed and
            counted. Default 8, what one default 100 ms tick carries
            (⌊120 / 14⌋). 0 never holds an ACK/NACK.

    Raises:
        TypeError: ``max_held_ack_frames`` is not an int.
        ValueError: ``max_held_ack_frames`` is negative.
    """

    max_held_ack_frames: int = 8

    def __post_init__(self) -> None:
        value = self.max_held_ack_frames
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"max_held_ack_frames must be an int, got {value!r}")
        if value < 0:
            raise ValueError(f"max_held_ack_frames must be non-negative, got {value}")


DEFAULT_OUTBOUND_CONFIG: Final = OutboundConfig()
"""The default outbound settings: at most 8 held ACK/NACK frames."""


@dataclass(frozen=True)
class FlightComputerConfig:
    """Every setting of the flight computer, in one record passed to
    :class:`FlightComputer` (#55).

    Attributes:
        safety: The safe-mode rules (#48), such as how many ticks a flag must be
            sustained.
        boot: The boot sequence (#49): how long BOOT lasts.
        telemetry: The telemetry cadence per mode (#55).
        downlink: The downlink session's settings (#56): the payload chunk size.
        outbound: The outbound queue's transmitter (#132): the held-ACK bound.

    Raises:
        TypeError: A field is not its settings record.
    """

    safety: SafetyConfig = DEFAULT_SAFETY_CONFIG
    boot: BootConfig = DEFAULT_BOOT_CONFIG
    telemetry: TelemetryConfig = DEFAULT_TELEMETRY_CONFIG
    downlink: DownlinkConfig = DEFAULT_DOWNLINK_CONFIG
    outbound: OutboundConfig = DEFAULT_OUTBOUND_CONFIG

    def __post_init__(self) -> None:
        for name, kind in (
            ("safety", SafetyConfig),
            ("boot", BootConfig),
            ("telemetry", TelemetryConfig),
            ("downlink", DownlinkConfig),
            ("outbound", OutboundConfig),
        ):
            value = getattr(self, name)
            if not isinstance(value, kind):
                raise TypeError(f"{name} must be a {kind.__name__}, got {type(value).__name__}")


DEFAULT_FLIGHT_COMPUTER_CONFIG: Final = FlightComputerConfig()
"""The default settings: 10-tick flag persistence, a 5 s boot, the default telemetry
cadence (``docs/spacecraft-modes.md``, "Telemetry cadence"), 64-byte chunks, and at
most 8 held ACK/NACK frames."""


@dataclass(frozen=True)
class OutboundProgress:
    """The frame in transmission at the end of a tick (#132, ADR-0008 §8).

    A record for tests and run records: ``FlightComputerOutput.transmitting`` and
    ``SilTick.transmitting``.

    Attributes:
        frame_type: The frame's type.
        sequence: Its downlink sequence number, given when it was committed.
        size_bytes: Its size on the wire, bytes.
        sent_bytes: The bytes of it transmitted so far, ``0 < sent_bytes < size_bytes``.
    """

    frame_type: FrameType
    sequence: int
    size_bytes: int
    sent_bytes: int


def _check_count(name: str, value: object) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {value!r}")
    if value < 0:
        raise ValueError(f"{name} must be non-negative, got {value}")


@dataclass(frozen=True, slots=True)
class FlightComputerOutput:
    """What one :meth:`FlightComputer.step` hands back to ``SilTarget`` (#59).

    Attributes:
        downlink_frames: Encoded frames **delivered** this tick: those whose last byte
            was transmitted in it, in transmit order (ADR-0008 §3). In a whole-frame
            tick that is ACK/NACK, then telemetry, then DATA (ADR-0004 §10), together no
            longer than the tick's capacity; a frame completing in a streaming tick may
            have started in an earlier one, so the total can exceed it. ``SilTarget``
            returns them from ``receive()``. ACK/NACK frames come from #51, TELEMETRY
            frames from #55, and DATA frames from the downlink session (#56).
        controls: The :class:`SpacecraftControls` for the next tick (ADR-0004 §2 step
            f), from the single controls function, with the downlink session's chunk
            release (#56). ``SilTarget`` merges fault overrides
            and the radio traffic into them at step a of the next tick (#59,
            ADR-0007).
        outbound_suppressed_count: ACK/NACK and telemetry frames that were due but
            will never be delivered, counted this tick: no room in a whole-frame tick
            or with the transmitter off, the held-frame bound, a held telemetry frame
            replaced by a newer one, or aborted mid-transmission or dropped while held
            (ADR-0004 §10, ADR-0007 §3, ADR-0008 §6). Per tick, not a running total;
            ``SilTarget`` passes it on in ``RadioTraffic.outbound_suppressed_count``.
        sent_bytes: Wire bytes **transmitted** this tick (ADR-0007's
            ``RadioTraffic.sent_bytes``; header, payload, and CRC of every frame type),
            including parts of frames delivered in other ticks (ADR-0008 §3). Never
            more than comms' ``transmit_capacity_bytes`` for the tick. In a whole-frame
            tick, the total length of :attr:`downlink_frames`. ``SilTarget`` passes it
            on to comms.
        outbound_aborted_count: Frames of any class aborted mid-transmission or dropped
            while held this tick, by the transmitter going off or a reboot, or a DATA
            frame aborted with its session (ADR-0008 §6). For tests and run records;
            not passed to comms.
        transmitting: The frame in transmission at the end of the tick, or ``None``.
        reboot_pending: A RESET command's reboot is waiting for its ACK at the end of
            the tick (ADR-0008 §6a).
        uplink_ignored_count: Uplink frames given to the step while a reboot was
            pending: not decoded, executed, or answered (ADR-0008 §6a).

    Raises:
        TypeError: A field has the wrong type.
        ValueError: A count is negative.
    """

    downlink_frames: tuple[bytes, ...]
    controls: SpacecraftControls
    outbound_suppressed_count: int = 0
    sent_bytes: int = field(kw_only=True)
    outbound_aborted_count: int = 0
    transmitting: OutboundProgress | None = None
    reboot_pending: bool = False
    uplink_ignored_count: int = 0

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
        suppressed, sent = self.outbound_suppressed_count, self.sent_bytes
        aborted, ignored = self.outbound_aborted_count, self.uplink_ignored_count
        # A fast path for plain non-negative ints: this runs every tick (#78's budget).
        if not (
            type(suppressed) is int
            and type(sent) is int
            and type(aborted) is int
            and type(ignored) is int
            and suppressed >= 0
            and sent >= 0
            and aborted >= 0
            and ignored >= 0
        ):
            _check_count("outbound_suppressed_count", suppressed)
            _check_count("sent_bytes", sent)
            _check_count("outbound_aborted_count", aborted)
            _check_count("uplink_ignored_count", ignored)
        transmitting = self.transmitting
        if transmitting is not None and type(transmitting) is not OutboundProgress:
            raise TypeError(
                f"transmitting must be an OutboundProgress or None, got {transmitting!r}"
            )
        if type(self.reboot_pending) is not bool:
            raise TypeError(f"reboot_pending must be a bool, got {self.reboot_pending!r}")


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


@dataclass(slots=True)
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
        downlink_frames: Encoded frames delivered this tick (their last byte was
            transmitted in it), in transmit order. Appended only by the outbound queue
            (:meth:`FlightComputer._queue_outbound` and the transmitter).
        outbound_suppressed_count: ACK/NACK and telemetry frames suppressed this tick.
        outbound_remaining_bytes: What is left of comms' ``transmit_capacity_bytes``
            (from ``readings``) after the bytes transmitted so far.
        outbound_class: The class of the last frame offered for transmission; frames
            must be offered in :class:`OutboundClass` order.
        streaming: This is a streaming tick (ADR-0008 §2); False for a whole-frame
            tick, where :meth:`reserve_outbound`'s rule applies. Set by the flight
            computer at the start of the tick.
        transmitter_off: The transmitter is off this tick (capacity 0 and
            ``transmitter_on`` false): every ACK/NACK and telemetry frame is suppressed
            and nothing is held (ADR-0008 §6). Implies ``streaming``.
        sent_bytes: Bytes transmitted this tick so far.
        outbound_aborted_count: Frames aborted or dropped this tick (ADR-0008 §6).
        completed_chunk_id: The last DATA chunk whose frame completed this tick, or
            ``None``: released in step f even if the session ended (ADR-0008 §6).
        uplink_ignored_count: Uplink frames ignored because a reboot is pending.
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
    streaming: bool = False
    transmitter_off: bool = False
    sent_bytes: int = 0
    outbound_aborted_count: int = 0
    completed_chunk_id: int | None = None
    uplink_ignored_count: int = 0

    def __post_init__(self) -> None:
        self.outbound_remaining_bytes = self.readings.comms.transmit_capacity_bytes

    def check_order(self, outbound_class: OutboundClass) -> None:
        """Record that a frame of ``outbound_class`` is offered, checking the order.

        Raises:
            RuntimeError: A frame of a higher class is offered after a lower one.
        """
        if outbound_class < self.outbound_class:
            raise RuntimeError(
                f"{outbound_class.name} frame offered after a {self.outbound_class.name} "
                "frame; outbound frames go in OutboundClass order (ADR-0004 §10)"
            )
        self.outbound_class = outbound_class

    def reserve_outbound(self, size: int, outbound_class: OutboundClass) -> bool:
        """Claim ``size`` bytes of this tick's transmit capacity for one whole frame.

        The capacity rule of ADR-0004 §10 and ADR-0007 §4 for a **whole-frame tick**
        (ADR-0008 §2), in one place. Frames are offered in priority order and each is
        taken if it fits what is left; one that doesn't is not queued (and counted as
        suppressed unless it is DATA). A frame that does not fit does not block a
        later, smaller one. Streaming ticks use the flight computer's transmitter
        instead (:meth:`FlightComputer._queue_outbound`).

        Args:
            size: The frame's size on the wire, bytes (header, payload, and CRC).
            outbound_class: The frame's priority class.

        Returns:
            True if the frame fits and its bytes are now reserved; False if it is
            suppressed (or, for DATA, left for a later tick).

        Raises:
            RuntimeError: A frame of a higher class is offered after a lower one.
        """
        self.check_order(outbound_class)
        if size <= self.outbound_remaining_bytes:
            self.outbound_remaining_bytes -= size
            self.sent_bytes += size
            return True
        if outbound_class.counts_suppression:
            self.outbound_suppressed_count += 1
        return False


@dataclass(slots=True)
class _InTransmission:
    """The frame in transmission: transient flight computer state (ADR-0008 §1)."""

    frame: bytes
    outbound_class: OutboundClass
    sent_bytes: int = 0
    chunk_id: int | None = None  # DATA: the chunk it carries
    reset_ack: bool = False  # the ACK a pending RESET reboot waits for (ADR-0008 §6a)


@dataclass(frozen=True, slots=True)
class _HeldAck:
    """An ACK/NACK frame's payload, held behind the frame in transmission."""

    payload: bytes
    reset_ack: bool = False


class _ResetAck(IntEnum):
    """Where the RESET command's ACK is (ADR-0008 §6a)."""

    NONE = 0  # no RESET command this tick, or no reboot pending
    WAITING = 1  # in transmission or held
    DONE = 2  # its last byte was transmitted
    LOST = 3  # suppressed or aborted: it will never go out


@dataclass(frozen=True, slots=True)
class _PendingReboot:
    """A RESET command's reboot waiting for its ACK (ADR-0008 §6a): the ``RESET`` and
    every mode event after it in its tick, kept in order, and that tick's guard."""

    events: tuple[ModeEvent, ...]
    safe_exit_allowed: bool


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
    - Telemetry follows :class:`TelemetryConfig`'s per-mode cadence (#55): none in BOOT
      but the boot-complete beacon, a frame at every mode change, then one per period.
    - In DOWNLINK a downlink session (#56, :mod:`pocketsat.flight.downlink`) sends the
      payload's chunks as DATA frames, releases them once their frames have been
      transmitted, and raises ``DOWNLINK_COMPLETE`` when none is left; leaving DOWNLINK
      any other way aborts it, and the next session resumes from the oldest unreleased
      chunk.
    - The outbound queue's transmitter (#132, ADR-0008) carries a frame across ticks
      when the tick is too short for it; see the module docstring.

    Deterministic: the same calls with the same arguments always give the same outputs.
    No randomness, no wall-clock time; simulated time comes from the caller.
    """

    def __init__(self, config: FlightComputerConfig = DEFAULT_FLIGHT_COMPUTER_CONFIG) -> None:
        """Create a flight computer in its power-on state at time 0 (see :meth:`reset`).

        Args:
            config: Every setting: the safe-mode rules (#48), the boot sequence (#49),
                the telemetry cadence (#55), the downlink chunk size (#56), and the
                held-ACK bound (#132).

        Raises:
            TypeError: ``config`` is not a :class:`FlightComputerConfig`.
        """
        if not isinstance(config, FlightComputerConfig):
            raise TypeError(f"config must be a FlightComputerConfig, got {type(config).__name__}")
        self._config = config
        self._max_frame_size = max(
            messages.ACK_FRAME_SIZE,
            messages.TELEMETRY_FRAME_SIZE,
            messages.data_frame_size(config.downlink.chunk_size_bytes),
        )
        self._max_held_acks = config.outbound.max_held_ack_frames
        self._safety_state: SafetyState = INITIAL_SAFETY_STATE
        self._telemetry_schedule: TelemetrySchedule = INITIAL_TELEMETRY_SCHEDULE
        self._mode_state: ModeState = INITIAL_STATE
        self._boot_count = 0
        self._boot_time_us = 0
        self._now_us = 0
        self._downlink_sequence = 0
        self._downlink_session: DownlinkSession | None = None
        self._transmitting: _InTransmission | None = None
        self._held_acks: deque[_HeldAck] = deque()
        self._held_telemetry: bytes | None = None
        self._reset_ack = _ResetAck.NONE
        self._pending_reboot: _PendingReboot | None = None
        self._carried_suppressed_count = 0
        self._carried_aborted_count = 0
        self.reset()

    # --- State ------------------------------------------------------------------------

    @property
    def config(self) -> FlightComputerConfig:
        """The settings this flight computer was built with."""
        return self._config

    @property
    def mode(self) -> Mode:
        """The current mode."""
        return self._mode_state.mode

    @property
    def mode_state(self) -> ModeState:
        """The mode state machine's whole state (mode and DOWNLINK return mode)."""
        return self._mode_state

    @property
    def downlink_session(self) -> DownlinkSession | None:
        """The current downlink session (#56): what it has sent so far. ``None``
        outside DOWNLINK."""
        return self._downlink_session

    @property
    def max_frame_size(self) -> int:
        """``L_max`` (ADR-0008 §2): the largest frame this flight software sends, bytes.

        The largest of an ACK/NACK, a TELEMETRY frame, and a DATA frame of
        ``config.downlink.chunk_size_bytes``: 78 with the default 64-byte chunk. A tick
        whose capacity is at least this, with nothing in transmission and nothing held,
        is a whole-frame tick.
        """
        return self._max_frame_size

    @property
    def transmitting(self) -> OutboundProgress | None:
        """The frame in transmission (ADR-0008 §1), or ``None`` when the transmitter is
        idle."""
        tx = self._transmitting
        if tx is None:
            return None
        header = decode_frame(tx.frame)
        return OutboundProgress(header.frame_type, header.sequence, len(tx.frame), tx.sent_bytes)

    @property
    def held_frame_count(self) -> int:
        """ACK/NACK and telemetry frames held behind the frame in transmission."""
        return len(self._held_acks) + (self._held_telemetry is not None)

    @property
    def reboot_pending(self) -> bool:
        """A RESET command's reboot is waiting for its ACK to be transmitted
        (ADR-0008 §6a). Never true in a whole-frame tick, so never at the default
        100 ms tick."""
        return self._pending_reboot is not None

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
        self._carried_suppressed_count = 0
        self._carried_aborted_count = 0
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

        Outbound (ADR-0008 §6): the frame in transmission is aborted and held frames
        are dropped; ACK/NACK and telemetry among them are counted as suppressed, and
        every one as aborted, in this step's output for a RESET command and in the
        next step's output for a call between steps. A reboot pending for a RESET
        command's ACK is cancelled (a ``forced_reset`` reboots once, not twice).

        Args:
            now_us: Simulated time of the reboot, integer microseconds. Uptime counts
                from here.

        Raises:
            TypeError: ``now_us`` is not an int.
            ValueError: ``now_us`` is negative or earlier than the last time seen.
        """
        self._advance_time(now_us)
        self._boot_count += 1
        self._drop_outbound()
        self._clear_transient_state()

    def _clear_transient_state(self) -> None:
        """Forget everything that does not survive a reboot and restart uptime.

        Today that is the mode state (back to BOOT, :data:`INITIAL_STATE`), the
        safe-mode persistence counters (#48), so a flag still set after a reboot must
        be sustained again from BOOT, the downlink sequence counter (#51), so the
        first frame after a reboot has sequence 0, the telemetry schedule (#55), so
        the next frame is the boot-complete beacon, the downlink session (#56), so
        a reboot during a pass aborts it and the next pass resumes from the payload's
        oldest unreleased chunk, and the transmitter (#132): the frame in transmission,
        the held frames, and a pending RESET reboot. Later tickets add their own
        transient state here. The boot counter is not transient.
        """
        self._mode_state = INITIAL_STATE
        self._safety_state = INITIAL_SAFETY_STATE
        self._telemetry_schedule = INITIAL_TELEMETRY_SCHEDULE
        self._boot_time_us = self._now_us
        self._downlink_sequence = 0
        self._downlink_session = None
        self._transmitting = None
        self._held_acks.clear()
        self._held_telemetry = None
        self._reset_ack = _ResetAck.NONE
        self._pending_reboot = None

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

        Before the phases, the transmitter (#132, ADR-0008) decides the kind of tick
        and, in a streaming tick, continues the frame in transmission
        (:meth:`_continue_transmission`, the start of step c).

        **While a RESET command's reboot is pending** (ADR-0008 §6a) the step runs the
        shutting-down path instead: uplink is not decoded but counted in
        ``uplink_ignored_count``, the transmitter finishes the frames ahead of and
        behind the RESET ACK, no flag rule runs and no new frame is created. In the
        step the RESET ACK's last byte goes out (or it is lost because the transmitter
        is off), the kept mode events apply and the flight computer reboots, as the
        update-mode phase does for a RESET in a whole-frame tick. Step f runs in every
        step.

        Returns:
            This tick's delivered downlink frames, the controls for the next tick, the
            bytes transmitted, and the suppressed- and aborted-frame counts.

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
        if (
            self._transmitting is not None
            or self._held_acks
            or self._held_telemetry is not None
            or tick.outbound_remaining_bytes < self._max_frame_size
        ):
            self._continue_transmission(tick)  # else a whole-frame tick: nothing to do
        if self._pending_reboot is None:
            self._decode_uplink(tick)
            self._execute_commands(tick)
            self._evaluate_flags(tick)
            self._update_mode(tick)
            if self._pending_reboot is None:
                self._emit_telemetry(tick)
        else:
            self._shut_down(tick)
        controls = self._produce_controls(tick)
        suppressed = tick.outbound_suppressed_count + self._carried_suppressed_count
        aborted = tick.outbound_aborted_count + self._carried_aborted_count
        self._carried_suppressed_count = self._carried_aborted_count = 0
        return FlightComputerOutput(
            tuple(tick.downlink_frames),
            controls,
            suppressed,
            sent_bytes=tick.sent_bytes,
            outbound_aborted_count=aborted,
            transmitting=None if self._transmitting is None else self.transmitting,
            reboot_pending=self._pending_reboot is not None,
            uplink_ignored_count=tick.uplink_ignored_count,
        )

    def _shut_down(self, tick: TickContext) -> None:
        """A step while a RESET command's reboot is pending (ADR-0008 §6a).

        Uplink is ignored and counted. The transmitter has already continued in
        :meth:`_continue_transmission`; the held telemetry behind the RESET ACK may go
        too. No flag rule runs, no mode event applies, and no telemetry or DATA is
        created. If the RESET ACK has now been transmitted, or can never be (the
        transmitter is off), the kept events apply and the flight computer reboots
        here, in step d (:meth:`_reboot_now`).
        """
        tick.uplink_ignored_count = len(tick.uplink_frames)
        self._pump(tick, OutboundClass.TELEMETRY)
        if self._reset_ack is not _ResetAck.WAITING:
            self._reboot_now(tick)

    def _reboot_now(self, tick: TickContext) -> None:
        """Apply a pending reboot's kept events (ADR-0008 §6a): the reboot in step d of
        the tick the RESET ACK went out (or was lost), then the session follows the
        mode and step e runs in BOOT, exactly as for a RESET in a whole-frame tick."""
        pending = self._pending_reboot
        assert pending is not None
        self._pending_reboot = None
        self._apply_mode_events(tick, pending.events, pending.safe_exit_allowed)
        self._follow_mode_with_session(tick)
        self._emit_telemetry(tick)

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
        ACK reports the command, and telemetry the final mode. A RESET ACK goes out
        before the reboot, numbered in the downlink sequence from before it; commands
        after a RESET in the same tick are answered as in BOOT.

        **Streaming ticks (ADR-0008 §5, §6a).** An ACK/NACK that cannot be committed
        at once waits behind the frame in transmission (bounded), so "same tick" holds
        in every whole-frame tick (always at the default 100 ms) and otherwise the ACK
        goes out after at most one frame in transmission and the ACKs ahead of it. The
        first RESET's ACK is committed right after the frame in transmission, ahead of
        every waiting frame, and the reboot waits for it (:meth:`_update_mode`).
        """
        if not tick.commands and not tick.streaming:
            return  # nothing to answer, nothing held (a whole-frame tick)
        state = self._mode_state
        safe_exit_allowed: bool | None = None  # worked out for the first mode command
        acks: list[bytes] = []
        reset_index: int | None = None  # the first RESET's ACK, in acks
        for received in tick.commands:
            command = received.command
            reason: messages.NackReason | None
            if isinstance(command, messages.MalformedCommand):
                reason = command.reason
            elif command.command_id is messages.CommandId.PING:
                reason = None
            else:
                if safe_exit_allowed is None:
                    safe_exit_allowed = not active_flags(tick.readings)
                event = _mode_event(command)
                result = transition(state, event, safe_exit_allowed=safe_exit_allowed)
                state = result.state
                tick.mode_events.append(event)
                reason = result.reason
                if event.kind is EventKind.RESET and reset_index is None:
                    reset_index = len(acks)
            ack = messages.CommandAck(received.sequence, command.command_id, reason)
            acks.append(messages.encode_ack(ack))

        if tick.streaming and not tick.transmitter_off:
            # ADR-0008 §6a: the RESET ACK goes right after the frame in transmission,
            # ahead of the held ACK/NACK and of this tick's other ACK/NACK, which keep
            # their order behind it. Otherwise held ACK/NACK go first (§4).
            if reset_index is not None:
                self._queue_outbound(
                    tick, FrameType.ACK, acks[reset_index], OutboundClass.ACK, reset_ack=True
                )
            self._pump(tick, OutboundClass.ACK)
        for index, payload in enumerate(acks):
            if index == reset_index and tick.streaming and not tick.transmitter_off:
                continue
            self._queue_outbound(
                tick, FrameType.ACK, payload, OutboundClass.ACK, reset_ack=index == reset_index
            )

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

        In a step that starts in DOWNLINK, appends DOWNLINK_COMPLETE when the downlink
        session has no unsent chunk left in this tick's payload readings (#56). By then
        the payload has released every chunk sent in the previous tick (the release
        applies in step b), so the session is done. It comes after the command and
        safety events, so a command or SAFE/FAULT entry in the same tick has already
        left DOWNLINK and it is ignored. A session entered in this step runs at least
        one step e before it can complete, so the ground sees DOWNLINK in telemetry.
        """
        verdict = evaluate(self._safety_state, tick.readings, self._config.safety)
        self._safety_state = verdict.state
        tick.safety = verdict
        tick.safe_exit_allowed = verdict.safe_exit_allowed
        tick.mode_events.extend(verdict.events)
        mode = self._mode_state.mode
        if mode is Mode.BOOT and boot_complete(self.uptime_us, self._config.boot):
            tick.mode_events.append(ModeEvent(EventKind.BOOT_COMPLETE))
        elif mode is Mode.DOWNLINK:
            session = self._downlink_session or DownlinkSession.start(tick.readings.payload)
            if not unsent_chunk_ids(session, tick.readings.payload):
                tick.mode_events.append(ModeEvent(EventKind.DOWNLINK_COMPLETE))

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

        **RESET waits for its ACK (ADR-0008 §6a).** If the RESET command's ACK has
        been transmitted in this tick (always, in a whole-frame tick where it fits) or
        can never be (suppressed, or the transmitter is off), the RESET reboots here as
        above. Otherwise, in a streaming tick, the reboot is **pending**: the ``RESET``
        and every event after it are kept, in order, and applied in step d of the tick
        its ACK's last byte goes out (see :meth:`step`). Events before it apply now.

        Then the downlink session follows the mode after all the events (#56): a session
        starts at the payload's oldest unreleased chunk when the mode is DOWNLINK and
        none is running, and ends when the mode is anything else (DOWNLINK_COMPLETE, or
        an abort: SET_MODE, ENTER_SAFE_MODE, SAFE_CONDITION, FAULT_DETECTED, RESET). A
        session that leaves and re-enters DOWNLINK within one step keeps running. A DATA
        frame still in transmission when its session ends is aborted; its chunk was
        never released, so the next session sends it first (ADR-0008 §6).
        """
        if tick.mode_events:
            self._apply_mode_events(tick, tick.mode_events, tick.safe_exit_allowed)
        self._follow_mode_with_session(tick)

    def _apply_mode_events(
        self, tick: TickContext, events: list[ModeEvent] | tuple[ModeEvent, ...], guard: bool
    ) -> None:
        """Apply mode events in order (step d), rebooting on RESET or, while the RESET
        ACK is still waiting, keeping the RESET and the events after it for later."""
        rebooted = False
        for index, event in enumerate(events):
            if rebooted and not event.kind.is_command:
                tick.transitions.append(Transition(Outcome.IGNORED, self._mode_state))
                continue
            if (
                event.kind is EventKind.RESET
                and not rebooted
                and self._reset_ack is _ResetAck.WAITING
            ):
                self._pending_reboot = _PendingReboot(tuple(events[index:]), guard)
                return
            result = transition(self._mode_state, event, safe_exit_allowed=guard)
            self._mode_state = result.state
            tick.transitions.append(result)
            if event.kind is EventKind.RESET:
                self.reboot(now_us=tick.now_us)
                rebooted = True

    def _follow_mode_with_session(self, tick: TickContext) -> None:
        """Start or end the downlink session to match the mode after step d (#56)."""
        if self._mode_state.mode is not Mode.DOWNLINK:
            if self._downlink_session is not None:
                self._downlink_session = None
                tx = self._transmitting
                if tx is not None and tx.outbound_class is OutboundClass.DATA:
                    self._transmitting = None  # aborted with its session (ADR-0008 §6)
                    tick.outbound_aborted_count += 1
        elif self._downlink_session is None:
            self._downlink_session = DownlinkSession.start(tick.readings.payload)

    def _emit_telemetry(self, tick: TickContext) -> None:
        """Phase 5 (ADR-0004 step e): emit telemetry if due, and DATA in DOWNLINK.

        **Telemetry (#55).** :func:`~pocketsat.flight.telemetry.telemetry_due` decides
        from the mode after step d and this tick's simulated time: nothing in BOOT; a
        frame in the step the mode changes (the boot-complete beacon is the first frame
        after BOOT); otherwise one per period of the mode (:class:`TelemetryConfig`).
        A due frame is encoded with #54's ``encode_telemetry`` from this tick's
        readings (reported values only, ADR-0004 §7) and the flight computer's own
        state (the mode after step d, ``uptime_ms``, ``boot_count``), and queued with
        ``OutboundClass.TELEMETRY`` through :meth:`_queue_outbound`: after any
        ACK/NACK, within the remaining transmit capacity, with the next downlink
        sequence number. One that does not fit is suppressed and counted, not retried;
        its slot is used, so the next frame is due one period later.

        A NaN reading has no telemetry encoding (``docs/protocol.md``). The
        evaluate-flags phase has already raised FAULT_DETECTED for it in this tick
        (``NON_FINITE_READING``, #48), so the due frame is dropped, uncounted, rather
        than raising out of :meth:`step`.

        **DATA (#56)** then fills what capacity is left (:meth:`_emit_data`).

        **Streaming ticks (ADR-0008 §5).** A telemetry frame held from an earlier tick
        is offered first. A due frame that cannot be committed at once is held behind
        the frame in transmission; if one is still held, the newer frame replaces it
        and the older is counted as suppressed. The schedule is unchanged: a due frame
        uses its slot either way.
        """
        if tick.streaming:
            self._pump(tick, OutboundClass.TELEMETRY)
        self._emit_telemetry_frame(tick)
        self._emit_data(tick)

    def _emit_telemetry_frame(self, tick: TickContext) -> None:
        """Telemetry for :meth:`_emit_telemetry` (#55): one frame if due."""
        due, self._telemetry_schedule = telemetry_due(
            self._telemetry_schedule, self._mode_state.mode, tick.now_us, self._config.telemetry
        )
        if not due:
            return
        state = messages.FlightComputerTelemetryState(
            uptime_ms=self.uptime_ms, mode=self._mode_state.mode, boot_count=self._boot_count
        )
        readings = tick.readings
        try:
            payload = messages.encode_telemetry(
                state,
                power=readings.power,
                thermal=readings.thermal,
                attitude=readings.attitude,
                payload=readings.payload,
                comms=readings.comms,
            )
        except ValueError:
            return
        self._queue_outbound(tick, FrameType.TELEMETRY, payload, OutboundClass.TELEMETRY)

    def _emit_data(self, tick: TickContext) -> None:
        """DATA for :meth:`_emit_telemetry` (#56): the downlink session's frames.

        Only while a session runs (the mode after step d is DOWNLINK), and only while no
        radio-transmit inhibit holds in this tick's readings
        (:func:`~pocketsat.flight.downlink.transmit_inhibited`: a power or thermal
        flag). Then, oldest first, each unsent whole chunk
        (:func:`~pocketsat.flight.downlink.unsent_chunk_ids`) becomes one DATA frame,
        its ID and its content (``chunk_content``, the payload's deterministic content
        function, #43), queued with ``OutboundClass.DATA`` through
        :meth:`_queue_outbound`, after any ACK/NACK and telemetry.

        - **Whole-frame tick:** it stops at the first chunk whose frame does not fit
          what is left of the capacity, so chunks are never sent out of order; that
          chunk is not counted as suppressed and is sent in a later tick.
        - **Streaming tick (ADR-0008 §2, §5):** one chunk at a time, committed only
          while the transmitter is idle, nothing else waits, and bytes are left; a
          frame that does not finish continues in the next tick. The inhibit stops
          new DATA only: a DATA frame in transmission finishes.

        The session records a chunk as sent when its frame **completes**, and step f
        releases it.
        """
        session = self._downlink_session
        if session is None or transmit_inhibited(tick.readings):
            return
        chunk_size = self._config.downlink.chunk_size_bytes
        if tick.streaming:
            self._stream_data(tick, chunk_size)
            return
        chunk_ids = unsent_chunk_ids(session, tick.readings.payload)
        frame_size = MIN_FRAME_SIZE + messages.DATA_CHUNK_ID_SIZE + chunk_size
        last: int | None = None
        for chunk_id in chunk_ids:
            if frame_size > tick.outbound_remaining_bytes:
                break  # don't build a frame that can't go; the next one couldn't either
            chunk = messages.DataChunk(chunk_id, chunk_content(chunk_id, chunk_size))
            if not self._queue_outbound(
                tick, FrameType.DATA, messages.encode_data(chunk), OutboundClass.DATA
            ):
                break
            last = chunk_id
        if last is not None:
            self._downlink_session = session.after_sending(chunk_ids.start, last)

    def _stream_data(self, tick: TickContext, chunk_size: int) -> None:
        """DATA in a streaming tick: commit the next chunk whenever the transmitter is
        idle, nothing else waits and bytes are left (ADR-0008 §2, §5)."""
        while (
            tick.outbound_remaining_bytes > 0
            and self._transmitting is None
            and not self._held_acks
            and self._held_telemetry is None
        ):
            session = self._downlink_session
            assert session is not None
            chunk_ids = unsent_chunk_ids(session, tick.readings.payload)
            if not chunk_ids:
                return
            chunk_id = chunk_ids.start
            chunk = messages.DataChunk(chunk_id, chunk_content(chunk_id, chunk_size))
            if not self._queue_outbound(
                tick,
                FrameType.DATA,
                messages.encode_data(chunk),
                OutboundClass.DATA,
                chunk_id=chunk_id,
            ):
                return

    def _produce_controls(self, tick: TickContext | None = None) -> SpacecraftControls:
        """Phase 6 (ADR-0004 step f): the controls for the next tick.

        The single controls function of ADR-0004 §14: every ``SpacecraftControls`` the
        flight computer produces comes from here, both from :meth:`step` and from
        :meth:`reset`. It returns #47's ``controls_for_mode()`` for the mode after
        step d, with the downlink session's chunk release (#56,
        :func:`~pocketsat.flight.downlink.release_through_chunk_id`): in Phase 1 the
        last chunk whose frame has been transmitted, so the payload releases every sent
        chunk in the next tick.

        **The completion-release corner (ADR-0008 §6).** A DATA frame carried in from
        the previous tick can complete in this tick's step c, and the session can end
        in the same tick's step d (``DOWNLINK_COMPLETE``, a command, a flag, or a
        pending RESET's reboot). The chunk is then released anyway, so it never goes
        down twice. This never happens in a whole-frame tick.
        """
        session = self._downlink_session
        if session is None and tick is not None and tick.completed_chunk_id is not None:
            release: int | None = tick.completed_chunk_id
        else:
            release = release_through_chunk_id(session)
        return controls_for_mode(self._mode_state.mode, release_through_chunk_id=release)

    # --- Outbound -----------------------------------------------------------------------

    def _queue_outbound(
        self,
        tick: TickContext,
        frame_type: FrameType,
        payload: bytes,
        outbound_class: OutboundClass,
        *,
        reset_ack: bool = False,
        chunk_id: int | None = None,
    ) -> bool:
        """Offer one downlink frame to the transmitter.

        The single way any phase sends a frame (ACK/NACK #51, telemetry #55, DATA #56).

        **Whole-frame tick** (ADR-0008 §2): the capacity and priority rule is
        :meth:`TickContext.reserve_outbound`. A frame that fits gets the next downlink
        sequence number (one counter for every frame type, wrapping from 0xFFFF to 0,
        ``docs/protocol.md``), is encoded, and is delivered at once (appended to
        ``tick.downlink_frames``). A frame that does not fit is never encoded and uses
        no sequence number, so a gap in the ground's sequence means a frame lost on
        the way down, not one the spacecraft never sent.

        **Streaming tick:** with the transmitter off, ACK/NACK and telemetry are
        suppressed and counted, DATA is left. Otherwise a frame is **committed** (given
        its sequence number and encoded, then transmitted for what is left of the
        tick) if the transmitter is idle, bytes are left, and nothing of its class or
        higher waits; else ACK/NACK and telemetry are **held** (ADR-0008 §5) and DATA is
        left for later. An ACK/NACK beyond the held bound is suppressed and counted;
        a telemetry frame replaces a held one, which is counted. The RESET ACK
        (``reset_ack``) goes to the front of the held ACK/NACK (ADR-0008 §6a).

        Args:
            tick: This tick's working data.
            frame_type: The frame type.
            payload: The frame payload.
            outbound_class: Its priority class.
            reset_ack: This is the ACK of the tick's first RESET command, which the
                reboot waits for (ADR-0008 §6a).
            chunk_id: For DATA from the downlink session, the chunk it carries: the
                session records it as sent when the frame completes.

        Returns:
            True if the frame was sent, committed, or held; False if it was suppressed
            or, for DATA, left for later.

        Raises:
            RuntimeError: Frames were offered out of :class:`OutboundClass` order.
        """
        if not tick.streaming:
            if not tick.reserve_outbound(MIN_FRAME_SIZE + len(payload), outbound_class):
                if reset_ack:
                    self._reset_ack = _ResetAck.LOST
                return False
            tick.downlink_frames.append(
                encode_frame_fields(frame_type, self._downlink_sequence, payload)
            )
            self._downlink_sequence = (self._downlink_sequence + 1) & MAX_SEQUENCE
            if reset_ack:
                self._reset_ack = _ResetAck.DONE
            if chunk_id is not None:
                self._chunk_sent(tick, chunk_id)
            return True

        tick.check_order(outbound_class)
        if tick.transmitter_off:
            if outbound_class.counts_suppression:
                tick.outbound_suppressed_count += 1
            if reset_ack:
                self._reset_ack = _ResetAck.LOST
            return False
        idle = self._transmitting is None and tick.outbound_remaining_bytes > 0
        if outbound_class is OutboundClass.ACK:
            held = self._held_acks
            if reset_ack:
                if idle or len(held) < self._max_held_acks:
                    self._reset_ack = _ResetAck.WAITING
                    held.appendleft(_HeldAck(payload, reset_ack=True))
                    self._pump(tick, OutboundClass.ACK)
                    return True
                self._reset_ack = _ResetAck.LOST
            elif idle and not held:
                self._commit(tick, frame_type, payload, outbound_class)
                return True
            elif len(held) < self._max_held_acks:
                held.append(_HeldAck(payload))
                return True
            tick.outbound_suppressed_count += 1
            return False
        if outbound_class is OutboundClass.TELEMETRY:
            if self._held_telemetry is not None:  # replaced by the newer frame
                self._held_telemetry = None
                tick.outbound_suppressed_count += 1
            if idle and not self._held_acks:
                self._commit(tick, frame_type, payload, outbound_class)
            else:
                self._held_telemetry = payload
            return True
        if idle and not self._held_acks and self._held_telemetry is None:
            self._commit(tick, frame_type, payload, outbound_class, chunk_id=chunk_id)
            return True
        return False

    def _commit(
        self,
        tick: TickContext,
        frame_type: FrameType,
        payload: bytes,
        outbound_class: OutboundClass,
        *,
        reset_ack: bool = False,
        chunk_id: int | None = None,
    ) -> None:
        """Commit a frame (ADR-0008 §2, §3): give it the next downlink sequence number,
        encode it, make it the frame in transmission, and transmit what the tick has
        left for it. The transmitter must be idle with bytes left."""
        frame = encode_frame_fields(frame_type, self._downlink_sequence, payload)
        self._downlink_sequence = (self._downlink_sequence + 1) & MAX_SEQUENCE
        self._transmitting = _InTransmission(
            frame, outbound_class, chunk_id=chunk_id, reset_ack=reset_ack
        )
        self._transmit(tick)

    def _transmit(self, tick: TickContext) -> None:
        """Send what the tick has left of the frame in transmission; deliver it
        (ADR-0008 §3) if its last byte went out."""
        tx = self._transmitting
        assert tx is not None
        size = len(tx.frame)
        count = min(size - tx.sent_bytes, tick.outbound_remaining_bytes)
        tx.sent_bytes += count
        tick.outbound_remaining_bytes -= count
        tick.sent_bytes += count
        if tx.sent_bytes < size:
            return
        self._transmitting = None
        tick.downlink_frames.append(tx.frame)
        if tx.reset_ack:
            self._reset_ack = _ResetAck.DONE
        if tx.chunk_id is not None:
            self._chunk_sent(tick, tx.chunk_id)

    def _chunk_sent(self, tick: TickContext, chunk_id: int) -> None:
        """The session records a chunk as sent once its whole frame has been
        transmitted (ADR-0008 §6); step f releases it."""
        session = self._downlink_session
        if session is not None:
            self._downlink_session = session.after_sending(chunk_id, chunk_id)
        tick.completed_chunk_id = chunk_id

    def _pump(self, tick: TickContext, upto: OutboundClass) -> None:
        """Stream the transmitter (ADR-0008 §2): continue the frame in transmission,
        then commit waiting frames of class ``upto`` or higher in priority order (held
        ACK/NACK oldest first, then the held telemetry frame), while bytes are left."""
        while tick.outbound_remaining_bytes > 0:
            if self._transmitting is not None:
                self._transmit(tick)
            elif self._held_acks:
                held = self._held_acks.popleft()
                self._commit(
                    tick, FrameType.ACK, held.payload, OutboundClass.ACK, reset_ack=held.reset_ack
                )
            elif upto >= OutboundClass.TELEMETRY and self._held_telemetry is not None:
                payload, self._held_telemetry = self._held_telemetry, None
                self._commit(tick, FrameType.TELEMETRY, payload, OutboundClass.TELEMETRY)
            else:
                return

    def _continue_transmission(self, tick: TickContext) -> None:
        """The start of step c (ADR-0008 §2, §6): decide the kind of tick and stream.

        - **Transmitter off** (capacity 0 and ``transmitter_on`` false): the frame in
          transmission is aborted and every held frame dropped (counted), and the
          tick's own ACK/NACK and telemetry will be suppressed. A pending RESET reboot
          then happens in this tick, because its ACK can no longer go out.
        - **Whole-frame tick** (capacity at least :attr:`max_frame_size`, nothing in
          transmission, nothing held): nothing to do; today's rule applies.
        - **Streaming tick:** the frame in transmission continues. Held ACK/NACK are
          committed by the execute-commands phase, after a RESET command's ACK if the
          tick has one (ADR-0008 §6a), so the RESET ACK follows this frame directly.

        A transmitter that is on with a capacity of 0 (a tick shorter than one byte's
        airtime) is a streaming tick in which nothing moves: frames wait.
        """
        comms = tick.readings.comms
        capacity = comms.transmit_capacity_bytes
        if capacity == 0 and not comms.transmitter_on:
            tick.streaming = tick.transmitter_off = True
            if self._transmitting is not None or self._held_acks or self._held_telemetry:
                self._drop_outbound()
            return
        if (
            capacity >= self._max_frame_size
            and self._transmitting is None
            and not self._held_acks
            and self._held_telemetry is None
        ):
            return
        tick.streaming = True
        if self._transmitting is not None:
            self._transmit(tick)

    def _drop_outbound(self) -> None:
        """Abort the frame in transmission and drop every held frame (ADR-0008 §6).

        Each is counted as aborted, and ACK/NACK and telemetry also as suppressed; the
        counts go into the output of the current step (or the next, for a reboot
        between steps). A DATA frame's chunk was never released, so it stays buffered.
        A RESET ACK among them is lost, so a pending reboot happens at once.
        """
        aborted = suppressed = 0
        tx = self._transmitting
        if tx is not None:
            aborted += 1
            suppressed += tx.outbound_class.counts_suppression
            if tx.reset_ack:
                self._reset_ack = _ResetAck.LOST
            self._transmitting = None
        for held in self._held_acks:
            aborted += 1
            suppressed += 1
            if held.reset_ack:
                self._reset_ack = _ResetAck.LOST
        self._held_acks.clear()
        if self._held_telemetry is not None:
            aborted += 1
            suppressed += 1
            self._held_telemetry = None
        self._carried_aborted_count += aborted
        self._carried_suppressed_count += suppressed


def _mode_event(command: "messages.ParsedCommand") -> ModeEvent:
    """The mode event a command raises: the :class:`EventKind` of the same name.

    Every :class:`~pocketsat.messages.CommandId` except PING names a command event
    (``docs/protocol.md``, "Command IDs"); SET_MODE carries its target mode.
    """
    return ModeEvent(EventKind[command.command_id.name], command.target)
