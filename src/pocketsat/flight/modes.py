"""Mode state machine, transition table, and mode-to-controls mapping (#47).

The flight computer is in exactly one :class:`Mode` at a time. Mode changes happen only
through :func:`transition`, which takes the current :class:`ModeState` and one
:class:`ModeEvent` and returns a :class:`Transition`: the outcome, the new state, and,
for a rejected command, a :class:`RejectReason` for the NACK.

This module defines the events and the table only. It does not decide *when* an event
happens:

- command events (``SET_MODE``, ``BEGIN_DOWNLINK``, ``ENTER_SAFE_MODE``, ``RESET``) come
  from the command dispatcher (#51) after payload decoding and validation (#52);
- ``BOOT_COMPLETE`` and the reboot behind ``RESET`` come from the boot sequence (#49);
- ``SAFE_CONDITION`` and ``FAULT_DETECTED``, and the "flags have cleared" guard for
  leaving SAFE, come from the automatic safe-mode and fault rules (#48);
- ``DOWNLINK_COMPLETE`` comes from the downlink session (#56).

Each mode's entry action is the set of :class:`SpacecraftControls` returned by
:func:`controls_for_mode`, which apply from the next tick (ADR-0004 §2). There are no
other entry or exit actions here.

See ``docs/spacecraft-modes.md`` for the table, the state diagram, and the rationale.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum, IntEnum
from types import MappingProxyType
from typing import Final

from pocketsat.spacecraft.controls import (
    AttitudeControls,
    PayloadControls,
    RadioControls,
    RadioMode,
    SpacecraftControls,
)


class Mode(IntEnum):
    """Flight computer mode.

    The numeric values are the wire encoding: telemetry (#54) carries the mode as a
    uint8 with exactly these numbers, and so does the ``SET_MODE`` argument (#52). They
    are part of the firmware contract and must never be renumbered or reused.
    """

    BOOT = 0
    """Power-on and after RESET. Only RESET (and PING, #51) are accepted."""

    NOMINAL = 1
    """Housekeeping: payload off, attitude control on."""

    SCIENCE = 2
    """Payload acquiring."""

    DOWNLINK = 3
    """Sending stored payload data (#56); returns to NOMINAL or SCIENCE when done."""

    SAFE = 4
    """Survival: payload off, attitude control on (sun-safe), radio on for recovery."""

    FAULT = 5
    """Internal consistency failure (#48). Only RESET leaves it."""


COMMANDABLE_TARGETS: Final = frozenset({Mode.NOMINAL, Mode.SCIENCE})
"""The only modes ``SET_MODE`` may request. DOWNLINK and SAFE have their own commands,
BOOT is reached by RESET, and FAULT is never commanded."""

RETURN_MODES: Final = frozenset({Mode.NOMINAL, Mode.SCIENCE})
"""Modes DOWNLINK can be entered from, and so returns to."""


class EventKind(Enum):
    """What happened. Command events can be rejected; automatic events are ignored when
    they don't apply."""

    SET_MODE = "set_mode"
    """Command (#51, #52): change to ``ModeEvent.target`` (NOMINAL or SCIENCE)."""

    BEGIN_DOWNLINK = "begin_downlink"
    """Command (#51): start a downlink session (#56)."""

    ENTER_SAFE_MODE = "enter_safe_mode"
    """Command (#51): enter SAFE."""

    RESET = "reset"
    """Command (#51) or the ``forced_reset`` fault (#60): reboot into BOOT (#49)."""

    BOOT_COMPLETE = "boot_complete"
    """Automatic (#49): the configured boot duration has elapsed."""

    SAFE_CONDITION = "safe_condition"
    """Automatic (#48): a safe-mode flag has been sustained for the configured ticks."""

    FAULT_DETECTED = "fault_detected"
    """Automatic (#48): an internal consistency failure."""

    DOWNLINK_COMPLETE = "downlink_complete"
    """Automatic (#56): no unsent chunks remain."""

    @property
    def is_command(self) -> bool:
        """True for events that come from a ground command and get an ACK or NACK."""
        return self in _COMMAND_KINDS


_COMMAND_KINDS: Final = frozenset(
    {
        EventKind.SET_MODE,
        EventKind.BEGIN_DOWNLINK,
        EventKind.ENTER_SAFE_MODE,
        EventKind.RESET,
    }
)


@dataclass(frozen=True)
class ModeEvent:
    """One event for the state machine.

    Attributes:
        kind: What happened.
        target: The requested mode, for ``SET_MODE`` only; ``None`` for every other kind.
            Any :class:`Mode` is representable so the table can reject the ones that
            aren't commandable; a byte that is not a mode at all is #52's to reject.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``target`` is missing for ``SET_MODE`` or given for another kind.
    """

    kind: EventKind
    target: Mode | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, EventKind):
            raise TypeError(f"kind must be an EventKind, got {self.kind!r}")
        if self.target is not None and not isinstance(self.target, Mode):
            raise TypeError(f"target must be a Mode or None, got {self.target!r}")
        if (self.kind is EventKind.SET_MODE) != (self.target is not None):
            raise ValueError(f"target is required for SET_MODE and only for it, got {self!r}")


ALL_EVENTS: Final[tuple[ModeEvent, ...]] = (
    *(ModeEvent(EventKind.SET_MODE, target) for target in Mode),
    *(ModeEvent(kind) for kind in EventKind if kind is not EventKind.SET_MODE),
)
"""Every distinct event: ``SET_MODE`` once per target mode, then each other kind."""


@dataclass(frozen=True)
class ModeState:
    """The state machine's whole state.

    Attributes:
        mode: The current mode.
        return_mode: In DOWNLINK, the mode it was entered from (NOMINAL or SCIENCE) and
            returns to on ``DOWNLINK_COMPLETE``. ``None`` in every other mode.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: ``return_mode`` is missing in DOWNLINK, given outside it, or not
            NOMINAL or SCIENCE.
    """

    mode: Mode
    return_mode: Mode | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.mode, Mode):
            raise TypeError(f"mode must be a Mode, got {self.mode!r}")
        if self.return_mode is not None and not isinstance(self.return_mode, Mode):
            raise TypeError(f"return_mode must be a Mode or None, got {self.return_mode!r}")
        if self.mode is Mode.DOWNLINK:
            if self.return_mode not in RETURN_MODES:
                raise ValueError(
                    f"DOWNLINK needs a return_mode of NOMINAL or SCIENCE, got {self.return_mode!r}"
                )
        elif self.return_mode is not None:
            raise ValueError(f"return_mode is only used in DOWNLINK, got {self!r}")


INITIAL_STATE: Final = ModeState(Mode.BOOT)
"""The state after power-on and after RESET (ADR-0005 §5)."""


class Outcome(Enum):
    """How the state machine handled an event."""

    TRANSITION = "transition"
    """The mode was entered (for RESET in BOOT, re-entered: the boot restarts). The new
    mode's controls apply from the next tick. A command gets an ACK."""

    NO_CHANGE = "no_change"
    """A command that asks for what is already true (for example SET_MODE to the
    current mode). Accepted and ACKed, so a ground retry after a lost ACK succeeds."""

    REJECTED = "rejected"
    """A command that is not allowed in this mode. Gets a NACK with the reason. Only
    command events are ever rejected."""

    IGNORED = "ignored"
    """An automatic event that has no effect in this mode. Nothing is sent."""


class RejectReason(IntEnum):
    """Why a command was rejected, as carried in the NACK payload (#51, #52).

    Values are uint8 wire codes. 0x00 is never a NACK reason: it is the reason byte of
    an ACK (``pocketsat.messages.ACK_REASON``). 0x01-0x0F are #52's decoding and
    argument errors (``pocketsat.messages.DecodeReason``: unknown command, truncated or
    over-long payload, invalid argument), checked before the mode; mode reasons use
    0x10-0x1F. Codes are never renumbered or reused.
    """

    BOOT_IN_PROGRESS = 0x10
    """In BOOT, only RESET (and PING, #51) are accepted."""

    FAULT_REQUIRES_RESET = 0x11
    """In FAULT, only RESET is accepted."""

    NOT_ALLOWED_IN_SAFE = 0x12
    """In SAFE, the only way out is SET_MODE NOMINAL (or RESET); SCIENCE and DOWNLINK
    must wait until the spacecraft is back in NOMINAL."""

    SAFE_CONDITIONS_ACTIVE = 0x13
    """SET_MODE NOMINAL from SAFE while the triggering flags have not cleared (#48)."""

    TARGET_NOT_COMMANDABLE = 0x14
    """SET_MODE asked for BOOT, DOWNLINK, SAFE, or FAULT. Use RESET, BEGIN_DOWNLINK, or
    ENTER_SAFE_MODE; FAULT is never commanded."""


@dataclass(frozen=True)
class Transition:
    """Result of one event.

    Attributes:
        outcome: How the event was handled.
        state: The state after the event (unchanged unless ``outcome`` is TRANSITION).
        reason: Set exactly when ``outcome`` is REJECTED.
    """

    outcome: Outcome
    state: ModeState
    reason: RejectReason | None = None

    def __post_init__(self) -> None:
        if (self.outcome is Outcome.REJECTED) != (self.reason is not None):
            raise ValueError(f"reason is required for REJECTED and only for it, got {self!r}")

    @property
    def mode(self) -> Mode:
        """The mode after the event."""
        return self.state.mode

    @property
    def acknowledged(self) -> bool:
        """For a command event: True for an ACK, False for a NACK."""
        return self.outcome in (Outcome.TRANSITION, Outcome.NO_CHANGE)


def transition(state: ModeState, event: ModeEvent, *, safe_exit_allowed: bool) -> Transition:
    """Apply one event to the state machine (the transition table).

    Pure and deterministic: the same inputs always give the same result.

    Rules are checked in this order, so the reason code is the first that applies:

    1. ``RESET`` always enters BOOT (re-enters it from BOOT).
    2. ``FAULT_DETECTED`` enters FAULT from every other mode.
    3. In FAULT, every other event is rejected (commands) or ignored.
    4. ``SAFE_CONDITION`` enters SAFE from every other mode, BOOT included.
    5. In BOOT, ``BOOT_COMPLETE`` enters NOMINAL; other commands are rejected.
    6. ``SET_MODE`` to a mode other than NOMINAL or SCIENCE is rejected.
    7. The per-mode rules for NOMINAL, SCIENCE, DOWNLINK, and SAFE.

    Args:
        state: The current state.
        event: The event to apply.
        safe_exit_allowed: Whether the flags that put the spacecraft in SAFE have
            cleared, so SET_MODE NOMINAL may leave SAFE. Computed by #48; only read for
            SET_MODE NOMINAL in SAFE.

    Returns:
        The outcome, the new state, and the reason for a rejected command.
    """
    mode = state.mode
    kind = event.kind

    def enter(new: Mode, return_mode: Mode | None = None) -> Transition:
        return Transition(Outcome.TRANSITION, ModeState(new, return_mode))

    def unchanged() -> Transition:
        return Transition(Outcome.NO_CHANGE, state)

    def reject(reason: RejectReason) -> Transition:
        return Transition(Outcome.REJECTED, state, reason)

    def ignore() -> Transition:
        return Transition(Outcome.IGNORED, state)

    def refuse(reason: RejectReason) -> Transition:
        # A command is rejected with a reason; an automatic event is ignored.
        return reject(reason) if kind.is_command else ignore()

    if kind is EventKind.RESET:
        return enter(Mode.BOOT)
    if kind is EventKind.FAULT_DETECTED:
        return ignore() if mode is Mode.FAULT else enter(Mode.FAULT)
    if mode is Mode.FAULT:
        return refuse(RejectReason.FAULT_REQUIRES_RESET)
    if kind is EventKind.SAFE_CONDITION:
        return ignore() if mode is Mode.SAFE else enter(Mode.SAFE)
    if mode is Mode.BOOT:
        if kind is EventKind.BOOT_COMPLETE:
            return enter(Mode.NOMINAL)
        return refuse(RejectReason.BOOT_IN_PROGRESS)
    if kind is EventKind.BOOT_COMPLETE:
        return ignore()

    if kind is EventKind.SET_MODE:
        target = event.target
        assert target is not None  # guaranteed by ModeEvent
        if target not in COMMANDABLE_TARGETS:
            return reject(RejectReason.TARGET_NOT_COMMANDABLE)
        if mode is Mode.SAFE:
            if target is not Mode.NOMINAL:
                return reject(RejectReason.NOT_ALLOWED_IN_SAFE)
            if not safe_exit_allowed:
                return reject(RejectReason.SAFE_CONDITIONS_ACTIVE)
        return unchanged() if target is mode else enter(target)

    if kind is EventKind.ENTER_SAFE_MODE:
        return unchanged() if mode is Mode.SAFE else enter(Mode.SAFE)

    if kind is EventKind.BEGIN_DOWNLINK:
        if mode is Mode.SAFE:
            return reject(RejectReason.NOT_ALLOWED_IN_SAFE)
        if mode is Mode.DOWNLINK:
            return unchanged()
        return enter(Mode.DOWNLINK, return_mode=mode)

    # DOWNLINK_COMPLETE is the only kind left.
    if mode is Mode.DOWNLINK:
        assert state.return_mode is not None  # guaranteed by ModeState
        return enter(state.return_mode)
    return ignore()


_MODE_CONTROLS: Final[Mapping[Mode, SpacecraftControls]] = MappingProxyType(
    {
        Mode.BOOT: SpacecraftControls(
            payload=PayloadControls(enabled=False),
            radio=RadioControls(RadioMode.RX_TX),
            attitude=AttitudeControls(enabled=False),
        ),
        Mode.NOMINAL: SpacecraftControls(
            payload=PayloadControls(enabled=False),
            radio=RadioControls(RadioMode.RX_TX),
            attitude=AttitudeControls(enabled=True),
        ),
        Mode.SCIENCE: SpacecraftControls(
            payload=PayloadControls(enabled=True),
            radio=RadioControls(RadioMode.RX_TX),
            attitude=AttitudeControls(enabled=True),
        ),
        Mode.DOWNLINK: SpacecraftControls(
            payload=PayloadControls(enabled=False),
            radio=RadioControls(RadioMode.RX_TX),
            attitude=AttitudeControls(enabled=True),
        ),
        Mode.SAFE: SpacecraftControls(
            payload=PayloadControls(enabled=False),
            radio=RadioControls(RadioMode.RX_TX),
            attitude=AttitudeControls(enabled=True),
        ),
        Mode.FAULT: SpacecraftControls(
            payload=PayloadControls(enabled=False),
            radio=RadioControls(RadioMode.RX_TX),
            attitude=AttitudeControls(enabled=True),
        ),
    }
)


def controls_for_mode(
    mode: Mode, *, release_through_chunk_id: int | None = None
) -> SpacecraftControls:
    """Return the controls the flight computer produces for the next tick in ``mode``.

    This is the single controls function of ADR-0004 §14: every mode's entry action is
    expressed here and nowhere else.

    - Radio ``RX_TX`` in every mode, so beacons, telemetry, and ACK/NACK can always go
      out (ADR-0004 §10). ``RX_ONLY`` and ``OFF`` are unused: with the idle transmitter
      at 0.15 W (#72) there is no energy case for them, ``RX_ONLY`` would hide the
      spacecraft's state from the ground, and ``OFF`` would make RESET impossible to
      receive. The ``transmitter_off`` fault still reaches comms as an override (#59).
    - Attitude control on in every mode except BOOT, so SAFE and FAULT hold sun-safe
      pointing and keep generating power.
    - Payload enabled only in SCIENCE; off in DOWNLINK (see docs/spacecraft-modes.md).
    - No fault overrides (``SilTarget`` merges those, #59).
    - The chunk release (#56): ``release_through_chunk_id`` as given, ``None`` (release
      nothing) by default. The flight computer passes the downlink session's release
      rule (:func:`pocketsat.flight.downlink.release_through_chunk_id`), so the release
      comes from this function like every other control.

    Args:
        mode: The mode after this tick's update (step d of ADR-0004 §2).
        release_through_chunk_id: Release every stored chunk up to and including this
            ID in the next tick, or ``None``.

    Returns:
        The controls for the next tick. Without a release, the same object for every
        call in ``mode``.

    Raises:
        TypeError: ``release_through_chunk_id`` is not an int or ``None``.
        ValueError: ``release_through_chunk_id`` is negative.
    """
    controls = _MODE_CONTROLS[mode]
    if release_through_chunk_id is None:
        return controls
    # Built field by field rather than with dataclasses.replace: this runs in every tick
    # of a DOWNLINK pass (#78's budget). The mode records carry no fault overrides.
    return SpacecraftControls(
        payload=PayloadControls(
            enabled=controls.payload.enabled, release_through_chunk_id=release_through_chunk_id
        ),
        radio=controls.radio,
        attitude=controls.attitude,
    )
