"""Flight computer: modes, and later command handling and telemetry (epic #45).

The flight computer is part of the SIL spacecraft model but outside ``STEP_ORDER``: it
runs after every subsystem each tick and produces the controls for the next one
(ADR-0004 §2).
"""

from pocketsat.flight.modes import (
    ALL_EVENTS,
    COMMANDABLE_TARGETS,
    INITIAL_STATE,
    RETURN_MODES,
    EventKind,
    Mode,
    ModeEvent,
    ModeState,
    Outcome,
    RejectReason,
    Transition,
    controls_for_mode,
    transition,
)

__all__ = [
    "ALL_EVENTS",
    "COMMANDABLE_TARGETS",
    "INITIAL_STATE",
    "RETURN_MODES",
    "EventKind",
    "Mode",
    "ModeEvent",
    "ModeState",
    "Outcome",
    "RejectReason",
    "Transition",
    "controls_for_mode",
    "transition",
]
