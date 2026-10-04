"""Flight computer: modes, and later command handling and telemetry (epic #45).

The flight computer is part of the SIL spacecraft model but outside ``STEP_ORDER``: it
runs after every subsystem each tick and produces the controls for the next one
(ADR-0004 §2). :class:`FlightComputer` (#101) is the component, with the single
``step()`` entry point; :mod:`pocketsat.flight.modes` (#47) is its mode state machine,
and :mod:`pocketsat.flight.safety` (#48) its automatic safe-mode and fault entry rules
(import it from that module).

Import order: :mod:`pocketsat.messages` imports :class:`Mode` from
:mod:`pocketsat.flight.modes`, which imports only :mod:`pocketsat.spacecraft.controls`,
so ``modes`` must stay free of ``pocketsat.messages``. Because importing
``pocketsat.flight.modes`` first runs this package's ``__init__`` (and so
``computer``), a module in this package that needs the telemetry codec (#55) imports
the module, ``from pocketsat import messages``, and reads its names at call time, so
``import pocketsat.messages`` still works when it comes first.
``tests/unit/test_flight_computer.py`` imports each module first in a fresh interpreter.
"""

from pocketsat.flight.computer import (
    PHASE_ORDER,
    FlightComputer,
    FlightComputerOutput,
    SpacecraftReadings,
)
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
    "PHASE_ORDER",
    "RETURN_MODES",
    "EventKind",
    "FlightComputer",
    "FlightComputerOutput",
    "Mode",
    "ModeEvent",
    "ModeState",
    "Outcome",
    "RejectReason",
    "SpacecraftReadings",
    "Transition",
    "controls_for_mode",
    "transition",
]
