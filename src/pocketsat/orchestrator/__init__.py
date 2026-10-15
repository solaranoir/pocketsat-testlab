"""The orchestrator: runs the environment, the ground station, the link and the target.

Phase 2 brings its run loop forward (ADR-0010 §7, #167); Phase 5 adds scenario loading,
the fault schedule and assertions to it. The orchestrator depends on
:class:`~pocketsat.targets.base.TestTarget` only, never on a concrete target (AGENTS.md
rule 1).
"""

from pocketsat.orchestrator.loop import (
    IssuedCommand,
    RunLoop,
    RunResult,
    ScheduledCommand,
    TickResult,
    station_geometry,
    station_location,
)

__all__ = [
    "IssuedCommand",
    "RunLoop",
    "RunResult",
    "ScheduledCommand",
    "TickResult",
    "station_geometry",
    "station_location",
]
