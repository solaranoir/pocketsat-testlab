"""The ``TestTarget`` boundary (ADR-0002).

The orchestrator depends only on the types in this module. SIL and HIL targets
implement :class:`TestTarget`; nothing above this boundary knows which one it has.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Protocol, runtime_checkable

FaultParam = bool | int | float | str
"""Allowed value types for :attr:`TargetFault.params`."""


@dataclass(frozen=True)
class TargetCapabilities:
    """What a target can do, declared up front so scenarios can be checked before a run.

    Attributes:
        deterministic: Same scenario and seed give bit-for-bit identical runs (SIL: True).
        real_time: ``advance(dt)`` takes ``dt`` of wall-clock time (HIL: True).
        supported_faults: Fault types accepted by :meth:`TestTarget.inject`.
    """

    deterministic: bool
    real_time: bool
    supported_faults: frozenset[str] = frozenset()


@dataclass(frozen=True)
class EnvironmentState:
    """Per-tick environment inputs produced on the orchestrator side.

    Minimal for now; thermal input, sensor noise and bias, and battery overrides are
    added in later phases.

    Attributes:
        sunlit: True in sunlight, False in eclipse.
    """

    sunlit: bool = True


@dataclass(frozen=True)
class TargetFault:
    """A fault delivered to a target through :meth:`TestTarget.inject`.

    Attributes:
        fault_type: Fault type name; must be in the target's
            ``capabilities.supported_faults``.
        params: Fault-specific parameters. Stored as a read-only mapping.
        duration_s: How long the fault lasts in simulated seconds, or ``None`` for a
            fault that persists until reset.
    """

    fault_type: str
    params: Mapping[str, FaultParam] = field(default_factory=dict)
    duration_s: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "params", MappingProxyType(dict(self.params)))


@runtime_checkable
class TestTarget(Protocol):
    """Uniform bytes-in/bytes-out interface to the spacecraft under test.

    Attributes:
        capabilities: Static description of what this target supports.
    """

    capabilities: TargetCapabilities

    def connect(self) -> None:
        """Open the connection to the target (a no-op for in-process targets)."""
        ...

    def reset(self, seed: int) -> None:
        """Reset the target to its initial state, seeding any randomness with ``seed``."""
        ...

    def send(self, frame: bytes) -> None:
        """Deliver one encoded uplink frame to the target."""
        ...

    def receive(self) -> list[bytes]:
        """Return the downlink frames produced since the last call, oldest first.

        Never blocks: frames produced so far are drained and returned, possibly none.
        """
        ...

    def apply_environment(self, env: EnvironmentState) -> None:
        """Set the environment inputs the target sees from now on."""
        ...

    def inject(self, fault: TargetFault) -> None:
        """Apply a fault. ``fault.fault_type`` must be in ``capabilities.supported_faults``."""
        ...

    def advance(self, dt: float) -> None:
        """Advance target time by ``dt`` seconds (instantly in SIL, in real time in HIL)."""
        ...

    def close(self) -> None:
        """Release the target's resources. The target is unusable afterwards."""
        ...
