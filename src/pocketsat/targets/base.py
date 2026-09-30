"""The ``TestTarget`` boundary (ADR-0002).

The orchestrator depends only on the types in this module. SIL and HIL targets
implement :class:`TestTarget`; nothing above this boundary knows which one it has.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Protocol, runtime_checkable

from pocketsat.core.clock import check_us

FaultParam = bool | int | float | str
"""Allowed value types for :attr:`TargetFault.params`."""


class UnsupportedFaultError(ValueError):
    """Raised by :meth:`TestTarget.inject` for a fault type the target does not support."""

    def __init__(self, fault_type: str, supported: frozenset[str]) -> None:
        self.fault_type = fault_type
        self.supported = supported
        listed = ", ".join(sorted(supported)) or "none"
        super().__init__(f"unsupported fault type {fault_type!r}; target supports: {listed}")


@dataclass(frozen=True)
class TargetCapabilities:
    """What a target can do, declared up front so scenarios can be checked before a run.

    Attributes:
        deterministic: Same scenario and seed give bit-for-bit identical runs (SIL: True).
        real_time: ``advance(dt_us)`` takes ``dt_us`` of wall-clock time (HIL: True).
        supported_faults: Fault types accepted by :meth:`TestTarget.inject`.
    """

    deterministic: bool
    real_time: bool
    supported_faults: frozenset[str] = frozenset()


ABSOLUTE_ZERO_C = -273.15
"""Absolute zero in degrees Celsius."""


def _real(name: str, value: float) -> float:
    """Return ``value`` as a float, rejecting bools, non-numbers, NaN, and infinities."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value}")
    return float(value)


@dataclass(frozen=True)
class EnvironmentState:
    """Per-tick environment inputs produced on the orchestrator side.

    The defaults describe a nominal environment: sunlit, mild temperature, nominal
    sensor noise, and no battery override.

    Attributes:
        sunlit: True in sunlight, False in eclipse.
        ambient_temp_c: Effective ambient temperature driving the thermal model, in
            degrees Celsius. Must be finite and above absolute zero.
        sensor_noise_scale: Multiplier on every subsystem's nominal sensor noise:
            ``1.0`` is nominal, ``0.0`` is noiseless. Must be finite and non-negative.
        battery_soc_override: If set, forces the battery state of charge to this value
            (``0.0`` to ``1.0``) for fault and edge-case scenarios; ``None`` leaves the
            power model in control.

    Raises:
        TypeError: A field has the wrong type.
        ValueError: A field is out of range.
    """

    sunlit: bool = True
    ambient_temp_c: float = 20.0
    sensor_noise_scale: float = 1.0
    battery_soc_override: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.sunlit, bool):
            raise TypeError(f"sunlit must be a bool, got {self.sunlit!r}")
        temp = _real("ambient_temp_c", self.ambient_temp_c)
        if temp <= ABSOLUTE_ZERO_C:
            raise ValueError(f"ambient_temp_c must be above absolute zero, got {temp}")
        if _real("sensor_noise_scale", self.sensor_noise_scale) < 0:
            raise ValueError(
                f"sensor_noise_scale must be non-negative, got {self.sensor_noise_scale}"
            )
        if self.battery_soc_override is not None:
            soc = _real("battery_soc_override", self.battery_soc_override)
            if not 0.0 <= soc <= 1.0:
                raise ValueError(f"battery_soc_override must be in 0..1, got {soc}")


@dataclass(frozen=True)
class TargetFault:
    """A fault delivered to a target through :meth:`TestTarget.inject`.

    Attributes:
        fault_type: Fault type name; must be in the target's
            ``capabilities.supported_faults``.
        params: Fault-specific parameters. Stored as a read-only mapping.
        duration_us: How long the fault lasts in simulated microseconds, or ``None`` for
            a fault that persists until reset. Scenario durations in seconds are converted
            when the scenario loads (see :func:`pocketsat.core.clock.seconds_to_ticks`).

    Raises:
        TypeError: ``duration_us`` is not an int or ``None``.
        ValueError: ``duration_us`` is negative.
    """

    fault_type: str
    params: Mapping[str, FaultParam] = field(default_factory=dict)
    duration_us: int | None = None

    def __post_init__(self) -> None:
        if self.duration_us is not None:
            check_us("duration_us", self.duration_us)
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
        """Apply a fault.

        Raises:
            UnsupportedFaultError: ``fault.fault_type`` is not in
                ``capabilities.supported_faults``.
        """
        ...

    def advance(self, dt_us: int) -> None:
        """Advance target time by ``dt_us`` microseconds (ADR-0003).

        Instant in SIL; in HIL, blocks until ``dt_us`` of real time has passed.
        """
        ...

    def close(self) -> None:
        """Release the target's resources. The target is unusable afterwards."""
        ...
