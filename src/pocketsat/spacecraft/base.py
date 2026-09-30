"""Subsystem framework for the SIL spacecraft model.

Every spacecraft subsystem (power, thermal, attitude, payload, communications)
implements :class:`Subsystem`. A :class:`SubsystemStack` resets and steps them in the
fixed order :data:`STEP_ORDER` and aggregates their snapshots into a
:class:`SpacecraftState` for telemetry and for other subsystems to read.

Time is integer microseconds and randomness comes only from named ``RngFactory``
streams (ADR-0003).
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Final, Protocol, runtime_checkable

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.targets.base import EnvironmentState

STEP_ORDER: Final[tuple[str, ...]] = ("power", "thermal", "attitude", "payload", "comms")
"""The order subsystems are stepped in each tick. This is the only place it is defined.

1. ``power``: bus state first; every other subsystem runs on it.
2. ``thermal``: heat from this tick's electrical loads and the environment.
3. ``attitude``: pointing, which the payload and radio depend on.
4. ``payload``: collects data given the current pointing and power.
5. ``comms``: last, so it can transmit what earlier subsystems produced this tick.

Changing this order changes recorded behavior, so it requires a new ADR (ADR-0003 §2).
"""


@dataclass(frozen=True)
class SubsystemSnapshot:
    """Base class for subsystem snapshots.

    Each subsystem defines a frozen dataclass subclass with its own fields. Python
    rejects a non-frozen dataclass subclass of a frozen one, so every snapshot is
    immutable. Fields should hold immutable values (numbers, strings, tuples, enums).
    """


@runtime_checkable
class Subsystem(Protocol):
    """A spacecraft subsystem stepped in simulated time.

    Attributes:
        name: Unique name, one of :data:`STEP_ORDER` (for example ``"power"``).
    """

    name: str

    def reset(self, rng: RngFactory) -> None:
        """Return to the initial state.

        Args:
            rng: The run's random factory. Request named streams from it, such as
                ``rng.stream("spacecraft.power.noise")``; never use global randomness.
        """
        ...

    def step(self, dt_us: int, env: EnvironmentState) -> None:
        """Advance the subsystem by ``dt_us`` microseconds under ``env``."""
        ...

    def snapshot(self) -> SubsystemSnapshot:
        """Return the current state as a frozen dataclass."""
        ...


@dataclass(frozen=True)
class SpacecraftState:
    """Snapshots of every subsystem at one instant, in :data:`STEP_ORDER` order.

    Attributes:
        subsystems: Snapshot by subsystem name (read-only).
    """

    subsystems: Mapping[str, SubsystemSnapshot]

    def __post_init__(self) -> None:
        for name, snap in self.subsystems.items():
            if not isinstance(snap, SubsystemSnapshot):
                raise TypeError(
                    f"snapshot for {name!r} must be a SubsystemSnapshot, got {type(snap).__name__}"
                )
        object.__setattr__(self, "subsystems", MappingProxyType(dict(self.subsystems)))

    def get[S: SubsystemSnapshot](self, name: str, kind: type[S]) -> S:
        """Return the named subsystem's snapshot, checked to be of type ``kind``.

        Raises:
            KeyError: No subsystem has that name.
            TypeError: The snapshot is not a ``kind``.
        """
        snap = self.subsystems[name]
        if not isinstance(snap, kind):
            raise TypeError(f"snapshot for {name!r} is {type(snap).__name__}, not {kind.__name__}")
        return snap


class SubsystemStack:
    """Resets and steps a set of subsystems in a fixed order.

    Subsystems may be supplied in any order; they always run in ``order``. Not every
    name in ``order`` needs a subsystem, so the spacecraft can be built up one
    subsystem at a time.
    """

    def __init__(self, subsystems: Iterable[Subsystem], order: tuple[str, ...] = STEP_ORDER):
        """Create a stack.

        Args:
            subsystems: The subsystems to run.
            order: Step order by name. Defaults to :data:`STEP_ORDER`; tests may pass
                their own.

        Raises:
            ValueError: A name is duplicated, or a subsystem's name is not in ``order``.
        """
        if len(set(order)) != len(order):
            raise ValueError(f"step order has duplicate names: {order}")
        by_name: dict[str, Subsystem] = {}
        for subsystem in subsystems:
            if subsystem.name in by_name:
                raise ValueError(f"duplicate subsystem name {subsystem.name!r}")
            if subsystem.name not in order:
                raise ValueError(f"subsystem {subsystem.name!r} is not in the step order {order}")
            by_name[subsystem.name] = subsystem
        self._subsystems = tuple(by_name[name] for name in order if name in by_name)

    @property
    def names(self) -> tuple[str, ...]:
        """Names of the subsystems in step order."""
        return tuple(s.name for s in self._subsystems)

    def reset(self, rng: RngFactory) -> None:
        """Reset every subsystem, in step order."""
        for subsystem in self._subsystems:
            subsystem.reset(rng)

    def step(self, dt_us: int, env: EnvironmentState) -> None:
        """Step every subsystem by ``dt_us`` microseconds, in step order.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
        """
        check_us("dt_us", dt_us)
        for subsystem in self._subsystems:
            subsystem.step(dt_us, env)

    def snapshot(self) -> SpacecraftState:
        """Collect every subsystem's snapshot, in step order."""
        return SpacecraftState(subsystems={s.name: s.snapshot() for s in self._subsystems})
