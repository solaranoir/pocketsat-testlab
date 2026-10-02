"""Subsystem framework for the SIL spacecraft model.

Every spacecraft subsystem (power, thermal, attitude, payload, communications)
implements :class:`Subsystem`. A :class:`SubsystemStack` resets and steps them in the
fixed order :data:`STEP_ORDER` and aggregates their snapshots into a
:class:`SpacecraftState` for telemetry and for other subsystems to read. Each tick,
every subsystem receives the same :class:`SpacecraftControls` (ADR-0004).

Time is integer microseconds and randomness comes only from named ``RngFactory``
streams (ADR-0003).
"""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Final, Protocol, runtime_checkable

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.targets.base import EnvironmentState

STEP_ORDER: Final[tuple[str, ...]] = ("power", "thermal", "attitude", "payload", "comms")
"""The order subsystems are stepped in each tick. This is the only place it is defined.

1. ``power``: bus state first; every other subsystem runs on it.
2. ``thermal``: heat from this tick's electrical loads and the environment.
3. ``attitude``: pointing, which the payload and radio depend on.
4. ``payload``: collects data given the current pointing and power.
5. ``comms``: last, so it can transmit what earlier subsystems produced this tick.

Changing this order changes recorded behavior, so it requires a new ADR (ADR-0003 §2).

The flight computer is not part of ``STEP_ORDER``. It always runs after all
subsystems, within ``advance()`` steps c to f, and the controls it produces take
effect in the next tick (ADR-0004 §2).
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
    """A spacecraft subsystem stepped in simulated time."""

    @property
    def name(self) -> str:
        """Unique name, one of :data:`STEP_ORDER` (for example ``"power"``).

        Read-only: a subsystem's name never changes after construction. A plain or
        ``Final`` class attribute, or an instance attribute, satisfies it.
        """
        ...

    def reset(self, rng: RngFactory) -> None:
        """Return to the initial state.

        Args:
            rng: The run's random factory. Request named streams from it, such as
                ``rng.stream("spacecraft.power.noise")``; never use global randomness.
        """
        ...

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Advance the subsystem by ``dt_us`` microseconds under ``env``.

        Args:
            dt_us: Time step in integer microseconds.
            env: Environment for this tick.
            controls: This tick's controls (ADR-0004). Read only this subsystem's own
                record and the fault overrides that concern it.
        """
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


class SnapshotReader(Protocol):
    """Read-only access to other subsystems' latest published snapshots (#85).

    A subsystem that reads others receives a reader when it is constructed, alongside
    its settings and starting records (ADR-0005). Subsystems never hold references to
    each other (ADR-0004 §3).
    """

    def get[S: SubsystemSnapshot](self, name: str, kind: type[S]) -> S:
        """Return the named subsystem's latest published snapshot, checked to be ``kind``."""
        ...

    def state(self) -> SpacecraftState:
        """Return every published snapshot as a :class:`SpacecraftState`."""
        ...


class SnapshotBoard:
    """The latest published snapshot of each subsystem (#85).

    :class:`SubsystemStack` publishes a subsystem's snapshot immediately after it is
    reset and immediately after it steps. A subsystem reading the board during its own
    step therefore sees:

    - the **current** tick for subsystems earlier in the step order (already stepped),
    - the **previous** tick for subsystems later in the step order (not yet stepped).

    That is the cross-read rule of ADR-0004 §3, with no special cases. Subsystems see
    the board only through the read-only :class:`SnapshotReader` interface.
    """

    def __init__(self) -> None:
        """Create an empty board."""
        self._latest: dict[str, SubsystemSnapshot] = {}

    def publish(self, name: str, snapshot: SubsystemSnapshot) -> None:
        """Record ``snapshot`` as the latest for ``name``. Called by the stack only.

        Raises:
            TypeError: ``snapshot`` is not a :class:`SubsystemSnapshot`.
        """
        if not isinstance(snapshot, SubsystemSnapshot):
            raise TypeError(
                f"snapshot for {name!r} must be a SubsystemSnapshot, got {type(snapshot).__name__}"
            )
        self._latest[name] = snapshot

    def clear(self) -> None:
        """Forget every published snapshot. Called by the stack on reset."""
        self._latest.clear()

    def has(self, name: str) -> bool:
        """Whether a snapshot has been published for ``name``."""
        return name in self._latest

    def get[S: SubsystemSnapshot](self, name: str, kind: type[S]) -> S:
        """Return the named subsystem's latest published snapshot, checked to be ``kind``.

        Raises:
            KeyError: Nothing has been published for ``name``: it is not in the stack,
                or the stack has not been reset yet.
            TypeError: The snapshot is not a ``kind``.
        """
        try:
            snap = self._latest[name]
        except KeyError:
            raise KeyError(
                f"no snapshot published for {name!r}: it is not in the stack, "
                "or the stack has not been reset"
            ) from None
        if not isinstance(snap, kind):
            raise TypeError(f"snapshot for {name!r} is {type(snap).__name__}, not {kind.__name__}")
        return snap

    def state(self) -> SpacecraftState:
        """Return every published snapshot as a :class:`SpacecraftState`."""
        return SpacecraftState(subsystems=self._latest)


class SubsystemStack:
    """Resets and steps a set of subsystems in a fixed order.

    Subsystems may be supplied in any order; they always run in ``order``. Not every
    name in ``order`` needs a subsystem, so the spacecraft can be built up one
    subsystem at a time.

    The stack publishes each subsystem's snapshot to its :class:`SnapshotBoard` right
    after that subsystem is reset or stepped, which is how subsystems read each other
    (#85). To let subsystems read others, create the board first, pass it (as a
    :class:`SnapshotReader`) to their constructors, then pass it to the stack.
    """

    def __init__(
        self,
        subsystems: Iterable[Subsystem],
        order: tuple[str, ...] = STEP_ORDER,
        board: SnapshotBoard | None = None,
    ):
        """Create a stack.

        Args:
            subsystems: The subsystems to run.
            order: Step order by name. Defaults to :data:`STEP_ORDER`; tests may pass
                their own.
            board: Where snapshots are published. A new board is created if omitted.

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
        self._board = SnapshotBoard() if board is None else board

    @property
    def board(self) -> SnapshotBoard:
        """The board this stack publishes snapshots to."""
        return self._board

    @property
    def names(self) -> tuple[str, ...]:
        """Names of the subsystems in step order."""
        return tuple(s.name for s in self._subsystems)

    def reset(self, rng: RngFactory) -> None:
        """Reset every subsystem in step order, then publish their snapshots."""
        self._board.clear()
        for subsystem in self._subsystems:
            subsystem.reset(rng)
        for subsystem in self._subsystems:
            self._board.publish(subsystem.name, subsystem.snapshot())

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Step every subsystem by ``dt_us`` microseconds, in step order.

        Every subsystem receives the same ``controls`` object (ADR-0004 §1).

        Raises:
            TypeError: ``dt_us`` is not an int, or ``controls`` is not a
                :class:`SpacecraftControls`.
            ValueError: ``dt_us`` is negative.
        """
        check_us("dt_us", dt_us)
        if not isinstance(controls, SpacecraftControls):
            raise TypeError(f"controls must be SpacecraftControls, got {type(controls).__name__}")
        for subsystem in self._subsystems:
            subsystem.step(dt_us, env, controls)
            self._board.publish(subsystem.name, subsystem.snapshot())

    def snapshot(self) -> SpacecraftState:
        """Return every subsystem's latest snapshot, in step order.

        After :meth:`reset`, this is the published state, so each subsystem's
        ``snapshot()`` runs once per tick. Before the first reset it collects fresh
        snapshots.
        """
        if all(self._board.has(s.name) for s in self._subsystems):
            return SpacecraftState(
                subsystems={
                    s.name: self._board.get(s.name, SubsystemSnapshot) for s in self._subsystems
                }
            )
        return SpacecraftState(subsystems={s.name: s.snapshot() for s in self._subsystems})
