"""Scriptable fake subsystems for tests (#76).

Each fake implements the ``Subsystem`` protocol and returns contract snapshots
(:mod:`pocketsat.spacecraft.snapshots`) that a test chooses, optionally changing over
time. Subsystem tests use fakes of the subsystems they read, so no subsystem waits for
another's implementation. Like ``EchoTarget``, the fakes live in the package so every
test directory can import them.

Ticks are counted from 0: the first ``step()`` after a ``reset()`` is tick 0. Before
any step, ``snapshot()`` returns the fake's initial snapshot. A script entry for tick
``n`` takes effect in tick ``n`` and holds until the next entry, so a payload draw that
steps up at tick 10 is::

    low = default_snapshot("payload")
    high = replace_truth(low, power_w=3.0)
    payload = FakeSubsystem("payload", low, script={10: high})

The fakes use no randomness and no wall-clock time.
"""

import dataclasses
from collections.abc import Callable, Iterable, Mapping
from types import MappingProxyType
from typing import Any, Final

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.base import STEP_ORDER, SubsystemStack
from pocketsat.spacecraft.controls import RadioMode, SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    SNAPSHOT_TYPES,
    AttitudeReadings,
    AttitudeSnapshot,
    AttitudeState,
    AttitudeTruth,
    CommsSnapshot,
    CommsTruth,
    ContractSnapshot,
    PayloadSnapshot,
    PayloadState,
    PayloadTruth,
    PowerReadings,
    PowerSnapshot,
    PowerTruth,
    ThermalReadings,
    ThermalSnapshot,
    ThermalTruth,
)
from pocketsat.targets.base import EnvironmentState

type Script[S] = Mapping[int, S] | Callable[[int, S], S]
"""Values over time for a fake.

- A mapping from tick to snapshot: the snapshot takes effect in that tick and holds
  until the next entry.
- A function ``(tick, previous) -> snapshot``, called every tick with the tick number
  and the previous snapshot.
"""

DEFAULT_SNAPSHOTS: Final[Mapping[str, ContractSnapshot[Any, Any]]] = MappingProxyType(
    {
        "power": PowerSnapshot(
            truth=PowerTruth(
                bus_v=7.8,
                battery_current_a=0.5,
                soc=0.8,
                generation_w=8.0,
                total_load_w=4.0,
            ),
            readings=PowerReadings(
                bus_v=7.8,
                battery_current_a=0.5,
                soc=0.8,
                low_battery=False,
                critical_battery=False,
            ),
        ),
        "thermal": ThermalSnapshot(
            truth=ThermalTruth(
                battery_c=20.0, electronics_c=20.0, heater_on=False, heater_power_w=0.0
            ),
            readings=ThermalReadings(
                battery_c=20.0, electronics_c=20.0, over_temp=False, under_temp=False
            ),
        ),
        "attitude": AttitudeSnapshot(
            truth=AttitudeTruth(
                pointing_error_deg=0.0,
                rate_dps=0.0,
                state=AttitudeState.STABILIZED,
                control_power_w=0.5,
            ),
            readings=AttitudeReadings(
                pointing_error_deg=0.0, rate_dps=0.0, state=AttitudeState.STABILIZED
            ),
        ),
        "payload": PayloadSnapshot.from_truth(
            PayloadTruth(
                state=PayloadState.OFF,
                buffered_bytes=0,
                buffer_capacity_bytes=65_536,
                oldest_unreleased_chunk_id=0,
                next_chunk_id=0,
                total_produced_bytes=0,
                total_released_bytes=0,
                power_w=0.0,
            )
        ),
        "comms": CommsSnapshot.from_truth(
            CommsTruth(
                radio_mode=RadioMode.RX_TX,
                receiver_on=True,
                transmitter_on=True,
                transmit_capacity_bytes=120,
                previous_tick_sent_bytes=0,
                uplink_lost_count=0,
                outbound_suppressed_count=0,
                transmit_power_w=1.0,
            )
        ),
    }
)
"""A plausible nominal snapshot for each subsystem: sunlit and charging, stabilized,
payload off, radio ``RX_TX`` and idle. The values are illustrative, not calibrated
(#72 calibrates the real models)."""


def default_snapshot(name: str) -> ContractSnapshot[Any, Any]:
    """Return the default fake snapshot for subsystem ``name``.

    Raises:
        KeyError: ``name`` is not a subsystem in :data:`STEP_ORDER`.
    """
    return DEFAULT_SNAPSHOTS[name]


def replace_truth[S: ContractSnapshot[Any, Any]](snapshot: S, **changes: Any) -> S:
    """Return ``snapshot`` with fields of its truth record changed.

    For a mirrored subsystem (payload, comms) the readings change with it. For a
    subsystem with sensors the readings are left as they are; change them with
    :func:`replace_readings`.
    """
    truth = dataclasses.replace(snapshot.truth, **changes)
    if snapshot.mirrored:
        return type(snapshot).from_truth(truth)
    return dataclasses.replace(snapshot, truth=truth)


def replace_readings[S: ContractSnapshot[Any, Any]](snapshot: S, **changes: Any) -> S:
    """Return ``snapshot`` with fields of its readings record changed.

    Raises:
        TypeError: The subsystem is mirrored (payload, comms); use
            :func:`replace_truth`, which keeps readings equal to truth.
    """
    if snapshot.mirrored:
        raise TypeError(f"{type(snapshot).__name__} readings equal its truth; use replace_truth")
    return dataclasses.replace(snapshot, readings=dataclasses.replace(snapshot.readings, **changes))


class FakeSubsystem[S: ContractSnapshot[Any, Any]]:
    """A ``Subsystem`` that returns scripted contract snapshots.

    It records what it was given, so tests can check how a stack drove it.

    Attributes:
        name: Subsystem name, one of :data:`STEP_ORDER`.
        tick: Number of steps since the last reset (the next step is this tick).
        elapsed_us: Simulated time stepped since the last reset, microseconds.
        last_env: Environment of the most recent step, or ``None`` before the first.
        last_controls: Controls of the most recent step, or ``None`` before the first.
        reset_count: Number of resets.
    """

    def __init__(self, name: str, initial: S, script: Script[S] | None = None) -> None:
        """Create a fake.

        Args:
            name: Subsystem name, one of :data:`STEP_ORDER`.
            initial: Snapshot returned after a reset, before the first step (and until
                the script changes it). Must be the subsystem's snapshot type.
            script: Values over time (see :data:`Script`), or ``None`` to hold
                ``initial``.

        Raises:
            ValueError: ``name`` is not in :data:`STEP_ORDER`.
            TypeError: ``initial`` is not the subsystem's snapshot type, or a script
                entry is not.
        """
        if name not in SNAPSHOT_TYPES:
            raise ValueError(f"unknown subsystem {name!r}; expected one of {STEP_ORDER}")
        self.name = name
        self._kind = SNAPSHOT_TYPES[name]
        self._check(initial)
        if isinstance(script, Mapping):
            for tick, snap in script.items():
                if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
                    raise ValueError(f"script ticks must be non-negative ints, got {tick!r}")
                self._check(snap)
        self._initial = initial
        self._script = script
        self._current = initial
        self.tick = 0
        self.elapsed_us = 0
        self.last_env: EnvironmentState | None = None
        self.last_controls: SpacecraftControls | None = None
        self.reset_count = 0

    def _check(self, snap: object) -> None:
        if type(snap) is not self._kind:
            raise TypeError(
                f"{self.name} fake needs a {self._kind.__name__}, got {type(snap).__name__}"
            )

    def reset(self, rng: RngFactory) -> None:
        """Return to the initial snapshot and tick 0. The fake requests no streams."""
        self._current = self._initial
        self.tick = 0
        self.elapsed_us = 0
        self.last_env = None
        self.last_controls = None
        self.reset_count += 1

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Apply the script for this tick, then advance the tick counter.

        Raises:
            TypeError: ``dt_us`` is not an int, or a scripted function returned the
                wrong snapshot type.
            ValueError: ``dt_us`` is negative.
        """
        check_us("dt_us", dt_us)
        script = self._script
        if isinstance(script, Mapping):
            if self.tick in script:
                self._current = script[self.tick]
        elif script is not None:
            snap = script(self.tick, self._current)
            self._check(snap)
            self._current = snap
        self.tick += 1
        self.elapsed_us += dt_us
        self.last_env = env
        self.last_controls = controls

    def snapshot(self) -> S:
        """Return the current scripted snapshot."""
        return self._current


def fake_subsystems(
    overrides: Mapping[str, FakeSubsystem[Any]] | None = None,
) -> tuple[FakeSubsystem[Any], ...]:
    """Return one fake per subsystem in :data:`STEP_ORDER`, holding the defaults.

    Args:
        overrides: Fakes to use instead of the defaults, by name.

    Raises:
        ValueError: An override's key does not match its fake's name.
    """
    overrides = overrides or {}
    for key, fake in overrides.items():
        if fake.name != key:
            raise ValueError(f"override {key!r} holds the {fake.name!r} fake")
    return tuple(
        overrides[name] if name in overrides else FakeSubsystem(name, default_snapshot(name))
        for name in STEP_ORDER
    )


def fake_stack(fakes: Iterable[FakeSubsystem[Any]] | None = None) -> SubsystemStack:
    """Return a :class:`SubsystemStack` of fakes, all five defaults if none are given."""
    return SubsystemStack(fake_subsystems() if fakes is None else fakes)
