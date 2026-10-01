"""Tests for the cross-subsystem read mechanism (SnapshotBoard, #85)."""

import dataclasses
from random import Random

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import (
    SnapshotBoard,
    SnapshotReader,
    SpacecraftControls,
    SpacecraftState,
    Subsystem,
    SubsystemSnapshot,
    SubsystemStack,
)
from pocketsat.targets.base import EnvironmentState

ENV = EnvironmentState()
CONTROLS = SpacecraftControls()
TICK_US = 100_000


@dataclasses.dataclass(frozen=True)
class CounterSnapshot(SubsystemSnapshot):
    ticks: int
    seen: tuple[tuple[str, int], ...] = ()


class Counter:
    """Counts its own steps and records what it reads from other subsystems."""

    def __init__(self, name: str, reader: SnapshotReader, reads: tuple[str, ...] = ()) -> None:
        self.name = name
        self._reader = reader
        self._reads = reads
        self._ticks = 0
        self._seen: tuple[tuple[str, int], ...] = ()
        self.snapshot_calls = 0

    def reset(self, rng: RngFactory) -> None:
        self._ticks = 0
        self._seen = ()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        self._ticks += 1
        self._seen = tuple(
            (other, self._reader.get(other, CounterSnapshot).ticks) for other in self._reads
        )

    def snapshot(self) -> CounterSnapshot:
        self.snapshot_calls += 1
        return CounterSnapshot(ticks=self._ticks, seen=self._seen)


def _build(
    reads: dict[str, tuple[str, ...]], order: tuple[str, ...] | None = None
) -> tuple[SubsystemStack, dict[str, Counter]]:
    board = SnapshotBoard()  # created first, so subsystems get it at construction
    counters = {name: Counter(name, board, deps) for name, deps in reads.items()}
    if order is None:
        stack = SubsystemStack(counters.values(), board=board)
    else:
        stack = SubsystemStack(counters.values(), order=order, board=board)
    return stack, counters


def test_counter_satisfies_unchanged_protocol() -> None:
    assert isinstance(Counter("power", SnapshotBoard()), Subsystem)


def test_earlier_is_current_tick_and_later_is_previous_tick() -> None:
    # STEP_ORDER: power, thermal, attitude. Thermal reads power (earlier) and
    # attitude (later); power reads attitude (later).
    stack, _ = _build({"power": ("attitude",), "thermal": ("power", "attitude"), "attitude": ()})
    stack.reset(RngFactory(0))
    for tick in range(1, 6):
        stack.step(TICK_US, ENV, CONTROLS)
        state = stack.snapshot()
        thermal = dict(state.get("thermal", CounterSnapshot).seen)
        power = dict(state.get("power", CounterSnapshot).seen)
        assert thermal["power"] == tick  # earlier in STEP_ORDER: this tick
        assert thermal["attitude"] == tick - 1  # later in STEP_ORDER: previous tick
        assert power["attitude"] == tick - 1


def test_first_tick_reads_the_reset_snapshots() -> None:
    stack, _ = _build({"power": ("attitude",), "attitude": ()})
    stack.reset(RngFactory(0))
    assert stack.board.get("attitude", CounterSnapshot).ticks == 0
    stack.step(TICK_US, ENV, CONTROLS)
    assert dict(stack.snapshot().get("power", CounterSnapshot).seen) == {"attitude": 0}


def test_rule_follows_a_custom_step_order() -> None:
    # Reverse the usual order: attitude now steps before power.
    stack, _ = _build({"power": ("attitude",), "attitude": ()}, order=("attitude", "power"))
    stack.reset(RngFactory(0))
    stack.step(TICK_US, ENV, CONTROLS)
    stack.step(TICK_US, ENV, CONTROLS)
    assert dict(stack.snapshot().get("power", CounterSnapshot).seen) == {"attitude": 2}


def test_reset_clears_and_republishes() -> None:
    stack, _ = _build({"power": (), "thermal": ("power",)})
    stack.reset(RngFactory(0))
    for _ in range(3):
        stack.step(TICK_US, ENV, CONTROLS)
    stack.reset(RngFactory(0))
    assert stack.board.get("power", CounterSnapshot).ticks == 0
    assert stack.board.get("thermal", CounterSnapshot).ticks == 0


def test_reading_a_subsystem_not_in_the_stack_raises() -> None:
    stack, _ = _build({"power": ("payload",)})
    stack.reset(RngFactory(0))
    with pytest.raises(KeyError, match="not in the stack"):
        stack.step(TICK_US, ENV, CONTROLS)


def test_reading_before_reset_raises() -> None:
    board = SnapshotBoard()
    with pytest.raises(KeyError, match="has not been reset"):
        board.get("power", CounterSnapshot)


def test_wrong_snapshot_type_raises() -> None:
    @dataclasses.dataclass(frozen=True)
    class OtherSnapshot(SubsystemSnapshot):
        pass

    stack, _ = _build({"power": ()})
    stack.reset(RngFactory(0))
    with pytest.raises(TypeError, match="not OtherSnapshot"):
        stack.board.get("power", OtherSnapshot)


def test_publish_rejects_non_snapshots() -> None:
    with pytest.raises(TypeError, match="must be a SubsystemSnapshot"):
        SnapshotBoard().publish("power", {"soc": 1.0})  # type: ignore[arg-type]


def test_snapshot_matches_subsystems_and_is_taken_once_per_tick() -> None:
    stack, counters = _build({"power": (), "thermal": ("power",), "attitude": ()})
    stack.reset(RngFactory(0))
    calls_after_reset = {name: c.snapshot_calls for name, c in counters.items()}
    for _ in range(4):
        stack.step(TICK_US, ENV, CONTROLS)
        state = stack.snapshot()
        assert state == SpacecraftState(
            subsystems={name: counters[name].snapshot() for name in stack.names}
        )
    # 4 publishes per subsystem from stepping, plus the 4 direct calls above.
    for name, counter in counters.items():
        assert counter.snapshot_calls == calls_after_reset[name] + 4 + 4


def test_snapshot_before_reset_collects_fresh() -> None:
    stack, _ = _build({"power": ()})
    assert stack.snapshot().get("power", CounterSnapshot).ticks == 0


def test_state_view_is_read_only() -> None:
    stack, _ = _build({"power": ()})
    stack.reset(RngFactory(0))
    with pytest.raises(TypeError):
        stack.board.state().subsystems["thermal"] = CounterSnapshot(ticks=0)  # type: ignore[index]


@dataclasses.dataclass(frozen=True)
class NoisySnapshot(SubsystemSnapshot):
    value: float


class Noisy:
    """Mixes its own noise with another subsystem's latest value."""

    def __init__(self, name: str, reader: SnapshotReader, source: str | None) -> None:
        self.name = name
        self._reader = reader
        self._source = source
        self._rng = Random(0)
        self._value = 0.0

    def reset(self, rng: RngFactory) -> None:
        self._rng = rng.stream(f"spacecraft.{self.name}.noise")
        self._value = 0.0

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        other = 0.0
        if self._source is not None:
            other = self._reader.get(self._source, NoisySnapshot).value
        self._value = 0.5 * self._value + 0.25 * other + self._rng.random()

    def snapshot(self) -> NoisySnapshot:
        return NoisySnapshot(value=self._value)


def test_determinism_unchanged_with_cross_reads() -> None:
    def run(seed: int) -> list[SpacecraftState]:
        board = SnapshotBoard()
        stack = SubsystemStack(
            [
                Noisy("power", board, "attitude"),
                Noisy("thermal", board, "power"),
                Noisy("attitude", board, "thermal"),
            ],
            board=board,
        )
        stack.reset(RngFactory(seed))
        states = []
        for _ in range(200):
            stack.step(TICK_US, ENV, CONTROLS)
            states.append(stack.snapshot())
        return states

    assert run(42) == run(42)
    assert run(42) != run(43)
