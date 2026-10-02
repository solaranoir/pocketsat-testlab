"""Tests for the subsystem framework, using a trivial fake subsystem."""

import dataclasses
from random import Random

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.base import (
    STEP_ORDER,
    SpacecraftState,
    Subsystem,
    SubsystemSnapshot,
    SubsystemStack,
)
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.targets.base import EnvironmentState


@dataclasses.dataclass(frozen=True)
class FakeSnapshot(SubsystemSnapshot):
    elapsed_us: int
    steps: int
    sunlit_steps: int
    noise: float


class FakeSubsystem:
    """Counts steps and sunlit time, and draws noise from its own named stream."""

    def __init__(self, name: str = "fake", log: list[tuple[str, str]] | None = None) -> None:
        self.name = name
        self._log = log
        self._rng = Random(0)
        self._elapsed_us = 0
        self._steps = 0
        self._sunlit_steps = 0
        self._noise = 0.0

    def reset(self, rng: RngFactory) -> None:
        if self._log is not None:
            self._log.append(("reset", self.name))
        self._rng = rng.stream(f"spacecraft.{self.name}.noise")
        self._elapsed_us = 0
        self._steps = 0
        self._sunlit_steps = 0
        self._noise = 0.0

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        if self._log is not None:
            self._log.append(("step", self.name))
        self._elapsed_us += dt_us
        self._steps += 1
        self._sunlit_steps += env.sunlit
        self._noise = self._rng.gauss(0.0, 1.0)

    def snapshot(self) -> FakeSnapshot:
        return FakeSnapshot(
            elapsed_us=self._elapsed_us,
            steps=self._steps,
            sunlit_steps=self._sunlit_steps,
            noise=self._noise,
        )


NOMINAL = SpacecraftControls()
SUNLIT = EnvironmentState(sunlit=True)
ECLIPSE = EnvironmentState(sunlit=False)


def test_fake_subsystem_satisfies_protocol() -> None:
    assert isinstance(FakeSubsystem(), Subsystem)


def test_fake_subsystem_steps_and_snapshots() -> None:
    fake = FakeSubsystem()
    fake.reset(RngFactory(42))
    fake.step(100_000, SUNLIT, NOMINAL)
    fake.step(100_000, ECLIPSE, NOMINAL)
    snap = fake.snapshot()
    assert (snap.elapsed_us, snap.steps, snap.sunlit_steps) == (200_000, 2, 1)
    assert type(snap.elapsed_us) is int


def test_snapshots_are_frozen() -> None:
    snap = FakeSubsystem().snapshot()
    with pytest.raises(dataclasses.FrozenInstanceError):
        snap.steps = 5  # type: ignore[misc]


def test_mutable_snapshot_subclass_is_impossible() -> None:
    with pytest.raises(TypeError):
        # Deliberately invalid: the test checks the runtime rejects what mypy also rejects.
        @dataclasses.dataclass
        class Mutable(SubsystemSnapshot):  # type: ignore[misc]
            value: int = 0


def test_step_order_is_documented_constant() -> None:
    assert STEP_ORDER == ("power", "thermal", "attitude", "payload", "comms")


# Supplied in neither step order nor alphabetical order. Step order is
# power, attitude, comms; alphabetical would be attitude, comms, power.
MIXED_ORDER = ("comms", "power", "attitude")
EXPECTED_ORDER = ["power", "attitude", "comms"]


def test_stack_steps_in_fixed_order_regardless_of_input_order() -> None:
    log: list[tuple[str, str]] = []
    stack = SubsystemStack([FakeSubsystem(name, log) for name in MIXED_ORDER])
    assert stack.names == tuple(EXPECTED_ORDER)
    stack.reset(RngFactory(1))
    log.clear()
    stack.step(100_000, SUNLIT, NOMINAL)
    stack.step(100_000, SUNLIT, NOMINAL)
    assert log == [("step", name) for name in EXPECTED_ORDER] * 2


def test_stack_resets_in_fixed_order() -> None:
    log: list[tuple[str, str]] = []
    stack = SubsystemStack([FakeSubsystem(name, log) for name in MIXED_ORDER])
    stack.reset(RngFactory(1))
    assert log == [("reset", name) for name in EXPECTED_ORDER]


def test_stack_accepts_custom_order_for_tests() -> None:
    log: list[tuple[str, str]] = []
    stack = SubsystemStack([FakeSubsystem("a", log), FakeSubsystem("b", log)], order=("b", "a"))
    stack.reset(RngFactory(1))
    stack.step(1, SUNLIT, NOMINAL)
    assert log == [("reset", "b"), ("reset", "a"), ("step", "b"), ("step", "a")]


def test_spacecraft_state_aggregates_snapshots_in_step_order() -> None:
    stack = SubsystemStack([FakeSubsystem(name) for name in MIXED_ORDER])
    stack.reset(RngFactory(7))
    stack.step(250_000, ECLIPSE, NOMINAL)
    state = stack.snapshot()
    assert isinstance(state, SpacecraftState)
    assert list(state.subsystems) == EXPECTED_ORDER
    assert list(state.subsystems) != sorted(state.subsystems)  # guards against sorting
    assert state.get("power", FakeSnapshot).elapsed_us == 250_000
    assert state.get("comms", FakeSnapshot).sunlit_steps == 0


def test_spacecraft_state_is_immutable() -> None:
    state = SpacecraftState(subsystems={"power": FakeSubsystem().snapshot()})
    with pytest.raises(TypeError):
        state.subsystems["thermal"] = FakeSubsystem().snapshot()  # type: ignore[index]
    with pytest.raises(dataclasses.FrozenInstanceError):
        state.subsystems = {}  # type: ignore[misc]


def test_spacecraft_state_get_errors() -> None:
    @dataclasses.dataclass(frozen=True)
    class OtherSnapshot(SubsystemSnapshot):
        pass

    state = SpacecraftState(subsystems={"power": FakeSubsystem().snapshot()})
    with pytest.raises(KeyError):
        state.get("thermal", FakeSnapshot)
    with pytest.raises(TypeError, match="not OtherSnapshot"):
        state.get("power", OtherSnapshot)


def test_spacecraft_state_rejects_non_snapshots() -> None:
    with pytest.raises(TypeError, match="must be a SubsystemSnapshot"):
        SpacecraftState(subsystems={"power": {"soc": 1.0}})  # type: ignore[dict-item]


def test_stack_is_deterministic_for_same_seed() -> None:
    def run(seed: int) -> list[SpacecraftState]:
        stack = SubsystemStack([FakeSubsystem("power"), FakeSubsystem("thermal")])
        stack.reset(RngFactory(seed))
        states = []
        for i in range(50):
            stack.step(100_000, SUNLIT if i % 3 else ECLIPSE, NOMINAL)
            states.append(stack.snapshot())
        return states

    assert run(42) == run(42)
    assert run(42) != run(43)


def test_subsystems_use_independent_named_streams() -> None:
    stack = SubsystemStack([FakeSubsystem("power"), FakeSubsystem("thermal")])
    stack.reset(RngFactory(42))
    stack.step(1, SUNLIT, NOMINAL)
    state = stack.snapshot()
    assert state.get("power", FakeSnapshot).noise != state.get("thermal", FakeSnapshot).noise

    alone = SubsystemStack([FakeSubsystem("power")])
    alone.reset(RngFactory(42))
    alone.step(1, SUNLIT, NOMINAL)
    # Adding the thermal subsystem did not change power's random values.
    assert (
        alone.snapshot().get("power", FakeSnapshot).noise == state.get("power", FakeSnapshot).noise
    )


def _run_steps(stack: SubsystemStack, n: int) -> list[SpacecraftState]:
    states = []
    for _ in range(n):
        stack.step(10, SUNLIT, NOMINAL)
        states.append(stack.snapshot())
    return states


def test_reset_restores_initial_state() -> None:
    stack = SubsystemStack([FakeSubsystem("power")])
    stack.reset(RngFactory(3))
    first = _run_steps(stack, 3)
    stack.reset(RngFactory(3))
    assert stack.snapshot().get("power", FakeSnapshot).steps == 0
    assert _run_steps(stack, 3) == first


@pytest.mark.parametrize(
    ("subsystems", "match"),
    [
        ([FakeSubsystem("power"), FakeSubsystem("power")], "duplicate subsystem"),
        ([FakeSubsystem("gravity")], "not in the step order"),
    ],
)
def test_stack_rejects_bad_subsystem_sets(subsystems: list[FakeSubsystem], match: str) -> None:
    with pytest.raises(ValueError, match=match):
        SubsystemStack(subsystems)


def test_stack_rejects_duplicate_order() -> None:
    with pytest.raises(ValueError, match="duplicate names"):
        SubsystemStack([], order=("a", "a"))


@pytest.mark.parametrize(
    ("dt_us", "error"), [(-1, ValueError), (0.1, TypeError), (True, TypeError)]
)
def test_stack_step_requires_integer_microseconds(dt_us: object, error: type[Exception]) -> None:
    stack = SubsystemStack([FakeSubsystem("power")])
    stack.reset(RngFactory(0))
    with pytest.raises(error):
        stack.step(dt_us, SUNLIT, NOMINAL)  # type: ignore[arg-type]


def test_empty_stack_is_allowed() -> None:
    stack = SubsystemStack([])
    stack.reset(RngFactory(0))
    stack.step(100_000, SUNLIT, NOMINAL)
    assert stack.snapshot().subsystems == {}
