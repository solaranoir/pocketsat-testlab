"""Tests for spacecraft configuration and starting state (ADR-0005)."""

import dataclasses
from collections.abc import Callable
from random import Random

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    AttitudeConfig,
    AttitudeInitial,
    CommsConfig,
    PayloadConfig,
    PayloadInitial,
    PowerConfig,
    PowerInitial,
    SpacecraftConfig,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    Subsystem,
    SubsystemSnapshot,
    SubsystemStack,
    ThermalConfig,
    ThermalInitial,
)
from pocketsat.targets.base import EnvironmentState

ENV = EnvironmentState()
CONTROLS = SpacecraftControls()
TICK_US = 100_000


# --- Collections and defaults ---------------------------------------------------------


def test_config_collects_one_record_per_subsystem() -> None:
    config = SpacecraftConfig()
    assert [f.name for f in dataclasses.fields(config)] == [
        "power",
        "thermal",
        "attitude",
        "payload",
        "comms",
    ]
    assert isinstance(config.power, PowerConfig)
    assert isinstance(config.thermal, ThermalConfig)
    assert isinstance(config.attitude, AttitudeConfig)
    assert isinstance(config.payload, PayloadConfig)
    assert isinstance(config.comms, CommsConfig)
    assert SpacecraftConfig() == NOMINAL_CONFIG


def test_initial_state_has_no_comms_record() -> None:
    names = [f.name for f in dataclasses.fields(SpacecraftInitialState)]
    assert names == ["power", "thermal", "attitude", "payload"]


def test_default_initial_state() -> None:
    initial = SpacecraftInitialState()
    assert initial == DEFAULT_INITIAL_STATE
    assert initial.power == PowerInitial(soc=0.8)
    assert initial.thermal == ThermalInitial(battery_c=20.0, electronics_c=20.0)
    assert initial.attitude == AttitudeInitial(pointing_error_deg=0.0, rate_dps=0.0)
    assert initial.payload == PayloadInitial(buffer_fill=0.0)


@pytest.mark.parametrize(
    ("record", "field"),
    [
        (SpacecraftConfig(), "power"),
        (SpacecraftInitialState(), "thermal"),
        (PowerInitial(), "soc"),
        (ThermalInitial(), "battery_c"),
        (AttitudeInitial(), "rate_dps"),
        (PayloadInitial(), "buffer_fill"),
    ],
)
def test_records_are_frozen(record: object, field: str) -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        setattr(record, field, None)


def test_single_values_can_be_overridden_with_replace() -> None:
    low = dataclasses.replace(DEFAULT_INITIAL_STATE, power=PowerInitial(soc=0.3))
    assert low.power.soc == 0.3
    assert low.thermal == DEFAULT_INITIAL_STATE.thermal


# --- Validation -----------------------------------------------------------------------


@pytest.mark.parametrize(
    "build",
    [
        lambda: PowerInitial(soc=0.0),
        lambda: PowerInitial(soc=1),
        lambda: ThermalInitial(battery_c=-150.0, electronics_c=85.0),
        lambda: AttitudeInitial(pointing_error_deg=180.0, rate_dps=12.5),
        lambda: PayloadInitial(buffer_fill=1.0),
    ],
)
def test_valid_starting_values_accepted(build: Callable[[], object]) -> None:
    build()


@pytest.mark.parametrize(
    ("build", "error"),
    [
        (lambda: PowerInitial(soc=1.01), ValueError),
        (lambda: PowerInitial(soc=-0.01), ValueError),
        (lambda: PowerInitial(soc=float("nan")), ValueError),
        (lambda: PowerInitial(soc=True), TypeError),
        (lambda: PowerInitial(soc="0.5"), TypeError),  # type: ignore[arg-type]
        (lambda: ThermalInitial(battery_c=-273.15), ValueError),
        (lambda: ThermalInitial(electronics_c=float("inf")), ValueError),
        (lambda: AttitudeInitial(pointing_error_deg=-1.0), ValueError),
        (lambda: AttitudeInitial(pointing_error_deg=181.0), ValueError),
        (lambda: AttitudeInitial(rate_dps=-0.1), ValueError),
        (lambda: PayloadInitial(buffer_fill=1.5), ValueError),
        (lambda: SpacecraftConfig(power=ThermalConfig()), TypeError),  # type: ignore[arg-type]
        (lambda: SpacecraftInitialState(power=0.8), TypeError),  # type: ignore[arg-type]
        (lambda: SpacecraftInitialState(payload=PowerInitial()), TypeError),  # type: ignore[arg-type]
    ],
)
def test_invalid_starting_values_rejected(
    build: Callable[[], object], error: type[Exception]
) -> None:
    with pytest.raises(error):
        build()


# --- The convention, with a fake subsystem --------------------------------------------


@dataclasses.dataclass(frozen=True)
class BatterySnapshot(SubsystemSnapshot):
    soc: float


class FakeBattery:
    """Built with its own settings and starting record; reset returns to the start."""

    name = "power"

    def __init__(self, config: PowerConfig, initial: PowerInitial) -> None:
        self._config = config
        self._initial = initial
        self._rng = Random(0)
        self._soc = initial.soc

    def reset(self, rng: RngFactory) -> None:
        self._rng = rng.stream("spacecraft.power.noise")
        self._soc = self._initial.soc

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        drift = 0.001 if env.sunlit else -0.002
        self._soc = min(1.0, max(0.0, self._soc + drift + 0.0001 * self._rng.random()))

    def snapshot(self) -> BatterySnapshot:
        return BatterySnapshot(soc=self._soc)


def _stack(config: SpacecraftConfig, initial: SpacecraftInitialState) -> SubsystemStack:
    # Each subsystem is built with only its own records.
    return SubsystemStack([FakeBattery(config.power, initial.power)])


def _run(stack: SubsystemStack, seed: int, ticks: int) -> list[SpacecraftState]:
    stack.reset(RngFactory(seed))
    states = [stack.snapshot()]
    for i in range(ticks):
        env = EnvironmentState(sunlit=i % 10 < 6)
        stack.step(TICK_US, env, CONTROLS)
        states.append(stack.snapshot())
    return states


def test_fake_satisfies_protocol_unchanged() -> None:
    assert isinstance(FakeBattery(PowerConfig(), PowerInitial()), Subsystem)


def test_subsystem_starts_from_its_starting_record() -> None:
    initial = SpacecraftInitialState(power=PowerInitial(soc=0.35))
    stack = _stack(NOMINAL_CONFIG, initial)
    stack.reset(RngFactory(1))
    assert stack.snapshot().get("power", BatterySnapshot).soc == 0.35


def test_reset_restores_starting_state_after_running() -> None:
    initial = SpacecraftInitialState(power=PowerInitial(soc=0.5))
    stack = _stack(NOMINAL_CONFIG, initial)
    first = _run(stack, seed=7, ticks=50)
    assert first[-1].get("power", BatterySnapshot).soc != 0.5  # it moved
    again = _run(stack, seed=7, ticks=50)  # reset returns to the start
    assert again[0].get("power", BatterySnapshot).soc == 0.5
    assert again == first


def test_same_starting_state_and_seed_are_identical() -> None:
    initial = SpacecraftInitialState(power=PowerInitial(soc=0.6))
    a = _run(_stack(NOMINAL_CONFIG, initial), seed=42, ticks=100)
    b = _run(_stack(NOMINAL_CONFIG, initial), seed=42, ticks=100)
    assert a == b


def test_different_starting_states_diverge() -> None:
    low = SpacecraftInitialState(power=PowerInitial(soc=0.2))
    high = SpacecraftInitialState(power=PowerInitial(soc=0.9))
    a = _run(_stack(NOMINAL_CONFIG, low), seed=42, ticks=100)
    b = _run(_stack(NOMINAL_CONFIG, high), seed=42, ticks=100)
    assert a != b
    assert a[-1].get("power", BatterySnapshot).soc < b[-1].get("power", BatterySnapshot).soc
