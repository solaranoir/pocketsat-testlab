"""Tests for the shared fake subsystems (#76)."""

from pathlib import Path

import pytest
from test_determinism_guard import PACKAGE_DIR, _simulation_modules, find_violations
from test_snapshot_contracts import CROSS_READS

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import snapshots
from pocketsat.spacecraft.base import STEP_ORDER, SpacecraftState, Subsystem, SubsystemStack
from pocketsat.spacecraft.controls import PayloadControls, SpacecraftControls
from pocketsat.spacecraft.fakes import (
    DEFAULT_SNAPSHOTS,
    FakeSubsystem,
    default_snapshot,
    fake_stack,
    fake_subsystems,
    replace_readings,
    replace_truth,
)
from pocketsat.spacecraft.snapshots import (
    SNAPSHOT_TYPES,
    AttitudeSnapshot,
    AttitudeState,
    PayloadSnapshot,
    PowerSnapshot,
    ThermalSnapshot,
)
from pocketsat.targets.base import EnvironmentState

ENV = EnvironmentState()
CONTROLS = SpacecraftControls()
TICK_US = 100_000


def _run(stack: SubsystemStack, ticks: int) -> list[SpacecraftState]:
    states = []
    for _ in range(ticks):
        stack.step(TICK_US, ENV, CONTROLS)
        states.append(stack.snapshot())
    return states


# --- Protocol and defaults ----------------------------------------------------------------


@pytest.mark.parametrize("name", STEP_ORDER)
def test_fake_satisfies_subsystem_protocol(name: str) -> None:
    fake = FakeSubsystem(name, default_snapshot(name))
    assert isinstance(fake, Subsystem)
    assert fake.name == name


@pytest.mark.parametrize("name", STEP_ORDER)
def test_default_snapshots_are_contract_snapshots(name: str) -> None:
    assert type(DEFAULT_SNAPSHOTS[name]) is SNAPSHOT_TYPES[name]


def test_fake_rejects_unknown_name_and_wrong_snapshot_type() -> None:
    with pytest.raises(ValueError, match="unknown subsystem"):
        FakeSubsystem("radio", default_snapshot("comms"))
    with pytest.raises(TypeError, match="needs a PowerSnapshot"):
        FakeSubsystem("power", default_snapshot("thermal"))  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="needs a PowerSnapshot"):
        FakeSubsystem("power", default_snapshot("power"), script={3: default_snapshot("thermal")})
    with pytest.raises(ValueError, match="non-negative ints"):
        FakeSubsystem("power", default_snapshot("power"), script={-1: default_snapshot("power")})


# --- Scripting ------------------------------------------------------------------------------


def test_mapping_script_steps_up_at_tick_10_and_holds() -> None:
    idle = default_snapshot("payload")
    high = replace_truth(idle, power_w=3.0)
    payload = FakeSubsystem("payload", idle, script={10: high})
    payload.reset(RngFactory(0))
    assert payload.snapshot() is idle
    draws = []
    for _ in range(15):
        payload.step(TICK_US, ENV, CONTROLS)
        draws.append(payload.snapshot().truth.power_w)
    assert draws == [0.0] * 10 + [3.0] * 5
    assert payload.snapshot().readings.power_w == 3.0


def test_function_script_gets_tick_and_previous() -> None:
    start = default_snapshot("power")
    seen: list[tuple[int, float]] = []

    def drain(tick: int, previous: PowerSnapshot) -> PowerSnapshot:
        seen.append((tick, previous.truth.soc))
        return replace_truth(previous, soc=previous.truth.soc - 0.1)

    power = FakeSubsystem("power", start, script=drain)
    power.reset(RngFactory(0))
    for _ in range(3):
        power.step(TICK_US, ENV, CONTROLS)
    assert [t for t, _ in seen] == [0, 1, 2]
    assert power.snapshot().truth.soc == pytest.approx(0.5)
    # Readings are left alone for a subsystem with sensors.
    assert power.snapshot().readings == start.readings


def test_function_script_returning_wrong_type_is_rejected() -> None:
    fake = FakeSubsystem("power", default_snapshot("power"), script=lambda t, p: p.truth)  # type: ignore[arg-type,return-value]
    with pytest.raises(TypeError):
        fake.step(TICK_US, ENV, CONTROLS)


def test_reset_returns_to_initial_and_records_inputs() -> None:
    idle = default_snapshot("payload")
    payload = FakeSubsystem("payload", idle, script={0: replace_truth(idle, power_w=1.0)})
    controls = SpacecraftControls(payload=PayloadControls(enabled=True))
    payload.step(TICK_US, ENV, controls)
    assert payload.tick == 1
    assert payload.elapsed_us == TICK_US
    assert payload.last_env is ENV
    assert payload.last_controls is controls
    payload.reset(RngFactory(0))
    assert payload.snapshot() is idle
    assert (payload.tick, payload.elapsed_us, payload.reset_count) == (0, 0, 1)
    assert payload.last_env is None and payload.last_controls is None
    payload.step(TICK_US, ENV, CONTROLS)
    assert payload.snapshot().truth.power_w == 1.0


def test_step_validates_dt() -> None:
    fake = FakeSubsystem("power", default_snapshot("power"))
    with pytest.raises(TypeError):
        fake.step(0.1, ENV, CONTROLS)  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        fake.step(-1, ENV, CONTROLS)


def test_replace_readings_and_mirrored_rules() -> None:
    thermal = default_snapshot("thermal")
    hot = replace_readings(thermal, battery_c=55.0, over_temp=True)
    assert isinstance(hot, ThermalSnapshot)
    assert hot.readings.over_temp and hot.truth == thermal.truth
    with pytest.raises(TypeError, match="use replace_truth"):
        replace_readings(default_snapshot("comms"), sent_bytes=10)
    comms = replace_truth(default_snapshot("comms"), sent_bytes=10)
    assert comms.readings.sent_bytes == 10


# --- Fake-only stack ----------------------------------------------------------------------


def test_fake_subsystems_rejects_mismatched_override() -> None:
    with pytest.raises(ValueError, match="holds the 'thermal' fake"):
        fake_subsystems({"power": FakeSubsystem("thermal", default_snapshot("thermal"))})


def test_fake_only_stack_runs_all_five() -> None:
    fakes = fake_subsystems()
    stack = fake_stack(fakes)
    assert stack.names == STEP_ORDER
    stack.reset(RngFactory(7))
    states = _run(stack, 5)
    assert all(fake.tick == 5 and fake.reset_count == 1 for fake in fakes)
    assert tuple(states[-1].subsystems) == STEP_ORDER
    for name in STEP_ORDER:
        assert type(states[-1].subsystems[name]) is SNAPSHOT_TYPES[name]


@pytest.mark.parametrize(
    "row", CROSS_READS, ids=lambda r: f"{r['reader']}->{r['record']}.{r['field']}"
)
def test_each_documented_cross_read_works_through_spacecraft_state(row: dict[str, str]) -> None:
    record = getattr(snapshots, row["record"])
    source = next(n for n, k in SNAPSHOT_TYPES.items() if record in (k.truth_type, k.readings_type))
    kind = SNAPSHOT_TYPES[source]
    stack = fake_stack()
    stack.reset(RngFactory(0))
    state = _run(stack, 1)[-1]
    snap = state.get(source, kind)
    part = snap.truth if record is kind.truth_type else snap.readings
    assert type(part) is record
    getattr(part, row["field"])


def test_scripted_change_is_visible_to_a_reader_through_state() -> None:
    attitude = default_snapshot("attitude")
    tumbling = replace_readings(
        replace_truth(attitude, state=AttitudeState.TUMBLING, rate_dps=12.0),
        state=AttitudeState.TUMBLING,
        rate_dps=12.0,
    )
    stack = fake_stack(
        fake_subsystems({"attitude": FakeSubsystem("attitude", attitude, script={2: tumbling})})
    )
    stack.reset(RngFactory(0))
    states = _run(stack, 4)
    seen = [s.get("attitude", AttitudeSnapshot).readings.state for s in states]
    assert seen == [AttitudeState.STABILIZED] * 2 + [AttitudeState.TUMBLING] * 2
    payload_draw = states[-1].get("payload", PayloadSnapshot).truth.power_w
    assert payload_draw == 0.0


def test_fake_runs_are_repeatable() -> None:
    def run() -> list[SpacecraftState]:
        stack = fake_stack()
        stack.reset(RngFactory(3))
        return _run(stack, 3)

    assert [dict(s.subsystems) for s in run()] == [dict(s.subsystems) for s in run()]


# --- Determinism guard (#75) --------------------------------------------------------------


@pytest.mark.parametrize("module", ["spacecraft/fakes.py", "spacecraft/snapshots.py"])
def test_determinism_guard_covers_new_modules(module: str) -> None:
    scanned = {p.relative_to(PACKAGE_DIR).as_posix() for p in _simulation_modules()}
    assert module in scanned
    path: Path = PACKAGE_DIR / module
    assert find_violations(path.read_text(encoding="utf-8")) == []
