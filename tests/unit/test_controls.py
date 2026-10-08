"""Tests for flight computer to subsystem controls (ADR-0004)."""

import dataclasses
from collections.abc import Callable
from random import Random

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import (
    NO_RADIO_TRAFFIC,
    SENSOR_SUBSYSTEMS,
    AttitudeControls,
    PayloadControls,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SpacecraftControls,
    SpacecraftState,
    Subsystem,
    SubsystemSnapshot,
    SubsystemStack,
)
from pocketsat.targets.base import EnvironmentState

ENV = EnvironmentState()
TICK_US = 100_000


# --- Defaults and validation ----------------------------------------------------------


def test_defaults_describe_nominal_operation() -> None:
    controls = SpacecraftControls()
    assert controls.payload == PayloadControls(enabled=False, release_through_chunk_id=None)
    assert controls.radio.mode is RadioMode.RX_TX
    assert controls.attitude.enabled is True
    assert controls.frozen_sensors == frozenset()
    assert controls.extra_load_w == 0.0
    assert controls.radio_traffic == RadioTraffic(0, 0, 0)
    assert controls.radio_traffic is NO_RADIO_TRAFFIC  # shared, not rebuilt per record


def test_radio_traffic_is_per_tick_counts() -> None:
    traffic = RadioTraffic(sent_bytes=114, uplink_lost_count=2, outbound_suppressed_count=1)
    assert SpacecraftControls(radio_traffic=traffic).radio_traffic is traffic
    assert RadioTraffic() == NO_RADIO_TRAFFIC


def test_radio_modes() -> None:
    assert [m.name for m in RadioMode] == ["OFF", "RX_ONLY", "RX_TX"]


def test_sensor_subsystems() -> None:
    assert frozenset({"power", "thermal", "attitude"}) == SENSOR_SUBSYSTEMS


@pytest.mark.parametrize(
    ("record", "field"),
    [
        (SpacecraftControls(), "extra_load_w"),
        (SpacecraftControls(), "payload"),
        (PayloadControls(), "enabled"),
        (RadioControls(), "mode"),
        (AttitudeControls(), "enabled"),
        (SpacecraftControls(), "radio_traffic"),
        (RadioTraffic(), "sent_bytes"),
    ],
)
def test_records_are_frozen(record: object, field: str) -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        setattr(record, field, None)


def test_valid_overrides_accepted() -> None:
    controls = SpacecraftControls(
        payload=PayloadControls(enabled=True, release_through_chunk_id=0),
        radio=RadioControls(mode=RadioMode.OFF),
        attitude=AttitudeControls(enabled=False),
        frozen_sensors=frozenset({"power", "thermal", "attitude"}),
        extra_load_w=2,
    )
    assert controls.payload.release_through_chunk_id == 0
    assert controls.extra_load_w == 2


@pytest.mark.parametrize(
    ("build", "error"),
    [
        (lambda: PayloadControls(enabled=1), TypeError),  # type: ignore[arg-type]
        (lambda: PayloadControls(release_through_chunk_id=-1), ValueError),
        (lambda: PayloadControls(release_through_chunk_id=1.0), TypeError),  # type: ignore[arg-type]
        (lambda: PayloadControls(release_through_chunk_id=True), TypeError),
        (lambda: RadioControls(mode="rx_tx"), TypeError),  # type: ignore[arg-type]
        (lambda: AttitudeControls(enabled=None), TypeError),  # type: ignore[arg-type]
        (lambda: SpacecraftControls(payload=RadioControls()), TypeError),  # type: ignore[arg-type]
        (lambda: SpacecraftControls(frozen_sensors={"power"}), TypeError),  # type: ignore[arg-type]
        (lambda: SpacecraftControls(frozen_sensors=frozenset({"payload"})), ValueError),
        (lambda: SpacecraftControls(frozen_sensors=frozenset({"comms"})), ValueError),
        (lambda: SpacecraftControls(extra_load_w=-0.1), ValueError),
        (lambda: SpacecraftControls(extra_load_w=float("nan")), ValueError),
        (lambda: SpacecraftControls(extra_load_w=float("inf")), ValueError),
        (lambda: SpacecraftControls(extra_load_w="1"), TypeError),  # type: ignore[arg-type]
        (lambda: SpacecraftControls(extra_load_w=True), TypeError),
        (lambda: SpacecraftControls(radio_traffic=(0, 0, 0)), TypeError),  # type: ignore[arg-type]
        (lambda: RadioTraffic(sent_bytes=-1), ValueError),
        (lambda: RadioTraffic(uplink_lost_count=-1), ValueError),
        (lambda: RadioTraffic(outbound_suppressed_count=-1), ValueError),
        (lambda: RadioTraffic(sent_bytes=1.0), TypeError),  # type: ignore[arg-type]
        (lambda: RadioTraffic(uplink_lost_count=True), TypeError),
        (lambda: RadioTraffic(outbound_suppressed_count=None), TypeError),  # type: ignore[arg-type]
    ],
)
def test_invalid_controls_rejected(build: Callable[[], object], error: type[Exception]) -> None:
    with pytest.raises(error):
        build()


# --- Fakes ----------------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class RecordingSnapshot(SubsystemSnapshot):
    steps: int


class RecordingSubsystem:
    """Records the controls object it receives each step."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.received: list[SpacecraftControls] = []

    def reset(self, rng: RngFactory) -> None:
        self.received.clear()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        self.received.append(controls)

    def snapshot(self) -> RecordingSnapshot:
        return RecordingSnapshot(steps=len(self.received))


@dataclasses.dataclass(frozen=True)
class PayloadSnapshot(SubsystemSnapshot):
    acquiring: bool
    acquired_bytes: int


class FakePayload:
    """Obeys only its own controls record."""

    name = "payload"

    def __init__(self) -> None:
        self._acquiring = False
        self._acquired = 0

    def reset(self, rng: RngFactory) -> None:
        self._acquiring = False
        self._acquired = 0

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        self._acquiring = controls.payload.enabled
        if self._acquiring:
            self._acquired += 100

    def snapshot(self) -> PayloadSnapshot:
        return PayloadSnapshot(acquiring=self._acquiring, acquired_bytes=self._acquired)


# Snapshot convention (ADR-0004 §7): a truth record and a readings record.
@dataclasses.dataclass(frozen=True)
class PowerTruth:
    dissipation_w: float


@dataclasses.dataclass(frozen=True)
class PowerReadings:
    dissipation_w: float
    low_battery: bool


@dataclasses.dataclass(frozen=True)
class PowerSnapshot(SubsystemSnapshot):
    truth: PowerTruth
    readings: PowerReadings


class FakePower:
    """True dissipation plus noisy, possibly frozen readings and a flag."""

    name = "power"

    def __init__(self, dissipation_w: float, reported_offset_w: float, low_battery: bool) -> None:
        self._true = dissipation_w
        self.reported_offset_w = reported_offset_w
        self._low = low_battery
        self._readings = PowerReadings(dissipation_w=dissipation_w, low_battery=low_battery)

    def reset(self, rng: RngFactory) -> None:
        pass

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        if "power" not in controls.frozen_sensors:
            self._readings = PowerReadings(
                dissipation_w=self._true + self.reported_offset_w, low_battery=self._low
            )

    def snapshot(self) -> PowerSnapshot:
        return PowerSnapshot(truth=PowerTruth(self._true), readings=self._readings)


@dataclasses.dataclass(frozen=True)
class ThermalSnapshot(SubsystemSnapshot):
    heat_j: float
    inhibited: bool


class FakeThermal:
    """Reads power's truth for physics and power's readings for a decision.

    The real cross-subsystem read mechanism is defined in #36; here the test wires a
    state source directly to exercise the ADR-0004 rule.
    """

    name = "thermal"

    def __init__(self, state: Callable[[], SpacecraftState]) -> None:
        self._state = state
        self._heat = 0.0
        self._inhibited = False

    def reset(self, rng: RngFactory) -> None:
        self._heat = 0.0
        self._inhibited = False

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        power = self._state().get("power", PowerSnapshot)
        self._heat += power.truth.dissipation_w * dt_us / 1_000_000  # physics: true value
        self._inhibited = power.readings.low_battery  # decision: reported flag

    def snapshot(self) -> ThermalSnapshot:
        return ThermalSnapshot(heat_j=self._heat, inhibited=self._inhibited)


@dataclasses.dataclass(frozen=True)
class NoisySnapshot(SubsystemSnapshot):
    value: float


class NoisySubsystem:
    """Draws noise each step, scaled by whether attitude control is on."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._rng = Random(0)
        self._value = 0.0

    def reset(self, rng: RngFactory) -> None:
        self._rng = rng.stream(f"spacecraft.{self.name}.noise")
        self._value = 0.0

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        gain = 1.0 if controls.attitude.enabled else 3.0
        self._value += gain * self._rng.random() + controls.extra_load_w

    def snapshot(self) -> NoisySnapshot:
        return NoisySnapshot(value=self._value)


# --- Stack behavior -------------------------------------------------------------------


def test_fakes_satisfy_protocol() -> None:
    assert isinstance(RecordingSubsystem("power"), Subsystem)
    assert isinstance(FakePayload(), Subsystem)


def test_stack_passes_same_controls_to_every_subsystem() -> None:
    subsystems = [RecordingSubsystem(name) for name in ("comms", "power", "thermal")]
    stack = SubsystemStack(subsystems)
    stack.reset(RngFactory(1))
    first = SpacecraftControls(radio=RadioControls(mode=RadioMode.RX_ONLY))
    second = SpacecraftControls(extra_load_w=1.5)
    stack.step(TICK_US, ENV, first)
    stack.step(TICK_US, ENV, second)
    for subsystem in subsystems:
        assert subsystem.received[0] is first
        assert subsystem.received[1] is second


def test_stack_rejects_non_controls() -> None:
    stack = SubsystemStack([RecordingSubsystem("power")])
    with pytest.raises(TypeError, match="SpacecraftControls"):
        stack.step(TICK_US, ENV, None)  # type: ignore[arg-type]


def test_subsystem_obeys_its_own_record() -> None:
    payload = FakePayload()
    payload.step(TICK_US, ENV, SpacecraftControls(payload=PayloadControls(enabled=True)))
    assert payload.snapshot() == PayloadSnapshot(acquiring=True, acquired_bytes=100)
    payload.step(TICK_US, ENV, SpacecraftControls())
    assert payload.snapshot() == PayloadSnapshot(acquiring=False, acquired_bytes=100)


def test_subsystem_ignores_other_subsystems_records() -> None:
    def run(controls: SpacecraftControls) -> PayloadSnapshot:
        payload = FakePayload()
        for _ in range(5):
            payload.step(TICK_US, ENV, controls)
        return payload.snapshot()

    baseline = run(SpacecraftControls(payload=PayloadControls(enabled=True)))
    other_records_changed = run(
        SpacecraftControls(
            payload=PayloadControls(enabled=True),
            radio=RadioControls(mode=RadioMode.OFF),
            attitude=AttitudeControls(enabled=False),
            frozen_sensors=frozenset({"power", "thermal"}),
            extra_load_w=5.0,
        )
    )
    assert other_records_changed == baseline


def test_physics_reads_truth_and_decisions_read_readings() -> None:
    # Reported dissipation is offset from the truth, and the reported flag says low battery.
    power = FakePower(dissipation_w=4.0, reported_offset_w=10.0, low_battery=True)
    states: list[SpacecraftState] = []
    thermal = FakeThermal(state=lambda: states[-1])
    stack = SubsystemStack([power, thermal])
    stack.reset(RngFactory(0))
    states.append(stack.snapshot())
    for _ in range(10):
        stack.step(TICK_US, ENV, SpacecraftControls())
        states.append(stack.snapshot())
    final = states[-1]
    assert final.get("power", PowerSnapshot).readings.dissipation_w == 14.0
    thermal_snap = final.get("thermal", ThermalSnapshot)
    assert thermal_snap.heat_j == pytest.approx(10 * 4.0 * 0.1)  # truth, not 14 W
    assert thermal_snap.inhibited is True  # from the readings flag


def test_frozen_sensors_override_holds_readings() -> None:
    power = FakePower(dissipation_w=1.0, reported_offset_w=0.5, low_battery=False)
    power.step(TICK_US, ENV, SpacecraftControls())
    live = power.snapshot().readings
    power.reported_offset_w = 99.0  # the live reading would change...
    power.step(TICK_US, ENV, SpacecraftControls(frozen_sensors=frozenset({"power"})))
    assert power.snapshot().readings == live  # ...but the frozen reading holds
    power.step(TICK_US, ENV, SpacecraftControls())
    assert power.snapshot().readings.dissipation_w == 100.0  # released: live again


def test_determinism_unchanged_with_varying_controls() -> None:
    schedule = [
        SpacecraftControls(),
        SpacecraftControls(attitude=AttitudeControls(enabled=False)),
        SpacecraftControls(extra_load_w=0.25),
        SpacecraftControls(radio=RadioControls(mode=RadioMode.RX_ONLY)),
    ]

    def run(seed: int) -> list[SpacecraftState]:
        stack = SubsystemStack([NoisySubsystem("power"), NoisySubsystem("attitude")])
        stack.reset(RngFactory(seed))
        states = []
        for i in range(200):
            stack.step(TICK_US, ENV, schedule[i % len(schedule)])
            states.append(stack.snapshot())
        return states

    assert run(42) == run(42)
    assert run(42) != run(43)
