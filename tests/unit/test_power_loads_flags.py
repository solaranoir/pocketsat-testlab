"""Tests for the power subsystem's loads, sensor noise, SOC estimate, flags, and
``sensor_freeze`` (#36). The battery and solar model itself is tested in
``test_power.py``."""

import dataclasses
import math
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft import (
    AttitudeControls,
    PayloadControls,
    Power,
    PowerConfig,
    PowerInitial,
    PowerSnapshot,
    PowerTruth,
    RadioControls,
    RadioMode,
    SnapshotBoard,
    SpacecraftControls,
    SubsystemStack,
)
from pocketsat.spacecraft.fakes import (
    FakeSubsystem,
    default_snapshot,
    fake_subsystems,
    replace_truth,
)
from pocketsat.spacecraft.power import NOISE_STREAM
from pocketsat.targets.base import EnvironmentState

TICK_US = 100_000
SUN = EnvironmentState(sunlit=True)
ECLIPSE = EnvironmentState(sunlit=False)
CONTROLS = SpacecraftControls()
FROZEN = SpacecraftControls(frozen_sensors=frozenset({"power"}))
CONFIG = PowerConfig(base_load_w=2.0)
"""The settings these model tests use: the defaults, except a round 2 W base load, so
the hand-computed expectations stay independent of the budget's calibration (#72)."""

DRAW_FIELDS = {
    "thermal": "heater_power_w",
    "attitude": "control_power_w",
    "payload": "power_w",
    "comms": "transmit_power_w",
}
"""The draw power reads from each downstream subsystem's truth record."""

SOC_TOLERANCE = 6 * 0.008 / 2.4
"""Documented bound on |estimate - true SOC| at nominal noise: the voltage noise is
bounded at 6 sigma (sigma = 0.008 V) over the 2.4 V span, so 0.02."""


def _power(soc: float = 0.5, config: PowerConfig = CONFIG, seed: int = 1) -> Power:
    power = Power(config, PowerInitial(soc=soc))
    power.reset(RngFactory(seed))
    return power


def _truth(power: Power) -> PowerTruth:
    return power.snapshot().truth


def _flags(power: Power) -> tuple[bool, bool]:
    readings = power.snapshot().readings
    return readings.low_battery, readings.critical_battery


def _run(
    power: Power,
    ticks: int,
    env: EnvironmentState = SUN,
    controls: SpacecraftControls = CONTROLS,
) -> None:
    for _ in range(ticks):
        power.step(TICK_US, env, controls)


def _with_draw(name: str, watts: float) -> Any:
    return replace_truth(default_snapshot(name), **{DRAW_FIELDS[name]: watts})


def _others(**overrides: FakeSubsystem[Any]) -> list[FakeSubsystem[Any]]:
    """Fakes of the four subsystems power reads, holding the defaults unless overridden."""
    return [f for f in fake_subsystems(overrides) if f.name != "power"]


def _draw_fakes(
    payload_w: float = 0.0,
    control_w: float = 0.0,
    transmit_w: float = 0.0,
    heater_w: float = 0.0,
) -> list[FakeSubsystem[Any]]:
    return _others(
        thermal=FakeSubsystem("thermal", _with_draw("thermal", heater_w)),
        attitude=FakeSubsystem("attitude", _with_draw("attitude", control_w)),
        payload=FakeSubsystem("payload", _with_draw("payload", payload_w)),
        comms=FakeSubsystem("comms", _with_draw("comms", transmit_w)),
    )


def _stack(fakes: list[FakeSubsystem[Any]], soc: float = 0.5) -> tuple[SubsystemStack, Power]:
    board = SnapshotBoard()
    power = Power(CONFIG, PowerInitial(soc=soc), reader=board)
    stack = SubsystemStack([power, *fakes], board=board)
    stack.reset(RngFactory(1))
    return stack, power


# --- Loads -----------------------------------------------------------------------------


def test_total_load_sums_every_draw() -> None:
    fakes = _draw_fakes(payload_w=1.25, control_w=0.5, transmit_w=1.0, heater_w=2.0)
    stack, power = _stack(fakes)
    stack.step(TICK_US, ECLIPSE, SpacecraftControls(extra_load_w=0.25))
    truth = _truth(power)
    assert truth.total_load_w == 2.0 + 1.25 + 0.5 + 1.0 + 2.0 + 0.25
    assert truth.net_power_w == -7.0
    assert truth.battery_current_a == pytest.approx(-7.0 / truth.bus_v)


def test_default_fakes_load() -> None:
    # The shared fakes: attitude control 0.5 W, transmit 1.0 W, payload off, heater off.
    stack, power = _stack(_others())
    stack.step(TICK_US, SUN, CONTROLS)
    assert _truth(power).total_load_w == 3.5


@pytest.mark.parametrize("name", sorted(DRAW_FIELDS))
def test_downstream_draw_lags_one_tick(name: str) -> None:
    # Every subsystem power reads steps after it, so a draw that changes in tick 1 is
    # seen by power in tick 2 (#85, ADR-0004 §3).
    fake = FakeSubsystem(name, _with_draw(name, 0.0), script={1: _with_draw(name, 3.0)})
    stack, power = _stack([f for f in _draw_fakes() if f.name != name] + [fake])
    loads = []
    for _ in range(4):
        stack.step(TICK_US, SUN, CONTROLS)
        loads.append(_truth(power).total_load_w)
    assert loads == [2.0, 2.0, 5.0, 5.0]


def test_draw_switching_off_also_lags_one_tick() -> None:
    payload = FakeSubsystem(
        "payload", _with_draw("payload", 2.0), script={2: _with_draw("payload", 0.0)}
    )
    stack, power = _stack(_others(payload=payload))
    loads = []
    for _ in range(5):
        stack.step(TICK_US, SUN, CONTROLS)
        loads.append(_truth(power).total_load_w)
    assert loads == [5.5, 5.5, 5.5, 3.5, 3.5]


def test_draws_are_read_from_truth() -> None:
    # Thermal and attitude readings carry no draw, so the draw must come from truth;
    # changing only their readings changes nothing.
    thermal = dataclasses.replace(
        _with_draw("thermal", 1.5),
        readings=dataclasses.replace(default_snapshot("thermal").readings, under_temp=True),
    )
    stack, power = _stack(_others(thermal=FakeSubsystem("thermal", thermal)))
    stack.step(TICK_US, SUN, CONTROLS)
    assert _truth(power).total_load_w == 5.0


def test_without_reader_no_downstream_draws() -> None:
    power = _power()
    power.step(TICK_US, SUN, SpacecraftControls(extra_load_w=1.0))
    assert _truth(power).total_load_w == 3.0


@pytest.mark.parametrize("missing", sorted(DRAW_FIELDS))
def test_reader_missing_a_subsystem_raises(missing: str) -> None:
    stack, _ = _stack([f for f in _others() if f.name != missing])
    with pytest.raises(KeyError, match=missing):
        stack.step(TICK_US, SUN, CONTROLS)


def test_load_does_not_depend_on_commands() -> None:
    # Power never reads the mode or the subsystem commands: only reported draws count.
    commanded = SpacecraftControls(
        payload=PayloadControls(enabled=True),
        radio=RadioControls(mode=RadioMode.OFF),
        attitude=AttitudeControls(enabled=False),
    )
    results = []
    for controls in (CONTROLS, commanded):
        stack, power = _stack(_others())
        for _ in range(50):
            stack.step(TICK_US, SUN, controls)
        results.append(power.snapshot())
    assert results[0] == results[1]


def test_override_still_beats_drain_with_draws() -> None:
    fakes = _draw_fakes(payload_w=2.0, transmit_w=1.0)
    stack, power = _stack(fakes, soc=0.8)
    pinned = EnvironmentState(sunlit=False, battery_soc_override=0.6)
    for _ in range(50):
        stack.step(TICK_US, pinned, SpacecraftControls(extra_load_w=5.0))
        truth = _truth(power)
        assert truth.soc == 0.6
        assert truth.total_load_w == 10.0
        assert truth.battery_current_a == pytest.approx(-10.0 / truth.bus_v)


# --- Signs -----------------------------------------------------------------------------


def test_signs_at_net_charging_point() -> None:
    stack, power = _stack(_others())
    stack.step(TICK_US, SUN, CONTROLS)
    snap = power.snapshot()
    assert snap.truth.generation_w == 8.0
    assert snap.truth.total_load_w == 3.5
    assert snap.truth.net_power_w == 4.5
    assert snap.truth.battery_current_a > 0
    assert snap.readings.battery_current_a > 0
    assert snap.truth.soc > 0.5


def test_signs_at_net_discharging_point() -> None:
    stack, power = _stack(_others())
    stack.step(TICK_US, ECLIPSE, CONTROLS)
    snap = power.snapshot()
    assert snap.truth.generation_w == 0.0
    assert snap.truth.total_load_w == 3.5
    assert snap.truth.net_power_w == -3.5
    assert snap.truth.battery_current_a < 0
    assert snap.readings.battery_current_a < 0
    assert snap.truth.soc < 0.5


def test_signs_in_sunlight_with_heavy_draws() -> None:
    # Sunlit but net discharging: generation and every load stay non-negative.
    stack, power = _stack(_draw_fakes(payload_w=2.0, control_w=0.5, transmit_w=3.0, heater_w=1.5))
    stack.step(TICK_US, SUN, CONTROLS)
    truth = _truth(power)
    assert truth.generation_w == 8.0
    assert truth.total_load_w == 9.0
    assert truth.net_power_w == -1.0
    assert truth.battery_current_a < 0


# --- Noise -----------------------------------------------------------------------------


def _readings_run(
    seed: int, scale: float, ticks: int = 200, soc: float = 0.6
) -> list[PowerSnapshot]:
    power = _power(soc=soc, seed=seed)
    out = []
    for i in range(ticks):
        env = EnvironmentState(sunlit=(i // 50) % 2 == 0, sensor_noise_scale=scale)
        power.step(TICK_US, env, CONTROLS)
        out.append(power.snapshot())
    return out


def test_requests_the_noise_stream() -> None:
    rng = RngFactory(7)
    Power(CONFIG, PowerInitial()).reset(rng)
    assert NOISE_STREAM == "spacecraft.power.noise"
    assert list(rng.stream_seeds) == [NOISE_STREAM]


def test_step_before_reset_raises() -> None:
    with pytest.raises(RuntimeError, match="reset"):
        Power(CONFIG, PowerInitial()).step(TICK_US, SUN, CONTROLS)


def test_noise_reproducible_for_a_seed() -> None:
    first = _readings_run(seed=42, scale=1.0)
    assert first == _readings_run(seed=42, scale=1.0)
    other = _readings_run(seed=43, scale=1.0)
    assert [s.truth for s in first] == [s.truth for s in other]
    assert [s.readings for s in first] != [s.readings for s in other]


def test_reset_restarts_the_noise() -> None:
    power = Power(CONFIG, PowerInitial(soc=0.6))
    runs = []
    for _ in range(2):
        power.reset(RngFactory(5))
        run = []
        for _ in range(20):
            power.step(TICK_US, SUN, CONTROLS)
            run.append(power.snapshot())
        runs.append(run)
    assert runs[0] == runs[1]


def test_noise_is_exact_at_scale_zero() -> None:
    for snap in _readings_run(seed=42, scale=0.0):
        assert snap.readings.bus_v == snap.truth.bus_v
        assert snap.readings.battery_current_a == snap.truth.battery_current_a
        assert snap.readings.soc == pytest.approx(snap.truth.soc, abs=1e-12)


def test_noise_doubles_at_scale_two() -> None:
    nominal = _readings_run(seed=42, scale=1.0)
    double = _readings_run(seed=42, scale=2.0)
    for one, two in zip(nominal, double, strict=True):
        assert one.truth == two.truth
        dv1 = one.readings.bus_v - one.truth.bus_v
        dv2 = two.readings.bus_v - two.truth.bus_v
        da1 = one.readings.battery_current_a - one.truth.battery_current_a
        da2 = two.readings.battery_current_a - two.truth.battery_current_a
        assert dv2 == pytest.approx(2.0 * dv1, abs=1e-12)
        assert da2 == pytest.approx(2.0 * da1, abs=1e-12)
    assert any(s.readings.bus_v != s.truth.bus_v for s in double)


def test_noise_has_configured_spread() -> None:
    snaps = _readings_run(seed=3, scale=1.0, ticks=5000)
    dv = [s.readings.bus_v - s.truth.bus_v for s in snaps]
    da = [s.readings.battery_current_a - s.truth.battery_current_a for s in snaps]
    for errors, sigma in ((dv, 0.008), (da, 0.01)):
        mean = sum(errors) / len(errors)
        std = math.sqrt(sum((e - mean) ** 2 for e in errors) / len(errors))
        assert abs(mean) < 0.1 * sigma
        assert std == pytest.approx(sigma, rel=0.05)
        assert max(abs(e) for e in errors) < 6 * sigma


# --- SOC estimate ----------------------------------------------------------------------


def test_soc_estimate_tracks_true_soc_within_tolerance() -> None:
    # 12 W of eclipse load from 0.98 down to about 0.15, then 6 W net charge back up.
    power = _power(soc=0.98, seed=11)
    drain = SpacecraftControls(extra_load_w=10.0)
    errors = []
    lowest = 1.0
    for i in range(70_000):
        down = i < 50_000
        power.step(TICK_US, ECLIPSE if down else SUN, drain if down else CONTROLS)
        snap = power.snapshot()
        errors.append(snap.readings.soc - snap.truth.soc)
        lowest = min(lowest, snap.truth.soc)
        assert 0.0 <= snap.readings.soc <= 1.0
    assert lowest < 0.2  # it covered most of the range
    assert _truth(power).soc > 0.3
    assert max(abs(e) for e in errors) <= SOC_TOLERANCE
    assert abs(sum(errors) / len(errors)) < 0.001


def test_soc_estimate_uses_the_reported_voltage() -> None:
    power = _power()
    for _ in range(10):
        power.step(TICK_US, SUN, CONTROLS)
        readings = power.snapshot().readings
        assert readings.soc == pytest.approx((readings.bus_v - 6.0) / 2.4, abs=1e-12)


def test_soc_estimate_clamped_to_range() -> None:
    noisy = dataclasses.replace(CONFIG, voltage_noise_v=0.5)
    for soc in (0.0, 1.0):
        power = _power(soc=soc, config=noisy, seed=2)
        estimates = []
        for _ in range(200):
            power.step(TICK_US, EnvironmentState(battery_soc_override=soc), CONTROLS)
            estimates.append(power.snapshot().readings.soc)
        assert min(estimates) >= 0.0
        assert max(estimates) <= 1.0
        assert soc in estimates


# --- Flags and hysteresis --------------------------------------------------------------


def _expected_flags(low: bool, critical: bool, estimate: float) -> tuple[bool, bool]:
    """The documented hysteresis rule, written independently of the model."""
    low_set, low_clear = CONFIG.low_battery_soc, CONFIG.low_battery_clear_soc
    critical_set, critical_clear = CONFIG.critical_battery_soc, CONFIG.critical_battery_clear_soc
    return (
        estimate <= low_clear if low else estimate < low_set,
        estimate <= critical_clear if critical else estimate < critical_set,
    )


@pytest.mark.parametrize(
    ("soc", "flags"),
    [(0.5, (False, False)), (0.32, (False, False)), (0.25, (True, False)), (0.1, (True, True))],
)
def test_starting_readings_and_flags(soc: float, flags: tuple[bool, bool]) -> None:
    power = _power(soc=soc)
    snap = power.snapshot()
    assert _flags(power) == flags
    assert snap.readings.bus_v == snap.truth.bus_v
    assert snap.readings.battery_current_a == snap.truth.battery_current_a == 0.0
    assert snap.readings.soc == snap.truth.soc == soc


def test_thresholds_and_hysteresis() -> None:
    power = _power(soc=0.5)
    # (true SOC pinned for one tick, flags expected), with noise off.
    sequence = [
        (0.50, (False, False)),
        (0.31, (False, False)),  # above the low set threshold
        (0.29, (True, False)),  # below 0.30: low sets
        (0.33, (True, False)),  # inside the band: low holds
        (0.349, (True, False)),
        (0.36, (False, False)),  # above 0.35: low clears
        (0.32, (False, False)),  # inside the band: stays clear
        (0.16, (True, False)),
        (0.14, (True, True)),  # below 0.15: critical sets
        (0.18, (True, True)),  # inside the critical band: holds
        (0.21, (True, False)),  # above 0.20: critical clears, low holds
        (0.17, (True, False)),  # inside the critical band: stays clear
        (0.36, (False, False)),
        (0.05, (True, True)),  # both set in one tick
        (0.40, (False, False)),  # both clear in one tick
    ]
    for soc, flags in sequence:
        env = EnvironmentState(battery_soc_override=soc, sensor_noise_scale=0.0)
        power.step(TICK_US, env, CONTROLS)
        assert _flags(power) == flags, soc


@pytest.mark.parametrize("soc", [0.30, 0.325, 0.35, 0.15, 0.175, 0.20])
def test_flags_never_flap_with_true_soc_at_a_threshold(soc: float) -> None:
    # The hysteresis band (0.05) exceeds the estimate's full noise spread (0.04), so
    # with the true SOC pinned anywhere near a threshold each flag changes at most once.
    power = _power(soc=soc)
    changes = 0
    previous = _flags(power)
    for _ in range(10_000):
        power.step(TICK_US, EnvironmentState(battery_soc_override=soc), CONTROLS)
        flags = _flags(power)
        changes += (flags[0] != previous[0]) + (flags[1] != previous[1])
        previous = flags
    assert changes <= 1


def test_flags_follow_the_hysteresis_rule_down_and_up() -> None:
    power = _power(soc=0.45)
    drain = SpacecraftControls(extra_load_w=6.0)
    low, critical = _flags(power)
    changes = []
    for i in range(90_000):
        # 8 W of eclipse load for about 1.1 h (down to near-empty), then 6 W net charge
        # for about 1.4 h (back above both clear thresholds).
        down = i < 40_000
        power.step(TICK_US, ECLIPSE if down else SUN, drain if down else CONTROLS)
        readings = power.snapshot().readings
        expected = _expected_flags(low, critical, readings.soc)
        assert (readings.low_battery, readings.critical_battery) == expected
        assert readings.low_battery or not readings.critical_battery  # critical => low
        if expected != (low, critical):
            changes.append(expected)
        low, critical = expected
    # Exactly one change at each crossing: low sets, critical sets, critical clears,
    # low clears.
    assert changes == [(True, False), (True, True), (True, False), (False, False)]


# --- sensor_freeze ---------------------------------------------------------------------


def test_freeze_holds_readings_exactly_while_truth_changes() -> None:
    power = _power(soc=0.7)
    _run(power, 10, ECLIPSE)
    held = power.snapshot().readings
    truths = []
    for _ in range(100):
        power.step(TICK_US, ECLIPSE, FROZEN)
        snap = power.snapshot()
        assert snap.readings == held
        truths.append(snap.truth)
    assert len({t.soc for t in truths}) == 100  # the truth kept evolving
    # Live readings resume on release.
    power.step(TICK_US, SUN, CONTROLS)
    live = power.snapshot()
    assert live.readings != held
    assert live.readings.bus_v == pytest.approx(live.truth.bus_v, abs=6 * 0.008)


def test_freeze_reuses_the_held_readings_record() -> None:
    power = _power()
    power.step(TICK_US, SUN, CONTROLS)
    held = power.snapshot().readings
    power.step(TICK_US, SUN, FROZEN)
    assert power.snapshot().readings is held


def test_freeze_from_the_first_tick_holds_the_starting_readings() -> None:
    power = _power(soc=0.7)
    start = power.snapshot().readings
    _run(power, 50, ECLIPSE, FROZEN)
    assert power.snapshot().readings == start


def test_freezing_another_subsystem_does_not_freeze_power() -> None:
    power = _power(soc=0.7)
    power.step(TICK_US, SUN, CONTROLS)
    before = power.snapshot().readings
    power.step(TICK_US, SUN, SpacecraftControls(frozen_sensors=frozenset({"thermal"})))
    assert power.snapshot().readings != before


def test_frozen_reading_hides_a_threshold_crossing() -> None:
    power = _power(soc=0.40)
    power.step(TICK_US, ECLIPSE, CONTROLS)
    held = power.snapshot().readings
    assert _flags(power) == (False, False)
    # 42 W for 10 minutes drains 7 Wh of 20 Wh, 0.35 of SOC: through both thresholds.
    drain = SpacecraftControls(extra_load_w=40.0, frozen_sensors=frozenset({"power"}))
    _run(power, 6000, ECLIPSE, drain)
    snap = power.snapshot()
    assert snap.truth.soc < CONFIG.critical_battery_soc - SOC_TOLERANCE
    assert snap.readings == held
    assert _flags(power) == (False, False)
    # On release, the flags see the real state.
    power.step(TICK_US, ECLIPSE, CONTROLS)
    assert _flags(power) == (True, True)


def test_noise_after_release_does_not_depend_on_the_freeze() -> None:
    # Noise is drawn every tick, frozen or not, so after a release the readings match a
    # run that was never frozen.
    plain = _power(soc=0.7)
    frozen = _power(soc=0.7)
    for i in range(100):
        plain.step(TICK_US, SUN, CONTROLS)
        frozen.step(TICK_US, SUN, FROZEN if 20 <= i < 60 else CONTROLS)
    assert plain.snapshot() == frozen.snapshot()


# --- Configuration (invalid settings: test_power.py) --------------------------------


def test_custom_thresholds_and_zero_noise_are_valid() -> None:
    config = PowerConfig(
        voltage_noise_v=0.0,
        current_noise_a=0.0,
        low_battery_soc=0.5,
        low_battery_clear_soc=0.6,
        critical_battery_soc=0.5,
        critical_battery_clear_soc=0.6,
    )
    power = _power(soc=0.55, config=config)
    power.step(TICK_US, EnvironmentState(battery_soc_override=0.45), CONTROLS)
    assert _flags(power) == (True, True)
