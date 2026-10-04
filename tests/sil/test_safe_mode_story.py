"""Safe-mode and fault rules against the real subsystems (#48).

The five real subsystems (``_reference_profile.real_stack``) step under the controls the
real :class:`FlightComputer` produces, one tick late, as ``SilTarget`` will run them
(ADR-0004 §2; #59 is not merged, so :func:`run` is a minimal stand-in for its loop and
its fault merge, precedence fault > flight computer). The flight computer sees only the
readings.

Commands (#51) and BOOT_COMPLETE (#49) don't exist yet, so :class:`ScriptedComputer`
raises scripted mode events in the execute-commands phase. Without a scripted
BOOT_COMPLETE the flight computer stays in BOOT, which exercises the BOOT → SAFE path.

The cold case is #72's contract-suite environment, -150 °C in sunlight and eclipse
(``docs/power-thermal-budget.md``, "Cold case"): ``under_temp`` sets about 2 minutes in.
"""

import dataclasses
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

import pytest
from _reference_profile import real_stack

from pocketsat.core.clock import SimClock
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import (
    EventKind,
    FlightComputer,
    Mode,
    ModeEvent,
    Outcome,
    RejectReason,
    SpacecraftReadings,
    Transition,
    controls_for_mode,
)
from pocketsat.flight.computer import TickContext
from pocketsat.flight.safety import DEFAULT_SAFETY_CONFIG
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    PowerInitial,
    PowerSnapshot,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    ThermalSnapshot,
)
from pocketsat.targets.base import EnvironmentState

SEED = 1
N = DEFAULT_SAFETY_CONFIG.sustain_tick_count
US_PER_S = 1_000_000

NOMINAL_ENV = NominalEnvironment()
COLD_ENV = NominalEnvironment(ambient_temp_c=-150.0, eclipse_ambient_temp_c=-150.0)
"""#72's cold case: -150 °C in sunlight and eclipse."""

BOOT_COMPLETE = ModeEvent(EventKind.BOOT_COMPLETE)
SET_NOMINAL = ModeEvent(EventKind.SET_MODE, Mode.NOMINAL)
SET_SCIENCE = ModeEvent(EventKind.SET_MODE, Mode.SCIENCE)


class ScriptedComputer(FlightComputer):
    """Raises ``script[k]``'s mode events in the execute-commands phase of the k-th
    tick (stand-in for #49 and #51)."""

    def __init__(self, script: Mapping[int, Sequence[ModeEvent]]) -> None:
        self.script = script
        self.tick_count = 0
        self.command_results: dict[int, list[Transition]] = {}
        super().__init__()

    def _execute_commands(self, tick: TickContext) -> None:
        tick.mode_events.extend(self.script.get(self.tick_count, ()))

    def _update_mode(self, tick: TickContext) -> None:
        super()._update_mode(tick)
        scripted = len(self.script.get(self.tick_count, ()))
        if scripted:
            self.command_results[self.tick_count] = tick.transitions[:scripted]
        self.tick_count += 1


@dataclass(frozen=True)
class Overrides:
    """Fault overrides for one tick, merged over the flight computer's controls."""

    frozen_sensors: frozenset[str] = frozenset()
    extra_load_w: float = 0.0


NO_OVERRIDES = Overrides()


@dataclass
class Run:
    """Per-tick record of a run. Index k is tick k (the k-th flight computer step)."""

    modes: list[Mode] = field(default_factory=list)
    states: list[SpacecraftState] = field(default_factory=list)
    controls: list[SpacecraftControls] = field(default_factory=list)
    """Controls the subsystems stepped under in tick k (after the merge)."""
    command_results: dict[int, list[Transition]] = field(default_factory=dict)

    def power(self, k: int) -> PowerSnapshot:
        return self.states[k].get("power", PowerSnapshot)

    def thermal(self, k: int) -> ThermalSnapshot:
        return self.states[k].get("thermal", ThermalSnapshot)

    def first(self, predicate: Callable[[int], bool]) -> int:
        return next(k for k in range(len(self.modes)) if predicate(k))


def run(
    ticks: int,
    *,
    environment: Callable[[int], NominalEnvironment] = lambda now_us: NOMINAL_ENV,
    overrides: Callable[[int], Overrides] = lambda now_us: NO_OVERRIDES,
    script: Mapping[int, Sequence[ModeEvent]] | None = None,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
    stop: Callable[[Run], bool] | None = None,
) -> Run:
    """Step the real stack and the flight computer together, as ``SilTarget`` will."""
    stack = real_stack(SEED, NOMINAL_CONFIG, initial)
    clock = SimClock()
    fc = ScriptedComputer(script or {})
    controls = fc.reset()
    record = Run(command_results=fc.command_results)
    for _ in range(ticks):
        now_us = clock.now_us
        extra = overrides(now_us)
        merged = dataclasses.replace(
            controls,
            frozen_sensors=controls.frozen_sensors | extra.frozen_sensors,
            extra_load_w=controls.extra_load_w + extra.extra_load_w,
        )
        env: EnvironmentState = environment(now_us).state_at(now_us)
        stack.step(clock.tick_us, env, merged)
        clock.advance_one_tick()
        state = stack.snapshot()
        controls = fc.step((), SpacecraftReadings.from_state(state), clock.now_us).controls
        record.modes.append(fc.mode)
        record.states.append(state)
        record.controls.append(merged)
        if stop is not None and stop(record):
            break
    return record


def ticks_of(seconds: float) -> int:
    return round(seconds * US_PER_S) // SimClock().tick_us


def every(
    period_s: float, event: ModeEvent, start_s: float, end_s: float
) -> dict[int, list[ModeEvent]]:
    """``event`` scripted every ``period_s`` from ``start_s`` to ``end_s``."""
    step = ticks_of(period_s)
    return {k: [event] for k in range(ticks_of(start_s), ticks_of(end_s), step)}


def cold_until(end_s: float) -> Callable[[int], NominalEnvironment]:
    return lambda now_us: COLD_ENV if now_us < end_s * US_PER_S else NOMINAL_ENV


# --- Cold case: BOOT -> SAFE from physics, heater keeps running -----------------------


def test_cold_case_goes_from_boot_to_safe_after_n_ticks_of_under_temp() -> None:
    result = run(ticks_of(240), environment=lambda now_us: COLD_ENV)
    flagged = result.first(lambda k: result.thermal(k).readings.under_temp)
    assert flagged < ticks_of(300), "under_temp within 5 minutes (#72)"
    entered = result.first(lambda k: result.modes[k] is Mode.SAFE)
    assert entered == flagged + N - 1
    assert result.modes[:entered] == [Mode.BOOT] * entered
    assert all(m is Mode.SAFE for m in result.modes[entered:])
    # SAFE's controls apply from the next tick.
    assert result.controls[entered] == controls_for_mode(Mode.BOOT)
    assert result.controls[entered + 1] == controls_for_mode(Mode.SAFE)


def test_survival_heater_keeps_running_in_safe_under_sustained_cold() -> None:
    result = run(ticks_of(12 * 60), environment=lambda now_us: COLD_ENV)
    entered = result.first(lambda k: result.modes[k] is Mode.SAFE)
    heater_on = result.first(lambda k: result.thermal(k).truth.heater_on)
    for k in range(max(entered, heater_on), len(result.modes)):
        assert result.modes[k] is Mode.SAFE
        truth = result.thermal(k).truth
        assert truth.heater_on, f"heater off in SAFE at tick {k}"
        assert truth.heater_power_w > 0
    # Many minutes of SAFE with the heater on, and the battery still below its limit.
    assert len(result.modes) - max(entered, heater_on) > ticks_of(8 * 60)
    last = result.thermal(len(result.modes) - 1).truth
    assert last.battery_c < NOMINAL_CONFIG.thermal.battery_under_temp_c


# --- Reported, not true: a frozen sensor delays or prevents SAFE ------------------------


def thermal_frozen_until(end_s: float | None) -> Callable[[int], Overrides]:
    frozen = Overrides(frozen_sensors=frozenset({"thermal"}))
    return lambda now_us: frozen if end_s is None or now_us < end_s * US_PER_S else NO_OVERRIDES


def below_under_temp(result: Run, k: int) -> bool:
    truth = result.thermal(k).truth
    thermal = NOMINAL_CONFIG.thermal
    return (
        truth.battery_c < thermal.battery_under_temp_c
        or truth.electronics_c < thermal.electronics_under_temp_c
    )


def test_frozen_thermal_sensor_delays_safe_entry() -> None:
    release_s = 300
    result = run(
        ticks_of(release_s + 10),
        environment=lambda now_us: COLD_ENV,
        overrides=thermal_frozen_until(release_s),
    )
    release = ticks_of(release_s)
    # While frozen, the truth is past the threshold for minutes but the readings say
    # nothing, so the flight computer stays out of SAFE.
    past = [k for k in range(release) if below_under_temp(result, k)]
    assert len(past) > ticks_of(60)
    assert not any(result.thermal(k).readings.under_temp for k in range(release))
    assert result.modes[:release] == [Mode.BOOT] * release
    # On release the live readings show it, and SAFE follows N ticks later.
    flagged = result.first(lambda k: result.thermal(k).readings.under_temp)
    assert flagged == release  # the first tick stepped without the freeze
    assert result.modes.index(Mode.SAFE) == flagged + N - 1


def test_frozen_thermal_sensor_prevents_safe_entry() -> None:
    result = run(
        ticks_of(600),
        environment=lambda now_us: COLD_ENV,
        overrides=thermal_frozen_until(None),
    )
    assert all(below_under_temp(result, k) for k in range(ticks_of(300), len(result.modes)))
    assert set(result.modes) == {Mode.BOOT}


# --- Battery drain: SAFE from physics, out of SAFE by command once clear -----------------


def test_battery_drain_enters_safe_and_a_command_recovers_once_the_flag_clears() -> None:
    # Start just above critical, add a battery_drain-style 15 W for 2 minutes. Once the
    # drain stops, the arrays recharge the battery past the clear threshold. SET_MODE
    # NOMINAL is sent every 30 s: NACKed while critical_battery is set, accepted after.
    drain = Overrides(extra_load_w=15.0)
    initial = dataclasses.replace(DEFAULT_INITIAL_STATE, power=PowerInitial(soc=0.17))
    script: dict[int, list[ModeEvent]] = {0: [BOOT_COMPLETE], **every(30, SET_NOMINAL, 30, 1200)}
    result = run(
        ticks_of(1200),
        initial=initial,
        overrides=lambda now_us: drain if now_us < 120 * US_PER_S else NO_OVERRIDES,
        script=script,
        stop=lambda r: r.modes[-1] is Mode.NOMINAL and Mode.SAFE in r.modes,
    )
    flagged = result.first(lambda k: result.power(k).readings.critical_battery)
    entered = result.first(lambda k: result.modes[k] is Mode.SAFE)
    assert entered == flagged + N - 1
    assert result.modes[:entered] == [Mode.NOMINAL] * entered

    nacked = 0
    for k, transitions in sorted(result.command_results.items()):
        if k <= entered:
            continue
        (outcome,) = transitions
        critical = result.power(k).readings.critical_battery
        if critical:
            assert outcome.reason is RejectReason.SAFE_CONDITIONS_ACTIVE
            nacked += 1
        else:
            assert outcome.outcome is Outcome.TRANSITION
            assert result.modes[k] is Mode.NOMINAL
    assert nacked >= 1, "at least one SET_MODE NOMINAL was refused while critical"
    assert result.modes[-1] is Mode.NOMINAL
    assert result.controls[-1] == controls_for_mode(Mode.SAFE)  # NOMINAL's apply next tick


# --- A whole orbit, with a cold spell and recovery ---------------------------------------


@pytest.mark.slow
def test_orbit_with_a_cold_spell_enters_safe_and_recovers_to_nominal() -> None:
    # One orbit in SCIENCE with a 3-minute cold spell at the start. under_temp sets,
    # SAFE follows N ticks later, the spacecraft warms once the spell ends, and SET_MODE
    # NOMINAL (sent every 30 s) is refused until under_temp clears, then accepted. The
    # ground then returns to SCIENCE. No FAULT anywhere: the consistency checks hold on
    # the real subsystems' readings in every tick.
    orbit_ticks = NOMINAL_ENV.orbit_period_us // SimClock().tick_us
    script: dict[int, list[ModeEvent]] = {
        0: [BOOT_COMPLETE],
        1: [SET_SCIENCE],
        **every(30, SET_NOMINAL, 180, 30 * 60),
        ticks_of(30 * 60): [SET_SCIENCE],
    }
    result = run(orbit_ticks, environment=cold_until(180), script=script)

    assert Mode.FAULT not in result.modes
    flagged = result.first(lambda k: result.thermal(k).readings.under_temp)
    entered = result.first(lambda k: result.modes[k] is Mode.SAFE)
    assert entered == flagged + N - 1
    assert result.modes[2:entered] == [Mode.SCIENCE] * (entered - 2)

    cleared = result.first(lambda k: k > entered and not result.thermal(k).readings.under_temp)
    left = result.first(lambda k: k > entered and result.modes[k] is Mode.NOMINAL)
    assert cleared <= left < cleared + ticks_of(30) + 1
    assert all(m is Mode.SAFE for m in result.modes[entered:left])
    for k, (transition,) in result.command_results.items():
        if entered < k < left:
            assert transition.reason is RejectReason.SAFE_CONDITIONS_ACTIVE

    back = ticks_of(30 * 60)
    assert result.modes[back:] == [Mode.SCIENCE] * (orbit_ticks - back)
    assert not any(
        result.thermal(k).readings.under_temp or result.power(k).readings.critical_battery
        for k in range(back, orbit_ticks)
    )
