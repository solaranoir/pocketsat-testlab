"""Tests for the automatic safe-mode and fault entry rules (#48).

Two levels:

- the pure rules in :mod:`pocketsat.flight.safety` (:func:`evaluate`), table-driven
  over every flag, combinations, persistence, clearing, and every consistency check;
- the :class:`FlightComputer` running them in its evaluate-flags phase, fed
  :class:`SpacecraftReadings` from fake snapshots, with the resulting mode and controls.

Commands (#51) don't exist yet, so the flight-computer tests script those mode events
into the execute-commands phase (:class:`ScriptedComputer`). They also script
BOOT_COMPLETE to reach a mode in one tick; #49's own BOOT_COMPLETE comes only after the
boot duration (5 s, 50 ticks), longer than these tests run unless they say otherwise.
"""

import dataclasses
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import pytest

from pocketsat.flight import (
    EventKind,
    FlightComputer,
    FlightComputerConfig,
    Mode,
    ModeEvent,
    Outcome,
    RejectReason,
    SpacecraftReadings,
    Transition,
    controls_for_mode,
)
from pocketsat.flight.computer import TickContext
from pocketsat.flight.safety import (
    DEFAULT_SAFETY_CONFIG,
    INITIAL_SAFETY_STATE,
    SAFE_FLAGS,
    ConsistencyCheck,
    SafeFlag,
    SafetyConfig,
    SafetyState,
    SafetyVerdict,
    active_flags,
    consistency_failures,
    evaluate,
)
from pocketsat.spacecraft import PowerSnapshot, SpacecraftControls
from pocketsat.spacecraft.fakes import (
    FakeSubsystem,
    default_snapshot,
    fake_stack,
    fake_subsystems,
    replace_readings,
)
from pocketsat.targets.base import EnvironmentState

TICK_US = 100_000

N = DEFAULT_SAFETY_CONFIG.sustain_tick_count

_FAKE_READINGS = SpacecraftReadings.from_state(fake_stack().snapshot())
NOMINAL_READINGS = dataclasses.replace(
    _FAKE_READINGS,
    payload=dataclasses.replace(
        _FAKE_READINGS.payload,
        buffered_bytes=64_000,
        next_chunk_id=1_000,
        total_produced_bytes=64_000,
    ),
)
"""Consistent readings with every flag clear (the fakes' defaults), and a payload backlog
of 1000 chunks, so a DOWNLINK session (#56) has data to send and DOWNLINK lasts."""

ALL_FLAG_SETS: list[frozenset[SafeFlag]] = [
    frozenset(flag for i, flag in enumerate(SAFE_FLAGS) if bits >> i & 1)
    for bits in range(1 << len(SAFE_FLAGS))
]
"""Every combination of safe-mode flags, the empty set included."""


def flag_id(flags: frozenset[SafeFlag]) -> str:
    return "+".join(sorted(f.value for f in flags)) or "none"


def readings_with(
    flags: frozenset[SafeFlag] = frozenset(),
    *,
    low_battery: bool | None = None,
    base: SpacecraftReadings = NOMINAL_READINGS,
) -> SpacecraftReadings:
    """``base`` with exactly ``flags`` set. ``low_battery`` follows ``critical_battery``
    (critical is only ever set together with low) unless given."""
    critical = SafeFlag.CRITICAL_BATTERY in flags
    power = dataclasses.replace(
        base.power,
        critical_battery=critical,
        low_battery=critical if low_battery is None else low_battery,
    )
    thermal = dataclasses.replace(
        base.thermal,
        over_temp=SafeFlag.OVER_TEMP in flags,
        under_temp=SafeFlag.UNDER_TEMP in flags,
    )
    return dataclasses.replace(base, power=power, thermal=thermal)


def run_rules(
    sequence: Sequence[frozenset[SafeFlag]], config: SafetyConfig = DEFAULT_SAFETY_CONFIG
) -> list[SafetyVerdict]:
    """Evaluate the rules over a sequence of flag sets, one tick each."""
    state = INITIAL_SAFETY_STATE
    verdicts = []
    for flags in sequence:
        verdict = evaluate(state, readings_with(flags), config)
        verdicts.append(verdict)
        state = verdict.state
    return verdicts


def pattern(text: str, flag: SafeFlag = SafeFlag.CRITICAL_BATTERY) -> list[frozenset[SafeFlag]]:
    """A one-flag sequence from a string: ``#`` is a tick with ``flag`` set, ``.`` clear."""
    return [frozenset({flag}) if ch == "#" else frozenset() for ch in text]


# --- Configuration and state ------------------------------------------------------------


def test_default_persistence_is_ten_ticks() -> None:
    # 1 s at the default 100 ms tick; documented in docs/spacecraft-modes.md.
    assert SafetyConfig().sustain_tick_count == 10 == N


@pytest.mark.parametrize("bad", [0, -1])
def test_config_rejects_counts_below_one(bad: int) -> None:
    with pytest.raises(ValueError, match="at least 1"):
        SafetyConfig(sustain_tick_count=bad)


@pytest.mark.parametrize("bad", [1.0, True, "10", None])
def test_config_rejects_non_int_counts(bad: Any) -> None:
    with pytest.raises(TypeError):
        SafetyConfig(sustain_tick_count=bad)


@pytest.mark.parametrize("counts", [(0, 0), (0, 0, 0, 0), (0, -1, 0), (0, 1.0, 0), (True, 0, 0)])
def test_state_rejects_bad_counts(counts: tuple[Any, ...]) -> None:
    with pytest.raises(ValueError):
        SafetyState(counts)


def test_initial_state_has_every_counter_at_zero() -> None:
    assert all(INITIAL_SAFETY_STATE.count(flag) == 0 for flag in SafeFlag)


def test_safe_flags_are_the_issue_48_flags_and_name_readings_fields() -> None:
    assert [f.value for f in SAFE_FLAGS] == ["critical_battery", "over_temp", "under_temp"]
    assert hasattr(NOMINAL_READINGS.power, "critical_battery")
    assert hasattr(NOMINAL_READINGS.thermal, "over_temp")
    assert hasattr(NOMINAL_READINGS.thermal, "under_temp")


# --- Safe-mode flags: each alone and in combination -----------------------------------


@pytest.mark.parametrize("flags", ALL_FLAG_SETS, ids=flag_id)
def test_active_flags_are_read_from_the_readings(flags: frozenset[SafeFlag]) -> None:
    assert active_flags(readings_with(flags)) == flags


@pytest.mark.parametrize("flags", ALL_FLAG_SETS, ids=flag_id)
@pytest.mark.parametrize("sustain", [1, 2, 10])
def test_flags_held_raise_safe_condition_on_the_nth_tick(
    flags: frozenset[SafeFlag], sustain: int
) -> None:
    config = SafetyConfig(sustain_tick_count=sustain)
    verdicts = run_rules([flags] * (sustain + 3), config)
    for tick, verdict in enumerate(verdicts):
        expected = bool(flags) and tick >= sustain - 1
        assert verdict.safe_condition is expected, f"tick {tick}"
        assert verdict.sustained == (flags if expected else frozenset())
        assert verdict.active == flags
        assert verdict.safe_exit_allowed is (not flags)
        assert verdict.fault_detected is False
        assert verdict.events == ((ModeEvent(EventKind.SAFE_CONDITION),) if expected else ())


def test_low_battery_alone_is_not_a_safe_condition() -> None:
    verdicts = [
        evaluate(INITIAL_SAFETY_STATE, readings_with(low_battery=True)) for _ in range(N + 5)
    ]
    assert not any(v.safe_condition for v in verdicts)
    assert all(v.safe_exit_allowed for v in verdicts)


# --- Persistence and clearing ------------------------------------------------------------

PERSISTENCE_CASES: list[tuple[str, str, int, list[int]]] = [
    # (id, pattern, sustain, ticks on which SAFE_CONDITION is raised)
    ("never set", "." * 20, 10, []),
    ("one tick short", "#" * 9, 10, []),
    ("exactly N", "#" * 10, 10, [9]),
    ("held past N: raised every tick", "#" * 13, 10, [9, 10, 11, 12]),
    ("one clear tick restarts the count", "#" * 9 + "." + "#" * 9, 10, []),
    ("restart then sustained", "#" * 9 + "." + "#" * 10, 10, [19]),
    ("clears the tick after", "#" * 10 + "." * 5, 10, [9]),
    ("re-raised only after N fresh ticks", "#" * 10 + "." + "#" * 10, 10, [9, 20]),
    ("N = 1: immediate", ".#.##.", 1, [1, 3, 4]),
    ("N = 3", "##.###.##", 3, [5]),
]


@pytest.mark.parametrize(
    ("text", "sustain", "raised"),
    [case[1:] for case in PERSISTENCE_CASES],
    ids=[case[0] for case in PERSISTENCE_CASES],
)
@pytest.mark.parametrize("flag", SAFE_FLAGS, ids=lambda f: f.value)
def test_persistence(text: str, sustain: int, raised: list[int], flag: SafeFlag) -> None:
    verdicts = run_rules(pattern(text, flag), SafetyConfig(sustain_tick_count=sustain))
    assert [t for t, v in enumerate(verdicts) if v.safe_condition] == raised


@pytest.mark.parametrize("flag", SAFE_FLAGS, ids=lambda f: f.value)
def test_counter_counts_consecutive_ticks_and_saturates(flag: SafeFlag) -> None:
    verdicts = run_rules(pattern("#" * 25 + "." + "##", flag))
    counts = [v.state.count(flag) for v in verdicts]
    assert counts == [*range(1, N + 1), *[N] * (25 - N), 0, 1, 2]
    others = [f for f in SAFE_FLAGS if f is not flag]
    assert all(v.state.count(f) == 0 for v in verdicts for f in others)


def test_each_flag_has_its_own_counter() -> None:
    # A hand-off between different flags does not add up: each must be sustained alone.
    half = N // 2
    crit, under = SafeFlag.CRITICAL_BATTERY, SafeFlag.UNDER_TEMP
    sequence = [frozenset({crit})] * half + [frozenset({under})] * (N - half)
    assert not any(v.safe_condition for v in run_rules(sequence))


def test_overlapping_flags_trigger_on_the_first_one_sustained() -> None:
    crit, over = SafeFlag.CRITICAL_BATTERY, SafeFlag.OVER_TEMP
    sequence = [frozenset({crit})] * 4 + [frozenset({crit, over})] * N
    verdicts = run_rules(sequence)
    first = next(t for t, v in enumerate(verdicts) if v.safe_condition)
    assert first == N - 1
    assert verdicts[first].sustained == {crit}
    assert verdicts[4 + N - 1].sustained == {crit, over}


@pytest.mark.parametrize("flag", SAFE_FLAGS, ids=lambda f: f.value)
@pytest.mark.parametrize("on_ticks", [1, 3, N - 1])
def test_no_chatter_from_a_flag_flickering_at_its_threshold(flag: SafeFlag, on_ticks: int) -> None:
    # A flag that flickers on and off near its threshold (never N ticks in a row) never
    # raises SAFE_CONDITION, however long it goes on.
    text = ("#" * on_ticks + ".") * 200
    assert not any(v.safe_condition for v in run_rules(pattern(text, flag)))


def test_safe_exit_allowed_only_when_every_flag_is_clear() -> None:
    for flags in ALL_FLAG_SETS:
        verdict = evaluate(INITIAL_SAFETY_STATE, readings_with(flags))
        assert verdict.safe_exit_allowed is (not flags), flag_id(flags)


def test_safe_exit_is_blocked_by_a_flag_before_it_is_sustained() -> None:
    # The guard reads the flags as they are now, not the sustained ones.
    verdict = run_rules(pattern("#"))[0]
    assert not verdict.safe_condition
    assert not verdict.safe_exit_allowed


def test_rules_are_pure_and_deterministic() -> None:
    sequence = pattern("##.#####.##########...###", SafeFlag.OVER_TEMP)
    assert run_rules(sequence) == run_rules(sequence)
    state = SafetyState((3, 0, 7))
    readings = readings_with(frozenset({SafeFlag.CRITICAL_BATTERY}))
    assert evaluate(state, readings) == evaluate(state, readings)
    assert state == SafetyState((3, 0, 7))


# --- Consistency checks (FAULT_DETECTED) --------------------------------------------------


def with_power(**changes: Any) -> SpacecraftReadings:
    return dataclasses.replace(
        NOMINAL_READINGS, power=dataclasses.replace(NOMINAL_READINGS.power, **changes)
    )


def with_thermal(**changes: Any) -> SpacecraftReadings:
    return dataclasses.replace(
        NOMINAL_READINGS, thermal=dataclasses.replace(NOMINAL_READINGS.thermal, **changes)
    )


def with_attitude(**changes: Any) -> SpacecraftReadings:
    return dataclasses.replace(
        NOMINAL_READINGS, attitude=dataclasses.replace(NOMINAL_READINGS.attitude, **changes)
    )


def with_payload(**changes: Any) -> SpacecraftReadings:
    return dataclasses.replace(
        NOMINAL_READINGS, payload=dataclasses.replace(NOMINAL_READINGS.payload, **changes)
    )


def with_comms(**changes: Any) -> SpacecraftReadings:
    return dataclasses.replace(
        NOMINAL_READINGS, comms=dataclasses.replace(NOMINAL_READINGS.comms, **changes)
    )


CAPACITY = NOMINAL_READINGS.comms.transmit_capacity_bytes
"""The fake comms readings' transmit capacity (the nominal 120 bytes)."""

AFTER_A_FULL_CAPACITY_TICK = SafetyState(previous_transmit_capacity_bytes=CAPACITY)
"""The rules' state after a tick with the fake's full transmit capacity."""

NON_FINITE = ConsistencyCheck.NON_FINITE_READING
PAYLOAD = ConsistencyCheck.PAYLOAD_BOOKKEEPING
COMMS = ConsistencyCheck.COMMS_CAPACITY

FAILURE_CASES: list[tuple[str, SpacecraftReadings, tuple[ConsistencyCheck, ...]]] = [
    *(
        (f"power.{name}={value}", with_power(**{name: value}), (NON_FINITE,))
        for name in ("bus_v", "battery_current_a", "soc")
        for value in (math.nan, math.inf, -math.inf)
    ),
    *(
        (f"thermal.{name}={value}", with_thermal(**{name: value}), (NON_FINITE,))
        for name in ("battery_c", "electronics_c")
        for value in (math.nan, -math.inf)
    ),
    *(
        (f"attitude.{name}=nan", with_attitude(**{name: math.nan}), (NON_FINITE,))
        for name in ("pointing_error_deg", "rate_dps")
    ),
    (
        "critical without low",
        with_power(critical_battery=True, low_battery=False),
        (ConsistencyCheck.BATTERY_FLAG_ORDER,),
    ),
    ("buffer not produced - released", with_payload(buffered_bytes=64), (PAYLOAD,)),
    (
        "buffer over capacity",
        with_payload(buffered_bytes=70_000, total_produced_bytes=70_000),
        (PAYLOAD,),
    ),
    (
        "buffer negative",
        with_payload(buffered_bytes=-64, total_released_bytes=64),
        (PAYLOAD,),
    ),
    ("oldest chunk past next", with_payload(oldest_unreleased_chunk_id=1_001), (PAYLOAD,)),
    ("sent over capacity", with_comms(previous_tick_sent_bytes=121), (COMMS,)),
    ("sent negative", with_comms(previous_tick_sent_bytes=-1), (COMMS,)),
    ("capacity with transmitter off", with_comms(transmitter_on=False), (COMMS,)),
    ("lost count negative", with_comms(uplink_lost_count=-1), (COMMS,)),
    ("suppressed count negative", with_comms(outbound_suppressed_count=-1), (COMMS,)),
    (
        "several at once, in check order",
        dataclasses.replace(
            with_comms(previous_tick_sent_bytes=500),
            power=dataclasses.replace(NOMINAL_READINGS.power, soc=math.nan, critical_battery=True),
        ),
        (NON_FINITE, ConsistencyCheck.BATTERY_FLAG_ORDER, COMMS),
    ),
]


@pytest.mark.parametrize(
    ("readings", "expected"),
    [case[1:] for case in FAILURE_CASES],
    ids=[case[0] for case in FAILURE_CASES],
)
def test_consistency_failures(
    readings: SpacecraftReadings, expected: tuple[ConsistencyCheck, ...]
) -> None:
    # The previous tick had the fake's full capacity, so the bytes are bounded by it.
    assert consistency_failures(readings, CAPACITY) == expected
    verdict = evaluate(AFTER_A_FULL_CAPACITY_TICK, readings)
    assert verdict.fault_detected
    assert verdict.events[0] == ModeEvent(EventKind.FAULT_DETECTED)


PASSING_CASES: list[tuple[str, SpacecraftReadings]] = [
    ("fake defaults", NOMINAL_READINGS),
    ("every safe flag set", readings_with(frozenset(SAFE_FLAGS))),
    ("low without critical", readings_with(low_battery=True)),
    (
        "buffer full and partly released",
        with_payload(
            buffered_bytes=65_536,
            total_produced_bytes=70_000,
            total_released_bytes=4_464,
            oldest_unreleased_chunk_id=10,
            next_chunk_id=1_034,
        ),
    ),
    ("no chunk stored", with_payload(oldest_unreleased_chunk_id=5, next_chunk_id=5)),
    ("full capacity sent", with_comms(previous_tick_sent_bytes=120)),
    (
        "transmitter off (transmitter_off fault or RX_ONLY)",
        with_comms(transmitter_on=False, transmit_capacity_bytes=0),
    ),
    ("extreme but finite values", with_thermal(battery_c=-273.0, electronics_c=1e9)),
]


@pytest.mark.parametrize(
    "readings", [case[1] for case in PASSING_CASES], ids=[case[0] for case in PASSING_CASES]
)
def test_consistent_readings_pass_every_check(readings: SpacecraftReadings) -> None:
    assert consistency_failures(readings, CAPACITY) == ()
    assert consistency_failures(readings) == ()
    assert not evaluate(AFTER_A_FULL_CAPACITY_TICK, readings).fault_detected
    assert not evaluate(INITIAL_SAFETY_STATE, readings).fault_detected


def test_fault_is_raised_in_the_first_tick_without_persistence() -> None:
    bad = with_comms(previous_tick_sent_bytes=999)
    verdict = evaluate(AFTER_A_FULL_CAPACITY_TICK, bad, SafetyConfig(sustain_tick_count=50))
    assert verdict.events == (ModeEvent(EventKind.FAULT_DETECTED),)


def test_fault_comes_before_safe_condition_in_the_same_tick() -> None:
    bad = readings_with(
        frozenset({SafeFlag.UNDER_TEMP}), base=with_comms(previous_tick_sent_bytes=999)
    )
    verdict = evaluate(AFTER_A_FULL_CAPACITY_TICK, bad, SafetyConfig(sustain_tick_count=1))
    assert verdict.events == (
        ModeEvent(EventKind.FAULT_DETECTED),
        ModeEvent(EventKind.SAFE_CONDITION),
    )


# --- Comms bytes against the previous tick's capacity (#98, from PR #110) -----------------

TRANSMITTER_JUST_OFF = with_comms(
    transmitter_on=False, transmit_capacity_bytes=0, previous_tick_sent_bytes=CAPACITY
)
"""The first tick of ``transmitter_off``: capacity 0 now, but the previous tick's full
capacity is still being reported (ADR-0007)."""


def test_bytes_sent_before_the_transmitter_switched_off_are_consistent() -> None:
    # Checked against the previous tick's capacity, not this tick's 0: no false FAULT.
    assert consistency_failures(TRANSMITTER_JUST_OFF, CAPACITY) == ()
    assert not evaluate(AFTER_A_FULL_CAPACITY_TICK, TRANSMITTER_JUST_OFF).fault_detected


def test_bytes_while_the_transmitter_was_already_off_are_a_fault() -> None:
    # Two ticks into transmitter_off the previous capacity is 0 too.
    after_off = evaluate(AFTER_A_FULL_CAPACITY_TICK, TRANSMITTER_JUST_OFF).state
    assert after_off.previous_transmit_capacity_bytes == 0
    still_sending = with_comms(
        transmitter_on=False, transmit_capacity_bytes=0, previous_tick_sent_bytes=1
    )
    assert evaluate(after_off, still_sending).failures == (COMMS,)


def test_evaluate_remembers_this_ticks_capacity_for_the_next_tick() -> None:
    assert evaluate(INITIAL_SAFETY_STATE, NOMINAL_READINGS).state == SafetyState(
        (0, 0, 0), CAPACITY
    )
    off = with_comms(transmitter_on=False, transmit_capacity_bytes=0)
    assert evaluate(AFTER_A_FULL_CAPACITY_TICK, off).state.previous_transmit_capacity_bytes == 0


def test_unknown_previous_capacity_checks_only_the_lower_bound() -> None:
    # The first tick after power-on or a reboot: the bytes were sent by the software
    # before the reboot, under a capacity this one never saw.
    assert INITIAL_SAFETY_STATE.previous_transmit_capacity_bytes is None
    assert consistency_failures(TRANSMITTER_JUST_OFF) == ()
    assert consistency_failures(with_comms(previous_tick_sent_bytes=999)) == ()
    assert consistency_failures(with_comms(previous_tick_sent_bytes=-1)) == (COMMS,)


@pytest.mark.parametrize("bad", [-1, 1.0, True, "120"])
def test_state_rejects_a_bad_previous_capacity(bad: Any) -> None:
    with pytest.raises(ValueError):
        SafetyState((0, 0, 0), bad)


def test_transmitter_switching_off_with_bytes_in_flight_raises_no_fault() -> None:
    # The flight computer itself, across the two ticks: full capacity sent in tick N,
    # transmitter_off from tick N+1 (comms reports the bytes with capacity 0).
    fc = FlightComputer()
    fc.reset()
    fc.step((), NOMINAL_READINGS, TICK_US)
    fc.step((), with_comms(previous_tick_sent_bytes=CAPACITY), 2 * TICK_US)
    modes = []
    for n, readings in enumerate((TRANSMITTER_JUST_OFF, TRANSMITTER_JUST_OFF), start=3):
        fc.step((), readings, n * TICK_US)
        modes.append(fc.mode)
    # No FAULT in the first tick of transmitter_off; 120 bytes reported again in the
    # next tick, when the previous capacity was 0 too, is one.
    assert modes == [Mode.BOOT, Mode.FAULT]


# --- The flight computer -----------------------------------------------------------------


class ScriptedComputer(FlightComputer):
    """A flight computer with scripted command-phase events.

    Stands in for #51 (commands) and #49 (BOOT_COMPLETE): ``script[k]`` lists the mode
    events raised in the execute-commands phase of the k-th :meth:`step` call (0-based),
    before the evaluate-flags phase runs. Every step's tick context is kept in
    :attr:`ticks`.
    """

    def __init__(
        self,
        script: Mapping[int, Sequence[ModeEvent]] | None = None,
        *,
        safety: SafetyConfig = DEFAULT_SAFETY_CONFIG,
    ) -> None:
        self.script = dict(script or {})
        self.ticks: list[TickContext] = []
        super().__init__(FlightComputerConfig(safety=safety))

    def _execute_commands(self, tick: TickContext) -> None:
        tick.mode_events.extend(self.script.get(len(self.ticks), ()))
        self.ticks.append(tick)


BOOT_COMPLETE = ModeEvent(EventKind.BOOT_COMPLETE)
SET_NOMINAL = ModeEvent(EventKind.SET_MODE, Mode.NOMINAL)
SET_SCIENCE = ModeEvent(EventKind.SET_MODE, Mode.SCIENCE)
BEGIN_DOWNLINK = ModeEvent(EventKind.BEGIN_DOWNLINK)
ENTER_SAFE = ModeEvent(EventKind.ENTER_SAFE_MODE)
RESET = ModeEvent(EventKind.RESET)

TO_MODE: dict[Mode, list[ModeEvent]] = {
    Mode.BOOT: [],
    Mode.NOMINAL: [BOOT_COMPLETE],
    Mode.SCIENCE: [BOOT_COMPLETE, SET_SCIENCE],
    Mode.DOWNLINK: [BOOT_COMPLETE, BEGIN_DOWNLINK],
    Mode.SAFE: [BOOT_COMPLETE, ENTER_SAFE],
}
"""Events, one tick each, that take a new flight computer to each mode (FAULT aside)."""


class Driver:
    """Steps a :class:`ScriptedComputer` one tick at a time with given readings."""

    def __init__(self, fc: ScriptedComputer) -> None:
        self.fc = fc
        self.now_us = 0
        self.controls = fc.reset()
        self.modes: list[Mode] = []

    def step(self, readings: SpacecraftReadings = NOMINAL_READINGS, ticks: int = 1) -> Mode:
        for _ in range(ticks):
            self.now_us += TICK_US
            self.controls = self.fc.step((), readings, self.now_us).controls
            self.modes.append(self.fc.mode)
        return self.fc.mode

    @property
    def last_transitions(self) -> list[Transition]:
        return self.fc.ticks[-1].transitions


def driver_in(mode: Mode, safety: SafetyConfig = DEFAULT_SAFETY_CONFIG) -> Driver:
    """A driver whose flight computer has reached ``mode`` through nominal ticks."""
    events = TO_MODE[mode]
    driver = Driver(ScriptedComputer({k: [e] for k, e in enumerate(events)}, safety=safety))
    driver.step(ticks=len(events))
    assert driver.fc.mode is mode
    return driver


def test_flight_computer_config_rejects_a_bad_safety_config() -> None:
    with pytest.raises(TypeError, match="SafetyConfig"):
        FlightComputerConfig(safety=10)  # type: ignore[arg-type]


def test_nominal_readings_raise_nothing() -> None:
    driver = Driver(ScriptedComputer())
    driver.step(ticks=100)
    assert all(t.safety is not None and t.safety.events == () for t in driver.fc.ticks)
    # The only event is #49's BOOT_COMPLETE, once, when the boot duration has elapsed.
    assert [e for t in driver.fc.ticks for e in t.mode_events] == [BOOT_COMPLETE]
    assert all(t.safe_exit_allowed for t in driver.fc.ticks)


@pytest.mark.parametrize("mode", list(TO_MODE), ids=lambda m: m.name)
@pytest.mark.parametrize("flag", SAFE_FLAGS, ids=lambda f: f.value)
def test_a_sustained_flag_enters_safe_from_every_mode(mode: Mode, flag: SafeFlag) -> None:
    driver = driver_in(mode)
    flagged = readings_with(frozenset({flag}))
    assert driver.step(flagged, ticks=N - 1) is mode
    assert driver.step(flagged) is Mode.SAFE
    assert driver.controls == controls_for_mode(Mode.SAFE)
    # Level-triggered: still raised (and ignored) while it stays sustained.
    driver.step(flagged)
    assert [t.outcome for t in driver.last_transitions] == [Outcome.IGNORED]


@pytest.mark.parametrize("flag", SAFE_FLAGS, ids=lambda f: f.value)
def test_boot_goes_straight_to_safe(flag: SafeFlag) -> None:
    # #107 decision 7: SAFE_CONDITION is honoured in BOOT, without BOOT_COMPLETE.
    driver = Driver(ScriptedComputer())
    flagged = readings_with(frozenset({flag}))
    driver.step(flagged, ticks=N)
    assert driver.modes == [Mode.BOOT] * (N - 1) + [Mode.SAFE]
    assert driver.controls == controls_for_mode(Mode.SAFE)
    assert driver.controls.attitude.enabled


def test_a_flag_flickering_never_enters_safe() -> None:
    driver = driver_in(Mode.SCIENCE)
    flagged = readings_with(frozenset({SafeFlag.UNDER_TEMP}))
    start = len(driver.modes)
    for _ in range(50):
        driver.step(flagged, ticks=N - 1)
        driver.step()
    assert set(driver.modes[start:]) == {Mode.SCIENCE}


@pytest.mark.parametrize("sustain", [1, 3, 25])
def test_persistence_is_configurable(sustain: int) -> None:
    driver = driver_in(Mode.NOMINAL, SafetyConfig(sustain_tick_count=sustain))
    flagged = readings_with(frozenset({SafeFlag.OVER_TEMP}))
    if sustain > 1:
        assert driver.step(flagged, ticks=sustain - 1) is Mode.NOMINAL
    assert driver.step(flagged) is Mode.SAFE


def test_safe_condition_is_applied_after_the_ticks_commands() -> None:
    # Commands (step c) come first; the automatic rules have the last word in a tick.
    driver = driver_in(Mode.NOMINAL)
    flagged = readings_with(frozenset({SafeFlag.CRITICAL_BATTERY}))
    driver.step(flagged, ticks=N - 1)
    driver.fc.script[len(driver.fc.ticks)] = [SET_SCIENCE]
    assert driver.step(flagged) is Mode.SAFE
    assert [t.mode for t in driver.last_transitions] == [Mode.SCIENCE, Mode.SAFE]


# --- Leaving SAFE: the guard ---------------------------------------------------------------


def command_now(driver: Driver, event: ModeEvent, readings: SpacecraftReadings) -> Transition:
    driver.fc.script[len(driver.fc.ticks)] = [event]
    driver.step(readings)
    return driver.last_transitions[0]


@pytest.mark.parametrize("flag", SAFE_FLAGS, ids=lambda f: f.value)
def test_safe_is_left_by_command_only_once_the_flags_clear(flag: SafeFlag) -> None:
    driver = driver_in(Mode.NOMINAL)
    flagged = readings_with(frozenset({flag}))
    driver.step(flagged, ticks=N)
    assert driver.fc.mode is Mode.SAFE

    # Flags still set: NACK, and the spacecraft stays in SAFE.
    result = command_now(driver, SET_NOMINAL, flagged)
    assert result.outcome is Outcome.REJECTED
    assert result.reason is RejectReason.SAFE_CONDITIONS_ACTIVE
    assert driver.fc.mode is Mode.SAFE

    # Flags clear, but no command: SAFE is never left automatically.
    assert driver.step(ticks=5 * N) is Mode.SAFE

    # Flags clear and the command: back to NOMINAL, with NOMINAL's controls.
    result = command_now(driver, SET_NOMINAL, NOMINAL_READINGS)
    assert result.outcome is Outcome.TRANSITION
    assert driver.modes[-1] is Mode.NOMINAL
    assert driver.controls == controls_for_mode(Mode.NOMINAL)


@pytest.mark.parametrize("flags", [f for f in ALL_FLAG_SETS if f], ids=flag_id)
def test_any_flag_set_blocks_the_exit(flags: frozenset[SafeFlag]) -> None:
    # Whichever flag triggered SAFE, every safe-mode flag must be clear to leave.
    driver = driver_in(Mode.SAFE)
    result = command_now(driver, SET_NOMINAL, readings_with(flags))
    assert result.reason is RejectReason.SAFE_CONDITIONS_ACTIVE
    assert driver.fc.mode is Mode.SAFE


def test_a_flag_set_for_one_tick_blocks_the_exit() -> None:
    driver = driver_in(Mode.SAFE)
    result = command_now(driver, SET_NOMINAL, readings_with(frozenset({SafeFlag.OVER_TEMP})))
    assert result.reason is RejectReason.SAFE_CONDITIONS_ACTIVE


def test_safe_entered_by_command_is_left_once_flags_are_clear() -> None:
    driver = driver_in(Mode.SAFE)  # ENTER_SAFE_MODE with every flag clear
    assert command_now(driver, SET_NOMINAL, NOMINAL_READINGS).outcome is Outcome.TRANSITION
    assert driver.fc.mode is Mode.NOMINAL


def test_low_battery_does_not_block_the_exit() -> None:
    driver = driver_in(Mode.SAFE)
    result = command_now(driver, SET_NOMINAL, readings_with(low_battery=True))
    assert result.outcome is Outcome.TRANSITION


def test_no_chatter_after_leaving_safe() -> None:
    # After the exit every counter is 0, so a flag that comes back must be sustained
    # for N fresh ticks before SAFE is re-entered.
    driver = driver_in(Mode.NOMINAL)
    flagged = readings_with(frozenset({SafeFlag.UNDER_TEMP}))
    driver.step(flagged, ticks=N)
    command_now(driver, SET_NOMINAL, NOMINAL_READINGS)
    assert driver.fc.mode is Mode.NOMINAL
    assert driver.step(flagged, ticks=N - 1) is Mode.NOMINAL
    driver.step()
    assert driver.step(flagged, ticks=N - 1) is Mode.NOMINAL
    assert driver.step(flagged) is Mode.SAFE


def test_tick_context_carries_the_verdict_and_guard() -> None:
    driver = Driver(ScriptedComputer())
    flagged = readings_with(frozenset({SafeFlag.CRITICAL_BATTERY}))
    driver.step(flagged)
    tick = driver.fc.ticks[-1]
    assert tick.safety is not None
    assert tick.safety.active == {SafeFlag.CRITICAL_BATTERY}
    assert tick.safe_exit_allowed is False
    driver.step()
    assert driver.fc.ticks[-1].safe_exit_allowed is True


# --- FAULT ---------------------------------------------------------------------------------

INCONSISTENT = with_payload(buffered_bytes=64)
"""Readings that fail the payload bookkeeping check."""


@pytest.mark.parametrize("mode", list(TO_MODE), ids=lambda m: m.name)
def test_an_inconsistency_enters_fault_in_one_tick_from_every_mode(mode: Mode) -> None:
    driver = driver_in(mode)
    assert driver.step(INCONSISTENT) is Mode.FAULT
    assert driver.controls == controls_for_mode(Mode.FAULT)


@pytest.mark.parametrize(
    "event",
    [SET_NOMINAL, SET_SCIENCE, BEGIN_DOWNLINK, ENTER_SAFE, BOOT_COMPLETE],
    ids=lambda e: e.kind.name if e.target is None else f"SET_MODE_{e.target.name}",
)
def test_only_reset_leaves_fault(event: ModeEvent) -> None:
    driver = driver_in(Mode.NOMINAL)
    driver.step(INCONSISTENT)
    # Consistent again and every flag clear: FAULT holds for any number of ticks...
    assert driver.step(ticks=5 * N) is Mode.FAULT
    # ...against every other event, including while a safe flag is sustained...
    command_now(driver, event, NOMINAL_READINGS)
    assert driver.fc.mode is Mode.FAULT
    assert driver.step(readings_with(frozenset({SafeFlag.UNDER_TEMP})), ticks=2 * N) is Mode.FAULT
    # ...and RESET leaves it, to BOOT.
    assert command_now(driver, RESET, NOMINAL_READINGS).mode is Mode.BOOT
    assert driver.modes[-1] is Mode.BOOT


def test_reboot_leaves_fault() -> None:
    driver = driver_in(Mode.NOMINAL)
    driver.step(INCONSISTENT)
    driver.fc.reboot(now_us=driver.now_us)
    assert driver.fc.mode is Mode.BOOT


def test_inconsistency_that_persists_after_reset_re_enters_fault() -> None:
    driver = driver_in(Mode.NOMINAL)
    driver.step(INCONSISTENT)
    command_now(driver, RESET, NOMINAL_READINGS)
    assert driver.step(INCONSISTENT) is Mode.FAULT


def test_fault_outranks_safe_in_the_same_tick() -> None:
    driver = driver_in(Mode.NOMINAL, SafetyConfig(sustain_tick_count=1))
    both = readings_with(frozenset({SafeFlag.OVER_TEMP}), base=INCONSISTENT)
    assert driver.step(both) is Mode.FAULT
    outcomes = [t.outcome for t in driver.last_transitions]
    assert outcomes == [Outcome.TRANSITION, Outcome.IGNORED]


# --- Reboot and determinism ------------------------------------------------------------


def test_reboot_clears_the_persistence_counters() -> None:
    driver = Driver(ScriptedComputer())
    flagged = readings_with(frozenset({SafeFlag.CRITICAL_BATTERY}))
    driver.step(flagged, ticks=N - 1)
    driver.fc.reboot(now_us=driver.now_us)
    assert driver.step(flagged, ticks=N - 1) is Mode.BOOT
    assert driver.step(flagged) is Mode.SAFE


def test_reset_clears_the_persistence_counters() -> None:
    driver = Driver(ScriptedComputer())
    flagged = readings_with(frozenset({SafeFlag.CRITICAL_BATTERY}))
    driver.step(flagged, ticks=N - 1)
    driver.fc.reset(now_us=driver.now_us)
    assert driver.step(flagged, ticks=N - 1) is Mode.BOOT


def test_flight_computer_is_deterministic() -> None:
    script = {0: [BOOT_COMPLETE], 3: [SET_SCIENCE], 20: [SET_NOMINAL], 60: [SET_NOMINAL]}
    under = readings_with(frozenset({SafeFlag.UNDER_TEMP}))
    sequence = [NOMINAL_READINGS] * 5 + [under] * 30 + [NOMINAL_READINGS] * 40 + [INCONSISTENT]

    def run() -> list[tuple[Mode, SpacecraftControls]]:
        driver = Driver(ScriptedComputer(script))
        out = []
        for readings in sequence:
            driver.step(readings)
            out.append((driver.fc.mode, driver.controls))
        return out

    first = run()
    assert first == run()
    modes = [m for m, _ in first]
    assert modes[0] is Mode.NOMINAL
    assert modes[4] is Mode.SCIENCE
    assert modes[5 + N - 1] is Mode.SAFE
    assert modes[20] is Mode.SAFE  # flags still set when SET_MODE NOMINAL came
    assert modes[60] is Mode.NOMINAL  # cleared by then
    assert modes[-1] is Mode.FAULT


# --- From fake subsystem snapshots ---------------------------------------------------------


def test_safe_entry_from_fake_snapshots_through_the_stack() -> None:
    # A scripted power fake reports critical_battery from tick 5: the readings the
    # flight computer gets come from the stack's SpacecraftState, as in SilTarget.
    power = default_snapshot("power")
    assert isinstance(power, PowerSnapshot)
    critical = replace_readings(power, soc=0.1, low_battery=True, critical_battery=True)
    fake: FakeSubsystem[PowerSnapshot] = FakeSubsystem("power", power, {5: critical})
    stack = fake_stack(fake_subsystems({"power": fake}))
    fc = ScriptedComputer({0: [BOOT_COMPLETE]})
    controls = fc.reset()
    env = EnvironmentState()
    modes = []
    for tick in range(5 + N + 2):
        stack.step(TICK_US, env, controls)
        out = fc.step((), SpacecraftReadings.from_state(stack.snapshot()), (tick + 1) * TICK_US)
        controls = out.controls
        modes.append(fc.mode)
    assert modes[: 5 + N - 1] == [Mode.NOMINAL] * (5 + N - 1)
    assert modes[5 + N - 1 :] == [Mode.SAFE] * 3
    assert controls == controls_for_mode(Mode.SAFE)
    # The truth was never consulted: it still says the battery is fine.
    assert stack.snapshot().get("power", PowerSnapshot).truth.soc == power.truth.soc


# --- Documentation -------------------------------------------------------------------------


def test_docs_describe_every_rule() -> None:
    doc = (Path(__file__).resolve().parents[2] / "docs" / "spacecraft-modes.md").read_text(
        encoding="utf-8"
    )
    section = doc.split("## Safe and fault entry rules", 1)[1].split("\n## ", 1)[0]
    for flag in SAFE_FLAGS:
        assert f"`{flag.value}`" in section
    for name in (
        "Reading not finite",
        "Battery flag order",
        "Payload bookkeeping",
        "Comms capacity",
    ):
        assert f"| {name} |" in section
    assert len(ConsistencyCheck) == 4, "add the new check to the doc's rules table"
    assert f"(default {N})" in section
