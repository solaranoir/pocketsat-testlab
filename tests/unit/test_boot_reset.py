"""Tests for the BOOT sequence, the RESET reboot, and the uptime counter (#49).

The flight computer is fed readings from fake snapshots. :class:`CommandStub` stands in
for the command dispatcher (#51), so each test raises exactly the events it needs: it
raises scripted mode
events in the execute-commands phase and queues a marker frame for each RESET, as #51
queues the RESET's ACK there.
"""

import dataclasses
import random
from collections.abc import Mapping, Sequence
from typing import Any

import pytest

from pocketsat.flight import (
    EventKind,
    FlightComputer,
    FlightComputerConfig,
    Mode,
    ModeEvent,
    ModeState,
    Outcome,
    RejectReason,
    SpacecraftReadings,
    controls_for_mode,
)
from pocketsat.flight.boot import (
    DEFAULT_BOOT_CONFIG,
    US_PER_MS,
    BootConfig,
    boot_complete,
    uptime_ms,
)
from pocketsat.flight.computer import TickContext
from pocketsat.flight.safety import (
    DEFAULT_SAFETY_CONFIG,
    INITIAL_SAFETY_STATE,
    SafeFlag,
    SafetyState,
    active_flags,
    consistency_failures,
)
from pocketsat.frame import FrameType, decode_frame
from pocketsat.messages import (
    UINT16_MAX,
    UINT32_MAX,
    FlightComputerTelemetryState,
    decode_telemetry,
    encode_telemetry,
)
from pocketsat.spacecraft.fakes import fake_stack

TICK_US = 100_000
N = DEFAULT_SAFETY_CONFIG.sustain_tick_count
BOOT_TICKS = DEFAULT_BOOT_CONFIG.duration_us // TICK_US
"""Ticks under BOOT's controls after power-on: 50 at the default 5 s and 100 ms tick."""

NOMINAL_READINGS = SpacecraftReadings.from_state(fake_stack().snapshot())
CRITICAL_READINGS = dataclasses.replace(
    NOMINAL_READINGS,
    power=dataclasses.replace(NOMINAL_READINGS.power, low_battery=True, critical_battery=True),
)
"""``critical_battery`` set: SAFE_CONDITION once sustained for N ticks."""
INCONSISTENT_READINGS = dataclasses.replace(
    NOMINAL_READINGS,
    power=dataclasses.replace(NOMINAL_READINGS.power, low_battery=False, critical_battery=True),
)
"""``critical_battery`` without ``low_battery``: FAULT_DETECTED in the first tick."""

BOOT_CONTROLS = controls_for_mode(Mode.BOOT)
ACK_MARKER = b"ACK RESET"
"""Stands in for the RESET's ACK frame, which #51 queues in the execute-commands phase."""

BOOT_COMPLETE = ModeEvent(EventKind.BOOT_COMPLETE)
RESET = ModeEvent(EventKind.RESET)
SET_SCIENCE = ModeEvent(EventKind.SET_MODE, Mode.SCIENCE)
BEGIN_DOWNLINK = ModeEvent(EventKind.BEGIN_DOWNLINK)
ENTER_SAFE = ModeEvent(EventKind.ENTER_SAFE_MODE)
FAULT_DETECTED = ModeEvent(EventKind.FAULT_DETECTED)


class CommandStub(FlightComputer):
    """A flight computer whose execute-commands phase raises ``script[k]``'s events in
    the k-th :meth:`step` call (0-based), queuing :data:`ACK_MARKER` for each RESET.

    Every step's tick context is kept in :attr:`ticks`.
    """

    def __init__(
        self, script: Mapping[int, Sequence[ModeEvent]] | None = None, **kwargs: Any
    ) -> None:
        self.script = dict(script or {})
        self.ticks: list[TickContext] = []
        self.steps = 0
        super().__init__(**kwargs)

    def step(self, *args: Any, **kwargs: Any) -> Any:
        try:
            return super().step(*args, **kwargs)
        finally:
            self.steps += 1

    def _execute_commands(self, tick: TickContext) -> None:
        for event in self.script.get(self.steps, ()):
            tick.mode_events.append(event)
            if event.kind is EventKind.RESET:
                tick.downlink_frames.append(ACK_MARKER)
        self.ticks.append(tick)


class Driver:
    """Steps a flight computer one tick at a time from power-on at time 0."""

    def __init__(self, fc: CommandStub) -> None:
        self.fc = fc
        self.now_us = 0
        self.controls = fc.reset()
        self.frames: list[tuple[bytes, ...]] = []
        self.modes: list[Mode] = []
        self.controls_history = [self.controls]
        """Controls in force in tick k: BOOT's for tick 0, then each step's output."""

    def step(self, readings: SpacecraftReadings = NOMINAL_READINGS, ticks: int = 1) -> Mode:
        for _ in range(ticks):
            self.now_us += TICK_US
            out = self.fc.step((), readings, self.now_us)
            self.controls = out.controls
            self.controls_history.append(out.controls)
            self.frames.append(out.downlink_frames)
            self.modes.append(self.fc.mode)
        return self.fc.mode


def mode_of(fc: FlightComputer) -> Mode:
    """``fc.mode``, read through a call so mypy doesn't narrow it between steps."""
    return fc.mode


def reach(mode: Mode, script: dict[int, list[ModeEvent]] | None = None) -> Driver:
    """A driver whose flight computer has reached ``mode`` through scripted events (a
    scripted BOOT_COMPLETE skips the wait), with ``script``'s further events."""
    path: dict[Mode, list[ModeEvent]] = {
        Mode.BOOT: [],
        Mode.NOMINAL: [BOOT_COMPLETE],
        Mode.SCIENCE: [BOOT_COMPLETE, SET_SCIENCE],
        Mode.DOWNLINK: [BOOT_COMPLETE, BEGIN_DOWNLINK],
        Mode.SAFE: [BOOT_COMPLETE, ENTER_SAFE],
        Mode.FAULT: [FAULT_DETECTED],
    }
    events = {k: [event] for k, event in enumerate(path[mode])}
    driver = Driver(CommandStub({**events, **(script or {})}))
    driver.step(ticks=len(events))
    assert mode_of(driver.fc) is mode
    return driver


# --- Settings and helpers -----------------------------------------------------------------


def test_default_boot_lasts_five_seconds() -> None:
    assert BootConfig() == DEFAULT_BOOT_CONFIG
    assert DEFAULT_BOOT_CONFIG.duration_us == 5_000_000
    assert BOOT_TICKS == 50


@pytest.mark.parametrize(("bad", "error"), [(-1, ValueError), (1.5, TypeError), (True, TypeError)])
def test_boot_config_rejects_bad_durations(bad: Any, error: type[Exception]) -> None:
    with pytest.raises(error, match="duration_us"):
        BootConfig(duration_us=bad)


def test_flight_computer_config_rejects_a_bad_boot_config() -> None:
    with pytest.raises(TypeError, match="BootConfig"):
        FlightComputerConfig(boot=5_000_000)  # type: ignore[arg-type]


def test_boot_complete_once_the_uptime_reaches_the_duration() -> None:
    config = BootConfig(duration_us=300)
    assert not boot_complete(0, config)
    assert not boot_complete(299, config)
    assert boot_complete(300, config)
    assert boot_complete(10**12, config)
    assert boot_complete(0, BootConfig(duration_us=0))


@pytest.mark.parametrize(
    ("uptime_us", "expected"),
    [(0, 0), (999, 0), (1000, 1), (1999, 1), (5_000_000, 5000), (2**40 + 7, (2**40 + 7) // 1000)],
)
def test_uptime_ms_rounds_down_and_does_not_wrap(uptime_us: int, expected: int) -> None:
    assert uptime_ms(uptime_us) == expected
    assert US_PER_MS == 1000


@pytest.mark.parametrize(("bad", "error"), [(-1, ValueError), (1.0, TypeError)])
def test_uptime_ms_rejects_bad_values(bad: Any, error: type[Exception]) -> None:
    with pytest.raises(error):
        uptime_ms(bad)


# --- Power-on and BOOT_COMPLETE ---------------------------------------------------------


def test_power_on_boots_to_nominal_after_the_boot_duration() -> None:
    driver = Driver(CommandStub())
    driver.step(ticks=BOOT_TICKS + 20)
    # The step at the end of tick 49 (uptime 5 s) raises BOOT_COMPLETE.
    assert driver.modes == [Mode.BOOT] * (BOOT_TICKS - 1) + [Mode.NOMINAL] * 21
    # BOOT's controls apply to ticks 0 to 49 (5 s), NOMINAL's from tick 50.
    nominal = controls_for_mode(Mode.NOMINAL)
    assert driver.controls_history[: BOOT_TICKS + 1] == [BOOT_CONTROLS] * BOOT_TICKS + [nominal]
    completed = driver.fc.ticks[BOOT_TICKS - 1]
    assert completed.now_us == DEFAULT_BOOT_CONFIG.duration_us
    assert completed.mode_events == [BOOT_COMPLETE]
    assert completed.transitions[0].outcome is Outcome.TRANSITION


def test_boot_complete_is_raised_exactly_once() -> None:
    driver = Driver(CommandStub())
    driver.step(ticks=3 * BOOT_TICKS)
    events = [event for tick in driver.fc.ticks for event in tick.mode_events]
    assert events == [BOOT_COMPLETE]


@pytest.mark.parametrize(
    ("duration_us", "boot_ticks"),
    [(0, 1), (1, 1), (TICK_US, 1), (TICK_US + 1, 2), (250_000, 3), (3 * TICK_US, 3)],
)
def test_boot_duration_is_configurable_and_rounds_up_to_whole_ticks(
    duration_us: int, boot_ticks: int
) -> None:
    driver = Driver(
        CommandStub(config=FlightComputerConfig(boot=BootConfig(duration_us=duration_us)))
    )
    driver.step(ticks=boot_ticks + 2)
    assert driver.modes == [Mode.BOOT] * (boot_ticks - 1) + [Mode.NOMINAL] * 3
    # BOOT's controls apply to tick 0 even when the boot completes in its step.
    assert driver.controls_history[:boot_ticks] == [BOOT_CONTROLS] * boot_ticks


def test_boot_counts_uptime_from_power_on_at_any_time() -> None:
    fc = CommandStub(config=FlightComputerConfig(boot=BootConfig(duration_us=3 * TICK_US)))
    fc.reset(now_us=10 * TICK_US)
    fc.step((), NOMINAL_READINGS, 12 * TICK_US)
    assert mode_of(fc) is Mode.BOOT
    fc.step((), NOMINAL_READINGS, 13 * TICK_US)
    assert mode_of(fc) is Mode.NOMINAL


def test_boot_controls_are_held_throughout_boot() -> None:
    driver = Driver(CommandStub())
    driver.step(ticks=BOOT_TICKS - 1)
    assert mode_of(driver.fc) is Mode.BOOT
    for controls in driver.controls_history:
        assert controls == BOOT_CONTROLS
        assert not controls.attitude.enabled
        assert not controls.payload.enabled
        assert controls.radio == controls_for_mode(Mode.NOMINAL).radio  # RX_TX


def test_boot_sends_nothing_but_the_boot_complete_beacon() -> None:
    # No telemetry in BOOT (#55); the first frame is the beacon, sent in the step that
    # completes the boot, showing NOMINAL. The next is one NOMINAL period later.
    driver = Driver(CommandStub())
    driver.step(ticks=BOOT_TICKS + 5)
    assert all(frames == () for frames in driver.frames[: BOOT_TICKS - 1])
    [beacon] = driver.frames[BOOT_TICKS - 1]
    frame = decode_frame(beacon)
    assert frame.frame_type is FrameType.TELEMETRY and frame.sequence == 0
    telemetry = decode_telemetry(frame.payload)
    assert (telemetry.mode, telemetry.uptime_ms, telemetry.boot_count) == (Mode.NOMINAL, 5000, 0)
    assert all(frames == () for frames in driver.frames[BOOT_TICKS:])


# --- RESET from every mode ------------------------------------------------------------------


@pytest.mark.parametrize("mode", list(Mode), ids=lambda m: m.name)
def test_reset_from_every_mode_reboots_into_boot(mode: Mode) -> None:
    driver = reach(mode)
    driver.fc.script[len(driver.modes) + 4] = [RESET]
    driver.step(ticks=4)
    before = driver.fc.uptime_us
    assert driver.fc.boot_count == 0

    driver.step()  # the RESET tick
    tick = driver.fc.ticks[-1]
    assert tick.transitions[0].outcome is Outcome.TRANSITION
    assert mode_of(driver.fc) is Mode.BOOT
    assert driver.fc.mode_state == ModeState(Mode.BOOT)
    assert driver.fc.boot_count == 1
    assert driver.fc.uptime_us == 0 < before
    assert driver.controls == BOOT_CONTROLS  # from step f of the RESET tick
    assert driver.frames[-1] == (ACK_MARKER,)  # the ACK still goes out

    # Uptime restarts from the RESET, and the boot runs again in full.
    driver.step(ticks=BOOT_TICKS - 1)
    assert mode_of(driver.fc) is Mode.BOOT
    assert driver.fc.uptime_us == (BOOT_TICKS - 1) * TICK_US
    driver.step()  # uptime 5 s: BOOT's controls applied to the 50 ticks after the RESET
    assert driver.controls_history[-BOOT_TICKS - 1 : -1] == [BOOT_CONTROLS] * BOOT_TICKS
    assert mode_of(driver.fc) is Mode.NOMINAL
    assert driver.fc.boot_count == 1


def test_boot_count_persists_across_resets_and_power_on_clears_it() -> None:
    driver = Driver(CommandStub({3: [RESET], 70: [RESET], 71: [RESET]}))
    driver.step(ticks=72)
    assert driver.fc.boot_count == 3
    driver.fc.reboot(now_us=driver.now_us)  # forced_reset's path (#60)
    assert driver.fc.boot_count == 4
    driver.fc.reset(now_us=driver.now_us)  # power-on
    assert driver.fc.boot_count == 0


def test_reset_clears_the_safe_mode_counters() -> None:
    driver = reach(Mode.NOMINAL)
    driver.step(CRITICAL_READINGS, ticks=N - 1)  # one tick short of SAFE
    assert mode_of(driver.fc) is Mode.NOMINAL
    driver.fc.script[len(driver.modes)] = [RESET]
    driver.step(CRITICAL_READINGS)  # the N-th flagged tick, with RESET
    assert mode_of(driver.fc) is Mode.BOOT
    assert driver.fc._safety_state == INITIAL_SAFETY_STATE
    # The flag must be sustained again from BOOT: N more ticks, not 1.
    assert driver.step(CRITICAL_READINGS, ticks=N - 1) is Mode.BOOT
    assert driver.step(CRITICAL_READINGS) is Mode.SAFE


def test_automatic_events_after_a_reset_are_not_applied() -> None:
    # In the RESET tick, the rules sustained critical_battery (SAFE_CONDITION) and the
    # boot duration had elapsed (BOOT_COMPLETE). Both came from the state the reboot
    # discarded, so they are recorded as IGNORED and the mode stays BOOT.
    fc = CommandStub({0: [RESET]})
    fc._safety_state = SafetyState((N - 1, 0, 0))
    assert fc.step((), CRITICAL_READINGS, DEFAULT_BOOT_CONFIG.duration_us).controls == (
        BOOT_CONTROLS
    )
    tick = fc.ticks[-1]
    assert [e.kind for e in tick.mode_events] == [
        EventKind.RESET,
        EventKind.SAFE_CONDITION,
        EventKind.BOOT_COMPLETE,
    ]
    assert [t.outcome for t in tick.transitions] == [
        Outcome.TRANSITION,
        Outcome.IGNORED,
        Outcome.IGNORED,
    ]
    assert mode_of(fc) is Mode.BOOT
    assert fc.boot_count == 1


def test_commands_after_a_reset_meet_boot() -> None:
    driver = reach(Mode.NOMINAL)
    driver.fc.script[len(driver.modes)] = [RESET, SET_SCIENCE, RESET]
    driver.step()
    tick = driver.fc.ticks[-1]
    outcomes = [(t.outcome, t.reason) for t in tick.transitions]
    assert outcomes == [
        (Outcome.TRANSITION, None),
        (Outcome.REJECTED, RejectReason.BOOT_IN_PROGRESS),
        (Outcome.TRANSITION, None),
    ]
    assert driver.fc.boot_count == 2
    assert driver.frames[-1] == (ACK_MARKER, ACK_MARKER)


def test_events_before_a_reset_still_apply() -> None:
    driver = reach(Mode.NOMINAL)
    driver.fc.script[len(driver.modes)] = [SET_SCIENCE, RESET]
    driver.step()
    assert [t.mode for t in driver.fc.ticks[-1].transitions] == [Mode.SCIENCE, Mode.BOOT]
    assert mode_of(driver.fc) is Mode.BOOT


# --- BOOT -> SAFE and BOOT -> FAULT ----------------------------------------------------


def test_safe_during_boot_goes_straight_to_safe_without_boot_complete() -> None:
    driver = Driver(CommandStub())
    driver.step(CRITICAL_READINGS, ticks=N)
    assert driver.modes == [Mode.BOOT] * (N - 1) + [Mode.SAFE]
    # The boot steps that ran at power-on stand: boot count and uptime are kept.
    assert driver.fc.boot_count == 0
    assert driver.fc.uptime_us == N * TICK_US
    # SAFE's controls (attitude control on) from the next tick.
    assert driver.controls == controls_for_mode(Mode.SAFE)
    assert driver.controls.attitude.enabled

    # Long past the boot duration, with the flag cleared: no BOOT_COMPLETE, still SAFE.
    driver.step(ticks=2 * BOOT_TICKS)
    assert mode_of(driver.fc) is Mode.SAFE
    events = [e.kind for tick in driver.fc.ticks for e in tick.mode_events]
    assert EventKind.BOOT_COMPLETE not in events
    assert driver.fc.uptime_us == (N + 2 * BOOT_TICKS) * TICK_US


def test_safe_during_boot_after_a_reset() -> None:
    driver = reach(Mode.SCIENCE, {5: [RESET]})
    driver.step(ticks=4)  # steps 2 to 5; the RESET at step 5
    assert mode_of(driver.fc) is Mode.BOOT
    assert driver.fc.boot_count == 1
    assert driver.step(CRITICAL_READINGS, ticks=N) is Mode.SAFE
    assert driver.fc.boot_count == 1
    assert driver.fc.uptime_us == N * TICK_US


def test_safe_and_boot_complete_in_the_same_tick_gives_safe() -> None:
    fc = CommandStub(config=FlightComputerConfig(boot=BootConfig(duration_us=N * TICK_US)))
    driver = Driver(fc)
    driver.step(CRITICAL_READINGS, ticks=N)
    tick = fc.ticks[-1]
    assert [e.kind for e in tick.mode_events] == [EventKind.SAFE_CONDITION, EventKind.BOOT_COMPLETE]
    assert [t.outcome for t in tick.transitions] == [Outcome.TRANSITION, Outcome.IGNORED]
    assert mode_of(fc) is Mode.SAFE


def test_fault_during_boot_goes_straight_to_fault() -> None:
    driver = Driver(CommandStub())
    driver.step(INCONSISTENT_READINGS)
    assert mode_of(driver.fc) is Mode.FAULT
    driver.step(ticks=2 * BOOT_TICKS)
    assert mode_of(driver.fc) is Mode.FAULT  # only RESET leaves it
    driver.fc.script[len(driver.modes)] = [RESET]
    driver.step()
    assert mode_of(driver.fc) is Mode.BOOT
    driver.step(ticks=BOOT_TICKS)
    assert mode_of(driver.fc) is Mode.NOMINAL


# --- forced_reset's path: reboot() between steps ----------------------------------------------


def test_a_reboot_between_steps_takes_effect_immediately() -> None:
    # forced_reset (#60) calls reboot() from SilTarget, outside step().
    driver = reach(Mode.SCIENCE)
    driver.step(ticks=3)
    driver.fc.reboot(now_us=driver.now_us)
    assert mode_of(driver.fc) is Mode.BOOT
    assert driver.fc.boot_count == 1
    assert driver.fc.uptime_us == 0
    assert driver.fc._safety_state == INITIAL_SAFETY_STATE
    driver.step()  # BOOT's controls come from this step's step f
    assert driver.controls == BOOT_CONTROLS
    assert driver.fc.uptime_us == TICK_US


def test_a_reboot_after_skipped_steps_counts_uptime_from_the_reboot() -> None:
    # As #60 holds in reset: the target skips the flight computer's steps for the hold,
    # then reboots it at the end of the hold, so the boot runs from there in full.
    driver = reach(Mode.NOMINAL)
    hold_end_us = driver.now_us + 7 * TICK_US
    driver.fc.reboot(now_us=hold_end_us)
    driver.now_us = hold_end_us
    driver.step(ticks=BOOT_TICKS - 1)
    assert mode_of(driver.fc) is Mode.BOOT
    driver.step()
    assert mode_of(driver.fc) is Mode.NOMINAL
    assert driver.fc.uptime_us == BOOT_TICKS * TICK_US


# --- Uptime and boot count as telemetry sees them --------------------------------------------


def telemetry_of(fc: FlightComputer) -> tuple[int, int]:
    """Encode and decode ``fc``'s state as the telemetry scheduler (#55) will."""
    state = FlightComputerTelemetryState(
        uptime_ms=fc.uptime_ms, mode=fc.mode, boot_count=fc.boot_count
    )
    r = NOMINAL_READINGS
    payload = encode_telemetry(
        state,
        power=r.power,
        thermal=r.thermal,
        attitude=r.attitude,
        payload=r.payload,
        comms=r.comms,
    )
    decoded = decode_telemetry(payload)
    return decoded.uptime_ms, decoded.boot_count


def test_uptime_ms_is_exposed_for_telemetry() -> None:
    fc = FlightComputer()
    fc.step((), NOMINAL_READINGS, 1_234_567)
    assert fc.uptime_us == 1_234_567
    assert fc.uptime_ms == 1234
    assert telemetry_of(fc) == (1234, 0)


def test_uptime_ms_wraps_on_the_wire_after_49_7_days() -> None:
    fc = FlightComputer(FlightComputerConfig(boot=BootConfig(duration_us=0)))
    wrap_us = (UINT32_MAX + 1) * US_PER_MS  # 2**32 ms, about 49.7 days
    fc.step((), NOMINAL_READINGS, wrap_us - US_PER_MS)
    assert telemetry_of(fc) == (UINT32_MAX, 0)
    fc.step((), NOMINAL_READINGS, wrap_us)
    assert fc.uptime_ms == UINT32_MAX + 1  # not wrapped in the flight computer
    assert telemetry_of(fc) == (0, 0)
    fc.step((), NOMINAL_READINGS, wrap_us + 1234 * US_PER_MS + 999)
    assert telemetry_of(fc) == (1234, 0)
    # A RESET restarts the counter.
    fc.reboot(now_us=wrap_us + 2000 * US_PER_MS)
    assert telemetry_of(fc) == (0, 1)


def test_boot_count_saturates_on_the_wire() -> None:
    fc = FlightComputer()
    for _ in range(UINT16_MAX):
        fc.reboot(now_us=0)
    assert telemetry_of(fc) == (0, UINT16_MAX)
    fc.reboot(now_us=0)
    fc.reboot(now_us=0)
    assert fc.boot_count == UINT16_MAX + 2
    assert telemetry_of(fc) == (0, UINT16_MAX)  # saturated, never wrapped to look new


# --- Determinism --------------------------------------------------------------------------


def _history(seed: int) -> list[tuple[Any, ...]]:
    rng = random.Random(seed)
    script = {
        k: [rng.choice([RESET, SET_SCIENCE, ENTER_SAFE, BEGIN_DOWNLINK])]
        for k in range(400)
        if rng.random() < 0.05
    }
    fc = CommandStub(script)
    fc.reset()
    history: list[tuple[Any, ...]] = []
    for k in range(400):
        now_us = (k + 1) * TICK_US
        if rng.random() < 0.01:
            fc.reboot(now_us=now_us - TICK_US)
        readings = CRITICAL_READINGS if rng.random() < 0.3 else NOMINAL_READINGS
        out = fc.step((), readings, now_us)
        history.append((out, fc.mode_state, fc.boot_count, fc.uptime_us))
    return history


def test_boot_and_reset_are_deterministic() -> None:
    a = _history(7)
    assert a == _history(7)
    modes = {state for _, state, *_ in a}
    assert len(modes) > 2, "the run exercises several modes"
    assert a[-1][2] > 0, "and reboots"


def test_the_readings_fixtures_raise_what_the_tests_assume() -> None:
    assert active_flags(CRITICAL_READINGS) == {SafeFlag.CRITICAL_BATTERY}
    assert consistency_failures(CRITICAL_READINGS) == ()
    assert consistency_failures(INCONSISTENT_READINGS) != ()
