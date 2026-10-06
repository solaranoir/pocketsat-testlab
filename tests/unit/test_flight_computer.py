"""Tests for the FlightComputer skeleton and its step() interface (#101).

These tests pin the seams the phases are filled in through: the phase order, the single
controls function, BOOT on reset, the RESET path (reboot), the output record ADR-0007
relies on, and the readings-only input. The command dispatcher (#51) is tested in
``test_command_dispatcher.py``.
"""

import random
import subprocess
import sys
import textwrap
from collections.abc import Callable
from pathlib import Path
from typing import Any

import pytest

import pocketsat.flight.computer as computer
from pocketsat.flight import (
    PHASE_ORDER,
    EventKind,
    FlightComputer,
    FlightComputerOutput,
    Mode,
    ModeEvent,
    ModeState,
    SpacecraftReadings,
    Transition,
    controls_for_mode,
    transition,
)
from pocketsat.flight.computer import TickContext
from pocketsat.frame import Frame, FrameType, encode_frame
from pocketsat.spacecraft import STEP_ORDER, PowerSnapshot, SpacecraftControls
from pocketsat.spacecraft.fakes import fake_stack

REPO_ROOT = Path(__file__).resolve().parents[2]

TICK_US = 100_000

STATE = fake_stack().snapshot()
READINGS = SpacecraftReadings.from_state(STATE)

COMMAND_FRAME = encode_frame(Frame(FrameType.COMMAND, sequence=7, payload=b"\x01\x02"))
"""A well-formed COMMAND frame: PING with a stray argument byte, so it is NACKed
``PAYLOAD_TOO_LONG`` (#52) and changes nothing."""

UPLINK_SAMPLES: list[tuple[bytes, ...]] = [
    (),
    (COMMAND_FRAME,),
    (COMMAND_FRAME, COMMAND_FRAME),
    (b"",),
    (b"\x00" * 3,),
    (COMMAND_FRAME[:-1],),
    (bytes(range(256)),),
    (encode_frame(Frame(FrameType.TELEMETRY, sequence=1, payload=b"")),),
]
"""Valid, truncated, empty, garbage, and wrong-direction frames."""


def boot_complete_on_first_step(fc: FlightComputer, monkeypatch: pytest.MonkeyPatch) -> None:
    """Stand in for #49: raise BOOT_COMPLETE from the evaluate-flags phase every tick."""

    def evaluate_flags(tick: TickContext) -> None:
        tick.mode_events.append(ModeEvent(EventKind.BOOT_COMPLETE))

    monkeypatch.setattr(fc, "_evaluate_flags", evaluate_flags)


# --- Phases --------------------------------------------------------------------------


def test_phase_order_is_adr_0004_steps_c_to_f() -> None:
    assert PHASE_ORDER == (
        "decode_uplink",
        "execute_commands",
        "evaluate_flags",
        "update_mode",
        "emit_telemetry",
        "produce_controls",
    )
    for name in PHASE_ORDER:
        method = getattr(FlightComputer, f"_{name}")
        assert callable(method)
        assert method.__doc__, f"phase {name} needs a docstring naming its ticket"


def test_step_runs_every_phase_once_in_order(monkeypatch: pytest.MonkeyPatch) -> None:
    fc = FlightComputer()
    calls: list[str] = []

    def spy(name: str) -> None:
        original: Callable[..., Any] = getattr(fc, f"_{name}")

        def wrapper(*args: Any) -> Any:
            calls.append(name)
            return original(*args)

        monkeypatch.setattr(fc, f"_{name}", wrapper)

    for name in PHASE_ORDER:
        spy(name)

    fc.step((COMMAND_FRAME,), READINGS, 0)
    assert calls == list(PHASE_ORDER)
    calls.clear()
    fc.step((), READINGS, TICK_US)
    assert calls == list(PHASE_ORDER)


def test_phases_share_one_tick_context(monkeypatch: pytest.MonkeyPatch) -> None:
    fc = FlightComputer()
    seen: list[TickContext] = []
    for name in PHASE_ORDER[:-1]:
        original: Callable[[TickContext], None] = getattr(fc, f"_{name}")

        def wrapper(tick: TickContext, original: Callable[[TickContext], None] = original) -> None:
            seen.append(tick)
            original(tick)

        monkeypatch.setattr(fc, f"_{name}", wrapper)

    fc.step((COMMAND_FRAME,), READINGS, 300)
    assert len(seen) == len(PHASE_ORDER) - 1
    assert all(tick is seen[0] for tick in seen)
    assert seen[0].now_us == 300
    assert seen[0].readings is READINGS
    assert seen[0].uplink_frames == (COMMAND_FRAME,)


@pytest.mark.parametrize("frames", UPLINK_SAMPLES)
def test_any_uplink_is_handled_without_raising(frames: tuple[bytes, ...]) -> None:
    # Only the valid COMMAND frames are answered (#51); the rest are ignored.
    fc = FlightComputer()
    for tick in range(3):
        out = fc.step(frames, READINGS, tick * TICK_US)
        assert len(out.downlink_frames) == frames.count(COMMAND_FRAME)
        assert out.controls == controls_for_mode(Mode.BOOT)
    assert fc.mode is Mode.BOOT


def test_uplink_accepts_any_iterable_of_bytes() -> None:
    out = FlightComputer().step(iter([COMMAND_FRAME, b"\xff"]), READINGS, 0)
    assert out == FlightComputer().step([COMMAND_FRAME, b"\xff"], READINGS, 0)


def test_no_mode_events_means_the_mode_stays(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[ModeEvent] = []
    real = transition

    def spy(state: ModeState, event: ModeEvent, *, safe_exit_allowed: bool) -> Transition:
        calls.append(event)
        return real(state, event, safe_exit_allowed=safe_exit_allowed)

    monkeypatch.setattr(computer, "transition", spy)
    fc = FlightComputer()
    for tick in range(5):
        fc.step((COMMAND_FRAME,), READINGS, tick * TICK_US)
    assert calls == []
    assert fc.mode is Mode.BOOT


def test_update_mode_applies_events_through_transition(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[ModeState, ModeEvent]] = []
    real = transition

    def spy(state: ModeState, event: ModeEvent, *, safe_exit_allowed: bool) -> Transition:
        calls.append((state, event))
        return real(state, event, safe_exit_allowed=safe_exit_allowed)

    monkeypatch.setattr(computer, "transition", spy)
    fc = FlightComputer()
    boot_complete_on_first_step(fc, monkeypatch)

    out = fc.step((), READINGS, 0)
    assert calls == [(ModeState(Mode.BOOT), ModeEvent(EventKind.BOOT_COMPLETE))]
    assert fc.mode is Mode.NOMINAL
    # Step f uses the mode after step d.
    assert out.controls == controls_for_mode(Mode.NOMINAL)

    # BOOT_COMPLETE outside BOOT is ignored by the table, so the mode stays.
    out = fc.step((), READINGS, TICK_US)
    assert fc.mode is Mode.NOMINAL
    assert out.controls == controls_for_mode(Mode.NOMINAL)


# --- The single controls function ---------------------------------------------------


MARKED = SpacecraftControls(extra_load_w=1.25)
"""Controls no mode produces, to see where controls come from."""


def test_controls_come_only_from_produce_controls(monkeypatch: pytest.MonkeyPatch) -> None:
    fc = FlightComputer()
    monkeypatch.setattr(fc, "_produce_controls", lambda: MARKED)
    assert fc.reset() is MARKED
    assert fc.step((COMMAND_FRAME,), READINGS, 0).controls is MARKED


def test_produce_controls_uses_controls_for_mode(monkeypatch: pytest.MonkeyPatch) -> None:
    modes: list[Mode] = []

    def spy(mode: Mode) -> SpacecraftControls:
        modes.append(mode)
        return MARKED

    monkeypatch.setattr(computer, "controls_for_mode", spy)
    fc = FlightComputer()
    modes.clear()
    assert fc.reset() is MARKED
    assert fc.step((), READINGS, 0).controls is MARKED
    assert modes == [Mode.BOOT, Mode.BOOT]


@pytest.mark.parametrize("mode", list(Mode), ids=lambda m: m.name)
def test_produce_controls_follows_the_mode(mode: Mode) -> None:
    fc = FlightComputer()
    return_mode = Mode.NOMINAL if mode is Mode.DOWNLINK else None
    fc._mode_state = ModeState(mode, return_mode)
    assert fc._produce_controls() == controls_for_mode(mode)


# --- reset and reboot ---------------------------------------------------------------


def test_a_new_flight_computer_is_powered_on_in_boot() -> None:
    # ADR-0005 §5: the flight computer always starts in BOOT.
    fc = FlightComputer()
    assert fc.mode is Mode.BOOT
    assert fc.mode_state == ModeState(Mode.BOOT)
    assert fc.boot_count == 0
    assert fc.uptime_us == 0


def test_reset_returns_boot_controls_for_tick_0(monkeypatch: pytest.MonkeyPatch) -> None:
    fc = FlightComputer()
    boot_complete_on_first_step(fc, monkeypatch)
    fc.step((), READINGS, 0)
    fc.reboot(now_us=TICK_US)
    fc.step((), READINGS, 2 * TICK_US)
    assert fc.mode_state == ModeState(Mode.NOMINAL)
    assert fc.boot_count == 1

    controls = fc.reset()
    assert controls == controls_for_mode(Mode.BOOT)
    assert not controls.attitude.enabled  # BOOT's controls, not the nominal defaults
    assert controls != SpacecraftControls()
    assert fc.mode is Mode.BOOT
    assert fc.boot_count == 0
    assert fc.uptime_us == 0


def test_reset_at_a_given_time_starts_uptime_there() -> None:
    fc = FlightComputer()
    fc.reset(now_us=5 * TICK_US)
    fc.step((), READINGS, 7 * TICK_US)
    assert fc.uptime_us == 2 * TICK_US


def test_reboot_increments_boot_count_and_resets_uptime(monkeypatch: pytest.MonkeyPatch) -> None:
    fc = FlightComputer()
    boot_complete_on_first_step(fc, monkeypatch)
    for tick in range(6):
        fc.step((), READINGS, tick * TICK_US)
    assert fc.uptime_us == 5 * TICK_US
    assert fc.mode_state == ModeState(Mode.NOMINAL)

    fc.reboot(now_us=6 * TICK_US)
    assert fc.boot_count == 1
    assert fc.uptime_us == 0
    assert fc.mode is Mode.BOOT
    assert fc.mode_state == ModeState(Mode.BOOT)

    fc.step((), READINGS, 9 * TICK_US)
    assert fc.uptime_us == 3 * TICK_US

    fc.reboot(now_us=9 * TICK_US)
    fc.reboot(now_us=9 * TICK_US)
    assert fc.boot_count == 3
    assert fc.uptime_us == 0


def test_reboot_produces_boot_controls_at_the_next_step_f(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # ADR-0004 §9: BOOT's controls come from step f of the tick the reboot completes in.
    fc = FlightComputer()
    fc._mode_state = ModeState(Mode.SCIENCE)

    def reboot_in_execute_commands(tick: TickContext) -> None:
        fc.reboot(now_us=tick.now_us)  # what RESET will do (#49, #51)

    monkeypatch.setattr(fc, "_execute_commands", reboot_in_execute_commands)
    out = fc.step((COMMAND_FRAME,), READINGS, TICK_US)
    assert fc.mode is Mode.BOOT
    assert out.controls == controls_for_mode(Mode.BOOT)
    assert fc.boot_count == 1


# --- Output record ------------------------------------------------------------------


def test_output_traffic_counts_match_the_frames_sent() -> None:
    fc = FlightComputer()
    for tick, frames in enumerate(UPLINK_SAMPLES):
        out = fc.step(frames, READINGS, tick * TICK_US)
        assert isinstance(out, FlightComputerOutput)
        assert len(out.downlink_frames) == frames.count(COMMAND_FRAME)  # one NACK each
        assert out.sent_bytes == sum(len(frame) for frame in out.downlink_frames)
        assert out.outbound_suppressed_count == 0  # the fake's capacity fits them all


def test_output_sent_bytes_is_the_wire_length_of_every_frame() -> None:
    # ADR-0007 §1: header, payload, and CRC of every outbound frame, all types.
    frames = (COMMAND_FRAME, b"\x01\x02\x03")
    out = FlightComputerOutput(frames, controls_for_mode(Mode.BOOT), outbound_suppressed_count=2)
    assert out.sent_bytes == len(COMMAND_FRAME) + 3
    assert out.outbound_suppressed_count == 2


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"downlink_frames": [b"\x01"]}, TypeError),
        ({"downlink_frames": ("frame",)}, TypeError),
        ({"controls": None}, TypeError),
        ({"outbound_suppressed_count": -1}, ValueError),
        ({"outbound_suppressed_count": 1.0}, TypeError),
        ({"outbound_suppressed_count": True}, TypeError),
    ],
)
def test_output_is_validated(kwargs: dict[str, Any], error: type[Exception]) -> None:
    base: dict[str, Any] = {
        "downlink_frames": (),
        "controls": controls_for_mode(Mode.BOOT),
        "outbound_suppressed_count": 0,
    }
    with pytest.raises(error):
        FlightComputerOutput(**(base | kwargs))


# --- Determinism --------------------------------------------------------------------


def _run(seed: int, monkeypatch: pytest.MonkeyPatch) -> list[tuple[FlightComputerOutput, Mode]]:
    rng = random.Random(seed)
    fc = FlightComputer()
    boot_complete_on_first_step(fc, monkeypatch)
    history: list[tuple[FlightComputerOutput, Mode]] = [(fc.step((), READINGS, 0), fc.mode)]
    for tick in range(1, 50):
        frames = tuple(rng.randbytes(rng.randrange(0, 40)) for _ in range(rng.randrange(0, 3)))
        if rng.random() < 0.1:
            fc.reboot(now_us=tick * TICK_US)
        history.append((fc.step(frames, READINGS, tick * TICK_US), fc.mode))
    return history


def test_step_is_deterministic(monkeypatch: pytest.MonkeyPatch) -> None:
    assert _run(11, monkeypatch) == _run(11, monkeypatch)


def test_step_has_no_hidden_inputs() -> None:
    a, b = FlightComputer(), FlightComputer()
    for tick, frames in enumerate(UPLINK_SAMPLES):
        out_a = a.step(frames, READINGS, tick * TICK_US)
        out_b = b.step(frames, READINGS, tick * TICK_US)
        assert out_a == out_b
        assert (a.mode_state, a.boot_count, a.uptime_us) == (
            b.mode_state,
            b.boot_count,
            b.uptime_us,
        )


# --- Inputs -------------------------------------------------------------------------


def test_readings_are_taken_from_the_spacecraft_state() -> None:
    assert READINGS.power is STATE.get("power", PowerSnapshot).readings
    for name in STEP_ORDER:
        snapshot: Any = STATE.subsystems[name]
        assert getattr(READINGS, name) is snapshot.readings


def test_readings_reject_a_truth_record() -> None:
    power: Any = STATE.get("power", PowerSnapshot).truth
    with pytest.raises(TypeError, match="PowerReadings"):
        SpacecraftReadings(
            power=power,
            thermal=READINGS.thermal,
            attitude=READINGS.attitude,
            payload=READINGS.payload,
            comms=READINGS.comms,
        )


def test_step_rejects_a_spacecraft_state() -> None:
    state: Any = STATE
    with pytest.raises(TypeError, match="SpacecraftReadings"):
        FlightComputer().step((), state, 0)


@pytest.mark.parametrize(
    ("frames", "now_us", "error"),
    [
        (("frame",), 0, TypeError),
        ((bytearray(b"\x01"),), 0, TypeError),
        ((), 0.0, TypeError),
        ((), True, TypeError),
        ((), -1, ValueError),
    ],
)
def test_step_validates_its_arguments(frames: Any, now_us: Any, error: type[Exception]) -> None:
    with pytest.raises(error):
        FlightComputer().step(frames, READINGS, now_us)


def test_time_never_goes_backwards() -> None:
    fc = FlightComputer()
    fc.step((), READINGS, 2 * TICK_US)
    with pytest.raises(ValueError, match="backwards"):
        fc.step((), READINGS, TICK_US)
    with pytest.raises(ValueError, match="backwards"):
        fc.reboot(now_us=TICK_US)
    fc.step((), READINGS, 2 * TICK_US)  # the same tick again is allowed


# --- Placement ----------------------------------------------------------------------


def test_flight_computer_is_not_in_step_order() -> None:
    assert not any("flight" in name for name in STEP_ORDER)
    assert FlightComputer.__doc__ is not None
    assert "STEP_ORDER" in FlightComputer.__doc__
    assert "ADR-0004" in FlightComputer.__doc__


@pytest.mark.parametrize(
    "first",
    [
        "pocketsat.messages",
        "pocketsat.flight",
        "pocketsat.flight.computer",
        "pocketsat.flight.telemetry",
    ],
)
def test_imports_have_no_cycle_whichever_module_comes_first(first: str) -> None:
    # pocketsat.messages imports Mode from pocketsat.flight.modes; see pocketsat.flight.
    code = f"import {first}\nimport pocketsat.messages, pocketsat.flight\n"
    subprocess.run([sys.executable, "-c", code], check=True, cwd=REPO_ROOT)


@pytest.mark.slow
def test_mypy_rejects_truth_records_and_spacecraft_state(tmp_path: Path) -> None:
    # ADR-0004 §6, §7: decisions read reported values only, so the input is typed to them.
    from mypy import api

    source = textwrap.dedent(
        """\
        from pocketsat.flight import FlightComputer, SpacecraftReadings
        from pocketsat.spacecraft import PowerSnapshot, SpacecraftState
        from pocketsat.spacecraft.fakes import fake_stack

        state: SpacecraftState = fake_stack().snapshot()
        readings = SpacecraftReadings.from_state(state)
        truth = state.get("power", PowerSnapshot).truth
        FlightComputer().step((), state, 0)
        SpacecraftReadings(truth, readings.thermal, readings.attitude, readings.payload,
                           readings.comms)
        """
    )
    module = tmp_path / "flight_input.py"
    module.write_text(source)
    stdout, stderr, status = api.run(
        ["--strict", "--no-incremental", "--cache-dir", str(tmp_path / ".mypy_cache"), str(module)]
    )
    assert status == 1, stdout + stderr
    assert 'Argument 2 to "step" of "FlightComputer"' in stdout
    assert 'Argument 1 to "SpacecraftReadings"' in stdout
    assert stdout.count("error:") == 2, stdout
