"""Tests for the telemetry scheduler (#55) and the flight computer's settings record.

The flight computer is fed readings from fake snapshots and stepped every 100 ms of
simulated time. Most tests put it straight into a mode (as the dispatcher tests do), so
its first step is a mode change and sends a frame at once; BOOT, the boot-complete
beacon, and RESET are reached the real way. ``SilTarget`` with the real subsystems is
covered in ``tests/sil`` (``test_boot_reset_story.py``, ``test_sil_faults.py``,
``test_sil_target.py``, ``test_sil_determinism.py``).
"""

import dataclasses
import math
import random
from pathlib import Path
from typing import Any

import pytest

from pocketsat import messages
from pocketsat.flight import (
    DEFAULT_FLIGHT_COMPUTER_CONFIG,
    FlightComputer,
    FlightComputerConfig,
    FlightComputerOutput,
    Mode,
    ModeState,
    SpacecraftReadings,
)
from pocketsat.flight.boot import DEFAULT_BOOT_CONFIG, BootConfig
from pocketsat.flight.safety import DEFAULT_SAFETY_CONFIG, SafetyConfig
from pocketsat.flight.telemetry import (
    DEFAULT_TELEMETRY_CONFIG,
    INITIAL_TELEMETRY_SCHEDULE,
    TelemetryConfig,
    TelemetrySchedule,
    telemetry_due,
)
from pocketsat.frame import MAX_SEQUENCE, Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    TELEMETRY_FRAME_SIZE,
    Command,
    CommandAck,
    CommandId,
    FlightComputerTelemetryState,
    Telemetry,
    decode_ack,
    decode_telemetry,
    encode_command,
    encode_telemetry,
)
from pocketsat.spacecraft.fakes import fake_stack

TICK_US = 100_000
DOC = Path(__file__).resolve().parents[2] / "docs" / "spacecraft-modes.md"

READINGS = SpacecraftReadings.from_state(fake_stack().snapshot())
"""Nominal readings: no flags, radio RX_TX, 120 bytes of transmit capacity."""

CAPACITY = READINGS.comms.transmit_capacity_bytes

CADENCE_TICKS: dict[Mode, int] = {
    Mode.NOMINAL: 10,
    Mode.SCIENCE: 10,
    Mode.DOWNLINK: 10,
    Mode.SAFE: 5,
    Mode.FAULT: 5,
}
"""The default cadence in ticks at the 100 ms tick (``docs/spacecraft-modes.md``)."""

EVERY_TICK = FlightComputerConfig(telemetry=TelemetryConfig.uniform(TICK_US))


def with_capacity(capacity: int) -> SpacecraftReadings:
    """``READINGS`` with comms' transmit capacity set; 0 also turns the transmitter off,
    as ``transmitter_off`` does (#60)."""
    comms = dataclasses.replace(
        READINGS.comms, transmit_capacity_bytes=capacity, transmitter_on=capacity > 0
    )
    return dataclasses.replace(READINGS, comms=comms)


def command_frame(command: Command, sequence: int = 1) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


class Driver:
    """Steps a flight computer one tick at a time from power-on at time 0, keeping
    every output."""

    def __init__(self, fc: FlightComputer) -> None:
        self.fc = fc
        self.now_us = 0
        fc.reset()
        self.outputs: list[FlightComputerOutput] = []

    def step(
        self,
        frames: tuple[bytes, ...] = (),
        readings: SpacecraftReadings = READINGS,
        ticks: int = 1,
    ) -> FlightComputerOutput:
        for _ in range(ticks):
            self.now_us += TICK_US
            self.outputs.append(self.fc.step(frames, readings, self.now_us))
        return self.outputs[-1]

    def telemetry_ticks(self, start: int = 0) -> list[int]:
        """The indices (0-based steps) of the outputs that carry a TELEMETRY frame."""
        return [
            k
            for k, out in enumerate(self.outputs)
            if k >= start and telemetry_frames(out.downlink_frames)
        ]


def in_mode(mode: Mode, config: FlightComputerConfig = DEFAULT_FLIGHT_COMPUTER_CONFIG) -> Driver:
    """A driver whose flight computer is put straight into ``mode`` (DOWNLINK returns
    to NOMINAL), so its first step is a mode change."""
    fc = FlightComputer(config)
    driver = Driver(fc)
    fc._mode_state = ModeState(mode, Mode.NOMINAL if mode is Mode.DOWNLINK else None)
    return driver


def telemetry_frames(frames: tuple[bytes, ...] | list[bytes]) -> list[Telemetry]:
    decoded = [decode_frame(frame) for frame in frames]
    return [decode_telemetry(f.payload) for f in decoded if f.frame_type is FrameType.TELEMETRY]


def mode_of(fc: FlightComputer) -> Mode:
    """``fc.mode``, read through a call so mypy doesn't narrow it between steps."""
    return fc.mode


# --- Settings ---------------------------------------------------------------------------


def test_default_cadence_per_mode() -> None:
    config = TelemetryConfig()
    assert config == DEFAULT_TELEMETRY_CONFIG
    assert config.period_us(Mode.BOOT) is None
    assert {mode: config.period_us(mode) for mode in CADENCE_TICKS} == {
        mode: ticks * TICK_US for mode, ticks in CADENCE_TICKS.items()
    }
    # Faster in SAFE (and FAULT) than in the normal modes (#55).
    assert config.safe_period_us < config.nominal_period_us


def test_every_mode_but_boot_has_a_period() -> None:
    config = TelemetryConfig(1, 2, 3, 4, 5)
    assert [config.period_us(mode) for mode in Mode] == [None, 1, 2, 3, 4, 5]


def test_uniform_sets_every_period() -> None:
    config = TelemetryConfig.uniform(TICK_US)
    assert {config.period_us(mode) for mode in Mode} == {None, TICK_US}


@pytest.mark.parametrize("field", [f.name for f in dataclasses.fields(TelemetryConfig)])
@pytest.mark.parametrize(
    ("bad", "error"), [(0, ValueError), (-1, ValueError), (1.0, TypeError), (True, TypeError)]
)
def test_telemetry_config_rejects_bad_periods(field: str, bad: Any, error: type[Exception]) -> None:
    with pytest.raises(error, match=field):
        TelemetryConfig(**{field: bad})


def test_flight_computer_config_holds_every_setting() -> None:
    config = FlightComputerConfig()
    assert config == DEFAULT_FLIGHT_COMPUTER_CONFIG
    assert config.safety == DEFAULT_SAFETY_CONFIG
    assert config.boot == DEFAULT_BOOT_CONFIG
    assert config.telemetry == DEFAULT_TELEMETRY_CONFIG
    assert FlightComputer().config is DEFAULT_FLIGHT_COMPUTER_CONFIG
    custom = FlightComputerConfig(
        safety=SafetyConfig(sustain_tick_count=3),
        boot=BootConfig(duration_us=0),
        telemetry=TelemetryConfig.uniform(TICK_US),
    )
    assert FlightComputer(custom).config is custom


@pytest.mark.parametrize("field", ["safety", "boot", "telemetry"])
def test_flight_computer_config_rejects_other_records(field: str) -> None:
    with pytest.raises(TypeError, match=field):
        FlightComputerConfig(**{field: object()})  # type: ignore[arg-type]


def test_flight_computer_takes_only_a_config_record() -> None:
    with pytest.raises(TypeError, match="FlightComputerConfig"):
        FlightComputer(SafetyConfig())  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        FlightComputer(safety=SafetyConfig())  # type: ignore[call-arg]


def _render_cadence_table() -> list[str]:
    lines = ["| Mode | Period | Ticks at 100 ms |", "|---|---|---|"]
    for mode in Mode:
        period_us = DEFAULT_TELEMETRY_CONFIG.period_us(mode)
        if period_us is None:
            lines.append(f"| {mode.name} | none: the boot-complete beacon only | — |")
        else:
            lines.append(f"| {mode.name} | {period_us / 1e6:.1f} s | {period_us // TICK_US} |")
    return lines


def test_docs_cadence_table_matches_the_defaults() -> None:
    text = DOC.read_text(encoding="utf-8")
    block = text.split("<!-- telemetry-cadence:start -->", 1)[1]
    block = block.split("<!-- telemetry-cadence:end -->", 1)[0]
    lines = [line for line in block.strip().splitlines() if line.startswith("|")]
    assert lines == _render_cadence_table()


# --- The schedule, as a pure function -----------------------------------------------------


def test_boot_is_never_due() -> None:
    schedule = INITIAL_TELEMETRY_SCHEDULE
    for now_us in range(0, 100 * TICK_US, TICK_US):
        due, schedule = telemetry_due(schedule, Mode.BOOT, now_us)
        assert not due
    assert schedule == TelemetrySchedule(Mode.BOOT)


def test_a_mode_change_is_due_at_once_and_restarts_the_period() -> None:
    due, schedule = telemetry_due(TelemetrySchedule(Mode.NOMINAL, 10**9), Mode.SAFE, 7)
    assert due and schedule == TelemetrySchedule(Mode.SAFE, 7 + 500_000)
    due, schedule = telemetry_due(INITIAL_TELEMETRY_SCHEDULE, Mode.NOMINAL, 7)
    assert due and schedule == TelemetrySchedule(Mode.NOMINAL, 7 + 1_000_000)


def test_due_once_the_time_reaches_the_next_due_time() -> None:
    schedule = TelemetrySchedule(Mode.NOMINAL, 1_000_000)
    assert telemetry_due(schedule, Mode.NOMINAL, 999_999) == (False, schedule)
    assert telemetry_due(schedule, Mode.NOMINAL, 1_000_000) == (
        True,
        TelemetrySchedule(Mode.NOMINAL, 2_000_000),
    )


@pytest.mark.parametrize(
    ("period_us", "every_ticks"),
    [(1, 1), (TICK_US, 1), (TICK_US + 1, 2), (250_000, 3), (3 * TICK_US, 3)],
)
def test_a_period_rounds_up_to_whole_ticks(period_us: int, every_ticks: int) -> None:
    driver = in_mode(
        Mode.NOMINAL, FlightComputerConfig(telemetry=TelemetryConfig.uniform(period_us))
    )
    driver.step(ticks=30)
    assert driver.telemetry_ticks() == list(range(0, 30, every_ticks))


# --- Cadence per mode -------------------------------------------------------------------


@pytest.mark.parametrize("mode", list(CADENCE_TICKS), ids=lambda m: m.name)
def test_each_mode_sends_at_its_cadence(mode: Mode) -> None:
    driver = in_mode(mode)
    driver.step(ticks=100)
    assert mode_of(driver.fc) is mode
    assert driver.telemetry_ticks() == list(range(0, 100, CADENCE_TICKS[mode]))
    frames = [t for out in driver.outputs for t in telemetry_frames(out.downlink_frames)]
    assert {t.mode for t in frames} == {mode}
    assert all(len(out.downlink_frames) <= 1 for out in driver.outputs)  # one per slot


def test_the_cadence_is_configurable_per_mode() -> None:
    config = FlightComputerConfig(telemetry=TelemetryConfig(nominal_period_us=3 * TICK_US))
    driver = in_mode(Mode.NOMINAL, config)
    driver.step(ticks=10)
    assert driver.telemetry_ticks() == [0, 3, 6, 9]


def test_boot_sends_nothing_until_the_boot_complete_beacon() -> None:
    driver = Driver(FlightComputer())
    boot_ticks = DEFAULT_BOOT_CONFIG.duration_us // TICK_US
    driver.step(ticks=boot_ticks + 25)
    beacon = boot_ticks - 1  # the step at uptime 5 s raises BOOT_COMPLETE
    assert driver.telemetry_ticks() == [beacon, beacon + 10, beacon + 20]
    [frame] = telemetry_frames(driver.outputs[beacon].downlink_frames)
    assert (frame.mode, frame.uptime_ms, frame.boot_count) == (Mode.NOMINAL, 5000, 0)


def test_a_mode_change_sends_a_frame_in_its_tick_after_the_ack() -> None:
    driver = in_mode(Mode.NOMINAL)
    driver.step(ticks=4)  # frame in step 0; the next is due in step 10
    out = driver.step((command_frame(Command.set_mode(Mode.SCIENCE), sequence=9),))
    assert [decode_frame(f).frame_type for f in out.downlink_frames] == [
        FrameType.ACK,
        FrameType.TELEMETRY,
    ]
    assert decode_ack(decode_frame(out.downlink_frames[0]).payload) == CommandAck(
        9, CommandId.SET_MODE
    )
    # Telemetry from the ACK tick shows the new mode (ADR-0004 §2, docs/protocol.md).
    assert [t.mode for t in telemetry_frames(out.downlink_frames)] == [Mode.SCIENCE]
    # The schedule restarts from the change: SCIENCE's next frame is 1 s later.
    driver.step(ticks=20)
    assert driver.telemetry_ticks() == [0, 4, 14, 24]


def test_a_command_that_changes_nothing_does_not_add_a_frame() -> None:
    driver = in_mode(Mode.NOMINAL)
    driver.step(ticks=4)
    out = driver.step((command_frame(Command.set_mode(Mode.NOMINAL)),))  # ACK, no change
    assert [decode_frame(f).frame_type for f in out.downlink_frames] == [FrameType.ACK]
    driver.step(ticks=10)
    assert driver.telemetry_ticks() == [0, 10]


def test_safe_entry_switches_to_the_safe_cadence() -> None:
    driver = in_mode(Mode.SCIENCE)
    driver.step(ticks=3)
    driver.step((command_frame(Command.enter_safe_mode()),))  # step 3
    driver.step(ticks=12)
    assert mode_of(driver.fc) is Mode.SAFE
    assert driver.telemetry_ticks() == [0, 3, 8, 13]


# --- Content -------------------------------------------------------------------------------


def test_frame_is_built_from_the_readings_and_the_flight_computer_state() -> None:
    driver = in_mode(Mode.SCIENCE)
    out = driver.step()
    [raw] = out.downlink_frames
    state = FlightComputerTelemetryState(uptime_ms=100, mode=Mode.SCIENCE, boot_count=0)
    expected = encode_telemetry(
        state,
        power=READINGS.power,
        thermal=READINGS.thermal,
        attitude=READINGS.attitude,
        payload=READINGS.payload,
        comms=READINGS.comms,
    )
    assert decode_frame(raw) == Frame(FrameType.TELEMETRY, 0, expected)
    assert len(raw) == TELEMETRY_FRAME_SIZE == 36


def test_frames_carry_this_ticks_readings() -> None:
    driver = in_mode(Mode.NOMINAL, EVERY_TICK)
    warm = dataclasses.replace(
        READINGS, thermal=dataclasses.replace(READINGS.thermal, battery_c=31.25)
    )
    [first] = telemetry_frames(driver.step(readings=READINGS).downlink_frames)
    [second] = telemetry_frames(driver.step(readings=warm).downlink_frames)
    assert first.battery_c == READINGS.thermal.battery_c
    assert second.battery_c == 31.25
    assert (first.uptime_ms, second.uptime_ms) == (100, 200)


def test_boot_count_and_uptime_after_a_reset_command() -> None:
    driver = in_mode(Mode.NOMINAL, FlightComputerConfig(boot=BootConfig(duration_us=TICK_US)))
    driver.step(ticks=3)
    out = driver.step((command_frame(Command.reset(), sequence=4),))
    # The RESET tick sends its ACK only: the mode after step d is BOOT.
    assert [decode_frame(f).frame_type for f in out.downlink_frames] == [FrameType.ACK]
    out = driver.step()  # uptime 100 ms: the boot completes, the beacon goes out
    [beacon] = telemetry_frames(out.downlink_frames)
    assert (beacon.mode, beacon.boot_count, beacon.uptime_ms) == (Mode.NOMINAL, 1, 100)
    assert decode_frame(out.downlink_frames[0]).sequence == 0  # the reboot restarted it


def test_a_reboot_between_steps_clears_the_schedule() -> None:
    # forced_reset's path (#60): BOOT again, silent until its beacon.
    driver = in_mode(Mode.SCIENCE, FlightComputerConfig(boot=BootConfig(duration_us=3 * TICK_US)))
    driver.step(ticks=5)
    driver.fc.reboot(now_us=driver.now_us)
    driver.step(ticks=15)
    assert driver.telemetry_ticks(start=5) == [7, 17]
    [beacon] = telemetry_frames(driver.outputs[7].downlink_frames)
    assert (beacon.mode, beacon.boot_count, beacon.uptime_ms) == (Mode.NOMINAL, 1, 300)


def test_a_nan_reading_enters_fault_and_drops_the_frame_without_raising() -> None:
    # NaN has no telemetry encoding (docs/protocol.md); FAULT_DETECTED is raised for it
    # in the same tick (#48). The due frame is dropped and not counted as suppressed.
    bad = dataclasses.replace(READINGS, power=dataclasses.replace(READINGS.power, bus_v=math.nan))
    driver = in_mode(Mode.NOMINAL)
    out = driver.step(readings=bad)
    assert mode_of(driver.fc) is Mode.FAULT
    assert out.downlink_frames == () and out.outbound_suppressed_count == 0
    # Readings sound again: FAULT's cadence carries on from the dropped slot.
    driver.step(ticks=6)
    assert driver.telemetry_ticks() == [5]
    [frame] = telemetry_frames(driver.outputs[5].downlink_frames)
    assert frame.mode is Mode.FAULT


# --- Outbound priority, capacity, and sequence ---------------------------------------------


def test_telemetry_goes_after_every_ack_and_shares_the_sequence_counter() -> None:
    driver = in_mode(Mode.NOMINAL, EVERY_TICK)
    pings = tuple(command_frame(Command.ping(), sequence=100 + n) for n in range(3))
    out = driver.step(pings)
    decoded = [decode_frame(f) for f in out.downlink_frames]
    assert [f.frame_type for f in decoded] == [FrameType.ACK] * 3 + [FrameType.TELEMETRY]
    assert [f.sequence for f in decoded] == [0, 1, 2, 3]
    out = driver.step()
    assert [decode_frame(f).sequence for f in out.downlink_frames] == [4]


def test_the_downlink_sequence_wraps_through_telemetry() -> None:
    driver = in_mode(Mode.NOMINAL, EVERY_TICK)
    driver.fc._downlink_sequence = MAX_SEQUENCE - 1
    driver.step(ticks=4)
    sequences = [decode_frame(out.downlink_frames[0]).sequence for out in driver.outputs]
    assert sequences == [MAX_SEQUENCE - 1, MAX_SEQUENCE, 0, 1]


@pytest.mark.parametrize(
    ("capacity", "acks_sent", "telemetry_sent", "suppressed"),
    [
        (0, 0, False, 3),
        (TELEMETRY_FRAME_SIZE - 1, 2, False, 1),  # 35 B: two 14-byte ACKs, then 7 B left
        (TELEMETRY_FRAME_SIZE, 2, False, 1),  # 36 B: the ACKs come first
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE - 1, 2, False, 1),
        (2 * ACK_FRAME_SIZE + TELEMETRY_FRAME_SIZE, 2, True, 0),
        (CAPACITY, 2, True, 0),
    ],
)
def test_telemetry_fits_the_capacity_left_after_the_acks(
    capacity: int, acks_sent: int, telemetry_sent: bool, suppressed: int
) -> None:
    driver = in_mode(Mode.NOMINAL)
    pings = (command_frame(Command.ping(), 1), command_frame(Command.ping(), 2))
    out = driver.step(pings, readings=with_capacity(capacity))
    types = [decode_frame(f).frame_type for f in out.downlink_frames]
    assert types == [FrameType.ACK] * acks_sent + [FrameType.TELEMETRY] * telemetry_sent
    assert out.outbound_suppressed_count == suppressed
    assert out.sent_bytes <= capacity


def test_suppressed_telemetry_is_counted_not_queued_and_the_cadence_resumes() -> None:
    # Capacity 0 (transmitter_off, #60) over the slots of steps 10 and 20; it returns
    # in step 21. Each suppressed frame is counted once and never sent later; the next
    # frame is in the slot after the last suppressed one (step 30), not at once.
    driver = in_mode(Mode.NOMINAL)
    driver.step(ticks=5)
    driver.step(readings=with_capacity(0), ticks=16)  # steps 5-20
    driver.step(ticks=15)  # steps 21-35
    counts = [out.outbound_suppressed_count for out in driver.outputs]
    assert [k for k, count in enumerate(counts) if count] == [10, 20]
    assert sum(counts) == 2
    assert driver.telemetry_ticks() == [0, 30]
    [frame] = telemetry_frames(driver.outputs[30].downlink_frames)
    assert frame.uptime_ms == 3100
    # Suppressed frames took no sequence number.
    assert decode_frame(driver.outputs[30].downlink_frames[0]).sequence == 1


def test_a_mode_change_while_the_transmitter_is_off_is_suppressed_once() -> None:
    driver = in_mode(Mode.NOMINAL)
    driver.step(ticks=2)
    out = driver.step((command_frame(Command.enter_safe_mode()),), with_capacity(0))
    assert out.outbound_suppressed_count == 2  # the ACK and SAFE's first frame
    driver.step(ticks=6)
    assert driver.telemetry_ticks() == [0, 7]  # SAFE's next slot, 0.5 s later


# --- Determinism --------------------------------------------------------------------------


def _history(seed: int) -> list[tuple[Any, ...]]:
    rng = random.Random(seed)
    commands = [
        Command.ping(),
        Command.set_mode(Mode.SCIENCE),
        Command.set_mode(Mode.NOMINAL),
        Command.begin_downlink(),
        Command.enter_safe_mode(),
        Command.reset(),
    ]
    driver = Driver(FlightComputer(FlightComputerConfig(boot=BootConfig(duration_us=TICK_US))))
    history: list[tuple[Any, ...]] = []
    for n in range(400):
        frames = (command_frame(rng.choice(commands), n),) if rng.random() < 0.05 else ()
        capacity = 0 if rng.random() < 0.1 else CAPACITY
        out = driver.step(frames, with_capacity(capacity))
        history.append((out.downlink_frames, out.outbound_suppressed_count, driver.fc.mode))
    return history


def test_telemetry_is_deterministic() -> None:
    first = _history(55)
    assert first == _history(55)
    assert first != _history(56)
    assert any(telemetry_frames(frames) for frames, _, _ in first)


def test_emit_phase_uses_the_codec_through_the_module() -> None:
    # The import rule (pocketsat.flight docstring, #101): computer.py reads the codec's
    # names from the module at call time, never with a from-import.
    source = (Path(__file__).resolve().parents[2] / "src/pocketsat/flight/computer.py").read_text()
    assert "from pocketsat import messages" in source
    assert "from pocketsat.messages import" not in source
    assert "messages.encode_telemetry(" in source
    assert messages.encode_telemetry is encode_telemetry
