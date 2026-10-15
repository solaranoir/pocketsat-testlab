"""The mission console over a short pass against ``SilTarget`` (#165).

The station, the ideal link (#153) and ``SilTarget`` with the real flight computer run in
the run loop's step order (``docs/architecture.md`` §8), as in story #146's test, with
pass geometry from ``OrbitGeometry.with_pass`` (a 20° pass peaking at 2.5 minutes). The
station commands the spacecraft: PING, SET_MODE SCIENCE, two commands the spacecraft
NACKs, BEGIN_DOWNLINK in the pass, then RESET and a command during the reboot. Every
event goes through a :class:`MissionConsole`.

Two runs with the same seed must give byte-identical text and JSON lines (ADR-0003), and
the output must cover AOS, LOS, command outcomes, telemetry, DATA and the reboot.
"""

import io
import json
from dataclasses import dataclass

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.environment.orbit import OrbitGeometry
from pocketsat.flight import Mode
from pocketsat.groundstation import GroundStation, StationEvent
from pocketsat.groundstation.console import MissionConsole
from pocketsat.link import IdealLink
from pocketsat.messages import Command
from pocketsat.targets.sil import SilTarget

SEED = 165
TICK_US = 100_000
PASS_PEAK_US = 150_000_000
PASS_MAX_ELEVATION_DEG = 20.0
RUN_TICKS = 3_000
"""5 minutes: past LOS."""

SCRIPT: dict[int, tuple[Command, ...]] = {
    10: (Command.ping(),),
    60: (Command.set_mode(Mode.SCIENCE),),  # BOOT lasts 5 s (#49)
    720: (Command.set_mode(Mode.SAFE), Command(0x7F)),  # NACKed: not commandable, unknown
    2_000: (Command.begin_downlink(),),
    2_200: (Command.reset(),),
    2_220: (Command.set_mode(Mode.SCIENCE),),  # NACKed: still in BOOT
}
"""Commands submitted at the start of each tick."""


@dataclass(frozen=True)
class ConsoleRun:
    """What one run printed, and the events it printed them from."""

    text: str
    log: str
    events: tuple[StationEvent, ...]


def _run(seed: int) -> ConsoleRun:
    geometry = OrbitGeometry.with_pass(PASS_MAX_ELEVATION_DEG, PASS_PEAK_US)
    text, log = io.StringIO(), io.StringIO()
    console = MissionConsole(text, log)
    events: list[StationEvent] = []
    station, link, target = GroundStation(), IdealLink(), SilTarget()
    rngs = RngFactory(seed)
    station.reset(rngs)
    link.reset(rngs)
    target.connect()
    target.reset(seed)
    for tick in range(RUN_TICKS):
        start_us, end_us = tick * TICK_US, (tick + 1) * TICK_US
        sample = geometry.geometry_at(start_us)
        for command in SCRIPT.get(tick, ()):
            station.submit(command)
        uplink = station.begin_tick(start_us, sample)  # steps 3 and 4
        for frame in link.carry_uplink(uplink.packets, start_us, sample):
            target.send(frame)
        target.advance(TICK_US)  # step 6
        downlink = link.carry_downlink(target.receive(), end_us, sample)  # steps 7 and 8
        tick_events = (*uplink.events, *station.end_tick(end_us, downlink))  # step 9
        console.emit_all(tick_events)
        events.extend(tick_events)
    target.close()
    return ConsoleRun(text.getvalue(), log.getvalue(), tuple(events))


@pytest.fixture(scope="module")
def runs() -> tuple[ConsoleRun, ConsoleRun]:
    return _run(SEED), _run(SEED)


def test_same_seed_gives_byte_identical_console_output(
    runs: tuple[ConsoleRun, ConsoleRun],
) -> None:
    first, second = runs
    assert first.text.encode() == second.text.encode()
    assert first.log.encode() == second.log.encode()


def test_one_text_line_and_one_json_line_per_event(runs: tuple[ConsoleRun, ConsoleRun]) -> None:
    run = runs[0]
    lines, records = run.text.splitlines(), run.log.splitlines()
    assert len(lines) == len(records) == len(run.events) > 0
    assert run.text.endswith("\n") and run.log.endswith("\n")
    for event, record in zip(run.events, map(json.loads, records), strict=True):
        assert record["event"] == type(event).__name__
        assert record["time_us"] == event.time_us


def test_output_is_a_function_of_the_events_alone(runs: tuple[ConsoleRun, ConsoleRun]) -> None:
    run = runs[0]
    text, log = io.StringIO(), io.StringIO()
    MissionConsole(text, log).emit_all(run.events)
    assert (text.getvalue(), log.getvalue()) == (run.text, run.log)


def test_the_pass_is_on_the_console(runs: tuple[ConsoleRun, ConsoleRun]) -> None:
    lines = runs[0].text.splitlines()
    tags = {line.split()[1] for line in lines}
    assert {"AOS", "LOS", "CMD", "TLM", "DATA", "REBOOT"} <= tags
    details = [line[21:] for line in lines]
    for expected in (
        "#0 PING ACK seq",
        "#1 SET_MODE ACK seq",
        "#2 SET_MODE NACK TARGET_NOT_COMMANDABLE seq",
        "#3 0x7F NACK UNKNOWN_COMMAND seq",
        "#4 BEGIN_DOWNLINK ACK seq",
        "#5 RESET ACK seq",
        "#6 SET_MODE NACK BOOT_IN_PROGRESS seq",
        "pass 1 az",
        "boot 1 seq",
    ):
        assert any(detail.startswith(expected) for detail in details), expected
    assert any(" SCIENCE " in line and " ACQUIRING " in line for line in lines)
    data = [line for line in lines if line.split()[1] == "DATA"]
    assert data[-1].endswith(f", {len(data)} chunks stored")
