"""Golden telemetry captures: capture, compare, and deliberate regeneration (#64).

A golden capture is every downlink frame a scenario produces for one seed, byte for
byte, stored as text under ``tests/golden/``. ``test_golden_telemetry.py`` regenerates
each capture from its recorded seed and compares the bytes; the first difference is
reported with its tick, its frame index, and both frames' decoded fields.

**Format** (``FORMAT_VERSION`` 1, UTF-8, ``\\n`` line endings): a header of
``# key: value`` lines, then one line per downlink frame, in the order ``receive()``
returned them::

    <tick> <time_s> <TYPE> <frame hex>

``tick`` counts from 0 (the first ``advance()`` after ``reset(seed)``); ``time_s`` is
the simulated time at the **end** of that tick, the instant the flight computer's
readings describe (``SilTarget``); ``TYPE`` is the frame type from the header byte,
for reading only; the hex is the whole wire frame (sync to CRC, ``docs/protocol.md``).
The header records what the bytes depend on: the scenario and its seed, the tick, the
orbit, the settings by name, and the protocol version, plus the frame count and
``frames_sha256``, a digest of the body (each frame as its tick, uint32 big-endian,
its length, uint16 big-endian, and its bytes, as in ``test_behavior_digests.py``).

**Scenarios.** :data:`SCENARIOS` maps each capture to the function that produces its
frames, ``seed -> [(tick, frame), ...]`` (:data:`ScenarioRun`). The nominal capture's
function runs #63's nominal scenario harness, ``_nominal_scenario.run_nominal_scenario``
(boot, PING, SET_MODE SCIENCE, one 10-minute DOWNLINK pass at the end of the orbit,
SET_MODE NOMINAL; seed 42), and :func:`sensor_freeze_run` reuses #63's command times.

**Regenerating** is deliberate and reviewed (``tests/golden/README.md``)::

    uv run python tests/sil/_golden.py --check          # report differences only
    uv run python tests/sil/_golden.py --regenerate     # rewrite every capture
    uv run python tests/sil/_golden.py --regenerate nominal_seed42
"""

import argparse
import hashlib
import sys
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass, field
from enum import Enum, Flag
from pathlib import Path
from typing import Final

from _nominal_scenario import (
    ENVIRONMENT,
    PASS_END_US,
    PASS_START_US,
    PING_US,
    RUN_US,
    run_nominal_scenario,
)
from _reference_profile import SCIENCE_COMMAND_US

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import Mode
from pocketsat.frame import (
    PROTOCOL_VERSION,
    Frame,
    FrameError,
    FrameType,
    decode_frame,
    encode_frame,
)
from pocketsat.messages import Command, decode_ack, decode_data, decode_telemetry, encode_command
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import SENSOR_FREEZE, SilTarget

GOLDEN_DIR: Final = Path(__file__).resolve().parents[1] / "golden"
"""``tests/golden/``."""

FORMAT_VERSION: Final = 1
"""The capture file format above. A change to it is a deliberate regeneration."""

TICK_US: Final = DEFAULT_TICK_US
US_PER_S: Final = 1_000_000

DownlinkFrame = tuple[int, bytes]
"""One downlink frame and the tick ``receive()`` returned it after."""

ScenarioRun = Callable[[int], Sequence[DownlinkFrame]]
"""A scenario: seed in, every downlink frame in ``receive()`` order out."""


# --- The scenarios -----------------------------------------------------------------------

Script = Callable[[int], tuple[list[Command], list[TargetFault]]]
"""The commands to send and the faults to inject before tick ``n``."""


def _drive(
    seed: int, ticks: int, script: Script, environment: NominalEnvironment
) -> list[DownlinkFrame]:
    """Run ``script`` on a ``SilTarget`` through ``TestTarget`` only, in ADR-0003 step
    order with frame loopback: encode the commands, inject the faults, send, apply the
    environment sampled at the orchestrator's ``SimClock``, advance one tick, receive."""
    target = SilTarget()
    target.connect()
    target.reset(seed)
    clock = SimClock(TICK_US)
    sequence = 0
    downlink: list[DownlinkFrame] = []
    for n in range(ticks):
        commands, faults = script(n)
        for fault in faults:
            target.inject(fault)
        for command in commands:
            sequence += 1
            target.send(encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command))))
        target.apply_environment(environment.state_at(clock.now_us))
        target.advance(TICK_US)
        clock.advance_one_tick()
        downlink.extend((n, frame) for frame in target.receive())
    target.close()
    return downlink


NOMINAL_ENVIRONMENT: Final = ENVIRONMENT
"""#63's orbit: ``NominalEnvironment()``, 92 minutes, 35% eclipse at the end."""

# #63's timeline, as tick indices (the commands are sent before these ticks).
PING_TICK: Final = PING_US // TICK_US
SCIENCE_TICK: Final = SCIENCE_COMMAND_US // TICK_US
PASS_START_TICK: Final = PASS_START_US // TICK_US
NOMINAL_TICK: Final = PASS_END_US // TICK_US
NOMINAL_TICKS: Final = RUN_US // TICK_US
"""One full orbit, then 2 s in NOMINAL: 55,220 ticks."""


def nominal_run(seed: int) -> list[DownlinkFrame]:
    """#63's nominal scenario (``run_nominal_scenario``), as ``(tick, frame)`` pairs."""
    return [(t.tick, f) for t in run_nominal_scenario(seed).downlink for f in t.frames]


FREEZE_START_TICK: Final = 2_000
"""200.0 s, in SCIENCE with the payload acquiring (from about 158 s, once attitude
has stabilised): ``sensor_freeze`` on every sensor for :data:`FREEZE_US`."""

FREEZE_US: Final = 20 * US_PER_S
FREEZE_TICKS: Final = 2_300
"""230 s: the freeze ends at 220 s, then 10 s of live readings."""


def _freeze_script(n: int) -> tuple[list[Command], list[TargetFault]]:
    if n == PING_TICK:
        return [Command.ping()], []
    if n == SCIENCE_TICK:
        return [Command.set_mode(Mode.SCIENCE)], []
    if n == FREEZE_START_TICK:
        return [], [TargetFault(SENSOR_FREEZE, {}, duration_us=FREEZE_US)]
    return [], []


def sensor_freeze_run(seed: int) -> list[DownlinkFrame]:
    """The nominal start (PING, SCIENCE), then ``sensor_freeze`` on power, thermal and
    attitude from 200 s for 20 s (ADR-0004 §8): the reported values hold while the
    payload buffer, which is not a sensor, keeps filling. 2,300 ticks."""
    return _drive(seed, FREEZE_TICKS, _freeze_script, NOMINAL_ENVIRONMENT)


@dataclass(frozen=True)
class GoldenScenario:
    """One golden capture: the scenario that produces it, and what its header records.

    Attributes:
        name: The file stem under ``tests/golden/``.
        description: One line for the header.
        seed: The recorded master seed.
        ticks: How many ticks the scenario runs.
        run: Seed in, downlink frames out.
        slow: Whether regenerating takes more than about a second (``pytest -m slow``).
    """

    name: str
    description: str
    seed: int
    ticks: int
    run: ScenarioRun
    slow: bool

    @property
    def path(self) -> Path:
        """The capture file."""
        return GOLDEN_DIR / f"{self.name}.golden"

    def header(self, frames: Sequence[DownlinkFrame]) -> dict[str, str]:
        """The header for a capture of ``frames``, in file order."""
        return {
            "pocketsat golden telemetry capture": "#64, see tests/golden/README.md",
            "format": str(FORMAT_VERSION),
            "scenario": self.name,
            "description": self.description,
            "seed": str(self.seed),
            "tick_us": str(TICK_US),
            "ticks": str(self.ticks),
            "orbit_period_us": str(NOMINAL_ENVIRONMENT.orbit_period_us),
            "environment": "NominalEnvironment()",
            "config": "NOMINAL_CONFIG",
            "initial_state": "DEFAULT_INITIAL_STATE",
            "flight_computer": "DEFAULT_FLIGHT_COMPUTER_CONFIG",
            "protocol_version": str(PROTOCOL_VERSION),
            "frames": str(len(frames)),
            "frames_sha256": frames_sha256(frames),
            "columns": "tick time_s type frame_hex",
        }


SCENARIOS: Final = {
    s.name: s
    for s in (
        GoldenScenario(
            name="nominal_seed42",
            description=(
                "#63 nominal scenario: PING, SCIENCE, one 10-minute DOWNLINK pass at the "
                "end of a 92-minute orbit, NOMINAL"
            ),
            seed=42,
            ticks=NOMINAL_TICKS,
            run=nominal_run,
            slow=True,
        ),
        GoldenScenario(
            name="sensor_freeze_seed42",
            description="PING, SCIENCE, then sensor_freeze on every sensor at 200 s for 20 s",
            seed=42,
            ticks=FREEZE_TICKS,
            run=sensor_freeze_run,
            slow=False,
        ),
    )
}


# --- Writing and reading -----------------------------------------------------------------


def frames_sha256(frames: Sequence[DownlinkFrame]) -> str:
    """SHA-256 of each frame's tick (uint32 BE), length (uint16 BE), and bytes."""
    digest = hashlib.sha256()
    for tick, frame in frames:
        digest.update(tick.to_bytes(4, "big") + len(frame).to_bytes(2, "big") + frame)
    return digest.hexdigest()


def _time_s(tick: int) -> str:
    end_us = (tick + 1) * TICK_US
    return f"{end_us // US_PER_S}.{end_us % US_PER_S // 1000:03d}"


def _type_name(frame: bytes) -> str:
    try:
        return FrameType(frame[3]).name
    except (IndexError, ValueError):
        return "?"


def render(scenario: GoldenScenario, frames: Sequence[DownlinkFrame]) -> str:
    """The capture file's text for ``frames``."""
    lines = [f"# {key}: {value}" for key, value in scenario.header(frames).items()]
    lines.extend(
        f"{tick} {_time_s(tick)} {_type_name(frame)} {frame.hex()}" for tick, frame in frames
    )
    return "\n".join(lines) + "\n"


@dataclass(frozen=True)
class Capture:
    """A capture file, parsed."""

    header: dict[str, str]
    frames: list[DownlinkFrame] = field(default_factory=list)


class GoldenFormatError(ValueError):
    """A capture file does not follow the format."""


def parse(text: str) -> Capture:
    """Parse a capture file's text, checking each line's time and type label against
    its tick and frame.

    Raises:
        GoldenFormatError: A line is malformed or inconsistent.
    """
    header: dict[str, str] = {}
    frames: list[DownlinkFrame] = []
    for number, line in enumerate(text.splitlines(), start=1):
        if line.startswith("# "):
            if frames:
                raise GoldenFormatError(f"line {number}: header line after the frames")
            key, sep, value = line[2:].partition(": ")
            if not sep:
                raise GoldenFormatError(f"line {number}: header line without ': '")
            header[key] = value
            continue
        parts = line.split(" ")
        if len(parts) != 4:
            raise GoldenFormatError(f"line {number}: expected 4 fields, got {len(parts)}")
        tick_text, time_text, type_text, hex_text = parts
        try:
            tick, frame = int(tick_text), bytes.fromhex(hex_text)
        except ValueError as error:
            raise GoldenFormatError(f"line {number}: {error}") from None
        if time_text != _time_s(tick) or type_text != _type_name(frame):
            raise GoldenFormatError(f"line {number}: time or type label does not match")
        frames.append((tick, frame))
    return Capture(header, frames)


def load(scenario: GoldenScenario) -> Capture:
    """The stored capture for ``scenario``."""
    return parse(scenario.path.read_text(encoding="utf-8"))


# --- Comparing ---------------------------------------------------------------------------


def _scalar(value: object) -> object:
    if isinstance(value, Flag):
        return "|".join(str(member.name) for member in value) or "none"
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, bytes):
        return value.hex()
    return value


def decoded_fields(frame: bytes) -> dict[str, object]:
    """A frame's header fields and decoded payload fields, for reports.

    Never raises: what does not decode is reported as an error string.
    """
    try:
        decoded = decode_frame(frame)
    except FrameError as error:
        return {"frame_error": f"{type(error).__name__}: {error}"}
    fields: dict[str, object] = {
        "version": decoded.version,
        "type": decoded.frame_type.name,
        "sequence": decoded.sequence,
        "length": len(decoded.payload),
    }
    payload: object
    try:
        if decoded.frame_type is FrameType.TELEMETRY:
            payload = decode_telemetry(decoded.payload)
        elif decoded.frame_type is FrameType.ACK:
            payload = decode_ack(decoded.payload)
        elif decoded.frame_type is FrameType.DATA:
            payload = decode_data(decoded.payload)
        else:
            return fields | {"payload": decoded.payload.hex()}
    except ValueError as error:
        return fields | {"payload_error": f"{type(error).__name__}: {error}"}
    for key, value in asdict(payload).items():
        fields[key] = _scalar(value)
    return fields


def _describe(label: str, item: DownlinkFrame) -> list[str]:
    tick, frame = item
    return [f"  {label}: tick {tick} (t={_time_s(tick)} s) {frame.hex()}"]


def first_difference(
    expected: Sequence[DownlinkFrame], actual: Sequence[DownlinkFrame]
) -> str | None:
    """A report of the first frame where ``actual`` departs from ``expected``, or
    ``None`` when they are identical (ticks and bytes).

    The report names the frame index, the tick and time, the differing byte offsets,
    and every decoded field of both frames, marking the fields that differ.
    """
    for index, (want, got) in enumerate(zip(expected, actual, strict=False)):
        if want == got:
            continue
        lines = [f"first difference at frame {index} (0-based, in receive() order):"]
        lines += _describe("expected", want) + _describe("actual  ", got)
        if want[0] != got[0]:
            lines.append(f"  tick differs: expected {want[0]}, actual {got[0]}")
        offsets = [
            i
            for i in range(max(len(want[1]), len(got[1])))
            if want[1][i : i + 1] != got[1][i : i + 1]
        ]
        if offsets:
            shown = ", ".join(str(i) for i in offsets[:16])
            more = f" (+{len(offsets) - 16} more)" if len(offsets) > 16 else ""
            lines.append(f"  differing byte offsets: {shown}{more}")
        want_fields, got_fields = decoded_fields(want[1]), decoded_fields(got[1])
        lines.append("  decoded fields (expected -> actual):")
        for key in dict.fromkeys([*want_fields, *got_fields]):
            a, b = want_fields.get(key, "<absent>"), got_fields.get(key, "<absent>")
            mark = "*" if a != b else " "
            lines.append(f"  {mark} {key}: {a!r}" + (f" -> {b!r}" if a != b else ""))
        return "\n".join(lines)
    if len(expected) == len(actual):
        return None
    index = min(len(expected), len(actual))
    if len(expected) > len(actual):
        lines = [f"actual ends after {index} frames; expected {len(expected)}. First missing:"]
        lines += _describe("expected", expected[index])
    else:
        lines = [f"actual has {len(actual)} frames; expected {index}. First extra:"]
        lines += _describe("actual  ", actual[index])
    return "\n".join(lines)


def check(scenario: GoldenScenario) -> str | None:
    """Regenerate ``scenario`` from its recorded seed and compare with the stored
    capture: ``None`` if identical (frames, then header), otherwise a report."""
    stored = load(scenario)
    if stored.header.get("seed") != str(scenario.seed):
        return f"{scenario.name}: stored seed {stored.header.get('seed')}, scenario {scenario.seed}"
    frames = list(scenario.run(scenario.seed))
    report = first_difference(stored.frames, frames)
    if report is not None:
        return f"{scenario.name} differs from {scenario.path}\n{report}"
    header = scenario.header(frames)
    if stored.header != header:
        keys = [
            k
            for k in dict.fromkeys([*stored.header, *header])
            if stored.header.get(k) != header.get(k)
        ]
        diffs = "\n".join(f"  {k}: {stored.header.get(k)!r} -> {header.get(k)!r}" for k in keys)
        return f"{scenario.name}: frames identical, header differs:\n{diffs}"
    return None


def regenerate(scenario: GoldenScenario) -> Path:
    """Rewrite ``scenario``'s capture from its recorded seed. Deliberate only: see
    ``tests/golden/README.md`` for when this is allowed."""
    frames = list(scenario.run(scenario.seed))
    GOLDEN_DIR.mkdir(exist_ok=True)
    with scenario.path.open("w", encoding="utf-8", newline="\n") as file:
        file.write(render(scenario, frames))
    return scenario.path


def main(argv: Sequence[str] | None = None) -> int:
    """Check or regenerate the golden captures. Exit status 1 if any differs (check)."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0] if __doc__ else None)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check", action="store_true", help="compare, write nothing")
    action.add_argument(
        "--regenerate",
        action="store_true",
        help="rewrite the captures (deliberate; see tests/golden/README.md)",
    )
    parser.add_argument("names", nargs="*", help=f"default: all of {', '.join(SCENARIOS)}")
    args = parser.parse_args(argv)
    names: list[str] = args.names or list(SCENARIOS)
    unknown = [name for name in names if name not in SCENARIOS]
    if unknown:
        parser.error(f"unknown capture(s): {', '.join(unknown)}")
    status = 0
    for name in names:
        scenario = SCENARIOS[name]
        if args.regenerate:
            path = regenerate(scenario)
            print(f"wrote {path} ({path.stat().st_size} bytes)")
        else:
            report = check(scenario)
            print(report if report else f"{name}: identical")
            status |= report is not None
    return status


if __name__ == "__main__":
    sys.exit(main())
