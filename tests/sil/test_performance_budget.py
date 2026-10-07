"""Performance budget benchmark (#78, two budgets since #120).

Times the full stack (five real subsystems, the flight computer, and ``SilTarget``) and
each real subsystem against the shared fakes (#76), and prints microseconds per tick in
the CI log on every run as ``[perf #78]`` lines. The budgets and how they were set are
in ``docs/spacecraft.md`` ("Performance budget (#78)").

- **Orbit average:** at most :data:`ORBIT_AVERAGE_BUDGET_US` per tick on CI Linux, over
  #72's reference profile (SCIENCE plus one 10-minute DOWNLINK pass per orbit). It
  drives CI time and #63's under-10-seconds-per-orbit target. Measured over a
  :data:`ORBIT_SCALE`-times shorter orbit with the same structure
  (:func:`_time_scaled_orbit`); the slow test
  :func:`test_a_full_reference_orbit_matches_the_scaled_orbit` measures a whole orbit
  and prints both, so CI shows that the shortened mix is representative.
- **Worst-case tick:** at most :data:`WORST_CASE_BUDGET_US` on CI Linux: the busiest
  tick the flight software produces. :data:`WORST_CASE_SCENARIOS` lists the scenarios
  that define it, today one: a DOWNLINK pass at full capacity with ACKs and telemetry
  (:func:`_time_worst_case`). Each tick is timed on its own and grouped by the frames it
  sent; the worst case is the costliest group. **A feature that makes a busier tick adds
  a scenario there.**
- **Per subsystem:** ``step()`` plus ``snapshot()`` within about
  :data:`SUBSYSTEM_GUIDANCE_US`. Guidance only: printed, never a failure.

**In CI** (the ``CI`` environment variable is ``true``, which GitHub Actions sets) a
full-stack test fails above :data:`FAIL_FACTOR` times its budget (200 us average, 300 us
worst case), so it catches real slowdowns without failing on a noisy shared runner.
**Locally** it only prints the numbers, because a busy developer machine can be several
times slower (one run at load average ~15 measured 392 us per tick, against 57 to 62
unloaded).

Each measurement is the median of :data:`REPEATS` runs, each a fresh ``reset(seed)``.
The loop is the orchestrator's: send any command, sample ``NominalEnvironment``,
``apply_environment``, ``advance`` one tick, drain ``receive()``. The simulation is
deterministic, so every run sends the same frames, and the tests check that the timed
ticks are the scenario they claim to be. Wall-clock timing is allowed here: the
determinism guard covers only simulation code in ``src/pocketsat`` (ADR-0003).
"""

import os
import statistics
import time
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Final

import pytest

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import FlightComputer, Mode, controls_for_mode
from pocketsat.frame import MIN_FRAME_SIZE, Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    DATA_CHUNK_ID_SIZE,
    Command,
    decode_ack,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    STEP_ORDER,
    Attitude,
    AttitudeInitial,
    Comms,
    Payload,
    PayloadInitial,
    Power,
    SnapshotBoard,
    SpacecraftInitialState,
    Subsystem,
    SubsystemStack,
    Thermal,
)
from pocketsat.spacecraft.fakes import fake_subsystems
from pocketsat.targets.base import EnvironmentState
from pocketsat.targets.sil import SilTarget

ORBIT_AVERAGE_BUDGET_US: Final = 100.0
"""Full stack, mean per tick over #72's reference profile, on CI Linux (#120)."""

WORST_CASE_BUDGET_US: Final = 150.0
"""Full stack, the busiest tick (:data:`WORST_CASE_SCENARIOS`), on CI Linux (#120)."""

SUBSYSTEM_GUIDANCE_US: Final = 15.0
"""One subsystem's ``step()`` plus ``snapshot()``, per tick, on CI (guidance, #78)."""

FAIL_FACTOR: Final = 2.0
"""In CI, a full-stack measurement fails only above this multiple of its budget."""

REPEATS: Final = 5
SUBSYSTEM_TICKS: Final = 2_000
BOOT_TICKS: Final = 60
"""Untimed ticks before each full-stack run: BOOT lasts 50 ticks (#49)."""

TICK_US: Final = DEFAULT_TICK_US
ORBIT_TICKS: Final = NominalEnvironment().orbit_period_us // TICK_US
"""One default 92-minute ``NominalEnvironment`` orbit at the 100 ms tick: 55,200."""

PASS_TICKS: Final = 10 * 60 * 10
"""#72's DOWNLINK pass: the last 10 minutes of each orbit (the end of eclipse)."""

ORBIT_SCALE: Final = 20
"""The orbit-average benchmark runs a reference orbit this many times shorter: 2,760
ticks, 2,460 of SCIENCE then a 300-tick pass window (see :func:`_time_scaled_orbit`)."""

PING_PERIOD_TICKS: Final = 100
"""The orbit-average run sends a PING every 10 s, as the ground might."""

SEED: Final = 78

STEADY_ATTITUDE: Final = AttitudeInitial(pointing_error_deg=0.0, rate_dps=0.0)
"""Start pointed and still, so the payload acquires from the first SCIENCE tick, as it
does for the rest of the mission once detumbled (#41). The default start detumbles for
about 150 s, during which the payload does not acquire and a tick is cheaper."""

STEADY_STATE: Final = SpacecraftInitialState(attitude=STEADY_ATTITUDE)
"""The orbit average's starting state: detumbled, empty buffer."""


def gate_enforced(environ: Mapping[str, str] = os.environ) -> bool:
    """Whether the 2x-budget check fails the test: only in CI (``CI=true``)."""
    return environ.get("CI", "").lower() == "true"


def _mode() -> str:
    return "CI: fails above 2x budget" if gate_enforced() else "local: report only"


def _report(lines: list[str], capsys: pytest.CaptureFixture[str]) -> None:
    """Print to the terminal (and so the CI log) even though pytest captures output."""
    with capsys.disabled():
        print("".join(f"\n[perf #78] {line}" for line in lines))


def _check_budget(
    what: str, us_per_tick: float, budget_us: float, capsys: pytest.CaptureFixture[str]
) -> None:
    """Fail above :data:`FAIL_FACTOR` times ``budget_us``, in CI only (see the module
    docstring); locally, note that the figure would fail in CI."""
    limit_us = FAIL_FACTOR * budget_us
    if gate_enforced():
        assert us_per_tick <= limit_us, (
            f"{what} takes {us_per_tick:.1f} us per tick, over twice the "
            f"{budget_us:.0f} us budget (#78, #120)"
        )
    elif us_per_tick > limit_us:
        _report([f"over {limit_us:.0f} us: would fail in CI (is this machine busy?)"], capsys)


def _command_frame(command: Command, sequence: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


def _booted(initial: SpacecraftInitialState, command: Command) -> SilTarget:
    """A reset ``SilTarget`` with the real flight computer, booted (untimed), with
    ``command`` sent in the last boot tick, so it acts from the first timed tick."""
    target = SilTarget(initial=initial, flight_computer_factory=FlightComputer)
    target.connect()
    target.reset(SEED)
    environment = NominalEnvironment().state_at(0)
    for n in range(BOOT_TICKS):
        if n == BOOT_TICKS - 1:
            target.send(_command_frame(command, sequence=1))
        target.apply_environment(environment)
        target.advance(target.tick_us)
        target.receive()
    return target


@dataclass(frozen=True)
class TimedRun:
    """One timed run: each tick's wall-clock time and the frames it sent."""

    tick_s: list[float]
    """Seconds per timed tick, in order."""
    frames: list[list[Frame]]
    """The decoded frames ``receive()`` returned after each timed tick (decoded after
    the clock stopped)."""

    def us_per_tick(self, start: int = 0, stop: int | None = None) -> float:
        """Mean microseconds per tick over ``tick_s[start:stop]``."""
        ticks = self.tick_s[start:stop]
        return sum(ticks) / len(ticks) * 1e6

    def frame_types(self, tick: int) -> tuple[FrameType, ...]:
        """The frame types sent in timed tick ``tick``, in transmit order."""
        return tuple(frame.frame_type for frame in self.frames[tick])


def _timed_ticks(
    target: SilTarget,
    ticks: int,
    uplink: Mapping[int, bytes],
    environment_at: Callable[[int], EnvironmentState],
) -> TimedRun:
    """Time ``ticks`` ticks of the orchestrator's loop, each on its own: send
    ``uplink[k]`` if any, apply ``environment_at(k)``, advance one tick, drain the
    downlink. ``perf_counter`` adds well under 1 us per tick."""
    tick_us = target.tick_us
    clock = time.perf_counter
    tick_s: list[float] = []
    raw: list[list[bytes]] = []
    for k in range(ticks):
        started = clock()
        frame = uplink.get(k)
        if frame is not None:
            target.send(frame)
        target.apply_environment(environment_at(k))
        target.advance(tick_us)
        sent = target.receive()
        tick_s.append(clock() - started)
        raw.append(sent)
    return TimedRun(tick_s, [[decode_frame(f) for f in tick] for tick in raw])


# --- Orbit average --------------------------------------------------------------------


@dataclass(frozen=True)
class OrbitProfile:
    """#72's reference profile, ``scale`` times shorter in time."""

    scale: int

    @property
    def ticks(self) -> int:
        """Timed ticks: one orbit."""
        return ORBIT_TICKS // self.scale

    @property
    def pass_start(self) -> int:
        """The timed tick that sends BEGIN_DOWNLINK: the pass window is the rest."""
        return self.ticks - PASS_TICKS // self.scale


SCALED_ORBIT: Final = OrbitProfile(ORBIT_SCALE)
FULL_ORBIT: Final = OrbitProfile(1)


def _time_orbit(profile: OrbitProfile) -> TimedRun:
    """One orbit of #72's reference profile, ``profile.scale`` times shorter.

    Boot (untimed) into SCIENCE from :data:`STEADY_STATE`, then time one orbit: SCIENCE
    with a PING every 10 s, BEGIN_DOWNLINK at the start of the pass window (the last 10
    minutes, scaled), and the flight computer sends every stored chunk and returns to
    SCIENCE with ``DOWNLINK_COMPLETE`` (#56) inside the window, as in the full orbit.

    The environment runs ``scale`` times faster than simulated time, so the shortened
    orbit still sees the whole sunlight/eclipse cycle, with the pass at the end of
    eclipse. Data is produced in every SCIENCE and DOWNLINK tick and sent in the pass at
    one chunk per tick, so DATA ticks take the same share of the shortened orbit as of a
    full one (see :func:`_check_orbit_mix`).
    """
    target = _booted(STEADY_STATE, Command.set_mode(Mode.SCIENCE))
    uplink = {
        k: _command_frame(Command.ping(), sequence=2 + k)
        for k in range(PING_PERIOD_TICKS // 2, profile.ticks, PING_PERIOD_TICKS)
    }
    uplink[profile.pass_start] = _command_frame(Command.begin_downlink(), sequence=1)
    environment = NominalEnvironment()
    start_us, scale = target.now_us, profile.scale
    return _timed_ticks(
        target,
        profile.ticks,
        uplink,
        lambda k: environment.state_at(scale * (target.now_us - start_us)),
    )


def _time_scaled_orbit() -> TimedRun:
    return _time_orbit(SCALED_ORBIT)


STEADY_DATA_TICK_SHARE: Final = (
    NOMINAL_CONFIG.payload.data_rate_bytes_per_s
    * TICK_US
    / 1_000_000
    / NOMINAL_CONFIG.payload.chunk_size_bytes
)
"""Share of an orbit's ticks that send DATA once passes repeat: an orbit's data, at
4 bytes per tick, goes down at one 64-byte chunk per tick, so 1/16."""


def _check_orbit_mix(run: TimedRun, profile: OrbitProfile) -> int:
    """Assert the timed orbit is the reference profile; return its DATA ticks.

    SCIENCE until the pass, DOWNLINK from the pass start until every stored chunk is
    sent, then SCIENCE again inside the window (telemetry shows the modes), no power or
    thermal flag, and DATA ticks within one percentage point of
    :data:`STEADY_DATA_TICK_SHARE`. The first orbit starts with an empty buffer, so its
    pass sends slightly less than a repeating orbit's.
    """
    data_ticks = [k for k in range(profile.ticks) if FrameType.DATA in run.frame_types(k)]
    assert data_ticks, "the pass sent no DATA"
    assert data_ticks[0] > profile.pass_start
    telemetry = [
        (k, decode_telemetry(frame.payload))
        for k, frames in enumerate(run.frames)
        for frame in frames
        if frame.frame_type is FrameType.TELEMETRY
    ]
    assert all(not t.flags for _, t in telemetry)
    before = {t.mode for k, t in telemetry if k < profile.pass_start}
    in_pass = [t.mode for k, t in telemetry if k > profile.pass_start]
    assert before == {Mode.SCIENCE}
    assert set(in_pass) == {Mode.SCIENCE, Mode.DOWNLINK}
    assert in_pass[-1] is Mode.SCIENCE, "the pass never completed"
    share = len(data_ticks) / profile.ticks
    assert abs(share - STEADY_DATA_TICK_SHARE) < 0.01, share
    return len(data_ticks)


def _orbit_line(label: str, run: TimedRun, profile: OrbitProfile, data_ticks: int) -> str:
    us = run.us_per_tick()
    return (
        f"orbit average ({label}: {profile.pass_start} SCIENCE + "
        f"{profile.ticks - profile.pass_start} pass-window ticks, {data_ticks} with DATA): "
        f"{us:.1f} us/tick, {us * ORBIT_TICKS / 1e6:.1f} s/orbit; SCIENCE ticks "
        f"{run.us_per_tick(0, profile.pass_start):.1f}, pass-window ticks "
        f"{run.us_per_tick(profile.pass_start):.1f} "
        f"(budget {ORBIT_AVERAGE_BUDGET_US:.0f} us on CI Linux; {_mode()})"
    )


def _median_run(run: Callable[[], TimedRun]) -> TimedRun:
    """The run with the median mean time per tick, of :data:`REPEATS` runs."""
    runs = sorted((run() for _ in range(REPEATS)), key=TimedRun.us_per_tick)
    return runs[len(runs) // 2]


def test_orbit_average_us_per_tick_is_within_twice_the_budget(
    capsys: pytest.CaptureFixture[str],
) -> None:
    run = _median_run(_time_scaled_orbit)
    data_ticks = _check_orbit_mix(run, SCALED_ORBIT)
    label = f"reference profile, 1/{ORBIT_SCALE}-scale orbit"
    _report([_orbit_line(label, run, SCALED_ORBIT, data_ticks)], capsys)
    _check_budget("the orbit average", run.us_per_tick(), ORBIT_AVERAGE_BUDGET_US, capsys)


@pytest.mark.slow
def test_a_full_reference_orbit_matches_the_scaled_orbit(
    capsys: pytest.CaptureFixture[str],
) -> None:
    # The fast benchmark's shortened orbit stands in for this one: one whole 55,200-tick
    # orbit of the same profile (about 4 s), timed once and printed beside the scaled
    # figure, so every CI run shows how well the shortened mix tracks a real orbit.
    full = _time_orbit(FULL_ORBIT)
    data_ticks = _check_orbit_mix(full, FULL_ORBIT)
    scaled = _median_run(_time_scaled_orbit)
    ratio = full.us_per_tick() / scaled.us_per_tick()
    _report(
        [
            _orbit_line("reference profile, full orbit", full, FULL_ORBIT, data_ticks),
            f"full orbit / 1/{ORBIT_SCALE}-scale orbit: {ratio:.2f} "
            f"(scaled {scaled.us_per_tick():.1f} us/tick in the same job)",
        ],
        capsys,
    )
    _check_budget("the full-orbit average", full.us_per_tick(), ORBIT_AVERAGE_BUDGET_US, capsys)


# --- Worst-case tick ------------------------------------------------------------------


@dataclass(frozen=True)
class WorstCaseScenario:
    """A scenario whose busiest tick may be the worst case (#120)."""

    name: str
    initial: SpacecraftInitialState
    """Where the spacecraft starts."""
    mode_command: Command
    """Sent in the last boot tick."""
    ticks: int
    """Timed ticks."""
    command_period_ticks: int
    """A PING is sent every this many timed ticks."""
    groups: frozenset[tuple[FrameType, ...]]
    """The tick groups (frames sent, in transmit order) the timed ticks must fall in,
    every one present: the run checks it is the scenario it claims to be."""


FULL_CAPACITY_PASS: Final = WorstCaseScenario(
    name="DOWNLINK pass at full capacity",
    # 2048 chunks stored, more than the timed ticks can send, so a DATA frame fills
    # every tick it fits in; detumbled, so the payload also acquires (and rebuilds its
    # snapshot) every tick, as it does in a real pass.
    initial=SpacecraftInitialState(
        attitude=STEADY_ATTITUDE, payload=PayloadInitial(buffer_fill=0.25)
    ),
    mode_command=Command.begin_downlink(),
    ticks=1_000,
    # Every 3 ticks: the 10-tick telemetry period then meets an ACK in every phase, so
    # every combination of ACK, telemetry and DATA that fits a tick occurs.
    command_period_ticks=3,
    groups=frozenset(
        {
            (FrameType.DATA,),
            (FrameType.ACK, FrameType.DATA),
            (FrameType.TELEMETRY, FrameType.DATA),
            (FrameType.ACK, FrameType.TELEMETRY),
        }
    ),
)
"""The busiest pass ticks: each releases the last tick's chunk, builds and frames a
78-byte DATA frame (chunk content, frame, CRC), and in some ticks also decodes a command
and sends its 14-byte ACK, or encodes a 36-byte telemetry frame, or both. At the default
120-byte capacity ACK, telemetry and DATA together (128 bytes) do not fit one tick:
with both ACK and telemetry the DATA frame is not built and waits a tick."""

WORST_CASE_SCENARIOS: Final = (FULL_CAPACITY_PASS,)
"""The scenarios that define the worst-case tick. **When a feature makes a busier tick
(for example by sending more per tick), add a scenario for it here.**"""


def _time_worst_case(scenario: WorstCaseScenario) -> TimedRun:
    """One run of ``scenario``: boot (untimed), then ``scenario.ticks`` timed ticks."""
    target = _booted(scenario.initial, scenario.mode_command)
    uplink = {
        k: _command_frame(Command.ping(), sequence=2 + k)
        for k in range(0, scenario.ticks, scenario.command_period_ticks)
    }
    environment = NominalEnvironment()
    return _timed_ticks(
        target, scenario.ticks, uplink, lambda k: environment.state_at(target.now_us)
    )


def _group_us(run: TimedRun) -> dict[tuple[FrameType, ...], float]:
    """Median microseconds per tick of each group of ticks (by the frames sent)."""
    groups: dict[tuple[FrameType, ...], list[float]] = {}
    for k, seconds in enumerate(run.tick_s):
        groups.setdefault(run.frame_types(k), []).append(seconds)
    return {group: statistics.median(ticks) * 1e6 for group, ticks in groups.items()}


def _check_worst_case_scenario(run: TimedRun, scenario: WorstCaseScenario) -> None:
    """Assert the timed ticks are ``scenario``: exactly its tick groups, every PING
    ACKed, and full capacity (no room for another frame in a DATA tick)."""
    counts = Counter(run.frame_types(k) for k in range(scenario.ticks))
    assert set(counts) == scenario.groups, counts
    acks = [
        decode_ack(frame.payload)
        for frames in run.frames
        for frame in frames
        if frame.frame_type is FrameType.ACK
    ]
    assert len(acks) == len(range(0, scenario.ticks, scenario.command_period_ticks))
    assert all(ack.accepted for ack in acks)
    capacity = NOMINAL_CONFIG.comms.transmit_capacity_bytes
    data_frame = MIN_FRAME_SIZE + DATA_CHUNK_ID_SIZE + NOMINAL_CONFIG.payload.chunk_size_bytes
    for frames in run.frames:
        used = sum(MIN_FRAME_SIZE + len(frame.payload) for frame in frames)
        assert capacity - data_frame < used <= capacity  # no room for another DATA frame


def _group_name(group: tuple[FrameType, ...]) -> str:
    return " + ".join(frame_type.name for frame_type in group)


def test_worst_case_tick_us_is_within_twice_the_budget(
    capsys: pytest.CaptureFixture[str],
) -> None:
    lines: list[str] = []
    worst: tuple[float, str] = (0.0, "")
    for scenario in WORST_CASE_SCENARIOS:
        runs = [_time_worst_case(scenario) for _ in range(REPEATS)]
        _check_worst_case_scenario(runs[0], scenario)
        per_run = [_group_us(run) for run in runs]
        counts = Counter(runs[0].frame_types(k) for k in range(scenario.ticks))
        groups = {
            group: statistics.median(result[group] for result in per_run)
            for group in sorted(scenario.groups, key=_group_name)
        }
        lines.append(
            f"tick groups ({scenario.name}, median tick per group): "
            + ", ".join(
                f"{_group_name(group)} {us:.1f} us (x{counts[group]})"
                for group, us in groups.items()
            )
        )
        for group, us in groups.items():
            worst = max(worst, (us, f"{scenario.name}: {_group_name(group)}"))
    us, what = worst
    lines.append(
        f"worst-case tick ({what}): {us:.1f} us "
        f"(budget {WORST_CASE_BUDGET_US:.0f} us on CI Linux; {_mode()})"
    )
    _report(lines, capsys)
    _check_budget(f"the worst-case tick ({what})", us, WORST_CASE_BUDGET_US, capsys)


def test_gate_mode_is_reported_and_active_on_github_actions(
    capsys: pytest.CaptureFixture[str],
) -> None:
    _report([f"benchmark mode: {_mode()}"], capsys)
    # GitHub Actions sets both CI=true and GITHUB_ACTIONS=true: the gate must be on there.
    if os.environ.get("GITHUB_ACTIONS", "").lower() == "true":
        assert gate_enforced(), "CI gate inactive on GitHub Actions: is CI=true set?"
    assert gate_enforced({"CI": "true"})
    assert gate_enforced({"CI": "TRUE"})
    assert not gate_enforced({})
    assert not gate_enforced({"CI": "false"})


# --- Per subsystem --------------------------------------------------------------------


def _real_subsystem(name: str, board: SnapshotBoard) -> Subsystem:
    config, initial = NOMINAL_CONFIG, DEFAULT_INITIAL_STATE
    builders: dict[str, Callable[[], Subsystem]] = {
        "power": lambda: Power(config.power, initial.power, reader=board),
        "thermal": lambda: Thermal(config.thermal, initial.thermal, reader=board),
        "attitude": lambda: Attitude(config.attitude, initial.attitude),
        "payload": lambda: Payload(config.payload, initial.payload, reader=board),
        "comms": lambda: Comms(config.comms),
    }
    return builders[name]()


def _orbit_sample(ticks: int) -> list[EnvironmentState]:
    """``ticks`` environment states spread evenly over one orbit, sunlight and eclipse."""
    env = NominalEnvironment()
    stride_us = env.orbit_period_us // ticks
    return [env.state_at(n * stride_us) for n in range(ticks)]


def _time_subsystem(name: str, envs: list[EnvironmentState]) -> float:
    """One run: the real ``name`` subsystem with the four default fakes on its board,
    in SCIENCE controls; times only its ``step()`` plus ``snapshot()``."""
    board = SnapshotBoard()
    real = _real_subsystem(name, board)
    others = [fake for fake in fake_subsystems() if fake.name != name]
    SubsystemStack([real, *others], board=board).reset(RngFactory(SEED))
    controls = controls_for_mode(Mode.SCIENCE)
    step, snapshot = real.step, real.snapshot
    started = time.perf_counter()
    for env in envs:
        step(DEFAULT_TICK_US, env, controls)
        snapshot()
    return (time.perf_counter() - started) / len(envs) * 1e6


def test_subsystem_us_per_tick_is_reported(capsys: pytest.CaptureFixture[str]) -> None:
    envs = _orbit_sample(SUBSYSTEM_TICKS)
    results = {
        name: statistics.median(_time_subsystem(name, envs) for _ in range(REPEATS))
        for name in STEP_ORDER
    }
    _report(
        [
            f"{name}: step + snapshot {us:.1f} us/tick (guidance {SUBSYSTEM_GUIDANCE_US:.0f} us)"
            + ("" if us <= SUBSYSTEM_GUIDANCE_US else "  <- over guidance")
            for name, us in results.items()
        ],
        capsys,
    )
    assert set(results) == set(STEP_ORDER)
