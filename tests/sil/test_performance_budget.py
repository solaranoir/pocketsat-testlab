"""Performance budget benchmark (#78).

Times the full stack (five real subsystems, the flight computer, and ``SilTarget``) and
each real subsystem against the shared fakes (#76), and prints microseconds per tick in
the CI log on every run. The budgets and how they were set are in
``docs/spacecraft.md`` ("Performance budget").

- **Full stack:** at most :data:`FULL_STACK_BUDGET_US` per tick on CI Linux (about 5.5 s
  per 55,200-tick orbit, under #63's 10 s). **In CI** (the ``CI`` environment variable
  is ``true``, which GitHub Actions sets) the test fails above twice the budget, so it
  catches real slowdowns without failing on a noisy shared runner. **Locally** it only
  prints the numbers, because a busy developer machine can be several times slower
  (one run at load average ~15 measured 392 us per tick, against 57 to 62 unloaded).
- **Per subsystem:** ``step()`` plus ``snapshot()`` within about
  :data:`SUBSYSTEM_GUIDANCE_US`. Guidance only: printed, never a failure.

Each measurement is the median of :data:`REPEATS` runs, each a fresh ``reset(seed)``
followed by a fixed number of timed ticks. Wall-clock timing is allowed here: the
determinism guard covers only simulation code in ``src/pocketsat`` (ADR-0003).

Three full-stack runs are reported, all with the real flight computer answering a PING
every 10 s:

- in SCIENCE at the **default telemetry cadence** (#55: one frame per second). This is
  the budgeted figure.
- in SCIENCE with **telemetry every tick** (``TelemetryConfig.uniform(tick)``), the most
  telemetry the scheduler can be configured for. Each frame is encoded with the frame
  CRC, so this run guards the lookup-table CRC (#78) and the telemetry encoder.
- in a **DOWNLINK pass at full capacity** (#56): a payload buffer large enough that the
  session sends a 78-byte DATA frame in every timed tick but the few that also carry an
  ACK and a telemetry frame (chunk content generated,
  encoded, framed, and drained), with telemetry once a second.
"""

import os
import statistics
import time
from collections.abc import Callable, Mapping
from typing import Final

import pytest

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import FlightComputer, FlightComputerConfig, Mode, controls_for_mode
from pocketsat.flight.telemetry import TelemetryConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import Command, encode_command
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    STEP_ORDER,
    Attitude,
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

FULL_STACK_BUDGET_US: Final = 100.0
"""Full stack, per tick, on CI Linux (#78)."""

SUBSYSTEM_GUIDANCE_US: Final = 15.0
"""One subsystem's ``step()`` plus ``snapshot()``, per tick, on CI (guidance, #78)."""

FAIL_FACTOR: Final = 2.0
"""In CI, the benchmark fails only above this multiple of the budget."""

REPEATS: Final = 5
FULL_STACK_TICKS: Final = 1_000
SUBSYSTEM_TICKS: Final = 2_000
BOOT_TICKS: Final = 60
"""Untimed ticks before each full-stack run: BOOT lasts 50 ticks (#49)."""

ORBIT_TICKS: Final = 92 * 60 * 10
"""One default 92-minute ``NominalEnvironment`` orbit at the 100 ms tick."""

SEED: Final = 78


def gate_enforced(environ: Mapping[str, str] = os.environ) -> bool:
    """Whether the 2x-budget check fails the test: only in CI (``CI=true``)."""
    return environ.get("CI", "").lower() == "true"


def _mode() -> str:
    return "CI: fails above 2x budget" if gate_enforced() else "local: report only"


def _report(lines: list[str], capsys: pytest.CaptureFixture[str]) -> None:
    """Print to the terminal (and so the CI log) even though pytest captures output."""
    with capsys.disabled():
        print("".join(f"\n[perf #78] {line}" for line in lines))


def _command(command: Command, sequence: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


DOWNLINK_BUFFER: Final = SpacecraftInitialState(payload=PayloadInitial(buffer_fill=0.25))
"""A starting buffer of 2048 default chunks, more than the timed ticks can send."""


TO_SCIENCE: Final = Command.set_mode(Mode.SCIENCE)


def _time_full_stack(
    factory: Callable[[], FlightComputer],
    mode_command: Command = TO_SCIENCE,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
) -> float:
    """One run: reset, boot, then ``mode_command`` (untimed), then time
    ``FULL_STACK_TICKS`` ticks of the orchestrator's loop (sample the environment, apply
    it, advance one tick, drain the downlink). Returns microseconds per tick."""
    target = SilTarget(initial=initial, flight_computer_factory=factory)
    env = NominalEnvironment()
    target.connect()
    target.reset(SEED)
    for n in range(BOOT_TICKS):
        if n == BOOT_TICKS - 1:
            target.send(_command(mode_command, sequence=1))
        target.apply_environment(env.state_at(target.now_us))
        target.advance(target.tick_us)
        target.receive()

    tick_us = target.tick_us
    received = 0
    started = time.perf_counter()
    for n in range(FULL_STACK_TICKS):
        if n % 100 == 50:
            target.send(_command(Command.ping(), sequence=n))
        target.apply_environment(env.state_at(target.now_us))
        target.advance(tick_us)
        received += len(target.receive())
    elapsed = time.perf_counter() - started
    assert received > 0
    return elapsed / FULL_STACK_TICKS * 1e6


def _time_downlink_pass() -> float:
    """One run of :func:`_time_full_stack` in a DOWNLINK pass at full capacity (#56)."""
    return _time_full_stack(FlightComputer, Command.begin_downlink(), DOWNLINK_BUFFER)


def _median_us[*A](run: Callable[[*A], float], *args: *A) -> float:
    """The median of :data:`REPEATS` runs of ``run(*args)``."""
    return statistics.median(run(*args) for _ in range(REPEATS))


@pytest.mark.parametrize(
    ("label", "factory"),
    [
        ("real flight computer, SCIENCE, telemetry 1 Hz", FlightComputer),
        (
            "real flight computer, SCIENCE, telemetry every tick",
            lambda: FlightComputer(
                FlightComputerConfig(telemetry=TelemetryConfig.uniform(DEFAULT_TICK_US))
            ),
        ),
    ],
    ids=["real-flight-computer", "telemetry-every-tick"],
)
def test_full_stack_us_per_tick_is_within_twice_the_budget(
    label: str, factory: Callable[[], FlightComputer], capsys: pytest.CaptureFixture[str]
) -> None:
    us_per_tick = _median_us(_time_full_stack, factory)
    _check_full_stack(label, us_per_tick, capsys)


def test_downlink_pass_us_per_tick_is_within_twice_the_budget(
    capsys: pytest.CaptureFixture[str],
) -> None:
    us_per_tick = _median_us(_time_downlink_pass)
    _check_full_stack("real flight computer, DOWNLINK pass at full capacity", us_per_tick, capsys)


def test_the_downlink_benchmark_sends_data_in_every_timed_tick() -> None:
    # Guards the benchmark itself: the timed ticks are a pass at full capacity.
    target = SilTarget(initial=DOWNLINK_BUFFER)
    target.reset(SEED)
    for n in range(BOOT_TICKS):
        if n == BOOT_TICKS - 1:
            target.send(_command(Command.begin_downlink(), sequence=1))
        target.advance(target.tick_us)
    for _ in range(FULL_STACK_TICKS):
        target.advance(target.tick_us)
        frames = [decode_frame(f) for f in target.receive()]
        assert [f.frame_type for f in frames].count(FrameType.DATA) == 1


def _check_full_stack(label: str, us_per_tick: float, capsys: pytest.CaptureFixture[str]) -> None:
    orbit_s = us_per_tick * ORBIT_TICKS / 1e6
    _report(
        [
            f"full stack ({label}): {us_per_tick:.1f} us/tick, {orbit_s:.1f} s/orbit "
            f"(budget {FULL_STACK_BUDGET_US:.0f} us on CI Linux; {_mode()})"
        ],
        capsys,
    )
    limit_us = FAIL_FACTOR * FULL_STACK_BUDGET_US
    if gate_enforced():
        assert us_per_tick <= limit_us, (
            f"full stack ({label}) takes {us_per_tick:.1f} us per tick, over twice the "
            f"{FULL_STACK_BUDGET_US:.0f} us budget (#78)"
        )
    elif us_per_tick > limit_us:
        _report([f"over {limit_us:.0f} us: would fail in CI (is this machine busy?)"], capsys)


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
    results = {name: _median_us(_time_subsystem, name, envs) for name in STEP_ORDER}
    _report(
        [
            f"{name}: step + snapshot {us:.1f} us/tick (guidance {SUBSYSTEM_GUIDANCE_US:.0f} us)"
            + ("" if us <= SUBSYSTEM_GUIDANCE_US else "  <- over guidance")
            for name, us in results.items()
        ],
        capsys,
    )
    assert set(results) == set(STEP_ORDER)
