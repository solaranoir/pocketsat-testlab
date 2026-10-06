"""Determinism of ``SilTarget`` over long runs, across seeds, and across resets (#61).

Every test here uses the real flight computer with its default settings (5 s BOOT) and
drives the target the way the orchestrator will (ADR-0003 §2): each tick, the
environment is sampled from :class:`NominalEnvironment` at the time of an
orchestrator-side :class:`SimClock`, applied, and the target advances one tick. The run
script mixes every command, malformed and undecodable uplink, and all four target
faults (#60), so the comparison covers the command, mode, fault, and reboot paths.

What is compared:

- **Downlink bytes**, the ground's view: every frame, with the tick it came out in.
  Until #55 the real flight computer sends only ACK frames, so the seeded sensor noise
  is not in the downlink yet; it is in the tick records.
- **Every** :class:`SilTick`: merged controls, every subsystem's truth and readings
  (seeded noise included), uplink, traffic, faults, and reboots, tick by tick.

Not here, by design: the contract suite (``tests/contract``, where ``SilTarget`` is
registered with the same real flight computer); the one-orbit run under the default
92-minute orbit (``test_one_orbit_is_deterministic_and_eclipse_reaches_the_subsystems``
in ``test_sil_target.py``); faulted determinism over 100 ticks (``test_sil_faults.py``);
golden telemetry (#64); behavior-level SIL tests (#102).
"""

import dataclasses
from collections.abc import Callable
from typing import Any

import pytest

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import Mode
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import Command, decode_ack, encode_command
from pocketsat.spacecraft import AttitudeSnapshot, PowerSnapshot, ThermalSnapshot
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import (
    BATTERY_DRAIN,
    FORCED_RESET,
    SENSOR_FREEZE,
    TRANSMITTER_OFF,
    SilTarget,
    SilTick,
)

TICK = DEFAULT_TICK_US
LONG_RUN_TICKS = 10_000
"""#61: byte-identical downlink over at least 10,000 ticks (1,000 s simulated)."""

ORBIT = NominalEnvironment(orbit_period_us=400_000_000)
"""A 400 s orbit (35% eclipse), so a 10,000-tick run crosses into eclipse and back out
twice. The default 92-minute orbit is covered by the one-orbit test in
``test_sil_target.py``."""

CYCLE_TICKS = 1_000
"""The run script repeats every 100 s."""


def _command(command: Command, n: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, n & 0xFFFF, encode_command(command)))


def _fault(fault_type: str, duration_us: int | None, **params: float | str) -> TargetFault:
    return TargetFault(fault_type=fault_type, params=params, duration_us=duration_us)


def _script(n: int) -> tuple[list[bytes], list[TargetFault]]:
    """The uplink frames to send and the faults to inject before tick ``n``.

    One 100 s cycle: a PING in BOOT; SCIENCE, DOWNLINK, NOMINAL, SAFE, and back to
    NOMINAL after the boot; a drain, ``transmitter_off`` with a PING under it, and a
    thermal freeze overlapping; a malformed command and an undecodable frame; a
    ``forced_reset`` (held 0.5 s on even cycles, a pulse on odd ones) with a PING during
    it; a RESET command; and a PING in the new BOOT.
    """
    cycle, step = divmod(n, CYCLE_TICKS)
    frames: list[bytes] = []
    faults: list[TargetFault] = []
    if step == 5:
        frames.append(_command(Command.ping(), n))
    elif step == 60:
        frames.append(_command(Command.set_mode(Mode.SCIENCE), n))
    elif step == 150:
        faults.append(_fault(BATTERY_DRAIN, 20_000_000, load_w=1.5))
    elif step == 200:
        frames.append(_command(Command.begin_downlink(), n))
    elif step == 260:
        faults.append(_fault(TRANSMITTER_OFF, 2_000_000))
    elif step == 261:
        frames.append(_command(Command.ping(), n))
    elif step == 280:
        faults.append(_fault(SENSOR_FREEZE, 3_000_000, subsystem="thermal"))
    elif step == 400:
        frames.append(_command(Command.set_mode(Mode.NOMINAL), n))
    elif step == 450:
        frames.append(_command(Command.enter_safe_mode(), n))
    elif step == 500:
        frames.append(_command(Command.set_mode(Mode.NOMINAL), n))
    elif step == 600:
        frames.append(_command(Command(0x7F), n))  # unknown command: NACK
        frames.append(_command(Command.ping(), n)[:-1] + b"\x00")  # bad CRC: dropped
    elif step == 700:
        faults.append(_fault(FORCED_RESET, 500_000 if cycle % 2 == 0 else None))
        frames.append(_command(Command.ping(), n))
    elif step == 800:
        frames.append(_command(Command.reset(), n))
    elif step == 820:
        frames.append(_command(Command.ping(), n))
    return frames, faults


@dataclasses.dataclass
class Run:
    """What one run produced: the ground's view and the target's tick records."""

    downlink: list[tuple[int, bytes]] = dataclasses.field(default_factory=list)
    ticks: list[SilTick] = dataclasses.field(default_factory=list)


def drive(
    target: SilTarget,
    ticks: int,
    *,
    script: Callable[[int], tuple[list[bytes], list[TargetFault]]] = _script,
    environment: NominalEnvironment = ORBIT,
) -> Run:
    """Drive ``target`` (already reset) for ``ticks`` ticks, as the orchestrator will.

    Before each tick: inject the script's faults, send its frames, and apply the
    environment sampled at the time of an orchestrator-side ``SimClock`` (ADR-0003 §2).
    """
    run = Run()
    clock = SimClock(TICK)
    for n in range(ticks):
        frames, faults = script(n)
        for fault in faults:
            target.inject(fault)
        for frame in frames:
            target.send(frame)
        assert clock.now_us == target.now_us
        target.apply_environment(environment.state_at(clock.now_us))
        target.advance(TICK)
        clock.advance_one_tick()
        assert target.last_tick is not None
        run.ticks.append(target.last_tick)
        run.downlink.extend((n, frame) for frame in target.receive())
    return run


def fresh_run(seed: int, ticks: int) -> Run:
    target = SilTarget()
    target.connect()
    target.reset(seed)
    return drive(target, ticks)


def _check_the_script_was_exercised(run: Run) -> None:
    """Guard against a comparison that passes because nothing happened."""
    answers = [decode_ack(decode_frame(frame).payload) for _, frame in run.downlink]
    assert any(answer.accepted for answer in answers)
    assert any(not answer.accepted for answer in answers)
    assert any(t.active_faults for t in run.ticks)
    assert any(t.flight_computer_held for t in run.ticks)
    assert any(t.flight_computer_rebooted for t in run.ticks)
    assert any(t.traffic.outbound_suppressed_count for t in run.ticks)
    assert any(t.undecodable_uplink_count for t in run.ticks)
    radio_modes = {t.controls.radio.mode for t in run.ticks}
    assert len(radio_modes) > 1  # the commanded and fault-merged controls changed


# --- Same seed: byte-identical ------------------------------------------------------------


@pytest.mark.slow
def test_same_seed_gives_byte_identical_downlink_over_10000_ticks() -> None:
    first = fresh_run(seed=61, ticks=LONG_RUN_TICKS)
    second = fresh_run(seed=61, ticks=LONG_RUN_TICKS)

    _check_the_script_was_exercised(first)
    sunlit = [t.environment.sunlit for t in first.ticks]
    assert sunlit[0] and not all(sunlit) and sunlit[-1]
    generation = [t.state.get("power", PowerSnapshot).truth.generation_w for t in first.ticks]
    assert all(g == 0.0 for g, s in zip(generation, sunlit, strict=True) if not s)
    assert generation[0] > 0.0 and generation[-1] > 0.0  # stops in eclipse, resumes after

    assert len(first.downlink) >= 60  # at least six answered commands per cycle
    assert first.downlink == second.downlink
    assert first.ticks == second.ticks


def test_same_seed_gives_identical_runs_over_two_script_cycles() -> None:
    # The fast guard for the plain ``pytest`` run; the 10,000-tick run above is slow.
    first = fresh_run(seed=61, ticks=2 * CYCLE_TICKS)
    second = fresh_run(seed=61, ticks=2 * CYCLE_TICKS)
    _check_the_script_was_exercised(first)
    assert first.downlink == second.downlink
    assert first.ticks == second.ticks


# --- Different seed: different noise ------------------------------------------------------

NOISY_READINGS: dict[str, tuple[str, ...]] = {
    "power": ("bus_v", "battery_current_a", "soc"),
    "thermal": ("battery_c", "electronics_c"),
    "attitude": ("pointing_error_deg", "rate_dps"),
}
SNAPSHOT_TYPES = {"power": PowerSnapshot, "thermal": ThermalSnapshot, "attitude": AttitudeSnapshot}


def _noise(run: Run, subsystem: str, field: str) -> list[float]:
    """The sensor noise on one reading, tick by tick: reported value minus truth."""
    snapshot_type = SNAPSHOT_TYPES[subsystem]
    snapshots: list[Any] = [t.state.get(subsystem, snapshot_type) for t in run.ticks]
    return [getattr(s.readings, field) - getattr(s.truth, field) for s in snapshots]


def _idle_run(seed: int) -> Run:
    target = SilTarget()
    target.reset(seed)
    return drive(target, 20, script=lambda n: ([], []))


@pytest.mark.parametrize(
    ("subsystem", "field"),
    [(subsystem, field) for subsystem, fields in NOISY_READINGS.items() for field in fields],
)
def test_different_seed_gives_different_sensor_noise(subsystem: str, field: str) -> None:
    # Guards against an ignored seed: the noise on every noisy reading changes with the
    # seed, and repeats with it. Until #55 the noise is not in the downlink, so it is
    # read from the tick records.
    seed_a = _noise(_idle_run(seed=1), subsystem, field)
    seed_b = _noise(_idle_run(seed=2), subsystem, field)
    assert any(seed_a)
    assert seed_a != seed_b
    assert _noise(_idle_run(seed=1), subsystem, field) == seed_a


def test_different_seed_gives_a_different_run_with_the_same_script() -> None:
    first = fresh_run(seed=61, ticks=200)
    second = fresh_run(seed=62, ticks=200)
    assert [t.state for t in first.ticks] != [t.state for t in second.ticks]
    assert [t.controls for t in first.ticks] == [t.controls for t in second.ticks]


# --- reset(seed) mid-run ------------------------------------------------------------------

RESUMED_TICKS = 900
"""Long enough after the reset for every event in the script's first cycle."""


@pytest.mark.parametrize(
    ("interrupted_seed", "interrupt_at"),
    [(61, 55), (7, 702), (7, 265)],
    ids=["same-seed-after-boot", "other-seed-while-held", "other-seed-while-faulted"],
)
def test_reset_mid_run_reproduces_a_fresh_run(interrupted_seed: int, interrupt_at: int) -> None:
    """``reset(seed)`` anywhere in a run gives exactly a fresh target's run.

    Interrupted just after the boot completes (same seed before and after); while a
    ``forced_reset`` holds the flight computer; and with a drain and ``transmitter_off``
    active (another seed before the reset). At the reset, uplink is queued, downlink is
    undrained, a freeze without a duration is active, and a hold is pending.
    """
    target = SilTarget()
    target.reset(interrupted_seed)
    before = drive(target, interrupt_at)
    assert before.ticks[-1].flight_computer_held == (interrupt_at == 702)
    target.inject(_fault(SENSOR_FREEZE, None))
    target.send(_command(Command.ping(), 0xBEEF))
    target.advance(TICK)  # undrained downlink
    target.send(_command(Command.reset(), 0xBEF0))  # queued uplink
    target.inject(_fault(FORCED_RESET, 1_000_000))  # pending hold

    target.reset(61)
    resumed = drive(target, RESUMED_TICKS)
    fresh = fresh_run(seed=61, ticks=RESUMED_TICKS)
    assert resumed.downlink == fresh.downlink
    assert resumed.ticks == fresh.ticks
