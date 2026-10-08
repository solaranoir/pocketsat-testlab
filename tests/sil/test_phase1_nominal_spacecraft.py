"""Phase 1 definition of done: the nominal scenario, end to end through TestTarget (#63).

The roadmap's Phase 1 milestone: run a nominal scenario and receive deterministic
spacecraft telemetry. :func:`test_phase1_nominal_spacecraft` runs the scenario of
``_nominal_scenario.py`` (#72's reference profile for one full orbit, sunlight and
eclipse: boot, PING, SET_MODE SCIENCE, BEGIN_DOWNLINK for the 10-minute pass, back to
NOMINAL) on a ``SilTarget`` with seed 42, driven through the ``TestTarget`` interface
only, with real radio traffic (#98). Every check uses what the ground received, and
checks properties and margins, never exact values such as a specific SOC.

ADR-0004's latencies are part of the checks, not tolerances around them: a command is
ACKed in the tick it is sent, and the telemetry frame of that tick already reports the
new mode (#51), but the subsystems obey the new mode's controls only from the next
tick, so that frame may still show the old payload state (the one-frame telemetry lag,
ADR-0004 §2); the next frame shows the effect.

The orbit tests are ``slow`` (each runs one 55,220-tick orbit, about 4 s; AGENTS.md
marks anything over a second slow), so CI runs them in a slow shard. The harness's
short checks and the different-seed check run in the fast suite.
"""

import functools
import itertools
import time
from collections.abc import Iterable

import pytest
from _nominal_scenario import (
    BOOT_US,
    ENVIRONMENT,
    PASS_END_US,
    PASS_START_US,
    PING_US,
    RUN_US,
    SEED,
    TICK_US,
    ScenarioRun,
    TickDownlink,
    run_nominal_scenario,
)
from _reference_profile import PASS_US, SCIENCE_COMMAND_US
from test_performance_budget import ORBIT_AVERAGE_BUDGET_US, gate_enforced

from pocketsat.flight import DEFAULT_FLIGHT_COMPUTER_CONFIG, Mode
from pocketsat.frame import MIN_FRAME_SIZE
from pocketsat.messages import (
    ACK_PAYLOAD_SIZE,
    TELEMETRY_FRAME_SIZE,
    CommandId,
    Telemetry,
    TelemetryFlags,
    data_frame_size,
)
from pocketsat.spacecraft import NOMINAL_CONFIG, PayloadState, chunk_content

CHUNK = NOMINAL_CONFIG.payload.chunk_size_bytes
"""Payload chunk size, bytes (64): one DATA frame carries one chunk."""

CAPACITY = NOMINAL_CONFIG.comms.transmit_capacity_bytes
"""Comms' transmit capacity per tick, bytes (120 at the 100 ms tick)."""

LOW_BATTERY = NOMINAL_CONFIG.power.low_battery_soc
SOC_MARGIN = 0.10
"""#72's minimum margin above ``low_battery``: 10 percentage points."""

DATA_MARGIN = 0.20
"""#72's data budget: an orbit's data fits one pass's net capacity with 20% to spare."""

TICKS_PER_S = 1_000_000 // TICK_US
PASS_S = PASS_US // 1_000_000
TELEMETRY_TICKS_PER_PASS_S = 1
"""#72 budgets one tick per second of the pass for telemetry and an ACK/NACK, leaving no
room for a DATA frame in it."""

NET_PASS_CAPACITY_BYTES = (TICKS_PER_S - TELEMETRY_TICKS_PER_PASS_S) * CHUNK * PASS_S
"""#72's net pass capacity: 9 chunks a second for 10 minutes, 345,600 bytes."""

RUNTIME_LIMIT_S = 10.0
"""#63: the scenario runs in CI in under 10 seconds."""

TELEMETRY_PERIOD_TICKS = {
    mode: period // TICK_US
    for mode in (Mode.NOMINAL, Mode.SCIENCE, Mode.DOWNLINK)
    if (period := DEFAULT_FLIGHT_COMPUTER_CONFIG.telemetry.period_us(mode)) is not None
}
"""Telemetry period per mode, ticks (#55): 10 (1 s) in each of the scenario's modes."""

SENSOR_FIELDS = ("bus_v", "soc", "battery_c", "electronics_c", "pointing_error_deg")
"""Telemetry fields carrying seeded sensor noise (ADR-0004 §7)."""


def tick(at_us: int) -> int:
    """The tick that starts at ``at_us`` (the tick a command sent then is handled in)."""
    assert at_us % TICK_US == 0
    return at_us // TICK_US


@functools.cache
def timed_run(seed: int) -> tuple[ScenarioRun, float]:
    """The whole scenario for ``seed`` and its wall-clock seconds, run once per session
    (the determinism test runs its second copy itself)."""
    started = time.perf_counter()
    run = run_nominal_scenario(seed)
    return run, time.perf_counter() - started


def modes_in_order(telemetry: Iterable[tuple[int, Telemetry]]) -> list[tuple[int, Mode]]:
    """``(tick, mode)`` at every change of the reported mode, starting with the first."""
    changes: list[tuple[int, Mode]] = []
    for n, frame in telemetry:
        if not changes or frame.mode is not changes[-1][1]:
            changes.append((n, frame.mode))
    return changes


def next_frame_after(run: ScenarioRun, n: int) -> Telemetry:
    """The first telemetry frame sent after tick ``n``."""
    return next(frame for k, frame in run.telemetry if k > n)


def at_tick(run: ScenarioRun, n: int) -> TickDownlink:
    """What tick ``n`` sent (it must have sent something)."""
    return next(downlink for downlink in run.downlink if downlink.tick == n)


# --- The acceptance test --------------------------------------------------------------


@pytest.mark.slow
def test_phase1_nominal_spacecraft(capsys: pytest.CaptureFixture[str]) -> None:
    run, elapsed_s = timed_run(SEED)
    telemetry = run.telemetry
    science_tick, pass_tick, nominal_tick = (
        tick(SCIENCE_COMMAND_US),
        tick(PASS_START_US),
        tick(PASS_END_US),
    )

    # The run covers a full orbit, sunlight and eclipse, and telemetry came from both.
    assert run.ticks_run == tick(RUN_US) and ENVIRONMENT.orbit_period_us < RUN_US
    in_eclipse = [ENVIRONMENT.state_at(n * TICK_US).sunlit is False for n, _ in telemetry]
    assert any(in_eclipse) and not all(in_eclipse)

    # Real radio traffic only (#98): ACK, telemetry and DATA frames of their real sizes,
    # never more in a tick than comms' capacity.
    sizes = {len(frame) for frame in run.frames}
    assert sizes == {
        MIN_FRAME_SIZE + ACK_PAYLOAD_SIZE,
        TELEMETRY_FRAME_SIZE,
        data_frame_size(CHUNK),
    }
    assert all(sum(map(len, downlink.frames)) <= CAPACITY for downlink in run.downlink)

    # BOOT: silent (BOOT sends no telemetry) until the boot-complete beacon in the tick
    # that ends at the 5 s boot, which reports NOMINAL (#49, #55).
    first = run.downlink[0]
    assert first.tick == tick(BOOT_US) - 1
    assert first.acks == () and first.data == () and len(first.telemetry) == 1
    beacon = first.telemetry[0]
    assert beacon.mode is Mode.NOMINAL and beacon.uptime_ms == BOOT_US // 1000

    # Every command is ACKed in the tick it was sent (ADR-0004: command -> ACK, same
    # tick), and nothing is NACKed.
    assert [(n, ack.command_id, ack.reason) for n, ack in run.acks] == [
        (tick(PING_US), CommandId.PING, None),
        (science_tick, CommandId.SET_MODE, None),
        (pass_tick, CommandId.BEGIN_DOWNLINK, None),
        (nominal_tick, CommandId.SET_MODE, None),
    ]

    # Mode sequence: BOOT (the silence above), NOMINAL, SCIENCE, DOWNLINK, SCIENCE
    # (DOWNLINK_COMPLETE inside the pass window), NOMINAL; each commanded change is
    # reported in its command's tick.
    changes = modes_in_order(telemetry)
    assert [mode for _, mode in changes] == [
        Mode.NOMINAL,
        Mode.SCIENCE,
        Mode.DOWNLINK,
        Mode.SCIENCE,
        Mode.NOMINAL,
    ]
    complete_tick = changes[3][0]
    assert [n for n, _ in changes] == [
        first.tick,
        science_tick,
        pass_tick,
        complete_tick,
        nominal_tick,
    ]
    assert pass_tick < complete_tick < nominal_tick, "the pass did not complete in its window"

    # No SAFE or FAULT, and no power or thermal flag, in the whole run.
    assert {frame.mode for _, frame in telemetry}.isdisjoint({Mode.SAFE, Mode.FAULT})
    assert all(frame.flags == TelemetryFlags(0) for _, frame in telemetry)

    # Battery: the reported SOC never comes within 10 points of low_battery (#72).
    min_soc = min(frame.soc for _, frame in telemetry)
    assert min_soc >= LOW_BATTERY + SOC_MARGIN, (
        f"minimum SOC {min_soc:.3f} is less than {SOC_MARGIN:.0%} above "
        f"low_battery {LOW_BATTERY:.2f}"
    )

    # Physical effects one tick after each mode change (ADR-0004): the change's own
    # frame may still show the old payload state (the one-frame lag); the next frame
    # shows the payload as the new mode's controls set it (#47 mode table).
    assert next_frame_after(run, science_tick).payload_state is not PayloadState.OFF
    assert next_frame_after(run, pass_tick).payload_state is PayloadState.OFF
    assert next_frame_after(run, complete_tick).payload_state is not PayloadState.OFF
    assert next_frame_after(run, nominal_tick).payload_state is PayloadState.OFF

    # Telemetry cadence (#55): one frame every period of the mode, never a longer gap;
    # a shorter gap only where a mode change sends its frame at once.
    change_ticks = {n for n, _ in changes}
    for (previous, _), (n, frame) in itertools.pairwise(telemetry):
        period = TELEMETRY_PERIOD_TICKS[frame.mode]
        gap = n - previous
        assert gap == period or (gap < period and n in change_ticks), (previous, n, frame.mode)

    # Payload data generated in SCIENCE: acquiring, and the buffer grew before the pass.
    before_pass = [frame for n, frame in telemetry if science_tick < n < pass_tick]
    assert any(frame.payload_state is PayloadState.ACQUIRING for frame in before_pass)
    entry = at_tick(run, pass_tick).telemetry[0]
    assert entry.mode is Mode.DOWNLINK and entry.buffered_bytes >= before_pass[0].buffered_bytes
    whole_chunks = entry.buffered_bytes // CHUNK
    assert whole_chunks > 0

    # Downlinked as DATA frames: only in DOWNLINK (from the tick after the ACK to the
    # completion tick), every one a valid chunk with its expected content (#43, #56),
    # each whole chunk buffered at entry exactly once, in ID order.
    data = run.data
    assert all(pass_tick < n <= complete_tick for n, _ in data)
    ids = [chunk.chunk_id for _, chunk in data]
    assert ids == list(range(ids[0], ids[0] + whole_chunks))
    assert all(chunk.content == chunk_content(chunk.chunk_id, CHUNK) for _, chunk in data)

    # The buffer does not keep growing: under one chunk when the session completes, and
    # at the end of the pass window well inside #72's data budget (an orbit's data must
    # fit the net pass capacity with 20% to spare), and below its size at pass entry.
    completion = at_tick(run, complete_tick).telemetry[0]
    assert completion.buffered_bytes < CHUNK
    end_of_pass = [frame for n, frame in telemetry if n < nominal_tick][-1]
    budget = (1 - DATA_MARGIN) * NET_PASS_CAPACITY_BYTES
    assert end_of_pass.buffered_bytes <= budget, (end_of_pass.buffered_bytes, budget)
    assert end_of_pass.buffered_bytes < entry.buffered_bytes

    # Runtime: under 10 s in CI (#63), and microseconds per tick against #120's
    # orbit-average budget (the whole harness loop, frame decoding included).
    us_per_tick = elapsed_s / run.ticks_run * 1e6
    mode = "CI: fails at 10 s" if gate_enforced() else "local: report only"
    with capsys.disabled():
        print(
            f"\n[perf #63] nominal scenario, {run.ticks_run} ticks (one orbit + "
            f"{RUN_US - ENVIRONMENT.orbit_period_us} us): {elapsed_s:.2f} s, "
            f"{us_per_tick:.1f} us/tick (orbit-average budget "
            f"{ORBIT_AVERAGE_BUDGET_US:.0f} us on CI Linux; limit {RUNTIME_LIMIT_S:.0f} s; {mode})"
        )
    if gate_enforced():
        assert elapsed_s < RUNTIME_LIMIT_S, f"the nominal scenario took {elapsed_s:.1f} s"


# --- Determinism ----------------------------------------------------------------------


@pytest.mark.slow
def test_same_seed_gives_byte_identical_downlink_frames() -> None:
    first, _ = timed_run(SEED)
    second = run_nominal_scenario(SEED)
    assert second.downlink_bytes() == first.downlink_bytes()
    assert [(d.tick, d.frames) for d in second.downlink] == [
        (d.tick, d.frames) for d in first.downlink
    ]
    assert second.uplink == first.uplink


SHORT_RUN_US = 60 * 1_000_000
"""The first minute: boot, PING, SCIENCE, and 54 telemetry frames."""


def test_a_different_seed_gives_different_sensor_noise() -> None:
    # The first minute is enough: every frame carries noisy readings (#54).
    a = run_nominal_scenario(SEED, until_us=SHORT_RUN_US)
    b = run_nominal_scenario(SEED + 1, until_us=SHORT_RUN_US)
    # Same scenario: the same frames at the same ticks, the same modes and states.
    assert [n for n, _ in a.telemetry] == [n for n, _ in b.telemetry]
    assert [(t.mode, t.payload_state) for _, t in a.telemetry] == [
        (t.mode, t.payload_state) for _, t in b.telemetry
    ]
    assert a.acks == b.acks
    # Different noise: every noisy field differs somewhere.
    for name in SENSOR_FIELDS:
        values_a = [getattr(t, name) for _, t in a.telemetry]
        values_b = [getattr(t, name) for _, t in b.telemetry]
        assert values_a != values_b, f"{name} is identical for seeds {SEED} and {SEED + 1}"
    assert a.downlink_bytes() != b.downlink_bytes()


# --- The harness (fast) ---------------------------------------------------------------


def test_harness_runs_the_timeline_through_the_target_with_frame_loopback() -> None:
    # The first 10 s: the beacon, then PING and SET_MODE SCIENCE ACKed in their ticks,
    # with sequence numbers 1 and 2; the downlink is decoded and kept raw.
    run = run_nominal_scenario(until_us=10 * 1_000_000)
    assert run.ticks_run == 100 and run.seed == SEED
    assert [n for n, _ in run.uplink] == [tick(PING_US), tick(SCIENCE_COMMAND_US)]
    assert [(n, ack.sequence) for n, ack in run.acks] == [
        (tick(PING_US), 1),
        (tick(SCIENCE_COMMAND_US), 2),
    ]
    assert run.downlink_bytes() == b"".join(f for d in run.downlink for f in d.frames)
    assert modes_in_order(run.telemetry) == [
        (tick(BOOT_US) - 1, Mode.NOMINAL),
        (tick(SCIENCE_COMMAND_US), Mode.SCIENCE),
    ]


def test_timeline_is_the_reference_profile() -> None:
    # #72's reference profile: SCIENCE at 6 s, the pass in the last 10 minutes of the
    # orbit (the end of eclipse), and the ground returns to NOMINAL at its end.
    assert ENVIRONMENT.orbit_period_us == PASS_END_US
    assert PASS_END_US - PASS_START_US == PASS_US == 10 * 60 * 1_000_000
    assert not ENVIRONMENT.state_at(PASS_END_US - TICK_US).sunlit
    assert BOOT_US < PING_US < SCIENCE_COMMAND_US < PASS_START_US < PASS_END_US < RUN_US
