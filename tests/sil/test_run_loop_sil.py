"""The run loop (#167) against ``SilTarget``: one full pass, and the Phase 1 harness.

**A full pass.** The default orbit is placed (``OrbitGeometry.with_pass``) so the
default station sees a 60° pass peaking 6 minutes into the run: above the horizon from
about 54 s to 666 s, above the 10° mask from about 177 s to 543 s. The loop runs the
station, the ideal link and ``SilTarget`` with the real flight computer for the whole
pass and 30 s after it. The station sends a PING before the spacecraft rises (the link
drops it and every retry), then at AOS a PING and SET_MODE SCIENCE, and three minutes
later BEGIN_DOWNLINK. The test checks the pass state against the prediction, every
command's outcome, that the link carried exactly the frames sent while the spacecraft
was above the horizon, and that the data and telemetry arrived whole.

**The Phase 1 nominal harness** (``tests/sil/_nominal_scenario.py``, #63) stays as it
is (#167): its golden capture (#64) numbers the uplink from 1, and the station numbers
it from 0 (ADR-0010 §2), so on the loop the four ACK frames, which echo the uplink
sequence, would differ. The slow test below runs the nominal timeline on the loop and
checks that this is the only difference from the stored capture.
"""

from dataclasses import dataclass, replace
from typing import Final

import pytest
from _golden import SCENARIOS, load
from _nominal_scenario import RUN_US, nominal_timeline

from pocketsat.environment import OrbitGeometry, PredictedPass
from pocketsat.flight import Mode
from pocketsat.frame import FrameType, decode_frame, encode_frame_fields
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    Aos,
    CommandAcked,
    CommandFailed,
    DataChunkReceived,
    FailureReason,
    FrameRejected,
    Los,
    SequenceGap,
    StationEvent,
    TelemetryReceived,
)
from pocketsat.link import IdealLink
from pocketsat.messages import Command, decode_ack, encode_ack
from pocketsat.orchestrator import RunLoop, RunResult, TickResult
from pocketsat.targets.sil import SilTarget

TICK_US: Final = 100_000
SEED: Final = 167
US_PER_S: Final = 1_000_000

GEOMETRY: Final = OrbitGeometry.with_pass(60.0, 360 * US_PER_S)
"""The default orbit and station, with a 60° pass peaking at 6 minutes."""

HORIZON_PASS: Final = GEOMETRY.next_pass(0, resolution_us=1_000)
MASK_PASS: Final = GEOMETRY.next_pass(
    0, min_elevation_deg=DEFAULT_STATION_CONFIG.elevation_mask_deg, resolution_us=1_000
)


def _tick_at_or_after(at_us: int) -> int:
    return -(-at_us // TICK_US) * TICK_US


def _pass(predicted: PredictedPass | None) -> PredictedPass:
    assert predicted is not None
    return predicted


AOS_US: Final = _tick_at_or_after(_pass(MASK_PASS).aos_us)
EARLY_PING_US: Final = 10 * US_PER_S
"""Before the spacecraft rises: the link drops the PING and both retries."""
DOWNLINK_US: Final = AOS_US + 180 * US_PER_S
END_US: Final = _tick_at_or_after(_pass(HORIZON_PASS).los_us) + 30 * US_PER_S


@dataclass
class Run:
    result: RunResult
    ticks: list[TickResult]
    link: IdealLink


def _run() -> Run:
    link = IdealLink()
    loop = RunLoop(SilTarget(), seed=SEED, link=link, geometry=GEOMETRY)
    loop.schedule(EARLY_PING_US, Command.ping())
    loop.schedule(AOS_US, Command.ping())
    loop.schedule(AOS_US, Command.set_mode(Mode.SCIENCE))
    loop.schedule(DOWNLINK_US, Command.begin_downlink())
    ticks: list[TickResult] = []
    with loop:
        result = loop.run(END_US, stop_when=lambda tick: ticks.append(tick) is not None)
    return Run(result, ticks, link)


@pytest.fixture(scope="module")
def run() -> Run:
    return _run()


def _events(run: Run) -> tuple[StationEvent, ...]:
    return run.result.events


def test_the_pass_is_seen_when_predicted(run: Run) -> None:
    mask = _pass(MASK_PASS)
    (aos,) = [e for e in _events(run) if isinstance(e, Aos)]
    (los,) = [e for e in _events(run) if isinstance(e, Los)]
    assert aos.pass_id == los.pass_id == 1
    # Sampled at tick starts: the first tick at or after each crossing.
    assert 0 <= aos.time_us - mask.aos_us < TICK_US
    assert 0 <= los.time_us - mask.los_us < TICK_US
    assert 0.0 <= mask.max_elevation_deg - los.max_elevation_deg < 0.01
    assert run.result.end_us == END_US
    assert run.result.tick_count == END_US // TICK_US


def test_every_command_reaches_its_outcome(run: Run) -> None:
    early, ping, science, downlink = run.result.issued
    outcomes = {
        e.command_ref: e for e in _events(run) if isinstance(e, CommandAcked | CommandFailed)
    }
    # Below the horizon the PING and both its retries are dropped by the link.
    assert outcomes[early.command_ref] == CommandFailed(
        outcomes[early.command_ref].time_us, early.command_ref, FailureReason.RETRIES_EXHAUSTED
    )
    assert run.link.uplink_dropped_count == 3
    # In the pass each is acknowledged in the tick it was sent (ADR-0004).
    for issued in (ping, science, downlink):
        acked = outcomes[issued.command_ref]
        assert isinstance(acked, CommandAcked)
        assert acked.time_us == issued.issued_us + TICK_US
    assert run.result.issued[1].issued_us == AOS_US


def test_the_link_carries_exactly_the_frames_sent_above_the_horizon(run: Run) -> None:
    horizon = _pass(HORIZON_PASS)

    def visible(tick: TickResult) -> bool:
        assert tick.geometry is not None
        return tick.geometry.elevation_deg >= 0.0

    sent_visible = sum(len(t.downlink) for t in run.ticks if visible(t))
    sent_hidden = sum(len(t.downlink) for t in run.ticks if not visible(t))
    assert sent_hidden > 0  # the boot beacon and NOMINAL telemetry before the pass
    assert run.link.downlink_dropped_count == sent_hidden
    received = [t for t in run.ticks if any(isinstance(e, TelemetryReceived) for e in t.events)]
    assert received
    assert all(horizon.aos_us <= t.start_us < horizon.los_us + TICK_US for t in received)
    frames_decoded = sum(
        1
        for e in _events(run)
        if isinstance(e, TelemetryReceived | CommandAcked | DataChunkReceived)
    )
    assert frames_decoded == sent_visible


def test_the_downlink_completes_whole(run: Run) -> None:
    received = [e for e in _events(run) if isinstance(e, DataChunkReceived)]
    assert len(received) > 50
    assert all(e.content_ok and not e.duplicate for e in received)
    stats = run.result.data.statistics
    assert (stats.stored_count, stats.missing_count) == (len(received), 0)
    modes = [e.telemetry.mode for e in _events(run) if isinstance(e, TelemetryReceived)]
    assert Mode.DOWNLINK in modes
    assert modes[-1] is Mode.SCIENCE  # DOWNLINK_COMPLETE returned it to SCIENCE (#56)


def test_telemetry_is_stored_as_received(run: Run) -> None:
    decoded = [e for e in _events(run) if isinstance(e, TelemetryReceived)]
    store = run.result.telemetry
    assert [(r.time_us, r.sequence, r.telemetry) for r in store.history] == [
        (e.time_us, e.sequence, e.telemetry) for e in decoded
    ]
    # One unbroken pass: the first frame heard sets the expected sequence (#156), and
    # nothing after it is lost or malformed.
    assert not any(isinstance(e, FrameRejected | SequenceGap) for e in _events(run))


def test_a_full_pass_is_deterministic(run: Run) -> None:
    again = _run()
    assert again.result.events == run.result.events
    assert [t.downlink for t in again.ticks] == [t.downlink for t in run.ticks]


@pytest.mark.slow
def test_the_nominal_scenario_on_the_loop_differs_from_its_golden_only_in_ack_sequence() -> None:
    # The evidence for leaving the Phase 1 harness as it is (#167, module docstring). One
    # orbit on the loop, about 6 s.
    loop = RunLoop(SilTarget(), seed=SCENARIOS["nominal_seed42"].seed)
    for scheduled in nominal_timeline():
        loop.schedule(scheduled.at_us, scheduled.command)
    frames: list[tuple[int, bytes]] = []
    with loop:
        while loop.now_us < RUN_US:
            tick = loop.step()
            frames.extend((tick.tick_id, frame) for frame in tick.downlink)
    golden = load(SCENARIOS["nominal_seed42"]).frames
    assert len(frames) == len(golden)
    differences = [
        (ours, theirs) for ours, theirs in zip(frames, golden, strict=True) if ours != theirs
    ]
    assert len(differences) == len(nominal_timeline()) == 4  # one ACK per command
    for (tick, ours), (golden_tick, theirs) in differences:
        assert tick == golden_tick
        ours_frame, theirs_frame = decode_frame(ours), decode_frame(theirs)
        assert ours_frame.frame_type is theirs_frame.frame_type is FrameType.ACK
        ours_ack, theirs_ack = decode_ack(ours_frame.payload), decode_ack(theirs_frame.payload)
        # The station's uplink sequence starts at 0, the harness's at 1.
        assert replace(ours_ack, sequence=ours_ack.sequence + 1) == theirs_ack
        assert ours == encode_frame_fields(FrameType.ACK, ours_frame.sequence, encode_ack(ours_ack))
