"""Story #146: downlink decoding over a SCIENCE period and a DOWNLINK pass (#158).

The ground station, the ideal link (#153) and ``SilTarget`` with the real flight
computer run in the run loop's step order (``docs/architecture.md`` §8), using only the
``TestTarget`` interface. The station commands the spacecraft itself (#154): SCIENCE
after BOOT, then BEGIN_DOWNLINK after ten minutes of payload data. The run continues
until telemetry shows the spacecraft back in SCIENCE (``DOWNLINK_COMPLETE``, #47).

The test compares what the station decoded with the raw downlink the target produced:

- every TELEMETRY frame produced a ``TelemetryReceived`` event, in order, and was
  stored in the telemetry store (#157);
- every DATA chunk was reassembled once, in ID order, with no ID missing, and its
  content verified against ``chunk_content`` (#158);
- no frame was rejected and no downlink sequence gap was seen;
- a second run with the same seed decodes and stores exactly the same records.
"""

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.flight import Mode
from pocketsat.frame import FrameType, decode_frame
from pocketsat.groundstation import (
    DataChunkReceived,
    GroundStation,
    StationEvent,
    TelemetryReceived,
)
from pocketsat.link import IdealLink
from pocketsat.messages import Command, decode_data
from pocketsat.spacecraft import chunk_content
from pocketsat.targets.sil import SilTarget

TICK_US = 100_000
SEED = 146
SCIENCE_AT_TICK = 60
"""6 s: BOOT lasts 5 s by default (#49), so SET_MODE SCIENCE is accepted."""
SCIENCE_TICKS = 10 * 60 * 10
"""Ten minutes of SCIENCE. At 40 B/s that is at most 375 64-byte chunks; acquisition
starts once the payload is acquiring, so the run with this seed produces 285."""
DOWNLINK_AT_TICK = SCIENCE_AT_TICK + SCIENCE_TICKS
MAX_DOWNLINK_TICKS = 5 * 60 * 10
"""Generous bound on the pass: the chunks go down at about one per tick."""


class Run:
    """The station's events and the raw downlink of one run."""

    def __init__(self) -> None:
        self.events: list[StationEvent] = []
        self.raw: list[bytes] = []
        self.station = GroundStation()
        self.downlink_seen = False
        self.back_in_science = False


def _run() -> Run:
    run = Run()
    station, link, target = run.station, IdealLink(), SilTarget()
    rngs = RngFactory(SEED)
    station.reset(rngs)
    link.reset(rngs)
    target.connect()
    target.reset(SEED)
    for tick in range(DOWNLINK_AT_TICK + MAX_DOWNLINK_TICKS):
        start_us, end_us = tick * TICK_US, (tick + 1) * TICK_US
        if tick == SCIENCE_AT_TICK:
            station.submit(Command.set_mode(Mode.SCIENCE))
        if tick == DOWNLINK_AT_TICK:
            station.submit(Command.begin_downlink())
        uplink = station.begin_tick(start_us, None)  # steps 3 and 4
        run.events.extend(uplink.events)
        for frame in link.carry_uplink(uplink.packets, start_us, None):
            target.send(frame)
        target.advance(TICK_US)  # step 6
        raw = target.receive()  # step 7
        run.raw.extend(raw)
        events = station.end_tick(end_us, link.carry_downlink(raw, end_us, None))  # steps 8, 9
        run.events.extend(events)
        for event in events:
            if isinstance(event, TelemetryReceived):
                if event.telemetry.mode is Mode.DOWNLINK:
                    run.downlink_seen = True
                elif run.downlink_seen and event.telemetry.mode is Mode.SCIENCE:
                    run.back_in_science = True
        if run.back_in_science:
            break
    target.close()
    return run


@pytest.fixture(scope="module")
def run() -> Run:
    return _run()


def test_the_pass_completes(run: Run) -> None:
    assert run.downlink_seen, "the spacecraft never entered DOWNLINK"
    assert run.back_in_science, "the downlink session did not complete within the bound"


def test_every_telemetry_frame_is_decoded(run: Run) -> None:
    sent = [
        frame.sequence
        for frame in map(decode_frame, run.raw)
        if frame.frame_type is FrameType.TELEMETRY
    ]
    decoded = [e.sequence for e in run.events if isinstance(e, TelemetryReceived)]
    assert sent, "no telemetry was produced"
    assert decoded == sent


def test_every_decoded_telemetry_frame_is_stored(run: Run) -> None:
    decoded = [e for e in run.events if isinstance(e, TelemetryReceived)]
    store = run.station.telemetry
    assert store.dropped_count == 0
    assert [(r.time_us, r.sequence, r.telemetry) for r in store.history] == [
        (e.time_us, e.sequence, e.telemetry) for e in decoded
    ]


def test_every_chunk_is_reassembled_once_in_order_and_verified(run: Run) -> None:
    sent = [
        decode_data(frame.payload).chunk_id
        for frame in map(decode_frame, run.raw)
        if frame.frame_type is FrameType.DATA
    ]
    received = [e for e in run.events if isinstance(e, DataChunkReceived)]
    data = run.station.data

    assert len(sent) > 250, f"expected ten minutes of SCIENCE data, got {len(sent)} chunks"
    assert [e.chunk_id for e in received] == sent
    assert all(e.content_ok and not e.duplicate for e in received)
    # Once, in order, none missing: the stored IDs are exactly the run of IDs sent.
    assert data.chunk_ids() == tuple(sent) == tuple(range(sent[0], sent[0] + len(sent)))
    for chunk_id in sent:
        assert data.chunk(chunk_id) == chunk_content(chunk_id, data.chunk_size_bytes)

    stats = data.statistics
    assert stats.stored_count == len(sent)
    assert (stats.duplicate_count, stats.corrupted_count, stats.missing_count) == (0, 0, 0)
    assert stats.stored_bytes == len(sent) * data.chunk_size_bytes
    assert stats.data_rate_bytes_per_s is not None and stats.data_rate_bytes_per_s > 0


def test_no_frame_errors_and_no_gaps(run: Run) -> None:
    stats = run.station.link_statistics
    assert stats.received_count == len(run.raw)
    assert stats.rejected_count == 0
    assert stats.missing_count == 0
    assert stats.restart_count == 0


def test_the_same_seed_decodes_the_same_records(run: Run) -> None:
    again = _run()
    assert again.events == run.events
    assert again.station.telemetry.history == run.station.telemetry.history
    assert again.station.data.chunk_ids() == run.station.data.chunk_ids()
    assert again.station.data.statistics == run.station.data.statistics
