"""Behaviour: BEGIN_DOWNLINK with payload data buffered (#102 scenario 2).

Driven through TestTarget only (see ``_ground.py``). The data comes from the payload
itself: the spacecraft boots, is commanded to SCIENCE, and acquires once attitude
control has stabilized it (about 2.5 min of simulated time from power-on, about 0.2 s
of wall-clock time). The session's per-tick accounting, checked against the payload's
true state, is in ``tests/sil/test_downlink_sil.py`` (#56) and over three orbits in
``tests/sil/test_downlink_story.py``; here the ground sees only frames.
"""

from _ground import Downlink, Ground

from pocketsat.flight import Mode
from pocketsat.messages import Command, CommandId
from pocketsat.spacecraft import NOMINAL_CONFIG, PayloadState, chunk_content

CHUNK = NOMINAL_CONFIG.payload.chunk_size_bytes
"""Payload chunk size, bytes (default 64): one DATA frame carries one chunk."""

CHUNKS_WANTED = 5
"""Whole chunks to buffer before the pass."""


def with_payload_data() -> tuple[Ground, int]:
    """Given: booted, in SCIENCE, until telemetry reports at least ``CHUNKS_WANTED``
    whole chunks buffered. Returns the ground and the reported buffered bytes."""
    sat = Ground.sil()
    sat.boot()
    sat.command_accepted(Command.set_mode(Mode.SCIENCE))
    report = sat.run_until(lambda t: t.buffered_bytes >= CHUNKS_WANTED * CHUNK, within_s=300)
    assert report.payload_state is PayloadState.ACQUIRING
    return sat, report.buffered_bytes


def test_begin_downlink_sends_buffered_chunks_as_data_and_releases_them_only_once_sent() -> None:
    """Scenario 2. Pins #56 (BEGIN_DOWNLINK -> DOWNLINK, one chunk per DATA frame in ID
    order with its content, release only once sent, DOWNLINK_COMPLETE returns to the
    entry mode), ADR-0004 §13 (the payload owns the data; the flight computer releases
    it) and #51 (ACK and the mode-change telemetry in the command's tick)."""
    # Given: payload data buffered in SCIENCE
    sat, _ = with_payload_data()

    # When: the ground starts a pass
    entry = sat.tick(Command.begin_downlink())

    # Then: ACKed, the same tick's telemetry reports DOWNLINK, and no DATA goes out yet
    assert entry.answers == [(CommandId.BEGIN_DOWNLINK, None)]
    assert [t.mode for t in entry.telemetry] == [Mode.DOWNLINK]
    buffered_at_entry = entry.telemetry[0].buffered_bytes
    whole_chunks = buffered_at_entry // CHUNK
    assert whole_chunks >= CHUNKS_WANTED

    # ... DATA frames follow until the pass completes and returns to SCIENCE
    pass_ticks: list[Downlink] = []
    for downlink in sat.ticking(30):
        pass_ticks.append(downlink)
        if any(t.mode is Mode.SCIENCE for t in downlink.telemetry):
            break
    else:
        raise AssertionError("the pass did not complete within 30 s")
    received = [chunk for downlink in pass_ticks for chunk in downlink.data]

    # ... every whole buffered chunk arrives once, in ID order, with the content its ID
    # gives (the partial chunk still accumulating is not sent)
    ids = [chunk.chunk_id for chunk in received]
    assert len(ids) == whole_chunks
    assert ids == list(range(ids[0], ids[0] + whole_chunks))
    assert all(chunk.content == chunk_content(chunk.chunk_id, CHUNK) for chunk in received)

    # ... and the payload never released a chunk before its DATA frame reached the
    # ground: in DOWNLINK the payload is off, so the buffer only shrinks by releases,
    # and every telemetry frame still holds all the bytes not yet received in an
    # earlier tick
    received_before = 0
    for downlink in pass_ticks:
        for report in downlink.telemetry:
            if report.mode is Mode.DOWNLINK:
                assert report.buffered_bytes >= buffered_at_entry - received_before * CHUNK
        received_before += len(downlink.data)
    # ... and it did release them once sent: the frame that reports the return to
    # SCIENCE (sent in the completion tick, payload still off) shows only the partial
    # chunk left
    completion = pass_ticks[-1].telemetry[-1]
    assert completion.mode is Mode.SCIENCE
    assert completion.buffered_bytes < CHUNK
