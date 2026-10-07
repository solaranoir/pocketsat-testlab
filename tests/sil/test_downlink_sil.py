"""DOWNLINK data transfer end to end through ``SilTarget`` (#56; #102 scenario 2).

The real flight computer and the five real subsystems, driven the way the ground will
drive them: COMMAND frames in with ``send()``, ``advance()`` one tick at a time, and
the downlink drained with ``receive()``. The ground side decodes every frame and checks
every DATA frame against the chunk content function (#43). The tests read the payload's
state from ``SilTarget.last_tick`` (truth, which equals readings for the payload) only
to check the accounting, never to drive the run.

Checked in every tick of every test (``_downlink_ground.Ground.tick``):

- **Released only once transmitted** (#56's tightened criterion): every chunk the
  payload has released was in a DATA frame that ``receive()`` returned in an earlier
  tick. No timer-based release.
- **Data accounting** (story #42): produced = buffered + released, and the bytes of the
  chunks sent minus the bytes released are exactly the chunks sent in this tick, so
  never more than one tick's worth.
- **Radio-transmit inhibits** (story #42): no DATA in a tick whose power or thermal
  readings carry a flag.
- **Order:** each chunk is received once, in ID order, with the content its ID gives.

The transmitter-off case (#60) is in ``test_sil_faults.py``; the three-orbit run of the
reference profile, in ``test_downlink_story.py`` (slow).
"""

from itertools import pairwise

import pytest
from _downlink_ground import CHUNK, TICK, Ground, payload_with

from pocketsat.flight import Mode
from pocketsat.messages import Command, CommandAck, CommandId
from pocketsat.spacecraft import PowerInitial, SpacecraftInitialState, ThermalInitial
from pocketsat.targets.base import EnvironmentState, TargetFault
from pocketsat.targets.sil import FORCED_RESET


def booted(
    chunks: int, *, thermal: ThermalInitial | None = None, power: PowerInitial | None = None
) -> Ground:
    """A ground with ``chunks`` stored chunks, booted into NOMINAL (1 s boot)."""
    initial = SpacecraftInitialState(
        payload=payload_with(chunks),
        thermal=thermal or ThermalInitial(),
        power=power or PowerInitial(),
    )
    ground = Ground(initial)
    ground.tick()
    assert ground.payload().next_chunk_id == chunks
    ground.until(Mode.NOMINAL)
    return ground


@pytest.mark.parametrize("entry", [Mode.NOMINAL, Mode.SCIENCE], ids=lambda m: m.name)
def test_begin_downlink_sends_every_chunk_releases_it_after_transmission_and_returns(
    entry: Mode,
) -> None:
    # #102 scenario 2: BEGIN_DOWNLINK with data buffered leads to DOWNLINK and DATA
    # frames; chunks are released only after their frames were transmitted.
    ground = booted(60)
    if entry is Mode.SCIENCE:
        ground.tick(Command.set_mode(Mode.SCIENCE))
    ground.tick(Command.begin_downlink())
    assert ground.mode is Mode.DOWNLINK
    assert ground.acks[-1] == CommandAck(ground.sequence, CommandId.BEGIN_DOWNLINK)
    start = len(ground.ticks)

    ground.until(entry)
    total = ground.payload().next_chunk_id
    assert total >= 60  # SCIENCE may have added a chunk before the payload went off
    assert ground.order == list(range(total))  # all of them, in order, once each
    payload = ground.payload()
    assert payload.oldest_unreleased_chunk_id == total
    assert payload.buffered_bytes < CHUNK  # only the partial chunk is left
    # About one DATA frame per tick at the default capacity, and telemetry once a
    # second in DOWNLINK, then a frame in the step DOWNLINK_COMPLETE returns.
    pass_ticks = len(ground.ticks) - start
    assert total <= pass_ticks <= total + 2
    downlink = [n for n, t in ground.telemetry if t.mode is Mode.DOWNLINK]
    assert all(b - a == 10 for a, b in pairwise(downlink))
    assert len(downlink) >= pass_ticks // 10
    assert ground.telemetry[-1][1].mode is entry
    # The payload stays off through the pass and the transmitter carried the DATA.
    pass_records = ground.ticks[start:]
    assert not any(t.controls.payload.enabled for t in pass_records)
    assert sum(t.traffic.sent_bytes for t in pass_records) >= total * 78


def test_a_power_flag_mid_pass_stops_data_in_that_tick_and_nothing_is_lost() -> None:
    # low_battery is a radio-transmit inhibit (#56) but not a safe-mode flag (#48), so
    # the pass pauses in DOWNLINK and resumes when the flag clears.
    ground = booted(80)
    ground.tick(Command.begin_downlink())
    for _ in range(20):
        ground.tick()
    assert ground.order and ground.flags() == (False, False)

    low = EnvironmentState(battery_soc_override=0.2)
    flagged_at = None
    for _ in range(50):
        sent = ground.tick(env=low)
        if ground.flags()[0]:
            flagged_at = ground.count - 1
            assert sent == []  # DATA stops in the tick the flag appears
            break
        assert sent
    assert flagged_at is not None
    paused = ground.payload()
    for _ in range(5):
        assert ground.tick() == []
    assert ground.mode is Mode.DOWNLINK
    # Nothing lost: all the stored chunks are still there, minus those sent.
    assert ground.payload().oldest_unreleased_chunk_id == len(ground.order)
    assert ground.payload().total_produced_bytes == paused.total_produced_bytes

    ground.tick(env=EnvironmentState(battery_soc_override=0.6))
    ground.until(Mode.NOMINAL, env=EnvironmentState())
    assert ground.order == list(range(80))


def test_a_thermal_flag_mid_pass_stops_data_safe_aborts_and_the_next_pass_resumes() -> None:
    # over_temp inhibits DATA in the tick it appears; sustained, it takes the
    # spacecraft to SAFE (#48), which aborts the session. Nothing is lost, and the
    # next session resumes from the oldest unreleased chunk.
    ground = booted(400, thermal=ThermalInitial(battery_c=43.0, electronics_c=30.0))
    ground.tick(Command.begin_downlink())
    hot = EnvironmentState(ambient_temp_c=150.0)
    flagged_at = None
    for _ in range(300):
        sent = ground.tick(env=hot)
        if ground.flags()[1]:
            flagged_at = ground.count - 1
            assert sent == []
            break
    assert flagged_at is not None
    assert ground.data_per_tick[-2], "DATA was flowing in the tick before the flag"

    ground.until(Mode.SAFE)
    assert ground.computers[-1].downlink_session is None
    sent_before = len(ground.order)
    assert ground.payload().oldest_unreleased_chunk_id == sent_before  # nothing lost

    cold = EnvironmentState(ambient_temp_c=-20.0)
    for _ in range(3000):
        ground.tick(env=cold)
        if ground.flags() == (False, False):
            break
    ground.tick(Command.set_mode(Mode.NOMINAL), env=EnvironmentState())
    assert ground.mode is Mode.NOMINAL
    ground.tick(Command.begin_downlink())
    session = ground.computers[-1].downlink_session
    assert session is not None and session.start_chunk_id == sent_before
    ground.until(Mode.NOMINAL)
    assert ground.order == list(range(400))


@pytest.mark.parametrize("how", ["RESET command", "forced_reset pulse", "forced_reset hold"])
def test_a_reboot_mid_pass_aborts_and_the_next_pass_resumes(how: str) -> None:
    ground = booted(50)
    ground.tick(Command.begin_downlink())
    for _ in range(15):
        ground.tick()
    if how == "RESET command":
        ground.tick(Command.reset())
    elif how == "forced_reset pulse":
        ground.target.inject(TargetFault(FORCED_RESET))
        ground.tick()
    else:
        ground.target.inject(TargetFault(FORCED_RESET, duration_us=5 * TICK))
        for _ in range(6):
            ground.tick()  # held: the held DOWNLINK controls release nothing new
    assert ground.mode is Mode.BOOT
    sent_before = len(ground.order)
    assert ground.payload().oldest_unreleased_chunk_id == sent_before

    ground.until(Mode.NOMINAL)
    ground.tick(Command.begin_downlink())
    ground.until(Mode.NOMINAL)
    assert ground.order == list(range(50))
    assert ground.payload().total_released_bytes == 50 * CHUNK
