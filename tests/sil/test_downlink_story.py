"""Three orbits of the reference profile with the real flight computer's downlink (#56).

Story #42's two criteria that waited for a flight computer, now with one: "with a flight
computer present, bytes sent minus bytes released is never more than one tick's worth",
and "radio transmit respects the power and thermal inhibits". The story test itself
(``test_payload_comms_story.py``) keeps its scripted release: it runs the subsystem
stack with the payload commanded on for the whole run, which a flight computer would
never do, and so covers acquisition against its inhibits for three full orbits.

Here ``SilTarget`` runs the five real subsystems and the real flight computer at its
defaults (5 s boot), driven only through the ``TestTarget`` interface, with
``NominalEnvironment`` applied before every tick, for three full orbits of #72's
reference profile: SCIENCE for the orbit, and BEGIN_DOWNLINK 10 minutes before the end
of each orbit (the end of eclipse). The flight computer sends every stored chunk, then
``DOWNLINK_COMPLETE`` returns it to SCIENCE (#47).

Every tick, ``_downlink_ground.Ground`` checks that chunks are released only once their
DATA frame was received, produced = buffered + released, sent - released is exactly the
chunks sent in that tick (never more than one tick's worth), no DATA goes out while a
power or thermal flag is set in the readings, and every chunk arrives once, in order,
with its content. This test adds, per pass, that the session completes inside the
10-minute pass and leaves only the partial chunk in the buffer, so the buffer does not
grow from orbit to orbit (#72's data budget), and that the spacecraft never enters SAFE
or FAULT.
"""

from dataclasses import dataclass, field

import pytest
from _downlink_ground import CHUNK, TICK, Ground

from pocketsat.environment import NominalEnvironment
from pocketsat.flight import DEFAULT_FLIGHT_COMPUTER_CONFIG, Mode
from pocketsat.messages import Command

# Multi-orbit story run: skipped by a plain `pytest`, run with `pytest -m slow`.
pytestmark = pytest.mark.slow

ORBITS = 3
PASS_TICKS = 10 * 60 * 10
"""The 10-minute pass at the end of each orbit, in 100 ms ticks (#72)."""


@dataclass
class Pass:
    """One DOWNLINK pass."""

    start_tick: int
    chunks: int = 0
    end_tick: int | None = None
    buffered_after: int | None = None


@dataclass
class Run:
    passes: list[Pass] = field(default_factory=list)
    modes: set[Mode] = field(default_factory=set)
    inhibited_ticks: int = 0
    ground: Ground | None = None


def _reference_run(seed: int) -> Run:
    environment = NominalEnvironment()
    ticks_per_orbit = environment.orbit_period_us // TICK
    assert ticks_per_orbit * TICK == environment.orbit_period_us
    ground = Ground(config=DEFAULT_FLIGHT_COMPUTER_CONFIG, seed=seed, record_ticks=False)
    run = Run(ground=ground)
    current: Pass | None = None

    for n in range(ORBITS * ticks_per_orbit):
        commands: list[Command] = []
        if n == 60:  # BOOT lasts 5 s (50 ticks)
            commands.append(Command.set_mode(Mode.SCIENCE))
        if n % ticks_per_orbit == ticks_per_orbit - PASS_TICKS:
            commands.append(Command.begin_downlink())
            current = Pass(start_tick=n)
            run.passes.append(current)
        sent = ground.tick(*commands, env=environment.state_at(ground.target.now_us))
        run.modes.add(ground.mode)
        run.inhibited_ticks += ground.flags() != (False, False)
        if current is not None:
            current.chunks += len(sent)
            if current.end_tick is None and ground.mode is Mode.SCIENCE and n > current.start_tick:
                current.end_tick = n
                current.buffered_after = ground.payload().buffered_bytes
    return run


@pytest.fixture(scope="module")
def reference() -> Run:
    return _reference_run(seed=42)


def test_every_pass_sends_all_stored_chunks_and_returns_to_science(reference: Run) -> None:
    assert len(reference.passes) == ORBITS
    for downlink_pass in reference.passes:
        assert downlink_pass.end_tick is not None, "the session never completed"
        # Complete inside the 10-minute pass (#72: about 6 minutes are used).
        assert downlink_pass.end_tick - downlink_pass.start_tick < PASS_TICKS
        assert downlink_pass.chunks > 0
        # Only the partial chunk is left: the buffer does not grow from pass to pass.
        assert downlink_pass.buffered_after is not None
        assert downlink_pass.buffered_after < CHUNK


def test_every_chunk_produced_before_the_last_pass_was_received_once_in_order(
    reference: Run,
) -> None:
    ground = reference.ground
    assert ground is not None
    assert ground.order == list(range(len(ground.order)))
    total = sum(p.chunks for p in reference.passes)
    assert len(ground.order) == total
    # Over three orbits of SCIENCE at 40 B/s, roughly 10 000 chunks.
    assert total * CHUNK > 2.5 * 200_000


def test_the_reference_profile_never_enters_safe_or_fault(reference: Run) -> None:
    assert reference.modes == {Mode.BOOT, Mode.NOMINAL, Mode.SCIENCE, Mode.DOWNLINK}
    assert reference.inhibited_ticks == 0  # nominal: no power or thermal flag (#72)
