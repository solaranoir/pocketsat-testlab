"""Story #145: command uplink, closed loop, against ``SilTarget``.

The station (uplink #154, decoding #156, correlation #155), the ideal link (#153) and
``SilTarget`` run in the run loop (#167, ``docs/architecture.md`` §8), using only the
``TestTarget`` interface. Every one of the five commands reaches a known outcome:
NACKs while the spacecraft boots, ACKs once it is up, and timeouts with retries, then
failure, while its transmitter is off.
"""

from pocketsat.environment.geometry import PassGeometry
from pocketsat.flight.modes import Mode, RejectReason
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    CommandAcked,
    CommandFailed,
    CommandNacked,
    CommandRef,
    CommandSent,
    CommandTimedOut,
    FailureReason,
    FrameRejected,
    GroundStation,
    SpacecraftRebooted,
    StationEvent,
    UnmatchedAck,
)
from pocketsat.messages import Command, CommandId
from pocketsat.orchestrator import RunLoop
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import SilTarget

TICK_US = 100_000
SEED = 42
GEOMETRY = PassGeometry(
    azimuth_deg=180.0, elevation_deg=35.0, range_km=900.0, range_rate_km_per_s=-2.0
)
SILENT_FROM_TICK = 160
"""The tick from which the spacecraft's transmitter is off (``transmitter_off``): after
the boot-complete beacon that follows the RESET at tick 90 (BOOT sends no telemetry for
5 s, ``docs/spacecraft-modes.md``), so the ground sees the reboot first."""

SCHEDULE: dict[int, list[Command]] = {
    0: [Command.ping(), Command.set_mode(Mode.SCIENCE)],  # in BOOT (5 s)
    60: [Command.set_mode(Mode.SCIENCE)],
    70: [Command.begin_downlink()],
    80: [Command.enter_safe_mode()],
    90: [Command.reset()],
    SILENT_FROM_TICK + 1: [Command.ping(), Command.reset()],
}
TICKS = 260


class FixedGeometry:
    """The spacecraft held at :data:`GEOMETRY`, in sight, for the whole run."""

    def geometry_at(self, now_us: int) -> PassGeometry:
        return GEOMETRY


def _run() -> tuple[GroundStation, list[StationEvent], list[CommandRef]]:
    """Run the schedule and return the station, every event, and the refs in order."""
    target = SilTarget()
    loop = RunLoop(target, seed=SEED, tick_us=TICK_US, geometry=FixedGeometry())
    for tick, commands in SCHEDULE.items():
        for command in commands:
            loop.schedule(tick * TICK_US, command)  # issued at step 3 of that tick
    with loop:
        for tick in range(TICKS):
            if tick == SILENT_FROM_TICK:
                target.inject(TargetFault("transmitter_off"))  # step 1, by hand until Phase 4
            loop.step()
    return loop.station, list(loop.events), [issued.command_ref for issued in loop.issued]


def _outcome(events: list[StationEvent], ref: CommandRef) -> StationEvent:
    """The event that resolved a command: its ACK, NACK or failure."""
    (outcome,) = [
        e
        for e in events
        if isinstance(e, CommandAcked | CommandNacked | CommandFailed) and e.command_ref == ref
    ]
    return outcome


def test_every_command_reaches_a_known_outcome() -> None:
    station, events, refs = _run()
    ping_in_boot, set_mode_in_boot, science, downlink, safe, reset, silent_ping, silent_reset = refs

    # During BOOT only PING is accepted (docs/spacecraft-modes.md).
    assert isinstance(_outcome(events, ping_in_boot), CommandAcked)
    nack = _outcome(events, set_mode_in_boot)
    assert isinstance(nack, CommandNacked)
    assert nack.reason is RejectReason.BOOT_IN_PROGRESS

    # Once up, every command is acknowledged in the tick it is sent (ADR-0004).
    for ref in (science, downlink, safe, reset):
        outcome = _outcome(events, ref)
        assert isinstance(outcome, CommandAcked)
        (sent,) = [e for e in events if isinstance(e, CommandSent) and e.command_ref == ref]
        assert outcome.time_us == sent.time_us + TICK_US

    # The RESET rebooted the spacecraft.
    assert any(isinstance(e, SpacecraftRebooted) for e in events)

    # With the transmitter off nothing is answered: PING is retried, then fails; RESET is
    # never retried (ADR-0010 §2).
    failed_ping = _outcome(events, silent_ping)
    assert failed_ping == CommandFailed(
        failed_ping.time_us, silent_ping, FailureReason.RETRIES_EXHAUSTED
    )
    ping_attempts = [
        e for e in events if isinstance(e, CommandSent) and e.command_ref == silent_ping
    ]
    assert [e.retry_count for e in ping_attempts] == [0, 1, 2]
    assert len({e.sequence for e in ping_attempts}) == 3
    timeout_us = DEFAULT_STATION_CONFIG.command_timeout_us
    assert failed_ping.time_us == ping_attempts[-1].time_us + timeout_us

    failed_reset = _outcome(events, silent_reset)
    assert failed_reset == CommandFailed(
        failed_reset.time_us, silent_reset, FailureReason.NOT_RETRYABLE
    )
    reset_attempts = [
        e for e in events if isinstance(e, CommandSent) and e.command_ref == silent_reset
    ]
    assert len(reset_attempts) == 1
    assert failed_reset.time_us == reset_attempts[0].time_us + timeout_us
    assert [e.will_retry for e in events if isinstance(e, CommandTimedOut)] == [
        True,
        False,  # the RESET, which times out in the same tick as the PING's first attempt
        True,
        False,
    ]

    # Nothing left waiting, nothing unexplained on the downlink.
    assert station.outstanding_commands == ()
    assert station.unmatched_ack_count == 0
    assert not any(isinstance(e, UnmatchedAck | FrameRejected) for e in events)
    assert {e.command_id for e in events if isinstance(e, CommandSent)} == set(CommandId)


def test_the_closed_loop_is_deterministic() -> None:
    assert _run()[1] == _run()[1]
