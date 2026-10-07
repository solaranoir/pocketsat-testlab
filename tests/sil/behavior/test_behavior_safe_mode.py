"""Behaviour: SAFE mode, entered automatically and by command (#102 scenarios 1 and 7).

Driven through TestTarget only (see ``_ground.py``). The safe-mode rules themselves are
table-tested in ``tests/unit/test_safety.py`` and run with the real subsystems in
``tests/sil/test_safe_mode_story.py``; these tests pin what the ground sees.
"""

import pytest
from _ground import Ground

from pocketsat.flight import Mode, RejectReason
from pocketsat.flight.safety import DEFAULT_SAFETY_CONFIG
from pocketsat.messages import Command, CommandId, TelemetryFlags
from pocketsat.spacecraft import PayloadState, RadioMode
from pocketsat.targets.base import EnvironmentState

SUSTAIN_TICKS = DEFAULT_SAFETY_CONFIG.sustain_tick_count
"""Consecutive flagged ticks that raise ``SAFE_CONDITION`` (#48; default 10, 1 s)."""

CRITICAL = EnvironmentState(battery_soc_override=0.05)
"""A battery well below ``critical_battery_soc`` (0.15), further than the noise bound."""

HEALTHY = EnvironmentState(battery_soc_override=0.6)
"""A battery well above every flag's clear threshold (0.35 at most)."""

SAFE_FLAGS = TelemetryFlags.critical_battery | TelemetryFlags.over_temp | TelemetryFlags.under_temp


def in_science() -> Ground:
    """Given: booted, then commanded to SCIENCE, so the payload is commanded on."""
    sat = Ground.sil()
    sat.boot()
    sat.command_accepted(Command.set_mode(Mode.SCIENCE))
    return sat


def test_sustained_critical_battery_enters_safe_and_keeps_attitude_control_and_the_radio() -> None:
    """Scenario 1. Pins #48 (critical battery sustained -> SAFE_CONDITION) and #47 /
    ``docs/spacecraft-modes.md`` "Mode to controls" (SAFE: payload off, attitude
    control on, radio RX_TX in every mode, ADR-0004 §10)."""
    # Given: SCIENCE, payload commanded on (IDLE while detumbling inhibits acquisition)
    sat = in_science()
    sat.run(1)
    assert sat.last_telemetry.payload_state is not PayloadState.OFF

    # When: the battery is held critically low
    sat.apply_environment(CRITICAL)
    safe = sat.run_until(lambda t: t.mode is Mode.SAFE, within_s=3)

    # Then: telemetry reports SAFE, with the flag that caused it
    assert TelemetryFlags.critical_battery in safe.flags
    after = [t for downlink in sat.run(30) for t in downlink.telemetry]
    assert after, "SAFE keeps sending telemetry"
    assert all(t.mode is Mode.SAFE for t in after)
    # ... the radio is still transmitting (these frames arrived), and reports RX_TX
    assert all(t.radio_mode is RadioMode.RX_TX for t in [safe, *after])
    # ... the payload is off, not merely inhibited (SAFE's controls apply from the next tick)
    assert all(t.payload_state is PayloadState.OFF for t in after)
    # ... and attitude control is on: the pointing error keeps converging (with control
    # off, as in BOOT, it drifts upward)
    assert after[-1].pointing_error_deg < after[0].pointing_error_deg - 3.0
    # ... and the ground can still command it: PING is answered in SAFE
    assert sat.tick(Command.ping()).answers == [(CommandId.PING, None)]


@pytest.mark.parametrize(
    ("flagged_ticks", "expected"),
    [(SUSTAIN_TICKS - 1, Mode.SCIENCE), (SUSTAIN_TICKS, Mode.SAFE)],
    ids=["one-tick-short-of-sustained", "sustained"],
)
def test_critical_battery_must_be_sustained_to_enter_safe(
    flagged_ticks: int, expected: Mode
) -> None:
    """Scenario 1, "sustained". Pins #48: SAFE_CONDITION needs ``sustain_tick_count``
    consecutive ticks of the flag; a shorter dip is ignored."""
    # Given: SCIENCE with a healthy battery
    sat = in_science()
    sat.apply_environment(HEALTHY)
    sat.run(1)

    # When: the battery dips critically low for `flagged_ticks` ticks, then recovers
    sat.apply_environment(CRITICAL)
    for _ in range(flagged_ticks):
        sat.tick()
    sat.apply_environment(HEALTHY)
    sat.run(3)

    # Then: only the sustained dip entered SAFE (the two runs differ in one flagged
    # tick, so the shorter one is not let off by a flag that never appeared)
    assert sat.last_telemetry.mode is expected


def test_enter_safe_mode_by_command_is_acked_and_set_mode_nominal_leaves_it() -> None:
    """Scenario 7. Pins #51 (ENTER_SAFE_MODE ACKed, telemetry from the ACK tick shows
    the new mode, ADR-0004 §2), #47 (nothing automatic leaves SAFE; SCIENCE and
    BEGIN_DOWNLINK are refused NOT_ALLOWED_IN_SAFE) and #48 (SET_MODE NOMINAL leaves
    SAFE while no safe-mode flag is set)."""
    # Given: SCIENCE, healthy, no safe-mode flag set
    sat = in_science()
    assert not sat.last_telemetry.flags & SAFE_FLAGS

    # When: the ground commands SAFE
    entry = sat.tick(Command.enter_safe_mode())

    # Then: the command is ACKed and the same tick's telemetry reports SAFE
    assert entry.answers == [(CommandId.ENTER_SAFE_MODE, None)]
    assert [t.mode for t in entry.telemetry] == [Mode.SAFE]
    # ... and it stays in SAFE: nothing automatic leaves it
    sat.run(5)
    assert sat.last_telemetry.mode is Mode.SAFE
    # ... where the way up is SET_MODE NOMINAL first
    refused = sat.tick(Command.set_mode(Mode.SCIENCE), Command.begin_downlink())
    assert refused.answers == [
        (CommandId.SET_MODE, RejectReason.NOT_ALLOWED_IN_SAFE),
        (CommandId.BEGIN_DOWNLINK, RejectReason.NOT_ALLOWED_IN_SAFE),
    ]

    # When: the ground commands NOMINAL with the flags clear
    leave = sat.tick(Command.set_mode(Mode.NOMINAL))

    # Then: ACKed, and telemetry from that tick reports NOMINAL
    assert leave.answers == [(CommandId.SET_MODE, None)]
    assert [t.mode for t in leave.telemetry] == [Mode.NOMINAL]
    # ... and normal operations resume
    science = sat.command_accepted(Command.set_mode(Mode.SCIENCE))
    assert [t.mode for t in science.telemetry] == [Mode.SCIENCE]


def test_set_mode_nominal_from_safe_is_refused_until_the_safe_flags_clear() -> None:
    """Scenario 7, "once flags are clear". Pins #48's exit guard: SET_MODE NOMINAL in
    SAFE is NACKed SAFE_CONDITIONS_ACTIVE while any safe-mode flag is set in that
    tick's readings, whichever way SAFE was entered, and accepted once they clear."""
    # Given: SAFE by command, then the battery becomes critically low
    sat = Ground.sil()
    sat.boot()
    sat.command_accepted(Command.enter_safe_mode())
    sat.apply_environment(CRITICAL)
    sat.run_until(lambda t: TelemetryFlags.critical_battery in t.flags, within_s=2)

    # When: the ground commands NOMINAL while the flag is set
    refused = sat.tick(Command.set_mode(Mode.NOMINAL))

    # Then: NACKed, and the spacecraft stays in SAFE
    assert refused.answers == [(CommandId.SET_MODE, RejectReason.SAFE_CONDITIONS_ACTIVE)]
    sat.run(1)
    assert sat.last_telemetry.mode is Mode.SAFE

    # When: the battery recovers and telemetry shows the flags clear
    sat.apply_environment(HEALTHY)
    clear = sat.run_until(lambda t: not t.flags & SAFE_FLAGS, within_s=2)
    assert clear.mode is Mode.SAFE  # clearing alone does not leave SAFE
    leave = sat.tick(Command.set_mode(Mode.NOMINAL))

    # Then: SET_MODE NOMINAL is accepted and leaves SAFE
    assert leave.answers == [(CommandId.SET_MODE, None)]
    assert [t.mode for t in leave.telemetry] == [Mode.NOMINAL]
