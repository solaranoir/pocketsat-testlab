"""Behaviour: reboots and the fault/command latency asymmetry (#102 scenario 3).

Driven through TestTarget only (see ``_ground.py``). BOOT sends no telemetry (only the
boot-complete beacon at its end, #55), so the wire cannot show which controls the
subsystems obey in the ticks right after a reboot; those ``then`` steps read the
controls from the test-only truth probe (``SilTarget.last_tick``). Everything else is
what the ground receives. The mechanics are unit-tested in ``tests/sil/test_sil_faults.py``
and ``tests/sil/test_boot_reset_story.py``.
"""

import dataclasses
from collections.abc import Callable

import pytest
from _ground import BOOT_TICKS, TICK_US, Ground

from pocketsat.flight import Mode, RejectReason, controls_for_mode
from pocketsat.messages import Command, CommandId
from pocketsat.spacecraft import NO_RADIO_TRAFFIC, PayloadState, SpacecraftControls
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import FORCED_RESET, TRANSMITTER_OFF

BOOT_CONTROLS = controls_for_mode(Mode.BOOT)
NOMINAL_CONTROLS = controls_for_mode(Mode.NOMINAL)
BOOT_IN_PROGRESS = RejectReason.BOOT_IN_PROGRESS


def decided(controls: SpacecraftControls) -> SpacecraftControls:
    """The controls without the radio traffic SilTarget merges in for comms (ADR-0007,
    #98): what the flight computer decided."""
    return dataclasses.replace(controls, radio_traffic=NO_RADIO_TRAFFIC)


def booted() -> Ground:
    """Given: powered on and booted into NOMINAL (attitude control on)."""
    sat = Ground.sil()
    sat.boot()
    sat.run(1)
    return sat


def reset_command(sat: Ground) -> None:
    sat.send(Command.reset())


def forced_reset(sat: Ground) -> None:
    sat.inject(TargetFault(FORCED_RESET))  # a momentary pulse


@pytest.mark.parametrize(
    "reboot", [reset_command, forced_reset], ids=["RESET command", "forced_reset pulse"]
)
def test_reboot_happens_in_tick_n_and_boot_controls_apply_from_n_plus_1(
    reboot: Callable[[Ground], None],
) -> None:
    """Scenario 3. Pins ADR-0004 §9 (RESET and ``forced_reset`` are one reboot path:
    reboot in tick N, BOOT's controls produced at step f of N, so they apply from
    N+1) and #49 (boot count +1, the 5 s boot restarts, then the beacon)."""
    # Given: NOMINAL, boot count 0
    sat = booted()
    assert sat.last_telemetry.boot_count == 0

    # When: the reboot is triggered for tick N
    reboot(sat)
    tick_n = sat.tick()

    # Then: the subsystems still obeyed NOMINAL's controls in tick N ...
    assert decided(sat.probe.controls) == NOMINAL_CONTROLS
    # ... the flight computer is already in BOOT: no telemetry, commands in N+1 refused
    assert tick_n.telemetry == ()
    tick_n1 = sat.tick(Command.set_mode(Mode.SCIENCE), Command.ping())
    assert tick_n1.answers == [(CommandId.SET_MODE, BOOT_IN_PROGRESS), (CommandId.PING, None)]
    # ... and BOOT's controls (attitude control off) apply from tick N+1
    assert decided(sat.probe.controls) == BOOT_CONTROLS
    assert not sat.probe.controls.attitude.enabled

    # ... until the restarted boot ends with a beacon that counts the reboot
    beacon = sat.run_until(lambda _: True, within_s=6)
    assert beacon.mode is Mode.NOMINAL
    assert beacon.boot_count == 1
    assert sat.ticks_run - tick_n.tick >= BOOT_TICKS


def test_forced_reset_reboots_at_the_start_of_its_tick_so_that_ticks_commands_meet_boot() -> None:
    """Scenario 3, ``forced_reset``. Pins ADR-0004 §9 and #60: the pulse reboots the
    flight computer at the start of tick N, before it handles tick N's uplink, so
    every command in that tick is handled in BOOT."""
    # Given: NOMINAL
    sat = booted()

    # When: a forced_reset pulse is injected and commands arrive in the same tick
    sat.inject(TargetFault(FORCED_RESET))
    tick_n = sat.tick(Command.set_mode(Mode.SCIENCE), Command.enter_safe_mode(), Command.ping())

    # Then: they are answered by BOOT
    assert tick_n.answers == [
        (CommandId.SET_MODE, BOOT_IN_PROGRESS),
        (CommandId.ENTER_SAFE_MODE, BOOT_IN_PROGRESS),
        (CommandId.PING, None),
    ]
    assert tick_n.telemetry == ()


def test_reset_command_is_acked_and_reboots_at_step_d_so_only_later_commands_meet_boot() -> None:
    """Scenario 3, RESET. Pins #49 / ``docs/spacecraft-modes.md`` "RESET and
    forced_reset": the RESET's ACK goes out in its tick, commands before it in that
    tick are handled in the old mode, and the reboot at step d means commands after it
    are handled in BOOT."""
    # Given: NOMINAL
    sat = booted()

    # When: SET_MODE SCIENCE, RESET, SET_MODE SCIENCE arrive in one tick
    tick_n = sat.tick(
        Command.set_mode(Mode.SCIENCE), Command.reset(), Command.set_mode(Mode.SCIENCE)
    )

    # Then: the first is accepted in NOMINAL, the RESET is ACKed, the last meets BOOT
    assert tick_n.answers == [
        (CommandId.SET_MODE, None),
        (CommandId.RESET, None),
        (CommandId.SET_MODE, BOOT_IN_PROGRESS),
    ]
    # ... and no telemetry follows: the mode after step d is BOOT
    assert tick_n.telemetry == ()


def test_transmitter_off_injected_before_tick_n_acts_in_tick_n() -> None:
    """Scenario 3, the fault half of ADR-0004 §2's asymmetry: a fault injected before
    tick N is merged at step a of tick N, so it acts in tick N (ADR-0004 §5, §10)."""
    # Given: NOMINAL, PING answered
    sat = booted()
    assert sat.tick(Command.ping()).answers == [(CommandId.PING, None)]

    # When: transmitter_off is injected for exactly one tick, and PING is sent in it
    sat.inject(TargetFault(TRANSMITTER_OFF, duration_us=TICK_US))
    tick_n = sat.tick(Command.ping())

    # Then: nothing reaches the ground in tick N: the ACK is suppressed at once
    assert tick_n.frames == ()
    # ... and the fault is released for tick N+1, so its PING is answered again
    assert sat.tick(Command.ping()).answers == [(CommandId.PING, None)]


def test_set_mode_sent_in_tick_n_takes_effect_in_tick_n_plus_1() -> None:
    """Scenario 3, the command half of ADR-0004 §2's asymmetry: a command in tick N is
    ACKed in tick N, and its controls take effect in tick N+1 (one-tick latency;
    telemetry may lag by one frame)."""
    # Given: NOMINAL, payload off
    sat = booted()

    # When: SET_MODE SCIENCE is sent in tick N
    tick_n = sat.tick(Command.set_mode(Mode.SCIENCE))

    # Then: tick N's ACK and telemetry report SCIENCE, but the payload is still off:
    # the subsystems obeyed NOMINAL's controls in tick N
    assert tick_n.answers == [(CommandId.SET_MODE, None)]
    (report_n,) = tick_n.telemetry
    assert report_n.mode is Mode.SCIENCE
    assert report_n.payload_state is PayloadState.OFF

    # When: in tick N+1 a second mode change makes the spacecraft send a frame at once
    tick_n1 = sat.tick(Command.enter_safe_mode())

    # Then: that frame shows the payload commanded on in tick N+1 (IDLE: enabled, and
    # inhibited until attitude is stabilized; OFF would mean still disabled)
    (report_n1,) = tick_n1.telemetry
    assert report_n1.mode is Mode.SAFE
    assert report_n1.payload_state is PayloadState.IDLE
