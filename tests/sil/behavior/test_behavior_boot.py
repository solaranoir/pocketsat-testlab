"""Behaviour: commands during BOOT (#102 scenario 5).

Driven through TestTarget only (see ``_ground.py``). Every command in every mode is
table-tested on the flight computer in ``tests/unit/test_command_dispatcher.py``; the
tick-0 answers through ``SilTarget`` are in ``tests/sil/test_command_dispatch_sil.py``.
Here the ground sends every command at several points of the 5 s boot, including the
boot that follows a RESET, and checks the boot is unaffected.
"""

import pytest
from _ground import BOOT_TICKS, Ground

from pocketsat.flight import Mode, RejectReason
from pocketsat.messages import Command, CommandId

REFUSED = (
    Command.set_mode(Mode.NOMINAL),
    Command.set_mode(Mode.SCIENCE),
    Command.begin_downlink(),
    Command.enter_safe_mode(),
)
"""Every valid command but PING and RESET."""


def boot_until(sat: Ground, ticks_into_boot: int) -> None:
    """Given: advance to ``ticks_into_boot`` ticks into the current boot."""
    for _ in range(ticks_into_boot):
        assert sat.tick().telemetry == ()  # BOOT sends no telemetry


@pytest.mark.parametrize(
    ("after_reset", "ticks_into_boot"),
    [
        (False, 0),
        (False, BOOT_TICKS // 2),
        (False, BOOT_TICKS - 1),
        (True, 0),
        (True, BOOT_TICKS - 1),
    ],
    ids=[
        "power-on-first-tick",
        "power-on-mid-boot",
        "power-on-last-boot-tick",
        "after-RESET-first-tick",
        "after-RESET-last-boot-tick",
    ],
)
def test_commands_during_boot_are_nacked_except_ping_and_reset(
    after_reset: bool, ticks_into_boot: int
) -> None:
    """Scenario 5. Pins #51 and #47 (in BOOT only PING and RESET are accepted; every
    other command is NACKed ``BOOT_IN_PROGRESS``) and #49 (BOOT lasts 5 s from power-on
    or a reboot; the last BOOT tick, which raises ``BOOT_COMPLETE``, handles its
    commands at step c, still in BOOT, ADR-0004 §2)."""
    # Given: some way into a boot, from power-on or from a RESET in NOMINAL
    sat = Ground.sil()
    if after_reset:
        sat.boot()
        sat.command_accepted(Command.reset())
    boot_until(sat, ticks_into_boot)

    # When: every command but RESET arrives in one tick
    downlink = sat.tick(*REFUSED, Command.ping())

    # Then: all are NACKed BOOT_IN_PROGRESS except PING, which is ACKed
    assert downlink.answers == [
        *[(command.command_id, RejectReason.BOOT_IN_PROGRESS) for command in REFUSED],
        (CommandId.PING, None),
    ]
    # ... and none of them changed the boot: it still ends in NOMINAL, not SAFE
    beacon = sat.last_telemetry if downlink.telemetry else sat.run_until(lambda _: True, within_s=5)
    assert beacon.mode is Mode.NOMINAL
    assert beacon.boot_count == (1 if after_reset else 0)


def test_reset_during_boot_is_acked_and_restarts_the_boot() -> None:
    """Scenario 5, RESET. Pins #47 (RESET in BOOT re-enters BOOT, so the boot restarts)
    and #49 (boot count +1)."""
    # Given: half way through the power-on boot
    sat = Ground.sil()
    boot_until(sat, BOOT_TICKS // 2)

    # When: RESET arrives
    downlink = sat.tick(Command.reset())

    # Then: ACKed, and the boot runs a full 5 s again from there
    assert downlink.answers == [(CommandId.RESET, None)]
    boot_until(sat, BOOT_TICKS - 1)
    beacon = sat.run_until(lambda _: True, within_s=1)
    assert beacon.mode is Mode.NOMINAL
    assert beacon.boot_count == 1


def test_commands_are_accepted_from_the_tick_after_the_boot_completes() -> None:
    """Scenario 5, the boundary. Pins #49: once the beacon has reported NOMINAL, the
    next tick's commands are handled in NOMINAL."""
    # Given: the boot has just completed
    sat = Ground.sil()
    sat.boot()

    # When: SET_MODE SCIENCE arrives in the next tick
    downlink = sat.tick(Command.set_mode(Mode.SCIENCE))

    # Then: it is accepted
    assert downlink.answers == [(CommandId.SET_MODE, None)]
    assert [t.mode for t in downlink.telemetry] == [Mode.SCIENCE]
