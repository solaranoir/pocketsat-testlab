"""Behaviour: target faults through ``inject()`` (#102 scenarios 4 and 6).

Driven through TestTarget only (see ``_ground.py``). Scenario 4 compares telemetry with
the true temperature, which only the test-only truth probe (``SilTarget.last_tick``)
can supply; #102 allows exactly that. The merge mechanics of each fault are covered in
``tests/sil/test_sil_faults.py`` (#60).
"""

import pytest
from _ground import Ground

from pocketsat.messages import Command, CommandId, Telemetry, TelemetryFlags
from pocketsat.targets.base import EnvironmentState, TargetFault, UnsupportedFaultError
from pocketsat.targets.sil import SENSOR_FREEZE

COLD_ECLIPSE = EnvironmentState(sunlit=False, ambient_temp_c=-150.0)
"""Drives the true temperatures down by a few degrees in 20 s."""

FREEZE_S = 20


def test_sensor_freeze_holds_the_reported_temperature_while_the_true_one_keeps_falling() -> None:
    """Scenario 4. Pins ADR-0004 §8 (``sensor_freeze`` holds readings at their last
    reported value; the physics carries on) and §7 (telemetry carries reported
    values only), and #60 (the target releases the fault after ``duration_us``)."""
    # Given: booted, NOMINAL, in a cold eclipse that cools the battery
    sat = Ground.sil()
    sat.boot()
    sat.apply_environment(COLD_ECLIPSE)
    sat.run(1)

    # When: the thermal sensors freeze for 20 s
    sat.inject(
        TargetFault(SENSOR_FREEZE, {"subsystem": "thermal"}, duration_us=FREEZE_S * 1_000_000)
    )
    frozen: list[tuple[Telemetry, float]] = []
    for downlink in sat.ticking(FREEZE_S):
        frozen += [(t, sat.probe.thermal_truth.battery_c) for t in downlink.telemetry]

    # Then: every frame reports the same temperatures ...
    assert len(frozen) >= FREEZE_S - 1
    assert len({(t.battery_c, t.electronics_c) for t, _ in frozen}) == 1
    # ... while the true battery temperature keeps falling, away from the report
    truths = [truth for _, truth in frozen]
    assert truths == sorted(truths, reverse=True)
    assert truths[0] - truths[-1] > 2.0
    reported = frozen[-1][0].battery_c
    assert reported - truths[-1] > 2.0
    # ... and no flag or mode change came from the hidden cooling
    assert all(not t.flags & TelemetryFlags.under_temp for t, _ in frozen)

    # When: the fault expires
    sat.run(2)

    # Then: telemetry follows the true temperature again (within the sensor noise)
    assert abs(sat.last_telemetry.battery_c - sat.probe.thermal_truth.battery_c) < 1.5
    assert sat.last_telemetry.battery_c < reported - 2.0


@pytest.mark.parametrize("fault_type", ["solar_flare", "", "SENSOR_FREEZE", "mute"])
def test_inject_with_an_unsupported_fault_type_raises_a_clear_error_and_changes_nothing(
    fault_type: str,
) -> None:
    """Scenario 6. Pins ADR-0002 §4: a target rejects a fault type it does not declare
    in ``capabilities.supported_faults`` with ``UnsupportedFaultError`` (a
    ``ValueError``) naming the type and what is supported, and the run is unaffected.
    ``mute`` is the echo target's fault: supported elsewhere is still unsupported here."""
    # Given: a powered-on target, and an identical one as the reference
    sat = Ground.sil()
    reference = Ground.sil()
    supported = sat.target.capabilities.supported_faults
    assert fault_type not in supported

    # When: an unsupported fault is injected
    with pytest.raises(UnsupportedFaultError) as raised:
        sat.inject(TargetFault(fault_type))

    # Then: the error says what was wrong and what would have been accepted
    error = raised.value
    assert isinstance(error, ValueError)
    assert error.fault_type == fault_type
    assert error.supported == supported
    message = str(error)
    assert repr(fault_type) in message
    assert all(name in message for name in supported)
    # ... and nothing was injected: the run is byte for byte the reference run
    sat.boot()
    reference.boot()
    assert sat.tick(Command.ping()).answers == [(CommandId.PING, None)]
    reference.tick(Command.ping())
    assert [d.frames for d in sat.history] == [d.frames for d in reference.history]
