"""mypy rejects a truth record passed to the telemetry encoder (#54, ADR-0004 §7).

True values never go on the wire: ``encode_telemetry`` accepts only readings records
(plus the flight computer's state), so passing any subsystem's truth record is a static
type error. Payload and comms truth and readings records have identical fields, so this
also proves the check is nominal, not structural.
"""

import re
import textwrap
from pathlib import Path

import pytest
from mypy import api

PREAMBLE = textwrap.dedent(
    """\
    from pocketsat.messages import FlightComputerTelemetryState, encode_telemetry
    from pocketsat.spacecraft.controls import RadioMode
    from pocketsat.spacecraft.snapshots import (
        AttitudeReadings,
        AttitudeState,
        AttitudeTruth,
        CommsReadings,
        CommsTruth,
        PayloadReadings,
        PayloadState,
        PayloadTruth,
        PowerReadings,
        PowerTruth,
        ThermalReadings,
        ThermalTruth,
    )

    FC = FlightComputerTelemetryState(uptime_ms=0, mode_id=1, boot_count=0)
    POWER = PowerReadings(
        bus_v=7.4, battery_current_a=0.0, soc=0.8, low_battery=False, critical_battery=False
    )
    THERMAL = ThermalReadings(
        battery_c=10.0, electronics_c=20.0, over_temp=False, under_temp=False
    )
    ATTITUDE = AttitudeReadings(
        pointing_error_deg=1.0, rate_dps=0.0, state=AttitudeState.STABILIZED
    )
    PAYLOAD = PayloadReadings(PayloadState.IDLE, 0, 64, 0, 0, 0, 0, 0.0)
    COMMS = CommsReadings(RadioMode.RX_TX, True, True, 120, 0, 0, 0, 0.15)

    POWER_TRUTH = PowerTruth(
        bus_v=7.4, battery_current_a=0.0, soc=0.8, generation_w=0.0, total_load_w=1.0
    )
    THERMAL_TRUTH = ThermalTruth(
        battery_c=10.0, electronics_c=20.0, heater_on=False, heater_power_w=0.0
    )
    ATTITUDE_TRUTH = AttitudeTruth(
        pointing_error_deg=1.0, rate_dps=0.0, state=AttitudeState.STABILIZED, control_power_w=0.0
    )
    PAYLOAD_TRUTH = PayloadTruth(PayloadState.IDLE, 0, 64, 0, 0, 0, 0, 0.0)
    COMMS_TRUTH = CommsTruth(RadioMode.RX_TX, True, True, 120, 0, 0, 0, 0.15)
    """
)

READINGS_CALL = (
    "encode_telemetry(FC, power=POWER, thermal=THERMAL, attitude=ATTITUDE, "
    "payload=PAYLOAD, comms=COMMS)"
)

TRUTH_SLOTS = {
    "power": "POWER_TRUTH",
    "thermal": "THERMAL_TRUTH",
    "attitude": "ATTITUDE_TRUTH",
    "payload": "PAYLOAD_TRUTH",
    "comms": "COMMS_TRUTH",
}


def _mypy(tmp_path: Path, body: str) -> tuple[int, str]:
    module = tmp_path / "telemetry_input.py"
    module.write_text(PREAMBLE + body)
    stdout, stderr, status = api.run(
        [
            "--strict",
            "--no-incremental",
            "--cache-dir",
            str(tmp_path / ".mypy_cache"),
            str(module),
        ]
    )
    return status, stdout + stderr


@pytest.mark.slow
def test_mypy_rejects_every_truth_record_and_accepts_readings(tmp_path: Path) -> None:
    body = READINGS_CALL + "\n"
    for slot, truth in TRUTH_SLOTS.items():
        body += READINGS_CALL.replace(f"{slot}={slot.upper()}", f"{slot}={truth}") + "\n"
    status, output = _mypy(tmp_path, body)

    assert status == 1, output
    errors = [line for line in output.splitlines() if ": error:" in line]
    first_call_line = PREAMBLE.count("\n") + 1
    assert len(errors) == len(TRUTH_SLOTS), output
    for (slot, truth), error in zip(TRUTH_SLOTS.items(), errors, strict=True):
        record = truth.removesuffix("_TRUTH").title() + "Truth"
        assert f'Argument "{slot}" to "encode_telemetry"' in error, error
        assert f'"{record}"' in error and "[arg-type]" in error, error
        line = int(re.search(r":(\d+): error", error).group(1))  # type: ignore[union-attr]
        assert line > first_call_line, "the readings-only call must type-check"


@pytest.mark.slow
def test_mypy_accepts_readings_only(tmp_path: Path) -> None:
    status, output = _mypy(tmp_path, READINGS_CALL + "\n")
    assert status == 0, output
