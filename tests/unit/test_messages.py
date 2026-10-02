"""Unit tests for message and target-boundary dataclasses."""

import dataclasses

import pytest

from pocketsat.messages import (
    TELEMETRY_PAYLOAD_SIZE,
    Command,
    FlightComputerTelemetryState,
    Packet,
    decode_telemetry,
)
from pocketsat.targets import base


@pytest.mark.parametrize(
    "instance",
    [
        Command(command_id=1),
        decode_telemetry(bytes(TELEMETRY_PAYLOAD_SIZE)),
        FlightComputerTelemetryState(uptime_ms=0, mode_id=0, boot_count=0),
        Packet(frame=b"\xa5\x5a"),
        base.TargetCapabilities(deterministic=True, real_time=False),
        base.EnvironmentState(),
        base.TargetFault(fault_type="mcu_reset"),
    ],
)
def test_frozen(instance: object) -> None:
    field = dataclasses.fields(instance)[0].name  # type: ignore[arg-type]
    with pytest.raises(dataclasses.FrozenInstanceError):
        setattr(instance, field, None)


def test_packet_link_state_defaults_to_none() -> None:
    packet = Packet(frame=b"\x01")
    link_fields = [f.name for f in dataclasses.fields(packet) if f.name != "frame"]
    assert link_fields
    assert all(getattr(packet, name) is None for name in link_fields)


def test_target_fault_params_are_read_only_copy() -> None:
    params: dict[str, base.FaultParam] = {"sensor": "battery"}
    fault = base.TargetFault(fault_type="sensor_freeze", params=params, duration_us=10_000_000)
    params["sensor"] = "thermal"
    assert fault.params["sensor"] == "battery"
    with pytest.raises(TypeError):
        fault.params["sensor"] = "thermal"  # type: ignore[index]


def test_capabilities_supported_faults() -> None:
    caps = base.TargetCapabilities(
        deterministic=False, real_time=True, supported_faults=frozenset({"mcu_reset"})
    )
    assert "mcu_reset" in caps.supported_faults
    assert (
        base.TargetCapabilities(deterministic=True, real_time=False).supported_faults == frozenset()
    )


class _MinimalTarget:
    capabilities = base.TargetCapabilities(deterministic=True, real_time=False)

    def connect(self) -> None: ...
    def reset(self, seed: int) -> None: ...
    def send(self, frame: bytes) -> None: ...
    def receive(self) -> list[bytes]:
        return []

    def apply_environment(self, env: base.EnvironmentState) -> None: ...
    def inject(self, fault: base.TargetFault) -> None: ...
    def advance(self, dt_us: int) -> None: ...
    def close(self) -> None: ...


def test_structural_conformance() -> None:
    assert isinstance(_MinimalTarget(), base.TestTarget)


def test_missing_method_does_not_conform() -> None:
    class Incomplete:
        capabilities = base.TargetCapabilities(deterministic=True, real_time=False)

        def connect(self) -> None: ...

    assert not isinstance(Incomplete(), base.TestTarget)


@pytest.mark.parametrize("duration_us", [0, 1, 10_000_000, None])
def test_target_fault_accepts_integer_microseconds(duration_us: int | None) -> None:
    assert base.TargetFault(fault_type="mute", duration_us=duration_us).duration_us == duration_us


@pytest.mark.parametrize(
    ("duration_us", "error"),
    [(0.5, TypeError), (1.0, TypeError), (True, TypeError), ("10", TypeError), (-1, ValueError)],
)
def test_target_fault_rejects_non_integer_or_negative_duration(
    duration_us: object, error: type[Exception]
) -> None:
    with pytest.raises(error):
        base.TargetFault(fault_type="mute", duration_us=duration_us)  # type: ignore[arg-type]
