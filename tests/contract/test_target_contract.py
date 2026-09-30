"""Target-agnostic contract tests every TestTarget must pass (ADR-0002).

Tests only use the public TestTarget interface and the case data from conftest.py,
so the same suite applies to EchoTarget, SilTarget, and HilTarget.
"""

import pytest
from contract_support import CONTRACT_ENVIRONMENTS, STEP_US, TargetCase

from pocketsat.frame import decode_frame
from pocketsat.targets import base


def _stimulate(target: base.TestTarget, case: TargetCase) -> list[bytes]:
    """Send the case's stimulus, advance until a response is due, and drain output."""
    target.send(case.stimulus)
    for _ in range(case.response_steps):
        target.advance(STEP_US)
    return target.receive()


# --- Interface and capabilities -------------------------------------------------------


def test_conforms_to_protocol(target: base.TestTarget) -> None:
    assert isinstance(target, base.TestTarget)


def test_capabilities_are_declared(target: base.TestTarget) -> None:
    caps = target.capabilities
    assert isinstance(caps, base.TargetCapabilities)
    assert isinstance(caps.deterministic, bool)
    assert isinstance(caps.real_time, bool)
    assert isinstance(caps.supported_faults, frozenset)
    assert all(isinstance(name, str) and name for name in caps.supported_faults)


def test_sample_faults_cover_supported_faults(
    target: base.TestTarget, target_case: TargetCase
) -> None:
    sampled = {fault.fault_type for fault in target_case.sample_faults}
    assert sampled == target.capabilities.supported_faults


# --- Faults ---------------------------------------------------------------------------


def test_inject_accepts_supported_faults(target: base.TestTarget, target_case: TargetCase) -> None:
    for fault in target_case.sample_faults:
        target.reset(seed=0)
        target.inject(fault)


def test_inject_unsupported_fault_raises_clear_error(target: base.TestTarget) -> None:
    unsupported = "contract_test_unsupported_fault"
    assert unsupported not in target.capabilities.supported_faults
    with pytest.raises(base.UnsupportedFaultError, match=unsupported) as exc_info:
        target.inject(base.TargetFault(fault_type=unsupported))
    assert exc_info.value.fault_type == unsupported
    assert exc_info.value.supported == target.capabilities.supported_faults


# --- Environment ----------------------------------------------------------------------


ENV_IDS = list(CONTRACT_ENVIRONMENTS)


@pytest.mark.parametrize("env", CONTRACT_ENVIRONMENTS.values(), ids=ENV_IDS)
def test_apply_environment_accepted_in_any_state_after_reset(
    target: base.TestTarget, target_case: TargetCase, env: base.EnvironmentState
) -> None:
    nominal = CONTRACT_ENVIRONMENTS["nominal"]

    target.apply_environment(env)  # immediately after reset
    target.apply_environment(nominal)  # repeated with no step in between
    target.send(target_case.stimulus)
    target.apply_environment(env)  # with uplink pending
    target.advance(STEP_US)
    target.apply_environment(nominal)  # with downlink not yet drained
    target.receive()
    target.apply_environment(env)  # after draining
    for fault in target_case.sample_faults:
        target.inject(fault)
        target.apply_environment(env)  # with a fault active
        target.advance(STEP_US)
    target.reset(seed=1)
    target.apply_environment(env)  # after a second reset
    target.advance(STEP_US)


@pytest.mark.parametrize("env", CONTRACT_ENVIRONMENTS.values(), ids=ENV_IDS)
def test_target_keeps_operating_under_any_valid_environment(
    target: base.TestTarget, target_case: TargetCase, env: base.EnvironmentState
) -> None:
    # A target may legitimately stop transmitting (for example with an empty battery),
    # so only require that it keeps stepping and that anything it sends is valid.
    target.apply_environment(env)
    for _ in range(3):
        for frame in _stimulate(target, target_case):
            assert isinstance(frame, bytes)
            decode_frame(frame)


# --- receive() ------------------------------------------------------------------------


def test_receive_is_empty_before_time_advances(target: base.TestTarget) -> None:
    assert target.receive() == []


def test_stimulus_produces_valid_frames(target: base.TestTarget, target_case: TargetCase) -> None:
    frames = _stimulate(target, target_case)
    assert frames, "stimulus produced no downlink frames"
    for frame in frames:
        assert isinstance(frame, bytes)
        decode_frame(frame)


def test_receive_returns_only_frames_since_previous_call(
    target: base.TestTarget, target_case: TargetCase
) -> None:
    first = _stimulate(target, target_case)
    assert first
    assert target.receive() == []  # already drained; no time has passed

    target.advance(STEP_US)
    target.receive()  # drain anything produced during the idle step
    second = _stimulate(target, target_case)
    assert second
    assert target.receive() == []


def test_reset_discards_undrained_output(target: base.TestTarget, target_case: TargetCase) -> None:
    target.send(target_case.stimulus)
    target.advance(STEP_US)
    target.reset(seed=0)
    assert target.receive() == []


# --- Determinism ----------------------------------------------------------------------


def test_deterministic_target_repeats_output_for_same_seed(
    target: base.TestTarget, target_case: TargetCase
) -> None:
    if not target.capabilities.deterministic:
        pytest.skip("target does not declare deterministic=True")

    def run() -> list[list[bytes]]:
        target.reset(seed=42)
        outputs = []
        for env in CONTRACT_ENVIRONMENTS.values():
            target.apply_environment(env)
            outputs.append(_stimulate(target, target_case))
        for fault in target_case.sample_faults:
            target.inject(fault)
            outputs.append(_stimulate(target, target_case))
        return outputs

    assert run() == run()
