"""Target registry for the TestTarget contract suite.

To run the suite against a new target, add a ``TargetCase`` to ``TARGET_CASES``.
See README.md in this directory.
"""

from collections.abc import Callable
from dataclasses import dataclass

import pytest

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.frame import Frame, FrameType, encode_frame
from pocketsat.targets import base
from pocketsat.targets.echo import MUTE_FAULT, EchoTarget

STEP_US = DEFAULT_TICK_US
"""Simulated time passed to ``advance()`` per step, in integer microseconds (one 100 ms tick)."""


@dataclass(frozen=True)
class TargetCase:
    """How the contract suite builds and drives one kind of target.

    Attributes:
        name: Test ID suffix, e.g. ``"echo"``, ``"sil"``, ``"hil"``.
        factory: Returns a new, unconnected target.
        stimulus: An encoded uplink frame that makes the target produce at least one
            downlink frame within ``response_steps`` calls to ``advance(STEP_US)``.
        sample_faults: One valid fault per entry in ``capabilities.supported_faults``.
        response_steps: Steps to advance after sending ``stimulus``.
        marks: Extra pytest marks for this target (e.g. a HIL hardware marker).
    """

    name: str
    factory: Callable[[], base.TestTarget]
    stimulus: bytes
    sample_faults: tuple[base.TargetFault, ...] = ()
    response_steps: int = 1
    marks: tuple[pytest.MarkDecorator, ...] = ()


CONTRACT_ENVIRONMENTS: dict[str, base.EnvironmentState] = {
    "nominal": base.EnvironmentState(),
    "eclipse": base.EnvironmentState(sunlit=False),
    "cold": base.EnvironmentState(ambient_temp_c=-150.0),
    "hot": base.EnvironmentState(ambient_temp_c=120.0),
    "noiseless": base.EnvironmentState(sensor_noise_scale=0.0),
    "noisy": base.EnvironmentState(sensor_noise_scale=5.0),
    "battery_empty": base.EnvironmentState(battery_soc_override=0.0),
    "battery_full": base.EnvironmentState(battery_soc_override=1.0),
    "worst_case": base.EnvironmentState(
        sunlit=False, ambient_temp_c=-150.0, sensor_noise_scale=5.0, battery_soc_override=0.05
    ),
}
"""Valid environments covering every ``EnvironmentState`` field at nominal and extreme
values. Every target must accept all of them in any state after ``reset()``."""


PING_FRAME = encode_frame(Frame(frame_type=FrameType.COMMAND, sequence=1, payload=b"\x01"))

TARGET_CASES: list[TargetCase] = [
    TargetCase(
        name="echo",
        factory=EchoTarget,
        stimulus=PING_FRAME,
        sample_faults=(base.TargetFault(fault_type=MUTE_FAULT, duration_us=1_000_000),),
    ),
]
