"""Test targets: the ``TestTarget`` protocol and its supporting types."""

from pocketsat.targets.base import (
    EnvironmentState,
    FaultParam,
    TargetCapabilities,
    TargetFault,
    TestTarget,
    UnsupportedFaultError,
)

__all__ = [
    "EnvironmentState",
    "FaultParam",
    "TargetCapabilities",
    "TargetFault",
    "TestTarget",
    "UnsupportedFaultError",
]
