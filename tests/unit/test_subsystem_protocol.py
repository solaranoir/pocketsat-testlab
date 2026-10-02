"""Every real subsystem satisfies the ``Subsystem`` protocol, at runtime and statically.

Regression test: ``Thermal`` and ``Payload`` declared ``name: Final``, which mypy
rejected for a protocol member declared as a settable ``name: str``, so typed code
could not put them in a ``SubsystemStack``.
"""

import textwrap
from pathlib import Path

import pytest
from mypy import api

from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    STEP_ORDER,
    Attitude,
    Comms,
    Payload,
    Power,
    Thermal,
)
from pocketsat.spacecraft.base import SnapshotBoard, Subsystem, SubsystemStack

TYPED_STACK = textwrap.dedent(
    """\
    from pocketsat.spacecraft import (
        DEFAULT_INITIAL_STATE,
        NOMINAL_CONFIG,
        Attitude,
        Comms,
        Payload,
        Power,
        Thermal,
    )
    from pocketsat.spacecraft.base import SnapshotBoard, Subsystem, SubsystemStack


    def build() -> SubsystemStack:
        board = SnapshotBoard()
        subsystems: list[Subsystem] = [
            Power(NOMINAL_CONFIG.power, DEFAULT_INITIAL_STATE.power, board),
            Thermal(NOMINAL_CONFIG.thermal, DEFAULT_INITIAL_STATE.thermal, board),
            Attitude(NOMINAL_CONFIG.attitude, DEFAULT_INITIAL_STATE.attitude),
            Payload(NOMINAL_CONFIG.payload, DEFAULT_INITIAL_STATE.payload, board),
            Comms(NOMINAL_CONFIG.comms),
        ]
        return SubsystemStack(subsystems, board=board)


    def build_inline() -> SubsystemStack:
        board = SnapshotBoard()
        return SubsystemStack(
            [
                Power(NOMINAL_CONFIG.power, DEFAULT_INITIAL_STATE.power, board),
                Thermal(NOMINAL_CONFIG.thermal, DEFAULT_INITIAL_STATE.thermal, board),
                Attitude(NOMINAL_CONFIG.attitude, DEFAULT_INITIAL_STATE.attitude),
                Payload(NOMINAL_CONFIG.payload, DEFAULT_INITIAL_STATE.payload, board),
                Comms(NOMINAL_CONFIG.comms),
            ],
            board=board,
        )
    """
)


def _real_subsystems() -> list[Subsystem]:
    board = SnapshotBoard()
    return [
        Power(NOMINAL_CONFIG.power, DEFAULT_INITIAL_STATE.power, board),
        Thermal(NOMINAL_CONFIG.thermal, DEFAULT_INITIAL_STATE.thermal, board),
        Attitude(NOMINAL_CONFIG.attitude, DEFAULT_INITIAL_STATE.attitude),
        Payload(NOMINAL_CONFIG.payload, DEFAULT_INITIAL_STATE.payload, board),
        Comms(NOMINAL_CONFIG.comms),
    ]


@pytest.mark.parametrize("subsystem", _real_subsystems(), ids=lambda s: s.name)
def test_real_subsystem_satisfies_protocol_at_runtime(subsystem: Subsystem) -> None:
    assert isinstance(subsystem, Subsystem)


def test_real_subsystems_cover_step_order() -> None:
    assert SubsystemStack(_real_subsystems()).names == STEP_ORDER


@pytest.mark.slow
def test_typed_stack_of_real_subsystems_passes_mypy_strict(tmp_path: Path) -> None:
    module = tmp_path / "typed_stack.py"
    module.write_text(TYPED_STACK)
    stdout, stderr, status = api.run(
        [
            "--strict",
            "--no-incremental",
            "--cache-dir",
            str(tmp_path / ".mypy_cache"),
            str(module),
        ]
    )
    assert status == 0, stdout + stderr
