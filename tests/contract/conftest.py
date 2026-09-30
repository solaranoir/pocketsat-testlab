"""Fixtures for the TestTarget contract suite. Targets are registered in contract_support.py."""

from collections.abc import Iterator

import pytest
from contract_support import TARGET_CASES, TargetCase

from pocketsat.targets import base


@pytest.fixture(
    params=[pytest.param(case, id=case.name, marks=case.marks) for case in TARGET_CASES]
)
def target_case(request: pytest.FixtureRequest) -> TargetCase:
    """The target case under test; the suite runs once per registered case."""
    case: TargetCase = request.param
    return case


@pytest.fixture
def target(target_case: TargetCase) -> Iterator[base.TestTarget]:
    """A connected target, reset with seed 0, closed after the test."""
    instance = target_case.factory()
    instance.connect()
    instance.reset(seed=0)
    try:
        yield instance
    finally:
        instance.close()
