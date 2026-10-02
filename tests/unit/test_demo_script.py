"""Smoke test for the epic #30 demo script (scripts/demo_spacecraft.py)."""

import importlib.util
import io
import sys
from pathlib import Path
from types import ModuleType

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "demo_spacecraft.py"

# Coarse ticks keep each scenario fast; the dynamics stay stable at 10 s steps.
FAST = ["--tick-ms", "10000", "--orbits", "0.25"]


@pytest.fixture(scope="module")
def demo() -> ModuleType:
    spec = importlib.util.spec_from_file_location("demo_spacecraft", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module


def run(demo: ModuleType, *argv: str) -> str:
    out = io.StringIO()
    assert demo.main([*FAST, *argv], out=out) == 0
    return out.getvalue()


@pytest.mark.parametrize(
    ("scenario", "expected"),
    [
        ("nominal", ["1. NOMINAL ORBIT", "Summary", "energy:", "flags raised:"]),
        ("detumble", ["2. DETUMBLE", "tumbling", "stabilized", "payload first ACQUIRING"]),
        ("faults", ["3a. FAULT: battery_drain", "critical_battery set at", "3b. FAULT"]),
        ("determinism", ["4. DETERMINISM", "identical truth and readings in every case: True"]),
        ("cold", ["5. COLD CASE", "heater first on at", "UNDER"]),
    ],
)
def test_each_scenario_runs(demo: ModuleType, scenario: str, expected: list[str]) -> None:
    text = run(demo, "--scenario", scenario)
    assert "comms: STAND-IN" in text
    for snippet in expected:
        assert snippet in text, snippet
    assert "Done: 1 scenario(s)" in text


def test_sensor_freeze_holds_readings(demo: ModuleType) -> None:
    assert "power readings held=True" in run(demo, "--scenario", "faults")


def test_noise_only_seed_change(demo: ModuleType) -> None:
    text = run(demo, "--scenario", "determinism")
    assert "truth identical=True, readings identical=False" in text


def test_rejects_bad_arguments(demo: ModuleType) -> None:
    with pytest.raises(SystemExit):
        demo.main(["--scenario", "nope"], out=io.StringIO())
    with pytest.raises(SystemExit):
        demo.main(["--orbits", "0"], out=io.StringIO())
