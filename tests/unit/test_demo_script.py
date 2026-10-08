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
        ("nominal", ["1. NOMINAL ORBIT", "energy:", "released 0 B once sent", "comms: rx_tx"]),
        ("detumble", ["2. DETUMBLE", "tumbling", "stabilized", "payload first ACQUIRING"]),
        ("faults", ["3a. FAULT: battery_drain", "critical_battery set at", "3b.", "3c."]),
        ("determinism", ["4. DETERMINISM", "identical truth and readings in every case: True"]),
        ("cold", ["5. COLD CASE", "heater first on at", "UNDER"]),
    ],
)
def test_each_scenario_runs(demo: ModuleType, scenario: str, expected: list[str]) -> None:
    text = run(demo, "--scenario", scenario)
    assert "payload, comms on a shared SnapshotBoard" in text
    for snippet in expected:
        assert snippet in text, snippet
    assert "Done: 1 scenario(s)" in text


def test_nominal_orbit_downlinks_through_the_flight_computer(demo: ModuleType) -> None:
    # #56: the nominal orbit's chunks are released by the real flight computer once sent
    # as DATA frames, not by a scripted stand-in. A whole orbit at 10 s ticks has one
    # pass, which empties the buffer: the radio's rate is 1200 B/s at any tick length
    # (#122), 12 000 B in each 10 s tick, so the pass sends as much as at 100 ms.
    out = io.StringIO()
    assert demo.main(["--tick-ms", "10000", "--orbits", "1", "--scenario", "nominal"], out=out) == 0
    text = out.getvalue()
    assert "downlink: passes from min 82.0; 2993 DATA frames, 191552 B of chunks sent" in text
    assert "released 191552 B once sent" in text
    assert "rate 1200 B/s (12000 B this tick)" in text
    assert "flags raised: none" in text
    assert "stand-in" not in text
    assert "per tick" not in text  # no warning that the rate depends on the tick (#122)


def test_sensor_freeze_holds_readings(demo: ModuleType) -> None:
    assert "power readings held=True" in run(demo, "--scenario", "faults")


def test_transmitter_off_drops_capacity_and_draw(demo: ModuleType) -> None:
    text = run(demo, "--scenario", "faults").split("3c.")[1]
    rows = [line.split() for line in text.splitlines() if "rx_only" in line]
    assert rows
    for row in rows:
        mode, rx, tx, capacity, draw = row[2:7]
        assert (mode, rx, tx, capacity, draw) == ("rx_only", "on", "off", "0", "0.00")


def test_noise_only_seed_change(demo: ModuleType) -> None:
    text = run(demo, "--scenario", "determinism")
    assert "truth identical=True, readings identical=False" in text


def test_rejects_bad_arguments(demo: ModuleType) -> None:
    with pytest.raises(SystemExit):
        demo.main(["--scenario", "nope"], out=io.StringIO())
    with pytest.raises(SystemExit):
        demo.main(["--orbits", "0"], out=io.StringIO())
