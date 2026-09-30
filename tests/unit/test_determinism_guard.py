"""Flag wall-clock reads and global random state in simulation code (ADR-0003).

Scans every module under src/pocketsat except the reporting package, which is the one
place allowed to read wall-clock time (for run_id and started_utc).
"""

import ast
from pathlib import Path

import pytest

import pocketsat

PACKAGE_DIR = Path(pocketsat.__file__).parent
EXCLUDED = {PACKAGE_DIR / "reporting"}

WALL_CLOCK_ATTRS = {
    ("datetime", "now"),
    ("datetime", "utcnow"),
    ("datetime", "today"),
    ("date", "today"),
    ("datetime.datetime", "now"),
    ("datetime.datetime", "utcnow"),
    ("datetime.datetime", "today"),
    ("datetime.date", "today"),
    ("time", "time"),
    ("time", "time_ns"),
}


def _dotted(node: ast.expr) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        base = _dotted(node.value)
        return f"{base}.{node.attr}" if base else None
    return None


def find_violations(source: str) -> list[str]:
    """Return a description of each forbidden use in ``source``."""
    found: list[str] = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "random" or alias.name.startswith("random."):
                    found.append(f"line {node.lineno}: import {alias.name} (global random state)")
        elif isinstance(node, ast.ImportFrom):
            names = {alias.name for alias in node.names}
            if node.module == "random" and names - {"Random"}:
                bad = ", ".join(sorted(names - {"Random"}))
                found.append(f"line {node.lineno}: from random import {bad} (global random state)")
            if node.module == "time" and names & {"time", "time_ns"}:
                found.append(f"line {node.lineno}: from time import time (wall clock)")
        elif (
            isinstance(node, ast.Attribute) and (_dotted(node.value), node.attr) in WALL_CLOCK_ATTRS
        ):
            found.append(f"line {node.lineno}: {_dotted(node)} (wall clock)")
    return found


def _simulation_modules() -> list[Path]:
    return sorted(
        path
        for path in PACKAGE_DIR.rglob("*.py")
        if not any(path.is_relative_to(excluded) for excluded in EXCLUDED)
    )


def test_scan_covers_simulation_code() -> None:
    names = {path.relative_to(PACKAGE_DIR).as_posix() for path in _simulation_modules()}
    assert {
        "core/clock.py",
        "core/rng.py",
        "environment/nominal.py",
        "spacecraft/base.py",
        "targets/base.py",
        "targets/echo.py",
    } <= names
    assert not any(name.startswith("reporting/") for name in names)


@pytest.mark.parametrize(
    "path", _simulation_modules(), ids=lambda p: p.relative_to(PACKAGE_DIR).as_posix()
)
def test_no_wall_clock_or_global_random(path: Path) -> None:
    violations = find_violations(path.read_text(encoding="utf-8"))
    assert not violations, f"{path}: " + "; ".join(violations)


@pytest.mark.parametrize(
    "source",
    [
        "import random",
        "import random as r",
        "from random import random",
        "from random import Random, seed",
        "from random import choice",
        "import time\ntime.time()",
        "import time\ntime.time_ns()",
        "from time import time",
        "import datetime\ndatetime.datetime.now()",
        "from datetime import datetime\ndatetime.now()",
        "from datetime import datetime\ndatetime.utcnow()",
        "from datetime import date\ndate.today()",
    ],
)
def test_guard_flags_forbidden_uses(source: str) -> None:
    assert find_violations(source)


@pytest.mark.parametrize(
    "source",
    [
        "from random import Random\nRandom(1).random()",
        "import time\ntime.monotonic()",  # HIL I/O timeouts may use monotonic time
        "from datetime import UTC, datetime\ndatetime(2026, 1, 1, tzinfo=UTC)",
        "clock.now_us",
    ],
)
def test_guard_allows_deterministic_uses(source: str) -> None:
    assert find_violations(source) == []
