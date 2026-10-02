"""Flag non-deterministic and non-portable code in simulation code.

ADR-0003: no wall-clock reads and no global random state.
ADR-0006: no platform maths library calls, no library random distributions, and no
`**` unless the exponent is an integer literal, so results are byte-identical on every
platform.

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


NONPORTABLE_MATH = frozenset(
    {
        "exp",
        "expm1",
        "log",
        "log1p",
        "log2",
        "log10",
        "pow",
        "sin",
        "cos",
        "tan",
        "asin",
        "acos",
        "atan",
        "atan2",
        "sinh",
        "cosh",
        "tanh",
        "hypot",
        "erf",
        "gamma",
    }
)
"""`math` functions that call the platform maths library (ADR-0006)."""

NONPORTABLE_RANDOM_METHODS = frozenset(
    {
        "gauss",
        "normalvariate",
        "lognormvariate",
        "expovariate",
        "vonmisesvariate",
        "gammavariate",
        "betavariate",
        "paretovariate",
        "weibullvariate",
    }
)
"""`random.Random` methods that call the platform maths library (ADR-0006)."""


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
            if node.module == "math" and names & NONPORTABLE_MATH:
                bad = ", ".join(sorted(names & NONPORTABLE_MATH))
                found.append(f"line {node.lineno}: from math import {bad} (not portable)")
        elif isinstance(node, ast.Attribute):
            if (_dotted(node.value), node.attr) in WALL_CLOCK_ATTRS:
                found.append(f"line {node.lineno}: {_dotted(node)} (wall clock)")
            elif _dotted(node.value) == "math" and node.attr in NONPORTABLE_MATH:
                found.append(f"line {node.lineno}: math.{node.attr} (not portable)")
            elif node.attr in NONPORTABLE_RANDOM_METHODS:
                found.append(
                    f"line {node.lineno}: .{node.attr} (not portable; use portable_normal)"
                )
        elif isinstance(node, ast.BinOp) and isinstance(node.op, ast.Pow):
            exponent = node.right
            if not (
                isinstance(exponent, ast.Constant)
                and type(exponent.value) is int
                and exponent.value >= 0
            ):
                found.append(f"line {node.lineno}: ** needs a non-negative int literal exponent")
        elif (
            isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "pow"
        ):
            found.append(f"line {node.lineno}: pow() (not portable)")
    return found


def _simulation_modules() -> list[Path]:
    return sorted(
        path
        for path in PACKAGE_DIR.rglob("*.py")
        if not any(path.is_relative_to(excluded) for excluded in EXCLUDED)
    )


SPACECRAFT_MODULES = frozenset(
    {
        "spacecraft/__init__.py",
        "spacecraft/attitude.py",
        "spacecraft/base.py",
        "spacecraft/comms.py",
        "spacecraft/config.py",
        "spacecraft/controls.py",
        "spacecraft/fakes.py",
        "spacecraft/payload.py",
        "spacecraft/power.py",
        "spacecraft/snapshots.py",
        "spacecraft/thermal.py",
    }
)
"""Every spacecraft module, listed by name (epic #30 close-out, #70): the five subsystems
(power, thermal, attitude, payload, comms) and the framework they share."""


def test_scan_covers_simulation_code() -> None:
    names = {path.relative_to(PACKAGE_DIR).as_posix() for path in _simulation_modules()}
    assert {
        "core/clock.py",
        "core/portable.py",
        "core/rng.py",
        "environment/nominal.py",
        "flight/__init__.py",
        "flight/modes.py",
        "targets/base.py",
        "targets/echo.py",
    } <= names
    assert not any(name.startswith("reporting/") for name in names)


def test_scan_covers_every_spacecraft_module() -> None:
    # Each subsystem module is scanned by name, and a new spacecraft module must be added
    # to the list, so none is left out of the explicit coverage.
    names = {path.relative_to(PACKAGE_DIR).as_posix() for path in _simulation_modules()}
    scanned = {name for name in names if name.startswith("spacecraft/")}
    assert scanned == SPACECRAFT_MODULES, (
        f"missing from the scan: {sorted(SPACECRAFT_MODULES - scanned)};"
        f" not listed in SPACECRAFT_MODULES: {sorted(scanned - SPACECRAFT_MODULES)}"
    )


@pytest.mark.parametrize(
    "path", _simulation_modules(), ids=lambda p: p.relative_to(PACKAGE_DIR).as_posix()
)
def test_simulation_code_is_deterministic_and_portable(path: Path) -> None:
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
        # ADR-0006: platform maths library
        "import math\nmath.exp(1.0)",
        "import math\nmath.cos(x)",
        "import math\nmath.atan2(y, x)",
        "import math\nmath.pow(x, 2)",
        "import math\nf = math.log",
        "from math import cos",
        "from math import sqrt, log",
        "pow(x, 2)",
        # ADR-0006: library random distributions
        "rng.gauss(0.0, 1.0)",
        "rng.normalvariate(0.0, 1.0)",
        "self._rng.expovariate(2.0)",
        "draw = rng.gauss",
        # ADR-0006: ** without a non-negative integer literal exponent
        "x ** 0.5",
        "x ** n",
        "x ** -1",
        "x ** 2.0",
        "x ** True",
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
        # ADR-0006: portable operations
        "import math\nmath.sqrt(x)",
        "import math\nmath.floor(x) + math.ceil(x)",
        "import math\nmath.isfinite(x)",
        "from math import sqrt, isfinite",
        "abs(x) + min(x, y) + max(x, y)",
        "x ** 2",
        "2 ** 64",
        "rng.random()",
        "rng.uniform(0.0, 1.0)",
        "portable_normal(rng, 0.0, 1.0)",
        "portable_cos_deg(angle)",
        "def f(**kwargs: int) -> None: ...",
        "x * x / y - z",
    ],
)
def test_guard_allows_deterministic_uses(source: str) -> None:
    assert find_violations(source) == []
