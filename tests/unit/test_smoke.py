"""Smoke tests: the package imports and the CLI reports its version."""

import subprocess
import sys
from importlib.metadata import version
from pathlib import Path

import pytest

import pocketsat
from pocketsat.cli import main


def test_package_version_matches_metadata() -> None:
    assert pocketsat.__version__ == version("pocketsat")


def test_package_version_falls_back_without_metadata() -> None:
    """Importing from a bare source tree, with no installed package metadata, works and
    reports ``0.0.0+unknown`` (#103)."""
    src = Path(pocketsat.__file__).resolve().parent.parent
    # -S skips site-packages, where the installed package's metadata lives, so only the
    # source tree is importable.
    script = (
        "import sys; sys.path.insert(0, sys.argv[1]); import pocketsat; "
        "print(pocketsat.__file__); print(pocketsat.__version__)"
    )
    result = subprocess.run(
        [sys.executable, "-S", "-c", script, str(src)],
        capture_output=True,
        text=True,
        check=True,
    )
    imported_from, reported = result.stdout.splitlines()
    assert Path(imported_from).resolve().parent.parent == src
    assert reported == "0.0.0+unknown"


def test_cli_version_flag(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as exc_info:
        main(["--version"])
    assert exc_info.value.code == 0
    assert capsys.readouterr().out.strip() == f"pocketsat {pocketsat.__version__}"


def test_module_entrypoint_version() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "pocketsat.cli", "--version"],
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout.strip() == f"pocketsat {pocketsat.__version__}"
