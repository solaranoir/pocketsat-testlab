"""Smoke tests: the package imports and the CLI reports its version."""

import subprocess
import sys
from importlib.metadata import version

import pytest

import pocketsat
from pocketsat.cli import main


def test_package_version_matches_metadata() -> None:
    assert pocketsat.__version__ == version("pocketsat")


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
