"""Unit tests for pocketsat.reporting.run_record."""

import json
import subprocess
from datetime import UTC, datetime, timedelta, timezone
from pathlib import Path
from typing import Any

import pytest

from pocketsat.reporting.run_record import (
    SCHEMA_VERSION,
    UNKNOWN,
    AssertionResult,
    CodeVersion,
    RunRecord,
    RunRecordError,
    RunResult,
    RuntimeEnvironment,
    UnsupportedSchemaVersionError,
    detect_code_version,
    detect_environment,
    lock_file_sha256,
    new_run_identity,
    read_run_record,
    write_run_record,
)
from pocketsat.targets.base import TargetCapabilities

ADR_0003_FIELDS = [
    "schema_version",
    "run_id",
    "started_utc",
    "scenario_name",
    "scenario_sha256",
    "master_seed",
    "stream_seeds",
    "tick_us",
    "target",
    "capabilities",
    "code_version",
    "environment",
    "result",
    "assertion_results",
]


def _record(**overrides: Any) -> RunRecord:
    fields: dict[str, Any] = {
        "run_id": "20260930T181500Z-3fa9c1",
        "started_utc": "2026-09-30T18:15:00.000000Z",
        "scenario_name": "low_elevation_pass",
        "scenario_sha256": "ab" * 32,
        "master_seed": 42,
        "stream_seeds": {"rf.loss": 1, "env.sensor_noise": 2**64 - 1},
        "tick_us": 100_000,
        "target": "sil",
        "capabilities": TargetCapabilities(
            deterministic=True, real_time=False, supported_faults=frozenset({"mute", "b"})
        ),
        "code_version": CodeVersion(git_sha="0" * 40, dirty=False),
        "environment": RuntimeEnvironment(python_version="3.12.14", lock_sha256="cd" * 32),
        "result": RunResult.FAILED,
        "assertion_results": (
            AssertionResult(name="telemetry_received_min: 5", passed=True),
            AssertionResult(name="mode_never: FAULT", passed=False, message="FAULT at t=12.3s"),
        ),
    }
    fields.update(overrides)
    return RunRecord(**fields)


def test_fields_match_adr_0003(tmp_path: Path) -> None:
    path = tmp_path / "run.json"
    write_run_record(_record(), path)
    assert list(json.loads(path.read_text())) == ADR_0003_FIELDS
    assert _record().schema_version == SCHEMA_VERSION == 1


@pytest.mark.parametrize(
    "record",
    [
        _record(),
        _record(code_version=CodeVersion(git_sha=UNKNOWN, dirty=UNKNOWN)),
        _record(assertion_results=(), stream_seeds={}, result=RunResult.ERROR),
        _record(capabilities=TargetCapabilities(deterministic=False, real_time=True)),
    ],
)
def test_round_trip(tmp_path: Path, record: RunRecord) -> None:
    path = tmp_path / "run.json"
    write_run_record(record, path)
    assert read_run_record(path) == record
    assert path.read_text().endswith("}\n")


def test_record_is_immutable() -> None:
    record = _record()
    with pytest.raises(AttributeError):
        record.master_seed = 1  # type: ignore[misc]
    with pytest.raises(TypeError):
        record.stream_seeds["new"] = 1  # type: ignore[index]


@pytest.mark.parametrize("version", [0, 2, 99, "1", None, True])
def test_unknown_schema_version_rejected(tmp_path: Path, version: object) -> None:
    data = _record().to_dict()
    data["schema_version"] = version
    path = tmp_path / "run.json"
    path.write_text(json.dumps(data))
    with pytest.raises(
        UnsupportedSchemaVersionError, match=r"unsupported run\.json schema_version"
    ):
        read_run_record(path)


def test_missing_schema_version_rejected() -> None:
    data = _record().to_dict()
    del data["schema_version"]
    with pytest.raises(UnsupportedSchemaVersionError):
        RunRecord.from_dict(data)


def test_missing_and_extra_fields_rejected() -> None:
    data = _record().to_dict()
    del data["master_seed"]
    with pytest.raises(RunRecordError, match="missing fields: master_seed"):
        RunRecord.from_dict(data)
    data = _record().to_dict() | {"surprise": 1}
    with pytest.raises(RunRecordError, match="unexpected fields: surprise"):
        RunRecord.from_dict(data)


@pytest.mark.parametrize(
    ("path", "value"),
    [
        (("master_seed",), "42"),
        (("master_seed",), True),
        (("tick_us",), 0.1),
        (("result",), "maybe"),
        (("stream_seeds", "rf.loss"), "1"),
        (("capabilities", "deterministic"), 1),
        (("code_version", "dirty"), "yes"),
        (("assertion_results",), [{"name": "x", "passed": "no", "message": ""}]),
        (("environment",), {"python_version": "3.12"}),
    ],
)
def test_wrong_types_rejected(path: tuple[str, ...], value: object) -> None:
    data = _record().to_dict()
    target = data
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(RunRecordError):
        RunRecord.from_dict(data)


def test_invalid_json_rejected(tmp_path: Path) -> None:
    path = tmp_path / "run.json"
    path.write_text("{not json")
    with pytest.raises(RunRecordError, match="not valid JSON"):
        read_run_record(path)


def test_run_identity_from_given_time() -> None:
    now = datetime(2026, 9, 30, 11, 15, 0, 123456, tzinfo=timezone(timedelta(hours=-7)))
    identity = new_run_identity(now)
    assert identity.started_utc == "2026-09-30T18:15:00.123456Z"
    stamp, suffix = identity.run_id.split("-")
    assert stamp == "20260930T181500Z"
    assert len(suffix) == 6
    int(suffix, 16)


def test_run_ids_are_unique_for_same_time() -> None:
    now = datetime(2026, 9, 30, tzinfo=UTC)
    assert len({new_run_identity(now).run_id for _ in range(50)}) == 50


def test_run_identity_requires_aware_time() -> None:
    with pytest.raises(ValueError, match="timezone-aware"):
        new_run_identity(datetime(2026, 9, 30))


def test_run_identity_defaults_to_now() -> None:
    before = datetime.now(UTC).replace(microsecond=0)
    started = datetime.fromisoformat(new_run_identity().started_utc)
    assert started >= before


# --- code_version and environment helpers --------------------------------------------


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@example.com", *args],
        cwd=repo,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()


@pytest.fixture
def isolated(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A temp directory git cannot resolve to any enclosing repository."""
    monkeypatch.setenv("GIT_CEILING_DIRECTORIES", str(tmp_path.parent))
    return tmp_path


def test_code_version_unknown_outside_git(isolated: Path) -> None:
    assert detect_code_version(isolated) == CodeVersion(git_sha=UNKNOWN, dirty=UNKNOWN)


def test_code_version_unknown_without_git_binary(
    isolated: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("PATH", "")
    assert detect_code_version(isolated) == CodeVersion(git_sha=UNKNOWN, dirty=UNKNOWN)


def test_code_version_clean_and_dirty(isolated: Path) -> None:
    _git(isolated, "init", "-q")
    (isolated / "a.txt").write_text("a")
    _git(isolated, "add", "a.txt")
    _git(isolated, "commit", "-q", "-m", "init")
    sha = _git(isolated, "rev-parse", "HEAD")

    assert detect_code_version(isolated) == CodeVersion(git_sha=sha, dirty=False)
    (isolated / "a.txt").write_text("changed")
    assert detect_code_version(isolated) == CodeVersion(git_sha=sha, dirty=True)
    _git(isolated, "checkout", "-q", "a.txt")
    (isolated / "untracked.txt").write_text("new")
    assert detect_code_version(isolated).dirty is True


def test_code_version_unknown_in_repo_without_commits(isolated: Path) -> None:
    _git(isolated, "init", "-q")
    assert detect_code_version(isolated).git_sha == UNKNOWN


def test_lock_hash(tmp_path: Path) -> None:
    lock = tmp_path / "uv.lock"
    assert lock_file_sha256(lock) == UNKNOWN
    lock.write_bytes(b"")
    assert lock_file_sha256(lock) == (
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    )
    env = detect_environment(tmp_path)
    assert env.lock_sha256 == lock_file_sha256(lock)
    assert env.python_version.startswith("3.")
    assert detect_environment(tmp_path / "missing").lock_sha256 == UNKNOWN
