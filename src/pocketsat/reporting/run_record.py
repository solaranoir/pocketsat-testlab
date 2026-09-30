"""The ``run.json`` run record (ADR-0003 section 4).

Every run writes one record identifying exactly what ran, with which seeds, on which
code, and with what outcome, so any run can be replayed.

Wall-clock time is read only here, by :func:`new_run_identity`, to produce ``run_id``
and ``started_utc``. It never enters the simulation.
"""

import hashlib
import json
import platform
import secrets
import subprocess
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Final, Literal

from pocketsat.targets.base import TargetCapabilities

SCHEMA_VERSION: Final = 1
"""Current ``run.json`` schema version."""

SUPPORTED_SCHEMA_VERSIONS: Final = frozenset({SCHEMA_VERSION})

UNKNOWN: Final = "unknown"
"""Value used when the code version or lock hash cannot be determined."""


class RunRecordError(ValueError):
    """A ``run.json`` file is malformed or missing required fields."""


class UnsupportedSchemaVersionError(RunRecordError):
    """A ``run.json`` file declares a schema version this code cannot read."""

    def __init__(self, version: object) -> None:
        self.version = version
        supported = ", ".join(str(v) for v in sorted(SUPPORTED_SCHEMA_VERSIONS))
        super().__init__(f"unsupported run.json schema_version {version!r}; supported: {supported}")


class RunResult(StrEnum):
    """Overall outcome of a run."""

    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"


@dataclass(frozen=True)
class AssertionResult:
    """Outcome of one scenario assertion.

    Attributes:
        name: The assertion as written in the scenario.
        passed: Whether it held.
        message: Detail, typically set on failure.
    """

    name: str
    passed: bool
    message: str = ""


@dataclass(frozen=True)
class CodeVersion:
    """The code a run executed.

    Attributes:
        git_sha: Commit SHA, or ``"unknown"`` outside a git checkout.
        dirty: Whether the working tree had uncommitted or untracked changes, or
            ``"unknown"`` outside a git checkout.
    """

    git_sha: str
    dirty: bool | Literal["unknown"]


@dataclass(frozen=True)
class RuntimeEnvironment:
    """The runtime a run executed on.

    Attributes:
        python_version: Python version, e.g. ``"3.12.14"``.
        lock_sha256: SHA-256 of ``uv.lock``, or ``"unknown"`` if it was not found.
    """

    python_version: str
    lock_sha256: str


@dataclass(frozen=True)
class RunIdentity:
    """Wall-clock identity of one execution.

    Attributes:
        run_id: Unique ID, ``<UTC timestamp>-<6 hex chars>``, e.g. ``20260930T181500Z-3fa9c1``.
        started_utc: ISO 8601 UTC start time, e.g. ``2026-09-30T18:15:00.123456Z``.
    """

    run_id: str
    started_utc: str


@dataclass(frozen=True, kw_only=True)
class RunRecord:
    """Everything needed to identify, reproduce, and judge one run.

    Fields follow ADR-0003 section 4 and are keyword-only.

    Attributes:
        schema_version: ``run.json`` format version.
        run_id: Unique per execution (see :class:`RunIdentity`).
        started_utc: ISO 8601 UTC start time.
        scenario_name: Scenario name.
        scenario_sha256: SHA-256 of the scenario file contents.
        master_seed: The run's master seed.
        stream_seeds: Seed issued to each named random stream.
        tick_us: Simulated tick length in microseconds.
        target: Target type, e.g. ``"sil"`` or ``"hil"``.
        capabilities: The target's declared capabilities.
        code_version: Git SHA and dirty flag.
        environment: Python version and lock file hash.
        result: Overall outcome.
        assertion_results: Per-assertion outcomes, in scenario order.
    """

    schema_version: int = SCHEMA_VERSION
    run_id: str
    started_utc: str
    scenario_name: str
    scenario_sha256: str
    master_seed: int
    stream_seeds: Mapping[str, int]
    tick_us: int
    target: str
    capabilities: TargetCapabilities
    code_version: CodeVersion
    environment: RuntimeEnvironment
    result: RunResult
    assertion_results: tuple[AssertionResult, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "stream_seeds", MappingProxyType(dict(self.stream_seeds)))
        object.__setattr__(self, "assertion_results", tuple(self.assertion_results))

    def to_dict(self) -> dict[str, Any]:
        """Return the JSON-ready form of the record, with ``schema_version`` first."""
        return {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "started_utc": self.started_utc,
            "scenario_name": self.scenario_name,
            "scenario_sha256": self.scenario_sha256,
            "master_seed": self.master_seed,
            "stream_seeds": dict(self.stream_seeds),
            "tick_us": self.tick_us,
            "target": self.target,
            "capabilities": {
                "deterministic": self.capabilities.deterministic,
                "real_time": self.capabilities.real_time,
                "supported_faults": sorted(self.capabilities.supported_faults),
            },
            "code_version": {
                "git_sha": self.code_version.git_sha,
                "dirty": self.code_version.dirty,
            },
            "environment": {
                "python_version": self.environment.python_version,
                "lock_sha256": self.environment.lock_sha256,
            },
            "result": self.result.value,
            "assertion_results": [
                {"name": a.name, "passed": a.passed, "message": a.message}
                for a in self.assertion_results
            ],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RunRecord":
        """Build a record from its JSON form.

        Raises:
            UnsupportedSchemaVersionError: ``schema_version`` is missing or unsupported.
            RunRecordError: A field is missing, unexpected, or has the wrong type.
        """
        if not isinstance(data, Mapping):
            raise RunRecordError("run record must be a JSON object")
        version = data.get("schema_version")
        if isinstance(version, bool) or version not in SUPPORTED_SCHEMA_VERSIONS:
            raise UnsupportedSchemaVersionError(version)

        expected = set(cls._json_fields())
        missing = sorted(expected - data.keys())
        extra = sorted(data.keys() - expected)
        if missing:
            raise RunRecordError(f"run record missing fields: {', '.join(missing)}")
        if extra:
            raise RunRecordError(f"run record has unexpected fields: {', '.join(extra)}")

        try:
            caps = _obj(data, "capabilities")
            code = _obj(data, "code_version")
            env = _obj(data, "environment")
            dirty = code["dirty"]
            if dirty != UNKNOWN and not isinstance(dirty, bool):
                raise RunRecordError(f"code_version.dirty must be bool or {UNKNOWN!r}")
            return cls(
                schema_version=version,
                run_id=_typed(data, "run_id", str),
                started_utc=_typed(data, "started_utc", str),
                scenario_name=_typed(data, "scenario_name", str),
                scenario_sha256=_typed(data, "scenario_sha256", str),
                master_seed=_typed(data, "master_seed", int),
                stream_seeds={
                    _check(k, str, "stream_seeds key"): _check(v, int, f"stream_seeds[{k!r}]")
                    for k, v in _obj(data, "stream_seeds").items()
                },
                tick_us=_typed(data, "tick_us", int),
                target=_typed(data, "target", str),
                capabilities=TargetCapabilities(
                    deterministic=_typed(caps, "deterministic", bool),
                    real_time=_typed(caps, "real_time", bool),
                    supported_faults=frozenset(
                        _check(f, str, "supported_faults entry")
                        for f in _typed(caps, "supported_faults", list)
                    ),
                ),
                code_version=CodeVersion(git_sha=_typed(code, "git_sha", str), dirty=dirty),
                environment=RuntimeEnvironment(
                    python_version=_typed(env, "python_version", str),
                    lock_sha256=_typed(env, "lock_sha256", str),
                ),
                result=RunResult(_typed(data, "result", str)),
                assertion_results=tuple(
                    AssertionResult(
                        name=_typed(a, "name", str),
                        passed=_typed(a, "passed", bool),
                        message=_typed(a, "message", str),
                    )
                    for a in (
                        _check(item, dict, "assertion_results entry")
                        for item in _typed(data, "assertion_results", list)
                    )
                ),
            )
        except KeyError as exc:
            raise RunRecordError(f"run record missing field: {exc.args[0]}") from None
        except ValueError as exc:
            if isinstance(exc, RunRecordError):
                raise
            raise RunRecordError(f"invalid run record: {exc}") from None

    @staticmethod
    def _json_fields() -> tuple[str, ...]:
        return (
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
        )


def _check[T](value: object, kind: type[T], what: str) -> T:
    if isinstance(value, kind) and not (kind is int and isinstance(value, bool)):
        return value
    raise RunRecordError(f"{what} must be {kind.__name__}, got {type(value).__name__}")


def _typed[T](data: Mapping[str, Any], key: str, kind: type[T]) -> T:
    return _check(data[key], kind, key)


def _obj(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    return _typed(data, key, dict)


def write_run_record(record: RunRecord, path: Path) -> None:
    """Write ``record`` to ``path`` as indented JSON with a trailing newline."""
    path.write_text(json.dumps(record.to_dict(), indent=2) + "\n", encoding="utf-8")


def read_run_record(path: Path) -> RunRecord:
    """Read a ``run.json`` file.

    Raises:
        UnsupportedSchemaVersionError: The file's schema version is unknown.
        RunRecordError: The file is not valid JSON or does not match the schema.
    """
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise RunRecordError(f"{path} is not valid JSON: {exc}") from None
    return RunRecord.from_dict(data)


def new_run_identity(now: datetime | None = None) -> RunIdentity:
    """Create a ``run_id`` and ``started_utc`` from the current wall-clock time.

    This is the only wall-clock read in the run lifecycle.

    Args:
        now: Timezone-aware time to use instead of the current time (for tests).
    """
    if now is None:
        now = datetime.now(UTC)
    elif now.tzinfo is None:
        raise ValueError("now must be timezone-aware")
    now = now.astimezone(UTC)
    return RunIdentity(
        run_id=f"{now:%Y%m%dT%H%M%SZ}-{secrets.token_hex(3)}",
        started_utc=now.isoformat(timespec="microseconds").replace("+00:00", "Z"),
    )


def detect_code_version(repo_dir: Path) -> CodeVersion:
    """Return the git SHA and dirty flag for ``repo_dir``.

    Untracked files count as dirty, since they can change what a run does.
    Returns ``"unknown"`` for both fields if git is unavailable or ``repo_dir`` is not
    inside a git checkout.
    """

    def git(*args: str) -> str:
        return subprocess.run(
            ["git", *args],
            cwd=repo_dir,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()

    try:
        sha = git("rev-parse", "HEAD")
        dirty = bool(git("status", "--porcelain"))
    except (OSError, subprocess.CalledProcessError):
        return CodeVersion(git_sha=UNKNOWN, dirty=UNKNOWN)
    return CodeVersion(git_sha=sha, dirty=dirty)


def lock_file_sha256(lock_path: Path) -> str:
    """Return the SHA-256 hex digest of the dependency lock file, or ``"unknown"`` if absent."""
    try:
        return hashlib.sha256(lock_path.read_bytes()).hexdigest()
    except OSError:
        return UNKNOWN


def detect_environment(project_root: Path) -> RuntimeEnvironment:
    """Return the Python version and the hash of ``project_root / "uv.lock"``."""
    return RuntimeEnvironment(
        python_version=platform.python_version(),
        lock_sha256=lock_file_sha256(project_root / "uv.lock"),
    )
