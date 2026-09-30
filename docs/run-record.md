# Run record (`run.json`)

Status: Phase 0. Implements ADR-0003 section 4. Python implementation:
`pocketsat.reporting.run_record`.

Every run writes one `run.json`. It identifies exactly what ran, with which seeds, on
which code, and with what outcome, so any run can be replayed.

## Schema version 1

Fields appear in this order. All are required.

| Field | Type | Description |
|---|---|---|
| `schema_version` | int | Record format version. Currently `1`. |
| `run_id` | string | Unique per execution: `<UTC yyyymmddThhmmssZ>-<6 hex chars>`, e.g. `20260930T181500Z-3fa9c1`. |
| `started_utc` | string | ISO 8601 UTC start time with microseconds, e.g. `2026-09-30T18:15:00.123456Z`. |
| `scenario_name` | string | Scenario name. |
| `scenario_sha256` | string | SHA-256 hex digest of the scenario file contents. |
| `master_seed` | int | The run's master seed. |
| `stream_seeds` | object | Stream name → derived seed, for every stream requested during the run, in first-request order. |
| `tick_us` | int | Tick length in microseconds. |
| `target` | string | Target type, e.g. `"sil"`, `"hil"`. |
| `capabilities` | object | `deterministic` (bool), `real_time` (bool), `supported_faults` (sorted list of strings). |
| `code_version` | object | `git_sha` (string) and `dirty` (bool). Both are `"unknown"` outside a git checkout. Untracked files count as dirty. |
| `environment` | object | `python_version` (string) and `lock_sha256` (SHA-256 of `uv.lock`, or `"unknown"`). |
| `result` | string | `"passed"`, `"failed"`, or `"error"`. |
| `assertion_results` | array | Objects with `name` (string), `passed` (bool), `message` (string), in scenario order. |

Example (hashes abbreviated):

```json
{
  "schema_version": 1,
  "run_id": "20260930T181500Z-3fa9c1",
  "started_utc": "2026-09-30T18:15:00.123456Z",
  "scenario_name": "low_elevation_pass",
  "scenario_sha256": "9f2c…",
  "master_seed": 42,
  "stream_seeds": {"rf.loss": 2733109445733428389},
  "tick_us": 100000,
  "target": "sil",
  "capabilities": {"deterministic": true, "real_time": false, "supported_faults": ["mute"]},
  "code_version": {"git_sha": "0690116…", "dirty": false},
  "environment": {"python_version": "3.12.14", "lock_sha256": "4e1a…"},
  "result": "passed",
  "assertion_results": [{"name": "mode_never: FAULT", "passed": true, "message": ""}]
}
```

## Reading and versioning

- Readers reject a missing or unknown `schema_version` with `UnsupportedSchemaVersionError`,
  naming the versions they support.
- Readers reject missing fields, unexpected fields, and wrong types with `RunRecordError`.
- Any change to the fields or their meaning bumps `schema_version`.

## Wall-clock time

`run_id` and `started_utc` are the only wall-clock values in a run, and they are
generated in the reporting layer (`new_run_identity()`). Simulation code under
`src/pocketsat`, outside `pocketsat.reporting`, may not read wall-clock time or use
global `random` state. `tests/unit/test_determinism_guard.py` enforces this.

## Seeds

Stream seeds are derived as described in ADR-0003: the first 8 bytes of
`SHA-256(f"{master_seed}:{stream_name}")`, read as a big-endian unsigned integer
(`pocketsat.core.rng.derive_seed`). Campaign run *i* uses
`derive_seed(campaign_seed, f"run:{i}")` as its master seed.
