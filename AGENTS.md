# AGENTS.md — PocketSat Test Lab

Instructions for AI coding agents (and humans) working in this repository.

## What this project is

A Python-based Software/Hardware-in-the-Loop (SIL/HIL) test platform for a simulated spacecraft, a simulated RF channel, and an amateur-radio-style ground station. The point is **test infrastructure**: scenarios, orchestration, fault injection, reproducibility, reporting.

**Core principle:** the scenario runner talks only to the `TestTarget` interface. The same scenario must run against the Python simulator (SIL) or an MCU (HIL) without changes.

## Repository layout

```
pocketsat-test-lab/
├── AGENTS.md
├── README.md
├── LICENSE
├── pyproject.toml
├── docs/
│   ├── architecture.md          # component boundaries, data flow
│   ├── roadmap.md               # phase plan, milestones, v1 scope
│   └── adr/                     # architecture decision records (NNNN-title.md)
├── src/pocketsat/
│   ├── core/                    # SimClock, RngFactory (determinism foundations)
│   ├── environment/             # environment models producing EnvironmentState
│   ├── targets/                 # TestTarget protocol, SilTarget, HilTarget
│   ├── spacecraft/              # power, thermal, attitude, payload, comms, flight computer
│   ├── groundstation/           # pass state, radio control, uplink, decoding
│   ├── rf/                      # channel model (elevation, range, Doppler, SNR, loss)
│   ├── faults/                  # reusable fault injectors
│   ├── scenarios/               # YAML DSL loader, schema, assertions
│   ├── orchestrator/            # scenario runner
│   ├── campaigns/               # seeded batch runs
│   ├── reporting/               # results, plots, artifacts
│   └── cli.py                   # `pocketsat` entry point
├── firmware/                    # MCU flight software (Phase 7)
├── scenarios/                   # YAML scenario library
├── tests/
│   ├── unit/
│   ├── contract/                # TestTarget contract suite, run against every target
│   ├── sil/
│   └── scenarios/
└── .github/
    ├── workflows/ci.yml
    └── ISSUE_TEMPLATE/
```

Only create directories for the phase you are working on. Do not scaffold future phases.

## Rules that protect the architecture

1. **Orchestrator never imports a concrete target.** It depends on `pocketsat.targets.base.TestTarget` only.
2. **Determinism is non-negotiable.** No `time.time()`, `datetime.now()`, or unseeded `random` in simulation code. Use the injected simulated clock and an injected `random.Random(seed)` / `numpy.random.Generator`. Every run records its seed and run ID.
3. **Simulated time, not wall-clock time.** Simulation advances via an explicit clock so runs are fast and reproducible. Wall-clock use is limited to HIL I/O timeouts.
4. **Components communicate through typed messages** (`Command`, `Telemetry`, `Packet`), not shared mutable state.
5. **Faults are reusable objects**, not ad hoc code paths inside subsystems.
6. **Scenarios are data (YAML).** Do not require Python edits to add a scenario.

## Tooling

- Python 3.12+, `src/` layout, dependencies managed with `uv`
- Lint/format: `ruff check` and `ruff format`
- Types: `mypy --strict` on `src/`
- Tests: `pytest`

Before opening a PR, run:

```
uv run ruff check . && uv run ruff format --check . && uv run mypy src && uv run pytest
```

Multi-orbit story and integration tests are marked `@pytest.mark.slow` and skipped by a plain `pytest` run. If your change touches simulation behavior, also run `uv run pytest -m slow`; CI runs them on every PR in a separate job. Mark any test that takes more than about a second `slow`.

## Workflow

- **One issue per PR.** Branch name: `phase-N/short-description` (e.g. `phase-0/test-target-interface`).
- Reference the issue in the PR (`Closes #N`).
- Stay in scope: do not implement later-phase features or refactor unrelated code.
- Add or update tests with every behavior change. Bug fixes get a regression test.
- Architecture-affecting decisions get an ADR in `docs/adr/` (context, decision, consequences). If an issue forces a choice between designs, write the ADR in the same PR.
- Keep PRs small and reviewable. A human reviews and merges; agents do not merge their own PRs.
- Commit style: imperative, concise subject line (`Add TestTarget protocol`).

## Definition of done

- Acceptance criteria in the issue are met
- CI is green
- New public interfaces have docstrings and type hints
- Relevant docs or ADRs updated
- No new dependencies without a stated reason in the PR description

## Scope reminder

Version 1 is the 10-week plan in `docs/roadmap.md`: SIL + MCU HIL, scenario DSL, fault injection, seeded campaigns, CI. Deferred to v2: SDR, extensive physical sensors, sophisticated orbital mechanics, real-satellite validation, Kubernetes, anomaly detection. A finished, documented SIL/HIL platform beats a sprawling unfinished lab.
