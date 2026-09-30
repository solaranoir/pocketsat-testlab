#!/usr/bin/env bash
# Creates labels, the Phase 0 milestone, and Phase 0 issues for PocketSat Test Lab.
# Prereqs: GitHub CLI installed and authenticated (gh auth login).
# Usage: run from inside a clone of the repo:  bash bootstrap_phase0.sh
set -euo pipefail

MILESTONE="Phase 0 - Architecture & Skeleton"

# Milestone (ignore error if it already exists)
gh api "repos/{owner}/{repo}/milestones" -f title="$MILESTONE" \
  -f description="Repo skeleton, architecture doc, TestTarget interface, initial CI" >/dev/null 2>&1 || true

# Labels (ignore errors if they already exist)
gh label create "phase-0" --color "0E8A16" --description "Phase 0" 2>/dev/null || true
gh label create "architecture" --color "5319E7" 2>/dev/null || true
gh label create "infra" --color "1D76DB" 2>/dev/null || true
gh label create "docs" --color "0075CA" 2>/dev/null || true
gh label create "agent-ready" --color "FBCA04" --description "Well-scoped; safe to hand to a coding agent" 2>/dev/null || true

issue() {
  gh issue create --title "$1" --body "$2" --label "$3" --milestone "$MILESTONE"
}

issue "Repo hygiene: README stub, LICENSE, .gitignore, AGENTS.md" \
'Set up baseline repo files.

Acceptance criteria:
- README with project goal and architecture diagram placeholder
- Apache-2.0 (or MIT) LICENSE
- Python .gitignore
- AGENTS.md committed at repo root
- Issue and PR templates under .github/' \
"phase-0,docs,agent-ready"

issue "Python project skeleton: pyproject.toml, src layout, tooling" \
'Create the package skeleton.

Acceptance criteria:
- pyproject.toml (Python 3.12+), src/pocketsat package, uv-managed deps
- ruff, mypy (strict on src), pytest configured
- One passing smoke test
- pocketsat --version CLI entry point works' \
"phase-0,infra,agent-ready"

issue "Write docs/architecture.md: component boundaries and data flow" \
'Document boundaries among spacecraft, environment, RF channel, ground station, orchestrator, and test target.

Acceptance criteria:
- Responsibilities and non-responsibilities for each component
- Data-flow diagram (Mermaid)
- Message types named: Command, Telemetry, Packet
- SIL vs HIL paths shown' \
"phase-0,architecture,docs"

issue "ADR-0001 record architecture decisions; ADR-0002 TestTarget abstraction" \
'Create docs/adr/ with a template and the first two ADRs.

Acceptance criteria:
- ADR template (context, decision, consequences)
- ADR-0001: use ADRs
- ADR-0002: SIL/HIL abstraction via a single TestTarget interface, alternatives considered' \
"phase-0,architecture,docs"

issue "ADR-0003: time model and determinism rules" \
'Decide how simulated time and randomness work before any subsystem is written.

Acceptance criteria:
- Simulated clock vs wall clock decision recorded
- Seeded RNG injection approach recorded
- Run ID and seed recording format sketched' \
"phase-0,architecture,docs"

issue "Define TestTarget protocol and core message types" \
'Implement pocketsat.targets.base and shared message types.

Acceptance criteria:
- TestTarget protocol: connect, reset(seed), send_command, read_telemetry(timeout), advance(dt), close
- Command, Telemetry, Packet as typed dataclasses
- Full type hints and docstrings; mypy strict passes' \
"phase-0,architecture,agent-ready"

issue "Contract test suite and stub target for TestTarget" \
'Create a reusable contract test suite any target must pass, plus a trivial stub target.

Acceptance criteria:
- tests/ contains target-agnostic contract tests parameterized over targets
- EchoTarget (stub) passes them
- Documented so SilTarget and HilTarget can reuse the suite later' \
"phase-0,architecture,agent-ready"

issue "Initial CI: GitHub Actions for lint, types, tests" \
'Add .github/workflows/ci.yml.

Acceptance criteria:
- Runs ruff check, ruff format --check, mypy, pytest on pull requests and main
- Uses uv with dependency caching
- Status badge added to README' \
"phase-0,infra,agent-ready"

issue "Project board and milestones for Phases 1-7" \
'Set up the GitHub Project board and create milestones (no issues yet) for Phases 1-7 so work can be planned per phase.

Acceptance criteria:
- Project board with Backlog / In progress / Review / Done
- Milestones for Phases 1-7 created
- Phase 0 issues added to the board' \
"phase-0,infra"

echo "Done. Review issues with: gh issue list --milestone \"$MILESTONE\""
