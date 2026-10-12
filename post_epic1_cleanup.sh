#!/usr/bin/env bash
# Post-Epic-1 cleanup, from the Phase 1 critique:
#   A. New task: FlightComputer skeleton and step() interface (Epic 2)
#   B. New task: SilTarget behavioral tests, end to end (Epic 3)
#   C. New task: Phase 1 documentation and hygiene cleanup (Epic 3)
#   D. Comments tightening acceptance criteria on #56, #61, #63
#
# Usage (from inside the repo):
#   DRY_RUN=1 bash post_epic1_cleanup.sh    # preview; creates and posts nothing
#   bash post_epic1_cleanup.sh              # run ONCE
#
# Issue numbers for D come from the order the Phase 1 script created issues. Each is
# checked against an expected title keyword and skipped with a warning on mismatch.
set -euo pipefail

MILESTONE="Phase 1 - Virtual Spacecraft"
DRY_RUN="${DRY_RUN:-0}"

COMMON='Definition of done: follows AGENTS.md (one issue per PR, branch phase-1/<short-name>); tests included; ruff and mypy strict pass; no wall-clock time or global random in simulation code (use SimClock and named RngFactory streams per ADR-0003).'

ISSUES_JSON=""
if [ "$DRY_RUN" != "1" ]; then
  ISSUES_JSON=$(gh issue list --milestone "$MILESTONE" --state all --limit 200 --json number,title)
fi

find_issue() { # title-regex -> issue number or empty
  [ "$DRY_RUN" = "1" ] && { echo "?"; return; }
  printf '%s' "$ISSUES_JSON" | python3 -c '
import json, re, sys
pat = sys.argv[1]
for i in json.load(sys.stdin):
    if re.search(pat, i["title"], re.I):
        print(i["number"]); break
' "$1"
}

new_task() { # title labels parent-title-regex body
  local title="$1" labels="$2" parent_re="$3" body="$4" parent n url pbody id
  parent=$(find_issue "$parent_re")
  if [ "$DRY_RUN" = "1" ]; then
    echo "WOULD CREATE: $title  (parent matching: $parent_re)"; return 0
  fi
  if [ -z "$parent" ]; then
    echo "WARN: no parent found for '$parent_re'; creating '$title' without a parent link"
  fi
  url=$(gh issue create --title "$title" --label "$labels" --milestone "$MILESTONE" \
    --body "$body${parent:+

Part of #$parent.}

$COMMON")
  n="${url##*/}"
  echo "Created #$n: $title"
  if [ -n "$parent" ]; then
    # sub-issue link (best effort)
    id=$(gh api "repos/{owner}/{repo}/issues/$n" --jq .id 2>/dev/null || true)
    [ -n "$id" ] && gh api -X POST "repos/{owner}/{repo}/issues/$parent/sub_issues" -F sub_issue_id="$id" >/dev/null 2>&1 || true
    # checklist entry in the parent body
    pbody=$(gh issue view "$parent" --json body --jq .body)
    gh issue edit "$parent" --body "$pbody
- [ ] #$n $title" >/dev/null
  fi
  sleep 1
}

note() { # number expected-title-keyword body
  local n="$1" kw="$2" body="$3" title
  title=$(gh issue view "$n" --json title --jq .title 2>/dev/null) || { echo "SKIP #$n: could not read issue"; return 0; }
  if ! printf '%s' "$title" | grep -qi -- "$kw"; then
    echo "SKIP #$n: title is '$title', expected to contain '$kw'"; return 0
  fi
  if [ "$DRY_RUN" = "1" ]; then echo "WOULD COMMENT on #$n: $title"; return 0; fi
  gh issue comment "$n" --body "Post-epic review update. This comment tightens the acceptance criteria above; where they conflict, this comment wins.

$body" >/dev/null
  echo "Commented on #$n: $title"
  sleep 1
}

# =============================================================================
# A. FlightComputer skeleton
# =============================================================================
new_task "FlightComputer skeleton and step() interface" "phase-1,task,spacecraft,agent-ready" "^Story: Mode state machine" \
'Nothing currently owns the flight computer as a component. The mode machine, command dispatcher, telemetry scheduler, and downlink logic are separate tickets; this ticket creates the single place they plug into, per ADR-0004 steps c to f.

Do this first in Epic 2. It blocks the mode state machine, safe-mode rules, BOOT/RESET, command dispatcher, telemetry scheduler, and DOWNLINK transfer tickets.

Acceptance criteria:
- pocketsat.spacecraft.flight_computer.FlightComputer with a single step() entry point. Inputs: queued uplink frames, the current SpacecraftState (readings records), and simulated time. Outputs: downlink frames and the SpacecraftControls for the next tick
- Mode enum with the six values BOOT, NOMINAL, SCIENCE, DOWNLINK, SAFE, FAULT (values only; transitions belong to the mode state machine ticket)
- step() runs fixed internal phases in this order, each a small method that is a documented no-op or pass-through for now: decode uplink, execute commands, evaluate flags, update mode, emit telemetry, produce controls. Later tickets fill in one method each
- Controls are produced by ONE function (the documented seam from ADR-0004 section 12)
- reset() puts the flight computer in BOOT and returns BOOT controls, which apply to tick 0
- A reboot() method that clears transient state, resets uptime, and increments the boot counter; RESET and forced_reset will both call it
- The flight computer is not part of STEP_ORDER; docstring points to ADR-0004
- Tests: phases run in the specified order (spy test), reset returns BOOT controls, step is deterministic, uplink frames pass through the no-op stages without error
- Full type hints and docstrings; mypy strict passes'

# =============================================================================
# B. SilTarget behavioral tests
# =============================================================================
new_task "SilTarget behavioral tests (end to end, through the target)" "phase-1,task,spacecraft,agent-ready" "^Story: Nominal scenario and Phase 1 milestone" \
'Subsystem and table-driven flight computer tests do not catch integration bugs. Add behavior-level tests that drive SilTarget only through the TestTarget interface, written as given/when/then.

Depends on: SilTarget, target fault support, command dispatcher, mode rules, telemetry encoder, and DOWNLINK transfer tickets.

Acceptance criteria:
- Tests live under tests/sil/behavior/ and use only the TestTarget interface (send, advance, receive, apply_environment, inject, reset); no direct access to subsystems or controls
- A small helper (builder or fixtures) keeps each test readable as given/when/then
- Scenarios covered:
  1. Sustained critical battery leads to SAFE: payload disabled, radio restricted, telemetry reports SAFE
  2. BEGIN_DOWNLINK with payload data buffered leads to DOWNLINK mode and DATA frames; payload chunks are released only after their frames are transmitted
  3. RESET command shows BOOT controls one tick later; forced_reset fault shows them on the same tick (the ADR-0004 asymmetry as an explicit regression test)
  4. sensor_freeze via inject(): telemetry shows a frozen reading while the true value keeps changing (use a test-only accessor for truth)
  5. A command received during BOOT is NACKed (except PING and RESET)
  6. inject() with an unsupported fault type raises a clear error
- Each test names the ADR section or ticket whose behavior it pins'

# =============================================================================
# C. Documentation and hygiene cleanup
# =============================================================================
new_task "Phase 1 documentation and hygiene cleanup" "phase-1,task,docs,agent-ready" "^Story: Nominal scenario and Phase 1 milestone" \
'Documentation drifted ahead of the implementation. Fix it so the repo says only what is true. ADR-0001 allows clarifications to an accepted ADR, so no new ADR is needed.

Acceptance criteria:
- ADR-0004: change statements that planned work is implemented (for example "implemented in #47, #59, #60") to "to be implemented in". Add an implementation-status table with columns Item, Status (Implemented or Planned), and Ticket. SpacecraftControls and Subsystem.step(controls) are Implemented; the flight computer controls function, fault merge and expiry, steps a to f in SilTarget, and the telemetry encoder are Planned with their ticket numbers
- Roadmap status: Phase 1 changed from "Not started" to "In progress", with a checklist of what is done and what remains
- README: "Version 1 covers ..." becomes "Version 1 will cover ..."; fault injection described as "subsystem fault effects implemented; target-level injection pending"
- tests/contract/README.md: say which targets are actually registered today; do not claim SilTarget or HilTarget coverage until they are registered
- Remove the stale comment in demo build_stack() about name: Final and the Subsystem protocol (fixed by the read-only protocol member change); type the tuple properly if mypy now allows it
- pocketsat.__init__: fall back to "0.0.0+unknown" when the package metadata is not found, so importing from a bare source tree works
- No behavior changes beyond the version fallback'

# =============================================================================
# D. Tightening comments
# =============================================================================
note 56 "DOWNLINK" \
'Tightened acceptance criteria:
- A payload data chunk is released from the payload buffer only when its DATA frame has actually been transmitted (subject to Comms capacity and power flags) and appears in SilTarget.receive(). No timer-based release.
- A test shows that disabling the transmitter (radio off or the transmitter_off fault) stalls chunk release and keeps the data in the buffer.
- The demo scripted chunk release should be replaced by this path once SilTarget exists.'

note 61 "Contract" \
'Tightened acceptance criteria:
- Register SilTarget in the contract suite alongside EchoTarget (parametrized fixtures). HilTarget is a placeholder that is skipped with an explicit reason until Phase 7.
- Contract tests that depend on capabilities EchoTarget lacks are selected by the declared capabilities, not hard-coded per target.
- tests/contract/README.md is updated to list exactly the targets registered; it must not claim coverage that does not exist.'

note 63 "Nominal scenario" \
'Tightened acceptance criteria: this test is the Phase 1 definition of done (the roadmap milestone: run a nominal scenario and receive deterministic spacecraft telemetry).
- Named test_phase1_nominal_spacecraft, driving SilTarget through the TestTarget interface only: connect, reset(seed=42), apply_environment, send PING and receive an ACK, SET_MODE to SCIENCE, BEGIN_DOWNLINK.
- Asserts the mode sequence BOOT, NOMINAL, SCIENCE, DOWNLINK, telemetry cadence, payload data generated, and that data actually downlinked as DATA frames.
- Asserts no SAFE or FAULT entry in the nominal run.
- Determinism: two runs with the same seed produce byte-identical downlink frames; a different seed produces different sensor-noise values.
- Runs in CI in under 10 seconds.
- If a harness that bypasses SilTarget remains, it is a helper only, not the acceptance test.'

echo
if [ "$DRY_RUN" = "1" ]; then
  echo "Dry run only: nothing created or posted."
else
  echo "Done. Review: gh issue list --milestone \"$MILESTONE\" --limit 100"
fi
