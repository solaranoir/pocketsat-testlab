# ADR-0003: Time model, tick scheduling, seeding, and run records

- **Status:** Accepted
- **Phase:** 0
- **Amends:** ADR-0002 (`advance` signature)

## Context

Phase 6 promises that any failed run in a campaign of thousands can be reproduced exactly. That promise is cheap to keep if the time model and randomness are designed up front, and very expensive to retrofit. Five questions need answers before any subsystem code is written:

1. How is simulated time represented and advanced?
2. In what order do things happen within one step, so the same inputs always produce the same outputs?
3. How is randomness seeded and partitioned, so adding a new random source later does not change existing runs?
4. What identifies and describes a run so it can be replayed?
5. How does the same loop work against an MCU that runs in real time?

## Decision

### 1. Fixed-tick, lockstep simulation with integer time

- The orchestrator runs a **fixed-tick loop**. Simulated time is an **integer count of microseconds** held by a `SimClock`. There is no floating-point time in the simulation, so there is no drift and no platform-dependent rounding.
- Default tick is **100 ms** (`tick_us = 100_000`), overridable per scenario. A 10-minute pass is then 6,000 ticks, which is fast in SIL.
- All scheduled events (commands, faults, pass boundaries) are specified in time units and quantized to tick boundaries when a scenario is loaded. Sub-tick RF latency is rounded up to the next tick boundary; this quantization is documented and recorded in the run record.
- **Amendment to ADR-0002:** `advance(dt: float)` becomes `advance(dt_us: int)`. The rest of the `TestTarget` interface is unchanged.

### 2. Fixed order of operations within a tick

Each tick executes these steps, always in this order:

1. Activate and expire scheduled faults (RF and ground-station faults take effect in their components; target faults go through `inject()`).
2. Step the environment model to produce `EnvironmentState`.
3. Issue scheduled commands from the scenario to the ground station; the ground station encodes frames and hands them to the RF channel.
4. RF channel delivers **due** uplink packets to the target via `send()`.
5. `target.apply_environment(env)`.
6. `target.advance(dt_us)`.
7. Drain `target.receive()` into the RF channel, which schedules downlink delivery (latency, loss, corruption).
8. RF channel delivers **due** downlink packets to the ground station.
9. Ground station decodes; decoded telemetry and events go to the orchestrator.
10. Evaluate assertions that are due, then record tick data.

Components are processed in a fixed, documented order, and any collection iterated during a tick (queues, fault lists) has a defined order (time, then insertion sequence). No step depends on dictionary or set ordering.

### 3. Named, derived random streams

- Every run has one **master seed**. Each random consumer requests a **named stream** from an `RngFactory`, for example `rf.loss`, `rf.jitter`, `env.sensor_noise`.
- A stream's seed is derived deterministically from the master seed and the stream name, using the first 8 bytes of `SHA-256(f"{master_seed}:{stream_name}")` as an integer.
- Because streams are independent, **adding a new random consumer does not change the values seen by existing ones.** Existing runs stay reproducible after the code grows.
- v1 uses the standard library `random.Random` behind the `RngFactory`, so the generator can be swapped later without touching consumers. Consumers are restricted to a small set of methods (`random`, `uniform`, `gauss`) whose output is stable for a given seed.
- Simulation code never uses the global `random` module state and never reads wall-clock time.
- **Campaigns:** run *i* of a campaign gets `derive(campaign_seed, f"run:{i}")` as its master seed, so any single run can be replayed by itself without executing the rest of the campaign.

### 4. Run identity and run record

Two identifiers serve two purposes:

- **`run_id`**: unique per execution, for naming artifacts (UTC timestamp plus short random suffix). Wall-clock time appears **only here and in the record's metadata**, never inside the simulation.
- **Replay key**: the tuple that determines a SIL run's behavior: scenario content hash, master seed, tick size, target type and version, and code version.

Every run writes a `run.json` with:

| Field | Purpose |
|---|---|
| `schema_version` | Record format version |
| `run_id`, `started_utc` | Identity and metadata |
| `scenario_name`, `scenario_sha256` | Exactly which scenario ran |
| `master_seed`, `stream_seeds` | Seeds, including each derived stream |
| `tick_us` | Time resolution |
| `target` and `capabilities` | SIL or HIL, determinism flags |
| `code_version` | Git SHA plus a dirty-tree flag |
| `environment` | Python version and a hash of the dependency lock file |
| `result`, `assertion_results` | Outcome |

`pocketsat replay <run.json>` reruns the recorded scenario with the recorded seed and verifies the replay key matches. For SIL runs the expected result is an identical outcome and identical telemetry; the replay command checks this and reports any divergence.

### 5. HIL uses the same loop with wall-clock pacing

- The loop and step order are identical. The difference is pacing: for a real-time target, `advance(dt_us)` blocks until a **deadline** (tick start plus `dt_us` of real time), then drains the serial buffer. Deadline-based scheduling avoids cumulative drift from repeated sleeps.
- HIL runs are **repeatable, not deterministic** (`capabilities.deterministic = False`). The run record stores the seed and scenario as usual, and HIL also logs measured tick jitter and raw serial timestamps so a failure can be analyzed even if it cannot be replayed bit-for-bit.

## Alternatives considered

- **Discrete-event simulation.** More efficient and more natural for sparse events, but it complicates lockstep with a real-time MCU and makes ordering rules harder to see. Rejected for v1; a tick loop is easier to reason about and to debug.
- **Floating-point seconds for time.** Simple, but accumulates rounding error and invites nondeterminism. Rejected.
- **One shared RNG.** Any new consumer or changed call order silently alters every downstream value. Rejected in favor of named streams.
- **Seeding only the campaign, not each run.** Reproducing one failure would require replaying everything before it. Rejected.

## Consequences

- SIL reproducibility is a testable property: replay tests can assert identical telemetry for the same replay key, and this becomes a CI check.
- Quantizing events to ticks means very fine timing effects (under 100 ms by default) are not representable. This is acceptable for the pass-level behavior in v1, and `tick_us` can be lowered for specific scenarios.
- The fixed step order is part of the contract. Changing it changes recorded behavior, so it must be done through a new ADR.
- Some small foundation code is needed before Phase 1: `SimClock`, `RngFactory`, and the run record writer.

## Follow-ups

- Amend ADR-0002 and issue #6 so `advance` takes `dt_us: int`.
  - **Done** for ADR-0002 (#25: "Amended by" header and interface note). Issue #6 was already closed, so it was not changed.
  - **Done** in code by #26: `TestTarget.advance(dt_us: int)`, `EchoTarget`, and the contract suite all use integer microseconds.
  - **Clarification (#26):** following §1's "no floating-point time" rule, `TargetFault.duration_s: float` became `duration_us: int | None`. Scenario durations written in seconds are converted to integer microseconds when the scenario is loaded, like every other scheduled time.
- Create a Phase 0 issue for `SimClock`, `RngFactory`, and the `run.json` schema (with tests for stream independence and seed derivation).
  - **Done:** #19, implemented in #22.
- Update `docs/architecture.md` sections 7 and 8 to reference the step order above.
  - **Done** in #23.
- **Amended by ADR-0006** (#75): §3's list of stable methods is superseded. `gauss()` calls the platform maths library and is not portable across platforms; consumers use `random()`, `uniform()`, and `portable_normal()`. The original text above stays as written.
