# ADR-0005: Spacecraft configuration and initial state

- **Status:** Proposed
- **Phase:** 1
- **Amends / Supersedes:** None. Related: ADR-0002 (`TestTarget`, unsupported faults), ADR-0003 (determinism, run records), ADR-0004 (per-subsystem record pattern). Issue: #74.

## Context

Nothing defined where the spacecraft starts (initial battery charge, temperatures, attitude rate, buffer fill) or how a scenario changes it. `Subsystem.reset(rng)` implied fixed defaults, and `TestTarget.reset(seed)` carries only a seed. Separately, each subsystem was going to get its own settings dataclass (#35, #38), but nothing collected them into one spacecraft-wide configuration. Scenario overrides (Phase 5) and the run record (Phase 6) will need one place to find both.

Both problems have the same shape, so this ADR settles them together.

## Decision

### 1. Settings and starting state are separate

- **Settings** (`SpacecraftConfig`) describe what the spacecraft *is*: capacities, array size, loads, thresholds, heater setpoints.
- **Starting state** (`SpacecraftInitialState`) describes where it *starts*: battery charge, temperatures, attitude, buffer fill.

Both are frozen collections of per-subsystem frozen records, following the pattern of ADR-0004's controls and snapshot convention:

| Collection | Records |
|---|---|
| `SpacecraftConfig` | `PowerConfig`, `ThermalConfig`, `AttitudeConfig`, `PayloadConfig`, `CommsConfig` |
| `SpacecraftInitialState` | `PowerInitial(soc)`, `ThermalInitial(battery_c, electronics_c)`, `AttitudeInitial(pointing_error_deg, rate_dps)`, `PayloadInitial(buffer_fill)` |

Communications has no starting record: it holds no data (ADR-0004 §13). Starting records validate their values (for example SOC in 0..1, temperatures above absolute zero). Each subsystem reads only its own records.

The nominal settings are the named instance `NOMINAL_CONFIG`; the power and thermal budget (#72) calibrates them and adds a stressed set. `DEFAULT_INITIAL_STATE` is the default starting state.

### 2. Each subsystem is built with its records; `reset(rng)` returns to them

A subsystem receives its settings record and its starting record when it is constructed. `reset(rng)` restores the starting record and requests its named random streams. The `Subsystem` protocol does not change.

### 3. Supplied when the target is created, not through `TestTarget.reset()`

A scenario builds `SilTarget(config=..., initial=...)` through a factory, and `reset(seed)` returns to exactly what the target was built with. `TestTarget` and ADR-0002 are unchanged. Single values can be overridden on top of a named set with `dataclasses.replace()`; how scenarios express that in YAML is a Phase 5 decision.

### 4. HIL rule

A scenario that sets starting conditions on a target that can't honor them fails up front, like an unsupported fault (ADR-0002). Whether HIL can emulate them through the sensor-input channel is decided in Phase 7b.

### 5. The flight computer always starts in BOOT

Starting state covers physical things only. There is no "start in SCIENCE" in v1; a scenario that wants SCIENCE commands it after boot.

### 6. Starting charge versus `battery_soc_override`

- The **starting charge** (`PowerInitial.soc`) begins the run at a value and lets physics proceed. Use it for "start low and see what happens".
- The **override** (`EnvironmentState.battery_soc_override`) pins the charge while it is set. It is a test lever for flags and edge cases.

### 7. Starting values are plain values, never random

Subsystems never randomize their own starting state. Campaigns (Phase 6) generate varied starting values from seeded streams and pass them in. A later run-record schema version should record the configuration and starting state each run used; there is no schema change now (see `docs/roadmap.md`, Phase 6).

## Alternatives considered

- **Starting values folded into the settings.** Simpler, but it blurs #72's nominal and stressed sets with "starts cold" or "starts nearly empty". Rejected.
- **Extending `TestTarget.reset()` to take starting conditions.** Changes ADR-0002, and HIL can't set a real battery on command. Rejected.
- **Subsystems randomizing their own starting state from their streams.** Hides variation inside the model, where scenarios and run records can't see or replay it. Rejected; campaigns generate the values instead.
- **A "start in SCIENCE" option.** Would bypass the BOOT rules (#49, #51) and the tick-0 BOOT controls (ADR-0004 §2). Deferred until boot time slows scenarios down.

## Consequences

- Settings and starting state each have one home, and scenarios supply both the same way.
- `TestTarget` stays uniform across SIL and HIL; starting conditions are a SIL capability unless a HIL target declares support.
- Runs stay reproducible: the same configuration, starting state, and seed give the same run.
- Each subsystem ticket adds its fields to its settings record and finalizes its starting defaults: power and thermal (#35, #36, #38, #39, values from #72), attitude (#41), payload (#43), comms (#44). The current starting defaults are provisional.
- The run record does not yet capture configuration or starting state, which matters once campaigns vary them (Phase 6).

## Follow-ups

- `docs/architecture.md` §4.1 describes settings, starting state, and how scenarios supply them (in the same change as this ADR).
- #72 adds the stressed configuration and sets the final power and thermal starting values; #41 sets the attitude defaults.
- #59: `SilTarget` is built with `config` and `initial`, and `reset(seed)` restores them.
- Phase 5 (scenario DSL) and Phase 6 (run record) follow-ups are noted in `docs/roadmap.md`.
