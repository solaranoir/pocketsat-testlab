# PocketSat Test Lab — Roadmap

Status: Phase 0 complete. Adapted from the original project roadmap, updated for the
decisions recorded since (ADR-0002, ADR-0003). Work is tracked per phase as GitHub
milestones and on the "PocketSat Test Lab" project board.

## Goal

Build a Python-based automated spacecraft test infrastructure capable of running
repeatable mission scenarios against simulated and physical components, including a
simulated RF communications link between a spacecraft and an amateur-radio-style
ground station.

**Core principle:** the scenario runner uses a common test interface (`TestTarget`), so
the same mission scenarios run against the software simulator (SIL) or physical
hardware (HIL). See `docs/architecture.md`.

## Version 1 phases

| Phase | Name | Estimate | Status |
|---|---|---|---|
| 0 | Architecture & Skeleton | 1 weekend | **Done** |
| 1 | Virtual Spacecraft | 1–2 weeks | Not started |
| 2 | Ground Station | ~1 week | Not started |
| 3 | Simulated RF Channel | 1–2 weeks | Not started |
| 4 | RF Fault Injection | ~1 week | Not started |
| 5 | Mission Scenario DSL | ~1 week | Not started |
| 6 | Deterministic Chaos Campaigns | ~1 week | Not started |
| 7a | Minimal MCU Firmware HIL | Re-estimate pending (ADR-0002) | Not started |
| 7b | HIL Test Channels | Re-estimate pending (ADR-0002) | Not started |
| 12 | CI/CD Test Infrastructure | Throughout | Initial CI done in Phase 0 |

Phases 0–7b each have a GitHub milestone of the same name. Phase 12 runs alongside the
others and has no milestone of its own.

### Phase 0 — Architecture & Skeleton (done)

Define boundaries among spacecraft, environment, RF channel, ground station,
orchestrator, and test target. Create the repository structure and architecture
decision records. Establish the SIL/HIL abstraction before detailed spacecraft behavior.

**Deliverable:** repository skeleton, architecture document, test-target interface,
initial CI.

Delivered:

- Repository skeleton, tooling, and CI: #1, #2, #8
- `docs/architecture.md`: #3
- ADR-0001 (use ADRs), ADR-0002 (TestTarget abstraction, wire format), ADR-0003 (time
  model and determinism), all Accepted: #4, #5
- `TestTarget` protocol, message types, frame codec, and shared test vectors: #6
- Contract test suite and `EchoTarget` stub: #7
- `SimClock`, `RngFactory`, and the `run.json` run record: #19
- `advance(dt_us)` in integer microseconds: #26
- Project board and Phase 1–7 milestones: #9

Phase 0 grew beyond its original weekend estimate: the determinism foundations
(ADR-0003) were pulled forward from later phases so every subsystem is built on them.

### Phase 1 — Virtual Spacecraft

Create a Python spacecraft simulator with Power, Thermal, Attitude, Payload,
Communications, and Flight Computer subsystems. Model BOOT, NOMINAL, SCIENCE, DOWNLINK,
SAFE, and FAULT modes. Generate timestamped telemetry and accept commands such as
`PING`, `SET_MODE`, `BEGIN_DOWNLINK`, `ENTER_SAFE_MODE`, and `RESET`.

**Milestone:** run a nominal scenario and receive deterministic spacecraft telemetry.

Builds on: `SilTarget` must pass the contract suite (`tests/contract/`); command IDs and
the telemetry layout are filled in in `docs/protocol.md`.

### Phase 2 — Ground Station

Build a simplified amateur-satellite ground station with pass state, radio control,
command uplink, telemetry decoding, and mission-console output. Introduce AOS, LOS, SNR,
Doppler, azimuth/elevation, frequency, packet decoding, and station identity.

**Milestone:** send commands and receive decoded telemetry through a ground-station
interface.

### Phase 3 — Simulated RF Channel

Insert an RF channel simulator between spacecraft and ground station. Each packet
receives link-state attributes such as elevation, range, Doppler shift, SNR,
packet-loss probability, and latency. Start with deterministic models and later refine
the link budget.

**Milestone:** telemetry quality naturally improves and deteriorates over a simulated
LEO pass.

### Phase 4 — RF Fault Injection

Add packet loss, corruption, latency, jitter, interference, complete outage, frequency
offset, Doppler tracking error, ground-station failure, and spacecraft transmitter
failure as reusable test conditions.

**Milestone:** repeatable off-nominal communications scenarios with explicit expected
behavior.

### Phase 5 — Mission Scenario DSL

Create YAML-defined scenarios describing initial state, pass geometry, commands,
injected faults, timing, and assertions. The orchestrator interprets these scenarios
rather than requiring Python for every campaign.

**Milestone:** another engineer could author a mission test from documented YAML.

Planning note: scenarios supply a whole `SpacecraftConfig` and `SpacecraftInitialState`
when the target is created (ADR-0005, #74). The DSL also needs a way to override single
settings or starting values (for example "battery capacity -20%") on top of a named
configuration. The settings are frozen dataclasses, so `dataclasses.replace()` already
does this in Python; Phase 5 decides how scenarios express it in YAML and how invalid
overrides are reported.

### Phase 6 — Deterministic Chaos Campaigns

Add seeded variability across SNR, latency, packet loss, pass geometry, sensor noise,
battery condition, and command timing. Record seeds and run identifiers so every
failure is reproducible.

**Milestone:** run hundreds or thousands of simulations and reproduce any failed run
exactly.

Builds on: named RNG streams, campaign run seeds, and `run.json` (ADR-0003). The
`pocketsat replay` command lands here.

Planning note: once campaigns vary settings and starting conditions, `run.json` must
record the `SpacecraftConfig` and `SpacecraftInitialState` each run used, so a failed
run can be replayed from its record alone (ADR-0005, #74). That needs a new run-record
schema version (`docs/run-record.md`). Until then, a run's configuration is fixed by its
code version, which `run.json` already records.

Review point: at the start of Phase 6, reassess the realism of the power and thermal
budget (`docs/power-thermal-budget.md`, #72): its parameter ranges, the "what's
optimistic" list, and the traffic stand-ins, against the real downlink traffic of #56
and #59. Campaigns that vary battery condition start from `STRESSED_CONFIG`.

### Phase 7 — Hardware-in-the-Loop (split into 7a and 7b)

Move the flight-computer role to a Raspberry Pi Pico, ESP32, or similar
microcontroller. Keep Python as the orchestrator.

**Milestone:** the same scenario can execute against SIL or an MCU-backed HIL target.

The original estimate was 2 weekends. ADR-0002 added a sensor-input channel, a
test-control channel, and a physical reset line to make HIL repeatable, so that
estimate no longer holds and Phase 7 is split:

- **7a — Minimal MCU Firmware HIL:** C/C++ firmware with framing and CRC (checked
  against `tests/vectors/`), commands, telemetry, state machine, watchdog, safe mode,
  and reset recovery.
- **7b — HIL Test Channels:** sensor-input channel, test-control channel, reset line,
  and capability declaration.

If the 10-week plan slips, 7b is the first item cut; HIL then runs nominal scenarios
only. Both phases still need a revised estimate.

Open point: the firmware's telemetry encoder must decide how a non-finite sensor value
is sent. The SIL encoder raises on NaN because the simulation validates its inputs; the
options are a per-field sentinel or a "sensor invalid" flag bit from a reserved group
(`docs/protocol.md`, #54).

### Phase 12 — CI/CD Test Infrastructure (throughout)

Run unit tests, SIL tests, nominal mission scenarios, and selected fault scenarios on
pull requests. Use a self-hosted Linux runner for scheduled HIL campaigns that reset
hardware, flash firmware, execute scenarios, capture telemetry, and publish artifacts.

**Milestone:** software and hardware test campaigns operate as repeatable engineering
infrastructure.

Status: lint, type checks, and tests run on every pull request and on `main` since
Phase 0.

## 10-week build plan (Version 1)

| Week | Focus |
|---|---|
| 1 | Architecture + repository + spacecraft state model |
| 2 | Flight computer + telemetry + command protocol |
| 3 | Ground-station simulator |
| 4 | Orbital pass + RF link simulator |
| 5 | Scenario DSL + test orchestrator |
| 6 | Fault-injection framework |
| 7 | Campaign runner + deterministic failures |
| 8 | Microcontroller HIL integration (7a; 7b if time allows) |
| 9 | GitHub Actions + Docker + reporting |
| 10 | Polish, documentation, architecture diagrams, and demo mission |

## Scenario library

Target scenarios for Version 1, authored as YAML once the DSL exists (Phase 5):

| Scenario | Primary behavior under test |
|---|---|
| Nominal pass | Baseline system behavior |
| Low-elevation pass | Weak RF link |
| Ground-station outage | Loss and recovery of communications |
| Severe packet loss | Downlink resilience |
| Doppler tracking error | Frequency mismatch |
| Command corruption | Protocol handling |
| Delayed ACK | Timing and retry behavior |
| Battery depletion | Power management |
| Sensor freeze | Bad or stale telemetry |
| Thermal excursion | Safe-mode behavior |
| MCU reset | Flight-computer recovery |
| Ground-station restart | Session recovery |
| Radio reset | Communications recovery |
| Partial downlink | Resume/retransmission |
| Simultaneous faults | System-level resilience |

## Scope boundary

Treat the 10-week plan as Version 1. Defer SDR integration, extensive physical sensors,
sophisticated orbital mechanics, real-satellite validation, Kubernetes, and anomaly
detection until Version 2. A finished, documented SIL/HIL platform is stronger than a
sprawling unfinished laboratory.

## Version 2 (deferred)

These phases are out of scope for Version 1 and have no milestones yet.

| Phase | Summary | Milestone |
|---|---|---|
| 8 — Physical Electronics | Inexpensive sensors and indicators (temperature, current/voltage, IMU, LEDs, photoresistor); later a programmable power supply for brownouts, voltage sag, and power cycling | Test campaigns incorporate real electrical and sensor behavior |
| 9 — SDR Signal Path | SDR reception or a shielded/cabled RF loopback: packet generation, modulation, impairment, reception, demodulation, decoding, and telemetry validation, observing applicable radio regulations | Selected tests exercise a real RF signal path rather than packet-layer simulation only |
| 10 — Real Amateur-Satellite Comparison | Predict real amateur-satellite passes from public orbital elements and compare observed reception with the simulator's AOS-to-LOS link model | Document where simulated behavior matches or diverges from real LEO reception |
| 11 — Statistics & Campaign Analysis | Analyze SNR vs. packet loss, elevation vs. successful downlink, latency vs. command failure, and battery state vs. safe-mode events | A campaign report with failure modes, reliability measures, and diagnostic plots |

## What the project demonstrates

- **Testing infrastructure, not just tests:** reusable tooling, orchestration,
  scenario definition, reporting, and target abstractions.
- **Mission-level failure thinking:** tests ask what happens to the whole mission when
  a component or the communications link fails.
- **Radio knowledge as systems engineering:** AOS/LOS, Doppler, SNR, interference,
  outages, and packet loss become controllable test inputs.
- **Progressive realism:** from pure simulation to MCU-backed HIL and, optionally, a
  real RF signal path.
- **Reproducibility:** seeded campaigns make stochastic failures replayable.
