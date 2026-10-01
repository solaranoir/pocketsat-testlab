# ADR-0004: Flight computer to subsystem controls

- **Status:** Proposed
- **Phase:** 1
- **Amends / Supersedes:** Amends ADR-0003 (refines step 6, `advance(dt_us)`, into steps a to f). Related: ADR-0002 (`inject()` and target faults). Issue: #71.

## Context

Subsystems are stepped by `step(dt_us, env)`, which carries no information about what the flight computer decided. Several Phase 1 tasks need that path: power needs the active loads (#36), payload and communications need on/off and radio state (#43, #44), and the mode state machine needs its entry and exit actions defined in one place (#47).

Without a defined path, the subsystem epic (#30) and the flight computer epic (#45) would each depend on the other's implementation. This ADR adds the missing flight computer to subsystem path as a shared data type, so both epics depend on the type and not on each other.

It also settles the questions next to that path: what happens inside one tick, how faults reach subsystems, which values physics and decisions read, how downlink data moves, how the radio behaves, and what `sensor_freeze` and `forced_reset` mean precisely.

## Decision

### 1. The flight computer produces a frozen control record each tick; subsystems obey it

`SpacecraftControls` is a frozen dataclass in `pocketsat.spacecraft`, built from per-subsystem frozen records plus fault overrides:

- `PayloadControls(enabled, release_through_chunk_id)`
- `RadioControls(mode)`, where `mode` is `OFF`, `RX_ONLY`, or `RX_TX` (section 10)
- `AttitudeControls(enabled)`
- Fault overrides: `frozen_sensors: frozenset[str]` (used by `sensor_freeze`) and `extra_load_w` (used by `battery_drain`)

Each subsystem reads only its own record and the overrides that concern it. Defaults describe nominal operation: payload off, radio `RX_TX`, attitude control on, no overrides. Subsystem and story-level tests run with default controls and need no flight computer.

The `Subsystem` protocol changes to `step(dt_us, env, controls)`. `SubsystemStack.step` passes the same controls to every subsystem.

### 2. Timing inside a tick

ADR-0003 step 6 (`advance(dt_us)`) expands to the following. Tick N is the tick being advanced.

- a. Merge the flight computer's tick N-1 controls with the active fault overrides.
- b. Subsystems step in `STEP_ORDER` with the merged controls.
- c. The flight computer decodes queued uplink, executes commands, and sends ACK/NACK.
- d. The flight computer evaluates flags and state and updates the mode.
- e. The flight computer emits telemetry if due.
- f. The flight computer produces controls for tick N+1.

**One-tick latency:** controls produced in tick N take effect in tick N+1. The flight computer is not part of `STEP_ORDER`; it always runs after all subsystems, so telemetry reflects the current tick. The `STEP_ORDER` docstring states this and points to this ADR.

**Faults act on the same tick; commands act one tick later.** A fault injected in tick N is merged at step a and takes effect in tick N, so a fault scheduled at time t hits at t. A command's effect lands in tick N+1. This asymmetry is intentional and must not be "fixed" in either direction without a new ADR.

**First tick after a reset.** `reset()` obtains the flight computer's BOOT-mode controls, which apply to tick 0. The nominal defaults in section 1 are a test convenience, not the power-on state.

**Telemetry can lag by one frame.** Telemetry at tick N shows the mode after steps c and d, alongside subsystem state produced under the tick N-1 controls. For one frame after a mode change the two can disagree (for example mode SCIENCE with the payload still idle). This is correct behavior.

Latency of the main chains, at the 100 ms default tick:

| Chain | Latency |
|---|---|
| Command → ACK | same tick |
| Command → physical effect | +1 tick |
| Effect → power sees the draw | +1 tick (downstream read, section 3) |
| Fault → effect | 0 ticks |

### 3. Cross-subsystem reads go only through `SpacecraftState`

Subsystems never hold references to each other. Cross-subsystem reads go only through `SpacecraftState`, using the rule defined in #36: a subsystem earlier in `STEP_ORDER` is read as of the current tick, and a later one as of the previous tick. For example, thermal reads power's dissipation from the current tick, and power reads the heater, payload, attitude-control, and transmit draws one tick late. The mechanism itself is implemented in #36.

### 4. Power does not read the mode

Power load is a base load plus the draw of whatever is actually switched on: payload, attitude control, radio transmit, the survival heater (section 12), and `extra_load_w`. Mode-dependent behavior reaches power only through the controls the flight computer produces.

### 5. Faults are control overrides

`SilTarget` merges fault overrides into the controls in one place, with a documented precedence: **fault > flight computer**. The merge function also tracks each fault's duration and releases it on expiry; subsystems never track fault timing. The orchestrator calls `inject()` once when a fault activates, and the target owns expiry using the fault's `duration_us`.

| Fault | Effect in the merge |
|---|---|
| `sensor_freeze` | Adds the affected subsystems to `frozen_sensors` (section 8) |
| `battery_drain` | Sets `extra_load_w` (section 11) |
| `transmitter_off` | Disables the transmitter only; the receiver keeps working (section 10) |
| `forced_reset` | Reboots the flight computer through the RESET path (section 9) |

### 6. Physics reads true values; decisions and telemetry read reported values

Subsystem snapshots carry both true values and reported values (noisy, possibly frozen).

- Physical coupling between subsystems (thermal heating from power dissipation, power draw from active loads, later link quality from true pointing) uses **true** values.
- Limit flags, inhibits, flight computer logic, and telemetry use **reported** values, so the spacecraft only knows what real sensors would tell it.
- True values remain available for test assertions.
- Hysteresis (low-battery flags #36, thermal limit flags #39) and sustained-duration checks (automatic safe-mode entry #48) absorb noise near thresholds.

### 7. Snapshot convention

Each subsystem snapshot holds two nested frozen records: a truth record (for example `PowerTruth`) and a readings record (for example `PowerReadings`).

- Flags live on the readings side, because they are decisions.
- `sensor_freeze` holds the readings record.
- The telemetry encoder (#54) accepts only readings types, so mypy stops true values from reaching the wire.
- Subsystems without sensors (payload, communications) still expose a readings record, equal to their true values and never noisy or frozen, so every subsystem has the same shape.

### 8. `sensor_freeze`

Each frozen reading holds exactly the last value it reported before the freeze (no noise). True values keep evolving. Live readings resume on release. The fault takes an optional `subsystem` parameter (`power`, `thermal`, or `attitude`), defaulting to all three. Payload buffer fill and radio state are not sensor readings and are not frozen.

### 9. `forced_reset` is a flight computer reboot, not a physics reset

`forced_reset` reboots the flight computer through the same path as the RESET command (#49): mode to BOOT, transient flight computer state cleared, uptime reset, boot counter incremented. Subsystem physical state (battery SOC, temperatures, attitude) continues unchanged.

`duration_us` is the time held in reset. During the hold the flight computer performs no command handling, telemetry, or new controls, and subsystems keep stepping under the controls that were in force when the reset began. When the reboot completes, the controls revert to BOOT's: the flight computer produces BOOT's controls at step f of the tick in which the hold ends, so they apply from the next tick, like any controls it produces. `None` or `0` means a momentary pulse: the flight computer reboots in the tick the fault is injected and produces BOOT's controls at step f of that tick.

Active faults persist through the RESET command and `forced_reset`, as a physical failure would survive a flight computer reboot; only the target's `reset(seed)` clears them.

### 10. The radio is full duplex

The radio is a crossband transceiver (separate uplink receiver and downlink transmitter, as is common on amateur satellites), so the spacecraft can receive while transmitting.

| Mode | Receiver | Transmitter | Use |
|---|---|---|---|
| `OFF` | off | off | Power emergency |
| `RX_ONLY` | on | off | Listening without transmitting |
| `RX_TX` | on | on | Every normal flight mode, so beacons and ACKs can go out |

- Uplink is received only while the receiver is on, judged from comms' true state for that tick. Frames delivered while it is off are lost, not queued, and counted.
- Outbound frames share the transmit capacity in priority order: ACK/NACK, then telemetry, then DATA. Frames that don't fit are not queued; suppressed ACK/NACK and telemetry are counted.
- Transmit power is proportional to the bytes actually sent.
- `transmitter_off` disables the transmitter only. The receiver keeps working, so the spacecraft can hear and execute commands but cannot reply.

Revisit trigger: if a scenario needs a half-duplex transceiver, that requires new rules for listen windows and ACK latency.

### 11. `battery_soc_override` beats `battery_drain`

The override (an environment and test-setup lever) pins SOC while set. The drain (a fault) still appears in the reported current and power draw. When the override clears, integration resumes from the pinned value with the drain still active.

### 12. The survival heater is hardwired, not a control

The battery survival heater (#38) is driven by a mechanical thermostat on the battery's true temperature, so it is physics, not a flight computer decision. It has no `SpacecraftControls` field, and no mode (SAFE included) can switch it off.

Revisit trigger: if the flight computer ever needs to shed heater load, add a `ThermalControls` record through the per-subsystem controls structure in section 1.

### 13. Downlink data ownership

The payload owns stored data, the flight computer moves it (#56), and comms provides transmit capacity only and holds no data. The flight computer releases sent chunks through `PayloadControls.release_through_chunk_id`; the payload never deletes data on its own. Phase 1 releases a chunk once it is sent. Later phases can switch to releasing once the ground acknowledges, by changing only the flight computer's rule.

### 14. Seams that keep a later move to a message bus cheap

A later move to a message bus would cost roughly 2 to 4 days instead of 1 to 2 weeks if these seams hold:

- Controls are produced by a single flight computer function and merged with fault overrides in a single `SilTarget` function (implemented in #47, #59, and #60).
- Subsystems never hold references to each other (section 3).
- Controls are part of the per-tick record alongside `SpacecraftState`, so a run can be inspected and replayed.

Revisit triggers for a message bus:

- several producers of the same control that need arbitration
- components running at different rates
- more than about 10 components exchanging data
- subsystems split across processes or MCUs

## Alternatives considered

- **Pass the mode to `step()`.** Spreads mode logic across every subsystem, and the mode table can't be read in one place. Rejected.
- **Setter calls from the flight computer.** Timing is implicit, and the calls are hard to record and replay. Rejected.
- **A message bus.** More machinery than v1 needs. Deferred; see the revisit triggers in section 14.

## Consequences

- Both Phase 1 epics depend on `SpacecraftControls`, not on each other's implementation.
- Mode logic lives in one place, the flight computer's controls function. Subsystems are simple to test with default controls.
- Replay and debugging improve: controls are recorded with the state each tick.
- The one-tick latency on commands and the zero-tick latency on faults are part of the contract and are asserted in tests.
- `Subsystem.step` gains a `controls` parameter, so every subsystem implementation and test fake passes it.
- Several tasks are shaped by this ADR: power loads are no longer per mode (#36), the thermal model gains a hardwired survival heater (#38), the telemetry encoder is typed to readings records (#54), the radio is full duplex (#44), the payload owns downlink data (#43, #56), and the fault merge and expiry live in `SilTarget` (#59, #60).
- ADR-0003's tick order is refined, which only an ADR may do; this is that ADR.

## Follow-ups

- `docs/architecture.md` §4.1 (controls, steps a to f, latency table, where the flight computer sits relative to `STEP_ORDER`) and the §8 lifecycle diagram (step 6 expands to a to f) are updated in the same change as this ADR.
- The affected issues (#35, #36, #38, #39, #41, #43, #44, #47, #49, #54, #59, #60, #76) already carry these decisions in their acceptance criteria.
- Implementation of the controls themselves: payload (#43), comms (#44), attitude (#41), the flight computer's controls function (#47), and the `SilTarget` merge (#59, #60).
