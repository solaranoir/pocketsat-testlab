# PocketSat Test Lab — Spacecraft model

Status: Phase 1. Describes the subsystem snapshot contracts (#76). The full spacecraft model description is extended by #65.

The SIL spacecraft is five subsystems (power, thermal, attitude, payload, comms) stepped in `STEP_ORDER`, plus a flight computer that runs after them (ADR-0004, [architecture §4.1](architecture.md#41-spacecraft-subsystems-sil)). This page defines what each subsystem exposes to the others and to the flight computer.

## Snapshot contracts

The contract lives in `pocketsat.spacecraft.snapshots`. Each subsystem's snapshot holds two frozen records (ADR-0004 §7):

- the **truth** record: true physical values. Physics and physical coupling between subsystems read it.
- the **readings** record: what the sensors report, possibly noisy or frozen by `sensor_freeze`, plus the limit flags. Decisions (inhibits, the flight computer) and telemetry (#54) read it.

Payload and comms have no sensors: their readings always equal their truth, and their snapshots are built with `PayloadSnapshot.from_truth(...)` and `CommsSnapshot.from_truth(...)`.

Field names follow the [naming and units convention](architecture.md#naming-and-units-convention). Each field's meaning and unit is documented in its docstring in `pocketsat.spacecraft.snapshots`. Subsystem tickets implement these records exactly, and add fields only by extending the contract (this page and the module) in the same change.

| Subsystem | Truth record | Readings record |
|---|---|---|
| Power (#35, #36) | `PowerTruth`: `bus_v`, `battery_current_a` (positive when charging), `soc`, `generation_w`, `total_load_w`; derived `net_power_w` | `PowerReadings`: `bus_v`, `battery_current_a`, `soc` (estimate from the reported voltage), flags `low_battery`, `critical_battery` |
| Thermal (#38, #39) | `ThermalTruth`: `battery_c`, `electronics_c`, `heater_on`, `heater_power_w` (hardwired survival heater) | `ThermalReadings`: `battery_c`, `electronics_c`, flags `over_temp`, `under_temp` |
| Attitude (#41) | `AttitudeTruth`: `pointing_error_deg`, `rate_dps`, `state`, `control_power_w` | `AttitudeReadings`: `pointing_error_deg`, `rate_dps`, `state` |
| Payload (#43) | `PayloadTruth`: `state`, `buffered_bytes`, `buffer_capacity_bytes`, `oldest_unreleased_chunk_id`, `next_chunk_id`, `total_produced_bytes`, `total_released_bytes`, `power_w`; derived `buffer_fill` | `PayloadReadings`: equal to truth |
| Comms (#44) | `CommsTruth`: `radio_mode`, `receiver_on`, `transmitter_on`, `transmit_capacity_bytes`, `previous_tick_sent_bytes`, `uplink_lost_count`, `outbound_suppressed_count`, `transmit_power_w` | `CommsReadings`: equal to truth |

States are enums defined in the contract: `AttitudeState` (`TUMBLING`, `DETUMBLING`, `STABILIZED`) and `PayloadState` (`OFF`, `IDLE`, `ACQUIRING`). The radio mode reuses `RadioMode` (`OFF`, `RX_ONLY`, `RX_TX`) from the controls.

The payload owns stored data and comms holds none (ADR-0004 §13): comms exposes only how much it may send this tick, and the flight computer (#56) moves chunks from the payload into DATA frames within that capacity.

**Payload chunk store (#43).** The payload (`pocketsat.spacecraft.payload.Payload`) stores data as fixed-size chunks (`PayloadConfig.chunk_size_bytes`, at most `MAX_CHUNK_SIZE_BYTES` = 65 531, so a chunk plus its 4-byte ID fits in one DATA frame payload). Chunks are created in ID order and released as a prefix through `controls.payload.release_through_chunk_id`, so the stored chunks are exactly the IDs `oldest_unreleased_chunk_id` to `next_chunk_id - 1`, plus a partial chunk still accumulating. A chunk's content is a pure function of its ID, `chunk_content(chunk_id, chunk_size_bytes)` (documented and pinned in that module), which #56's DATA frames and the receiver share. At every tick `total_produced_bytes == buffered_bytes + total_released_bytes`. The payload acquires only while `controls.payload.enabled` is set and none of its inhibits (the payload rows of the table below) applies.

## Cross-subsystem reads

Subsystems never hold references to each other; they read each other's snapshots through `SpacecraftState` (ADR-0004 §3). The timing rule, from #36: a subsystem earlier in `STEP_ORDER` is read as of the current tick, and a later one as of the previous tick.

**Mechanism (#85).** A `SnapshotBoard` holds each subsystem's latest snapshot. `SubsystemStack` publishes a subsystem's snapshot immediately after it is reset and immediately after it steps, so a reader automatically sees the current tick for subsystems that already stepped and the previous tick for those that haven't. A subsystem that reads others receives the board, typed as the read-only `SnapshotReader`, when it is constructed (alongside its settings and starting records), and calls `reader.get("power", PowerSnapshot)`. Create the board first, pass it to the subsystems, then to the stack:

```python
board = SnapshotBoard()
thermal = Thermal(config.thermal, initial.thermal, reader=board)
stack = SubsystemStack([power, thermal, ...], board=board)
```

Physics reads truth records; decisions read readings records. A test (`tests/unit/test_snapshot_contracts.py`) checks every row of these tables against the contract and the timing rule.

| Reader | Reads | Use | When |
|---|---|---|---|
| power | `ThermalTruth.heater_power_w` | physics: survival heater draw (#36) | previous tick |
| power | `AttitudeTruth.control_power_w` | physics: attitude-control draw (#36) | previous tick |
| power | `AttitudeTruth.pointing_error_deg` | physics: solar generation pointing factor (#35) | previous tick |
| power | `PayloadTruth.power_w` | physics: payload draw (#36) | previous tick |
| power | `CommsTruth.transmit_power_w` | physics: transmit draw (#36) | previous tick |
| thermal | `PowerTruth.total_load_w` | physics: power dissipated as heat (#38) | current tick |
| payload | `PowerReadings.low_battery` | decision: acquisition inhibit (#43) | current tick |
| payload | `PowerReadings.critical_battery` | decision: acquisition inhibit (#43) | current tick |
| payload | `ThermalReadings.over_temp` | decision: acquisition inhibit (#43) | current tick |
| payload | `ThermalReadings.under_temp` | decision: acquisition inhibit (#43) | current tick |
| payload | `AttitudeReadings.state` | decision: acquire only while `STABILIZED` (#43) | current tick |

Attitude and comms read nothing from other subsystems (#41, #44).

### Flight computer reads

The flight computer is not in `STEP_ORDER`: it runs after every subsystem, so it always reads the current tick. It reads readings records only, because everything it does is a decision.

| Reader | Reads | Use | When |
|---|---|---|---|
| flight computer | `PowerReadings.low_battery` | decision: mode logic, DATA inhibit, telemetry flag (#48, #56, #54) | current tick |
| flight computer | `PowerReadings.critical_battery` | decision: mode logic, DATA inhibit, telemetry flag (#48, #56, #54) | current tick |
| flight computer | `ThermalReadings.over_temp` | decision: mode logic, DATA inhibit, telemetry flag (#48, #56, #54) | current tick |
| flight computer | `ThermalReadings.under_temp` | decision: mode logic, DATA inhibit, telemetry flag (#48, #56, #54) | current tick |
| flight computer | `PayloadReadings.oldest_unreleased_chunk_id` | decision: next chunk to downlink (#56) | current tick |
| flight computer | `PayloadReadings.next_chunk_id` | decision: whether a chunk is stored (#56) | current tick |
| flight computer | `CommsReadings.transmit_capacity_bytes` | decision: downlink rate limit (#56) | current tick |

Telemetry (#54) encodes readings records only.

## Power (#35, #36)

`pocketsat.spacecraft.Power` is built as `Power(config.power, initial.power, reader=board)`. Each tick it:

1. computes solar generation: `PowerConfig.solar_array_w` × `portable_cos_deg` of attitude's true pointing error (previous tick); zero in eclipse and at or beyond 90°;
2. sums the total load, in this order: `base_load_w` + payload draw (`PayloadTruth.power_w`) + attitude-control draw (`AttitudeTruth.control_power_w`) + transmit draw (`CommsTruth.transmit_power_w`) + survival heater draw (`ThermalTruth.heater_power_w`) + `controls.extra_load_w` (the `battery_drain` override). Every draw comes from a truth record of a subsystem that steps after power, so power sees it one tick late (see the cross-read table). Power never reads the mode or the subsystem commands (ADR-0004 §4);
3. integrates net power (generation minus load) over `dt_us` into the SOC, clamped to 0..1, using true values only;
4. derives the bus voltage linearly from the SOC between `battery_empty_v` and `battery_full_v`, and the battery current as net power divided by bus voltage, positive while charging;
5. updates the readings (below).

- **The reader is optional.** Without one (`reader=None`), pointing is ideal (factor 1.0) and there are no per-subsystem draws; only `base_load_w` and `extra_load_w` count. With one, attitude, thermal, payload, and comms must all be published on it, or `step()` raises the board's `KeyError`; tests use `fake_subsystems()` for the four others.
- `reset(rng)` must be called before the first `step()` (it requests the noise stream); otherwise `step()` raises `RuntimeError`.
- `EnvironmentState.battery_soc_override` pins the SOC (and so the bus voltage) while set, even with `extra_load_w` active; the drain still shows in `total_load_w` and the battery current (ADR-0004 §11). When the override clears, integration resumes from the pinned value.

**Readings.** `PowerReadings` holds the reported values and flags (ADR-0004 §6 to §8):

- **Noise.** Reported bus voltage and battery current are the true values plus noise from `portable_normal` on the stream `spacecraft.power.noise`, with standard deviations `voltage_noise_v` (default 0.008 V) and `current_noise_a` (default 0.01 A), each times `EnvironmentState.sensor_noise_scale` (0.0 gives the true values exactly). Two draws (voltage, then current) are taken every tick, frozen or not, so a freeze does not shift the noise that follows it. `portable_normal` is bounded at ±6σ.
- **SOC estimate.** `soc` inverts the voltage curve on the *reported* voltage, `(bus_v - battery_empty_v) / (battery_full_v - battery_empty_v)`, clamped to 0..1, so noise and freezes carry through. At nominal noise it is within ±`6 × voltage_noise_v / 2.4 V` = **±0.02** of the true SOC (a hard bound, because the noise is bounded).
- **Flags with hysteresis**, computed from the SOC estimate:

  | Flag | Sets when the estimate falls below | Clears when the estimate rises above |
  |---|---|---|
  | `low_battery` | `low_battery_soc` = 0.30 | `low_battery_clear_soc` = 0.35 |
  | `critical_battery` | `critical_battery_soc` = 0.15 | `critical_battery_clear_soc` = 0.20 |

  Between the two thresholds a flag keeps its previous value. The hysteresis band (0.05) is wider than the estimate's full noise spread at nominal noise (0.04), so a steady true SOC near a threshold cannot make a flag flap; at `sensor_noise_scale` above 1.25 it can, while the true SOC stays within the band. `PowerConfig` validates that each clear threshold is above its set threshold and that critical's thresholds are at or below low's, so `critical_battery` is only ever set together with `low_battery`. After a reset each flag is set if the starting SOC is below its set threshold, and the readings equal the truth.
- **`sensor_freeze`.** While `"power"` is in `controls.frozen_sensors`, the whole readings record (voltage, current, SOC estimate, and flags) holds exactly its last pre-freeze value with no noise, while the truth keeps evolving; a real threshold crossing during a freeze is hidden from the flags. On release, live readings resume and the flags continue from their pre-freeze state.

The `PowerConfig` defaults (20 Wh battery, 8 W array, 1.8 W base load, 6.0 to 8.4 V, the noise levels, and the flag thresholds) are calibrated by the power and thermal budget ([power-thermal-budget.md](power-thermal-budget.md), #72), and the default starting charge (`PowerInitial.soc`) is 0.5. With the defaults and the shared fakes' draws (attitude control 0.5 W, transmit 1.0 W), a nominal 92-minute orbit with 35% eclipse is net positive (about +2.9 Wh per orbit), which the story-level test (`tests/sil/test_power_story.py`) checks over three orbits. `step()` plus `snapshot()` costs about 10 µs on a developer laptop.

## Thermal (#38, #39)

`pocketsat.spacecraft.Thermal` is built as `Thermal(config.thermal, initial.thermal, reader=board)`. It is a lumped model with two temperature nodes, the **battery** (`B`) and the **electronics** (`E`), integrated step by step with forward Euler and portable arithmetic only (ADR-0006, no `exp`). Each tick, with `dt` in seconds (`dt_us / 1_000_000`) and `T_amb` = `EnvironmentState.ambient_temp_c`:

1. **Electrical dissipation** `P`. All electrical load becomes heat. Thermal reads power's *true* `PowerTruth.total_load_w` through the reader; power steps first, so this is the current tick (see the cross-read table). Power's total load already includes the survival heater draw thermal published in the previous tick, which is exactly the heater power applied this tick, so thermal subtracts it: `P = max(0, total_load_w - H)`. The heater's heat is counted once, in the battery.
2. **Heater** `H` = `heater_power_w` while the heater is on, else 0. The on/off state is the one decided at the end of the previous tick (or at reset), which is also what power drew this tick.
3. **Integration**, with `f` = `battery_dissipation_fraction`, `G_B`, `G_E` the conductances to the ambient, `G_BE` the coupling conductance, and `C_B`, `C_E` the heat capacities:

   ```
   Q_BE = G_BE * (T_B - T_E)
   T_B += (f * P + H - G_B * (T_B - T_amb) - Q_BE) * dt / C_B
   T_E += ((1 - f) * P - G_E * (T_E - T_amb) + Q_BE) * dt / C_E
   ```

   Both nodes cool (or warm) toward the ambient and exchange heat through the coupling. The **environment** acts only through `ambient_temp_c`, the effective sink temperature, which already includes solar heating (`NominalEnvironment`: 20 °C sunlit, -20 °C eclipse, #73); there is no separate solar term. The steady state is linear in `T_amb`, so a change in ambient moves both steady-state temperatures by the same amount.
4. **Survival heater thermostat** (ADR-0004 §12): a mechanical thermostat on the battery node reads the *true* battery temperature after integration. An off heater switches on below `heater_on_setpoint_c`; an on heater switches off above `heater_off_setpoint_c`; between the two it keeps its state (hysteresis, so no chatter). It has no control field and no mode can switch it off. `ThermalTruth.heater_on` and `heater_power_w` report it; power (#36) adds `heater_power_w` to the total load one tick later, the tick in which thermal heats the battery with it.

- **The reader is optional.** Without one (`reader=None`) there is no electrical heating, only the environment and the heater. With one, power must be published on it, or `step()` raises the board's `KeyError`.
- **Step size.** Forward Euler is monotone (no overshoot or oscillation) while `dt <= C / (sum of the node's conductances)` for both nodes. `Thermal.max_step_us` is that limit (500 s with the defaults); `step()` raises `ValueError` beyond it.
- **Reset.** `reset(rng)` returns to `ThermalInitial` (20 °C / 20 °C, confirmed by #72) and requests the noise stream; it must be called before the first `step()`, otherwise `step()` raises `RuntimeError`. The heater starts on if the starting battery temperature is below the ON setpoint.

**Readings (#39).** `ThermalReadings` holds the reported temperatures and the limit flags (ADR-0004 §6 to §8). The thermostat and the physics never read them.

- **Noise.** The reported battery and electronics temperatures are the true values plus noise from `portable_normal` on the stream `spacecraft.thermal.noise`, with standard deviation `temperature_noise_c` (default 0.2 °C) times `EnvironmentState.sensor_noise_scale` (0.0 gives the true values exactly). Two draws (battery, then electronics) are taken every tick, frozen or not, so a freeze does not shift the noise that follows it. `portable_normal` is bounded at ±6σ, so at nominal noise a reading is within ±1.2 °C of the truth.
- **Flags, per node, with hysteresis.** The battery is heated and the electronics are not (in a nominal eclipse the electronics fall to about -2 °C while the heater holds the battery near 0 °C to 4 °C), so one shared threshold cannot serve both: each node has its own under- and over-temperature thresholds, evaluated on that node's *reported* temperature. A node's condition sets when its reading is beyond the set threshold, clears when it is beyond the clear threshold, and between the two keeps its previous value:

  | Node | Under-temp sets below | clears above | Over-temp sets above | clears below |
  |---|---|---|---|---|
  | Battery | `battery_under_temp_c` = -5 °C | `battery_under_temp_clear_c` = -2 °C | `battery_over_temp_c` = 45 °C | `battery_over_temp_clear_c` = 42 °C |
  | Electronics | `electronics_under_temp_c` = -25 °C | `electronics_under_temp_clear_c` = -22 °C | `electronics_over_temp_c` = 60 °C | `electronics_over_temp_clear_c` = 57 °C |

  **The flags are OR-ed across the nodes:** `under_temp` is set while the battery's *or* the electronics' under-temperature condition is set, and `over_temp` likewise; a flag clears only when both nodes have cleared. Every hysteresis band (3 °C) is wider than a steady reading's full noise spread at nominal noise (2.4 °C), so a steady temperature near a threshold cannot make a flag flap; at `sensor_noise_scale` above 1.25 it can. After a reset each condition is set if its node's starting temperature is beyond its set threshold, and the readings equal the truth.
- **Threshold order** (validated by `ThermalConfig`, `ValueError` otherwise). Battery: `battery_survival_limit_c` (default -10 °C) < `battery_under_temp_c` < `heater_on_setpoint_c` < `heater_off_setpoint_c`, with `battery_under_temp_c` at least 5 °C below `heater_on_setpoint_c` (the #72 margin rule), so the heater acts well before the flag. Each node: `under_temp_c` < `under_temp_clear_c` < `over_temp_clear_c` < `over_temp_c`. And `battery_over_temp_clear_c` is above `heater_off_setpoint_c`, so the heater cannot drive the battery into over-temperature. The survival limit is not used by the model; it anchors the order and documents the battery's survival range.
- **`sensor_freeze`.** While `"thermal"` is in `controls.frozen_sensors`, the whole readings record (both temperatures and both flags) holds exactly its last pre-freeze value with no noise, while the truth (and the heater) keeps evolving; a real threshold crossing during a freeze is hidden from the flags. On release, live readings resume and each node's conditions continue from their pre-freeze state.

**Settings** (`ThermalConfig`; calibrated by the power and thermal budget, [power-thermal-budget.md](power-thermal-budget.md), #72):

| Setting | Default | Meaning |
|---|---|---|
| `battery_heat_capacity_j_per_c` | 80.0 | battery heat capacity, J/°C (about a 100 g lithium-ion pack) |
| `electronics_heat_capacity_j_per_c` | 300.0 | electronics heat capacity, J/°C |
| `battery_conductance_w_per_c` | 0.12 | battery to ambient, W/°C (positive) |
| `electronics_conductance_w_per_c` | 0.25 | electronics to ambient, W/°C (positive) |
| `coupling_conductance_w_per_c` | 0.04 | battery to electronics, W/°C (0 decouples) |
| `battery_dissipation_fraction` | 0.25 | share of the electrical dissipation (heater excluded) heating the battery; the rest heats the electronics |
| `heater_power_w` | 3.0 | survival heater power while on, W |
| `heater_on_setpoint_c` | 1.0 | heater switches on below this battery temperature, °C |
| `heater_off_setpoint_c` | 5.0 | heater switches off above this battery temperature, °C; above the ON setpoint |
| `temperature_noise_c` | 0.2 | standard deviation of each reported temperature's noise at `sensor_noise_scale` 1.0, °C |
| `battery_survival_limit_c` | -10.0 | lowest battery temperature it survives, °C; bottom of the threshold order |
| `battery_under_temp_c` / `battery_under_temp_clear_c` | -5.0 / -2.0 | battery under-temperature set / clear, °C |
| `battery_over_temp_c` / `battery_over_temp_clear_c` | 45.0 / 42.0 | battery over-temperature set / clear, °C |
| `electronics_under_temp_c` / `electronics_under_temp_clear_c` | -25.0 / -22.0 | electronics under-temperature set / clear, °C |
| `electronics_over_temp_c` / `electronics_over_temp_clear_c` | 60.0 / 57.0 | electronics over-temperature set / clear, °C |

With these defaults, the real power subsystem, and the shared fakes' draws (3.3 W of load), the battery peaks near 28 °C and the electronics near 30 °C in a nominal orbit's sunlight. In the -20 °C eclipse the battery would settle well below the ON setpoint without the heater and well above the OFF setpoint with it on, so the heater cycles between its setpoints several times per nominal eclipse, and the battery never falls more than a few hundredths of a degree below the ON setpoint. With all five real subsystems in the budget's reference profile the heater cycles three times per eclipse at about 28% duty (about 0.5 Wh per orbit; see the budget). The values were chosen for that #73 behavior: heater power and battery isolation sized so the heater-on steady state is well above the OFF setpoint.

The flag thresholds (#39) were chosen to leave nominal orbits clear with margin: over a nominal orbit the true battery stays within about 1 °C to 29 °C and the electronics within about -4 °C to 31 °C, so even with the ±1.2 °C noise bound the readings stay at least 4.8 °C from the nearest battery threshold (-5 °C) and more than 20 °C from the electronics thresholds. The battery thresholds bracket a lithium-ion pack's usual range (no charging below about 0 °C, at most about 45 °C); the electronics thresholds are below the -20 °C eclipse sink, which the electronics, always dissipating, never reach in a nominal orbit, and well below typical component limits. The budget (#72) keeps every temperature at least 5 °C inside them. The story-level test (`tests/sil/test_thermal_story.py`, story #37) runs the real power and thermal subsystems over three nominal orbits and checks that the temperatures settle into the same pattern each orbit, that no flag is raised, that the heater cycles in every eclipse with its draw in power's load, and determinism. `step()` plus `snapshot()` costs about 10 µs on a developer laptop.

## Attitude (#41)

`pocketsat.spacecraft.Attitude` is built as `Attitude(config.attitude, initial.attitude)` and reads nothing from other subsystems. It models two damped scalars integrated step by step with portable arithmetic only (ADR-0006): the pointing error from the sun-optimal attitude (degrees, 0..180; at 0° the solar arrays point at the sun) and the angular rate magnitude (°/s). Each tick, with `dt` in seconds (`dt_us / 1_000_000`):

1. **Rate.** While attitude control is on, the rate shrinks by `rate_damping_per_s * dt`. The seeded disturbance (stream `spacecraft.attitude.disturbance`, `portable_normal`) adds `disturbance_mean_dps_per_s * dt + disturbance_sd_dps_per_s * sqrt(dt) * n`. The disturbance is true dynamics, so `sensor_noise_scale` does not scale it. Its mean is non-negative, so with `controls.attitude.enabled` off nothing corrects it and the rate drifts up toward `TUMBLING`.
2. **Pointing.** A rotation phase advances by `rate * dt`; the pointing error is the phase folded into 0..180°, so a tumbling spacecraft sweeps its error from 0° to 180° and back. While control is on, the error then shrinks by `pointing_gain_per_s * dt`; it settles near `rate / pointing_gain_per_s` (about 2° with the defaults).
3. **State**, from the true rate and pointing error with hysteresis (thresholds in `AttitudeConfig`):
   - `TUMBLING` whenever the rate is above `tumbling_enter_rate_dps` (2.0 °/s); left for `DETUMBLING` only with control on and the rate at or below `tumbling_exit_rate_dps` (1.5 °/s).
   - `STABILIZED` once the rate is below `stabilized_enter_rate_dps` (0.2 °/s) and the error is at or below `stabilized_enter_pointing_error_deg` (5°); left for `DETUMBLING` when the rate exceeds `stabilized_exit_rate_dps` (0.4 °/s) or the error exceeds `stabilized_exit_pointing_error_deg` (10°).
   - `DETUMBLING` otherwise: control is reducing the rate between the two.
4. **Control draw.** `AttitudeTruth.control_power_w` is `AttitudeConfig.control_power_w` (0.5 W) while control is on and 0 while it is off; power (#36) adds it to the total load.

Readings report the pointing error and rate with noise from the stream `spacecraft.attitude.noise` (standard deviations `pointing_noise_deg`, 0.5°, and `rate_noise_dps`, 0.01 °/s, times `EnvironmentState.sensor_noise_scale`; 0.0 gives the exact true values), clamped to their ranges. The reported state is decided from the reported values with the same rules. While `attitude` is in `controls.frozen_sensors`, the readings record holds exactly its last pre-freeze value while the truth keeps evolving; live readings resume on release. Before the first step the readings equal the truth and the control draw is zero.

**Starting state.** `AttitudeInitial` defaults to 1.0 °/s and 45° of pointing error: deployment tip-off already partly damped, starting `DETUMBLING`. With the default settings and control on it reaches `STABILIZED` within a few minutes of simulated time.

**Causes of tumbling:** the starting rate (`AttitudeInitial`), attitude control switched off (for example during BOOT, #49), and the seeded disturbance. An attitude-control failure fault is a Phase 4 candidate, not part of Phase 1.

**Simplifications (Phase 1):**

- A single scalar pointing error and a single scalar rate magnitude; no 3-D attitude.
- No actuator saturation and no momentum build-up.
- No environmental torques beyond the seeded disturbance.
- Antenna pointing does not affect the link in Phase 1; Phase 3's RF channel may use the true pointing error for antenna gain.

All `AttitudeConfig` defaults are illustrative; the power and thermal budget (#72) confirmed `control_power_w` (0.5 W).

## Communications (#44)

`pocketsat.spacecraft.Comms` is built as `Comms(config.comms)`. It has no starting record (it holds no data, ADR-0005), uses no randomness, and reads nothing from other subsystems. The radio is full duplex (ADR-0004 §10); each tick comms obeys `controls.radio.mode`:

| Mode | `receiver_on` | `transmitter_on` | `transmit_capacity_bytes` |
|---|---|---|---|
| `OFF` | false | false | 0 |
| `RX_ONLY` | true | false | 0 |
| `RX_TX` | true | true | `transmit_rate_bytes_per_s` over this tick (below): 120 every 100 ms tick |

- **Comms holds no data** (ADR-0004 §13). It only exposes how many bytes may be sent this tick; the flight computer (#56) moves payload chunks into DATA frames within that capacity and rate-limits downlink against it.
- **Transmit rate (#122).** The radio has a fixed data rate, `CommsConfig.transmit_rate_bytes_per_s` (1200 bytes/s, 9600 bit/s), whatever the tick length. Each tick with the transmitter on, comms converts it into whole bytes exactly, in integers, with the fraction of a byte carried in micro-bytes: `capacity, carry = divmod(rate * dt_us + carry, 1_000_000)`. So over any run of transmitter-on ticks the capacities add up to `floor(rate * on_time_us / 1_000_000)`, with no drift, at any tick length. At the default 100 ms tick the division is exact: 120 bytes every tick and a carry of 0, so behaviour is unchanged from the per-tick capacity it replaces. When the rate does not divide evenly into ticks the capacity varies by one byte (1234 bytes/s over 30 ms: 37 bytes, 38 every 50th tick); the flight computer reads each tick's capacity as before, and comms checks each tick's traffic against the capacity of the tick it was sent in. **While the transmitter is off** (radio `OFF` or `RX_ONLY`, including `transmitter_off`) the capacity is 0 and the carry is **held**: it neither accumulates (no burst when the transmitter returns) nor is discarded, as the payload holds its acquisition remainder while not acquiring, so the sum above counts transmitter-on time only. `reset(seed)` clears the carry. Frames are not split across ticks, so a tick shorter than one DATA frame's airtime (78 bytes: 65 ms at the default rate) can never send DATA.
- **Per-byte draw (#122).** `transmit_power_per_byte_w` is stated for bytes sent within one 100 ms tick (`PER_BYTE_DRAW_TICK_US`), so a byte costs `0.02 W * 0.1 s = 2 mJ`. Comms scales the per-byte draw by `100 ms / dt` of the tick the bytes were sent in, so a byte costs the same energy, and full rate the same 2.4 W, at any tick length. At 100 ms the factor is exactly 1 and the draw is bit-for-bit the unscaled formula.
- **Transmitter failure is a mode downgrade.** The controls have no "transmitter failed" field. `SilTarget` (#60) represents the `transmitter_off` fault by downgrading `controls.radio.mode` from `RX_TX` to `RX_ONLY`: transmitter off and capacity 0, receiver still on, so the spacecraft can hear and execute commands but cannot reply. Comms has no fault hook; it simply obeys the mode.
- **Transmit draw.** `transmit_draw_w(config, transmitter_on, sent_bytes, *, capacity_bytes, dt_us=100_000)` is a pure function: `transmitter_on_power_w + transmit_power_per_byte_w * (sent_bytes * 100_000 / dt_us)` while the transmitter is on, 0 while it is off; it rejects bytes above `capacity_bytes`. Comms uses it to check each tick's traffic against that tick's transmitter state and capacity. `CommsTruth.transmit_power_w` is the idle draw of this tick's transmitter state plus the previous tick's bytes' scaled per-byte draw (ADR-0007 §2; equal to `transmit_draw_w` of those bytes while the transmitter stays on); power (#36) adds it to the total load one tick late.
- **Radio traffic** ([ADR-0007](adr/0007-radio-traffic-input-to-comms.md), #98). Comms is told about the frames rather than seeing them: `SilTarget` passes each tick's traffic as `controls.radio_traffic` (`RadioTraffic(sent_bytes, uplink_lost_count, outbound_suppressed_count)`, per-tick values) in the next tick. The bytes are the wire bytes of every frame the flight computer sent (ACK/NACK, telemetry, DATA); lost uplink is counted by `SilTarget` (receiver off); suppressed ACK/NACK and telemetry frames are counted by the flight computer. Comms checks the record against its own previous tick (`transmit_draw_w`'s checks on the bytes against that tick's capacity, and no lost uplink while the receiver was on) and raises `ValueError` on a violation, never clipping. So comms reports tick N's traffic in tick N+1 and power sees the transmit energy in tick N+2.
- **Traffic counters.** `previous_tick_sent_bytes` is per tick and describes the previous tick: it is bounded by that tick's capacity, so it can be non-zero while `transmitter_on` is false (for example in the first tick of `transmitter_off`). `uplink_lost_count` and `outbound_suppressed_count` are running totals since `reset(seed)`; they survive the RESET command and `forced_reset`, which reboot only the flight computer. With default controls (subsystem tests) all three stay 0.
- **Radio-transmit inhibits** (story #42) belong to the flight computer: its downlink session (#56) sends no DATA while a power or thermal flag is set in the readings (`docs/spacecraft-modes.md`, "Downlink session"). Comms has no inhibits of its own.

After a reset, and until the first step, the radio is `OFF` with no draw, the rate carry and the counters at 0; the first step applies the commanded mode, and its traffic record must be empty. Readings equal truth (`CommsSnapshot.from_truth`).

| `CommsConfig` field | Default | Meaning |
|---|---|---|
| `transmit_rate_bytes_per_s` | 1200 | data rate while the transmitter is on, wire bytes per second (9600 bit/s at any tick length; 120 bytes per 100 ms tick, which fits one default 78-byte DATA frame plus an ACK) |
| `transmitter_on_power_w` | 0.15 | fixed (idle) draw while the transmitter is on, W |
| `transmit_power_per_byte_w` | 0.02 | extra draw per byte sent within a 100 ms tick, W per byte, scaled to the actual tick (2 mJ per byte; 2.4 W at full rate, so 2.55 W keyed, at any tick length) |

The draws follow the budget's transmitter model (#72, "Option C"): the radio stays `RX_TX` in every normal mode, so the fixed draw is a small idle draw and energy follows airtime ([power-thermal-budget.md](power-thermal-budget.md)). The shared fakes keep their illustrative 1.0 W transmit draw. `step()` plus `snapshot()` costs well under 1 µs on a developer laptop: the snapshot is rebuilt only when the mode, the capacity, or the traffic changes, and reused from a small cache when they repeat.

The story-level test for #42 (`tests/sil/test_payload_comms_story.py`) runs the real power, thermal, attitude, payload, and comms subsystems over three nominal orbits with the payload commanded on and the radio `RX_TX`, with scripted chunk releases (the payload is commanded on all the time, which no flight computer mode does). At every tick it checks the data accounting (`total_produced_bytes == buffered_bytes + total_released_bytes`), that the payload acquires exactly when its inhibits allow, that comms sends nothing, and that power sees every draw one tick late, plus determinism. A second scenario uses a `battery_drain` extra load to set `low_battery` and shows acquisition stopping and resuming. The story's two flight computer criteria, bytes sent minus bytes released never more than one tick's worth and the radio-transmit inhibits, are verified with the real flight computer's downlink (#56): every tick of three orbits of the reference profile through `SilTarget` (`tests/sil/test_downlink_story.py`), with flags forced mid-pass in `tests/sil/test_downlink_sil.py` and `tests/unit/test_downlink.py`.

## Shared test fakes

`pocketsat.spacecraft.fakes` provides fakes that every test directory can import, following `EchoTarget`'s precedent. Subsystem tickets test against fakes of the subsystems they read, so none waits for another's implementation.

- `FakeSubsystem(name, initial, script=None)` implements the `Subsystem` protocol and returns contract snapshots. Ticks count from 0 after `reset()`. A script is either a mapping from tick to snapshot (each entry takes effect in its tick and holds until the next) or a function `(tick, previous) -> snapshot`. The fake records `tick`, `elapsed_us`, `last_env`, `last_controls`, and `reset_count`.
- `DEFAULT_SNAPSHOTS` / `default_snapshot(name)`: plausible nominal snapshots (illustrative, not calibrated).
- `replace_truth(snapshot, **changes)` and `replace_readings(snapshot, **changes)` build scripted values; for payload and comms, `replace_truth` keeps readings equal to truth.
- `fake_subsystems(overrides)` and `fake_stack(fakes)` build all five fakes and a `SubsystemStack` of them.

For example, a payload draw that steps up at tick 10:

```python
from pocketsat.spacecraft.fakes import FakeSubsystem, default_snapshot, replace_truth

idle = default_snapshot("payload")
payload = FakeSubsystem("payload", idle, script={10: replace_truth(idle, power_w=3.0)})
```

The fakes use no randomness or wall-clock time, and the determinism guard scans them with the rest of `pocketsat`.

## Performance budget (#78)

Every multi-orbit test runs the model at the 100 ms tick (55,200 ticks per 92-minute orbit), so simulation speed is budgeted. The budgets are revisable by a PR that states and justifies the change with new measurements. Since #120 the full stack has two budgets, as flight software budgets average utilization and worst-case cycle time separately, with margin at the peak.

| Budget | Value | How it is checked |
|---|---|---|
| Full stack (five real subsystems, the flight computer, and `SilTarget`): **orbit average**, mean per tick over #72's reference profile (SCIENCE plus one 10-minute DOWNLINK pass per orbit) | at most **100 µs on CI Linux** (about 5.5 s per orbit). It drives CI time and #63's under-10-seconds-per-orbit target | `tests/sil/test_performance_budget.py` prints both full-stack figures on every run as `[perf #78]` lines; **in CI only** (`CI=true`) it **fails above twice each budget** (200 µs average, 300 µs worst case), so it catches real slowdowns without failing on a noisy shared runner. Locally it only prints: one run on a developer machine at load average ~15 measured 392 µs, against 57 to 62 µs unloaded |
| Full stack: **worst-case tick**, the busiest tick the flight software produces (the benchmark's `WORST_CASE_SCENARIOS`: today a DOWNLINK pass tick at full capacity with its ACKs, telemetry and DATA; at the default capacity at most two of the three fit one tick) | at most **150 µs on CI Linux** | as above |
| One subsystem's `step()` plus `snapshot()`, per tick, against the shared fakes | about **15 µs on CI** | guidance only: printed (marked "over guidance" if above), never a failure |
| Each CI test job (lint, the fast suite, each slow shard), per operating system | at most **5 minutes** | job times are recorded in the PR that changes them; the slow jobs print their 10 slowest tests (`--durations=10`) |

**The benchmark.** Each figure is the median of 5 runs, each a fresh `reset(seed)` with the real flight computer, booted untimed. The full-stack loop is the orchestrator's: send any command, sample `NominalEnvironment`, `apply_environment`, `advance` one tick, drain `receive()`; each tick is timed on its own. The simulation is deterministic, so every run sends the same frames, and the benchmark asserts that its timed ticks are the scenario it claims. The benchmark is part of the fast suite (about 2 s). Wall-clock timing is allowed there: the determinism guard covers only simulation code in `src/pocketsat`.

- **Orbit average (#120).** One reference orbit 20 times shorter (2,760 timed ticks): SCIENCE with a PING every 10 s, then BEGIN_DOWNLINK for the last 300 ticks (the 10-minute pass window, scaled); the flight computer sends every stored chunk and `DOWNLINK_COMPLETE` returns it to SCIENCE inside the window, as in a full orbit. It starts detumbled (pointing error and rate 0), so the payload acquires from the first SCIENCE tick, as for the rest of the mission once detumbled; the default start detumbles for about 150 s, in which a tick is cheaper. The environment runs 20 times faster than simulated time, so the shortened orbit still sees sunlight and eclipse, with the pass at the end of eclipse. **Why the mix is representative:** data is produced at 4 bytes per tick in every SCIENCE and DOWNLINK tick and goes down at one 64-byte chunk per tick, so DATA ticks are the same share of a shortened orbit as of a full one (153 of 2,760 and 3,075 of 55,200, 5.5 to 5.6%; 1/16 once passes repeat). The test asserts the mode sequence, no power or thermal flag, and a DATA share within one percentage point of 1/16. A whole orbit would take about 4 s per run, too slow for the fast suite five times over, so the full orbit is measured once by a slow test instead (`test_a_full_reference_orbit_matches_the_scaled_orbit`, slow shard 2), which prints the full-orbit average beside the scaled one in the same job: the two agree within a few per cent (figures below). The printed line also splits the average into its SCIENCE ticks and its pass-window ticks.
- **Worst-case tick (#120).** `WORST_CASE_SCENARIOS` lists the scenarios that define the worst case, today one, a **DOWNLINK pass at full capacity**: 2,048 chunks stored (more than the timed ticks can send), detumbled so the payload acquires as in a real pass, 1,000 timed ticks with a PING every 3 ticks, so the 10-tick telemetry period meets an ACK in every phase. Every tick releases the last tick's chunk and sends what fits the 120-byte capacity, in ADR-0004 §10 order. Ticks are grouped by the frames they sent; the worst case is the costliest group's median tick. At the default capacity ACK (14 bytes), telemetry (36) and DATA (78) together (128 bytes) do not fit one tick, so the groups are DATA alone, ACK + DATA, telemetry + DATA, and ACK + telemetry (the DATA frame is not built and waits a tick); the test asserts exactly these groups, every PING ACKed, and no room for another DATA frame in any tick. **When a feature makes a busier tick (for example by sending more per tick), add a scenario for it to `WORST_CASE_SCENARIOS`.** Rare transition ticks (a mode entry, a reboot) are not budgeted.
- **Per subsystem.** Each subsystem is timed alone in SCIENCE controls, with the four default fakes (#76) on its board and environment states spread over a whole orbit.

Until #120 the benchmark ran SCIENCE at the default telemetry cadence (the single budgeted figure), SCIENCE with telemetry every tick (`TelemetryConfig.uniform(tick)`; until #55 a test-only scripted flight computer), and a DOWNLINK pass at full capacity, each 1,000 ticks timed as a whole and detumbling. The orbit average replaces the first; telemetry encoding and its frame CRC are now timed in the worst-case groups that send telemetry, so the second was removed; the worst case replaces the third. Their figures are kept in the history below.

**Measured (2026-10-05; the #55 rows 2026-10-06)**, µs per tick:

| | Apple Silicon laptop | CI ubuntu-latest | CI macos-latest |
|---|---|---|---|
| Full stack, real flight computer, before #55 (no telemetry) | 57 | 61, 61 | 46, 26 |
| Full stack, scripted double, telemetry frame every tick (table CRC) | 66 | 71, 71 | 70, 30 |
| Full stack, scripted double, telemetry frame every tick (old bit-by-bit CRC) | 97 | — | — |
| Full stack, real flight computer, default telemetry cadence (#55, 2026-10-06) | 61 | 33 | 72 |
| Full stack, real flight computer, telemetry every tick (#55, 2026-10-06) | 81 | 46 | 76 |
| Full stack, real flight computer, DOWNLINK pass at full capacity, table CRC (#56, 2026-10-07) | 112 | 111 | 76 |
| Full stack, real flight computer, DOWNLINK pass at full capacity, `binascii` CRC (#56, 2026-10-07) | 97 | 103, 81 | 77, 85 |
| **Orbit average**, reference profile, 1/20-scale orbit (#120, 2026-10-07, fast suite) | 75, 75 | 37, 81, 79, 57, 83 | 54, 85, 62, 66, 45 |
| **Orbit average**, reference profile, full orbit (#120, slow test) | 81 | 56, 46, 44, 77, 78 | 54, 88, 58, 75, 50 |
| **Worst-case tick**, full-capacity pass: DATA / ACK + DATA / telemetry + DATA / ACK + telemetry (#120) | 96 / 114 / 117 / 118 | 48 / 58 / 59 / **60**; 99 / 119 / 121 / **123**; 99 / 119 / 121 / **123**; 74 / 89 / 92 / **92**; 99 / 118 / 120 / **122** | 63 / 74 / **77** / 77; 67 / 81 / **90** / 87; 64 / 77 / **81** / 79; 64 / 77 / **83** / 80; 57 / 68 / 70 / **70** |
| power / thermal / attitude / payload / comms | 9.0 / 7.3 / 9.2 / 9.6 / 0.3 | 7.0 / 5.8 / 7.4 / 8.2 / 0.2 | 8.5 / 6.6 / 7.4 / 10.7 / 0.2 |
| **Orbit average**, 1/20-scale orbit, `main` after #98 (4ad4a4e, before #121; 2026-10-08) | 77, 76, 77 | 72, 84 | 61, 73 |
| **Worst-case tick**, `main` after #98 (before #121): DATA / ACK + DATA / telemetry + DATA / ACK + telemetry | 104 / 122 / 124 / 125 | 96 / 112 / 114 / **115**; 114 / 135 / 137 / **139** | 77 / 90 / 97 / **99**; 67 / 79 / **86** / 85 |
| **Orbit average**, 1/20-scale orbit, after #121 (on 4ad4a4e, fast suite) | 63, 63, 63 | 38, 70, 69 | 53, 79, 70 |
| **Orbit average**, full orbit, after #121 (slow test) | — | 50, 36, 62 | 57, 44, 35 |
| **Worst-case tick** after #121: DATA / ACK + DATA / telemetry + DATA / ACK + telemetry | 82 / 95 / 94 / 95 | 49 / 57 / 57 / **58**; 88 / **103** / 101 / 103; 90 / **105** / 103 / 104 | 55 / 63 / **65** / 65; 97 / 121 / **127** / 99; 58 / 68 / 68 / **69** |
| power / thermal / attitude / payload / comms, after #121 | 7.6 / 5.9 / 7.2 / 6.4 / 0.3 | 6.3 / 4.9 / 6.1 / 5.2 / 0.2 (third run) | 8.2 / 6.8 / 5.5 / 5.7 / 0.2 (third run) |

CI figures are from two runs of the same commit; macOS runners varied by almost a factor of two between runs. GitHub's Linux runners measured about as fast as the laptop for this single-threaded code, not the 1.5 to 2.5 times slower first assumed, so the 100 µs budget has about 40% headroom with the real flight computer and about 30% with telemetry every tick. #55's figures are from one CI run; on the same laptop run, the default cadence costs about 4 µs per tick over no telemetry (57 → 61) and telemetry every tick about 24 µs (one frame encoded, framed, and drained per tick); that CI run's ubuntu runner was unusually fast.

**Frame CRC.** The bit-by-bit CRC-16/CCITT-FALSE cost about 12 µs per telemetry frame, which took the telemetry-every-tick stack to about 97 µs. #78 replaced it with a 256-entry lookup table (one lookup per byte, integer-only, so portable under ADR-0006), about six times faster, with byte-identical output. #56's DOWNLINK pass frames a 78-byte DATA frame every tick, where the table still cost about 10 µs and took the pass over the budget on CI Linux (111 µs), so the frame codec now computes the CRC with `binascii.crc_hqx(data, 0xFFFF)` (C). The table stays as `crc16_ccitt_false_table`, the algorithm the Phase 7 firmware mirrors; the unit tests check the codec's CRC, the table, and the bitwise reference against each other and against the shared vectors.

**DOWNLINK pass (#56).** The third full-stack run is a DOWNLINK pass at full capacity: a starting buffer of 2048 chunks, so the session sends one 78-byte DATA frame in every timed tick (telemetry once a second beside it). Per tick it adds the chunk content (`chunk_content`, about 9 µs: a Python xorshift loop, 16 words per 64-byte chunk), the frame and its CRC, the release in the controls, and the payload's snapshot rebuild after each release (about 8 µs; the payload pays the same in every acquiring SCIENCE tick, which the SCIENCE run, still detumbling in its first 106 s, does not show). With the `binascii` CRC the pass measured 97 µs per tick on the laptop and 103 and 81 µs on CI ubuntu in two runs (SCIENCE 66 and 50 µs in the same runs), so a pass tick sits at or under the budget, well under the 2x CI gate. A pass is 10 minutes of a 92-minute orbit, so the reference profile's orbit average stays near 70 µs. #121 took both remaining levers, a faster `chunk_content` and a cheaper payload snapshot rebuild (below).

**Two budgets (#120).** The figures above for #120 are from five CI runs of the same benchmark code (three pushes, two of them changing only documentation, and two re-runs); the worst case is in bold. GitHub's ubuntu runners came in two speeds: in the fast-suite jobs the orbit average measured 37 and 57 µs on two runs and 79 to 83 µs on the other three, and the worst-case tick 60 and 92 µs against 122 to 123 µs. So on CI ubuntu the orbit average is at most 83 µs (about 17% headroom under 100 µs) and the worst-case tick at most 123 µs (about 20% under 150 µs), both far from the 200 and 300 µs gates. The worst case is ACK + telemetry on ubuntu and mostly telemetry + DATA on macOS; the three busy groups are within about 4 µs of each other on ubuntu (9 µs on macOS), and each is 10 to 24 µs over a DATA-only pass tick: an ACK (uplink frame decode, command dispatch, ACK frame) costs about as much as a telemetry frame. The orbit average is higher than the about 70 µs estimated in #56 because #56's SCIENCE run was still detumbling, when the payload does not acquire; detumbled, a SCIENCE tick costs about 73 µs on the laptop and 36 to 79 µs on CI ubuntu (the printed SCIENCE share). The pass window is about 12 µs per tick dearer than SCIENCE, but only about half of it sends DATA (the pass completes after about 5 of its 10 minutes), so it adds only about 1.5 µs to the average. The slow full-orbit test agreed with the scaled orbit measured in the same job within 3% on ubuntu (ratios 0.99 to 1.03) and within 20% on macOS (0.80 to 1.03, whose runners vary more between consecutive measurements); on the laptop 0.98. It takes about 5.5 s on either OS, in slow shard 2. #121 speeds up the pass tick against these figures.

**Pass tick speed-up (#121).** Profiled first (cProfile over the worst-case pass and the scaled orbit, then each hot spot timed alone with `timeit`), then the cheap hot spots were made faster without changing any output. Per call on the laptop, before → after (measured on 07371fb; #98 did not change these functions):

| Hot spot | Before | After | What changed |
|---|---|---|---|
| `chunk_content(id, 64)` (every DATA frame) | 8.1 µs | 1.0 µs | Each xorshift step only shifts and XORs, so the content is linear over GF(2) in the starting state: it is the XOR of four 256-entry tables indexed by the state's bytes, built once per chunk size (up to 256 bytes; larger chunks still run the loop). Integer XOR only (ADR-0006) |
| Payload snapshot rebuild (every release and every acquiring tick) | 9.1 µs | 4.9 µs | The mirrored-snapshot check (`_values`) built its field list with `dataclasses.fields` on every call; it now keeps one `attrgetter` per record type |
| `portable_normal` (7 draws per tick) | 1.26 µs | 0.61 µs | The 12 uniform draws are added unrolled, left to right, exactly as the loop added them |
| `encode_telemetry` | 10.0 µs | 4.4 µs | Flags ORed as plain ints rather than `IntFlag` (same packed value); `quantize` skips its type checks for a `float` |
| Framing an outbound frame | 2.1 µs | 0.8 µs | The flight computer encodes from the fields (`encode_frame_fields`, the same bytes and errors) instead of building a `Frame` first |
| `decode_frame` (uplink, twice per command) / `decode_command` | 2.8 / 1.3 µs | 2.2 / 0.2 µs | Dict lookups instead of enum calls; `decode_command` returns one shared, immutable `ParsedCommand` per possible result |
| Controls with a release, repeated (a pass tick with ACK and telemetry and no room for DATA) | 3.3 µs | 0.2 µs | `controls_for_mode` remembers the controls of its last two releases |
| `SubsystemStack.snapshot` (every tick) | 3.5 µs | 1.6 µs | Reads the board's published snapshots directly |

The dispatcher no longer works out the SAFE exit guard (`active_flags`) for a tick whose commands are all PINGs; the ACK preview transition (#51) runs only for mode commands, which a pass does not send. **Behaviour neutral:** `tests/sil/test_behavior_digests.py` hashes the downlink bytes and every tick's `SpacecraftState` as values only (each subsystem's truth and readings field values in field order, enums by value, floats by `repr`, so compared exactly; field names are not hashed, so a rename such as #98's keeps the digests) over two scenarios and two seeds each (SCIENCE while acquiring, a pass interrupted by `transmitter_off`, `sensor_freeze`, `battery_drain` and a reboot and then resumed to `DOWNLINK_COMPLETE`, every command, a NACK, undecodable uplink, a held reset, SAFE), against digests recorded on `main` at 4ad4a4e (after #98) before the change; the shared vectors are unchanged, and unit tests check the table-driven chunk content, the unrolled normal draw and the field encoder against their definitions.

#98 added per-tick work (about 7 µs on a laptop pass tick, 118 → 125 µs, in the radio traffic merge and comms), so #121's figures were re-measured on top of it. On CI (three runs of the final code on 4ad4a4e, table above) the worst-case tick measured 103 and 105 µs on slower ubuntu runners (`main` after #98: 139 µs on a slower runner, so about 25% less) and 58 µs on a faster one, and 65 to 127 µs on macOS (`main`: 86 and 99; macOS runners varied by a factor of two between runs). The orbit average measured 69 and 70 µs on the slower ubuntu runners (`main`: 84) and 38 µs on the faster one; the full orbit agreed with the scaled one within 4% on ubuntu. So on the slower ubuntu runners the worst case has about 30% headroom under 150 µs and the orbit average about 30% under 100 µs. #121 targeted about 90 µs for the worst case on CI ubuntu; before #98 three of four runs met it and the slowest measured 96 µs, which the maintainer accepted; with #98's extra work the slower runners now measure 103 to 105 µs. The remaining levers are backlog #128 (next paragraph). On the laptop the worst case went from 125 to 95 µs and the orbit average from 77 to 63 µs (both on 4ad4a4e).

**CI suite time with #56.** The three-orbit downlink story (`tests/sil/test_downlink_story.py`, slow shard 2) takes 7 to 10 s on ubuntu and 12 to 14 s on macOS (two CI runs); the shard 2 jobs took 58 s (ubuntu) and 1m38s (macOS), shard 1 jobs 1m34s and 1m38s. The new fast tests (`test_downlink.py`, `test_downlink_sil.py`, the DATA codec and vectors) add about 2 s to the fast suite.

**Remaining hot spots** after #121 (timed alone on the laptop, per tick of a full-capacity pass): the subsystems' `step()` plus `snapshot()` about 33 µs, of which the seven `portable_normal` draws are about 4 µs and building the frozen snapshot records most of the rest; the safety rules (`safety.evaluate`) about 7 µs; `SilTarget`'s tick record (`SilTick`) about 5 µs; the controls with a new release about 3.5 µs, mostly `SpacecraftControls` validation; `SpacecraftReadings.from_state` about 2.5 µs; decoding each uplink frame twice (`SilTarget` drops undecodable frames, then the flight computer decodes it) about 2.2 µs a frame; a telemetry frame about 8 µs in all (`encode_telemetry` 4.4); a DATA frame about 6 µs (the session record, the chunk, framing). These, and the per-tick cost #98 added in `SilTarget`'s merge and comms, are backlog issue #128 (not a Phase 1 blocker).

**CI suite time.** The fast suite takes about 13 s on each OS. The slow suite (`pytest -m slow`) took 178 s on ubuntu and 166 s on macOS in a single job on `main` before #78 (jobs 3m13s and 3m02s), close to the 5-minute budget with #55, #56, #63 and #64 still to add. It now runs as two shards per OS in parallel jobs: shard 1 is the files in `SLOW_SHARD_1_FILES` in `.github/workflows/ci.yml` (the three-stack and multi-orbit SIL stories), shard 2 is every other slow test, so a new slow test always runs. In the final CI run the shards took 99 and 68 s of pytest on ubuntu and 104 and 74 s on macOS (jobs 1m21s to 2m03s); runner speed varied by about 30% between runs. If a shard approaches 4 minutes, move whole files into `SLOW_SHARD_1_FILES` to rebalance, or add a shard; never coarsen the tick or skip a test on PRs.
