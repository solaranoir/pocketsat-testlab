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
| Comms (#44) | `CommsTruth`: `radio_mode`, `receiver_on`, `transmitter_on`, `transmit_capacity_bytes`, `sent_bytes`, `uplink_lost_count`, `outbound_suppressed_count`, `transmit_power_w` | `CommsReadings`: equal to truth |

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
| flight computer | `PowerReadings.low_battery` | decision: mode logic, telemetry flag (#48, #54) | current tick |
| flight computer | `PowerReadings.critical_battery` | decision: mode logic, telemetry flag (#48, #54) | current tick |
| flight computer | `ThermalReadings.over_temp` | decision: mode logic, telemetry flag (#48, #54) | current tick |
| flight computer | `ThermalReadings.under_temp` | decision: mode logic, telemetry flag (#48, #54) | current tick |
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

All `PowerConfig` defaults (20 Wh battery, 8 W array, 2 W base load, 6.0 to 8.4 V, the noise levels, and the flag thresholds) are provisional; #72 calibrates them. With the defaults and the shared fakes' draws (attitude control 0.5 W, transmit 1.0 W), a nominal 92-minute orbit with 35% eclipse is net positive (about +2.6 Wh per orbit), which the story-level test (`tests/sil/test_power_story.py`) checks over three orbits. `step()` plus `snapshot()` costs about 10 µs on a developer laptop.

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
- **Reset.** `reset(rng)` returns to `ThermalInitial` (provisional 20 °C / 20 °C until #72) and requests the noise stream; it must be called before the first `step()`, otherwise `step()` raises `RuntimeError`. The heater starts on if the starting battery temperature is below the ON setpoint.

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

**Settings** (`ThermalConfig`; every default is provisional, #72 calibrates them):

| Setting | Default | Meaning |
|---|---|---|
| `battery_heat_capacity_j_per_c` | 80.0 | battery heat capacity, J/°C (about a 100 g lithium-ion pack) |
| `electronics_heat_capacity_j_per_c` | 300.0 | electronics heat capacity, J/°C |
| `battery_conductance_w_per_c` | 0.12 | battery to ambient, W/°C (positive) |
| `electronics_conductance_w_per_c` | 0.25 | electronics to ambient, W/°C (positive) |
| `coupling_conductance_w_per_c` | 0.04 | battery to electronics, W/°C (0 decouples) |
| `battery_dissipation_fraction` | 0.25 | share of the electrical dissipation (heater excluded) heating the battery; the rest heats the electronics |
| `heater_power_w` | 3.0 | survival heater power while on, W |
| `heater_on_setpoint_c` | 0.0 | heater switches on below this battery temperature, °C |
| `heater_off_setpoint_c` | 4.0 | heater switches off above this battery temperature, °C; above the ON setpoint |
| `temperature_noise_c` | 0.2 | standard deviation of each reported temperature's noise at `sensor_noise_scale` 1.0, °C |
| `battery_survival_limit_c` | -10.0 | lowest battery temperature it survives, °C; bottom of the threshold order |
| `battery_under_temp_c` / `battery_under_temp_clear_c` | -5.0 / -2.0 | battery under-temperature set / clear, °C |
| `battery_over_temp_c` / `battery_over_temp_clear_c` | 45.0 / 42.0 | battery over-temperature set / clear, °C |
| `electronics_under_temp_c` / `electronics_under_temp_clear_c` | -25.0 / -22.0 | electronics under-temperature set / clear, °C |
| `electronics_over_temp_c` / `electronics_over_temp_clear_c` | 60.0 / 57.0 | electronics over-temperature set / clear, °C |

With these defaults, the real power subsystem, and the shared fakes' draws (3.5 W of load), the battery peaks near 28 °C and the electronics near 30 °C in a nominal orbit's sunlight. In the -20 °C eclipse the battery would settle near -11 °C without the heater and near +9 °C with it on, so the heater cycles between its setpoints, three times per nominal eclipse (about 12 minutes on in total), and at 1 s ticks the battery never falls more than a few hundredths of a degree below the ON setpoint. The values were chosen for that #73 behavior: heater power and battery isolation sized so the heater-on steady state is well above the OFF setpoint.

The flag thresholds (#39) were chosen to leave nominal orbits clear with margin: over a nominal orbit the true battery stays within about 0 °C to 28 °C and the electronics within about -2 °C to 30 °C, so even with the ±1.2 °C noise bound the readings stay at least 3.8 °C from the nearest battery threshold (-5 °C) and more than 20 °C from the electronics thresholds. The battery thresholds bracket a lithium-ion pack's usual range (no charging below about 0 °C, at most about 45 °C); the electronics thresholds are below the -20 °C eclipse sink, which the electronics, always dissipating, never reach in a nominal orbit, and well below typical component limits. They are provisional; #72 calibrates them. The story-level test (`tests/sil/test_thermal_story.py`, story #37) runs the real power and thermal subsystems over three nominal orbits and checks that the temperatures settle into the same pattern each orbit, that no flag is raised, that the heater cycles in every eclipse with its draw in power's load, and determinism. `step()` plus `snapshot()` costs about 10 µs on a developer laptop.

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

All `AttitudeConfig` defaults are illustrative; the power and thermal budget (#72) finalizes `control_power_w`.

## Communications (#44)

`pocketsat.spacecraft.Comms` is built as `Comms(config.comms)`. It has no starting record (it holds no data, ADR-0005), uses no randomness, and reads nothing from other subsystems. The radio is full duplex (ADR-0004 §10); each tick comms obeys `controls.radio.mode`:

| Mode | `receiver_on` | `transmitter_on` | `transmit_capacity_bytes` |
|---|---|---|---|
| `OFF` | false | false | 0 |
| `RX_ONLY` | true | false | 0 |
| `RX_TX` | true | true | `CommsConfig.transmit_capacity_bytes` |

- **Comms holds no data** (ADR-0004 §13). It only exposes how many bytes may be sent this tick; the flight computer (#56) moves payload chunks into DATA frames within that capacity and rate-limits downlink against it. The capacity is per tick, whatever the tick length.
- **Transmitter failure is a mode downgrade.** The controls have no "transmitter failed" field. `SilTarget` (#60) represents the `transmitter_off` fault by downgrading `controls.radio.mode` from `RX_TX` to `RX_ONLY`: transmitter off and capacity 0, receiver still on, so the spacecraft can hear and execute commands but cannot reply. Comms has no fault hook; it simply obeys the mode.
- **Transmit draw.** `transmit_draw_w(config, transmitter_on, sent_bytes)` is a pure function: `transmitter_on_power_w + transmit_power_per_byte_w * sent_bytes` while the transmitter is on, 0 while it is off. `CommsTruth.transmit_power_w` is its value for the tick; power (#36) adds it to the total load one tick late.
- **Traffic counters.** `sent_bytes`, `uplink_lost_count`, and `outbound_suppressed_count` depend on what the flight computer sends (#56) and on the uplink frames `SilTarget` delivers (#59). Neither exists yet, so the counters are 0 and the draw is computed with `sent_bytes = 0`; #56 and #59 define the input path and fill them.
- **Radio-transmit inhibits** (story #42) belong to the flight computer: its downlink (#56) decides whether to send, and #56 verifies them. Comms has no inhibits of its own.

After a reset, and until the first step, the radio is `OFF` with no draw and the counters at 0; the first step applies the commanded mode. Readings equal truth (`CommsSnapshot.from_truth`).

| `CommsConfig` field | Default | Meaning |
|---|---|---|
| `transmit_capacity_bytes` | 120 | bytes per tick while the transmitter is on (9600 bit/s at the 100 ms tick; fits one default 78-byte DATA frame plus an ACK) |
| `transmitter_on_power_w` | 1.0 | fixed draw while the transmitter is on, W (the shared fakes' transmit draw) |
| `transmit_power_per_byte_w` | 0.005 | extra draw per byte sent in the tick, W per byte (0.6 W at full capacity) |

The defaults are provisional; #72 calibrates the draws. `step()` plus `snapshot()` costs well under 1 µs on a developer laptop: the snapshot is rebuilt only when the mode changes.

The story-level test for #42 (`tests/sil/test_payload_comms_story.py`) runs the real power, thermal, attitude, payload, and comms subsystems over three nominal orbits with the payload commanded on and the radio `RX_TX`, with scripted chunk releases standing in for the flight computer. At every tick it checks the data accounting (`total_produced_bytes == buffered_bytes + total_released_bytes`), that the payload acquires exactly when its inhibits allow, that comms sends nothing, and that power sees every draw one tick late, plus determinism. A second scenario uses a `battery_drain` extra load to set `low_battery` and shows acquisition stopping and resuming. The downlink accounting (bytes sent minus bytes released) and the radio-transmit inhibits are verified with the flight computer's downlink (#56).

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
