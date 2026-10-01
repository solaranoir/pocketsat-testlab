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
