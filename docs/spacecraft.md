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

## Cross-subsystem reads

Subsystems never hold references to each other; they read each other's snapshots through `SpacecraftState` (ADR-0004 §3). The read mechanism is implemented in #36. The timing rule, also from #36: a subsystem earlier in `STEP_ORDER` is read as of the current tick, and a later one as of the previous tick.

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
