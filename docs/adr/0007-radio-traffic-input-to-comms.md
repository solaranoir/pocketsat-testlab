# ADR-0007: Radio traffic input to comms

- **Status:** Accepted (implemented by #98; amended by #122, see below)
- **Phase:** 1
- **Amends / Supersedes:** Amends ADR-0004 (§1 the controls record, §2 step a and the latency table, §10 transmit power). Related: ADR-0002 (`TestTarget`, HIL), ADR-0003 (determinism), ADR-0006 (portable arithmetic). Issue: #104. Implemented by #98.

## Context

Comms (#44) reports three traffic values in `CommsTruth`: `sent_bytes` (renamed `previous_tick_sent_bytes` by this ADR, section 3), `uplink_lost_count`, and `outbound_suppressed_count`. Its transmit draw is `transmit_draw_w(config, transmitter_on, sent_bytes)`, which power adds to its load one tick late (ADR-0004 §3). Today all three values are always 0, because comms reads nothing (ADR-0004 §1, §3) and no input path exists. The traffic is decided outside comms:

- **Outbound bytes:** the flight computer sends ACK/NACK (#51), telemetry (#55), and DATA (#56), in that priority order, within the `transmit_capacity_bytes` it reads from comms' snapshot (ADR-0004 §10).
- **Lost uplink:** `SilTarget` drops uplink frames that arrive while comms' receiver is off (#59).
- **Suppressed outbound:** ACK/NACK and telemetry that do not fit the capacity are not queued and are counted (#51, #55), including everything the spacecraft would send while `transmitter_off` holds the capacity at 0 (#60).

The timing constraint comes from ADR-0004 §2. Within tick N:

- a. Merge the flight computer's tick N-1 controls with the active fault overrides.
- b. Subsystems step in `STEP_ORDER` with the merged controls; **comms steps last**.
- c. The flight computer decodes uplink, executes commands, and sends ACK/NACK.
- d. The flight computer updates the mode.
- e. The flight computer emits telemetry if due.
- f. The flight computer produces controls for tick N+1.

Lost uplink is known only after step b (it depends on comms' tick N receiver state), and the outbound bytes only after step e. Comms has already stepped by then. So comms cannot see tick N's traffic in tick N's `step()` without a second entry point into the subsystem or a change to the step order.

## Decision

### 1. The path: a `RadioTraffic` record in `SpacecraftControls`, filled by `SilTarget`

A new frozen record in `pocketsat.spacecraft.controls`:

```python
@dataclass(frozen=True)
class RadioTraffic:
    sent_bytes: int = 0                 # outbound bytes transmitted in the tick
    uplink_lost_count: int = 0          # uplink frames lost in the tick (receiver off)
    outbound_suppressed_count: int = 0  # ACK/NACK and telemetry frames suppressed in the tick
```

All three are per-tick, non-negative `int`s (validated like the other control records: `TypeError` for a non-int or a bool, `ValueError` for a negative value). `SpacecraftControls` gains one field, `radio_traffic: RadioTraffic = RadioTraffic()`.

- **Who fills it:** `SilTarget`, in the same merge function that applies fault overrides at step a (ADR-0004 §5, §14). The flight computer's controls function never sets it; its controls carry the default (zeros), and the merge replaces the field. It is a target-supplied field like `frozen_sensors` and `extra_load_w`, not a flight computer decision.
- **Where the numbers come from, for tick N:**
  - `sent_bytes`: the sum of the encoded lengths of every frame the flight computer handed to `SilTarget` for transmission in tick N (the frames `receive()` will return), all frame types. Bytes are wire bytes (header, payload, and CRC), the same unit as `transmit_capacity_bytes`.
  - `uplink_lost_count`: the frames `SilTarget` dropped in tick N because comms' receiver was off after step b (#59).
  - `outbound_suppressed_count`: the number of ACK/NACK and telemetry frames the flight computer suppressed in tick N under its capacity rule (#51, #55). **The flight computer counts suppression; `SilTarget` only passes the count on.** The flight computer's per-tick output to `SilTarget`, from `pocketsat.flight.FlightComputer.step()` (#101), therefore includes this count alongside its downlink frames and next-tick controls; #101 defines the output shape and #51 and #55 fill the count.
- **Who reads it:** comms only. It is comms' own record, so comms still reads nothing from other subsystems or from the flight computer (ADR-0004 §1, §3).
- **What does not change:** the `Subsystem` protocol, `SubsystemStack.step`, the `SnapshotBoard`, `STEP_ORDER`, and `TestTarget`. Subsystem and story-level tests that use default controls see zero traffic, exactly as today.

Why this fits steps a to f: step a is already the single place where `SilTarget` puts things into the controls that the flight computer did not decide (fault overrides), and it is the first point after step e at which data can reach comms through an existing channel. `SilTarget` holds the tick N traffic between step e of tick N and step a of tick N+1, next to the tick N controls it already holds for the same hand-over. Because controls are recorded each tick alongside `SpacecraftState` (ADR-0004 §14), the traffic is recorded and replayable with no extra mechanism.

### 2. Timing: comms reports tick N's traffic in tick N+1; power sees the energy in tick N+2

ADR-0004 §2 step a becomes: *Merge the flight computer's tick N-1 controls with the active fault overrides and the tick N-1 radio traffic.*

In tick N+1, comms' snapshot holds two kinds of field:

- **State of tick N+1** (unchanged meaning): `radio_mode`, `receiver_on`, `transmitter_on`, `transmit_capacity_bytes`. The flight computer reads this capacity in tick N+1 to decide tick N+1's traffic.
- **Traffic of tick N** (the previous tick): `previous_tick_sent_bytes`, and the increments added to `uplink_lost_count` and `outbound_suppressed_count`.

`transmit_power_w` in tick N+1 is the idle draw of tick N+1's state plus the per-byte draw of tick N's bytes:

```
transmit_power_w = transmit_draw_w(config, transmitter_on, 0)
                 + config.transmit_power_per_byte_w * radio_traffic.sent_bytes
```

(#122: the per-byte part is scaled by `100 ms / tick N's length`, a factor of exactly 1 at the 100 ms tick; see the amendment at the end of section 4.) While the transmitter is on in both ticks this is bit-for-bit `transmit_draw_w(config, True, radio_traffic.sent_bytes)`, because `transmit_draw_w(config, True, 0)` returns `transmitter_on_power_w` exactly and the sum is evaluated in the same order. While the transmitter is off in tick N+1 (for example `transmitter_off` injected in tick N+1), the idle part is 0 and the per-byte part of tick N is still charged, so no transmitted byte goes unpaid.

So the snapshot's byte count can be non-zero in a tick whose `transmitter_on` is false and whose capacity is 0: it is bounded by the **previous** tick's capacity. That is why section 3 renames the field to `previous_tick_sent_bytes`.

**This two-tick delay is decided** (accepted by the maintainer in the review of #104), in preference to the same-tick report that would save one tick (see Alternatives considered).

**End-to-end delay.** A frame sent in tick N (step c or e) is reported by comms at step b of tick N+1, and power, which steps before comms and so reads it one tick late (ADR-0004 §3), integrates its energy at step b of tick N+2. **Total: two ticks** (200 ms at the default 100 ms tick). The idle draw keeps today's timing: a radio mode change takes effect in tick N+1 and power sees it in tick N+2, the same as every other draw.

New rows for ADR-0004's latency table:

| Chain | Latency (100 ms default tick) |
|---|---|
| Frame sent → comms reports `previous_tick_sent_bytes` | +1 tick |
| Frame sent → power sees its transmit energy | +2 ticks |
| Uplink lost or outbound suppressed → comms' count | +1 tick |

This matches the command chain #59 already tests: a command received in tick N is ACKed in tick N, its effect lands in tick N+1, and power sees the draw in tick N+2. The ACK's own energy also reaches power in tick N+2.

Over a pass, the per-byte energy is conserved and only shifted in time: the per-byte energy of the frames sent in ticks N..M appears in comms' `transmit_power_w` in ticks N+1..M+1 and in power's load in ticks N+2..M+2. A test window that extends two ticks past the last frame therefore sees all of it. A 200 ms shift is far below anything the budget (#72) or the scenarios measure.

### 3. Who counts what

| `CommsTruth` field | Kind | Counted by | Reported by comms |
|---|---|---|---|
| `previous_tick_sent_bytes` (renamed from `sent_bytes`) | **per tick**: bytes sent in the previous tick | `SilTarget`, summing the flight computer's outbound frames (all types: ACK/NACK, telemetry, DATA) | tick N+1 |
| `uplink_lost_count` | **running total** since `reset(seed)` | `SilTarget` per tick (#59); comms accumulates | tick N+1 |
| `outbound_suppressed_count` | **running total** since `reset(seed)` | flight computer per tick, through its capacity rule (#51, #55, and #60 through capacity 0), output by `FlightComputer.step()` (#101); `SilTarget` passes it on; comms accumulates | tick N+1 |

- `RadioTraffic` always carries **per-tick** values. Comms owns the running totals and adds each tick's values to them. #59 and #60 keep no separate counters of their own (per-tick values in `SilTarget` exist only for the hand-over to the next tick).
- **Rename: `CommsTruth.sent_bytes` / `CommsReadings.sent_bytes` become `previous_tick_sent_bytes`.** The snapshot pairs the current tick's radio state (`transmitter_on`, `transmit_capacity_bytes`) with the previous tick's bytes, so the value can be non-zero while `transmitter_on` is false, or larger than the current capacity. The old name read as "bytes sent this tick" and invited exactly that misreading; the new name states the lag. #98 implements the contract change (`snapshots.py`, comms, tests, docs). The name ends in `_bytes`, so the naming convention needs no allowlist entry.
- **`RadioTraffic.sent_bytes` keeps its name.** The record describes one tick, the tick it reports on, so its bytes are that tick's bytes and "previous" would be wrong from the record's point of view. The lag is in when comms applies it, and the comms field name carries it.
- `previous_tick_sent_bytes` stays per tick because the draw is per tick; a running byte total is the sum over the recorded states.
- The running totals are confirmed by the maintainer and match the existing contract docstrings ("since the last reset").
- DATA that does not fit is **not** "suppressed": it stays in the payload buffer and is sent later (ADR-0004 §13, #56). Only ACK/NACK and telemetry, which are not queued, count (ADR-0004 §10).
- Uplink frames that arrive while the receiver is on but fail to decode are received, not lost. They are #59's "undecodable" count and not part of `uplink_lost_count`.
- Comms is a subsystem, so its totals survive the RESET command and `forced_reset` (ADR-0004 §9); only `reset(seed)` returns them to 0. After `reset(seed)`, tick 0 receives a zero `RadioTraffic`.

### 4. Capacity enforcement and violation

- **The flight computer guarantees the limits.** It reads `CommsReadings.transmit_capacity_bytes` for tick N after step b and sends at most that many bytes in tick N, in priority order. A capacity of 0 (radio `OFF`, `RX_ONLY`, or `transmitter_off`) means it sends nothing. `SilTarget` does not re-arbitrate, trim, or drop outbound frames.
- **Comms enforces them**, as the owner of the rule, when it applies the record in tick N+1. It checks the record against its **own previous tick's** state (its tick N mode, which it remembers; this is its own history, not a read): it calls `transmit_draw_w(config, previous_transmitter_on, radio_traffic.sent_bytes)`, which already raises `ValueError` for bytes above the capacity or any bytes while the transmitter was off. It also rejects a non-zero `uplink_lost_count` when its receiver was on in the previous tick.
- **Amendment (#122).** The capacity is no longer a config constant: comms derives each tick's `transmit_capacity_bytes` from `CommsConfig.transmit_rate_bytes_per_s` and the tick length, with an integer carry, so it can differ by a byte from tick to tick. Comms therefore also remembers its previous tick's capacity and length and calls `transmit_draw_w(config, previous_transmitter_on, sent_bytes, capacity_bytes=previous_capacity, dt_us=previous_dt_us)`. The rule is unchanged: bytes are bounded by the capacity of the tick they were sent in. The per-byte draw is scaled by `PER_BYTE_DRAW_TICK_US / dt_us` (100 ms over that tick's length) so a byte costs the same energy at any tick length. At the default 100 ms tick both are exactly as before (capacity 120 every tick, factor 1).
- **On violation** the `ValueError` propagates out of `SilTarget.advance()` for tick N+1. A violation is a simulator bug (the flight computer or `SilTarget` broke the contract), never a scenario outcome, so it fails the run loudly. Nothing is clipped: clipping would silently hide transmit energy.

### 5. `transmitter_off` and other overrides

- `transmitter_off` stays exactly as ADR-0004 §5 and #44 define it: the merge downgrades `controls.radio.mode` from `RX_TX` to `RX_ONLY`. It does not touch `radio_traffic`; the two are independent fields set in the same merge.
- A fault injected in tick N acts in tick N (ADR-0004 §2): comms' tick N capacity is 0, so the flight computer suppresses every ACK/NACK and telemetry frame in tick N and sends no DATA. That is the ordinary capacity rule of #51 and #55, not a separate `SilTarget` path. The flight computer counts the suppressed frames and `SilTarget` only passes the count on (accepted by the maintainer in the review of #104; this supersedes #104's context sentence that `SilTarget` suppresses). The suppressed frames are counted in tick N's record and appear in `outbound_suppressed_count` in tick N+1. The receiver keeps working, so `uplink_lost_count` does not change.
- The frames sent in the tick **before** the fault were sent with the transmitter on; comms validates them against that tick's state, so they are accepted and charged in tick N even though the transmitter is now off (section 2).
- No fault writes `radio_traffic`. A future fault that corrupts or drops frames after transmission belongs to the RF channel, not to this record; the bytes were still transmitted and still cost energy.

### 6. Determinism and portability

- The record is built from integers counted in a fixed order inside `advance()`: no randomness, no wall-clock time (ADR-0003). Two runs with the same seed produce the same records.
- The draw uses only `+` and `*` on the existing `CommsConfig` floats (ADR-0006). Counts are Python `int`s.
- `RadioTraffic` is part of the recorded controls, so replaying a run's controls reproduces comms exactly.

### 7. HIL (ADR-0002)

- Nothing crosses `TestTarget`: `RadioTraffic` is internal to `SilTarget`, and the ground sees the same frames through `receive()` either way.
- On HIL the MCU firmware is the flight computer and counts its own traffic; nothing on the host needs this record. If a later HIL design keeps the Python subsystem models on the host and feeds them the MCU's outputs, a per-tick traffic report from the MCU is exactly what this record is, so the path ports.
- The two-tick energy delay is a property of the SIL model. Contract tests (run against every target) must not assert it; it is asserted only in SIL tests (#98). Scenarios assert margins and properties, never the tick in which a draw appears (#63).
- Telemetry (#54) carries none of these counters today, so the wire format is unaffected.

## Alternatives considered

- **A separate `step` argument (protocol change):** `Subsystem.step(dt_us, env, controls, traffic)` or a new `SubsystemStack.step` parameter. Same timing as the decision, but every subsystem, fake, and test changes to pass a value only comms reads. ADR-0004 already made the per-subsystem record inside `SpacecraftControls` the way to give one subsystem its own input. Rejected.
- **A per-tick record passed to comms alone, outside the stack** (`SilTarget` calls `comms.set_traffic(record)` before stepping). Same timing, but `SilTarget` needs a typed reference to `Comms` and a special case, and the input is not part of the recorded controls. ADR-0004 rejected setter calls for these reasons (implicit timing, hard to record and replay). Rejected.
- **Same-tick report after the flight computer** (`SilTarget` calls `comms.record_traffic(record)` after step e and republishes comms' snapshot). It gives a consistent single-tick snapshot and a one-tick energy delay. But it adds a second state change per tick after the snapshot was published at step b, so the flight computer (and telemetry at step e) read a comms snapshot that later changes within the same tick; it needs a republish API on `SubsystemStack` and the `SnapshotBoard` (today published only by the stack, right after a step); and it is a setter outside the recorded controls. The gain is one tick (100 ms) of energy timing. Rejected.
- **The flight computer writes the traffic into its own controls** (for example fields on `RadioControls`, set at step f). Same timing, but the flight computer does not know lost uplink (`SilTarget` does); it would mix reports of what happened into the record of what it decides, and during a `forced_reset` hold it produces no controls at all (ADR-0004 §9) while traffic still needs reporting. Rejected in favor of `SilTarget` filling the field in the merge.
- **Comms reads the flight computer's output** (through the `SnapshotBoard` or a new reader). Breaks comms' "reads nothing" property and couples a subsystem to the flight computer. Because the flight computer runs after comms, it would still see the previous tick, so it buys nothing over the decision. Rejected.
- **Change the order: step comms after the flight computer, or split comms into a before and after phase.** Changes ADR-0003/ADR-0004's step order. The flight computer would read the previous tick's capacity, so `transmitter_off` would no longer stop outbound frames in the tick it is injected, contradicting "faults act on the same tick". Rejected.

## Consequences

- #56, #59, and #60 are built against one decided interface: they produce per-tick numbers, and #98 wires them into `RadioTraffic` and comms.
- The `Subsystem` protocol, `SubsystemStack`, `SnapshotBoard`, `STEP_ORDER`, and `TestTarget` are unchanged. `SpacecraftControls` gains one field with a zero default, so existing tests are unaffected.
- Comms gains a small amount of state: the previous tick's transmitter and receiver state (for validation) and two running totals.
- The traffic is recorded with the controls each tick, so it is visible in run records and replay.
- Cost: comms' snapshot mixes the current tick's radio state with the previous tick's traffic, so `previous_tick_sent_bytes` may exceed the current capacity (it is bounded by the previous tick's). The field name, the docstrings, and `docs/spacecraft.md` say this plainly.
- The rename is a contract change to `CommsTruth` and `CommsReadings`; #98 makes it in one change with its tests. Telemetry (#54) does not carry the field.
- Cost: transmit energy reaches the battery two ticks after the frame, one tick later than other draws. It is conserved and documented, and tests assert it.
- #72's traffic stand-in charged the per-byte energy in the tick it assumed the bytes were sent; real traffic charges it two ticks later. The budget margins do not move measurably; #98 re-checks them anyway.

## Follow-ups

- #98 implements this: `RadioTraffic` and the `SpacecraftControls.radio_traffic` field; the rename of `CommsTruth`/`CommsReadings.sent_bytes` to `previous_tick_sent_bytes`; comms' validation, running totals, and draw; the `SilTarget` hand-over; the `CommsTruth` and `Comms` docstrings; the tests listed in #98 at the ticks stated in section 2; and `docs/architecture.md` §4.1 (controls table, step a, latency table) and `docs/spacecraft.md` (Communications) updated from "Proposed" to the implemented behavior.
- The flight computer's per-tick output to `SilTarget`, from `FlightComputer.step()` (#101), includes its downlink frames and the number of ACK/NACK and telemetry frames it suppressed (filled by #51 and #55).
- #104's context sentence saying `SilTarget` suppresses outbound frames under `transmitter_off` is superseded by section 5: the flight computer suppresses and counts them.
- The traffic-path wording of #98 and the one-line references in #56, #59, and #60 are aligned with this ADR (listed in the PR for #104).
- On acceptance, ADR-0004 is not edited; this ADR records the amendment.
