# ADR-0010: Ground station architecture and interfaces

- **Status:** Proposed
- **Phase:** 2
- **Amends / Supersedes:** Amends ADR-0003 (the `Packet` time field, §7). Related: ADR-0002 (TestTarget boundary), ADR-0005 (configuration by constructor injection), ADR-0006 (portable arithmetic), ADR-0009 (protocol version 1). ADR-0011 (#159) decides the pass geometry model; this ADR decides only where geometry lives and who consumes it. Issue: #151.

## Context

Phase 2 builds the ground station (`docs/roadmap.md`, milestone: send commands and receive decoded telemetry through a ground-station interface). `docs/architecture.md` places it above the `TestTarget` boundary as shared infrastructure: it owns pass state, radio control, command uplink, packet decoding, station identity and console output, and does **not** own orbital truth or fault scheduling (§4). Its uplink and downlink pass through the RF channel model, which does not exist until Phase 3 (§3), and the run loop that drives it all belongs to the orchestrator, which is Phase 5 (§8).

Twenty Phase 2 tickets (#152 to #170) build on one set of interfaces. They need these settled first:

- the station's public API and how it fits the ten-step tick order (§8, ADR-0003 §2);
- its configuration and the events it emits;
- a stand-in for the RF channel that Phase 3 replaces without touching the station;
- where pass geometry is computed, and where the Phase 2 run loop lives;
- two contract questions that are expensive to change later: the float `Packet.latency_s`, which conflicts with ADR-0003's integer microseconds, and whether the station's callsign goes on the wire, which would change protocol version 1 (ADR-0009) and the Phase 7a firmware contract.

Constraints carried over: determinism and integer-microsecond simulated time (ADR-0003); portable arithmetic in simulation code (ADR-0006); typed immutable messages between components (AGENTS.md rule 4); the orchestrator depends on `TestTarget` only (AGENTS.md rule 1); the same scenario runs on SIL and HIL, so nothing above the boundary may depend on which target it has (ADR-0002).

## Decision

### 1. The station is a pure component: messages in, messages out

`GroundStation` (in `pocketsat.groundstation`) never calls the target, the link, or the clock. Each tick it receives simulated time, pass geometry and downlink packets, and returns uplink packets and events. The run loop (§6) moves packets between the station, the link (§5) and the target. This keeps the station unit-testable without a target, identical under SIL and HIL, and indifferent to whether the link is ideal (Phase 2) or modelled (Phase 3).

### 2. Public API: two calls per tick, matching the step order

The architecture's tick order has the station act twice: it encodes uplink at step 3 (before `advance`) and decodes downlink at steps 8 and 9 (after `receive`). A single per-tick call would move one of the two across `advance` and add a tick of latency to either commands or decoded telemetry, so the station has one entry point for each half:

```python
class GroundStation:
    def __init__(self, config: StationConfig = DEFAULT_STATION_CONFIG) -> None: ...
    def reset(self, rngs: RngFactory) -> None: ...

    def submit(self, command: Command) -> CommandRef: ...                  # step 3: queue
    def begin_tick(self, now_us: int, geometry: PassGeometry | None) -> Uplink: ...   # steps 3-4
    def end_tick(self, now_us: int, downlink: Sequence[Packet]) -> tuple[StationEvent, ...]:  # steps 8-9

    @property
    def telemetry(self) -> TelemetryStore: ...                             # read-only view (#157)
```

- `submit()` queues a `Command` and returns a `CommandRef`, a ground-local integer that identifies the command across retries in every later event. Nothing is encoded until `begin_tick`.
- `begin_tick(now_us, geometry)` runs at the start of the tick (`now_us` is the tick's start). In order: update pass state from `geometry` (AOS/LOS, #161), expire timed-out commands and schedule their retries (#155), then, if the radio may transmit (#164), encode queued commands into COMMAND frames (#154). It returns `Uplink(packets, events)`.
- `end_tick(now_us, downlink)` runs after the target has advanced (`now_us` is the tick's end, the time `SilTarget` gives the flight computer). It decodes each packet in arrival order and returns the events, in a fixed order: per packet, frame-level events first (rejection, sequence gap), then the decoded content (telemetry, ACK/NACK, DATA). It updates the telemetry store.
- `reset(rngs)` returns the station to its configured initial state, with the same shape as the subsystem `reset(rng)` (ADR-0005). The station draws no random numbers in Phase 2. Phase 4's ground-station faults (Doppler tracking error, station failure) request streams named `groundstation.*`.
- Everything else in the package is internal. The public names are `GroundStation`, `StationConfig`, `DEFAULT_STATION_CONFIG`, `CommandRef`, `Uplink`, `TelemetryStore`, the event types (§4), and the link types (§5).

**Command retries are a policy per command, not a global setting.** A retry after a lost ACK re-executes the command on the spacecraft. That is safe for `PING`, `SET_MODE`, `ENTER_SAFE_MODE` and `BEGIN_DOWNLINK`, which ADR-0009 §4 makes idempotent. It is **not** safe for `RESET`: RESET is accepted in every mode, BOOT included, and restarts the boot (`docs/spacecraft-modes.md`), so a retried RESET whose first ACK was lost reboots the spacecraft a second time and increments `boot_count` again. `RESET` is therefore never retried automatically; on timeout it is reported as `CommandFailed` and the scenario decides. Each retry is a new COMMAND frame with a new uplink sequence number (the sequence is per frame, ADR-0009 §4, and correlation is by sequence), linked to the original by its `CommandRef`.

**Uplink sequence.** The station keeps the ground-to-spacecraft sequence counter, which starts at 0 on `reset()` and wraps `0xFFFF` → `0x0000` like the downlink one (`docs/protocol.md`).

### 3. Station configuration: a frozen record injected at construction

Like the spacecraft settings (ADR-0005), the station is built with a frozen, validated `StationConfig`, and `reset()` returns to it. Scenarios change single values with `dataclasses.replace()`. Fields, named under the units convention (`docs/architecture.md` §4.1):

| Field | Type | Meaning |
|---|---|---|
| `callsign` | `str` | Station identity, uppercase letters, digits and `/` (validated) |
| `latitude_deg`, `longitude_deg` | `float` | Station location |
| `altitude_km` | `float` | Station altitude above the reference ellipsoid |
| `elevation_mask_deg` | `float` | Minimum elevation at which the station operates (AOS/LOS threshold, #161) |
| `uplink_frequency_hz`, `downlink_frequency_hz` | `int` | Nominal carrier frequencies before Doppler correction (#162) |
| `command_timeout_us` | `int` | Time from a command's transmission to its timeout, simulated µs |
| `command_retry_count` | `int` | Retries after the first attempt, for commands whose policy allows them (§2) |
| `identification_interval_us` | `int` | Station identification interval while transmitting (§8, #164) |

The default values are chosen in #152 and justified there; this ADR fixes the fields, their units and how they are supplied. The command timeout must exceed the longest legitimate command-to-ACK delay: the same tick at the default tick length, longer for a deferred RESET at short ticks (ADR-0008 §6a). #155 states the bound and #152's validation enforces it against the configured tick.

### 4. Events: one immutable stream that the console and the assertions both read

Every observable thing the station does is an event: a frozen dataclass carrying `time_us: int`. `StationEvent` is the union of the event types. Events are returned from `begin_tick` and `end_tick`; the station keeps no event history of its own (the run loop collects them, §6).

| Event | Emitted by | Carries |
|---|---|---|
| `Aos`, `Los` | pass state (#161) | pass number, azimuth and elevation; `Los` also the pass's maximum elevation |
| `CommandSent` | uplink (#154) | `CommandRef`, command ID, uplink sequence, attempt number |
| `CommandAcked`, `CommandNacked` | ACK correlation (#155) | `CommandRef`, sequence; `CommandNacked` also the reason (`DecodeReason` or `RejectReason`) |
| `CommandTimedOut` | ACK correlation (#155) | `CommandRef`, attempt number, whether a retry follows |
| `CommandFailed` | ACK correlation (#155) | `CommandRef`, why (retries exhausted, not retryable) |
| `UnmatchedAck` | ACK correlation (#155) | the ACK's sequence and command ID (unknown or already answered) |
| `TelemetryReceived` | decoding (#156) | the decoded `Telemetry`, downlink sequence |
| `DataChunkReceived` | DATA (#158) | chunk ID, whether the content matches `chunk_content` |
| `FrameRejected` | decoding (#156) | the reason (sync, length, CRC, type, payload decode, wrong direction) and the raw bytes |
| `SequenceGap` | decoding (#156) | expected and received downlink sequence, frames missing |
| `SpacecraftRebooted` | decoding (#156) | inferred from a downlink sequence restart paired with a higher `boot_count` (ADR-0009 §7) |
| `RadioStateChanged`, `StationIdentified` | radio control (#164) | the new state; the callsign |

- **The console (#165)** is a pure function from an event to one line of text with a simulated timestamp. It reads nothing from the station but events, and writes to an injected text stream, never with a wall-clock time.
- **Assertions** (the Phase 2 story tests now, the Phase 5 DSL later) query the collected event stream and the telemetry store. Adding a scenario assertion should never need a new station accessor.
- A frame that decodes but travels the wrong way (a COMMAND frame on the downlink, as `EchoTarget` produces) is `FrameRejected` with reason "wrong direction" and its bytes. Story #144's echo test asserts on that event.
- Link-level drops (§5) are counted by the link, not the station: the station cannot observe a frame it never received.

### 5. The link: a `Link` protocol, an ideal implementation now, the RF channel in Phase 3

The run loop moves frames through a `Link`. The station never sees the link, so replacing it changes nothing in the station.

```python
class Link(Protocol):
    def reset(self, rngs: RngFactory) -> None: ...
    def carry_uplink(self, packets: Sequence[Packet], now_us: int,
                     geometry: PassGeometry | None) -> list[bytes]: ...      # step 4: frames due at the target
    def carry_downlink(self, frames: Sequence[bytes], now_us: int,
                       geometry: PassGeometry | None) -> list[Packet]: ...   # step 8: packets due at the station
```

- The link does not call the target. It returns the frames due now; the run loop calls `send()`. A link with latency (Phase 3) queues packets internally and returns them on a later call, so the interface already accommodates delay.
- **`IdealLink`** (#153) delivers every packet in the same call, unchanged, in order. It fills `elevation_deg` and `range_km` from `geometry` when given, and `doppler_hz` and `snr_db` from the Phase 2 indicator functions (#162, #163). With no geometry, every attribute is `None`.
- **Visibility is physics and belongs to the link.** The link drops frames in both directions while the spacecraft is below the horizon (`geometry.elevation_deg < 0`) and counts them. With no geometry it treats the spacecraft as always visible. The **elevation mask** is the station's operating policy (§3, #164): the station chooses not to transmit below the mask, while the link decides whether a frame physically arrives. The two thresholds stay separate.
- Module: `pocketsat/link.py`, beside `frame.py` and `messages.py`, holding `Link` and `IdealLink`. Phase 3's `pocketsat.rf` implements `Link` without depending on `pocketsat.groundstation`, and the station does not depend on either.

### 6. Pass geometry lives on the orchestrator side, as its own message

- **The geometry model is orchestrator-side world truth, beside the environment model,** in `pocketsat.environment`, stepped at step 2 from the run loop's `SimClock`. The model itself is ADR-0011's decision (#159).
- **It produces a separate `PassGeometry` record, not new fields on `EnvironmentState`.** `EnvironmentState` is what the target sees: it crosses the `TestTarget` boundary, and in HIL the bridge serializes it onto the sensor-input channel (ADR-0002). The spacecraft does not need station-relative geometry, so adding it there would grow the HIL sensor contract for nothing and break the rule that the target knows nothing of the ground station.
- `PassGeometry` holds at least `azimuth_deg`, `elevation_deg`, `range_km` and `range_rate_km_per_s`. ADR-0011 may add fields.
- **One record per tick, given to both consumers.** The link uses it as physical truth (visibility, link attributes). The station uses it as its pass prediction, as a real station uses a predicted orbit. In Phase 2 they are the same object. Phase 4's Doppler tracking error gives the station a perturbed copy, with no API change.

### 7. The Phase 2 run loop lives in `pocketsat.orchestrator`

The run loop (#167) is placed in `pocketsat/orchestrator/loop.py`, as the first piece of the orchestrator, not inside the ground station:

- It runs the ten steps of `docs/architecture.md` §8 in order. Step 1 (scheduled faults) and step 10 (assertions) are documented no-ops until Phases 4 and 5; step 3 issues the commands its caller scheduled.
- It owns the `SimClock` and the `RngFactory`, depends on `TestTarget` only (AGENTS.md rule 1), and returns the collected events and the telemetry store.
- Phase 5 adds scenario loading, the fault schedule and assertions to this loop rather than replacing it.
- The CLI (#166) and the milestone scenario (#168) call it. The Phase 1 nominal harness (`tests/sil/_nominal_scenario.py`) moves onto it only if the golden captures stay byte-identical (#167).

This brings a small part of Phase 5 forward, as Phase 0 brought ADR-0003 forward, so that Phase 2 is not built on a throwaway loop.

### 8. Decision A: replace `Packet.latency_s` with integer microseconds (maintainer decision)

**Decision (maintainer, 2026-10-13): Option A1.** Rename the field to `latency_us: int | None` in #152, before anything reads it. This amends ADR-0003, whose integer-microsecond rule covers every simulated time.

`Packet.latency_s: float | None` was written in Phase 0, before ADR-0003. It has no callers today; the only reference is its own declaration. Phase 3's RF channel is the first component to set it, and it must schedule delivery on integer tick boundaries.

| | A1: `latency_us: int` (chosen) | A2: keep `latency_s: float`, convert at use | A3: remove latency, add `sent_us` and `delivered_us` timestamps |
|---|---|---|---|
| Fits ADR-0003 | Yes: one time type throughout | No: a float time in the model, converted wherever it is used | Yes |
| Determinism risk | None: integer arithmetic | Rounding at each conversion. Two code paths that round differently give different delivery ticks | None |
| Cost now | Rename one field and its docstring; no callers to change | None | Change `Packet`'s shape and decide who stamps each time |
| Cost later | None | A conversion rule documented and tested at every use (RF channel, faults, reports) | The RF channel and station both stamp times; more to keep consistent |
| What it expresses | The latency the channel applied to this packet | Same, in a unit the simulation does not use | When the packet left and arrived; latency is the difference |
| Naming convention test | Passes (`_us` is a listed suffix) | Passes | Passes |

**Trade-off.** A1 is cheapest now and keeps one rule for time. Its only cost is Phase 0 history: the field name changes once, while nothing reads it. A2 is free today but is the kind of float time ADR-0003 was written to exclude, and every Phase 3 and Phase 4 use pays for the conversion. A3 is the most informative (a report can show both times), but it designs Phase 3's packet timestamps before Phase 3 has a model; it can still be added on top of A1 then. The wire protocol is unaffected under every option: `Packet` exists only above the target boundary.

### 9. Decision B: station identity stays off the wire (maintainer decision)

**Decision (maintainer, 2026-10-13): Option B1.** The callsign lives in `StationConfig`, the console, the run record and a `StationIdentified` event emitted at `identification_interval_us` while transmitting. Protocol version 1 is unchanged.

In amateur practice a station identifies by transmitting its callsign at set intervals; AX.25-based amateur satellites also put callsigns in every frame header. PocketSat's frames carry no addresses (ADR-0002, ADR-0009), and the protocol is the Phase 7a firmware contract.

| | B1: configuration, console and run record (chosen) | B2: callsign in every COMMAND payload | B3: a new IDENT frame type, sent at the interval | B4: callsign in the frame header for all frames |
|---|---|---|---|---|
| Wire change | None | COMMAND layout changes, so protocol version 2 (ADR-0009 §8) | A fifth frame type (ground → spacecraft); a version-1 receiver rejects it as an unknown type | Frame layout changes, so protocol version 2, and ADR-0002 is superseded |
| Firmware (Phase 7a) | Nothing new | Parse and validate a callsign field; new vectors | Accept and count IDENT frames; new vectors | Every frame encoder and decoder changes; every vector regenerates |
| Golden captures | Unchanged | Unchanged (uplink is not captured) | Unchanged | Regenerated (every downlink frame grows) |
| Overhead | None | About 6 bytes per command | One short frame per interval | About 6 to 12 bytes per frame, including every TELEMETRY and DATA frame |
| What it lets us test | Station-side identification timing and logging | Spacecraft-side station authorisation (a NACK for an unknown station); multiple stations | Identification as a real transmission that the RF channel can lose or corrupt | Full AX.25-style addressing; multiple spacecraft and stations |
| Realism | The identification is logged, not transmitted | Partial | Close to amateur practice | Closest to amateur practice |
| Scope fit (v1) | Fits | Authorisation is not on the v1 roadmap | Fits, at the cost of a new type | Out of scope: v1 has one station and one spacecraft |

**Trade-off.** B1 costs nothing and keeps the Phase 7a contract stable. What it gives up is testing the spacecraft's response to identity, which v1 does not need: one station, one spacecraft, and CRC already guards against corrupted commands (ADR-0009 §3). B3 is the cheapest wire option if a scenario later needs identification to be a real transmission. It adds a frame type without touching existing layouts, so it can be added after Phase 2 without a version change, provided it lands before the 7a firmware is written. B2 and B4 change existing layouts, so each is a protocol version 2. They are worth doing only for station authorisation or multi-station work, which belong in Version 2. **Timing matters more than the choice:** any wire option is cheapest before the Phase 7a firmware exists and expensive after, so if B2 to B4 are wanted at all, decide them before 7a starts.

## Alternatives considered

- **One `step()` per tick for the station.** Simpler to call, but either commands wait a tick (encoded after `advance`) or decoded telemetry is a tick late for step-10 assertions (decoded before the next `advance`). Rejected for the two calls in §2, which follow §8 exactly.
- **The station owns the link and calls the target.** Fewer moving parts in Phase 2, but the station would depend on `TestTarget` and on the link, and Phase 3 would rewrite it. Rejected (§1).
- **The link calls `TestTarget.send()` itself** (`docs/architecture.md` §3 draws the arrow that way). It works, but the link then needs the target and the run loop loses control of step order. Rejected: the link returns the frames that are due, and the loop sends them. The architecture diagram keeps its meaning (the RF path carries the frames) and §8's steps are unchanged.
- **Geometry on `EnvironmentState`.** One message instead of two, but it crosses the target boundary and grows the HIL sensor-input contract (§6). Rejected.
- **Geometry computed inside the station.** Contradicts §4 of the architecture (the station does not own orbital truth) and leaves the link without the truth it needs for visibility. Rejected.
- **A visibility function injected into the link** (the wording of #153). It works for Phase 2, but Phase 3's link budget needs range and elevation too, and computing geometry once per tick for both consumers is cheaper. Rejected for passing `PassGeometry` (§5, §6).
- **The run loop in `pocketsat.groundstation`, or in the tests only.** In the station package it pulls fault scheduling and target control into a component whose architecture row excludes them. In the tests only, the CLI (#166) cannot use it. Rejected for `pocketsat.orchestrator` (§7).
- **A global retry count for every command.** Simpler, but it reboots the spacecraft twice when a RESET ACK is lost. Rejected for a per-command policy (§2).
- **Reuse the uplink sequence number on retry.** The spacecraft would see a duplicate sequence and the ground could not tell which attempt an ACK answers. Rejected: a new sequence per frame, correlated through `CommandRef`.

## Consequences

- The station is testable alone with hand-built packets, and the same station runs over the ideal link now and the RF channel in Phase 3, on SIL or HIL.
- Phase 3 implements `Link` and nothing else changes. Phase 4's ground-station faults have two hook points: the geometry the station is given, and a `groundstation.*` random stream.
- The event stream is the single source for the console, the story tests and the Phase 5 assertions, so a new observable is a new event type, not a new accessor.
- `pocketsat.orchestrator` exists from Phase 2. AGENTS.md says not to scaffold future phases; this is a deliberate exception, because Phase 2's milestone needs a run loop and a throwaway one would be rewritten in Phase 5.
- RESET is never retried automatically, so a lost RESET ACK surfaces as a failed command that the scenario must handle.
- `Packet` changes shape once in Phase 2 (A1). Under B1, the wire protocol and the Phase 7a contract are unchanged.
- The cost: two station calls per tick instead of one, and one more module (`pocketsat/link.py`) above the boundary.

## Open questions for the maintainer

1. **Decision A (§8):** decided 2026-10-13, A1 (`latency_us`).
2. **Decision B (§9):** decided 2026-10-13, B1 (off the wire). Revisit only if a scenario needs identification as a real transmission; B3 is then the route, before Phase 7a starts.
3. **The orchestrator brought forward (§7).** Recommended: accept. The alternative is a loop in the tests plus a CLI-only copy, both rewritten in Phase 5.
4. **RESET retries (§2).** Recommended: never automatic. The alternative is to retry RESET only after a downlink sequence restart and `boot_count` show the first attempt did not reboot. That is more automation, but it adds state for an edge case.
5. **Link visibility at elevation 0° (§5).** Recommended: the geometric horizon, with no refraction and no terrain. ADR-0011 may revise this if its model needs to.

## Follow-ups

- On acceptance: the maintainer sets the status to Accepted, and `docs/architecture.md` adds `pocketsat/link.py` and `PassGeometry` to §3, §5 and §11 and this ADR to §12 (#152).
- #152 implements §2 to §4 and, under A1, the `Packet` rename; #153 implements §5; #160 and #161 consume §6; #164 implements §9; #167 implements §7.
- #153's acceptance criterion "a visibility gate as an injected function" is superseded by §5 (geometry passed per call); update the issue on acceptance.
- #155: record the per-command retry policy and the timeout bound (§2, §3).
