# ADR-0002: TestTarget abstraction, wire format, environment feed, and fault delivery

- **Status:** Proposed
- **Phase:** 0
- **Amends:** `docs/architecture.md` sections 5 and 6

## Context

The core promise of PocketSat Test Lab is that one scenario runs unchanged against a Python simulation (SIL) or a microcontroller running flight software (HIL). That promise lives or dies at a single boundary, the `TestTarget`. Four design questions have to be settled before any subsystem code is written:

1. At what level of abstraction does the boundary sit: commands and telemetry objects, or raw bytes?
2. What is the wire format, given it must be implementable in both Python and C/C++ on a small MCU?
3. How does an MCU receive battery, temperature, and other sensor inputs, since it has no physics of its own?
4. How are target-side faults (MCU reset, sensor freeze, radio failure) delivered without reaching into internals, and what happens when a target cannot support a fault?

## Decision

### 1. The boundary is raw frames (bytes in, bytes out)

`TestTarget` exchanges encoded byte frames, not Python objects. The ground station encodes commands into frames; the RF channel operates on `Packet` objects that wrap a frame plus link-state attributes; the target sees only bytes.

```python
@dataclass(frozen=True)
class TargetCapabilities:
    deterministic: bool              # SIL True, HIL False
    real_time: bool                  # SIL False, HIL True
    supported_faults: frozenset[str]

class TestTarget(Protocol):
    capabilities: TargetCapabilities

    def connect(self) -> None: ...
    def reset(self, seed: int) -> None: ...
    def send(self, frame: bytes) -> None: ...               # uplink into target
    def receive(self) -> list[bytes]: ...                   # downlink frames produced since last call
    def apply_environment(self, env: EnvironmentState) -> None: ...
    def inject(self, fault: TargetFault) -> None: ...
    def advance(self, dt: float) -> None: ...
    def close(self) -> None: ...
```

Changes from the draft in `architecture.md`:

- `send`/`receive` carry `bytes`, not `Packet`.
- `receive()` has **no timeout**. It drains frames produced so far. Simulated time makes a timeout meaningless in SIL; in HIL, `advance(dt)` already waits real time and drains the serial buffer.
- Added `apply_environment`, `inject`, and `capabilities`.

### 2. Wire format

Big-endian, fixed header, CRC-protected. Simple to parse with `struct` in Python and with a small state machine in C.

| Field | Size | Notes |
|---|---|---|
| Sync | 2 bytes | `0xA5 0x5A` |
| Version | 1 byte | Protocol version, starts at 1 |
| Type | 1 byte | `0x01` COMMAND, `0x02` TELEMETRY, `0x03` ACK |
| Sequence | 2 bytes | Per-direction counter, wraps |
| Length | 2 bytes | Payload length in bytes |
| Payload | 0..N bytes | Type-specific; max size fixed in protocol doc |
| CRC | 2 bytes | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over Version through Payload |

Consequences for testing: packet corruption is implemented as byte-level mutation in the RF channel, and the CRC makes corruption detectable by real code paths. The full field-level spec (command IDs, telemetry layout) goes in `docs/protocol.md`, written alongside the `TestTarget` implementation.

Spacecraft time in telemetry is an uptime counter in the payload, not part of the header. Link timestamps and simulated time are attached by the framework outside the wire format.

### 3. Environment is a separate model, delivered through `apply_environment`

The orchestrator side owns an `Environment` model producing an `EnvironmentState` each tick (for example: sunlit or eclipse, ambient thermal input, injected sensor noise or bias, battery-condition overrides). The orchestrator calls `target.apply_environment(state)` each tick.

- **SIL:** the spacecraft subsystems read `EnvironmentState` directly.
- **HIL:** the HIL bridge serializes `EnvironmentState` into sensor-input frames and sends them to the MCU on a dedicated channel.

This keeps `TestTarget` as one interface, keeps physics models out of the firmware, and lets the same environment drive both targets.

### 4. Target faults go through `inject`, with declared capabilities

`inject(fault)` delivers a `TargetFault` (type, parameters, duration). Each target declares the fault types it supports in `capabilities.supported_faults`.

- **SIL:** faults are handled by the simulation (for example a frozen sensor value, a forced reset, a transmitter-off flag).
- **HIL:** faults are delivered on a **test-control channel** separate from the RF-path command stream. An MCU reset uses a physical reset line or a GPIO toggle where available; firmware-level faults use test-only control messages compiled into a test build.
- **Unsupported faults:** if a scenario requests a fault the target does not list, the orchestrator **fails the run up front** with an explicit error (not silently skipping it). Scenarios may mark a fault as `optional` to skip it per target.

## Alternatives considered

- **Command/telemetry objects at the boundary.** Easier to write for SIL, but hides the wire format, makes byte-level corruption awkward, and forces HIL to hide a serialization layer inside the target. Rejected.
- **Environment inside `TestTarget` only.** Would make the HIL bridge own sensor modeling, duplicating physics in two places. Rejected.
- **Faults as ad hoc target methods** (`target.reset_mcu()`, `target.freeze_sensor()`). Couples scenarios to specific targets and doesn't scale. Rejected.
- **JSON or text framing on the wire.** Simpler to read, but wasteful for the MCU and unrepresentative of real amateur-satellite link protocols. Rejected.

## Consequences

- SIL and HIL share everything above the boundary, including fault scheduling, ground station, and RF corruption.
- The HIL firmware must include a test-control channel and a sensor-input channel, including a physical reset line for MCU reset. This is extra firmware work in Phase 7, and the scope is **confirmed**: it is what makes HIL repeatable, so it is not trimmed.
- The roadmap's two-weekend estimate for Phase 7 no longer holds. Phase 7 is re-planned as two milestones: **7a** (minimal firmware: framing and CRC, commands, telemetry, state machine, watchdog, safe mode, reset recovery) and **7b** (sensor-input channel, test-control channel, reset line, capability declaration). The v1 schedule absorbs this by treating 7b as the first item cut if the 10-week plan slips, with HIL then running nominal scenarios only.
- The CRC and framing code is written twice (Python and C) and must be verified against shared test vectors. This is a cost and a useful test in its own right.
- Target capability declarations make SIL-only and HIL-only scenarios explicit instead of accidental.

## Follow-ups

- Update `docs/architecture.md` sections 5 and 6 to match the interface above.
- Update issue #6 (TestTarget protocol and core message types) to add `apply_environment`, `inject`, `capabilities`, `EnvironmentState`, `TargetFault`, and frame encode/decode with CRC.
- Add shared CRC/frame test vectors, used by both Python tests and (later) firmware tests.
- Create Phase 7a and Phase 7b milestones in place of a single Phase 7 milestone, and note the revised estimate in the roadmap.
- ADR-0003 settles tick size and how `advance(dt)` is scheduled relative to `apply_environment`.
