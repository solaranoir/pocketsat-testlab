# PocketSat Wire Protocol

Status: Skeleton (Phase 0). Implements ADR-0002. Command IDs, telemetry layout, and
the maximum payload size are **TBD** and will be filled in with the subsystems that use them.

Python implementation: `pocketsat.frame`. Shared test vectors: `tests/vectors/frames.json`.

## Frame layout

All multi-byte fields are big-endian.

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0 | Sync | 2 | `0xA5 0x5A` |
| 2 | Version | 1 | Protocol version, currently `1` |
| 3 | Type | 1 | See [Frame types](#frame-types) |
| 4 | Sequence | 2 | Per-direction counter, wraps `0xFFFF` → `0x0000` |
| 6 | Length | 2 | Payload length N in bytes |
| 8 | Payload | N | Type-specific |
| 8 + N | CRC | 2 | CRC-16/CCITT-FALSE over Version through Payload (offsets 2 .. 7 + N) |

Total frame size is `10 + N` bytes. The length field limits N to 65535; a smaller
protocol maximum is **TBD** (MCU buffer sizing, Phase 7a).

Example: COMMAND, sequence 1, payload `01 02`:

```
a5 5a | 01 | 01 | 00 01 | 00 02 | 01 02 | 19 ce
sync    ver  type  seq    len     payload  crc
```

## CRC

CRC-16/CCITT-FALSE: width 16, poly `0x1021`, init `0xFFFF`, no input or output
reflection, no final XOR. Check value: CRC of ASCII `"123456789"` is `0x29B1`.

## Decoding and errors

A decoder takes exactly one complete frame and checks, in order:

| Check | Error (`pocketsat.frame`) |
|---|---|
| First two bytes are `0xA55A` | `FrameSyncError` |
| At least 10 bytes, and size equals `10 + Length` | `FrameLengthError` |
| Received CRC equals computed CRC | `FrameCrcError` |
| Type is a known code | `FrameTypeError` |

All four subclass `FrameError` (a `ValueError`). Frames are never silently repaired.
Stream resynchronization (finding frames in a serial byte stream) is **TBD** for the
HIL bridge.

## Frame types

| Code | Type | Direction | Payload |
|---|---|---|---|
| `0x01` | COMMAND | ground → spacecraft | Command ID and arguments (**TBD**) |
| `0x02` | TELEMETRY | spacecraft → ground | Uptime counter, then telemetry fields (**TBD**) |
| `0x03` | ACK | spacecraft → ground | Acknowledged sequence and status (**TBD**) |

## Message types

Frames are the only thing that crosses the `TestTarget` boundary. Above it, components
use typed, immutable messages (`pocketsat.messages`, `pocketsat.targets.base`).

| Type | Module | Purpose |
|---|---|---|
| `Command` | `pocketsat.messages` | Ground-originated request: `command_id`, `payload`. Encoded into a COMMAND frame. |
| `Telemetry` | `pocketsat.messages` | Decoded spacecraft state: `uptime_ms`, remaining `payload`. |
| `Packet` | `pocketsat.messages` | A frame in transit plus optional link state (elevation, range, Doppler, SNR, loss probability, latency) attached by the RF channel. |
| `EnvironmentState` | `pocketsat.targets.base` | Per-tick environment inputs; delivered by `apply_environment()`. |
| `TargetFault` | `pocketsat.targets.base` | Fault type, parameters, duration; delivered by `inject()`. |

### Command IDs

**TBD.** Planned commands: `PING`, `SET_MODE`, `BEGIN_DOWNLINK`, `ENTER_SAFE_MODE`, `RESET`.

### Telemetry layout

**TBD.** The payload starts with the spacecraft uptime counter. Link timestamps and
simulated time are attached by the framework outside the wire format.
