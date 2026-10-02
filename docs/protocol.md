# PocketSat Wire Protocol

Status: Phase 1. Implements ADR-0002. The [telemetry payload](#telemetry-payload) is
defined (#54); command IDs and the maximum payload size are **TBD** and will be filled in
with the subsystems that use them.

Python implementation: `pocketsat.frame` (frames) and `pocketsat.messages` (telemetry
payload). Shared test vectors: `tests/vectors/frames.json` and
`tests/vectors/telemetry.json`.

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
| `0x02` | TELEMETRY | spacecraft → ground | Fixed 26-byte [telemetry payload](#telemetry-payload) |
| `0x03` | ACK | spacecraft → ground | Acknowledged sequence and status (**TBD**) |

## Message types

Frames are the only thing that crosses the `TestTarget` boundary. Above it, components
use typed, immutable messages (`pocketsat.messages`, `pocketsat.targets.base`).

| Type | Module | Purpose |
|---|---|---|
| `Command` | `pocketsat.messages` | Ground-originated request: `command_id`, `payload`. Encoded into a COMMAND frame. |
| `Telemetry` | `pocketsat.messages` | Decoded TELEMETRY payload: reported values at wire resolution (`decode_telemetry`). |
| `FlightComputerTelemetryState` | `pocketsat.messages` | The flight computer's input to `encode_telemetry`: `uptime_ms`, `mode` (a `pocketsat.flight.Mode`), `boot_count` (#49, #47, via #55). |
| `Packet` | `pocketsat.messages` | A frame in transit plus optional link state (elevation, range, Doppler, SNR, loss probability, latency) attached by the RF channel. |
| `EnvironmentState` | `pocketsat.targets.base` | Per-tick environment inputs; delivered by `apply_environment()`. |
| `TargetFault` | `pocketsat.targets.base` | Fault type, parameters, duration; delivered by `inject()`. |

### Command IDs

**TBD.** Planned commands: `PING`, `SET_MODE`, `BEGIN_DOWNLINK`, `ENTER_SAFE_MODE`, `RESET`.

### Telemetry layout

See [Telemetry payload](#telemetry-payload).

## Telemetry payload

Defined in #54. Python: `encode_telemetry` and `decode_telemetry` in `pocketsat.messages`.
Shared vectors: `tests/vectors/telemetry.json`. This section and those vectors are the
contract the Phase 7 firmware must match.

The TELEMETRY payload is a fixed **26 bytes**, so a whole TELEMETRY frame is **36 bytes**
(26 + the 10-byte frame overhead). Link timestamps and simulated time are attached by the
framework outside the wire format.

**Reported values only.** Every sensor field comes from a subsystem's readings record
(`pocketsat.spacecraft.snapshots`, #76), never from a truth record (ADR-0004 §7, #71):
`encode_telemetry` accepts only `PowerReadings`, `ThermalReadings`, `AttitudeReadings`,
`PayloadReadings`, and `CommsReadings`, plus the flight computer's
`FlightComputerTelemetryState` (uptime, mode, boot count, supplied by #49 and #47 through
the telemetry scheduler #55). Passing a truth record is a mypy error and a `TypeError`.

### Layout and mapping

All fields are fixed-width big-endian integers with no padding; each sits at an offset
that is a multiple of its size. A C implementation writes them byte by byte (most
significant byte first) rather than relying on a packed struct.

| Offset | Field | Type | Source | Conversion | Unit | Range | Out of range |
|---|---|---|---|---|---|---|---|
| 0 | `uptime_ms` | uint32 | `FlightComputerTelemetryState.uptime_ms` (#49) | as is | ms | 0 .. 4 294 967 295 | wraps modulo 2³² (after 49.7 days) |
| 4 | `boot_count` | uint16 | `FlightComputerTelemetryState.boot_count` (#49) | as is | boots | 0 .. 65 535 | saturates at 65 535 |
| 6 | `flags` | uint16 | readings flags, see [Flag bits](#flag-bits) | bit field | — | see the bit table | reserved bits sent as 0 |
| 8 | `mode` | uint8 | `FlightComputerTelemetryState.mode` (#47) | `Mode` value, see [Mode](#mode) | — | 0 .. 5 | cannot occur: only a `Mode` is accepted |
| 9 | `radio_mode` | uint8 | `CommsReadings.radio_mode` | see [Radio mode](#radio-mode) | — | 0 .. 2 | — |
| 10 | `attitude_state` | uint8 | `AttitudeReadings.state` | see [Attitude state](#attitude-state) | — | 0 .. 2 | — |
| 11 | `payload_state` | uint8 | `PayloadReadings.state` | see [Payload state](#payload-state) | — | 0 .. 2 | — |
| 12 | `bus_mv` | uint16 | `PowerReadings.bus_v` | `bus_v` × 1000 | mV | 0 .. 65 535 (0 .. 65.535 V) | saturates at 0 and 65 535 |
| 14 | `soc_permille` | uint16 | `PowerReadings.soc` (power's SOC estimate, #36) | `soc` × 1000 | 0.1 % | 0 .. 1000 (0 .. 100.0 %) | saturates at 0 and 1000 |
| 16 | `battery_centi_c` | int16 | `ThermalReadings.battery_c` | `battery_c` × 100 | 0.01 °C | −32 768 .. 32 767 (−327.68 .. 327.67 °C) | saturates at both limits |
| 18 | `electronics_centi_c` | int16 | `ThermalReadings.electronics_c` | `electronics_c` × 100 | 0.01 °C | −32 768 .. 32 767 (−327.68 .. 327.67 °C) | saturates at both limits |
| 20 | `buffered_bytes` | uint32 | `PayloadReadings.buffered_bytes` | as is | bytes | 0 .. 4 294 967 295 | saturates at 4 294 967 295 |
| 24 | `pointing_error_centi_deg` | uint16 | `AttitudeReadings.pointing_error_deg` | `pointing_error_deg` × 100 | 0.01° | 0 .. 18 000 (0 .. 180.00°) | saturates at 0 and 18 000 |

Example, the `nominal_no_flags` vector (NOMINAL, sequence 1, 7.400 V, SOC 85.0 %,
18.25 °C and 24.50 °C, 4096 bytes buffered, 3.50° pointing error, no flags):

```
a5 5a 01 02 00 01 00 1a | 00 01 e2 40 | 00 03 | 00 00 | 01 | 02 | 02 | 02 | 1c e8 | 03 52 | 07 21 | 09 92 | 00 00 10 00 | 01 5e | b3 78
frame header (len 26)     uptime_ms     boot    flags  mode radio att pay  bus_mv  soc     batt    elec    buffered      point   crc
```

### Encoding rules

**Physical values** (`bus_mv`, `soc_permille`, the temperatures, `pointing_error_centi_deg`)
are converted by `pocketsat.messages.quantize`, using only exactly rounded IEEE 754
operations (ADR-0006), so the result is bit-identical on every platform and in C:

1. `q = value × scale`, one IEEE 754 **double** multiplication.
2. Round `q` to the nearest integer, **halves away from zero**: `n = floor(|q|)`, plus 1 if
   `|q| − n ≥ 0.5`, with the sign of `q`. In C this is `lround(q)`.
3. **Saturate**: clamp `n` to the field's range.

The rule rounds the double product, not the decimal value a person would write:
2.675 °C gives `q = 267.5` and encodes as 268, while 1.005 °C gives
`q = 100.49999999999999` and encodes as 100. Firmware must compute step 1 in IEEE double
precision (software double on an MCU without a double-precision FPU); single precision
can differ at rounding boundaries, and the vectors include such cases.

**Saturation.** A value beyond a field's range is sent as the nearest limit, so a decoded
value at a limit means "at or beyond the limit". There is no separate out-of-range
indicator.

**Non-finite values.** `+inf` saturates to the field maximum and `−inf` to the minimum.
NaN has no encoding: `encode_telemetry` raises `ValueError` rather than inventing a value.
Readings never legitimately contain NaN, so it indicates a model fault.

**Counters.** `uptime_ms` is sent modulo 2³² (it wraps like a C `uint32_t` millisecond
counter); `boot_count` and `buffered_bytes` saturate at their maxima (a wrapped boot count
would look like a fresh spacecraft). `FlightComputerTelemetryState` rejects negative
`uptime_ms` and `boot_count`.

**Decoding.** `decode_telemetry` divides each physical field by its scale (one IEEE 754
division), so decoded values carry the wire resolution (1 mV, 0.1 %, 0.01 °C, 0.01°).
Unknown flag bits are ignored (cleared). A payload that is not exactly 26 bytes, or a
`mode`, `radio_mode`, `attitude_state`, or `payload_state` value not listed below, raises
`TelemetryDecodeError`. Re-encoding a decoded payload reproduces the same bytes.

### State fields

State fields are enumerations with fixed numeric values; they do not use flag bits.
Values are never reassigned.

#### Mode

The flight computer's `Mode` enum is defined by #47 (`pocketsat.flight`, docs/spacecraft-modes.md),
and its values are the wire values: `FlightComputerTelemetryState.mode` and the decoded
`Telemetry.mode` are `Mode` members, and the codec sends `mode.value`, so there is no
second table to keep in step (#101). A test checks every member against this table.

| Value | Mode |
|---|---|
| 0 | `BOOT` |
| 1 | `NOMINAL` |
| 2 | `SCIENCE` |
| 3 | `DOWNLINK` |
| 4 | `SAFE` |
| 5 | `FAULT` |

#### Radio mode

`CommsReadings.radio_mode` (`RadioMode`, ADR-0004 §10).

| Value | Radio mode |
|---|---|
| 0 | `OFF` |
| 1 | `RX_ONLY` |
| 2 | `RX_TX` |

#### Attitude state

`AttitudeReadings.state` (`AttitudeState`, #41).

| Value | Attitude state |
|---|---|
| 0 | `TUMBLING` |
| 1 | `DETUMBLING` |
| 2 | `STABILIZED` |

#### Payload state

`PayloadReadings.state` (`PayloadState`, #43).

| Value | Payload state |
|---|---|
| 0 | `OFF` |
| 1 | `IDLE` |
| 2 | `ACQUIRING` |

### Flag bits

`flags` (uint16) carries the readings flags: the reported-value flags the flight computer
acts on (ADR-0004, #71), never flags computed from true values. Bits are grouped by
subsystem so each group can grow independently. The single source of truth is
`pocketsat.messages.TelemetryFlags`, whose member names are the readings flag names
(`READINGS_FLAGS`, #76); a test checks this table against it.

| Bit | Mask | Group | Flag |
|---|---|---|---|
| 0 | `0x0001` | Power | `low_battery` |
| 1 | `0x0002` | Power | `critical_battery` |
| 2 | `0x0004` | Power | reserved |
| 3 | `0x0008` | Power | reserved |
| 4 | `0x0010` | Thermal | `over_temp` |
| 5 | `0x0020` | Thermal | `under_temp` |
| 6 | `0x0040` | Thermal | reserved |
| 7 | `0x0080` | Thermal | reserved |
| 8 | `0x0100` | Attitude | reserved |
| 9 | `0x0200` | Attitude | reserved |
| 10 | `0x0400` | Attitude | reserved |
| 11 | `0x0800` | Attitude | reserved |
| 12 | `0x1000` | Payload and comms | reserved |
| 13 | `0x2000` | Payload and comms | reserved |
| 14 | `0x4000` | Payload and comms | reserved |
| 15 | `0x8000` | Payload and comms | reserved |

**Wire-contract rules:**

- A bit is never reassigned.
- A new flag takes a reserved bit in its subsystem's group. Reserved bits are sent as 0
  and receivers ignore unknown bits, so this needs no protocol version change.
- A removed flag leaves its bit reserved; the bit is never reused.
- Reassigning or reusing a bit requires a protocol version change (the frame Version
  byte) and an ADR.
