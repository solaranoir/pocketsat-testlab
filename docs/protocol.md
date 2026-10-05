# PocketSat Wire Protocol

Status: Phase 1. Implements ADR-0002. The [telemetry payload](#telemetry-payload) (#54)
and the [command and ACK payloads](#commands) with their NACK reason codes (#52) are
defined; the maximum payload size is **TBD**.

Python implementation: `pocketsat.frame` (frames) and `pocketsat.messages` (telemetry,
command, and ACK payloads). Shared test vectors: `tests/vectors/frames.json`,
`tests/vectors/telemetry.json`, and `tests/vectors/commands.json`.

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

`pocketsat.frame` computes it with a 256-entry lookup table, one table lookup per byte
(#78); entry `n` is the register after shifting byte `n` through it bit by bit from zero.
The output is identical to the bit-by-bit definition, which the unit tests keep as a
reference. The table mirrors what the Phase 7 MCU firmware will do. Python's
`binascii.crc_hqx(data, 0xFFFF)` computes the same CRC in C, about 20 times faster than
the table; it is an equivalent drop-in if CRC cost ever matters again (the unit tests
already check the equivalence).

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
| `0x01` | COMMAND | ground → spacecraft | Command ID and arguments, see [COMMAND payload](#command-payload) |
| `0x02` | TELEMETRY | spacecraft → ground | Fixed 26-byte [telemetry payload](#telemetry-payload) |
| `0x03` | ACK | spacecraft → ground | Fixed 4-byte [ACK payload](#ack-payload), for both ACK and NACK |

## Message types

Frames are the only thing that crosses the `TestTarget` boundary. Above it, components
use typed, immutable messages (`pocketsat.messages`, `pocketsat.targets.base`).

| Type | Module | Purpose |
|---|---|---|
| `Command` | `pocketsat.messages` | Ground-originated request: `command_id`, argument bytes `payload`. Encoded into a COMMAND payload (`encode_command`). |
| `ParsedCommand`, `MalformedCommand` | `pocketsat.messages` | Result of `decode_command` on the spacecraft: a valid command (`command_id`, SET_MODE `target`), or the received ID and a `DecodeReason` for the NACK (#52). |
| `CommandAck` | `pocketsat.messages` | An ACK frame payload: the answered command's `sequence` and `command_id`, and the NACK `reason` (`None` for an ACK) (`encode_ack`, `decode_ack`). |
| `Telemetry` | `pocketsat.messages` | Decoded TELEMETRY payload: reported values at wire resolution (`decode_telemetry`). |
| `FlightComputerTelemetryState` | `pocketsat.messages` | The flight computer's input to `encode_telemetry`: `uptime_ms`, `mode` (a `pocketsat.flight.Mode`), `boot_count` (#49, #47, via #55). |
| `Packet` | `pocketsat.messages` | A frame in transit plus optional link state (elevation, range, Doppler, SNR, loss probability, latency) attached by the RF channel. |
| `EnvironmentState` | `pocketsat.targets.base` | Per-tick environment inputs; delivered by `apply_environment()`. |
| `TargetFault` | `pocketsat.targets.base` | Fault type, parameters, duration; delivered by `inject()`. |

### Command and ACK layouts

See [Commands](#commands).

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

## Commands

Defined in #52. Python: `encode_command`, `decode_command`, `encode_ack`, and `decode_ack`
in `pocketsat.messages`. Shared vectors: `tests/vectors/commands.json`. This section and
those vectors are the contract the Phase 7 firmware must match. Executing commands and
sending the ACK frames is the command dispatcher (#51); what each command does to the
mode is the state machine in [docs/spacecraft-modes.md](spacecraft-modes.md).

Every COMMAND frame that decodes as a frame gets exactly one ACK frame back: an ACK if it
was accepted, or a NACK with a [reason code](#nack-reason-codes). A frame that fails frame
decoding (sync, length, CRC, or type, see [Decoding and errors](#decoding-and-errors)) is
dropped without a reply, because its sequence number cannot be trusted.

### COMMAND payload

All multi-byte fields are big-endian.

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `command_id` | uint8 | See [Command IDs](#command-ids). `0x00` is reserved and never assigned |
| 1 | `arguments` | per command | Exactly the command's argument bytes, no padding |

So a COMMAND payload is 1 + the command's argument size, and the whole COMMAND frame is
that plus the 10-byte frame overhead (11 or 12 bytes for the commands below).

### Command IDs

The single source of truth is `pocketsat.messages.CommandId` and
`COMMAND_ARGUMENT_SIZES`; a test checks this table against them.

| ID | Command | Argument bytes | Arguments | Mode event (#51) |
|---|---|---|---|---|
| `0x01` | `PING` | 0 | none | none: never changes the mode, answered with an ACK in every mode (#51) |
| `0x02` | `SET_MODE` | 1 | offset 1: target mode, uint8 [`Mode`](#mode) value | `SET_MODE` with that target |
| `0x03` | `BEGIN_DOWNLINK` | 0 | none | `BEGIN_DOWNLINK` |
| `0x04` | `ENTER_SAFE_MODE` | 0 | none | `ENTER_SAFE_MODE` |
| `0x05` | `RESET` | 0 | none | `RESET` |

The `SET_MODE` argument uses the same numbers as the telemetry `mode` field (the
[Mode](#mode) table, `pocketsat.flight.Mode`). Any of the six modes decodes: only NOMINAL
and SCIENCE are commandable, and the state machine NACKs the others with
`TARGET_NOT_COMMANDABLE` (0x14). A byte that is not a mode at all (`0x06` to `0xFF`) is
`INVALID_ARGUMENT` (0x04).

Example, the `set_mode_science` vector (COMMAND, sequence 4, SET_MODE SCIENCE):

```
a5 5a 01 01 00 04 00 02 | 02 | 02 | 6f ca
frame header (len 2)      id   mode crc
```

### Decoding and validation

`decode_command` takes the COMMAND frame's payload and **never raises**: every problem
becomes a `MalformedCommand` carrying the received command ID and a `DecodeReason`, which
the dispatcher sends back in a NACK. Checks run in this order, so a payload with several
problems gets the first reason that applies:

| Order | Check | NACK reason | `command_id` in the NACK |
|---|---|---|---|
| 1 | The payload is not empty | `PAYLOAD_TRUNCATED` (0x02) | `0x00` (nothing to echo) |
| 2 | `command_id` is an assigned ID | `UNKNOWN_COMMAND` (0x01) | the received byte |
| 3 | At least the command's argument bytes are present | `PAYLOAD_TRUNCATED` (0x02) | the received byte |
| 4 | No bytes beyond the command's arguments | `PAYLOAD_TOO_LONG` (0x03) | the received byte |
| 5 | Every argument has a meaning (SET_MODE: a `Mode` value) | `INVALID_ARGUMENT` (0x04) | the received byte |

Only a command that passes all five reaches the state machine, so decoding reasons come
before mode reasons: in BOOT, an unknown command is NACKed `UNKNOWN_COMMAND`, not
`BOOT_IN_PROGRESS`. Arguments of an unknown command are never read.

### ACK payload

ACK and NACK share frame type `0x03` and one fixed **4-byte** payload, so an ACK frame is
always **14 bytes**. The ACK frame's own header sequence is the spacecraft's downlink
counter; the command it answers is named in the payload.

| Offset | Field | Type | Notes |
|---|---|---|---|
| 0 | `sequence` | uint16 | Header sequence of the COMMAND frame being answered |
| 2 | `command_id` | uint8 | Command ID byte received, echoed even if unassigned; `0x00` for an empty payload |
| 3 | `reason` | uint8 | `0x00` = ACK (accepted); any other value = NACK, see [NACK reason codes](#nack-reason-codes) |

Example, the `nack_target_not_commandable` vector (ACK frame, downlink sequence 12,
answering SET_MODE in command sequence 108, reason `0x14`):

```
a5 5a 01 03 00 0c 00 04 | 00 6c | 02 | 14 | a2 88
frame header (len 4)      seq     id   rsn  crc
```

`decode_ack` raises `AckDecodeError` for a payload that is not exactly 4 bytes or a
reason code that is neither `0x00` nor listed below.

### NACK reason codes

One uint8 namespace for every NACK, split by source:

- `0x00`: reserved. It is the `reason` of an ACK and never a NACK reason.
- `0x01`–`0x0F`: decoding and argument errors, `pocketsat.messages.DecodeReason`
  (this section, #52).
- `0x10`–`0x1F`: mode rejections, `pocketsat.flight.RejectReason`, defined by the state
  machine (#47, [docs/spacecraft-modes.md](spacecraft-modes.md#reason-codes)) and
  reused here unchanged.

The single source of truth is `pocketsat.messages.NACK_REASONS`, built from those two
enums; a test checks this table against it.

| Code | Reason | Enum | Meaning |
|---|---|---|---|
| `0x01` | `UNKNOWN_COMMAND` | `DecodeReason` | Command ID not assigned (`0x00` included) |
| `0x02` | `PAYLOAD_TRUNCATED` | `DecodeReason` | Payload empty or shorter than the command's layout |
| `0x03` | `PAYLOAD_TOO_LONG` | `DecodeReason` | Bytes beyond the command's layout |
| `0x04` | `INVALID_ARGUMENT` | `DecodeReason` | An argument value with no meaning, for example a SET_MODE byte that is not a mode |
| `0x10` | `BOOT_IN_PROGRESS` | `RejectReason` | In BOOT only RESET (and PING) are accepted |
| `0x11` | `FAULT_REQUIRES_RESET` | `RejectReason` | In FAULT only RESET is accepted (PING is answered in every mode) |
| `0x12` | `NOT_ALLOWED_IN_SAFE` | `RejectReason` | From SAFE, SET_MODE NOMINAL comes first; SCIENCE and BEGIN_DOWNLINK are refused |
| `0x13` | `SAFE_CONDITIONS_ACTIVE` | `RejectReason` | SET_MODE NOMINAL from SAFE while the triggering flags have not cleared |
| `0x14` | `TARGET_NOT_COMMANDABLE` | `RejectReason` | SET_MODE asked for BOOT, DOWNLINK, SAFE, or FAULT |

### Dispatch and ACK timing

Implemented by the command dispatcher (#51), the flight computer's decode-uplink and
execute-commands phases (`pocketsat.flight.computer`, ADR-0004 §2 step c).

- **One answer per command.** Every COMMAND frame received in a tick is answered in that
  tick, in arrival order, with one ACK frame naming its header sequence and command ID.
  A frame that fails frame decoding gets no answer (above); so does a valid frame of
  another type (TELEMETRY or ACK), which only the spacecraft sends.
- **Check order.** Decoding first ([Decoding and validation](#decoding-and-validation),
  reasons 0x01–0x04), then the mode: PING is ACKed in every mode and changes nothing;
  SET_MODE, BEGIN_DOWNLINK, ENTER_SAFE_MODE, and RESET raise the mode event of the same
  name, and the [transition table](spacecraft-modes.md#transition-table) decides ACK or
  NACK (reasons 0x10–0x14). Commands in one tick are judged in arrival order, each
  against the mode the commands before it left: after ENTER_SAFE_MODE, a SET_MODE
  SCIENCE in the same tick is NACKed `NOT_ALLOWED_IN_SAFE`; after RESET, everything but
  PING and RESET is NACKed `BOOT_IN_PROGRESS`.
- **Timing.** A command received in tick N is ACKed in tick N's downlink (ADR-0004 §2:
  command → ACK, same tick). The ACK means the command was accepted and the mode it
  asked for is in force at the end of tick N (or already was, for a `NO_CHANGE`), so
  telemetry from tick N shows the new mode. The new mode's controls take effect in tick
  N+1 (commands act one tick later), so the physical effect follows the ACK by one tick.
  An automatic event in the same tick (`SAFE_CONDITION`, `FAULT_DETECTED`) is applied
  after the commands and can still override an ACKed mode change; the ACK reports what
  the command did, and telemetry the final mode. The RESET ACK goes out in the tick of
  the reset, before the reboot (#49) restarts the downlink sequence.
- **Transmitter off.** With no transmit capacity (radio `RX_ONLY` or `OFF`, or the
  `transmitter_off` fault) commands are still received and executed, but every ACK and
  NACK is suppressed and counted (below); none is sent later.

### Command wire-contract rules

- A command ID, an argument layout, or a reason code is never renumbered, reassigned,
  or reused. `0x00` stays reserved as both a command ID and a reason code.
- A new command takes the next unassigned ID; a new decoding reason takes the next free
  code in `0x01`–`0x0F`, and a new mode reason the next in `0x10`–`0x1F`. This needs no
  protocol version change: firmware that does not know a new command NACKs it
  `UNKNOWN_COMMAND`, and a ground decoder that does not know a new reason code reports
  it (`AckDecodeError`) instead of guessing.
- A removed command or reason leaves its code unassigned; it is never reused.
- Argument sizes are exact: there are no optional or trailing arguments, so a longer
  layout for an existing command would be a new command ID.
- Changing an existing layout, or reusing a code, requires a protocol version change
  (the frame Version byte) and an ADR.

## Downlink sequence and transmit capacity

Every frame the spacecraft sends goes through one outbound queue in the flight computer
(`FlightComputer._queue_outbound`, #51), which applies ADR-0004 §10 and ADR-0007 §4:

- **Capacity.** Each tick the frames sent fit within comms' `transmit_capacity_bytes`
  for that tick (wire bytes: header, payload, and CRC), read from comms' readings. A
  capacity of 0 sends nothing.
- **Priority.** Frames are offered in priority order: ACK/NACK (step c), then telemetry
  (#55), then DATA (#56) (step e). Each is sent if it fits what is left; one that does
  not is dropped, not queued for a later tick. A frame that does not fit does not block
  a later, smaller one.
- **Suppression.** ACK/NACK and telemetry frames that do not fit are counted in the
  tick's `outbound_suppressed_count` (`FlightComputerOutput`, ADR-0007 §3). DATA that does
  not fit is not counted: its chunk stays in the payload buffer for a later tick.
- **Sequence.** The header sequence of every downlink frame, all types, comes from one
  counter (the spacecraft → ground direction) that starts at 0 on power-on and after
  every reboot, goes up by one per frame **sent**, and wraps `0xFFFF` → `0x0000`. A
  suppressed frame uses no number, so a gap seen on the ground means a frame lost on the
  way down. The ground pairs a restart at 0 with the boot counter in telemetry.
