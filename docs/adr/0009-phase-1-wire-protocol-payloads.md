# ADR-0009: Phase 1 wire protocol payloads (protocol version 1)

- **Status:** Accepted (reviewed in the #65 milestone check, 2026-10-12)
- **Phase:** 1
- **Amends / Supersedes:** Amends ADR-0002 §2 (adds frame type `0x04` DATA to the frame-type list, and records the field-level payloads ADR-0002 left to `docs/protocol.md`). Related: ADR-0004 (§7 readings only on the wire, §10 outbound priority, §13 downlink data ownership), ADR-0006 (portable arithmetic), ADR-0007 and ADR-0008 (outbound capacity, sequence numbering). Issues: #52, #54, #56 (PRs #109, #106, #119), with the mode values and reason codes of #47 (PR #107). Recorded by #65.

## Context

ADR-0002 fixed the frame (sync, version, type, sequence, length, payload, CRC) and three frame types, and left "the full field-level spec (command IDs, telemetry layout)" to `docs/protocol.md`. Phase 1 filled that in through three tickets, each reviewed by the maintainer before merging:

- #54 (PR #106, `needs-human`): the 26-byte telemetry payload, its units, rounding, saturation and flag bits;
- #52 (PR #109): the COMMAND payload, the command IDs, the 4-byte ACK payload for ACK and NACK, and the NACK reason codes, reusing #47's mode reason codes (PR #107);
- #56 (PR #119): a fourth frame type, DATA (`0x04`), carrying one payload data chunk.

Together these payloads are the contract the Phase 7a MCU firmware must implement byte for byte, and the one the Phase 2 ground station decodes. `docs/protocol.md` says that changing any of them needs a protocol version change and an ADR, but no ADR records the baseline itself, and ADR-0002's frame-type table still lists three types. #65 (the Phase 1 milestone check) reviewed Phase 1 for decisions that are cross-cutting or hard to reverse and not yet in an ADR; the wire payloads are the clearest case.

This ADR records the decisions **as already implemented and accepted in review**. It changes nothing. `docs/protocol.md` and the shared vectors in `tests/vectors/` remain the canonical specification; this record keeps the decisions, the reasons, and the rejected options in one place, so that later phases change them deliberately.

## Decision

### 1. Protocol version 1 has four frame types

| Code | Type | Direction | Payload |
|---|---|---|---|
| `0x01` | COMMAND | ground → spacecraft | command ID and exact argument bytes (§3) |
| `0x02` | TELEMETRY | spacecraft → ground | fixed 26 bytes (§5) |
| `0x03` | ACK | spacecraft → ground | fixed 4 bytes, for both ACK and NACK (§4) |
| `0x04` | DATA | spacecraft → ground | one payload chunk: uint32 chunk ID and its content (§6) |

The frame layout, the CRC (CRC-16/CCITT-FALSE) and the Version byte (`1`) are unchanged from ADR-0002. DATA is the only addition to ADR-0002's frame-type list (#56, PR #119).

### 2. Where the specification lives, and how it is kept honest

- `docs/protocol.md` is the specification: every field, offset, type, unit, range, code and rule.
- The shared vectors (`tests/vectors/frames.json`, `telemetry.json`, `commands.json`, `data.json`) are the bytes the firmware is tested against. They are generated independently of `pocketsat` (`struct`, `decimal`, `binascii.crc_hqx`, a separately written xorshift32), so a codec bug cannot hide in its own vectors.
- Each table in `docs/protocol.md` has one source of truth in code (`FrameType`, `CommandId` and `COMMAND_ARGUMENT_SIZES`, `NACK_REASONS`, `TELEMETRY_FIELDS`, `TelemetryFlags`, `Mode`), and a unit test checks the table against it.

### 3. Commands (#52, PR #109)

- A COMMAND payload is a uint8 command ID followed by exactly that command's argument bytes, with no padding: `PING` `0x01`, `SET_MODE` `0x02` (one byte, a `Mode` value), `BEGIN_DOWNLINK` `0x03`, `ENTER_SAFE_MODE` `0x04`, `RESET` `0x05`. ID `0x00` is reserved and never assigned.
- **Argument sizes are exact.** A payload longer than its command's layout is NACKed (`PAYLOAD_TOO_LONG`), not trimmed, so a longer layout for an existing command must be a new command ID.
- **Decoding never raises.** Every problem becomes a NACK with a reason, checked in a fixed order (empty, unknown ID, truncated, too long, invalid argument), and decoding reasons come before mode reasons: in BOOT an unknown command gets `UNKNOWN_COMMAND`, not `BOOT_IN_PROGRESS`.
- `RESET` has no confirmation argument: the CRC already guards against a corrupted byte becoming RESET.

### 4. One ACK frame per command, and one reason-code namespace (#51, #52, #47)

- Every COMMAND frame that decodes as a frame gets exactly one ACK frame: the 4-byte payload names the command frame's header sequence (uint16) and the command ID received (echoed even if unassigned), and a reason byte, `0x00` for ACK and anything else for NACK. A frame that fails frame decoding (sync, length, CRC, type) gets no reply, because its sequence cannot be trusted.
- A command that asks for what is already true (SET_MODE to the current mode, ENTER_SAFE_MODE in SAFE, BEGIN_DOWNLINK in DOWNLINK) is ACKed, not NACKed, so a ground retry after a lost ACK succeeds (#47 decision 8).
- NACK reasons share one uint8 namespace: `0x00` reserved (it is the ACK's reason byte), `0x01`–`0x0F` decoding and argument errors (`DecodeReason`, #52), `0x10`–`0x1F` mode rejections (`RejectReason`, defined by #47's state machine). They are two sibling enums combined in `NACK_REASONS`, so the state machine's enum holds only what the state machine produces.
- The `Mode` values (BOOT 0 to FAULT 5) are wire values, shared by the telemetry `mode` field and the `SET_MODE` argument.

### 5. Telemetry (#54, PR #106)

- **Fixed 26 bytes** of big-endian integers, each at an offset that is a multiple of its size, with no padding; a C implementation serializes them byte by byte rather than through a packed struct. A TELEMETRY frame is 36 bytes.
- **Reported values only.** Every sensor field comes from a readings record, never a truth record (ADR-0004 §7); the encoder accepts only readings types, enforced by mypy and at run time.
- **Physical values are integers in fixed units** (mV, 0.1 % SOC, 0.01 °C, 0.01° pointing error), converted by one portable rule (ADR-0006): one IEEE double multiplication, round half away from zero (C's `lround`), then saturate to the field's range. Firmware must multiply in double precision.
- **Counters:** `uptime_ms` wraps modulo 2³² like a C `uint32_t`; `boot_count` and `buffered_bytes` saturate, because a wrapped boot count would look like a fresh spacecraft.
- **Payload fill is `buffered_bytes`**, exact and unscaled, so the ground can check the downlink accounting; the fill fraction follows from the configured capacity.
- **NaN has no encoding.** The SIL encoder raises, because readings never legitimately contain NaN, and the flight computer enters FAULT on a non-finite reading (#48).
- **Flags** are the readings flags the flight computer acts on, in a uint16 grouped by subsystem (power bits 0–3, thermal 4–7, attitude 8–11, payload and comms 12–15) with reserved bits sent as 0.
- **Decoding** is strict about size and enumerated values (`TelemetryDecodeError`) and ignores unknown flag bits, so a new flag needs no receiver change but a new state value does.

### 6. DATA (#56, PR #119)

- A DATA payload is the chunk ID (uint32) followed by the chunk's content. **The chunk size is not on the wire**: both ends are configured with it (`PayloadConfig.chunk_size_bytes` and `FlightComputerConfig.downlink`, 64 bytes by default, so a DATA frame is 78 bytes), and `SilTarget` refuses a mismatch.
- **Chunk content is part of the contract.** It is a pure function of the chunk ID (`chunk_content`, 32-bit integer operations only), so the payload stores no bytes and the ground regenerates every chunk to detect corruption; changing the function changes every chunk.
- Within a session, chunk IDs go up by one from frame to frame, never skip and never repeat; a gap means a loss.
- In Phase 1 a chunk is **released once its whole frame has been transmitted**, with no acknowledgement from the ground (ADR-0004 §13, ADR-0008 §6): a DATA frame lost on the way down is lost.

### 7. One downlink sequence counter

The header sequence of every spacecraft-to-ground frame, all types, comes from one counter that starts at 0 at power-on and after every reboot, goes up by one per frame committed to the transmitter, and wraps `0xFFFF` → `0x0000`. A suppressed frame takes no number, so a gap seen on the ground means a frame lost on the way down or aborted mid-transmission (ADR-0007 §3, ADR-0008). The ground pairs a restart at 0 with the boot counter in telemetry.

### 8. Change rules

These rules, stated in `docs/protocol.md`, are part of this decision:

- A command ID, argument layout, reason code, flag bit, state value or mode value is never renumbered, reassigned or reused; a removed one stays unassigned.
- A new command takes the next unassigned ID, a new reason the next free code in its range, and a new flag a reserved bit in its subsystem's group. None of these needs a protocol version change.
- Changing an existing layout (including putting the chunk size on the wire, or changing the chunk content function), or reusing a code or bit, requires a protocol version change (the frame Version byte) and an ADR that amends or supersedes this one.
- A change that alters the downlink bytes of the golden captures (`tests/golden/`, #64) is regenerated deliberately, in its own commit, under the policy in `tests/golden/README.md`.

## Alternatives considered

- **No ADR; `docs/protocol.md` as the only record.** The table-level detail belongs in `docs/protocol.md`, and stays there. But the payloads are the firmware contract that later phases depend on, ADR-0002's frame-type list was out of date, and `docs/protocol.md` requires an ADR for any change without an ADR for the baseline. Rejected for the decisions and their reasons; the specification itself stays in the protocol doc.
- **Floating-point telemetry values.** Simple in Python, but float formats, rounding and NaN handling vary across MCUs and compilers. Rejected (#54): fixed-unit integers with one portable rounding rule.
- **Payload buffer fill as a uint16 fraction.** Two bytes smaller, but lossy and scaled. Rejected for exact `buffered_bytes` (#54 decision 2).
- **A per-field NaN sentinel.** Costs one code per field and is never needed in SIL. Deferred to the firmware (open question below).
- **Decoding reasons added to the state machine's `RejectReason`.** One enum, but the state machine would then hold codes it never produces. Rejected for two sibling enums in one namespace (#52 decision 1).
- **A 5-byte ACK with a separate status byte.** Rejected: one fixed 4-byte layout for ACK and NACK, with reason `0x00` meaning ACK, keeps firmware buffers fixed (#52 decision 2).
- **Ignoring trailing argument bytes.** More forgiving, but it hides ground-side mistakes and blocks extending a layout safely. Rejected (#52 decision 3).
- **The chunk size on the wire, or in the payload readings.** Both sides are built for one payload, so the field would only cost bytes; a contract change for nothing. Rejected (#56 decisions 6 and 8).
- **Releasing a chunk only when the ground acknowledges it.** The right rule for a lossy link, but it needs a ground acknowledgement and retransmission design. Deferred: the downlink session keeps what it sent, so only the flight computer's release rule changes, not this layout (#56, ADR-0004 §13).

## Consequences

- The Phase 7a firmware has one written contract: this ADR for the decisions, `docs/protocol.md` for the details, and the shared vectors for the bytes. The Phase 2 ground station decodes the same frames.
- ADR-0002's frame-type list is read together with §1 of this ADR.
- Every later change to a payload is visible as an ADR, a protocol version change where the rules require one, and a deliberate golden regeneration, rather than an edit to a table.
- The cost is a second document to keep consistent with `docs/protocol.md`. This ADR therefore states decisions and rules, not field tables, and points to the protocol doc for the details.

## Open questions for later phases

- **Maximum payload size and serial resynchronisation.** `docs/protocol.md` leaves both TBD; they must be settled before frames cross a serial link. Tracked by #123 (Phase 7a), which owns any frame-format ADR.
- **An unknown Version byte.** The frame decoder accepts any Version value and the payload codecs assume version 1. Before a version 2 exists, decide what a receiver does with a frame of another version (drop and count it, NACK it, or decode it under that version's rules).
- **Invalid sensor values in firmware.** A per-field sentinel or a "sensor invalid" flag bit from a reserved group (`docs/roadmap.md`, Phase 7).
- **Extending telemetry.** The decoder requires exactly 26 bytes, so a new field (for example battery current, angular rate, or the anomaly counter proposed in backlog #118) is a layout change and so a protocol version change. Decide before the first such field whether that is the mechanism, or whether new data goes in a new frame type.
- **Ground-acknowledged DATA release and retransmission** (a later phase, after the RF channel of Phase 3 makes downlink loss real): whether the acknowledgement is a new command and how the session retransmits; the DATA layout itself is not expected to change.

## Follow-ups

- `docs/protocol.md` status line points to this ADR. **Done** in #65.
- `docs/architecture.md` §12 lists this ADR. **Done** in #65.
- On acceptance, the maintainer sets the status to Accepted.
