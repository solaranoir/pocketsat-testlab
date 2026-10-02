# Shared test vectors

Language-neutral test data for the wire format in ADR-0002 and `docs/protocol.md`.
The Python tests (`tests/unit/test_frame_vectors.py`) use these files, and the MCU
firmware tests will use the same files later, so both implementations stay in step.

## `frames.json`

Byte strings are lowercase hex with no separators. Numbers are JSON integers
(so `"crc": 10673` is `0x29B1`).

- `crc16_ccitt_false[]`: `input_hex` and the expected `crc` (CRC-16/CCITT-FALSE:
  poly `0x1021`, init `0xFFFF`, no reflection, no final XOR).
- `valid_frames[]`: `frame_hex` must decode to `version`, `type` (`COMMAND`,
  `TELEMETRY`, `ACK`), `sequence`, and `payload_hex`, with trailing CRC `crc`.
  Encoding those fields must produce exactly `frame_hex`.
- `invalid_frames[]`: `frame_hex` must be rejected with the error kind in `error`:
  - `sync`: missing or wrong sync marker
  - `length`: frame size doesn't match the header's length field
  - `crc`: CRC mismatch
  - `type`: unknown frame type code (valid CRC)

Decoders check sync, then length, then CRC, then type, so each invalid vector has
exactly one expected error.

The vectors were generated independently of `pocketsat.frame` (with Python's
`binascii.crc_hqx` and hand-packed headers). Don't regenerate them from the
implementation under test; add new vectors by hand or from an independent reference.

The payloads in `frames.json` are opaque to the frame codec; for example
`telemetry_uptime` predates the telemetry layout and is not a valid TELEMETRY payload.

## `telemetry.json`

TELEMETRY payload vectors (#54), checked by `tests/unit/test_telemetry_vectors.py`. The
layout, conversions, rounding, and saturation rules are in `docs/protocol.md`
("Telemetry payload"). `payload_size` and `frame_size` give the fixed sizes (26 and 36).

- `valid_telemetry[]`: one telemetry frame each.
  - `input`: the encoder's inputs. `uptime_ms`, `mode_id`, `boot_count` are the flight
    computer state; `power`, `thermal`, `attitude`, `payload`, and `comms` hold only the
    readings fields that go on the wire (other readings fields don't affect the payload).
    States are enum member names. A float may be the string `"inf"` or `"-inf"`, which
    JSON can't express.
  - `fields`: the expected wire integer of every payload field, by its name in
    `docs/protocol.md`.
  - `payload_hex`: the 26-byte payload. Encoding `input` must produce exactly this, and
    decoding it must give back `fields` (scaled by the documented factors).
  - `sequence`, `crc`, `frame_hex`: the whole TELEMETRY frame (version 1) carrying
    `payload_hex`.
  - The set covers no flags, each flag alone, all flags, every field at both limits and
    beyond them, infinities, and rounding at and just below one half.
- `decoder_ignores[]`: payloads with reserved flag bits set; decoding keeps only the
  assigned bits (`decoded_flags`).
- `invalid_payloads[]`: payloads the decoder must reject, with the reason in `error`:
  `size` (not 26 bytes), `mode`, `radio_mode`, `attitude_state`, or `payload_state`
  (unknown enumerated value).

These vectors were generated independently of `pocketsat.messages`: hand-packed with
`struct`, rounded with `decimal` (`ROUND_HALF_UP` on the IEEE double product), and
checksummed with `binascii.crc_hqx`. As for `frames.json`, don't regenerate them from the
implementation under test.
