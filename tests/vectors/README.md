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
