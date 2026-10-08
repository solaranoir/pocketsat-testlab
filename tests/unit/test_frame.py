"""Unit tests for pocketsat.frame beyond the shared vectors."""

import binascii
import random

import pytest

from pocketsat.frame import (
    CRC16_INIT,
    CRC16_POLY,
    MAX_PAYLOAD_SIZE,
    MIN_FRAME_SIZE,
    Frame,
    FrameCrcError,
    FrameError,
    FrameLengthError,
    FrameSyncError,
    FrameType,
    FrameTypeError,
    crc16_ccitt_false,
    crc16_ccitt_false_table,
    decode_frame,
    encode_frame,
    encode_frame_fields,
)


@pytest.mark.parametrize("frame_type", list(FrameType))
@pytest.mark.parametrize("payload", [b"", b"\x00", bytes(range(256))])
def test_round_trip(frame_type: FrameType, payload: bytes) -> None:
    frame = Frame(frame_type=frame_type, sequence=42, payload=payload)
    assert decode_frame(encode_frame(frame)) == frame


@pytest.mark.parametrize("frame_type", list(FrameType))
@pytest.mark.parametrize("sequence", [0, 1, 0x1234, 0xFFFF])
@pytest.mark.parametrize("payload", [b"", b"\x00", bytes(range(256)), bytes(MAX_PAYLOAD_SIZE)])
def test_encode_frame_fields_is_encode_frame(
    frame_type: FrameType, sequence: int, payload: bytes
) -> None:
    # The flight computer's encoder (#121) gives the bytes of the Frame path.
    assert encode_frame_fields(frame_type, sequence, payload) == encode_frame(
        Frame(frame_type, sequence, payload)
    )


@pytest.mark.parametrize(
    ("sequence", "payload"),
    [(-1, b""), (0x10000, b""), (0, bytes(MAX_PAYLOAD_SIZE + 1))],
)
def test_encode_frame_fields_rejects_what_frame_rejects(sequence: int, payload: bytes) -> None:
    with pytest.raises(ValueError) as from_frame:
        Frame(FrameType.DATA, sequence, payload)
    with pytest.raises(ValueError) as from_fields:
        encode_frame_fields(FrameType.DATA, sequence, payload)
    assert str(from_fields.value) == str(from_frame.value)


def test_encoded_size() -> None:
    frame = Frame(frame_type=FrameType.COMMAND, sequence=0, payload=b"abc")
    assert len(encode_frame(frame)) == MIN_FRAME_SIZE + 3


def test_every_single_bit_flip_is_rejected() -> None:
    raw = encode_frame(Frame(frame_type=FrameType.TELEMETRY, sequence=9, payload=b"\x10\x20"))
    for i in range(len(raw)):
        for bit in range(8):
            corrupted = bytearray(raw)
            corrupted[i] ^= 1 << bit
            with pytest.raises(FrameError):
                decode_frame(bytes(corrupted))


def test_errors_are_value_errors() -> None:
    for error in (FrameSyncError, FrameLengthError, FrameCrcError, FrameTypeError):
        assert issubclass(error, FrameError)
    assert issubclass(FrameError, ValueError)


def test_error_messages_are_explicit() -> None:
    with pytest.raises(FrameSyncError, match="bad sync"):
        decode_frame(b"\x00\x00")
    with pytest.raises(FrameLengthError, match="too short"):
        decode_frame(b"\xa5\x5a\x01")


@pytest.mark.parametrize(
    "kwargs",
    [
        {"sequence": -1},
        {"sequence": 0x10000},
        {"sequence": 0, "version": 256},
        {"sequence": 0, "payload": bytes(MAX_PAYLOAD_SIZE + 1)},
    ],
)
def test_frame_rejects_out_of_range_fields(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        Frame(frame_type=FrameType.ACK, **kwargs)  # type: ignore[arg-type]


# --- CRC: the codec's (binascii, #56) and the table-driven reference (#78) ----------------


def _bitwise_crc16_ccitt_false(data: bytes) -> int:
    """The bit-by-bit reference the lookup table replaced (#78)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def _crc_inputs() -> list[bytes]:
    """Every length 0 to 300 of seeded random bytes, every single byte, and edge
    patterns (all zeros, all ones, the check string)."""
    rng = random.Random(78)
    inputs = [bytes(rng.getrandbits(8) for _ in range(n)) for n in range(301)]
    inputs += [bytes([b]) for b in range(256)]
    inputs += [b"\x00" * 64, b"\xff" * 64, b"123456789", bytes(range(256)) * 4]
    return inputs


def test_crc_parameters_are_ccitt_false() -> None:
    assert (CRC16_POLY, CRC16_INIT) == (0x1021, 0xFFFF)
    for crc in (crc16_ccitt_false, crc16_ccitt_false_table):
        assert crc(b"123456789") == 0x29B1
        assert crc(b"") == 0xFFFF


def test_table_crc_matches_the_bitwise_reference() -> None:
    for data in _crc_inputs():
        assert crc16_ccitt_false_table(data) == _bitwise_crc16_ccitt_false(data), data.hex()


def test_codec_crc_matches_the_table_and_binascii_crc_hqx_from_init_0xffff() -> None:
    # binascii.crc_hqx is the same polynomial (0x1021, unreflected, no final XOR) with
    # the initial value as its second argument, so from 0xFFFF it is CCITT-FALSE.
    for data in _crc_inputs():
        expected = crc16_ccitt_false_table(data)
        assert crc16_ccitt_false(data) == expected == binascii.crc_hqx(data, 0xFFFF), data.hex()
