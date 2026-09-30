"""Unit tests for pocketsat.frame beyond the shared vectors."""

import pytest

from pocketsat.frame import (
    MAX_PAYLOAD_SIZE,
    MIN_FRAME_SIZE,
    Frame,
    FrameCrcError,
    FrameError,
    FrameLengthError,
    FrameSyncError,
    FrameType,
    FrameTypeError,
    decode_frame,
    encode_frame,
)


@pytest.mark.parametrize("frame_type", list(FrameType))
@pytest.mark.parametrize("payload", [b"", b"\x00", bytes(range(256))])
def test_round_trip(frame_type: FrameType, payload: bytes) -> None:
    frame = Frame(frame_type=frame_type, sequence=42, payload=payload)
    assert decode_frame(encode_frame(frame)) == frame


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
