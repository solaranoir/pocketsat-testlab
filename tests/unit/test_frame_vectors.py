"""Check the frame codec against the shared vectors in tests/vectors/frames.json."""

import json
from pathlib import Path
from typing import Any

import pytest

from pocketsat.frame import (
    Frame,
    FrameCrcError,
    FrameError,
    FrameLengthError,
    FrameSyncError,
    FrameType,
    FrameTypeError,
    crc16_ccitt_false,
    decode_frame,
    encode_frame,
)

VECTORS: dict[str, Any] = json.loads(
    (Path(__file__).parents[1] / "vectors" / "frames.json").read_text()
)

ERRORS: dict[str, type[FrameError]] = {
    "sync": FrameSyncError,
    "length": FrameLengthError,
    "crc": FrameCrcError,
    "type": FrameTypeError,
}


def _ids(key: str) -> list[str]:
    return [v["name"] for v in VECTORS[key]]


@pytest.mark.parametrize("vector", VECTORS["crc16_ccitt_false"], ids=_ids("crc16_ccitt_false"))
def test_crc(vector: dict[str, Any]) -> None:
    assert crc16_ccitt_false(bytes.fromhex(vector["input_hex"])) == vector["crc"]


@pytest.mark.parametrize("vector", VECTORS["valid_frames"], ids=_ids("valid_frames"))
def test_decode_valid(vector: dict[str, Any]) -> None:
    raw = bytes.fromhex(vector["frame_hex"])
    frame = decode_frame(raw)
    assert frame.version == vector["version"]
    assert frame.frame_type is FrameType[vector["type"]]
    assert frame.sequence == vector["sequence"]
    assert frame.payload == bytes.fromhex(vector["payload_hex"])
    assert int.from_bytes(raw[-2:], "big") == vector["crc"]


@pytest.mark.parametrize("vector", VECTORS["valid_frames"], ids=_ids("valid_frames"))
def test_encode_valid(vector: dict[str, Any]) -> None:
    frame = Frame(
        frame_type=FrameType[vector["type"]],
        sequence=vector["sequence"],
        payload=bytes.fromhex(vector["payload_hex"]),
        version=vector["version"],
    )
    assert encode_frame(frame).hex() == vector["frame_hex"]


@pytest.mark.parametrize("vector", VECTORS["invalid_frames"], ids=_ids("invalid_frames"))
def test_decode_invalid(vector: dict[str, Any]) -> None:
    with pytest.raises(ERRORS[vector["error"]]):
        decode_frame(bytes.fromhex(vector["frame_hex"]))
