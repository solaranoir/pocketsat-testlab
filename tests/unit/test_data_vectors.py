"""Check the DATA codec against the shared vectors in tests/vectors/data.json (#56).

The vectors were generated independently of ``pocketsat`` (see ``tests/vectors/README.md``),
so these tests pin the wire contract, the chunk content function included, not the code.
"""

import json
from pathlib import Path
from typing import Any

import pytest

from pocketsat.frame import MIN_FRAME_SIZE, Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    DATA_CHUNK_ID_SIZE,
    MIN_DATA_PAYLOAD_SIZE,
    DataChunk,
    DataDecodeError,
    data_frame_size,
    decode_data,
    encode_data,
)
from pocketsat.spacecraft import NOMINAL_CONFIG, chunk_content

VECTORS: dict[str, Any] = json.loads(
    (Path(__file__).parents[1] / "vectors" / "data.json").read_text()
)


def _ids(key: str) -> list[str]:
    return [v["name"] for v in VECTORS[key]]


def test_sizes_and_type_code_match() -> None:
    assert VECTORS["frame_type"] == FrameType.DATA.value
    assert VECTORS["chunk_id_size"] == DATA_CHUNK_ID_SIZE
    assert VECTORS["min_payload_size"] == MIN_DATA_PAYLOAD_SIZE
    assert VECTORS["default_chunk_size"] == NOMINAL_CONFIG.payload.chunk_size_bytes
    assert VECTORS["default_frame_size"] == data_frame_size(VECTORS["default_chunk_size"])
    assert VECTORS["default_frame_size"] - VECTORS["default_chunk_size"] == (
        MIN_FRAME_SIZE + DATA_CHUNK_ID_SIZE
    )


@pytest.mark.parametrize("vector", VECTORS["valid_data"], ids=_ids("valid_data"))
def test_chunk_content_matches(vector: dict[str, Any]) -> None:
    assert chunk_content(vector["chunk_id"], vector["chunk_size"]).hex() == vector["content_hex"]


@pytest.mark.parametrize("vector", VECTORS["valid_data"], ids=_ids("valid_data"))
def test_encode_matches(vector: dict[str, Any]) -> None:
    chunk = DataChunk(vector["chunk_id"], bytes.fromhex(vector["content_hex"]))
    assert encode_data(chunk).hex() == vector["payload_hex"]


@pytest.mark.parametrize("vector", VECTORS["valid_data"], ids=_ids("valid_data"))
def test_decode_matches(vector: dict[str, Any]) -> None:
    chunk = decode_data(bytes.fromhex(vector["payload_hex"]))
    assert chunk == DataChunk(vector["chunk_id"], bytes.fromhex(vector["content_hex"]))


@pytest.mark.parametrize("vector", VECTORS["valid_data"], ids=_ids("valid_data"))
def test_whole_frame_matches(vector: dict[str, Any]) -> None:
    payload = bytes.fromhex(vector["payload_hex"])
    raw = encode_frame(Frame(FrameType.DATA, vector["sequence"], payload))
    assert raw.hex() == vector["frame_hex"]
    assert int.from_bytes(raw[-2:], "big") == vector["crc"]
    assert len(raw) == data_frame_size(vector["chunk_size"])
    frame = decode_frame(raw)
    assert (frame.frame_type, frame.sequence, frame.payload) == (
        FrameType.DATA,
        vector["sequence"],
        payload,
    )


@pytest.mark.parametrize("vector", VECTORS["invalid_data"], ids=_ids("invalid_data"))
def test_invalid_payloads_are_rejected(vector: dict[str, Any]) -> None:
    assert vector["error"] == "size"
    with pytest.raises(DataDecodeError):
        decode_data(bytes.fromhex(vector["payload_hex"]))
