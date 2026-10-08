"""Unit tests for the DATA payload codec (#56) and its docs/protocol.md tables."""

import re
import struct
from pathlib import Path

import pytest

from pocketsat.frame import FrameType
from pocketsat.messages import (
    DATA_CHUNK_ID_FORMAT,
    DATA_CHUNK_ID_SIZE,
    MIN_DATA_PAYLOAD_SIZE,
    DataChunk,
    DataDecodeError,
    data_frame_size,
    decode_data,
    encode_data,
)
from pocketsat.spacecraft import (
    CHUNK_ID_SIZE_BYTES,
    MAX_CHUNK_ID,
    MAX_CHUNK_SIZE_BYTES,
    NOMINAL_CONFIG,
    chunk_content,
)

PROTOCOL = (Path(__file__).parents[2] / "docs" / "protocol.md").read_text(encoding="utf-8")


def test_frame_type_code_is_fixed() -> None:
    assert FrameType.DATA.value == 0x04


def test_sizes() -> None:
    assert DATA_CHUNK_ID_SIZE == CHUNK_ID_SIZE_BYTES == 4
    assert MIN_DATA_PAYLOAD_SIZE == 5
    assert data_frame_size(NOMINAL_CONFIG.payload.chunk_size_bytes) == 78
    # The default DATA frame fits one default 100 ms tick's capacity (#44, #72, #122).
    assert data_frame_size(64) <= NOMINAL_CONFIG.comms.transmit_rate_bytes_per_s // 10


@pytest.mark.parametrize(
    ("chunk_id", "size"), [(0, 1), (1, 64), (12345, 7), (MAX_CHUNK_ID, 64), (3, 300)]
)
def test_round_trip(chunk_id: int, size: int) -> None:
    chunk = DataChunk(chunk_id, chunk_content(chunk_id, size))
    payload = encode_data(chunk)
    assert len(payload) == DATA_CHUNK_ID_SIZE + size
    assert payload[:4] == chunk_id.to_bytes(4, "big")
    assert payload[4:] == chunk.content
    assert decode_data(payload) == chunk


def test_largest_chunk_fits_the_frame_length_field() -> None:
    chunk = DataChunk(0, bytes(MAX_CHUNK_SIZE_BYTES))
    assert len(encode_data(chunk)) == 0xFFFF


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"chunk_id": -1, "content": b"x"}, ValueError),
        ({"chunk_id": MAX_CHUNK_ID + 1, "content": b"x"}, ValueError),
        ({"chunk_id": 1.0, "content": b"x"}, TypeError),
        ({"chunk_id": True, "content": b"x"}, TypeError),
        ({"chunk_id": 0, "content": b""}, ValueError),
        ({"chunk_id": 0, "content": bytes(MAX_CHUNK_SIZE_BYTES + 1)}, ValueError),
        ({"chunk_id": 0, "content": bytearray(b"x")}, TypeError),
        ({"chunk_id": 0, "content": "x"}, TypeError),
    ],
)
def test_data_chunk_validates(kwargs: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        DataChunk(**kwargs)  # type: ignore[arg-type]


@pytest.mark.parametrize("size", range(MIN_DATA_PAYLOAD_SIZE))
def test_decode_rejects_payloads_without_an_id_and_content(size: int) -> None:
    with pytest.raises(DataDecodeError):
        decode_data(bytes(size))


def test_decode_returns_the_content_as_received() -> None:
    # Checking the content is the receiver's job: a corrupted chunk still decodes.
    payload = bytes([0, 0, 0, 9]) + b"\x00" * 64
    assert decode_data(payload) == DataChunk(9, b"\x00" * 64)
    assert decode_data(payload).content != chunk_content(9, 64)


def test_decode_accepts_any_bytes_like_payload() -> None:
    payload = encode_data(DataChunk(5, b"abc"))
    assert decode_data(memoryview(payload)) == DataChunk(5, b"abc")  # type: ignore[arg-type]


# --- docs/protocol.md agrees with the code ---------------------------------------------


def _table_after(heading: str) -> list[list[str]]:
    """Rows of the first Markdown table after ``heading``, header and rule excluded."""
    start = PROTOCOL.index(heading + "\n")
    rows: list[list[str]] = []
    for line in PROTOCOL[start:].splitlines()[1:]:
        if line.startswith("|"):
            rows.append([cell.strip() for cell in line.strip().strip("|").split("|")])
        elif rows:
            break
    return rows[2:]


def _code(cell: str) -> str:
    match = re.fullmatch(r"`([^`]+)`", cell)
    assert match, cell
    return match.group(1)


def test_docs_frame_type_table_matches_the_enum() -> None:
    rows = _table_after("## Frame types")
    assert {row[1]: int(_code(row[0]), 16) for row in rows} == {t.name: t.value for t in FrameType}


def test_docs_data_layout_table_matches_the_format() -> None:
    rows = _table_after("### DATA layout")
    assert [(row[0], _code(row[1]), row[2]) for row in rows] == [
        ("0", "chunk_id", "uint32"),
        ("4", "content", "chunk size"),
    ]
    assert DATA_CHUNK_ID_FORMAT == ">I"
    assert struct.calcsize(DATA_CHUNK_ID_FORMAT) == 4


def test_docs_data_example_is_the_vector() -> None:
    chunk = DataChunk(0, chunk_content(0, 8))
    assert encode_data(chunk).hex() == "0000000029d04a5133d5399c"
    assert "00 00 00 00 | 29 d0 4a 51 33 d5 39 9c | 0c f2" in PROTOCOL
