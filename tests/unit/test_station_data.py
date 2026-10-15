"""DATA reception and chunk reassembly in the ground station (#158)."""

import dataclasses
import json
from pathlib import Path
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    ChunkRange,
    DataChunkReceived,
    DataStatistics,
    DataStore,
    GroundStation,
    StationConfig,
)
from pocketsat.messages import DataChunk, Packet
from pocketsat.spacecraft import NOMINAL_CONFIG, chunk_content
from pocketsat.spacecraft.config import MAX_CHUNK_SIZE_BYTES

SIZE = 64
TICK_US = 100_000
DATA_VECTORS: dict[str, Any] = json.loads(
    (Path(__file__).resolve().parents[1] / "vectors" / "data.json").read_text(encoding="utf-8")
)


def _good(chunk_id: int, size: int = SIZE) -> DataChunk:
    return DataChunk(chunk_id, chunk_content(chunk_id, size))


def _receive(store: DataStore, chunk_ids: list[int], start_us: int = 0) -> list[DataChunkReceived]:
    return [
        store.receive(start_us + n * TICK_US, n, _good(chunk_id))
        for n, chunk_id in enumerate(chunk_ids)
    ]


# --- Configuration ----------------------------------------------------------------------


def test_default_chunk_size_matches_the_spacecraft_default() -> None:
    # Both ends are configured with the chunk size; it is not on the wire (ADR-0009 §6).
    assert DEFAULT_STATION_CONFIG.data_chunk_size_bytes == NOMINAL_CONFIG.payload.chunk_size_bytes


@pytest.mark.parametrize("size", [0, -1, MAX_CHUNK_SIZE_BYTES + 1])
def test_chunk_size_out_of_range_is_rejected(size: int) -> None:
    with pytest.raises(ValueError, match="data_chunk_size_bytes"):
        StationConfig(data_chunk_size_bytes=size)
    with pytest.raises(ValueError, match="chunk_size_bytes"):
        DataStore(size)


@pytest.mark.parametrize("size", [True, 64.0, "64"])
def test_chunk_size_of_the_wrong_type_is_rejected(size: object) -> None:
    with pytest.raises(TypeError):
        StationConfig(data_chunk_size_bytes=size)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        DataStore(size)  # type: ignore[arg-type]


def test_station_checks_content_at_its_configured_chunk_size() -> None:
    vector = next(v for v in DATA_VECTORS["valid_data"] if v["name"] == "chunk_0_size_8")
    packet = Packet(frame=bytes.fromhex(vector["frame_hex"]))

    at_8 = GroundStation(StationConfig(data_chunk_size_bytes=8))
    at_default = GroundStation()

    assert at_8.end_tick(0, [packet]) == (DataChunkReceived(0, vector["sequence"], 0, True),)
    assert at_8.data.chunk(0) == bytes.fromhex(vector["content_hex"])
    # The same frame is corrupted for a station configured for 64-byte chunks.
    assert at_default.end_tick(0, [packet]) == (DataChunkReceived(0, vector["sequence"], 0, False),)
    assert at_default.data.chunk(0) is None


# --- Storing, duplicates and corruption -------------------------------------------------


def test_a_good_chunk_is_stored_once_with_its_event() -> None:
    store = DataStore(SIZE)
    event = store.receive(500, 7, _good(3))
    assert event == DataChunkReceived(500, sequence=7, chunk_id=3, content_ok=True)
    assert store.chunk_ids() == (3,)
    assert store.chunk(3) == chunk_content(3, SIZE)
    assert store.chunk(4) is None


def test_a_duplicate_is_counted_and_the_stored_copy_kept() -> None:
    store = DataStore(SIZE)
    store.receive(0, 0, _good(3))
    event = store.receive(TICK_US, 1, _good(3))
    assert event.duplicate and event.content_ok
    stats = store.statistics
    assert (stats.received_count, stats.stored_count, stats.duplicate_count) == (2, 1, 1)
    assert stats.received_bytes == 2 * SIZE
    assert stats.stored_bytes == SIZE
    assert store.chunk_ids() == (3,)


@pytest.mark.parametrize(
    "content",
    [
        bytes([chunk_content(5, SIZE)[0] ^ 0x01]) + chunk_content(5, SIZE)[1:],  # one bit
        chunk_content(5, SIZE)[:-1],  # one byte short
        chunk_content(5, SIZE) + b"\x00",  # one byte long
        chunk_content(6, SIZE),  # another chunk's content
    ],
    ids=["bit_flip", "short", "long", "wrong_id"],
)
def test_corrupted_content_is_counted_never_stored(content: bytes) -> None:
    store = DataStore(SIZE)
    event = store.receive(0, 0, DataChunk(5, content))
    assert event == DataChunkReceived(0, 0, 5, content_ok=False)
    assert store.chunk(5) is None
    stats = store.statistics
    assert (stats.received_count, stats.stored_count, stats.corrupted_count) == (1, 0, 1)
    assert stats.received_bytes == len(content)
    assert stats.stored_bytes == 0


def test_a_good_copy_after_a_corrupted_one_is_stored() -> None:
    store = DataStore(SIZE)
    bad = DataChunk(5, bytes(SIZE))
    store.receive(0, 0, bad)
    event = store.receive(TICK_US, 1, _good(5))
    assert event == DataChunkReceived(TICK_US, 1, 5, content_ok=True, duplicate=False)
    assert store.chunk(5) == chunk_content(5, SIZE)
    assert store.statistics.duplicate_count == 0


def test_a_corrupted_copy_of_a_stored_chunk_does_not_replace_it() -> None:
    store = DataStore(SIZE)
    store.receive(0, 0, _good(5))
    event = store.receive(TICK_US, 1, DataChunk(5, bytes(SIZE)))
    assert event == DataChunkReceived(TICK_US, 1, 5, content_ok=False, duplicate=True)
    assert store.chunk(5) == chunk_content(5, SIZE)
    stats = store.statistics
    assert (stats.corrupted_count, stats.duplicate_count) == (1, 0)


# --- Order and missing chunks -----------------------------------------------------------


def test_out_of_order_chunks_are_stored_by_id() -> None:
    store = DataStore(SIZE)
    _receive(store, [10, 12, 11, 13])
    assert store.chunk_ids() == (10, 11, 12, 13)
    assert store.statistics.missing_ranges == ()
    assert store.statistics.missing_count == 0


def test_missing_ids_are_reported_as_ranges() -> None:
    store = DataStore(SIZE)
    _receive(store, [0, 1, 5, 9, 10])
    stats = store.statistics
    assert stats.missing_ranges == (ChunkRange(2, 4), ChunkRange(6, 8))
    assert stats.missing_count == 6


def test_a_late_arrival_fills_its_gap() -> None:
    store = DataStore(SIZE)
    _receive(store, [0, 2])
    assert store.statistics.missing_ranges == (ChunkRange(1, 1),)
    store.receive(5 * TICK_US, 9, _good(1))
    assert not store.statistics.missing_ranges


def test_a_huge_gap_costs_one_range() -> None:
    # Ranges, not lists: the largest possible gap is one entry, computed at once.
    store = DataStore(SIZE)
    _receive(store, [0, 0xFFFF_FFFF])
    stats = store.statistics
    assert stats.missing_ranges == (ChunkRange(1, 0xFFFF_FFFE),)
    assert stats.missing_count == 0xFFFF_FFFE


def test_corrupted_and_duplicate_chunks_do_not_count_as_received_ids() -> None:
    store = DataStore(SIZE)
    store.receive(0, 0, _good(0))
    store.receive(TICK_US, 1, DataChunk(1, bytes(SIZE)))
    store.receive(2 * TICK_US, 2, _good(2))
    assert store.statistics.missing_ranges == (ChunkRange(1, 1),)


# --- Passes and rate --------------------------------------------------------------------


def test_start_pass_restarts_pass_statistics_and_keeps_the_chunks() -> None:
    store = DataStore(SIZE)
    _receive(store, [0, 1, 3])
    store.start_pass()
    assert store.pass_statistics == DataStatistics()
    _receive(store, [7, 9], start_us=10 * TICK_US)

    pass_stats = store.pass_statistics
    assert (pass_stats.received_count, pass_stats.stored_count) == (2, 2)
    assert pass_stats.missing_ranges == (ChunkRange(8, 8),)
    total = store.statistics
    assert total.stored_count == 5
    assert total.missing_ranges == (ChunkRange(2, 2), ChunkRange(4, 6), ChunkRange(8, 8))
    assert store.chunk_ids() == (0, 1, 3, 7, 9)


def test_a_chunk_stored_in_an_earlier_pass_is_a_duplicate_in_a_later_one() -> None:
    store = DataStore(SIZE)
    _receive(store, [4])
    store.start_pass()
    event = store.receive(TICK_US, 1, _good(4))
    assert event.duplicate
    assert store.pass_statistics.duplicate_count == 1
    assert store.pass_statistics.stored_count == 0


def test_data_rate_is_measured_between_the_first_and_last_chunk() -> None:
    store = DataStore(SIZE)
    assert store.statistics.data_rate_bytes_per_s is None
    _receive(store, [0])
    assert store.statistics.data_rate_bytes_per_s is None  # one chunk: no interval
    _receive(store, [1, 2, 3, 4], start_us=TICK_US)
    stats = store.statistics
    # One 64-byte chunk every 100 ms.
    assert stats.data_rate_bytes_per_s == 640.0
    assert (stats.first_chunk_us, stats.last_chunk_us) == (0, 4 * TICK_US)


def test_chunks_in_one_tick_give_no_rate() -> None:
    store = DataStore(SIZE)
    store.receive(TICK_US, 0, _good(0))
    store.receive(TICK_US, 1, _good(1))
    assert store.statistics.data_rate_bytes_per_s is None


# --- Through the station ----------------------------------------------------------------


def test_station_reset_clears_the_data() -> None:
    station = GroundStation()
    vector = next(v for v in DATA_VECTORS["valid_data"] if v["name"] == "chunk_1_default_size")
    station.end_tick(0, [Packet(frame=bytes.fromhex(vector["frame_hex"]))])
    assert station.data.chunk_ids() == (1,)
    station.reset(RngFactory(1))
    assert station.data.chunk_ids() == ()
    assert station.data.statistics == DataStatistics()


def test_statistics_are_immutable() -> None:
    stats = DataStore(SIZE).statistics
    with pytest.raises(dataclasses.FrozenInstanceError):
        stats.stored_count = 1  # type: ignore[misc]
