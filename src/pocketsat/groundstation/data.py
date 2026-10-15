"""DATA reception and chunk reassembly (#158, ADR-0009 §6, ADR-0010 §4).

:class:`DataStore` receives every decoded DATA chunk the downlink decoder hands it
(:class:`~pocketsat.groundstation.downlink.DownlinkDecoder`) and keeps the payload data
the station has collected:

- **Content check.** A chunk's content is a pure function of its ID and the chunk size
  (``chunk_content``, #43). Both ends are configured with the chunk size, which is not
  on the wire (ADR-0009 §6): the station's is
  :attr:`~pocketsat.groundstation.StationConfig.data_chunk_size_bytes`, and it must
  match the spacecraft's ``PayloadConfig.chunk_size_bytes``. A chunk whose content does
  not match, including one of another length, is **corrupted**: it is counted and never
  stored, so a later good copy of the same ID is still stored.
- **Stored once.** A good chunk is stored by its ID. A chunk whose ID is already stored
  is a **duplicate**: counted, and the stored copy is kept.
- **Order.** Chunks are stored by ID whatever order they arrive in; :meth:`chunk_ids`
  lists them in ID order.
- **Missing IDs.** Within a downlink session the spacecraft sends chunk IDs going up by
  one and never repeats one, and a chunk is released once sent (ADR-0009 §6), so an ID
  never received between the lowest and highest stored is lost for good. Missing IDs
  are reported as ranges, for the run (between the lowest and highest ID stored since
  the reset) and for the current pass (between the lowest and highest ID stored in the
  pass). Ranges, not lists, so a long loss costs one entry.

Every DATA frame produces one :class:`~pocketsat.groundstation.DataChunkReceived`
event. Pass boundaries come from the pass state machine (#161), through
:meth:`start_pass`; until then the whole run is one pass.
"""

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Final

from pocketsat.groundstation.events import DataChunkReceived
from pocketsat.messages import DataChunk
from pocketsat.spacecraft.config import MAX_CHUNK_SIZE_BYTES
from pocketsat.spacecraft.payload import chunk_content

US_PER_S: Final = 1_000_000


@dataclass(frozen=True)
class ChunkRange:
    """A run of consecutive chunk IDs, both ends included.

    Attributes:
        first_id: The lowest ID in the range.
        last_id: The highest ID in the range, ``>= first_id``.
    """

    first_id: int
    last_id: int

    @property
    def count(self) -> int:
        """How many IDs the range holds."""
        return self.last_id - self.first_id + 1


@dataclass(frozen=True)
class DataStatistics:
    """DATA counts over a span of the run: the total, or one pass.

    Attributes:
        received_count: DATA chunks received, whatever became of them.
        stored_count: Chunks stored (good and new).
        duplicate_count: Chunks whose ID was already stored.
        corrupted_count: Chunks whose content did not match ``chunk_content``.
        received_bytes: Content bytes of every chunk received.
        stored_bytes: Content bytes of the chunks stored.
        missing_ranges: IDs never stored between the lowest and highest stored in the
            span, in ID order.
        missing_count: How many IDs ``missing_ranges`` holds.
        first_chunk_us: Simulated time of the span's first chunk, or ``None``.
        last_chunk_us: Simulated time of the span's last chunk, or ``None``.
        data_rate_bytes_per_s: Received content bytes per simulated second, measured
            from the first chunk to the last: the bytes of the chunks after the first,
            divided by the time between them. ``None`` until two chunks have arrived at
            different times.
    """

    received_count: int = 0
    stored_count: int = 0
    duplicate_count: int = 0
    corrupted_count: int = 0
    received_bytes: int = 0
    stored_bytes: int = 0
    missing_ranges: tuple[ChunkRange, ...] = ()
    missing_count: int = 0
    first_chunk_us: int | None = None
    last_chunk_us: int | None = None
    data_rate_bytes_per_s: float | None = None


def _missing(ids: Iterable[int]) -> tuple[ChunkRange, ...]:
    """The gaps between consecutive IDs of ``ids`` (any order, no repeats)."""
    ranges: list[ChunkRange] = []
    previous: int | None = None
    for chunk_id in sorted(ids):
        if previous is not None and chunk_id > previous + 1:
            ranges.append(ChunkRange(previous + 1, chunk_id - 1))
        previous = chunk_id
    return tuple(ranges)


class _Span:
    """Mutable counters behind one :class:`DataStatistics` snapshot."""

    def __init__(self) -> None:
        self.received_count = 0
        self.duplicate_count = 0
        self.corrupted_count = 0
        self.received_bytes = 0
        self.stored: dict[int, int] = {}
        """Chunk ID -> content length, for the chunks stored in the span."""
        self.first_chunk_us: int | None = None
        self.last_chunk_us: int | None = None
        self.first_chunk_bytes = 0

    def receive(self, now_us: int, size_bytes: int) -> None:
        self.received_count += 1
        self.received_bytes += size_bytes
        if self.first_chunk_us is None:
            self.first_chunk_us = now_us
            self.first_chunk_bytes = size_bytes
        self.last_chunk_us = now_us

    def snapshot(self) -> DataStatistics:
        missing = _missing(self.stored)
        rate: float | None = None
        first, last = self.first_chunk_us, self.last_chunk_us
        if first is not None and last is not None and last > first:
            rate = (self.received_bytes - self.first_chunk_bytes) * US_PER_S / (last - first)
        return DataStatistics(
            received_count=self.received_count,
            stored_count=len(self.stored),
            duplicate_count=self.duplicate_count,
            corrupted_count=self.corrupted_count,
            received_bytes=self.received_bytes,
            stored_bytes=sum(self.stored.values()),
            missing_ranges=missing,
            missing_count=sum(r.count for r in missing),
            first_chunk_us=first,
            last_chunk_us=last,
            data_rate_bytes_per_s=rate,
        )


class DataStore:
    """The payload data the station has received, reassembled by chunk ID (#158).

    Args:
        chunk_size_bytes: The configured chunk size, ``1..MAX_CHUNK_SIZE_BYTES``; must
            equal the spacecraft's ``PayloadConfig.chunk_size_bytes`` (ADR-0009 §6).

    Raises:
        TypeError: ``chunk_size_bytes`` is not an int.
        ValueError: ``chunk_size_bytes`` is out of range.
    """

    def __init__(self, chunk_size_bytes: int) -> None:
        if isinstance(chunk_size_bytes, bool) or not isinstance(chunk_size_bytes, int):
            raise TypeError(f"chunk_size_bytes must be an int, got {chunk_size_bytes!r}")
        if not 1 <= chunk_size_bytes <= MAX_CHUNK_SIZE_BYTES:
            raise ValueError(
                f"chunk_size_bytes must be in 1..{MAX_CHUNK_SIZE_BYTES}, got {chunk_size_bytes}"
            )
        self._chunk_size_bytes = chunk_size_bytes
        self._chunks: dict[int, bytes] = {}
        self._total = _Span()
        self._pass = _Span()

    @property
    def chunk_size_bytes(self) -> int:
        """The configured chunk size."""
        return self._chunk_size_bytes

    @property
    def statistics(self) -> DataStatistics:
        """DATA statistics since the station's last reset."""
        return self._total.snapshot()

    @property
    def pass_statistics(self) -> DataStatistics:
        """DATA statistics since the current pass began (:meth:`start_pass`)."""
        return self._pass.snapshot()

    def chunk_ids(self) -> tuple[int, ...]:
        """The IDs of every chunk stored, in ID order."""
        return tuple(sorted(self._chunks))

    def chunk(self, chunk_id: int) -> bytes | None:
        """The stored content of ``chunk_id``, or ``None`` if it was never stored."""
        return self._chunks.get(chunk_id)

    def start_pass(self) -> None:
        """Begin a new pass: the per-pass statistics start again from zero.

        Called through the downlink decoder at AOS (#161). Stored chunks are kept.
        """
        self._pass = _Span()

    def receive(
        self, now_us: int, sequence: int, chunk: DataChunk, *, snr_db: float | None = None
    ) -> DataChunkReceived:
        """Check, store and count one decoded DATA chunk.

        Args:
            now_us: Simulated time stamped on the event.
            sequence: The DATA frame's downlink header sequence.
            chunk: The decoded chunk.
            snr_db: The packet's SNR indicator (#163), carried on the event.

        Returns:
            The chunk's event: whether its content matched, and whether its ID was
            already stored.
        """
        size_bytes = len(chunk.content)
        content_ok = chunk.content == chunk_content(chunk.chunk_id, self._chunk_size_bytes)
        duplicate = chunk.chunk_id in self._chunks
        for span in (self._total, self._pass):
            span.receive(now_us, size_bytes)
            if not content_ok:
                span.corrupted_count += 1
            elif duplicate:
                span.duplicate_count += 1
            else:
                span.stored[chunk.chunk_id] = size_bytes
        if content_ok and not duplicate:
            self._chunks[chunk.chunk_id] = chunk.content
        return DataChunkReceived(
            now_us, sequence, chunk.chunk_id, content_ok, duplicate, snr_db=snr_db
        )
