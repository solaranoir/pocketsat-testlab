"""Payload subsystem (#43): science data acquisition and the chunk store.

The payload owns stored science data (ADR-0004 §13). While acquiring, bytes
accumulate at the configured true data rate; each time a full chunk's worth has
accumulated, a chunk with the next sequential ID is created. The buffer is capped at
its capacity. Chunks are deleted only when the flight computer releases them through
``controls.payload.release_through_chunk_id``; the payload never deletes data on its
own.

Because chunks are always created in ID order and released as a prefix (everything up
to and including an ID), the chunk store is exactly the contiguous ID range
``oldest_unreleased_chunk_id .. next_chunk_id - 1``, plus a partial chunk still
accumulating. Chunk content is a pure function of the chunk ID
(:func:`chunk_content`), so the store holds no bytes and a receiver can regenerate
and verify every chunk.

**Acquisition** happens only while ``controls.payload.enabled`` is true and no inhibit
applies. Inhibits are decisions, so they read the **readings** records of subsystems
earlier in ``STEP_ORDER`` (current tick, ADR-0004 §3 and §6). Acquisition is
inhibited while any of these holds:

- ``PowerReadings.low_battery`` or ``PowerReadings.critical_battery`` is set,
- ``ThermalReadings.over_temp`` or ``ThermalReadings.under_temp`` is set,
- ``AttitudeReadings.state`` is not ``AttitudeState.STABILIZED`` (``TUMBLING`` or
  ``DETUMBLING``).

The payload never switches itself on: with ``enabled`` false it is ``OFF`` whatever
the inhibits say. Arithmetic is integer bytes and microseconds (ADR-0006).
"""

import math
from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.base import SnapshotReader
from pocketsat.spacecraft.config import (
    CHUNK_ID_SIZE_BYTES,
    MAX_CHUNK_SIZE_BYTES,
    PayloadConfig,
    PayloadInitial,
)
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.snapshots import (
    AttitudeSnapshot,
    AttitudeState,
    PayloadReadings,
    PayloadSnapshot,
    PayloadState,
    PayloadTruth,
    PowerSnapshot,
    ThermalSnapshot,
)
from pocketsat.targets.base import EnvironmentState

__all__ = [
    "CHUNK_ID_SIZE_BYTES",
    "MAX_CHUNK_ID",
    "MAX_CHUNK_SIZE_BYTES",
    "Payload",
    "chunk_content",
]

MAX_CHUNK_ID: Final = 0xFFFF_FFFF
"""Largest chunk ID: IDs are carried as uint32 in DATA frames (#56)."""

_US_PER_S: Final = 1_000_000
_MASK32: Final = 0xFFFF_FFFF
_SEED_MULTIPLIER: Final = 0x9E37_79B1
_SEED_OFFSET: Final = 0x7F4A_7C15


def chunk_content(chunk_id: int, chunk_size_bytes: int) -> bytes:
    """Return the content of chunk ``chunk_id``: deterministic, no randomness.

    Shared with the DATA frames of #56, so a receiver can regenerate a chunk from its
    ID and detect corruption, loss, or duplication. Defined with 32-bit integer
    operations only, so it is reproducible bit for bit in C firmware:

    1. ``x = (chunk_id * 0x9E3779B1 + 0x7F4A7C15) mod 2**32``; if ``x`` is 0, use 1.
       (The multiplier is odd, so distinct IDs give distinct seeds.)
    2. Repeat: advance ``x`` with xorshift32 (``x ^= x << 13``, ``x ^= x >> 17``,
       ``x ^= x << 5``, each mod 2**32) and append ``x`` as 4 big-endian bytes.
    3. Truncate to ``chunk_size_bytes``.

    For example, ``chunk_content(0, 8).hex()`` is ``"29d04a5133d5399c"``.

    Args:
        chunk_id: Chunk ID, 0..:data:`MAX_CHUNK_ID`.
        chunk_size_bytes: Content length, bytes, 0..:data:`MAX_CHUNK_SIZE_BYTES`.

    Raises:
        TypeError: An argument is not an int.
        ValueError: An argument is out of range.
    """
    for name, value, high in (
        ("chunk_id", chunk_id, MAX_CHUNK_ID),
        ("chunk_size_bytes", chunk_size_bytes, MAX_CHUNK_SIZE_BYTES),
    ):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an int, got {value!r}")
        if not 0 <= value <= high:
            raise ValueError(f"{name} must be in 0..{high}, got {value}")
    x = (chunk_id * _SEED_MULTIPLIER + _SEED_OFFSET) & _MASK32 or 1
    out = bytearray()
    while len(out) < chunk_size_bytes:
        x ^= (x << 13) & _MASK32
        x ^= x >> 17
        x ^= (x << 5) & _MASK32
        out += x.to_bytes(4, "big")
    return bytes(out[:chunk_size_bytes])


class Payload:
    """The payload subsystem (#43). Implements the ``Subsystem`` protocol.

    Built with its settings record, its starting record (ADR-0005), and a
    :class:`SnapshotReader` for the power, thermal, and attitude readings it uses as
    inhibits (#85). ``reset(rng)`` returns to the starting record; the payload uses no
    randomness and requests no streams.

    Each step:

    1. Releases chunks through ``controls.payload.release_through_chunk_id``: every
       stored chunk up to and including that ID. Releasing a chunk that doesn't exist
       yet releases only the chunks that do; releasing one already released is a
       no-op. Only whole chunks are released, never the partial chunk.
    2. Decides the state: ``OFF`` unless ``controls.payload.enabled``; otherwise
       ``IDLE`` while an inhibit applies (see the module docstring) or the buffer is
       full, else ``ACQUIRING``.
    3. While acquiring, adds ``data_rate_bytes_per_s * dt`` bytes (integer bytes; the
       sub-byte remainder carries to the next acquiring tick), capped so the buffer
       never exceeds its capacity. Data that doesn't fit is not produced.

    At every tick ``total_produced_bytes == buffered_bytes + total_released_bytes``.

    Attributes:
        name: Always ``"payload"``.
    """

    name: Final = "payload"

    def __init__(
        self, config: PayloadConfig, initial: PayloadInitial, reader: SnapshotReader
    ) -> None:
        """Create the payload. Call :meth:`reset` before stepping.

        Args:
            config: Payload settings.
            initial: Starting state, restored by every :meth:`reset`.
            reader: Read-only access to other subsystems' snapshots.

        Raises:
            TypeError: ``config`` or ``initial`` is not the expected record type.
        """
        if not isinstance(config, PayloadConfig):
            raise TypeError(f"config must be a PayloadConfig, got {config!r}")
        if not isinstance(initial, PayloadInitial):
            raise TypeError(f"initial must be a PayloadInitial, got {initial!r}")
        self._config = config
        self._initial = initial
        self._reader = reader
        self._chunk = config.chunk_size_bytes
        self._capacity = config.buffer_capacity_bytes
        self._rate = config.data_rate_bytes_per_s
        self._idle_w = float(config.idle_power_w)
        self._acquiring_w = float(config.acquiring_power_w)
        self._restore()

    def _restore(self) -> None:
        start_bytes = math.floor(self._initial.buffer_fill * self._capacity)
        self._state = PayloadState.OFF
        self._oldest_id = 0
        self._next_id = start_bytes // self._chunk
        self._partial_bytes = start_bytes - self._next_id * self._chunk
        self._produced_bytes = start_bytes
        self._released_bytes = 0
        self._remainder_ubytes = 0
        self._power_w = 0.0
        self._snapshot = self._build_snapshot()

    def reset(self, rng: RngFactory) -> None:
        """Return to the starting record. The payload requests no random streams."""
        self._restore()

    @property
    def buffered_bytes(self) -> int:
        """Stored, unreleased bytes: whole chunks plus the partial chunk."""
        return (self._next_id - self._oldest_id) * self._chunk + self._partial_bytes

    def _inhibited(self) -> bool:
        reader = self._reader
        power = reader.get("power", PowerSnapshot).readings
        if power.low_battery or power.critical_battery:
            return True
        thermal = reader.get("thermal", ThermalSnapshot).readings
        if thermal.over_temp or thermal.under_temp:
            return True
        attitude = reader.get("attitude", AttitudeSnapshot).readings
        return attitude.state is not AttitudeState.STABILIZED

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Release, decide the state, and acquire for ``dt_us`` microseconds.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
            KeyError: Enabled, and power, thermal, or attitude has not published a
                snapshot to the reader.
        """
        check_us("dt_us", dt_us)
        changed = False
        commands = controls.payload

        release_id = commands.release_through_chunk_id
        if (
            release_id is not None
            and release_id >= self._oldest_id
            and self._oldest_id < self._next_id
        ):
            new_oldest = min(release_id + 1, self._next_id)
            self._released_bytes += (new_oldest - self._oldest_id) * self._chunk
            self._oldest_id = new_oldest
            changed = True

        if not commands.enabled:
            state = PayloadState.OFF
        elif self._inhibited() or self.buffered_bytes >= self._capacity:
            state = PayloadState.IDLE
        else:
            state = PayloadState.ACQUIRING
            ubytes = self._remainder_ubytes + self._rate * dt_us
            new_bytes = ubytes // _US_PER_S
            room = self._capacity - self.buffered_bytes
            if new_bytes >= room:
                new_bytes = room
                self._remainder_ubytes = 0
            else:
                self._remainder_ubytes = ubytes - new_bytes * _US_PER_S
            if new_bytes:
                partial = self._partial_bytes + new_bytes
                chunks = partial // self._chunk
                self._next_id += chunks
                self._partial_bytes = partial - chunks * self._chunk
                self._produced_bytes += new_bytes
                changed = True

        if state is not self._state:
            self._state = state
            if state is PayloadState.ACQUIRING:
                self._power_w = self._acquiring_w
            elif state is PayloadState.IDLE:
                self._power_w = self._idle_w
            else:
                self._power_w = 0.0
            changed = True
        if changed:
            self._snapshot = self._build_snapshot()

    def _build_snapshot(self) -> PayloadSnapshot:
        args = (
            self._state,
            self.buffered_bytes,
            self._capacity,
            self._oldest_id,
            self._next_id,
            self._produced_bytes,
            self._released_bytes,
            self._power_w,
        )
        return PayloadSnapshot(truth=PayloadTruth(*args), readings=PayloadReadings(*args))

    def snapshot(self) -> PayloadSnapshot:
        """Return the current state. Readings equal truth: the payload has no sensors.

        The snapshot is rebuilt only when something changed, so repeated calls in an
        unchanged state return the same object.
        """
        return self._snapshot
