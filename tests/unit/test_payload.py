"""Tests for the payload subsystem (#43).

Power, thermal, and attitude are shared fakes (#76) read through a ``SnapshotBoard``
(#85), stepped before the payload as in ``STEP_ORDER``.
"""

import dataclasses
from random import Random
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.frame import MAX_PAYLOAD_SIZE
from pocketsat.spacecraft import (
    CHUNK_ID_SIZE_BYTES,
    MAX_CHUNK_ID,
    MAX_CHUNK_SIZE_BYTES,
    NOMINAL_CONFIG,
    AttitudeState,
    Payload,
    PayloadConfig,
    PayloadControls,
    PayloadInitial,
    PayloadSnapshot,
    PayloadState,
    SnapshotBoard,
    SpacecraftControls,
    Subsystem,
    SubsystemStack,
    chunk_content,
)
from pocketsat.spacecraft.fakes import (
    FakeSubsystem,
    Script,
    default_snapshot,
    replace_readings,
    replace_truth,
)
from pocketsat.targets.base import EnvironmentState

ENV = EnvironmentState()
TICK_US = 100_000  # 100 ms

# 50 bytes/s at 100 ms ticks is 5 bytes per tick: one 10-byte chunk every 2 ticks, and
# the 100-byte buffer is full after 20 ticks.
CONFIG = PayloadConfig(
    buffer_capacity_bytes=100,
    chunk_size_bytes=10,
    data_rate_bytes_per_s=50,
    idle_power_w=0.25,
    acquiring_power_w=1.5,
)

ON = SpacecraftControls(payload=PayloadControls(enabled=True))
OFF = SpacecraftControls()


def controls(enabled: bool = True, release: int | None = None) -> SpacecraftControls:
    return SpacecraftControls(
        payload=PayloadControls(enabled=enabled, release_through_chunk_id=release)
    )


class Rig:
    """The payload plus fakes of power, thermal, and attitude, on one board."""

    def __init__(
        self,
        config: PayloadConfig = CONFIG,
        initial: PayloadInitial | None = None,
        scripts: dict[str, Script[Any]] | None = None,
    ) -> None:
        scripts = scripts or {}
        board = SnapshotBoard()
        self.fakes = {
            name: FakeSubsystem(name, default_snapshot(name), scripts.get(name))
            for name in ("power", "thermal", "attitude")
        }
        self.payload = Payload(config, initial or PayloadInitial(), reader=board)
        self.stack = SubsystemStack([*self.fakes.values(), self.payload], board=board)
        self.stack.reset(RngFactory(0))

    def step(self, ctl: SpacecraftControls = ON, n: int = 1) -> PayloadSnapshot:
        for _ in range(n):
            self.stack.step(TICK_US, ENV, ctl)
        return self.snap

    @property
    def snap(self) -> PayloadSnapshot:
        return self.stack.board.get("payload", PayloadSnapshot)


def check_invariants(snap: PayloadSnapshot, config: PayloadConfig = CONFIG) -> None:
    t = snap.truth
    assert t.total_produced_bytes == t.buffered_bytes + t.total_released_bytes
    assert 0 <= t.buffered_bytes <= config.buffer_capacity_bytes
    assert t.buffer_capacity_bytes == config.buffer_capacity_bytes
    assert t.oldest_unreleased_chunk_id <= t.next_chunk_id
    stored_chunks = t.next_chunk_id - t.oldest_unreleased_chunk_id
    partial = t.buffered_bytes - stored_chunks * config.chunk_size_bytes
    assert 0 <= partial < config.chunk_size_bytes
    assert t.total_released_bytes % config.chunk_size_bytes == 0
    assert dataclasses.astuple(snap.readings) == dataclasses.astuple(t)


# --- Settings and starting state -------------------------------------------------------


def test_nominal_config_has_payload_defaults() -> None:
    cfg = NOMINAL_CONFIG.payload
    assert cfg == PayloadConfig()
    assert cfg.buffer_capacity_bytes % cfg.chunk_size_bytes == 0
    assert 0 < cfg.chunk_size_bytes <= MAX_CHUNK_SIZE_BYTES
    assert cfg.data_rate_bytes_per_s > 0
    assert 0 <= cfg.idle_power_w <= cfg.acquiring_power_w


def test_chunk_size_limit_leaves_room_for_the_chunk_id() -> None:
    assert CHUNK_ID_SIZE_BYTES == 4
    assert MAX_CHUNK_SIZE_BYTES == MAX_PAYLOAD_SIZE - 4 == 65_531
    PayloadConfig(chunk_size_bytes=MAX_CHUNK_SIZE_BYTES, buffer_capacity_bytes=65_531)
    with pytest.raises(ValueError, match="chunk_size_bytes"):
        PayloadConfig(chunk_size_bytes=MAX_CHUNK_SIZE_BYTES + 1, buffer_capacity_bytes=65_532)


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"chunk_size_bytes": 0}, ValueError),
        ({"chunk_size_bytes": -1}, ValueError),
        ({"buffer_capacity_bytes": 0}, ValueError),
        ({"buffer_capacity_bytes": 65_537}, ValueError),  # not a multiple of 64
        ({"data_rate_bytes_per_s": -1}, ValueError),
        ({"idle_power_w": -0.1}, ValueError),
        ({"acquiring_power_w": float("nan")}, ValueError),
        ({"chunk_size_bytes": 64.0}, TypeError),
        ({"buffer_capacity_bytes": True}, TypeError),
        ({"data_rate_bytes_per_s": 1.5}, TypeError),
        ({"idle_power_w": "1"}, TypeError),
    ],
)
def test_config_validation(changes: dict[str, Any], error: type[Exception]) -> None:
    with pytest.raises(error):
        PayloadConfig(**changes)


def test_payload_is_a_subsystem() -> None:
    rig = Rig()
    assert isinstance(rig.payload, Subsystem)
    assert rig.payload.name == "payload"


def test_rejects_wrong_records() -> None:
    board = SnapshotBoard()
    with pytest.raises(TypeError):
        Payload(PayloadInitial(), PayloadInitial(), reader=board)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        Payload(CONFIG, CONFIG, reader=board)  # type: ignore[arg-type]


def test_default_start_is_empty_and_off() -> None:
    t = Rig().snap.truth
    assert t.state is PayloadState.OFF
    assert t.buffered_bytes == 0
    assert t.oldest_unreleased_chunk_id == 0
    assert t.next_chunk_id == 0
    assert t.total_produced_bytes == 0
    assert t.total_released_bytes == 0
    assert t.power_w == 0.0
    assert t.buffer_fill == 0.0


def test_non_empty_start_is_whole_chunks_from_id_0_plus_a_partial_chunk() -> None:
    rig = Rig(initial=PayloadInitial(buffer_fill=0.25))  # 25 bytes
    t = rig.snap.truth
    assert t.buffered_bytes == 25
    assert t.buffer_fill == 0.25
    assert (t.oldest_unreleased_chunk_id, t.next_chunk_id) == (0, 2)
    assert t.total_produced_bytes == 25
    check_invariants(rig.snap)
    snap = rig.step(n=1)  # 5 more bytes complete chunk 2
    assert snap.truth.next_chunk_id == 3


def test_reset_returns_to_the_starting_record() -> None:
    rig = Rig(initial=PayloadInitial(buffer_fill=0.5))
    start = rig.snap
    rig.step(n=7)
    rig.step(controls(release=3))
    assert rig.snap != start
    rig.stack.reset(RngFactory(1))
    assert rig.snap == start


# --- States and buffer growth ----------------------------------------------------------


def test_disabled_payload_stays_off_and_never_switches_itself_on() -> None:
    rig = Rig()
    for _ in range(30):
        t = rig.step(OFF).truth
        assert t.state is PayloadState.OFF
        assert t.buffered_bytes == 0
        assert t.power_w == 0.0


def test_buffer_grows_at_the_configured_rate_while_acquiring() -> None:
    rig = Rig()
    for tick in range(1, 11):
        t = rig.step().truth
        assert t.state is PayloadState.ACQUIRING
        assert t.power_w == CONFIG.acquiring_power_w
        assert t.buffered_bytes == 5 * tick
        assert t.total_produced_bytes == 5 * tick


def test_sub_byte_rate_carries_the_remainder() -> None:
    rig = Rig(config=dataclasses.replace(CONFIG, data_rate_bytes_per_s=3))  # 0.3 B/tick
    produced = [rig.step().truth.total_produced_bytes for _ in range(10)]
    assert produced == [0, 0, 0, 1, 1, 1, 2, 2, 2, 3]


def test_stops_growing_while_disabled_then_resumes() -> None:
    rig = Rig()
    rig.step(n=3)
    assert rig.step(OFF, n=5).truth.buffered_bytes == 15
    assert rig.step().truth.buffered_bytes == 20


def test_buffer_is_capped_at_capacity() -> None:
    rig = Rig()
    for _ in range(20):
        check_invariants(rig.step())
    assert rig.snap.truth.buffered_bytes == 100
    assert rig.snap.truth.buffer_fill == 1.0
    for _ in range(20):
        t = rig.step().truth
        assert t.buffered_bytes == 100
        assert t.total_produced_bytes == 100
        assert t.state is PayloadState.IDLE
        assert t.power_w == CONFIG.idle_power_w
        assert t.oldest_unreleased_chunk_id == 0  # the payload never deletes on its own
        assert t.next_chunk_id == 10


def test_last_tick_before_full_produces_only_what_fits() -> None:
    rig = Rig(config=dataclasses.replace(CONFIG, data_rate_bytes_per_s=70))  # 7 B/tick
    assert rig.step(n=14).truth.buffered_bytes == 98
    t = rig.step().truth
    assert (t.state, t.buffered_bytes, t.total_produced_bytes) == (PayloadState.ACQUIRING, 100, 100)
    assert rig.step().truth.state is PayloadState.IDLE


# --- Inhibits --------------------------------------------------------------------------

INHIBITS = [
    ("power", {"low_battery": True}),
    ("power", {"critical_battery": True}),
    ("thermal", {"over_temp": True}),
    ("thermal", {"under_temp": True}),
    ("attitude", {"state": AttitudeState.TUMBLING}),
    ("attitude", {"state": AttitudeState.DETUMBLING}),
]


@pytest.mark.parametrize(("name", "change"), INHIBITS, ids=lambda v: str(v))
def test_inhibit_stops_acquisition_and_acquisition_resumes_when_it_clears(
    name: str, change: dict[str, Any]
) -> None:
    nominal = default_snapshot(name)
    inhibited = replace_readings(nominal, **change)
    rig = Rig(scripts={name: {5: inhibited, 10: nominal}})
    for _ in range(5):
        assert rig.step().truth.state is PayloadState.ACQUIRING
    for _ in range(5):  # ticks 5..9: inhibit read as of the current tick
        t = rig.step().truth
        assert t.state is PayloadState.IDLE
        assert t.power_w == CONFIG.idle_power_w
        assert t.buffered_bytes == 25
    t = rig.step().truth  # tick 10: cleared, acquisition resumes
    assert t.state is PayloadState.ACQUIRING
    assert t.buffered_bytes == 30


def test_inhibit_while_disabled_is_off_not_idle() -> None:
    tumbling = replace_readings(default_snapshot("attitude"), state=AttitudeState.TUMBLING)
    rig = Rig(scripts={"attitude": {0: tumbling}})
    assert rig.step(OFF).truth.state is PayloadState.OFF


def test_inhibits_read_readings_not_truth() -> None:
    attitude = default_snapshot("attitude")
    # Truth tumbling, readings stabilized: the spacecraft only knows its readings.
    truth_only = replace_truth(attitude, state=AttitudeState.TUMBLING)
    assert Rig(scripts={"attitude": {0: truth_only}}).step().truth.state is (PayloadState.ACQUIRING)
    readings_only = replace_readings(attitude, state=AttitudeState.TUMBLING)
    assert Rig(scripts={"attitude": {0: readings_only}}).step().truth.state is PayloadState.IDLE


def test_flags_that_do_not_inhibit() -> None:
    # Only the documented readings inhibit; e.g. the heater running does not.
    heater = replace_truth(default_snapshot("thermal"), heater_on=True, heater_power_w=1.0)
    assert Rig(scripts={"thermal": {0: heater}}).step().truth.state is PayloadState.ACQUIRING


# --- Chunks and release ----------------------------------------------------------------


def test_chunks_are_created_with_sequential_ids() -> None:
    rig = Rig()
    next_ids = [rig.step().truth.next_chunk_id for _ in range(8)]
    assert next_ids == [0, 1, 1, 2, 2, 3, 3, 4]
    assert rig.snap.truth.oldest_unreleased_chunk_id == 0


def test_release_deletes_chunks_up_to_and_including_the_id() -> None:
    rig = Rig()
    rig.step(n=10)  # 50 bytes, chunks 0..4
    t = rig.step(controls(release=2)).truth  # +5 bytes this tick
    assert t.oldest_unreleased_chunk_id == 3
    assert t.next_chunk_id == 5
    assert t.total_released_bytes == 30
    assert t.buffered_bytes == 25
    check_invariants(rig.snap)


def test_releasing_already_released_chunks_is_a_no_op() -> None:
    rig = Rig()
    rig.step(n=10)
    rig.step(controls(release=2))
    before = rig.snap.truth
    for release in (2, 1, 0):  # repeated, and older than the oldest stored chunk
        t = rig.step(controls(enabled=False, release=release)).truth
        assert t.total_released_bytes == before.total_released_bytes
        assert t.oldest_unreleased_chunk_id == 3


def test_releasing_chunks_that_do_not_exist_yet_releases_only_existing_ones() -> None:
    rig = Rig()
    rig.step(n=5)  # 25 bytes: chunks 0, 1 and a 5-byte partial chunk
    t = rig.step(controls(enabled=False, release=1000)).truth
    assert (t.oldest_unreleased_chunk_id, t.next_chunk_id) == (2, 2)
    assert t.total_released_bytes == 20
    assert t.buffered_bytes == 5  # the partial chunk is never released
    # Chunk IDs keep counting; chunk 2 is not released by the earlier request.
    t = rig.step(controls(release=None)).truth
    assert (t.oldest_unreleased_chunk_id, t.next_chunk_id) == (2, 3)
    check_invariants(rig.snap)


def test_release_with_an_empty_store_is_a_no_op() -> None:
    rig = Rig()
    t = rig.step(controls(enabled=False, release=0)).truth
    assert (t.oldest_unreleased_chunk_id, t.next_chunk_id, t.total_released_bytes) == (0, 0, 0)


def test_release_frees_space_for_acquisition_in_the_same_tick() -> None:
    rig = Rig()
    rig.step(n=21)
    assert rig.snap.truth.state is PayloadState.IDLE
    t = rig.step(controls(release=0)).truth
    assert t.state is PayloadState.ACQUIRING
    assert t.buffered_bytes == 95
    assert t.total_produced_bytes == 105


# --- Chunk content ---------------------------------------------------------------------


def test_chunk_content_is_pinned() -> None:
    # Pinned so firmware and #56's receiver can check their implementation.
    assert chunk_content(0, 8).hex() == "29d04a5133d5399c"
    assert chunk_content(1, 8).hex() == "441daf1ace2c6b45"


def test_chunk_content_is_deterministic_and_distinct_per_id() -> None:
    contents = [chunk_content(i, 64) for i in range(500)]
    assert contents == [chunk_content(i, 64) for i in range(500)]
    assert len(set(contents)) == 500
    assert all(len(c) == 64 for c in contents)


def test_chunk_content_is_a_prefix_of_longer_content() -> None:
    assert chunk_content(7, 13) == chunk_content(7, 64)[:13]
    assert chunk_content(7, 0) == b""
    assert len(chunk_content(MAX_CHUNK_ID, MAX_CHUNK_SIZE_BYTES)) == MAX_CHUNK_SIZE_BYTES


def _xorshift_reference(chunk_id: int, chunk_size_bytes: int) -> bytes:
    """The definition in ``chunk_content``'s docstring, step by step (#43)."""
    x = (chunk_id * 0x9E3779B1 + 0x7F4A7C15) % 2**32 or 1
    out = b""
    while len(out) < chunk_size_bytes:
        x ^= (x << 13) % 2**32
        x ^= x >> 17
        x ^= (x << 5) % 2**32
        out += x.to_bytes(4, "big")
    return out[:chunk_size_bytes]


SEED_ZERO_CHUNK_ID = 0x7F4A7C15 * pow(0x9E3779B1, -1, 2**32) * -1 % 2**32
"""The chunk ID whose seed is 0 (so ``x`` starts at 1)."""


@pytest.mark.parametrize("chunk_size_bytes", [*range(0, 70), 255, 256, 257, 300, 1000])
def test_chunk_content_tables_match_the_definition(chunk_size_bytes: int) -> None:
    # #121 builds chunks of up to 256 bytes from GF(2) tables and runs the loop above
    # that: both must give the definition's bytes for every ID.
    rng = Random(chunk_size_bytes)
    ids = [0, 1, 2, 255, 256, 65535, 65536, SEED_ZERO_CHUNK_ID, MAX_CHUNK_ID]
    ids += [rng.randrange(MAX_CHUNK_ID + 1) for _ in range(200)]
    for chunk_id in ids:
        assert chunk_content(chunk_id, chunk_size_bytes) == _xorshift_reference(
            chunk_id, chunk_size_bytes
        ), chunk_id


def test_seed_zero_chunk_id_is_the_one_that_starts_at_one() -> None:
    assert (SEED_ZERO_CHUNK_ID * 0x9E3779B1 + 0x7F4A7C15) % 2**32 == 0


@pytest.mark.parametrize(
    ("args", "error"),
    [
        ((-1, 8), ValueError),
        ((MAX_CHUNK_ID + 1, 8), ValueError),
        ((0, MAX_CHUNK_SIZE_BYTES + 1), ValueError),
        ((0, -1), ValueError),
        ((1.0, 8), TypeError),
        ((True, 8), TypeError),
        ((0, 8.0), TypeError),
    ],
)
def test_chunk_content_validation(args: tuple[Any, Any], error: type[Exception]) -> None:
    with pytest.raises(error):
        chunk_content(*args)


# --- Accounting ------------------------------------------------------------------------


def test_produced_equals_buffered_plus_released_at_every_tick() -> None:
    script_rng = Random(43)
    states = [
        default_snapshot("attitude"),
        replace_readings(default_snapshot("attitude"), state=AttitudeState.TUMBLING),
    ]
    rig = Rig(
        config=dataclasses.replace(CONFIG, data_rate_bytes_per_s=73),
        initial=PayloadInitial(buffer_fill=0.37),
        scripts={"attitude": {tick: script_rng.choice(states) for tick in range(0, 2000, 37)}},
    )
    check_invariants(rig.snap)
    for _ in range(2000):
        oldest, nxt = rig.snap.truth.oldest_unreleased_chunk_id, rig.snap.truth.next_chunk_id
        release = None
        if script_rng.random() < 0.2:
            release = script_rng.randint(max(0, oldest - 3), nxt + 3)
        snap = rig.step(controls(enabled=script_rng.random() < 0.8, release=release))
        check_invariants(snap)
    t = rig.snap.truth
    assert t.total_produced_bytes > 0
    assert t.total_released_bytes > 0


def test_snapshot_readings_equal_truth() -> None:
    rig = Rig()
    snap = rig.step(n=3)
    assert isinstance(snap, PayloadSnapshot)
    assert type(snap).mirrored
    assert dataclasses.astuple(snap.readings) == dataclasses.astuple(snap.truth)
    assert rig.payload.snapshot() is snap


def test_rejects_negative_dt() -> None:
    rig = Rig()
    with pytest.raises(ValueError):
        rig.payload.step(-1, ENV, ON)
