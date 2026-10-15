"""Downlink decoding, sequence tracking and link statistics (#156)."""

import json
import random
from pathlib import Path
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.frame import FrameType, encode_frame_fields
from pocketsat.groundstation import (
    FrameRejected,
    GroundStation,
    LinkStatistics,
    RejectionReason,
    SequenceGap,
    SpacecraftRebooted,
    StationEvent,
    TelemetryReceived,
    UnmatchedAck,
)
from pocketsat.groundstation.downlink import DownlinkDecoder, SequenceResult, SequenceTracker
from pocketsat.messages import (
    CommandAck,
    CommandId,
    DataChunk,
    Packet,
    Telemetry,
    decode_telemetry,
    encode_ack,
)

VECTORS = Path(__file__).resolve().parents[1] / "vectors"
NOW_US = 100_000


def _load(name: str) -> dict[str, Any]:
    data: dict[str, Any] = json.loads((VECTORS / f"{name}.json").read_text(encoding="utf-8"))
    return data


FRAMES = _load("frames")
TELEMETRY = _load("telemetry")
DATA = _load("data")

TELEMETRY_PAYLOAD = bytes.fromhex(TELEMETRY["valid_telemetry"][0]["payload_hex"])
ACK_PAYLOAD = encode_ack(CommandAck(sequence=9, command_id=CommandId.PING, reason=None))

ERROR_REASONS = {
    "sync": RejectionReason.SYNC,
    "length": RejectionReason.LENGTH,
    "crc": RejectionReason.CRC,
    "type": RejectionReason.TYPE,
}


class RecordingDecoder(DownlinkDecoder):
    """A decoder whose hooks (#155, #157, #158) record what they are given."""

    def __init__(self) -> None:
        super().__init__()
        self.telemetry: list[Telemetry] = []
        self.acks: list[tuple[int, CommandAck]] = []
        self.chunks: list[tuple[int, DataChunk]] = []

    def on_telemetry(self, now_us: int, telemetry: Telemetry) -> None:
        self.telemetry.append(telemetry)

    def on_ack(self, now_us: int, sequence: int, ack: CommandAck) -> list[StationEvent]:
        self.acks.append((sequence, ack))
        return super().on_ack(now_us, sequence, ack)

    def on_data(self, now_us: int, sequence: int, chunk: DataChunk) -> None:
        self.chunks.append((sequence, chunk))


def _packet(frame_type: FrameType, sequence: int, payload: bytes) -> Packet:
    return Packet(frame=encode_frame_fields(frame_type, sequence, payload))


def _telemetry_payload(boot_count: int) -> bytes:
    # boot_count is the uint16 at offset 4 of the TELEMETRY payload (docs/protocol.md).
    return TELEMETRY_PAYLOAD[:4] + boot_count.to_bytes(2, "big") + TELEMETRY_PAYLOAD[6:]


def _telemetry(sequence: int, boot_count: int = 3) -> Packet:
    return _packet(FrameType.TELEMETRY, sequence, _telemetry_payload(boot_count))


def _ack(sequence: int) -> Packet:
    return _packet(FrameType.ACK, sequence, ACK_PAYLOAD)


def _decode_all(decoder: DownlinkDecoder, packets: list[Packet]) -> list[StationEvent]:
    events: list[StationEvent] = []
    for packet in packets:
        events.extend(decoder.decode(NOW_US, packet))
    return events


# --- Shared vectors -----------------------------------------------------------------


@pytest.mark.parametrize("vector", TELEMETRY["valid_telemetry"], ids=lambda v: v["name"])
def test_telemetry_vectors_decode(vector: dict[str, Any]) -> None:
    decoder = RecordingDecoder()
    events = decoder.decode(NOW_US, Packet(frame=bytes.fromhex(vector["frame_hex"])))
    expected = decode_telemetry(bytes.fromhex(vector["payload_hex"]))
    assert events == [TelemetryReceived(NOW_US, vector["sequence"], expected)]
    assert decoder.telemetry == [expected]


@pytest.mark.parametrize("vector", DATA["valid_data"], ids=lambda v: v["name"])
def test_data_vectors_decode_and_reach_the_data_hook(vector: dict[str, Any]) -> None:
    decoder = RecordingDecoder()
    events = decoder.decode(NOW_US, Packet(frame=bytes.fromhex(vector["frame_hex"])))
    # DataChunkReceived needs the chunk size for content_ok, so it is #158's to emit.
    assert events == []
    content = bytes.fromhex(vector["content_hex"])
    assert decoder.chunks == [(vector["sequence"], DataChunk(vector["chunk_id"], content))]
    assert decoder.statistics.rejected_count == 0


@pytest.mark.parametrize("vector", FRAMES["valid_frames"], ids=lambda v: v["name"])
def test_valid_frame_vectors_pass_frame_decoding(vector: dict[str, Any]) -> None:
    events = DownlinkDecoder().decode(NOW_US, Packet(frame=bytes.fromhex(vector["frame_hex"])))
    rejected = [e.reason for e in events if isinstance(e, FrameRejected)]
    if vector["type"] == "COMMAND":
        assert rejected == [RejectionReason.WRONG_DIRECTION]
    else:
        # The frame is valid; only its payload may not be one of its type.
        assert set(rejected) <= {RejectionReason.PAYLOAD}


@pytest.mark.parametrize("vector", FRAMES["invalid_frames"], ids=lambda v: v["name"])
def test_invalid_frame_vectors_are_rejected_and_counted_by_reason(
    vector: dict[str, Any],
) -> None:
    decoder = DownlinkDecoder()
    raw = bytes.fromhex(vector["frame_hex"])
    reason = ERROR_REASONS[vector["error"]]
    assert decoder.decode(NOW_US, Packet(frame=raw)) == [FrameRejected(NOW_US, reason, raw)]
    assert decoder.statistics == LinkStatistics(
        received_count=1, rejected_count=1, rejected_by_reason={reason: 1}
    )


@pytest.mark.parametrize("vector", TELEMETRY["invalid_payloads"], ids=lambda v: v["name"])
def test_invalid_telemetry_payloads_are_rejected_as_payload(vector: dict[str, Any]) -> None:
    packet = _packet(FrameType.TELEMETRY, 0, bytes.fromhex(vector["payload_hex"]))
    decoder = DownlinkDecoder()
    assert decoder.decode(NOW_US, packet) == [
        FrameRejected(NOW_US, RejectionReason.PAYLOAD, packet.frame)
    ]


@pytest.mark.parametrize("vector", DATA["invalid_data"], ids=lambda v: v["name"])
def test_invalid_data_payloads_are_rejected_as_payload(vector: dict[str, Any]) -> None:
    packet = _packet(FrameType.DATA, 0, bytes.fromhex(vector["payload_hex"]))
    decoder = RecordingDecoder()
    assert decoder.decode(NOW_US, packet) == [
        FrameRejected(NOW_US, RejectionReason.PAYLOAD, packet.frame)
    ]
    assert decoder.chunks == []


# --- ACK and wrong direction ---------------------------------------------------------


def test_acks_reach_the_ack_hook_without_events_when_no_handler() -> None:
    decoder = RecordingDecoder()
    assert decoder.decode(NOW_US, _ack(4)) == []
    assert decoder.acks == [(4, CommandAck(sequence=9, command_id=CommandId.PING, reason=None))]


def test_acks_go_to_the_ack_handler_and_its_event_follows() -> None:
    # The station's command tracker is the handler (#155).
    answered: list[tuple[int, CommandAck]] = []

    def handler(now_us: int, ack: CommandAck) -> StationEvent:
        answered.append((now_us, ack))
        return UnmatchedAck(now_us, ack.sequence, ack.command_id)

    decoder = DownlinkDecoder(ack_handler=handler)
    assert decoder.decode(NOW_US, _ack(4)) == [UnmatchedAck(NOW_US, 9, CommandId.PING)]
    assert answered == [(NOW_US, CommandAck(sequence=9, command_id=CommandId.PING, reason=None))]


def test_bad_ack_payload_is_rejected_as_payload() -> None:
    packet = _packet(FrameType.ACK, 0, b"\x00\x01")
    decoder = RecordingDecoder()
    assert decoder.decode(NOW_US, packet) == [
        FrameRejected(NOW_US, RejectionReason.PAYLOAD, packet.frame)
    ]
    assert decoder.acks == []


def test_command_frame_on_the_downlink_is_rejected_as_wrong_direction() -> None:
    # EchoTarget echoes uplink COMMAND frames back down.
    packet = _packet(FrameType.COMMAND, 0, bytes([CommandId.PING]))
    decoder = DownlinkDecoder()
    assert decoder.decode(NOW_US, packet) == [
        FrameRejected(NOW_US, RejectionReason.WRONG_DIRECTION, packet.frame)
    ]
    # Not a downlink frame: the sequence tracker never saw it.
    assert decoder.decode(NOW_US, _telemetry(5)) == [
        TelemetryReceived(NOW_US, 5, decode_telemetry(_telemetry_payload(3)))
    ]


# --- Sequence tracking ---------------------------------------------------------------


@pytest.mark.parametrize(
    ("sequences", "results"),
    [
        pytest.param([7], [SequenceResult()], id="first-frame-any-sequence"),
        pytest.param([0, 1, 2], [SequenceResult()] * 3, id="in-order"),
        pytest.param([0xFFFE, 0xFFFF, 0, 1], [SequenceResult()] * 4, id="wrap-without-loss"),
        pytest.param([3, 6], [SequenceResult(), SequenceResult(missing_count=2)], id="gap"),
        pytest.param(
            [0xFFFD, 1], [SequenceResult(), SequenceResult(missing_count=3)], id="gap-across-wrap"
        ),
        pytest.param([40, 0], [SequenceResult(), SequenceResult(restarted=True)], id="restart"),
        pytest.param(
            [40, 3],
            [SequenceResult(), SequenceResult(missing_count=3, restarted=True)],
            id="restart-with-new-frames-lost",
        ),
        pytest.param(
            [0xFFF0, 0], [SequenceResult(), SequenceResult(restarted=True)], id="zero-is-restart"
        ),
        # The expected sequence after 100 is 101: up to 0x7FFF ahead is a gap, from 0x8000
        # ahead the sequence went backwards.
        pytest.param(
            [100, 101 + 0x7FFF],
            [SequenceResult(), SequenceResult(missing_count=0x7FFF)],
            id="largest-forward-gap",
        ),
        pytest.param(
            [100, 101 + 0x8000],
            [SequenceResult(), SequenceResult(missing_count=101 + 0x8000, restarted=True)],
            id="half-window-reads-as-backwards",
        ),
    ],
)
def test_sequence_tracker(sequences: list[int], results: list[SequenceResult]) -> None:
    tracker = SequenceTracker()
    assert [tracker.observe(s) for s in sequences] == results


def test_gap_event_comes_before_the_content_and_is_counted() -> None:
    decoder = DownlinkDecoder()
    events = _decode_all(decoder, [_telemetry(10), _ack(11), _telemetry(14)])
    assert events == [
        TelemetryReceived(NOW_US, 10, decode_telemetry(_telemetry_payload(3))),
        SequenceGap(NOW_US, 14, 2),
        TelemetryReceived(NOW_US, 14, decode_telemetry(_telemetry_payload(3))),
    ]
    assert decoder.statistics.missing_count == 2


def test_rejected_frames_take_no_part_in_sequence_tracking() -> None:
    decoder = DownlinkDecoder()
    corrupted = bytearray(_telemetry(1).frame)
    corrupted[-1] ^= 0x01
    events = _decode_all(decoder, [_telemetry(0), Packet(frame=bytes(corrupted)), _telemetry(2)])
    assert [type(e) for e in events] == [
        TelemetryReceived,
        FrameRejected,
        SequenceGap,
        TelemetryReceived,
    ]
    gap = events[2]
    assert isinstance(gap, SequenceGap)
    assert gap.missing_count == 1  # the corrupted frame counts as missing too


def test_payload_rejections_keep_their_trusted_sequence() -> None:
    # The CRC vouched for the header, so a bad payload's sequence still counts.
    decoder = DownlinkDecoder()
    bad = _packet(FrameType.TELEMETRY, 1, b"\x00")
    events = _decode_all(decoder, [_telemetry(0), bad, _telemetry(2)])
    assert [type(e) for e in events] == [TelemetryReceived, FrameRejected, TelemetryReceived]


# --- Reboots -------------------------------------------------------------------------


def test_reboot_is_reported_after_the_telemetry_that_shows_it() -> None:
    decoder = DownlinkDecoder()
    events = _decode_all(
        decoder, [_telemetry(41, boot_count=3), _ack(42), _telemetry(0, boot_count=4)]
    )
    assert events[1:] == [
        TelemetryReceived(NOW_US, 0, decode_telemetry(_telemetry_payload(4))),
        SpacecraftRebooted(NOW_US, 0, 4),
    ]
    assert decoder.statistics.restart_count == 1
    assert decoder.statistics.missing_count == 0


def test_reboot_whose_first_frames_were_lost_counts_them_missing() -> None:
    decoder = DownlinkDecoder()
    events = _decode_all(decoder, [_telemetry(500, boot_count=3), _telemetry(2, boot_count=4)])
    assert events[1:] == [
        SequenceGap(NOW_US, 2, 2),
        TelemetryReceived(NOW_US, 2, decode_telemetry(_telemetry_payload(4))),
        SpacecraftRebooted(NOW_US, 2, 4),
    ]


def test_first_telemetry_and_unchanged_boot_count_report_no_reboot() -> None:
    decoder = DownlinkDecoder()
    events = _decode_all(decoder, [_telemetry(0, boot_count=7), _telemetry(1, boot_count=7)])
    assert not any(isinstance(e, SpacecraftRebooted) for e in events)


# --- Statistics and the station ------------------------------------------------------


def test_statistics_total_and_per_pass() -> None:
    decoder = DownlinkDecoder()
    _decode_all(decoder, [_telemetry(0), Packet(frame=b"junk"), _telemetry(3)])
    assert decoder.pass_statistics == decoder.statistics  # one pass until #161
    decoder.start_pass()
    _decode_all(decoder, [_packet(FrameType.COMMAND, 0, b"\x01"), _telemetry(5)])
    assert decoder.statistics == LinkStatistics(
        received_count=5,
        rejected_count=2,
        rejected_by_reason={RejectionReason.SYNC: 1, RejectionReason.WRONG_DIRECTION: 1},
        missing_count=3,
    )
    assert decoder.pass_statistics == LinkStatistics(
        received_count=2,
        rejected_count=1,
        rejected_by_reason={RejectionReason.WRONG_DIRECTION: 1},
        missing_count=1,
    )


def test_statistics_are_read_only_snapshots() -> None:
    decoder = DownlinkDecoder()
    decoder.decode(NOW_US, Packet(frame=b""))
    snapshot = decoder.statistics
    decoder.decode(NOW_US, Packet(frame=b""))
    assert snapshot.received_count == 1
    with pytest.raises(TypeError):
        snapshot.rejected_by_reason[RejectionReason.SYNC] = 9  # type: ignore[index]


def test_station_end_tick_decodes_in_arrival_order_and_reset_clears() -> None:
    station = GroundStation()
    downlink = [_telemetry(0), Packet(frame=b""), _telemetry(2)]
    events = station.end_tick(NOW_US, downlink)
    assert [type(e) for e in events] == [
        TelemetryReceived,
        FrameRejected,
        SequenceGap,
        TelemetryReceived,
    ]
    assert all(e.time_us == NOW_US for e in events)
    assert station.link_statistics.received_count == 3
    assert station.pass_link_statistics == station.link_statistics
    station.reset(RngFactory(1))
    assert station.link_statistics == LinkStatistics()
    # After a reset the next frame is a first frame again: no gap.
    assert station.end_tick(0, [_telemetry(9)]) == (
        TelemetryReceived(0, 9, decode_telemetry(_telemetry_payload(3))),
    )


def test_decoding_never_raises_for_arbitrary_bytes() -> None:
    rng = random.Random(156)
    decoder = DownlinkDecoder()
    valid = [_telemetry(0).frame, _ack(1).frame, bytes.fromhex(DATA["valid_data"][0]["frame_hex"])]
    for _ in range(2_000):
        if rng.random() < 0.5:
            raw = bytes(rng.randrange(256) for _ in range(rng.randrange(40)))
        else:
            mutated = bytearray(rng.choice(valid))
            mutated[rng.randrange(len(mutated))] ^= 1 << rng.randrange(8)
            raw = bytes(mutated)
        decoder.decode(NOW_US, Packet(frame=raw))
    assert decoder.statistics.received_count == 2_000
