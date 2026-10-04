"""Check the command and ACK codec against the shared vectors in tests/vectors/commands.json.

The vectors were generated independently of ``pocketsat.messages`` (see
``tests/vectors/README.md``), so these tests pin the wire contract, not the code.
"""

import json
from pathlib import Path
from typing import Any

import pytest

from pocketsat.flight import Mode, RejectReason
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    ACK_PAYLOAD_SIZE,
    COMMAND_ARGUMENT_SIZES,
    NACK_REASONS,
    AckDecodeError,
    Command,
    CommandAck,
    CommandId,
    DecodeReason,
    MalformedCommand,
    ParsedCommand,
    decode_ack,
    decode_command,
    encode_ack,
    encode_command,
)

VECTORS: dict[str, Any] = json.loads(
    (Path(__file__).parents[1] / "vectors" / "commands.json").read_text()
)


def _ids(key: str) -> list[str]:
    return [v["name"] for v in VECTORS[key]]


def _reason(name: str | None) -> DecodeReason | RejectReason | None:
    if name is None:
        return None
    return DecodeReason[name] if name in DecodeReason.__members__ else RejectReason[name]


# --- Code tables ----------------------------------------------------------------------


def test_command_ids_match() -> None:
    assert VECTORS["command_ids"] == {c.name: c.value for c in CommandId}


def test_argument_sizes_match() -> None:
    assert VECTORS["argument_sizes"] == {c.name: n for c, n in COMMAND_ARGUMENT_SIZES.items()}


def test_nack_reasons_match() -> None:
    assert VECTORS["nack_reasons"] == {r.name: code for code, r in NACK_REASONS.items()}


def test_ack_sizes_match() -> None:
    assert VECTORS["ack_payload_size"] == ACK_PAYLOAD_SIZE
    assert VECTORS["ack_frame_size"] == ACK_FRAME_SIZE


# --- COMMAND payloads -----------------------------------------------------------------


def _command(vector: dict[str, Any]) -> Command:
    if "target" in vector:
        return Command.set_mode(Mode[vector["target"]])
    return Command(CommandId[vector["command"]])


@pytest.mark.parametrize("vector", VECTORS["valid_commands"], ids=_ids("valid_commands"))
def test_encode_valid_command(vector: dict[str, Any]) -> None:
    payload = encode_command(_command(vector))
    assert payload.hex() == vector["payload_hex"]
    frame = Frame(FrameType.COMMAND, vector["sequence"], payload)
    assert encode_frame(frame).hex() == vector["frame_hex"]


@pytest.mark.parametrize("vector", VECTORS["valid_commands"], ids=_ids("valid_commands"))
def test_decode_valid_command(vector: dict[str, Any]) -> None:
    raw = bytes.fromhex(vector["frame_hex"])
    frame = decode_frame(raw)
    assert frame.frame_type is FrameType.COMMAND
    assert frame.sequence == vector["sequence"]
    assert int.from_bytes(raw[-2:], "big") == vector["crc"]
    target = Mode[vector["target"]] if "target" in vector else None
    assert decode_command(frame.payload) == ParsedCommand(CommandId[vector["command"]], target)


@pytest.mark.parametrize("vector", VECTORS["invalid_commands"], ids=_ids("invalid_commands"))
def test_decode_invalid_command(vector: dict[str, Any]) -> None:
    frame = decode_frame(bytes.fromhex(vector["frame_hex"]))
    assert frame.payload.hex() == vector["payload_hex"]
    assert frame.sequence == vector["sequence"]
    expected = MalformedCommand(vector["command_id"], DecodeReason[vector["reason"]])
    assert decode_command(frame.payload) == expected


@pytest.mark.parametrize("vector", VECTORS["invalid_commands"], ids=_ids("invalid_commands"))
def test_invalid_command_can_be_sent(vector: dict[str, Any]) -> None:
    # The ground side can build every malformed payload on purpose, except an empty one,
    # which is a COMMAND frame with no payload at all.
    payload = bytes.fromhex(vector["payload_hex"])
    if payload:
        assert encode_command(Command(payload[0], payload[1:])) == payload


# --- ACK payloads ---------------------------------------------------------------------


def _ack(vector: dict[str, Any]) -> CommandAck:
    return CommandAck(vector["sequence"], vector["command_id"], _reason(vector["reason"]))


@pytest.mark.parametrize("vector", VECTORS["acks"], ids=_ids("acks"))
def test_encode_ack(vector: dict[str, Any]) -> None:
    payload = encode_ack(_ack(vector))
    assert payload.hex() == vector["payload_hex"]
    frame = Frame(FrameType.ACK, vector["frame_sequence"], payload)
    assert encode_frame(frame).hex() == vector["frame_hex"]


@pytest.mark.parametrize("vector", VECTORS["acks"], ids=_ids("acks"))
def test_decode_ack(vector: dict[str, Any]) -> None:
    raw = bytes.fromhex(vector["frame_hex"])
    assert len(raw) == ACK_FRAME_SIZE
    frame = decode_frame(raw)
    assert frame.frame_type is FrameType.ACK
    assert frame.sequence == vector["frame_sequence"]
    assert int.from_bytes(raw[-2:], "big") == vector["crc"]
    ack = decode_ack(frame.payload)
    assert ack == _ack(vector)
    assert ack.accepted == (vector["reason"] is None)


def test_every_reason_has_an_ack_vector() -> None:
    covered = {v["reason"] for v in VECTORS["acks"] if v["reason"] is not None}
    assert covered == {r.name for r in NACK_REASONS.values()}


@pytest.mark.parametrize("vector", VECTORS["invalid_acks"], ids=_ids("invalid_acks"))
def test_decode_invalid_ack(vector: dict[str, Any]) -> None:
    match = {"size": "bytes", "reason": "reason"}[vector["error"]]
    with pytest.raises(AckDecodeError, match=match):
        decode_ack(bytes.fromhex(vector["payload_hex"]))
