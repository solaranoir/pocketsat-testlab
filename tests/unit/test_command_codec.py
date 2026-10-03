"""Unit tests for the COMMAND and ACK payload codec (#52) and its docs/protocol.md tables."""

import random
import re
import struct
from pathlib import Path

import pytest

from pocketsat.flight import COMMANDABLE_TARGETS, Mode, RejectReason
from pocketsat.messages import (
    ACK_FORMAT,
    ACK_PAYLOAD_SIZE,
    COMMAND_ARGUMENT_SIZES,
    NACK_REASONS,
    RESERVED_COMMAND_ID,
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

PROTOCOL = (Path(__file__).parents[2] / "docs" / "protocol.md").read_text(encoding="utf-8")

# --- Pinned wire codes ----------------------------------------------------------------

COMMAND_IDS = {
    "PING": 0x01,
    "SET_MODE": 0x02,
    "BEGIN_DOWNLINK": 0x03,
    "ENTER_SAFE_MODE": 0x04,
    "RESET": 0x05,
}
"""Command IDs (docs/protocol.md, #52), written out by hand so a renumbering fails."""

ALL_NACK_REASONS = {
    0x01: "UNKNOWN_COMMAND",
    0x02: "PAYLOAD_TRUNCATED",
    0x03: "PAYLOAD_TOO_LONG",
    0x04: "INVALID_ARGUMENT",
    0x10: "BOOT_IN_PROGRESS",
    0x11: "FAULT_REQUIRES_RESET",
    0x12: "NOT_ALLOWED_IN_SAFE",
    0x13: "SAFE_CONDITIONS_ACTIVE",
    0x14: "TARGET_NOT_COMMANDABLE",
}
"""Every NACK reason code (docs/protocol.md), written out by hand."""


def test_command_ids_are_pinned() -> None:
    assert {c.name: c.value for c in CommandId} == COMMAND_IDS
    assert RESERVED_COMMAND_ID == 0x00
    assert RESERVED_COMMAND_ID not in {c.value for c in CommandId}


def test_every_command_has_an_argument_size() -> None:
    assert set(COMMAND_ARGUMENT_SIZES) == set(CommandId)
    assert dict(COMMAND_ARGUMENT_SIZES) == {
        CommandId.PING: 0,
        CommandId.SET_MODE: 1,
        CommandId.BEGIN_DOWNLINK: 0,
        CommandId.ENTER_SAFE_MODE: 0,
        CommandId.RESET: 0,
    }


def test_nack_reason_codes_are_pinned() -> None:
    assert {code: r.name for code, r in NACK_REASONS.items()} == ALL_NACK_REASONS
    assert list(NACK_REASONS) == sorted(NACK_REASONS)


def test_reason_code_ranges_are_disjoint_and_0x00_is_reserved() -> None:
    # Decoding and argument errors use 0x01-0x0F; the mode state machine's reasons are
    # reused as they are, in 0x10-0x1F.
    assert all(0x01 <= r.value <= 0x0F for r in DecodeReason)
    assert all(0x10 <= r.value <= 0x1F for r in RejectReason)
    assert len(NACK_REASONS) == len(DecodeReason) + len(RejectReason)
    assert set(NACK_REASONS.values()) == {*DecodeReason, *RejectReason}
    assert 0x00 not in NACK_REASONS


def test_mode_rejections_are_the_state_machine_enum() -> None:
    for code in range(0x10, 0x20):
        if code in NACK_REASONS:
            assert NACK_REASONS[code] is RejectReason(code)


# --- Encoding commands ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("command", "payload"),
    [
        (Command.ping(), b"\x01"),
        (Command.set_mode(Mode.SCIENCE), b"\x02\x02"),
        (Command.begin_downlink(), b"\x03"),
        (Command.enter_safe_mode(), b"\x04"),
        (Command.reset(), b"\x05"),
        (Command(0xFF, b"\x00\x01"), b"\xff\x00\x01"),
    ],
)
def test_encode_command(command: Command, payload: bytes) -> None:
    assert encode_command(command) == payload


@pytest.mark.parametrize("mode", list(Mode))
def test_set_mode_carries_the_mode_value(mode: Mode) -> None:
    payload = encode_command(Command.set_mode(mode))
    assert payload == bytes([CommandId.SET_MODE, mode.value])
    assert decode_command(payload) == ParsedCommand(CommandId.SET_MODE, mode)


@pytest.mark.parametrize("command", [Command.ping(), Command.reset(), Command.set_mode(Mode.SAFE)])
def test_constructors_build_well_formed_commands(command: Command) -> None:
    assert isinstance(decode_command(encode_command(command)), ParsedCommand)


@pytest.mark.parametrize(
    ("command_id", "payload", "error"),
    [
        (-1, b"", ValueError),
        (0x100, b"", ValueError),
        (True, b"", TypeError),
        (1.0, b"", TypeError),
        (1, bytearray(b"\x00"), TypeError),
        (1, "00", TypeError),
    ],
)
def test_command_rejects_bad_fields(command_id: object, payload: object, error: type) -> None:
    with pytest.raises(error):
        Command(command_id, payload)  # type: ignore[arg-type]


def test_set_mode_requires_a_mode() -> None:
    with pytest.raises(TypeError):
        Command.set_mode(1)  # type: ignore[arg-type]


# --- Decoding and validation ----------------------------------------------------------


@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        (b"", MalformedCommand(0x00, DecodeReason.PAYLOAD_TRUNCATED)),
        (b"\x00", MalformedCommand(0x00, DecodeReason.UNKNOWN_COMMAND)),
        (b"\x06", MalformedCommand(0x06, DecodeReason.UNKNOWN_COMMAND)),
        (b"\x06\x00\x00", MalformedCommand(0x06, DecodeReason.UNKNOWN_COMMAND)),
        (b"\x02", MalformedCommand(0x02, DecodeReason.PAYLOAD_TRUNCATED)),
        (b"\x02\x01\x01", MalformedCommand(0x02, DecodeReason.PAYLOAD_TOO_LONG)),
        (b"\x02\x09\x01", MalformedCommand(0x02, DecodeReason.PAYLOAD_TOO_LONG)),
        (b"\x02\x06", MalformedCommand(0x02, DecodeReason.INVALID_ARGUMENT)),
        (b"\x01\x00", MalformedCommand(0x01, DecodeReason.PAYLOAD_TOO_LONG)),
        (b"\x05\x00", MalformedCommand(0x05, DecodeReason.PAYLOAD_TOO_LONG)),
    ],
)
def test_decode_reports_the_first_problem(payload: bytes, expected: MalformedCommand) -> None:
    assert decode_command(payload) == expected


@pytest.mark.parametrize("mode", sorted(set(Mode) - COMMANDABLE_TARGETS))
def test_non_commandable_mode_decodes_for_the_state_machine_to_reject(mode: Mode) -> None:
    # A real mode that SET_MODE may not request is TARGET_NOT_COMMANDABLE (0x14), decided
    # by the state machine, not INVALID_ARGUMENT.
    assert decode_command(bytes([0x02, mode])) == ParsedCommand(CommandId.SET_MODE, mode)


def _all_payloads_up_to_two_bytes() -> list[bytes]:
    return (
        [b""]
        + [bytes([a]) for a in range(256)]
        + [bytes([a, b]) for a in range(256) for b in range(256)]
    )


def test_decode_never_raises_and_classifies_every_short_payload() -> None:
    # Exhaustive over every payload of 0, 1, and 2 bytes, checked against the rules.
    ids = {c.value for c in CommandId}
    modes = {m.value for m in Mode}
    for payload in _all_payloads_up_to_two_bytes():
        result = decode_command(payload)
        if not payload:
            assert result == MalformedCommand(0, DecodeReason.PAYLOAD_TRUNCATED)
            continue
        code, size = payload[0], len(payload) - 1
        if code not in ids:
            assert result == MalformedCommand(code, DecodeReason.UNKNOWN_COMMAND)
        elif size < COMMAND_ARGUMENT_SIZES[CommandId(code)]:
            assert result == MalformedCommand(code, DecodeReason.PAYLOAD_TRUNCATED)
        elif size > COMMAND_ARGUMENT_SIZES[CommandId(code)]:
            assert result == MalformedCommand(code, DecodeReason.PAYLOAD_TOO_LONG)
        elif code == CommandId.SET_MODE and payload[1] not in modes:
            assert result == MalformedCommand(code, DecodeReason.INVALID_ARGUMENT)
        else:
            assert isinstance(result, ParsedCommand)
            assert result.command_id == code


def test_decode_never_raises_on_random_payloads() -> None:
    rng = random.Random(52)
    for _ in range(2000):
        payload = rng.randbytes(rng.randrange(3, 300))
        result = decode_command(payload)
        assert isinstance(result, MalformedCommand)
        assert result.command_id == payload[0]
        assert result.reason in (DecodeReason.UNKNOWN_COMMAND, DecodeReason.PAYLOAD_TOO_LONG)


def test_every_decode_reason_is_reachable() -> None:
    reasons = {
        r.reason
        for p in _all_payloads_up_to_two_bytes()
        if isinstance(r := decode_command(p), MalformedCommand)
    }
    assert reasons == set(DecodeReason)


@pytest.mark.parametrize(
    ("command_id", "target", "error"),
    [
        (CommandId.SET_MODE, None, ValueError),
        (CommandId.PING, Mode.NOMINAL, ValueError),
        (2, Mode.NOMINAL, TypeError),
        (CommandId.SET_MODE, 1, TypeError),
    ],
)
def test_parsed_command_validates(command_id: object, target: object, error: type) -> None:
    with pytest.raises(error):
        ParsedCommand(command_id, target)  # type: ignore[arg-type]


# --- ACK payloads ---------------------------------------------------------------------


def test_ack_layout() -> None:
    assert struct.calcsize(ACK_FORMAT) == ACK_PAYLOAD_SIZE == 4
    assert encode_ack(CommandAck(0x0102, 0x02)) == b"\x01\x02\x02\x00"
    nack = CommandAck(0x0102, 0x02, RejectReason.TARGET_NOT_COMMANDABLE)
    assert encode_ack(nack) == b"\x01\x02\x02\x14"
    assert not nack.accepted


@pytest.mark.parametrize("reason", [None, *NACK_REASONS.values()])
def test_ack_round_trip(reason: DecodeReason | RejectReason | None) -> None:
    ack = CommandAck(65535, 0x7F, reason)
    assert decode_ack(encode_ack(ack)) == ack
    assert ack.accepted == (reason is None)


@pytest.mark.parametrize("code", sorted(set(range(1, 256)) - set(NACK_REASONS)))
def test_decode_ack_rejects_unknown_reason_codes(code: int) -> None:
    with pytest.raises(AckDecodeError):
        decode_ack(bytes([0, 1, 1, code]))


@pytest.mark.parametrize(
    ("sequence", "command_id", "reason", "error"),
    [
        (-1, 1, None, ValueError),
        (0x10000, 1, None, ValueError),
        (0, 0x100, None, ValueError),
        (True, 1, None, TypeError),
        (0, 1, 0x10, TypeError),
    ],
)
def test_command_ack_validates(
    sequence: object, command_id: object, reason: object, error: type
) -> None:
    with pytest.raises(error):
        CommandAck(sequence, command_id, reason)  # type: ignore[arg-type]


def test_decode_ack_rejects_wrong_sizes() -> None:
    for size in (0, 3, 5, 26):
        with pytest.raises(AckDecodeError):
            decode_ack(bytes(size))


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


def test_docs_command_id_table_matches_the_enum() -> None:
    rows = _table_after("### Command IDs")
    documented = {_code(row[1]): (int(_code(row[0]), 16), int(row[2])) for row in rows}
    assert documented == {c.name: (c.value, COMMAND_ARGUMENT_SIZES[c]) for c in CommandId}


def test_docs_reason_code_table_matches_the_enums() -> None:
    rows = _table_after("### NACK reason codes")
    documented = [(int(_code(row[0]), 16), _code(row[1]), _code(row[2])) for row in rows]
    expected = [(code, r.name, type(r).__name__) for code, r in NACK_REASONS.items()]
    assert documented == expected


def test_docs_ack_layout_table_matches_the_format() -> None:
    rows = _table_after("### ACK payload")
    types = {"uint16": "H", "uint8": "B"}
    assert ">" + "".join(types[row[2]] for row in rows) == ACK_FORMAT
    sizes = [struct.calcsize(">" + types[row[2]]) for row in rows]
    assert [int(row[0]) for row in rows] == [sum(sizes[:i]) for i in range(len(sizes))]


def test_docs_command_layout_table() -> None:
    rows = _table_after("### COMMAND payload")
    assert [(row[0], _code(row[1]), row[2]) for row in rows] == [
        ("0", "command_id", "uint8"),
        ("1", "arguments", "per command"),
    ]
