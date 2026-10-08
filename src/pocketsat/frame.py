"""Wire frame encoding and decoding (ADR-0002).

A frame is the unit that crosses the ``TestTarget`` boundary. Layout, big-endian:

====================  =======  ============================================
Field                 Size     Notes
====================  =======  ============================================
Sync                  2        ``0xA5 0x5A``
Version               1        Protocol version, starts at 1
Type                  1        See :class:`FrameType`
Sequence              2        Per-direction counter, wraps
Length                2        Payload length in bytes
Payload               Length   Type-specific
CRC                   2        CRC-16/CCITT-FALSE over Version through Payload
====================  =======  ============================================

The same format is implemented in MCU firmware; both implementations are checked
against the shared vectors in ``tests/vectors/``.
"""

import binascii
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Final

SYNC: Final = 0xA55A
"""Two-byte sync marker at the start of every frame."""

PROTOCOL_VERSION: Final = 1
"""Current protocol version."""

HEADER_FORMAT: Final = ">HBBHH"
"""``struct`` format for sync, version, type, sequence, length."""

HEADER_SIZE: Final = struct.calcsize(HEADER_FORMAT)
CRC_SIZE: Final = 2
MIN_FRAME_SIZE: Final = HEADER_SIZE + CRC_SIZE
MAX_PAYLOAD_SIZE: Final = 0xFFFF
"""Largest payload the 2-byte length field can describe."""

MAX_SEQUENCE: Final = 0xFFFF


class FrameType(IntEnum):
    """Frame type codes carried in the header ``Type`` byte."""

    COMMAND = 0x01
    TELEMETRY = 0x02
    ACK = 0x03
    DATA = 0x04
    """Spacecraft to ground: one payload data chunk, its ID and content (#56)."""


class FrameError(ValueError):
    """Base class for all frame decoding errors."""


class FrameSyncError(FrameError):
    """The frame does not start with the sync marker ``0xA55A``."""


class FrameLengthError(FrameError):
    """The frame size does not match the header's length field."""


class FrameCrcError(FrameError):
    """The CRC in the frame does not match the computed CRC."""


class FrameTypeError(FrameError):
    """The header carries an unknown frame type code."""


@dataclass(frozen=True)
class Frame:
    """A decoded frame.

    Attributes:
        frame_type: Kind of frame.
        sequence: Per-direction sequence number, ``0..0xFFFF``.
        payload: Type-specific payload bytes.
        version: Protocol version, ``0..0xFF``.
    """

    frame_type: FrameType
    sequence: int
    payload: bytes = b""
    version: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        if not 0 <= self.sequence <= MAX_SEQUENCE:
            raise ValueError(f"sequence out of range 0..{MAX_SEQUENCE}: {self.sequence}")
        if not 0 <= self.version <= 0xFF:
            raise ValueError(f"version out of range 0..255: {self.version}")
        if len(self.payload) > MAX_PAYLOAD_SIZE:
            raise ValueError(f"payload exceeds {MAX_PAYLOAD_SIZE} bytes: {len(self.payload)}")


CRC16_POLY: Final = 0x1021
"""CRC-16/CCITT-FALSE generator polynomial (no reflection)."""

CRC16_INIT: Final = 0xFFFF
"""CRC-16/CCITT-FALSE initial value (no final XOR)."""


def _crc16_table() -> tuple[int, ...]:
    """The 256-entry lookup table: entry ``n`` is the CRC register after shifting the
    byte ``n`` through it bit by bit from zero."""
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC16_POLY) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC16_TABLE: Final = _crc16_table()


def crc16_ccitt_false(data: bytes) -> int:
    """Compute CRC-16/CCITT-FALSE (poly ``0x1021``, init ``0xFFFF``, no reflection, no xorout).

    The frame codec's CRC. It runs in C, as ``binascii.crc_hqx(data, 0xFFFF)``: the same
    polynomial, unreflected, no final XOR, with the initial value as its second argument.
    #56 switched to it from :func:`crc16_ccitt_false_table` (#78) because a DOWNLINK pass
    frames a 78-byte DATA frame every tick, and the table loop in Python cost about 10 us
    of the 100 us per-tick budget; the output is identical (checked against the table, a
    bitwise reference, and the shared vectors in ``tests/unit/test_frame.py``).

    Args:
        data: Bytes to checksum.

    Returns:
        The 16-bit CRC. The check value for ``b"123456789"`` is ``0x29B1``.
    """
    return binascii.crc_hqx(data, CRC16_INIT)


def crc16_ccitt_false_table(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, table-driven: the reference the MCU firmware mirrors (#78).

    One lookup per byte in a 256-entry table, integer-only, so portable (ADR-0006) and
    the same algorithm the Phase 7 firmware will use. Identical output to
    :func:`crc16_ccitt_false`, which the frame codec uses because it runs in C.

    Args:
        data: Bytes to checksum.

    Returns:
        The 16-bit CRC.
    """
    table = _CRC16_TABLE
    crc = CRC16_INIT
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ table[(crc >> 8) ^ byte]
    return crc


_BODY_HEADER: Final = struct.Struct(">" + HEADER_FORMAT[2:])
"""The header after the sync marker (version, type, sequence, length): the bytes the CRC
covers start here."""

_SYNC_BYTES: Final = SYNC.to_bytes(2, "big")

_FRAME_TYPES: Final = {frame_type.value: frame_type for frame_type in FrameType}
"""Frame type by code: a dict lookup, cheaper than calling the enum (#121)."""


def encode_frame(frame: Frame) -> bytes:
    """Encode a frame into its wire bytes.

    Args:
        frame: The frame to encode.

    Returns:
        Header, payload, and trailing big-endian CRC.
    """
    return _encode(frame.version, frame.frame_type, frame.sequence, frame.payload)


def encode_frame_fields(frame_type: FrameType, sequence: int, payload: bytes) -> bytes:
    """Encode a frame of the current protocol version from its fields.

    Part of the public codec, for any sender that has the fields in hand and no use for
    a :class:`Frame` object: the flight computer (up to three frames every tick, #121),
    and later senders such as the Phase 2 ground station's uplink. It gives the same
    bytes, and the same ``ValueError`` for an out-of-range field, as
    ``encode_frame(Frame(frame_type, sequence, payload))``, without building the
    :class:`Frame` (about 1 µs saved per frame). Use :func:`encode_frame` when you
    already hold a :class:`Frame` or need another protocol version.

    Args:
        frame_type: Kind of frame.
        sequence: Sequence number, ``0..0xFFFF``.
        payload: Type-specific payload bytes, at most :data:`MAX_PAYLOAD_SIZE`.

    Returns:
        Header, payload, and trailing big-endian CRC.

    Raises:
        ValueError: ``sequence`` is out of range or ``payload`` is too long.
    """
    if not 0 <= sequence <= MAX_SEQUENCE:
        raise ValueError(f"sequence out of range 0..{MAX_SEQUENCE}: {sequence}")
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD_SIZE} bytes: {len(payload)}")
    return _encode(PROTOCOL_VERSION, frame_type, sequence, payload)


def _encode(version: int, frame_type: FrameType, sequence: int, payload: bytes) -> bytes:
    body = _BODY_HEADER.pack(version, frame_type, sequence, len(payload)) + payload
    return _SYNC_BYTES + body + crc16_ccitt_false(body).to_bytes(CRC_SIZE, "big")


def decode_frame(data: bytes) -> Frame:
    """Decode exactly one frame from wire bytes.

    Checks run in order: sync, length, CRC, type.

    Args:
        data: The bytes of a single complete frame.

    Returns:
        The decoded frame.

    Raises:
        FrameSyncError: The sync marker is missing or wrong.
        FrameLengthError: ``data`` is shorter than a minimal frame, or its size does not
            match the header's length field.
        FrameCrcError: The CRC does not match.
        FrameTypeError: The type code is unknown.
    """
    if len(data) < 2 or int.from_bytes(data[:2], "big") != SYNC:
        raise FrameSyncError(f"bad sync: expected 0x{SYNC:04X}, got {data[:2].hex() or 'nothing'}")
    if len(data) < MIN_FRAME_SIZE:
        raise FrameLengthError(f"frame too short: {len(data)} bytes, minimum {MIN_FRAME_SIZE}")

    version, type_code, sequence, length = _BODY_HEADER.unpack_from(data, 2)
    expected_size = MIN_FRAME_SIZE + length
    if len(data) != expected_size:
        raise FrameLengthError(
            f"length field says {length}-byte payload ({expected_size}-byte frame), "
            f"got {len(data)} bytes"
        )

    received_crc = int.from_bytes(data[-CRC_SIZE:], "big")
    computed_crc = crc16_ccitt_false(data[2:-CRC_SIZE])
    if received_crc != computed_crc:
        raise FrameCrcError(
            f"bad CRC: frame has 0x{received_crc:04X}, computed 0x{computed_crc:04X}"
        )

    frame_type = _FRAME_TYPES.get(type_code)
    if frame_type is None:
        raise FrameTypeError(f"unknown frame type 0x{type_code:02X}")

    return Frame(frame_type, sequence, bytes(data[HEADER_SIZE:-CRC_SIZE]), version)
