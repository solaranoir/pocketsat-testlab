"""Golden telemetry: regenerate from the recorded seed, compare byte for byte (#64).

The captures in ``tests/golden/`` hold every downlink frame of a scenario for one seed
(format, scenarios, and the regeneration command: ``_golden.py`` and
``tests/golden/README.md``). ADR-0006 makes SIL arithmetic portable, so the same bytes
come out on Linux and macOS; CI runs these tests on both.

- **Replay:** each capture is regenerated from the seed in its header and compared
  frame by frame, tick and bytes; a failure names the first differing frame, its tick,
  and both frames' decoded fields. The nominal capture is a whole 92-minute orbit
  (about 4 s), so it is ``slow``; the ``sensor_freeze`` capture is fast.
- **What the bytes show,** read from the stored captures (fast): ADR-0004's
  within-tick order and latencies, the nominal mode sequence, seeded noise in the
  reported values, and ``sensor_freeze`` holding them.
- **The comparison itself:** a changed byte, a shifted tick, and a missing or extra
  frame are each reported as such.
"""

import struct
from itertools import pairwise

import pytest
from _golden import (
    FREEZE_START_TICK,
    FREEZE_US,
    NOMINAL_TICK,
    PASS_START_TICK,
    PING_TICK,
    SCENARIOS,
    SCIENCE_TICK,
    TICK_US,
    Capture,
    DownlinkFrame,
    GoldenFormatError,
    GoldenScenario,
    check,
    first_difference,
    frames_sha256,
    load,
    parse,
    render,
)

from pocketsat.flight import Mode
from pocketsat.frame import PROTOCOL_VERSION, Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    TELEMETRY_FIELDS,
    TELEMETRY_FORMAT,
    CommandAck,
    CommandId,
    Telemetry,
    decode_ack,
    decode_data,
    decode_telemetry,
)
from pocketsat.spacecraft import PayloadState

NOMINAL = SCENARIOS["nominal_seed42"]
FREEZE = SCENARIOS["sensor_freeze_seed42"]

REPORTED = ("bus_v", "soc", "battery_c", "electronics_c", "pointing_error_deg")
"""The telemetry fields that carry sensor readings: seeded noise, frozen by
``sensor_freeze`` (ADR-0004 §6 to §8)."""


def _types(capture: Capture, frame_type: FrameType) -> list[tuple[int, bytes]]:
    """``(tick, payload)`` of the frames of one type, in order."""
    out = []
    for tick, frame in capture.frames:
        decoded = decode_frame(frame)
        if decoded.frame_type is frame_type:
            out.append((tick, decoded.payload))
    return out


def _telemetry(capture: Capture) -> list[tuple[int, Telemetry]]:
    return [(tick, decode_telemetry(p)) for tick, p in _types(capture, FrameType.TELEMETRY)]


def _acks(capture: Capture) -> list[tuple[int, CommandAck]]:
    return [(tick, decode_ack(p)) for tick, p in _types(capture, FrameType.ACK)]


def _after(telemetry: list[tuple[int, Telemetry]], tick: int) -> Telemetry:
    """The first telemetry frame after ``tick``."""
    return next(t for n, t in telemetry if n > tick)


def _at(telemetry: list[tuple[int, Telemetry]], tick: int) -> Telemetry:
    (frame,) = [t for n, t in telemetry if n == tick]
    return frame


# --- Replay: byte for byte ---------------------------------------------------------------


def _assert_replays(scenario: GoldenScenario) -> None:
    report = check(scenario)
    assert report is None, (
        f"{report}\n\nIf this change is deliberate (tests/golden/README.md), regenerate "
        f"with: uv run python tests/sil/_golden.py --regenerate {scenario.name}"
    )


@pytest.mark.slow
def test_nominal_capture_replays_byte_for_byte() -> None:
    assert NOMINAL.slow
    _assert_replays(NOMINAL)


def test_sensor_freeze_capture_replays_byte_for_byte() -> None:
    assert not FREEZE.slow
    _assert_replays(FREEZE)


@pytest.mark.parametrize("scenario", SCENARIOS.values(), ids=list(SCENARIOS))
def test_capture_file_is_consistent(scenario: GoldenScenario) -> None:
    # Guards against hand edits: the stored file is exactly what render() writes for
    # its frames, its digest matches, and every frame decodes.
    text = scenario.path.read_text(encoding="utf-8")
    capture = parse(text)
    assert capture.header["format"] == "1"
    assert capture.header["seed"] == str(scenario.seed)
    assert capture.header["protocol_version"] == str(PROTOCOL_VERSION)
    assert capture.header["frames"] == str(len(capture.frames))
    assert capture.header["frames_sha256"] == frames_sha256(capture.frames)
    assert render(scenario, capture.frames) == text
    ticks = [tick for tick, _ in capture.frames]
    assert ticks == sorted(ticks) and ticks[0] >= 0 and ticks[-1] < scenario.ticks
    assert all(decode_frame(frame).version == PROTOCOL_VERSION for _, frame in capture.frames)


# --- What the nominal capture shows (read from the file) ---------------------------------


def test_nominal_capture_follows_the_nominal_timeline() -> None:
    capture = load(NOMINAL)
    telemetry = _telemetry(capture)
    modes = [t.mode for _, t in telemetry]
    sequence = [m for i, m in enumerate(modes) if i == 0 or m != modes[i - 1]]
    # BOOT sends no telemetry; the first frame is the boot beacon in NOMINAL. The pass
    # ends at DOWNLINK_COMPLETE, back to SCIENCE, before the ground commands NOMINAL.
    assert sequence == [Mode.NOMINAL, Mode.SCIENCE, Mode.DOWNLINK, Mode.SCIENCE, Mode.NOMINAL]
    assert Mode.SAFE not in modes and Mode.FAULT not in modes
    assert all(t.boot_count == telemetry[0][1].boot_count for _, t in telemetry)
    # Telemetry about once a second throughout (1 s in NOMINAL, SCIENCE and DOWNLINK).
    gaps = [b - a for (a, _), (b, _) in pairwise(telemetry)]
    assert max(gaps) <= 1_000_000 // TICK_US
    # Data was acquired in SCIENCE and downlinked in the pass, in chunk order.
    chunks = [decode_data(p).chunk_id for _, p in _types(capture, FrameType.DATA)]
    assert len(chunks) > 1_000
    assert chunks == list(range(chunks[0], chunks[0] + len(chunks)))
    assert any(t.payload_state is PayloadState.ACQUIRING for _, t in telemetry)


def test_nominal_capture_reflects_adr_0004_order_and_latencies() -> None:
    capture = load(NOMINAL)
    telemetry = _telemetry(capture)

    # Command -> ACK in the same tick (ADR-0004 §2), in command order.
    commands = [
        (PING_TICK, CommandId.PING, None),
        (SCIENCE_TICK, CommandId.SET_MODE, Mode.SCIENCE),
        (PASS_START_TICK, CommandId.BEGIN_DOWNLINK, Mode.DOWNLINK),
        (NOMINAL_TICK, CommandId.SET_MODE, Mode.NOMINAL),
    ]
    acks = _acks(capture)
    assert [(tick, ack.sequence, ack.command_id, ack.accepted) for tick, ack in acks] == [
        (tick, i, command_id, True) for i, (tick, command_id, _) in enumerate(commands, 1)
    ]

    # The mode is in the ACK tick's telemetry (steps c and d before e); the physical
    # effect, here the payload's state, lands one tick later, so that frame lags by one
    # (ADR-0004 §2, "Telemetry can lag by one frame").
    lag = {
        Mode.SCIENCE: (PayloadState.OFF, PayloadState.IDLE),  # powered on, detumbling
        Mode.DOWNLINK: (PayloadState.ACQUIRING, PayloadState.OFF),  # off for the pass
        Mode.NOMINAL: (PayloadState.ACQUIRING, PayloadState.OFF),
    }
    for tick, _, mode in commands[1:]:
        assert mode is not None
        same_tick, next_frame = _at(telemetry, tick), _after(telemetry, tick)
        assert same_tick.mode is mode and next_frame.mode is mode
        assert (same_tick.payload_state, next_frame.payload_state) == lag[mode]

    # Within each tick, frames go out in ADR-0004 §10's priority order: ACK/NACK, then
    # telemetry, then DATA.
    priority = {FrameType.ACK: 0, FrameType.TELEMETRY: 1, FrameType.DATA: 2}
    per_tick: dict[int, list[int]] = {}
    for tick, frame in capture.frames:
        per_tick.setdefault(tick, []).append(priority[decode_frame(frame).frame_type])
    assert all(order == sorted(order) for order in per_tick.values())
    kinds = {tuple(sorted(set(order))) for order in per_tick.values()}
    assert {(0, 1), (1, 2)} <= kinds  # ACK with telemetry, and telemetry with DATA

    # DATA only while in DOWNLINK: never before BEGIN_DOWNLINK's tick, and not after
    # the pass ended.
    data_ticks = [tick for tick, _ in _types(capture, FrameType.DATA)]
    end_of_pass = next(n for n, t in telemetry if n > PASS_START_TICK and t.mode is Mode.SCIENCE)
    assert data_ticks[0] >= PASS_START_TICK and data_ticks[-1] < end_of_pass


def test_nominal_capture_carries_seeded_noise() -> None:
    # Reported values, not truth (ADR-0004 §6): consecutive frames a second apart in
    # steady SCIENCE jitter both ways, which slow physics alone would not do.
    telemetry = [t for _, t in _telemetry(load(NOMINAL)) if t.mode is Mode.SCIENCE]
    for name in REPORTED:
        values = [getattr(t, name) for t in telemetry[200:400]]
        steps = [b - a for a, b in pairwise(values)]
        assert any(s > 0 for s in steps) and any(s < 0 for s in steps), name


# --- Seeded noise and sensor_freeze ------------------------------------------------------


def test_another_seed_changes_only_the_noisy_bytes() -> None:
    # Byte-identical for the recorded seed (the replay tests); another seed sends the
    # same frames at the same ticks, the same ACKs, and different telemetry bytes.
    stored = load(FREEZE)
    other = FREEZE.run(FREEZE.seed + 1)
    assert [tick for tick, _ in other] == [tick for tick, _ in stored.frames]
    other_capture = Capture(stored.header, list(other))
    assert _types(other_capture, FrameType.ACK) == _types(stored, FrameType.ACK)
    assert _types(other_capture, FrameType.TELEMETRY) != _types(stored, FrameType.TELEMETRY)


def test_sensor_freeze_holds_the_reported_values_in_the_capture() -> None:
    telemetry = _telemetry(load(FREEZE))
    end = FREEZE_START_TICK + FREEZE_US // TICK_US
    before = [t for n, t in telemetry if FREEZE_START_TICK - 100 <= n < FREEZE_START_TICK]
    during = [t for n, t in telemetry if FREEZE_START_TICK <= n < end]
    after = [t for n, t in telemetry if n >= end]
    assert len(before) >= 10 and len(during) >= 19 and len(after) >= 9
    assert all(t.mode is Mode.SCIENCE for t in before + during + after)
    for name in REPORTED:
        # Frozen at the last value reported before the fault, and live again after it.
        assert len({getattr(t, name) for t in during}) == 1, name
        assert len({getattr(t, name) for t in before}) > 1, name
        assert len({getattr(t, name) for t in after}) > 1, name
    # The payload buffer is not a sensor reading: it keeps filling under the freeze.
    assert all(t.payload_state is PayloadState.ACQUIRING for t in during)
    buffered = [t.buffered_bytes for t in during]
    assert buffered == sorted(buffered) and buffered[0] < buffered[-1]


# --- The comparison report ---------------------------------------------------------------


_BUS_MV = [name for name, _ in TELEMETRY_FIELDS].index("bus_mv")


def _sample() -> list[DownlinkFrame]:
    return load(FREEZE).frames[:40]


def test_identical_captures_have_no_difference() -> None:
    assert first_difference(_sample(), _sample()) is None


def test_a_changed_byte_is_reported_with_tick_offset_and_decoded_fields() -> None:
    expected = _sample()
    index, (tick, frame) = next(
        (i, f)
        for i, f in enumerate(expected)
        if decode_frame(f[1]).frame_type is FrameType.TELEMETRY
    )
    # Re-encode with the bus voltage 1 mV higher, so the frame still decodes (valid CRC).
    telemetry = decode_frame(frame)
    values = list(struct.unpack(TELEMETRY_FORMAT, telemetry.payload))
    values[_BUS_MV] += 1
    payload = struct.pack(TELEMETRY_FORMAT, *values)
    changed = encode_frame(Frame(FrameType.TELEMETRY, telemetry.sequence, payload))
    actual = [*expected[:index], (tick, changed), *expected[index + 1 :]]
    report = first_difference(expected, actual)
    assert report is not None
    assert f"frame {index}" in report and f"tick {tick}" in report
    assert "differing byte offsets" in report
    assert "* bus_v:" in report  # the decoded field that differs is marked
    assert "  mode:" in report  # and the ones that don't are listed too


def test_a_shifted_tick_is_reported() -> None:
    expected = _sample()
    tick, frame = expected[5]
    actual = [*expected[:5], (tick + 1, frame), *expected[6:]]
    report = first_difference(expected, actual)
    assert report is not None and f"tick differs: expected {tick}, actual {tick + 1}" in report


def test_missing_and_extra_frames_are_reported() -> None:
    expected = _sample()
    missing = first_difference(expected, expected[:-1])
    extra = first_difference(expected[:-1], expected)
    assert missing is not None and "First missing" in missing
    assert extra is not None and "First extra" in extra


def test_malformed_capture_lines_are_rejected() -> None:
    good = render(FREEZE, _sample())
    with pytest.raises(GoldenFormatError):
        parse(good.replace(" TELEMETRY ", " ACK ", 1))
    with pytest.raises(GoldenFormatError):
        parse(good + "not a frame line\n")
