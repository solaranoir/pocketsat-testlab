"""ACK/NACK correlation, timeouts and retries (#155, ADR-0010 §2).

The station is driven by hand: ``begin_tick`` sends, hand-built ACK frames come back
through ``end_tick``, so every timing is exact.
"""

import dataclasses

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.flight.modes import Mode, RejectReason
from pocketsat.frame import FrameType, encode_frame_fields
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    CommandAcked,
    CommandFailed,
    CommandNacked,
    CommandRef,
    CommandSent,
    CommandTimedOut,
    FailureReason,
    GroundStation,
    StationEvent,
    UnmatchedAck,
    max_answer_latency_us,
)
from pocketsat.groundstation.commands import (
    NON_RETRYABLE_COMMANDS,
    WORST_CASE_ANSWER_BYTES,
)
from pocketsat.messages import (
    Command,
    CommandAck,
    CommandId,
    DecodeReason,
    NackReason,
    Packet,
    encode_ack,
)

TICK_US = 100_000
TIMEOUT_US = 300_000
CONFIG = dataclasses.replace(
    DEFAULT_STATION_CONFIG, command_timeout_us=TIMEOUT_US, command_retry_count=2
)
COMMAND_EVENTS = (CommandSent, CommandAcked, CommandNacked, CommandTimedOut, CommandFailed)


class Downlink:
    """Builds ACK frames with consecutive downlink sequences, so no gaps are reported."""

    def __init__(self) -> None:
        self._sequence = 0

    def ack(self, sequence: int, command_id: int, reason: NackReason | None = None) -> Packet:
        payload = encode_ack(CommandAck(sequence, command_id, reason))
        frame = encode_frame_fields(FrameType.ACK, self._sequence, payload)
        self._sequence += 1
        return Packet(frame=frame)


def _command_events(events: tuple[StationEvent, ...]) -> list[StationEvent]:
    return [e for e in events if isinstance(e, (*COMMAND_EVENTS, UnmatchedAck))]


def _sent(events: tuple[StationEvent, ...]) -> list[CommandSent]:
    return [e for e in events if isinstance(e, CommandSent)]


def _station(**changes: object) -> GroundStation:
    return GroundStation(dataclasses.replace(CONFIG, **changes))  # type: ignore[arg-type]


# --- The timeout bound ------------------------------------------------------------------


def test_worst_case_answer_is_the_frame_ahead_the_held_acks_and_the_answer() -> None:
    # ADR-0008 §5: r + A + 14 with r <= 77 and at most 8 held ACKs of 14 bytes.
    assert WORST_CASE_ANSWER_BYTES == 77 + 8 * 14 + 14 == 203


@pytest.mark.parametrize(
    ("tick_us", "rate", "expected_us"),
    [
        (100_000, 1200, 200_000),  # 204 bytes take 170 ms: two default ticks
        (2_000_000, 1200, 2_000_000),  # never less than one tick
        (30_000, 1200, 180_000),  # streaming ticks: 170 ms rounds up to 6 ticks
        (100_000, 2400, 100_000),  # 85 ms at twice the rate: one tick
        (170_000, 1200, 170_000),  # exactly one tick of airtime
    ],
)
def test_max_answer_latency(tick_us: int, rate: int, expected_us: int) -> None:
    assert max_answer_latency_us(tick_us, rate) == expected_us


def test_max_answer_latency_rejects_bad_arguments() -> None:
    with pytest.raises(ValueError):
        max_answer_latency_us(0)
    with pytest.raises(ValueError):
        max_answer_latency_us(TICK_US, 0)
    with pytest.raises(TypeError):
        max_answer_latency_us(TICK_US, 1.5)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        max_answer_latency_us(TICK_US, True)


def test_reset_is_the_only_non_retryable_command() -> None:
    assert frozenset({CommandId.RESET}) == NON_RETRYABLE_COMMANDS


# --- Correlation ---------------------------------------------------------------------


def test_ack_answers_its_command() -> None:
    station, down = _station(), Downlink()
    ref = station.submit(Command.ping())
    (sent,) = _sent(station.begin_tick(0, None).events)
    assert station.outstanding_commands == (ref,)

    events = station.end_tick(TICK_US, [down.ack(sent.sequence, CommandId.PING)])

    assert events == (CommandAcked(TICK_US, ref, sent.sequence),)
    assert station.outstanding_commands == ()


@pytest.mark.parametrize("reason", [RejectReason.BOOT_IN_PROGRESS, DecodeReason.UNKNOWN_COMMAND])
def test_nack_answers_its_command_with_the_reason_by_name(reason: NackReason) -> None:
    station, down = _station(), Downlink()
    ref = station.submit(Command.set_mode(Mode.SCIENCE))
    (sent,) = _sent(station.begin_tick(0, None).events)

    (event,) = station.end_tick(TICK_US, [down.ack(sent.sequence, CommandId.SET_MODE, reason)])

    assert event == CommandNacked(TICK_US, ref, sent.sequence, reason)
    assert isinstance(event, CommandNacked)
    assert event.reason.name == reason.name
    assert station.outstanding_commands == ()


def test_acks_correlate_by_sequence_in_any_order() -> None:
    station, down = _station(), Downlink()
    first = station.submit(Command.ping())
    second = station.submit(Command.enter_safe_mode())
    a, b = _sent(station.begin_tick(0, None).events)

    events = station.end_tick(
        TICK_US,
        [down.ack(b.sequence, CommandId.ENTER_SAFE_MODE), down.ack(a.sequence, CommandId.PING)],
    )

    assert events == (
        CommandAcked(TICK_US, second, b.sequence),
        CommandAcked(TICK_US, first, a.sequence),
    )


def test_unknown_sequence_is_unmatched_and_counted() -> None:
    station, down = _station(), Downlink()
    events = station.end_tick(0, [down.ack(42, CommandId.PING)])
    assert events == (UnmatchedAck(0, 42, CommandId.PING),)
    assert station.unmatched_ack_count == 1


def test_mismatched_command_id_is_unmatched_and_leaves_the_command_waiting() -> None:
    station, down = _station(), Downlink()
    ref = station.submit(Command.ping())
    (sent,) = _sent(station.begin_tick(0, None).events)

    events = station.end_tick(TICK_US, [down.ack(sent.sequence, CommandId.RESET)])

    assert events == (UnmatchedAck(TICK_US, sent.sequence, CommandId.RESET),)
    assert station.outstanding_commands == (ref,)
    assert station.unmatched_ack_count == 1


def test_duplicate_ack_is_unmatched() -> None:
    station, down = _station(), Downlink()
    ref = station.submit(Command.ping())
    (sent,) = _sent(station.begin_tick(0, None).events)

    events = station.end_tick(
        TICK_US,
        [down.ack(sent.sequence, CommandId.PING), down.ack(sent.sequence, CommandId.PING)],
    )

    assert events == (
        CommandAcked(TICK_US, ref, sent.sequence),
        UnmatchedAck(TICK_US, sent.sequence, CommandId.PING),
    )
    assert station.unmatched_ack_count == 1


# --- Timeouts and retries ----------------------------------------------------------------


def test_answer_decoded_at_the_timeout_is_not_a_timeout() -> None:
    # end_tick at the timeout runs before begin_tick at the same time.
    station, down = _station(), Downlink()
    ref = station.submit(Command.ping())
    (sent,) = _sent(station.begin_tick(0, None).events)
    for now in (100_000, 200_000):
        assert station.end_tick(now, []) == ()
        assert station.begin_tick(now, None).events == ()

    assert station.end_tick(TIMEOUT_US, [down.ack(sent.sequence, CommandId.PING)]) == (
        CommandAcked(TIMEOUT_US, ref, sent.sequence),
    )
    assert station.begin_tick(TIMEOUT_US, None).events == ()


def test_timeout_retries_with_a_new_sequence_then_fails() -> None:
    station = _station()
    ref = station.submit(Command.ping())
    first = _sent(station.begin_tick(0, None).events)[0]

    second_tick = station.begin_tick(TIMEOUT_US - 1, None)
    assert second_tick.events == ()

    retry_1 = station.begin_tick(TIMEOUT_US, None)
    assert _command_events(retry_1.events) == [
        CommandTimedOut(TIMEOUT_US, ref, retry_count=0, will_retry=True),
        CommandSent(TIMEOUT_US, ref, CommandId.PING, first.sequence + 1, retry_count=1),
    ]
    assert len(retry_1.packets) == 1

    retry_2 = station.begin_tick(2 * TIMEOUT_US, None)
    assert _command_events(retry_2.events) == [
        CommandTimedOut(2 * TIMEOUT_US, ref, retry_count=1, will_retry=True),
        CommandSent(2 * TIMEOUT_US, ref, CommandId.PING, first.sequence + 2, retry_count=2),
    ]

    failed = station.begin_tick(3 * TIMEOUT_US, None)
    assert failed.events == (
        CommandTimedOut(3 * TIMEOUT_US, ref, retry_count=2, will_retry=False),
        CommandFailed(3 * TIMEOUT_US, ref, FailureReason.RETRIES_EXHAUSTED),
    )
    assert failed.packets == ()
    assert station.outstanding_commands == ()


def test_no_retries_configured_fails_after_one_timeout() -> None:
    station = _station(command_retry_count=0)
    ref = station.submit(Command.ping())
    station.begin_tick(0, None)
    assert station.begin_tick(TIMEOUT_US, None).events == (
        CommandTimedOut(TIMEOUT_US, ref, retry_count=0, will_retry=False),
        CommandFailed(TIMEOUT_US, ref, FailureReason.RETRIES_EXHAUSTED),
    )


def test_reset_is_never_retried() -> None:
    # A retried RESET whose ACK was lost would reboot the spacecraft twice (ADR-0010 §2).
    station = _station()
    ref = station.submit(Command.reset())
    station.begin_tick(0, None)

    result = station.begin_tick(TIMEOUT_US, None)

    assert result.events == (
        CommandTimedOut(TIMEOUT_US, ref, retry_count=0, will_retry=False),
        CommandFailed(TIMEOUT_US, ref, FailureReason.NOT_RETRYABLE),
    )
    assert result.packets == ()
    assert station.begin_tick(10 * TIMEOUT_US, None).packets == ()


def test_late_answer_to_an_earlier_attempt_answers_the_command() -> None:
    # The spacecraft executed the first attempt; its ACK was only slow.
    station, down = _station(), Downlink()
    ref = station.submit(Command.ping())
    first = _sent(station.begin_tick(0, None).events)[0]
    retry = _sent(station.begin_tick(TIMEOUT_US, None).events)[0]

    events = station.end_tick(TIMEOUT_US + TICK_US, [down.ack(first.sequence, CommandId.PING)])

    assert events == (CommandAcked(TIMEOUT_US + TICK_US, ref, first.sequence),)
    assert station.outstanding_commands == ()
    # The retry's own answer then answers nothing.
    assert station.end_tick(TIMEOUT_US + TICK_US, [down.ack(retry.sequence, CommandId.PING)]) == (
        UnmatchedAck(TIMEOUT_US + TICK_US, retry.sequence, CommandId.PING),
    )


def test_late_answer_cancels_a_retry_still_queued() -> None:
    gate_open = [True]
    station = GroundStation(CONFIG, transmit_gate=lambda now_us: gate_open[0])
    down = Downlink()
    ref = station.submit(Command.ping())
    first = _sent(station.begin_tick(0, None).events)[0]

    gate_open[0] = False
    timed_out = station.begin_tick(TIMEOUT_US, None)
    assert timed_out.events == (CommandTimedOut(TIMEOUT_US, ref, retry_count=0, will_retry=True),)
    assert timed_out.packets == ()  # the retry waits in the queue

    station.end_tick(TIMEOUT_US + TICK_US, [down.ack(first.sequence, CommandId.PING)])
    gate_open[0] = True

    after = station.begin_tick(TIMEOUT_US + TICK_US, None)
    assert after.packets == ()
    assert after.events == ()


def test_retry_that_expires_in_the_queue_fails_and_is_forgotten() -> None:
    station = GroundStation(
        dataclasses.replace(CONFIG, command_expiry_us=TICK_US), transmit_gate=lambda t: t == 0
    )
    down = Downlink()
    ref = station.submit(Command.ping())
    first = _sent(station.begin_tick(0, None).events)[0]
    station.begin_tick(TIMEOUT_US, None)  # times out; the retry is queued, gate closed

    assert station.begin_tick(TIMEOUT_US + TICK_US, None).events == (
        CommandFailed(TIMEOUT_US + TICK_US, ref, FailureReason.EXPIRED),
    )
    # A late answer to the first attempt now answers nothing.
    assert station.end_tick(TIMEOUT_US + TICK_US, [down.ack(first.sequence, CommandId.PING)]) == (
        UnmatchedAck(TIMEOUT_US + TICK_US, first.sequence, CommandId.PING),
    )


def test_each_command_times_out_on_its_own_clock() -> None:
    station = _station()
    early = station.submit(Command.ping())
    station.begin_tick(0, None)
    late = station.submit(Command.ping())
    station.begin_tick(TICK_US, None)

    assert _command_events(station.begin_tick(TIMEOUT_US, None).events)[0] == CommandTimedOut(
        TIMEOUT_US, early, retry_count=0, will_retry=True
    )
    assert _command_events(station.begin_tick(TIMEOUT_US + TICK_US, None).events)[
        0
    ] == CommandTimedOut(TIMEOUT_US + TICK_US, late, retry_count=0, will_retry=True)


def test_reset_clears_outstanding_commands_and_counts() -> None:
    station, down = _station(), Downlink()
    station.submit(Command.ping())
    station.begin_tick(0, None)
    station.end_tick(TICK_US, [down.ack(999, CommandId.PING)])
    assert station.outstanding_commands == (CommandRef(0),)
    assert station.unmatched_ack_count == 1

    station.reset(RngFactory(1))

    assert station.outstanding_commands == ()
    assert station.unmatched_ack_count == 0
    assert station.begin_tick(10 * TIMEOUT_US, None).events == ()
