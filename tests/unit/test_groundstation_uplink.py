"""Command queue, uplink sequencing and encoding (#154, ADR-0010 §2)."""

import dataclasses
import json
from pathlib import Path
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.flight.modes import Mode
from pocketsat.frame import MAX_SEQUENCE, FrameType, decode_frame
from pocketsat.groundstation import (
    CommandFailed,
    CommandRef,
    CommandSent,
    FailureReason,
    GroundStation,
    StationConfig,
)
from pocketsat.groundstation.config import DEFAULT_MAX_UPLINK_COMMAND_COUNT
from pocketsat.groundstation.uplink import CommandQueue, always_open
from pocketsat.messages import Command, CommandId, decode_command

TICK_US = 100_000

VECTORS: dict[str, Any] = json.loads(
    (Path(__file__).parents[1] / "vectors" / "commands.json").read_text()
)


def _vector_command(vector: dict[str, Any]) -> Command:
    name = vector["command"]
    if name == "SET_MODE":
        return Command.set_mode(Mode[vector["target"]])
    builders = {
        "PING": Command.ping,
        "BEGIN_DOWNLINK": Command.begin_downlink,
        "ENTER_SAFE_MODE": Command.enter_safe_mode,
        "RESET": Command.reset,
    }
    return builders[name]()


def _station(**changes: Any) -> GroundStation:
    gate = changes.pop("transmit_gate", always_open)
    return GroundStation(dataclasses.replace(StationConfig(), **changes), transmit_gate=gate)


def _sequences(station: GroundStation, now_us: int) -> list[int]:
    return [decode_frame(p.frame).sequence for p in station.begin_tick(now_us, None).packets]


# --- Encoding ----------------------------------------------------------------------------


def test_every_command_encodes_to_the_shared_vectors() -> None:
    # The vectors number their frames from sequence 1, so one command takes sequence 0
    # first; the rest must then match byte for byte, all five commands included.
    vectors = VECTORS["valid_commands"]
    assert {v["command"] for v in vectors} == {c.name for c in CommandId}
    station = _station(max_uplink_command_count=len(vectors) + 1)
    station.submit(Command.ping())
    for vector in vectors:
        station.submit(_vector_command(vector))

    packets = station.begin_tick(0, None).packets

    assert len(packets) == len(vectors) + 1
    for packet, vector in zip(packets[1:], vectors, strict=True):
        assert packet.frame.hex() == vector["frame_hex"], vector["name"]
        assert decode_frame(packet.frame).sequence == vector["sequence"]


def test_uplink_packets_carry_only_the_frame() -> None:
    station = _station()
    station.submit(Command.ping())
    (packet,) = station.begin_tick(0, None).packets
    frame = decode_frame(packet.frame)
    assert frame.frame_type is FrameType.COMMAND
    assert decode_command(frame.payload).command_id == CommandId.PING
    assert (packet.elevation_deg, packet.range_km, packet.latency_us) == (None, None, None)


def test_each_frame_gets_a_command_sent_event() -> None:
    station = _station()
    station.begin_tick(0, None)
    station.end_tick(TICK_US, [])
    ping = station.submit(Command.ping())
    safe = station.submit(Command.enter_safe_mode())

    uplink = station.begin_tick(TICK_US, None)

    assert uplink.events == (
        CommandSent(TICK_US, ping, CommandId.PING, sequence=0, retry_count=0),
        CommandSent(TICK_US, safe, CommandId.ENTER_SAFE_MODE, sequence=1, retry_count=0),
    )


# --- Sequencing ----------------------------------------------------------------------------


def test_sequence_counts_up_across_ticks_and_restarts_on_reset() -> None:
    station = _station(max_uplink_command_count=2)
    for _ in range(5):
        station.submit(Command.ping())
    assert _sequences(station, 0) == [0, 1]
    assert _sequences(station, TICK_US) == [2, 3]
    assert _sequences(station, 2 * TICK_US) == [4]

    station.reset(RngFactory(1))
    station.submit(Command.ping())
    assert _sequences(station, 0) == [0]


def test_sequence_wraps_from_0xffff_to_0() -> None:
    queue = CommandQueue(max_uplink_command_count=MAX_SEQUENCE + 2, command_expiry_us=None)
    for ref_id in range(MAX_SEQUENCE + 2):
        queue.add(CommandRef(ref_id), Command.ping(), due_us=0)

    packets, _ = queue.transmit(0, gate_open=True)

    sequences = [decode_frame(p.frame).sequence for p in packets]
    assert sequences[-3:] == [MAX_SEQUENCE - 1, MAX_SEQUENCE, 0]
    assert queue.next_sequence == 1


# --- Gate ----------------------------------------------------------------------------------


def test_commands_wait_while_the_gate_is_closed_then_go_out_in_order() -> None:
    open_from_us = 3 * TICK_US
    station = _station(transmit_gate=lambda now_us: now_us >= open_from_us)
    first = station.submit(Command.ping())
    second = station.submit(Command.set_mode(Mode.SCIENCE))

    for tick in range(3):
        assert station.begin_tick(tick * TICK_US, None).events == ()
        station.end_tick((tick + 1) * TICK_US, [])

    uplink = station.begin_tick(open_from_us, None)
    assert [e.command_ref for e in uplink.events if isinstance(e, CommandSent)] == [first, second]
    assert [decode_frame(p.frame).sequence for p in uplink.packets] == [0, 1]


def test_the_default_gate_is_open() -> None:
    assert GroundStation()._transmit_gate is always_open
    assert always_open(0) and always_open(10**12)


def test_transmit_gate_must_be_callable() -> None:
    with pytest.raises(TypeError, match="transmit_gate"):
        GroundStation(transmit_gate=True)  # type: ignore[arg-type]


# --- Due time ------------------------------------------------------------------------------


def test_a_command_is_sent_from_its_due_time_and_never_blocks_earlier_ones() -> None:
    station = _station()
    later = station.submit(Command.ping(), due_us=2 * TICK_US)
    now = station.submit(Command.enter_safe_mode())

    sent_at = {}
    for tick in range(3):
        for event in station.begin_tick(tick * TICK_US, None).events:
            assert isinstance(event, CommandSent)
            sent_at[event.command_ref] = event.time_us
        station.end_tick((tick + 1) * TICK_US, [])

    assert sent_at == {now: 0, later: 2 * TICK_US}


def test_due_commands_go_out_by_due_time_then_submission_order() -> None:
    station = _station()
    station.begin_tick(0, None)
    station.end_tick(5 * TICK_US, [])
    b = station.submit(Command.ping(), due_us=2 * TICK_US)
    a = station.submit(Command.ping(), due_us=TICK_US)
    c = station.submit(Command.ping(), due_us=2 * TICK_US)

    events = station.begin_tick(5 * TICK_US, None).events

    assert [e.command_ref for e in events if isinstance(e, CommandSent)] == [a, b, c]


def test_due_time_defaults_to_the_station_clock() -> None:
    station = _station(transmit_gate=lambda now_us: False)
    station.begin_tick(0, None)
    station.end_tick(4 * TICK_US, [])
    station.submit(Command.ping())
    assert station._uplink.queued[0].due_us == 4 * TICK_US


def test_negative_due_time_is_rejected() -> None:
    with pytest.raises(ValueError, match="due_us"):
        GroundStation().submit(Command.ping(), due_us=-1)


# --- Per-tick limit ------------------------------------------------------------------------


def test_at_most_the_configured_number_of_commands_per_tick() -> None:
    station = _station()
    refs = [station.submit(Command.ping()) for _ in range(DEFAULT_MAX_UPLINK_COMMAND_COUNT + 2)]

    first = station.begin_tick(0, None)
    station.end_tick(TICK_US, [])
    second = station.begin_tick(TICK_US, None)

    assert len(first.packets) == DEFAULT_MAX_UPLINK_COMMAND_COUNT
    assert len(second.packets) == 2
    sent = [e.command_ref for e in (*first.events, *second.events) if isinstance(e, CommandSent)]
    assert sent == refs


# --- Expiry --------------------------------------------------------------------------------


def test_a_command_expires_if_the_gate_stays_closed() -> None:
    expiry_us = 3 * TICK_US
    gate_open = {"value": False}
    station = _station(command_expiry_us=expiry_us, transmit_gate=lambda now_us: gate_open["value"])
    ref = station.submit(Command.ping())

    for tick in range(3):
        assert station.begin_tick(tick * TICK_US, None).events == ()
        station.end_tick((tick + 1) * TICK_US, [])
    expired = station.begin_tick(expiry_us, None)
    station.end_tick(expiry_us + TICK_US, [])
    gate_open["value"] = True
    after = station.begin_tick(expiry_us + TICK_US, None)

    assert expired.events == (CommandFailed(expiry_us, ref, FailureReason.EXPIRED),)
    assert expired.packets == ()
    assert after == type(after)()


def test_expiry_counts_from_the_due_time() -> None:
    station = _station(command_expiry_us=TICK_US, transmit_gate=lambda now_us: now_us >= TICK_US)
    ref = station.submit(Command.ping(), due_us=TICK_US)

    assert station.begin_tick(0, None).events == ()
    station.end_tick(TICK_US, [])
    sent = station.begin_tick(TICK_US, None).events

    assert sent == (CommandSent(TICK_US, ref, CommandId.PING, sequence=0, retry_count=0),)


def test_without_expiry_a_command_waits_indefinitely() -> None:
    station = _station(transmit_gate=lambda now_us: now_us >= 10**9)
    ref = station.submit(Command.ping())
    assert station.begin_tick(10**9 - TICK_US, None).events == ()
    station.end_tick(10**9, [])
    (event,) = station.begin_tick(10**9, None).events
    assert isinstance(event, CommandSent) and event.command_ref == ref


# --- Configuration -------------------------------------------------------------------------


@pytest.mark.parametrize("value", [0, -1])
def test_max_uplink_command_count_must_be_positive(value: int) -> None:
    with pytest.raises(ValueError, match="max_uplink_command_count"):
        StationConfig(max_uplink_command_count=value)


@pytest.mark.parametrize("value", [True, 1.5, "4"])
def test_max_uplink_command_count_must_be_an_int(value: object) -> None:
    with pytest.raises(TypeError, match="max_uplink_command_count"):
        StationConfig(max_uplink_command_count=value)  # type: ignore[arg-type]


def test_command_expiry_must_be_non_negative_int_or_none() -> None:
    assert StationConfig().command_expiry_us is None
    assert StationConfig(command_expiry_us=0).command_expiry_us == 0
    with pytest.raises(ValueError, match="command_expiry_us"):
        StationConfig(command_expiry_us=-1)
    with pytest.raises(TypeError, match="command_expiry_us"):
        StationConfig(command_expiry_us=1.0)  # type: ignore[arg-type]
