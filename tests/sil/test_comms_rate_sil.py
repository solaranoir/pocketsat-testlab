"""The transmit rate at uneven ticks, through ``SilTarget`` (#122).

Comms' transmit rate is in bytes per second; each tick's ``transmit_capacity_bytes`` is
the rate over the tick, with the fraction of a byte carried. When the rate does not
divide evenly into ticks the capacity varies from tick to tick (for example 87 and 88
bytes at 1250 bytes/s and 70 ms). These runs check, with the real flight computer and a
whole DOWNLINK pass, that the pieces built for a fixed capacity still agree:

- the flight computer sends at most this tick's capacity (ADR-0004 §10);
- comms accepts every tick's traffic against its previous tick's capacity (ADR-0007
  §4; a violation would raise ``ValueError`` out of ``advance``);
- the #48 consistency rule, which checks ``previous_tick_sent_bytes`` against the
  previous tick's capacity, never raises a fault;
- the capacities add up to the rate times the transmitter's on-time, and the whole
  buffer is sent, in order, with the same downlink for the same seed.

At the default 100 ms tick the capacity is exactly 120 bytes every tick, so nothing
changes there; that is #121's behaviour digests' job (``test_behavior_digests.py``).
"""

import dataclasses
import itertools

import pytest

from pocketsat.flight import FlightComputer, FlightComputerConfig, Mode
from pocketsat.flight.boot import BootConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import Command, decode_data, encode_command
from pocketsat.spacecraft import (
    NOMINAL_CONFIG,
    CommsSnapshot,
    CommsTruth,
    PayloadInitial,
    SpacecraftInitialState,
)
from pocketsat.targets.sil import SilTarget, SilTick

CHUNK = NOMINAL_CONFIG.payload.chunk_size_bytes
BUFFER = NOMINAL_CONFIG.payload.buffer_capacity_bytes
CHUNKS = 60
US_PER_S = 1_000_000


def comms(tick: SilTick) -> CommsTruth:
    return tick.state.get("comms", CommsSnapshot).truth


def sent(tick: SilTick) -> int:
    return sum(len(frame) for frame in tick.downlink_frames)


def run_pass(tick_us: int, rate: int, seed: int = 5) -> tuple[list[SilTick], list[Mode], list[int]]:
    """Boot, start a DOWNLINK pass of ``CHUNKS`` chunks, and run until it completes.

    Returns every tick's record, the flight computer's mode after each tick, and the
    chunk IDs received in order.
    """
    config = dataclasses.replace(
        NOMINAL_CONFIG,
        comms=dataclasses.replace(NOMINAL_CONFIG.comms, transmit_rate_bytes_per_s=rate),
    )
    initial = SpacecraftInitialState(
        payload=PayloadInitial(buffer_fill=(CHUNKS * CHUNK + CHUNK // 2) / BUFFER)
    )
    computers: list[FlightComputer] = []

    def factory() -> FlightComputer:
        computer = FlightComputer(FlightComputerConfig(boot=BootConfig(duration_us=US_PER_S)))
        computers.append(computer)
        return computer

    target = SilTarget(
        config=config, initial=initial, tick_us=tick_us, flight_computer_factory=factory
    )
    target.connect()
    target.reset(seed)
    ticks: list[SilTick] = []
    modes: list[Mode] = []
    received: list[int] = []
    began = False
    for _ in range(20_000):
        mode = computers[-1].mode
        if mode is Mode.NOMINAL and not began:
            frame = Frame(FrameType.COMMAND, 1, encode_command(Command.begin_downlink()))
            target.send(encode_frame(frame))
            began = True
        elif Mode.DOWNLINK in modes and mode is not Mode.DOWNLINK:
            break  # DOWNLINK_COMPLETE
        target.advance(tick_us)
        tick = target.last_tick
        assert tick is not None
        ticks.append(tick)
        modes.append(computers[-1].mode)
        for raw in target.receive():
            decoded = decode_frame(raw)
            if decoded.frame_type is FrameType.DATA:
                received.append(decode_data(decoded.payload).chunk_id)
    else:
        raise AssertionError("the pass never completed")
    return ticks, modes, received


UNEVEN = [
    pytest.param(70_000, 1_250, id="70ms-87.5B"),
    pytest.param(30_000, 4_321, id="30ms-129.63B"),
    pytest.param(300_000, 1_234, id="300ms-370.2B"),
    pytest.param(1_000_000, 1_200, id="1s-default-rate"),
]


@pytest.mark.parametrize(("tick_us", "rate"), UNEVEN)
def test_a_pass_at_an_uneven_tick_respects_each_ticks_capacity(tick_us: int, rate: int) -> None:
    ticks, modes, received = run_pass(tick_us, rate)
    assert Mode.FAULT not in modes and Mode.SAFE not in modes  # #48 never fired
    assert Mode.DOWNLINK in modes
    assert received == list(range(CHUNKS))  # the whole buffer, in order
    on_us = total = 0
    for tick in ticks:
        truth = comms(tick)
        # The flight computer used this tick's capacity, whatever it was.
        assert sent(tick) <= truth.transmit_capacity_bytes
        if truth.transmitter_on:
            on_us += tick_us
        total += truth.transmit_capacity_bytes
        # Exact, never drifting: floor(rate x on-time).
        assert total == rate * on_us // US_PER_S
    # Comms took each tick's traffic, checked against that tick's capacity, in the next.
    for before, after in itertools.pairwise(ticks):
        assert comms(after).previous_tick_sent_bytes == sent(before)
        assert sent(before) <= comms(before).transmit_capacity_bytes
    pass_ticks = [t for t, m in zip(ticks, modes, strict=True) if m is Mode.DOWNLINK]
    capacities = {comms(t).transmit_capacity_bytes for t in pass_ticks}
    exact = rate * tick_us % US_PER_S == 0
    assert len(capacities) == (1 if exact else 2)


def test_the_capacity_alternates_and_traffic_is_bounded_by_the_previous_tick() -> None:
    # At 1250 bytes/s and 70 ms the capacity alternates 87 and 88 bytes. Every tick's
    # bytes, reported by comms in the next tick, are bounded by the capacity of the tick
    # they were sent in, which is not the capacity comms reports alongside them.
    ticks, _, _ = run_pass(70_000, 1_250)
    caps = [comms(t).transmit_capacity_bytes for t in ticks]
    assert {87, 88} <= set(caps)
    for n in range(len(ticks) - 1):
        assert comms(ticks[n + 1]).previous_tick_sent_bytes <= caps[n]
    assert any(c != caps[n + 1] for n, c in enumerate(caps[:-1]) if comms(ticks[n]).transmitter_on)


@pytest.mark.parametrize(("tick_us", "rate"), UNEVEN[:2])
def test_same_seed_same_downlink_at_an_uneven_tick(tick_us: int, rate: int) -> None:
    first, _, _ = run_pass(tick_us, rate)
    second, _, _ = run_pass(tick_us, rate)
    assert [t.downlink_frames for t in first] == [t.downlink_frames for t in second]
    assert [t.state for t in first] == [t.state for t in second]


def test_the_rate_not_the_tick_sets_how_long_a_pass_takes() -> None:
    # 60 chunks in 78-byte DATA frames are 4680 bytes: at 1200 bytes/s no pass can be
    # shorter than 3.9 s, at any tick length, and none is much longer (before #122 a
    # 1 s tick allowed 120 bytes per second, about 40 s). Frames are not split across
    # ticks, so the unused end of each tick makes the pass somewhat slower than the
    # rate; below 65 ms the default rate gives fewer than 78 bytes per tick and a DATA
    # frame never fits.
    rate = NOMINAL_CONFIG.comms.transmit_rate_bytes_per_s
    shortest_us = CHUNKS * 78 * US_PER_S // rate
    for tick_us in (70_000, 100_000, 1_000_000):
        ticks, modes, received = run_pass(tick_us, rate)
        assert received == list(range(CHUNKS))
        downlink_ticks = sum(m is Mode.DOWNLINK for m in modes)
        assert shortest_us <= downlink_ticks * tick_us <= 2 * shortest_us + 2 * tick_us
        data_bytes = sum(len(f) for t in ticks for f in t.downlink_frames if len(f) == 78)
        assert data_bytes == CHUNKS * 78
