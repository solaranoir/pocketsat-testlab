"""The telemetry store against ``SilTarget`` (#157).

``SilTarget``'s telemetry goes down through the ideal link (#153) to the station, which
decodes and stores it (#156, #157). Each stored record is checked against the readings
the flight computer encoded in the same tick, within half the field's resolution
(``TELEMETRY_QUANTITIES``, the quantisation of ``docs/protocol.md``).
"""

from fractions import Fraction

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.flight import SpacecraftReadings
from pocketsat.groundstation import GroundStation, TelemetryRecord
from pocketsat.groundstation.telemetry import quantity
from pocketsat.link import IdealLink
from pocketsat.targets.sil import SilTarget

TICK_US = 100_000
SEED = 42
TICKS = 300  # 30 s: past BOOT (5 s), into NOMINAL, about 30 telemetry frames


def _expected(readings: SpacecraftReadings) -> dict[str, float]:
    """The reported values telemetry encodes, by telemetry field name."""
    return {
        "bus_v": readings.power.bus_v,
        "soc": readings.power.soc,
        "battery_c": readings.thermal.battery_c,
        "electronics_c": readings.thermal.electronics_c,
        "pointing_error_deg": readings.attitude.pointing_error_deg,
        "buffered_bytes": readings.payload.buffered_bytes,
    }


def _run() -> list[tuple[TelemetryRecord, SpacecraftReadings]]:
    """Run the station, link and target; pair each stored record with its tick's readings."""
    station, link, target = GroundStation(), IdealLink(), SilTarget()
    rngs = RngFactory(SEED)
    station.reset(rngs)
    link.reset(rngs)
    target.connect()
    target.reset(SEED)
    pairs: list[tuple[TelemetryRecord, SpacecraftReadings]] = []
    for tick in range(TICKS):
        start_us, end_us = tick * TICK_US, (tick + 1) * TICK_US
        uplink = station.begin_tick(start_us, None)
        for frame in link.carry_uplink(uplink.packets, start_us, None):
            target.send(frame)
        target.advance(TICK_US)
        stored = len(station.telemetry)
        station.end_tick(end_us, link.carry_downlink(target.receive(), end_us, None))
        last_tick = target.last_tick
        assert last_tick is not None
        readings = SpacecraftReadings.from_state(last_tick.state)
        pairs.extend((record, readings) for record in station.telemetry.history[stored:])
    target.close()
    return pairs


def test_stored_telemetry_round_trips_sil_readings_within_quantisation() -> None:
    pairs = _run()
    assert len(pairs) >= 25, "expected about one telemetry frame per second"
    for record, readings in pairs:
        for name, expected in _expected(readings).items():
            resolution = quantity(name).resolution
            assert resolution is not None
            tolerance = float(resolution / 2) if resolution != Fraction(1) else 0.0
            assert record.value(name) == pytest.approx(expected, abs=tolerance), name


def test_records_carry_increasing_time_and_sequence_in_one_pass() -> None:
    records = [record for record, _ in _run()]
    assert [r.time_us for r in records] == sorted(r.time_us for r in records)
    assert all(r.time_us % TICK_US == 0 for r in records)
    sequences = [r.sequence for r in records]
    assert sequences == sorted(set(sequences))
    assert {r.pass_id for r in records} == {0}  # one pass until #161
