"""The ground station's telemetry store (#157): records, units, queries and retention."""

import dataclasses
import json
import re
from fractions import Fraction
from pathlib import Path
from typing import Any

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.flight import Mode
from pocketsat.frame import FrameType, encode_frame_fields
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    TELEMETRY_QUANTITIES,
    GroundStation,
    StationConfig,
    TelemetryRecord,
    TelemetryStore,
)
from pocketsat.groundstation.config import DEFAULT_TELEMETRY_RETENTION_COUNT
from pocketsat.groundstation.downlink import DownlinkDecoder
from pocketsat.groundstation.telemetry import quantity
from pocketsat.messages import TELEMETRY_FIELDS, Packet, Telemetry, decode_telemetry

ROOT = Path(__file__).resolve().parents[2]
PROTOCOL = (ROOT / "docs" / "protocol.md").read_text(encoding="utf-8")
VECTORS: dict[str, Any] = json.loads(
    (ROOT / "tests" / "vectors" / "telemetry.json").read_text(encoding="utf-8")
)
VALID = {v["name"]: v for v in VECTORS["valid_telemetry"]}


def _telemetry(name: str = "nominal_no_flags") -> Telemetry:
    return decode_telemetry(bytes.fromhex(VALID[name]["payload_hex"]))


NOMINAL = _telemetry()


def _record(time_us: int, sequence: int = 0, pass_id: int = 0) -> TelemetryRecord:
    return TelemetryRecord(time_us, sequence, pass_id, NOMINAL)


def _store(times_us: list[int], retention_count: int = 100) -> TelemetryStore:
    store = TelemetryStore(retention_count)
    for sequence, time_us in enumerate(times_us):
        store.append(_record(time_us, sequence))
    return store


# --- Engineering units -----------------------------------------------------------------


def _table_after(heading: str) -> list[list[str]]:
    """Rows of the first Markdown table after ``heading``, header and rule excluded."""
    start = PROTOCOL.index(heading)
    rows: list[list[str]] = []
    for line in PROTOCOL[start:].splitlines()[1:]:
        if line.startswith("|"):
            rows.append([cell.strip() for cell in line.strip().strip("|").split("|")])
        elif rows:
            break
    return rows[2:]


def test_quantities_cover_every_wire_field_in_payload_order() -> None:
    assert [q.wire_field for q in TELEMETRY_QUANTITIES] == [name for name, _ in TELEMETRY_FIELDS]
    assert [q.name for q in TELEMETRY_QUANTITIES] == [f.name for f in dataclasses.fields(Telemetry)]


def test_quantities_match_the_protocol_mapping_table() -> None:
    rows = {row[1].strip("`"): row for row in _table_after("### Layout and mapping")}
    for q in TELEMETRY_QUANTITIES:
        conversion, unit = rows[q.wire_field][4], rows[q.wire_field][5]
        scaled = re.fullmatch(r"`(\w+)` \u00d7 (\d+)", conversion)
        if scaled is not None:
            # The decoded value is the wire integer divided by the documented scale.
            assert scaled.group(1) == q.name
            assert q.resolution == Fraction(1, int(scaled.group(2)))
        elif conversion == "as is":
            assert (q.resolution, q.unit) == (Fraction(1), unit)
        else:
            # Enumerated values and the flag bit field have no unit or resolution.
            assert (q.resolution, q.unit, unit) == (None, "", "—")


def test_quantity_lookup() -> None:
    assert quantity("bus_v").unit == "V"
    assert quantity("soc").resolution == Fraction(1, 1000)
    with pytest.raises(KeyError, match="not a telemetry field"):
        quantity("bus_mv")


def test_record_values_and_names() -> None:
    record = _record(0)
    assert record.value("bus_v") == pytest.approx(7.4)
    assert record.value("soc") == pytest.approx(0.85)
    assert record.value("battery_c") == pytest.approx(18.25)
    assert record.value("mode") is Mode.NOMINAL
    assert record.state_name("mode") == "NOMINAL"
    assert record.state_name("radio_mode") == NOMINAL.radio_mode.name
    assert record.flag_names == ()
    with pytest.raises(KeyError):
        record.value("bus_mv")
    for not_enumerated in ("flags", "bus_v"):
        with pytest.raises(ValueError, match="not an enumerated"):
            record.state_name(not_enumerated)


def test_flag_names_in_bit_order() -> None:
    record = TelemetryRecord(0, 0, 0, _telemetry("all_flags"))
    assert record.flag_names == ("low_battery", "critical_battery", "over_temp", "under_temp")
    one = TelemetryRecord(0, 0, 0, _telemetry("flag_over_temp_only"))
    assert one.flag_names == ("over_temp",)


def test_records_are_immutable() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        _record(0).time_us = 1  # type: ignore[misc]


# --- Queries -----------------------------------------------------------------------


def test_empty_store() -> None:
    store = TelemetryStore()
    assert len(store) == 0
    assert store.latest is None
    assert store.latest_value("bus_v") is None
    assert store.history == ()
    assert store.window(0, 10) == ()
    assert store.pass_ids == ()
    assert store.retention_count == DEFAULT_TELEMETRY_RETENTION_COUNT
    with pytest.raises(KeyError):
        store.latest_value("nope")


def test_latest_and_latest_value() -> None:
    store = _store([0, 1_000_000, 2_000_000])
    assert store.latest == _record(2_000_000, 2)
    assert store.latest_value("bus_v") == pytest.approx(7.4)
    assert list(store) == list(store.history)


def test_window_is_half_open() -> None:
    store = _store([0, 1_000_000, 2_000_000, 3_000_000])
    assert [r.time_us for r in store.window(1_000_000, 3_000_000)] == [1_000_000, 2_000_000]
    assert store.window(3_000_001, 9_000_000) == ()
    with pytest.raises(ValueError):
        store.window(-1, 5)
    with pytest.raises(TypeError):
        store.window(0.0, 5)  # type: ignore[arg-type]


def test_per_pass_grouping() -> None:
    store = TelemetryStore()
    for time_us, pass_id in [(0, 0), (1, 1), (2, 1), (3, 3)]:
        store.append(_record(time_us, pass_id=pass_id))
    assert store.pass_ids == (0, 1, 3)
    assert [r.time_us for r in store.by_pass(1)] == [1, 2]
    assert store.by_pass(2) == ()


def test_append_rejects_bad_records() -> None:
    store = _store([5])
    with pytest.raises(ValueError, match="before the latest"):
        store.append(_record(4))
    with pytest.raises(TypeError):
        store.append(NOMINAL)  # type: ignore[arg-type]
    store.append(_record(5))  # the same receive time: several frames in one tick
    assert len(store) == 2


# --- Retention ---------------------------------------------------------------------


def test_retention_drops_the_oldest_and_counts_it() -> None:
    store = _store(list(range(7)), retention_count=3)
    assert [r.time_us for r in store.history] == [4, 5, 6]
    assert store.dropped_count == 4
    assert store.retention_count == 3


@pytest.mark.parametrize("bad", [0, -1])
def test_retention_must_be_positive(bad: int) -> None:
    with pytest.raises(ValueError):
        TelemetryStore(bad)
    with pytest.raises(ValueError):
        StationConfig(telemetry_retention_count=bad)


@pytest.mark.parametrize("bad", [True, 1.5, "3"])
def test_retention_must_be_an_int(bad: object) -> None:
    with pytest.raises(TypeError):
        TelemetryStore(bad)  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        StationConfig(telemetry_retention_count=bad)  # type: ignore[arg-type]


def test_default_retention_is_the_documented_one() -> None:
    assert DEFAULT_STATION_CONFIG.telemetry_retention_count == 20_000
    assert GroundStation().telemetry.retention_count == 20_000
    small = GroundStation(StationConfig(telemetry_retention_count=2))
    assert small.telemetry.retention_count == 2


# --- Through the decoder and the station ------------------------------------------------


def _telemetry_packet(sequence: int) -> Packet:
    payload = bytes.fromhex(VALID["nominal_no_flags"]["payload_hex"])
    return Packet(frame=encode_frame_fields(FrameType.TELEMETRY, sequence, payload))


def test_decoder_stores_with_time_sequence_and_pass() -> None:
    store = TelemetryStore()
    decoder = DownlinkDecoder(store)
    decoder.decode(100, _telemetry_packet(0))
    decoder.start_pass()
    decoder.decode(200, _telemetry_packet(1))
    assert store.history == (
        TelemetryRecord(100, 0, 0, NOMINAL),
        TelemetryRecord(200, 1, 1, NOMINAL),
    )
    assert decoder.pass_id == 1


def test_decoder_without_a_store_still_decodes() -> None:
    decoder = DownlinkDecoder()
    assert len(decoder.decode(100, _telemetry_packet(0))) == 1


def test_station_fills_its_store_and_reset_empties_it() -> None:
    station = GroundStation(StationConfig(telemetry_retention_count=2))
    for tick in range(3):
        station.end_tick((tick + 1) * 100_000, [_telemetry_packet(tick)])
    assert [r.sequence for r in station.telemetry] == [1, 2]
    assert station.telemetry.dropped_count == 1
    station.reset(RngFactory(1))
    assert len(station.telemetry) == 0
    assert station.telemetry.dropped_count == 0
