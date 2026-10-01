"""Tests for the subsystem snapshot contracts (#76, ADR-0004 §7)."""

import ast
import dataclasses
import re
from pathlib import Path
from typing import Any

import pytest

from pocketsat.spacecraft import snapshots
from pocketsat.spacecraft.base import STEP_ORDER, SubsystemSnapshot
from pocketsat.spacecraft.fakes import default_snapshot
from pocketsat.spacecraft.snapshots import (
    READINGS_FLAGS,
    SNAPSHOT_TYPES,
    AttitudeState,
    CommsReadings,
    CommsSnapshot,
    CommsTruth,
    ContractSnapshot,
    PayloadReadings,
    PayloadSnapshot,
    PayloadState,
    PayloadTruth,
    PowerReadings,
    PowerSnapshot,
    PowerTruth,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SNAPSHOTS_SOURCE = Path(snapshots.__file__)

TELEMETRY_FLAG_NAMES = {"low_battery", "critical_battery", "over_temp", "under_temp"}
"""The flags reserved in #54's flag table (bits 0, 1, 4, 5)."""


def _records() -> list[type[Any]]:
    """Every truth and readings record type in the contract."""
    out: list[type[Any]] = []
    for kind in SNAPSHOT_TYPES.values():
        out += [kind.truth_type, kind.readings_type]
    return out


RECORDS = _records()


def _example(record: type[Any]) -> Any:
    snap = default_snapshot(_subsystem_of(record))
    return snap.truth if record is type(snap.truth) else snap.readings


def _subsystem_of(record: type[Any]) -> str:
    for name, kind in SNAPSHOT_TYPES.items():
        if record in (kind.truth_type, kind.readings_type):
            return name
    raise AssertionError(f"{record.__name__} is not a contract record")


# --- Shape --------------------------------------------------------------------------------


def test_every_subsystem_has_a_contract_in_step_order() -> None:
    assert tuple(SNAPSHOT_TYPES) == STEP_ORDER


@pytest.mark.parametrize("name", STEP_ORDER)
def test_truth_and_readings_types_are_named_for_the_subsystem(name: str) -> None:
    kind = SNAPSHOT_TYPES[name]
    prefix = kind.__name__.removesuffix("Snapshot")
    assert kind.truth_type.__name__ == f"{prefix}Truth"
    assert kind.readings_type.__name__ == f"{prefix}Readings"
    assert issubclass(kind, SubsystemSnapshot)
    assert [f.name for f in dataclasses.fields(kind)] == ["truth", "readings"]


def test_only_payload_and_comms_are_mirrored() -> None:
    assert {n for n, k in SNAPSHOT_TYPES.items() if k.mirrored} == {"payload", "comms"}


@pytest.mark.parametrize("record", RECORDS, ids=lambda r: r.__name__)
def test_every_record_is_frozen(record: type[Any]) -> None:
    assert dataclasses.is_dataclass(record)
    instance = _example(record)
    field = dataclasses.fields(record)[0].name
    with pytest.raises(dataclasses.FrozenInstanceError):
        setattr(instance, field, getattr(instance, field))


@pytest.mark.parametrize("name", STEP_ORDER)
def test_every_snapshot_is_frozen(name: str) -> None:
    snap = default_snapshot(name)
    with pytest.raises(dataclasses.FrozenInstanceError):
        snap.truth = snap.truth  # type: ignore[misc]


def _field_docstrings() -> dict[str, dict[str, str | None]]:
    """For each class in the contract module, each annotated field and its docstring."""
    tree = ast.parse(SNAPSHOTS_SOURCE.read_text(encoding="utf-8"))
    out: dict[str, dict[str, str | None]] = {}
    for node in tree.body:
        if not isinstance(node, ast.ClassDef):
            continue
        docs: dict[str, str | None] = {}
        for stmt, following in zip(node.body, [*node.body[1:], None], strict=True):
            if isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name):
                doc = None
                if (
                    isinstance(following, ast.Expr)
                    and isinstance(following.value, ast.Constant)
                    and isinstance(following.value.value, str)
                ):
                    doc = following.value.value
                docs[stmt.target.id] = doc
        out[node.name] = docs
    return out


FIELD_DOCS = _field_docstrings()


@pytest.mark.parametrize("record", RECORDS, ids=lambda r: r.__name__)
def test_every_field_has_a_docstring(record: type[Any]) -> None:
    for field in dataclasses.fields(record):
        owner = next(c for c in record.__mro__ if field.name in FIELD_DOCS.get(c.__name__, {}))
        doc = FIELD_DOCS[owner.__name__][field.name]
        assert doc, f"{record.__name__}.{field.name} has no docstring"


# --- Flags and states --------------------------------------------------------------------


def test_flags_are_bools_on_readings_only() -> None:
    for name, kind in SNAPSHOT_TYPES.items():
        truth_fields = {f.name for f in dataclasses.fields(kind.truth_type)}
        readings_types = {f.name: f.type for f in dataclasses.fields(kind.readings_type)}
        # A flag is a bool on the readings record with no counterpart on the truth record.
        flags = {n for n, t in readings_types.items() if t is bool and n not in truth_fields}
        assert flags == set(READINGS_FLAGS.get(name, ())), name
        assert not truth_fields & TELEMETRY_FLAG_NAMES, name


def test_flag_names_are_the_telemetry_flag_names() -> None:
    names = [flag for flags in READINGS_FLAGS.values() for flag in flags]
    assert len(names) == len(set(names))
    assert set(names) == TELEMETRY_FLAG_NAMES


def test_states_are_contract_enums() -> None:
    assert [s.name for s in AttitudeState] == ["TUMBLING", "DETUMBLING", "STABILIZED"]
    assert [s.name for s in PayloadState] == ["OFF", "IDLE", "ACQUIRING"]


# --- Behavior -----------------------------------------------------------------------------


def test_snapshot_rejects_wrong_record_types() -> None:
    power = default_snapshot("power")
    with pytest.raises(TypeError, match="truth must be a PowerTruth"):
        PowerSnapshot(truth=power.readings, readings=power.readings)
    with pytest.raises(TypeError, match="readings must be a PowerReadings"):
        PowerSnapshot(truth=power.truth, readings=power.truth)


def test_mirrored_snapshot_requires_readings_equal_to_truth() -> None:
    payload = default_snapshot("payload")
    truth = dataclasses.replace(payload.truth, power_w=2.0)
    with pytest.raises(ValueError, match="readings must equal its truth"):
        PayloadSnapshot(truth=truth, readings=payload.readings)
    snap = PayloadSnapshot.from_truth(truth)
    assert isinstance(snap.readings, PayloadReadings)
    assert dataclasses.astuple(snap.readings) == dataclasses.astuple(truth)


def test_mirrored_readings_are_a_distinct_type() -> None:
    comms = default_snapshot("comms")
    assert isinstance(comms, CommsSnapshot)
    assert type(comms.truth) is CommsTruth
    assert type(comms.readings) is CommsReadings
    assert not isinstance(comms.readings, CommsTruth)


def test_from_truth_is_only_for_mirrored_subsystems() -> None:
    with pytest.raises(TypeError, match="has sensors"):
        PowerSnapshot.from_truth(default_snapshot("power").truth)


def test_buffer_fill_is_derived() -> None:
    truth = dataclasses.replace(
        default_snapshot("payload").truth, buffered_bytes=1024, buffer_capacity_bytes=4096
    )
    assert truth.buffer_fill == 0.25
    assert PayloadSnapshot.from_truth(truth).readings.buffer_fill == 0.25


def test_net_power_is_generation_minus_load() -> None:
    truth = PowerTruth(
        bus_v=7.4, battery_current_a=-0.4, soc=0.5, generation_w=0.0, total_load_w=3.0
    )
    assert truth.net_power_w == -3.0


def test_contract_records_are_exported() -> None:
    from pocketsat import spacecraft

    for record in [*RECORDS, *SNAPSHOT_TYPES.values(), ContractSnapshot]:
        assert getattr(spacecraft, record.__name__) is record
    assert PowerReadings in RECORDS
    assert PayloadTruth in RECORDS


# --- Cross-read table (docs/spacecraft.md) -----------------------------------------------

ROW = re.compile(
    r"^\|\s*(?P<reader>[a-z ]+?)\s*\|\s*`(?P<record>\w+)\.(?P<field>\w+)`\s*\|"
    r"\s*(?P<use>physics|decision)\b[^|]*\|\s*(?P<when>current|previous) tick\s*\|$"
)


def cross_reads() -> list[dict[str, str]]:
    """Rows of the cross-read tables in docs/spacecraft.md."""
    text = (REPO_ROOT / "docs" / "spacecraft.md").read_text(encoding="utf-8")
    return [m.groupdict() for line in text.splitlines() if (m := ROW.match(line))]


CROSS_READS = cross_reads()


def test_cross_read_table_is_found() -> None:
    readers = {row["reader"] for row in CROSS_READS}
    assert {"power", "thermal", "payload", "flight computer"} <= readers
    assert len(CROSS_READS) >= 18


@pytest.mark.parametrize(
    "row", CROSS_READS, ids=lambda r: f"{r['reader']}->{r['record']}.{r['field']}"
)
def test_cross_read_row_matches_contract_and_timing_rule(row: dict[str, str]) -> None:
    record = getattr(snapshots, row["record"])
    assert hasattr(record, row["field"]) or row["field"] in {
        f.name for f in dataclasses.fields(record)
    }, f"{row['record']} has no field {row['field']}"
    source = _subsystem_of(record)
    reader = row["reader"]
    assert reader != source
    # Physics reads truth; decisions read readings (ADR-0004 §6).
    expected_use = "physics" if record is SNAPSHOT_TYPES[source].truth_type else "decision"
    assert row["use"] == expected_use
    # Earlier in STEP_ORDER: current tick; later: previous tick (#36). The flight
    # computer runs after every subsystem.
    if reader == "flight computer":
        expected_when = "current"
    else:
        earlier = STEP_ORDER.index(source) < STEP_ORDER.index(reader)
        expected_when = "current" if earlier else "previous"
    assert row["when"] == expected_when
