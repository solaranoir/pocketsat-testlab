"""Naming and units convention check (#76, docs/architecture.md §4.1).

Walks every record dataclass in ``pocketsat`` and checks every numeric (``int`` or
``float``) field: its name must end in an approved unit suffix, be an ID (``_id``) or a
count (``_count``), or be on the unitless allowlist. Non-numeric fields (booleans,
enums, bytes, strings, collections, nested records) are exempt.

Adding a suffix or an allowlist entry means updating architecture §4.1 in the same
change; a test below checks the two agree.
"""

import dataclasses
import importlib
import pkgutil
import types
import typing
from pathlib import Path
from typing import Any

import pytest

import pocketsat

REPO_ROOT = Path(__file__).resolve().parents[2]

UNIT_SUFFIXES = (
    "_v",
    "_a",
    "_w",
    "_wh",
    "_c",
    "_deg",
    "_dps",
    "_bytes",
    "_us",
    "_ms",
    "_s",
    "_km",
    "_hz",
    "_db",
)
"""Approved unit suffixes (architecture §4.1)."""

UNITLESS_SUFFIXES = ("_id", "_count")
"""IDs and counts."""

FRACTIONS = frozenset(
    {"soc", "buffer_fill", "sensor_noise_scale", "battery_soc_override", "loss_probability"}
)
"""Named fractions, kept in 0..1."""

EXISTING_UNITLESS = frozenset({"sequence", "version", "schema_version", "master_seed"})
"""Existing Phase 0 unitless numbers, kept as they are."""


def follows_convention(name: str) -> bool:
    """Whether a numeric field name follows the convention."""
    return (
        name.endswith(UNIT_SUFFIXES + UNITLESS_SUFFIXES)
        or name in FRACTIONS
        or name in EXISTING_UNITLESS
    )


def is_numeric(hint: Any) -> bool:
    """Whether a type hint is ``int`` or ``float``, alone, in a union, or optional.

    ``bool`` is a separate, exempt type even though it subclasses ``int``.
    """
    if hint in (int, float):
        return True
    if typing.get_origin(hint) in (typing.Union, types.UnionType):
        args = [a for a in typing.get_args(hint) if a is not type(None)]
        return bool(args) and all(a in (int, float) for a in args)
    return False


def record_dataclasses() -> list[type[Any]]:
    """Every dataclass defined in a ``pocketsat`` module."""
    found: dict[str, type[Any]] = {}
    for info in pkgutil.walk_packages(pocketsat.__path__, prefix="pocketsat."):
        module = importlib.import_module(info.name)
        for obj in vars(module).values():
            if (
                isinstance(obj, type)
                and dataclasses.is_dataclass(obj)
                and obj.__module__ == module.__name__
            ):
                found[f"{obj.__module__}.{obj.__qualname__}"] = obj
    return [found[key] for key in sorted(found)]


RECORDS = record_dataclasses()


def numeric_fields(record: type[Any]) -> list[str]:
    hints = typing.get_type_hints(record)
    return [f.name for f in dataclasses.fields(record) if is_numeric(hints[f.name])]


def test_walk_finds_every_kind_of_record() -> None:
    names = {r.__name__ for r in RECORDS}
    assert {
        "PowerTruth",
        "CommsReadings",
        "PowerSnapshot",
        "SpacecraftControls",
        "PayloadControls",
        "PowerConfig",
        "SpacecraftConfig",
        "ThermalInitial",
        "EnvironmentState",
        "TargetFault",
        "Command",
        "Telemetry",
        "Packet",
        "Frame",
        "RunRecord",
    } <= names


@pytest.mark.parametrize("record", RECORDS, ids=lambda r: f"{r.__module__}.{r.__qualname__}")
def test_numeric_fields_follow_the_naming_convention(record: type[Any]) -> None:
    bad = [name for name in numeric_fields(record) if not follows_convention(name)]
    assert not bad, (
        f"{record.__qualname__}: numeric fields {bad} need a unit suffix "
        f"{UNIT_SUFFIXES}, an _id/_count suffix, or an allowlist entry (architecture §4.1)"
    )


@pytest.mark.parametrize(
    ("name", "ok"),
    [
        ("bus_v", True),
        ("battery_current_a", True),
        ("pack_wh", True),
        ("next_chunk_id", True),
        ("uplink_lost_count", True),
        ("soc", True),
        ("sequence", True),
        ("voltage", False),
        ("temperature", False),
        ("rate", False),
        ("buffer_percent", False),
    ],
)
def test_rule(name: str, ok: bool) -> None:
    assert follows_convention(name) is ok


@pytest.mark.parametrize(
    ("hint", "numeric"),
    [
        (int, True),
        (float, True),
        (float | None, True),
        (int | float, True),
        (bool, False),
        (str, False),
        (bytes, False),
        (frozenset[str], False),
        (str | None, False),
    ],
)
def test_numeric_detection(hint: Any, numeric: bool) -> None:
    assert is_numeric(hint) is numeric


def test_architecture_documents_the_same_lists() -> None:
    text = (REPO_ROOT / "docs" / "architecture.md").read_text(encoding="utf-8")
    start = text.index("#### Naming and units convention")
    section = text[start : text.index("\n## ", start)]
    for entry in (*UNIT_SUFFIXES, *UNITLESS_SUFFIXES, *FRACTIONS, *EXISTING_UNITLESS):
        assert f"`{entry}`" in section, f"{entry} is not documented in architecture §4.1"
