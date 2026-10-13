"""Unit tests for the PassGeometry record (ADR-0010 §6)."""

import dataclasses

import pytest

from pocketsat.environment.geometry import PassGeometry

VALID = {
    "azimuth_deg": 0.0,
    "elevation_deg": -5.0,
    "range_km": 2500.0,
    "range_rate_km_per_s": 6.8,
}


def test_valid_record_is_frozen() -> None:
    geometry = PassGeometry(**VALID)
    assert geometry.elevation_deg == -5.0
    with pytest.raises(dataclasses.FrozenInstanceError):
        geometry.range_km = 1.0  # type: ignore[misc]


@pytest.mark.parametrize(
    "changes",
    [
        {"azimuth_deg": 359.999},
        {"elevation_deg": 90.0},
        {"elevation_deg": -90.0},
        {"range_km": 0.0},
        {"range_rate_km_per_s": -7.5},
        {"azimuth_deg": 10},
    ],
)
def test_bounds(changes: dict[str, float]) -> None:
    PassGeometry(**{**VALID, **changes})


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"azimuth_deg": 360.0}, ValueError),
        ({"azimuth_deg": -0.1}, ValueError),
        ({"elevation_deg": 90.1}, ValueError),
        ({"elevation_deg": -90.1}, ValueError),
        ({"range_km": -1.0}, ValueError),
        ({"range_rate_km_per_s": float("nan")}, ValueError),
        ({"range_km": float("inf")}, ValueError),
        ({"azimuth_deg": True}, TypeError),
        ({"elevation_deg": "10"}, TypeError),
    ],
)
def test_validation(changes: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        PassGeometry(**{**VALID, **changes})  # type: ignore[arg-type]
