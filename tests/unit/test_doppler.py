"""Tests for the Doppler shift and the station's radio tuning (#162)."""

import dataclasses

import pytest

from pocketsat.core.rng import RngFactory
from pocketsat.doppler import SPEED_OF_LIGHT_KM_PER_S, doppler_shift_hz
from pocketsat.environment.geometry import PassGeometry
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    Aos,
    GroundStation,
    Los,
    RadioTuned,
    StationConfig,
)
from pocketsat.groundstation.tuning import DopplerTuner, round_to_step
from pocketsat.link import DEFAULT_DOWNLINK_FREQUENCY_HZ, IdealLink

UPLINK_HZ = DEFAULT_STATION_CONFIG.uplink_frequency_hz
DOWNLINK_HZ = DEFAULT_STATION_CONFIG.downlink_frequency_hz


def _geometry(elevation_deg: float, range_rate_km_per_s: float) -> PassGeometry:
    return PassGeometry(
        azimuth_deg=90.0,
        elevation_deg=elevation_deg,
        range_km=1_000.0,
        range_rate_km_per_s=range_rate_km_per_s,
    )


# --- The Doppler shift ------------------------------------------------------------


def test_speed_of_light_is_the_si_value() -> None:
    assert SPEED_OF_LIGHT_KM_PER_S == 299_792.458


@pytest.mark.parametrize(
    ("frequency_hz", "range_rate", "expected_hz"),
    [
        # Exact values, so a reordering of the arithmetic shows up on any platform.
        (435_500_000, -3.5, 5084.350721057833),
        (145_850_000, -3.5, 1702.7613149627668),
        (435_500_000, 6.25, -9079.197716174702),
        (145_850_000, 7.0, -3405.5226299255337),
    ],
)
def test_doppler_shift_pinned_values(
    frequency_hz: int, range_rate: float, expected_hz: float
) -> None:
    assert doppler_shift_hz(frequency_hz, range_rate) == expected_hz


def test_doppler_is_positive_approaching_zero_at_closest_approach_negative_receding() -> None:
    assert doppler_shift_hz(DOWNLINK_HZ, -1.0) > 0.0
    assert doppler_shift_hz(DOWNLINK_HZ, 0.0) == 0.0
    assert doppler_shift_hz(DOWNLINK_HZ, 1.0) < 0.0


def test_doppler_magnitude_is_f_v_over_c() -> None:
    # LEO orbital speed, about 7.7 km/s, gives about 11 kHz at 435.5 MHz.
    assert doppler_shift_hz(DOWNLINK_HZ, -7.7) == pytest.approx(DOWNLINK_HZ * 7.7 / 299_792.458)
    assert 11_000 < doppler_shift_hz(DOWNLINK_HZ, -7.7) < 11_300
    # Proportional to the carrier frequency, antisymmetric in the range rate.
    assert doppler_shift_hz(2 * UPLINK_HZ, 2.0) == 2 * doppler_shift_hz(UPLINK_HZ, 2.0)
    assert doppler_shift_hz(UPLINK_HZ, -2.0) == -doppler_shift_hz(UPLINK_HZ, 2.0)


# --- Rounding to the tuning step --------------------------------------------------


@pytest.mark.parametrize(
    ("shift_hz", "step_hz", "expected"),
    [
        (0.0, 10, 0),
        (4.999, 10, 0),
        (5.0, 10, 10),  # ties round up
        (-5.0, 10, 0),
        (-5.001, 10, -10),
        (5084.35, 10, 5080),
        (-9079.2, 10, -9080),
        (1702.76, 1, 1703),
        (1702.76, 1000, 2000),
    ],
)
def test_round_to_step(shift_hz: float, step_hz: int, expected: int) -> None:
    result = round_to_step(shift_hz, step_hz)
    assert result == expected
    assert type(result) is int


# --- The tuner --------------------------------------------------------------------


def _tuner(*, correction: bool = True, step_hz: int = 10) -> DopplerTuner:
    return DopplerTuner(UPLINK_HZ, DOWNLINK_HZ, step_hz, correction)


def test_corrected_tuning_follows_the_predicted_shift() -> None:
    event = _tuner().tune(7, _geometry(30.0, -3.5))
    assert event == RadioTuned(
        time_us=7,
        uplink_doppler_hz=1702.7613149627668,
        downlink_doppler_hz=5084.350721057833,
        uplink_frequency_hz=UPLINK_HZ - 1700,  # transmit low; the uplink is shifted up
        downlink_frequency_hz=DOWNLINK_HZ + 5080,  # receive where the downlink lands
        uplink_residual_hz=1702.7613149627668 - 1700,
        downlink_residual_hz=5084.350721057833 - 5080,
    )


@pytest.mark.parametrize("range_rate", [-7.3, -3.5, -0.01, 0.0, 0.004, 2.25, 7.3])
@pytest.mark.parametrize("step_hz", [1, 10, 100])
def test_corrected_residual_is_at_most_half_a_step(range_rate: float, step_hz: int) -> None:
    event = _tuner(step_hz=step_hz).tune(0, _geometry(45.0, range_rate))
    assert abs(event.uplink_residual_hz) <= step_hz / 2
    assert abs(event.downlink_residual_hz) <= step_hz / 2
    # The spacecraft hears the station within the residual of the nominal uplink, and the
    # downlink lands within the residual of the tuned receive frequency.
    assert event.uplink_frequency_hz + event.uplink_doppler_hz - UPLINK_HZ == pytest.approx(
        event.uplink_residual_hz
    )
    assert DOWNLINK_HZ + event.downlink_doppler_hz - event.downlink_frequency_hz == (
        pytest.approx(event.downlink_residual_hz)
    )


def test_uncorrected_tuning_stays_nominal_and_leaves_the_full_shift() -> None:
    event = _tuner(correction=False).tune(0, _geometry(30.0, -3.5))
    assert event.uplink_frequency_hz == UPLINK_HZ
    assert event.downlink_frequency_hz == DOWNLINK_HZ
    assert event.uplink_residual_hz == event.uplink_doppler_hz == 1702.7613149627668
    assert event.downlink_residual_hz == event.downlink_doppler_hz == 5084.350721057833


# --- Configuration ----------------------------------------------------------------


def test_doppler_configuration_defaults() -> None:
    assert DEFAULT_STATION_CONFIG.doppler_correction is True
    assert DEFAULT_STATION_CONFIG.tuning_step_hz == 10


@pytest.mark.parametrize("bad", [1, "yes", None])
def test_doppler_correction_must_be_a_bool(bad: object) -> None:
    with pytest.raises(TypeError):
        StationConfig(doppler_correction=bad)  # type: ignore[arg-type]


@pytest.mark.parametrize(("bad", "error"), [(0, ValueError), (-10, ValueError), (2.5, TypeError)])
def test_tuning_step_must_be_a_positive_int(bad: object, error: type[Exception]) -> None:
    with pytest.raises(error):
        StationConfig(tuning_step_hz=bad)  # type: ignore[arg-type]


# --- The station ------------------------------------------------------------------


def _tuned(events: tuple[object, ...]) -> list[RadioTuned]:
    return [event for event in events if isinstance(event, RadioTuned)]


def test_station_tunes_every_tick_of_a_pass_after_aos_and_not_outside() -> None:
    station = GroundStation()
    station.reset(RngFactory(1))
    elevations = [5.0, 12.0, 40.0, 12.0, 5.0, 3.0]
    range_rates = [-6.0, -5.0, 0.0, 5.0, 6.0, 6.2]
    per_tick = []
    for tick, (elevation, range_rate) in enumerate(zip(elevations, range_rates, strict=True)):
        per_tick.append(station.begin_tick(tick * 100_000, _geometry(elevation, range_rate)).events)

    assert [len(_tuned(events)) for events in per_tick] == [0, 1, 1, 1, 0, 0]
    aos_events = per_tick[1]
    assert isinstance(aos_events[0], Aos) and isinstance(aos_events[1], RadioTuned)
    assert isinstance(per_tick[4][0], Los)
    (approaching,), (overhead,), (receding,) = (_tuned(per_tick[i]) for i in (1, 2, 3))
    assert approaching.downlink_doppler_hz > 0.0
    assert overhead.downlink_doppler_hz == 0.0
    assert receding.downlink_doppler_hz < 0.0
    assert approaching.time_us == 100_000


def test_station_without_geometry_does_not_tune() -> None:
    station = GroundStation()
    assert _tuned(station.begin_tick(0, None).events) == []


def test_station_uses_its_configured_frequencies_and_switch() -> None:
    config = dataclasses.replace(
        DEFAULT_STATION_CONFIG,
        uplink_frequency_hz=145_900_000,
        downlink_frequency_hz=437_000_000,
        tuning_step_hz=1,
        doppler_correction=False,
    )
    station = GroundStation(config)
    (event,) = _tuned(station.begin_tick(0, _geometry(30.0, -3.5)).events)
    assert event.uplink_doppler_hz == doppler_shift_hz(145_900_000, -3.5)
    assert event.downlink_doppler_hz == doppler_shift_hz(437_000_000, -3.5)
    assert event.uplink_frequency_hz == 145_900_000
    assert event.downlink_frequency_hz == 437_000_000


# --- The ideal link ---------------------------------------------------------------


def test_ideal_link_default_carrier_is_the_station_default() -> None:
    assert DEFAULT_DOWNLINK_FREQUENCY_HZ == DOWNLINK_HZ


def test_ideal_link_puts_the_true_downlink_shift_on_packets() -> None:
    geometry = _geometry(30.0, -3.5)
    (packet,) = IdealLink().carry_downlink([b"frame"], 0, geometry)
    assert packet.doppler_hz == 5084.350721057833
    (packet,) = IdealLink(downlink_frequency_hz=145_850_000).carry_downlink([b"x"], 0, geometry)
    assert packet.doppler_hz == 1702.7613149627668
    (packet,) = IdealLink().carry_downlink([b"frame"], 0, None)
    assert packet.doppler_hz is None


@pytest.mark.parametrize(("bad", "error"), [(0, ValueError), (True, TypeError), (1.5, TypeError)])
def test_ideal_link_rejects_a_bad_carrier(bad: object, error: type[Exception]) -> None:
    with pytest.raises(error):
        IdealLink(downlink_frequency_hz=bad)  # type: ignore[arg-type]
