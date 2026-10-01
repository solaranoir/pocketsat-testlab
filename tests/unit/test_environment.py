"""Tests for EnvironmentState defaults and the NominalEnvironment cycle."""

import dataclasses
from fractions import Fraction
from itertools import pairwise

import pytest

from pocketsat.core.clock import SimClock
from pocketsat.environment import (
    DEFAULT_ECLIPSE_AMBIENT_TEMP_C,
    DEFAULT_ECLIPSE_FRACTION,
    DEFAULT_ORBIT_PERIOD_US,
    EnvironmentModel,
    NominalEnvironment,
)
from pocketsat.targets.base import EnvironmentState

# --- EnvironmentState defaults and validation ---------------------------------------


def test_defaults_are_nominal() -> None:
    env = EnvironmentState()
    assert env.sunlit is True
    assert env.ambient_temp_c == 20.0
    assert env.sensor_noise_scale == 1.0
    assert env.battery_soc_override is None


def test_fields_match_issue() -> None:
    names = [f.name for f in dataclasses.fields(EnvironmentState)]
    assert names == ["sunlit", "ambient_temp_c", "sensor_noise_scale", "battery_soc_override"]


def test_accepts_valid_values() -> None:
    env = EnvironmentState(
        sunlit=False, ambient_temp_c=-40, sensor_noise_scale=0.0, battery_soc_override=0.0
    )
    assert env.ambient_temp_c == -40
    assert EnvironmentState(battery_soc_override=1.0).battery_soc_override == 1.0


def test_is_frozen() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        EnvironmentState().sunlit = False  # type: ignore[misc]


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"sunlit": 1}, TypeError),
        ({"ambient_temp_c": "20"}, TypeError),
        ({"ambient_temp_c": True}, TypeError),
        ({"ambient_temp_c": -273.15}, ValueError),
        ({"ambient_temp_c": float("nan")}, ValueError),
        ({"sensor_noise_scale": -0.1}, ValueError),
        ({"sensor_noise_scale": float("inf")}, ValueError),
        ({"battery_soc_override": 1.01}, ValueError),
        ({"battery_soc_override": -0.01}, ValueError),
        ({"battery_soc_override": float("nan")}, ValueError),
    ],
)
def test_rejects_invalid_values(kwargs: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        EnvironmentState(**kwargs)  # type: ignore[arg-type]


# --- NominalEnvironment cycle ---------------------------------------------------------


def test_default_parameters() -> None:
    env = NominalEnvironment()
    assert env.orbit_period_us == DEFAULT_ORBIT_PERIOD_US == 5_520_000_000
    assert DEFAULT_ECLIPSE_FRACTION == 0.35
    assert env.eclipse_us == 1_932_000_000  # exactly 35%, no float error
    assert env.sunlit_us + env.eclipse_us == env.orbit_period_us
    assert isinstance(env, EnvironmentModel)


def test_cycle_boundaries() -> None:
    env = NominalEnvironment(orbit_period_us=1_000, eclipse_fraction=0.25)
    assert (env.sunlit_us, env.eclipse_us) == (750, 250)
    assert env.state_at(0).sunlit
    assert env.state_at(749).sunlit
    assert not env.state_at(750).sunlit
    assert not env.state_at(999).sunlit
    assert env.state_at(1_000).sunlit  # next orbit starts in sunlight


def test_cycle_repeats_every_orbit() -> None:
    env = NominalEnvironment(orbit_period_us=6_000_000, eclipse_fraction=Fraction(1, 3))
    for t in range(0, 6_000_000, 50_000):
        for k in (1, 7, 1_000):
            assert env.state_at(t + k * 6_000_000) == env.state_at(t)


def test_cycle_over_sim_clock_ticks() -> None:
    env = NominalEnvironment()  # 92 min orbit, 35% eclipse
    clock = SimClock()  # 100 ms ticks
    ticks_per_orbit = env.orbit_period_us // clock.tick_us
    states = []
    for _ in range(ticks_per_orbit * 3):
        states.append(env.sample(clock))
        clock.advance_one_tick()
    eclipse_ticks = sum(not s.sunlit for s in states[:ticks_per_orbit])
    assert eclipse_ticks == env.eclipse_us // clock.tick_us == 19_320
    # One sunlit block then one eclipse block per orbit, identical every orbit.
    transitions = sum(a.sunlit != b.sunlit for a, b in pairwise(states))
    assert transitions == 5
    assert states[:ticks_per_orbit] == states[ticks_per_orbit : 2 * ticks_per_orbit]


def test_repeatable_across_instances() -> None:
    a = NominalEnvironment(orbit_period_us=5_400_000_000, eclipse_fraction=0.4)
    b = NominalEnvironment(orbit_period_us=5_400_000_000, eclipse_fraction=0.4)
    times = range(0, 3 * 5_400_000_000, 7_777_777)
    assert [a.state_at(t) for t in times] == [b.state_at(t) for t in times]


def test_other_fields_are_constant_and_configurable() -> None:
    env = NominalEnvironment(
        orbit_period_us=100,
        eclipse_fraction=0.5,
        ambient_temp_c=-10.0,
        eclipse_ambient_temp_c=-30.0,
        sensor_noise_scale=2.0,
    )
    assert env.state_at(0).ambient_temp_c == -10.0  # sunlit
    assert env.state_at(60).ambient_temp_c == -30.0  # eclipse
    for t in (0, 60):
        state = env.state_at(t)
        assert state.sensor_noise_scale == 2.0
        assert state.battery_soc_override is None
    assert NominalEnvironment().state_at(0) == EnvironmentState()


# --- Eclipse ambient temperature (#73) ------------------------------------------------


def test_default_eclipse_ambient() -> None:
    assert DEFAULT_ECLIPSE_AMBIENT_TEMP_C == -20.0
    env = NominalEnvironment()
    assert env.state_at(0).ambient_temp_c == 20.0
    assert env.state_at(env.sunlit_us).ambient_temp_c == -20.0


def test_ambient_switches_exactly_at_boundaries() -> None:
    env = NominalEnvironment(
        orbit_period_us=1_000, eclipse_fraction=0.25, eclipse_ambient_temp_c=-40.0
    )
    expected = [
        (0, 20.0),
        (749, 20.0),  # last sunlit microsecond
        (750, -40.0),  # first eclipse microsecond
        (999, -40.0),  # last eclipse microsecond
        (1_000, 20.0),  # next orbit, sunlit again
        (1_750, -40.0),
    ]
    for now_us, ambient in expected:
        state = env.state_at(now_us)
        assert state.ambient_temp_c == ambient, now_us
        assert state.sunlit is (ambient == 20.0)


def test_eclipse_ambient_on_every_eclipse_tick_over_several_orbits() -> None:
    env = NominalEnvironment(eclipse_ambient_temp_c=-25.0)  # 92 min orbit, 35% eclipse
    clock = SimClock()  # 100 ms ticks
    ticks = 3 * env.orbit_period_us // clock.tick_us
    eclipse_ticks = 0
    for _ in range(ticks):
        state = env.sample(clock)
        assert state.ambient_temp_c == (20.0 if state.sunlit else -25.0)
        eclipse_ticks += not state.sunlit
        clock.advance_one_tick()
    assert eclipse_ticks == 3 * 19_320


def test_eclipse_ambient_with_extreme_fractions() -> None:
    always_dark = NominalEnvironment(
        orbit_period_us=1_000, eclipse_fraction=1, eclipse_ambient_temp_c=-60.0
    )
    never_dark = NominalEnvironment(
        orbit_period_us=1_000, eclipse_fraction=0, eclipse_ambient_temp_c=-60.0
    )
    assert {always_dark.state_at(t).ambient_temp_c for t in range(0, 3_000, 7)} == {-60.0}
    assert {never_dark.state_at(t).ambient_temp_c for t in range(0, 3_000, 7)} == {20.0}


@pytest.mark.parametrize(
    ("value", "error"),
    [
        (-273.15, ValueError),
        (-300.0, ValueError),
        (float("nan"), ValueError),
        (float("inf"), ValueError),
        ("-20", TypeError),
        (True, TypeError),
    ],
)
def test_invalid_eclipse_ambient_rejected(value: object, error: type[Exception]) -> None:
    with pytest.raises(error, match="eclipse_ambient_temp_c"):
        NominalEnvironment(eclipse_ambient_temp_c=value)  # type: ignore[arg-type]


@pytest.mark.parametrize(("fraction", "always_sunlit"), [(0, True), (0.0, True), (1, False)])
def test_eclipse_fraction_extremes(fraction: float, always_sunlit: bool) -> None:
    env = NominalEnvironment(orbit_period_us=1_000, eclipse_fraction=fraction)
    assert {env.state_at(t).sunlit for t in range(0, 3_000, 7)} == {always_sunlit}


def test_eclipse_rounds_to_nearest_microsecond() -> None:
    assert NominalEnvironment(orbit_period_us=3, eclipse_fraction=Fraction(1, 3)).eclipse_us == 1
    assert NominalEnvironment(orbit_period_us=7, eclipse_fraction=0.3).eclipse_us == 2  # 2.1


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"orbit_period_us": 0}, ValueError),
        ({"orbit_period_us": -1}, ValueError),
        ({"orbit_period_us": 5.52e9}, TypeError),
        ({"eclipse_fraction": 1.1}, ValueError),
        ({"eclipse_fraction": -0.1}, ValueError),
        ({"eclipse_fraction": float("nan")}, ValueError),
        ({"eclipse_fraction": "0.35"}, TypeError),
        ({"eclipse_fraction": True}, TypeError),
        ({"ambient_temp_c": -300.0}, ValueError),
    ],
)
def test_rejects_invalid_parameters(kwargs: dict[str, object], error: type[Exception]) -> None:
    with pytest.raises(error):
        NominalEnvironment(**kwargs)  # type: ignore[arg-type]


@pytest.mark.parametrize(("now_us", "error"), [(-1, ValueError), (0.5, TypeError)])
def test_state_at_requires_integer_time(now_us: object, error: type[Exception]) -> None:
    with pytest.raises(error):
        NominalEnvironment().state_at(now_us)  # type: ignore[arg-type]
