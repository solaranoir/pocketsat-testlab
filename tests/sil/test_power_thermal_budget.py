"""Power, thermal, and data budget tests (#72). See ``docs/power-thermal-budget.md``.

The real ``Power``, ``Thermal``, ``Attitude``, ``Payload``, and ``Comms`` run in one
``SubsystemStack`` sharing a ``SnapshotBoard``, driven by ``NominalEnvironment``
(92-minute orbit, 35% eclipse, 20 °C sunlit and -20 °C eclipse ambient) and a
``SimClock`` at the default 100 ms tick, never coarsened: transmit capacity and the
per-byte transmit draw are per tick, so the budget only holds at 100 ms.

There is no flight computer yet, so each profile scripts ``SpacecraftControls`` the way
the flight computer's mode table will (#47), through ``ProfileDriver``. The profiles, the
driver, and the traffic and release stand-ins (until #56 and #59 send real traffic) live
in ``_reference_profile.py``, shared with the epic #30 close-out test (#70).

Re-check this budget once #56 and #59 send real traffic.

Every profile runs whole orbits at the default tick. Failure messages name the margin
that was violated and its actual value. Each profile's results are printed (run with
``-s`` to see them).
"""

import itertools
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import pytest
from _reference_profile import (
    BEACON_PERIOD_US,
    PASS_US,
    REFERENCE,
    US_PER_S,
    Profile,
    ProfileDriver,
    data_frames_in_tick,
    real_stack,
)

from pocketsat.core.clock import SimClock
from pocketsat.environment import NominalEnvironment
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    STRESSED_CONFIG,
    CommsSnapshot,
    PayloadSnapshot,
    PowerSnapshot,
    SpacecraftConfig,
    SpacecraftInitialState,
    SpacecraftState,
    ThermalSnapshot,
)

# Multi-orbit budget runs: skipped by a plain `pytest`, run with `pytest -m slow`.
pytestmark = pytest.mark.slow

SEED = 1
S_PER_H = 3600

# --- Budget requirements (#72) ---------------------------------------------------------

MARGIN_BAND = (0.05, 0.25)
"""Reference profile's orbit energy margin, (generation - consumption) / consumption."""

MIN_SOC_ABOVE_LOW_BAND = (0.10, 0.30)
"""Reference profile's minimum SOC above ``low_battery_soc``, as SOC fractions."""

TEMPERATURE_MARGIN_C = 5.0
"""Every temperature stays this far inside its under- and over-temperature limits."""

NOISE_SIGMAS = 5.0
"""Reported values stay this many standard deviations of nominal noise away from every
flag threshold."""

DATA_MARGIN = 0.20
"""Science data per orbit is at most ``1 - DATA_MARGIN`` of one pass's net capacity."""

ACCOUNTING_TOLERANCE_WH = 1e-6
"""Energy accounting: |battery energy change - (generation - consumption)| per orbit.
The model has no conversion losses, and the profiles never reach full or empty, where
power would discard energy; the residue is floating-point summation only."""

SENSITIVITY_LOAD_W = 1.0
"""Known extra load for the sensitivity check, watts."""

SENSITIVITY_RAW_TOLERANCE = 0.20
"""The raw SOC change from :data:`SENSITIVITY_LOAD_W` may differ from the analytic
``1 W x orbit / capacity`` by this fraction: the extra watt also heats the battery, so
the survival heater runs less (about 12% of the effect). A sign error or a unit error
(hours, kilo) is off by a factor of 2 or more."""

DOWNLINK_LOW_BATTERY_ORBITS = 3
"""Continuous DOWNLINK raises ``low_battery`` within this many orbits from the default
start (about 1.9 Wh lost per orbit; with seed 1 it sets at the end of the second
orbit)."""

STRESSED_LOW_BATTERY_ORBITS = 2
"""SCIENCE under :data:`STRESSED_CONFIG` raises ``low_battery`` within this many orbits
from the default start (about 1.7 Wh lost per orbit from a 14 Wh battery; with seed 1
it sets about 1.9 orbits in)."""

COLD_AMBIENT_C = -150.0
"""The contract suite's severe cold environment, applied in sunlight and eclipse."""

COLD_FLAG_WITHIN_US = 5 * 60 * US_PER_S
"""In the cold case ``under_temp`` sets within this time of the start (about 2 minutes:
the electronics, unheated, cross -25 °C first)."""

COLD_HEATER_DUTY = 0.95
"""In the cold case the heater is on for at least this fraction of the ticks after it
first switches on."""

NOMINAL = Profile("NOMINAL", payload=False, downlink="none")
SCIENCE = Profile("SCIENCE", downlink="none")
CONTINUOUS_DOWNLINK = Profile("continuous DOWNLINK", downlink="continuous")
TUMBLING = Profile("tumbling (attitude control off)", attitude=False)
STRESSED_SCIENCE = Profile("SCIENCE, stressed configuration", downlink="none")
COLD_SCIENCE = Profile("SCIENCE, -150 °C cold case", downlink="none")


@dataclass
class OrbitStats:
    """Accumulated results for one orbit (or the part of it that ran)."""

    ticks: int = 0
    duration_h: float = 0.0
    generation_wh: float = 0.0
    consumption_wh: float = 0.0
    battery_wh: float = 0.0
    """Σ battery current x bus voltage x dt: charging-positive (#76)."""
    battery_change_wh: float = 0.0
    """(SOC at the end - SOC at the start) x battery capacity."""
    heater_wh: float = 0.0
    beacon_wh: float = 0.0
    pass_wh: float = 0.0
    transmitter_idle_wh: float = 0.0
    sunlit_ticks: int = 0
    eclipse_ticks: int = 0
    heater_eclipse_ticks: int = 0
    heater_ticks: int = 0
    eclipse_heater_cycles: list[int] = field(default_factory=list)
    soc_start: float = 0.0
    soc_end: float = 0.0
    soc_min: float = 1.0
    soc_max: float = 0.0
    battery_c: tuple[float, float] = (math.inf, -math.inf)
    electronics_c: tuple[float, float] = (math.inf, -math.inf)
    produced_bytes: int = 0
    buffered_end_bytes: int = 0
    flags: dict[str, int] = field(default_factory=dict)
    """Flag name -> simulated time (µs from the start of the run) it first set."""

    @property
    def net_wh(self) -> float:
        return self.generation_wh - self.consumption_wh

    @property
    def margin(self) -> float:
        return self.net_wh / self.consumption_wh

    @property
    def eclipse_heater_duty(self) -> float:
        return self.heater_eclipse_ticks / self.eclipse_ticks if self.eclipse_ticks else 0.0


@dataclass
class Result:
    profile: Profile
    config: SpacecraftConfig
    orbits: list[OrbitStats]
    runtime_s: float
    flags: dict[str, int]
    heater_after_first_on: tuple[int, int] = (0, 0)
    """(ticks on, ticks) from the heater's first switch-on to the end."""


def pass_capacity_bytes(config: SpacecraftConfig, tick_us: int) -> int:
    """Chunk bytes one DOWNLINK pass can carry, net of ACK/NACK and telemetry."""
    ticks_per_beacon = BEACON_PERIOD_US // tick_us
    frames_per_beacon = data_frames_in_tick(config, True) + (
        ticks_per_beacon - 1
    ) * data_frames_in_tick(config, False)
    beacons = PASS_US // BEACON_PERIOD_US
    return beacons * frames_per_beacon * config.payload.chunk_size_bytes


def run_profile(
    profile: Profile,
    orbits: int,
    *,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
    environment: NominalEnvironment | None = None,
    extra_load_w: float = 0.0,
    stop_on: str | None = None,
) -> Result:
    """Run ``profile`` for ``orbits`` orbits (or until flag ``stop_on`` sets)."""
    started = time.perf_counter()
    env_model = NominalEnvironment() if environment is None else environment
    stack = real_stack(SEED, config, initial)
    clock = SimClock()
    tick_us = clock.tick_us
    dt_h = tick_us / US_PER_S / S_PER_H
    period_us = env_model.orbit_period_us
    driver = ProfileDriver(profile, config, period_us, extra_load_w)
    capacity_wh = config.power.battery_capacity_wh

    previous = stack.snapshot()
    flags: dict[str, int] = {}
    results: list[OrbitStats] = []
    heater_first_on: int | None = None
    heater_on_after = 0
    heater_ticks_after = 0
    stop = False
    for _ in range(orbits):
        stats = OrbitStats()
        stats.soc_start = previous.get("power", PowerSnapshot).truth.soc
        produced_start = previous.get("payload", PayloadSnapshot).truth.total_produced_bytes
        in_eclipse = False
        heater_was_on = previous.get("thermal", ThermalSnapshot).truth.heater_on
        for _ in range(period_us // tick_us):
            now_us = clock.now_us
            command = driver.command(now_us, previous)
            in_pass, traffic_w = command.in_pass, command.traffic_w
            env = env_model.state_at(now_us)
            stack.step(tick_us, env, command.controls)
            clock.advance_one_tick()
            state = stack.snapshot()
            _accumulate(stats, previous, state, env.sunlit, dt_h)
            if in_pass:
                stats.pass_wh += traffic_w * dt_h
            else:
                stats.beacon_wh += traffic_w * dt_h
            stats.transmitter_idle_wh += (
                previous.get("comms", CommsSnapshot).truth.transmit_power_w * dt_h
            )

            thermal = state.get("thermal", ThermalSnapshot).truth
            if not env.sunlit:
                if not in_eclipse:
                    stats.eclipse_heater_cycles.append(0)
                    in_eclipse = True
                if thermal.heater_on and not heater_was_on:
                    stats.eclipse_heater_cycles[-1] += 1
            else:
                in_eclipse = False
            heater_was_on = thermal.heater_on
            if heater_first_on is None and thermal.heater_on:
                heater_first_on = clock.now_us
            if heater_first_on is not None:
                heater_ticks_after += 1
                heater_on_after += thermal.heater_on

            for name in _raised(state):
                if name not in flags:
                    flags[name] = clock.now_us
                    stats.flags[name] = clock.now_us
            previous = state
            if stop_on is not None and stop_on in flags:
                stop = True
                break
        stats.soc_end = previous.get("power", PowerSnapshot).truth.soc
        stats.battery_change_wh = (stats.soc_end - stats.soc_start) * capacity_wh
        payload = previous.get("payload", PayloadSnapshot).truth
        stats.produced_bytes = payload.total_produced_bytes - produced_start
        stats.buffered_end_bytes = payload.buffered_bytes
        results.append(stats)
        if stop:
            break
    return Result(
        profile=profile,
        config=config,
        orbits=results,
        runtime_s=time.perf_counter() - started,
        flags=flags,
        heater_after_first_on=(heater_on_after, heater_ticks_after),
    )


def _accumulate(
    stats: OrbitStats,
    previous: SpacecraftState,
    state: SpacecraftState,
    sunlit: bool,
    dt_h: float,
) -> None:
    power = state.get("power", PowerSnapshot).truth
    thermal = state.get("thermal", ThermalSnapshot).truth
    stats.ticks += 1
    stats.duration_h += dt_h
    stats.generation_wh += power.generation_w * dt_h
    stats.consumption_wh += power.total_load_w * dt_h
    stats.battery_wh += power.battery_current_a * power.bus_v * dt_h
    # Power drew the heater thermal published in the previous tick (#85).
    heater_w = previous.get("thermal", ThermalSnapshot).truth.heater_power_w
    stats.heater_wh += heater_w * dt_h
    stats.heater_ticks += heater_w > 0
    if sunlit:
        stats.sunlit_ticks += 1
    else:
        stats.eclipse_ticks += 1
        stats.heater_eclipse_ticks += heater_w > 0
    stats.soc_min = min(stats.soc_min, power.soc)
    stats.soc_max = max(stats.soc_max, power.soc)
    low, high = stats.battery_c
    stats.battery_c = (min(low, thermal.battery_c), max(high, thermal.battery_c))
    low, high = stats.electronics_c
    stats.electronics_c = (min(low, thermal.electronics_c), max(high, thermal.electronics_c))


def _raised(state: SpacecraftState) -> list[str]:
    power = state.get("power", PowerSnapshot).readings
    thermal = state.get("thermal", ThermalSnapshot).readings
    return [
        name
        for name, on in (
            ("low_battery", power.low_battery),
            ("critical_battery", power.critical_battery),
            ("under_temp", thermal.under_temp),
            ("over_temp", thermal.over_temp),
        )
        if on
    ]


def describe(result: Result) -> str:
    """One line per orbit, for the budget document and the test log."""
    lines = [f"{result.profile.name} ({result.runtime_s:.1f} s wall)"]
    for i, o in enumerate(result.orbits, 1):
        lines.append(
            f"  orbit {i}: {o.ticks} ticks, generation {o.generation_wh:.2f} Wh,"
            f" consumption {o.consumption_wh:.2f} Wh, net {o.net_wh:+.2f} Wh,"
            f" margin {o.margin:+.1%}, SOC {o.soc_start:.3f} -> {o.soc_end:.3f}"
            f" (min {o.soc_min:.3f}), heater {o.heater_wh:.2f} Wh"
            f" ({o.eclipse_heater_duty:.0%} of eclipse, cycles {o.eclipse_heater_cycles}),"
            f" transmit idle {o.transmitter_idle_wh:.2f} / beacon {o.beacon_wh:.2f}"
            f" / pass {o.pass_wh:.2f} Wh,"
            f" battery {o.battery_c[0]:.1f}..{o.battery_c[1]:.1f} °C,"
            f" electronics {o.electronics_c[0]:.1f}..{o.electronics_c[1]:.1f} °C,"
            f" produced {o.produced_bytes} B, buffered at end {o.buffered_end_bytes} B,"
            f" flags {o.flags or 'none'}"
        )
    return "\n".join(lines)


def _fixture(build: Callable[[], Result]) -> Result:
    result = build()
    print("\n" + describe(result))
    return result


# --- Profiles (each runs once per module) ----------------------------------------------


@pytest.fixture(scope="module")
def reference() -> Result:
    """The reference profile, three orbits: one per orbit for the energy budget, three
    for the data budget."""
    return _fixture(lambda: run_profile(REFERENCE, 3))


@pytest.fixture(scope="module")
def nominal() -> Result:
    return _fixture(lambda: run_profile(NOMINAL, 1))


@pytest.fixture(scope="module")
def science() -> Result:
    return _fixture(lambda: run_profile(SCIENCE, 1))


# --- Energy margins --------------------------------------------------------------------


def test_reference_energy_margin_is_inside_the_band(reference: Result) -> None:
    low, high = MARGIN_BAND
    for i, orbit in enumerate(reference.orbits, 1):
        assert low <= orbit.margin <= high, (
            f"orbit {i}: energy margin {orbit.margin:+.2%} outside {low:+.0%}..{high:+.0%}"
            f" (generation {orbit.generation_wh:.3f} Wh, consumption"
            f" {orbit.consumption_wh:.3f} Wh)"
        )


def test_reference_minimum_soc_is_inside_the_band(reference: Result) -> None:
    # From the default starting state; the first orbit is the budgeted one. Being
    # energy-positive, later orbits start higher until the battery is full.
    threshold = NOMINAL_CONFIG.power.low_battery_soc
    low, high = MIN_SOC_ABOVE_LOW_BAND
    above = reference.orbits[0].soc_min - threshold
    assert low <= above <= high, (
        f"minimum SOC {reference.orbits[0].soc_min:.4f} is {above:+.4f} above low_battery"
        f" ({threshold}); required {low:+.2f}..{high:+.2f}"
    )


@pytest.mark.parametrize("name", ["NOMINAL", "SCIENCE"])
def test_nominal_and_science_are_energy_positive(
    name: str, nominal: Result, science: Result
) -> None:
    result = nominal if name == "NOMINAL" else science
    orbit = result.orbits[0]
    assert orbit.net_wh > 0, f"{name}: net energy {orbit.net_wh:+.3f} Wh per orbit, required > 0"
    assert orbit.soc_end > orbit.soc_start
    assert not result.flags, f"{name}: flags raised {result.flags}"


def test_pointing_loss_is_included(reference: Result) -> None:
    # Generation uses the stabilized pointing error through cos(pointing error) (#35,
    # #41): below the ideal, but only slightly once stabilized (orbits 2 and 3).
    array_w = NOMINAL_CONFIG.power.solar_array_w
    for orbit in reference.orbits[1:]:
        ideal_wh = array_w * orbit.sunlit_ticks * orbit.duration_h / orbit.ticks
        factor = orbit.generation_wh / ideal_wh
        assert 0.99 <= factor < 1.0, f"mean pointing factor {factor:.5f}, expected 0.99..1"


def test_heater_cycles_in_every_eclipse_and_is_budgeted(reference: Result) -> None:
    for i, orbit in enumerate(reference.orbits, 1):
        assert orbit.eclipse_heater_cycles, f"orbit {i}: no eclipse"
        assert all(c >= 1 for c in orbit.eclipse_heater_cycles), (
            f"orbit {i}: heater cycles per eclipse {orbit.eclipse_heater_cycles}, required >= 1"
        )
        assert 0 < orbit.eclipse_heater_duty < 1
        assert orbit.heater_wh > 0  # included in consumption, which power totals


# --- Temperatures and noise distance ---------------------------------------------------


@pytest.mark.parametrize("name", ["reference", "NOMINAL", "SCIENCE"])
def test_temperatures_stay_inside_their_limits_with_margin(
    name: str, reference: Result, nominal: Result, science: Result
) -> None:
    result = {"reference": reference, "NOMINAL": nominal, "SCIENCE": science}[name]
    thermal = NOMINAL_CONFIG.thermal
    for i, orbit in enumerate(result.orbits, 1):
        for node, (low, high) in (
            ("battery", orbit.battery_c),
            ("electronics", orbit.electronics_c),
        ):
            under = getattr(thermal, f"{node}_under_temp_c")
            over = getattr(thermal, f"{node}_over_temp_c")
            assert low - under >= TEMPERATURE_MARGIN_C, (
                f"{name} orbit {i}: {node} minimum {low:.2f} °C is {low - under:.2f} °C"
                f" above under-temp ({under} °C); required {TEMPERATURE_MARGIN_C} °C"
            )
            assert over - high >= TEMPERATURE_MARGIN_C, (
                f"{name} orbit {i}: {node} maximum {high:.2f} °C is {over - high:.2f} °C"
                f" below over-temp ({over} °C); required {TEMPERATURE_MARGIN_C} °C"
            )


def test_reported_values_stay_five_sigma_from_every_flag_threshold(reference: Result) -> None:
    # Flags come from reported values (ADR-0004 §6). With the true values 5 sigma of
    # nominal noise away from every set threshold, noise cannot trip a flag.
    power = NOMINAL_CONFIG.power
    thermal = NOMINAL_CONFIG.thermal
    soc_sigma = power.voltage_noise_v / (power.battery_full_v - power.battery_empty_v)
    t_sigma = thermal.temperature_noise_c
    for i, orbit in enumerate(reference.orbits, 1):
        for threshold in (power.low_battery_soc, power.critical_battery_soc):
            distance = (orbit.soc_min - threshold) / soc_sigma
            assert distance >= NOISE_SIGMAS, (
                f"orbit {i}: minimum SOC {orbit.soc_min:.4f} is {distance:.1f} sigma above"
                f" the {threshold} threshold; required {NOISE_SIGMAS} sigma"
            )
        for node, (low, high) in (
            ("battery", orbit.battery_c),
            ("electronics", orbit.electronics_c),
        ):
            for distance in (
                (low - getattr(thermal, f"{node}_under_temp_c")) / t_sigma,
                (getattr(thermal, f"{node}_over_temp_c") - high) / t_sigma,
            ):
                assert distance >= NOISE_SIGMAS, (
                    f"orbit {i}: {node} is {distance:.1f} sigma from a limit;"
                    f" required {NOISE_SIGMAS} sigma"
                )
    assert not reference.flags, f"flags raised in the reference profile: {reference.flags}"


# --- Energy accounting -----------------------------------------------------------------


@pytest.mark.parametrize("name", ["reference", "NOMINAL", "SCIENCE"])
def test_energy_accounting_balances(
    name: str, reference: Result, nominal: Result, science: Result
) -> None:
    # Sign convention (#76): generation and every load are non-negative, battery
    # current is positive while charging. So battery energy change = generation -
    # consumption - losses, where the model's only loss is energy discarded at full or
    # empty, which these profiles never reach.
    result = {"reference": reference, "NOMINAL": nominal, "SCIENCE": science}[name]
    for i, orbit in enumerate(result.orbits, 1):
        assert orbit.soc_min > 0.0 and orbit.soc_max < 1.0  # no energy discarded
        change_wh = orbit.battery_change_wh
        assert abs(change_wh - orbit.net_wh) <= ACCOUNTING_TOLERANCE_WH, (
            f"{name} orbit {i}: battery energy change {change_wh:+.9f} Wh, generation -"
            f" consumption {orbit.net_wh:+.9f} Wh"
        )
        # Charging-positive current: current x voltage integrates to the same change.
        assert abs(orbit.battery_wh - orbit.net_wh) <= ACCOUNTING_TOLERANCE_WH


def test_known_extra_load_lowers_soc_by_the_expected_amount(reference: Result) -> None:
    plus_1w = Profile(f"{REFERENCE.name}, +{SENSITIVITY_LOAD_W} W extra load")
    loaded = _fixture(lambda: run_profile(plus_1w, 1, extra_load_w=SENSITIVITY_LOAD_W))
    base, plus = reference.orbits[0], loaded.orbits[0]
    capacity = NOMINAL_CONFIG.power.battery_capacity_wh
    analytic_wh = SENSITIVITY_LOAD_W * base.duration_h
    drop_wh = (base.soc_end - plus.soc_end) * capacity
    # Exact once the survival heater's response (the only load that reacts) is counted.
    heater_change_wh = plus.heater_wh - base.heater_wh
    assert drop_wh == pytest.approx(analytic_wh + heater_change_wh, abs=ACCOUNTING_TOLERANCE_WH)
    # And the raw effect is the analytic one within the heater feedback.
    assert drop_wh == pytest.approx(analytic_wh, rel=SENSITIVITY_RAW_TOLERANCE), (
        f"+{SENSITIVITY_LOAD_W} W lowered the end-of-orbit energy by {drop_wh:.4f} Wh,"
        f" analytic {analytic_wh:.4f} Wh"
    )


# --- Data budget -----------------------------------------------------------------------


def test_data_per_orbit_fits_one_pass_with_margin(reference: Result) -> None:
    capacity = pass_capacity_bytes(NOMINAL_CONFIG, SimClock().tick_us)
    for i, orbit in enumerate(reference.orbits, 1):
        margin = 1 - orbit.produced_bytes / capacity
        assert margin >= DATA_MARGIN, (
            f"orbit {i}: produced {orbit.produced_bytes} B against a pass capacity of"
            f" {capacity} B, data margin {margin:.1%}; required {DATA_MARGIN:.0%}"
        )


def test_buffer_does_not_grow_from_pass_to_pass(reference: Result) -> None:
    # Each orbit ends with its pass, so the buffer at the end of each orbit is the
    # buffer at the end of each pass: only the partial chunk still accumulating.
    chunk = NOMINAL_CONFIG.payload.chunk_size_bytes
    ends = [orbit.buffered_end_bytes for orbit in reference.orbits]
    assert all(end < chunk for end in ends), f"buffered at the end of each pass: {ends} B"
    assert all(b <= a + chunk for a, b in itertools.pairwise(ends)), ends


# --- Profiles that must fail -----------------------------------------------------------


def test_continuous_downlink_runs_out_of_energy() -> None:
    result = _fixture(
        lambda: run_profile(CONTINUOUS_DOWNLINK, DOWNLINK_LOW_BATTERY_ORBITS, stop_on="low_battery")
    )
    first = result.orbits[0]
    assert first.net_wh < 0, f"continuous DOWNLINK net {first.net_wh:+.3f} Wh, required < 0"
    assert "low_battery" in result.flags, (
        f"low_battery not raised within {DOWNLINK_LOW_BATTERY_ORBITS} orbits"
        f" (SOC {result.orbits[-1].soc_end:.3f})"
    )


def test_science_under_the_stressed_configuration_runs_out_of_energy() -> None:
    result = _fixture(
        lambda: run_profile(
            STRESSED_SCIENCE,
            STRESSED_LOW_BATTERY_ORBITS,
            config=STRESSED_CONFIG,
            stop_on="low_battery",
        )
    )
    first = result.orbits[0]
    assert first.net_wh < 0, f"stressed SCIENCE net {first.net_wh:+.3f} Wh, required < 0"
    assert "low_battery" in result.flags, (
        f"low_battery not raised within {STRESSED_LOW_BATTERY_ORBITS} orbits"
        f" (SOC {result.orbits[-1].soc_end:.3f})"
    )


def test_tumbling_is_reported_for_information() -> None:
    # Not a pass/fail budget check: the payload stops acquiring while not STABILIZED,
    # and the outcome depends on the disturbance. Only the accounting is checked.
    result = _fixture(lambda: run_profile(TUMBLING, 1))
    orbit = result.orbits[0]
    change_wh = orbit.battery_change_wh
    assert abs(change_wh - orbit.net_wh) <= ACCOUNTING_TOLERANCE_WH


def test_cold_case_heater_saturates_and_a_flag_is_raised() -> None:
    # Expected behavior, not a bug: at -150 °C the heater runs nearly all the time,
    # energy goes negative, and under_temp sets (the flight computer then enters SAFE,
    # #48).
    cold = NominalEnvironment(ambient_temp_c=COLD_AMBIENT_C, eclipse_ambient_temp_c=COLD_AMBIENT_C)
    result = _fixture(lambda: run_profile(COLD_SCIENCE, 1, environment=cold))
    orbit = result.orbits[0]
    on, ticks = result.heater_after_first_on
    assert ticks and on / ticks >= COLD_HEATER_DUTY, (
        f"heater duty {on / max(ticks, 1):.1%} after first switch-on;"
        f" required {COLD_HEATER_DUTY:.0%}"
    )
    assert orbit.net_wh < 0, f"cold case net {orbit.net_wh:+.3f} Wh, required < 0"
    assert result.flags.get("under_temp", math.inf) <= COLD_FLAG_WITHIN_US, (
        f"under_temp first set at {result.flags.get('under_temp')} µs;"
        f" required within {COLD_FLAG_WITHIN_US} µs"
    )
