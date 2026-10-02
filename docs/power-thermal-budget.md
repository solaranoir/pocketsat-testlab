# PocketSat Test Lab — Power, thermal, and data budget

Status: Phase 1 (#72). Calibrates the subsystem defaults as a set and records the budget they produce. Guarded by `tests/sil/test_power_thermal_budget.py` (marked `slow`, run in CI on every PR). **Review point:** the budget's realism is reassessed at the start of Phase 6 (see [roadmap](roadmap.md#phase-6--deterministic-chaos-campaigns)).

Every subsystem ticket picked its own provisional defaults. Measured together they did not add up: with all five real subsystems, SCIENCE all orbit lost 0.72 Wh per orbit, and the data rate left about 9% of a pass's capacity spare. This page sets the defaults so the nominal case works, and keeps it from working too comfortably.

## Operating profiles

All profiles run the real `Power`, `Thermal`, `Attitude`, `Payload`, and `Comms` in one `SubsystemStack`, driven by `NominalEnvironment` (92-minute orbit, 35% eclipse, 20 °C sunlit and -20 °C eclipse ambient) at the default **100 ms tick, never coarsened**: `transmit_capacity_bytes` and the per-byte transmit draw are per tick, so the 9600 bit/s link and the transmit energy below hold only at 100 ms. Seed 1, default starting state (`DEFAULT_INITIAL_STATE`) unless stated.

There is no flight computer yet, so the tests script `SpacecraftControls` the way the mode table (#47) will:

| Profile | Payload | Radio | Attitude control | Transmit traffic |
|---|---|---|---|---|
| NOMINAL | off | `RX_TX` | on | beacon |
| SCIENCE | enabled | `RX_TX` | on | beacon |
| DOWNLINK | enabled | `RX_TX` | on | full capacity |
| **Reference** (#63, #70 use it) | SCIENCE for the whole orbit except one 10-minute DOWNLINK pass at the end of each orbit | | | |

- The pass sits at the **end of eclipse**, the worst place for the minimum SOC.
- The payload stays enabled during DOWNLINK. The mode table (#47, [spacecraft-modes.md](spacecraft-modes.md)) turns it off in DOWNLINK, so keeping it on here is deliberately pessimistic for both energy and data. With the payload off during the pass the reference margin would rise by about 0.25 Wh, to roughly +16%, still inside the band.

### Transmitter energy model ("Option C", decided 2026-10-01)

The radio stays `RX_TX` in every normal mode (ADR-0004 §10, #47), so beacons and ACKs can go out; this budget does not change that. What changed is the cost of the transmitter, through `CommsConfig` only, so energy follows airtime as on a real small-satellite UHF radio: a small idle draw while the transmitter is enabled but not keyed (`transmitter_on_power_w` 0.15 W) plus a per-byte draw (`transmit_power_per_byte_w` 0.02 W per byte per 100 ms tick), so full capacity (120 bytes per tick) costs 0.15 + 2.4 = **2.55 W keyed**.

On its own (old base load and payload draws), Option C takes the reference profile from -0.72 Wh to about +0.01 Wh per orbit (+0.1%), below the +5% floor. The rest of the gap was closed by the base load and payload draws (see the parameter table), not by enlarging the array or battery.

### Traffic stand-in (until #56 and #59)

Comms has no traffic input yet: `sent_bytes` is always 0 (#44), so comms reports only the idle draw. The budget tests cost the traffic a real flight computer would send with `transmit_draw_w` from `pocketsat.spacecraft.comms`, and add the per-byte part through `SpacecraftControls.extra_load_w` in the ticks it would be sent:

- **Beacon:** one telemetry frame per second in every mode, assumed 64 bytes (a 54-byte payload plus the 10-byte frame overhead of [protocol.md](protocol.md); #54's field list encodes to about 24 bytes, so this leaves room). 1.28 W for one tick in ten, 0.128 W on average.
- **DOWNLINK pass:** full transmit capacity, 120 bytes in every tick for 10 minutes, whatever frames fill it (pessimistic: the DATA packing below fills about 65% of it).

Beacon and pass energy are reported separately below. **Follow-up:** once #56 and #59 send real traffic, comms' own `transmit_power_w` carries it; drop the stand-in and re-check this budget.

### Data release stand-in (until #56)

In each pass tick the test releases (`release_through_chunk_id`) as many of the oldest chunks as whole DATA frames fit in the capacity left after telemetry and ACK/NACK, in the ADR-0004 §10 priority order, as #56 will. Frames that don't fit are not queued. Nothing is released outside the pass.

## Results

Reference profile, three orbits; the other profiles one orbit (or until `low_battery`). Margin is (generation - consumption) / consumption over the orbit. Temperatures are true values; limits are the under- and over-temperature set thresholds (battery -5 / 45 °C, electronics -25 / 60 °C).

| Profile | Generation Wh | Consumption Wh | Net Wh | Margin | SOC start → end (min) | Battery °C | Electronics °C | Heater (eclipse duty, Wh) | Flags |
|---|---|---|---|---|---|---|---|---|---|
| Reference, orbit 1 | 7.94 | 7.04 | +0.90 | **+12.8%** | 0.500 → 0.545 (**0.500**) | 1.0 .. 29.1 | 2.3 .. 31.3 | 28%, 0.46 | none |
| Reference, orbit 2 | 7.97 | 7.10 | +0.86 | +12.2% | 0.545 → 0.588 (0.545) | 1.0 .. 28.8 | 2.2 .. 30.6 | 29%, 0.47 | none |
| Reference, orbit 3 | 7.97 | 7.10 | +0.87 | +12.2% | 0.588 → 0.631 (0.588) | 1.0 .. 28.8 | 2.2 .. 30.6 | 29%, 0.47 | none |
| NOMINAL | 7.94 | 4.74 | +3.20 | +67.5% | 0.500 → 0.660 (0.500) | 1.0 .. 25.8 | -4.3 .. 27.2 | 49%, 0.79 | none |
| SCIENCE (no pass) | 7.94 | 6.75 | +1.19 | +17.7% | 0.500 → 0.560 (0.500) | 1.0 .. 29.1 | -0.3 .. 31.3 | 34%, 0.54 | none |
| Continuous DOWNLINK | 7.94 / 7.97 | 9.85 / 9.89 | -1.91 / -1.92 | -19.4% | 0.500 → 0.308 over 2 orbits | 1.0 .. 34.2 | 5.8 .. 37.7 | 10%, 0.16 | `low_battery` at 11 020 s (end of orbit 2) |
| SCIENCE, `STRESSED_CONFIG` | 5.95 / 5.98 | 7.61 / 7.02 | -1.66 / -1.05 | -21.8% | 0.500 → 0.307 over 1.9 orbits | 1.0 .. 30.6 | 1.5 .. 33.2 | 25%, 0.41 | `low_battery` at 10 642 s (1.9 orbits) |
| Tumbling (information) | 2.49 | 4.80 | -2.31 | -48.1% | 0.500 → 0.385 (0.385) | 1.0 .. 25.3 | -2.2 .. 26.6 | 48%, 0.77 | none |
| Cold case, -150 °C | 7.94 | 8.94 | -1.01 | -11.3% | 0.500 → 0.450 (0.450) | -123.5 .. 20 | -137.8 .. 20 | 100%, 4.53 | `under_temp` at 117 s |

Two-orbit rows show orbit 1 / orbit 2; orbit 2 stops when `low_battery` sets, and in the stressed case its consumption is lower because the payload stops acquiring once the flag is set.

**Reference profile consumption breakdown** (orbit 2, 7.10 Wh): base load 2.76, payload 2.30, attitude control 0.77, transmitter idle 0.23, beacon 0.17, pass 0.40, survival heater 0.47. Transmit energy is costed from the bytes sent, so beacon-only time (0.17 Wh) and the pass (0.40 Wh) appear separately.

**Pointing loss is included:** generation is `solar_array_w × cos(true pointing error)`, with the pointing error held near 2° once stabilized (#41, #35). Orbit 1 also carries the initial detumble (about 3 minutes at up to 45°).

### Requirements and how they are checked

| Requirement (#72) | Result | Test |
|---|---|---|
| Reference margin +5% .. +25% | +12.2% .. +12.8% every orbit | `test_reference_energy_margin_is_inside_the_band` |
| Reference minimum SOC 10 .. 30 points above `low_battery` (0.30) | 0.500, 20 points above | `test_reference_minimum_soc_is_inside_the_band` |
| NOMINAL and SCIENCE energy-positive | +3.20 and +1.19 Wh per orbit | `test_nominal_and_science_are_energy_positive` |
| Temperatures at least 5 °C inside limits | closest: battery minimum 1.0 °C, 6.0 °C above -5 °C | `test_temperatures_stay_inside_their_limits_with_margin` |
| Reported values 5σ from every flag threshold | SOC 60σ from 0.30 (σ = 0.0033); temperatures 30σ (σ = 0.2 °C); no flag set | `test_reported_values_stay_five_sigma_from_every_flag_threshold` |
| Heater in the budget, cycles every eclipse | 3 cycles per eclipse, 28–29% duty, 0.47 Wh | `test_heater_cycles_in_every_eclipse_and_is_budgeted` |
| Energy accounting identity | residue below 1e-6 Wh per orbit | `test_energy_accounting_balances` |
| +1 W sensitivity | see below | `test_known_extra_load_lowers_soc_by_the_expected_amount` |
| Data budget, margin ≥ 20% | 36.1% | `test_data_per_orbit_fits_one_pass_with_margin`, `test_buffer_does_not_grow_from_pass_to_pass` |
| Continuous DOWNLINK fails | -1.9 Wh per orbit, `low_battery` within 3 orbits (2 with seed 1) | `test_continuous_downlink_runs_out_of_energy` |
| Stressed SCIENCE fails | -1.7 Wh per orbit, `low_battery` within 2 orbits | `test_science_under_the_stressed_configuration_runs_out_of_energy` |
| Cold case chain | heater 100% duty, -1.0 Wh per orbit, `under_temp` within 5 minutes | `test_cold_case_heater_saturates_and_a_flag_is_raised` |

Failure messages name the margin that was violated and its actual value.

**About the minimum-SOC band.** `NominalEnvironment` starts each orbit in sunlight, and the reference profile is energy-positive, so over the budgeted first orbit the minimum SOC is the starting charge: the band therefore fixes the default `PowerInitial.soc` (0.5) together with the requirement that the orbit never dips below it. Over later orbits the SOC climbs about 0.043 per orbit until the battery is full, after which the eclipse depth of discharge is about 15%. A battery small enough to put the *steady-state* minimum in the band (about 6 Wh, about 50% depth of discharge every orbit) would be unrealistic for lithium-ion cycle life. Reviewer: confirm this reading.

**Energy accounting.** Sign convention (#76): generation and every load are non-negative, and battery current is positive while charging. Over each orbit, battery energy change ((SOC end - SOC start) × capacity) = generation - consumption - losses. The model has no conversion losses; its only loss is energy discarded when the SOC clamps at full or empty, which none of the checked profiles reaches. The test also integrates battery current × bus voltage and requires the same change. Tolerance: 1e-6 Wh per orbit (floating-point summation).

**Sensitivity check.** Adding 1 W through `extra_load_w` for the reference orbit lowers the end-of-orbit energy by 1.36 Wh against an analytic 1 W × 1.533 h = 1.53 Wh. The difference is exactly the survival heater's response: the extra watt is also dissipated as heat, so the heater runs 0.18 Wh less. The test requires the drop to equal the analytic value plus the heater change within 1e-6 Wh, and the raw drop to be within 20% of the analytic value; a sign error or a unit error (hours, kilo) is off by a factor of 2 or more.

### Data budget

| Quantity | Value |
|---|---|
| Science data per orbit | 220 800 B (40 B/s × 5520 s; 214 856 B in orbit 1, which starts with a detumble) |
| Pass capacity, raw | 720 000 B (120 B per 100 ms tick × 6000 ticks) |
| DATA frame | 78 B: 64 B chunk + 4 B chunk ID + 10 B frame overhead |
| Per second of pass | 1 tick with the 64 B telemetry frame and one 13 B ACK/NACK (43 B left, no DATA frame fits); 9 ticks with one DATA frame each |
| Pass capacity, net of ACK/NACK and telemetry | 9 chunks/s × 64 B × 600 s = **345 600 B** |
| Margin | 1 - 220 800 / 345 600 = **36.1%** (required 20%) |
| Buffer at the end of each pass | 8 B, every orbit (only the partial chunk still accumulating) |
| Pass time used | about 6 of the 10 minutes |

The ACK/NACK allowance (one per second) is pessimistic; with whole-frame packing it costs nothing extra, because the telemetry tick has no room for a DATA frame anyway. Packing efficiency is low (48% of raw capacity), because a 78-byte frame leaves 42 bytes of each tick unused; a larger chunk would pack better, but the data rate was the least constrained default.

### Profiles that must fail

These prove the model can run out of energy and that the low-battery path fires from physics, not only from faults.

- **Continuous DOWNLINK** for the whole orbit loses about 1.9 Wh per orbit; from the default start `low_battery` sets within **3 orbits** (seed 1: at the end of the second).
- **SCIENCE under `STRESSED_CONFIG`** loses about 1.7 Wh per orbit; `low_battery` sets within **2 orbits** (seed 1: 1.9 orbits in).
- **Tumbling, for information only:** with attitude control off from the start, the rate drifts up, the payload never acquires (it needs `STABILIZED`), and the arrays sweep through all angles. Orbit-average generation 2.49 Wh (31% of the stabilized value), margin -48%, minimum SOC 0.385 after one orbit. Not a pass/fail check: the outcome depends on the disturbance.

### Cold case: an expected outcome, not a bug

Under the contract suite's severe -150 °C environment (applied in sunlight and eclipse, with the nominal orbit), the unheated electronics cross -25 °C and `under_temp` sets about **2 minutes** in (the test allows 5). The payload, inhibited by `under_temp`, idles. The survival heater switches on when the battery falls below 1 °C and never switches off (100% duty after its first switch-on): at -150 °C its 3 W cannot hold the battery anywhere near the OFF setpoint, and both nodes fall far below their limits. Energy goes negative (-1.0 Wh per orbit), so at that rate `low_battery` follows after about 4 orbits and `critical_battery` later. In the full system the flight computer acts on the flag and enters SAFE (#48). This is the designed chain: hardwired heater at full duty, flag, SAFE.

## Parameters

Everything not listed is unchanged and was checked against the profiles above. Real-world ranges are for a 1U–3U CubeSat-class spacecraft.

| Parameter | Old → new default | Plausible range | Rationale |
|---|---|---|---|
| `CommsConfig.transmitter_on_power_w` | 1.0 → **0.15** W | 0.05–0.5 W (UHF transceiver enabled, not keyed) | Option C: idle draw only, since the radio stays `RX_TX` |
| `CommsConfig.transmit_power_per_byte_w` | 0.005 → **0.02** W/byte | full-capacity keyed draw 1.5–4 W for 0.5–1 W RF out | 2.55 W keyed at full capacity; per 100 ms tick |
| `CommsConfig.transmit_capacity_bytes` | 120 (unchanged) | 1200–19 200 bit/s UHF | 9600 bit/s at the 100 ms tick |
| `PowerConfig.base_load_w` | 2.0 → **1.8** W | 0.8–2.5 W (OBC, receiver, EPS quiescent, ADCS sensors) | closes the energy gap within a realistic avionics load |
| `PayloadConfig.acquiring_power_w` | 2.0 → **1.5** W | 0.5–3 W (small imager or instrument) | closes the energy gap |
| `PayloadConfig.idle_power_w` | 0.5 → **0.3** W | 0.1–0.5 W | consistent with the lower acquiring draw |
| `PayloadConfig.data_rate_bytes_per_s` | 100 → **40** B/s | instrument-dependent | sized to one pass's net capacity with a 36% margin |
| `PayloadConfig.buffer_capacity_bytes` | 65 536 → **524 288** B (512 KiB) | MB–GB of flash | must hold an orbit's data until the pass; holds 2.4 orbits, so one missed pass loses nothing |
| `PayloadConfig.chunk_size_bytes` | 64 (unchanged) | — | 78-byte DATA frame fits one tick with room for an ACK |
| `ThermalConfig.heater_on_setpoint_c` | 0.0 → **1.0** °C | 0–10 °C for lithium-ion survival heaters | the battery dips just below the ON setpoint, so ON must be more than 5 °C above `battery_under_temp_c` (-5 °C) for the 5 °C margin |
| `ThermalConfig.heater_off_setpoint_c` | 4.0 → **5.0** °C | ON + 2–5 °C | keeps the 4 °C hysteresis |
| `ThermalConfig.heater_power_w` | 3.0 (unchanged) | 1–5 W for a small pack | heater-on steady state stays well above OFF at -20 °C, so it cycles |
| `PowerConfig.battery_capacity_wh` | 20.0 (unchanged) | 10–40 Wh | 15% eclipse depth of discharge once full; not enlarged |
| `PowerConfig.solar_array_w` | 8.0 (unchanged) | 6–9 W body-mounted 3U, sun-pointed; 15–30 W deployable | not enlarged |
| `AttitudeConfig.control_power_w` | 0.5 (unchanged) | 0.2–1.5 W (magnetorquers, small wheel) | confirmed |
| `PowerInitial.soc` | 0.8 → **0.5** | 0.3–1.0 at deployment (launch storage charge is often 30–60%) | puts the reference minimum SOC 20 points above `low_battery`, mid-band |
| `ThermalInitial` battery / electronics | 20 / 20 °C (unchanged) | — | the nodes settle within about an hour, long before the first eclipse |
| `NominalEnvironment` `eclipse_ambient_temp_c` | -20 °C (unchanged; confirmed) | -40 .. 0 °C effective internal sink | heater cycles 3 times in every nominal eclipse at about 28% duty while every margin holds |
| Power flag thresholds | low 0.30 / 0.35, critical 0.15 / 0.20 (unchanged) | — | 5σ and band requirements met |
| Thermal limits | battery -5 / -2, 45 / 42 °C; electronics -25 / -22, 60 / 57 °C; survival -10 °C (unchanged) | lithium-ion: no charging below 0 °C, at most about 45 °C | every temperature at least 5 °C inside |

**Battery threshold order** (validated by `ThermalConfig`, #39): survival limit (-10 °C) < `battery_under_temp_c` (-5 °C) < heater ON (+1 °C) < heater OFF (+5 °C), with under-temp at least 5 °C below heater ON.

### Nominal and stressed sets

`NOMINAL_CONFIG` holds the defaults above. `STRESSED_CONFIG` (exported from `pocketsat.spacecraft`) differs in:

| Setting | Nominal | Stressed | Why |
|---|---|---|---|
| `solar_array_w` | 8.0 W | 6.0 W (-25%) | end-of-life degradation, hot arrays, off-pointing |
| `battery_capacity_wh` | 20.0 Wh | 14.0 Wh (-30%) | aged battery |
| `base_load_w` | 1.8 W | 2.16 W (+20%) | higher loads |
| payload `idle_power_w` / `acquiring_power_w` | 0.3 / 1.5 W | 0.36 / 1.8 W (+20%) | higher loads |

It is used by the must-fail profile and is available to later phases (Phase 6 campaigns vary battery condition).

## What's optimistic

- The ambient temperature steps at the eclipse boundaries (#73); real sink temperatures ramp, and the thermal time constant only partly smooths the step.
- A single scalar pointing error, with the arrays sun-pointed whenever stabilized (#41); no array geometry, no seasonal beta angle, a fixed 35% eclipse.
- No battery ageing, temperature-dependent capacity, or internal resistance in the nominal set; no conversion losses (MPPT, regulators, charge efficiency); energy above full is simply discarded.
- Solar output does not depend on array temperature or radiation dose.
- One ground pass every orbit, each a full 10 minutes. A single ground station sees a LEO satellite a few times a day, and pass lengths vary.
- Traffic is a stand-in (fixed beacon size and rate, full capacity in the pass); no retransmissions or acknowledgement-based release (#56 releases once sent).
- Sensor noise is Gaussian bounded at 6σ, with no bias or drift.

Pessimistic on purpose: the pass sits at the end of eclipse, the payload stays on during DOWNLINK, and the pass is costed at full capacity.

## Rule for scenario authors

The nominal defaults are deliberately comfortable. **Any scenario about power or thermal stress uses `STRESSED_CONFIG`, a different starting state (`SpacecraftInitialState`), or `EnvironmentState` overrides; never retuned nominal defaults.** "Start low and see what happens" uses the starting charge (`PowerInitial.soc`); `battery_soc_override` pins the charge and is for exercising flags and edge cases (ADR-0005).

## Runtime

Each profile runs whole orbits at the 100 ms tick (55 200 ticks per orbit), about 2.3–2.9 s per orbit on an Apple Silicon laptop. Locally the whole budget module takes about 33 s: reference (3 orbits) 8.6 s, continuous DOWNLINK 5.7 s, stressed SCIENCE 5.4 s, SCIENCE 3.0 s, +1 W 2.9 s, cold 2.4 s, NOMINAL 2.3 s, tumbling 2.3 s. It runs in CI's parallel slow job (#78).
