# PocketSat Test Lab — Power, thermal, and data budget

Status: Phase 1 (#72; re-checked with real radio traffic and the real flight computer in #98, no default changed). Calibrates the subsystem defaults as a set and records the budget they produce. Guarded by `tests/sil/test_power_thermal_budget.py` (marked `slow`, run in CI on every PR). **Review point:** the budget's realism is reassessed at the start of Phase 6 (see [roadmap](roadmap.md#phase-6--deterministic-chaos-campaigns)).

Every subsystem ticket picked its own provisional defaults. Measured together they did not add up: with all five real subsystems, SCIENCE all orbit lost 0.72 Wh per orbit, and the data rate left about 9% of a pass's capacity spare. This page sets the defaults so the nominal case works, and keeps it from working too comfortably.

## Operating profiles

All profiles run on `SilTarget`: the real `Power`, `Thermal`, `Attitude`, `Payload`, and `Comms` on one `SnapshotBoard` and the real flight computer, driven only through the `TestTarget` interface by the ground helper `ProfileGround` (`tests/sil/_reference_profile.py`), under `NominalEnvironment` (92-minute orbit, 35% eclipse, 20 °C sunlit and -20 °C eclipse ambient) at the default **100 ms tick, never coarsened**. Since #122 the link is a fixed 1200 bytes/s (9600 bit/s) at any tick length and a byte costs the same energy at any tick length (`docs/spacecraft.md`, "Communications"); at 100 ms comms' capacity is exactly 120 bytes every tick and the per-byte draw is unscaled, so every figure on this page is unchanged by #122. The profiles stay at 100 ms because the other models' dynamics, telemetry cadence and frame packing were budgeted there. Seed 1, default starting state (`DEFAULT_INITIAL_STATE`) unless stated.

The flight computer sets the controls through its mode table (#47, [spacecraft-modes.md](spacecraft-modes.md)); the ground only sends commands:

| Profile | Ground commands | Payload | Attitude control | Transmit traffic (real frames) |
|---|---|---|---|---|
| NOMINAL | none (BOOT, then NOMINAL) | off | on | telemetry, 1 Hz |
| SCIENCE | SET_MODE SCIENCE at 6 s | acquiring | on | telemetry, 1 Hz |
| **Reference** (#63, #70 use it) | SCIENCE, and BEGIN_DOWNLINK 10 minutes before the end of each orbit | acquiring, off in DOWNLINK | on | telemetry; in the pass, every stored chunk as DATA frames, then back to SCIENCE (`DOWNLINK_COMPLETE`) |
| Continuous full-capacity transmit (must fail) | as SCIENCE | acquiring | on | every tick filled to the 120-byte capacity (see below) |
| SCIENCE, `STRESSED_CONFIG` (must fail) | as SCIENCE | acquiring until `low_battery` | on | telemetry, 1 Hz |
| Tumbling (information) | as reference | inhibited (never `STABILIZED`) | **off** (see below) | as reference |
| Cold case, -150 °C | as SCIENCE | off once SAFE | on | telemetry, 1 Hz, then 2 Hz in SAFE |

- The pass sits at the **end of eclipse**, the worst place for the minimum SOC. The flight computer's downlink session sends every stored chunk in about 5.4 minutes, then returns to SCIENCE for the rest of the window.
- BOOT (5 s, attitude control off, no telemetry) and one second of NOMINAL precede SCIENCE in the first orbit.

**Two profiles ask for what the flight software never does.** They use `ProfileFlightComputer`, the real flight computer with one documented change to its output, so their traffic is still real frames handed to comms:

- **Continuous full-capacity transmit** replaces #72's "continuous DOWNLINK". The real DOWNLINK mode turns the payload off and ends when the buffer is empty (`DOWNLINK_COMPLETE`), so a real continuous DOWNLINK is cheap and cannot fail. What the must-fail profile proves is that the transmitter at full duty with the payload acquiring empties the battery: the flight computer runs SCIENCE and, after its real frames, fills what is left of comms' capacity with filler frames in every tick. Physically this is #72's profile (payload acquiring, 120 bytes in every tick), now costed by comms.
- **Tumbling** holds attitude control off in every controls record the flight computer produces (BOOT already has it off). No command does this; the profile is for information only.

### Transmitter energy model ("Option C", decided 2026-10-01)

The radio stays `RX_TX` in every normal mode (ADR-0004 §10, #47), so beacons and ACKs can go out; this budget does not change that. What changed is the cost of the transmitter, through `CommsConfig` only, so energy follows airtime as on a real small-satellite UHF radio: a small idle draw while the transmitter is enabled but not keyed (`transmitter_on_power_w` 0.15 W) plus a per-byte draw (`transmit_power_per_byte_w` 0.02 W per byte sent within a 100 ms tick, 2 mJ per byte, scaled to the tick length since #122), so full rate (1200 bytes/s, 120 bytes per 100 ms tick) costs 0.15 + 2.4 = **2.55 W keyed** at any tick length.

On its own (old base load and payload draws), Option C took the reference profile from -0.72 Wh to about +0.01 Wh per orbit (+0.1%), below the +5% floor. The rest of the gap was closed by the base load and payload draws (see the parameter table), not by enlarging the array or battery.

### Measured traffic (#98)

Every byte is a real frame the flight computer sent, reported to comms by `SilTarget` (ADR-0007): comms charges it in the next tick's `transmit_power_w`, and power integrates it two ticks after the frame. The budget tests check that power's per-byte transmit energy over a run equals `transmit_power_per_byte_w` × the bytes sent (`test_reference_traffic_is_real_frames_and_comms_charges_every_byte`). Reference profile, orbit 2:

| Traffic | Frames | Bytes | Per-byte energy |
|---|---|---|---|
| Telemetry, 36-byte frames, 1 per second in SCIENCE and DOWNLINK (#55) | 5 521 | 198 756 | about 0.11 Wh |
| DATA, 78-byte frames (64-byte chunk), about 10 per second in the pass (#56) | 3 264 | 254 592 | about 0.14 Wh |
| ACK, 14-byte frames (BEGIN_DOWNLINK; orbit 1 also SET_MODE SCIENCE) | 1 | 14 | negligible |
| **Total** | | 453 362 | **0.25 Wh** (reported as telemetry 0.098 outside the pass window, pass 0.153 inside it) |

#72's stand-in assumed a 64-byte beacon once a second (0.17 Wh per orbit) and full capacity for all 10 minutes of the pass (0.40 Wh). The real traffic costs 0.25 Wh in all, and the DOWNLINK session uses 5.4 of the 10 minutes with the payload off (#47), where the stand-in kept it acquiring for the whole pass.

The 200 ms delay between a frame and its energy in power's load (ADR-0007) shifts energy across the orbit boundary by at most two ticks; it does not move any margin.

## Results

Reference profile, three orbits; the other profiles one orbit (or until `low_battery`). Margin is (generation - consumption) / consumption over the orbit. Temperatures are true values; limits are the under- and over-temperature set thresholds (battery -5 / 45 °C, electronics -25 / 60 °C).

| Profile | Generation Wh | Consumption Wh | Net Wh | Margin | SOC start → end (min) | Battery °C | Electronics °C | Heater (eclipse duty, Wh) | Flags |
|---|---|---|---|---|---|---|---|---|---|
| Reference, orbit 1 | 7.93 | 6.67 | +1.26 | **+18.8%** | 0.500 → 0.563 (**0.500**) | 1.0 .. 29.0 | -0.4 .. 31.2 | 34%, 0.55 | none |
| Reference, orbit 2 | 7.97 | 6.75 | +1.22 | +18.1% | 0.563 → 0.624 (0.563) | 1.0 .. 28.6 | -0.5 .. 30.4 | 35%, 0.58 | none |
| Reference, orbit 3 | 7.97 | 6.75 | +1.22 | +18.1% | 0.624 → 0.685 (0.624) | 1.0 .. 28.6 | -0.5 .. 30.4 | 35%, 0.58 | none |
| NOMINAL | 7.93 | 4.66 | +3.27 | +70.1% | 0.500 → 0.663 (0.500) | 1.0 .. 25.6 | -4.4 .. 27.0 | 49%, 0.79 | none |
| SCIENCE (no pass) | 7.93 | 6.67 | +1.26 | +18.9% | 0.500 → 0.563 (0.500) | 1.0 .. 29.0 | -0.4 .. 31.2 | 34%, 0.55 | none |
| Continuous full-capacity transmit | 7.93 / 7.97 | 9.85 / 9.89 | -1.92 / -1.92 | -19.5% | 0.500 → 0.308 over 2 orbits | 1.0 .. 34.2 | 5.8 .. 37.7 | 10%, 0.16 | `low_battery` at 11 020 s (end of orbit 2) |
| SCIENCE, `STRESSED_CONFIG` | 5.95 / 5.98 | 7.53 / 7.04 | -1.59 / -1.06 | -21.1% | 0.500 → 0.311 over 1.9 orbits | 1.0 .. 30.4 | 1.3 .. 33.0 | 26%, 0.42 | `low_battery` at 10 684 s (1.9 orbits) |
| Tumbling (information) | 2.49 | 4.38 | -1.89 | -43.2% | 0.500 → 0.405 (0.405) | 1.0 .. 25.2 | -4.9 .. 26.5 | 51%, 0.82 | none |
| Cold case, -150 °C | 7.93 | 8.51 | -0.59 | -6.9% | 0.500 → 0.471 (0.471) | -124.1 .. 20 | -138.7 .. 20 | 100%, 4.53 | `under_temp` at 117 s, then SAFE |

Two-orbit rows show orbit 1 / orbit 2; orbit 2 stops when `low_battery` sets, and in the stressed case its consumption is lower because the payload stops acquiring once the flag is set.

**Reference profile consumption breakdown** (orbit 2, 6.75 Wh): base load 2.76, payload about 2.16, attitude control 0.77, transmitter idle 0.23, telemetry 0.10, pass 0.15, survival heater 0.58.

**Before and after #98** (stand-in traffic and scripted controls → real traffic and the real flight computer). Every margin holds; **no default was retuned.**

| Profile | Before (#72) | After (#98) | Why it moved |
|---|---|---|---|
| Reference margin, orbits 1 / 2 / 3 | +12.8% / +12.2% / +12.2% | +18.8% / +18.1% / +18.1% | payload off during the DOWNLINK session (#47), about 0.2 Wh; real traffic 0.25 Wh instead of 0.57 Wh |
| Reference minimum SOC | 0.500 (20 points above `low_battery`) | 0.500 (20 points) | the starting charge, unchanged |
| Reference heater | 28–29% of eclipse, 0.46–0.47 Wh | 34–35%, 0.55–0.58 Wh | less waste heat at the end of eclipse with the payload off and the transmitter mostly idle |
| Reference electronics minimum | 2.2 °C | -0.5 °C | same reason; 24.5 °C above its -25 °C limit |
| NOMINAL net | +3.20 Wh (+67.5%) | +3.27 Wh (+70.1%) | 36-byte telemetry instead of the 64-byte beacon |
| SCIENCE net | +1.19 Wh (+17.7%) | +1.26 Wh (+18.9%) | same |
| Must fail: full-capacity transmit | -1.91 Wh, `low_battery` at 11 020 s | -1.92 Wh, `low_battery` at 11 020 s | the same physics, now through comms |
| Must fail: stressed SCIENCE | -1.66 Wh, `low_battery` at 10 642 s | -1.59 Wh, `low_battery` at 10 684 s | real telemetry is smaller |
| Tumbling (information) | -2.31 Wh, minimum SOC 0.385 | -1.89 Wh, minimum SOC 0.405 | payload off in DOWNLINK; smaller traffic |
| Cold case | -1.01 Wh, `under_temp` at 117 s | -0.59 Wh, `under_temp` at 117 s, then SAFE | the real flight computer enters SAFE (payload off) 1 s after the flag |
| Data margin | 36.1% | 39.9% | about 13 KB less data per orbit: the payload is off during the session |

The reference margin is now in the upper half of the +5% .. +25% band. The band is still met, so per #72's rules nothing was retuned; tightening it again (for example a higher base load) is a maintainer decision, not this change's.

### Requirements and how they are checked

| Requirement (#72) | Result | Test |
|---|---|---|
| Reference margin +5% .. +25% | +18.1% .. +18.8% every orbit | `test_reference_energy_margin_is_inside_the_band` |
| Reference minimum SOC 10 .. 30 points above `low_battery` (0.30) | 0.500, 20 points above | `test_reference_minimum_soc_is_inside_the_band` |
| NOMINAL and SCIENCE energy-positive | +3.27 and +1.26 Wh per orbit | `test_nominal_and_science_are_energy_positive` |
| Temperatures at least 5 °C inside limits | closest: battery minimum 1.0 °C, 6.0 °C above -5 °C | `test_temperatures_stay_inside_their_limits_with_margin` |
| Reported values 5σ from every flag threshold | SOC 60σ from 0.30 (σ = 0.0033); temperatures 30σ (σ = 0.2 °C); no flag set | `test_reported_values_stay_five_sigma_from_every_flag_threshold` |
| Heater in the budget, cycles every eclipse | 3 cycles per eclipse, 34–35% duty, 0.58 Wh | `test_heater_cycles_in_every_eclipse_and_is_budgeted` |
| Energy accounting identity | residue below 1e-6 Wh per orbit | `test_energy_accounting_balances` |
| +1 W sensitivity | see below | `test_known_extra_load_lowers_soc_by_the_expected_amount` |
| Data budget, margin ≥ 20% | 39.9% | `test_data_per_orbit_fits_one_pass_with_margin`, `test_buffer_does_not_grow_from_pass_to_pass`, `test_every_pass_completes_inside_the_window` |
| Real traffic, every byte charged (#98) | telemetry, ACK, and DATA frames only; per-byte energy = 0.02 W × bytes × tick | `test_reference_traffic_is_real_frames_and_comms_charges_every_byte` |
| Continuous full-capacity transmit fails | -1.9 Wh per orbit, `low_battery` within 3 orbits (2 with seed 1) | `test_continuous_full_capacity_transmit_runs_out_of_energy` |
| Stressed SCIENCE fails | -1.6 Wh per orbit, `low_battery` within 2 orbits | `test_science_under_the_stressed_configuration_runs_out_of_energy` |
| Cold case chain | heater 100% duty, -0.6 Wh per orbit, `under_temp` within 5 minutes | `test_cold_case_heater_saturates_and_a_flag_is_raised` |

Failure messages name the margin that was violated and its actual value.

**About the minimum-SOC band.** `NominalEnvironment` starts each orbit in sunlight, and the reference profile is energy-positive, so over the budgeted first orbit the minimum SOC is the starting charge: the band therefore fixes the default `PowerInitial.soc` (0.5) together with the requirement that the orbit never dips below it. Over later orbits the SOC climbs about 0.06 per orbit until the battery is full, after which the eclipse depth of discharge is about 15%. A battery small enough to put the *steady-state* minimum in the band (about 6 Wh, about 50% depth of discharge every orbit) would be unrealistic for lithium-ion cycle life. Reviewer: confirm this reading.

**Energy accounting.** Sign convention (#76): generation and every load are non-negative, and battery current is positive while charging. Over each orbit, battery energy change ((SOC end - SOC start) × capacity) = generation - consumption - losses. The model has no conversion losses; its only loss is energy discarded when the SOC clamps at full or empty, which none of the checked profiles reaches. The test also integrates battery current × bus voltage and requires the same change. Tolerance: 1e-6 Wh per orbit (floating-point summation).

**Sensitivity check.** A 1 W `battery_drain` fault (the only `extra_load_w` left in these tests) for the reference orbit lowers the end-of-orbit energy by about 1.32 Wh against an analytic 1 W × 1.533 h = 1.53 Wh. The difference is exactly the survival heater's response: the extra watt is also dissipated as heat, so the heater runs about 0.21 Wh less. The test requires the drop to equal the analytic value plus the heater change within 1e-6 Wh, and the raw drop to be within 20% of the analytic value; a sign error or a unit error (hours, kilo) is off by a factor of 2 or more.

### Data budget

| Quantity | Value |
|---|---|
| Science data per orbit | 207 800 B measured (orbits 2 and 3; 202 656 B in orbit 1, which starts with BOOT and a detumble). SCIENCE produces 40 B/s; the payload is off during the DOWNLINK session |
| Pass capacity, raw | 720 000 B (120 B per 100 ms tick × 6000 ticks) |
| DATA frame | 78 B: 64 B chunk + 4 B chunk ID + 10 B frame overhead |
| Per second of pass (budgeted) | 1 tick with the 36 B telemetry frame and a 14 B ACK/NACK allowance (50 B, no DATA frame fits in the 70 B left); 9 ticks with one DATA frame each |
| Pass capacity, net of ACK/NACK and telemetry | 9 chunks/s × 64 B × 600 s = **345 600 B** |
| Margin | 1 - 207 812 / 345 600 = **39.9%** (required 20%) |
| Measured | about 10 DATA frames per second: a pass has one ACK, so the telemetry tick (36 + 78 = 114 B) carries a DATA frame too; the session takes 298–326 s |
| Buffer when each session completes | under one chunk, every orbit (only the partial chunk still accumulating) |

The ACK/NACK allowance (one per second) is pessimistic: a real pass sends one, BEGIN_DOWNLINK's. Packing efficiency is low (about 65% of raw capacity), because a 78-byte frame leaves 42 bytes of each tick unused; a larger chunk would pack better, but the data rate was the least constrained default.

### Profiles that must fail

These prove the model can run out of energy and that the low-battery path fires from physics, not only from faults.

- **Continuous full-capacity transmit** (the transmitter keyed at its full 120 bytes per 100 ms tick for the whole orbit, payload acquiring) loses about 1.9 Wh per orbit; from the default start `low_battery` sets within **3 orbits** (seed 1: at the end of the second).
- **SCIENCE under `STRESSED_CONFIG`** loses about 1.6 Wh per orbit; `low_battery` sets within **2 orbits** (seed 1: 1.9 orbits in).
- **Tumbling, for information only:** with attitude control off from the start, the rate drifts up, the payload never acquires (it needs `STABILIZED`), and the arrays sweep through all angles. Orbit-average generation 2.49 Wh (31% of the stabilized value), margin -43%, minimum SOC 0.405 after one orbit. Not a pass/fail check: the outcome depends on the disturbance.

### Cold case: an expected outcome, not a bug

Under the contract suite's severe -150 °C environment (applied in sunlight and eclipse, with the nominal orbit), the unheated electronics cross -25 °C and `under_temp` sets about **2 minutes** in (the test allows 5). The real flight computer enters SAFE 0.9 s after the flag appears (#48), which turns the payload off. The survival heater switches on when the battery falls below 1 °C and never switches off (100% duty after its first switch-on): at -150 °C its 3 W cannot hold the battery anywhere near the OFF setpoint, and both nodes fall far below their limits. Energy goes negative (-0.6 Wh per orbit), so at that rate `low_battery` follows after about 6 orbits and `critical_battery` later. This is the designed chain: hardwired heater at full duty, flag, SAFE (`tests/sil/test_safe_mode_story.py` follows it through recovery).

## Parameters

Everything not listed is unchanged and was checked against the profiles above. Real-world ranges are for a 1U–3U CubeSat-class spacecraft.

| Parameter | Old → new default | Plausible range | Rationale |
|---|---|---|---|
| `CommsConfig.transmitter_on_power_w` | 1.0 → **0.15** W | 0.05–0.5 W (UHF transceiver enabled, not keyed) | Option C: idle draw only, since the radio stays `RX_TX` |
| `CommsConfig.transmit_power_per_byte_w` | 0.005 → **0.02** W/byte | full-capacity keyed draw 1.5–4 W for 0.5–1 W RF out | 2.55 W keyed at full rate; stated per 100 ms tick and scaled to the tick (#122), so 2 mJ per byte at any tick |
| `CommsConfig.transmit_rate_bytes_per_s` (was `transmit_capacity_bytes` = 120 per tick until #122) | 1200 B/s (unchanged: 120 per 100 ms tick) | 1200–19 200 bit/s UHF | 9600 bit/s at any tick length |
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
- Traffic is the flight computer's own (#98), but the ground is ideal: every pass is commanded on time and nothing is lost on the way down; no retransmissions or acknowledgement-based release (#56 releases once sent).
- Sensor noise is Gaussian bounded at 6σ, with no bias or drift.

Pessimistic on purpose: the pass sits at the end of eclipse, and the data budget keeps an ACK/NACK allowance in every telemetry tick of the pass.

## Rule for scenario authors

The nominal defaults are deliberately comfortable. **Any scenario about power or thermal stress uses `STRESSED_CONFIG`, a different starting state (`SpacecraftInitialState`), or `EnvironmentState` overrides; never retuned nominal defaults.** "Start low and see what happens" uses the starting charge (`PowerInitial.soc`); `battery_soc_override` pins the charge and is for exercising flags and edge cases (ADR-0005).

## Runtime

Each profile runs whole orbits at the 100 ms tick (55 200 ticks per orbit) through `SilTarget` with the real flight computer (#98), about 4–5 s per orbit on an Apple Silicon laptop (the subsystem stack alone took 2.3–2.9 s). Locally the whole budget module takes about 65 s: reference (3 orbits) 15.6 s, continuous full-capacity transmit 11.6 s, stressed SCIENCE 9.0 s, +1 W 5.2 s, SCIENCE 4.7 s, cold 4.7 s, tumbling 4.4 s, NOMINAL 3.9 s. It runs in CI's parallel slow job (#78).
