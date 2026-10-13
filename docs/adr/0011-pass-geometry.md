# ADR-0011: Pass geometry under portable arithmetic

- **Status:** Proposed
- **Phase:** 2
- **Amends / Supersedes:** Amends ADR-0006 §1 and §2 (adds `math.frexp` to the allowed operations and four helpers to `pocketsat.core.portable`), if Decision C (§6) is accepted as recommended. Related: ADR-0003 (integer-microsecond time), ADR-0010 §5 and §6 (who consumes `PassGeometry`, and where it is computed). Issue: #159.

## Context

ADR-0010 §6 places pass geometry on the orchestrator side, beside the environment model, as a separate `PassGeometry` record (`azimuth_deg`, `elevation_deg`, `range_km`, `range_rate_km_per_s`) that the run loop computes once per tick and gives to both the link (physical truth: visibility, link attributes) and the ground station (its pass prediction). It leaves the model to this ADR.

Four Phase 2 tickets consume it: #160 (the geometry provider and pass prediction), #161 (AOS/LOS), #162 (Doppler from range rate), #163 (an SNR indicator from a link budget). Phase 3's RF channel builds its loss model on the same outputs, and its milestone is that "telemetry quality naturally improves and deteriorates over a simulated LEO pass" (`docs/roadmap.md`). Phase 5 scenarios declare pass geometry as input (`docs/architecture.md` §10: `pass: max_elevation_deg: 12, duration_s: 420`).

Constraints:

- **Portable arithmetic (ADR-0006).** Azimuth and elevation need sine, cosine and arctangent over the full circle; the SNR needs a logarithm. None of `math.sin`, `cos`, `atan2`, `log10` may be called in simulation code. `portable_cos_deg` exists but is clamped to ±90° and accurate only to about 0.0016, so it is not usable here.
- **Integer time (ADR-0003).** Geometry is a pure function of `now_us`, and must not drift over long runs or campaigns.
- **Performance.** The full SIL stack is budgeted at 100 µs per tick averaged over an orbit on CI Linux, and measures 37 to 85 µs today (`docs/spacecraft.md`, "Performance budget (#78)"). Geometry is outside `SilTarget`, so it is not inside that figure, but #167 must report the whole loop against it, and the station "must not push the orbit average over budget". There is little headroom.
- **Scope.** "Sophisticated orbital mechanics" is deferred to Version 2 (`docs/roadmap.md`, scope boundary). Geometry runs only on the ground side; the Phase 7a firmware never computes it, so C portability is not a concern, only Linux versus macOS Python.

**Evidence.** A throwaway prototype of the recommended design (not committed) measured, on an Apple Silicon laptop: sine and cosine within 3.9e-13 of `math` over three full turns, `atan2` within 1.4e-13 rad over a 601 × 601 grid, decibels within 5.7e-14 dB over 1e-20 to 1e20; **4.8 µs per tick** for the whole geometry record and **1.2 µs** per decibel conversion; analytic range rate equal to a 1 ms finite difference to four decimals (−6.3437 km/s); and, with the default 92-minute period (altitude 380 km), 51.6° inclination and a station at 40° N, 105° W, **13 passes in two days** above 0° with maximum elevations from 6° to 70°.

## Decision

### 1. The model: a circular orbit over a rotating spherical Earth (Decision A, maintainer)

Options:

| | A1: circular orbit, rotating Earth, orbital elements configured | A2: parametric pass model | **A3: circular orbit, configured by orbital elements *or* by a pass anchor (recommended)** |
|---|---|---|---|
| What it is | Two-body circular orbit (no J2, no drag), spherical Earth turning at the sidereal rate, station at a fixed latitude, longitude, altitude | Each pass is configured directly (start time, duration, maximum elevation, azimuths); elevation, range and range rate are synthesised along a flyby; below the horizon between passes | A1's model. A scenario can also give "maximum elevation E at time t" (and ascending or descending); the orbit's node and phase are solved at construction so that pass happens |
| Realism | Realistic pass shapes, durations, ground-track drift from pass to pass, and daily pass pattern | One pass at a time; a sequence of passes has no orbit behind it. Range, range rate and azimuth must be kept consistent by hand | As A1 |
| Phase 3 link model | Range and elevation rise and fall together, as the Phase 3 milestone needs | Consistency of range with elevation and of range rate with d(range)/dt is the synthesiser's job, and a test burden | As A1 |
| Phase 5 scenarios ("a 12° pass at t = 300 s") | Hard: the author must search for orbital elements | Direct | Direct: the solver does the search |
| Multi-orbit and multi-day tests | Natural (#160: several passes a day, one high) | Each pass listed by hand | Natural |
| Cost | About 5 µs per tick (prototype) | Less, about 2 to 3 µs | As A1 at run time; a deterministic bisection once at construction |
| Scope (v1) | Fits: no perturbations, no ellipsoid | Fits | Fits |

**Recommendation: A3.** One physical model serves both uses: a default orbit that gives the realistic daily pattern #160 asks for, and scenario passes placed exactly where a test needs them. A2 is easier to configure but would have to reinvent consistency between its outputs, which A1's physics gives for free, and Phase 3's link model depends on that consistency.

The solver (part of #160): for a requested maximum elevation `E` at time `t`, bisect the orbit's right ascension of the ascending node so the pass's maximum elevation is `E` (within 0.01°), then shift the initial argument of latitude so it occurs at `t` (within one tick). Both searches step the forward model only, with a fixed iteration count, so construction is deterministic. Elevations above what the orbit's inclination and the station latitude allow are rejected at construction with the reachable maximum in the message.

**Consequence for the scenario schema (Decision B, maintainer).** With a physical orbit, a pass's duration follows from its maximum elevation and the orbit's altitude (a 12° pass at 380 km lasts about 8 to 9 minutes above 0°). The `duration_s` in `docs/architecture.md` §10's example is then over-determined:

| | **B1: drop `duration_s` as an input; scenarios assert it instead (recommended)** | B2: keep it as an input, checked against the solved pass within a tolerance | B3: keep both as inputs (only possible with A2) |
|---|---|---|---|
| Scenario author | Gives maximum elevation and time; can assert `pass_duration_s` bounds | Must give a consistent pair or the scenario fails at load | Free to give any pair, including unphysical ones |
| Model | Unchanged | Unchanged, plus a load-time check | Requires A2 |

Recommendation: B1. Phase 5 owns the schema; this ADR records the constraint so Phase 5 does not inherit a field the model cannot honour.

### 2. Model details

- **Period is the input; radius is derived.** The orbit is configured with `orbit_period_us` (default `DEFAULT_ORBIT_PERIOD_US`, the same constant `NominalEnvironment` uses), inclination, right ascension of the ascending node at `t = 0`, and argument of latitude at `t = 0`. The orbit radius follows from Kepler's third law, a = (μ (P / 2π)²)^(1/3), computed once at construction by Newton's method with a fixed iteration count (arithmetic only). With the defaults, a = 6750.8 km (altitude 380 km).
- **Angles are carried in turns, reduced by integer modulo before any float operation.** The orbital phase is `(now_us % period_us) / period_us` and the Earth rotation angle is `(now_us % SIDEREAL_DAY_US) / SIDEREAL_DAY_US`, with `SIDEREAL_DAY_US = 86_164_090_500` (an integer constant). Both operands are integers below 2⁵³, so each phase is exact to one rounding, at every `now_us`. Accuracy does not degrade over a long run or a campaign.
- **Earth:** a sphere of radius 6371.0 km, rotating at 1 turn per sidereal day. No ellipsoid, no polar motion, no precession.
- **Station:** its latitude, longitude and altitude come from `StationConfig` (ADR-0010 §3). Its Earth-fixed position and topocentric rotation are computed once at construction.
- **Per tick:** the spacecraft's Earth-fixed position and velocity from the argument of latitude `u` and the node angle relative to the rotating Earth (Ω − θ); the station-to-spacecraft vector; range by `sqrt`; east, north and up components; **elevation = atan2(up, √(east² + north²))**, **azimuth = atan2(east, north)** mapped to [0, 360). No `asin` is needed.
- **Range rate is analytic,** ρ · ρ̇ / |ρ|, where ρ̇ is the spacecraft's Earth-fixed velocity (the derivative of the position above, with du/dt = n and d(Ω − θ)/dt = −ω_E). It reuses the same four sines and cosines, so it costs a few multiplications. #160's tests check it against a finite difference of range.
- **No refraction, no terrain:** visibility is the geometric horizon (ADR-0010 open question 5). The elevation mask is the station's policy (ADR-0010 §5).

### 3. Outputs per tick

`PassGeometry` keeps exactly ADR-0010 §6's fields; this ADR adds none. Pass events (AOS, LOS, pass number, maximum elevation) are derived by the station (#161), not the model. Pass *prediction* (next AOS and LOS, the pass's maximum elevation) is a method of the provider (#160) that steps the forward model (a coarse step, then bisection on integer microseconds to one tick), never closed-form inverse trigonometry.

The provider has the same shape as `EnvironmentModel`: `geometry_at(now_us: int) -> PassGeometry`, a pure function of time, and `sample(clock)`. Computed once per tick by the run loop and passed to both consumers (ADR-0010 §6).

### 4. Sunlight and eclipse stay with `NominalEnvironment` (Decision D)

`NominalEnvironment` defines eclipse as a fixed window of each orbit by time (`now_us % P`). There is no Sun vector in v1.

| | **D1: shared period, eclipse stays time-based (recommended)** | D2: derive sunlit state from geometry (Sun direction, cylindrical shadow) |
|---|---|---|
| Consistency | The geometry model and `NominalEnvironment` take the same `orbit_period_us`, so each orbit has exactly one eclipse and both stay in step forever. The run loop checks the two periods are equal at construction | Exact: eclipse follows the orbit's actual Sun geometry and beta angle |
| What changes | Nothing in `NominalEnvironment`, the power and thermal budget (#72), or the golden captures | A Sun model, a new `NominalEnvironment`, budget recalibration (#72), and every golden capture regenerated |
| What it does not model | Where in the orbit the eclipse falls relative to the ground track (a pass can be placed anywhere relative to eclipse, by time) | — |
| Scope | v1 | Closer to "sophisticated orbital mechanics" (v2) |

Recommendation: D1. RF geometry does not depend on the Sun, and the scenarios that care about "a pass at the end of eclipse" (as the performance benchmark does) place it by time. The pass-anchor solver (§1) moves the orbit's phase, not the eclipse window, so placing a pass never changes the power profile.

### 5. Portable functions

The geometry needs sine and cosine over the full circle and a full-circle arctangent; the SNR needs a logarithm. Options for the trigonometry:

| | **Range reduction plus fixed polynomials (recommended)** | CORDIC | Table plus interpolation |
|---|---|---|---|
| Method | Reduce turns to an octant with `floor`; Taylor polynomials on [0, π/4] in Horner form with coefficients 1/n! computed by division (exactly rounded). Arctangent: reduce to [0, tan(π/8)], halve the argument once with `sqrt` (atan x = 2 atan(x / (1 + √(1 + x²)))), then a Taylor series | Shift-and-add rotations, one iteration per bit | Precomputed values with linear or cubic interpolation |
| Accuracy | Prototype: 3.9e-13 (sin, cos), 1.4e-13 rad (atan2) | About 2⁻ⁿ after n iterations; 1e-12 needs about 40 | Bounded by table size; 1e-12 needs a very large table or high-order interpolation |
| Cost in CPython | About 1 µs per call | A Python loop of about 40 iterations: several times slower | An index and a few multiplications, but the table must be literal constants (computing it with `math` at import is not portable) |
| Portability | Only `+ - * /`, `sqrt`, `floor`: exactly rounded everywhere | Same | Same, if the table is literal |

Recommendation: range reduction plus polynomials. They are the most accurate and fastest of the three in CPython.

New helpers in `pocketsat.core.portable` (ADR-0006 §2), each with pinned values and accuracy tests against `math` (tests may use `math`):

| Helper | Domain | Pinned accuracy bound (absolute) |
|---|---|---|
| `portable_sincos_turns(turns) -> tuple[float, float]` | any finite value, in turns | 1e-12 |
| `portable_atan2(y, x) -> float` (radians, (−π, π], `atan2(0, 0) = 0`) | any finite pair | 1e-12 rad |
| `portable_log2(x) -> float` | x > 0, finite | 1e-12 |
| `portable_db(ratio) -> float` (10 log₁₀) | ratio > 0, finite | 1e-11 dB |

The bounds give the prototype's measured errors a margin of about 2.5 times or more. Byte-identical results across Linux and macOS are additionally pinned by exact expected values (as `portable_cos_deg`'s tests do), so a reordering of operations shows up as a test failure.

### 6. Decibels without `log10` (Decision C, maintainer)

#163's link budget is additive in decibels: SNR = P_tx + G_tx + G_rx − FSPL − losses − N, where FSPL = 20 log₁₀(d) + 20 log₁₀(f) + constant and N = −174 dBm/Hz + 10 log₁₀(B) + NF. Only the range term changes per tick; the frequency, bandwidth and constant terms are computed once at construction (with the same portable function).

| | **C1: `frexp` decomposition plus a series (recommended)** | C2: integer dB table | C3: stay linear; convert to dB only in reporting |
|---|---|---|---|
| Method | `math.frexp` splits x into m · 2^e exactly (no rounding: it reads the float's bits). Shift m into [1/√2, √2), then ln m = 2 atanh((m − 1)/(m + 1)) by a 12-term odd series; log₂ x = e + ln m / ln 2; dB = 10 log₁₀2 · log₂ x | A table of 10 log₁₀ over one decade, in 0.01 dB steps, scaled by powers of ten found by comparison | The link budget multiplies linear ratios; `Packet.snr_db` is filled by the reporting layer, which ADR-0006 exempts |
| Accuracy | Prototype: 5.7e-14 dB over 1e-20 to 1e20 | 0.01 dB steps, or interpolation error | Exact in the simulation |
| Cost | 1.2 µs per call (prototype) | Similar | None in the simulation |
| ADR-0006 change | Adds `math.frexp` to the allowed list. It is exact on every platform (IEEE 754 binary decomposition), like `floor` | None, if the table is literal constants | None |
| Downside | One more allowed function, and the guard's allow-list changes | 900 literal constants to review, or interpolation to test | `Packet.snr_db` is in the simulation path (Phase 3 maps SNR to loss on every packet), so decibels in reporting only would mean Phase 3 thresholds in linear units, contrary to the field's name and the ADR-0010 contract |

Recommendation: C1. `frexp` is not a maths-library approximation; it is a bit-exact operation in the same class as `floor`, and the guard does not flag it today. Accepting C1 amends ADR-0006 §1 to list it explicitly, so the reason is recorded rather than left to the guard's omission.

Doppler (#162) needs no new function: Δf = −f · ṙ / c, arithmetic only, with c as a literal constant (299 792.458 km/s). It is positive while the spacecraft approaches (ṙ < 0).

### 7. Determinism and performance

- **Determinism.** Geometry is a pure function of `now_us` and construction parameters; it draws no random numbers. The new module is under `src/pocketsat`, so the determinism guard scans it with no listing change; CI runs the pinned values on Linux and macOS. Geometry is not an input to `SilTarget`, so the golden captures are unaffected.
- **Performance.** The prototype costs 4.8 µs per tick on a laptop, 5 to 13 % of the 37 to 85 µs the SIL stack measures on CI today. Budget: **at most 10 µs per tick for `geometry_at` on CI Linux**, printed by #160 as guidance (the same treatment as the per-subsystem figure in `docs/spacecraft.md`), and included in #167's whole-loop figure. If the loop goes over budget, the first lever is computing az/el only when the station or link needs them (both need elevation every tick; azimuth only near a pass), before any coarser cadence.
- **Computed once per tick** by the run loop and shared by the link and the station (ADR-0010 §6). #163's SNR adds one `portable_db` per downlink tick (about 1.2 µs).

## Alternatives considered

- **A2, the parametric pass model.** Rejected (§1): easiest to configure, but synthesised range, range rate and azimuth must be kept consistent by hand, and multi-pass or multi-day tests (#160's passes per day) have no orbit behind them.
- **SGP4 with two-line element sets.** Realistic and familiar to amateur operators, but it is "sophisticated orbital mechanics" (v2), needs many more trigonometric evaluations per tick, and real TLEs bring real-world epochs into a deterministic simulation. Rejected for v1.
- **An oblate Earth (WGS-84) for the station.** Changes the station position by up to about 21 km, which matters for real tracking but not for a test platform's link model. Rejected; a single constant could change later without changing any interface.
- **`asin` for elevation.** Needs a separate approximation that loses accuracy near ±90°; `atan2(up, horizontal)` reuses the one arctangent and is well conditioned everywhere. Rejected.
- **Range rate by finite difference.** Doubles the per-tick cost and introduces a step-size choice. Rejected for the analytic form, which reuses the same sines and cosines.
- **Using `portable_cos_deg`.** Clamped to ±90° and accurate to 0.0016, which gives errors of kilometres in position. Rejected; it stays for its projected-area use.
- **Angles in radians or degrees from floating-point time.** Accuracy degrades as `now_us` grows (a large float argument needs a long range reduction). Rejected for turns reduced by integer modulo (§2).

## Consequences

- The ground station, the link and Phase 3's RF channel get a physically consistent geometry series, and scenarios can still ask for a specific pass.
- `pocketsat.core.portable` grows four general helpers that later phases can reuse (for example Phase 3's loss model, which needs decibels).
- Under C1, ADR-0006's allowed list gains `math.frexp`.
- The `duration_s` input in `docs/architecture.md` §10 no longer fits the model; Phase 5 resolves it per Decision B.
- Geometry adds about 5 µs to every tick of the Phase 2 loop, against a budget that already has little headroom; #167 measures it.
- No perturbations: passes repeat exactly, with no drag decay or nodal precession over a campaign. A Version 2 orbit model replaces the provider behind the same `geometry_at` shape.

## Open questions for the maintainer

1. **Decision A (§1):** A3 circular orbit with a pass-anchor solver (recommended), A1 orbital elements only, or A2 parametric passes.
2. **Decision B (§1):** B1 drop `duration_s` as a scenario input and assert it instead (recommended), B2 keep it with a consistency check, or B3 keep both (needs A2).
3. **Decision C (§6):** C1 `frexp` plus a series, amending ADR-0006 (recommended), C2 a literal dB table, or C3 linear SNR in the simulation.
4. **Decision D (§4):** D1 shared period with time-based eclipse (recommended), or D2 eclipse from Sun geometry.
5. **Default orbit and station (#160).** Recommended: the 92-minute default period, 51.6° inclination, and a mid-latitude station (the prototype's 40° N, 105° W gives about 6.5 passes a day above 0°, with high passes near 70°). #160 chooses and documents the values.

## Follow-ups

- **#160**: the four portable helpers with pinned values and accuracy tests; the geometry provider in `pocketsat.environment` (§2, §3), the pass-anchor solver (§1), and pass prediction by stepping; the 10 µs guidance figure; documented defaults; the run loop's check that geometry and `NominalEnvironment` share the period (or #167, which builds the loop).
- **#161**: AOS/LOS from `PassGeometry.elevation_deg` against the station's mask; no new geometry functions.
- **#162**: Doppler from `range_rate_km_per_s` (§6), arithmetic only.
- **#163**: the link budget in decibels with `portable_db` (§6), constants computed once at construction.
- Under C1: ADR-0006 §1 lists `math.frexp`, and the guard's self-tests include an allowed `frexp` snippet (in #160).
- Phase 5: the scenario `pass:` schema follows Decision B.
