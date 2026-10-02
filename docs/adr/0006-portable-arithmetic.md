# ADR-0006: Portable arithmetic in simulation code

- **Status:** Accepted
- **Phase:** 1
- **Amends / Supersedes:** Amends ADR-0003 §3, which names `gauss` as a stable method. Issue: #75.

## Context

ADR-0003 makes a SIL run bit-for-bit reproducible for the same seed, and the golden telemetry test (#64) compares frames byte for byte. That only holds across machines if the arithmetic itself is identical on every platform. Development happens on macOS (Apple Silicon) and CI runs on Linux (x86-64).

- IEEE 754 requires `+ - * /` and square root to be exactly rounded, so they give identical results everywhere, and CPython evaluates each operation separately.
- `math.exp`, `log`, `sin`, `cos`, `atan2`, `pow`, and the rest call the platform's maths library (Apple libm versus glibc). Those are not required to be exactly rounded and can differ in the last bit.
- `random.Random.gauss` calls `log` and `cos` internally, so sensor noise drawn with it is not portable either, although ADR-0003 lists `gauss` as stable. It is stable across runs on one machine, not across platforms.

A last-bit difference rarely stays small. Near a flag threshold it can flip a flag, change the tick SAFE is entered, and make every later frame differ.

## Decision

### 1. Simulation code uses only portable operations

Simulation code is everything under `src/pocketsat` except `pocketsat.reporting`.

**Allowed:** `+ - * /`, `math.sqrt`, `abs`, `min`, `max`, `math.floor`, `math.ceil`, `math.isfinite`, comparisons, `**` with a non-negative integer-literal exponent, `random.Random.random()`, and `uniform()`.

**Not allowed:**

- `math.exp`, `expm1`, `log`, `log1p`, `log2`, `log10`, `pow`, `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `sinh`, `cosh`, `tanh`, `hypot`, `erf`, `gamma`, and the built-in `pow()`
- `**` with any exponent that is not a non-negative integer literal (`x ** 0.5`, `x ** n`, `x ** -1`)
- `random.Random` distributions: `gauss`, `normalvariate`, `lognormvariate`, `expovariate`, `vonmisesvariate`, `gammavariate`, `betavariate`, `paretovariate`, `weibullvariate`

One caveat about `**`: for a *float* base, CPython evaluates even `x ** 2` through the platform `pow`. The static check can't tell an integer base from a float one, so it allows any integer-literal exponent, but simulation code writes float squares as `x * x`. Integer powers such as `2 ** 64` are exact.

### 2. Portable helpers replace the disallowed functions

- **Noise:** `pocketsat.core.rng.portable_normal(rng, mu, sigma)` sums 12 `rng.random()` draws and subtracts 6 (the Irwin-Hall approximation): mean 0 and variance exactly 1 before scaling, using only arithmetic. Samples are bounded at ±6σ, so the extreme tails of a true normal distribution never occur, which is acceptable for sensor noise.
- **Cosine:** `pocketsat.core.portable.portable_cos_deg(angle_deg)` uses Bhaskara's approximation, cos x ≈ (π² - 4x²) / (π² + x²) for |x| ≤ 90°, with π as a literal constant. It is clamped to 0 beyond ±90°, because its use is a projected-area factor (solar generation against pointing error, #35). The maximum absolute error over ±90° is about 0.0016.
- `RngFactory.stream()` documents that consumers use `random()`, `uniform()`, and `portable_normal()`.

### 3. Modelling consequences

- Step-by-step (explicit) integration instead of closed-form exponentials (thermal, #38).
- Scalar dynamics instead of trigonometry where possible (attitude, #41).
- Sensor noise from `portable_normal` (#36, #39, #41).
- Where a trigonometric function is genuinely needed, a documented arithmetic approximation in `pocketsat.core.portable`, with its accuracy pinned by tests.

### 4. Enforcement

- The determinism guard test (`tests/unit/test_determinism_guard.py`) checks every simulation module statically for the disallowed functions, imports, distribution methods, `pow()`, and `**` exponents, with self-tests for flagged and allowed snippets.
- CI runs the full test suite on both Linux and macOS. The pinned values of the portable helpers, and later the golden telemetry (#64), must match bit for bit on both, which catches anything the static check misses.

## Alternatives considered

- **Generate and compare golden files only on CI's platform.** Hides the problem instead of removing it, makes the golden test unusable locally, and ties the file to one runner image, so a runner upgrade could break it. Rejected.
- **Compare decoded values within a tolerance.** Can't absorb the divergence that follows once a last-bit difference flips a flag or a mode, and it weakens ADR-0003's "same seed, identical frames" guarantee. Rejected.
- **Keep the maths library and pin one platform for everything.** The same problem as the first alternative, and it rules out running the simulation on a developer's machine. Rejected.

## Consequences

- SIL results, and therefore golden files, are byte-identical on every platform.
- The same arithmetic is straightforward to reproduce in C for the Phase 7 firmware.
- Modelling is restricted to explicit integration and approximate normal noise; both are adequate for v1's fidelity.
- New simulation code that reaches for `math.exp` or `gauss` fails the guard test with a message naming the line and the reason.
- Test code is not restricted. Tests may use `math.cos` and friends as references.

## Follow-ups

- ADR-0003's follow-ups point to this ADR (same change).
- `docs/architecture.md` §7 mentions the portable-arithmetic rule (same change).
- The subsystem tickets already require this: #35 (pointing factor through `portable_cos_deg`), #36 and #39 (noise through `portable_normal`), #38 (explicit integration), #41 (scalar dynamics), and #64 (golden file on any platform).
