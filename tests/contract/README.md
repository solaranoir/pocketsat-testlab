# TestTarget contract suite

Target-agnostic tests that every `TestTarget` implementation must pass (ADR-0002).
The suite drives targets only through the public interface, so the same tests apply
to `EchoTarget` (stub), `SilTarget`, and `HilTarget`.

Run just the contract suite:

```sh
uv run pytest tests/contract
```

Each test runs once per registered target, with the target's name as the test ID
suffix (`[echo]`, `[sil]`).

## Registered targets

| ID | Target | Stimulus | Sample faults | Status |
|---|---|---|---|---|
| `echo` | `EchoTarget` (stub, `pocketsat.targets.echo`) | `PING_FRAME`, echoed back | `mute` for 1 s | Runs |
| `sil` | `SilTarget` with the real `FlightComputer` and its default settings (`pocketsat.targets.sil`) | `PING_FRAME`, answered with an ACK frame in the same tick | `forced_reset` (held 1 s), `sensor_freeze` (`subsystem="thermal"`), `transmitter_off`, `battery_drain` (`load_w=2.0`), each for 1 s | Runs |
| `hil` | `HilTarget` | `PING_FRAME` | none | Placeholder, skipped with an explicit reason until Phase 7; `HilTarget` does not exist yet |

The `sil` case runs inside the flight computer's 5 s BOOT, because the suite never
advances more than a few seconds. That is deliberate: PING is answered in every mode,
BOOT included (#51), so the contract does not depend on the mode. Mode-dependent
command behavior is SIL-specific and is tested in `tests/sil/` and
`tests/unit/test_command_dispatcher.py`, not here.

## What the contract checks

| Area | Requirement |
|---|---|
| Interface | The target satisfies the `TestTarget` protocol. |
| Capabilities | `capabilities` is a `TargetCapabilities` with bool flags and a `frozenset[str]` of fault types. |
| Faults | `sample_faults` covers `supported_faults` exactly. Every supported fault is accepted by `inject()`, is released after its `duration_us` (the stimulus gets a reply again), and is cleared by `reset()` when it has no duration. An unsupported fault type raises `UnsupportedFaultError` naming the fault. These tests are selected by `capabilities.supported_faults` and skipped for a target that declares none. |
| Environment | `apply_environment()` is accepted in any state after `reset()`: right after reset, with uplink pending, with downlink undrained, after draining, with a fault active, and after a second reset. This runs for every environment in `CONTRACT_ENVIRONMENTS`, which covers each `EnvironmentState` field at nominal and extreme values (eclipse, -150 °C and 120 °C, zero and 5x sensor noise, battery override at 0 and 1, and a combined worst case). Under each one the target keeps stepping, and any frames it sends decode as valid frames; it may send none, for example with an empty battery. |
| `receive()` | Returns `[]` before time advances. Returns only frames produced since the previous call, so a second call with no `advance()` in between returns `[]`. Every frame decodes as a valid wire frame. |
| `reset()` | Discards output that was produced but not yet received. |
| Determinism | If `capabilities.deterministic` is true, the same seed and inputs, including a sequence through every contract environment and every sample fault, produce identical output, and a `reset(seed)` in the middle of a run (uplink pending, output undrained, faults active) reproduces a fresh target's run exactly. Selected by `capabilities.deterministic`, so skipped for non-deterministic targets (HIL). |

## Adding a target

Register a `TargetCase` in `contract_support.py`. The `sil` case, for example:

```python
TargetCase(
    name="sil",
    factory=SilTarget,                    # returns a new, unconnected target
    stimulus=PING_FRAME,                  # an uplink frame that gets a downlink response
    sample_faults=(                       # one valid fault per supported fault type
        base.TargetFault(fault_type="forced_reset", duration_us=1_000_000),
        base.TargetFault(fault_type="sensor_freeze", params={"subsystem": "thermal"}, ...),
        base.TargetFault(fault_type="transmitter_off", duration_us=1_000_000),
        base.TargetFault(fault_type="battery_drain", params={"load_w": 2.0}, ...),
    ),
    response_steps=1,                     # advance(STEP_US) calls before a response is due
),
```

- `sample_faults` must cover `capabilities.supported_faults` exactly, with valid
  parameters; the suite checks this. The suite may replace a sample's `duration_us`.
- `stimulus` must produce at least one valid downlink frame within `response_steps`
  steps whenever no fault is active: after `reset()`, later in the run, and after a
  fault is released.
- For hardware targets, pass `marks=(pytest.mark.hil,)` (or `skipif` on missing hardware)
  so the HIL case can be selected or skipped in CI. Register the marker in
  `pyproject.toml`, since pytest runs with `--strict-markers`. Until Phase 7 the `hil`
  case carries `pytest.mark.skip` with the reason, and its factory raises.
- Tests for something only some targets have are selected by the declared
  `capabilities` (for example `deterministic`, `supported_faults`), never by the case's
  name.

The `target` fixture (in `conftest.py`) builds the target, calls `connect()` and
`reset(seed=0)`, and calls `close()` afterwards.

## Time step

`STEP_US` in `contract_support.py` is the `advance(dt_us)` step: one default tick
(100 ms), in integer microseconds as ADR-0003 requires. Fault durations in
`sample_faults` are integer microseconds too (`duration_us`).

Target-specific behavior is tested elsewhere, not here: `EchoTarget`'s `mute` fault in
`tests/unit/`, and `SilTarget`'s faults, BOOT, and determinism over long runs in
`tests/sil/` (`test_sil_faults.py`, `test_sil_determinism.py`).
