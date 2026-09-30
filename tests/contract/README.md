# TestTarget contract suite

Target-agnostic tests that every `TestTarget` implementation must pass (ADR-0002).
The suite drives targets only through the public interface, so the same tests apply
to `EchoTarget` (stub), `SilTarget`, and `HilTarget`.

Run just the contract suite:

```sh
uv run pytest tests/contract
```

Each test runs once per registered target, with IDs like `[echo]`, `[sil]`, `[hil]`.

## What the contract checks

| Area | Requirement |
|---|---|
| Interface | The target satisfies the `TestTarget` protocol. |
| Capabilities | `capabilities` is a `TargetCapabilities` with bool flags and a `frozenset[str]` of fault types. |
| Faults | Every supported fault is accepted by `inject()`. An unsupported fault type raises `UnsupportedFaultError` naming the fault. |
| Environment | `apply_environment()` is accepted in any state after `reset()`: right after reset, with uplink pending, with downlink undrained, after draining, with a fault active, and after a second reset. |
| `receive()` | Returns `[]` before time advances. Returns only frames produced since the previous call, so a second call with no `advance()` in between returns `[]`. Every frame decodes as a valid wire frame. |
| `reset()` | Discards output that was produced but not yet received. |
| Determinism | If `capabilities.deterministic` is true, the same seed and inputs produce identical output. Skipped for non-deterministic targets (HIL). |

## Adding a target

Register a `TargetCase` in `contract_support.py`:

```python
TargetCase(
    name="sil",
    factory=SilTarget,                    # returns a new, unconnected target
    stimulus=PING_FRAME,                  # an uplink frame that gets a downlink response
    sample_faults=(                       # one valid fault per supported fault type
        base.TargetFault(fault_type="mcu_reset"),
        base.TargetFault(fault_type="sensor_freeze", params={"sensor": "battery"}),
    ),
    response_steps=1,                     # advance(STEP_S) calls before a response is due
),
```

- `sample_faults` must cover `capabilities.supported_faults` exactly; the suite checks this.
- `stimulus` must produce at least one valid downlink frame within `response_steps` steps.
- For hardware targets, pass `marks=(pytest.mark.hil,)` (or `skipif` on missing hardware)
  so the HIL case can be selected or skipped in CI. Register the marker in
  `pyproject.toml`, since pytest runs with `--strict-markers`.

The `target` fixture (in `conftest.py`) builds the target, calls `connect()` and
`reset(seed=0)`, and calls `close()` afterwards.

## Time step

`STEP_S` in `contract_support.py` is the `advance()` step in seconds. ADR-0003 proposes
changing `advance(dt: float)` to `advance(dt_us: int)`; when that lands, only `STEP_S`
and the `advance` calls change.

Target-specific behavior (for example, how `EchoTarget` handles its `mute` fault) is
tested in `tests/unit/`, not here.
