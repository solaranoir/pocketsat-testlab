"""Demo of epic #30: the spacecraft subsystem models running together.

Wires the real power, thermal, attitude, and payload subsystems through a shared
``SnapshotBoard``, drives them with ``NominalEnvironment`` and a ``SimClock``, and
prints a text report for a few short scenarios::

    uv run python scripts/demo_spacecraft.py                     # every scenario
    uv run python scripts/demo_spacecraft.py --scenario faults   # just one
    uv run python scripts/demo_spacecraft.py --orbits 3 --seed 7

Communications (#44) is not merged yet, so the stack uses a clearly labelled stand-in
from ``pocketsat.spacecraft.fakes`` (power needs comms' ``transmit_power_w``). The
stand-in lives in :func:`build_stack` only.

Only public APIs are used. Simulated time is integer microseconds; the wall clock is
read only to print how long the demo took.
"""

import argparse
import hashlib
import math
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from fractions import Fraction
from typing import Any, TextIO

from pocketsat.core import RngFactory, SimClock
from pocketsat.environment import EnvironmentModel, NominalEnvironment
from pocketsat.spacecraft import (
    NOMINAL_CONFIG,
    Attitude,
    AttitudeConfig,
    AttitudeControls,
    AttitudeInitial,
    AttitudeSnapshot,
    AttitudeState,
    Payload,
    PayloadControls,
    PayloadSnapshot,
    PayloadState,
    Power,
    PowerInitial,
    PowerSnapshot,
    SnapshotBoard,
    SpacecraftConfig,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    SubsystemStack,
    Thermal,
    ThermalInitial,
    ThermalSnapshot,
)
from pocketsat.spacecraft.fakes import FakeSubsystem, default_snapshot
from pocketsat.targets.base import EnvironmentState

US_PER_S = 1_000_000
US_PER_MIN = 60 * US_PER_S

SCENARIOS = ("nominal", "detumble", "faults", "determinism", "cold")

COMMS_STAND_IN_NOTE = (
    "comms: STAND-IN (fakes.FakeSubsystem holding the default comms snapshot, "
    "constant 1.0 W transmit draw) until #44 merges"
)

# --- Stack and run loop ----------------------------------------------------------------


def build_stack(
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState | None = None,
) -> SubsystemStack:
    """Build the real subsystems on one shared ``SnapshotBoard``.

    Power, thermal, attitude, and payload are the real models from ``main``. Comms is a
    stand-in until #44 merges; it is the only fake in the stack.
    """
    initial = SpacecraftInitialState() if initial is None else initial
    board = SnapshotBoard()
    # TODO(#44): replace this stand-in with the real subsystem, e.g.
    #   comms = Comms(config.comms, board)
    comms = FakeSubsystem("comms", default_snapshot("comms"))
    # Typed loosely: Thermal and Payload declare ``name: Final``, which mypy treats as
    # not matching the Subsystem protocol's settable ``name: str``.
    subsystems: tuple[Any, ...] = (
        Power(config.power, initial.power, board),
        Thermal(config.thermal, initial.thermal, board),
        Attitude(config.attitude, initial.attitude),
        Payload(config.payload, initial.payload, board),
        comms,
    )
    return SubsystemStack(subsystems, board=board)


@dataclass(frozen=True)
class Sample:
    """One tick's result: the time at the end of the tick, its environment and
    controls, and every subsystem's snapshot."""

    now_us: int
    env: EnvironmentState
    controls: SpacecraftControls
    state: SpacecraftState

    @property
    def power(self) -> PowerSnapshot:
        return self.state.get("power", PowerSnapshot)

    @property
    def thermal(self) -> ThermalSnapshot:
        return self.state.get("thermal", ThermalSnapshot)

    @property
    def attitude(self) -> AttitudeSnapshot:
        return self.state.get("attitude", AttitudeSnapshot)

    @property
    def payload(self) -> PayloadSnapshot:
        return self.state.get("payload", PayloadSnapshot)

    @property
    def minutes(self) -> float:
        return self.now_us / US_PER_MIN


ControlsAt = Callable[[int], SpacecraftControls]
"""Controls for the tick starting at simulated time ``now_us``. In the full system the
flight computer and fault injectors produce these; the demo scripts them."""

PAYLOAD_ON = SpacecraftControls(payload=PayloadControls(enabled=True))


@dataclass
class Run:
    """A stack, a clock, and an environment model, stepped together."""

    stack: SubsystemStack
    env_model: EnvironmentModel
    clock: SimClock
    seed: int
    initial: SpacecraftState = field(init=False)

    def __post_init__(self) -> None:
        self.stack.reset(RngFactory(self.seed))
        self.initial = self.stack.snapshot()

    def advance(self, ticks: int, controls_at: ControlsAt) -> list[Sample]:
        """Step ``ticks`` ticks and return a sample per tick."""
        samples = []
        for _ in range(ticks):
            env = self.env_model.state_at(self.clock.now_us)
            controls = controls_at(self.clock.now_us)
            self.stack.step(self.clock.tick_us, env, controls)
            self.clock.advance_one_tick()
            samples.append(Sample(self.clock.now_us, env, controls, self.stack.snapshot()))
        return samples


def new_run(
    seed: int,
    tick_us: int,
    *,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState | None = None,
    env_model: EnvironmentModel | None = None,
) -> Run:
    """Reset a fresh stack under ``seed`` and return it ready to step."""
    return Run(
        stack=build_stack(config, initial),
        env_model=NominalEnvironment() if env_model is None else env_model,
        clock=SimClock(tick_us),
        seed=seed,
    )


def ticks_for(duration_us: int | Fraction, tick_us: int) -> int:
    """Whole ticks covering ``duration_us`` (rounded up), with no float time."""
    return math.ceil(Fraction(duration_us) / tick_us)


def every(samples: Sequence[Sample], interval_us: int) -> list[Sample]:
    """The samples that fall on multiples of ``interval_us``, plus the last one."""
    picked = [s for s in samples if s.now_us % interval_us == 0]
    if samples and (not picked or picked[-1] is not samples[-1]):
        picked.append(samples[-1])
    return picked


# --- Formatting ------------------------------------------------------------------------


def heading(out: TextIO, title: str) -> None:
    out.write(f"\n{'=' * 100}\n{title}\n{'=' * 100}\n")


def att_short(state: AttitudeState) -> str:
    return {"tumbling": "TUMBL", "detumbling": "DETUM", "stabilized": "STAB"}[state.value]


def flags_of(s: Sample) -> str:
    names = [
        name
        for name, on in (
            ("LOW", s.power.readings.low_battery),
            ("CRIT", s.power.readings.critical_battery),
            ("UNDER", s.thermal.readings.under_temp),
            ("OVER", s.thermal.readings.over_temp),
        )
        if on
    ]
    return ",".join(names) or "-"


TIMELINE_HEADER = (
    "  t(min) light   SOC true/est  bus V  gen W load W  batt C  elec C  heat  "
    "attitude  ptg deg  payload   buf B  flags"
)


def timeline_row(s: Sample) -> str:
    p, th, at, pl = s.power, s.thermal, s.attitude, s.payload
    return (
        f"  {s.minutes:6.1f} {'SUN ' if s.env.sunlit else 'ECL '}"
        f"  {p.truth.soc:5.3f}/{p.readings.soc:5.3f}"
        f"  {p.readings.bus_v:5.2f}"
        f"  {p.truth.generation_w:5.2f} {p.truth.total_load_w:6.2f}"
        f"  {th.truth.battery_c:6.1f}  {th.truth.electronics_c:6.1f}"
        f"  {'ON ' if th.truth.heater_on else 'off'}"
        f"  {att_short(at.truth.state):8s} {at.truth.pointing_error_deg:7.1f}"
        f"  {pl.truth.state.value:9s} {pl.truth.buffered_bytes:6d}"
        f"  {flags_of(s)}"
    )


def write_timeline(out: TextIO, samples: Sequence[Sample]) -> None:
    out.write(TIMELINE_HEADER + "\n")
    for s in samples:
        out.write(timeline_row(s) + "\n")


# --- Scenario 1: nominal orbit ---------------------------------------------------------


def eclipse_spans(samples: Sequence[Sample]) -> list[list[Sample]]:
    """Group consecutive eclipse samples."""
    spans: list[list[Sample]] = []
    current: list[Sample] = []
    for s in samples:
        if not s.env.sunlit:
            current.append(s)
        elif current:
            spans.append(current)
            current = []
    if current:
        spans.append(current)
    return spans


def heater_cycles(span: Sequence[Sample], before_on: bool) -> int:
    """Off-to-on transitions of the survival heater across ``span``."""
    cycles, was_on = 0, before_on
    for s in span:
        on = s.thermal.truth.heater_on
        cycles += on and not was_on
        was_on = on
    return cycles


def raised_flags(samples: Sequence[Sample]) -> list[str]:
    seen: dict[str, None] = {}
    for s in samples:
        for name in flags_of(s).split(","):
            if name != "-":
                seen[name] = None
    return list(seen) or ["none"]


def scenario_nominal(out: TextIO, seed: int, tick_us: int, orbits: Fraction) -> None:
    heading(out, f"1. NOMINAL ORBIT  ({float(orbits):g} orbit(s), payload enabled, seed {seed})")
    env = NominalEnvironment()
    run = new_run(seed, tick_us, env_model=env)
    samples = run.advance(ticks_for(orbits * env.orbit_period_us, tick_us), lambda _t: PAYLOAD_ON)
    out.write(
        f"Orbit {env.orbit_period_us // US_PER_MIN} min:"
        f" sunlit {env.sunlit_us / US_PER_MIN:.1f} min,"
        f" eclipse {env.eclipse_us / US_PER_MIN:.1f} min (ambient +20 C sunlit, -20 C eclipse).\n"
        "Starts DETUMBLING at 45 deg / 1.0 dps (default AttitudeInitial). Truth values,"
        " except 'est' (SOC estimate from the noisy voltage) and bus V (reported).\n\n"
    )
    write_timeline(out, every(samples, 5 * US_PER_MIN))

    dt_h = tick_us / US_PER_S / 3600
    gen_wh = sum(s.power.truth.generation_w for s in samples) * dt_h
    load_wh = sum(s.power.truth.total_load_w for s in samples) * dt_h
    capacity = NOMINAL_CONFIG.power.battery_capacity_wh
    soc0 = run.initial.get("power", PowerSnapshot).truth.soc
    soc1 = samples[-1].power.truth.soc
    max_err = max(abs(s.power.readings.soc - s.power.truth.soc) for s in samples)
    stab = next((s for s in samples if s.attitude.truth.state is AttitudeState.STABILIZED), None)
    out.write("\nSummary\n")
    out.write(
        f"  energy: generated {gen_wh:.2f} Wh, consumed {load_wh:.2f} Wh,"
        f" net {gen_wh - load_wh:+.2f} Wh"
        f"  | battery {capacity * soc0:.2f} -> {capacity * soc1:.2f} Wh"
        f" (SOC {soc0:.3f} -> {soc1:.3f})\n"
    )
    out.write(f"  SOC estimate: max |est - true| = {max_err:.4f} (bound 0.02)\n")
    if stab is not None:
        out.write(f"  attitude: STABILIZED after {stab.minutes:.1f} min\n")
    previous_on = run.initial.get("thermal", ThermalSnapshot).truth.heater_on
    for i, span in enumerate(eclipse_spans(samples), 1):
        before = samples[samples.index(span[0]) - 1] if samples.index(span[0]) else None
        before_on = previous_on if before is None else before.thermal.truth.heater_on
        on_ticks = sum(s.thermal.truth.heater_on for s in span)
        low = min(s.thermal.truth.battery_c for s in span)
        out.write(
            f"  eclipse {i}: heater cycles {heater_cycles(span, before_on)},"
            f" duty {100 * on_ticks / len(span):.0f}%, battery min {low:.1f} C"
            f" ({span[-1].minutes - span[0].minutes + tick_us / US_PER_MIN:.1f} min)\n"
        )
    last = samples[-1].payload.truth
    out.write(
        f"  payload: produced {last.total_produced_bytes} B, buffer fill {last.buffer_fill:.1%}\n"
    )
    out.write(f"  flags raised: {', '.join(raised_flags(samples))}\n")


# --- Scenario 2: detumble --------------------------------------------------------------


def scenario_detumble(out: TextIO, seed: int, tick_us: int, control_off_min: int = 4) -> None:
    heading(out, "2. DETUMBLE  (high starting rate; attitude control off, then on)")
    initial = SpacecraftInitialState(
        attitude=AttitudeInitial(pointing_error_deg=120.0, rate_dps=6.0)
    )
    run = new_run(seed, tick_us, initial=initial)
    off_until = control_off_min * US_PER_MIN

    def controls_at(now_us: int) -> SpacecraftControls:
        return SpacecraftControls(
            payload=PayloadControls(enabled=True),
            attitude=AttitudeControls(enabled=now_us >= off_until),
        )

    samples = run.advance(ticks_for(12 * US_PER_MIN, tick_us), controls_at)
    out.write(
        f"Starts at 6.0 dps, 120 deg off sun. Attitude control is OFF for the first"
        f" {control_off_min} min (as in BOOT), then ON."
        " Sunlit throughout; payload commanded on.\n\n"
    )
    out.write("  t(min) ctrl  true state  rate dps  ptg deg  reported    gen W  payload    buf B\n")
    for s in every(samples, 30 * US_PER_S):
        at = s.attitude
        out.write(
            f"  {s.minutes:6.1f} {'on ' if s.controls.attitude.enabled else 'OFF'}"
            f"  {at.truth.state.value:10s} {at.truth.rate_dps:8.3f}"
            f" {at.truth.pointing_error_deg:8.1f}"
            f"  {at.readings.state.value:10s} {s.power.truth.generation_w:5.2f}"
            f"  {s.payload.truth.state.value:9s} {s.payload.truth.buffered_bytes:5d}\n"
        )
    out.write("\nSummary (mean solar generation by true attitude state)\n")
    for st in AttitudeState:
        gens = [s.power.truth.generation_w for s in samples if s.attitude.truth.state is st]
        if gens:
            span_min = len(gens) * tick_us / US_PER_MIN
            out.write(f"  {st.value:10s} {sum(gens) / len(gens):5.2f} W over {span_min:4.1f} min\n")
    first_acq = next((s for s in samples if s.payload.truth.state is PayloadState.ACQUIRING), None)
    stab = next((s for s in samples if s.attitude.readings.state is AttitudeState.STABILIZED), None)
    if stab is not None and first_acq is not None:
        out.write(
            f"  reported STABILIZED at {stab.minutes:.2f} min; payload first ACQUIRING at"
            f" {first_acq.minutes:.2f} min (inhibited until then)\n"
        )


# --- Scenario 3: fault overrides -------------------------------------------------------

STABLE_START = AttitudeInitial(pointing_error_deg=2.0, rate_dps=0.05)


def scenario_faults(out: TextIO, seed: int, tick_us: int) -> None:
    heading(out, "3a. FAULT: battery_drain  (SpacecraftControls.extra_load_w = 50 W from t=1 min)")
    initial = SpacecraftInitialState(power=PowerInitial(soc=0.4), attitude=STABLE_START)
    run = new_run(seed, tick_us, initial=initial)
    drain_from = US_PER_MIN

    def drain(now_us: int) -> SpacecraftControls:
        return SpacecraftControls(
            payload=PayloadControls(enabled=True),
            extra_load_w=50.0 if now_us >= drain_from else 0.0,
        )

    samples = run.advance(ticks_for(10 * US_PER_MIN, tick_us), drain)
    out.write(
        "Sunlit, stabilized, SOC 0.40, payload on. Thresholds: low < 0.30, critical < 0.15.\n\n"
    )
    out.write("  t(min) drain W  load W  SOC true   est   bus V  low   crit  payload    buf B\n")
    for s in every(samples, US_PER_MIN):
        p = s.power
        out.write(
            f"  {s.minutes:6.1f} {s.controls.extra_load_w:7.1f} {p.truth.total_load_w:7.2f}"
            f"  {p.truth.soc:7.3f} {p.readings.soc:5.3f}  {p.readings.bus_v:5.2f}"
            f"  {'SET ' if p.readings.low_battery else '-   '}"
            f"  {'SET ' if p.readings.critical_battery else '-   '}"
            f"  {s.payload.truth.state.value:9s} {s.payload.truth.buffered_bytes:5d}\n"
        )
    for flag in ("low_battery", "critical_battery"):
        hit = next((s for s in samples if getattr(s.power.readings, flag)), None)
        if hit is not None:
            out.write(
                f"  {flag} set at {hit.minutes:.2f} min"
                f" (SOC estimate {hit.power.readings.soc:.3f})\n"
            )

    heading(
        out,
        "3b. FAULT: sensor_freeze"
        "  (frozen_sensors = {power, thermal, attitude} from t=2 to t=6 min)",
    )
    run = new_run(seed, tick_us, initial=SpacecraftInitialState(attitude=STABLE_START))
    frozen = frozenset({"power", "thermal", "attitude"})

    def freeze(now_us: int) -> SpacecraftControls:
        on = 2 * US_PER_MIN <= now_us < 6 * US_PER_MIN
        return SpacecraftControls(
            payload=PayloadControls(enabled=True),
            extra_load_w=10.0,
            frozen_sensors=frozen if on else frozenset(),
        )

    samples = run.advance(ticks_for(8 * US_PER_MIN, tick_us), freeze)
    out.write("A constant 10 W drain makes the truth move; readings hold while frozen.\n\n")
    out.write(
        "  t(min) frozen  SOC true    rep   batt C true    rep"
        "  elec C true    rep  ptg true   rep\n"
    )
    for s in every(samples, 30 * US_PER_S):
        p, th, at = s.power, s.thermal, s.attitude
        out.write(
            f"  {s.minutes:6.1f} {'YES' if s.controls.frozen_sensors else 'no ':6s}"
            f"  {p.truth.soc:8.4f} {p.readings.soc:6.4f}"
            f"  {th.truth.battery_c:11.2f} {th.readings.battery_c:6.2f}"
            f"  {th.truth.electronics_c:11.2f} {th.readings.electronics_c:6.2f}"
            f"  {at.truth.pointing_error_deg:8.2f} {at.readings.pointing_error_deg:5.2f}\n"
        )
    during = [s for s in samples if s.controls.frozen_sensors]
    held = all(s.power.readings == during[0].power.readings for s in during)
    moved = during[-1].power.truth.soc - during[0].power.truth.soc
    out.write(
        f"\n  while frozen: power readings held={held}; true SOC moved {moved:+.4f};"
        " readings resume on release\n"
    )


# --- Scenario 4: determinism -----------------------------------------------------------


def digest(samples: Sequence[Sample], part: str) -> str:
    """SHA-256 over every subsystem's ``part`` record (``truth`` or ``readings``)."""
    h = hashlib.sha256()
    for s in samples:
        h.update(str(s.now_us).encode())
        for name, snap in s.state.subsystems.items():
            h.update(f"{name}:{getattr(snap, part)!r}".encode())
    return h.hexdigest()[:16]


def _truth(snapshot: object) -> object:
    return getattr(snapshot, "truth")  # noqa: B009 - every stack snapshot is a contract snapshot


def first_divergence(a: Sequence[Sample], b: Sequence[Sample]) -> tuple[Sample, list[str]] | None:
    """The first sample whose truth differs between two runs, and which subsystems differ."""
    for x, y in zip(a, b, strict=True):
        names = [
            n
            for n, snap in x.state.subsystems.items()
            if _truth(snap) != _truth(y.state.subsystems[n])
        ]
        if names:
            return x, names
    return None


def scenario_determinism(out: TextIO, seed: int, tick_us: int, minutes: int = 20) -> None:
    heading(out, f"4. DETERMINISM  ({minutes} min per run, SHA-256 over every tick's snapshots)")
    other = seed + 1
    quiet = SpacecraftConfig(attitude=AttitudeConfig(disturbance_sd_dps_per_s=0.0))
    payload_off = SpacecraftControls()

    def trace(
        run_seed: int, config: SpacecraftConfig, controls: SpacecraftControls
    ) -> list[Sample]:
        run = new_run(run_seed, tick_us, config=config)
        return run.advance(ticks_for(minutes * US_PER_MIN, tick_us), lambda _t: controls)

    cases = (
        ("A nominal, payload on", NOMINAL_CONFIG, PAYLOAD_ON),
        ("B no disturbance, payload off", quiet, payload_off),
        ("C no disturbance, payload on", quiet, PAYLOAD_ON),
    )
    out.write("  case                              seed  truth digest      readings digest\n")
    traces: dict[tuple[str, int], list[list[Sample]]] = {}
    for label, config, controls in cases:
        for run_seed in (seed, seed, other):
            samples = trace(run_seed, config, controls)
            traces.setdefault((label, run_seed), []).append(samples)
            out.write(
                f"  {label:32s} {run_seed:5d}"
                f"  {digest(samples, 'truth')}  {digest(samples, 'readings')}\n"
            )

    def same(label: str, part: str, s1: int, s2: int, i: int = 0, j: int = 0) -> bool:
        return digest(traces[label, s1][i], part) == digest(traces[label, s2][j], part)

    a, b, c = (label for label, _, _ in cases)
    repeat_ok = all(
        same(label, part, seed, seed, 0, 1) for label in (a, b, c) for part in ("truth", "readings")
    )
    out.write(f"\n  same seed twice -> identical truth and readings in every case: {repeat_ok}\n")
    out.write(
        f"  A, seed {seed} vs {other}: truth identical={same(a, 'truth', seed, other)}."
        " The attitude disturbance is seeded *true* dynamics\n"
        "      (stream spacecraft.attitude.disturbance), so a new seed is a new trajectory.\n"
    )
    out.write(
        f"  B, seed {seed} vs {other}: truth identical={same(b, 'truth', seed, other)},"
        f" readings identical={same(b, 'readings', seed, other)}"
        " -> only the sensor noise changed.\n"
    )
    split = first_divergence(traces[c, seed][0], traces[c, other][0])
    if split is not None:
        s, names = split
        out.write(
            f"  C, seed {seed} vs {other}: truth splits at {s.minutes:.2f} min"
            f" in {', '.join(names)}:"
            " the payload inhibit reads the *reported*\n"
            "      attitude state (ADR-0004 §6), so noise moves when acquisition starts and the"
            " payload draw feeds back into power.\n"
        )
    else:
        out.write(f"  C, seed {seed} vs {other}: truth identical\n")


# --- Scenario 5: cold case -------------------------------------------------------------


def scenario_cold(out: TextIO, seed: int, tick_us: int, ambient_c: float = -60.0) -> None:
    heading(out, f"5. COLD CASE  (permanent eclipse, ambient {ambient_c:g} C)")
    env = NominalEnvironment(eclipse_fraction=1, eclipse_ambient_temp_c=ambient_c)
    initial = SpacecraftInitialState(
        thermal=ThermalInitial(battery_c=10.0, electronics_c=10.0), attitude=STABLE_START
    )
    run = new_run(seed, tick_us, initial=initial, env_model=env)
    samples = run.advance(ticks_for(60 * US_PER_MIN, tick_us), lambda _t: PAYLOAD_ON)
    out.write(
        "Survival heater: ON below 0 C, OFF above 4 C (3 W). At this ambient it cannot reach 4 C,\n"
        "so once on it never switches off. Starts at 10 C.\n\n"
    )
    write_timeline(out, every(samples, 5 * US_PER_MIN))
    first_on = next((i for i, s in enumerate(samples) if s.thermal.truth.heater_on), None)
    if first_on is not None:
        rest = samples[first_on:]
        out.write(
            f"\n  heater first on at {samples[first_on].minutes:.1f} min; on for"
            f" {sum(s.thermal.truth.heater_on for s in rest)}/{len(rest)} ticks after that"
            f" (cycles: {heater_cycles(rest, False)})\n"
        )
    out.write(f"  flags raised: {', '.join(raised_flags(samples))}\n")


# --- Entry point -----------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0] if __doc__ else None)
    parser.add_argument("--scenario", choices=("all", *SCENARIOS), default="all")
    parser.add_argument("--seed", type=int, default=42, help="master seed (default 42)")
    parser.add_argument(
        "--orbits", type=Fraction, default=Fraction(1), help="orbits for 'nominal' (default 1)"
    )
    parser.add_argument(
        "--tick-ms", type=int, default=1000, help="simulation tick, milliseconds (default 1000)"
    )
    args = parser.parse_args(argv)
    if args.orbits <= 0:
        parser.error("--orbits must be positive")
    if args.tick_ms <= 0:
        parser.error("--tick-ms must be positive")
    return args


def main(argv: Sequence[str] | None = None, out: TextIO | None = None) -> int:
    """Run the selected scenarios and print the report. Returns the exit code."""
    args = parse_args(argv)
    out = sys.stdout if out is None else out
    tick_us = args.tick_ms * 1000
    started = time.perf_counter()  # display only; the simulation never reads it
    out.write(
        f"PocketSat epic #30 demo: spacecraft subsystem models (seed {args.seed},"
        f" tick {args.tick_ms} ms)\n"
        "Real subsystems: power, thermal, attitude, payload on a shared SnapshotBoard.\n"
        f"{COMMS_STAND_IN_NOTE}\n"
    )
    chosen = SCENARIOS if args.scenario == "all" else (args.scenario,)
    for name in chosen:
        if name == "nominal":
            scenario_nominal(out, args.seed, tick_us, args.orbits)
        elif name == "detumble":
            scenario_detumble(out, args.seed, tick_us)
        elif name == "faults":
            scenario_faults(out, args.seed, tick_us)
        elif name == "determinism":
            scenario_determinism(out, args.seed, tick_us)
        else:
            scenario_cold(out, args.seed, tick_us)
    out.write(
        f"\nDone: {len(chosen)} scenario(s) in {time.perf_counter() - started:.2f} s wall time.\n"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
