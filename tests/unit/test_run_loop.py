"""The run loop (#167, ADR-0010 §7): step order, scheduling, lifecycle, validation.

The step order is checked with spies on every collaborator (target, link, station,
environment and geometry models), against ``docs/architecture.md`` §8. The loop runs
against ``EchoTarget`` here; ``tests/sil/test_run_loop_sil.py`` runs it against
``SilTarget``.
"""

import ast
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path

import pytest

import pocketsat.orchestrator
from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment, Orbit, OrbitGeometry, StationLocation
from pocketsat.environment.geometry import PassGeometry
from pocketsat.groundstation import (
    DEFAULT_STATION_CONFIG,
    Aos,
    CommandRef,
    CommandSent,
    FrameRejected,
    GroundStation,
    StationConfig,
    StationEvent,
    Uplink,
)
from pocketsat.link import IdealLink
from pocketsat.messages import Command, CommandId, Packet
from pocketsat.orchestrator import (
    RunLoop,
    ScheduledCommand,
    TickResult,
    station_geometry,
    station_location,
)
from pocketsat.targets.base import EnvironmentState, TargetCapabilities, TargetFault
from pocketsat.targets.echo import EchoTarget

TICK_US = 100_000
SEED = 167
GEOMETRY = PassGeometry(
    azimuth_deg=90.0, elevation_deg=30.0, range_km=1000.0, range_rate_km_per_s=-1.0
)
DOWN_FRAME = b"not a frame"


# --- Spies ----------------------------------------------------------------------------


Log = list[tuple[object, ...]]


class SpyTarget:
    """A ``TestTarget`` that logs every call and sends one frame down per tick."""

    capabilities = TargetCapabilities(deterministic=True, real_time=False)

    def __init__(self, log: Log) -> None:
        self.log = log
        self.environments: list[EnvironmentState] = []

    def connect(self) -> None:
        self.log.append(("target.connect",))

    def reset(self, seed: int) -> None:
        self.log.append(("target.reset", seed))

    def send(self, frame: bytes) -> None:
        self.log.append(("target.send", frame))

    def receive(self) -> list[bytes]:
        self.log.append(("target.receive",))
        return [DOWN_FRAME]

    def apply_environment(self, env: EnvironmentState) -> None:
        self.log.append(("target.apply_environment",))
        self.environments.append(env)

    def inject(self, fault: TargetFault) -> None:
        self.log.append(("target.inject", fault))

    def advance(self, dt_us: int) -> None:
        self.log.append(("target.advance", dt_us))

    def close(self) -> None:
        self.log.append(("target.close",))


class SpyLink(IdealLink):
    """The ideal link, logging each call and the geometry it was given."""

    def __init__(self, log: Log) -> None:
        super().__init__()
        self.log = log
        self.geometries: list[PassGeometry | None] = []

    def reset(self, rngs: RngFactory) -> None:
        self.log.append(("link.reset", rngs.master_seed))
        super().reset(rngs)

    def carry_uplink(
        self, packets: Sequence[Packet], now_us: int, geometry: PassGeometry | None
    ) -> list[bytes]:
        self.log.append(("link.carry_uplink", len(packets), now_us))
        self.geometries.append(geometry)
        return super().carry_uplink(packets, now_us, geometry)

    def carry_downlink(
        self, frames: Sequence[bytes], now_us: int, geometry: PassGeometry | None
    ) -> list[Packet]:
        self.log.append(("link.carry_downlink", tuple(frames), now_us))
        self.geometries.append(geometry)
        return super().carry_downlink(frames, now_us, geometry)


class SpyStation(GroundStation):
    """The real station, logging each call and the geometry it was given."""

    def __init__(self, log: Log, config: StationConfig = DEFAULT_STATION_CONFIG) -> None:
        super().__init__(config)
        self.log = log
        self.geometries: list[PassGeometry | None] = []

    def reset(self, rngs: RngFactory) -> None:
        self.log.append(("station.reset", rngs.master_seed))
        super().reset(rngs)

    def submit(self, command: Command, *, due_us: int | None = None) -> CommandRef:
        self.log.append(("station.submit", command.command_id))
        return super().submit(command, due_us=due_us)

    def begin_tick(self, now_us: int, geometry: PassGeometry | None) -> Uplink:
        self.log.append(("station.begin_tick", now_us))
        self.geometries.append(geometry)
        return super().begin_tick(now_us, geometry)

    def end_tick(self, now_us: int, downlink: Sequence[Packet]) -> tuple[StationEvent, ...]:
        self.log.append(("station.end_tick", now_us, len(downlink)))
        return super().end_tick(now_us, downlink)


class SpyEnvironment:
    def __init__(self, log: Log) -> None:
        self.log = log
        self.model = NominalEnvironment()

    def state_at(self, now_us: int) -> EnvironmentState:
        self.log.append(("environment.state_at", now_us))
        return self.model.state_at(now_us)


class SpyGeometry:
    def __init__(self, log: Log) -> None:
        self.log = log

    def geometry_at(self, now_us: int) -> PassGeometry:
        self.log.append(("geometry.geometry_at", now_us))
        return replace(GEOMETRY, azimuth_deg=float(now_us // TICK_US))


class Spies:
    def __init__(self) -> None:
        self.log: Log = []
        self.target = SpyTarget(self.log)
        self.link = SpyLink(self.log)
        self.station = SpyStation(self.log)
        self.environment = SpyEnvironment(self.log)
        self.geometry = SpyGeometry(self.log)

    def loop(self) -> RunLoop:
        return RunLoop(
            self.target,
            seed=SEED,
            station=self.station,
            link=self.link,
            environment=self.environment,
            geometry=self.geometry,
        )


# --- Step order -----------------------------------------------------------------------


def test_start_connects_and_resets_everything_with_one_seed() -> None:
    spies = Spies()
    loop = spies.loop()
    assert spies.log == []  # nothing happens before start
    loop.start()
    assert spies.log == [
        ("target.connect",),
        ("target.reset", SEED),
        ("station.reset", SEED),
        ("link.reset", SEED),
    ]
    assert loop.rngs.master_seed == SEED


def test_each_tick_runs_the_architecture_steps_in_order() -> None:
    spies = Spies()
    loop = spies.loop()
    loop.schedule(TICK_US, Command.ping())
    with loop:
        spies.log.clear()
        loop.step()  # tick 0: nothing to issue
        (ping_frame,) = loop.step().uplink  # tick 1: the PING
    assert spies.log == [
        # Tick 0.
        ("environment.state_at", 0),  # step 2
        ("geometry.geometry_at", 0),  # step 2
        ("station.begin_tick", 0),  # step 3
        ("link.carry_uplink", 0, 0),  # step 4 (called every tick)
        ("target.apply_environment",),  # step 5
        ("target.advance", TICK_US),  # step 6
        ("target.receive",),  # step 7
        ("link.carry_downlink", (DOWN_FRAME,), TICK_US),  # step 8
        ("station.end_tick", TICK_US, 1),  # steps 8 and 9
        # Tick 1.
        ("environment.state_at", TICK_US),
        ("geometry.geometry_at", TICK_US),
        ("station.submit", CommandId.PING),  # step 3: the due command, then begin_tick
        ("station.begin_tick", TICK_US),
        ("link.carry_uplink", 1, TICK_US),  # step 4
        ("target.send", ping_frame),
        ("target.apply_environment",),
        ("target.advance", TICK_US),
        ("target.receive",),
        ("link.carry_downlink", (DOWN_FRAME,), 2 * TICK_US),
        ("station.end_tick", 2 * TICK_US, 1),
        # Close.
        ("target.close",),
    ]


def test_one_geometry_record_per_tick_goes_to_link_and_station() -> None:
    spies = Spies()
    with spies.loop() as loop:
        results = [loop.step() for _ in range(3)]
    for tick, result in enumerate(results):
        assert result.geometry is not None
        assert result.geometry.azimuth_deg == tick
        assert spies.station.geometries[tick] is result.geometry
        assert spies.link.geometries[2 * tick] is result.geometry  # uplink
        assert spies.link.geometries[2 * tick + 1] is result.geometry  # downlink


def test_the_sampled_environment_is_applied() -> None:
    spies = Spies()
    with spies.loop() as loop:
        results = [loop.step() for _ in range(2)]
    assert spies.target.environments == [r.environment for r in results]


def test_the_tick_result_records_the_tick() -> None:
    spies = Spies()
    loop = spies.loop()
    scheduled = loop.schedule(0, Command.ping())
    with loop:
        result = loop.step()
    assert (result.tick_id, result.start_us, result.end_us) == (0, 0, TICK_US)
    (issued,) = result.issued
    assert issued.scheduled == scheduled
    assert issued.issued_us == 0
    (frame,) = result.uplink
    assert ("target.send", frame) in spies.log
    assert result.downlink == (DOWN_FRAME,)
    aos, sent, rejected = result.events  # begin_tick's events, then end_tick's
    assert isinstance(aos, Aos) and aos.elevation_deg == GEOMETRY.elevation_deg
    assert isinstance(sent, CommandSent) and sent.command_ref == issued.command_ref
    assert isinstance(rejected, FrameRejected) and rejected.frame == DOWN_FRAME
    assert loop.events == result.events
    assert loop.issued == (issued,)


# --- Scheduling -----------------------------------------------------------------------


def _issued_ticks(loop: RunLoop, ticks: int) -> list[tuple[int, tuple[int, ...]]]:
    with loop:
        results = [loop.step() for _ in range(ticks)]
    return [
        (r.tick_id, tuple(i.scheduled.command.command_id for i in r.issued))
        for r in results
        if r.issued
    ]


def test_commands_are_issued_at_the_first_tick_starting_at_or_after_their_time() -> None:
    loop = RunLoop(EchoTarget(), seed=SEED)
    loop.schedule(2 * TICK_US, Command.ping())  # on a boundary: tick 2
    loop.schedule(2 * TICK_US + 1, Command.reset())  # just after: tick 3
    loop.schedule(TICK_US - 1, Command.begin_downlink())  # just before: tick 1
    assert _issued_ticks(loop, 4) == [
        (1, (CommandId.BEGIN_DOWNLINK,)),
        (2, (CommandId.PING,)),
        (3, (CommandId.RESET,)),
    ]


def test_commands_due_together_go_in_time_then_schedule_order() -> None:
    loop = RunLoop(
        EchoTarget(),
        seed=SEED,
        commands=[
            ScheduledCommand(50_000, Command.ping()),
            ScheduledCommand(10_000, Command.reset()),
        ],
    )
    loop.schedule(50_000, Command.begin_downlink())
    loop.schedule(10_000, Command.enter_safe_mode())
    assert [s.command.command_id for s in loop.pending] == [
        CommandId.RESET,
        CommandId.ENTER_SAFE_MODE,
        CommandId.PING,
        CommandId.BEGIN_DOWNLINK,
    ]
    assert _issued_ticks(loop, 2) == [
        (
            1,
            (
                CommandId.RESET,
                CommandId.ENTER_SAFE_MODE,
                CommandId.PING,
                CommandId.BEGIN_DOWNLINK,
            ),
        )
    ]
    assert loop.pending == ()


def test_a_command_scheduled_in_the_past_goes_at_the_next_tick() -> None:
    with RunLoop(EchoTarget(), seed=SEED) as loop:
        loop.run(5 * TICK_US)
        loop.schedule(0, Command.ping())
        result = loop.step()
    assert result.tick_id == 5
    assert [i.issued_us for i in result.issued] == [5 * TICK_US]


def test_issued_commands_carry_the_station_reference() -> None:
    loop = RunLoop(EchoTarget(), seed=SEED)
    for tick in range(3):
        loop.schedule(tick * TICK_US, Command.ping())
    with loop:
        result = loop.run(3 * TICK_US)
    sent = [e.command_ref for e in result.events if isinstance(e, CommandSent)]
    assert [i.command_ref for i in result.issued] == sent
    assert len(set(sent)) == 3


# --- Running --------------------------------------------------------------------------


def test_run_stops_at_the_first_tick_boundary_at_or_after_until() -> None:
    with RunLoop(EchoTarget(), seed=SEED) as loop:
        assert loop.run(0).tick_count == 0
        result = loop.run(TICK_US + 1)
        assert (result.tick_count, result.end_us) == (2, 2 * TICK_US)
        assert loop.run(2 * TICK_US).tick_count == 2  # already there
        assert loop.run(5 * TICK_US).tick_count == 5


def test_run_stops_when_asked() -> None:
    seen: list[TickResult] = []

    def stop(result: TickResult) -> bool:
        seen.append(result)
        return result.tick_id == 3

    with RunLoop(EchoTarget(), seed=SEED) as loop:
        result = loop.run(100 * TICK_US, stop_when=stop)
    assert result.tick_count == 4
    assert [r.tick_id for r in seen] == [0, 1, 2, 3]


def test_the_result_holds_the_events_and_the_stores() -> None:
    loop = RunLoop(EchoTarget(), seed=SEED)
    loop.schedule(0, Command.ping())
    with loop:
        events: list[StationEvent] = []
        for _ in range(3):
            events.extend(loop.step().events)
        result = loop.result()
    assert result.events == tuple(events) == loop.events
    assert (result.master_seed, result.tick_us, result.tick_count) == (SEED, TICK_US, 3)
    assert result.telemetry is loop.station.telemetry
    assert result.data is loop.station.data
    # EchoTarget returns the COMMAND frame: rejected as travelling the wrong way.
    assert any(isinstance(e, FrameRejected) for e in result.events)


def test_the_loop_is_deterministic() -> None:
    def run() -> list[TickResult]:
        loop = RunLoop(EchoTarget(), seed=SEED, geometry=station_geometry(DEFAULT_STATION_CONFIG))
        for tick in range(0, 50, 7):
            loop.schedule(tick * TICK_US, Command.ping())
        with loop:
            return [loop.step() for _ in range(50)]

    assert run() == run()


# --- Lifecycle ------------------------------------------------------------------------


def test_step_needs_a_started_open_loop() -> None:
    loop = RunLoop(EchoTarget(), seed=SEED)
    with pytest.raises(RuntimeError, match="not been started"):
        loop.step()
    loop.start()
    with pytest.raises(RuntimeError, match="single use"):
        loop.start()
    loop.close()
    loop.close()  # idempotent
    with pytest.raises(RuntimeError, match="closed"):
        loop.step()


def test_the_with_block_closes_the_target_on_error() -> None:
    spies = Spies()
    with pytest.raises(ValueError, match="boom"), spies.loop():
        raise ValueError("boom")
    assert spies.log[-1] == ("target.close",)


def test_closing_an_unstarted_loop_does_not_touch_the_target() -> None:
    spies = Spies()
    spies.loop().close()
    assert spies.log == []


# --- Validation -----------------------------------------------------------------------


@pytest.mark.parametrize(
    ("kwargs", "error"),
    [
        ({"seed": "1"}, TypeError),
        ({"tick_us": 0}, ValueError),
        ({"station": object()}, TypeError),
        ({"link": object()}, TypeError),
        ({"environment": object()}, TypeError),
        ({"geometry": object()}, TypeError),
        ({"commands": [object()]}, TypeError),
    ],
)
def test_bad_arguments_are_rejected(kwargs: dict[str, object], error: type[Exception]) -> None:
    arguments: dict[str, object] = {"seed": SEED, **kwargs}
    with pytest.raises(error):
        RunLoop(EchoTarget(), **arguments)  # type: ignore[arg-type]


def test_the_target_must_be_a_test_target() -> None:
    with pytest.raises(TypeError, match="TestTarget"):
        RunLoop(object(), seed=SEED)  # type: ignore[arg-type]


def test_the_station_timeout_is_checked_against_the_tick() -> None:
    # At a 5 s tick a command can be answered 5 s after it is sent: longer than the
    # default 2 s timeout (StationConfig.check_tick, #155).
    with pytest.raises(ValueError, match="command_timeout_us"):
        RunLoop(EchoTarget(), seed=SEED, tick_us=5_000_000)


def test_geometry_and_environment_must_share_the_orbit_period() -> None:
    geometry = station_geometry(DEFAULT_STATION_CONFIG, Orbit(orbit_period_us=6_000_000_000))
    with pytest.raises(ValueError, match="orbit period"):
        RunLoop(EchoTarget(), seed=SEED, geometry=geometry)
    RunLoop(
        EchoTarget(),
        seed=SEED,
        geometry=geometry,
        environment=NominalEnvironment(orbit_period_us=6_000_000_000),
    )


def test_the_geometry_must_be_seen_from_the_station() -> None:
    elsewhere = OrbitGeometry(station=StationLocation(latitude_deg=50.0))
    with pytest.raises(ValueError, match="station_geometry"):
        RunLoop(EchoTarget(), seed=SEED, geometry=elsewhere)
    config = replace(DEFAULT_STATION_CONFIG, latitude_deg=50.0)
    RunLoop(EchoTarget(), seed=SEED, station=GroundStation(config), geometry=elsewhere)


def test_scheduled_commands_are_validated() -> None:
    loop = RunLoop(EchoTarget(), seed=SEED)
    with pytest.raises(ValueError):
        loop.schedule(-1, Command.ping())
    with pytest.raises(TypeError):
        loop.schedule(0, "PING")  # type: ignore[arg-type]
    with pytest.raises(TypeError):
        loop.run(1.5)  # type: ignore[arg-type]


def test_station_location_follows_the_station_config() -> None:
    config = replace(DEFAULT_STATION_CONFIG, latitude_deg=-33.9, longitude_deg=18.4)
    assert station_location(config) == StationLocation(-33.9, 18.4, config.altitude_km)
    assert station_geometry(config).station == station_location(config)
    assert station_location(DEFAULT_STATION_CONFIG) == StationLocation()


# --- Architecture ---------------------------------------------------------------------


def test_the_orchestrator_imports_no_concrete_target() -> None:
    # AGENTS.md rule 1: the orchestrator depends on pocketsat.targets.base only.
    package = Path(pocketsat.orchestrator.__file__).parent
    imported: set[str] = set()
    for path in package.rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module)
                imported.update(f"{node.module}.{alias.name}" for alias in node.names)
    targets = {name for name in imported if name.startswith("pocketsat.targets")}
    assert targets <= {
        "pocketsat.targets.base",
        "pocketsat.targets.base.EnvironmentState",
        "pocketsat.targets.base.TestTarget",
    }, targets
