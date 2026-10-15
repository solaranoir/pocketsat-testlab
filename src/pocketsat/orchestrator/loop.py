"""The run loop: environment, geometry, ground station, link and target, tick by tick.

ADR-0010 §7 places the Phase 2 run loop here, as the first piece of the orchestrator.
It runs ``docs/architecture.md`` §8's ten steps in order, once per tick, with time kept
by its own :class:`~pocketsat.core.clock.SimClock` (ADR-0003):

1. scheduled faults: a no-op until Phase 4;
2. sample the environment model and the pass geometry at the tick's start;
3. issue the scheduled commands that are due to the station (:meth:`GroundStation.submit
   <pocketsat.groundstation.GroundStation.submit>`), then
   :meth:`~pocketsat.groundstation.GroundStation.begin_tick`;
4. carry the uplink packets over the link and ``send()`` each frame the link delivers;
5. ``apply_environment(env)``;
6. ``advance(tick_us)``, exactly one tick;
7. ``receive()`` the downlink frames;
8. carry them over the link and give the packets the link delivers to
   :meth:`~pocketsat.groundstation.GroundStation.end_tick`;
9. collect the decoded telemetry (the station's telemetry store) and the events;
10. assertions: a no-op until Phase 5 (callers evaluate their own over the result).

The one pass geometry record of a tick goes to both the link and the station
(ADR-0010 §6). The loop depends on :class:`~pocketsat.targets.base.TestTarget` only
(AGENTS.md rule 1), so the same run drives ``SilTarget``, ``EchoTarget`` or a HIL
target. One master seed seeds everything: the target gets it in ``reset(seed)``, and the
station and link get an :class:`~pocketsat.core.rng.RngFactory` built from it, from which
they request named streams (``groundstation.*``, ``link.*``).

Phase 5 adds scenario loading, the fault schedule and assertions to this loop rather
than replacing it.
"""

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from types import TracebackType
from typing import Final, NamedTuple, Self

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock, check_us
from pocketsat.core.rng import RngFactory
from pocketsat.environment.geometry import PassGeometry
from pocketsat.environment.nominal import EnvironmentModel, NominalEnvironment
from pocketsat.environment.orbit import (
    DEFAULT_ORBIT,
    GeometryModel,
    Orbit,
    OrbitGeometry,
    StationLocation,
)
from pocketsat.groundstation import (
    DataStore,
    GroundStation,
    StationConfig,
    StationEvent,
    TelemetryStore,
)
from pocketsat.groundstation.events import CommandRef
from pocketsat.link import IdealLink, Link
from pocketsat.messages import Command
from pocketsat.targets.base import EnvironmentState, TestTarget


@dataclass(frozen=True)
class ScheduledCommand:
    """A command the loop issues to the station at a simulated time (step 3).

    Attributes:
        at_us: Simulated time from which the command is due. It is issued at the start
            of the first tick that starts at or after ``at_us``, so a command scheduled
            at a tick boundary goes up in the tick that starts then.
        command: The command.

    Raises:
        TypeError: ``at_us`` is not an int, or ``command`` is not a :class:`Command`.
        ValueError: ``at_us`` is negative.
    """

    at_us: int
    command: Command

    def __post_init__(self) -> None:
        check_us("at_us", self.at_us)
        if not isinstance(self.command, Command):
            raise TypeError(f"command must be a Command, got {self.command!r}")


@dataclass(frozen=True)
class IssuedCommand:
    """A scheduled command, issued to the station, and the reference it was given.

    Attributes:
        scheduled: What was scheduled.
        command_ref: The station's reference, which names the command in every later
            event (``CommandSent``, ``CommandAcked``, ...).
        issued_us: The start of the tick in which it was issued.
    """

    scheduled: ScheduledCommand
    command_ref: CommandRef
    issued_us: int


class TickResult(NamedTuple):
    """What one tick of the loop did.

    A named tuple rather than a frozen dataclass, the other records' form: it is built
    every tick, and a named tuple is built in about half the time (#167's performance
    figure, ``docs/spacecraft.md``). Immutable all the same.

    Attributes:
        tick_id: The tick's number since the start of the run, from 0; it runs from
            ``start_us`` to ``end_us``.
        start_us: Simulated time at the start of the tick.
        end_us: Simulated time at the end of the tick.
        environment: The environment applied to the target (step 2, step 5).
        geometry: The tick's pass geometry, or ``None`` when no geometry model runs.
        issued: The scheduled commands issued to the station this tick (step 3).
        uplink: The frames the link delivered and the loop sent to the target (step 4).
        downlink: The raw frames ``receive()`` returned (step 7), before the link.
        events: The station's events: those of ``begin_tick`` then those of
            ``end_tick`` (step 9).
    """

    tick_id: int
    start_us: int
    end_us: int
    environment: EnvironmentState
    geometry: PassGeometry | None
    issued: tuple[IssuedCommand, ...]
    uplink: tuple[bytes, ...]
    downlink: tuple[bytes, ...]
    events: tuple[StationEvent, ...]


@dataclass(frozen=True)
class RunResult:
    """The result of :meth:`RunLoop.run`: the events and the stores (ADR-0010 §4).

    The stores are the station's own, read-only to callers, and keep filling if the
    loop runs on.

    Attributes:
        master_seed: The run's master seed.
        tick_us: The tick length.
        tick_count: Ticks run since the start of the run.
        end_us: Simulated time at the end of the last tick.
        events: Every station event since the start of the run, in order.
        issued: Every scheduled command issued so far, in order.
        telemetry: The station's decoded telemetry (#157).
        data: The station's reassembled payload data (#158).
    """

    master_seed: int
    tick_us: int
    tick_count: int
    end_us: int
    events: tuple[StationEvent, ...]
    issued: tuple[IssuedCommand, ...]
    telemetry: TelemetryStore
    data: DataStore


def station_location(config: StationConfig) -> StationLocation:
    """The geometry model's :class:`StationLocation` for a station configuration.

    ``pocketsat.environment`` does not import the ground station (ADR-0011), so the
    orchestrator builds the location from the station's configuration.
    """
    return StationLocation(
        latitude_deg=config.latitude_deg,
        longitude_deg=config.longitude_deg,
        altitude_km=config.altitude_km,
    )


def station_geometry(config: StationConfig, orbit: Orbit = DEFAULT_ORBIT) -> OrbitGeometry:
    """An :class:`OrbitGeometry` of ``orbit`` seen from the station ``config`` describes."""
    return OrbitGeometry(orbit, station_location(config))


_NOT_STARTED: Final = "the run loop has not been started: call start() or use it in a with block"


class RunLoop:
    """Runs a ground station, a link and a target together, one tick at a time.

    Single use: build it, :meth:`schedule` commands (before or during the run),
    :meth:`start` it (connect and reset everything with the seed), then :meth:`step` or
    :meth:`run`, and :meth:`close` it. As a context manager it starts on entry and
    closes on exit::

        with RunLoop(SilTarget(), seed=42, geometry=station_geometry(config)) as loop:
            loop.schedule(6_000_000, Command.set_mode(Mode.SCIENCE))
            result = loop.run(until_us=600_000_000)

    Args:
        target: The spacecraft under test, driven through ``TestTarget`` only. The loop
            connects it, resets it with ``seed``, and closes it.
        seed: The run's master seed (ADR-0003).
        tick_us: The tick length, a multiple of the target's tick (default 100 ms).
        station: The ground station (default ``GroundStation()``). It is reset at start.
        link: The link (default :class:`~pocketsat.link.IdealLink`). It is reset at start.
        environment: The environment model sampled at step 2 (default
            ``NominalEnvironment()``).
        geometry: The pass geometry model sampled at step 2, or ``None`` (the default)
            for no geometry: the link then treats the spacecraft as always visible and
            the station is never in a pass. :func:`station_geometry` builds one for the
            station's location.
        commands: Commands to schedule, as :meth:`schedule`.

    Raises:
        TypeError: An argument has the wrong type.
        ValueError: ``tick_us`` is not positive, the station's ``command_timeout_us``
            is too short for it (:meth:`StationConfig.check_tick
            <pocketsat.groundstation.StationConfig.check_tick>`), or an
            :class:`OrbitGeometry` and a :class:`NominalEnvironment` disagree on the
            orbit period, or the geometry's station is not the station's location.
    """

    def __init__(
        self,
        target: TestTarget,
        *,
        seed: int,
        tick_us: int = DEFAULT_TICK_US,
        station: GroundStation | None = None,
        link: Link | None = None,
        environment: EnvironmentModel | None = None,
        geometry: GeometryModel | None = None,
        commands: Iterable[ScheduledCommand] = (),
    ) -> None:
        if not isinstance(target, TestTarget):
            raise TypeError(f"target must be a TestTarget, got {target!r}")
        station = GroundStation() if station is None else station
        if not isinstance(station, GroundStation):
            raise TypeError(f"station must be a GroundStation, got {station!r}")
        link = IdealLink() if link is None else link
        if not isinstance(link, Link):
            raise TypeError(f"link must be a Link, got {link!r}")
        environment = NominalEnvironment() if environment is None else environment
        if not isinstance(environment, EnvironmentModel):
            raise TypeError(f"environment must be an EnvironmentModel, got {environment!r}")
        if geometry is not None and not isinstance(geometry, GeometryModel):
            raise TypeError(f"geometry must be a GeometryModel or None, got {geometry!r}")
        self._rngs = RngFactory(seed)  # validates the seed
        self._clock = SimClock(tick_us)  # validates the tick
        station.config.check_tick(tick_us)
        _check_geometry(geometry, environment, station.config)

        self._target = target
        self._seed = seed
        self._station = station
        self._link = link
        self._environment = environment
        self._geometry = geometry
        self._schedule: list[ScheduledCommand] = []
        self._events: list[StationEvent] = []
        self._issued: list[IssuedCommand] = []
        self._started = False
        self._closed = False
        for scheduled in commands:
            if not isinstance(scheduled, ScheduledCommand):
                raise TypeError(f"commands must be ScheduledCommands, got {scheduled!r}")
            self._add(scheduled)

    # --- Read-only views --------------------------------------------------------------

    @property
    def master_seed(self) -> int:
        """The run's master seed."""
        return self._seed

    @property
    def rngs(self) -> RngFactory:
        """The run's random streams, given to the station and the link at start."""
        return self._rngs

    @property
    def tick_us(self) -> int:
        """The tick length."""
        return self._clock.tick_us

    @property
    def now_us(self) -> int:
        """Simulated time: the start of the next tick, the end of the last one."""
        return self._clock.now_us

    @property
    def tick_count(self) -> int:
        """Ticks run since the start of the run."""
        return self._clock.tick_index

    @property
    def station(self) -> GroundStation:
        """The ground station. Read its stores and statistics; :meth:`schedule`
        commands rather than submitting them directly, so they are issued at step 3."""
        return self._station

    @property
    def link(self) -> Link:
        """The link."""
        return self._link

    @property
    def events(self) -> tuple[StationEvent, ...]:
        """Every station event since the start of the run, in order."""
        return tuple(self._events)

    @property
    def issued(self) -> tuple[IssuedCommand, ...]:
        """Every scheduled command issued to the station so far, in order."""
        return tuple(self._issued)

    @property
    def pending(self) -> tuple[ScheduledCommand, ...]:
        """Scheduled commands not yet issued, in the order they will be."""
        return tuple(self._schedule)

    # --- Lifecycle --------------------------------------------------------------------

    def schedule(self, at_us: int, command: Command) -> ScheduledCommand:
        """Schedule ``command`` to be issued to the station at ``at_us`` (step 3).

        It is issued at the start of the first tick that starts at or after ``at_us``;
        a time already past is issued at the next tick. Commands due in the same tick
        are issued in time order, then in the order they were scheduled.

        Returns:
            The scheduled command; :attr:`issued` later pairs it with its reference.

        Raises:
            TypeError: ``at_us`` is not an int, or ``command`` is not a ``Command``.
            ValueError: ``at_us`` is negative.
        """
        scheduled = ScheduledCommand(at_us, command)
        self._add(scheduled)
        return scheduled

    def start(self) -> None:
        """Connect the target and reset the target, station and link with the seed.

        Raises:
            RuntimeError: The loop was already started.
        """
        if self._started:
            raise RuntimeError("the run loop is single use and was already started")
        self._started = True
        self._target.connect()
        self._target.reset(self._seed)
        self._station.reset(self._rngs)
        self._link.reset(self._rngs)

    def close(self) -> None:
        """Close the target. Idempotent; the loop cannot step afterwards."""
        if self._started and not self._closed:
            self._target.close()
        self._closed = True

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    # --- Running ----------------------------------------------------------------------

    def step(self) -> TickResult:
        """Run one tick, steps 1 to 10 (see the module docstring).

        Returns:
            What the tick did.

        Raises:
            RuntimeError: The loop is not started, or is closed.
        """
        if not self._started or self._closed:
            raise RuntimeError(_NOT_STARTED if not self._started else "the run loop is closed")
        clock, station, link, target = self._clock, self._station, self._link, self._target
        tick = clock.tick_index
        start_us = clock.now_us
        tick_us = clock.tick_us
        end_us = start_us + tick_us
        # 1. Scheduled faults: none until Phase 4.
        # 2. The world at the tick's start: one geometry record for link and station.
        environment = self._environment.state_at(start_us)
        geometry = None if self._geometry is None else self._geometry.geometry_at(start_us)
        # 3. Issue due commands, then start the station's tick.
        issued = self._issue(start_us) if self._schedule else ()
        uplink = station.begin_tick(start_us, geometry)
        # 4. Uplink over the link to the target.
        # The link is called every tick, packets or not: a link with latency (Phase 3)
        # releases frames queued in earlier ticks.
        sent = link.carry_uplink(uplink.packets, start_us, geometry)
        for frame in sent:
            target.send(frame)
        # 5, 6. The target's tick.
        target.apply_environment(environment)
        target.advance(tick_us)
        # 7, 8. Downlink from the target, over the link, to the station.
        raw = target.receive()
        delivered = link.carry_downlink(raw, end_us, geometry)
        # 9. Decoded telemetry (in the station's store) and events.
        ended = station.end_tick(end_us, delivered)
        events = uplink.events + ended
        self._events.extend(events)
        # 10. Assertions: none until Phase 5.
        clock.advance_one_tick()
        return TickResult(
            tick_id=tick,
            start_us=start_us,
            end_us=end_us,
            environment=environment,
            geometry=geometry,
            issued=issued,
            uplink=tuple(sent),
            downlink=tuple(raw),
            events=events,
        )

    def run(
        self, until_us: int, *, stop_when: Callable[[TickResult], bool] | None = None
    ) -> RunResult:
        """Run ticks until simulated time reaches ``until_us``, or ``stop_when`` is true.

        Args:
            until_us: Run every tick that starts before this time, so the run ends at the
                first tick boundary at or after it. A time already reached runs nothing.
            stop_when: Called after each tick with its result; the run stops after the
                first tick for which it returns true.

        Returns:
            The events and stores of the whole run so far (not only of this call).

        Raises:
            TypeError: ``until_us`` is not an int.
            ValueError: ``until_us`` is negative.
            RuntimeError: The loop is not started, or is closed.
        """
        check_us("until_us", until_us)
        clock = self._clock
        while clock.now_us < until_us:
            result = self.step()
            if stop_when is not None and stop_when(result):
                break
        return self.result()

    def result(self) -> RunResult:
        """The events and stores of the run so far."""
        return RunResult(
            master_seed=self._seed,
            tick_us=self._clock.tick_us,
            tick_count=self._clock.tick_index,
            end_us=self._clock.now_us,
            events=tuple(self._events),
            issued=tuple(self._issued),
            telemetry=self._station.telemetry,
            data=self._station.data,
        )

    # --- Internals --------------------------------------------------------------------

    def _add(self, scheduled: ScheduledCommand) -> None:
        """Insert in time order, after every command scheduled at the same time."""
        schedule = self._schedule
        index = len(schedule)
        while index > 0 and schedule[index - 1].at_us > scheduled.at_us:
            index -= 1
        schedule.insert(index, scheduled)

    def _issue(self, now_us: int) -> tuple[IssuedCommand, ...]:
        """Step 3: submit every scheduled command due by ``now_us`` to the station."""
        schedule = self._schedule
        count = 0
        while count < len(schedule) and schedule[count].at_us <= now_us:
            count += 1
        if count == 0:
            return ()
        due, self._schedule = schedule[:count], schedule[count:]
        issued = tuple(
            IssuedCommand(scheduled, self._station.submit(scheduled.command), now_us)
            for scheduled in due
        )
        self._issued.extend(issued)
        return issued


def _check_geometry(
    geometry: GeometryModel | None, environment: EnvironmentModel, config: StationConfig
) -> None:
    """The geometry and the environment describe one orbit, seen from this station.

    Eclipse stays with ``NominalEnvironment`` (ADR-0011 §4), so an
    :class:`OrbitGeometry` must have its orbit period; and its station must be where the
    station's configuration says it is.
    """
    if not isinstance(geometry, OrbitGeometry):
        return
    if (
        isinstance(environment, NominalEnvironment)
        and geometry.orbit_period_us != environment.orbit_period_us
    ):
        raise ValueError(
            f"the geometry's orbit period ({geometry.orbit_period_us} µs) must equal the "
            f"environment's ({environment.orbit_period_us} µs)"
        )
    location = station_location(config)
    if geometry.station != location:
        raise ValueError(
            f"the geometry's station {geometry.station} is not the station's location "
            f"{location}: build it with station_geometry(config)"
        )
