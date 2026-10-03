"""The software-in-the-loop target: the Python spacecraft behind ``TestTarget`` (#59).

:class:`SilTarget` wires the five real subsystems (power, thermal, attitude, payload,
comms) on one shared ``SnapshotBoard`` and the :class:`~pocketsat.flight.FlightComputer`
behind the :class:`~pocketsat.targets.base.TestTarget` protocol. The orchestrator sees
only bytes in and bytes out (ADR-0002).

**One tick** of ``advance()`` runs ADR-0004 §2 steps a to f, in this order:

a. :meth:`SilTarget._merge_controls`: the flight computer's controls from the previous
   tick (BOOT's controls on tick 0), merged with fault overrides. No faults exist yet,
   so it passes the controls through unchanged; #60 adds the overrides and #98 the
   radio traffic (ADR-0007), both in that one function.
b. The subsystems step in ``STEP_ORDER`` with the merged controls, publishing each
   snapshot to the board.
c. to f. The uplink frames sent since the last tick are delivered to the flight
   computer **only if comms' receiver is on**, judged from comms' true state after
   step b (ADR-0004 §10). Otherwise they are lost: dropped, not queued, and counted.
   Delivered frames that are not valid wire frames are dropped and counted too, and
   never raise. Then :meth:`FlightComputer.step` runs once with the delivered frames
   and :meth:`SpacecraftReadings.from_state` of the stack's state (readings only,
   ADR-0004 §6), and returns the downlink frames, the next tick's controls, and its
   suppressed-frame count.

Afterwards the downlink frames are queued for :meth:`SilTarget.receive`, the next
tick's controls are kept for step a, and the tick's radio traffic is kept as a
:class:`TickTraffic` for the hand-over to the next tick (ADR-0007, see below). Every
tick is described by a :class:`SilTick` record: the controls the subsystems obeyed,
next to the :class:`~pocketsat.spacecraft.SpacecraftState` they produced (ADR-0004
§14).

**Time.** Simulated time is integer microseconds on the target's own ``SimClock``
(ADR-0003), which starts at 0 on ``reset(seed)``. ``advance(dt_us)`` runs
``dt_us // tick_us`` whole ticks; ``dt_us`` must be a multiple of the tick, because
ADR-0003 quantizes every scheduled event to a tick boundary before a run, so a partial
tick is a caller error and raises ``ValueError`` instead of being rounded. The flight
computer is called with the time at the **end** of the tick, the instant the readings
it receives describe.

**Radio traffic and #98 (ADR-0007).** ADR-0007 routes each tick's traffic to comms as
a ``RadioTraffic`` record in ``SpacecraftControls.radio_traffic``, filled at step a of
the next tick. That record, the controls field, and comms' use of it are #98's. This
module computes the three per-tick values and holds them in :class:`TickTraffic` (the
same fields as ``RadioTraffic``), so #98 only has to merge them into the controls in
:meth:`SilTarget._merge_controls`. No running totals are kept here: comms will own them
(ADR-0007 §3).

Deterministic: same configuration, starting state, seed, and inputs give identical
downlink bytes. No wall-clock time and no global randomness; every random stream
comes from the run's ``RngFactory`` (ADR-0003).
"""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Final

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock, check_us
from pocketsat.core.rng import RngFactory
from pocketsat.flight.computer import FlightComputer, SpacecraftReadings
from pocketsat.frame import FrameError, decode_frame
from pocketsat.spacecraft.attitude import Attitude
from pocketsat.spacecraft.base import SnapshotBoard, SpacecraftState, Subsystem, SubsystemStack
from pocketsat.spacecraft.comms import Comms
from pocketsat.spacecraft.config import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    SpacecraftConfig,
    SpacecraftInitialState,
)
from pocketsat.spacecraft.controls import SpacecraftControls
from pocketsat.spacecraft.payload import Payload
from pocketsat.spacecraft.power import Power
from pocketsat.spacecraft.snapshots import CommsSnapshot
from pocketsat.spacecraft.thermal import Thermal
from pocketsat.targets.base import (
    EnvironmentState,
    TargetCapabilities,
    TargetFault,
    UnsupportedFaultError,
)

__all__ = ["SIL_CAPABILITIES", "SilTarget", "SilTick", "TickObserver", "TickTraffic"]

SIL_CAPABILITIES: Final = TargetCapabilities(
    deterministic=True,
    real_time=False,
    supported_faults=frozenset(),
)
"""What :class:`SilTarget` declares. No target faults yet: #60 adds ``forced_reset``,
``sensor_freeze``, ``transmitter_off``, and ``battery_drain``."""


@dataclass(frozen=True, slots=True)
class TickTraffic:
    """One tick's radio traffic, held by ``SilTarget`` for the hand-over to the next
    tick (ADR-0007 §1, §3).

    Internal to ``SilTarget``: it never crosses ``TestTarget``. The fields are exactly
    those of ADR-0007's ``RadioTraffic``, which #98 adds to ``SpacecraftControls``; #98
    builds that record from this one in :meth:`SilTarget._merge_controls`. All values
    are per tick, never running totals. Built only by ``SilTarget``, so not validated.

    Attributes:
        sent_bytes: Wire bytes of every frame the flight computer sent in the tick
            (``FlightComputerOutput.sent_bytes``: header, payload, and CRC, all types).
        uplink_lost_count: Uplink frames dropped in the tick because comms' receiver
            was off after the subsystem step.
        outbound_suppressed_count: ACK/NACK and telemetry frames the flight computer
            suppressed in the tick (``FlightComputerOutput.outbound_suppressed_count``).
    """

    sent_bytes: int = 0
    uplink_lost_count: int = 0
    outbound_suppressed_count: int = 0


@dataclass(frozen=True, slots=True)
class SilTick:
    """Everything one tick of :meth:`SilTarget.advance` did (ADR-0004 §14).

    The controls are recorded alongside the state they produced, so a run can be
    inspected and replayed. Internal to the SIL target: SIL tests and (later) the run
    record read it through :attr:`SilTarget.last_tick` or a tick observer; the
    orchestrator never does. Built only by ``SilTarget``, so not validated.

    Attributes:
        now_us: Simulated time at the end of the tick, integer microseconds; the time
            passed to the flight computer.
        environment: The environment the subsystems stepped under.
        controls: The merged controls the subsystems obeyed (step a): the flight
            computer's controls from the previous tick, or BOOT's on tick 0, with the
            fault overrides.
        state: Every subsystem's snapshot after the subsystem step (step b), truth and
            readings.
        delivered_uplink: The uplink frames passed to the flight computer, in arrival
            order.
        undecodable_uplink_count: Frames received while the receiver was on that were
            not valid wire frames (bad sync, length, CRC, or type); dropped and never
            passed on. Not part of ``uplink_lost_count`` (ADR-0007 §3).
        traffic: The tick's radio traffic, handed to comms in the next tick (#98).
        downlink_frames: The frames the flight computer sent, in transmit order; the
            same frames :meth:`SilTarget.receive` returns.
        next_controls: The controls the flight computer produced for the next tick
            (step f), before any merge.
    """

    now_us: int
    environment: EnvironmentState
    controls: SpacecraftControls
    state: SpacecraftState
    delivered_uplink: tuple[bytes, ...]
    undecodable_uplink_count: int
    traffic: TickTraffic
    downlink_frames: tuple[bytes, ...]
    next_controls: SpacecraftControls


TickObserver = Callable[[SilTick], None]
"""Called with each tick's :class:`SilTick` at the end of the tick, for recording."""


def _build_stack(config: SpacecraftConfig, initial: SpacecraftInitialState) -> SubsystemStack:
    """Build the five real subsystems on one shared ``SnapshotBoard`` (#85)."""
    board = SnapshotBoard()
    subsystems: tuple[Subsystem, ...] = (
        Power(config.power, initial.power, board),
        Thermal(config.thermal, initial.thermal, board),
        Attitude(config.attitude, initial.attitude),
        Payload(config.payload, initial.payload, board),
        Comms(config.comms),
    )
    return SubsystemStack(subsystems, board=board)


class SilTarget:
    """The Python spacecraft model as a ``TestTarget`` (ADR-0002, #59).

    Built with the spacecraft's settings and starting state (ADR-0005); ``reset(seed)``
    returns to exactly those, with the flight computer in BOOT. See the module
    docstring for what one tick does.

    Call :meth:`reset` before :meth:`advance`: the seed is never implicit.

    Attributes:
        capabilities: Deterministic, not real time, no target faults yet (#60).
    """

    capabilities: TargetCapabilities = SIL_CAPABILITIES

    def __init__(
        self,
        *,
        config: SpacecraftConfig = NOMINAL_CONFIG,
        initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
        tick_us: int = DEFAULT_TICK_US,
        flight_computer_factory: Callable[[], FlightComputer] = FlightComputer,
        tick_observer: TickObserver | None = None,
    ) -> None:
        """Create a target. It is not running until :meth:`reset` is called.

        Args:
            config: Spacecraft settings (default: the nominal set).
            initial: Starting state (default: the default starting state).
            tick_us: Tick length in integer microseconds (ADR-0003; default 100 ms).
                ``advance(dt_us)`` takes multiples of it.
            flight_computer_factory: Builds a fresh flight computer on every
                :meth:`reset`. The default is the real one; SIL tests pass a scripted
                subclass to exercise the wiring before the flight software fills in.
            tick_observer: Called with every tick's :class:`SilTick`, for recording.

        Raises:
            TypeError: ``config``, ``initial``, or ``tick_us`` has the wrong type.
            ValueError: ``tick_us`` is not positive.
        """
        if not isinstance(config, SpacecraftConfig):
            raise TypeError(f"config must be a SpacecraftConfig, got {config!r}")
        if not isinstance(initial, SpacecraftInitialState):
            raise TypeError(f"initial must be a SpacecraftInitialState, got {initial!r}")
        self._config = config
        self._initial = initial
        self._clock = SimClock(tick_us)  # validates tick_us
        self._flight_computer_factory = flight_computer_factory
        self._tick_observer = tick_observer
        self._stack: SubsystemStack | None = None
        self._flight_computer: FlightComputer | None = None
        self._environment = EnvironmentState()
        self._next_controls = SpacecraftControls()
        self._handover_traffic = TickTraffic()
        self._uplink: list[bytes] = []
        self._outbox: list[bytes] = []
        self._last_tick: SilTick | None = None

    # --- Read-only views -----------------------------------------------------------------

    @property
    def config(self) -> SpacecraftConfig:
        """The settings this target was built with."""
        return self._config

    @property
    def initial(self) -> SpacecraftInitialState:
        """The starting state ``reset(seed)`` returns to."""
        return self._initial

    @property
    def tick_us(self) -> int:
        """Tick length, integer microseconds."""
        return self._clock.tick_us

    @property
    def now_us(self) -> int:
        """Simulated time since the last ``reset(seed)``, integer microseconds."""
        return self._clock.now_us

    @property
    def last_tick(self) -> SilTick | None:
        """The record of the most recent tick, or ``None`` since the last reset."""
        return self._last_tick

    @property
    def handover_traffic(self) -> TickTraffic:
        """The radio traffic of the most recent tick, waiting for step a of the next.

        Zero after ``reset(seed)``, so tick 0 hands over nothing (ADR-0007 §3). #98
        merges it into ``SpacecraftControls.radio_traffic``.
        """
        return self._handover_traffic

    # --- TestTarget ------------------------------------------------------------------------

    def connect(self) -> None:
        """No-op: the target runs in-process."""

    def reset(self, seed: int) -> None:
        """Rebuild every subsystem and the flight computer, seeded from ``seed``.

        The subsystems are rebuilt from the target's ``config`` and ``initial``
        (ADR-0005) and reset with a new ``RngFactory(seed)``, so every named stream
        restarts. A new flight computer powers on in BOOT and its BOOT controls apply to
        tick 0 (ADR-0004 §2). Simulated time returns to 0; queued uplink, undrained
        downlink, the traffic hand-over, and the last tick record are discarded; the
        environment returns to the nominal ``EnvironmentState()`` until the next
        :meth:`apply_environment`.

        Raises:
            TypeError: ``seed`` is not an int.
        """
        rng = RngFactory(seed)  # validates the seed
        stack = _build_stack(self._config, self._initial)
        stack.reset(rng)
        flight_computer = self._flight_computer_factory()
        self._clock = SimClock(self._clock.tick_us)
        self._next_controls = flight_computer.reset(now_us=self._clock.now_us)
        self._stack = stack
        self._flight_computer = flight_computer
        self._environment = EnvironmentState()
        self._handover_traffic = TickTraffic()
        self._uplink.clear()
        self._outbox.clear()
        self._last_tick = None

    def send(self, frame: bytes) -> None:
        """Queue one uplink frame for the next tick.

        Whether it reaches the flight computer is decided in that tick: it is lost if
        comms' receiver is off, and dropped if it is not a valid frame. Neither raises.

        Raises:
            TypeError: ``frame`` is not bytes-like.
        """
        if not isinstance(frame, bytes | bytearray | memoryview):
            raise TypeError(f"frame must be bytes, got {type(frame).__name__}")
        self._uplink.append(bytes(frame))

    def receive(self) -> list[bytes]:
        """Return and clear the downlink frames produced since the last call, oldest
        first."""
        frames, self._outbox = self._outbox, []
        return frames

    def apply_environment(self, env: EnvironmentState) -> None:
        """Set the environment the subsystems step under from the next tick on.

        Raises:
            TypeError: ``env`` is not an :class:`EnvironmentState`.
        """
        if not isinstance(env, EnvironmentState):
            raise TypeError(f"env must be an EnvironmentState, got {type(env).__name__}")
        self._environment = env

    def inject(self, fault: TargetFault) -> None:
        """Apply a target fault. No fault types are supported yet (#60).

        Raises:
            UnsupportedFaultError: Always, until #60 adds the SIL faults.
        """
        if fault.fault_type not in self.capabilities.supported_faults:
            raise UnsupportedFaultError(fault.fault_type, self.capabilities.supported_faults)

    def advance(self, dt_us: int) -> None:
        """Run ``dt_us // tick_us`` ticks, each ADR-0004 §2 steps a to f.

        Uplink queued by :meth:`send` arrives in the first of these ticks. ``dt_us = 0``
        does nothing.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative or not a multiple of ``tick_us``.
            RuntimeError: :meth:`reset` has not been called.
        """
        check_us("dt_us", dt_us)
        tick_us = self._clock.tick_us
        if dt_us % tick_us:
            raise ValueError(
                f"dt_us must be a multiple of the tick ({tick_us} us), got {dt_us}; "
                "events are quantized to ticks before a run (ADR-0003 §1)"
            )
        if self._stack is None:
            raise RuntimeError("call reset(seed) before advance()")
        for _ in range(dt_us // tick_us):
            self._run_tick()

    def close(self) -> None:
        """Discard queued uplink and undrained downlink."""
        self._uplink.clear()
        self._outbox.clear()

    # --- The tick --------------------------------------------------------------------------

    def _merge_controls(self, flight_controls: SpacecraftControls) -> SpacecraftControls:
        """Step a: merge the flight computer's controls with the target-supplied fields.

        The single merge function of ADR-0004 §5 and §14. Today it is a documented
        pass-through, because no target faults exist and the traffic field does not:

        - **#60 (fault overrides)** fills it with precedence fault > flight computer:
          ``transmitter_off`` downgrades ``radio.mode`` from ``RX_TX`` to ``RX_ONLY``,
          ``sensor_freeze`` adds to ``frozen_sensors``, ``battery_drain`` sets
          ``extra_load_w``. It also tracks each fault's ``duration_us`` and releases it
          on expiry, so faults injected before tick N act in tick N.
        - **#98 (ADR-0007)** sets ``radio_traffic`` from :attr:`handover_traffic`, the
          previous tick's :class:`TickTraffic` (zeros on tick 0).

        Args:
            flight_controls: The flight computer's controls from the previous tick
                (BOOT's on tick 0).

        Returns:
            The controls every subsystem obeys this tick.
        """
        return flight_controls

    def _run_tick(self) -> None:
        stack = self._stack
        flight_computer = self._flight_computer
        assert stack is not None and flight_computer is not None
        env = self._environment

        # a. Merge the previous tick's controls with the fault overrides.
        controls = self._merge_controls(self._next_controls)

        # b. Step the subsystems in STEP_ORDER.
        stack.step(self._clock.tick_us, env, controls)
        self._clock.advance_one_tick()
        now_us = self._clock.now_us
        state = stack.snapshot()

        # Uplink: received only if the receiver is on after step b (ADR-0004 §10).
        arrived, self._uplink = self._uplink, []
        lost = 0
        undecodable = 0
        delivered: list[bytes] = []
        if state.get("comms", CommsSnapshot).truth.receiver_on:
            for frame in arrived:
                try:
                    decode_frame(frame)
                except FrameError:
                    undecodable += 1
                else:
                    delivered.append(frame)
        else:
            lost = len(arrived)

        # c. to f. The flight computer: commands, mode, telemetry, next controls.
        uplink = tuple(delivered)
        output = flight_computer.step(uplink, SpacecraftReadings.from_state(state), now_us)

        self._outbox.extend(output.downlink_frames)
        self._next_controls = output.controls
        traffic = TickTraffic(
            sent_bytes=output.sent_bytes,
            uplink_lost_count=lost,
            outbound_suppressed_count=output.outbound_suppressed_count,
        )
        self._handover_traffic = traffic
        record = SilTick(
            now_us=now_us,
            environment=env,
            controls=controls,
            state=state,
            delivered_uplink=uplink,
            undecodable_uplink_count=undecodable,
            traffic=traffic,
            downlink_frames=output.downlink_frames,
            next_controls=output.controls,
        )
        self._last_tick = record
        if self._tick_observer is not None:
            self._tick_observer(record)
