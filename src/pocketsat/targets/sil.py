"""The software-in-the-loop target: the Python spacecraft behind ``TestTarget`` (#59).

:class:`SilTarget` wires the five real subsystems (power, thermal, attitude, payload,
comms) on one shared ``SnapshotBoard`` and the :class:`~pocketsat.flight.FlightComputer`
behind the :class:`~pocketsat.targets.base.TestTarget` protocol. The orchestrator sees
only bytes in and bytes out (ADR-0002).

**One tick** of ``advance()`` runs ADR-0004 §2 steps a to f, in this order:

a. :meth:`SilTarget._merge_controls`: the flight computer's controls from the previous
   tick (BOOT's controls on tick 0), merged with the active fault overrides, with
   precedence fault > flight computer (#60, see below), and the previous tick's radio
   traffic for comms (ADR-0007, #98, see below).
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

**Target faults (#60; ADR-0002 §4, ADR-0004 §5, §8 to §11).** :meth:`SilTarget.inject`
validates a fault and records it with the simulated time it was injected, which is
the start of the next tick, so a fault injected before tick N acts in tick N
(ADR-0004 §2: faults act on the same tick, commands one tick later). A fault injected
at ``t`` with ``duration_us = d`` is active in every tick that starts in
``[t, t + d)``, then released; ``duration_us = None`` lasts until ``reset(seed)``.
Faults belong to the target, not to the flight computer, so they survive the RESET
command and ``forced_reset``; only ``reset(seed)`` clears them.

- ``transmitter_off`` (no parameters): the merge downgrades ``radio.mode`` from
  ``RX_TX`` to ``RX_ONLY`` (``OFF`` and ``RX_ONLY`` are left as they are). Comms'
  transmit capacity is then 0, so the flight computer suppresses and counts its
  outbound frames by its ordinary rule; ``SilTarget`` adds no suppression path of its
  own (ADR-0007 §5). The receiver keeps working.
- ``sensor_freeze`` (optional ``subsystem``: ``power``, ``thermal``, or ``attitude``;
  default all three): the merge adds the subsystems to ``frozen_sensors``.
- ``battery_drain`` (required ``load_w``, watts, finite and > 0): the merge sets
  ``extra_load_w`` to the sum of the active drains, in injection order.
- ``forced_reset`` (no parameters): the flight computer is held in reset for
  ``duration_us`` (``None`` or 0: a momentary pulse), then rebooted through
  :meth:`FlightComputer.reboot`, the RESET path (ADR-0004 §9). Subsystem physical
  state is never touched.

Overlapping faults combine: ``frozen_sensors`` is the union, the drains add up, and
``transmitter_off`` applies while any instance is active. Each is released on its own
expiry.

**Held in reset.** While a ``forced_reset`` holds it, the flight computer is not
stepped: no command handling, no telemetry, no new controls. The subsystems keep
stepping under the controls in force when the reset began (still merged with the
active faults). Uplink the receiver hears is dropped, because no software is running
to take it, and counted in :attr:`SilTick.uplink_dropped_in_reset_count`. The reboot
happens in the first tick that starts at or after the end of the hold (the injection
tick itself for a pulse): :meth:`FlightComputer.reboot` is called with that tick's
start time, so uptime counts from the moment the processor leaves reset, and the
flight computer then steps as usual in that tick and produces BOOT's controls at step
f. A ``forced_reset`` injected while another is pending extends the hold to the later
end; the two make one reboot.

**Time.** Simulated time is integer microseconds on the target's own ``SimClock``
(ADR-0003), which starts at 0 on ``reset(seed)``. ``advance(dt_us)`` runs
``dt_us // tick_us`` whole ticks; ``dt_us`` must be a multiple of the tick, because
ADR-0003 quantizes every scheduled event to a tick boundary before a run, so a partial
tick is a caller error and raises ``ValueError`` instead of being rounded. The flight
computer is called with the time at the **end** of the tick, the instant the readings
it receives describe.

**Radio traffic (ADR-0007, #98).** Each tick ``SilTarget`` computes the tick's three
per-tick traffic values, the wire bytes of the flight computer's downlink frames
(``FlightComputerOutput.sent_bytes``), the uplink frames lost because the receiver was
off, and the flight computer's suppressed-frame count, and holds them in
:class:`TickTraffic` (:attr:`SilTarget.handover_traffic`). At step a of the next tick
:meth:`SilTarget._merge_controls` puts them into ``SpacecraftControls.radio_traffic``
as a :class:`~pocketsat.spacecraft.controls.RadioTraffic`, and comms validates them,
charges the bytes, and keeps the running totals. So comms reports tick N's traffic in
tick N+1, and power sees its energy in tick N+2. ``SilTarget`` keeps no running totals
and does not re-arbitrate, trim, or drop outbound frames: the flight computer keeps
within the capacity, and comms rejects a record that does not (``ValueError`` out of
:meth:`SilTarget.advance`). The traffic is handed over in every tick, also while a
``forced_reset`` holds the flight computer (it then sends nothing, but lost uplink is
still counted).

Deterministic: same configuration, starting state, seed, and inputs give identical
downlink bytes. No wall-clock time and no global randomness; every random stream
comes from the run's ``RngFactory`` (ADR-0003).
"""

import math
from collections.abc import Callable
from dataclasses import dataclass, replace
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
from pocketsat.spacecraft.controls import (
    NO_RADIO_TRAFFIC,
    SENSOR_SUBSYSTEMS,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SpacecraftControls,
)
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

__all__ = [
    "BATTERY_DRAIN",
    "FORCED_RESET",
    "SENSOR_FREEZE",
    "SIL_CAPABILITIES",
    "TRANSMITTER_OFF",
    "SilTarget",
    "SilTick",
    "TickObserver",
    "TickTraffic",
]

FORCED_RESET: Final = "forced_reset"
"""Reboot the flight computer through the RESET path; ``duration_us`` is the time held
in reset (``None`` or 0: a momentary pulse). No parameters (ADR-0004 §9)."""

SENSOR_FREEZE: Final = "sensor_freeze"
"""Hold readings at their last reported value. Optional parameter ``subsystem``:
``power``, ``thermal``, or ``attitude``; default all three (ADR-0004 §8)."""

TRANSMITTER_OFF: Final = "transmitter_off"
"""Transmitter off, receiver still on: ``RX_TX`` becomes ``RX_ONLY``. No parameters
(ADR-0004 §10)."""

BATTERY_DRAIN: Final = "battery_drain"
"""An extra electrical load. Required parameter ``load_w``: watts, finite and > 0
(ADR-0004 §11)."""

SIL_CAPABILITIES: Final = TargetCapabilities(
    deterministic=True,
    real_time=False,
    supported_faults=frozenset({FORCED_RESET, SENSOR_FREEZE, TRANSMITTER_OFF, BATTERY_DRAIN}),
)
"""What :class:`SilTarget` declares: deterministic, not real time, and the four SIL
target faults (#60)."""


@dataclass(frozen=True, slots=True)
class TickTraffic:
    """One tick's radio traffic, held by ``SilTarget`` for the hand-over to the next
    tick (ADR-0007 §1, §3).

    Internal to ``SilTarget``: it never crosses ``TestTarget``. The fields are exactly
    those of ADR-0007's :class:`~pocketsat.spacecraft.controls.RadioTraffic`, which
    :meth:`SilTarget._merge_controls` builds from this record for
    ``SpacecraftControls.radio_traffic`` in the next tick. All values are per tick,
    never running totals. Built only by ``SilTarget``, so not validated (the
    ``RadioTraffic`` built from it is).

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
        traffic: The tick's radio traffic, handed to comms in the next tick as
            ``controls.radio_traffic`` (ADR-0007). This tick's :attr:`controls` carry
            the previous tick's.
        downlink_frames: The frames the flight computer sent, in transmit order; the
            same frames :meth:`SilTarget.receive` returns.
        next_controls: The controls the flight computer produced for the next tick
            (step f), before any merge. While the flight computer is held in reset,
            the controls in force when the reset began, unchanged.
        active_faults: The override faults (``transmitter_off``, ``sensor_freeze``,
            ``battery_drain``) merged into :attr:`controls` this tick, in injection
            order. A ``forced_reset`` shows in the next two fields instead.
        flight_computer_held: A ``forced_reset`` held the flight computer in reset,
            so it did not step this tick.
        flight_computer_rebooted: A ``forced_reset`` ended at the start of this tick:
            the flight computer was rebooted, then stepped.
        uplink_dropped_in_reset_count: Frames the receiver heard while the flight
            computer was held in reset; dropped and never passed on. Not part of
            ``uplink_lost_count``, because the receiver was on.
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
    active_faults: tuple[TargetFault, ...]
    flight_computer_held: bool
    flight_computer_rebooted: bool
    uplink_dropped_in_reset_count: int


@dataclass(frozen=True, slots=True)
class _ActiveFault:
    """An injected override fault, validated, with the time it is released."""

    fault: TargetFault
    end_us: int | None  # exclusive; None lasts until reset(seed)
    frozen_sensors: frozenset[str]
    extra_load_w: float


def _check_param_names(fault: TargetFault, allowed: frozenset[str]) -> None:
    unknown = sorted(set(fault.params) - allowed)
    if unknown:
        takes = ", ".join(sorted(allowed)) or "no parameters"
        raise ValueError(f"{fault.fault_type} takes {takes}; got unknown parameters {unknown}")


def _validate_fault(fault: TargetFault) -> tuple[frozenset[str], float]:
    """Check a supported fault's parameters (#60) and return what it merges in.

    Returns:
        The fault's ``frozen_sensors`` and ``extra_load_w`` contributions.

    Raises:
        ValueError: A parameter is unknown, missing, of the wrong type, or out of range.
    """
    kind = fault.fault_type
    if kind == SENSOR_FREEZE:
        _check_param_names(fault, frozenset({"subsystem"}))
        if "subsystem" not in fault.params:
            return SENSOR_SUBSYSTEMS, 0.0
        subsystem = fault.params["subsystem"]
        if not isinstance(subsystem, str) or subsystem not in SENSOR_SUBSYSTEMS:
            raise ValueError(
                f"sensor_freeze subsystem must be one of {sorted(SENSOR_SUBSYSTEMS)}, "
                f"got {subsystem!r}"
            )
        return frozenset({subsystem}), 0.0
    if kind == BATTERY_DRAIN:
        _check_param_names(fault, frozenset({"load_w"}))
        if "load_w" not in fault.params:
            raise ValueError("battery_drain requires load_w (watts, > 0)")
        load = fault.params["load_w"]
        if isinstance(load, bool) or not isinstance(load, int | float):
            raise ValueError(f"battery_drain load_w must be a number of watts, got {load!r}")
        if not math.isfinite(load) or load <= 0:
            raise ValueError(f"battery_drain load_w must be finite and > 0, got {load}")
        return frozenset(), float(load)
    # forced_reset and transmitter_off take no parameters.
    _check_param_names(fault, frozenset())
    return frozenset(), 0.0


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
        capabilities: Deterministic, not real time, and the four SIL target faults
            (:data:`SIL_CAPABILITIES`).
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
        self._faults: list[_ActiveFault] = []
        self._reset_release_us: int | None = None

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

        Zero after ``reset(seed)``, so tick 0 hands over nothing (ADR-0007 §3).
        :meth:`_merge_controls` puts it into ``SpacecraftControls.radio_traffic``.
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
        tick 0 (ADR-0004 §2). Simulated time returns to 0; every active or pending
        fault, queued uplink, undrained downlink, the traffic hand-over, and the last
        tick record are discarded; the
        environment returns to the nominal ``EnvironmentState()`` until the next
        :meth:`apply_environment`.

        Raises:
            TypeError: ``seed`` is not an int.
            ValueError: The flight computer's chunk size
                (``FlightComputerConfig.downlink``) is not the payload's (#56).
        """
        rng = RngFactory(seed)  # validates the seed
        stack = _build_stack(self._config, self._initial)
        stack.reset(rng)
        flight_computer = self._flight_computer_factory()
        chunk_size = flight_computer.config.downlink.chunk_size_bytes
        if chunk_size != self._config.payload.chunk_size_bytes:
            raise ValueError(
                f"the flight computer's DownlinkConfig.chunk_size_bytes ({chunk_size}) must "
                f"equal the payload's PayloadConfig.chunk_size_bytes "
                f"({self._config.payload.chunk_size_bytes}): its DATA frames carry the "
                "payload's chunks (#56)"
            )
        self._clock = SimClock(self._clock.tick_us)
        self._next_controls = flight_computer.reset(now_us=self._clock.now_us)
        self._stack = stack
        self._flight_computer = flight_computer
        self._environment = EnvironmentState()
        self._handover_traffic = TickTraffic()
        self._uplink.clear()
        self._outbox.clear()
        self._last_tick = None
        self._faults = []
        self._reset_release_us = None

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
        """Apply a target fault from the next tick on (#60).

        The fault starts at the current simulated time, the start of the next tick, and
        acts in that tick. The target owns its expiry (ADR-0004 §5): it is released
        after ``fault.duration_us``, or at ``reset(seed)`` if that is ``None``. For
        ``forced_reset`` the duration is the time held in reset, and ``None`` or 0 is a
        momentary pulse. See the module docstring for each fault's effect.

        Raises:
            TypeError: ``fault`` is not a :class:`TargetFault`.
            UnsupportedFaultError: The fault type is not in
                ``capabilities.supported_faults`` (ADR-0002 §4).
            ValueError: A parameter is unknown, missing, of the wrong type, or out of
                range.
            RuntimeError: :meth:`reset` has not been called.
        """
        if not isinstance(fault, TargetFault):
            raise TypeError(f"fault must be a TargetFault, got {type(fault).__name__}")
        if fault.fault_type not in self.capabilities.supported_faults:
            raise UnsupportedFaultError(fault.fault_type, self.capabilities.supported_faults)
        frozen, load = _validate_fault(fault)
        if self._stack is None:
            raise RuntimeError("call reset(seed) before inject()")
        now_us = self._clock.now_us
        if fault.fault_type == FORCED_RESET:
            release_us = now_us + (fault.duration_us or 0)
            pending = self._reset_release_us
            self._reset_release_us = release_us if pending is None else max(pending, release_us)
            return
        end_us = None if fault.duration_us is None else now_us + fault.duration_us
        self._faults.append(_ActiveFault(fault, end_us, frozen, load))

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

        The single merge function of ADR-0004 §5 and §14, run at the start of the tick
        (``now_us`` is the tick's start time):

        - **Expiry (#60).** Every override fault whose ``duration_us`` has run out by
          the start of this tick is released first, so a fault injected at ``t`` with
          duration ``d`` acts in the ticks starting in ``[t, t + d)``.
        - **Fault overrides (#60), fault > flight computer.** ``transmitter_off``
          downgrades ``radio.mode`` from ``RX_TX`` to ``RX_ONLY`` (the receiver keeps
          working; ``OFF`` and ``RX_ONLY`` already have the transmitter off).
          ``sensor_freeze`` adds its subsystems to ``frozen_sensors``.
          ``battery_drain`` sets ``extra_load_w`` to the sum of the active drains'
          ``load_w``, in injection order.
        - **Radio traffic (ADR-0007, #98).** ``radio_traffic`` is set from
          :attr:`handover_traffic`, the previous tick's :class:`TickTraffic` (zeros on
          tick 0), replacing whatever the flight computer's controls carry (its own
          controls carry :data:`~pocketsat.spacecraft.controls.NO_RADIO_TRAFFIC`). It
          is independent of the fault overrides: ``transmitter_off`` never touches it.

        With no fault active and no traffic to hand over, the flight computer's
        controls pass through unchanged (the same object).

        ``forced_reset`` is not a control override: it is handled in the tick itself
        (see the module docstring), and the controls it holds are these merged ones.

        Args:
            flight_controls: The flight computer's controls from the previous tick
                (BOOT's on tick 0, or the held controls during a ``forced_reset``).

        Returns:
            The controls every subsystem obeys this tick.
        """
        now_us = self._clock.now_us
        handover = self._handover_traffic
        if handover.sent_bytes or handover.uplink_lost_count or handover.outbound_suppressed_count:
            traffic = RadioTraffic(
                handover.sent_bytes,
                handover.uplink_lost_count,
                handover.outbound_suppressed_count,
            )
        else:
            traffic = NO_RADIO_TRAFFIC
        self._faults = [f for f in self._faults if f.end_us is None or now_us < f.end_us]
        if not self._faults:
            if flight_controls.radio_traffic is traffic:
                return flight_controls
            # Field by field rather than dataclasses.replace: this runs in every tick
            # that sent a frame (#78's budget).
            return SpacecraftControls(
                flight_controls.payload,
                flight_controls.radio,
                flight_controls.attitude,
                flight_controls.frozen_sensors,
                flight_controls.extra_load_w,
                traffic,
            )

        transmitter_off = False
        frozen = flight_controls.frozen_sensors
        drain_w: float | None = None
        for active in self._faults:
            kind = active.fault.fault_type
            if kind == TRANSMITTER_OFF:
                transmitter_off = True
            elif kind == SENSOR_FREEZE:
                frozen = frozen | active.frozen_sensors
            else:  # BATTERY_DRAIN
                drain_w = active.extra_load_w if drain_w is None else drain_w + active.extra_load_w

        radio = flight_controls.radio
        if transmitter_off and radio.mode is RadioMode.RX_TX:
            radio = RadioControls(mode=RadioMode.RX_ONLY)
        extra_load_w = flight_controls.extra_load_w if drain_w is None else drain_w
        if (
            radio is flight_controls.radio
            and frozen == flight_controls.frozen_sensors
            and extra_load_w == flight_controls.extra_load_w
            and traffic is flight_controls.radio_traffic
        ):
            return flight_controls  # the faults change nothing here (e.g. OFF stays OFF)
        return replace(
            flight_controls,
            radio=radio,
            frozen_sensors=frozen,
            extra_load_w=extra_load_w,
            radio_traffic=traffic,
        )

    def _reset_hold(self) -> tuple[bool, bool]:
        """Whether a ``forced_reset`` holds the flight computer this tick, and whether
        it is rebooted at the start of this tick (the hold has ended)."""
        release_us = self._reset_release_us
        if release_us is None:
            return False, False
        if self._clock.now_us < release_us:
            return True, False
        self._reset_release_us = None
        return False, True

    def _run_tick(self) -> None:
        stack = self._stack
        flight_computer = self._flight_computer
        assert stack is not None and flight_computer is not None
        env = self._environment

        start_us = self._clock.now_us

        # a. Merge the previous tick's controls with the fault overrides and the
        # previous tick's radio traffic (ADR-0007).
        controls = self._merge_controls(self._next_controls)
        active_faults = tuple(active.fault for active in self._faults)
        held, rebooted = self._reset_hold()

        # b. Step the subsystems in STEP_ORDER.
        stack.step(self._clock.tick_us, env, controls)
        self._clock.advance_one_tick()
        now_us = self._clock.now_us
        state = stack.snapshot()

        # Uplink: received only if the receiver is on after step b (ADR-0004 §10), and
        # taken only if the flight computer is running.
        arrived, self._uplink = self._uplink, []
        lost = 0
        undecodable = 0
        dropped_in_reset = 0
        delivered: list[bytes] = []
        if not state.get("comms", CommsSnapshot).truth.receiver_on:
            lost = len(arrived)
        elif held:
            dropped_in_reset = len(arrived)
        else:
            for frame in arrived:
                try:
                    decode_frame(frame)
                except FrameError:
                    undecodable += 1
                else:
                    delivered.append(frame)
        uplink = tuple(delivered)

        # c. to f. The flight computer: commands, mode, telemetry, next controls. Held
        # in reset, it does nothing and the controls in force stay in force.
        if held:
            downlink: tuple[bytes, ...] = ()
            traffic = TickTraffic(uplink_lost_count=lost)
        else:
            if rebooted:
                flight_computer.reboot(now_us=start_us)
            output = flight_computer.step(uplink, SpacecraftReadings.from_state(state), now_us)
            downlink = output.downlink_frames
            self._next_controls = output.controls
            traffic = TickTraffic(
                sent_bytes=output.sent_bytes,
                uplink_lost_count=lost,
                outbound_suppressed_count=output.outbound_suppressed_count,
            )

        self._outbox.extend(downlink)
        self._handover_traffic = traffic
        record = SilTick(
            now_us=now_us,
            environment=env,
            controls=controls,
            state=state,
            delivered_uplink=uplink,
            undecodable_uplink_count=undecodable,
            traffic=traffic,
            downlink_frames=downlink,
            next_controls=self._next_controls,
            active_faults=active_faults,
            flight_computer_held=held,
            flight_computer_rebooted=rebooted,
            uplink_dropped_in_reset_count=dropped_in_reset,
        )
        self._last_tick = record
        if self._tick_observer is not None:
            self._tick_observer(record)
