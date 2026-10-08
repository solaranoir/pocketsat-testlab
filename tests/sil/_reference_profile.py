"""#72's operating profiles, run with real radio traffic (#98), shared by the SIL tests.

The power and thermal budget tests (#72, ``test_power_thermal_budget.py``) and the epic
#30 close-out test (#70, ``test_epic_30_closeout.py``) run the **reference profile**:
SCIENCE for the whole orbit, and BEGIN_DOWNLINK 10 minutes before the end of each orbit
(the end of eclipse, the worst place for the minimum SOC).

**Real traffic (#98).** :class:`ProfileGround` drives a ``SilTarget`` with the real
flight computer through the ``TestTarget`` interface, as the ground will: COMMAND frames
in with ``send()``, ``NominalEnvironment`` applied before every tick, one tick per
``advance()``. Every byte the spacecraft transmits is a real frame (36-byte telemetry at
the per-mode cadence, ACK/NACK, 78-byte DATA frames in DOWNLINK), handed to comms by
``SilTarget`` (ADR-0007), so comms' own ``transmit_power_w`` carries the transmit
energy and power integrates it two ticks after each frame. The flight computer follows
its mode table (#47): payload enabled only in SCIENCE (off in DOWNLINK), radio
``RX_TX``, attitude control on after BOOT; the downlink session sends every stored
chunk and releases it once sent (#56), then ``DOWNLINK_COMPLETE`` returns to SCIENCE.
#72's traffic stand-ins (an assumed 64-byte beacon once a second and full transmit
capacity for the whole pass, charged through ``extra_load_w``, and a scripted chunk
release) are gone.

Two must-fail and information profiles ask for what the flight software never does.
They use :class:`ProfileFlightComputer`, the real flight computer with one documented
change to its output, so their traffic is still real frames through comms:

- ``keyed``: the transmitter at full capacity in every tick (the old "continuous
  DOWNLINK" worst case, with the payload acquiring as in SCIENCE): after the real
  frames, filler frames fill the rest of comms' capacity.
- ``attitude=False``: attitude control off (tumbling).

:class:`ProfileDriver` is the stack-level driver kept for the one test that cannot run
``SilTarget``: the close-out's named-stream independence check swaps subsystems for
fakes, which needs a bare ``SubsystemStack``. It scripts the mode table's controls and
reports scripted traffic of the real frame sizes through ``controls.radio_traffic``,
comms' real input (never ``extra_load_w``).
"""

from collections.abc import Callable, Iterable
from dataclasses import dataclass, replace
from typing import Literal

from pocketsat.core.rng import RngFactory
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import (
    DEFAULT_FLIGHT_COMPUTER_CONFIG,
    FlightComputer,
    FlightComputerConfig,
    FlightComputerOutput,
    Mode,
    SpacecraftReadings,
)
from pocketsat.frame import MIN_FRAME_SIZE, Frame, FrameType, encode_frame
from pocketsat.messages import TELEMETRY_FRAME_SIZE, Command, encode_command
from pocketsat.spacecraft import (
    CHUNK_ID_SIZE_BYTES,
    DEFAULT_INITIAL_STATE,
    NO_RADIO_TRAFFIC,
    NOMINAL_CONFIG,
    Attitude,
    AttitudeControls,
    Comms,
    Payload,
    PayloadControls,
    PayloadSnapshot,
    Power,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SnapshotBoard,
    SpacecraftConfig,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    Subsystem,
    SubsystemStack,
    Thermal,
)
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import BATTERY_DRAIN, SilTarget, SilTick

US_PER_S = 1_000_000

PASS_US = 10 * 60 * US_PER_S
"""DOWNLINK pass length: 10 minutes, at the end of each orbit (the end of eclipse, the
worst place for the minimum SOC)."""

SCIENCE_COMMAND_US = 6 * US_PER_S
"""When the ground sends SET_MODE SCIENCE: one second after the default 5 s boot."""

TELEMETRY_PERIOD_US = DEFAULT_FLIGHT_COMPUTER_CONFIG.telemetry.science_period_us
"""The real telemetry cadence in SCIENCE and DOWNLINK (1 s, #55)."""

Downlink = Literal["none", "pass"]


@dataclass(frozen=True)
class Profile:
    """An operating profile: what the ground commands, and the flight computer's one
    scripted deviation, if any (see the module docstring).

    Attributes:
        name: For reports.
        science: Command SCIENCE after the boot; otherwise the flight computer stays in
            NOMINAL (payload off).
        downlink: ``"pass"``: BEGIN_DOWNLINK at the start of the last
            :data:`PASS_US` of every orbit.
        attitude: Attitude control as the mode table commands it; ``False`` holds it
            off (tumbling).
        keyed: Fill the transmit capacity with filler frames in every tick.
    """

    name: str
    science: bool = True
    downlink: Downlink = "pass"
    attitude: bool = True
    keyed: bool = False


REFERENCE = Profile("reference (SCIENCE + 10-min DOWNLINK pass)")
"""#72's reference profile, used by #63 and #70."""


# --- The SilTarget harness (real traffic) ----------------------------------------------


class ProfileFlightComputer(FlightComputer):
    """The real flight computer, with at most one scripted change to its output.

    Args:
        config: The flight computer's settings.
        attitude: ``False`` turns attitude control off in every controls record it
            produces (BOOT already has it off).
        keyed: After the real frames, fill what is left of comms' capacity with filler
            frames (never decoded), so the transmitter sends at full capacity in every
            tick it is on.
    """

    def __init__(
        self,
        config: FlightComputerConfig = DEFAULT_FLIGHT_COMPUTER_CONFIG,
        *,
        attitude: bool = True,
        keyed: bool = False,
    ) -> None:
        super().__init__(config)
        self._attitude_off = not attitude
        self._keyed = keyed

    def step(
        self, uplink_frames: Iterable[bytes], readings: SpacecraftReadings, now_us: int
    ) -> FlightComputerOutput:
        output = super().step(uplink_frames, readings, now_us)
        if self._attitude_off and output.controls.attitude.enabled:
            output = replace(
                output, controls=replace(output.controls, attitude=AttitudeControls(False))
            )
        if self._keyed:
            room = readings.comms.transmit_capacity_bytes - output.sent_bytes
            if room >= MIN_FRAME_SIZE:
                filler = encode_frame(Frame(FrameType.DATA, 0, bytes(room - MIN_FRAME_SIZE)))
                output = replace(output, downlink_frames=(*output.downlink_frames, filler))
        return output


class ProfileGround:
    """Runs ``profile`` on a ``SilTarget`` with real traffic, one tick per :meth:`tick`.

    Only the ``TestTarget`` methods drive the run (``apply_environment``, ``send``,
    ``advance``, ``receive``, ``inject``); the tests read each tick's
    :class:`~pocketsat.targets.sil.SilTick` to measure it.

    Args:
        profile: What to command.
        seed: The run's seed.
        config: Spacecraft settings.
        initial: Starting state.
        environment: The orbit; ``NominalEnvironment()`` by default.
        extra_load_w: A ``battery_drain`` fault of this load for the whole run, watts
            (0: none).
        observer: Called with every tick's record.
    """

    def __init__(
        self,
        profile: Profile,
        seed: int,
        *,
        config: SpacecraftConfig = NOMINAL_CONFIG,
        initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
        environment: NominalEnvironment | None = None,
        extra_load_w: float = 0.0,
        observer: Callable[[SilTick], None] | None = None,
    ) -> None:
        self.profile = profile
        self.environment = NominalEnvironment() if environment is None else environment
        self.computers: list[ProfileFlightComputer] = []

        def factory() -> ProfileFlightComputer:
            computer = ProfileFlightComputer(attitude=profile.attitude, keyed=profile.keyed)
            self.computers.append(computer)
            return computer

        self.target = SilTarget(
            config=config, initial=initial, flight_computer_factory=factory, tick_observer=observer
        )
        self.target.connect()
        self.target.reset(seed)
        if extra_load_w:
            self.target.inject(TargetFault(BATTERY_DRAIN, {"load_w": extra_load_w}))
        self.tick_us = self.target.tick_us
        period = self.environment.orbit_period_us
        assert period % self.tick_us == 0 and PASS_US % self.tick_us == 0
        self.ticks_per_orbit = period // self.tick_us
        self._pass_start_tick = (period - PASS_US) // self.tick_us
        self._science_tick = SCIENCE_COMMAND_US // self.tick_us
        self.count = 0
        """Ticks run so far."""
        self.sequence = 0
        self.downlink_bytes = 0
        """Bytes ``receive()`` returned over the run."""

    @property
    def mode(self) -> Mode:
        """The flight computer's mode (for reports only; the ground never reads it)."""
        return self.computers[-1].mode

    def in_pass(self, tick_index: int) -> bool:
        """Whether tick ``tick_index`` is in the 10-minute pass window."""
        return (
            self.profile.downlink == "pass"
            and tick_index % self.ticks_per_orbit >= self._pass_start_tick
        )

    def _send(self, command: Command) -> None:
        self.sequence += 1
        frame = Frame(FrameType.COMMAND, self.sequence, encode_command(command))
        self.target.send(encode_frame(frame))

    def tick(self) -> SilTick:
        """Run one tick: the environment, the ground's commands, the tick, the downlink."""
        n = self.count
        target = self.target
        target.apply_environment(self.environment.state_at(target.now_us))
        if self.profile.science and n == self._science_tick:
            self._send(Command.set_mode(Mode.SCIENCE))
        if self.profile.downlink == "pass" and n % self.ticks_per_orbit == self._pass_start_tick:
            self._send(Command.begin_downlink())
        target.advance(self.tick_us)
        self.downlink_bytes += sum(len(frame) for frame in target.receive())
        self.count += 1
        record = target.last_tick
        assert record is not None
        return record


def reset_state(
    seed: int,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
) -> SpacecraftState:
    """The subsystems' state after ``reset(seed)``, before tick 0: what ``SilTarget``
    starts from (it builds the same stack from the same settings)."""
    return real_stack(seed, config, initial).snapshot()


# --- Subsystem stacks ------------------------------------------------------------------


def real_subsystems(
    board: SnapshotBoard,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
) -> dict[str, Subsystem]:
    """The five real subsystems, by name, reading each other through ``board``."""
    return {
        "power": Power(config.power, initial.power, reader=board),
        "thermal": Thermal(config.thermal, initial.thermal, reader=board),
        "attitude": Attitude(config.attitude, initial.attitude),
        "payload": Payload(config.payload, initial.payload, reader=board),
        "comms": Comms(config.comms),
    }


def real_stack(
    seed: int,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
) -> SubsystemStack:
    """The five real subsystems in one stack (default ``STEP_ORDER``), reset with ``seed``."""
    board = SnapshotBoard()
    stack = SubsystemStack(real_subsystems(board, config, initial).values(), board=board)
    stack.reset(RngFactory(seed))
    return stack


def data_frame_bytes(config: SpacecraftConfig) -> int:
    """One chunk's DATA frame: chunk ID, chunk content, and the frame overhead (#56)."""
    return CHUNK_ID_SIZE_BYTES + config.payload.chunk_size_bytes + MIN_FRAME_SIZE


# --- The stack-level driver (for the close-out's stream-independence check) ------------


class ProfileDriver:
    """Scripts ``SpacecraftControls`` for the reference profile on a bare stack.

    Only for tests that cannot use ``SilTarget`` (see the module docstring). It follows
    the mode table: payload enabled outside the pass and off in it, radio ``RX_TX``,
    attitude control on. In each pass tick it sends (and releases at once) one DATA
    frame if a whole chunk is stored; a 36-byte telemetry frame goes out once a second.
    Each tick's bytes reach comms in the next tick's ``radio_traffic``, as ``SilTarget``
    hands them over (ADR-0007), so comms charges them.
    """

    def __init__(self, config: SpacecraftConfig, orbit_period_us: int) -> None:
        self._period_us = orbit_period_us
        self._pass_start_us = orbit_period_us - PASS_US
        self._data_frame = data_frame_bytes(config)
        self._radio = RadioControls(mode=RadioMode.RX_TX)
        self._attitude = AttitudeControls(enabled=True)
        self._traffic = NO_RADIO_TRAFFIC

    def command(self, now_us: int, previous: SpacecraftState) -> tuple[SpacecraftControls, bool]:
        """The controls for the tick starting at ``now_us``, given the state before it,
        and whether the tick is in the pass."""
        in_pass = now_us % self._period_us >= self._pass_start_us
        sent = TELEMETRY_FRAME_SIZE if now_us % TELEMETRY_PERIOD_US == 0 else 0
        release = None
        if in_pass:
            payload = previous.get("payload", PayloadSnapshot).truth
            if payload.next_chunk_id > payload.oldest_unreleased_chunk_id:
                release = payload.oldest_unreleased_chunk_id
                sent += self._data_frame
        controls = SpacecraftControls(
            payload=PayloadControls(enabled=not in_pass, release_through_chunk_id=release),
            radio=self._radio,
            attitude=self._attitude,
            radio_traffic=self._traffic,
        )
        self._traffic = RadioTraffic(sent) if sent else NO_RADIO_TRAFFIC
        return controls, in_pass
