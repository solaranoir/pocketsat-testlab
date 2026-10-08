"""Tests for ``SilTarget``'s target faults (#60; ADR-0002 §4, ADR-0004 §5, §8 to §11).

``forced_reset``, ``sensor_freeze``, ``transmitter_off``, and ``battery_drain`` are
merged into the controls by ``SilTarget._merge_controls`` with precedence fault >
flight computer, act from the tick after ``inject()`` (the "same tick" of ADR-0004 §2),
and are released on expiry in simulated time.

Flight computers used here:

- The real :class:`~pocketsat.flight.FlightComputer` (``RealRig``) for the physics,
  the RESET path (boot counter, uptime, BOOT and the boot duration, #49), every ACK
  (#51), telemetry (#55), and DATA (#56): the ``transmitter_off`` command, telemetry,
  and DATA tests and the ``forced_reset`` pulse and hold tests. Where a test needs to
  leave BOOT quickly it builds the flight computer with a short boot (``SHORT_BOOT``).
- :class:`ScriptedFlightComputer` (``Rig``) only where the real one lacks the
  behaviour: a command that switches the radio or attitude control directly, and
  controls forced from the test (to check which controls are held).

The same-tick half of the ADR-0004 §2 timeline is in ``test_sil_target.py``, next to
the command half.
"""

import dataclasses
import math

import pytest
from _scripted_flight_computer import (
    ScriptedFlightComputer,
    ack_sequence,
    ping,
    set_radio,
)

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.flight import (
    DEFAULT_FLIGHT_COMPUTER_CONFIG,
    FlightComputer,
    FlightComputerConfig,
    Mode,
    controls_for_mode,
)
from pocketsat.flight.boot import BootConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    TELEMETRY_FRAME_SIZE,
    Command,
    CommandAck,
    CommandId,
    Telemetry,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NO_RADIO_TRAFFIC,
    NOMINAL_CONFIG,
    SENSOR_SUBSYSTEMS,
    AttitudeControls,
    AttitudeSnapshot,
    CommsSnapshot,
    PayloadInitial,
    PayloadSnapshot,
    PayloadTruth,
    PowerSnapshot,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    ThermalSnapshot,
    chunk_content,
)
from pocketsat.targets.base import (
    EnvironmentState,
    FaultParam,
    TargetFault,
    UnsupportedFaultError,
)
from pocketsat.targets.sil import (
    BATTERY_DRAIN,
    FORCED_RESET,
    SENSOR_FREEZE,
    SIL_CAPABILITIES,
    TRANSMITTER_OFF,
    SilTarget,
    SilTick,
)

TICK = DEFAULT_TICK_US
BOOT_CONTROLS = controls_for_mode(Mode.BOOT)
ATTITUDE_ON = dataclasses.replace(BOOT_CONTROLS, attitude=AttitudeControls(enabled=True))


def fault(kind: str, duration_us: int | None = None, **params: FaultParam) -> TargetFault:
    return TargetFault(fault_type=kind, params=params, duration_us=duration_us)


class Rig:
    """A ``SilTarget`` with a scripted flight computer and a record of every tick."""

    def __init__(
        self,
        *,
        boot_controls: SpacecraftControls | None = None,
        telemetry: bool = False,
        seed: int = 7,
    ) -> None:
        self.computers: list[ScriptedFlightComputer] = []
        self.ticks: list[SilTick] = []

        def factory() -> ScriptedFlightComputer:
            computer = ScriptedFlightComputer(boot_controls, telemetry=telemetry)
            self.computers.append(computer)
            return computer

        self.target = SilTarget(flight_computer_factory=factory, tick_observer=self.ticks.append)
        self.target.reset(seed)

    @property
    def fc(self) -> ScriptedFlightComputer:
        return self.computers[-1]

    @property
    def last(self) -> SilTick:
        return self.ticks[-1]

    def tick(self, *frames: bytes) -> list[bytes]:
        """Send ``frames``, run one tick, and return the downlink it produced."""
        for frame in frames:
            self.target.send(frame)
        self.target.advance(TICK)
        return self.target.receive()

    def run(self, ticks: int) -> list[SilTick]:
        """Run ``ticks`` ticks and return their records."""
        self.target.advance(ticks * TICK)
        return self.ticks[-ticks:] if ticks else []


class RealRig:
    """A ``SilTarget`` with the real flight computer and a record of every tick."""

    def __init__(
        self,
        *,
        config: FlightComputerConfig = DEFAULT_FLIGHT_COMPUTER_CONFIG,
        initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
        seed: int = 7,
    ) -> None:
        self.computers: list[FlightComputer] = []
        self.ticks: list[SilTick] = []

        def factory() -> FlightComputer:
            computer = FlightComputer(config)
            self.computers.append(computer)
            return computer

        self.target = SilTarget(
            initial=initial, flight_computer_factory=factory, tick_observer=self.ticks.append
        )
        self.target.reset(seed)

    @property
    def fc(self) -> FlightComputer:
        return self.computers[-1]

    @property
    def last(self) -> SilTick:
        return self.ticks[-1]

    def tick(self, *frames: bytes) -> list[bytes]:
        """Send ``frames``, run one tick, and return the downlink it produced."""
        for frame in frames:
            self.target.send(frame)
        self.target.advance(TICK)
        return self.target.receive()


SHORT_BOOT = FlightComputerConfig(boot=BootConfig(duration_us=10 * TICK))
"""A 1 s boot, so the real flight computer reaches NOMINAL after 10 ticks; the default
telemetry cadence."""


def command_frame(command: Command, sequence: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))


def acks(frames: list[bytes]) -> list[CommandAck]:
    """The ACK frames among ``frames``, decoded."""
    decoded = [decode_frame(frame) for frame in frames]
    return [decode_ack(f.payload) for f in decoded if f.frame_type is FrameType.ACK]


def telemetry(frames: list[bytes]) -> list[Telemetry]:
    """The TELEMETRY frames among ``frames``, decoded."""
    decoded = [decode_frame(frame) for frame in frames]
    return [decode_telemetry(f.payload) for f in decoded if f.frame_type is FrameType.TELEMETRY]


def power(state: SpacecraftState) -> PowerSnapshot:
    return state.get("power", PowerSnapshot)


def thermal(state: SpacecraftState) -> ThermalSnapshot:
    return state.get("thermal", ThermalSnapshot)


def comms(state: SpacecraftState) -> CommsSnapshot:
    return state.get("comms", CommsSnapshot)


def mode_of(rig: RealRig) -> Mode:
    """The flight computer's mode (a call, so mypy doesn't narrow it across steps)."""
    return rig.fc.mode


def readings_of(state: SpacecraftState, subsystem: str) -> object:
    kinds = {"power": PowerSnapshot, "thermal": ThermalSnapshot, "attitude": AttitudeSnapshot}
    return state.get(subsystem, kinds[subsystem]).readings


# --- Declaration and validation ----------------------------------------------------------


def test_declares_the_four_sil_faults() -> None:
    assert SIL_CAPABILITIES.supported_faults == {
        FORCED_RESET,
        SENSOR_FREEZE,
        TRANSMITTER_OFF,
        BATTERY_DRAIN,
    }
    assert SilTarget().capabilities is SIL_CAPABILITIES


VALID_FAULTS = {
    "forced_reset": fault(FORCED_RESET),
    "forced_reset_held": fault(FORCED_RESET, duration_us=3 * TICK),
    "sensor_freeze_all": fault(SENSOR_FREEZE),
    "sensor_freeze_power": fault(SENSOR_FREEZE, subsystem="power"),
    "sensor_freeze_thermal": fault(SENSOR_FREEZE, subsystem="thermal"),
    "sensor_freeze_attitude": fault(SENSOR_FREEZE, subsystem="attitude"),
    "transmitter_off": fault(TRANSMITTER_OFF, duration_us=TICK),
    "battery_drain_float": fault(BATTERY_DRAIN, load_w=1.5),
    "battery_drain_int": fault(BATTERY_DRAIN, load_w=2),
}


@pytest.mark.parametrize("valid", VALID_FAULTS.values(), ids=list(VALID_FAULTS))
def test_valid_faults_are_accepted(valid: TargetFault) -> None:
    rig = Rig()
    rig.target.inject(valid)
    rig.run(4)


INVALID_FAULTS = {
    "drain_without_load": fault(BATTERY_DRAIN),
    "drain_zero": fault(BATTERY_DRAIN, load_w=0.0),
    "drain_negative": fault(BATTERY_DRAIN, load_w=-1.0),
    "drain_nan": fault(BATTERY_DRAIN, load_w=math.nan),
    "drain_inf": fault(BATTERY_DRAIN, load_w=math.inf),
    "drain_string": fault(BATTERY_DRAIN, load_w="2"),
    "drain_bool": fault(BATTERY_DRAIN, load_w=True),
    "drain_unknown_param": fault(BATTERY_DRAIN, load_w=1.0, subsystem="power"),
    "freeze_payload": fault(SENSOR_FREEZE, subsystem="payload"),
    "freeze_comms": fault(SENSOR_FREEZE, subsystem="comms"),
    "freeze_unknown_name": fault(SENSOR_FREEZE, subsystem="gyro"),
    "freeze_not_a_string": fault(SENSOR_FREEZE, subsystem=1),
    "freeze_unknown_param": fault(SENSOR_FREEZE, sensors="power"),
    "transmitter_off_with_param": fault(TRANSMITTER_OFF, load_w=1.0),
    "forced_reset_with_param": fault(FORCED_RESET, hold=True),
}


@pytest.mark.parametrize("invalid", INVALID_FAULTS.values(), ids=list(INVALID_FAULTS))
def test_invalid_parameters_raise_value_error_and_inject_nothing(invalid: TargetFault) -> None:
    rig = Rig()
    with pytest.raises(ValueError, match=invalid.fault_type):
        rig.target.inject(invalid)
    rig.tick()
    assert rig.last.active_faults == ()
    assert rig.last.controls == BOOT_CONTROLS  # nothing merged in
    assert not rig.last.flight_computer_held and not rig.last.flight_computer_rebooted
    assert rig.fc.reboots == []


def test_unsupported_type_is_rejected_before_its_parameters_are_read() -> None:
    target = SilTarget()
    target.reset(0)
    with pytest.raises(UnsupportedFaultError, match="memory_upset") as info:
        target.inject(fault("memory_upset", bits=3))
    assert info.value.supported == SIL_CAPABILITIES.supported_faults


def test_inject_rejects_other_types_and_requires_reset() -> None:
    target = SilTarget()
    with pytest.raises(TypeError, match="TargetFault"):
        target.inject("transmitter_off")  # type: ignore[arg-type]
    with pytest.raises(RuntimeError, match="reset"):
        target.inject(fault(TRANSMITTER_OFF))


# --- Timing: active for the duration, released on expiry ---------------------------------


@pytest.mark.parametrize(
    ("duration_us", "active_ticks"),
    [(0, 0), (1, 1), (TICK, 1), (TICK + 1, 2), (3 * TICK, 3), (3 * TICK - 1, 3)],
)
def test_fault_is_active_in_the_ticks_starting_within_its_duration(
    duration_us: int, active_ticks: int
) -> None:
    rig = Rig()
    rig.run(2)
    drain = fault(BATTERY_DRAIN, duration_us=duration_us, load_w=1.0)
    rig.target.inject(drain)
    ticks = rig.run(5)
    assert [t.controls.extra_load_w for t in ticks] == [1.0] * active_ticks + [0.0] * (
        5 - active_ticks
    )
    assert [t.active_faults for t in ticks] == [(drain,)] * active_ticks + [()] * (5 - active_ticks)


def test_fault_without_duration_lasts_until_reset_seed() -> None:
    rig = Rig()
    rig.target.inject(fault(TRANSMITTER_OFF))
    ticks = rig.run(50)
    assert all(t.controls.radio.mode is RadioMode.RX_ONLY for t in ticks)
    rig.target.reset(7)
    rig.tick()
    assert rig.last.controls.radio.mode is RadioMode.RX_TX
    assert rig.last.active_faults == ()


def test_reset_seed_clears_active_faults_and_a_pending_forced_reset() -> None:
    rig = Rig()
    rig.target.inject(fault(SENSOR_FREEZE))
    rig.target.inject(fault(FORCED_RESET, duration_us=10 * TICK))
    rig.tick()
    assert rig.last.flight_computer_held
    rig.target.reset(7)
    ticks = rig.run(3)
    assert all(t.controls == BOOT_CONTROLS for t in ticks)
    assert not any(t.flight_computer_held or t.flight_computer_rebooted for t in ticks)
    assert len(rig.fc.calls) == 3


def test_fault_injected_mid_run_starts_at_the_current_time() -> None:
    # The duration counts from inject(), not from the start of the run.
    rig = Rig()
    rig.run(7)
    rig.target.inject(fault(TRANSMITTER_OFF, duration_us=2 * TICK))
    ticks = rig.run(3)
    assert [t.controls.radio.mode for t in ticks] == [
        RadioMode.RX_ONLY,
        RadioMode.RX_ONLY,
        RadioMode.RX_TX,
    ]


# --- transmitter_off ---------------------------------------------------------------------


def test_transmitter_off_downgrades_rx_tx_to_rx_only_and_is_released() -> None:
    rig = Rig()
    rig.tick()
    rig.target.inject(fault(TRANSMITTER_OFF, duration_us=3 * TICK))
    during = rig.run(3)
    after = rig.run(2)
    for t in during:
        assert t.controls.radio.mode is RadioMode.RX_ONLY
        assert t.next_controls.radio.mode is RadioMode.RX_TX  # the flight computer's
        truth = comms(t.state).truth
        assert truth.receiver_on and not truth.transmitter_on
        assert truth.transmit_capacity_bytes == 0
    for t in after:
        assert t.controls.radio.mode is RadioMode.RX_TX
        assert comms(t.state).truth.transmitter_on
    # Only the radio mode is touched.
    assert dataclasses.replace(during[0].controls, radio=RadioControls()) == BOOT_CONTROLS


@pytest.mark.parametrize("mode", [RadioMode.OFF, RadioMode.RX_ONLY])
def test_transmitter_off_leaves_modes_without_a_transmitter_alone(mode: RadioMode) -> None:
    # OFF stays OFF: the fault never turns the receiver on.
    controls = dataclasses.replace(BOOT_CONTROLS, radio=RadioControls(mode=mode))
    rig = Rig(boot_controls=controls)
    rig.target.inject(fault(TRANSMITTER_OFF))
    rig.tick()
    assert rig.last.controls.radio.mode is mode
    assert rig.last.controls is controls


def test_under_transmitter_off_commands_are_executed_but_not_acknowledged() -> None:
    # #60 / ADR-0007 §5, with the real flight computer (#51): capacity 0, so it
    # suppresses the ACK by its ordinary outbound rule and counts it; SilTarget adds no
    # suppression of its own and passes the count on in the hand-over to tick N+1.
    rig = RealRig(config=SHORT_BOOT)
    rig.target.advance(10 * TICK)
    assert mode_of(rig) is Mode.NOMINAL
    rig.target.receive()
    rig.target.inject(fault(TRANSMITTER_OFF, duration_us=3 * TICK))

    to_science = command_frame(Command.set_mode(Mode.SCIENCE), sequence=3)
    downlink_n = rig.tick(to_science)
    tick_n = rig.last
    assert tick_n.delivered_uplink == (to_science,)  # received
    assert mode_of(rig) is Mode.SCIENCE  # executed
    assert downlink_n == [] and tick_n.downlink_frames == ()
    assert tick_n.traffic.sent_bytes == 0
    # The ACK, and the telemetry frame the mode change makes due (#55).
    assert tick_n.traffic.outbound_suppressed_count == 2
    assert tick_n.traffic.uplink_lost_count == 0
    assert rig.target.handover_traffic.outbound_suppressed_count == 2  # for tick N+1

    # Its effect lands in tick N+1, like any command: the payload is switched on.
    assert rig.tick(command_frame(Command.ping(), sequence=4)) == []
    assert rig.last.controls.payload.enabled
    assert rig.last.controls.radio.mode is RadioMode.RX_ONLY
    assert rig.last.traffic.outbound_suppressed_count == 1

    # Nothing to send (no telemetry due until 1 s after the mode change), nothing
    # suppressed.
    assert rig.tick() == []
    assert rig.last.traffic.outbound_suppressed_count == 0

    # Released: ACKs go out again.
    assert acks(rig.tick(command_frame(Command.ping(), sequence=5))) == [
        CommandAck(5, CommandId.PING)
    ]
    assert rig.last.traffic.outbound_suppressed_count == 0


def test_under_transmitter_off_telemetry_is_suppressed_and_counted_then_resumes() -> None:
    # The real flight computer's telemetry (#55) under transmitter_off (#60): the frame
    # due in a tick with no capacity is suppressed and counted (with the ACK), never
    # sent later, and the cadence resumes when capacity returns: the next frame is one
    # period after the suppressed one, with no burst of missed frames. DATA under
    # transmitter_off is the next test.
    rig = RealRig(config=SHORT_BOOT)
    rig.target.advance(10 * TICK)  # ticks 0-9: the boot-complete beacon in tick 9 (1.0 s)
    assert [t.mode for t in telemetry(rig.target.receive())] == [Mode.NOMINAL]
    rig.target.advance(5 * TICK)  # ticks 10-14: next frame due at 2.0 s (tick 19)
    assert rig.target.receive() == []
    rig.target.inject(fault(TRANSMITTER_OFF, duration_us=10 * TICK))  # ticks 15-24

    held = [rig.tick(command_frame(Command.ping(), sequence=3))]
    held += [rig.tick() for _ in range(9)]
    assert held == [[]] * 10
    faulted = rig.ticks[-10:]
    assert all(t.traffic.sent_bytes == 0 for t in faulted)
    # The ping's ACK in tick 15, the telemetry frame due in tick 19; nothing queued.
    assert [t.traffic.outbound_suppressed_count for t in faulted] == [1, 0, 0, 0, 1] + [0] * 5
    assert rig.target.handover_traffic.outbound_suppressed_count == 0

    # Capacity is back from tick 25; the next frame is due at 3.0 s (tick 29).
    after = [rig.tick() for _ in range(5)]
    assert after[:4] == [[]] * 4
    [frame] = telemetry(after[4])
    assert (frame.mode, frame.uptime_ms) == (Mode.NOMINAL, 3000)
    # Suppressed frames used no sequence number: the beacon was 0, this is 1.
    assert decode_frame(after[4][0]).sequence == 1
    assert rig.last.traffic.outbound_suppressed_count == 0


def test_under_transmitter_off_data_stalls_and_the_chunks_stay_buffered() -> None:
    # #56's tightened criterion and the #60 gap from PR #114, with the real flight
    # computer: disabling the transmitter mid-pass stops DATA (capacity 0) and so stalls
    # the release; the data stays in the payload buffer, DATA is not counted as
    # suppressed (only ACK/NACK and telemetry are, ADR-0007 §3), and the pass resumes
    # from the next chunk once the fault is released.
    chunks = 40
    fill = (chunks * 64 + 32) / NOMINAL_CONFIG.payload.buffer_capacity_bytes
    initial = dataclasses.replace(DEFAULT_INITIAL_STATE, payload=PayloadInitial(buffer_fill=fill))
    rig = RealRig(config=SHORT_BOOT, initial=initial)
    rig.target.advance(10 * TICK)
    assert mode_of(rig) is Mode.NOMINAL
    rig.tick(command_frame(Command.begin_downlink(), sequence=1))
    sent = [data_ids(rig.tick()) for _ in range(5)]
    assert sent == [[0], [1], [2], [3], [4]]

    rig.target.inject(fault(TRANSMITTER_OFF, duration_us=20 * TICK))
    stalled = [rig.tick() for _ in range(20)]
    assert stalled == [[]] * 20
    faulted = rig.ticks[-20:]
    assert all(t.traffic.sent_bytes == 0 for t in faulted)
    assert all(t.controls.radio.mode is RadioMode.RX_ONLY for t in faulted)
    # Chunk 4, sent just before the fault, is released in its first tick; nothing after.
    released = [payload_of(t).oldest_unreleased_chunk_id for t in faulted]
    assert released == [5] * 20
    assert all(
        payload_of(t).buffered_bytes == payload_of(faulted[0]).buffered_bytes for t in faulted
    )
    assert all(
        payload_of(t).total_produced_bytes
        == payload_of(t).buffered_bytes + payload_of(t).total_released_bytes
        for t in faulted
    )
    # Only the telemetry frames due during the fault are counted, never DATA.
    assert sum(t.traffic.outbound_suppressed_count for t in faulted) == 2
    assert mode_of(rig) is Mode.DOWNLINK

    # Released: the pass resumes at chunk 5 and completes.
    resumed: list[int] = []
    for _ in range(100):
        resumed += data_ids(rig.tick())
        if mode_of(rig) is Mode.NOMINAL:
            break
    assert resumed == list(range(5, chunks))
    assert payload_of(rig.last).oldest_unreleased_chunk_id == chunks


def data_ids(frames: list[bytes]) -> list[int]:
    """The chunk IDs of the DATA frames among ``frames``, checked against their content."""
    ids: list[int] = []
    for frame in map(decode_frame, frames):
        if frame.frame_type is FrameType.DATA:
            chunk = decode_data(frame.payload)
            assert chunk.content == chunk_content(chunk.chunk_id, 64)
            ids.append(chunk.chunk_id)
    return ids


def payload_of(tick: SilTick) -> PayloadTruth:
    return tick.state.get("payload", PayloadSnapshot).truth


def test_transmitter_off_with_a_radio_command_still_executes_it() -> None:
    # The flight computer commands OFF under the fault: the receiver goes off in the
    # next tick (the fault never overrides towards "more on"), and the frame is lost.
    rig = Rig()
    rig.target.inject(fault(TRANSMITTER_OFF))
    rig.tick(set_radio(RadioMode.OFF))
    rig.tick(ping(2))
    assert rig.last.controls.radio.mode is RadioMode.OFF
    assert rig.last.traffic.uplink_lost_count == 1
    assert rig.fc.calls[-1].uplink_frames == ()


def attitude_power(tick: SilTick) -> float:
    return tick.state.get("attitude", AttitudeSnapshot).truth.control_power_w


# --- sensor_freeze ------------------------------------------------------------------------


def test_sensor_freeze_holds_every_sensor_reading_and_releases_it() -> None:
    rig = Rig()  # nominal noise: live readings change every tick
    rig.run(3)
    before = rig.last
    rig.target.inject(fault(SENSOR_FREEZE, duration_us=4 * TICK))
    during = rig.run(4)
    after = rig.run(2)

    for t in during:
        assert t.controls.frozen_sensors == SENSOR_SUBSYSTEMS
        for subsystem in SENSOR_SUBSYSTEMS:
            assert readings_of(t.state, subsystem) == readings_of(before.state, subsystem)
    # True values keep evolving underneath.
    assert power(during[-1].state).truth != power(before.state).truth
    assert thermal(during[-1].state).truth != thermal(before.state).truth
    # Released: live readings resume.
    assert after[0].controls.frozen_sensors == frozenset()
    for subsystem in SENSOR_SUBSYSTEMS:
        assert readings_of(after[0].state, subsystem) != readings_of(before.state, subsystem)


@pytest.mark.parametrize("subsystem", sorted(SENSOR_SUBSYSTEMS))
def test_sensor_freeze_of_one_subsystem_leaves_the_others_live(subsystem: str) -> None:
    rig = Rig()
    rig.run(2)
    before = rig.last
    rig.target.inject(fault(SENSOR_FREEZE, duration_us=2 * TICK, subsystem=subsystem))
    during = rig.run(2)
    for t in during:
        assert t.controls.frozen_sensors == {subsystem}
        assert readings_of(t.state, subsystem) == readings_of(before.state, subsystem)
        for other in SENSOR_SUBSYSTEMS - {subsystem}:
            assert readings_of(t.state, other) != readings_of(before.state, other)


def test_overlapping_sensor_freezes_combine_and_release_separately() -> None:
    rig = Rig()
    rig.tick()
    rig.target.inject(fault(SENSOR_FREEZE, duration_us=4 * TICK, subsystem="power"))
    rig.target.inject(fault(SENSOR_FREEZE, duration_us=2 * TICK))
    rig.tick()
    rig.target.inject(fault(SENSOR_FREEZE, duration_us=TICK, subsystem="thermal"))
    ticks = [rig.last, *rig.run(4)]
    assert [t.controls.frozen_sensors for t in ticks] == [
        SENSOR_SUBSYSTEMS,
        SENSOR_SUBSYSTEMS,
        {"power"},
        {"power"},
        frozenset(),
    ]


# --- battery_drain ------------------------------------------------------------------------


def test_battery_drain_adds_its_load_and_is_released() -> None:
    drained, control = Rig(), Rig()
    for rig in (drained, control):
        rig.run(2)
    drained.target.inject(fault(BATTERY_DRAIN, duration_us=10 * TICK, load_w=3.0))
    during = drained.run(10)
    control_during = control.run(10)
    for t, c in zip(during, control_during, strict=True):
        assert t.controls.extra_load_w == 3.0
        assert power(t.state).truth.total_load_w - power(c.state).truth.total_load_w == (
            pytest.approx(3.0)
        )
    assert power(during[-1].state).truth.soc < power(control_during[-1].state).truth.soc

    after = drained.run(2)
    control_after = control.run(2)
    for t, c in zip(after, control_after, strict=True):
        assert t.controls.extra_load_w == 0.0
        assert power(t.state).truth.total_load_w == pytest.approx(power(c.state).truth.total_load_w)


def test_overlapping_battery_drains_add_up() -> None:
    rig = Rig()
    rig.target.inject(fault(BATTERY_DRAIN, duration_us=3 * TICK, load_w=1.0))
    rig.target.inject(fault(BATTERY_DRAIN, duration_us=TICK, load_w=0.5))
    ticks = rig.run(4)
    assert [t.controls.extra_load_w for t in ticks] == [1.5, 1.0, 1.0, 0.0]


def test_battery_soc_override_beats_battery_drain_which_still_shows_in_the_current() -> None:
    # ADR-0004 §11: the override pins SOC; the drain still appears in the reported
    # current and the load. When the override clears, integration resumes from the
    # pinned value with the drain still active.
    pinned = EnvironmentState(battery_soc_override=0.6, sensor_noise_scale=0.0)
    drained, control = Rig(), Rig()
    for rig in (drained, control):
        rig.target.apply_environment(pinned)
    drained.target.inject(fault(BATTERY_DRAIN, load_w=4.0))
    during = drained.run(5)
    control_during = control.run(5)
    for t, c in zip(during, control_during, strict=True):
        truth, readings = power(t.state).truth, power(t.state).readings
        assert truth.soc == 0.6 == power(c.state).truth.soc
        assert truth.total_load_w - power(c.state).truth.total_load_w == pytest.approx(4.0)
        assert readings.battery_current_a < power(c.state).readings.battery_current_a

    for rig in (drained, control):
        rig.target.apply_environment(EnvironmentState(sensor_noise_scale=0.0))
    resumed = drained.run(5)
    control_resumed = control.run(5)
    assert power(resumed[0].state).truth.soc < 0.6
    assert power(resumed[-1].state).truth.soc < power(control_resumed[-1].state).truth.soc
    assert all(t.controls.extra_load_w == 4.0 for t in resumed)


# --- forced_reset -------------------------------------------------------------------------


def test_forced_reset_pulse_reboots_and_leaves_physical_state_unchanged() -> None:
    faulted, control = RealRig(), RealRig()
    for rig in (faulted, control):
        rig.target.advance(20 * TICK)
    assert faulted.fc.boot_count == 0
    faulted.target.inject(fault(FORCED_RESET))
    for rig in (faulted, control):
        rig.target.advance(5 * TICK)

    reboot_tick = faulted.ticks[20]
    assert reboot_tick.flight_computer_rebooted and not reboot_tick.flight_computer_held
    assert faulted.fc.boot_count == 1
    assert mode_of(faulted) is Mode.BOOT
    # Rebooted at the start of tick 20 (t = 2.0 s), then stepped at its end.
    assert faulted.fc.uptime_us == 5 * TICK
    assert control.fc.uptime_us == 25 * TICK
    # Battery SOC, temperatures, and every other physical value continue unchanged.
    for t, c in zip(faulted.ticks, control.ticks, strict=True):
        assert power(t.state).truth == power(c.state).truth
        assert thermal(t.state).truth == thermal(c.state).truth
        assert t.state == c.state
    assert power(reboot_tick.state).truth.soc != faulted.target.initial.power.soc
    assert not any(t.flight_computer_rebooted for t in faulted.ticks[21:])


NOMINAL_CONTROLS = controls_for_mode(Mode.NOMINAL)


def nominal_real_rig() -> RealRig:
    """The real flight computer with a 1 s boot, run until NOMINAL's controls apply."""
    rig = RealRig(config=SHORT_BOOT)
    rig.target.advance(12 * TICK)
    assert mode_of(rig) is Mode.NOMINAL
    assert rig.last.controls == NOMINAL_CONTROLS
    rig.target.receive()
    return rig


def test_forced_reset_pulse_reboots_in_the_injection_tick_and_boot_controls_follow() -> None:
    # A pulse reboots at the start of the injection tick (t = 1.2 s); the flight
    # computer then steps as usual in it, in BOOT: the ping is ACKed (PING is accepted
    # in every mode, #51) with the downlink sequence restarted at 0, and BOOT's
    # controls are produced at step f, so they apply from the next tick.
    rig = nominal_real_rig()
    rig.target.inject(fault(FORCED_RESET, duration_us=0))
    downlink = rig.tick(command_frame(Command.ping(), sequence=8))
    assert rig.last.flight_computer_rebooted
    assert rig.last.controls == NOMINAL_CONTROLS  # the controls in force when it began
    assert acks(downlink) == [CommandAck(8, CommandId.PING)]
    assert decode_frame(downlink[0]).sequence == 0
    assert rig.fc.boot_count == 1
    assert mode_of(rig) is Mode.BOOT
    assert rig.fc.uptime_us == TICK  # rebooted at 1.2 s, stepped at 1.3 s
    assert rig.last.next_controls == BOOT_CONTROLS
    rig.tick()
    # BOOT's controls, with the PING's ACK from the reboot tick handed to comms.
    assert rig.last.controls == dataclasses.replace(
        BOOT_CONTROLS, radio_traffic=RadioTraffic(sent_bytes=len(downlink[0]))
    )


def test_forced_reset_holds_the_flight_computer_for_its_duration() -> None:
    rig = nominal_real_rig()  # 12 ticks run: the next tick starts at 1.2 s
    rig.target.inject(fault(FORCED_RESET, duration_us=3 * TICK))

    held = [
        rig.tick(command_frame(Command.ping(), sequence=1)),
        rig.tick(command_frame(Command.ping(), sequence=2)),
        rig.tick(),
    ]
    assert held == [[], [], []]
    held_ticks = rig.ticks[-3:]
    for t in held_ticks:
        # No command handling, no downlink, no new controls; subsystems keep stepping
        # under the controls in force when the reset began.
        assert t.flight_computer_held and not t.flight_computer_rebooted
        assert t.controls == NOMINAL_CONTROLS and t.next_controls == NOMINAL_CONTROLS
        assert t.delivered_uplink == () and t.downlink_frames == ()
        assert t.traffic.sent_bytes == 0 and t.traffic.outbound_suppressed_count == 0
        assert attitude_power(t) > 0.0
    assert [t.uplink_dropped_in_reset_count for t in held_ticks] == [1, 1, 0]
    assert [t.traffic.uplink_lost_count for t in held_ticks] == [0, 0, 0]
    # Not stepped while held: nothing about the flight computer has changed yet.
    assert rig.fc.boot_count == 0
    assert mode_of(rig) is Mode.NOMINAL
    assert rig.fc.uptime_us == 12 * TICK

    # The hold ends at t = 1.5 s: the reboot completes at the start of that tick, and
    # the flight computer steps, ACKing a new ping (never the dropped ones) as the first
    # frame since the reboot. Uptime counts from leaving reset.
    downlink = rig.tick(command_frame(Command.ping(), sequence=3))
    assert rig.last.flight_computer_rebooted and not rig.last.flight_computer_held
    assert acks(downlink) == [CommandAck(3, CommandId.PING)]
    assert decode_frame(downlink[0]).sequence == 0
    assert rig.fc.boot_count == 1
    assert mode_of(rig) is Mode.BOOT
    assert rig.fc.uptime_us == TICK
    assert rig.last.controls == NOMINAL_CONTROLS
    assert rig.last.next_controls == BOOT_CONTROLS

    # The 1 s boot is measured from the end of the hold: BOOT_COMPLETE in the step at
    # 2.5 s (uptime 1 s), so BOOT's controls apply for 9 more ticks, then NOMINAL's.
    rig.target.advance(10 * TICK)
    after = rig.ticks[-10:]
    # Traffic handed to comms: the PING's ACK from the reboot tick, then nothing until
    # the boot-complete beacon (TELEMETRY_FRAME_SIZE bytes) of the ninth tick.
    assert [t.controls.radio_traffic.sent_bytes for t in after] == [len(downlink[0])] + [0] * 8 + [
        TELEMETRY_FRAME_SIZE
    ]
    assert [dataclasses.replace(t.controls, radio_traffic=NO_RADIO_TRAFFIC) for t in after] == [
        BOOT_CONTROLS
    ] * 9 + [NOMINAL_CONTROLS]
    assert attitude_power(after[0]) == 0.0
    assert mode_of(rig) is Mode.NOMINAL
    assert rig.fc.boot_count == 1


def test_forced_reset_hold_keeps_scripted_controls_and_reboots_at_the_hold_end() -> None:
    # The same hold with the scripted double, which records its reboot time and
    # whose controls can be forced, to show exactly which controls are held.
    rig = Rig(boot_controls=BOOT_CONTROLS)
    rig.tick()
    rig.fc.force_controls(ATTITUDE_ON)
    rig.tick()
    calls_before = len(rig.fc.calls)
    rig.target.inject(fault(FORCED_RESET, duration_us=3 * TICK))
    rig.target.advance(3 * TICK)
    assert all(t.controls == ATTITUDE_ON for t in rig.ticks[-3:])
    assert len(rig.fc.calls) == calls_before  # not stepped, so nothing sent
    assert rig.target.receive() == []
    assert rig.fc.reboots == []
    downlink = rig.tick(ping(9))
    assert rig.fc.reboots == [5 * TICK]
    assert len(rig.fc.calls) == calls_before + 1  # stepped again after the reboot
    assert [ack_sequence(frame) for frame in downlink] == [9]
    assert rig.last.next_controls == BOOT_CONTROLS


def test_forced_reset_hold_drops_uplink_only_while_the_receiver_is_on() -> None:
    radio_off = dataclasses.replace(BOOT_CONTROLS, radio=RadioControls(mode=RadioMode.OFF))
    rig = Rig(boot_controls=radio_off)
    rig.target.inject(fault(FORCED_RESET, duration_us=TICK))
    rig.tick(ping(1))
    assert rig.last.flight_computer_held
    assert rig.last.traffic.uplink_lost_count == 1
    assert rig.last.uplink_dropped_in_reset_count == 0


def test_real_flight_computer_reboots_once_after_a_hold() -> None:
    rig = RealRig()
    rig.target.advance(3 * TICK)
    rig.target.inject(fault(FORCED_RESET, duration_us=4 * TICK))
    rig.target.advance(4 * TICK)
    assert rig.fc.boot_count == 0  # still held
    assert rig.fc.uptime_us == 3 * TICK  # not stepped since the hold began
    rig.target.advance(TICK)
    assert rig.fc.boot_count == 1
    assert rig.fc.uptime_us == TICK
    rig.target.advance(10 * TICK)
    assert rig.fc.boot_count == 1


def test_overlapping_forced_resets_extend_the_hold_and_make_one_reboot() -> None:
    rig = Rig()
    rig.target.inject(fault(FORCED_RESET, duration_us=3 * TICK))
    rig.tick()
    rig.target.inject(fault(FORCED_RESET, duration_us=4 * TICK))  # ends at 5 ticks
    rig.tick()
    rig.target.inject(fault(FORCED_RESET))  # a pulse inside the hold is absorbed
    rig.run(4)
    assert [t.flight_computer_held for t in rig.ticks] == [True] * 5 + [False]
    assert [t.flight_computer_rebooted for t in rig.ticks] == [False] * 5 + [True]
    assert rig.fc.reboots == [5 * TICK]


def test_two_pulses_in_the_same_tick_are_one_reboot() -> None:
    rig = RealRig()
    rig.target.inject(fault(FORCED_RESET))
    rig.target.inject(fault(FORCED_RESET))
    rig.target.advance(2 * TICK)
    assert rig.fc.boot_count == 1


def test_active_faults_survive_a_forced_reset() -> None:
    # A physical failure survives a flight computer reboot; only reset(seed) clears it.
    rig = Rig()
    overrides = (
        fault(TRANSMITTER_OFF),
        fault(SENSOR_FREEZE, subsystem="thermal"),
        fault(BATTERY_DRAIN, duration_us=20 * TICK, load_w=2.0),
    )
    for override in overrides:
        rig.target.inject(override)
    rig.tick()
    rig.target.inject(fault(FORCED_RESET, duration_us=2 * TICK))
    rig.run(2)  # held
    rig.tick()  # rebooted
    rig.target.inject(fault(FORCED_RESET))
    rig.tick()  # rebooted again
    assert rig.fc.reboots == [3 * TICK, 4 * TICK]
    for t in rig.ticks:
        assert t.active_faults == overrides
        assert t.controls.radio.mode is RadioMode.RX_ONLY
        assert t.controls.frozen_sensors == {"thermal"}
        assert t.controls.extra_load_w == 2.0


def test_faults_injected_during_a_hold_act_on_the_held_controls() -> None:
    rig = Rig(boot_controls=ATTITUDE_ON)
    rig.target.inject(fault(FORCED_RESET, duration_us=5 * TICK))
    rig.tick()
    rig.target.inject(fault(BATTERY_DRAIN, duration_us=TICK, load_w=1.0))
    rig.target.inject(fault(TRANSMITTER_OFF, duration_us=2 * TICK))
    rig.tick()
    assert rig.last.flight_computer_held
    assert rig.last.controls == dataclasses.replace(
        ATTITUDE_ON, radio=RadioControls(mode=RadioMode.RX_ONLY), extra_load_w=1.0
    )
    assert rig.last.next_controls == ATTITUDE_ON


# --- Combined faults and determinism ------------------------------------------------------


def run_fault_campaign(seed: int, target: SilTarget | None = None) -> list[SilTick]:
    """A run with every fault, overlapping, on the real flight computer."""
    ticks: list[SilTick] = []
    if target is None:
        target = SilTarget(tick_observer=ticks.append)
    target.reset(seed)
    schedule = {
        2: fault(SENSOR_FREEZE, duration_us=30 * TICK),
        5: fault(BATTERY_DRAIN, duration_us=50 * TICK, load_w=2.5),
        10: fault(TRANSMITTER_OFF, duration_us=20 * TICK),
        12: fault(FORCED_RESET, duration_us=5 * TICK),
        40: fault(SENSOR_FREEZE, subsystem="attitude"),
        60: fault(FORCED_RESET),
    }
    for index in range(100):
        if index in schedule:
            target.inject(schedule[index])
        target.advance(TICK)
    return ticks


def test_faulted_runs_are_deterministic() -> None:
    first = run_fault_campaign(seed=11)
    assert first == run_fault_campaign(seed=11)
    assert first != run_fault_campaign(seed=12)  # the seed reaches the sensor noise
    assert any(t.flight_computer_held for t in first)
    assert sum(t.flight_computer_rebooted for t in first) == 2


def test_reset_after_a_faulted_run_reproduces_it() -> None:
    ticks: list[SilTick] = []
    target = SilTarget(tick_observer=ticks.append)
    run_fault_campaign(seed=5, target=target)
    first = list(ticks)
    ticks.clear()
    run_fault_campaign(seed=5, target=target)
    assert ticks == first
