"""Tests for ``SilTarget`` (#59): wiring, tick order, frame flow, reset, and determinism.

Most wiring tests drive ``SilTarget`` with the real subsystems and a
:class:`ScriptedFlightComputer` (``_scripted_flight_computer.py``): the real flight
computer has no command that switches the radio or attitude control directly, and the
double records every call, so a test can compare what ``SilTarget`` delivered with what
the flight computer produced. The tests of downlink bytes (determinism, seeds, starting
states, the one-orbit run), full-duplex traffic, and capacity suppression use the real
flight computer and its real telemetry (#55); so do those that need only BOOT and the
controls path. Its command handling end to end is ``test_command_dispatch_sil.py``.

Not here, by design:

- Target faults are tested in ``test_sil_faults.py`` (#60). The fault half of the
  ADR-0004 §2 timeline ("a fault injected before tick N acts in tick N") is here, next
  to the command half.
- The ``TestTarget`` contract suite runs against ``SilTarget`` with the real flight
  computer (``tests/contract``, #61); the 10,000-tick determinism test and the seed and
  mid-run reset guards are in ``test_sil_determinism.py`` (#61).
- The per-tick traffic values are checked here in ``SilTick.traffic`` and
  ``SilTarget.handover_traffic``; their hand-over to comms (``RadioTraffic`` in the
  controls, comms' counters and draw, ADR-0007) in ``test_radio_traffic_sil.py``.
"""

import dataclasses
import itertools
import time

import pytest
from _scripted_flight_computer import (
    ScriptedFlightComputer,
    ack_sequence,
    is_telemetry,
    ping,
    set_attitude,
    set_radio,
)

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import (
    FlightComputer,
    FlightComputerConfig,
    Mode,
    SpacecraftReadings,
    controls_for_mode,
)
from pocketsat.flight.boot import BootConfig
from pocketsat.flight.telemetry import TelemetryConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    ACK_FRAME_SIZE,
    TELEMETRY_FRAME_SIZE,
    Command,
    DecodeReason,
    decode_ack,
    encode_command,
)
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NO_RADIO_TRAFFIC,
    NOMINAL_CONFIG,
    STEP_ORDER,
    AttitudeSnapshot,
    CommsSnapshot,
    PowerInitial,
    PowerSnapshot,
    RadioControls,
    RadioMode,
    RadioTraffic,
    SpacecraftControls,
    SpacecraftInitialState,
    ThermalSnapshot,
)
from pocketsat.targets import base
from pocketsat.targets.base import (
    EnvironmentState,
    TargetFault,
    UnsupportedFaultError,
)
from pocketsat.targets.sil import SilTarget, SilTick, TickTraffic

TICK = DEFAULT_TICK_US
QUIET = EnvironmentState(sensor_noise_scale=0.0)
BOOT_CONTROLS = controls_for_mode(Mode.BOOT)
RADIO_OFF = dataclasses.replace(BOOT_CONTROLS, radio=RadioControls(mode=RadioMode.OFF))


class Rig:
    """A ``SilTarget`` with a scripted flight computer and a record of every tick."""

    def __init__(
        self,
        *,
        boot_controls: SpacecraftControls | None = None,
        telemetry: bool = False,
        initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
        seed: int = 7,
    ) -> None:
        self.computers: list[ScriptedFlightComputer] = []
        self.ticks: list[SilTick] = []

        def factory() -> ScriptedFlightComputer:
            computer = ScriptedFlightComputer(boot_controls, telemetry=telemetry)
            self.computers.append(computer)
            return computer

        self.target = SilTarget(
            initial=initial, flight_computer_factory=factory, tick_observer=self.ticks.append
        )
        self.target.reset(seed)

    @property
    def fc(self) -> ScriptedFlightComputer:
        """The flight computer of the current run."""
        return self.computers[-1]

    def tick(self, *frames: bytes) -> list[bytes]:
        """Send ``frames``, run one tick, and return the downlink it produced."""
        for frame in frames:
            self.target.send(frame)
        self.target.advance(TICK)
        return self.target.receive()

    @property
    def last(self) -> SilTick:
        return self.ticks[-1]


def real_target(**kwargs: object) -> tuple[SilTarget, list[SilTick]]:
    ticks: list[SilTick] = []
    target = SilTarget(tick_observer=ticks.append, **kwargs)  # type: ignore[arg-type]
    return target, ticks


TELEMETRY_EVERY_TICK = FlightComputerConfig(
    boot=BootConfig(duration_us=0), telemetry=TelemetryConfig.uniform(TICK)
)
"""The real flight computer leaving BOOT in its first step and sending telemetry in
every tick after it (#55), so every tick has a TELEMETRY frame behind its ACKs."""


def telemetry_every_tick_target() -> tuple[SilTarget, list[SilTick]]:
    """A reset ``SilTarget`` with the real flight computer under
    :data:`TELEMETRY_EVERY_TICK`, and its tick records."""
    target, ticks = real_target(
        flight_computer_factory=lambda: FlightComputer(TELEMETRY_EVERY_TICK)
    )
    target.reset(7)
    return target, ticks


def real_ping(sequence: int) -> bytes:
    """A real PING COMMAND frame (#52)."""
    return encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(Command.ping())))


def frame_types(frames: list[bytes]) -> list[FrameType]:
    return [decode_frame(frame).frame_type for frame in frames]


def ack_sequences(frames: list[bytes]) -> list[int]:
    """The command sequence each real ACK frame among ``frames`` answers."""
    decoded = [decode_frame(frame) for frame in frames]
    return [decode_ack(f.payload).sequence for f in decoded if f.frame_type is FrameType.ACK]


# --- Interface, capabilities, construction ---------------------------------------------


def test_satisfies_the_testtarget_protocol_and_declares_capabilities() -> None:
    target = SilTarget()
    assert isinstance(target, base.TestTarget)
    caps = target.capabilities
    assert caps.deterministic is True
    assert caps.real_time is False
    assert caps.supported_faults == {
        "forced_reset",
        "sensor_freeze",
        "transmitter_off",
        "battery_drain",
    }


def test_defaults_are_the_nominal_config_and_default_starting_state() -> None:
    target = SilTarget()
    assert target.config is NOMINAL_CONFIG
    assert target.initial is DEFAULT_INITIAL_STATE
    assert target.tick_us == DEFAULT_TICK_US


def test_constructor_rejects_wrong_record_types() -> None:
    with pytest.raises(TypeError, match="config"):
        SilTarget(config=DEFAULT_INITIAL_STATE)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="initial"):
        SilTarget(initial=NOMINAL_CONFIG)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="tick_us"):
        SilTarget(tick_us=0)


def test_inject_rejects_unsupported_fault_types() -> None:
    target = SilTarget()
    target.reset(0)
    with pytest.raises(UnsupportedFaultError, match="attitude_control_failure") as info:
        target.inject(TargetFault(fault_type="attitude_control_failure"))
    assert info.value.supported == target.capabilities.supported_faults


def test_advance_requires_reset_first() -> None:
    target = SilTarget()
    with pytest.raises(RuntimeError, match="reset"):
        target.advance(TICK)


def test_connect_and_close_are_safe() -> None:
    target = SilTarget()
    target.connect()
    target.reset(0)
    target.send(ping())
    target.close()
    assert target.receive() == []


# --- Time: advance(dt_us) --------------------------------------------------------------


def test_advance_runs_one_tick_per_tick_us_and_rejects_partial_ticks() -> None:
    target, ticks = real_target()
    target.reset(0)
    target.advance(0)
    assert ticks == [] and target.now_us == 0
    target.advance(TICK)
    assert [t.now_us for t in ticks] == [TICK]
    target.advance(3 * TICK)
    assert [t.now_us for t in ticks] == [TICK, 2 * TICK, 3 * TICK, 4 * TICK]
    assert target.now_us == 4 * TICK
    with pytest.raises(ValueError, match="multiple of the tick"):
        target.advance(TICK + 1)
    with pytest.raises(ValueError, match="non-negative"):
        target.advance(-TICK)
    with pytest.raises(TypeError):
        target.advance(0.1)  # type: ignore[arg-type]
    assert target.now_us == 4 * TICK  # nothing ran


def test_custom_tick_length() -> None:
    target, ticks = real_target(tick_us=250_000)
    target.reset(0)
    target.advance(1_000_000)
    assert [t.now_us for t in ticks] == [250_000, 500_000, 750_000, 1_000_000]


def test_flight_computer_gets_end_of_tick_time() -> None:
    rig = Rig()
    rig.target.advance(2 * TICK)
    assert [call.now_us for call in rig.fc.calls] == [TICK, 2 * TICK]


def test_uplink_arrives_in_the_first_tick_of_a_multi_tick_advance() -> None:
    rig = Rig()
    rig.target.send(ping(1))
    rig.target.advance(3 * TICK)
    assert [len(call.uplink_frames) for call in rig.fc.calls] == [1, 0, 0]
    assert [ack_sequence(f) for f in rig.target.receive()] == [1]


# --- Tick order (ADR-0004 §2) ----------------------------------------------------------


def test_subsystems_step_before_the_flight_computer_which_reads_readings_only() -> None:
    rig = Rig()
    rig.target.apply_environment(EnvironmentState())  # noisy readings
    rig.target.advance(3 * TICK)
    for tick, call in zip(rig.ticks, rig.fc.calls, strict=True):
        assert tuple(tick.state.subsystems) == STEP_ORDER
        # The flight computer saw this tick's post-step readings, not truth.
        assert call.readings == SpacecraftReadings.from_state(tick.state)
        power = tick.state.get("power", PowerSnapshot)
        assert call.readings.power is power.readings


def test_boot_controls_apply_to_tick_zero() -> None:
    target, ticks = real_target()
    target.reset(0)
    target.advance(TICK)
    assert ticks[0].controls == BOOT_CONTROLS
    assert ticks[0].controls == FlightComputer().reset()
    # Their effect is visible in tick 0: BOOT has attitude control off.
    attitude = ticks[0].state.get("attitude", AttitudeSnapshot)
    assert attitude.truth.control_power_w == 0.0


def test_controls_from_tick_n_are_obeyed_in_tick_n_plus_1() -> None:
    rig = Rig()
    rig.tick()
    rig.tick(set_radio(RadioMode.RX_ONLY))
    rig.tick()
    first, second, third = rig.ticks
    assert second.next_controls.radio.mode is RadioMode.RX_ONLY
    assert second.controls.radio.mode is RadioMode.RX_TX
    # The flight computer's controls, with the previous tick's traffic merged in
    # (ADR-0007): the ACK sent in the second tick is reported in the third.
    ack_bytes = sum(len(frame) for frame in second.downlink_frames)
    assert ack_bytes > 0
    assert third.controls == dataclasses.replace(
        second.next_controls, radio_traffic=RadioTraffic(ack_bytes)
    )
    assert second.controls == first.next_controls


def test_merge_is_a_pass_through_without_faults() -> None:
    # Step a with no active fault and no traffic to hand over (BOOT sends nothing): the
    # subsystems obey exactly the controls the flight computer produced the tick before.
    target, ticks = real_target()
    target.reset(3)
    target.advance(5 * TICK)
    assert ticks[0].controls is BOOT_CONTROLS
    for before, after in itertools.pairwise(ticks):
        assert after.controls is before.next_controls


def test_merge_hands_the_previous_ticks_traffic_to_comms() -> None:
    # ADR-0007 §1: the merge replaces radio_traffic with the previous tick's traffic,
    # and changes nothing else; comms reports it in that tick.
    rig = Rig(telemetry=True)
    rig.tick(ping(sequence=1))
    rig.tick()
    first, second = rig.ticks
    sent = sum(len(frame) for frame in first.downlink_frames)
    assert len(first.downlink_frames) == 2  # the ACK and a telemetry frame
    assert first.traffic == TickTraffic(sent_bytes=sent)
    assert second.controls == dataclasses.replace(
        first.next_controls, radio_traffic=RadioTraffic(sent_bytes=sent)
    )
    comms = second.state.get("comms", CommsSnapshot).truth
    assert comms.previous_tick_sent_bytes == sent
    assert comms.transmit_power_w == NOMINAL_CONFIG.comms.transmitter_on_power_w + (
        NOMINAL_CONFIG.comms.transmit_power_per_byte_w * sent
    )


def test_timeline_ack_same_tick_effect_next_tick_power_two_ticks_later() -> None:
    # ADR-0004 §2: a command sent in tick N is ACKed in tick N, takes physical effect in
    # tick N+1, and power (stepped before attitude) sees the draw in tick N+2. The ACK's
    # own transmit energy reaches power in tick N+2 too (ADR-0007 §2).
    rig = Rig()
    rig.target.apply_environment(QUIET)
    for _ in range(3):
        rig.tick()
    downlink_n = rig.tick(set_attitude(True, sequence=9))
    rig.tick()
    rig.tick()
    tick_n, tick_n1, tick_n2 = rig.ticks[-3:]

    assert [ack_sequence(f) for f in downlink_n] == [9]

    def control_w(tick: SilTick) -> float:
        return tick.state.get("attitude", AttitudeSnapshot).truth.control_power_w

    def load_w(tick: SilTick) -> float:
        return tick.state.get("power", PowerSnapshot).truth.total_load_w

    assert control_w(tick_n) == 0.0
    assert control_w(tick_n1) == NOMINAL_CONFIG.attitude.control_power_w
    assert load_w(tick_n1) == load_w(tick_n)
    ack_w = NOMINAL_CONFIG.comms.transmit_power_per_byte_w * len(downlink_n[0])
    assert load_w(tick_n2) - load_w(tick_n1) == pytest.approx(
        NOMINAL_CONFIG.attitude.control_power_w + ack_w
    )


def test_timeline_fault_injected_before_tick_n_acts_in_tick_n() -> None:
    # ADR-0004 §2, the other half (#60): a fault injected before tick N is merged at
    # step a of tick N and acts in tick N itself, with no command latency. The real
    # flight computer (#51), in BOOT, where PING is accepted.
    target, ticks = real_target()
    target.reset(7)  # nominal sensor noise, so a frozen reading is visibly held
    target.advance(3 * TICK)
    tick_before = ticks[-1]
    target.inject(TargetFault("battery_drain", {"load_w": 2.0}))
    target.inject(TargetFault("transmitter_off"))
    target.inject(TargetFault("sensor_freeze", {"subsystem": "thermal"}))
    ping_9 = encode_frame(Frame(FrameType.COMMAND, 9, encode_command(Command.ping())))
    target.send(ping_9)
    target.advance(TICK)
    downlink_n = target.receive()
    tick_n = ticks[-1]

    # All three overrides are in tick N's merged controls...
    assert tick_n.controls.extra_load_w == 2.0
    assert tick_n.controls.radio.mode is RadioMode.RX_ONLY
    assert tick_n.controls.frozen_sensors == {"thermal"}
    # ...and act in tick N: power (first in STEP_ORDER) carries the drain, comms has
    # its transmitter off, so the ping is executed but its ACK is suppressed, and the
    # thermal readings hold tick N-1's values.
    power_before = tick_before.state.get("power", PowerSnapshot).truth
    power_n = tick_n.state.get("power", PowerSnapshot).truth
    assert power_n.total_load_w - power_before.total_load_w == pytest.approx(2.0)
    assert not tick_n.state.get("comms", CommsSnapshot).truth.transmitter_on
    assert tick_n.delivered_uplink == (ping_9,)
    assert downlink_n == []
    assert tick_n.traffic.outbound_suppressed_count == 1  # the PING's ACK
    thermal_before = tick_before.state.get("thermal", ThermalSnapshot)
    thermal_n = tick_n.state.get("thermal", ThermalSnapshot)
    assert thermal_n.readings == thermal_before.readings
    assert thermal_n.truth != thermal_before.truth


# --- Frame flow ------------------------------------------------------------------------


def test_receive_returns_frames_since_the_last_call() -> None:
    rig = Rig()
    assert rig.target.receive() == []
    rig.target.send(ping(1))
    rig.target.advance(TICK)
    rig.target.send(ping(2))
    rig.target.advance(TICK)
    assert [ack_sequence(f) for f in rig.target.receive()] == [1, 2]
    assert rig.target.receive() == []


def test_receive_returns_exactly_the_flight_computers_downlink_in_order() -> None:
    rig = Rig(telemetry=True)
    downlink = rig.tick(ping(4))
    assert downlink == list(rig.last.downlink_frames)
    assert downlink == list(rig.fc.calls[-1].output.downlink_frames)
    assert ack_sequence(downlink[0]) == 4 and is_telemetry(downlink[1])


def test_frames_sent_while_the_radio_is_off_are_lost_and_counted_not_queued() -> None:
    rig = Rig(boot_controls=RADIO_OFF)
    rig.tick(ping(1), ping(2))
    assert rig.fc.calls[0].uplink_frames == ()
    assert rig.last.traffic.uplink_lost_count == 2
    assert rig.last.undecodable_uplink_count == 0

    # Still off: lost again, and nothing from tick 0 was kept for later.
    rig.tick(ping(3))
    assert rig.fc.calls[1].uplink_frames == ()
    assert rig.last.traffic.uplink_lost_count == 1
    assert rig.target.receive() == []


def test_receiver_state_is_judged_after_the_subsystem_step() -> None:
    # The radio is OFF in ticks 0 and 1; the flight computer turns it on at step f of
    # tick 1, so it is on in tick 2. A frame sent before tick 2, while comms still
    # reported OFF, is received.
    rig = Rig(boot_controls=RADIO_OFF)
    rig.tick()
    rig.fc.force_controls(BOOT_CONTROLS)  # scripted: the next step f turns the radio on
    rig.tick()
    assert not rig.last.state.get("comms", CommsSnapshot).truth.receiver_on
    rig.tick(ping(5))
    assert rig.last.state.get("comms", CommsSnapshot).truth.receiver_on
    assert rig.fc.calls[-1].uplink_frames == (ping(5),)
    assert rig.last.traffic.uplink_lost_count == 0

    # And the reverse: the tick a commanded OFF takes effect, uplink is lost.
    rig.tick(set_radio(RadioMode.OFF))
    lost = rig.tick(ping(6))
    assert lost == []
    assert rig.last.state.get("comms", CommsSnapshot).truth.receiver_on is False
    assert rig.last.traffic.uplink_lost_count == 1


def test_rx_only_receives_and_executes_but_cannot_reply() -> None:
    # RX_ONLY is the mode the transmitter_off fault produces (ADR-0007 §5; the fault
    # itself is tested in test_sil_faults.py). Commands are still received and
    # executed, but the capacity is 0, so the ACK is suppressed by the flight computer
    # and counted; SilTarget passes the count on.
    rx_only = dataclasses.replace(BOOT_CONTROLS, radio=RadioControls(RadioMode.RX_ONLY))
    rig = Rig(boot_controls=rx_only)
    downlink = rig.tick(set_attitude(True, sequence=3))
    assert rig.fc.calls[0].uplink_frames == (set_attitude(True, sequence=3),)
    assert downlink == []
    assert rig.last.traffic == TickTraffic(
        sent_bytes=0, uplink_lost_count=0, outbound_suppressed_count=1
    )
    rig.tick()
    assert rig.last.state.get("attitude", AttitudeSnapshot).truth.control_power_w > 0.0


def test_full_duplex_commands_received_while_sending() -> None:
    # ADR-0004 §10: the radio receives and transmits in the same tick. The real flight
    # computer answers each PING and sends its telemetry behind the ACK (#55).
    target, ticks = telemetry_every_tick_target()
    for sequence in range(1, 6):
        target.send(real_ping(sequence))
        target.advance(TICK)
        downlink = target.receive()
        assert ticks[-1].delivered_uplink == (real_ping(sequence),)
        assert frame_types(downlink) == [FrameType.ACK, FrameType.TELEMETRY]
        assert ack_sequences(downlink) == [sequence]
        comms = ticks[-1].state.get("comms", CommsSnapshot).truth
        assert comms.receiver_on and comms.transmitter_on


GARBAGE: dict[str, bytes] = {
    "empty": b"",
    "noise": b"\x00\x01\x02",
    "bad_sync": b"\x00" * 12,
    "truncated": ping()[:-3],
    "bad_crc": ping()[:-1] + bytes([ping()[-1] ^ 0xFF]),
    "bad_type": encode_frame(Frame(frame_type=FrameType.ACK, sequence=1, payload=b""))[:3]
    + b"\x7f"
    + encode_frame(Frame(frame_type=FrameType.ACK, sequence=1, payload=b""))[4:],
}


@pytest.mark.parametrize("frame", GARBAGE.values(), ids=list(GARBAGE))
def test_undecodable_uplink_is_ignored_and_counted(frame: bytes) -> None:
    rig = Rig()
    downlink = rig.tick(frame, ping(2), frame)
    assert rig.fc.calls[-1].uplink_frames == (ping(2),)
    assert rig.last.undecodable_uplink_count == 2
    assert rig.last.traffic.uplink_lost_count == 0
    assert [ack_sequence(f) for f in downlink] == [2]


def test_undecodable_uplink_while_the_radio_is_off_counts_as_lost() -> None:
    # Undecodable frames are received frames (ADR-0007 §3); with the receiver off
    # nothing is received, so they are lost like any other frame.
    rig = Rig(boot_controls=RADIO_OFF)
    rig.tick(GARBAGE["noise"], ping())
    assert rig.last.undecodable_uplink_count == 0
    assert rig.last.traffic.uplink_lost_count == 2


def test_send_rejects_non_bytes() -> None:
    target = SilTarget()
    with pytest.raises(TypeError, match="bytes"):
        target.send("PING")  # type: ignore[arg-type]


# --- Per-tick traffic record (ADR-0007 hand-over, #98) ---------------------------------


def test_traffic_record_values_per_tick() -> None:
    rig = Rig(telemetry=True)
    assert rig.target.handover_traffic == TickTraffic()

    downlink = rig.tick(ping(1), ping(2))
    traffic = rig.last.traffic
    assert traffic.sent_bytes == sum(len(f) for f in downlink)
    assert traffic.sent_bytes == rig.fc.calls[-1].output.sent_bytes
    assert traffic.uplink_lost_count == 0
    assert traffic.outbound_suppressed_count == rig.fc.calls[-1].output.outbound_suppressed_count
    assert rig.target.handover_traffic is traffic

    # Per tick, not a running total: an idle tick reports only its own telemetry.
    downlink = rig.tick()
    assert rig.last.traffic.sent_bytes == sum(len(f) for f in downlink) < traffic.sent_bytes


def test_suppressed_count_is_passed_on_when_capacity_runs_out() -> None:
    # 120 bytes of capacity (#44) hold eight 14-byte ACKs (#51) with 8 bytes left. With
    # twelve commands, four ACKs and the 36-byte telemetry frame (#55) do not fit and
    # are suppressed by the real flight computer; SilTarget passes the count on.
    capacity = NOMINAL_CONFIG.comms.transmit_capacity_bytes
    assert (capacity // ACK_FRAME_SIZE, TELEMETRY_FRAME_SIZE) == (8, 36)
    target, ticks = telemetry_every_tick_target()
    for n in range(1, 13):
        target.send(real_ping(n))
    target.advance(TICK)
    downlink = target.receive()
    assert ack_sequences(downlink) == list(range(1, 9))
    assert frame_types(downlink) == [FrameType.ACK] * 8  # no room left for telemetry
    assert ticks[-1].traffic.outbound_suppressed_count == 5
    assert ticks[-1].traffic.sent_bytes == sum(len(f) for f in downlink) == 8 * ACK_FRAME_SIZE


def test_lost_count_is_in_the_handover_record() -> None:
    rig = Rig(boot_controls=RADIO_OFF)
    rig.tick(ping(), ping(), ping())
    assert rig.target.handover_traffic == TickTraffic(uplink_lost_count=3)


def test_reset_zeroes_the_handover_traffic() -> None:
    rig = Rig(telemetry=True)
    rig.tick(ping())
    assert rig.target.handover_traffic != TickTraffic()
    rig.target.reset(7)
    assert rig.target.handover_traffic == TickTraffic()
    assert rig.target.last_tick is None


# --- Environment -----------------------------------------------------------------------


def test_apply_environment_feeds_the_subsystems() -> None:
    rig = Rig()
    rig.tick()
    assert rig.last.state.get("power", PowerSnapshot).truth.generation_w > 0.0
    rig.target.apply_environment(EnvironmentState(sunlit=False))
    rig.tick()
    assert rig.last.environment == EnvironmentState(sunlit=False)
    assert rig.last.state.get("power", PowerSnapshot).truth.generation_w == 0.0


def test_reset_returns_to_the_nominal_environment() -> None:
    rig = Rig()
    rig.target.apply_environment(EnvironmentState(sunlit=False))
    rig.target.reset(7)
    rig.tick()
    assert rig.last.environment == EnvironmentState()


def test_apply_environment_rejects_other_types() -> None:
    with pytest.raises(TypeError, match="EnvironmentState"):
        SilTarget().apply_environment(object())  # type: ignore[arg-type]


# --- Reset, starting state, determinism ------------------------------------------------


def _run(target: SilTarget, seed: int, ticks: int) -> tuple[list[bytes], list[SilTick]]:
    """Reset under ``seed`` and run ``ticks`` ticks under a sampled orbit, sending a few
    commands; return the downlink and the tick records."""
    records: list[SilTick] = []
    target.reset(seed)
    env = NominalEnvironment()
    downlink: list[bytes] = []
    for n in range(ticks):
        if n % 50 == 10:
            target.send(ping(n))
        if n % 50 == 20:
            target.send(set_attitude(n % 100 == 20, sequence=n))
        target.apply_environment(env.state_at(target.now_us))
        target.advance(target.tick_us)
        assert target.last_tick is not None
        records.append(target.last_tick)
        downlink.extend(target.receive())
    return downlink, records


def test_reset_rebuilds_the_flight_computer_and_subsystems() -> None:
    rig = Rig()
    first = rig.fc
    rig.tick(set_attitude(True))
    rig.target.reset(7)
    assert rig.fc is not first and rig.fc.calls == []
    rig.tick()
    assert rig.last.controls == BOOT_CONTROLS  # the attitude command is forgotten


def test_reset_discards_queued_uplink_and_undrained_downlink() -> None:
    rig = Rig(telemetry=True)
    rig.target.advance(TICK)
    rig.target.send(ping())
    rig.target.reset(7)
    assert rig.target.receive() == []
    rig.tick()
    assert rig.fc.calls[0].uplink_frames == ()


def test_same_seed_gives_identical_downlink_bytes() -> None:
    # The real flight computer: ACK/NACK frames and its real telemetry (#55), which
    # carries the seeded sensor noise.
    first, _ = _run(SilTarget(), seed=11, ticks=600)
    second, _ = _run(SilTarget(), seed=11, ticks=600)
    assert frame_types(first).count(FrameType.TELEMETRY) >= 55  # 1 Hz after the boot
    assert first == second


def test_different_seed_gives_different_downlink_bytes() -> None:
    first, _ = _run(SilTarget(), seed=11, ticks=100)
    second, _ = _run(SilTarget(), seed=12, ticks=100)
    assert first != second


def test_reset_after_running_reproduces_the_first_run_exactly() -> None:
    target = SilTarget()
    downlink_a, ticks_a = _run(target, seed=5, ticks=400)
    _run(target, seed=99, ticks=123)  # something else in between
    downlink_b, ticks_b = _run(target, seed=5, ticks=400)
    assert downlink_a == downlink_b
    assert ticks_a == ticks_b  # controls, state, and traffic, tick by tick


def test_different_starting_states_diverge_and_reset_restores_each() -> None:
    low = SpacecraftInitialState(power=PowerInitial(soc=0.4))
    target_default = SilTarget()
    target_low = SilTarget(initial=low)
    assert target_low.initial is low

    _, default_ticks = _run(target_default, seed=5, ticks=20)
    _, low_ticks = _run(target_low, seed=5, ticks=20)

    def soc(tick: SilTick) -> float:
        return tick.state.get("power", PowerSnapshot).truth.soc

    assert soc(low_ticks[0]) < soc(default_ticks[0])
    assert soc(low_ticks[0]) == pytest.approx(0.4, abs=1e-3)
    _, low_again = _run(target_low, seed=5, ticks=20)
    assert low_again == low_ticks


def test_real_flight_computer_runs_and_is_reproducible() -> None:
    target = SilTarget()
    downlink_a, ticks_a = _run(target, seed=2, ticks=200)
    downlink_b, ticks_b = _run(target, seed=2, ticks=200)
    assert downlink_a == downlink_b
    # The real flight computer answers each command (#51): the scripted PING opcode is
    # the real PING (ACK); the scripted attitude opcode is an unknown command (NACK).
    frames = [decode_frame(frame) for frame in downlink_a]
    answers = [decode_ack(f.payload) for f in frames if f.frame_type is FrameType.ACK]
    assert [(a.command_id, a.reason) for a in answers] == [
        (0x01, None),
        (0x20, DecodeReason.UNKNOWN_COMMAND),
    ] * 4
    assert ticks_a == ticks_b
    # BOOT for the boot duration (5 s = 50 ticks), then NOMINAL (#49).
    nominal = controls_for_mode(Mode.NOMINAL)
    assert [dataclasses.replace(t.controls, radio_traffic=NO_RADIO_TRAFFIC) for t in ticks_a] == [
        BOOT_CONTROLS
    ] * 50 + [nominal] * 150


# --- Multi-orbit and performance -------------------------------------------------------

ORBIT_TICKS = 92 * 60 * 10  # NominalEnvironment's default 92-minute orbit at 100 ms
RUN_TICKS = ORBIT_TICKS + 600  # one orbit, ending in eclipse, then a minute of sunlight


@pytest.mark.slow
def test_one_orbit_is_deterministic_and_eclipse_reaches_the_subsystems() -> None:
    target = SilTarget()
    started = time.perf_counter()
    downlink_a, ticks_a = _run(target, seed=42, ticks=RUN_TICKS)
    per_tick_us = (time.perf_counter() - started) / RUN_TICKS * 1e6
    print(f"\nSilTarget, real flight computer with telemetry: {per_tick_us:.1f} us/tick")

    generation = [t.state.get("power", PowerSnapshot).truth.generation_w for t in ticks_a]
    sunlit = [t.environment.sunlit for t in ticks_a]
    assert sunlit[0] and not sunlit[ORBIT_TICKS - 1] and sunlit[-1]
    assert all(g == 0.0 for g, s in zip(generation, sunlit, strict=True) if not s)
    assert generation[0] > 0.0 and generation[-1] > 0.0  # stops in eclipse, resumes after

    downlink_b, _ = _run(target, seed=42, ticks=RUN_TICKS)
    assert downlink_a == downlink_b
