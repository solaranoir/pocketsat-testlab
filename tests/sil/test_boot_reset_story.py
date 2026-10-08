"""The boot sequence and the RESET command in ``SilTarget`` (#49), in simulated time.

The real subsystems and the real :class:`FlightComputer`, with #51's dispatcher, run
behind ``SilTarget``. RESET and SET_MODE go in as COMMAND frames with ``send()`` and
their ACKs come back from ``receive()``, alongside the real telemetry (#55): nothing in
BOOT, then the boot-complete beacon. :class:`Recorder` only notes the flight computer's
mode after each step.
"""

import dataclasses
from typing import Any

from pocketsat.flight import FlightComputer, Mode, RejectReason, controls_for_mode
from pocketsat.flight.boot import DEFAULT_BOOT_CONFIG
from pocketsat.flight.safety import DEFAULT_SAFETY_CONFIG
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    Command,
    CommandAck,
    CommandId,
    Telemetry,
    decode_ack,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NO_RADIO_TRAFFIC,
    AttitudeSnapshot,
    PowerInitial,
    PowerSnapshot,
    RadioTraffic,
    SpacecraftControls,
    SpacecraftInitialState,
)
from pocketsat.targets.sil import SilTarget, SilTick

SEED = 3
TICK_US = 100_000
BOOT_TICKS = DEFAULT_BOOT_CONFIG.duration_us // TICK_US
N = DEFAULT_SAFETY_CONFIG.sustain_tick_count

BOOT_CONTROLS = controls_for_mode(Mode.BOOT)
NOMINAL_CONTROLS = controls_for_mode(Mode.NOMINAL)


def decided(controls: SpacecraftControls) -> SpacecraftControls:
    """``controls`` without the radio traffic ``SilTarget`` merges in (ADR-0007): what the
    flight computer decided and the faults overrode."""
    return dataclasses.replace(controls, radio_traffic=NO_RADIO_TRAFFIC)


class Recorder(FlightComputer):
    """The real flight computer, recording its mode after every step."""

    def __init__(self) -> None:
        self.modes: list[Mode] = []
        super().__init__()

    def step(self, *args: Any, **kwargs: Any) -> Any:
        out = super().step(*args, **kwargs)
        self.modes.append(self.mode)
        return out


class Rig:
    """A powered-on ``SilTarget`` with a :class:`Recorder`, keeping every tick's record
    and every downlink frame."""

    def __init__(self, initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE) -> None:
        self.fc = Recorder()
        self.records: list[SilTick] = []
        self.downlink: list[list[bytes]] = []
        self.target = SilTarget(
            initial=initial,
            tick_us=TICK_US,
            flight_computer_factory=lambda: self.fc,
            tick_observer=self.records.append,
        )
        self.target.reset(SEED)

    def run(self, ticks: int, *, send: Command | None = None, sequence: int = 1) -> None:
        """Run ``ticks`` ticks; ``send`` arrives in the first of them."""
        if send is not None:
            frame = Frame(FrameType.COMMAND, sequence, encode_command(send))
            self.target.send(encode_frame(frame))
        for _ in range(ticks):
            self.target.advance(TICK_US)
            self.downlink.append(self.target.receive())

    def acks(self, tick: int) -> list[CommandAck]:
        """The ACK frames sent in ``tick``, decoded."""
        frames = [decode_frame(frame) for frame in self.downlink[tick]]
        return [decode_ack(f.payload) for f in frames if f.frame_type is FrameType.ACK]

    def telemetry(self, tick: int) -> list[Telemetry]:
        """The TELEMETRY frames sent in ``tick``, decoded."""
        frames = [decode_frame(frame) for frame in self.downlink[tick]]
        return [decode_telemetry(f.payload) for f in frames if f.frame_type is FrameType.TELEMETRY]


def attitude_control_w(record: SilTick) -> float:
    return record.state.get("attitude", AttitudeSnapshot).truth.control_power_w


def soc(record: SilTick) -> float:
    return record.state.get("power", PowerSnapshot).truth.soc


def test_power_on_runs_boot_then_nominal_in_simulated_time() -> None:
    rig = Rig()
    rig.run(BOOT_TICKS + 30)
    controls = [decided(r.controls) for r in rig.records]
    assert controls == [BOOT_CONTROLS] * BOOT_TICKS + [NOMINAL_CONTROLS] * 30
    # The boot-complete beacon goes out in the last BOOT tick and is reported to comms
    # in the next one (ADR-0007); BOOT itself sends nothing.
    assert all(r.controls.radio_traffic is NO_RADIO_TRAFFIC for r in rig.records[:BOOT_TICKS])
    beacon_bytes = sum(len(frame) for frame in rig.downlink[BOOT_TICKS - 1])
    assert rig.records[BOOT_TICKS].controls.radio_traffic == RadioTraffic(beacon_bytes)
    assert rig.fc.modes == [Mode.BOOT] * (BOOT_TICKS - 1) + [Mode.NOMINAL] * 31
    # The boot completes at 5.0 s of simulated time.
    assert rig.records[BOOT_TICKS - 1].now_us == DEFAULT_BOOT_CONFIG.duration_us
    # BOOT's attitude control is off in the physics too, and on from tick 50.
    assert all(attitude_control_w(r) == 0.0 for r in rig.records[:BOOT_TICKS])
    assert all(attitude_control_w(r) > 0.0 for r in rig.records[BOOT_TICKS:])
    assert rig.fc.boot_count == 0
    assert rig.fc.uptime_ms == (BOOT_TICKS + 30) * TICK_US // 1000


def test_commands_are_nacked_during_boot_and_accepted_after_it() -> None:
    rig = Rig()
    rig.run(1, send=Command.set_mode(Mode.SCIENCE), sequence=1)
    assert rig.acks(0) == [CommandAck(1, CommandId.SET_MODE, RejectReason.BOOT_IN_PROGRESS)]
    rig.run(BOOT_TICKS - 1)
    rig.run(1, send=Command.set_mode(Mode.SCIENCE), sequence=2)
    assert rig.acks(-1) == [CommandAck(2, CommandId.SET_MODE)]
    assert rig.fc.modes[-3:] == [Mode.BOOT, Mode.NOMINAL, Mode.SCIENCE]
    # Telemetry: silent in BOOT, the boot-complete beacon (NOMINAL), and in the ACK
    # tick a frame showing the new mode, after the ACK (#55).
    assert all(rig.telemetry(tick) == [] for tick in range(BOOT_TICKS - 1))
    assert [t.mode for t in rig.telemetry(BOOT_TICKS - 1)] == [Mode.NOMINAL]
    assert [t.mode for t in rig.telemetry(-1)] == [Mode.SCIENCE]
    assert decode_frame(rig.downlink[-1][0]).frame_type is FrameType.ACK


def test_reset_command_is_acked_in_its_tick_then_boot_then_nominal() -> None:
    reset_tick = 100
    plain = Rig()
    plain.run(reset_tick + 1)
    rig = Rig()
    rig.run(reset_tick)
    assert rig.fc.modes[-1] is Mode.NOMINAL
    rig.run(BOOT_TICKS + 10, send=Command.reset(), sequence=77)

    # The ACK goes out in the RESET tick, and the flight computer is in BOOT after it.
    assert rig.acks(reset_tick) == [CommandAck(77, CommandId.RESET)]
    assert rig.fc.modes[reset_tick] is Mode.BOOT
    assert rig.telemetry(reset_tick) == []  # in BOOT after step d: no telemetry
    # Silent through the new boot, then the boot-complete beacon with boot count 1 and
    # uptime counted from the RESET.
    beacon_tick = reset_tick + BOOT_TICKS
    assert all(frames == [] for frames in rig.downlink[reset_tick + 1 : beacon_tick])
    [beacon] = rig.telemetry(beacon_tick)
    assert (beacon.mode, beacon.boot_count, beacon.uptime_ms) == (Mode.NOMINAL, 1, 5000)
    assert decode_frame(rig.downlink[beacon_tick][0]).sequence == 0  # restarted by RESET

    # Up to and including the RESET tick, the physics doesn't differ: RESET doesn't
    # touch the spacecraft. Only the next tick's controls do: BOOT's, from step f.
    assert rig.records[:reset_tick] == plain.records[:reset_tick]
    assert rig.records[reset_tick].state == plain.records[reset_tick].state
    assert plain.records[reset_tick].next_controls == NOMINAL_CONTROLS
    assert rig.records[reset_tick].next_controls == BOOT_CONTROLS

    # BOOT's controls for the boot duration after the RESET tick, then NOMINAL again.
    after = [decided(r.controls) for r in rig.records[reset_tick + 1 :]]
    assert after == [BOOT_CONTROLS] * BOOT_TICKS + [NOMINAL_CONTROLS] * 9
    assert rig.fc.modes[reset_tick + BOOT_TICKS - 1] is Mode.BOOT
    assert rig.fc.modes[reset_tick + BOOT_TICKS] is Mode.NOMINAL
    assert rig.fc.boot_count == 1
    assert rig.fc.uptime_us == (BOOT_TICKS + 9) * TICK_US

    # The battery carries on from where it was, not from its starting charge.
    records = rig.records
    assert abs(soc(records[reset_tick + 1]) - soc(records[reset_tick])) < 1e-4


def test_critical_battery_at_power_on_goes_from_boot_straight_to_safe() -> None:
    rig = Rig(dataclasses.replace(DEFAULT_INITIAL_STATE, power=PowerInitial(soc=0.08)))
    rig.run(3 * BOOT_TICKS)
    assert rig.records[0].state.get("power", PowerSnapshot).readings.critical_battery
    # SAFE in the N-th tick, well inside the boot duration, and never NOMINAL.
    assert rig.fc.modes == [Mode.BOOT] * (N - 1) + [Mode.SAFE] * (3 * BOOT_TICKS - N + 1)
    # SAFE's controls (attitude control on) from the next tick.
    assert [decided(r.controls) for r in rig.records[: N + 1]] == [BOOT_CONTROLS] * N + [
        controls_for_mode(Mode.SAFE)
    ]
    assert attitude_control_w(rig.records[N]) > 0.0
    # No boot-complete beacon: the first frame is SAFE's, sent in the SAFE entry tick,
    # then one every SAFE period (0.5 s, 5 ticks), all showing SAFE (#55).
    sent = [tick for tick in range(len(rig.downlink)) if rig.telemetry(tick)]
    assert sent == list(range(N - 1, 3 * BOOT_TICKS, 5))
    assert {t.mode for tick in sent for t in rig.telemetry(tick)} == {Mode.SAFE}
    assert rig.fc.boot_count == 0
    assert rig.fc.uptime_us == 3 * BOOT_TICKS * TICK_US
