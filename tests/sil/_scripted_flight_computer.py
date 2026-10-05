"""A scripted flight computer for testing ``SilTarget``'s wiring (#59).

The real :class:`~pocketsat.flight.FlightComputer` answers commands (#51) but does not
yet emit telemetry (#55, #56), and no real command switches the radio or attitude
control directly. To test the frame flow, the tick timing, and the radio rules now,
these tests give ``SilTarget`` this subclass through its ``flight_computer_factory``
seam. The real flight computer's commands through ``SilTarget`` are tested in
``test_command_dispatch_sil.py``.

It is deliberately small and is **not** a model of the real flight software:

- COMMAND frames carry a test-only opcode in the first payload byte:
  :data:`OP_PING` (ACK only), :data:`OP_RADIO` (next byte: 0 ``OFF``, 1 ``RX_ONLY``,
  2 ``RX_TX``), and :data:`OP_ATTITUDE` (next byte: 0 off, 1 on). Each command is ACKed
  (an ACK frame whose payload is the command's sequence number), and the radio and
  attitude commands change the controls it produces at step f, so they act in the
  next tick like any command (ADR-0004 §2).
- If ``telemetry`` is set, it emits a real TELEMETRY frame (#54 encoder) every tick
  after the ACKs, built from the readings it was given, so the downlink bytes carry the
  seeded sensor noise.
- It follows ADR-0004 §10's outbound rule: frames go out in priority order (ACK, then
  telemetry) while they fit ``transmit_capacity_bytes``; the rest are dropped and
  counted in ``outbound_suppressed_count``.
- It records every call, so tests can see exactly what ``SilTarget`` delivered, and
  every :meth:`reboot` (the ``forced_reset`` path, #60), after which it produces its
  boot controls again, as the real flight computer produces BOOT's.
"""

from collections.abc import Iterable
from dataclasses import dataclass, replace

from pocketsat.flight import (
    FlightComputer,
    FlightComputerOutput,
    Mode,
    SpacecraftReadings,
    controls_for_mode,
)
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import FlightComputerTelemetryState, encode_telemetry
from pocketsat.spacecraft import (
    AttitudeControls,
    RadioControls,
    RadioMode,
    SpacecraftControls,
)

OP_PING = 0x01
OP_RADIO = 0x10
OP_ATTITUDE = 0x20

RADIO_CODES = {0: RadioMode.OFF, 1: RadioMode.RX_ONLY, 2: RadioMode.RX_TX}
_RADIO_BYTES = {mode: code for code, mode in RADIO_CODES.items()}


def command(opcode: int, *args: int, sequence: int = 1) -> bytes:
    """Encode a test COMMAND frame for the scripted flight computer."""
    payload = bytes((opcode, *args))
    return encode_frame(Frame(frame_type=FrameType.COMMAND, sequence=sequence, payload=payload))


def ping(sequence: int = 1) -> bytes:
    """A PING command frame."""
    return command(OP_PING, sequence=sequence)


def set_radio(mode: RadioMode, sequence: int = 1) -> bytes:
    """A command frame setting the radio mode from the next tick."""
    return command(OP_RADIO, _RADIO_BYTES[mode], sequence=sequence)


def set_attitude(enabled: bool, sequence: int = 1) -> bytes:
    """A command frame switching attitude control from the next tick."""
    return command(OP_ATTITUDE, int(enabled), sequence=sequence)


def ack_sequence(frame: bytes) -> int | None:
    """The acknowledged sequence number if ``frame`` is an ACK, else ``None``."""
    decoded = decode_frame(frame)
    if decoded.frame_type is not FrameType.ACK:
        return None
    return int.from_bytes(decoded.payload, "big")


def is_telemetry(frame: bytes) -> bool:
    """Whether ``frame`` is a TELEMETRY frame."""
    return decode_frame(frame).frame_type is FrameType.TELEMETRY


@dataclass(frozen=True)
class StepCall:
    """One recorded :meth:`ScriptedFlightComputer.step` call."""

    uplink_frames: tuple[bytes, ...]
    readings: SpacecraftReadings
    now_us: int
    output: FlightComputerOutput


class ScriptedFlightComputer(FlightComputer):
    """A ``FlightComputer`` that obeys the test opcodes above. See the module docstring.

    Args:
        boot_controls: Controls returned by :meth:`reset` for tick 0, and the starting
            point for the controls it produces. Default: the real BOOT controls.
        telemetry: Emit a TELEMETRY frame every tick.
    """

    def __init__(
        self, boot_controls: SpacecraftControls | None = None, *, telemetry: bool = False
    ) -> None:
        self._boot_controls = (
            controls_for_mode(Mode.BOOT) if boot_controls is None else (boot_controls)
        )
        self._telemetry = telemetry
        self._controls = self._boot_controls
        self.calls: list[StepCall] = []
        self.reboots: list[int] = []
        self.reset_count = 0
        super().__init__()

    def reset(self, *, now_us: int = 0) -> SpacecraftControls:
        super().reset(now_us=now_us)
        self._controls = self._boot_controls
        self.calls = []
        self.reboots = []
        self.reset_count += 1
        return self._controls

    def reboot(self, *, now_us: int) -> None:
        """The RESET path (``forced_reset``, #60): the base reboot, then BOOT's controls
        at the next step f, as the real flight computer produces them (ADR-0004 §9).
        Records the reboot time."""
        super().reboot(now_us=now_us)
        self._controls = self._boot_controls
        self.reboots.append(now_us)

    def force_controls(self, controls: SpacecraftControls) -> None:
        """Make the next step produce ``controls`` (plus any command effects)."""
        self._controls = controls

    def step(
        self,
        uplink_frames: Iterable[bytes],
        readings: SpacecraftReadings,
        now_us: int,
    ) -> FlightComputerOutput:
        frames = tuple(uplink_frames)
        outbound: list[bytes] = []
        controls = self._controls
        for raw in frames:
            frame = decode_frame(raw)  # SilTarget passes only valid frames
            if frame.frame_type is not FrameType.COMMAND or not frame.payload:
                continue
            opcode, args = frame.payload[0], frame.payload[1:]
            if opcode == OP_RADIO:
                controls = replace(controls, radio=RadioControls(mode=RADIO_CODES[args[0]]))
            elif opcode == OP_ATTITUDE:
                controls = replace(controls, attitude=AttitudeControls(enabled=bool(args[0])))
            outbound.append(
                encode_frame(
                    Frame(
                        frame_type=FrameType.ACK,
                        sequence=frame.sequence,
                        payload=frame.sequence.to_bytes(2, "big"),
                    )
                )
            )
        if self._telemetry:
            state = FlightComputerTelemetryState(
                uptime_ms=now_us // 1000, mode=Mode.BOOT, boot_count=0
            )
            payload = encode_telemetry(
                state,
                power=readings.power,
                thermal=readings.thermal,
                attitude=readings.attitude,
                payload=readings.payload,
                comms=readings.comms,
            )
            outbound.append(
                encode_frame(Frame(frame_type=FrameType.TELEMETRY, sequence=0, payload=payload))
            )

        # ADR-0004 §10: priority order, within capacity; the rest suppressed and counted.
        capacity = readings.comms.transmit_capacity_bytes
        sent: list[bytes] = []
        suppressed = 0
        for frame_bytes in outbound:
            if len(frame_bytes) <= capacity:
                sent.append(frame_bytes)
                capacity -= len(frame_bytes)
            else:
                suppressed += 1

        self._controls = controls
        output = FlightComputerOutput(
            downlink_frames=tuple(sent),
            controls=controls,
            outbound_suppressed_count=suppressed,
        )
        self.calls.append(StepCall(frames, readings, now_us, output))
        return output
