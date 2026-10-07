"""The given/when/then helper for the behavioural tests (#102).

:class:`Ground` drives a target **only through the TestTarget interface** (ADR-0002):
COMMAND frames go in with ``send()``, time passes with ``advance()`` one tick at a time,
environments and faults go in with ``apply_environment()`` and ``inject()``, and what
comes back from ``receive()`` is decoded into ACK/NACK, telemetry, and DATA. It never
touches a subsystem, the flight computer, or the controls.

A test reads as given / when / then::

    # Given a booted spacecraft in SCIENCE
    sat = Ground.sil()
    sat.boot()
    sat.command_accepted(Command.set_mode(Mode.SCIENCE))
    # When the battery is held critically low
    sat.apply_environment(EnvironmentState(battery_soc_override=0.05))
    safe = sat.run_until(lambda t: t.mode is Mode.SAFE, within_s=3)
    # Then telemetry reports SAFE
    assert TelemetryFlags.critical_battery in safe.flags

The one exception is :class:`TruthProbe` (``Ground.probe``): a read-only, test-only view
of ``SilTarget.last_tick`` (:class:`~pocketsat.targets.sil.SilTick`), the per-tick
record SilTarget already keeps (ADR-0004 §14). #102 allows it for the true value in
scenario 4; scenario 3 also reads the controls the subsystems obeyed from it, because
BOOT sends no telemetry, so "BOOT's controls from tick N+1" has no wire evidence. Tests
use it only in ``then`` steps, never to drive the run, and it exists only for a
``SilTarget``.
"""

from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from typing import Self

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.flight import Mode
from pocketsat.flight.boot import DEFAULT_BOOT_CONFIG
from pocketsat.frame import MAX_SEQUENCE, Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    Command,
    CommandAck,
    DataChunk,
    NackReason,
    Telemetry,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import SpacecraftControls, ThermalSnapshot, ThermalTruth
from pocketsat.targets.base import EnvironmentState, TargetFault, TestTarget
from pocketsat.targets.sil import SilTarget, SilTick

SEED = 102
"""Seed for every behavioural run (ADR-0003): the issue number, so it is easy to find."""

TICK_US = DEFAULT_TICK_US
"""The target's default tick, 100 ms. Every ``advance()`` is exactly one tick."""

TICKS_PER_S = 1_000_000 // TICK_US

BOOT_TICKS = -(-DEFAULT_BOOT_CONFIG.duration_us // TICK_US)
"""Ticks of BOOT after power-on at the default 5 s boot (#49): ticks 0 to 49, the last
of which raises ``BOOT_COMPLETE`` and sends the beacon."""


def ticks(seconds: float) -> int:
    """Whole ticks in ``seconds`` of simulated time (test durations are whole ticks)."""
    count = round(seconds * TICKS_PER_S)
    assert count == seconds * TICKS_PER_S, f"{seconds} s is not a whole number of ticks"
    return count


@dataclass(frozen=True)
class Downlink:
    """Everything one tick sent to the ground, decoded, in transmit order per type.

    Attributes:
        tick: Index of the tick since ``reset(seed)``, counted by :class:`Ground`
            (tick 0 is the first ``advance()``).
        frames: The raw frames ``receive()`` returned.
        acks: ACK and NACK payloads.
        telemetry: TELEMETRY payloads.
        data: DATA payloads.
    """

    tick: int
    frames: tuple[bytes, ...] = ()
    acks: tuple[CommandAck, ...] = ()
    telemetry: tuple[Telemetry, ...] = ()
    data: tuple[DataChunk, ...] = ()

    @property
    def answers(self) -> list[tuple[int, NackReason | None]]:
        """``(command ID, NACK reason or None for an ACK)`` per ACK frame, in order."""
        return [(ack.command_id, ack.reason) for ack in self.acks]


def decode_downlink(tick: int, frames: list[bytes]) -> Downlink:
    """Decode one tick's frames. Every frame must be valid; unknown types fail the test."""
    acks: list[CommandAck] = []
    telemetry: list[Telemetry] = []
    data: list[DataChunk] = []
    for raw in frames:
        frame = decode_frame(raw)
        if frame.frame_type is FrameType.ACK:
            acks.append(decode_ack(frame.payload))
        elif frame.frame_type is FrameType.TELEMETRY:
            telemetry.append(decode_telemetry(frame.payload))
        elif frame.frame_type is FrameType.DATA:
            data.append(decode_data(frame.payload))
        else:
            raise AssertionError(f"unexpected downlink frame type {frame.frame_type!r}")
    return Downlink(tick, tuple(frames), tuple(acks), tuple(telemetry), tuple(data))


class TruthProbe:
    """Test-only, read-only view of ``SilTarget.last_tick`` (#102 scenario 4).

    The record of the tick just advanced: the controls the subsystems obeyed and every
    subsystem's true state. Used only to state what the wire cannot show.
    """

    def __init__(self, target: SilTarget) -> None:
        self._target = target

    @property
    def tick(self) -> SilTick:
        """The last tick's record."""
        record = self._target.last_tick
        assert record is not None, "no tick has run since reset(seed)"
        return record

    @property
    def controls(self) -> SpacecraftControls:
        """The merged controls the subsystems obeyed in the last tick (step a)."""
        return self.tick.controls

    @property
    def thermal_truth(self) -> ThermalTruth:
        """Thermal's true temperatures after the last tick's subsystem step."""
        return self.tick.state.get("thermal", ThermalSnapshot).truth


@dataclass
class Ground:
    """A target seen from the ground, through TestTarget only (#102).

    Built connected and reset with ``seed``. Every method that advances time does so
    one tick at a time and keeps each tick's decoded :class:`Downlink` in
    :attr:`history`.

    Attributes:
        target: The target under test.
        seed: The seed passed to ``reset``.
        history: Every tick's downlink since the reset, oldest first.
    """

    target: TestTarget
    seed: int = SEED
    history: list[Downlink] = field(default_factory=list)
    _sequence: int = 0

    def __post_init__(self) -> None:
        self.target.connect()
        self.target.reset(self.seed)

    @classmethod
    def sil(cls, seed: int = SEED) -> Self:
        """A ``SilTarget`` with its defaults (nominal config, real flight computer, 5 s
        boot), powered on: tick 0 is next, in BOOT."""
        return cls(SilTarget(), seed)

    # --- State the ground can see --------------------------------------------------------

    @property
    def ticks_run(self) -> int:
        """Ticks advanced since the reset; the next tick's index."""
        return len(self.history)

    @property
    def telemetry(self) -> list[Telemetry]:
        """Every telemetry frame received since the reset, oldest first."""
        return [t for downlink in self.history for t in downlink.telemetry]

    @property
    def last_telemetry(self) -> Telemetry:
        """The most recent telemetry frame."""
        received = self.telemetry
        assert received, "no telemetry received yet"
        return received[-1]

    @property
    def probe(self) -> TruthProbe:
        """The test-only truth view; only a ``SilTarget`` has one."""
        assert isinstance(self.target, SilTarget), "the truth probe needs a SilTarget"
        return TruthProbe(self.target)

    # --- Inputs -------------------------------------------------------------------------

    def send(self, *commands: Command) -> None:
        """Queue COMMAND frames for the next tick, with increasing sequence numbers."""
        for command in commands:
            self._sequence = (self._sequence + 1) % (MAX_SEQUENCE + 1)
            payload = encode_command(command)
            self.target.send(encode_frame(Frame(FrameType.COMMAND, self._sequence, payload)))

    def apply_environment(self, env: EnvironmentState) -> None:
        """Set the environment from the next tick on."""
        self.target.apply_environment(env)

    def inject(self, fault: TargetFault) -> None:
        """Inject a fault; it acts from the next tick on (ADR-0004 §2)."""
        self.target.inject(fault)

    # --- Time ---------------------------------------------------------------------------

    def tick(self, *commands: Command) -> Downlink:
        """Send ``commands`` (if any), advance one tick, and return what came back."""
        self.send(*commands)
        self.target.advance(TICK_US)
        downlink = decode_downlink(self.ticks_run, self.target.receive())
        self.history.append(downlink)
        return downlink

    def ticking(self, seconds: float) -> Iterator[Downlink]:
        """Advance ``seconds`` one tick at a time, yielding each tick's downlink, so a
        ``then`` step can look at every tick (the probe included)."""
        for _ in range(ticks(seconds)):
            yield self.tick()

    def run(self, seconds: float) -> list[Downlink]:
        """Advance ``seconds`` and return each tick's downlink."""
        return list(self.ticking(seconds))

    def run_until(self, condition: Callable[[Telemetry], bool], *, within_s: float) -> Telemetry:
        """Advance until a telemetry frame satisfies ``condition``; return that frame.

        Raises:
            AssertionError: No frame did within ``within_s`` seconds.
        """
        for downlink in self.ticking(within_s):
            for frame in downlink.telemetry:
                if condition(frame):
                    return frame
        raise AssertionError(f"no telemetry frame met the condition within {within_s} s")

    # --- Givens -------------------------------------------------------------------------

    def boot(self) -> Telemetry:
        """Run the power-on boot to its end and return the boot-complete beacon (#49,
        #55): the first telemetry frame, which reports NOMINAL."""
        assert self.ticks_run == 0, "boot() runs from power-on"
        beacon = self.run_until(lambda _: True, within_s=BOOT_TICKS / TICKS_PER_S)
        assert beacon.mode is Mode.NOMINAL, beacon
        return beacon

    def command_accepted(self, command: Command) -> Downlink:
        """Send ``command`` alone in the next tick and require an ACK in that tick."""
        downlink = self.tick(command)
        assert downlink.answers == [(command.command_id, None)], downlink.answers
        return downlink
