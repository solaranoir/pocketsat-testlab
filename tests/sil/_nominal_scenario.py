"""The Phase 1 nominal scenario and its harness (#63), shared with the golden capture (#64).

A temporary stand-in for the Phase 5 scenario DSL and orchestrator: a Python timeline
and a loop that drives any :class:`~pocketsat.targets.base.TestTarget` **through the
TestTarget interface only** (ADR-0002), in ADR-0003's step order, with frame loopback
(no ground station or RF channel yet: what the target transmits is what the ground
receives). When the DSL arrives, the scenario becomes YAML and this module goes.

**The scenario** (:func:`nominal_timeline`) is #72's reference profile (SCIENCE for the
orbit except one 10-minute DOWNLINK pass at the end of eclipse, nominal parameter set,
default starting state) for one full ``NominalEnvironment`` orbit, sunlight and eclipse:

====================  =========================================================
Time                  Ground sends
====================  =========================================================
0 to 5 s              nothing: BOOT (no telemetry; the boot-complete beacon at 5 s)
5.5 s                 PING
6 s                   SET_MODE SCIENCE (as ``_reference_profile.ProfileGround``)
orbit end - 10 min    BEGIN_DOWNLINK: the flight computer sends every stored chunk,
                      then ``DOWNLINK_COMPLETE`` returns it to SCIENCE (#56)
orbit end             SET_MODE NOMINAL: the end of the pass window
orbit end + 2 s       end of the run (two NOMINAL telemetry periods)
====================  =========================================================

A command scheduled at ``t`` is sent before the tick that starts at ``t``, so the
flight computer handles it, and ACKs it, in that tick (ADR-0004 §2).

**The loop** (:func:`run_nominal_scenario`), once per tick, in ADR-0003 §2's order,
with time kept by the harness's own :class:`~pocketsat.core.clock.SimClock`:

2. sample ``NominalEnvironment`` at the harness clock (never a hand-built
   ``EnvironmentState`` sequence);
3. encode the commands due in this tick as COMMAND frames (sequence numbers 1, 2, ...);
4. ``target.send()`` each frame (loopback: no RF channel);
5. ``target.apply_environment(env)``;
6. ``target.advance(tick_us)``, exactly one tick;
7. to 9. ``target.receive()`` and decode every frame (ACK/NACK, TELEMETRY, DATA); an
   undecodable frame or an unexpected frame type raises ``AssertionError``;

then the clock advances one tick. Assertions (step 10) are evaluated by the caller over
the returned :class:`ScenarioRun`. Step 1 (faults) is empty in the nominal scenario.

**Interface for #64 (golden capture):**

``run_nominal_scenario(seed=SEED, *, until_us=None, target=None) -> ScenarioRun``
    Runs the scenario for ``seed`` on a fresh ``SilTarget()`` (nominal config, default
    start, real flight computer), or on ``target`` (any ``TestTarget``; it is
    connected and reset with ``seed``). ``until_us`` stops early (default: the whole
    scenario, :data:`RUN_US`); commands after it are not sent.

:class:`ScenarioRun` holds the run:

- ``downlink``: one :class:`TickDownlink` per tick that sent anything, oldest first:
  ``tick`` (index since ``reset``; the tick starts at ``tick * tick_us``), ``frames``
  (the raw bytes ``receive()`` returned, in transmit order), and the decoded
  ``acks``, ``telemetry`` and ``data``;
- ``frames``: every raw downlink frame, in order; ``downlink_bytes()``: the same,
  concatenated (frames are self-delimiting, sync word and length, so the
  concatenation is a lossless capture);
- ``uplink``: every ``(tick, raw COMMAND frame)`` sent, and ``timeline``,
  ``seed``, ``tick_us``, ``ticks_run``.

Same seed, same bytes: the run is deterministic (ADR-0003), and the bytes are portable
across platforms (ADR-0006).
"""

from dataclasses import dataclass
from typing import Final

from _reference_profile import PASS_US, SCIENCE_COMMAND_US

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock
from pocketsat.environment import NominalEnvironment
from pocketsat.flight import Mode
from pocketsat.flight.boot import DEFAULT_BOOT_CONFIG
from pocketsat.frame import MAX_SEQUENCE, Frame, FrameError, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    Command,
    CommandAck,
    DataChunk,
    Telemetry,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.targets.base import TestTarget
from pocketsat.targets.sil import SilTarget

SEED: Final = 42
"""The Phase 1 acceptance seed (#63)."""

TICK_US: Final = DEFAULT_TICK_US
"""The default 100 ms tick, never coarsened (#72: comms capacity is per tick)."""

US_PER_S: Final = 1_000_000

ENVIRONMENT: Final = NominalEnvironment()
"""The orbit: 92 minutes, 35% eclipse at the end of each orbit."""

ORBIT_US: Final = ENVIRONMENT.orbit_period_us

BOOT_US: Final = DEFAULT_BOOT_CONFIG.duration_us
"""The default 5 s boot (#49): the beacon goes out in the tick that ends at 5 s."""

PING_US: Final = BOOT_US + US_PER_S // 2
"""PING, half a second after the boot-complete beacon."""

PASS_START_US: Final = ORBIT_US - PASS_US
"""BEGIN_DOWNLINK: 10 minutes before the end of the orbit, the end of eclipse (#72)."""

PASS_END_US: Final = ORBIT_US
"""The end of the 10-minute pass window, where the ground returns to NOMINAL."""

SETTLE_US: Final = 2 * US_PER_S
"""Run on after SET_MODE NOMINAL: two telemetry periods, so the first NOMINAL frame
after the mode-change frame shows the physical effect (ADR-0004 latencies)."""

RUN_US: Final = PASS_END_US + SETTLE_US
"""The whole scenario: one orbit plus :data:`SETTLE_US`."""


@dataclass(frozen=True)
class ScheduledCommand:
    """One command of the timeline.

    Attributes:
        at_us: Simulated time the command is sent: before the tick starting then, a
            multiple of the tick.
        command: What is sent.
        name: For reports.
    """

    at_us: int
    command: Command
    name: str


def nominal_timeline() -> tuple[ScheduledCommand, ...]:
    """The nominal scenario's commands, in time order (see the module docstring)."""
    return (
        ScheduledCommand(PING_US, Command.ping(), "PING"),
        ScheduledCommand(SCIENCE_COMMAND_US, Command.set_mode(Mode.SCIENCE), "SET_MODE SCIENCE"),
        ScheduledCommand(PASS_START_US, Command.begin_downlink(), "BEGIN_DOWNLINK"),
        ScheduledCommand(PASS_END_US, Command.set_mode(Mode.NOMINAL), "SET_MODE NOMINAL"),
    )


@dataclass(frozen=True)
class TickDownlink:
    """What one tick sent to the ground, raw and decoded, in transmit order per type.

    Attributes:
        tick: Index of the tick since ``reset(seed)``; it runs from
            ``tick * tick_us`` to ``(tick + 1) * tick_us``.
        frames: The raw frames ``receive()`` returned after the tick.
        acks: ACK and NACK payloads.
        telemetry: TELEMETRY payloads.
        data: DATA payloads (chunk ID and content, not yet checked).
    """

    tick: int
    frames: tuple[bytes, ...]
    acks: tuple[CommandAck, ...] = ()
    telemetry: tuple[Telemetry, ...] = ()
    data: tuple[DataChunk, ...] = ()


def decode_tick(tick: int, raw: list[bytes]) -> TickDownlink:
    """Decode one tick's downlink (step 9). Every frame must be a valid wire frame of
    type ACK, TELEMETRY, or DATA.

    Raises:
        AssertionError: A frame does not decode, or has another type.
    """
    acks: list[CommandAck] = []
    telemetry: list[Telemetry] = []
    data: list[DataChunk] = []
    for frame_bytes in raw:
        try:
            frame = decode_frame(frame_bytes)
        except FrameError as exc:
            raise AssertionError(f"tick {tick}: undecodable downlink frame: {exc}") from exc
        if frame.frame_type is FrameType.TELEMETRY:
            telemetry.append(decode_telemetry(frame.payload))
        elif frame.frame_type is FrameType.DATA:
            data.append(decode_data(frame.payload))
        elif frame.frame_type is FrameType.ACK:
            acks.append(decode_ack(frame.payload))
        else:
            raise AssertionError(f"tick {tick}: unexpected downlink frame {frame.frame_type!r}")
    return TickDownlink(tick, tuple(raw), tuple(acks), tuple(telemetry), tuple(data))


@dataclass(frozen=True)
class ScenarioRun:
    """One run of the nominal scenario, as the ground saw it.

    Attributes:
        seed: The seed passed to ``reset``.
        tick_us: The tick length.
        ticks_run: Ticks advanced.
        timeline: The commands scheduled (those after the end of the run were not sent).
        uplink: Every ``(tick, raw COMMAND frame)`` sent, in order.
        downlink: Every tick that sent anything, oldest first (silent ticks are left
            out; :attr:`TickDownlink.tick` gives each one's index).
    """

    seed: int
    tick_us: int
    ticks_run: int
    timeline: tuple[ScheduledCommand, ...]
    uplink: tuple[tuple[int, bytes], ...]
    downlink: tuple[TickDownlink, ...]

    @property
    def frames(self) -> tuple[bytes, ...]:
        """Every raw downlink frame, in transmit order."""
        return tuple(frame for tick in self.downlink for frame in tick.frames)

    def downlink_bytes(self) -> bytes:
        """Every downlink frame concatenated: a lossless, self-delimiting capture."""
        return b"".join(self.frames)

    @property
    def telemetry(self) -> list[tuple[int, Telemetry]]:
        """``(tick, frame)`` for every telemetry frame, oldest first."""
        return [(tick.tick, t) for tick in self.downlink for t in tick.telemetry]

    @property
    def data(self) -> list[tuple[int, DataChunk]]:
        """``(tick, chunk)`` for every DATA frame, oldest first."""
        return [(tick.tick, d) for tick in self.downlink for d in tick.data]

    @property
    def acks(self) -> list[tuple[int, CommandAck]]:
        """``(tick, ACK or NACK)`` for every ACK frame, oldest first."""
        return [(tick.tick, a) for tick in self.downlink for a in tick.acks]

    def tick_of(self, at_us: int) -> int:
        """The index of the tick that starts at ``at_us``."""
        assert at_us % self.tick_us == 0, f"{at_us} us is not on a tick boundary"
        return at_us // self.tick_us


def run_nominal_scenario(
    seed: int = SEED, *, until_us: int | None = None, target: TestTarget | None = None
) -> ScenarioRun:
    """Run the nominal scenario for ``seed`` and return what the ground received.

    Args:
        seed: The run's master seed (ADR-0003).
        until_us: Stop after this much simulated time (a multiple of the tick);
            default the whole scenario, :data:`RUN_US`.
        target: The target to drive; default a fresh ``SilTarget()`` (nominal config,
            default starting state, real flight computer). It is connected and reset
            with ``seed`` here, and driven only through ``TestTarget``.

    Returns:
        The run: raw and decoded downlink per tick, and the uplink sent.
    """
    run_us = RUN_US if until_us is None else until_us
    assert run_us % TICK_US == 0, f"until_us {run_us} is not a whole number of ticks"
    timeline = nominal_timeline()
    schedule: dict[int, list[Command]] = {}
    for scheduled in timeline:
        assert scheduled.at_us % TICK_US == 0, scheduled
        schedule.setdefault(scheduled.at_us // TICK_US, []).append(scheduled.command)

    if target is None:
        target = SilTarget(tick_us=TICK_US)
    target.connect()
    target.reset(seed)
    clock = SimClock(TICK_US)
    environment = ENVIRONMENT
    sequence = 0
    uplink: list[tuple[int, bytes]] = []
    downlink: list[TickDownlink] = []
    send, apply_environment, advance, receive = (
        target.send,
        target.apply_environment,
        target.advance,
        target.receive,
    )
    for n in range(run_us // TICK_US):
        env = environment.sample(clock)  # step 2
        for command in schedule.get(n, ()):  # steps 3 and 4
            sequence = sequence % MAX_SEQUENCE + 1
            frame = encode_frame(Frame(FrameType.COMMAND, sequence, encode_command(command)))
            uplink.append((n, frame))
            send(frame)
        apply_environment(env)  # step 5
        advance(TICK_US)  # step 6
        raw = receive()  # steps 7 to 9, loopback
        if raw:
            downlink.append(decode_tick(n, raw))
        clock.advance_one_tick()
    return ScenarioRun(
        seed=seed,
        tick_us=TICK_US,
        ticks_run=clock.tick_index,
        timeline=timeline,
        uplink=tuple(uplink),
        downlink=tuple(downlink),
    )
