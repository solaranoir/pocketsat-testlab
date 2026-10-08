"""Recorded behavior digests: speed-ups must not change what the stack does (#121).

Each scenario drives ``SilTarget`` with the real flight computer (default 5 s boot) the
way the orchestrator does, and hashes two things tick by tick:

- **Downlink:** every frame ``receive()`` returned, with the tick it came out in.
- **State:** every tick's :class:`SpacecraftState` as values only (:func:`_state_values`):
  each subsystem's name, then its truth and readings field values in field order
  (seeded sensor noise included; enum members by value). Field names are not hashed,
  so renaming a field (as #98 renamed ``sent_bytes``) keeps the digests, while any
  changed value fails. Floats ``repr`` exactly, so equal digests mean bit-identical
  values.

The expected digests in :data:`RECORDED` were recorded on ``main`` at 4ad4a4e (after
#98), before #121's optimisations, so any change to the bytes sent or to any state
value, in any tick, fails here. A change that is *meant* to alter behavior (or one that
adds, removes or reorders snapshot fields) re-records them in the same PR and says so.

The scenarios cover what #121 touches: SCIENCE while acquiring (payload snapshot
rebuilds), a full DOWNLINK pass to ``DOWNLINK_COMPLETE`` (chunk content, DATA frames,
releases), commands with ACK, NACK and undecodable uplink (decode, dispatch, ACK
encoding), telemetry, and the four target faults including a reboot mid-pass.
"""

import hashlib
from collections.abc import Callable
from dataclasses import dataclass, fields
from enum import Enum
from typing import Final

import pytest

from pocketsat.environment import NominalEnvironment
from pocketsat.flight import Mode
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import Command, CommandId, decode_ack, decode_telemetry, encode_command
from pocketsat.spacecraft import (
    AttitudeInitial,
    PayloadInitial,
    PayloadSnapshot,
    PayloadState,
    SpacecraftInitialState,
    SpacecraftState,
)
from pocketsat.targets.base import TargetFault
from pocketsat.targets.sil import (
    BATTERY_DRAIN,
    FORCED_RESET,
    SENSOR_FREEZE,
    TRANSMITTER_OFF,
    SilTarget,
)

ORBIT: Final = NominalEnvironment(orbit_period_us=400_000_000)
"""A 400 s orbit (35% eclipse), so the runs see sunlight and eclipse."""

Script = Callable[[int], tuple[list[bytes], list[TargetFault]]]


def _command(command: Command, n: int) -> bytes:
    return encode_frame(Frame(FrameType.COMMAND, n & 0xFFFF, encode_command(command)))


def _fault(fault_type: str, duration_us: int | None, **params: float | str) -> TargetFault:
    return TargetFault(fault_type=fault_type, params=params, duration_us=duration_us)


def _science_pass_script(n: int) -> tuple[list[bytes], list[TargetFault]]:
    """SCIENCE (acquiring), a pass interrupted by faults and a reboot, a resumed pass
    that completes, then SCIENCE again. A PING every 7 ticks throughout, so ACKs meet
    telemetry and DATA in every combination."""
    frames: list[bytes] = []
    faults: list[TargetFault] = []
    if n % 7 == 3:
        frames.append(_command(Command.ping(), n))
    if n in (60, 1_000):
        frames.append(_command(Command.set_mode(Mode.SCIENCE), n))
    elif n in (400, 1_050):
        frames.append(_command(Command.begin_downlink(), n))
    elif n == 450:
        faults.append(_fault(TRANSMITTER_OFF, 1_000_000))
    elif n == 500:
        faults.append(_fault(SENSOR_FREEZE, 2_000_000, subsystem="thermal"))
    elif n == 600:
        faults.append(_fault(BATTERY_DRAIN, 5_000_000, load_w=1.0))
    elif n == 650:
        frames.append(_command(Command(0x7F), n))  # unknown command: NACK
        frames.append(_command(Command.ping(), n)[:-1] + b"\x00")  # bad CRC: dropped
    elif n == 900:
        faults.append(_fault(FORCED_RESET, None))  # reboot mid-pass
    return frames, faults


def _modes_script(n: int) -> tuple[list[bytes], list[TargetFault]]:
    """Every mode and command from the default (tumbling) start: NOMINAL, SCIENCE,
    DOWNLINK, SAFE, a held reset, a RESET command, a NACK, undecodable uplink, and
    the override faults."""
    frames: list[bytes] = []
    faults: list[TargetFault] = []
    if n == 5:
        frames.append(_command(Command.ping(), n))
    elif n == 60:
        frames.append(_command(Command.set_mode(Mode.SCIENCE), n))
    elif n == 120:
        frames.append(_command(Command.begin_downlink(), n))
    elif n == 130:
        faults.append(_fault(TRANSMITTER_OFF, 1_000_000))
        frames.append(_command(Command.ping(), n))
    elif n == 160:
        frames.append(_command(Command.set_mode(Mode.NOMINAL), n))
    elif n == 200:
        frames.append(_command(Command.enter_safe_mode(), n))
    elif n == 260:
        frames.append(_command(Command.set_mode(Mode.NOMINAL), n))
    elif n == 300:
        faults.append(_fault(FORCED_RESET, 500_000))
        frames.append(_command(Command.ping(), n))
    elif n == 400:
        frames.append(_command(Command.reset(), n))
    elif n == 420:
        frames.append(_command(Command.ping(), n))
    elif n in (480, 520):
        frames.append(_command(Command.set_mode(Mode.SCIENCE), n))
    elif n == 600:
        frames.append(_command(Command(0x7F), n))  # unknown command: NACK
        frames.append(_command(Command.ping(), n)[:-1] + b"\x00")  # bad CRC: dropped
    elif n == 700:
        faults.append(_fault(BATTERY_DRAIN, 20_000_000, load_w=1.5))
    elif n == 800:
        faults.append(_fault(SENSOR_FREEZE, 3_000_000, subsystem="power"))
    return frames, faults


@dataclass(frozen=True)
class Scenario:
    """A recorded run: where it starts, its script, and how long it runs."""

    initial: SpacecraftInitialState
    script: Script
    ticks: int


SCENARIOS: Final = {
    # Detumbled, so the payload acquires from the first SCIENCE tick; 600 chunks
    # stored so the pass lasts long enough to be interrupted and resumed.
    "science_pass": Scenario(
        SpacecraftInitialState(
            attitude=AttitudeInitial(pointing_error_deg=0.0, rate_dps=0.0),
            payload=PayloadInitial(buffer_fill=600 * 64 / 524_288),
        ),
        _science_pass_script,
        2_000,
    ),
    "modes": Scenario(SpacecraftInitialState(), _modes_script, 1_000),
}

RECORDED: Final = {
    # (scenario, seed): (downlink sha256, state sha256), recorded on main at 4ad4a4e.
    ("science_pass", 121): (
        "a141f3d3c3eee6cc2ebb45b08b89153667b51c9e9a5ac87c49f10317982c907c",
        "a791cfa14f3c192dbfa7c492991235cbbacc7af3ef65f731477fde8306742eb6",
    ),
    ("science_pass", 7): (
        "5d05b667c1fac2890abbccfaa852aca64a9a738daaeb583b708d762c82459c98",
        "8a24ce4010d19eb3c57224bf6c1d81809ec6c4c534f0ebd8426d49eb4d07ad3a",
    ),
    ("modes", 121): (
        "4ca6da4f1f2759133561e9a449df86c8536a70e5153599a24312ff7488f871c4",
        "2a80f2d33d78bc353d7579be2847b89189b29c852b778a51952557723cbf1ee6",
    ),
    ("modes", 7): (
        "f4479d41c04f19a429d271d199ccb39c22b75ba48118398e83b10b8bb503cf32",
        "34a8b495153dd11c449d4c14b0b2132dbfe84c4b70bc35f464a9648feacbf001",
    ),
}


@dataclass(frozen=True)
class Digests:
    """The digests of one run, and what it exercised (to guard against an empty run)."""

    downlink: str
    state: str
    frame_types: frozenset[FrameType]
    accepted: frozenset[bool]
    command_ids: frozenset[int]
    modes: frozenset[Mode]
    last_mode: Mode
    payload_states: frozenset[PayloadState]
    rebooted: bool
    faulted: bool


def _record_values(record: object) -> tuple[object, ...]:
    """A snapshot record's field values in field order; enum members by value."""
    values = (getattr(record, f.name) for f in fields(record))  # type: ignore[arg-type]
    return tuple(v.value if isinstance(v, Enum) else v for v in values)


def _state_values(state: SpacecraftState) -> tuple[object, ...]:
    """Every subsystem's name, truth values and readings values, in step order."""
    return tuple(
        (name, _record_values(snap.truth), _record_values(snap.readings))  # type: ignore[attr-defined]
        for name, snap in state.subsystems.items()
    )


def _run(scenario: Scenario, seed: int) -> Digests:
    target = SilTarget(initial=scenario.initial)
    target.connect()
    target.reset(seed)
    downlink, state = hashlib.sha256(), hashlib.sha256()
    frames: list[Frame] = []
    payload_states: set[PayloadState] = set()
    rebooted = faulted = False
    for n in range(scenario.ticks):
        uplink, faults = scenario.script(n)
        for fault in faults:
            target.inject(fault)
        for frame in uplink:
            target.send(frame)
        target.apply_environment(ORBIT.state_at(target.now_us))
        target.advance(target.tick_us)
        tick = target.last_tick
        assert tick is not None
        state.update(f"{n}:{_state_values(tick.state)!r}\n".encode())
        for frame in target.receive():
            downlink.update(n.to_bytes(4, "big") + len(frame).to_bytes(2, "big") + frame)
            frames.append(decode_frame(frame))
        payload_states.add(tick.state.get("payload", PayloadSnapshot).truth.state)
        rebooted |= tick.flight_computer_rebooted
        faulted |= bool(tick.active_faults)
    acks = [decode_ack(f.payload) for f in frames if f.frame_type is FrameType.ACK]
    telemetry = [decode_telemetry(f.payload) for f in frames if f.frame_type is FrameType.TELEMETRY]
    return Digests(
        downlink=downlink.hexdigest(),
        state=state.hexdigest(),
        frame_types=frozenset(f.frame_type for f in frames),
        accepted=frozenset(ack.accepted for ack in acks),
        command_ids=frozenset(ack.command_id for ack in acks if ack.accepted),
        modes=frozenset(t.mode for t in telemetry),
        last_mode=telemetry[-1].mode,
        payload_states=frozenset(payload_states),
        rebooted=rebooted,
        faulted=faulted,
    )


def _check_exercised(name: str, run: Digests) -> None:
    assert run.accepted == {True, False}
    assert run.rebooted and run.faulted
    if name == "science_pass":
        assert run.frame_types == {FrameType.ACK, FrameType.TELEMETRY, FrameType.DATA}
        assert PayloadState.ACQUIRING in run.payload_states
        assert run.modes == {Mode.NOMINAL, Mode.SCIENCE, Mode.DOWNLINK}
        assert run.last_mode is Mode.SCIENCE  # the resumed pass completed
    else:
        assert run.modes == {Mode.NOMINAL, Mode.SCIENCE, Mode.DOWNLINK, Mode.SAFE}
        assert CommandId.RESET in run.command_ids


@pytest.mark.parametrize(("name", "seed"), sorted(RECORDED))
def test_downlink_and_state_match_the_digests_recorded_before_121(name: str, seed: int) -> None:
    run = _run(SCENARIOS[name], seed)
    _check_exercised(name, run)
    assert (run.downlink, run.state) == RECORDED[(name, seed)]
