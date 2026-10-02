"""Tests for the mode state machine and the mode-to-controls mapping (#47).

The expected table below is written out by hand, independently of the implementation,
and covers every (mode, event) pair. DOWNLINK is checked from both return modes and the
SAFE exit guard both ways, so every cell is exercised under every input that matters.
The tables in docs/spacecraft-modes.md are checked against the code as well.
"""

import dataclasses
from collections.abc import Callable
from pathlib import Path

import pytest

import pocketsat.flight as flight
from pocketsat.flight import (
    ALL_EVENTS,
    COMMANDABLE_TARGETS,
    INITIAL_STATE,
    RETURN_MODES,
    EventKind,
    Mode,
    ModeEvent,
    ModeState,
    Outcome,
    RejectReason,
    Transition,
    controls_for_mode,
    transition,
)
from pocketsat.spacecraft import (
    AttitudeControls,
    PayloadControls,
    RadioControls,
    RadioMode,
    SpacecraftControls,
)

DOC = Path(__file__).resolve().parents[2] / "docs" / "spacecraft-modes.md"

B, N, S, D, SF, F = Mode.BOOT, Mode.NOMINAL, Mode.SCIENCE, Mode.DOWNLINK, Mode.SAFE, Mode.FAULT
BIP = RejectReason.BOOT_IN_PROGRESS
FRR = RejectReason.FAULT_REQUIRES_RESET
NAS = RejectReason.NOT_ALLOWED_IN_SAFE
SCA = RejectReason.SAFE_CONDITIONS_ACTIVE
TNC = RejectReason.TARGET_NOT_COMMANDABLE

SAME = "same"
"""Expected NO_CHANGE (a command accepted with an ACK, mode unchanged)."""

RETURN = "return"
"""Expected TRANSITION into DOWNLINK's return mode."""

GUARDED = "guarded"
"""Expected TRANSITION into NOMINAL if the SAFE flags have cleared, else REJECTED with
SAFE_CONDITIONS_ACTIVE."""

type Cell = Mode | RejectReason | str | None
"""A Mode is a TRANSITION into it, a RejectReason is REJECTED, None is IGNORED, and SAME,
RETURN, and GUARDED are as above."""


def _set(target: Mode) -> ModeEvent:
    return ModeEvent(EventKind.SET_MODE, target)


def _ev(kind: EventKind) -> ModeEvent:
    return ModeEvent(kind)


COLUMNS = (B, N, S, D, SF, F)

# fmt: off
EXPECTED: dict[ModeEvent, tuple[Cell, Cell, Cell, Cell, Cell, Cell]] = {
    #                                 BOOT  NOMINAL  SCIENCE  DOWNLINK  SAFE     FAULT
    _set(B):                         (BIP,  TNC,     TNC,     TNC,      TNC,     FRR),
    _set(N):                         (BIP,  SAME,    N,       N,        GUARDED, FRR),
    _set(S):                         (BIP,  S,       SAME,    S,        NAS,     FRR),
    _set(D):                         (BIP,  TNC,     TNC,     TNC,      TNC,     FRR),
    _set(SF):                        (BIP,  TNC,     TNC,     TNC,      TNC,     FRR),
    _set(F):                         (BIP,  TNC,     TNC,     TNC,      TNC,     FRR),
    _ev(EventKind.BEGIN_DOWNLINK):   (BIP,  D,       D,       SAME,     NAS,     FRR),
    _ev(EventKind.ENTER_SAFE_MODE):  (BIP,  SF,      SF,      SF,       SAME,    FRR),
    _ev(EventKind.RESET):            (B,    B,       B,       B,        B,       B),
    _ev(EventKind.BOOT_COMPLETE):    (N,    None,    None,    None,     None,    None),
    _ev(EventKind.SAFE_CONDITION):   (SF,   SF,      SF,      SF,       None,    None),
    _ev(EventKind.FAULT_DETECTED):   (F,    F,       F,       F,        F,       None),
    _ev(EventKind.DOWNLINK_COMPLETE):(None, None,    None,    RETURN,   None,    None),
}
# fmt: on

STATES = (
    ModeState(B),
    ModeState(N),
    ModeState(S),
    ModeState(D, return_mode=N),
    ModeState(D, return_mode=S),
    ModeState(SF),
    ModeState(F),
)
"""Every reachable state: each mode, with DOWNLINK once per return mode."""


def _expected(state: ModeState, event: ModeEvent, safe_exit_allowed: bool) -> Transition:
    cell = EXPECTED[event][COLUMNS.index(state.mode)]
    if cell == GUARDED:
        cell = N if safe_exit_allowed else SCA
    if cell == RETURN:
        assert state.return_mode is not None
        cell = state.return_mode
    if cell is None:
        return Transition(Outcome.IGNORED, state)
    if cell == SAME:
        return Transition(Outcome.NO_CHANGE, state)
    if isinstance(cell, RejectReason):
        return Transition(Outcome.REJECTED, state, cell)
    assert isinstance(cell, Mode)
    return_mode = state.mode if cell is D else None
    return Transition(Outcome.TRANSITION, ModeState(cell, return_mode))


def _state_id(state: ModeState) -> str:
    suffix = f"<{state.return_mode.name}" if state.return_mode is not None else ""
    return state.mode.name + suffix


def _event_id(event: ModeEvent) -> str:
    return event.kind.name + (f"_{event.target.name}" if event.target is not None else "")


CASES = [
    pytest.param(
        state,
        event,
        allowed,
        id=f"{_state_id(state)}-{_event_id(event)}-{'flags_clear' if allowed else 'flags_set'}",
    )
    for state in STATES
    for event in ALL_EVENTS
    for allowed in (True, False)
]


# --- Modes, events, reason codes ------------------------------------------------------


def test_mode_values_are_the_wire_encoding() -> None:
    # Telemetry (#54) and the SET_MODE argument (#52) carry these numbers as uint8.
    assert {m.name: m.value for m in Mode} == {
        "BOOT": 0,
        "NOMINAL": 1,
        "SCIENCE": 2,
        "DOWNLINK": 3,
        "SAFE": 4,
        "FAULT": 5,
    }


def test_reject_reason_values_are_fixed_uint8_codes() -> None:
    # NACK payload codes (#51, #52); 0x00-0x0F are left to #52's decoding errors.
    assert {r.name: r.value for r in RejectReason} == {
        "BOOT_IN_PROGRESS": 0x10,
        "FAULT_REQUIRES_RESET": 0x11,
        "NOT_ALLOWED_IN_SAFE": 0x12,
        "SAFE_CONDITIONS_ACTIVE": 0x13,
        "TARGET_NOT_COMMANDABLE": 0x14,
    }
    assert all(0x10 <= r.value <= 0x1F for r in RejectReason)


def test_every_reason_code_is_produced_by_some_cell() -> None:
    produced = {
        result.reason
        for state in STATES
        for event in ALL_EVENTS
        for allowed in (True, False)
        if (result := transition(state, event, safe_exit_allowed=allowed)).reason is not None
    }
    assert produced == set(RejectReason)


def test_all_events_lists_each_distinct_event_once() -> None:
    assert len(ALL_EVENTS) == len(set(ALL_EVENTS)) == len(Mode) + len(EventKind) - 1
    assert set(ALL_EVENTS) == set(EXPECTED)


def test_command_kinds() -> None:
    commands = {kind for kind in EventKind if kind.is_command}
    assert commands == {
        EventKind.SET_MODE,
        EventKind.BEGIN_DOWNLINK,
        EventKind.ENTER_SAFE_MODE,
        EventKind.RESET,
    }


def test_commandable_and_return_modes() -> None:
    assert frozenset({N, S}) == COMMANDABLE_TARGETS == RETURN_MODES


def test_initial_state_is_boot() -> None:
    assert (INITIAL_STATE.mode, INITIAL_STATE.return_mode) == (Mode.BOOT, None)


# --- The transition table, exhaustively -----------------------------------------------


@pytest.mark.parametrize(("state", "event", "safe_exit_allowed"), CASES)
def test_transition_table(state: ModeState, event: ModeEvent, safe_exit_allowed: bool) -> None:
    result = transition(state, event, safe_exit_allowed=safe_exit_allowed)
    assert result == _expected(state, event, safe_exit_allowed)


@pytest.mark.parametrize(("state", "event", "safe_exit_allowed"), CASES)
def test_commands_get_ack_or_nack_and_automatic_events_never_nack(
    state: ModeState, event: ModeEvent, safe_exit_allowed: bool
) -> None:
    result = transition(state, event, safe_exit_allowed=safe_exit_allowed)
    if event.kind.is_command:
        assert result.outcome is not Outcome.IGNORED
        assert result.acknowledged == (result.reason is None)
    else:
        assert result.outcome in (Outcome.TRANSITION, Outcome.IGNORED)
        assert result.reason is None


@pytest.mark.parametrize(("state", "event", "safe_exit_allowed"), CASES)
def test_state_is_unchanged_unless_the_mode_is_entered(
    state: ModeState, event: ModeEvent, safe_exit_allowed: bool
) -> None:
    result = transition(state, event, safe_exit_allowed=safe_exit_allowed)
    if result.outcome is not Outcome.TRANSITION:
        assert result.state is state


def test_the_guard_matters_only_for_set_mode_nominal_in_safe() -> None:
    differing = [
        (state, event)
        for state in STATES
        for event in ALL_EVENTS
        if transition(state, event, safe_exit_allowed=True)
        != transition(state, event, safe_exit_allowed=False)
    ]
    assert differing == [(ModeState(SF), _set(N))]


def test_fault_is_left_only_by_reset() -> None:
    for event in ALL_EVENTS:
        for allowed in (True, False):
            result = transition(ModeState(F), event, safe_exit_allowed=allowed)
            assert (result.mode is not F) == (event.kind is EventKind.RESET)


def test_safe_is_left_only_by_set_mode_nominal_reset_or_fault() -> None:
    leaving = {
        event
        for event in ALL_EVENTS
        if transition(ModeState(SF), event, safe_exit_allowed=True).mode is not SF
    }
    assert leaving == {_set(N), _ev(EventKind.RESET), _ev(EventKind.FAULT_DETECTED)}


def test_every_mode_is_reachable_from_power_on() -> None:
    seen = {INITIAL_STATE}
    frontier = [INITIAL_STATE]
    while frontier:
        state = frontier.pop()
        for event in ALL_EVENTS:
            for allowed in (True, False):
                new = transition(state, event, safe_exit_allowed=allowed).state
                if new not in seen:
                    seen.add(new)
                    frontier.append(new)
    assert seen == set(STATES)


def test_downlink_returns_to_the_mode_it_was_entered_from() -> None:
    state = INITIAL_STATE
    steps = [
        (_ev(EventKind.BOOT_COMPLETE), N),
        (_set(S), S),
        (_ev(EventKind.BEGIN_DOWNLINK), D),
        (_ev(EventKind.BEGIN_DOWNLINK), D),  # a retried command does not restart it
        (_ev(EventKind.DOWNLINK_COMPLETE), S),
        (_set(N), N),
        (_ev(EventKind.BEGIN_DOWNLINK), D),
        (_ev(EventKind.DOWNLINK_COMPLETE), N),
    ]
    for event, mode in steps:
        state = transition(state, event, safe_exit_allowed=True).state
        assert state.mode is mode
    assert state.return_mode is None


def test_safe_recovery_sequence() -> None:
    state = transition(
        ModeState(D, return_mode=S), _ev(EventKind.SAFE_CONDITION), safe_exit_allowed=False
    ).state
    assert state == ModeState(SF)
    rejected = transition(state, _set(N), safe_exit_allowed=False)
    assert (rejected.outcome, rejected.reason) == (Outcome.REJECTED, SCA)
    assert transition(state, _set(N), safe_exit_allowed=True).state == ModeState(N)


def test_transition_is_deterministic() -> None:
    first = [
        transition(s, e, safe_exit_allowed=a)
        for s in STATES
        for e in ALL_EVENTS
        for a in (True, False)
    ]
    second = [
        transition(s, e, safe_exit_allowed=a)
        for s in STATES
        for e in ALL_EVENTS
        for a in (True, False)
    ]
    assert first == second


# --- Record validation ----------------------------------------------------------------


def test_mode_event_requires_a_target_for_set_mode_only() -> None:
    with pytest.raises(ValueError, match="target"):
        ModeEvent(EventKind.SET_MODE)
    with pytest.raises(ValueError, match="target"):
        ModeEvent(EventKind.RESET, Mode.NOMINAL)
    with pytest.raises(TypeError, match="kind"):
        ModeEvent("reset")  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="target"):
        ModeEvent(EventKind.SET_MODE, 1)  # type: ignore[arg-type]


def test_mode_state_validates_return_mode() -> None:
    with pytest.raises(ValueError, match="return_mode"):
        ModeState(D)
    with pytest.raises(ValueError, match="return_mode"):
        ModeState(D, return_mode=SF)
    with pytest.raises(ValueError, match="return_mode"):
        ModeState(N, return_mode=N)
    with pytest.raises(TypeError, match="mode"):
        ModeState(1)  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="return_mode"):
        ModeState(D, return_mode=1)  # type: ignore[arg-type]


def test_transition_requires_a_reason_exactly_when_rejected() -> None:
    with pytest.raises(ValueError, match="reason"):
        Transition(Outcome.REJECTED, ModeState(N))
    with pytest.raises(ValueError, match="reason"):
        Transition(Outcome.NO_CHANGE, ModeState(N), BIP)


def test_records_are_frozen() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        INITIAL_STATE.mode = Mode.NOMINAL  # type: ignore[misc]


# --- Mode to controls -----------------------------------------------------------------

# fmt: off
EXPECTED_CONTROLS: dict[Mode, tuple[bool, RadioMode, bool]] = {
    #               payload  radio            attitude control
    Mode.BOOT:     (False,   RadioMode.RX_TX, False),
    Mode.NOMINAL:  (False,   RadioMode.RX_TX, True),
    Mode.SCIENCE:  (True,    RadioMode.RX_TX, True),
    Mode.DOWNLINK: (False,   RadioMode.RX_TX, True),
    Mode.SAFE:     (False,   RadioMode.RX_TX, True),
    Mode.FAULT:    (False,   RadioMode.RX_TX, True),
}
# fmt: on


@pytest.mark.parametrize("mode", list(Mode), ids=lambda m: m.name)
def test_controls_for_mode(mode: Mode) -> None:
    payload, radio, attitude = EXPECTED_CONTROLS[mode]
    assert controls_for_mode(mode) == SpacecraftControls(
        payload=PayloadControls(enabled=payload),
        radio=RadioControls(radio),
        attitude=AttitudeControls(enabled=attitude),
    )


@pytest.mark.parametrize("mode", list(Mode), ids=lambda m: m.name)
def test_controls_carry_no_fault_overrides_and_no_release(mode: Mode) -> None:
    controls = controls_for_mode(mode)
    assert controls.frozen_sensors == frozenset()
    assert controls.extra_load_w == 0.0
    assert controls.payload.release_through_chunk_id is None  # #56 owns release


def test_radio_is_rx_tx_in_every_mode() -> None:
    assert {controls_for_mode(m).radio.mode for m in Mode} == {RadioMode.RX_TX}


def test_attitude_control_is_off_only_in_boot() -> None:
    assert {m for m in Mode if not controls_for_mode(m).attitude.enabled} == {Mode.BOOT}


def test_payload_is_enabled_only_in_science() -> None:
    assert {m for m in Mode if controls_for_mode(m).payload.enabled} == {Mode.SCIENCE}


def test_initial_state_controls_are_boot_controls() -> None:
    # reset() applies BOOT's controls to tick 0 (ADR-0004 §2).
    assert controls_for_mode(INITIAL_STATE.mode) == controls_for_mode(Mode.BOOT)


# --- Public interface -----------------------------------------------------------------


def test_public_exports() -> None:
    assert sorted(flight.__all__) == sorted(
        [
            "ALL_EVENTS",
            "COMMANDABLE_TARGETS",
            "INITIAL_STATE",
            "RETURN_MODES",
            "EventKind",
            "Mode",
            "ModeEvent",
            "ModeState",
            "Outcome",
            "RejectReason",
            "Transition",
            "controls_for_mode",
            "transition",
        ]
    )
    for name in flight.__all__:
        assert getattr(flight, name) is not None


# --- docs/spacecraft-modes.md matches the code ----------------------------------------


def _cell_text(result: Transition) -> str:
    if result.outcome is Outcome.TRANSITION:
        return f"→ {result.mode.name}"
    if result.outcome is Outcome.NO_CHANGE:
        return "ACK, no change"
    if result.outcome is Outcome.REJECTED:
        assert result.reason is not None
        return f"NACK {result.reason.name}"
    return "ignored"


def _render_cell(mode: Mode, event: ModeEvent) -> str:
    # One cell, folding in the two inputs that can change it: DOWNLINK's return mode and
    # the SAFE exit guard.
    texts: dict[Mode | None, str] = {}
    for return_mode in sorted(RETURN_MODES) if mode is D else [None]:
        state = ModeState(mode, return_mode)
        clear = transition(state, event, safe_exit_allowed=True)
        active = transition(state, event, safe_exit_allowed=False)
        text = _cell_text(clear)
        if active != clear:
            text = f"{text} if flags cleared, else {_cell_text(active)}"
        texts[return_mode] = text
    if len(set(texts.values())) == 1:
        return next(iter(texts.values()))
    # Differs by return mode: allowed only for "go back to the return mode".
    assert all(text == f"→ {rm.name}" for rm, text in texts.items() if rm is not None), texts
    return "→ return mode"


def _render_transition_table() -> list[str]:
    lines = [
        "| Event | " + " | ".join(m.name for m in Mode) + " |",
        "|---" * (len(Mode) + 1) + "|",
    ]
    for event in ALL_EVENTS:
        label = event.kind.name + (f" {event.target.name}" if event.target is not None else "")
        cells = [_render_cell(mode, event) for mode in Mode]
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    return lines


def _render_controls_table() -> list[str]:
    lines = ["| Mode | Payload | Radio | Attitude control |", "|---|---|---|---|"]
    for mode in Mode:
        c = controls_for_mode(mode)
        payload = "enabled" if c.payload.enabled else "off"
        attitude = "on" if c.attitude.enabled else "off"
        lines.append(f"| {mode.name} | {payload} | {c.radio.mode.name} | {attitude} |")
    return lines


def _render_values_table() -> list[str]:
    lines = ["| Value | Mode |", "|---|---|"]
    lines += [f"| {m.value} | {m.name} |" for m in Mode]
    return lines


def _render_reasons_table() -> list[str]:
    lines = ["| Code | Reason |", "|---|---|"]
    lines += [f"| 0x{r.value:02X} | {r.name} |" for r in RejectReason]
    return lines


def _doc_block(name: str) -> list[str]:
    text = DOC.read_text(encoding="utf-8")
    start = f"<!-- {name}:start -->"
    end = f"<!-- {name}:end -->"
    assert start in text and end in text, f"{DOC.name} lacks the {name} markers"
    block = text.split(start, 1)[1].split(end, 1)[0]
    return [line for line in block.strip().splitlines() if line.startswith("|")]


@pytest.mark.parametrize(
    ("name", "render"),
    [
        ("mode-values", _render_values_table),
        ("transition-table", _render_transition_table),
        ("reason-codes", _render_reasons_table),
        ("mode-controls", _render_controls_table),
    ],
)
def test_doc_table_matches_code(name: str, render: Callable[[], list[str]]) -> None:
    expected = render()
    assert _doc_block(name) == expected, "\n" + "\n".join(expected)
