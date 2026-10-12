"""Checks that docs/spacecraft.md stays in step with the code and the other docs (#65).

``docs/spacecraft.md`` carries a copy of the mode state diagram from
``docs/spacecraft-modes.md`` and a summary of the telemetry fields whose canonical table
is in ``docs/protocol.md``. These tests keep both from drifting: the two diagrams must
be identical, the diagram's arrows must be exactly the transitions of
``pocketsat.flight.transition``, the field summary must list the wire fields in order,
and every relative Markdown link on the page must point to a file that exists.
"""

import re
from pathlib import Path

from pocketsat.flight import (
    ALL_EVENTS,
    RETURN_MODES,
    EventKind,
    Mode,
    ModeEvent,
    ModeState,
    Outcome,
    transition,
)
from pocketsat.messages import TELEMETRY_FIELDS

DOCS = Path(__file__).resolve().parents[2] / "docs"
SPACECRAFT = DOCS / "spacecraft.md"
MODES = DOCS / "spacecraft-modes.md"

EDGE = re.compile(r"^\s*(\[\*\]|[A-Z_]+) --> ([A-Z_]+)(?:: (.*))?$")
"""One Mermaid state-diagram arrow: source, target, and optional label."""

type Edge = tuple[Mode, Mode, str]
"""An arrow as (from, to, event label), one per event named in the arrow's label."""


def _block(path: Path, name: str) -> str:
    text = path.read_text(encoding="utf-8")
    start, end = f"<!-- {name}:start -->", f"<!-- {name}:end -->"
    assert text.count(start) == 1 and text.count(end) == 1, f"{path.name} lacks {name} markers"
    return text.split(start, 1)[1].split(end, 1)[0].strip()


def _event_label(event: ModeEvent) -> str:
    return event.kind.name + (f" {event.target.name}" if event.target is not None else "")


def _diagram_edges(diagram: str) -> set[Edge]:
    # Each label lists its events separated by " / "; a parenthesis (which return mode)
    # or a bracket (the SAFE exit guard) qualifies the event before it.
    edges: set[Edge] = set()
    for line in diagram.splitlines():
        match = EDGE.match(line)
        if match is None or match.group(1) == "[*]":
            continue
        source, target, label = Mode[match.group(1)], Mode[match.group(2)], match.group(3)
        assert label, f"arrow without an event: {line.strip()}"
        for part in label.split(" / "):
            event = re.sub(r"\s*[(\[].*?[)\]]", "", part).strip()
            edges.add((source, target, event))
    return edges


def _code_edges() -> set[Edge]:
    # Every TRANSITION the state machine makes, from every state (DOWNLINK from both
    # return modes) with the SAFE exit guard both ways. RESET is drawn once, from
    # FAULT, and covered from the other modes by the diagram's note.
    edges: set[Edge] = set()
    for mode in Mode:
        for return_mode in sorted(RETURN_MODES) if mode is Mode.DOWNLINK else [None]:
            state = ModeState(mode, return_mode)
            for event in ALL_EVENTS:
                for allowed in (True, False):
                    result = transition(state, event, safe_exit_allowed=allowed)
                    if result.outcome is not Outcome.TRANSITION:
                        continue
                    if event.kind is EventKind.RESET and mode is not Mode.FAULT:
                        continue
                    edges.add((mode, result.mode, _event_label(event)))
    return edges


def test_spacecraft_md_embeds_the_mode_diagram_unchanged() -> None:
    assert _block(SPACECRAFT, "mode-diagram") == _block(MODES, "mode-diagram")


def test_mode_diagram_arrows_are_the_transitions_in_the_code() -> None:
    diagram = _block(MODES, "mode-diagram")
    assert diagram.startswith("```mermaid\nstateDiagram-v2\n")
    assert _diagram_edges(diagram) == _code_edges()


def test_mode_diagram_note_matches_reset_from_every_mode() -> None:
    diagram = _block(MODES, "mode-diagram")
    assert "RESET from any mode enters BOOT" in diagram
    reset = ModeEvent(EventKind.RESET)
    for mode in Mode:
        for return_mode in sorted(RETURN_MODES) if mode is Mode.DOWNLINK else [None]:
            result = transition(ModeState(mode, return_mode), reset, safe_exit_allowed=False)
            assert (result.outcome, result.mode) == (Outcome.TRANSITION, Mode.BOOT)


def test_telemetry_field_summary_lists_the_wire_fields_in_order() -> None:
    rows = [line for line in _block(SPACECRAFT, "telemetry-fields").splitlines()[2:]]
    documented = [row.split("|")[1].strip().strip("`") for row in rows]
    assert documented == [name for name, _ in TELEMETRY_FIELDS]


def test_relative_links_point_to_existing_files() -> None:
    text = SPACECRAFT.read_text(encoding="utf-8")
    targets = re.findall(r"\]\(([^)#:]+\.md)(?:#[^)]*)?\)", text)
    assert "power-thermal-budget.md" in targets and "protocol.md" in targets
    missing = [target for target in targets if not (DOCS / target).is_file()]
    assert missing == []
