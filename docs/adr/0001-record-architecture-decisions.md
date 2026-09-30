# ADR-0001: Record architecture decisions

- **Status:** Accepted
- **Phase:** 0

## Context

PocketSat Test Lab is developed by one maintainer working with AI coding agents. Decisions about interfaces, time, and determinism shape everything that follows, and they are easy to lose or quietly reverse when work is split across many small pull requests. Agents in particular need a written record to stay consistent with prior choices.

## Decision

We record significant architecture decisions as Architecture Decision Records (ADRs) in `docs/adr/`.

- **Format:** one Markdown file per decision, named `NNNN-short-title.md` with a zero-padded sequence number. Use `0000-template.md` as the starting point.
- **Contents:** status, context, decision, alternatives considered, consequences, and follow-ups.
- **Statuses:** Proposed, Accepted, Superseded by ADR-NNNN.
- **Immutability:** once Accepted, an ADR is not rewritten. A changed decision gets a new ADR that supersedes the old one, and the old ADR's status is updated to point to it. Small clarifications and follow-up notes are allowed.
- **When to write one:** a choice that affects more than one component, changes a public interface, constrains future phases, or that a future reader would reasonably ask "why did we do it this way?" about.
- **Process:** an ADR is proposed in a pull request. It is merged as Accepted by the maintainer. Agents must read the ADRs in `docs/adr/` before working on architecture-affecting issues and must write an ADR if an issue forces a design choice.

## Consequences

- Decisions and their reasoning live next to the code and are reviewed like code.
- Agents have a stable source of truth, which reduces drift between pull requests.
- There is a small overhead to writing ADRs, accepted for the decisions that matter.
