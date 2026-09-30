# PocketSat Test Lab

[![CI](https://github.com/solaranoir/pocketsat-testlab/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/solaranoir/pocketsat-testlab/actions/workflows/ci.yml)

A Python Software/Hardware-in-the-Loop (SIL/HIL) test platform for a simulated
spacecraft, a simulated RF channel, and an amateur-radio-style ground station.

## Project goal

Build the **test infrastructure** for a small spacecraft: scenario definition,
orchestration, fault injection, reproducible seeded runs, and reporting. The same
scenario runs unchanged against a Python spacecraft simulation (SIL) or a
microcontroller running flight software (HIL), because the scenario runner only ever
talks to the `TestTarget` interface.

Version 1 covers SIL and MCU HIL, a YAML scenario DSL, reusable fault injection,
seeded test campaigns, and CI.

## Architecture

> **Diagram placeholder.** A rendered architecture diagram will go here.
> Until then, see [`docs/architecture.md`](docs/architecture.md) for component
> boundaries and data flow.

Design decisions are recorded as ADRs in [`docs/adr/`](docs/adr/). The wire format is
described in [`docs/protocol.md`](docs/protocol.md).

## Development

Requires Python 3.12+ and [uv](https://docs.astral.sh/uv/).

```sh
uv sync
uv run pocketsat --version
uv run ruff check . && uv run ruff format --check . && uv run mypy src && uv run pytest
```

Contributors, human or AI, should read [`AGENTS.md`](AGENTS.md) first.

## License

[Apache License 2.0](LICENSE)
