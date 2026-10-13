# Changelog

All notable changes to PocketSat Test Lab are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/). Before 1.0.0, each minor version marks a
completed roadmap phase (`docs/roadmap.md`); 1.0.0 is the end of the Version 1 plan.

## [0.1.0] - 2026-10-12

First tagged release: Phase 0 (architecture and skeleton) and Phase 1 (virtual
spacecraft). Phase 1's milestone is met: a nominal scenario runs through the
`TestTarget` interface and produces deterministic spacecraft telemetry.

### Added

**Phase 0: architecture and skeleton**
- Python project skeleton (`uv`, `ruff`, strict `mypy`, `pytest`), repository hygiene
  and `AGENTS.md` (#1, #2)
- `docs/architecture.md`: component boundaries and data flow (#3)
- ADR-0001 (recording decisions), ADR-0002 (`TestTarget` abstraction), ADR-0003 (time
  model and determinism) (#4, #5)
- The `TestTarget` protocol, core message types and the frame codec (#6)
- Target-agnostic contract test suite with a stub `EchoTarget` (#7)
- CI on GitHub Actions for lint, types and tests (#8)
- `SimClock` (integer microseconds), named `RngFactory` streams and the `run.json`
  run record (#19, #26)

**Phase 1: virtual spacecraft**
- Spacecraft subsystem models: power, thermal, attitude, payload and communications,
  with truth and readings records, sensor noise and limit flags (epic #30)
- Portable arithmetic for byte-identical results across platforms (ADR-0006), and
  spacecraft configuration and starting state (ADR-0005)
- Flight computer: modes BOOT, NOMINAL, SCIENCE, DOWNLINK, SAFE and FAULT; automatic
  safe and fault entry; BOOT and RESET; command dispatcher with ACK/NACK; periodic
  telemetry; DOWNLINK of payload data as DATA frames (epic #45, ADR-0004)
- `SilTarget`: the spacecraft behind `TestTarget`, with target-level faults
  (`forced_reset`, `sensor_freeze`, `transmitter_off`, `battery_drain`), radio traffic
  accounting (ADR-0007), a transmit rate in bytes per second, and outbound frames that
  span ticks at any tick length (ADR-0008) (epic #57)
- Wire protocol version 1: command, ACK/NACK, telemetry and DATA payloads, recorded in
  ADR-0009 as the contract the Phase 7 firmware must match
- Power, thermal and data budget with calibrated defaults (`docs/power-thermal-budget.md`)
- Nominal scenario harness over one orbit (#63) and golden telemetry captures checked
  byte for byte on Linux and macOS (#64)
- Contract, determinism, behavioural and digest tests for `SilTarget`; performance
  budgets (orbit average ≤ 100 µs per tick, worst-case tick ≤ 150 µs) checked in CI
- Demo script `scripts/demo_spacecraft.py`

### Known limitations

- Hardware-in-the-loop (Phase 7) is not built yet; `HilTarget` is a placeholder in the
  contract suite.
- Open protocol points for the firmware: maximum payload size and serial stream
  resynchronisation (#123), 64-bit firmware timekeeping (#68).
- Later work tracked elsewhere: commanded-versus-observed consistency check (#118,
  Phase 7a), always-streaming transmission (#135, Phase 3), further performance work
  (#128, backlog).

[0.1.0]: https://github.com/solaranoir/pocketsat-testlab/releases/tag/v0.1.0
