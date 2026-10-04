"""Automatic safe-mode and fault entry rules (#48).

The flight computer's evaluate-flags phase (ADR-0004 §2 step d) calls :func:`evaluate`
once per tick with the subsystems' **readings** (never their truth, ADR-0004 §6). It
returns the mode events to raise and the guard for leaving SAFE:

- **SAFE_CONDITION** while any safe-mode flag (:class:`SafeFlag`: ``critical_battery``,
  ``over_temp``, ``under_temp``) has been set for at least
  :attr:`SafetyConfig.sustain_tick_count` consecutive ticks. Each flag has its own
  counter. It counts from the tick the flag first appears in the readings, goes up by
  one per tick the flag stays set, and drops to 0 in any tick the flag is clear.
- **FAULT_DETECTED** in any tick a :class:`ConsistencyCheck` fails: the readings
  contradict themselves, which cannot happen while the flight software and its data
  are sound. No persistence; one tick is enough.
- **safe_exit_allowed** is true when no safe-mode flag is set in this tick's readings.
  SET_MODE NOMINAL may leave SAFE only then (#47).

The rules are level-triggered: SAFE_CONDITION is raised in every tick a flag is
sustained, and the state machine ignores it in SAFE and FAULT. Nothing here leaves SAFE
or FAULT; only commands do (SET_MODE NOMINAL behind the guard, RESET).

Pure and deterministic: no randomness, no wall-clock time, no floating-point arithmetic
beyond comparisons and ``math.isfinite`` (ADR-0006). Persistence is counted in ticks of
simulated time, so its duration is ``sustain_tick_count`` times the tick length (1 s at
the default 100 ms tick). See ``docs/spacecraft-modes.md``, "Safe and fault entry
rules".
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Final

from pocketsat.flight.modes import EventKind, ModeEvent

if TYPE_CHECKING:
    from pocketsat.flight.computer import SpacecraftReadings


class SafeFlag(Enum):
    """A readings flag that puts the spacecraft in SAFE once sustained (#48).

    The value is the flag's field name on its readings record. ``low_battery`` is not a
    safe-mode flag: it is a warning for the ground, and the payload inhibit acts on it
    (#43).
    """

    CRITICAL_BATTERY = "critical_battery"
    """``PowerReadings.critical_battery``: the SOC estimate is below the critical
    threshold."""

    OVER_TEMP = "over_temp"
    """``ThermalReadings.over_temp``: a reported temperature is above its limit."""

    UNDER_TEMP = "under_temp"
    """``ThermalReadings.under_temp``: a reported temperature is below its limit."""


SAFE_FLAGS: Final[tuple[SafeFlag, ...]] = tuple(SafeFlag)
"""The safe-mode flags, in the order of :attr:`SafetyState.flag_tick_counts`."""


class ConsistencyCheck(Enum):
    """An internal consistency check on the readings; a failure raises FAULT_DETECTED.

    Each check is an invariant the snapshot contracts (``docs/spacecraft.md``) state
    and the subsystems keep in every tick, faults included. A failure therefore means
    the data the flight software works from is corrupt, not that the spacecraft is in a
    bad physical state (that is what the safe-mode flags are for).
    """

    NON_FINITE_READING = "non_finite_reading"
    """A reported power, thermal, or attitude value is NaN or infinite. A NaN would
    also silently clear a flag (every comparison with it is false)."""

    BATTERY_FLAG_ORDER = "battery_flag_order"
    """``critical_battery`` is set without ``low_battery``. ``PowerConfig`` orders the
    thresholds so critical is only ever set together with low (#36)."""

    PAYLOAD_BOOKKEEPING = "payload_bookkeeping"
    """The payload's buffer does not add up: ``buffered_bytes`` is not
    ``total_produced_bytes - total_released_bytes``, is outside
    0..``buffer_capacity_bytes``, or ``oldest_unreleased_chunk_id`` is past
    ``next_chunk_id`` (#43)."""

    COMMS_CAPACITY = "comms_capacity"
    """Comms reports more bytes sent than its capacity, a negative count, or a
    non-zero capacity with the transmitter off (#44)."""


@dataclass(frozen=True)
class SafetyConfig:
    """Settings of the safe-mode rules.

    Attributes:
        sustain_tick_count: Consecutive ticks a safe-mode flag must be set before
            SAFE_CONDITION is raised. 1 raises it in the first tick the flag is set.
            Default 10 (1 s at the default 100 ms tick).

    Raises:
        TypeError: ``sustain_tick_count`` is not an int.
        ValueError: ``sustain_tick_count`` is below 1.
    """

    sustain_tick_count: int = 10

    def __post_init__(self) -> None:
        count = self.sustain_tick_count
        if isinstance(count, bool) or not isinstance(count, int):
            raise TypeError(f"sustain_tick_count must be an int, got {count!r}")
        if count < 1:
            raise ValueError(f"sustain_tick_count must be at least 1, got {count}")


DEFAULT_SAFETY_CONFIG: Final = SafetyConfig()
"""The default safe-mode settings."""


@dataclass(frozen=True)
class SafetyState:
    """What the rules remember between ticks: one persistence counter per flag.

    Transient flight computer state: a reboot (RESET, ``forced_reset``) clears it, so
    a flag still set after a reboot must be sustained again from BOOT.

    Attributes:
        flag_tick_counts: Consecutive ticks each flag in :data:`SAFE_FLAGS` has been
            set, up to and including the last evaluated tick. Saturates at the
            configured ``sustain_tick_count``, so it never grows without bound.

    Raises:
        ValueError: The counts don't match :data:`SAFE_FLAGS` or one is negative.
    """

    flag_tick_counts: tuple[int, ...] = (0,) * len(SAFE_FLAGS)

    def __post_init__(self) -> None:
        counts = self.flag_tick_counts
        if len(counts) != len(SAFE_FLAGS) or any(
            isinstance(c, bool) or not isinstance(c, int) or c < 0 for c in counts
        ):
            raise ValueError(
                f"flag_tick_counts must be {len(SAFE_FLAGS)} non-negative ints, got {counts!r}"
            )

    def count(self, flag: SafeFlag) -> int:
        """Consecutive ticks ``flag`` has been set (saturated)."""
        return self.flag_tick_counts[SAFE_FLAGS.index(flag)]


INITIAL_SAFETY_STATE: Final = SafetyState()
"""All counters at 0: after power-on and after every reboot."""


@dataclass(frozen=True)
class SafetyVerdict:
    """The result of one tick's evaluation.

    Attributes:
        state: The state to carry into the next tick.
        active: Safe-mode flags set in this tick's readings.
        sustained: Safe-mode flags set for at least ``sustain_tick_count`` ticks.
        failures: Consistency checks that failed this tick, in :class:`ConsistencyCheck`
            order.
    """

    state: SafetyState
    active: frozenset[SafeFlag]
    sustained: frozenset[SafeFlag]
    failures: tuple[ConsistencyCheck, ...]

    @property
    def safe_condition(self) -> bool:
        """True when SAFE_CONDITION is raised: some flag is sustained."""
        return bool(self.sustained)

    @property
    def fault_detected(self) -> bool:
        """True when FAULT_DETECTED is raised: some consistency check failed."""
        return bool(self.failures)

    @property
    def safe_exit_allowed(self) -> bool:
        """The guard for SET_MODE NOMINAL in SAFE: no safe-mode flag is set now."""
        return not self.active

    @property
    def events(self) -> tuple[ModeEvent, ...]:
        """The mode events to raise, FAULT_DETECTED first (it outranks SAFE)."""
        events: list[ModeEvent] = []
        if self.fault_detected:
            events.append(ModeEvent(EventKind.FAULT_DETECTED))
        if self.safe_condition:
            events.append(ModeEvent(EventKind.SAFE_CONDITION))
        return tuple(events)


def active_flags(readings: "SpacecraftReadings") -> frozenset[SafeFlag]:
    """The safe-mode flags set in ``readings``."""
    values = {
        SafeFlag.CRITICAL_BATTERY: readings.power.critical_battery,
        SafeFlag.OVER_TEMP: readings.thermal.over_temp,
        SafeFlag.UNDER_TEMP: readings.thermal.under_temp,
    }
    return frozenset(flag for flag in SAFE_FLAGS if values[flag])


def consistency_failures(readings: "SpacecraftReadings") -> tuple[ConsistencyCheck, ...]:
    """The :class:`ConsistencyCheck` invariants ``readings`` break, in check order."""
    power, thermal, attitude = readings.power, readings.thermal, readings.attitude
    payload, comms = readings.payload, readings.comms
    failed: list[ConsistencyCheck] = []

    measured = (
        power.bus_v,
        power.battery_current_a,
        power.soc,
        thermal.battery_c,
        thermal.electronics_c,
        attitude.pointing_error_deg,
        attitude.rate_dps,
    )
    if not all(math.isfinite(value) for value in measured):
        failed.append(ConsistencyCheck.NON_FINITE_READING)

    if power.critical_battery and not power.low_battery:
        failed.append(ConsistencyCheck.BATTERY_FLAG_ORDER)

    if (
        payload.buffered_bytes != payload.total_produced_bytes - payload.total_released_bytes
        or not 0 <= payload.buffered_bytes <= payload.buffer_capacity_bytes
        or payload.oldest_unreleased_chunk_id > payload.next_chunk_id
    ):
        failed.append(ConsistencyCheck.PAYLOAD_BOOKKEEPING)

    if (
        not 0 <= comms.sent_bytes <= comms.transmit_capacity_bytes
        or (comms.transmit_capacity_bytes != 0 and not comms.transmitter_on)
        or comms.uplink_lost_count < 0
        or comms.outbound_suppressed_count < 0
    ):
        failed.append(ConsistencyCheck.COMMS_CAPACITY)

    return tuple(failed)


def evaluate(
    state: SafetyState,
    readings: "SpacecraftReadings",
    config: SafetyConfig = DEFAULT_SAFETY_CONFIG,
) -> SafetyVerdict:
    """Apply the safe-mode and fault rules to one tick's readings.

    Pure: the same inputs always give the same verdict.

    Args:
        state: The counters carried from the previous tick (:data:`INITIAL_SAFETY_STATE`
            after power-on or a reboot).
        readings: This tick's readings, after the subsystem step.
        config: The persistence setting.

    Returns:
        The updated counters, the active and sustained flags, and the failed checks.
    """
    active = active_flags(readings)
    limit = config.sustain_tick_count
    counts = tuple(
        min(state.count(flag) + 1, limit) if flag in active else 0 for flag in SAFE_FLAGS
    )
    sustained = frozenset(
        flag for flag, count in zip(SAFE_FLAGS, counts, strict=True) if count >= limit
    )
    return SafetyVerdict(
        state=SafetyState(counts),
        active=active,
        sustained=sustained,
        failures=consistency_failures(readings),
    )
