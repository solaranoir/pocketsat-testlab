"""Telemetry cadence settings and the telemetry schedule (#55).

The flight computer's emit-telemetry phase (ADR-0004 §2 step e) asks
:func:`telemetry_due` once per step whether a TELEMETRY frame is due, and sends it
through the outbound queue after any ACK/NACK and before DATA (ADR-0004 §10). The
cadence depends on the mode (:class:`TelemetryConfig`); the rules, in full, are in
``docs/spacecraft-modes.md`` ("Telemetry cadence"):

- **BOOT sends no telemetry.** The only frame from a boot is the **boot-complete
  beacon**: the first frame of the mode BOOT leaves for, sent in the same step as
  ``BOOT_COMPLETE`` (or a SAFE or FAULT entry during BOOT).
- **A mode change sends a frame at once** (the step whose step d changed the mode) and
  restarts the schedule from that step, so the ground sees every new mode immediately.
- **Otherwise one frame per period** of the current mode: due in the first step whose
  simulated time has reached the previous frame's time plus the period. Periods are
  integer microseconds of simulated time (ADR-0003), so like :class:`BootConfig`'s
  duration they round up to whole ticks; the defaults are whole numbers of 100 ms
  ticks.
- **A due frame uses its slot whether or not it is sent.** A frame that doesn't fit
  comms' transmit capacity is suppressed and counted (ADR-0004 §10, ADR-0007 §3), not
  retried, and the next frame is due one period later, so the cadence resumes as soon
  as capacity returns.
- The schedule is transient state: power-on and every reboot clear it (#49).

Pure and deterministic: no randomness, no wall-clock time, integer arithmetic only. This
module imports only :mod:`pocketsat.flight.modes` (not ``pocketsat.messages``; see the
import rule in :mod:`pocketsat.flight`).
"""

from dataclasses import dataclass
from typing import Final, Self

from pocketsat.core.clock import check_us
from pocketsat.flight.modes import Mode


@dataclass(frozen=True)
class TelemetryConfig:
    """Telemetry cadence per mode: the period between frames, integer microseconds.

    A frame is due in the first step whose simulated time has reached the last frame's
    time plus the current mode's period, so a period rounds up to whole ticks. BOOT has
    no period: it sends no telemetry except the boot-complete beacon (#55).

    Attributes:
        nominal_period_us: NOMINAL. Default 1 s (10 ticks at the default 100 ms tick).
        science_period_us: SCIENCE. Default 1 s.
        downlink_period_us: DOWNLINK. Default 1 s; telemetry goes before DATA in each
            tick (ADR-0004 §10), so a shorter period costs downlink capacity.
        safe_period_us: SAFE. Default 0.5 s (5 ticks): faster, so the ground follows
            the recovery closely.
        fault_period_us: FAULT. Default 0.5 s, as SAFE: FAULT keeps SAFE's controls
            and the ground needs the state to diagnose it before sending RESET.

    Raises:
        TypeError: A period is not an int.
        ValueError: A period is not positive.
    """

    nominal_period_us: int = 1_000_000
    science_period_us: int = 1_000_000
    downlink_period_us: int = 1_000_000
    safe_period_us: int = 500_000
    fault_period_us: int = 500_000

    def __post_init__(self) -> None:
        for name in (
            "nominal_period_us",
            "science_period_us",
            "downlink_period_us",
            "safe_period_us",
            "fault_period_us",
        ):
            value = getattr(self, name)
            check_us(name, value)
            if value == 0:
                raise ValueError(f"{name} must be positive, got 0")

    @classmethod
    def uniform(cls, period_us: int) -> Self:
        """The same period in every mode that sends telemetry (not BOOT).

        Args:
            period_us: The period, integer microseconds; for example one tick to
                send telemetry every tick.

        Returns:
            A config with every period set to ``period_us``.

        Raises:
            TypeError: ``period_us`` is not an int.
            ValueError: ``period_us`` is not positive.
        """
        return cls(period_us, period_us, period_us, period_us, period_us)

    def period_us(self, mode: Mode) -> int | None:
        """The period between frames in ``mode``, or ``None`` for BOOT (no cadence).

        Args:
            mode: The flight mode.

        Returns:
            The period, integer microseconds, or ``None`` in BOOT.
        """
        if mode is Mode.BOOT:
            return None
        if mode is Mode.NOMINAL:
            return self.nominal_period_us
        if mode is Mode.SCIENCE:
            return self.science_period_us
        if mode is Mode.DOWNLINK:
            return self.downlink_period_us
        if mode is Mode.SAFE:
            return self.safe_period_us
        return self.fault_period_us


DEFAULT_TELEMETRY_CONFIG: Final = TelemetryConfig()
"""The default cadence: 1 s in NOMINAL, SCIENCE, and DOWNLINK; 0.5 s in SAFE and FAULT;
none in BOOT except the boot-complete beacon."""


@dataclass(frozen=True)
class TelemetrySchedule:
    """The telemetry schedule: transient flight computer state (#55).

    Attributes:
        mode: The mode the schedule runs for, as of the last emit-telemetry phase.
            ``None`` after power-on or a reboot, before the first step.
        next_due_us: Simulated time from which the next frame is due, integer
            microseconds. Meaningless while ``mode`` is BOOT or ``None``.
    """

    mode: Mode | None = None
    next_due_us: int = 0


INITIAL_TELEMETRY_SCHEDULE: Final = TelemetrySchedule()
"""The schedule at power-on and after every reboot."""


def telemetry_due(
    schedule: TelemetrySchedule,
    mode: Mode,
    now_us: int,
    config: TelemetryConfig = DEFAULT_TELEMETRY_CONFIG,
) -> tuple[bool, TelemetrySchedule]:
    """Whether a TELEMETRY frame is due in this step, and the schedule after it.

    Called once per flight computer step, after step d, with the mode that step left.

    - In BOOT: never due; the schedule just records BOOT.
    - In a mode other than the schedule's (a mode change since the last step, the
      boot-complete beacon included): due now.
    - Otherwise: due once ``now_us`` has reached ``schedule.next_due_us``.

    A due frame moves the next due time to ``now_us`` plus ``mode``'s period, whether
    or not the frame then fits the transmit capacity.

    Args:
        schedule: The schedule after the previous step.
        mode: The mode after this step's step d.
        now_us: Simulated time of this step, integer microseconds.
        config: The cadence settings.

    Returns:
        ``(due, schedule)``: whether to send a frame this step, and the new schedule.
    """
    period_us = config.period_us(mode)
    if period_us is None:
        if schedule.mode is mode:
            return False, schedule
        return False, TelemetrySchedule(mode)
    if mode is not schedule.mode or now_us >= schedule.next_due_us:
        return True, TelemetrySchedule(mode, now_us + period_us)
    return False, schedule
