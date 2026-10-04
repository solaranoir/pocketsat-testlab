"""Boot sequence settings and the uptime counter (#49).

The flight computer (:class:`~pocketsat.flight.FlightComputer`) powers on, and reboots,
into BOOT. BOOT lasts :attr:`BootConfig.duration_us` of uptime; then the evaluate-flags
phase raises ``BOOT_COMPLETE`` and the mode machine moves to NOMINAL (#47). A safe
condition or a fault during BOOT leaves BOOT earlier, for SAFE or FAULT, without
``BOOT_COMPLETE`` (see ``docs/spacecraft-modes.md``, "Boot sequence and RESET").

Uptime is counted in integer microseconds of simulated time (ADR-0003) and exposed in
milliseconds for telemetry by :func:`uptime_ms`. Pure and deterministic: no randomness,
no wall-clock time, integer arithmetic only.
"""

from dataclasses import dataclass
from typing import Final

from pocketsat.core.clock import check_us

US_PER_MS: Final = 1000
"""Microseconds per millisecond."""


@dataclass(frozen=True)
class BootConfig:
    """Settings of the boot sequence.

    Attributes:
        duration_us: Uptime BOOT lasts before ``BOOT_COMPLETE``, integer microseconds.
            ``BOOT_COMPLETE`` is raised in the first flight computer step whose uptime
            is at least this, so the duration is rounded up to whole ticks: with the
            default 100 ms tick, the default 5 s gives 50 ticks under BOOT's controls
            (ticks 0 to 49 after power-on) and NOMINAL's from tick 50. 0 completes the
            boot in the first step, so BOOT's controls still apply to exactly one tick.

    Raises:
        TypeError: ``duration_us`` is not an int.
        ValueError: ``duration_us`` is negative.
    """

    duration_us: int = 5_000_000

    def __post_init__(self) -> None:
        check_us("duration_us", self.duration_us)


DEFAULT_BOOT_CONFIG: Final = BootConfig()
"""The default boot settings: BOOT lasts 5 s."""


def boot_complete(uptime_us: int, config: BootConfig = DEFAULT_BOOT_CONFIG) -> bool:
    """Whether BOOT has lasted long enough for ``BOOT_COMPLETE``.

    Args:
        uptime_us: Uptime since the last power-on or reboot, integer microseconds.
        config: The boot settings.

    Returns:
        True once ``uptime_us`` has reached :attr:`BootConfig.duration_us`.
    """
    return uptime_us >= config.duration_us


def uptime_ms(uptime_us: int) -> int:
    """Uptime in whole milliseconds, as telemetry carries it.

    Rounded down (a counter shows the milliseconds that have fully elapsed) and not
    wrapped: the telemetry encoder sends it modulo 2**32, like a C ``uint32_t``
    millisecond counter, so the wire value wraps to 0 after about 49.7 days
    (``docs/protocol.md``).

    Args:
        uptime_us: Uptime, integer microseconds.

    Returns:
        ``uptime_us // 1000``.

    Raises:
        TypeError: ``uptime_us`` is not an int.
        ValueError: ``uptime_us`` is negative.
    """
    check_us("uptime_us", uptime_us)
    return uptime_us // US_PER_MS
