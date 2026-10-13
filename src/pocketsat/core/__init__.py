"""Determinism foundations: the simulated clock, seeded random streams (ADR-0003), and
portable arithmetic (ADR-0006)."""

from pocketsat.core.clock import DEFAULT_TICK_US, SimClock, check_us, seconds_to_ticks
from pocketsat.core.portable import (
    PI,
    portable_atan2,
    portable_cos_deg,
    portable_db,
    portable_log2,
    portable_sincos_turns,
)
from pocketsat.core.rng import RngFactory, derive_seed, portable_normal

__all__ = [
    "DEFAULT_TICK_US",
    "PI",
    "RngFactory",
    "SimClock",
    "check_us",
    "derive_seed",
    "portable_atan2",
    "portable_cos_deg",
    "portable_db",
    "portable_log2",
    "portable_normal",
    "portable_sincos_turns",
    "seconds_to_ticks",
]
