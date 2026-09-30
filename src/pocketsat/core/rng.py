"""Named, derived random streams (ADR-0003).

Each random consumer asks the :class:`RngFactory` for a stream by name. A stream's
seed depends only on the master seed and the stream name, so adding a new consumer
never changes the values an existing consumer sees.

This module is the only place in simulation code that constructs a random generator.
The global ``random`` module state is never used.
"""

import hashlib
from collections.abc import Mapping
from random import Random
from types import MappingProxyType


def derive_seed(parent_seed: int, label: str) -> int:
    """Derive a child seed from a parent seed and a label.

    The seed is the first 8 bytes of ``SHA-256(f"{parent_seed}:{label}")`` (UTF-8),
    read as a big-endian unsigned integer. Used both for stream seeds and for campaign
    run seeds (for example ``derive_seed(campaign_seed, f"run:{i}")``).

    Args:
        parent_seed: The parent (master or campaign) seed.
        label: Stream name or other label.

    Returns:
        An integer in ``0 .. 2**64 - 1``.
    """
    if isinstance(parent_seed, bool) or not isinstance(parent_seed, int):
        raise TypeError(f"seed must be an int, got {type(parent_seed).__name__}")
    digest = hashlib.sha256(f"{parent_seed}:{label}".encode()).digest()
    return int.from_bytes(digest[:8], "big")


class RngFactory:
    """Hands out independent, reproducible random streams derived from one master seed."""

    def __init__(self, master_seed: int) -> None:
        """Create a factory.

        Args:
            master_seed: The run's master seed.
        """
        derive_seed(master_seed, "")  # validates the seed type
        self._master_seed = master_seed
        self._issued: dict[str, int] = {}

    @property
    def master_seed(self) -> int:
        """The master seed this factory derives from."""
        return self._master_seed

    def stream(self, name: str) -> Random:
        """Return a new generator seeded from the master seed and ``name``.

        Every call returns a fresh generator with the same seed for the same name, so
        request a stream once and keep it. Consumers should use only ``random()``,
        ``uniform()``, and ``gauss()``, whose outputs are stable for a given seed.

        Args:
            name: Stream name, such as ``"rf.loss"``.

        Returns:
            A ``random.Random`` seeded with ``derive_seed(master_seed, name)``.
        """
        if not name:
            raise ValueError("stream name must be non-empty")
        seed = derive_seed(self._master_seed, name)
        self._issued[name] = seed
        return Random(seed)

    @property
    def stream_seeds(self) -> Mapping[str, int]:
        """Seeds issued so far, by stream name in first-request order, for the run record."""
        return MappingProxyType(dict(self._issued))
