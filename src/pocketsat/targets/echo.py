"""A trivial stub target that echoes uplink frames back as downlink frames.

``EchoTarget`` has no spacecraft model. It exists to exercise the ``TestTarget``
contract suite and, later, orchestrator plumbing without a real simulator.
"""

from pocketsat.core.clock import check_us
from pocketsat.targets.base import (
    EnvironmentState,
    TargetCapabilities,
    TargetFault,
    UnsupportedFaultError,
)

MUTE_FAULT = "mute"
"""Fault type that stops the target from echoing, like a transmitter failure."""


class EchoTarget:
    """Stub target: every frame sent is returned unchanged by ``receive()`` after ``advance()``.

    Frames sent during a step are echoed when the target next advances, so the output
    follows the same send, advance, receive order as a real target. While a ``mute``
    fault is active, frames sent are consumed but not echoed.
    """

    capabilities = TargetCapabilities(
        deterministic=True,
        real_time=False,
        supported_faults=frozenset({MUTE_FAULT}),
    )

    def __init__(self) -> None:
        self._pending: list[bytes] = []
        self._outbox: list[bytes] = []
        self._mute_remaining_us: int | None = None
        self._muted = False
        self.environment: EnvironmentState | None = None
        """The last environment applied, or ``None`` since the last reset."""

    def connect(self) -> None:
        """No-op: the target runs in-process."""

    def reset(self, seed: int) -> None:
        """Discard queued frames, clear faults, and forget the environment.

        ``seed`` is accepted for interface compatibility; the echo target has no randomness.
        """
        self._pending.clear()
        self._outbox.clear()
        self._muted = False
        self._mute_remaining_us = None
        self.environment = None

    def send(self, frame: bytes) -> None:
        """Queue a frame to be echoed on the next ``advance()``."""
        self._pending.append(bytes(frame))

    def receive(self) -> list[bytes]:
        """Return and clear the frames echoed since the last call."""
        frames, self._outbox = self._outbox, []
        return frames

    def apply_environment(self, env: EnvironmentState) -> None:
        """Record the environment. The echo target does not otherwise use it."""
        self.environment = env

    def inject(self, fault: TargetFault) -> None:
        """Mute for ``fault.duration_us`` microseconds, or until reset if it is ``None``.

        Raises:
            UnsupportedFaultError: The fault type is not ``mute``.
        """
        if fault.fault_type not in self.capabilities.supported_faults:
            raise UnsupportedFaultError(fault.fault_type, self.capabilities.supported_faults)
        self._muted = True
        self._mute_remaining_us = fault.duration_us

    def advance(self, dt_us: int) -> None:
        """Echo queued frames (unless muted), then advance fault timers by ``dt_us``.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
        """
        check_us("dt_us", dt_us)
        if not self._muted:
            self._outbox.extend(self._pending)
        self._pending.clear()
        if self._muted and self._mute_remaining_us is not None:
            self._mute_remaining_us -= dt_us
            if self._mute_remaining_us <= 0:
                self._muted = False
                self._mute_remaining_us = None

    def close(self) -> None:
        """Discard queued frames."""
        self._pending.clear()
        self._outbox.clear()
