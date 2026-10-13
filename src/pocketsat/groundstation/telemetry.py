"""The ground station's telemetry store (ADR-0010 §2).

A read-only view of the telemetry the station has decoded. #157 fills it from
:meth:`~pocketsat.groundstation.GroundStation.end_tick` and adds the history queries;
until then it stays empty.
"""

from pocketsat.messages import Telemetry


class TelemetryStore:
    """Telemetry decoded by the station, oldest first. Empty until #157."""

    def __init__(self) -> None:
        self._history: list[tuple[int, Telemetry]] = []

    def __len__(self) -> int:
        """Number of telemetry records stored."""
        return len(self._history)

    @property
    def latest(self) -> Telemetry | None:
        """The most recent telemetry, or ``None`` if none has been received."""
        return self._history[-1][1] if self._history else None

    @property
    def history(self) -> tuple[tuple[int, Telemetry], ...]:
        """Every stored record as ``(time_us, telemetry)``, oldest first."""
        return tuple(self._history)
