"""The ground station's telemetry store (#157, ADR-0010 §2).

Every TELEMETRY frame the downlink decoder (#156) decodes is kept as a
:class:`TelemetryRecord`: the decoded :class:`~pocketsat.messages.Telemetry`, the
simulated time it was received (the end of the tick, integer µs), its downlink sequence
number and the pass it arrived in. The console (#165) and assertions read the store
through :attr:`~pocketsat.groundstation.GroundStation.telemetry`; only the decoder
appends to it.

**Engineering units.** :class:`~pocketsat.messages.Telemetry` already holds every value
in its engineering unit at wire resolution (volts, SOC fraction, °C, degrees, enums).
:data:`TELEMETRY_QUANTITIES` is the one table of those units and resolutions, one row
per wire field in payload order, and a unit test checks it against the mapping table in
``docs/protocol.md`` ("Layout and mapping"). Mode and state fields are enums, so their
names come from the enum; flags are reported by member name.

**Retention.** The store is bounded by ``StationConfig.telemetry_retention_count``:
when full, the oldest record is dropped and counted in :attr:`TelemetryStore.dropped_count`.
Queries see only retained records.
"""

from collections import deque
from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from types import MappingProxyType
from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.groundstation.config import DEFAULT_TELEMETRY_RETENTION_COUNT
from pocketsat.messages import (
    BUS_V_SCALE,
    POINTING_SCALE,
    SOC_SCALE,
    TEMPERATURE_SCALE,
    Telemetry,
    TelemetryFlags,
)

TelemetryValue = int | float
"""A telemetry value: an ``int``, a ``float``, or an ``IntEnum``/``IntFlag`` member
(both ``int`` subclasses)."""


@dataclass(frozen=True)
class TelemetryQuantity:
    """One telemetry field in its engineering unit.

    Attributes:
        name: The :class:`~pocketsat.messages.Telemetry` attribute.
        wire_field: The wire field it is decoded from (``TELEMETRY_FIELDS``).
        unit: The engineering unit, or ``""`` for an enumerated or bit-field value.
        resolution: The smallest step between decoded values, in ``unit``: the inverse
            of the wire scale factor. ``None`` for an enumerated or bit-field value.
    """

    name: str
    wire_field: str
    unit: str
    resolution: Fraction | None


TELEMETRY_QUANTITIES: Final[tuple[TelemetryQuantity, ...]] = (
    TelemetryQuantity("uptime_ms", "uptime_ms", "ms", Fraction(1)),
    TelemetryQuantity("boot_count", "boot_count", "boots", Fraction(1)),
    TelemetryQuantity("flags", "flags", "", None),
    TelemetryQuantity("mode", "mode", "", None),
    TelemetryQuantity("radio_mode", "radio_mode", "", None),
    TelemetryQuantity("attitude_state", "attitude_state", "", None),
    TelemetryQuantity("payload_state", "payload_state", "", None),
    TelemetryQuantity("bus_v", "bus_mv", "V", Fraction(1, BUS_V_SCALE)),
    TelemetryQuantity("soc", "soc_permille", "fraction", Fraction(1, SOC_SCALE)),
    TelemetryQuantity("battery_c", "battery_centi_c", "°C", Fraction(1, TEMPERATURE_SCALE)),
    TelemetryQuantity("electronics_c", "electronics_centi_c", "°C", Fraction(1, TEMPERATURE_SCALE)),
    TelemetryQuantity("buffered_bytes", "buffered_bytes", "bytes", Fraction(1)),
    TelemetryQuantity(
        "pointing_error_deg", "pointing_error_centi_deg", "°", Fraction(1, POINTING_SCALE)
    ),
)
"""Every telemetry field in payload order, with its engineering unit and resolution.
Checked against ``docs/protocol.md`` by ``tests/unit/test_telemetry_store.py``."""

QUANTITIES_BY_NAME: Final[Mapping[str, TelemetryQuantity]] = MappingProxyType(
    {quantity.name: quantity for quantity in TELEMETRY_QUANTITIES}
)
""":data:`TELEMETRY_QUANTITIES` by :class:`~pocketsat.messages.Telemetry` attribute."""


def quantity(name: str) -> TelemetryQuantity:
    """Return the quantity of the :class:`~pocketsat.messages.Telemetry` field ``name``.

    Raises:
        KeyError: ``name`` is not a telemetry field.
    """
    try:
        return QUANTITIES_BY_NAME[name]
    except KeyError:
        raise KeyError(f"{name!r} is not a telemetry field") from None


@dataclass(frozen=True)
class TelemetryRecord:
    """One decoded TELEMETRY frame as the station received it.

    Attributes:
        time_us: Simulated time it was received: the end of the tick, integer µs.
        sequence: Its downlink frame sequence number (ADR-0009 §7).
        pass_id: The pass it arrived in: 0 until the pass state machine (#161) starts
            the first pass, then one more at each AOS.
        telemetry: The decoded telemetry, in engineering units at wire resolution.
    """

    time_us: int
    sequence: int
    pass_id: int
    telemetry: Telemetry

    def value(self, name: str) -> TelemetryValue:
        """The value of the telemetry field ``name`` in its engineering unit.

        Raises:
            KeyError: ``name`` is not a telemetry field.
        """
        quantity(name)
        value: TelemetryValue = getattr(self.telemetry, name)
        return value

    def state_name(self, name: str) -> str:
        """The name of the enumerated field ``name`` (for example ``"NOMINAL"``).

        Raises:
            KeyError: ``name`` is not a telemetry field.
            ValueError: ``name`` is not an enumerated field.
        """
        value = self.value(name)
        if not isinstance(value, Enum) or isinstance(value, TelemetryFlags):
            raise ValueError(f"{name!r} is not an enumerated telemetry field")
        return value.name

    @property
    def flag_names(self) -> tuple[str, ...]:
        """The names of the flags set, in bit order (``docs/protocol.md``, "Flag bits")."""
        flags = self.telemetry.flags
        return tuple(str(member.name) for member in TelemetryFlags if member in flags)


class TelemetryStore:
    """Telemetry the station has decoded, oldest first, with a bounded history.

    Read-only to everyone but the station's downlink decoder, which calls
    :meth:`append`.

    Args:
        retention_count: Most records kept; when full, the oldest is dropped
            (``StationConfig.telemetry_retention_count``). Default
            :data:`~pocketsat.groundstation.config.DEFAULT_TELEMETRY_RETENTION_COUNT`.

    Raises:
        TypeError: ``retention_count`` is not an int.
        ValueError: ``retention_count`` is not positive.
    """

    def __init__(self, retention_count: int = DEFAULT_TELEMETRY_RETENTION_COUNT) -> None:
        if isinstance(retention_count, bool) or not isinstance(retention_count, int):
            raise TypeError(f"retention_count must be an int, got {retention_count!r}")
        if retention_count <= 0:
            raise ValueError(f"retention_count must be positive, got {retention_count}")
        self._retention_count = retention_count
        self._records: deque[TelemetryRecord] = deque(maxlen=retention_count)
        self._dropped_count = 0

    def __len__(self) -> int:
        """Number of records retained."""
        return len(self._records)

    def __iter__(self) -> Iterator[TelemetryRecord]:
        """The retained records, oldest first."""
        return iter(self._records)

    @property
    def retention_count(self) -> int:
        """Most records kept."""
        return self._retention_count

    @property
    def dropped_count(self) -> int:
        """Records dropped by the retention limit since the station's last reset."""
        return self._dropped_count

    @property
    def latest(self) -> TelemetryRecord | None:
        """The most recent record, or ``None`` if none has been received."""
        return self._records[-1] if self._records else None

    @property
    def history(self) -> tuple[TelemetryRecord, ...]:
        """Every retained record, oldest first."""
        return tuple(self._records)

    def latest_value(self, name: str) -> TelemetryValue | None:
        """The latest value of the telemetry field ``name``, or ``None`` if none yet.

        Raises:
            KeyError: ``name`` is not a telemetry field.
        """
        quantity(name)
        latest = self.latest
        return None if latest is None else latest.value(name)

    def window(self, start_us: int, end_us: int) -> tuple[TelemetryRecord, ...]:
        """The retained records received in ``[start_us, end_us)``, oldest first.

        Raises:
            TypeError: A bound is not an int.
            ValueError: A bound is negative.
        """
        check_us("start_us", start_us)
        check_us("end_us", end_us)
        return tuple(r for r in self._records if start_us <= r.time_us < end_us)

    @property
    def pass_ids(self) -> tuple[int, ...]:
        """The passes with retained records, in order."""
        return tuple(dict.fromkeys(record.pass_id for record in self._records))

    def by_pass(self, pass_id: int) -> tuple[TelemetryRecord, ...]:
        """The retained records received in pass ``pass_id``, oldest first."""
        return tuple(record for record in self._records if record.pass_id == pass_id)

    def append(self, record: TelemetryRecord) -> None:
        """Store a record. Called only by the station's downlink decoder.

        Raises:
            TypeError: ``record`` is not a :class:`TelemetryRecord`.
            ValueError: ``record`` was received before the latest record.
        """
        if not isinstance(record, TelemetryRecord):
            raise TypeError(f"record must be a TelemetryRecord, got {record!r}")
        latest = self.latest
        if latest is not None and record.time_us < latest.time_us:
            raise ValueError(f"record time {record.time_us} is before the latest {latest.time_us}")
        if len(self._records) == self._retention_count:
            self._dropped_count += 1
        self._records.append(record)
