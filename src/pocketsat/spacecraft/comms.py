"""Communications subsystem (#44): the full-duplex radio.

The radio is a crossband transceiver with a separate uplink receiver and downlink
transmitter (ADR-0004 §10). Comms obeys ``controls.radio.mode``:

========== ======== ===========
Mode       Receiver Transmitter
========== ======== ===========
``OFF``    off      off
``RX_ONLY`` on      off
``RX_TX``  on       on
========== ======== ===========

**Comms holds no data** (ADR-0004 §13). The payload owns stored data; the flight
computer (#56) moves chunks into DATA frames. Comms only exposes how many bytes may be
sent this tick, ``transmit_capacity_bytes``: ``CommsConfig.transmit_capacity_bytes``
while the transmitter is on, 0 otherwise.

**Transmitter failure is a mode downgrade.** The ``transmitter_off`` fault is a
control override merged by ``SilTarget`` (ADR-0004 §5, #60): it downgrades
``controls.radio.mode`` from ``RX_TX`` to ``RX_ONLY``, so the transmitter is off (and
the capacity 0) while the receiver keeps working. Comms has no fault hook and no
"transmitter failed" input; it simply obeys the mode it is given.

**Radio traffic (ADR-0007).** Comms does not see the frames; it is told about them.
``SilTarget`` fills ``controls.radio_traffic`` (a
:class:`~pocketsat.spacecraft.controls.RadioTraffic`) at step a of tick N+1 with what
happened in tick N: the wire bytes the flight computer sent (every frame type), the
uplink frames lost because the receiver was off, and the ACK/NACK and telemetry frames
the flight computer suppressed for lack of capacity. In tick N+1 comms

- **validates** the record against its own tick N state, which it remembers (its own
  history, not a read): ``transmit_draw_w(config, previous_transmitter_on, sent_bytes)``
  rejects bytes above the capacity or any bytes while the transmitter was off, and a
  non-zero ``uplink_lost_count`` is rejected while the receiver was on. A violation is
  a simulator bug, never a scenario outcome: ``ValueError`` propagates out of
  ``step()`` and nothing is clipped;
- reports the bytes as ``previous_tick_sent_bytes`` (per tick) and adds the two counts
  to ``uplink_lost_count`` and ``outbound_suppressed_count``, running totals since
  :meth:`Comms.reset`. The RESET command and ``forced_reset`` reboot only the flight
  computer, so the totals survive them;
- **charges** the bytes: ``transmit_power_w`` is the idle draw of tick N+1's
  transmitter state plus ``transmit_power_per_byte_w`` times tick N's bytes::

      transmit_draw_w(config, transmitter_on, 0)
          + config.transmit_power_per_byte_w * radio_traffic.sent_bytes

  While the transmitter is on in both ticks this is bit-for-bit
  :func:`transmit_draw_w` of the bytes. When it has just switched off (for example
  ``transmitter_off`` injected in tick N+1), the idle part is 0 and tick N's bytes are
  still paid for.

So a frame sent in tick N is reported by comms in tick N+1, and power, which reads comms
one tick late (comms steps after power), integrates its energy in tick N+2. The idle
draw keeps the usual timing: a mode change in tick N+1 reaches power in tick N+2.
Subsystem tests with default controls see no traffic.

Comms reads nothing from other subsystems, uses no randomness, and has no starting
record (ADR-0005). Arithmetic is portable (ADR-0006).
"""

from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.config import CommsConfig
from pocketsat.spacecraft.controls import (
    NO_RADIO_TRAFFIC,
    RadioMode,
    RadioTraffic,
    SpacecraftControls,
)
from pocketsat.spacecraft.snapshots import CommsReadings, CommsSnapshot, CommsTruth
from pocketsat.targets.base import EnvironmentState

__all__ = ["Comms", "transmit_draw_w"]


def transmit_draw_w(config: CommsConfig, transmitter_on: bool, sent_bytes: int) -> float:
    """Return the transmit draw for one tick, watts.

    ``transmitter_on_power_w + transmit_power_per_byte_w * sent_bytes`` while the
    transmitter is on, and 0.0 while it is off. Pure: no state, no randomness, and
    portable arithmetic only (ADR-0006). With ``sent_bytes = 0`` the result is exactly
    ``transmitter_on_power_w``.

    Comms uses it to validate a tick's :class:`~pocketsat.spacecraft.controls.RadioTraffic`
    against the transmitter state of that same tick (ADR-0007 §4); the draw it reports
    one tick later is the module docstring's formula, which equals this one while the
    transmitter stays on.

    Args:
        config: Communications settings.
        transmitter_on: Whether the transmitter was on in the tick the bytes were sent.
        sent_bytes: Bytes sent in that tick, 0..``transmit_capacity_bytes``.

    Raises:
        TypeError: ``sent_bytes`` is not an int.
        ValueError: ``sent_bytes`` is negative or above the capacity, or non-zero
            while the transmitter is off.
    """
    if isinstance(sent_bytes, bool) or not isinstance(sent_bytes, int):
        raise TypeError(f"sent_bytes must be an int, got {sent_bytes!r}")
    if not transmitter_on:
        if sent_bytes:
            raise ValueError(f"sent_bytes must be 0 while the transmitter is off, got {sent_bytes}")
        return 0.0
    if not 0 <= sent_bytes <= config.transmit_capacity_bytes:
        raise ValueError(
            f"sent_bytes must be in 0..{config.transmit_capacity_bytes}, got {sent_bytes}"
        )
    return config.transmitter_on_power_w + config.transmit_power_per_byte_w * sent_bytes


_RECEIVER_ON: Final = {RadioMode.OFF: False, RadioMode.RX_ONLY: True, RadioMode.RX_TX: True}
_TRANSMITTER_ON: Final = {RadioMode.OFF: False, RadioMode.RX_ONLY: False, RadioMode.RX_TX: True}


class Comms:
    """The communications subsystem (#44). Implements the ``Subsystem`` protocol.

    Built with its settings record only (ADR-0005): it has no starting record and
    reads nothing from other subsystems. See the module docstring for the model and
    the radio traffic it is told about (ADR-0007).

    After :meth:`reset`, and before the first step, the radio is ``OFF``: receiver and
    transmitter off, capacity 0, no draw, counters 0. The first step applies the
    commanded mode; its traffic record must be empty, because nothing was sent before.

    Attributes:
        name: Always ``"comms"``.
    """

    name: Final = "comms"

    def __init__(self, config: CommsConfig) -> None:
        """Create the communications subsystem, in its reset state.

        Args:
            config: Communications settings.

        Raises:
            TypeError: ``config`` is not a :class:`CommsConfig`.
        """
        if not isinstance(config, CommsConfig):
            raise TypeError(f"config must be a CommsConfig, got {config!r}")
        self._config = config
        self._restore()

    def _restore(self) -> None:
        self._mode = RadioMode.OFF
        self._sent_bytes = 0
        self._uplink_lost_count = 0
        self._outbound_suppressed_count = 0
        self._snapshots: dict[tuple[RadioMode, int], CommsSnapshot] = {}
        self._snapshot = self._build_snapshot()

    def reset(self, rng: RngFactory) -> None:
        """Return to the reset state (radio ``OFF``, counters 0). Comms requests no
        random streams."""
        self._restore()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Apply this tick's radio mode and the previous tick's radio traffic.

        The capacity is per tick, so ``dt_us`` does not scale it. ``controls.radio_traffic``
        describes the previous tick and is checked against the state comms had then
        (ADR-0007 §4), before this tick's mode is applied.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative; or the traffic record reports bytes above
                the previous tick's capacity, any bytes while the transmitter was off,
                or lost uplink while the receiver was on.
        """
        check_us("dt_us", dt_us)
        traffic = controls.radio_traffic
        mode = controls.radio.mode
        if traffic is NO_RADIO_TRAFFIC or traffic == NO_RADIO_TRAFFIC:
            if mode is self._mode and not self._sent_bytes:
                return  # nothing changed: keep the snapshot object
            self._sent_bytes = 0
        else:
            self._apply_traffic(traffic)
        self._mode = mode
        # Snapshots are immutable and depend only on the mode, the bytes, and the
        # totals; while the totals stand still, a pass repeats a handful of byte counts,
        # so each is built once (#78's budget). The cache is cleared when a total moves.
        key = (mode, self._sent_bytes)
        snapshot = self._snapshots.get(key)
        if snapshot is None:
            snapshot = self._snapshots[key] = self._build_snapshot()
        self._snapshot = snapshot

    def _apply_traffic(self, traffic: RadioTraffic) -> None:
        """Validate ``traffic`` against the previous tick's state (still ``self._mode``)
        and take it into the counters."""
        previous = self._mode
        # Raises for bytes above the capacity or while the transmitter was off.
        transmit_draw_w(self._config, _TRANSMITTER_ON[previous], traffic.sent_bytes)
        if traffic.uplink_lost_count and _RECEIVER_ON[previous]:
            raise ValueError(
                f"uplink_lost_count must be 0 when the receiver was on in the previous tick"
                f" (radio {previous.name}), got {traffic.uplink_lost_count}"
            )
        self._sent_bytes = traffic.sent_bytes
        if traffic.uplink_lost_count or traffic.outbound_suppressed_count:
            self._uplink_lost_count += traffic.uplink_lost_count
            self._outbound_suppressed_count += traffic.outbound_suppressed_count
            self._snapshots.clear()  # the totals changed

    def _build_snapshot(self) -> CommsSnapshot:
        config = self._config
        mode = self._mode
        transmitter_on = _TRANSMITTER_ON[mode]
        sent_bytes = self._sent_bytes
        power_w = transmit_draw_w(config, transmitter_on, 0)
        if sent_bytes:
            # ADR-0007 §2: the idle draw of this tick plus the previous tick's bytes.
            power_w = power_w + config.transmit_power_per_byte_w * sent_bytes
        values = (
            mode,
            _RECEIVER_ON[mode],
            transmitter_on,
            config.transmit_capacity_bytes if transmitter_on else 0,
            sent_bytes,
            self._uplink_lost_count,
            self._outbound_suppressed_count,
            power_w,
        )
        # Positional, as CommsTruth's fields are declared: built every tick of a pass
        # (#78's budget), where CommsSnapshot.from_truth's field walk costs too much.
        return CommsSnapshot(truth=CommsTruth(*values), readings=CommsReadings(*values))

    def snapshot(self) -> CommsSnapshot:
        """Return the current state. Readings equal truth: comms has no sensors.

        Snapshots are reused: repeated calls in an unchanged mode with no traffic, and
        steps that repeat a mode and byte count while the running totals are unchanged,
        return the same object.
        """
        return self._snapshot
