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
sent this tick, ``transmit_capacity_bytes``: 0 while the transmitter is off, and while
it is on the bytes its fixed data rate allows in this tick (#122).

**Transmit rate (#122).** The radio has a fixed data rate,
``CommsConfig.transmit_rate_bytes_per_s``, whatever the tick length. Each tick with the
transmitter on, comms converts it into whole bytes exactly, in integers, carrying the
fraction of a byte to the next tick::

    total = transmit_rate_bytes_per_s * dt_us + carry   # micro-bytes
    transmit_capacity_bytes, carry = divmod(total, 1_000_000)

So over any run of transmitter-on ticks the capacities add up to
``floor(rate * on_time_us / 1_000_000)``, at any tick length, with no drift. At the
default 100 ms tick and 1200 bytes/s the division is exact: 120 bytes every tick and a
carry of 0, as before #122. While the transmitter is off the capacity is 0 and the
carry is **held**, neither growing nor discarded (as the payload holds its acquisition
remainder while not acquiring), so the sum above counts transmitter-on time only. The
carry is part of comms' state and :meth:`Comms.reset` clears it.

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
  history, not a read): ``transmit_draw_w(config, previous_transmitter_on, sent_bytes,
  capacity_bytes=previous_capacity, dt_us=previous_dt_us)`` rejects bytes above that
  tick's capacity (which varies from tick to tick when the rate does not divide evenly
  into ticks, #122) or any bytes while the transmitter was off, and a non-zero
  ``uplink_lost_count`` is rejected while the receiver was on. A violation is
  a simulator bug, never a scenario outcome: ``ValueError`` propagates out of
  ``step()`` and nothing is clipped;
- reports the bytes as ``previous_tick_sent_bytes`` (per tick) and adds the two counts
  to ``uplink_lost_count`` and ``outbound_suppressed_count``, running totals since
  :meth:`Comms.reset`. The RESET command and ``forced_reset`` reboot only the flight
  computer, so the totals survive them;
- **charges** the bytes: ``transmit_power_w`` is the idle draw of tick N+1's
  transmitter state plus ``transmit_power_per_byte_w`` times tick N's bytes, scaled to
  tick N's length so a byte costs the same energy at any tick length (#122;
  :data:`~pocketsat.spacecraft.config.PER_BYTE_DRAW_TICK_US`, a factor of exactly 1 at
  the 100 ms tick)::

      transmit_draw_w(config, transmitter_on, 0, capacity_bytes=0)
          + config.transmit_power_per_byte_w
          * (radio_traffic.sent_bytes * PER_BYTE_DRAW_TICK_US / previous_dt_us)

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

from pocketsat.core.clock import US_PER_S, check_us
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.config import PER_BYTE_DRAW_TICK_US, CommsConfig
from pocketsat.spacecraft.controls import (
    NO_RADIO_TRAFFIC,
    RadioMode,
    RadioTraffic,
    SpacecraftControls,
)
from pocketsat.spacecraft.snapshots import CommsReadings, CommsSnapshot, CommsTruth
from pocketsat.targets.base import EnvironmentState

__all__ = ["Comms", "transmit_draw_w"]


def transmit_draw_w(
    config: CommsConfig,
    transmitter_on: bool,
    sent_bytes: int,
    *,
    capacity_bytes: int,
    dt_us: int = PER_BYTE_DRAW_TICK_US,
) -> float:
    """Return the transmit draw for one tick, watts.

    While the transmitter is on::

        transmitter_on_power_w
            + transmit_power_per_byte_w * (sent_bytes * PER_BYTE_DRAW_TICK_US / dt_us)

    and 0.0 while it is off. The scale makes a byte's energy independent of the tick
    length (#122); at the 100 ms tick ``sent_bytes * 100_000 / 100_000`` is exactly
    ``sent_bytes``, so the draw is bit-for-bit
    ``transmitter_on_power_w + transmit_power_per_byte_w * sent_bytes``. Pure: no
    state, no randomness, and portable arithmetic only (ADR-0006). With
    ``sent_bytes = 0`` the result is exactly ``transmitter_on_power_w``.

    Comms uses it to validate a tick's :class:`~pocketsat.spacecraft.controls.RadioTraffic`
    against the transmitter state and capacity of that same tick (ADR-0007 §4); the
    draw it reports one tick later is the module docstring's formula, which equals this
    one while the transmitter stays on.

    Args:
        config: Communications settings.
        transmitter_on: Whether the transmitter was on in the tick the bytes were sent.
        sent_bytes: Bytes sent in that tick, 0..``capacity_bytes``.
        capacity_bytes: That tick's ``transmit_capacity_bytes`` (#122: it follows from
            the rate and the tick length, so it is not a config constant).
        dt_us: The length of the tick the bytes were sent in, integer microseconds
            (default 100 ms). Must be positive when ``sent_bytes`` is non-zero.

    Raises:
        TypeError: ``sent_bytes`` is not an int.
        ValueError: ``sent_bytes`` is negative or above ``capacity_bytes``, or non-zero
            while the transmitter is off or with ``dt_us`` not positive.
    """
    if isinstance(sent_bytes, bool) or not isinstance(sent_bytes, int):
        raise TypeError(f"sent_bytes must be an int, got {sent_bytes!r}")
    if not transmitter_on:
        if sent_bytes:
            raise ValueError(f"sent_bytes must be 0 while the transmitter is off, got {sent_bytes}")
        return 0.0
    if not 0 <= sent_bytes <= capacity_bytes:
        raise ValueError(f"sent_bytes must be in 0..{capacity_bytes}, got {sent_bytes}")
    if not sent_bytes:
        return config.transmitter_on_power_w
    if dt_us <= 0:
        raise ValueError(f"dt_us must be positive when bytes were sent, got {dt_us}")
    return config.transmitter_on_power_w + config.transmit_power_per_byte_w * (
        sent_bytes * PER_BYTE_DRAW_TICK_US / dt_us
    )


_RECEIVER_ON: Final = {RadioMode.OFF: False, RadioMode.RX_ONLY: True, RadioMode.RX_TX: True}
_TRANSMITTER_ON: Final = {RadioMode.OFF: False, RadioMode.RX_ONLY: False, RadioMode.RX_TX: True}


class Comms:
    """The communications subsystem (#44). Implements the ``Subsystem`` protocol.

    Built with its settings record only (ADR-0005): it has no starting record and
    reads nothing from other subsystems. See the module docstring for the model and
    the radio traffic it is told about (ADR-0007).

    After :meth:`reset`, and before the first step, the radio is ``OFF``: receiver and
    transmitter off, capacity 0, rate carry 0, no draw, counters 0. The first step applies the
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
        self._capacity_bytes = 0
        self._carry_ubytes = 0  # micro-bytes, 0..US_PER_S - 1 (#122)
        self._dt_us = 0  # the length of the tick this state describes
        self._sent_dt_us = 0  # the length of the tick _sent_bytes were sent in
        self._uplink_lost_count = 0
        self._outbound_suppressed_count = 0
        self._snapshots: dict[tuple[RadioMode, int, int, int], CommsSnapshot] = {}
        self._snapshot = self._build_snapshot()

    def reset(self, rng: RngFactory) -> None:
        """Return to the reset state (radio ``OFF``, rate carry 0, counters 0). Comms
        requests no random streams."""
        self._restore()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Apply the previous tick's radio traffic, then this tick's radio mode and
        capacity.

        ``controls.radio_traffic`` describes the previous tick and is checked against the
        state comms had then, including that tick's capacity (ADR-0007 §4), before this
        tick's mode is applied. The capacity of this tick is the transmit rate over
        ``dt_us``, with the fractional byte carried (module docstring, #122).

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative; or the traffic record reports bytes above
                the previous tick's capacity, any bytes while the transmitter was off,
                or lost uplink while the receiver was on.
        """
        check_us("dt_us", dt_us)
        traffic = controls.radio_traffic
        mode = controls.radio.mode
        no_traffic = traffic is NO_RADIO_TRAFFIC or traffic == NO_RADIO_TRAFFIC
        if not no_traffic:
            self._apply_traffic(traffic)  # raises before anything changes
        if _TRANSMITTER_ON[mode]:
            capacity, self._carry_ubytes = divmod(
                self._config.transmit_rate_bytes_per_s * dt_us + self._carry_ubytes, US_PER_S
            )
        else:
            capacity = 0  # the carry is held while the transmitter is off
        self._dt_us = dt_us
        if no_traffic:
            if mode is self._mode and not self._sent_bytes and capacity == self._capacity_bytes:
                return  # nothing changed: keep the snapshot object
            self._sent_bytes = 0
        self._mode = mode
        self._capacity_bytes = capacity
        # Snapshots are immutable and depend only on the mode, the capacity, the bytes,
        # and the totals; while the totals stand still, a pass repeats a handful of
        # values, so each is built once (#78's budget). The cache is cleared when a
        # total moves.
        key = (mode, capacity, self._sent_bytes, self._sent_dt_us)
        snapshot = self._snapshots.get(key)
        if snapshot is None:
            snapshot = self._snapshots[key] = self._build_snapshot()
        self._snapshot = snapshot

    def _apply_traffic(self, traffic: RadioTraffic) -> None:
        """Validate ``traffic`` against the previous tick's state (still ``self._mode``,
        ``self._capacity_bytes`` and ``self._dt_us``) and take it into the counters."""
        previous = self._mode
        # Raises for bytes above the previous capacity or while the transmitter was off.
        transmit_draw_w(
            self._config,
            _TRANSMITTER_ON[previous],
            traffic.sent_bytes,
            capacity_bytes=self._capacity_bytes,
            dt_us=self._dt_us,
        )
        if traffic.uplink_lost_count and _RECEIVER_ON[previous]:
            raise ValueError(
                f"uplink_lost_count must be 0 when the receiver was on in the previous tick"
                f" (radio {previous.name}), got {traffic.uplink_lost_count}"
            )
        self._sent_bytes = traffic.sent_bytes
        self._sent_dt_us = self._dt_us
        if traffic.uplink_lost_count or traffic.outbound_suppressed_count:
            self._uplink_lost_count += traffic.uplink_lost_count
            self._outbound_suppressed_count += traffic.outbound_suppressed_count
            self._snapshots.clear()  # the totals changed

    def _build_snapshot(self) -> CommsSnapshot:
        config = self._config
        mode = self._mode
        transmitter_on = _TRANSMITTER_ON[mode]
        sent_bytes = self._sent_bytes
        power_w = transmit_draw_w(config, transmitter_on, 0, capacity_bytes=0)
        if sent_bytes:
            # ADR-0007 §2: the idle draw of this tick plus the previous tick's bytes,
            # scaled to the length of the tick they were sent in (#122; exactly
            # sent_bytes at the 100 ms tick, so bit-for-bit as before).
            power_w = power_w + config.transmit_power_per_byte_w * (
                sent_bytes * PER_BYTE_DRAW_TICK_US / self._sent_dt_us
            )
        values = (
            mode,
            _RECEIVER_ON[mode],
            transmitter_on,
            self._capacity_bytes,
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

        Snapshots are reused: repeated calls in an unchanged mode and capacity with no
        traffic, and steps that repeat a mode, capacity, byte count, and tick length
        while the running totals are unchanged, return the same object.
        """
        return self._snapshot
