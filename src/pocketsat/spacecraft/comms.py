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

**Transmit draw** is :func:`transmit_draw_w`: a fixed draw while the transmitter is on
plus a per-byte draw times the bytes sent that tick. Power (#36) reads
``CommsTruth.transmit_power_w`` one tick late (comms steps after power).

**Traffic counters.** ``sent_bytes``, ``uplink_lost_count``, and
``outbound_suppressed_count`` depend on the frames the flight computer sends (#56) and
the uplink frames ``SilTarget`` delivers (#59). Neither exists yet and no input path is
defined, so in this version the counters are always 0 and the draw is computed with
``sent_bytes = 0``; #56 and #59 fill them.

Comms reads nothing from other subsystems, uses no randomness, and has no starting
record (ADR-0005). Arithmetic is portable (ADR-0006).
"""

from typing import Final

from pocketsat.core.clock import check_us
from pocketsat.core.rng import RngFactory
from pocketsat.spacecraft.config import CommsConfig
from pocketsat.spacecraft.controls import RadioMode, SpacecraftControls
from pocketsat.spacecraft.snapshots import CommsSnapshot, CommsTruth
from pocketsat.targets.base import EnvironmentState

__all__ = ["Comms", "transmit_draw_w"]


def transmit_draw_w(config: CommsConfig, transmitter_on: bool, sent_bytes: int) -> float:
    """Return the transmit draw for one tick, watts.

    ``transmitter_on_power_w + transmit_power_per_byte_w * sent_bytes`` while the
    transmitter is on, and 0.0 while it is off. Pure: no state, no randomness, and
    portable arithmetic only (ADR-0006). With ``sent_bytes = 0`` the result is exactly
    ``transmitter_on_power_w``.

    Args:
        config: Communications settings.
        transmitter_on: Whether the transmitter is on this tick.
        sent_bytes: Bytes actually sent this tick, 0..``transmit_capacity_bytes``.

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
    reads nothing from other subsystems. See the module docstring for the model.

    After :meth:`reset`, and before the first step, the radio is ``OFF``: receiver and
    transmitter off, capacity 0, no draw, counters 0. The first step applies the
    commanded mode.

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
        self._snapshot = self._build_snapshot(RadioMode.OFF)

    def reset(self, rng: RngFactory) -> None:
        """Return to the reset state (radio ``OFF``, counters 0). Comms requests no
        random streams."""
        self._restore()

    def step(self, dt_us: int, env: EnvironmentState, controls: SpacecraftControls) -> None:
        """Apply this tick's radio mode.

        The capacity is per tick, so ``dt_us`` does not scale it.

        Raises:
            TypeError: ``dt_us`` is not an int.
            ValueError: ``dt_us`` is negative.
        """
        check_us("dt_us", dt_us)
        mode = controls.radio.mode
        if mode is not self._mode:
            self._mode = mode
            self._snapshot = self._build_snapshot(mode)

    def _build_snapshot(self, mode: RadioMode) -> CommsSnapshot:
        config = self._config
        transmitter_on = _TRANSMITTER_ON[mode]
        sent_bytes = 0  # filled by the flight computer's downlink (#56)
        return CommsSnapshot.from_truth(
            CommsTruth(
                radio_mode=mode,
                receiver_on=_RECEIVER_ON[mode],
                transmitter_on=transmitter_on,
                transmit_capacity_bytes=config.transmit_capacity_bytes if transmitter_on else 0,
                sent_bytes=sent_bytes,
                uplink_lost_count=0,  # filled by SilTarget's uplink delivery (#59)
                outbound_suppressed_count=0,  # filled by the flight computer (#56)
                transmit_power_w=transmit_draw_w(config, transmitter_on, sent_bytes),
            )
        )

    def snapshot(self) -> CommsSnapshot:
        """Return the current state. Readings equal truth: comms has no sensors.

        The snapshot is rebuilt only when the mode changes, so repeated calls in an
        unchanged mode return the same object.
        """
        return self._snapshot
