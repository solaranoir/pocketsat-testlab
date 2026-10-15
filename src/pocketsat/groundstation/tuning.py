"""Radio tuning for Doppler (#162).

Each tick of a pass the station predicts the Doppler shift of both carriers from its
pass geometry (:func:`pocketsat.doppler.doppler_shift_hz`; the geometry is the station's
pass prediction, ADR-0010 §6) and retunes its radio:

- **Receive** on ``downlink_frequency_hz + correction``, so the shifted downlink lands on
  the tuned frequency.
- **Transmit** on ``uplink_frequency_hz - correction``, so the spacecraft hears the
  nominal uplink frequency once the uplink is shifted on its way up.

Each correction is the predicted shift rounded to a whole number of
``StationConfig.tuning_step_hz`` (ties round up), so tuned frequencies are whole hertz
and the **residual tuning error**, shift minus correction, is at most half a step. With
``StationConfig.doppler_correction`` off the corrections are zero: the radio stays on
the nominal frequencies and the residual is the full shift.

The residual is computed against the station's own prediction. In Phase 2 the station's
geometry and the link's are the same record, so it is also the true residual: the
downlink packets of the tick carry the same shift (``Packet.doppler_hz``). Phase 4's
Doppler tracking error either turns the correction off or gives the station a perturbed
geometry; the true residual is then the packets' shift against the tuned receive
frequency.
"""

import math

from pocketsat.doppler import doppler_shift_hz
from pocketsat.environment.geometry import PassGeometry
from pocketsat.groundstation.events import RadioTuned


class DopplerTuner:
    """Turns a tick's predicted geometry into the radio's tuning (#162).

    Args:
        uplink_frequency_hz: Nominal uplink carrier, Hz.
        downlink_frequency_hz: Nominal downlink carrier, Hz.
        tuning_step_hz: Tuning resolution, Hz, positive.
        correction: Whether to correct for Doppler at all.
    """

    def __init__(
        self,
        uplink_frequency_hz: int,
        downlink_frequency_hz: int,
        tuning_step_hz: int,
        correction: bool,
    ) -> None:
        self._uplink_hz = uplink_frequency_hz
        self._downlink_hz = downlink_frequency_hz
        self._step_hz = tuning_step_hz
        self._correction = correction

    def tune(self, now_us: int, geometry: PassGeometry) -> RadioTuned:
        """Tune for ``geometry`` and report it.

        Args:
            now_us: Simulated time, stamped on the event.
            geometry: The station's predicted geometry for the tick.

        Returns:
            The predicted shifts, tuned frequencies and residuals.
        """
        range_rate = geometry.range_rate_km_per_s
        uplink_doppler = doppler_shift_hz(self._uplink_hz, range_rate)
        downlink_doppler = doppler_shift_hz(self._downlink_hz, range_rate)
        if self._correction:
            uplink_correction = round_to_step(uplink_doppler, self._step_hz)
            downlink_correction = round_to_step(downlink_doppler, self._step_hz)
        else:
            uplink_correction = downlink_correction = 0
        return RadioTuned(
            time_us=now_us,
            uplink_doppler_hz=uplink_doppler,
            downlink_doppler_hz=downlink_doppler,
            uplink_frequency_hz=self._uplink_hz - uplink_correction,
            downlink_frequency_hz=self._downlink_hz + downlink_correction,
            uplink_residual_hz=uplink_doppler - uplink_correction,
            downlink_residual_hz=downlink_doppler - downlink_correction,
        )


def round_to_step(shift_hz: float, step_hz: int) -> int:
    """Round ``shift_hz`` to the nearest whole multiple of ``step_hz``; ties round up.

    ``floor`` is exact on every platform (ADR-0006), so the result is too.
    """
    return step_hz * math.floor(shift_hz / step_hz + 0.5)
