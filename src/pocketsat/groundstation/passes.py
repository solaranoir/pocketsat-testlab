"""Pass state: AOS and LOS against the station's elevation mask (#161).

The station is in a pass while the spacecraft is at or above the elevation mask
(``StationConfig.elevation_mask_deg``). :class:`PassTracker` follows the pass geometry
the run loop gives :meth:`~pocketsat.groundstation.GroundStation.begin_tick` each tick,
which is the station's pass prediction (ADR-0010 §6), and emits :class:`Aos` on the first
tick at or above the mask and :class:`Los` on the first tick below it. Passes are numbered
from 1 after each reset.

**The mask is the station's policy, not physics** (ADR-0010 §5). The link applies the
geometric horizon (0°) on its own, so between the horizon and the mask frames can still
arrive while the station is not in a pass. Radio control (#164) stops transmission below
the mask.

**Sampling.** The geometry is sampled once per tick, at the tick's start, so AOS and LOS
are reported on the first tick that shows the crossing: within one tick of the true
crossing. The maximum elevation is the highest sample of the pass, so it is below the
true peak by the change in elevation within half a tick of it (well under 0.01° at the
default 100 ms tick).

**No hysteresis and no minimum pass duration.** Neither ADR-0010 nor ADR-0011 calls for
one: the geometry is deterministic and noise-free (ADR-0011), so the elevation crosses the
mask once on the way up and once on the way down, and a pass that only grazes the mask is
a real, short pass that a test may want to see. Phase 4's tracking-error faults perturb
the station's geometry; if they make the elevation chatter around the mask, hysteresis
belongs with them.

**No geometry.** ``geometry=None`` (no geometry model runs) leaves the pass state
unchanged and emits nothing: a station that is never given geometry is never in a pass,
and the whole run is one pass for the per-pass link statistics (#156).
"""

from dataclasses import dataclass

from pocketsat.environment.geometry import PassGeometry
from pocketsat.groundstation.events import Aos, Los, StationEvent


@dataclass(frozen=True)
class PassState:
    """The station's pass state, read-only.

    Attributes:
        in_pass: True between AOS and LOS.
        pass_id: The current pass while in a pass, otherwise the last pass; 0 before the
            first AOS.
        max_elevation_deg: Highest elevation sampled in the current pass so far, or
            ``None`` outside a pass.
    """

    in_pass: bool = False
    pass_id: int = 0
    max_elevation_deg: float | None = None


class PassTracker:
    """The AOS/LOS state machine (#161).

    Args:
        elevation_mask_deg: The station's elevation mask: in a pass at or above it.
    """

    def __init__(self, elevation_mask_deg: float) -> None:
        self._mask_deg = elevation_mask_deg
        self._state = PassState()
        self._max_elevation_deg = 0.0
        """The pass's highest sample, kept as a float while in a pass and copied into
        :attr:`state` when read, so a rising pass does not build a record every tick."""

    @property
    def state(self) -> PassState:
        """The current pass state."""
        state = self._state
        if state.in_pass and state.max_elevation_deg != self._max_elevation_deg:
            state = self._state = PassState(
                in_pass=True, pass_id=state.pass_id, max_elevation_deg=self._max_elevation_deg
            )
        return state

    def update(self, now_us: int, geometry: PassGeometry | None) -> tuple[StationEvent, ...]:
        """Take one tick's geometry and return the AOS or LOS it causes, if any.

        Args:
            now_us: Simulated time of the sample, stamped on the event.
            geometry: The tick's pass geometry, or ``None`` (state unchanged).

        Returns:
            An :class:`Aos`, a :class:`Los`, or nothing.
        """
        if geometry is None:
            return ()
        elevation = geometry.elevation_deg
        above = elevation >= self._mask_deg
        state = self._state
        if not state.in_pass:
            if not above:
                return ()
            pass_id = state.pass_id + 1
            self._state = PassState(in_pass=True, pass_id=pass_id, max_elevation_deg=elevation)
            self._max_elevation_deg = elevation
            return (
                Aos(
                    time_us=now_us,
                    pass_id=pass_id,
                    azimuth_deg=geometry.azimuth_deg,
                    elevation_deg=elevation,
                ),
            )
        if above:
            if elevation > self._max_elevation_deg:
                self._max_elevation_deg = elevation
            return ()
        self._state = PassState(in_pass=False, pass_id=state.pass_id)
        return (
            Los(
                time_us=now_us,
                pass_id=state.pass_id,
                azimuth_deg=geometry.azimuth_deg,
                elevation_deg=elevation,
                max_elevation_deg=self._max_elevation_deg,
            ),
        )
