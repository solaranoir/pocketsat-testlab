"""#72's operating profiles and traffic stand-ins, shared by the SIL tests that use them.

The power and thermal budget tests (#72, ``test_power_thermal_budget.py``) and the
epic #30 close-out test (#70, ``test_epic_30_closeout.py``) drive the five real
subsystems with the same **reference profile**: SCIENCE for the whole orbit except one
10-minute DOWNLINK pass at the end of each orbit. There is no flight computer yet, so
:class:`ProfileDriver` scripts ``SpacecraftControls`` the way the flight computer's mode
table will (#47): payload enabled in SCIENCE and DOWNLINK, radio ``RX_TX`` in every mode
(ADR-0004 §10), attitude control on.

**Stand-ins until #56 and #59 send real traffic** (comms' ``sent_bytes`` is always 0):

- *Traffic energy.* The bytes a real flight computer would send are costed with
  :func:`~pocketsat.spacecraft.comms.transmit_draw_w` and added through
  ``SpacecraftControls.extra_load_w`` in the ticks they would be sent: an assumed
  beacon of one :data:`TELEMETRY_FRAME_BYTES` telemetry frame per second outside the
  pass, and full transmit capacity in every tick of the DOWNLINK pass. Comms' own
  truth already carries the idle draw (``transmitter_on_power_w``), so only the
  per-byte part is added.
- *Data release.* In each pass tick the driver releases as many of the oldest chunks as
  whole DATA frames fit in the capacity left after telemetry and ACK/NACK (ADR-0004
  §10 priority), as #56 will.

Re-check the budget once #56 and #59 send real traffic.
"""

from dataclasses import dataclass
from typing import Literal

from pocketsat.core.rng import RngFactory
from pocketsat.frame import MIN_FRAME_SIZE
from pocketsat.spacecraft import (
    CHUNK_ID_SIZE_BYTES,
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    Attitude,
    AttitudeControls,
    Comms,
    Payload,
    PayloadControls,
    PayloadSnapshot,
    Power,
    RadioControls,
    RadioMode,
    SnapshotBoard,
    SpacecraftConfig,
    SpacecraftControls,
    SpacecraftInitialState,
    SpacecraftState,
    Subsystem,
    SubsystemStack,
    Thermal,
    transmit_draw_w,
)

US_PER_S = 1_000_000

# --- Traffic assumptions (stand-ins until #56 and #59) ---------------------------------

TELEMETRY_PAYLOAD_BYTES = 54
"""Assumed telemetry payload, bytes. #54's field list (uptime, mode, flags, voltage,
SOC, two temperatures, pointing error, buffer fill, radio, attitude and payload states,
boot count) encodes to about 24 bytes; 54 leaves room for growth."""

TELEMETRY_FRAME_BYTES = MIN_FRAME_SIZE + TELEMETRY_PAYLOAD_BYTES
"""Assumed telemetry (beacon) frame, bytes: 64 with the 10-byte frame overhead
(``docs/protocol.md``)."""

ACK_FRAME_BYTES = MIN_FRAME_SIZE + 3
"""Assumed ACK/NACK frame, bytes: acknowledged sequence (2) and status (1) plus the
frame overhead, 13."""

BEACON_PERIOD_US = US_PER_S
"""One telemetry frame per second, in every mode."""

PASS_US = 10 * 60 * US_PER_S
"""DOWNLINK pass length: 10 minutes, at the end of each orbit (the end of eclipse, the
worst place for the minimum SOC)."""

Downlink = Literal["none", "pass", "continuous"]


@dataclass(frozen=True)
class Profile:
    """An operating profile: what the flight computer would command."""

    name: str
    payload: bool = True
    attitude: bool = True
    downlink: Downlink = "pass"


REFERENCE = Profile("reference (SCIENCE + 10-min DOWNLINK)")
"""#72's reference profile, used by #63 and #70."""


def real_subsystems(
    board: SnapshotBoard,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
) -> dict[str, Subsystem]:
    """The five real subsystems, by name, reading each other through ``board``."""
    return {
        "power": Power(config.power, initial.power, reader=board),
        "thermal": Thermal(config.thermal, initial.thermal, reader=board),
        "attitude": Attitude(config.attitude, initial.attitude),
        "payload": Payload(config.payload, initial.payload, reader=board),
        "comms": Comms(config.comms),
    }


def real_stack(
    seed: int,
    config: SpacecraftConfig = NOMINAL_CONFIG,
    initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
) -> SubsystemStack:
    """The five real subsystems in one stack (default ``STEP_ORDER``), reset with ``seed``."""
    board = SnapshotBoard()
    stack = SubsystemStack(real_subsystems(board, config, initial).values(), board=board)
    stack.reset(RngFactory(seed))
    return stack


def data_frame_bytes(config: SpacecraftConfig) -> int:
    """One chunk's DATA frame: chunk ID, chunk content, and the frame overhead (#56)."""
    return CHUNK_ID_SIZE_BYTES + config.payload.chunk_size_bytes + MIN_FRAME_SIZE


def data_frames_in_tick(config: SpacecraftConfig, beacon_tick: bool) -> int:
    """Whole DATA frames that fit in one pass tick after ACK/NACK and telemetry.

    Frames that don't fit are not queued (ADR-0004 §10). In the beacon tick the
    telemetry frame and an assumed ACK/NACK come first.
    """
    overhead = TELEMETRY_FRAME_BYTES + ACK_FRAME_BYTES if beacon_tick else 0
    return max(0, config.comms.transmit_capacity_bytes - overhead) // data_frame_bytes(config)


@dataclass(frozen=True)
class TickCommand:
    """What :class:`ProfileDriver` commands for one tick."""

    controls: SpacecraftControls
    in_pass: bool
    """Whether this tick is in a DOWNLINK pass."""
    traffic_w: float
    """Per-byte transmit draw of the stand-in traffic, watts (included in
    ``controls.extra_load_w``)."""


class ProfileDriver:
    """Scripts ``SpacecraftControls`` for ``profile``, tick by tick (see the module
    docstring for the traffic and release stand-ins)."""

    def __init__(
        self,
        profile: Profile,
        config: SpacecraftConfig,
        orbit_period_us: int,
        extra_load_w: float = 0.0,
    ) -> None:
        """Create a driver.

        Args:
            profile: What to command.
            config: The spacecraft settings (capacity, chunk size, transmit draw).
            orbit_period_us: The environment's orbit period; the pass is the last
                :data:`PASS_US` of each orbit.
            extra_load_w: A constant extra load added to every tick, watts.
        """
        self.profile = profile
        comms = config.comms
        idle_w = transmit_draw_w(comms, True, 0)
        self.beacon_w = transmit_draw_w(comms, True, TELEMETRY_FRAME_BYTES) - idle_w
        """Per-byte draw of one telemetry frame, watts."""
        self.pass_w = transmit_draw_w(comms, True, comms.transmit_capacity_bytes) - idle_w
        """Per-byte draw of a tick at full transmit capacity, watts."""
        self._frames = {beacon: data_frames_in_tick(config, beacon) for beacon in (False, True)}
        self._period_us = orbit_period_us
        self._pass_start_us = orbit_period_us - PASS_US
        self._extra_load_w = extra_load_w
        self._attitude = AttitudeControls(enabled=profile.attitude)
        self._radio = RadioControls(mode=RadioMode.RX_TX)

    def command(self, now_us: int, previous: SpacecraftState) -> TickCommand:
        """The controls for the tick starting at ``now_us``, given the state before it.

        In a pass tick, releases the oldest chunks that fit in this tick's DATA frames,
        read from ``previous`` (the flight computer acts on the last published state).
        """
        profile = self.profile
        beacon = now_us % BEACON_PERIOD_US == 0
        in_pass = profile.downlink == "continuous" or (
            profile.downlink == "pass" and now_us % self._period_us >= self._pass_start_us
        )
        release = None
        if in_pass:
            traffic_w = self.pass_w
            chunks = self._frames[beacon]
            payload = previous.get("payload", PayloadSnapshot).truth
            if chunks and payload.next_chunk_id > payload.oldest_unreleased_chunk_id:
                release = (
                    min(payload.oldest_unreleased_chunk_id + chunks, payload.next_chunk_id) - 1
                )
        else:
            traffic_w = self.beacon_w if beacon else 0.0
        controls = SpacecraftControls(
            payload=PayloadControls(enabled=profile.payload, release_through_chunk_id=release),
            radio=self._radio,
            attitude=self._attitude,
            extra_load_w=traffic_w + self._extra_load_w,
        )
        return TickCommand(controls=controls, in_pass=in_pass, traffic_w=traffic_w)
