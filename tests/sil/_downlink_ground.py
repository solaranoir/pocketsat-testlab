"""The ground's view of a DOWNLINK pass, for the #56 SIL tests.

:class:`Ground` drives a ``SilTarget`` with the real flight computer the way the ground
will: COMMAND frames in with ``send()``, ``advance()`` one tick at a time, the downlink
drained with ``receive()`` and decoded. In every tick it checks what #56 and story #42
require of the downlink (see :meth:`Ground.tick`). It reads the payload's state from
``SilTarget.last_tick`` only to check the accounting, never to drive the run.
"""

from pocketsat.core.clock import DEFAULT_TICK_US
from pocketsat.flight import FlightComputer, FlightComputerConfig, Mode
from pocketsat.flight.boot import BootConfig
from pocketsat.frame import Frame, FrameType, decode_frame, encode_frame
from pocketsat.messages import (
    Command,
    CommandAck,
    Telemetry,
    decode_ack,
    decode_data,
    decode_telemetry,
    encode_command,
)
from pocketsat.spacecraft import (
    DEFAULT_INITIAL_STATE,
    NOMINAL_CONFIG,
    PayloadInitial,
    PayloadSnapshot,
    PayloadTruth,
    PowerSnapshot,
    SpacecraftInitialState,
    ThermalSnapshot,
    chunk_content,
)
from pocketsat.targets.base import EnvironmentState
from pocketsat.targets.sil import SilTarget, SilTick

TICK = DEFAULT_TICK_US
CHUNK = NOMINAL_CONFIG.payload.chunk_size_bytes
BUFFER = NOMINAL_CONFIG.payload.buffer_capacity_bytes
SHORT_BOOT = FlightComputerConfig(boot=BootConfig(duration_us=10 * TICK))
"""A 1 s boot, so the flight computer reaches NOMINAL after 10 ticks."""


def payload_with(chunks: int) -> PayloadInitial:
    """A starting buffer of ``chunks`` whole chunks plus half a chunk."""
    return PayloadInitial(buffer_fill=(chunks * CHUNK + CHUNK // 2) / BUFFER)


class Ground:
    """A ``SilTarget`` with the real flight computer, seen from the ground.

    Every tick (:meth:`tick`) checks:

    - **Released only once transmitted** (#56): every chunk the payload has released
      was in a DATA frame that ``receive()`` returned in an earlier tick. No
      timer-based release.
    - **Data accounting** (story #42): produced = buffered + released, and the bytes of
      the chunks sent minus the bytes released are exactly the chunks sent in this
      tick, so never more than one tick's worth.
    - **Radio-transmit inhibits** (story #42): no DATA in a tick whose power or thermal
      readings carry a flag.
    - **Order:** each chunk is received once, in ID order, with the content its ID
      gives (#43).

    Args:
        initial: The starting state (:func:`payload_with` gives a starting buffer).
        config: The flight computer's settings (default: a 1 s boot).
        seed: The run's seed.
        record_ticks: Keep every tick's record in :attr:`ticks` and its DATA in
            :attr:`data_per_tick`; off for multi-orbit runs.
    """

    def __init__(
        self,
        initial: SpacecraftInitialState = DEFAULT_INITIAL_STATE,
        *,
        config: FlightComputerConfig = SHORT_BOOT,
        seed: int = 3,
        record_ticks: bool = True,
    ) -> None:
        self.computers: list[FlightComputer] = []
        self.ticks: list[SilTick] = []
        self.record_ticks = record_ticks

        def factory() -> FlightComputer:
            computer = FlightComputer(config)
            self.computers.append(computer)
            return computer

        self.target = SilTarget(initial=initial, flight_computer_factory=factory)
        self.target.connect()
        self.target.reset(seed)
        self.count = 0
        """Ticks run so far."""
        self.sequence = 0
        self.received: dict[int, int] = {}
        """Chunk ID -> the tick it was received in."""
        self.order: list[int] = []
        self.data_per_tick: list[list[int]] = []
        self.telemetry: list[tuple[int, Telemetry]] = []
        self.acks: list[CommandAck] = []
        self.oldest = 0

    @property
    def mode(self) -> Mode:
        return self.computers[-1].mode

    @property
    def last(self) -> SilTick:
        tick = self.target.last_tick
        if tick is None:
            raise AssertionError("no tick yet")
        return tick

    def payload(self) -> PayloadTruth:
        return self.last.state.get("payload", PayloadSnapshot).truth

    def flags(self) -> tuple[bool, bool]:
        """(any power flag, any thermal flag) in the last tick's readings."""
        state = self.last.state
        power = state.get("power", PowerSnapshot).readings
        thermal = state.get("thermal", ThermalSnapshot).readings
        return power.low_battery or power.critical_battery, thermal.over_temp or thermal.under_temp

    def tick(self, *commands: Command, env: EnvironmentState | None = None) -> list[int]:
        """Send ``commands``, run one tick, receive and check the downlink; return the
        chunk IDs received."""
        if env is not None:
            self.target.apply_environment(env)
        for command in commands:
            self.sequence += 1
            frame = Frame(FrameType.COMMAND, self.sequence, encode_command(command))
            self.target.send(encode_frame(frame))
        self.target.advance(TICK)
        n = self.count
        self.count += 1
        record = self.last
        if self.record_ticks:
            self.ticks.append(record)

        # Released only once transmitted: what the payload released this tick was
        # received in an earlier tick.
        payload = record.state.get("payload", PayloadSnapshot).truth
        for chunk_id in range(self.oldest, payload.oldest_unreleased_chunk_id):
            assert self.received[chunk_id] < n, f"chunk {chunk_id} released before it was sent"
        self.oldest = payload.oldest_unreleased_chunk_id

        sent: list[int] = []
        for raw in self.target.receive():
            frame = decode_frame(raw)
            if frame.frame_type is FrameType.DATA:
                chunk = decode_data(frame.payload)
                assert chunk.content == chunk_content(chunk.chunk_id, CHUNK)
                assert chunk.chunk_id not in self.received, "a chunk was sent twice"
                assert not self.order or chunk.chunk_id == self.order[-1] + 1, "out of order"
                self.received[chunk.chunk_id] = n
                self.order.append(chunk.chunk_id)
                sent.append(chunk.chunk_id)
            elif frame.frame_type is FrameType.TELEMETRY:
                self.telemetry.append((n, decode_telemetry(frame.payload)))
            elif frame.frame_type is FrameType.ACK:
                self.acks.append(decode_ack(frame.payload))
        if self.record_ticks:
            self.data_per_tick.append(sent)

        # Data accounting (story #42).
        assert payload.total_produced_bytes == payload.buffered_bytes + payload.total_released_bytes
        assert len(self.received) * CHUNK - payload.total_released_bytes == len(sent) * CHUNK
        # Radio-transmit inhibits (story #42), from the readings.
        if sent:
            assert self.flags() == (False, False), "DATA sent while a power or thermal flag was set"
        return sent

    def until(self, mode: Mode, limit: int = 5_000, env: EnvironmentState | None = None) -> int:
        """Tick until the flight computer is in ``mode``; return the ticks taken."""
        for n in range(limit):
            if self.mode is mode:
                return n
            self.tick(env=env)
        raise AssertionError(f"never reached {mode.name}")
