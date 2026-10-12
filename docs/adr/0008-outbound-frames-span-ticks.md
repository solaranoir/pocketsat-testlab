# ADR-0008: Outbound frames span ticks

- **Status:** Accepted (implemented by #132)
- **Phase:** 1
- **Amends / Supersedes:** Amends ADR-0004 (§2 the latency table, §9 the RESET command's reboot timing at short ticks, §10 outbound capacity and priority) and ADR-0007 (§1 where `sent_bytes` comes from, §3 who counts what, §4 how the flight computer keeps within the capacity). Related: ADR-0002 (`TestTarget`, HIL), ADR-0003 (determinism), ADR-0006 (portable arithmetic). Issue: #132. Follows #122 (PR #131).

## Context

Since #122, comms' transmit capacity comes from a data rate: `CommsConfig.transmit_rate_bytes_per_s` (1200 bytes/s) is converted into each tick's `transmit_capacity_bytes` exactly, with an integer carry held while the transmitter is off. The per-byte draw is scaled to the tick length, so a byte costs 2 mJ at any tick length (#131, decisions 1 to 3).

The flight computer's outbound queue (#51) still sends only **whole frames that fit what is left of the current tick's capacity** (`TickContext.reserve_outbound`). A frame that does not fit is suppressed and counted (ACK/NACK, telemetry), or left in the payload buffer (DATA, #56). #131's decision 4 kept this: frames are never split across ticks. The frames are:

| Frame | Size on the wire |
|---|---|
| ACK/NACK (#51) | 14 bytes |
| TELEMETRY (#54, #55) | 36 bytes |
| DATA (#56), 64-byte chunk | 78 bytes |

At the default rate, a tick shorter than 65 ms holds fewer than 78 bytes, so a DATA frame never fits, and a DOWNLINK pass never completes. A tick shorter than 30 ms cannot send telemetry, and one shorter than about 11.7 ms cannot send an ACK. A real radio streams bytes continuously: a frame longer than one software cycle takes several cycles to go out. Everything runs at the 100 ms default today, where every frame fits, so this is a model limitation rather than a live bug. #132 asks for the model to be correct at any tick length, with **no change at the default 100 ms tick**.

The constraints come from the accepted ADRs and tickets:

- **Comms reads nothing** (ADR-0004 §1, §3; ADR-0007 §1). It learns about traffic only through `controls.radio_traffic`, which `SilTarget` fills at step a of tick N+1 with tick N's per-tick values. Comms validates the bytes against the capacity of the tick they were sent in, charges them in N+1, and power sees the energy in N+2 (ADR-0007 §2, §4).
- **One controls function, one merge** (ADR-0004 §14). The flight computer produces controls only in `controls_for_mode`; `SilTarget` merges overrides and traffic only in `_merge_controls`.
- **`TestTarget` is bytes in, bytes out** (ADR-0002). `receive()` returns whole frames. SIL and HIL must behave the same at that boundary.
- **Within-tick order** (ADR-0004 §2): a. merge, b. subsystems step (comms last), c. decode uplink, execute commands, ACK/NACK, d. flags and mode, e. telemetry and DATA, f. controls for N+1. The flight computer runs at the end of tick N (`now_us` is the end of the tick). The frames it sends in steps c and e are the frames on air during tick N.
- **Priority** (ADR-0004 §10): ACK/NACK, then telemetry, then DATA.
- **ACK timing** (ADR-0004 §2 latency table, `docs/protocol.md` "Dispatch and ACK timing"): command → ACK in the same tick. The RESET ACK goes out in the RESET tick, numbered from before the reboot (#49), which happens in step d of that tick (ADR-0004 §9).
- **Downlink session** (#56): chunks go out in ID order, once each. A chunk is released (applied by the payload in the next tick) only after its frame was sent. produced = buffered + released, and sent − released is the chunks sent in the tick. Any exit from DOWNLINK aborts the session, and no DATA goes out in that step. The next session resumes at the oldest unreleased chunk.
- **Telemetry** (#55): a due frame uses its slot whether or not it is sent, and a suppressed frame is not retried.
- **`transmitter_off`** (#60): commands are executed, but every ACK and telemetry frame is suppressed and counted, and none is sent later. **Reboots** (#49, #60) clear transient flight computer state; `forced_reset` may hold the flight computer, which is then not stepped.
- **Sequence numbers** (#51): one downlink counter, used only by frames actually sent, so a gap on the ground means a frame lost on the way down.
- **Determinism and arithmetic** (ADR-0003, ADR-0006): integers only.
- **Performance** (#78, #120): at the 100 ms tick, at most 100 µs per tick averaged over an orbit, and 150 µs for the worst-case tick on CI Linux.
- **No change at 100 ms**: downlink bytes, `SpacecraftState` sequences, the golden captures (`tests/golden/`) and #121's digests (`tests/sil/test_behavior_digests.py`) stay identical.

## Decision

### 1. Spanning lives in the flight computer's outbound queue (option A)

The flight computer's outbound queue (`_queue_outbound`, `TickContext.reserve_outbound`) gains a **transmitter**: at most one frame in transmission, carried from tick to tick as transient flight computer state, plus a small queue of ACK/NACK and telemetry frames **held** while that frame occupies the transmitter. Each tick, the flight computer spends that tick's `transmit_capacity_bytes`, which it already reads from comms' readings, on the frame in transmission first, then on new frames chosen in priority order.

The flight computer models the radio driver's transmit FIFO and serializer as part of the flight software. That is where they live on the Phase 7 MCU (section 9). Comms is unchanged: it still exposes a capacity and is told the bytes. `SilTarget` is unchanged apart from reading one more output field. Section 11 compares the options.

### 2. Two kinds of tick: whole-frame ticks and streaming ticks

Let **`L_max`** be the largest frame the flight software sends: `max(ACK 14, TELEMETRY 36, DATA 14 + 4 + chunk_size_bytes)`. That is 78 bytes with the default 64-byte chunk. It is a build-time constant of the flight software, derived from `FlightComputerConfig.downlink`.

Each tick N, with capacity `C_N` from the readings, is one of two kinds:

- **A whole-frame tick:** `C_N ≥ L_max`, no frame is in transmission, and no frame is held. **Today's rule applies unchanged.** Frames are offered in priority order. Each frame is sent whole if it fits what is left. A frame that does not fit is suppressed and counted (ACK/NACK, telemetry) or left for later (DATA, with the session stopping at the first chunk that does not fit). A frame that does not fit does not block a later, smaller one. No frame spans: every frame fits a tick of this size.
- **A streaming tick:** any other tick. That is, `C_N < L_max`, or a frame is in transmission, or a frame is held. The transmitter streams frames back to back, as a serializer does:
  1. The frame in transmission continues first, for `min(remaining, C_N)` bytes. It is never preempted.
  2. Then, whenever the transmitter is idle and bytes are left, the highest-priority waiting frame is **committed**: held ACK/NACK (oldest first), this tick's ACK/NACK, held or due telemetry, then the downlink session's next chunk. A committed frame takes what is left of the tick and, if that is not enough, continues in the next tick. Committing it makes it the frame in transmission.
  3. When the tick's bytes run out, ACK/NACK and telemetry that are still waiting are **held** for the next tick (section 5), not suppressed. DATA is never held as a frame: the session offers its next chunk when the transmitter is next idle.

At the default 1200 bytes/s, `C_N ≥ 78` means a tick of at least 65 ms. **At 100 ms every tick is a whole-frame tick.** The capacity is 120 every tick. No frame is ever in transmission at a tick boundary, because every frame fits and is sent whole. So no frame is ever held. Section 7 gives the full argument.

Why not one rule for every tick (always stream)? Section 11 covers this. Streaming at 100 ms would use the bytes that today's rule leaves unused. For example, ACK + telemetry + DATA is 128 > 120 bytes; today the DATA frame waits a tick, but a streamer would start it and finish it in the next tick. That changes the downlink bytes, the golden captures and the digests. The two-kind rule is the smallest change that makes every tick length work and keeps 100 ms bit-identical. Open question 1 asks whether to move to always-streaming later.

### 3. Delivery timing and what is counted per tick

- **Delivery.** A frame is delivered (appended to `FlightComputerOutput.downlink_frames`, and so returned by `receive()`) in the tick its **last byte** is transmitted. Delivered frames are in transmit order. A frame is never split at the `TestTarget` boundary: the ground still receives whole frames.
- **Bytes per tick.** `FlightComputerOutput.sent_bytes` becomes an explicit field: the bytes **transmitted in this tick**. That includes the continuation of a frame started earlier and the first part of a frame that continues later. It is at most `C_N`. It is no longer the total length of the frames delivered in the tick: those can exceed `C_N` when a frame completes whose earlier bytes went out in earlier ticks. In a whole-frame tick the two are equal, as today.
- **Comms and energy.** `SilTarget` passes `sent_bytes` on in `RadioTraffic.sent_bytes` exactly as now. Comms reports it as `previous_tick_sent_bytes` in N+1 and validates it against tick N's capacity. That check still holds, because the flight computer never transmits more than `C_N` in tick N. Comms charges the bytes with the per-byte draw scaled to tick N's length, and power sees the energy in N+2. **Energy follows the bytes, not the frames.** A 78-byte frame sent over 3 ticks at 30 ms is charged 36 + 36 + 6 bytes, in the ticks after each, for a total of 78 × 2 mJ. The bytes of a frame that is later aborted (section 6) were transmitted, so they are charged.
- **Composition with ADR-0007's timing.** ADR-0007's N+1/N+2 delays apply **per byte**, counted from the tick the byte went on air, not from the tick the frame is delivered. The path (`RadioTraffic` in `SpacecraftControls`, filled by `SilTarget` at step a, read only by comms) and the step order do not change. The amendment only changes where the per-tick number comes from: the flight computer's transmitter, instead of the sum of the delivered frames' lengths.
- **Sequence numbers** are assigned when a frame is **committed** (its first byte goes on air), from the one downlink counter. The frame is encoded then. In a whole-frame tick, committing a frame is sending it, so numbering is exactly as today. A held frame gets its number when it is committed, so frames are numbered in transmit order. A frame that is suppressed before it is committed uses no number. The content is fixed when the frame is created: an ACK's payload when its command is executed (step c of the command's tick), and telemetry's when it is due (step e of the due tick, from that tick's readings and mode).

Worked example: 30 ms tick, default rate, so `C = 36` every tick, in a DOWNLINK pass with no telemetry due:

| Tick | On air | Delivered |
|---|---|---|
| 1 | DATA 0 committed: 36 of 78 | — |
| 2 | DATA 0: 36 (6 left) | — |
| 3 | DATA 0: 6, done; DATA 1 committed: 30 | DATA 0 |
| 4 | DATA 1: 36 (12 left) | — |
| 5 | DATA 1: 12, done; DATA 2 committed: 24 | DATA 1 |
| … | 6 frames in 13 ticks = 468 bytes = 13 × 36 | |

Every tick sends its full 36 bytes while there is something to send, so the long-run rate is exactly the configured 1200 bytes/s. DATA 0 is released in tick 3's step f, and the payload applies the release in tick 4.

### 4. Priority applies when a frame is chosen; a frame in transmission finishes first

- Priority (ACK/NACK > telemetry > DATA) decides **which frame is committed next**, at each point where the transmitter is idle. A frame in transmission is never preempted and never interleaved with another frame's bytes, whatever its class.
- The order of the phases still gives the priority order. ACK/NACK are offered in step c, telemetry and DATA in step e, and `reserve_outbound`'s check that a higher class is never offered after a lower one stays. Held ACK/NACK are offered at the start of step c, before this tick's ACK/NACK. Held telemetry is offered at the start of step e, before any newly due frame.
- No head-of-line blocking in whole-frame ticks (#51, as today). In streaming ticks the question does not arise: the next frame is committed whatever its size, and it spans if needed.

### 5. ACK/NACK and telemetry that come due while a frame is in transmission

They are **held, with a bound**. They are suppressed only when the bound overflows or the transmitter is off.

- **ACK/NACK:** a FIFO of at most **`max_held_ack_frames` = 8** frames (`FlightComputerConfig.outbound`, a new `OutboundConfig`). 8 is what one default 100 ms tick can carry: ⌊120 / 14⌋. If a ninth ACK is due while eight are held, the newest is suppressed and counted. That keeps the ACKs in command order, as today's capacity rule does.
- **Telemetry:** at most **one** frame held. If another frame comes due while one is still held, the newer frame (fresher readings) replaces it, and the older one is counted as suppressed. The scheduler is unchanged: a due frame uses its slot whether or not it is sent, and the next frame is due one period after. With the default periods (0.5 s or more) and at most 77 bytes of a frame ahead of it (64 ms of airtime at 1200 bytes/s), a held telemetry frame always goes out before the next one is due. So **telemetry keeps its period** at any tick length while the transmitter is on.
- **DATA** is never held. The session offers its next chunk only when the transmitter is idle and no ACK/NACK or telemetry is waiting. The order "stop at the first chunk that does not fit" becomes "one chunk at a time" in streaming ticks, so chunks still go out in ID order, once each.

**ACK latency.** "Command → ACK, same tick" (ADR-0004 §2) still holds in every whole-frame tick, **so it holds unchanged at the 100 ms default**, and at any tick of at least 65 ms at the default rate. The exceptions are the same as today: ACKs that do not fit the tick, and `transmitter_off`.

In a streaming tick, the ACK of a command received in tick N is delivered in the first tick `N + d` where

```
C_N + C_{N+1} + … + C_{N+d}  ≥  r + A + 14
```

- `r` is what is left of the frame in transmission at the start of tick N (0 if none).
- `A` is the bytes of the ACK/NACK frames ahead of it: the ones held from earlier ticks and the ones earlier in tick N.

With nothing ahead of it, the wait is bounded in **time**, not in ticks: at most `L_max − 1 + 14 = 91` bytes of airtime, about 76 ms at 1200 bytes/s. That is `d ≤ 2` at 30 ms (36 bytes per tick) and `d ≤ 1` at 50 ms (60 bytes per tick). The ACK's meaning is unchanged: it reports what the command did to the mode at the end of tick N, and its content is fixed in tick N.

### 6. Interruptions: abort and count; nothing is lost from the payload buffer

A partly transmitted frame is **aborted, not resent**. Its bytes so far were transmitted and stay charged. It is never delivered by `receive()`. On a real radio the receiver would see a truncated frame that fails sync, length or CRC.

| Interruption | What happens |
|---|---|
| **Transmitter off** (a tick with `C_N = 0` and a frame in transmission: `transmitter_off`, or the radio `RX_ONLY`/`OFF`) | The frame in transmission is aborted at the start of the tick. Held ACK/NACK and telemetry are suppressed and counted, as are this tick's (today's rule for capacity 0). None is sent later, as #60 requires. When the transmitter returns, the next tick starts idle. |
| **Reboot by the RESET command** | The reboot waits until the RESET ACK has been transmitted (section 6a). When it happens, in step d of the tick in which the ACK's last byte went out, frames completed in that tick before the reboot are delivered. A frame still in transmission at that point is aborted, and held frames are dropped. The reboot is at the end of the tick (`now_us`), after that tick's airtime. This matches the sequence restart: the next frame is numbered 0. |
| **Reboot by `forced_reset`** (between steps; a pulse, or the end of a hold) | A hardware-style reset: it does not wait for anything. The frame in transmission is aborted and held frames are dropped, when `reboot()` runs. That includes a RESET ACK whose reboot is pending (section 6a). **While held**, the flight computer is not stepped, so it transmits nothing: `sent_bytes` is 0, as today ("no telemetry, no new controls"). The frame stays in the flight computer's state, frozen, until the reboot aborts it. The counts are reported in the output of the next step. |
| **Leaving DOWNLINK** (SET_MODE, ENTER_SAFE_MODE, SAFE_CONDITION, FAULT_DETECTED in step d of tick N) | A DATA frame still in transmission at the end of tick N is aborted with its session, consistent with #56's "the session ends at once, and no DATA goes out in that step". Its chunk was never released, so it stays in the payload buffer, and the next session sends it first, in a new frame with a new sequence number. ACK/NACK and telemetry in transmission are not session state and continue. |
| **Radio-transmit inhibit** (a power or thermal flag, #56) | The flag stops the session from **committing** new DATA. A DATA frame already in transmission finishes, at most 77 bytes. The pass stays in DOWNLINK and resumes when the flag clears, as today. |

**Counting.**

- An aborted or dropped ACK/NACK or telemetry frame is counted in `outbound_suppressed_count`. Its meaning widens from "did not fit" to "due, but never delivered". So comms' running total still counts every ACK and telemetry frame the ground never received for the spacecraft's own reasons.
- An aborted DATA frame is not counted as suppressed, as for DATA that does not fit (ADR-0007 §3): its chunk stays buffered.
- Every abort, of any class, is also counted in a new per-tick `outbound_aborted_count` (section 8) for tests and run records. It is not passed to comms.

**The downlink session's release rule** is unchanged in substance: **a chunk is released only after its whole DATA frame has been transmitted.**

- The session's `last_sent_chunk_id` and `next_chunk_id` advance when the frame **completes**, not when it is committed. Step f releases through the last completed chunk, and the payload applies the release in the next tick.
- The chunk in transmission is neither sent nor released. It is still buffered, so an abort loses nothing, and the next session starts at it.
- While a DATA frame is in transmission, `DOWNLINK_COMPLETE` cannot be raised, because that chunk is still unsent.
- One corner needs an explicit rule. In a streaming tick, a DATA frame carried in from tick N−1 can complete at the start of tick N (before step c), and the same tick can end the session in step d. That happens on the last chunk, which raises `DOWNLINK_COMPLETE`, or on a command or flag. **Step f of that tick still releases the chunk that completed in it**, even though the session has ended. Otherwise a delivered chunk would stay buffered and go down twice. This corner never occurs in a whole-frame tick, where DATA completes only in step e of a running session, so the controls at 100 ms are unchanged.

**Data accounting** still holds at the end of every tick:

- produced = buffered + released (the payload, #43).
- Chunks delivered − chunks released = exactly the chunks delivered in that tick. That is at most one when `C_N < L_max`, because at most one 78-byte frame can complete in fewer than 78 bytes, and it is never more than one tick's worth.
- At most one further chunk is in flight (partly transmitted, unreleased, still buffered).

### 6a. A RESET command reboots only once its ACK has been transmitted (maintainer decision)

Real flight software makes sure the RESET ACK is out before it reboots, so the ground learns the reset was accepted. This section replaces "the RESET reboots in step d of its tick" for the one case where the ACK has not finished by then.

**Tick N, the RESET tick.**

- **Step c, as today.** The dispatcher answers the RESET in arrival order (the preview `transition()`, #51), raises its `RESET` event, and queues its ACK. Commands after it in tick N are still answered as in BOOT (a NACK `BOOT_IN_PROGRESS`, or another RESET's ACK).
- **Ordering in streaming ticks.** The RESET ACK is committed next: right after the frame in transmission, ahead of any ACK/NACK or telemetry still waiting, held or from earlier in tick N. This is the only exception to first-in, first-out ordering within the ACK class. It bounds the wait, below. The ACKs it overtakes keep their order behind it. In a whole-frame tick nothing is waiting, so the order is today's.
- **Step d.** Events before the `RESET` apply normally. Then one of three things happens:
  1. **The RESET ACK has completed in tick N.** It does in every whole-frame tick in which it fits. The `RESET` and every event after it apply exactly as today: `transition()` to BOOT, `reboot(now_us)`, automatic events after it IGNORED, and later command events through the table in BOOT.
  2. **The RESET ACK was suppressed or aborted.** That is, it did not fit a whole-frame tick, or the capacity is 0 (`transmitter_off`, `RX_ONLY`, `OFF`). It can never go out, so **the reboot happens now, as today**. The ACK is counted in `outbound_suppressed_count`.
  3. **Otherwise** (a streaming tick: the RESET ACK is in transmission, or waiting behind the frame in transmission), **the reboot is pending**. The `RESET` and every event after it in tick N are kept, in order, and applied at the reboot.

**While a reboot is pending** (from step d of tick N until the reboot), the flight computer is shutting down. In each step:

- **Uplink:** frames it is given are not decoded, executed or answered. They are counted in a new per-tick `FlightComputerOutput.uplink_ignored_count`. `SilTarget` adds this count to `SilTick.uplink_dropped_in_reset_count`, which has the same meaning: heard while no software could take it.
- **Flags and mode:** the evaluate-flags rules (#48 safe-mode and fault checks, `BOOT_COMPLETE`, `DOWNLINK_COMPLETE`) do not run, and no mode event is applied. The mode, the safety counters and the telemetry schedule stay as they were after step d of tick N. This pause lasts at most the bound below, about 76 ms of airtime.
- **Outbound:**
  - The transmitter keeps going: the frame in transmission, then the RESET ACK, then the frames waiting behind it, in order.
  - Nothing new is created: no telemetry is checked for being due, and no DATA is committed. The downlink session sends nothing new, but a DATA frame already in transmission finishes and is released by the completion rule (section 6).
- **Controls:** step f produces the controls of the unchanged mode, with the release of any chunk completed in this tick, as before the RESET. BOOT's controls come only after the reboot.
- **The reboot** happens in **step d of tick M**, the tick in which the RESET ACK's last byte is transmitted. The kept events apply there, in order, exactly as case 1 applies them: `transition()`, then `reboot(now_us=<end of tick M>)`. Then:
  - Frames completed in tick M are delivered: the RESET ACK, plus any frames behind it that also finished in M.
  - The frame in transmission is aborted. Held ACK/NACK and telemetry are dropped and counted as suppressed (section 6). The session ends.
  - Step f produces BOOT's controls, which apply from tick M+1. Uptime and the boot duration count from the end of tick M. The downlink sequence restarts at 0.

**The bound.** The RESET ACK is committed after at most the frame in transmission (`r ≤ L_max − 1` bytes left). So the reboot happens in the first tick M ≥ N with

```
C_N + … + C_M  ≥  r + 14
```

That is at most `L_max − 1 + 14 = 91` bytes of airtime after the start of tick N, about 76 ms at 1200 bytes/s. In ticks:

- 30 ms (36 bytes per tick): at most N+2.
- 50 ms (60 bytes per tick): at most N+1.
- A whole-frame tick: always N (case 1 or case 2), which keeps 100 ms bit-identical.

**Interactions.**

- **`transmitter_off` (or any capacity 0) during a pending reboot.** A tick with capacity 0 aborts the frame in transmission (section 6), and the RESET ACK can no longer go out. **The reboot happens in step d of that tick** instead of waiting for a transmitter that may never return. The RESET ACK, aborted or never committed, is counted in `outbound_suppressed_count`. The same applies when the capacity is 0 in tick N itself (case 2).
- **`forced_reset` during a pending reboot.** `forced_reset` is a hardware-style reset (ADR-0004 §9, #60): it reboots at the start of its tick (a pulse) or at the end of its hold, and it waits for nothing.
  - It **cancels** the pending reboot. The RESET ACK, if not yet delivered, is aborted and counted as suppressed.
  - The kept events are discarded, and **one** reboot happens: the boot count goes up by 1, not 2. This matches #60's rule that overlapping resets make one reboot.
  - During a hold the flight computer is not stepped, so nothing is transmitted, as today.
- **A second RESET command in tick N** is one of the kept events. At the reboot it goes through the table in BOOT, exactly as it would today. Its ACK is queued behind the first RESET's ACK and is delivered only if it completes by the reboot. No RESET command can arrive during the pending period, because uplink is ignored then.
- **A DATA frame in transmission at the RESET.** It finishes ahead of the RESET ACK and is released in the step it completes (section 6's completion rule). Its chunk therefore goes down once. Chunks never committed stay buffered, and the next session resumes at the oldest unreleased chunk.

**Reported state.** `FlightComputer.reboot_pending` (bool) is exposed for tests. `FlightComputerOutput.reboot_pending` and `SilTick.reboot_pending` record it per tick.

### 7. No change at the default 100 ms tick

At 1200 bytes/s and 100 ms, comms gives `C_N = 120` in every tick, with a carry of 0 (#131, asserted over 10,000 ticks). `L_max = 78 ≤ 120`. By induction over ticks:

1. After `reset`, no frame is in transmission and none is held.
2. In a tick with neither, `C_N ≥ L_max` makes it a whole-frame tick (section 2). Today's rule runs: frames are sent whole or not at all, so no frame is in transmission at the end of the tick, none is held, and the next tick is again a whole-frame tick.
3. So every tick at 100 ms runs exactly today's code path:
   - the same frames, in the same order, with the same sequence numbers (committed = sent);
   - the same suppression counts;
   - `sent_bytes` = the sum of the frame lengths;
   - the same release values;
   - no aborts, because nothing is ever in transmission at a boundary or at a reboot.

ACK 14 + telemetry 36 + DATA 78 = 128 > 120 still means the three never go out in one tick: today DATA waits a tick, and it still does, because a whole-frame tick does not span. So the downlink bytes, every `SpacecraftState`, the controls, the golden captures and #121's digests are unchanged, and the worst-case tick groups of #120 (DATA, ACK + DATA, telemetry + DATA, ACK + telemetry) are unchanged.

**Edges where 100 ms behaviour could have changed, and how the design avoids them:**

- **Leftover bytes after whole frames** (for example the 70 bytes after ACK + telemetry): a streamer would start DATA in them. Whole-frame ticks never span.
- **Overflow** (seven or more ACKs plus telemetry, or nine ACKs): today the excess is suppressed. Holding happens only in streaming ticks, so it is still suppressed.
- **`transmitter_off`**: capacity 0 makes a streaming tick, but with nothing in transmission or held it suppresses exactly what today's rule suppresses, and none is sent later.
- **The RESET ACK** is sent and numbered before the reboot, as today. It is framed when committed, in step c. In a whole-frame tick it either completes in the RESET tick or is suppressed, so the reboot is always in step d of the RESET tick (section 6a, cases 1 and 2). A reboot is never pending at 100 ms, so commands are never ignored, flags never pause, and BOOT's timing is unchanged.
- **The completion-release corner** (section 6) needs a carried-in DATA frame, which never exists at 100 ms.
- **Configurations at 100 ms with `C_N < L_max`**, for example a chunk size above 102 bytes (DATA frame over 120) or a lower rate, are streaming ticks. No default scenario, golden capture or digest uses them.

The unit tests that drive the outbound queue with **hand-set capacities below 78 bytes** are not the 100 ms default and their expectations change, because those capacities are now streaming ticks. They include #51's capacity boundaries (13, 14, 27, 28), #55's 35/36/63/64-byte cases, and #56's 0/77-byte cases. The implementation moves each to a capacity of at least 78 where it tests today's rule, and adds streaming counterparts.

### 8. Contract and interface changes

**Changed:**

- **`FlightComputerOutput`** (`pocketsat.flight.computer`):
  - `sent_bytes` becomes a validated `int` field (non-negative): the bytes transmitted this tick. It was a property summing `downlink_frames`.
  - New `outbound_aborted_count: int = 0`: frames of any class aborted mid-transmission or dropped while held, this tick.
  - New `transmitting: OutboundProgress | None`: the frame in transmission at the end of the tick, if any. `OutboundProgress(frame_type, sequence, size_bytes, sent_bytes)` is a frozen record for tests and run records.
  - New `reboot_pending: bool = False` and `uplink_ignored_count: int = 0` (section 6a).
  - The docstring of `downlink_frames` changes: frames **delivered** this tick, which can total more than the tick's capacity.
- **`FlightComputer`:**
  - New transient state: the frame in transmission (the encoded frame, bytes sent so far, its class, its chunk ID for DATA), the held ACK FIFO and the held telemetry payload, the abort count pending from a reboot between steps, and the pending reboot with its kept events (section 6a). All are cleared by `reset()` and `reboot()`, with the rest of the transient state, so a `forced_reset` cancels a pending reboot.
  - `_update_mode` defers the `RESET` and the events after it while the RESET ACK is outstanding in a streaming tick. `step()` runs the shutting-down path while a reboot is pending: the transmitter only, then step f. A new read-only property `reboot_pending` is added.
  - `TickContext.reserve_outbound` takes the section 2 rule. A new `_continue_transmission(tick)` runs at the start of step c.
  - The session update moves to completion.
  - `_produce_controls` gets the completion-release corner of section 6.
- **`FlightComputerConfig`** gains `outbound: OutboundConfig = OutboundConfig()`, with `max_held_ack_frames: int = 8`. It is validated as an int ≥ 0; 0 means never hold ACKs.
- **`SilTarget`:**
  - `TickTraffic.sent_bytes` takes `output.sent_bytes` (already the case, now a field).
  - `SilTick` gains `outbound_aborted_count`, `transmitting` and `reboot_pending`, copied from the output. `uplink_dropped_in_reset_count` also counts `output.uplink_ignored_count`.
  - The merge, the hold, the uplink rules and `receive()` are unchanged.
- **Docstrings:** `RadioTraffic.sent_bytes` ("bytes transmitted in the tick, including parts of frames delivered in other ticks") and `outbound_suppressed_count` ("ACK/NACK and telemetry frames that were due but never delivered: no room, overflow, or aborted").

**Not changed:** the `Subsystem` protocol, `SubsystemStack`, `STEP_ORDER`, `SnapshotBoard`, `SpacecraftControls` and `RadioTraffic` (fields), comms (code, state, validation, draw), `CommsTruth`/`CommsReadings`, the safety rules (#48's comms check still bounds `previous_tick_sent_bytes` by the previous capacity), `TestTarget`, the frame codec and wire format, the DATA and telemetry layouts, and the telemetry scheduler.

### 9. HIL and Phase 7 firmware (ADR-0002)

On the MCU, the flight software hands whole frames to the radio driver, and a UART (or the radio's FIFO) shifts them out at the link rate, over as many control cycles as that takes. Option A is the model of exactly that:

- The outbound queue and "frame in transmission" map to a TX ring buffer drained by the UART interrupt. The firmware does not need a per-tick byte capacity: the hardware sets the pace, and the queue commits a frame when the driver reports the previous one sent.
- "Delivered when the last byte is transmitted" is what the HIL bridge sees: it can return a frame from `receive()` only once all its bytes have arrived on the serial port.
- Release on completion maps to the driver's TX-complete callback.
- Aborting on transmitter disable or reset is what the hardware does.
- The two-kind rule is a property of the SIL model's tick quantization (section 2), not something the firmware needs. A firmware that streams always is the "always streaming" limit, and contract tests must not assert which bytes go out in which tick (ADR-0007 §7). SIL tests assert it.

Nothing crosses `TestTarget`, which is unchanged.

### 10. Determinism and performance

- **Determinism:** all new state is integers and byte strings, updated in a fixed order inside `step()`. There is no randomness and no wall-clock time (ADR-0003, ADR-0006). Same seed, same frames.
- **Performance:** at 100 ms the added work per tick is a few comparisons: is anything in transmission or held, and is `C_N ≥ L_max`. The frames, encodings and session updates are the same. The orbit average and the worst-case tick (#78, #120) should not move measurably. `WORST_CASE_SCENARIOS` gains no scenario, because no busier tick exists at 100 ms. Short-tick runs cost more per simulated second (3.3 times as many ticks at 30 ms), as any shorter tick does; the budgets are defined at 100 ms. The implementation PR re-measures both `[perf #78]` figures.

### 11. Options for where spanning lives

| Criterion | **A. Flight computer outbound queue** (chosen) | B. Comms models a transmit buffer | C. `SilTarget` serializes |
|---|---|---|---|
| "Comms reads nothing" | Kept: comms is unchanged and is told bytes per tick | Broken in spirit: comms must be given frames (or sizes) through `RadioTraffic` and hold them, but "comms holds no data" (ADR-0004 §13) | Kept for comms, but `SilTarget` would re-arbitrate and trim outbound traffic, which ADR-0007 §4 rules out |
| Timing | Bytes go on air in the tick the flight computer decides them, as today; ACK same tick in whole-frame ticks | Comms sees tick N's frames only in N+1 (ADR-0007), so serialization and delivery would lag a tick: ACK same tick breaks, and 100 ms changes | Same-tick serialization is possible, but the flight computer only learns of completion in a later step |
| Release after full transmission | The flight computer sees completion directly | The flight computer would read a "frame completed" flag from comms' readings one tick late, so the release timing changes at 100 ms | Needs a new feedback input from `SilTarget` into `FlightComputer.step()` |
| Single controls function | Unchanged | Unchanged, but `RadioTraffic` grows frame contents | Unchanged |
| `TestTarget` | Unchanged | Unchanged | Unchanged |
| HIL / firmware | Maps to the firmware's TX queue and driver (section 9) | The radio model would hold flight software state that, on the MCU, is in the firmware | The SIL flight software would not model the firmware: the MCU must implement its own queue, so SIL and HIL diverge |
| Determinism | Yes | Yes | Yes |
| Performance | A few comparisons per tick | An extra record per tick, and comms' snapshot cache churns per frame | Extra bookkeeping in `SilTarget` per tick |
| No change at 100 ms | Yes (section 7) | No (a tick of lag) | Possible, at the cost of a new interface |

## Alternatives considered

- **B. Comms models a transmit buffer and serializer.** Comms would have to hold frames (ADR-0004 §13: comms holds no data) and receive them through `RadioTraffic`, which reaches it a tick late (ADR-0007 §2). Delivery and release would move by a tick, breaking "command → ACK, same tick" and the 100 ms bit-identity. The flight computer would need a completion signal from comms' readings. Rejected.
- **C. `SilTarget` serializes.** The flight computer would hand over frames regardless of the capacity, and `SilTarget` would stream them and deliver them when complete. That makes `SilTarget` re-arbitrate outbound traffic (ADR-0007 §4), needs a new completion input to `FlightComputer.step()` for the release rule, and leaves the real flight software without the behaviour, so the Phase 7 firmware would have to implement it separately and SIL would stop modelling HIL. Rejected.
- **Always stream (one rule for every tick).** The simplest and most physical model: a work-conserving serializer at every tick length. It changes the 100 ms downlink (for example ACK + telemetry + DATA in one tick: today DATA waits, a streamer sends its first 70 bytes), and with it the golden captures, the digests and the pass timing. #132 forbids that. Kept as open question 1, for a later issue if wanted.
- **Span only frames larger than the whole tick** ("oversized" frames), with today's rule otherwise. Also unchanged at 100 ms, but at short ticks it inverts priority. At 30 ms (36 bytes), an ACK (14) then telemetry (36) leaves telemetry suppressed (it fits a tick, but not what is left), while the DATA frame behind it may span. Telemetry would be lost whenever a command arrives in its tick. Rejected.
- **Span only from the start of a tick** (a frame that does not fit may start only if nothing else was sent in the tick). Unchanged at 100 ms, but the tail of every spanning frame's last tick goes unused: 867 bytes/s instead of 1200 at 30 ms, so the long-run rate is not the configured rate. Rejected.
- **Suppress ACK/NACK and telemetry while a frame is in transmission** (no holding). At short ticks a pass keeps the transmitter busy almost every tick, so nearly every telemetry frame and most ACKs would be suppressed, and telemetry would lose its period. Rejected.
- **Preempt a DATA frame for an ACK** (abort and resend it later, or interleave). Interleaving is forbidden by #132. Aborting wastes airtime and makes a pass's progress depend on the uplink. The ACK wait is already bounded by about 76 ms. Rejected.
- **Resume or resend a partly transmitted frame after `transmitter_off`.** Resuming mid-frame is not what a radio does: the receiver loses sync. Resending an ACK or telemetry frame contradicts #60's "none is sent later". DATA is resent anyway, as a new frame, because its chunk was never released. Rejected.
- **Finish an in-progress DATA frame after its session ends.** It needs a release path outside the session (the chunk would be delivered while the session that would release it is gone). A new session started before that release could also send the chunk twice. It also delays the SAFE/FAULT entry telemetry frame by up to 77 bytes. Rejected in favour of aborting it (open question 6).
- **Abort the RESET ACK at the reboot in step d of the RESET tick** (the original recommendation). This keeps the reboot timing unchanged at every tick length, but at short ticks the ground may never learn the reset was accepted. The maintainer chose to wait for the ACK (section 6a). Rejected.
- **Wait for the RESET ACK in first-in, first-out order** (no overtaking). The wait would grow with the ACKs held ahead of it, up to 8 × 14 bytes more, and the flight computer would ignore uplink and pause its flag rules for longer. Overtaking keeps the bound at one frame plus the ACK. Rejected.
- **Wait for the RESET ACK even with the transmitter off.** The reboot could be delayed for ever by a fault. Rejected: it reboots at once and counts the ACK as suppressed.
- **Assign sequence numbers when a frame is created (queued) rather than committed.** Held frames that are later dropped would leave gaps that do not mean "lost on the way down". Rejected.

## Consequences

- Any tick length works:
  - A DOWNLINK pass completes at 30 ms and 50 ms.
  - Telemetry keeps its period.
  - The long-run byte rate equals the configured rate in streaming ticks.
  - Energy follows the bytes.
- 100 ms is bit-identical (section 7).
- ADR-0007's traffic path, its timing and comms are unchanged. Only the source of `RadioTraffic.sent_bytes` changes.
- **Cost: the pass rate depends on which kind of tick runs.** At 100 ms a pass moves one 78-byte DATA frame per 120-byte tick: 780 bytes/s of DATA frames, 640 bytes/s of chunk data. In streaming ticks a pass moves about 1164 bytes/s of DATA frames, which is 1200 minus telemetry at 1 Hz. A pass at 30 ms therefore finishes sooner than at 100 ms. This is the whole-frame rule's unused tail at 100 ms, kept to stay bit-identical, not something new. It is open question 1.
- **Cost:** ACK latency at short ticks is one frame's airtime at worst (about 76 ms), not "same tick". The latency table says so.
- **Cost (section 6a):** at short ticks a RESET command can reboot up to about 76 ms of airtime after its tick (N+2 at 30 ms). The ground always learns the reset was accepted, unless the transmitter is off or a `forced_reset` intervenes. In those few ticks the flight computer ignores uplink and pauses its flag rules. ADR-0004 §9's timing changes at short ticks only.
- **Cost:** a little more transient flight computer state (one frame in transmission, at most 8 held ACKs and one held telemetry frame), and three new output fields.
- Some unit tests at hand-set capacities below 78 bytes change their expectations, because those capacities are now streaming ticks (section 7).

## Follow-ups

On acceptance, the implementation for #132 (one PR):

- **Code:**
  - `flight/computer.py`: the transmitter, holding, abort and counts, `FlightComputerOutput`, `OutboundConfig`.
  - `flight/downlink.py`: the session advances on completion, and the completion-release corner.
  - `targets/sil.py`: the `SilTick` fields.
  - Docstrings in `spacecraft/controls.py` (`RadioTraffic`) and `spacecraft/comms.py` (module docstring: "bytes transmitted in the tick").
  - No change to comms' logic.
- **Tests:**
  - **Unchanged and passing:** `uv run python tests/sil/_golden.py --check` (identical), #121's `test_behavior_digests.py` (unchanged file), and an uncommitted A/B hash of every `SilTick` at 100 ms against `main`, as #131 did.
  - **At 30 ms and 50 ms** (`C = 36` and `60`), plus an uneven case (for example 1234 bytes/s at 30 ms, 37/38 bytes), through `SilTarget` with the real flight computer:
    - a DOWNLINK pass completes;
    - every chunk arrives exactly once and in order (`_downlink_ground.Ground`, with release only after receipt);
    - telemetry arrives at its period, with no suppression while the transmitter is on;
    - over a long run, the total of `sent_bytes` = `floor(rate × transmitter-on time)` minus at most one frame of idle tail;
    - comms' charged per-byte energy = 2 mJ × the total of `sent_bytes`, at N+1 in comms and N+2 in power;
    - every tick, `sent_bytes ≤ C_N`;
    - produced = buffered + released, and delivered − released = the chunks delivered in the tick (at most 1).
  - **Unit tests on the outbound queue:**
    - the boundary between the two kinds of tick (`C = 77, 78, 79`);
    - continuation before ACKs;
    - no interleaving;
    - priority at commit;
    - ACK latency equal to the section 5 formula;
    - the held-ACK bound (8, then suppression);
    - telemetry replaced while held;
    - sequence numbers in transmit order with no gaps;
    - `FlightComputerOutput.sent_bytes` versus the delivered lengths.
  - **Interruptions:**
    - `transmitter_off` mid-DATA, mid-ACK and mid-telemetry (aborted, counted, nothing sent later, the chunk resent);
    - **RESET (section 6a):**
      - At 100 ms: a RESET in every phase of the telemetry period, with an ACK before it in the same tick, and with 9 commands so the RESET ACK is suppressed. The reboot is always in step d of the RESET tick, and the frames, sequences, boot count, uptime and controls equal `main`'s (A/B hash).
      - At 30 ms with a DATA frame in transmission:
        - the reboot is deferred to the tick M in which the RESET ACK's last byte goes out, and M matches the bound formula;
        - the RESET ACK is delivered with the pre-reboot sequence number, and the next frame is numbered 0;
        - BOOT's controls apply from M+1, and uptime counts from the end of M;
        - the DATA frame ahead of the ACK completes, is released, and does not go down twice.
      - The RESET ACK overtakes waiting ACKs (held and from earlier in the tick); those keep their order behind it, and are delivered if they complete in M, otherwise counted as suppressed.
      - While pending:
        - uplink is ignored, with no reply, and counted in `uplink_ignored_count` and in `SilTick.uplink_dropped_in_reset_count`;
        - no telemetry and no new DATA are sent;
        - no flag rule runs: a forced `critical_battery` does not enter SAFE before the reboot;
        - the mode is unchanged and the controls are the previous mode's.
      - Two RESETs in tick N: the events are kept and applied at M exactly as today's `update_mode` would apply them at a reboot.
      - `transmitter_off` in tick N: immediate reboot, the ACK suppressed (as today). `transmitter_off` injected in a pending tick: the reboot happens in that tick, and the ACK is aborted and counted as suppressed.
      - A `forced_reset` pulse in a pending tick, and a hold that begins in one: the pending reboot is cancelled, the ACK is aborted and counted, and exactly one reboot happens (boot count +1).
      - At 50 ms the reboot is at most N+1, and at 30 ms at most N+2, over many random phases (seeded).
    - a `forced_reset` pulse and hold mid-frame (no bytes during the hold, aborted at the reboot);
    - ENTER_SAFE_MODE, SET_MODE, a SAFE flag and FAULT mid-DATA (aborted, the chunk resent first by the next session);
    - a transmit inhibit mid-DATA (the frame finishes);
    - the last chunk completing in the tick that raises `DOWNLINK_COMPLETE` (released).
  - **Determinism:** the same seed gives identical `SilTick`s at 30 ms.
  - **Updated:** the hand-set-capacity unit tests listed in section 7.
- **Docs:**
  - `docs/protocol.md` ("Downlink sequence and transmit capacity", "Dispatch and ACK timing", DATA "Order, release, and loss");
  - `docs/spacecraft.md` (Communications: replace the "Frames are not yet split across ticks (see #132)" note);
  - `docs/spacecraft-modes.md` (Telemetry cadence; Downlink session: send, release, abort, rate; "RESET and forced_reset: the reboot": the pending reboot of section 6a);
  - `docs/architecture.md` §4.1 (the latency table and the `FlightComputerOutput` sentence);
  - the edits to ADR-0004 and ADR-0007 listed in the PR for this ADR.
- Re-measure the `[perf #78]` figures and record them in the PR.

## Open questions for the maintainer

The maintainer has decided all nine: questions 1 to 3 and 5 to 9 as recommended, and question 4 changed to "the reboot waits for the RESET ACK" (section 6a). Accepted with the implementation (#132).

1. **Two kinds of tick, or always stream?** Recommendation: two kinds now (section 2), because it is the only rule found that keeps 100 ms bit-identical and makes the long-run rate exact at short ticks. Always streaming would make the pass rate tick-independent (the cost noted under Consequences), but re-records the golden captures and digests and changes #72's pass timing. If you want it, it is a later, separate issue.
2. **The held-ACK bound.** Recommendation: 8 (one default tick's worth), configurable in `FlightComputerConfig.outbound`. Alternatives: unbounded (the uplink rate bounds it anyway), or 1.
3. **Telemetry held while another comes due.** Recommendation: the newer frame replaces the older, which is counted as suppressed (fresh readings win). With the default periods this never happens while the transmitter is on.
4. **The RESET ACK at short ticks.** **Decided by the maintainer: the reboot waits until the RESET ACK has been transmitted** (section 6a), bounded by one frame in transmission plus the ACK's airtime. It reboots at once when the ACK cannot go out (suppressed, or the transmitter off), and a `forced_reset` cancels it. (Originally recommended: abort the ACK at the reboot.)
5. **Counting.** Recommendation: count aborted or dropped ACK/NACK and telemetry in `outbound_suppressed_count` (its meaning widens to "never delivered"), and add `outbound_aborted_count` (every class) to `FlightComputerOutput` and `SilTick` only, not to `RadioTraffic` or comms. The alternative is a separate comms counter, which is a contract change.
6. **Leaving DOWNLINK mid-DATA.** Recommendation: abort the frame (section 6). The alternative is to finish it (section "Alternatives considered").
7. **A transmit inhibit mid-DATA.** Recommendation: the frame in transmission finishes, because the inhibit stops new DATA, as it stops new frames today. The alternative is to abort it.
8. **Sequence numbers at commit.** Recommendation: yes (section 3). It is identical to today at 100 ms.
9. **Performance.** Recommendation: no new `WORST_CASE_SCENARIOS` entry, because no busier tick exists at 100 ms; short-tick runs are not budgeted. The PR re-measures both figures.

## Implementation notes (#132)

Points the implementation had to settle where the text above leaves room. None changes behaviour at the default 100 ms tick.

- **"Transmitter off"** is a tick whose readings show capacity 0 **and** `transmitter_on` false. A transmitter that is on with a capacity of 0 (a tick shorter than one byte's airtime, below about 0.83 ms at 1200 bytes/s) is a streaming tick in which nothing moves: frames wait instead of being aborted.
- **When held ACK/NACK are committed.** The frame in transmission continues at the start of step c (`_continue_transmission`); held ACK/NACK are committed by the execute-commands phase, just before this tick's ACK/NACK. In a tick with a RESET command the RESET ACK is queued first, so it directly follows the frame that was in transmission at the start of the tick, ahead of the held ACK/NACK and of the tick's other ACK/NACK (which keep their order behind it). That is what makes section 6a's bound `C_N + … + C_M ≥ r + 14` exact.
- **The held-ACK bound and the RESET ACK.** The RESET ACK is held at the front even when the bound is reached, as long as the transmitter is idle with bytes left; otherwise, with the bound reached, it is suppressed like any newest ACK and the reboot happens at once (section 6a, case 2).
- **Counting.** `outbound_aborted_count` counts the frame in transmission when it is aborted and every held frame dropped by the transmitter going off or a reboot. A held telemetry frame replaced by a newer one is counted only in `outbound_suppressed_count`. Counts from a reboot between steps (`forced_reset`) are reported in the next step's output.
- **Telemetry replaced at a mode change.** A mode change makes a frame due at once (#55). If a periodic frame is still held behind the frame in transmission, the mode change's frame replaces it (rule of section 5) and it is counted as suppressed. With the default periods this is the only way a frame is lost while the transmitter is on, and only in streaming ticks.
- **After a deferred reboot**, step e runs in BOOT in the reboot tick, as it does after a RESET in a whole-frame tick (no telemetry is due in BOOT), so the telemetry schedule matches the whole-frame path.
- **`max_held_ack_frames`** keeps the name given here; it is on the naming convention's allowlist (`docs/architecture.md` §4.1).
