# Axiomatic Adapter — Plan & Resolutions

This document captures the design plan and the resolution status of every issue identified during the Axiomatic adapter audit. It mirrors the working plan used while implementing the fixes on branch `zeerek/bugfix-axiomatic-flips-error-logic`.

---

## Context

The Axiomatic CAN-to-Ethernet adapter ([axiomatic_adapter/](axiomatic_adapter/)) is the TCP-over-Ethernet alternative to a Kvaser SocketCAN device. ECU flashing through it failed intermittently, while flashing via Kvaser was reliable. Two audit passes pinned the asymmetry to the TCP transport layer: the adapter treated the byte stream as one-CAN-frame-per-read and only understood the deprecated "CAN Stream" message type. Under the bursty traffic of a flash protocol, this dropped frames three different ways.

The work was staged into three phases so the flash bench could validate intermediate progress.

---

## Audit findings — status table

| # | Severity | Finding | Resolved in |
|---|----------|---------|-------------|
| 1 | CRITICAL | `send()` wrote 8 data bytes regardless of DLC; declared `message_length` disagreed with actual payload | Phase 1 |
| 2 | CRITICAL | No TCP stream framing — coalesced or split CAN frames were silently dropped | Phase 1 (partial) → Phase 2 (complete) |
| 3 | CRITICAL | Heartbeat (Message ID 4, every 1 s) and Status Response (ID 3) were rejected as garbage, corrupting stream alignment | Phase 2 |
| 4 | CRITICAL | The protocol's Message Data Length at bytes 9-10 was ignored; framing was guessed from the control byte | Phase 2 |
| 5 | CRITICAL | A single protocol message can carry multiple CAN + Notification frames; only the first was delivered | Phase 2 |
| 6 | CRITICAL | Bridge error callbacks had inverted logic + UB dereference of an empty `std::optional` | Already fixed in tree prior to this branch (verified) |
| 7 | HIGH | No `TCP_NODELAY` — Nagle's algorithm added up to ~40 ms latency per small write | Phase 3 |
| 8 | HIGH | Synchronous `boost::asio::write()` from the bridge thread raced `async_receive` on the same socket | Phase 3 |

---

## Phase 1 — Quick correctness fixes (DONE)

Goal: ship the smallest changes that make the wire protocol behave so the flash bench could validate immediately.

- **`send()` DLC fix.** `send()` now writes exactly `frame_data_length` bytes after the header instead of all 8 of `frame.get_data()`. The declared `message_length` and the actual payload bytes finally match.
- **First-cut TCP framer.** Added `AxiomaticFrameParser` with a persistent `std::vector<uint8_t>` buffer. The parser scans for a sync header, decodes a complete frame, leaves trailing partial bytes for the next read, and skips garbage prefixes.
- **`receive()` rewired** to drain the parser before doing any I/O and to append fresh bytes after a successful read. Coalesced frames now deliver back-to-back without additional `async_receive` calls.
- **`INCOMPLETE_FRAME_SENTINEL`** suppresses the "bytes arrived but not yet a full frame" case from the reception loop's `error_callback_`.
- **Stale test NOTE removed** — the "first message may fail" comment was a symptom of the absent framer.

Tests: 11 parser unit tests + 4 loopback send-length tests, no hardware required.

### Phase 1 gap that motivated Phase 2

The Phase 1 parser hard-coded a **7-byte** "sync header" including `0x01` as the last byte. That `0x01` is actually the first byte of the Message ID — specifically Message ID 1, "deprecated CAN Stream." The protocol also defines Message IDs 3 (Status Response), 4 (Heartbeat), and 5 (CAN FD Stream), all of which share the first **6** bytes but differ at bytes 6-7. The Phase 1 parser:

- Rejected heartbeats as garbage, dropping their bytes and risking mid-message desync.
- Computed total frame size from the control byte (only meaningful for CAN Stream messages), with no way to skip other message types cleanly.
- Returned only one CAN frame per protocol message, while the spec allows multiple CAN + Notification frames packed together.

---

## Phase 2 — Protocol-correct streaming parser (DONE)

Goal: make the parser understand the actual Axiomatic message envelope, not just a sync prefix.

- **`SYNC_PREFIX` is now 6 bytes** (`AXIO` + `0xBA 0x36`). Message ID lives at bytes 6-7 and is treated as data, not header.
- **Framing uses the protocol's Message Data Length field** (bytes 9-10). Total message size is `11 + data_length`. The parser waits for that many bytes before consuming anything.
- **Message ID dispatch.** CAN Stream messages (ID 1) are decoded; Heartbeat (ID 4), Status Response (ID 3), CAN FD Stream (ID 5), and any unknown ID are consumed and counted, never surfaced as errors.
- **Multi-frame iteration.** `decodeCanStreamBody()` walks the CAN Stream payload one control byte at a time, handling CAN frames (control bit 7 = 0) and skipping Notification frames (bit 7 = 1, fixed 5 bytes). Decoded CAN frames flow through an internal `std::deque` so the existing one-frame-per-call API still works.
- **Diagnostic counters** added: `heartbeats_seen`, `status_responses_seen`, `can_fd_messages_skipped`, `unknown_messages_skipped`, `notification_frames_skipped`, `dropped_bytes`.
- **Verbose mode.** `AxiomaticFrameParser::set_verbose(true)` prints one `std::cout` line per non-CAN event. Plumbed all the way through: `ros2 run axiomatic_adapter axiomatic_socketcan_bridge ... -v` now flips parser verbosity too.
- **`send()` wire layout unchanged**, but the source now references `SYNC_PREFIX` + `MSG_ID_CAN_STREAM_DEPRECATED` explicitly so the intent is obvious.

Tests: parser tests refactored around a `encodeCanStreamMessage` / `encodeHeartbeatMessage` / etc. helper set, plus new cases for packed multi-frame messages, mixed CAN+Notification bodies, heartbeats coalesced with CAN traffic, status responses, FD frames, and split-across-append scenarios.

---

## Phase 3 — Concurrency & throughput (DONE)

Goal: eliminate the remaining intermittent failure classes — Nagle-induced latency and the unsynchronized cross-thread socket access.

- **TCP_NODELAY** is set immediately after a successful connect. Failure is logged but non-fatal.
- **Persistent `io_context`** runs on a dedicated worker thread for the adapter's lifetime, held alive by an `executor_work_guard`. The per-call `restart()`/`run()` pattern is gone.
- **`boost::asio::strand`** serializes every socket operation. `send()` and `receive()` post async work onto the strand and block the calling thread on a `std::promise`/`std::future` — public sync signatures unchanged, so the bridge keeps working as-is.
- **Reception loop is now self-rescheduling.** `startReceptionThread()` no longer spawns a thread; it posts the first `async_receive` to the strand, and the completion handler chains itself. `joinReceptionThread()` posts a `tcp_socket_.cancel()` and waits for the chain to drain.
- **`closeSocket()` posts the close onto the strand** so it can't race the io thread.
- **Parser access is mutex-guarded** between the io thread (writer) and any sync `receive()` caller (reader).
- **`DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS` lowered from 100 ms → 20 ms** now that the loop is async and the timeout only bounds the sync `receive()` public API.
- **Sync `receive()` refuses to run while the reception thread is active** — the two paths would otherwise interleave bytes between two parsers. Pick one mode per adapter instance.

Tests: new `test_axiomatic_concurrent_io` exercise; a localhost peer simultaneously consumes our `send()` traffic and emits inbound CAN Stream messages while one thread hammers `send()` and the reception chain delivers frames via callback. Ready to be re-run under `-fsanitize=thread` for the formal TSAN sign-off.

---

## Verification

### Automated (no hardware required)

`colcon build --merge-install --packages-select axiomatic_adapter --cmake-args -DBUILD_TESTING=ON`
`colcon test --merge-install --packages-select axiomatic_adapter`

| Test binary | Cases | Assertions |
|-------------|-------|------------|
| `test_axiomatic_frame_parser` | 19 | 203 |
| `test_axiomatic_send_wire_format` | 4 | 60 |
| `test_axiomatic_concurrent_io` | 2 | 10 |
| **Total** | **25** | **273** |

All green on a clean build.

### Still owed (manual, requires hardware)

- Re-run the existing hardware Catch2 suite at `192.168.0.34:4000` and confirm the previously flaky "first read" no longer fails.
- Capture a raw TCP stream from the converter for ~30 s while idle and feed those bytes through the parser; expect zero CAN frames, `heartbeats_seen() ≈ 30`, `dropped_bytes() == 0`.
- End-to-end ECU flash through the bridge — should succeed without retries.
- Latency probe: a single `send()` → `receive()` round trip should drop from the previous ~40 ms (first-frame Nagle) to sub-millisecond.
- Optional: rebuild with `-fsanitize=thread` and run `test_axiomatic_concurrent_io` for the TSAN sign-off on Phase 3.

---

## Public API impact

None for existing consumers. The `AxiomaticAdapter` constructor gained a defaulted `bool verbose = false` parameter, so old call sites compile unchanged. `AxiomaticSocketcanBridge` now forwards its `-v` flag into the adapter.
