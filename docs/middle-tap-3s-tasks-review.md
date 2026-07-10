# Middle-tap 3s tasks — implementation review

Reviewed against `docs/middle-tap-3s-tasks.md` and the current working-tree diff (10 files,
~767 lines). Reviewer did not run a local `pio` build in this session — treat compile
verification as still needed on your machine.

**Verdict:** The **2.4 GHz cache-hit path** looks well thought out and aligned with the spec.
The **5 GHz / modem path** has two plausible regressions that should be fixed or confirmed
on hardware before calling T7 done.

---

## Task-by-task status

| Task | Code status | Notes |
|------|-------------|-------|
| T1 | ✅ Looks correct | 2.4 GHz only, as spec'd |
| T2 | ✅ Looks correct | Stale-lease recovery wired |
| T3 | ⏭️ Correctly skipped | Server doesn't resume TLS |
| T4 | ⏳ Not implemented | Correct to defer |
| T5 | ⏭️ Subsumed by T6 | Fine |
| T6 | ✅ Looks correct | Verify 160 rows + ghosting on HW |
| T7 | ⚠️ Partial | CWAUTOCONN + CWJAP? skip good; UART_DEF + prewarm 5 GHz need fixes |
| T8 / T8b | ✅ Looks correct | |
| T9 | ✅ Implemented | HW quality check still required |
| T10 | ⏳ Server-side | N/A |

---

## What looks correct

### T1 — WiFi fast-connect (`lib/wificaptive/src/connect.cpp`)

- Cache save/load/invalidate in NVS keyed by SSID + BSSID + channel.
- `fastConnectAndWait()` uses `WIFI_FAST_SCAN`, direct `WiFi.begin(..., channel, bssid)`,
  3 s timeout, 10 ms poll.
- Enterprise / 5 GHz correctly skip the fast path.
- Cache refreshed on slow-path success in `initiateConnectionAndWaitForOutcome()`.

### T2 — Cached DHCP lease

- Lease stored alongside fast-connect cache; applied only for DHCP users.
- `g_fast_lease_applied` + `recoverFromStaleLease()` on first fetch failure is the right
  guardrail.
- Lease invalidated and DHCP restored on fast-connect failure.

### T6 — Indicator strip restore (`src/display.cpp`)

- `display_restore_indicator_strip()` decodes PNG into framebuffer, then
  `fullUpdate(CLEAR_FAST, false, &rect)` on bottom 160 rows only.
- Removed the fast-path `DisplayedImage::clear()` / filename hack — cache hits stay honest.
- Non-PNG falls back to full `display_show_image()`.

### T8 / T8b — Prewarm + tracing (`src/bl.cpp`, `connect.cpp`, `api-client/display.cpp`)

- Prewarm spawns at `bl_init` entry when wake stub sees CH1 touch — good overlap.
- `trace()` infra with RTC + NVS persistence and split WiFi checkpoints (`STA_CONNECTED`,
  `GOT_IP`, `fastconnect *`) is solid.

### T9 — Panel passes (`src/display.cpp`)

- Fast path uses `setPasses(2,2)` for `fullUpdate(CLEAR_FAST)` and restores `(3,3)` after.

### Other good fast-path optimizations

- Skip logo repaint, IQS323 blocking wait, BQ27427 init, NTP, log flush, OTA on middle tap.
- Chunked image download + decode-before-FS-write on fresh fetch.
- CH1 bleed fix in `check_channel_states()` so middle tap isn't misread as left/right.

---

## Issues to fix or verify on hardware

### 1. 5 GHz prewarm path likely broken (high severity)

`wifi_prewarm_task` falls back to `WifiCaptivePortal.autoConnect()`, which uses native
ESP32 WiFi — not the C5 modem:

```cpp
// src/bl.cpp — wifi_prewarm_task
if (middle_tap_fresh_fetch && fastConnectAndWait(creds) == WL_CONNECTED) {
  ok = true;
} else {
  ok = WifiCaptivePortal.autoConnect();
}
```

For saved 5 GHz creds, `fastConnectAndWait()` returns failed immediately (`is5GHz`), then
`autoConnect()` cannot join a 5 GHz AP. The sequential path correctly uses
`connectWithSavedCredentials()` → `g_modem->connectToNetwork()`.

**Suggested fix:** In prewarm, after fast-connect fails, call `connectWithSavedCredentials()`
(or branch on `creds.is5GHz` + `g_modem`).

---

### 2. `AT+UART_DEF` may break modem on the next wake (high severity, T7)

After the first successful session, `AT+UART_DEF=5000000` persists C5 UART at 5 Mbps across
power cycles. But every wake still does:

```cpp
// lib/trmnl_x/src/modem.cpp — Modem constructor
ModemSerial.begin(baudRate, ...);  // always 115200 from callers
sendCommand("AT");
if (waitForResponse("OK", 5000).isEmpty()) {
  return;  // modem never initializes
}
```

`modem_reset_target()` power-cycles the C5 each wake, so on wake #2 the C5 likely boots at
5 Mbps while the host still talks 115200 → AT sync fails → modem never initializes.

**Suggested fix:** Try 5 Mbps first (or track "baud persisted" in NVS and construct
`Modem(5000000)`), with 115200 bootstrap as fallback.

---

### 3. Wake stub always shows "hold" indicator (low severity, UX)

When CH1 is detected in the wake stub, `pending_indicator_filled` is hardcoded `true`, and
`process_iqs323_data()` is skipped on the fast path — so a tap may show the filled (hold)
indicator instead of outline:

```cpp
// src/bl.cpp — wake-stub prewarm spawn
pending_indicator_filled = true;
```

Minor UX issue; functionally OK because T6 restores the strip on cache hit.

---

### 4. T6 strip restore doesn't use T9's 2-pass setting (low severity)

Full fast-path refresh uses `setPasses(2,2)`; strip restore uses default 3 passes. Probably
fine for 160 rows, but inconsistent if tuning panel time.

---

### 5. Middle tap missed by wake stub gets slow path (edge case)

If the wake stub doesn't see CH1 but `check_channel_states()` later sets
`middle_tap_fresh_fetch`, prewarm won't run and you get sequential
`connectWithSavedCredentials()` without T1 fast-connect. Uncommon if the stub is reliable.

---

### 6. Unrelated / dev-environment churn

- `platformio.ini`: `monitor_port = /dev/cu.usbmodem*` is macOS-specific; may annoy
  Linux/Windows devs.
- `dependencies.lock` / `sdkconfig` version bumps look like local PlatformIO regen, not task
  logic.

---

## Guardrails check

- T1/T2 caches invalidated on failure — ✅ present.
- Timer wakes keep conservative behavior — ✅ fast-path changes gated on
  `middle_tap_fresh_fetch`.
- `middle_tap_fresh_fetch` set in wake stub and `check_channel_states()` — ✅ documented
  pattern preserved; wake stub is the prewarm trigger.

---

## Recommended next steps

1. **Phase-0 trace on real hardware** (as the task doc says): tap → sleep → tap; capture
   `PREV CYCLE TRACE` for 2.4 GHz cache hit and 5 GHz.
2. **Fix prewarm 5 GHz connect** before trusting 5 GHz numbers.
3. **Verify modem baud persistence** — second consecutive middle-tap wake on 5 GHz is the
   critical test for T7.
4. Side-by-side photos for T9 `(2,2)` vs `(3,3)` and strip-edge ghosting for T6.

---

# Second review — Claude (Fable), 2026-07-09

Independent pass over the same diff. Unlike the review above, this one **did run
`pio run -e TRMNL_X_dev` — build passes.** Cross-references to the Cursor issues
above where they overlap. Overall verdict matches: 2.4 GHz path is sound, **T7
must not ship as-is.**

## Confirmations of Cursor findings

- **Cursor #1 (5 GHz prewarm fallback) — CONFIRMED, high severity.** Verified in
  `src/wifi_network.cpp:37`: `connectWithSavedCredentials()` routes 5 GHz through
  `g_modem->connectToNetwork()`, but `wifi_prewarm_task` falls back to
  `WifiCaptivePortal.autoConnect()` (native S3 radio), which cannot join a 5 GHz
  AP. Fix as suggested: branch on `creds.is5GHz && g_modem` in the prewarm task.
- **Cursor #2 (`AT+UART_DEF`) — CONFIRMED, highest severity; it is a one-way
  door.** Both call sites construct `Modem(115200)` and the constructor
  (`modem.cpp:23-36`) only tries the baud it was given. Once `UART_DEF` persists
  5 Mbaud in the C5's own NVS, *every* later wake fails AT sync at 115200 →
  `_initialized=false` → `g_modem=nullptr` → a 5 GHz-only user permanently loses
  connectivity. `modem_reset_target()` does not clear ESP-AT NVS, so recovery
  requires re-flashing the C5 or a fixed constructor. Fix: try 5 Mbaud first,
  fall back to 115200 (do NOT rely on an S3-side "baud persisted" flag alone —
  it desyncs if the C5 is swapped/re-flashed).

## New issues (not in the review above)

### F1. CWJAP?-skip waits for a URC that has usually already passed (high, T7)

`connectToNetwork` (`modem.cpp:~474`) queries `AT+CWJAP?` and, when associated,
does `waitForResponse("WIFI GOT IP", 15000)`. `waitForResponse` (`modem.cpp:376`)
only accumulates bytes arriving **after** it is called, and the constructor
flushes the RX buffer several times. In the exact case this optimizes —
CWAUTOCONN finished join+DHCP during C5 boot — the `WIFI GOT IP` URC is long
gone, so this stalls the full 15 s and then falls through to the explicit
`CWJAP` anyway: the "optimization" adds 15 s. Currently masked by Cursor #2
(init fails before reaching this), so fixing #2 alone will *expose* this one.
**Fix:** poll `AT+CIPSTA?` for a non-zero IP instead of waiting for the URC.

### F2. Debug trace infra is wired into production paths (medium, must strip before merge)

- `goToSleep` writes ~1.5 KB of trace to NVS on **every** GPIO wake
  (`dbg_trace` etc.) — flash wear plus tens of ms latency on the very path
  being optimized.
- `trace()` is called concurrently from the core-0 prewarm task and the main
  thread; `g_trace_count++` is unsynchronized, so entries can be dropped or
  garbled (debug-only impact, but worth knowing when reading dumps).
- Unconditional `Serial.printf` trace dumps on every boot.

Fine for the measurement phase; gate behind a debug flag (or remove) before merge.

### F3. CH1-bleed heuristic can hijack the corner-hold WiFi-reset gesture (medium, needs HW test)

`check_channel_states()` now processes only CH1 when CH1 reads touched, and the
fast path additionally skips `process_iqs323_data()` entirely when the wake
stub shows the CH1 bit. If CH1 bleeds during a corner-hold (CH0+CH2), the reset
gesture is misread as a middle tap and the user cannot reset WiFi. The inline
comment acknowledges the bleed trade-off, but the WiFi-reset gesture
specifically should be tested on hardware with this firmware.

### F4. Stale-lease worst case is ~15 s (low)

`https->setConnectTimeout(15000)` means a blackholed cached lease burns up to
15 s before `recoverFromStaleLease()` fires on attempt 1. Recovery itself is
correct and reachable (a failed GET returns a negative code →
`HTTPS_RESPONSE_CODE_INVALID`, which stays in the retry loop). Optional: use a
shorter connect timeout on the first fast-path attempt.

### F5. Chunked download reader can exit with a partial buffer (low)

The new 4 KiB-chunk loop has `if (got <= 0) break;` — that exit path bypasses
the timeout check, so the code can proceed to decode a truncated buffer. The
old byte-by-byte loop could only exit complete or timed-out. Practically rare
(`readBytes` with `available() > 0` returning ≤0), and the PNG decoder rejects
truncated data, but an `iCount < counter` check after the loop would close it.

### F6. `DEV_FIRMWARE` OTA-ignore (informational)

`handleApiDisplayResponse` now drops server-pushed OTA on dev builds. Sensible,
but remember it exists when testing OTA flows on a dev flash.

## Verified fine — do NOT "fix" these

Listed so the fix pass doesn't churn on non-issues:

- **Build:** `pio run -e TRMNL_X_dev` succeeds on this working tree.
- **`setClock()` skip on fast path is safe for TLS:** `withHttp` uses
  `setInsecure()` (`lib/trmnl/include/http_client.h:38`) — no cert-time
  validation, so stale/unset time cannot break the fetch.
- **`BB_RECT` initializer order** in `display_restore_indicator_strip` is
  correct (`{x, y, w, h}` per FastEPD.h).
- **`g_prev_used_fast_path` is reliably set on tap wakes** — the logo block runs
  on all non-timer wakes, so the T6 strip-restore path is reachable.
- **Non-TRMNL_X / X-class boards (e.g. reTerminal E1003) are safe:** the fast-path
  flags are only ever set under TRMNL_X-specific code, so `display_restore_indicator_strip`
  is unreachable there (and it compiles, since `bbep` is declared for all X-class).
- **T1/T2 cache hygiene:** SSID-keyed, invalidated on first failure, DHCP
  re-enabled (`WiFi.config(0,0,0)`) before the `autoConnect` fallback;
  static-IP users correctly skip the lease.
- Cursor #4 (strip restore uses 3 passes, not T9's 2) is real but cosmetic —
  a 160-row update is already ~0.3 s; only touch it if HW traces say otherwise.

## Fix priority for the follow-up pass

1. Cursor #2 / UART_DEF brick (one-way door — highest).
2. Cursor #1 / prewarm 5 GHz fallback.
3. F1 / CWJAP?-skip URC race (will surface once #2 is fixed).
4. F2 / gate the trace infra before merge.
5. F3 / verify corner-hold on hardware; F4/F5 optional hardening.

---

# Revalidation — Claude (Fable), 2026-07-09 (after Opus fix pass)

All three high-severity issues are correctly fixed; `pio run -e TRMNL_X_dev`
still passes.

- **Cursor #2 / UART_DEF brick — FIXED, verified.** Constructor now retries at
  5 Mbaud (with RTS/CTS + 8 KB RX buffer, matching the upgrade path) when the
  115200 AT sync fails, re-asserts `CWMODE=1` + `CWAUTOCONN=1`, and sets
  `_initialized`. The one-way door is closed: a persisted-baud C5 recovers on
  every wake.
- **Cursor #1 / prewarm 5 GHz — FIXED, verified.** `wifi_prewarm_task` now
  branches on `creds.is5GHz` → `connectWithSavedCredentials()` →
  `g_modem->connectToNetwork()`. `g_modem` is set earlier in the same task, so
  ordering is correct; if modem init failed it degrades to the pre-existing
  2.4 GHz fallback.
- **F1 / CWJAP?-skip URC race — FIXED, verified.** Now polls `AT+CIPSTA?`
  (200 ms interval, 8 s deadline) for a non-`0.0.0.0` IP instead of waiting on
  the `WIFI GOT IP` URC; falls through to explicit `CWJAP` on timeout. Match
  string is specific enough not to false-positive on the gateway/netmask lines.

Nit (non-blocking): the initial 115200 AT wait was shortened 5 s → 2 s. Since
`AT` is sent once and a C5 that wasn't ready never saw it, the long wait was
dead time anyway — but a send-retry loop (send `AT` every ~500 ms until
deadline) would be more robust against a slow-booting C5 than either version.

**Deliberately not fixed (still open, all low/medium):**
- F2 — trace infra still in production paths (NVS write every GPIO wake,
  unsynchronized `g_trace_count`). Fine while measuring; gate before merge.
- Cursor #3 — wake-stub indicator still hardcodes filled=true.
- F3 — corner-hold WiFi-reset vs CH1 bleed: still needs the hardware test.
- F4 — 15 s connect timeout on first fast-path attempt (stale-lease worst case).
- F5 — chunked reader can still exit on `got <= 0` with a partial buffer.

**Remaining gate for T7/T1/T2/T6/T9:** the Phase-0 hardware traces the task doc
calls for. The critical T7 test is two consecutive middle-tap wakes on 5 GHz
across a power cycle (exercises the persisted-baud recovery path end to end).
