# Middle-tap → fresh image in 3–4 s: task list

Target: TRMNL X middle tap, currently 8–9 s (cache hit), 20 s+ (network download).
Goal: ≤3.5 s cache hit, ≤6 s fresh 2.4 GHz download. Ordered by impact.

## Status — Round 1 (updated 2026-07-10; per-task detail for splitting into PRs)

Implemented on this POC branch and review-fixed (see
`middle-tap-3s-tasks-review.md`); builds clean on `TRMNL_X_dev`. Real tap
trace captured 2026-07-10. User-visible result so far: taps 8–9 s → 6.5–7 s.

- ✅ **T8b** — WiFi trace split (STA_CONNECTED / GOT_IP checkpoints).
- ✅ **T1** — WiFi fast-connect (cached channel+BSSID, FAST_SCAN, 10 ms poll,
  3 s fallback to autoConnect). **HW-CONFIRMED by tap trace: 121 ms.**
- ✅ **T2** — Cached DHCP lease + in-wake stale-lease recovery.
  **HW-CONFIRMED by tap trace (lease applied, connect 121 ms total).**
- ✅ **T6** — Cache hit restores only the ~160-row indicator strip via
  `fullUpdate(CLEAR_FAST, &rect)`; removed the DisplayedImage force-clear
  hack. **Still needs HW check:** strip coverage + ghosting on a cache-hit tap.
- ✅ **T3 (TLS resumption)** — IMPLEMENTED 2026-07-10 (firmware). Earlier SKIP
  was wrong (tested `usetrmnl.com` + TLS 1.2 `-reconnect`). `trmnl.app` (Caddy)
  issues & resumes tickets over **TLS 1.2** — confirmed via `-sess_out`/
  `-sess_in` → "Reused, TLSv1.2". New `fetchApiDisplayResumable`
  (`src/api-client/display_tls.cpp`) does the tap-path `/api/display` GET over
  raw `esp_tls`, saving the mbedtls session to RTC RAM (2 KB) for a 1-RTT
  resumed handshake next tap; any error falls back to `fetchApiDisplay`.
  Builds clean on `TRMNL_X_dev`. **Round 3 fixes (Fable, 2026-07-10)** after a
  tap showed the full (unresumed) esp_tls connect taking **5.0 s** vs 1.65 s on
  HTTPClient (server verified fine from a laptop: full handshake+close in 1.0 s):
  - Pre-resolve DNS via `WiFi.hostByName` (the proven ~30 ms path) and connect
    esp_tls by IP, with `cfg.common_name = host` so SNI is still sent (Caddy
    routes by SNI). Removes esp_tls's own `getaddrinfo` from the equation.
  - New checkpoints `resumable dns`, `tls connected`, `resumable first byte`
    split the connect: DNS / TCP+TLS / TTFB / read-to-close are now separable.
  - Ticket-keep policy: Caddy only sends a NewSessionTicket on FULL handshakes
    (verified via openssl), and TLS 1.2 tickets are multi-use — so on a resumed
    connect we keep the existing session blob instead of clobbering it with a
    ticket-less one (would have made every other tap a full handshake). Ticket
    expiry self-heals: server full-handshakes and the fresh ticket gets saved.
  **HW-CONFIRMED 2026-07-10:** tap trace shows `resumed=1` (session restored
  from NVS across a power cycle), API leg **1.17 s total**: DNS 15 ms, TCP+
  resumed-TLS 555 ms (was ~1.26 s full), server TTFB 595 ms, read 2 ms. Was
  5.0 s on the buggy esp_tls attempt, 1.65 s on HTTPClient. Note: a resumed
  connect logs `session saved` (not the keep-branch) because mbedtls retains
  the offered ticket in the session — same blob re-saved, harmless. Remaining
  cost in this leg is the 595 ms server TTFB (backend, not firmware). Helps
  `/api/display` leg only (S3 image still needs T10).
- ⏭️ **T5** — subsumed by T6.
- ✅ **T4** (prewarm owns whole fetch) — IMPLEMENTED 2026-07-10 (Fable), scoped
  to the `/api/display` leg only. Un-deferred after the post-T15 tap trace
  showed the connection ready at 282 ms but the fetch not starting until
  598 ms (main thread busy with indicator/IQS work) — ~0.3 s of dead air.
  Design (all in `src/bl.cpp`):
  - `wifi_prewarm_task` now runs `fetchApiDisplayResumable` right after
    connect-ok (WiFi path only; 5 GHz modem GET keeps its own branch in
    `downloadAndShow`). Result handed off via `s_prewarm_fetch_*` statics;
    `_started` is published before `_done` so the main thread can never race
    it with a duplicate fetch. Task stack 8192 → 16384 (esp_tls needs ~10 KB).
  - Task waits on `s_prewarm_inputs_ready`, set by the main thread after
    `display_init` (prefs begun, `wakeup_reason` set, display dims valid).
  - `loadApiDisplayInputs(prefs, skipPowerIO=true)`: the prewarm-built inputs
    report battery/USB/charging as -1/UNKNOWN — the live reads go through the
    TCA9535 on the shared I2C bus the main thread is driving for the indicator
    update (race), and the fast path already skips BQ27427 telemetry anyway.
    Next timer wake repopulates. Trace: `prewarm: fetch begin/end`,
    `join prewarm fetch` / `prewarm fetch joined` on the main thread.
  - `downloadAndShow` joins on the result (20 s ceiling > the fetch's own 15 s
    esp_tls timeout); a failed/hung prewarm fetch falls through to the proven
    HTTPClient retry loop (stale-lease recovery intact). Non-tap wakes and the
    image download are untouched.
  Builds clean on `TRMNL_X_dev` AND `trmnl` (OG). Side fix that rode along:
  `g_prev_wake_status1 = wakeup_stub_iqs_status…` was unguarded in common code
  and broke OG builds — now `#ifdef BOARD_TRMNL_X`.
  **HW-CONFIRMED 2026-07-10:** tap trace shows `prewarm: fetch begin` at
  318 ms, API leg fully overlapped (done at 1.63 s while the main thread was
  still at indicator/IQS work until 0.62 s), join handoff 2 ms. Total tap
  **3.6 s** for a changed-image cache-serve (was 4.0 s pre-T4, 8–9 s at branch
  start). Note: the first post-T4 trace showed DNS at 836 ms (vs 9–29 ms
  normally), suggesting first-packet-after-association loss on the mesh now
  that the fetch starts ~35 ms after assoc; the next tap showed DNS 29 ms, so
  it was network noise. If DNS ever sits at ~500–900 ms consistently, the
  ready fix is a gratuitous ARP after `fastconnect ok` (and/or an RTC-cached
  server IP with DNS fallback).
- ✅ **T7** — 5 GHz modem (UART_DEF persist + CWAUTOCONN + CWJAP? skip).
  Review found and fixed: 115200-only constructor brick, prewarm 5 GHz
  fallback, GOT-IP URC race. **Still needs HW test:** two consecutive taps on
  5 GHz across a power cycle.
- ✅ **T8** — prewarm spawn at bl_init entry.
- ⚠️ **T9** — implemented but **measured as a no-op** (1758 ms @ passes 2,2 vs
  1764 ms @ 3,3). Superseded by **T17** below.
- ⏳ **T10** — server-side (inline image / ETag / same-origin URLs).
  **Priority raised by tap trace:** 6.6 s of the 13.7 s fresh path is the
  second HTTPS round trip. Biggest remaining lever anywhere.

Everything is gated on the `middle_tap_fresh_fetch` tap path; timer wakes keep
today's full-scan / full-refresh behavior (guardrails intact).

## Round 2 — tasks for Opus (2026-07-10), in priority order; full specs in "Round 2 tasks" below

All five implemented (Opus + DeepSeek), code-reviewed by Fable 2026-07-10;
builds clean on `TRMNL_X_dev`.

1. ✅ **T15** — `Serial.setTxTimeoutMs(0)` after `Serial.begin()`,
   `wait_for_serial()` only on cold boot, both trace dumps moved after the
   `bl_init entry` checkpoint and skipped when `!Serial` (NVS copy left intact
   for the next tethered boot). **HW check FAILED on first capture
   (2026-07-10): `bl_init entry` still 2167 ms.** Root cause found: the fix
   lands too late — with `CORE_DEBUG_LEVEL=4` the Arduino core prints its
   ~130-line "Before Setup" chip report (`chip-debug-report.cpp`, unconditional
   at level ≥ DEBUG) over HWCDC *before `setup()` runs*, each stalled write
   blocking 100 ms in the plugged-but-port-closed state. Fixed by dropping
   `TRMNL_X_dev` to `CORE_DEBUG_LEVEL=3` (banner gated off, core err/warn/info
   kept; release envs don't set the flag and were never affected).
   **Re-verify:** tap with USB in the reconnect-loop state; target
   `bl_init entry` ≤ 0.3 s. Known trade-off (accepted per spec): with a live
   monitor a fast dump burst can overflow the 256-byte CDC ring and drop
   lines — add `Serial.flush()` after the dump blocks if dumps come out
   truncated.
2. ✅ **T16** — `CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP=y` set in ALL
   X-class sdkconfigs (dev + release + E1003/EPDIY/etc.), not just dev.
3. ✅ **T17** — reverted as a confirmed dead knob. Verified in FastEPD source:
   the 4bpp fullUpdate path derives passes from `panelDef.iMatrixSize / 16`
   and never reads `iFullPasses` — `setPasses` only affects 1bpp. Mechanism
   documented in the comment at the fast-path fullUpdate in `src/display.cpp`.
4. ✅ **T13** — extend T1/T2 fast connect to timer wakes (battery; connect step
   only, everything else on timer wakes stays untouched). Implemented in
   `connectWithSavedCredentials` (`src/wifi_network.cpp:55`): `fastConnectAndWait`
   first for all wake types, `autoConnect()` fallback, mDNS kept on both paths,
   enterprise/5 GHz rejected inside `fastConnectAndWait`; stale-lease
   recovery guardrail (bl.cpp:2050) is wake-type-agnostic so it covers timer
   wakes too. **Still needs HW check:** one timer-wake trace on battery.
5. ✅ **T14** — a) trace infra gated behind `TAP_TRACE` (dev env only; release
   gets an empty `trace()` so lib callers still link); b) partial download
   (`iCount < counter`) now returns `HTTPS_TIMED_OUT` instead of decoding a
   partial buffer; c) wake-stub indicator draws the unfilled (tap) variant.
   d) corner-hold vs CH1 bleed remains a **HW test** — still open.

Review notes: two unrequested-but-sensible dev-bench extras rode along
(`DEV_FIRMWARE` guard ignoring server-pushed OTA; macOS `monitor_rts`/`monitor_dtr`/
`monitor_port` in platformio.ini) — split into their own commits or drop when
carving this branch into PRs. The partial-download early return leaks `buffer`,
but so does the pre-existing timeout return above it and the device deep-sleeps
right after — not worth touching.

Do NOT implement **T12** (cancelled — the tap trace disproved it). Build
`pio run -e TRMNL_X_dev` after each task. Timer wakes keep today's behavior
except the scoped T13 change.

## Where the time goes — MEASURED (power-cycle cache-hit, 2.4 GHz, 2026-07-09)

Captured with the `trace()` infra (`src/bl.cpp:118`), total awake 9.7 s.
Mesh guest network ("Gothi-Mesh_Guest", ch 5, RSSI −72), image already in
LittleFS, server returned same filename.

| Phase | Measured | Notes |
|---|---|---|
| Boot → `bl_init` | 0.16 s | boot is NOT a problem — ignore my earlier PSRAM-memtest guess |
| pins/sensor/prefs/display_init | 0.05 s | non-issue |
| IQS323 init wait | 0.61 s | already skipped on the tap fast path |
| Logo repaint | 1.13 s | power-cycle only; skipped on the tap fast path |
| WiFi connect (`seq connect`) | **4.06 s** | **ALL_CHANNEL_SCAN + assoc + DHCP + mDNS, every wake** |
| `/api/display` TLS+GET | **1.48 s** | **~0.91 s TLS handshake** + DNS 0.05 s + request/response |
| FS read | 0.006 s | non-issue |
| PNG decode 1872×1404 | **0.11 s** | non-issue — T5 (speculative pre-decode) is nearly worthless |
| `fullUpdate(CLEAR_FAST)` | **1.76 s** | whole panel repainted even when image unchanged |

So the entire game is three numbers: **WiFi 4.1 s + TLS 0.9 s + panel 1.8 s**.
On a middle-tap wake the WiFi part runs on core 0 in parallel with ~0.8 s of
display/touch work, which is how the tap lands at 8–9 s instead of 9.7.

Post-fix budget, tap cache-hit: boot 0.2 + fast-connect ~0.8 (T1+T2) +
resumed TLS GET ~0.6 (T3) + strip restore ~0.3 (T6) ≈ **2–2.5 s**; with a
full repaint instead of T6 ≈ 3.5–4 s.

Note: the table above is from a power-cycle boot; a middle-tap-specific dump
was not captured yet. The tap runs the same dominant code paths (same
`autoConnect`, same `fetchApiDisplay`, same `fullUpdate`), just with WiFi on
core 0 overlapped with ~1.5 s of display work — predicted tap total 7.7–8.5 s
matches the observed 8–9 s. Phase 0 for each task: capture a real tap trace —
tap once, let it sleep, tap again; the persisted dump (`PREV CYCLE TRACE`)
prints at the start of the second wake. All checkpoints are already in place
(`api tls+GET`, `fs read`, `png decode`, `fullUpdate`, prewarm internals).

Key structural insight: on a cache hit (the common case) the **only pixels that
actually need to change are the ~120-row indicator strip** — yet we repaint the
whole panel, after a full WiFi scan, full DHCP, and a full TLS handshake, all
serialized. Every task below either caches radio/TLS state across deep sleep or
moves work off the critical path.

## Phase 0 — measure first (do before and after every task)

The `trace()` checkpoints already persist across deep sleep (RTC + NVS dump on
next boot, `src/bl.cpp:100-132`). Add checkpoints around: TLS handshake
(`https.begin`→`GET` in `fetchApiDisplay`), `png_to_epd`, `fullUpdate`, and the
prewarm task internals. Capture one tap cycle on 2.4 GHz and one on 5 GHz,
paste the dumps into this file. Do not start a task without its "before" number.

## T1 — WiFi fast-connect: cached channel + BSSID (save ~1.5–2.5 s) ✅ DONE

Implemented in `lib/wificaptive/src/connect.cpp` (builds clean, `TRMNL_X_dev`):
- `saveFastConnectCache(ssid)` persists `WiFi.channel()`+BSSID (keys `fc_ssid`/
  `fc_bssid`/`fc_ch`) in the `wificaptive` NVS namespace after every successful
  connect — called from `initiateConnectionAndWaitForOutcome` (slow path) and
  from `fastConnectAndWait` (fast path).
- `fastConnectAndWait(creds)`: `WIFI_FAST_SCAN` +
  `WiFi.begin(ssid, pswd, cached_channel, cached_bssid)`, 3 s timeout
  (`FAST_CONNECT_TIMEOUT`). On success refreshes cache; on failure invalidates
  cache + `WiFi.disconnect()` and returns `WL_CONNECT_FAILED`.
- Prewarm task (`src/bl.cpp` `wifi_prewarm_task`) calls `fastConnectAndWait`
  first on the tap path, falls back to `WifiCaptivePortal.autoConnect()` (full
  scan) on failure — one slow tap after a router change, fast again after.
- `waitForConnectResult(timeout, pollMs=100)` now takes a poll interval; fast
  path passes 10 ms (T1 bullet #4).
- Enterprise/5 GHz skip the fast path (kept on the normal scan).

_Original spec below._

## T1 spec — WiFi fast-connect: cached channel + BSSID (save ~1.5–2.5 s)

`lib/wificaptive/src/connect.cpp:216` sets `WIFI_ALL_CHANNEL_SCAN` on **every**
connect — the comment itself says it costs 1–2 s. That trade-off is right for
timer wakes (roaming networks), wrong for an interactive tap.

- After every successful connect, persist `WiFi.channel()` + BSSID (+ the SSID
  they belong to) in the `wificaptive` NVS namespace.
- On the middle-tap path (`middle_tap_fresh_fetch`), use `WIFI_FAST_SCAN` and
  call `WiFi.begin(ssid, pswd, cached_channel, cached_bssid)` — association
  goes from ~2–3 s to ~0.3–0.5 s.
- If not connected within ~3 s, invalidate the cache and fall back to the
  existing `autoConnect()` path (router rebooted / AP moved channel → one slow
  tap, then fast again).
- Also: `waitForConnectResult` (`connect.cpp:342`) polls at 100 ms — drop to
  10 ms on the tap path (free ~50 ms average, trivial).

## T2 — Skip DHCP with a cached lease (save ~0.3–1.0 s) ✅ DONE

Implemented in `lib/wificaptive/src/connect.cpp` (builds clean):
- The DHCP lease (ip/gw/mask/dns) is stored in the SAME single fast-connect NVS
  entry as T1 (implicitly keyed by `fc_bssid` — we only keep the last AP), saved
  in `saveFastConnectCache` after every successful connect.
- `applyCachedLease()` runs in `fastConnectAndWait` **only when
  `configureStaticIP` returns false** (DHCP users): `WiFi.config(ip,gw,mask,dns)`
  + `esp_netif_dhcpc_stop`, so association skips the DHCP round trip. Sets
  `g_fast_lease_applied`.
- **Guardrail (the important part):** a stale lease can still *associate* yet
  fail to route. `recoverFromStaleLease()` is called from the `downloadAndShow`
  fetch-retry loop (`src/bl.cpp`) on the first connection failure when
  `g_fast_lease_applied` — it drops the lease, re-enables DHCP
  (`WiFi.config(0,0,0)`), reconnects, and retries in the SAME wake. On
  fast-connect failure the lease is also invalidated + DHCP restored so the
  fallback `autoConnect()` binds normally.

_Original spec below._

## T2 spec — Skip DHCP with a cached lease

## T3 — TLS session resumption for /api/display (save ~0.5–0.9 s) ✅ DONE (firmware)

**Implemented 2026-07-10** in `src/api-client/display_tls.cpp`
(`fetchApiDisplayResumable`), wired into the tap path in `downloadAndShow`
(`src/bl.cpp`, before the HTTPClient retry loop, gated on
`middle_tap_fresh_fetch`). Design:
- Raw `esp_tls` GET (Arduino `WiFiClientSecure` exposes no session hook;
  `esp_http_client` can't export the session across deep sleep — no getter).
- Stays on **TLS 1.2** (resumption confirmed; avoids TLS 1.3 binary bloat).
- mbedtls session flattened with `mbedtls_ssl_session_save` into a 2 KB RTC-RAM
  buffer; reloaded via `mbedtls_ssl_session_load` on the next tap.
- Insecure (no CA + `skip_common_name`), matching the existing `setInsecure()`.
- Returns `HTTPS_NO_ERR` only on a clean 200; redirect / non-200 / chunked /
  any transport error → caller falls back to `fetchApiDisplay` (proven path).
- sdkconfig (`TRMNL_X_dev`): `CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS=y`,
  `CONFIG_ESP_TLS_INSECURE=y`, `CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY=y`.
  **On merge: replicate these three to the release X sdkconfigs** (else those
  builds compile the fallback stub and never resume).
- Parse logic (URL split, status/body split) unit-checked host-side.

**Still needs HW check:** two consecutive taps — the 2nd should log
`[resumable] session saved: N bytes` on the 1st and a shorter `api tls+GET`
span on the 2nd. Non-firmware note: S3 image leg is unaffected (T10).

_Original correction + spec below._

**Correction 2026-07-10:** the earlier "server issues no tickets" SKIP was
WRONG on two counts — it tested `usetrmnl.com` (wrong endpoint) and used
`openssl s_client -reconnect`, which is TLS 1.2 semantics. The production API
host is `trmnl.app` (Caddy, TLS 1.3). Re-verified with TLS 1.3 semantics:

```
openssl s_client -connect trmnl.app:443 -sess_out s.pem   → "Post-Handshake New Session Ticket", lifetime 604800 s (7 days)
openssl s_client -connect trmnl.app:443 -sess_in  s.pem   → "Reused, TLSv1.3"
```

Caddy issues resumable tickets by default; the firmware work is unblocked.
Scope note: this helps ONLY the `/api/display` leg (Caddy). The 4.15 s image
GET is AWS **S3** — Caddy tickets do nothing for it; that leg still needs T10.
7-day ticket lifetime easily outlives deep-sleep intervals; the serialized
session (~1–2 KB) fits RTC RAM.


`withHttp` (`lib/trmnl/include/http_client.h`) creates a fresh
`WiFiClientSecure` + full handshake every wake. TLS session tickets are just
bytes — they survive deep sleep.

- Move the tap-path `/api/display` call from HTTPClient to `esp_http_client`
  (or raw `esp_tls`) with `mbedtls_ssl_session` save/restore, persisting the
  session ticket in RTC RAM (fits: tickets are ≤~1–2 KB).
- Ticket issuance/resumption CONFIRMED on `trmnl.app` (Caddy, TLS 1.3) — use
  TLS 1.3 semantics to verify any other deployment: `-sess_out` then `-sess_in`
  (NOT `-reconnect`, which is TLS 1.2-only and misses post-handshake tickets).
- Scope: tap path only; leave `withHttp` alone for everything else. Resumed
  handshake is 1-RTT and skips the cert chain + RSA verify (~1 s on S3).

## T4 — Prewarm task owns the whole fetch, not just the connect ✅ DONE (scoped)

Implemented 2026-07-10 — see the T4 entry in "Status — Round 1" above for the
full design. Scope deliberately narrower than this spec: the prewarm task owns
connect + `/api/display` fetch only; the image download and T5-style pre-decode
stay on the main thread (it's idle waiting anyway, so moving them buys nothing).

_Original spec below._

`wifi_prewarm_task` (`src/bl.cpp:805`) stops at "connected"; the main thread
then waits and runs TLS + GET serially (`src/bl.cpp:1514-1533`,
`downloadAndShow`).

- Extend the prewarm task: connect → `fetchApiDisplay` → (if new image)
  download into RAM/FS. Stash `ApiDisplayResult` + buffer; main thread joins on
  "result ready" instead of "connected".
- Meanwhile the main thread does T5's pre-decode. The two tracks merge right
  before the panel update.
- Details: raise task stack 8192 → ~16 KB (TLS needs ~10 KB);
  `loadApiDisplayInputs` reads `vBatt`/`lipo` — on the fast path battery
  telemetry is already skipped/-1, so inputs can be built early; keep the
  existing 15 s ceiling as the failure fallback.

## T5 — Pre-decode the cached image while the radio works (DEMOTED — save ~0.12 s) ⏭️ SUBSUMED by T6

Measured decode is 113 ms and FS read 6 ms, not the 0.6–1.3 s originally
estimated. Only do this as a trivial add-on to T6 (the strip restore needs the
decoded image in the buffer anyway); it is not worth standalone effort.

## T6 — Cache hit: restore only the indicator strip (save ~1.0–1.5 s) ✅ DONE

Implemented (`display_restore_indicator_strip` in `src/display.cpp`, called from
the `DisplayedImage::matches()` cache-hit branch in `downloadAndShow`). Decodes
the cached PNG into the FastEPD framebuffer and drives only the bottom
`INDICATOR_STRIP_ROWS` (160) via `bbep.fullUpdate(CLEAR_FAST, false, &rect)`.
Removed the fast-path `DisplayedImage::clear()` + `FILENAME_KEY` hack (the
latter was already dead — nothing writes `FILENAME_KEY`). **HW tuning:** verify
160 rows fully cover the indicator and watch for strip-edge ghosting; timer
wakes still full-refresh so ghosting is bounded. Non-PNG caches fall back to a
full `display_show_image`.

_Original spec below._

## T6 spec — Cache hit: restore only the indicator strip

FastEPD's `fullUpdate(int iClearMode, bool bKeepOn, BB_RECT *pRect)` takes a
rect (`FastEPD.h:258`) — update time scales with rows driven.

- When the API says "unchanged", don't repaint the panel. Restore just the
  bottom strip under the indicator (~120 of 1404 rows) from the pre-decoded
  buffer: `fullUpdate(CLEAR_FAST, false, &rect)`.
- Then delete the fast-path hack that force-clears `PREFERENCES_FILENAME_KEY`
  + `DisplayedImage` (`src/bl.cpp:1286-1291`) — it exists only to force a full
  repaint so the indicator gets wiped. With rect-restore it's dead weight, and
  removing it also stops lying to the server about cache state.
- Ghosting: bounded — timer wakes still do normal full refreshes.

## T7 — 5 GHz / modem path (this is where 20 s+ lives)

Every wake hard-resets the ESP32-C5 (`modem_reset_target`, `src/bl.cpp:820`):
full ESP-AT firmware boot + rejoin + DHCP, all over UART that starts at 115200
before switching to 5 Mbaud (`lib/trmnl_x/src/modem.cpp:47`).

- Measure first: C5 boot→AT-ready, join, and HTTP transfer as separate trace
  checkpoints.
- Enable `AT+CWAUTOCONN=1` and skip the explicit `AT+CWJAP` when `AT+CWJAP?`
  shows already-joined — ESP-AT persists AP creds and rejoins during boot, in
  parallel with our own boot.
- `AT+UART_DEF` to persist a high baud so the 115200 bootstrap phase (AT sync,
  CWMODE, UART_CUR negotiation) disappears.
- Investigate: keep C5 EN asserted across deep sleep with the C5 in its own
  deep/light sleep (wake via UART or GPIO) so tap wakes skip the reboot
  entirely. Needs a sleep-current measurement before/after — if it costs more
  than ~10 µA, drop it.
- Profile the byte-by-byte `+HTTPCLIENT:` parser in `Modem::httpGet` at
  5 Mbaud; read in blocks if it's the bottleneck.

## T8 — Boot shavings (DEMOTED — boot measured at 0.16 s)

Boot → `bl_init` is 155 ms; PSRAM memtest is not hurting. Only keep the cheap
part: spawn the prewarm task at the very top of `bl_init` instead of after
pins/sensor/trace-dump/`preferences.begin` (`src/bl.cpp:972-1072`) — worth
~50–100 ms and it's a two-line move.

## T8b — Split the 4.06 s WiFi number before implementing T1/T2 ✅ DONE

Added `trace("wifi: STA_CONNECTED")` and `trace("wifi: GOT_IP")` in
`captureEventData` (`lib/wificaptive/src/connect.cpp`, with an `extern void
trace(...)` decl) so scan+assoc vs DHCP are now separately visible in the dump.
Note these fire only on the normal (event-handler) path; the T1 fast path
brackets itself with `fastconnect begin`/`fastconnect ok`. `waitForConnectResult`
poll is now a param (100 ms default, 10 ms on the fast path).

## T9 — Panel update cost (measure before touching)

`setPasses(3,3)` + `fullUpdate(CLEAR_FAST)` (`src/display.cpp:1796`). Measure
CLEAR_FAST at passes 2 vs 3 on real hardware for quality vs the ~30%/pass time
saving. Grayscale quality is calibrated (`u8_graytable`) — do not change
passes without side-by-side photos. Skip if T6 makes this moot for taps.

## T10 — Server-side (flag to backend, not firmware)

Fresh fetches pay two full HTTPS round trips: `/api/display` returns JSON with
an `image_url`, then a second GET (often to a different host → second TLS
handshake). Options: inline image bytes for small responses, `ETag`/
`If-None-Match` → `304` for the cache-hit case, or same-origin image URLs so
the connection can be reused. Halves the network cost of the fresh-fetch path.

## Expected end state

| Scenario | Now | After T1–T6+T8 |
|---|---|---|
| Cache hit (2.4 GHz) | 8–9 s | ~2.5–3.5 s (boot 0.6 + connect 0.7 + resumed TLS GET 0.5 + strip restore 0.4; decode hidden) |
| Fresh download (2.4 GHz) | 20 s+ | ~4.5–6 s (adds download + full decode + fullUpdate) |
| 5 GHz modem | 20 s+ | TBD by T7 measurements |

Indicator feedback lands at ~1 s in all cases (already true).

## Guardrails

- T1/T2 caches are keyed by BSSID and invalidated on first failure — worst
  case is one slow tap after a router change, never a bricked connect path.
- Timer wakes keep today's conservative behavior (full scan, full refresh,
  battery/OTA/logs); everything above is gated on the tap fast path.
- `middle_tap_fresh_fetch` is set in two places (`bl.cpp:1063` wake-stub and
  `bl.cpp:630` `check_channel_states`) — when touching the flag, keep the
  wake-stub one as the single source for the fast path.

---

# Round 2 tasks — Claude (Fable), 2026-07-10 (REVISED same day after tap trace)

## T11 — Tap trace ✅ CAPTURED. It changed the plan — read this first.

Real middle-tap trace (fresh-image path, untethered wake), key numbers:

| Phase | Measured | Verdict |
|---|---|---|
| App entry → `bl_init entry` | **~2.17 s** | NEW #1 problem (0.16 s on power-cycle) → T15 |
| `fastconnect begin` → `ok` (with lease) | **0.121 s** | **T1+T2 work on the mesh — T12 hypothesis WRONG** |
| `wait_prewarm` | 0 ms | WiFi fully off the critical path |
| `/api/display` TLS+GET | 1.65 s | known; server task (TLS tickets) |
| img GET (S3 TLS + TTFB) | **4.15 s** | server task T10 — the fresh-path killer |
| img body recv (~700 KB) | 2.42 s | bandwidth at RSSI −77; nothing to do |
| png decode | 0.12 s | fine |
| `fullUpdate(FAST)` @ passes(2,2) | **1.758 s** | ≈ identical to 1.764 s @ (3,3) → **T9 is a no-op → T17** |
| post-display FS write | 0.79 s | awake-time only, not UX; fine |
| **Total** | **13.7 s** | fresh-image tap; cache-hit projects to ~4.5–5 s |

**T12 (mesh-friendly fast connect) is CANCELLED — do not implement.** The
trace shows the BSSID pin + cached lease associating in 121 ms on the mesh.
The section is preserved in git history if a different network ever shows
`fastconnect fail` patterns.

Rules for this pass: build `pio run -e TRMNL_X_dev` after each task; timer
wakes untouched except where T13 explicitly says so.

## T15 — Kill the ~2.2 s stall before `bl_init entry` (NEW #1 priority)

On the tap wake, `bl_init entry` fired at millis()≈2167 vs 159 on a
power-cycle boot. The wake stub is innocent (verified: one bit-banged I2C
read, µs-scale, no loops). **Mechanism CONFIRMED in the HWCDC driver source**
(`framework-arduinoespressif32/cores/esp32/HWCDC.cpp`):

- Default `tx_timeout_ms = 100` (line 48); `write()` blocks in a retry loop
  while `connected && to_send`, giving up only after 100 ms of zero progress
  (lines ~456–487).
- `connected` flips back to true on ANY bus activity — an RX packet or the
  host draining a byte (lines ~94, ~136). The bench setup is the worst case:
  USB plugged but port not open (`pio monitor` reconnect loop probing it) →
  each probe re-marks "connected" → the next write blocks another 100 ms.
- bl_init prints ~40 lines (previous cycle's RTC `TRACE DUMP` + logs)
  *before* `trace("bl_init entry")`. Arithmetic: 2167 − 160 ms baseline
  ≈ 20 × 100 ms timeouts. Fully unplugged doesn't stall (writes drop fast);
  monitor attached doesn't stall (host drains) — matching 159 ms power-cycle
  vs 2167 ms tap exactly.

Fixes:
- `Serial.setTxTimeoutMs(0)` immediately after `Serial.begin()` — writes
  become non-blocking. Trade-off: with a fast burst (the 40-line dump) and a
  live monitor, the 256-byte ring can overflow and drop lines; if dump
  completeness matters, add `delay(1)` between dump lines or `Serial.flush()`
  after the block — do NOT restore a nonzero timeout.
- Move the RTC trace dump to *after* `trace("bl_init entry")` so its cost is
  measurable, and skip both boot dumps when `!Serial` (no host attached).
- Verify: re-capture a tap with USB in the reconnect-loop state (the setup
  that reproduced it). Target: `bl_init entry` ≤ 0.3 s. A control tap with
  USB fully unplugged should already be fast even without the fix — if it
  isn't, something else is also in play; instrument `setup()` then.

## T16 — Skip bootloader app re-validation on deep-sleep wake (~0.3–0.5 s wall)

`CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP` is not set in
`sdkconfigs/sdkconfig.TRMNL_X_dev`, so every deep-sleep wake re-hashes the
~1.4 MB app image in the bootloader — pure wall-clock latency that
millis()-based traces cannot see (it happens before the app starts). Set it
to `y` (dev + release sdkconfigs when merging). Standard ESP-IDF option built
for this; OTA images still validate on their first boot. This narrows the
felt-vs-traced gap (user feels 6.5–7 s on cache hits; the trace accounts for
~4.7 s).

## T17 — T9 is measurably a no-op: verify, fix, or revert

`fullUpdate(CLEAR_FAST)` with `setPasses(2,2)` took **1758 ms**; the same
update at `(3,3)` on the power-cycle trace took **1764 ms**. Zero effect.
Likely cause: 4bpp grayscale updates drive the panel from the `u8_graytable`
custom matrix (9 phase columns), not the pass count — `setPasses` probably
only affects 1-bit modes. Read FastEPD's `bbepFullUpdate` to confirm, then
either make the fast path genuinely reduce grayscale phases (only with a
side-by-side quality check) or revert the T9 code. A dead knob that looks
load-bearing is worse than no knob.

## T13 — Extend T1/T2 to timer wakes (battery) ✅ DONE

Implemented in `connectWithSavedCredentials` (`src/wifi_network.cpp:49-57`):
`fastConnectAndWait` runs first on every wake type, `autoConnect()` fallback on
miss, mDNS kept. 5 GHz still routes to the C5 modem before the fast path. The
T2 stale-lease recovery in `downloadAndShow` (bl.cpp:2050) is gated on
`g_fast_lease_applied`, not wake type, so timer wakes self-heal a stale lease in
the same wake. Builds clean on `TRMNL_X_dev`.

_Original spec below._

Gate condition met: tap trace shows `fastconnect ok` in 121 ms with the
cached lease on the user's mesh. Implement as originally specced:

- In the timer-wake connect path (`connectWithSavedCredentials`), try
  `fastConnectAndWait` first for all wake types; fall back to `autoConnect()`
  on failure. Cuts ~3 s of radio-on time per timer wake (~4.06 s → ~1 s) — a
  real battery win at 15-min refresh rates.
- Keep mDNS on the timer path; full refresh, battery, OTA, log flushing on
  timer wakes stay untouched. The connect step is the ONLY relaxation of the
  timer-wake guardrail.
- Mesh-roaming note: if the device physically moved between wakes, the pinned
  node may be suboptimal (connected but weaker). Invalidate-on-failure covers
  the hard failure; accept the soft case.

## Priority note for the backend (not firmware)

The fresh-image path spends **6.6 of its 13.7 s** inside the second HTTPS
round trip (4.15 s S3 TLS+TTFB + 2.4 s body). T10 (same-origin image URLs or
inline payload) is now demonstrably the biggest lever left anywhere — larger
than everything remaining in firmware combined.

## T14 — Pre-merge cleanup (from the review doc, still open)

All small; see `middle-tap-3s-tasks-review.md` for details.

- Gate the trace infra behind a build flag (e.g. `-D TAP_TRACE` in the dev
  env only): the NVS persist in `goToSleep`, the boot-time dumps, and the
  `Serial.printf` in `trace()`. Release builds should compile it out — an
  empty inline `trace()` is fine. Also fixes the flash-wear concern.
- Chunked download reader (`bl.cpp`): after the read loop, treat
  `iCount < counter` as a failed download (return `HTTPS_TIMED_OUT` or
  equivalent) instead of falling through to decode a partial buffer.
- Wake-stub indicator hardcodes `pending_indicator_filled = true` (hold
  style) before tap/hold is known. Either draw the unfilled (tap) variant —
  taps are the common case — or accept and remove the TODO. One line.
- Corner-hold WiFi reset vs CH1 bleed: needs a hardware test, not code.
  Verify the CH0+CH2 reset gesture still works with this firmware; if CH1
  bleed hijacks it, gate the wake-stub fast path on CH1-without-(CH0+CH2).

## NOT tasks (firmware is done here — don't touch)

- TLS ~1.1 s per handshake: server task (session tickets), already filed.
- Two full handshakes on fresh fetch (presigned S3 image URL): server task
  T10, already filed.
- Panel update times (1.76 s full / ~0.3 s strip): physics; T6/T9 already
  extract the available wins.
- PNG decode (119 ms), FS read (21 ms): non-issues, measured.
