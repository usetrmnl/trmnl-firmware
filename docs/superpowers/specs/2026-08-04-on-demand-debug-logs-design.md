# On-demand debug logs

Date: 2026-08-04
Status: Approved, not yet implemented
Repos: `trmnl-firmware`, `core`

## Problem

Devices send logs only at `LOG_ERROR` and above. When a device misbehaves there is no
way to see the `Log_info` / `Log_verbose` trail that explains why, and no way to ask a
specific device for it. Turning verbose logging on permanently would cost battery and
flash on every device in the fleet to serve the rare device under investigation.

## Constraint that shapes the design

Debug logs cannot be fetched retroactively. `src/app_logger.cpp:12` sets
`store_submit_threshold = LOG_ERROR`, so entries below that severity reach the serial
port and are discarded. Deep sleep clears RAM, so no in-memory history survives a
cycle either. A "send me everything since timestamp T" request would return an empty
set unless the device had already been storing everything — which is the fleet-wide
cost this design exists to avoid.

The timestamp therefore points forward, as a deadline, rather than backward as a
starting point.

## Design

The server marks a device as "capturing" until a deadline. While the deadline is in
the future the device lowers its log threshold to `LOG_VERBOSE` and writes entries to a
bounded file on flash. Entries upload through the existing `/api/log` path at the
existing flush points. When the deadline passes the device returns to `LOG_ERROR` on
its own, with no further server contact required.

### Why a deadline rather than a boolean

A device that stops receiving successful `/api/display` responses is a common reason to
be debugging it. Under a boolean flag such a device would stay verbose indefinitely,
draining battery and flash unobserved. A deadline terminates without needing the
server to be reachable. The server can still cancel early by sending a past timestamp.

### Why the storage backend swaps rather than a new pipeline being added

`logWithAction` → `storeLogString` → NVS → `submitStoredLogs` → POST → clear is already
store-and-forward, and already crash-survivable: entries reach flash before the radio
is involved. The only unsuitable part is the store itself — ten NVS slots
(`LOG_MAX_NOTES_NUMBER`, `include/config.h:19`). Swapping the backend behind
`storeLogString` leaves the serializer, the `/api/log` client, all five flush call
sites, and every `Log_*` call site untouched.

## Server changes (`core`)

The pattern already exists in `app/models/concerns/loggable.rb`, where `Mashup` and
`PluginSetting` carry `debug_logs_until` with `debug_logs_enabled?` and
`enable_debug_logs!` (1 day). Extend the same shape to `Device` rather than inventing a
parallel mechanism.

- Migration: `devices.debug_logs_until` (datetime, nullable)
- `Device`: `debug_logs_enabled?` and `enable_debug_logs!`, matching `Loggable`
- `app/controllers/api/device/display_controller.rb` (response hash, ~line 30–51):
  add `log_expires_at: @device.debug_logs_until&.to_i`
- UI: an enable button mirroring `plugin_settings/_debug_logs_status.html.erb`

`/api/log` needs no changes. `log_controller.rb` passes the submitted body straight to
`Telemetry.record_log` as an opaque `dump`, so the record shape below is accepted as
is. `skip_processing_logs?` maps `it['message']` over the array; records without a
`message` key yield `nil` and are harmless.

## Firmware changes (`trmnl-firmware`)

| File | Change |
| --- | --- |
| `lib/trmnl/include/api_types.h` | `uint32_t log_expires_at` on `ApiDisplayResponse` |
| `lib/trmnl/src/parse_response_api_display.cpp` | parse `doc["log_expires_at"]` |
| `src/bl.cpp`, after `fetchApiDisplay` | persist the value to preferences |
| `src/app_logger.cpp` | derive the threshold from preferences + clock |
| `src/bl.cpp` `storeLogString` | capturing → append to the ping-pong file |
| `src/bl.cpp` `submitStoredLogs` | file present → POST contents, delete on success |

### Mode evaluation

Evaluated once per wake, at logger init:

- `log_expires_at` absent from the response → leave the stored value unchanged
- stored value `0` or in the past → `LOG_ERROR`
- stored value in the future → `LOG_VERBOSE`
- `systemClock().getTime() == 0` (unsynced) → `LOG_ERROR`

Absent means "no change" rather than "off" because the maintenance-mode, 202, and
technical-difficulties paths in `display_controller.rb` render different response
hashes. Treating their silence as a cancel would drop a device out of capture exactly
when it is being watched. Termination is still guaranteed by the deadline.

An unsynced clock is treated as expired because the window cannot be verified, and
battery safety is the better default. `Clock::getTime()` returns `0` when unsynced and
NTP re-syncs at most every 24h, so a cycle of capture may be lost; the next successful
response re-arms it.

### Storage: two-file ping-pong

Per-line eviction from a single file requires rewriting it on every append — flash
churn on the 256KB OG partition, and a power cut mid-rewrite loses everything. Instead:

- Append to `/dbg_a` until it reaches 8KB, then switch to `/dbg_b`
- When the active file fills, delete the inactive one and swap
- Retention is therefore 8–16KB, and the newest entries always survive

Newest-survives is the correct direction: the lines immediately before a fault are the
diagnostic ones. Stop-when-full would preserve the boot banner and discard the crash.

Capture starts after `filesystem_init()` (`src/bl.cpp:1001` OG, `:900` X). Before the
mount the threshold stays at `LOG_ERROR` and entries fall back to the existing NVS
slots, exactly as today.

The threshold must not be raised before the file is available. The NVS store is ten
slots, five of them circular; a verbose boot sequence would thrash them and evict the
errors they exist to hold. The cost is losing verbose detail from the first moment of
boot — a device that faults before the mount yields the same error-only record it does
today.

### Record format

Two record types, distinguished by a `type` key, appended in order:

- A stamp record written once per wake cycle, carrying the `DeviceStatusStamp` fields
  that `serialize_log.cpp` currently repeats on every entry
- Lean log records: timestamp, level, message, source path, source line

The file can span multiple wake cycles when WiFi is unavailable, so one stamp per
upload would mislabel entries from earlier cycles. Writing a stamp per cycle keeps the
association correct; the server walks the array in order and carries the most recent
stamp forward.

This drops a verbose cycle from roughly 50KB to roughly 10KB. The size matters
because `submitLogToApi` holds the whole payload in heap alongside an open TLS
connection, which on the ESP32-C3 already consumes 40–50KB.

## Testing

- `lib/trmnl` builds under `env:native`, so the ping-pong rotation, the mode-evaluation
  table above, and record serialization are unit-testable there behind the existing
  `Persistence` / filesystem seams
- Response parsing of `log_expires_at`, including absence, covered in the existing
  `parse_response_api_display` tests
- On-device: enable for one device, confirm a capture cycle uploads and both files are
  removed, confirm free heap during the POST, confirm the device self-disables at the
  deadline with the server unreachable
- Both OG (arduino-esp32 2.x) and X (3.x) environments must build

## Open items

- The 8KB half-file size is an estimate. Confirm with a real free-heap measurement
  taken during the POST with TLS up before treating it as final.
- `Telemetry.record_log` collapses an entire upload into one ClickHouse row's `dump`
  string, so ~170 log lines become a single blob and per-line filtering in the logs UI
  will not work without splitting rows server-side. Independent of the firmware work;
  decide whether it belongs in this change or a follow-up.

## Not doing

- Retroactive log fetch from an arbitrary timestamp. Requires always-on capture
  fleet-wide.
- Mid-cycle flush when the buffer fills. Costs extra radio-on time; revisit only if
  16KB proves insufficient in practice.
- Per-board retention limits. X has far more flash headroom, but divergent behaviour
  under the same server command is one more variable while chasing a bug.
