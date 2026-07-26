# Firmware module architecture (god-module extraction)

`src/bl.cpp` and `src/display.cpp` historically held most device behavior. This
document describes the **first extraction** of network and image pipeline code
from `bl.cpp`, the temporary bridge for shared state, and the intended next
splits.

## Goals

1. Make the **server refresh path** (API → download → cache → show) reviewable
   without reading 3k+ lines of wake/UI/gesture code.
2. Keep behavior stable: same call order from `bl_init`, same globals semantics.
3. Leave clear seams for later extraction (sleep, captive portal, X touchbar).

## Module map (after this change)

```text
main.cpp
  └─ bl_init() / bl_process()          src/bl.cpp  (orchestration, wake, sleep, UI)
        │
        ├─ downloadAndShow()           src/display_session.cpp
        │     ├─ loadApiDisplayInputs()
        │     ├─ fetchApiDisplay()     src/api-client/display.cpp  (existing)
        │     ├─ handleApiDisplayResponse()
        │     └─ HTTP / modem image GET
        │           └─ ImagePipeline::*   src/image_pipeline.cpp
        │
        ├─ display_*                   src/display.cpp  (panel HAL — not split yet)
        ├─ WifiCaptive / WiFi          lib/wificaptive + bl_init
        └─ goToSleep / sensors / X UI  bl.cpp (still)

Shared state (temporary):
  include/bl_bridge.h  — externs used by display_session / image helpers
```

| Module | Responsibility | Not responsible for |
|--------|----------------|---------------------|
| **`display_session`** | `/api/display` inputs, fetch, response flags, orchestrating image download | Captive portal, deep sleep, gestures |
| **`image_pipeline`** | Stream read, SPIFFS write, NVS path keys, playlist order, previous-frame EPD load | API JSON, special-function policy |
| **`bl`** | Boot, Wi-Fi connect, credentials setup flow, sleep, messages, X touchbar | Low-level image I/O (prefer image_pipeline) |
| **`display`** | EPD init, `display_show_image`, `png_to_epd`, battery detect on X | Network |

## Public APIs

### `include/display_session.h`

- `loadApiDisplayInputs(Preferences &)`
- `downloadAndShow()`
- `handleApiDisplayResponse(ApiDisplayResponse &)`
- `szHTTPErrors[]` — names for `https_request_err_e`

### `include/image_pipeline.h` (`ImagePipeline` namespace)

- `downloadStream(WiFiClient *, int, uint8_t *)`
- `writeImageToFile(name, buf, size)`
- `persistImagePaths(preferences, newPath)` — last/current/browse (+ X playlist)
- `updatePlaylistOrder(...)`
- `loadPreviousImageIntoEpd()` — fixed to load `DisplayedImage` path (was a no-op stub)
- `cachedFileExists(String &)`

### `include/bl_bridge.h`

Documents **temporary** shared globals (`preferences`, `status`, `filename`,
`apiDisplayResult`, battery/modem on X, …). Prefer shrinking this over time by
passing a context struct into `downloadAndShow`.

## Call flow (typical wake with Wi-Fi)

1. `bl_init` — hardware, Wi-Fi, credentials if needed  
2. `downloadAndShow()`  
   - `loadApiDisplayInputs`  
   - `fetchApiDisplay` (or modem HTTP on X 5 GHz)  
   - `handleApiDisplayResponse` → sets `status`, `filename`, refresh rate, SF  
   - If image already cached: read SPIFFS → `display_show_image`  
   - Else: HTTP(S) GET image → `ImagePipeline::writeImageToFile` → show → `persistImagePaths`  
3. OTA / error screens / `goToSleep`

Setup path (`getDeviceCredentials` → `performApiSetup` → `downloadSetupImage`)
remains in `bl.cpp` for this stage; it already uses `ImagePipeline::downloadStream`
and `writeImageToFile`.

## Advantageous cleanups included in this stage

1. **`#pragma once`** on `include/api-client/display.h` (prevented `ApiDisplayResult` redefinition when headers fan out).  
2. **`png_to_epd` declared in `display.h`** (single declaration site).  
3. **Path persistence helper** (`persistImagePaths`) removes duplicated NVS rotation blocks.  
4. **`loadPreviousImageIntoEpd`** actually loads the previous file (previous `load_prev_image` never read storage).  
5. **Error table + session** colocated so logging of request results stays with the session module.  
6. **Architecture doc** (this file) for further splits.

## Recommended next extractions (ordered)

| Priority | Extract | From | Notes |
|----------|---------|------|--------|
| 1 | `performApiSetup` + `downloadSetupImage` + `getDeviceCredentials` | `bl.cpp` | Natural extension of display_session / `api_setup_session` |
| 2 | Captive portal + Wi-Fi connect block in `bl_init` | `bl.cpp` | `wifi_session.cpp` |
| 3 | Sleep / wake (`goToSleep`, button-only sleep, RTC stubs glue) | `bl.cpp` | `sleep_session.cpp` |
| 4 | X touchbar / IQS323 handlers | `bl.cpp` | Already partly in `lib/trmnl_x` |
| 5 | Split `display.cpp` by panel driver vs shared UI messages | `display.cpp` | Separate from network work |

## Rules for future PRs

- Prefer **new modules + thin bl_init** over new logic in `bl.cpp`.  
- Do not add features that only touch image I/O into `bl.cpp` — use `ImagePipeline`.  
- When a function needs many bl globals, either extend `bl_bridge.h` with a
  comment, or introduce a `struct DisplaySessionContext` and pass it.  
- Keep OG and X `#ifdef` paths in the same module until a board HAL exists;
  do not fork entire files per product yet.  
- Build both `trmnl` and `TRMNL_X_dev` (or CI `build-og` / `build-x`) before merge.

## Related docs

- [building-og-x.md](building-og-x.md) — PlatformIO families (OG/BWRY vs X) and switch script  
- [README.md](../README.md) — compilation entry points  
