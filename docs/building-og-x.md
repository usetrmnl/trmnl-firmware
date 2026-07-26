# Building TRMNL firmware: OG, BWRY, and X

This guide explains how to build the different TRMNL products from this repository, why **switching** between some of them requires an extra step, and how `scripts/pio-trmnl-switch` helps.

## Does “OG” include BWRY?

**Product-wise:** no. **OG** usually means the original mono / 1-bit TRMNL (ESP32-C3). **BWRY** is the four-color (black/white/red/yellow) variant, built as a **separate PlatformIO environment** (`trmnl_4clr`).

**Toolchain-wise:** **yes — BWRY is in the same PlatformIO family as OG.**

| Product | PlatformIO env | `BOARD_*` | PlatformIO family |
|---------|----------------|-----------|-------------------|
| TRMNL OG (mono) | `trmnl` (default) | `BOARD_TRMNL` | **og** |
| TRMNL BWRY (4-color) | `trmnl_4clr` | `BOARD_TRMNL_4CLR` | **og** |
| TRMNL X | `TRMNL_X` / `TRMNL_X_dev` | `BOARD_TRMNL_X` | **x** |

BWRY shares the OG stack: PlatformIO registry `espressif32@6.12`, Arduino-as-ESP-IDF component, ESP-IDF **4.4.x**, ESP32-C3-class flow. It does **not** use the pioarduino / IDF 5.x stack that TRMNL X uses.

So when documentation says “switch between **X** and **OG/BWRY**”, it means:

- **One side:** X (and other **x-family** envs)
- **Other side:** OG **or** BWRY (and other **og-family** envs)

You do **not** need a special “BWRY stack switch” when moving between `trmnl` and `trmnl_4clr` — only a normal `pio pkg install -e …` if packages are missing. You **do** need a family switch when moving between BWRY and X (same as OG ↔ X).

```text
                    PlatformIO families
  ┌─────────────────────────────────────────────────────────┐
  │  og family (espressif32@6.12, IDF ~4.4)                 │
  │    trmnl          ← OG mono                             │
  │    trmnl_4clr     ← BWRY (same family as OG)            │
  │    local, trmnl_test, many DIY / Seeed envs             │
  └─────────────────────────────────────────────────────────┘
                         ↕  thrash shared package names
  ┌─────────────────────────────────────────────────────────┐
  │  x family (pioarduino 53/55, IDF ~5.x)                  │
  │    TRMNL_X, TRMNL_X_dev, TRMNL_X_E1003, …               │
  │    trmnl_gen2, Sensoria, EPDIY / LilyGO / PaperS3, …    │
  └─────────────────────────────────────────────────────────┘
```

---

## Why switching is required

OG/BWRY and X both install packages under `~/.platformio/packages` with **the same directory names**, for example:

| Package slot | OG / BWRY (typical) | X (typical) |
|--------------|---------------------|-------------|
| `framework-espidf` | ESP-IDF **4.4.x** | ESP-IDF **5.5.x** |
| `framework-arduinoespressif32` | Arduino-ESP32 **2.0.x** line | Arduino-ESP32 **3.3.x** |
| `tool-esptoolpy` | esptool **2.x / 4.9** packaging | esptool **5.x** |
| `tool-cmake` / `tool-ninja` | PlatformIO registry builds | pioarduino builds |

If you build OG, then X (or the reverse) without cleanup, PlatformIO may:

1. Leave the **wrong** IDF/Arduino tree in the shared slot → `Missing Arduino framework directory 'None'` or obscure CMake errors  
2. Leave **empty stubs** such as `tool-cmake@src-…` / `tool-esptoolpy@src-…` (metadata only) →  
   - `FileNotFoundError: .../tool-cmake@src-.../bin/cmake`  
   - `does not appear to be a Python project` (esptool)

`scripts/pio-trmnl-switch` clears those shared slots, reinstalls packages for the target env, and for **x** materializes full tool trees from `~/.platformio/tools` into `packages/` when pioarduino only dropped stubs.

---

## Prerequisites

1. **PlatformIO Core** on `PATH`, ideally from the official installer’s penv:

   ```bash
   # Good (isolated Python 3.11 tooling)
   export PATH="$HOME/.platformio/penv/bin:$PATH"
   pio --version
   ```

   Avoid installing PlatformIO only with **system Python 3.14** `pip`: the pioarduino builder imports host modules (`littlefs`, `fatfs`, `intelhex`) that live in penv, not in a bare 3.14 site-packages.

2. Optional host modules (usually already in penv after an X install):

   ```bash
   pip install 'littlefs-python>=0.12' 'fatfs-ng>=0.1' 'intelhex>=2.3'
   ```

3. Run commands from the **repository root** (where `platformio.ini` lives).

---

## Quick start

### TRMNL X (README default for X hardware)

```bash
./scripts/pio-trmnl-switch x          # alias → TRMNL_X_dev
pio run -e TRMNL_X_dev
pio run -e TRMNL_X_dev -t upload
pio device monitor -e TRMNL_X_dev
```

Production X image (what CI / publish-dev use):

```bash
./scripts/pio-trmnl-switch TRMNL_X
pio run -e TRMNL_X
```

### TRMNL OG (mono)

```bash
./scripts/pio-trmnl-switch og         # alias → trmnl
pio run -e trmnl
pio run -e trmnl -t upload
pio device monitor -e trmnl
```

### TRMNL BWRY (4-color) — same family as OG

```bash
./scripts/pio-trmnl-switch bwry       # alias → trmnl_4clr
pio run -e trmnl_4clr
```

Switching **OG ↔ BWRY** does not need the heavy purge if you never built X in between; the script still does a clean `pkg install` for the target env, which is safe.

### After building X, build OG or BWRY again

```bash
./scripts/pio-trmnl-switch og         # or: bwry
pio run -e trmnl                      # or: -e trmnl_4clr
```

### Arbitrary env name

```bash
./scripts/pio-trmnl-switch TRMNL_X_E1003
./scripts/pio-trmnl-switch local
./scripts/pio-trmnl-switch seeed_reTerminal_E1001
```

Family is inferred from the env name / `platformio.ini` (`pioarduino` → **x**, otherwise **og**).

---

## Environment cheat sheet

### og family (`platform = espressif32@6.12` via `[env]`)

| Env | Product / role |
|-----|----------------|
| `trmnl` | **OG mono** (default `default_envs`) |
| `trmnl_4clr` | **BWRY** 4-color |
| `local` | OG with USB serial / dev flags |
| `trmnl_test` | OG on-device Unity tests |
| `xteink_x4`, Seeed DIY kits, Waveshare, … | Community / DIY on OG stack |

### x family (pioarduino URL in the env)

| Env | Product / role |
|-----|----------------|
| `TRMNL_X` | X production flags |
| `TRMNL_X_dev` | X developer flags (serial wait, dev firmware, …) |
| `TRMNL_X_dev_no_qa` | X dev without QA |
| `TRMNL_X_E1003`, `TRMNL_X_SENSORIA*`, … | Partner / Sensoria boards |
| `TRMNL_X_EPDIY`, `TRMNL_X_LILYGO_T5PRO`, `TRMNL_X_PAPERS3` | pioarduino **53.03.13** variant |
| `trmnl_gen2` | Gen2 (ESP32-C5), x-family tools |

### Neither (host)

| Env | Role |
|-----|------|
| `native` / `native-windows` | Unit tests on the host (no ESP flash tools thrash) |

---

## What the switch script does

1. Resolves alias → PlatformIO env + family (**og** or **x**).  
2. Deletes incomplete `tool-*@src-*` stubs and clears shared unversioned packages (`framework-espidf`, `tool-esptoolpy`, …).  
3. Removes `.pio/build/<env>` for that env (stale absolute paths / cert embeds).  
4. For **x**: clears `managed_components/` and root `dependencies.lock` so the component manager can re-fetch for the target.  
5. Runs `pio pkg install -e <env>`.  
6. For **x**: materializes full `tool-cmake` / `tool-esptoolpy` / `tool-ninja` / `tool-esp-rom-elfs` from `~/.platformio/tools` if `packages/` is still a stub; links IDF 5.x into `framework-espidf` when needed.  
7. Prints ready-to-run `pio run` / upload / monitor commands.

It does **not** flash a device and does **not** change `platformio.ini`.

---

## Minimal alternative (without the script)

If you only ever build one family, the classic README flow is enough:

```bash
pio pkg install -e TRMNL_X_dev   # once per machine / after switching families
pio run -e TRMNL_X_dev
```

```bash
pio pkg install -e trmnl
# or
pio pkg install -e trmnl_4clr
pio run -e trmnl   # or trmnl_4clr
```

If you switch families often, prefer `./scripts/pio-trmnl-switch` over bare `pkg install` so stubs and wrong IDF trees are cleared.

---

## Common failures

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `Missing Arduino framework directory 'None'` | Wrong/missing Arduino package after family switch | `./scripts/pio-trmnl-switch <target>` then rebuild |
| `No module named 'littlefs'` | `pio` running under system Python without host deps | Use `~/.platformio/penv/bin/pio`; install `littlefs-python` |
| `tool-cmake@src-.../bin/cmake` not found | Empty pioarduino stub package | Switch script materialize step, or re-run `./scripts/pio-trmnl-switch x` |
| `does not appear to be a Python project` (esptool) | Stub `tool-esptoolpy@src-*` | Same as above |
| RainMaker `Warning! Could not find file "*.crt"` early in X build | `managed_components` not fetched yet | Usually non-fatal; first configure fetches components |
| BWRY build after X fails in obscure CMake ways | Shared `framework-espidf` still IDF 5.x | `./scripts/pio-trmnl-switch bwry` (or `og`) |

---

## CI note

GitHub Actions builds **og** and **x** in **separate jobs** with separate caches so they never thrash each other. Locally you only have one `~/.platformio`; that is why the switch script exists for developers.

See `.github/workflows/build.yml`: `build-og` vs `build-x`.

---

## Related paths

| Path | Purpose |
|------|---------|
| `scripts/pio-trmnl-switch` | Family switch + `pkg install` |
| `platformio.ini` | All envs, platforms, board flags |
| `sdkconfigs/` | Per-target ESP-IDF sdkconfig for hybrid Arduino+IDF envs |
| `dependencies.lock.esp32c3` | Component lock for OG-class targets |
| `dependencies.lock.esp32s3` | Component lock for many X-class targets |
