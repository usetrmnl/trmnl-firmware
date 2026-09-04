"""
Apply TRMNL FastEPD patches to the downloaded library at build time.

Do not edit .pio/libdeps/FastEPD manually — those files are wiped on clean builds.
Patch definitions live in patches/ and are applied here on every pre-build.
"""

Import("env")
from pathlib import Path

MARKER = "TRMNL: feed task WDT"
EARLY_MARKER = "TRMNL: before heavy 4bpp row work"
RECT_MARKER = "must mask the row being written"

inl = Path(env["PROJECT_LIBDEPS_DIR"]) / env["PIOENV"] / "FastEPD" / "src" / "FastEPD.inl"
if not inl.exists():
    print(f"WARNING: FastEPD.inl not found (lib not installed yet?): {inl}")
    raise SystemExit(0)

text = inl.read_text()
original = text

# --- patches/FastEPD-4bpp-rect-mask.patch ---
if RECT_MARKER not in text:
    old = "dst = (uint32_t *)pState->dma_buf;"
    new = "dst = (uint32_t *)d; // must mask the row being written (not dma_buf base)"
    if old in text:
        text = text.replace(old, new, 1)
        print(f"Applied {RECT_MARKER} (see patches/FastEPD-4bpp-rect-mask.patch)")

# --- patches/FastEPD-trmnl-wdt.patch ---
if '#include "esp_task_wdt.h"' not in text:
    text = text.replace(
        "#include <esp_log.h>",
        '#include <esp_log.h>\n#include "esp_task_wdt.h" // TRMNL: long fullUpdate/clear feed task WDT',
        1,
    )

# Feed every 16 rows (128 was too sparse for 9-pass 4bpp regional updates).
text = text.replace(
    "if ((i & 127) == 0) esp_task_wdt_reset()",
    "if ((i & 15) == 0) esp_task_wdt_reset()",
)

if EARLY_MARKER not in text:
    anchor = (
        "            for (i = 0; i < pState->native_height; i++) {\n"
        "                d = &pState->dma_buf[iDMAOff];\n"
        "                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;"
    )
    repl = (
        "            for (i = 0; i < pState->native_height; i++) {\n"
        "                if ((i & 15) == 0) esp_task_wdt_reset(); // TRMNL: before heavy 4bpp row work\n"
        "                d = &pState->dma_buf[iDMAOff];\n"
        "                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;"
    )
    if anchor in text:
        text = text.replace(anchor, repl, 1)
        print(f"Applied {EARLY_MARKER}")

if MARKER not in text:
    clear_anchor = "            bbepWriteRow(pState, pState->dma_buf, pState->native_width / 4, (i!=0));"
    if clear_anchor in text and "feed task WDT during long clears" not in text:
        text = text.replace(
            clear_anchor,
            clear_anchor + "\n            if ((i & 15) == 0) esp_task_wdt_reset(); // TRMNL: feed task WDT during long clears",
            1,
        )

    update_anchor = (
        "                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));\n"
        "                iDMAOff ^= (pState->native_width / 4); // toggle offset"
    )
    if update_anchor in text and "feed task WDT during long updates" not in text:
        text = text.replace(
            update_anchor,
            "                if ((i & 15) == 0) esp_task_wdt_reset(); // TRMNL: feed task WDT during long updates\n"
            + update_anchor,
            1,
        )

if text != original:
    inl.write_text(text)
    print(f"Patched FastEPD: {inl}")
elif RECT_MARKER in text and MARKER in text and EARLY_MARKER in text:
    print(f"FastEPD TRMNL patches already present: {inl}")
