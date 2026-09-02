"""Generate the clock font headers from the committed TTF before each build.

The headers under include/fonts/dseg_* are build artifacts and are gitignored;
the committed source is the TTF. Regeneration is skipped when the headers are
newer than both the TTF and the generator.
"""

Import("env")
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(env["PROJECT_DIR"])
TTF = ROOT / "third_party/fonts/DSEG7Modern-BoldItalic.ttf"
MANIFEST = ROOT / "include/fonts/dseg_fonts.h"
GEN = ROOT / "scripts/gen_dseg_font.py"

if not TTF.is_file():
    raise SystemExit(f"Clock font source missing: {TTF}")

stale = True
if MANIFEST.is_file():
    newest_source = max(TTF.stat().st_mtime, GEN.stat().st_mtime)
    stale = MANIFEST.stat().st_mtime < newest_source

if not stale:
    print(f"Clock font headers up to date: {MANIFEST}")
else:
    print("Generating clock font headers from", TTF.name)
    child = dict(os.environ, PIOENV=env["PIOENV"])
    result = subprocess.run([sys.executable, str(GEN)], cwd=str(ROOT), env=child)
    if result.returncode != 0 or not MANIFEST.is_file():
        raise SystemExit(
            "Clock font generation failed. It needs freetype2 and builds FastEPD's\n"
            "fontconvert on first use. On macOS: brew install freetype"
        )
