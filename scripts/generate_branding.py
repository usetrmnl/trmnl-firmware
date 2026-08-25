#!/usr/bin/env python3
"""
generate_branding.py - Read config.yml and emit:
  1. include/branding.h                     (string / URL #define macros)
  2. src/wifi_connect_qr.h, wifi_failed_qr.h (QR bitmaps for the configured URLs)

These outputs are build artifacts (git-ignored) regenerated on every build.

Defaults reproduce the stock TRMNL strings/URLs, so an unmodified config.yml
produces byte-identical output.

Downstream white-labels put their overrides in config.local.yml (git-ignored),
which is deep-merged over config.yml. Customizing therefore never edits a
tracked file, so syncing upstream does not create merge conflicts.

Runs standalone or as a PlatformIO pre-build script. The QR step depends on the
`segno` package (see scripts/requirements.txt); it is installed on demand so the
QR bitmaps always regenerate from config.yml.
"""

import os
import re
import struct

# ---------------------------------------------------------------------------
# Minimal YAML parser (no external dependency for CI / PlatformIO)
# ---------------------------------------------------------------------------

def _parse_yaml(path):
    """Parse a *simple* YAML file (scalars + nested dicts). Returns a dict."""
    result = {}
    stack = [(result, -1)]  # (dict, indent)
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.rstrip("\n\r")
            stripped = line.lstrip()
            if not stripped or stripped.startswith("#"):
                continue
            indent = len(line) - len(stripped)
            while len(stack) > 1 and indent <= stack[-1][1]:
                stack.pop()
            parent = stack[-1][0]
            m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*):\s*(.*)", stripped)
            if not m:
                continue
            key, value = m.group(1), m.group(2).strip()
            if value == "" or value.startswith("#"):
                child = {}
                parent[key] = child
                stack.append((child, indent))
            else:
                if (value.startswith('"') and value.endswith('"')) or \
                   (value.startswith("'") and value.endswith("'")):
                    value = value[1:-1]
                parent[key] = value
    return result

def _deep_merge(base, override):
    """Recursively merge *override* into *base* (both dicts); returns *base*."""
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base

def _expand(template, variables):
    """Replace {key} placeholders in *template* with values from *variables*."""
    return re.sub(r"\{(\w+)\}", lambda m: variables.get(m.group(1), m.group(0)), template)


def _c_escape(s):
    return s.replace("\\", "\\\\").replace('"', '\\"')


# ---------------------------------------------------------------------------
# Group5 1-bpp encoder (pure-Python port of Larry Bank's encoder). Produces the
# same BB_BITMAP format that bbep.loadG5Image() consumes.
# ---------------------------------------------------------------------------

_BITCOUNT = [0] * 256
for _i in range(256):
    _n, _v = 0, _i
    while _v & 0x80:
        _n += 1
        _v = (_v << 1) & 0xFF
    _BITCOUNT[_i] = _n

_VTABLE = [
    (0b0000011, 7), (0b000011, 6), (0b011, 3), (0b1, 1),
    (0b010, 3), (0b000010, 6), (0b0000010, 7),
]
_HORIZ_SS, _HORIZ_SL, _HORIZ_LS, _HORIZ_LL = 0, 1, 2, 3
_MAX_FLIPS = 512


class _BitWriter:
    __slots__ = ("buf", "accum", "bit_off")

    def __init__(self):
        self.buf = bytearray()
        self.accum = 0
        self.bit_off = 0

    def put(self, code, length):
        self.accum |= (code << (32 - self.bit_off - length))
        self.bit_off += length
        while self.bit_off >= 8:
            self.buf.append((self.accum >> 24) & 0xFF)
            self.accum = (self.accum << 8) & 0xFFFFFFFF
            self.bit_off -= 8

    def flush(self):
        if self.bit_off:
            self.buf.append((self.accum >> 24) & 0xFF)
            self.accum = 0
            self.bit_off = 0


def _pixels_to_flips(row_bytes, width):
    flips = []
    x = 0
    byte_idx = 0
    nbytes = len(row_bytes)
    c = row_bytes[0] if nbytes > 0 else 0xFF
    cbits = 8
    while x < width:
        run = 0
        while True:
            i = _BITCOUNT[c]
            run += i
            c = (c << i) & 0xFF
            cbits -= i
            if cbits <= 0:
                run += cbits
                byte_idx += 1
                if byte_idx >= nbytes:
                    cbits = 0
                    break
                c = row_bytes[byte_idx]
                cbits = 8
            else:
                break
        x += run
        if x >= width:
            break
        flips.append(min(x, width))
        c = (~c) & 0xFF
        run = 0
        while True:
            i = _BITCOUNT[c]
            run += i
            c = (c << i) & 0xFF
            cbits -= i
            if cbits <= 0:
                run += cbits
                byte_idx += 1
                if byte_idx >= nbytes:
                    cbits = 0
                    break
                c = row_bytes[byte_idx]
                c = (~c) & 0xFF
                cbits = 8
            else:
                c = (~c) & 0xFF
                break
        x += run
        if x >= width:
            break
        flips.append(min(x, width))
    flips.extend([width] * 4)
    return flips


def g5_encode(pixels_1bpp, width, height):
    """Encode 1-bpp rows (MSB-first, white=1/black=0) into Group5 data."""
    hlen = (width - 1).bit_length()
    bw = _BitWriter()
    ref = [width] * (_MAX_FLIPS + 4)
    for y in range(height):
        cur = _pixels_to_flips(pixels_1bpp[y], width)
        a0 = 0
        icur = 0
        iref = 0
        while a0 < width:
            b1 = ref[iref]
            b2 = ref[iref + 1] if (iref + 1) < len(ref) else width
            a1 = cur[icur] if icur < len(cur) else width
            if b2 < a1:
                bw.put(0b0001, 4)
                a0 = b2
                iref += 2
            else:
                dx = b1 - a1
                if dx > 3 or dx < -3:
                    bw.put(0b001, 3)
                    run1 = (cur[icur] if icur < len(cur) else width) - a0
                    run2 = (cur[icur + 1] if (icur + 1) < len(cur) else width) - \
                           (cur[icur] if icur < len(cur) else width)
                    if run1 < 8:
                        if run2 < 8:
                            bw.put(_HORIZ_SS, 2); bw.put(run1, 3); bw.put(run2, 3)
                        else:
                            bw.put(_HORIZ_SL, 2); bw.put(run1, 3); bw.put(run2, hlen)
                    else:
                        if run2 < 8:
                            bw.put(_HORIZ_LS, 2); bw.put(run1, hlen); bw.put(run2, 3)
                        else:
                            bw.put(_HORIZ_LL, 2); bw.put(run1, hlen); bw.put(run2, hlen)
                    a0 = cur[icur + 1] if (icur + 1) < len(cur) else width
                    if a0 != width:
                        icur += 2
                        while iref < len(ref) and ref[iref] != width and ref[iref] <= a0:
                            iref += 2
                else:
                    vcode, vlen = _VTABLE[dx + 3]
                    bw.put(vcode, vlen)
                    a0 = a1
                    if a0 != width:
                        if iref != 0:
                            iref -= 2
                        iref += 1
                        icur += 1
                        while iref < len(ref) and ref[iref] <= a0 and ref[iref] != width:
                            iref += 2
        ref = cur
    bw.flush()
    return bytes(bw.buf)


def make_bb_bitmap(g5_data, width, height):
    """Wrap G5 data in a BB_BITMAP header (little-endian), matching loadG5Image."""
    hdr = struct.pack("<HHH H", 0xBBBF, width, height, len(g5_data))
    return hdr + g5_data


def _write_g5_header(rows, width, height, var_name, out_path, source_note):
    g5_data = g5_encode(rows, width, height)
    bb_data = make_bb_bitmap(g5_data, width, height)
    vals = [f"0x{b:02x}" for b in bb_data]
    hex_body = ",\n    ".join(",".join(vals[i:i + 16]) for i in range(0, len(vals), 16))
    content = (
        f"//\n// Auto-generated by scripts/generate_branding.py from {source_note}\n"
        f"// {width} x {height} x 1-bit per pixel\n"
        f"// compressed image data size = {len(g5_data)} bytes\n//\n"
        f"const uint8_t {var_name}[] = {{\n    {hex_body}}};\n"
    )
    if os.path.exists(out_path):
        with open(out_path, "r", encoding="utf-8") as f:
            if f.read() == content:
                print(f"[branding] {out_path} is up-to-date")
                return
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"[branding] wrote {out_path} ({width}x{height}, {len(g5_data)} bytes compressed)")


# ---------------------------------------------------------------------------
# QR code generation (dependency: segno, auto-installed on demand)
# ---------------------------------------------------------------------------

def _ensure_segno():
    """Import segno, installing it on demand so the QR codes always regenerate."""
    try:
        import segno
        return segno
    except ImportError:
        pass
    import subprocess
    import sys
    print("[branding] installing 'segno' to generate QR codes...")
    for extra in ([], ["--break-system-packages"]):
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", "segno"] + extra)
            break
        except subprocess.CalledProcessError:
            continue
    import segno  # raises if the install did not succeed
    return segno


def _qr_rows(matrix, scale, border):
    """Render a segno boolean matrix (True = dark) into 1-bpp byte rows
    (MSB-first, white=1/black=0) with a light quiet-zone border, each module
    expanded to *scale* pixels."""
    n = len(matrix)
    total = (n + 2 * border) * scale
    row_byte_width = (total + 7) // 8

    def module_dark(mx, my):
        if my < border or mx < border:
            return False
        my -= border
        mx -= border
        if my >= n or mx >= n:
            return False
        return bool(matrix[my][mx])

    rows = []
    for py in range(total):
        my = py // scale
        row = bytearray(b"\xff" * row_byte_width)  # start all-white (1)
        for px in range(total):
            mx = px // scale
            if module_dark(mx, my):
                row[px >> 3] &= ~(0x80 >> (px & 7)) & 0xFF  # black -> clear bit
        rows.append(bytes(row))
    return rows, total


def generate_qr_header(url, var_name, out_path, segno, scale=2, border=4):
    q = segno.make(url, error="m")
    matrix = list(q.matrix)
    rows, size = _qr_rows(matrix, scale, border)
    _write_g5_header(rows, size, size, var_name, out_path, f'QR for "{url}"')


# ---------------------------------------------------------------------------
# branding.h (strings + URLs)
# ---------------------------------------------------------------------------

def _string_variables(cfg):
    branding = cfg.get("branding", {})
    urls = cfg.get("urls", {})
    return {
        "device_name": branding.get("device_name", "TRMNL"),
        "setup_url": urls.get("setup_url", "trmnl.com/start"),
        "api_base_url": urls.get("api_base_url", "https://trmnl.app"),
    }


def generate_branding_h(cfg, out_path):
    variables = _string_variables(cfg)
    strings = cfg.get("strings", {})
    lines = [
        "// ============================================================",
        "// AUTO-GENERATED by scripts/generate_branding.py - DO NOT EDIT",
        "// Edit config.yml and re-run the generator instead.",
        "// ============================================================",
        "#ifndef BRANDING_H",
        "#define BRANDING_H",
        "",
        "// --- Core identity ---",
        f'#define BRAND_DEVICE_NAME  "{_c_escape(variables["device_name"])}"',
        "",
        "// --- URLs ---",
        f'#define BRAND_API_BASE_URL "{_c_escape(variables["api_base_url"])}"',
        f'#define BRAND_SETUP_URL    "{_c_escape(variables["setup_url"])}"',
        "",
        "// --- User-facing strings ---",
    ]
    for key, template in strings.items():
        macro = "BRAND_STR_" + key.upper()
        expanded = _expand(template, variables)
        lines.append(f'#define {macro:<40s} "{_c_escape(expanded)}"')
    lines += ["", "#endif  // BRANDING_H", ""]
    content = "\n".join(lines)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        with open(out_path, "r", encoding="utf-8") as f:
            if f.read() == content:
                print(f"[branding] {out_path} is up-to-date")
                return
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"[branding] wrote {out_path}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(project_dir=None):
    if project_dir is None:
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    config_path = os.path.join(project_dir, "config.yml")
    if not os.path.isfile(config_path):
        print(f"[branding] WARNING: {config_path} not found - using defaults")
        cfg = {}
    else:
        cfg = _parse_yaml(config_path)

    # Optional downstream overrides, deep-merged over config.yml. Keeping
    # customization in this git-ignored file means tracked files never change,
    # so pulling upstream never conflicts.
    local_path = os.path.join(project_dir, "config.local.yml")
    if os.path.isfile(local_path):
        print(f"[branding] applying overrides from {local_path}")
        _deep_merge(cfg, _parse_yaml(local_path))

    generate_branding_h(cfg, os.path.join(project_dir, "include", "branding.h"))

    # --- QR codes (regenerated from config.yml URLs) ---
    qr = cfg.get("qr", {})
    src_dir = os.path.join(project_dir, "src")
    specs = [
        (qr.get("connect_url", "https://trmnl.com/start"), "wifi_connect_qr", "wifi_connect_qr.h"),
        (qr.get("help_url", "https://help.usetrmnl.com"), "wifi_failed_qr", "wifi_failed_qr.h"),
    ]
    segno = _ensure_segno()
    for url, var_name, out_name in specs:
        generate_qr_header(url, var_name, os.path.join(src_dir, out_name), segno)


# PlatformIO pre-script hook ------------------------------------------------
try:
    Import("env")  # noqa: F821
    _proj_dir = env.subst("$PROJECT_DIR") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # noqa: F821
    main(_proj_dir)
except NameError:
    pass


if __name__ == "__main__":
    main()
