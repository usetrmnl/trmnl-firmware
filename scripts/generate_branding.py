#!/usr/bin/env python3
"""
generate_branding.py - Read config.yml and emit:
  1. include/branding.h                     (string / URL #define macros)
  2. src/wifi_connect_qr.h, wifi_failed_qr.h (QR bitmaps for the configured URLs)
  3. lib/wificaptive/src/WifiCaptivePage.h  (gzipped captive-portal page, from
     lib/wificaptive/portal/index.html with the accent colour, device name and
     optional logo applied)
  4. optional *.local.h sidecars (custom e-paper logos, Wi-Fi AP SSID prefix)

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

import gzip
import io
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
                dq = re.match(r'^"((?:[^"\\]|\\.)*)"\s*(?:#.*)?$', value)
                sq = re.match(r"^'([^']*)'\s*(?:#.*)?$", value)
                if dq:
                    value = re.sub(r"\\(.)", r"\1", dq.group(1))  # \" and \\ escapes
                elif sq:
                    value = sq.group(1)
                else:
                    value = re.sub(r"\s+#.*$", "", value)  # trailing comment
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
    _write_bb_bitmap_header(make_bb_bitmap(g5_data, width, height), var_name, out_path, source_note)


def _write_bb_bitmap_header(bb_data, var_name, out_path, source_note):
    width, height, g5_len = struct.unpack("<HHH", bb_data[2:8])
    g5_data = bb_data[8:]
    assert len(g5_data) == g5_len
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


# URLs encoded in the QR bitmaps that used to be committed as src/*_qr.h.
STOCK_QR_CONNECT_URL = "https://help.usetrmnl.com/en/articles/9416306-how-to-set-up-a-new-device"
STOCK_QR_HELP_URL = "https://help.usetrmnl.com/en/articles/10193157-device-wifi-troubleshooting"

# The bitmaps that were committed as src/*_qr.h (made with Larry Bank's
# imageconvert). A stock URL emits its original bitmap byte-for-byte, so stock
# builds draw exactly the QR codes the golden screenshots (spec/golden/) expect;
# other URLs are rendered with segno below.
STOCK_QR_BITMAPS = {
    STOCK_QR_CONNECT_URL: bytes.fromhex(
        "bfbb420042008a01281c4c89118422a10583bffe542a45848972281ca857ffffc8b72a181e2105916fffffff0c8a1349"
        "120fffffffff841c89621ffffffff8f225e1ca840444b1ffffff8ff84104448897c7fffff8f0e185879880fff9042244"
        "9030f841722c453fffff841926225861122864908a7ffffc33109912108431ffff22845022a10192422448889126ffff"
        "f8411120e1c2082108219120fffffc30893112082c91430821fffffcc230712641044c82ffff10c8b04452448924997f"
        "fff9262a10104448431088b07fffc8142088b6165440c2207ffff0c89430844c21185284453fff082c391422c488324d"
        "c987ffffc291410e10410445b225ffffe30a1112268186115083088105ffffca04430a11121ca8405ffff87108920411"
        "1408b04114c2ffffe11143881045a193210cc207ffc448a461122124c4cbffe503a14921342652250fffff950a864583"
        "0c632281ffff916e4c8c210724430ffffffe111208e2040932228105ffffff8c3088a4c2106185fffff1c8943c22252a"
        "241ffffe3891610c3228448222444bffff00"
    ),
    STOCK_QR_HELP_URL: bytes.fromhex(
        "bfbb420042009301281c488911322a18583bffe542a104106191222c1ca857ffffc8b71088b041112082c8b7ffffffe1"
        "850bc3ffffffffe1150cc3225fffffffe3c322410444899041112c7fffffe38410444bc897f1fffffe30e38c30b3101f"
        "fc8211224848a10449089722c453fffffe30f0c3870c92114ffff86308222444a5c387fffe4c843082228191422524df"
        "fff2090c8a1320c420820864483fffff082083308c48820a18410ffffe192208308a841142641044c82fffff8c61841c"
        "8a4997ffff249224448890842211160ffffe48846261508a4c2207ffc821122292a11872a144502229fffff0a1841870"
        "8894324dc987ffff10a1044d02e10445b225ffffe104111291688619844082ffff8448844208a84112ca8405ffff8624"
        "d2448499160822985ffff08891120e1041112228193210cc207ffff111c4308894890926265fffca0749a04104104113"
        "291287fffff2a150f0c286319140fffff22dc22250889061c910c3fffffff8410421916126445020bffffff191487308"
        "418617ffffc722818ca8907ffe38611842864508904448897ffff8"
    ),
}

# The message screens place the QR bitmaps as fixed 66x66 images (2x on TRMNL X).
QR_SIZE = 66
QR_SCALE = 2


def generate_qr_header(url, var_name, out_path, segno):
    """Render *url* as a QR_SIZE x QR_SIZE bitmap: 2 px per module, centred, with
    whatever quiet zone is left over (none for a version-4 code, like the stock
    help-article URLs; the white screen around it serves as the quiet zone)."""
    if url in STOCK_QR_BITMAPS:
        _write_bb_bitmap_header(STOCK_QR_BITMAPS[url], var_name, out_path, f'QR for "{url}" (stock bitmap)')
        return
    q = segno.make(url, error="l", micro=False)  # segno boosts the ECC level when it fits
    matrix = list(q.matrix)
    border = (QR_SIZE // QR_SCALE - len(matrix)) // 2
    if border < 0:
        raise ValueError(
            f"[branding] QR URL too long for a {QR_SIZE}x{QR_SIZE} code (needs QR version "
            f"{q.version}, max 4): {url}"
        )
    rows, size = _qr_rows(matrix, QR_SCALE, border)
    assert size == QR_SIZE
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
    logo_choice = str(cfg.get("branding", {}).get("message_logo_size", "small")).lower()
    logo_symbol = {"small": "logo_small", "medium": "logo_medium", "big": "logo_big"}.get(logo_choice, "logo_small")
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
        f"#define BRAND_MESSAGE_LOGO {logo_symbol}",
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
# Custom logo generation (dependency: pillow, auto-installed on demand)
# ---------------------------------------------------------------------------

def _ensure_pillow():
    """Import PIL.Image, installing pillow on demand. Returns Image or None."""
    try:
        from PIL import Image
        return Image
    except ImportError:
        pass
    import subprocess
    import sys
    print("[branding] installing 'pillow' to convert custom images...")
    for extra in ([], ["--break-system-packages"]):
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", "pillow"] + extra)
            break
        except subprocess.CalledProcessError:
            continue
    try:
        from PIL import Image
        return Image
    except ImportError:
        return None


def generate_image_header(png_path, var_name, expected_w, expected_h, out_path):
    """Convert a 1-bit PNG of exact size into a G5 BB_BITMAP C header."""
    Image = _ensure_pillow()
    if Image is None:
        print(f"[branding] WARNING: pillow unavailable, cannot convert {png_path}")
        return False
    img = Image.open(png_path).convert("1")  # force 1-bit
    if img.size != (expected_w, expected_h):
        raise ValueError(
            f"{png_path}: expected {expected_w}x{expected_h} but got {img.size[0]}x{img.size[1]}"
        )
    row_byte_width = (expected_w + 7) // 8
    rows = []
    for y in range(expected_h):
        row = bytearray(row_byte_width)
        for x in range(expected_w):
            if img.getpixel((x, y)):  # white pixel -> set bit (white=1)
                row[x >> 3] |= 0x80 >> (x & 7)
        tail = expected_w & 7
        if tail:
            row[-1] |= 0xFF >> tail
        rows.append(bytes(row))
    _write_g5_header(rows, expected_w, expected_h, var_name, out_path, os.path.basename(png_path))
    return True


# ---------------------------------------------------------------------------
# Captive-portal page (WifiCaptivePage.h) generation
# ---------------------------------------------------------------------------

STOCK_DEVICE_NAME = "TRMNL"
STOCK_ACCENT_COLOR = "#F86527"

# User-visible "TRMNL" occurrences in lib/wificaptive/portal/index.html, as
# (exact snippet, context). Only page text and aria-labels are rebranded; URLs,
# ids, placeholders, hostnames and the AP SSID are left alone. Context "html" is
# HTML text/attributes, "js" a single-quoted JS string set via textContent, and
# "js_html" a single-quoted JS string assigned to innerHTML.
_PORTAL_NAME_SITES = [
    ("<title>TRMNL Wi-Fi Configuration</title>", "html"),
    ("'Could not reach your TRMNL. Check that you are still on its Wi-Fi network, then try again.'", "js"),
    ("'Soft reset complete. Your TRMNL is back to its default settings.'", "js"),
    ("btn.textContent = 'Soft reset this TRMNL';", "js"),
    ("'<strong>Test did not run</strong>Check that you are still on your TRMNL\\'s Wi-Fi network.'", "js_html"),
    ('role="img" aria-label="TRMNL">', "html"),
    ('<h1 class="p-title" tabindex="-1">Connect your TRMNL</h1>', "html"),
    ("<span>Keep this page open. Your TRMNL restarts when it's connected.</span>", "html"),
    ('<p class="hint">Point your TRMNL at your own server (BYOS).</p>', "html"),
    ('onclick="confirmSoftReset()">Soft reset this\n            TRMNL</button>', "html"),
    ("<p id=\"resetTitle\"><strong>Soft reset this TRMNL?</strong></p>", "html"),
]


def _html_escape(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            .replace('"', "&quot;"))


def _js_escape(s):
    return s.replace("\\", "\\\\").replace("'", "\\'").replace("<", "\\x3c")


def _portal_settings(cfg):
    branding = cfg.get("branding", {})
    device_name = branding.get("device_name", STOCK_DEVICE_NAME) or STOCK_DEVICE_NAME
    accent = branding.get("accent_color", STOCK_ACCENT_COLOR) or STOCK_ACCENT_COLOR
    if re.search(r"[;{}<>]", accent):
        print(f"[branding] WARNING: ignoring invalid branding.accent_color {accent!r}")
        accent = STOCK_ACCENT_COLOR
    return device_name, accent


def _brand_portal_html(html_text, device_name, accent, svg_logo):
    """Apply branding to the stock captive-portal index.html (decoded text)."""
    # (a) Accent colour: the page derives every accent from the --spark CSS
    # variable (light and dark palettes); point both at the configured colour.
    if accent != STOCK_ACCENT_COLOR:
        html_text, n = re.subn(r"(--spark:\s*)#[0-9A-Fa-f]{3,8}(\s*;)",
                               lambda m: m.group(1) + accent + m.group(2), html_text)
        if n == 0:
            print("[branding] WARNING: no --spark declaration found in the portal page")

    # (b) Device name in user-visible text.
    if device_name != STOCK_DEVICE_NAME:
        encoded = {
            "html": _html_escape(device_name),
            "js": _js_escape(device_name),
            "js_html": _js_escape(_html_escape(device_name)),
        }
        for snippet, ctx in _PORTAL_NAME_SITES:
            if snippet not in html_text:
                print(f"[branding] WARNING: portal text not found, not rebranded: {snippet!r}")
                continue
            html_text = html_text.replace(snippet, snippet.replace("TRMNL", encoded[ctx]))
        leftover = len(re.findall(r"\bTRMNL\b", html_text))
        if leftover:
            print(f"[branding] WARNING: {leftover} 'TRMNL' occurrence(s) left in the portal page; "
                  "update _PORTAL_NAME_SITES in scripts/generate_branding.py")

    # (c) Custom logo: replace the inline <svg class="logo"> element.
    if svg_logo is not None:
        svg = re.sub(r"<\?xml.*?\?>|<!DOCTYPE.*?>|<!--.*?-->", "", svg_logo, flags=re.DOTALL).strip()
        m = re.match(r"<svg\b[^>]*>", svg)
        if not m:
            print("[branding] WARNING: images.portal_logo is not an <svg> element, using stock logo")
        else:
            tag = m.group(0)
            new_tag = tag
            if not re.search(r"\sclass=", tag):
                new_tag = new_tag[:4] + ' class="logo"' + new_tag[4:]
            if not re.search(r"\saria-label=", tag):
                new_tag = new_tag[:-1] + f' role="img" aria-label="{_html_escape(device_name)}">'
            svg = new_tag + svg[len(tag):]
            html_text, n = re.subn(r'<svg class="logo"[^>]*>.*?</svg>', lambda _m: svg, html_text,
                                   count=1, flags=re.DOTALL)
            if n == 0:
                print('[branding] WARNING: no <svg class="logo"> found in the portal page')
    return html_text


def _gzip_bytes(data):
    buf = io.BytesIO()
    with gzip.GzipFile(fileobj=buf, mode="wb", mtime=0) as gz:  # mtime=0 for reproducibility
        gz.write(data)
    return buf.getvalue()


def generate_captive_portal(cfg, portal_dir, out_path, svg_logo=None):
    """Brand the portal page, gzip it, and write the WifiCaptivePage.h header.

    The array names match what the old lib/wificaptive/portal/convert.py
    produced (INDEX_HTML / INDEX_HTML_LEN). With no portal branding configured,
    the raw source bytes are gzipped unchanged, so the page is the stock one.
    """
    if not os.path.isdir(portal_dir):
        print(f"[branding] WARNING: {portal_dir} not found, skipping portal page")
        return
    device_name, accent = _portal_settings(cfg)
    apply_branding = (device_name != STOCK_DEVICE_NAME or accent != STOCK_ACCENT_COLOR
                      or svg_logo is not None)
    entries = []
    for fname in sorted(os.listdir(portal_dir)):
        if not fname.endswith((".html", ".svg")):
            continue
        with open(os.path.join(portal_dir, fname), "rb") as f:
            data = f.read()
        if fname == "index.html" and apply_branding:
            data = _brand_portal_html(data.decode("utf-8"), device_name, accent, svg_logo).encode("utf-8")
        compressed = _gzip_bytes(data)
        array_name = fname.replace(".", "_").upper()
        hex_vals = ", ".join(hex(b) for b in compressed)
        entries.append(
            f"const uint8_t {array_name}[] PROGMEM = {{ {hex_vals} }};\n"
            f"const int {array_name}_LEN = sizeof({array_name});\n"
        )
    content = (
        "// AUTO-GENERATED by scripts/generate_branding.py from lib/wificaptive/portal/ - DO NOT EDIT\n"
        "#ifndef WifiCaptivePage_h\n"
        "#define WifiCaptivePage_h\n\n"
        "#include <pgmspace.h>\n\n"
        + "\n".join(entries)
        + "\n#endif\n"
    )
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

    src_dir = os.path.join(project_dir, "src")

    # --- QR codes (regenerated from config.yml URLs) ---
    qr = cfg.get("qr", {})
    specs = [
        (qr.get("connect_url", STOCK_QR_CONNECT_URL), "wifi_connect_qr", "wifi_connect_qr.h"),
        (qr.get("help_url", STOCK_QR_HELP_URL), "wifi_failed_qr", "wifi_failed_qr.h"),
    ]
    segno = _ensure_segno()
    for url, var_name, out_name in specs:
        generate_qr_header(url, var_name, os.path.join(src_dir, out_name), segno)

    # --- Captive-portal page (git-ignored build artifact) ---
    images = cfg.get("images", {})
    portal_dir = os.path.join(project_dir, "lib", "wificaptive", "portal")
    captive_h = os.path.join(project_dir, "lib", "wificaptive", "src", "WifiCaptivePage.h")
    svg_logo = None
    portal_logo = images.get("portal_logo", "")
    if portal_logo:
        abs_svg = os.path.join(project_dir, portal_logo)
        if os.path.isfile(abs_svg):
            with open(abs_svg, "r", encoding="utf-8") as f:
                svg_logo = f.read()
        else:
            print(f"[branding] WARNING: {portal_logo} not found, using stock portal logo")
    generate_captive_portal(cfg, portal_dir, captive_h, svg_logo)

    # --- Optional custom logos -> git-ignored *.local.h sidecars ---
    # Stock headers stay the tracked default; a configured image emits a sidecar
    # that the committed header picks up via __has_include, so customizing logos
    # never modifies a tracked file.
    image_specs = [
        ("logo_small",  "logo_small",  86,  86,  "logo_small.local.h"),
        ("logo_medium", "logo_medium", 240, 240, "logo_medium.local.h"),
        ("logo_big",    "logo_big",    640, 640, "logo_big.local.h"),
        ("loading",     "loading",     800, 480, "loading.local.h"),
    ]
    for cfg_key, var_name, w, h, out_name in image_specs:
        path = images.get(cfg_key, "")
        if not path:
            stale = os.path.join(src_dir, out_name)
            if os.path.exists(stale):
                os.remove(stale)  # override removed from config: revert to the stock image
            continue
        abs_png = os.path.join(project_dir, path)
        if not os.path.isfile(abs_png):
            print(f"[branding] WARNING: {path} not found, skipping {cfg_key}")
            continue
        generate_image_header(abs_png, var_name, w, h, os.path.join(src_dir, out_name))

    # --- Optional WiFi AP SSID prefix -> git-ignored lib-local sidecar ---
    # WifiCaptive.h picks it up via __has_include; stock falls back to "TRMNL".
    ap_prefix = cfg.get("wifi", {}).get("ap_ssid_prefix", "")
    ssid_h = os.path.join(project_dir, "lib", "wificaptive", "src", "wifi_ssid.local.h")
    if ap_prefix and ap_prefix != "TRMNL":
        ssid_content = (
            "// Auto-generated by scripts/generate_branding.py - DO NOT EDIT\n"
            f'#define BRAND_WIFI_AP_PREFIX "{_c_escape(ap_prefix)}"\n'
        )
        if not (os.path.exists(ssid_h) and open(ssid_h, encoding="utf-8").read() == ssid_content):
            with open(ssid_h, "w", encoding="utf-8") as f:
                f.write(ssid_content)
            print(f"[branding] wrote {ssid_h}")
    elif os.path.exists(ssid_h):
        os.remove(ssid_h)  # revert to stock prefix

    return cfg


# PlatformIO pre-script hook ------------------------------------------------
try:
    Import("env")  # noqa: F821
    _pio_env = env  # noqa: F821
except NameError:  # run standalone, not by PlatformIO
    _pio_env = None
if _pio_env is not None:
    main(_pio_env.subst("$PROJECT_DIR"))


if __name__ == "__main__" and _pio_env is None:
    main()
