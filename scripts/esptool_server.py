#!/usr/bin/env python3
"""Local Flash Assistant: serve a single-page web flasher.

Drop a merged_firmware.bin onto the page, plug the device in (in flashing mode),
and click Install. Flashing happens in the browser via ESP Web Tools — Python
is only serving the HTML.

Requires a Chromium-based browser with Web Serial API (Chrome, Edge, Opera, Arc).

Usage:
  scripts/esptool_server.py              # serve on a random free port, open browser
  scripts/esptool_server.py --port 9000   # pin to a specific port
  scripts/esptool_server.py --no-open
"""

import argparse
import http.server
import socketserver
import webbrowser


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>TRMNL Local Flash Assistant</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
           max-width: 640px; margin: 2rem auto; padding: 0 1rem; color: #111; }
    h1 { font-size: 1.5rem; margin-bottom: .25rem; }
    .muted { color: #64748b; font-size: .9rem; }
    .step { margin: 1.5rem 0; }
    .step-title { font-weight: 600; margin-bottom: .5rem; }
    select { padding: .4rem; font-size: 1rem; width: 100%;
             border: 1px solid #cbd5e1; border-radius: .375rem; }
    #drop { border: 2px dashed #cbd5e1; border-radius: .5rem;
            padding: 2rem; text-align: center; color: #64748b;
            transition: background .15s, border-color .15s; cursor: pointer; }
    #drop.hover { background: #f0f9ff; border-color: #38bdf8; }
    #drop.loaded { border-color: #10b981; color: #065f46; background: #ecfdf5; }
    #drop.bad { border-color: #ef4444; color: #991b1b; background: #fef2f2; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    esp-web-install-button {
      --esp-tools-button-color: #f8654b;
      --esp-tools-button-text-color: #ffffff;
      --esp-tools-button-border-radius: .5rem;
    }
    .hidden { display: none; }
  </style>
</head>
<body>
  <h1>TRMNL Local Flash Assistant</h1>
  <p class="muted">Flash a local <span class="mono">merged_firmware.bin</span> over USB.
    Requires Chrome, Edge, Opera, or Arc.</p>

  <div class="step">
    <div class="step-title">1. Select chip</div>
    <select id="chip">
      <option value="ESP32-C3">ESP32-C3 (TRMNL)</option>
      <option value="ESP32-S3">ESP32-S3 (TRMNL_X)</option>
    </select>
  </div>

  <div class="step">
    <div class="step-title">2. Drop merged_firmware.bin</div>
    <div id="drop">
      Drag <span class="mono">merged_firmware.bin</span> here, or click to browse.
    </div>
    <input id="file" type="file" accept=".bin" class="hidden">
  </div>

  <div class="step">
    <div class="step-title">3. Plug in the device in flashing mode, then click Install</div>
    <esp-web-install-button id="installer"></esp-web-install-button>
  </div>

  <script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module"></script>
  <script>
    const drop = document.getElementById('drop');
    const fileInput = document.getElementById('file');
    const chipSel = document.getElementById('chip');
    const installer = document.getElementById('installer');
    let firmwareUrl = null;

    drop.addEventListener('click', () => fileInput.click());

    ['dragenter', 'dragover'].forEach(ev => drop.addEventListener(ev, (e) => {
      e.preventDefault();
      drop.classList.add('hover');
    }));
    ['dragleave', 'drop'].forEach(ev => drop.addEventListener(ev, (e) => {
      e.preventDefault();
      drop.classList.remove('hover');
    }));
    drop.addEventListener('drop', (e) => {
      if (e.dataTransfer.files.length) handleFile(e.dataTransfer.files[0]);
    });
    fileInput.addEventListener('change', (e) => {
      if (e.target.files.length) handleFile(e.target.files[0]);
    });
    chipSel.addEventListener('change', rebuildManifest);

    function handleFile(file) {
      drop.classList.remove('loaded', 'bad');
      if (file.name !== 'merged_firmware.bin') {
        drop.classList.add('bad');
        drop.innerHTML = `Wrong filename: <span class="mono">${file.name}</span>.<br>` +
          `Expected <span class="mono">merged_firmware.bin</span>.`;
        if (firmwareUrl) { URL.revokeObjectURL(firmwareUrl); firmwareUrl = null; }
        installer.removeAttribute('manifest');
        return;
      }
      if (firmwareUrl) URL.revokeObjectURL(firmwareUrl);
      firmwareUrl = URL.createObjectURL(file);
      drop.classList.add('loaded');
      drop.innerHTML = `Loaded <span class="mono">merged_firmware.bin</span> ` +
        `(${(file.size / 1024 / 1024).toFixed(2)} MB). Click to replace.`;
      rebuildManifest();
    }

    function rebuildManifest() {
      if (!firmwareUrl) return;
      const manifest = {
        name: 'TRMNL Local Firmware',
        version: 'local',
        new_install_prompt_erase: false,
        builds: [{
          chipFamily: chipSel.value,
          parts: [{ path: firmwareUrl, offset: '0x0' }],
        }],
      };
      const blob = new Blob([JSON.stringify(manifest)], { type: 'application/json' });
      installer.manifest = URL.createObjectURL(blob);
    }
  </script>
</body>
</html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def log_message(self, fmt, *args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=0,
                        help="Port to bind (default: 0, pick a random free port)")
    parser.add_argument("--no-open", action="store_true",
                        help="Don't auto-open the browser")
    args = parser.parse_args()

    host = "127.0.0.1"
    with socketserver.TCPServer((host, args.port), Handler) as httpd:
        port = httpd.server_address[1]
        url = f"http://{host}:{port}/"
        print(f"Local Flash Assistant: {url}")
        print("Open in Chrome, Edge, Opera, or Arc (Web Serial required).")
        print("Ctrl-C to stop.")
        if not args.no_open:
            try:
                webbrowser.open(url)
            except Exception:
                pass
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
