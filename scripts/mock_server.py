#!/usr/bin/env python3
"""Local mock TRMNL backend with injectable HTTP failures.

Plays the server side of /api/setup, /api/display, /api/log and the image
fetch so that the firmware's error paths in downloadAndShow() can be
exercised on a real device without touching the real backend.

Failures are injected per route with --display and --image. Each flag
takes a SPEC and can be repeated; specs for a route form a queue that is
consumed in order. Once the queue is empty the route behaves normally.

  SPEC := KIND[=ARG][:COUNT]       COUNT omitted -> fail forever

Usage examples:

  scripts/mock_server.py                       # healthy backend
  scripts/mock_server.py --display 500         # /api/display always 500
  scripts/mock_server.py --display 500:3       # 500 three times, then ok
  scripts/mock_server.py --display 503:2 --display timeout:1
                                               # two 503s, one hang, then ok
  scripts/mock_server.py --image truncate:1    # first image download cut short
  scripts/mock_server.py --image slow=1024,20  # stall 20 s after 1 KB, forever
  scripts/mock_server.py --display status=202  # JSON status 202 (unregistered)

Failure kinds and the firmware error they should provoke:

  both routes
    <code>          any HTTP status, e.g. 500 404 503 429
                    display: HTTPS_RESPONSE_CODE_INVALID (retried 5x)
                    image:   HTTPS_REQUEST_FAILED
    timeout[=SECS]  accept, send nothing for SECS (default 20), close
    reset           TCP RST (SO_LINGER 0)
    close           clean close with zero bytes sent
    redirect[=CODE] 307/308 Location back to the same path; the firmware
                    follows once and then gets the next queued spec

  /api/display only
    bad-json        200 with non-JSON body   -> HTTPS_JSON_PARSING_ERR
    status=N        200 with JSON "status":N
                    202 -> HTTPS_NO_REGISTER (fast 5 s poll)
                    500 -> HTTPS_RESET  ** WIPES THE DEVICE'S CREDENTIALS **
    empty-state     JSON filename "empty_state" (logo screen)

  image only
    truncate[=BYTES] full Content-Length, send BYTES (default half), close
                     -> HTTPS_TIMED_OUT
    slow[=BYTES,SECS] send BYTES then stall SECS (default 1024,20)
                     -> HTTPS_TIMED_OUT (inactivity timeout is 15 s)
    empty           Content-Length: 0        -> HTTPS_WRONG_IMAGE_SIZE
    too-big         Content-Length 100000    -> HTTPS_IMAGE_FILE_TOO_BIG
    garbage         image/png of random bytes -> HTTPS_WRONG_IMAGE_FORMAT
    no-length       chunked, no Content-Length (writeToStream path)
    wrong-type      real BMP sent as image/png (firmware should sniff "BM")

Point the firmware at this server with the captive portal "custom server"
field, or in test/integration/test_config.h:

  #define TEST_BACKEND_URL "http://192.168.1.42:8080"

(use the dev machine's LAN IP, not localhost; the server prints candidates
on startup)
"""
import argparse
import json
import os
import random
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_IMAGE = os.path.join(REPO_ROOT, "test.bmp")

# firmware's non-PSRAM MAX_IMAGE_SIZE is 90000 (include/config.h)
TOO_BIG_LENGTH = 100000

COMMON_KINDS = {"timeout", "reset", "close", "redirect"}
DISPLAY_KINDS = COMMON_KINDS | {"bad-json", "status", "empty-state"}
IMAGE_KINDS = COMMON_KINDS | {"truncate", "slow", "empty", "too-big",
                              "garbage", "no-length", "wrong-type"}


# ---- failure specs ---------------------------------------------------------

class Spec:
    def __init__(self, kind, arg, count):
        self.kind = kind
        self.arg = arg
        self.count = count  # None == forever

    def __str__(self):
        s = self.kind
        if isinstance(self.arg, tuple):
            s += "=" + ",".join(str(a) for a in self.arg)
        elif self.arg is not None:
            s += f"={self.arg}"
        if self.count is None:
            s += " (forever)"
        else:
            s += f" ({self.count} left)"
        return s


def parse_spec(text: str, allowed: set, route: str) -> Spec:
    """SPEC := KIND[=ARG][:COUNT]"""
    body, _, count_str = text.partition(":")
    kind, _, arg = body.partition("=")
    arg = arg if arg != "" else None

    count = None
    if count_str:
        try:
            count = int(count_str)
        except ValueError:
            raise argparse.ArgumentTypeError(f"{text!r}: COUNT must be an integer")
        if count < 1:
            raise argparse.ArgumentTypeError(f"{text!r}: COUNT must be >= 1")

    if kind.isdigit():
        code = int(kind)
        if not 100 <= code <= 599:
            raise argparse.ArgumentTypeError(f"{text!r}: HTTP status must be 100-599")
        return Spec("code", code, count)

    if kind not in allowed:
        raise argparse.ArgumentTypeError(
            f"{text!r}: unknown {route} failure kind {kind!r}. "
            f"Valid: <http-code>, {', '.join(sorted(allowed))}")

    # validate / normalise args
    try:
        if kind == "timeout":
            arg = float(arg) if arg is not None else 20.0
        elif kind == "redirect":
            arg = int(arg) if arg is not None else 307
            if arg not in (307, 308):
                raise ValueError
        elif kind == "status":
            if arg is None:
                raise ValueError
            arg = int(arg)
        elif kind == "truncate":
            arg = int(arg) if arg is not None else None  # None -> half
        elif kind == "slow":
            if arg is None:
                arg = (1024, 20.0)
            else:
                b, s = arg.split(",")
                arg = (int(b), float(s))
        elif arg is not None:
            raise ValueError
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r}: bad argument for {kind}")

    return Spec(kind, arg, count)


class FailureQueue:
    def __init__(self, specs):
        self._specs = list(specs)
        self._lock = threading.Lock()

    def next(self):
        """Return the spec to apply now (consuming one use), or None."""
        with self._lock:
            if not self._specs:
                return None
            spec = self._specs[0]
            if spec.count is not None:
                spec.count -= 1
                if spec.count == 0:
                    self._specs.pop(0)
            return spec

    def remaining(self):
        with self._lock:
            return ", ".join(str(s) for s in self._specs) or "none"


# ---- handler ---------------------------------------------------------------

def _truncate(s: str, max_len: int = 4096) -> str:
    if len(s) <= max_len:
        return s
    return s[:max_len] + f"... <{len(s) - max_len} more bytes truncated>"


class MockHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"  # one request per connection

    # bound in make_handler()
    display_q: FailureQueue
    image_q: FailureQueue
    image_bytes: bytes
    image_type: str   # Content-Type sniffed from the file
    image_ext: str    # bmp / png / jpg
    refresh: int
    setup_status: int
    quiet: bool = False  # skip the per-request header/body dump
    counter_lock = threading.Lock()

    # ---- logging ---------------------------------------------------------

    def _log_request(self, body: bytes):
        peer = self.client_address[0]
        if self.quiet:
            print(f">>> {self.command} {self.path}  from {peer}")
            sys.stdout.flush()
            return
        print("\n" + "=" * 70)
        print(f">>> {self.command} {self.path}  from {peer}")
        for k, v in self.headers.items():
            print(f"    {k}: {v}")
        if body:
            try:
                decoded = body.decode("utf-8")
                print(f"    body ({len(body)} bytes):")
                for line in _truncate(decoded).splitlines() or [""]:
                    print(f"      {line}")
            except UnicodeDecodeError:
                print(f"    body: <{len(body)} non-UTF-8 bytes>")
        sys.stdout.flush()

    def _log(self, msg: str):
        print(f"<<< {msg}")
        sys.stdout.flush()

    def log_message(self, *_args):
        pass

    # ---- helpers ---------------------------------------------------------

    def _read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", "0") or 0)
        return self.rfile.read(length) if length else b""

    def _base_url(self) -> str:
        host = self.headers.get("Host") or f"{self.server.server_address[0]}:{self.server.server_address[1]}"
        return f"http://{host}"

    def _send(self, status: int, body: bytes, content_type="application/json",
              extra_headers=(), content_length=True):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        if content_length:
            self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        for k, v in extra_headers:
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)
        self.wfile.flush()

    def _send_json(self, status: int, obj: dict):
        self._send(status, json.dumps(obj).encode())

    def _hard_reset(self):
        # SO_LINGER with zero timeout makes close() emit RST instead of FIN.
        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                   struct.pack("ii", 1, 0))
        self.close_connection = True
        self.connection.close()

    # ---- shared failure kinds -------------------------------------------

    def _apply_common(self, spec: Spec):
        """Handle kinds valid on both routes.

        Returns a description of what was sent, or None if the kind is not a
        common one and the caller must handle it.
        """
        if spec.kind == "code":
            self._send(spec.arg, f"mock failure {spec.arg}\n".encode(), "text/plain")
            return f"HTTP {spec.arg} (text/plain body)"
        if spec.kind == "timeout":
            self._log(f"holding connection open for {spec.arg} s")
            time.sleep(spec.arg)
            self.close_connection = True
            return f"timeout: sent nothing for {spec.arg} s, then closed"
        if spec.kind == "reset":
            self._hard_reset()
            return "reset: TCP RST, nothing sent"
        if spec.kind == "close":
            self.close_connection = True
            return "close: clean close, nothing sent"
        if spec.kind == "redirect":
            self.send_response(spec.arg)
            self.send_header("Location", self.path)
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.end_headers()
            return f"redirect: HTTP {spec.arg} Location {self.path}"
        return None

    # ---- routes ----------------------------------------------------------

    def _handle_display(self):
        spec = self.display_q.next()
        if spec is not None:
            self._log(f"/api/display -> FAIL {spec}   [queue: {self.display_q.remaining()}]")
            outcome = self._apply_common(spec)
            if outcome is not None:
                self._log(f"/api/display sent {outcome}")
                return
            if spec.kind == "bad-json":
                self._send(200, b"this is not json {{{", "application/json")
            elif spec.kind == "status":
                self._send_json(200, {"status": spec.arg, "refresh_rate": self.refresh})
            elif spec.kind == "empty-state":
                self._send_json(200, self._display_payload(filename="empty_state"))
            return

        payload = self._display_payload()
        self._log(f"/api/display -> 200 {payload['filename']}")
        self._send_json(200, payload)

    def _display_payload(self, filename=None) -> dict:
        # timestamped filename: the firmware uses `filename` as its SPIFFS cache
        # key and skips the download if it already has that file
        name = filename or f"mock-{int(time.time() * 1000)}.{self.image_ext}"
        return {
            "status": 0,
            "image_url": f"{self._base_url()}/img/{name}",
            "filename": name,
            "refresh_rate": self.refresh,
            "update_firmware": False,
            "reset_firmware": False,
            "special_function": "none",
        }

    image_fetches = [0]

    def _handle_image(self):
        """Log every image fetch (who, what, outcome, bytes, elapsed) around _serve_image."""
        with self.counter_lock:
            self.image_fetches[0] += 1
            n = self.image_fetches[0]
        dev = self.headers.get("ID", "?")
        self._log(f"IMAGE fetch #{n}: {self.path} from {self.client_address[0]} (ID {dev})")
        t0 = time.monotonic()
        outcome = "?"
        try:
            outcome = self._serve_image()
        except (BrokenPipeError, ConnectionResetError) as e:
            outcome = f"client dropped: {e}"
            raise
        finally:
            ms = int((time.monotonic() - t0) * 1000)
            self._log(f"IMAGE fetch #{n} done: {outcome} in {ms} ms")

    def _serve_image(self) -> str:
        data = self.image_bytes
        ctype = self.image_type
        spec = self.image_q.next()
        if spec is None:
            self._send(200, data, ctype)
            return f"200 {ctype}, {len(data)} bytes"

        self._log(f"{self.path} -> FAIL {spec}   [queue: {self.image_q.remaining()}]")
        outcome = self._apply_common(spec)
        if outcome is not None:
            return f"FAIL {outcome}"

        kind = spec.kind
        if kind == "truncate":
            n = spec.arg if spec.arg is not None else len(data) // 2
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data[:n])
            self.wfile.flush()
            self.close_connection = True
            return f"truncate: sent {n}/{len(data)} bytes then closed"
        elif kind == "slow":
            n, secs = spec.arg
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data[:n])
            self.wfile.flush()
            self._log(f"sent {n} bytes, stalling {secs} s")
            time.sleep(secs)
            try:
                self.wfile.write(data[n:])
                self.wfile.flush()
                return f"slow: sent {n} bytes, stalled {secs} s, sent remaining {len(data) - n}"
            except (BrokenPipeError, ConnectionResetError):
                return f"slow: sent {n} bytes, client gave up during {secs} s stall"
        elif kind == "empty":
            self._send(200, b"", ctype)
            return "empty: 200 with Content-Length 0"
        elif kind == "too-big":
            self._send(200, b"\x00" * TOO_BIG_LENGTH, ctype)
            return f"too-big: 200 with {TOO_BIG_LENGTH} zero bytes"
        elif kind == "garbage":
            size = max(len(data), 4096)
            self._send(200, random.randbytes(size), "image/png")
            return f"garbage: 200 image/png, {size} random bytes"
        elif kind == "no-length":
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data)
            self.wfile.flush()
            self.close_connection = True
            return f"no-length: 200 {ctype}, {len(data)} bytes without Content-Length"
        elif kind == "wrong-type":
            # lie about the type: a BMP claims PNG (firmware should sniff "BM"),
            # a PNG/JPEG claims BMP (firmware falls into the BMP parser)
            lie = "image/png" if ctype != "image/png" else "image/bmp"
            self._send(200, data, lie)
            return f"wrong-type: 200 {lie} (really {ctype}), {len(data)} bytes"
        return f"FAIL {spec.kind} (unhandled kind, nothing sent)"

    def _handle_setup(self):
        if self.setup_status == 404:
            self._log("/api/setup -> 404 (not registered)")
            self._send_json(200, {"status": 404})
            return
        payload = {
            "status": 200,
            "api_key": "mock-api-key",
            "friendly_id": "MOCK01",
            "image_url": f"{self._base_url()}/img/setup.{self.image_ext}",
            "message": "mock backend",
        }
        self._log("/api/setup -> 200")
        self._send_json(200, payload)

    def _handle_log(self, body: bytes):
        self._log("/api/log -> 204")
        self.send_response(204)
        self.send_header("Connection", "close")
        self.end_headers()

    # ---- dispatch --------------------------------------------------------

    def do_GET(self):
        self._dispatch()

    def do_POST(self):
        self._dispatch()

    def do_HEAD(self):
        self._dispatch()

    def _dispatch(self):
        body = self._read_body()
        self._log_request(body)
        path = self.path.split("?", 1)[0]
        try:
            if path == "/api/display":
                self._handle_display()
            elif path.startswith("/img/"):
                self._handle_image()
            elif path == "/api/setup":
                self._handle_setup()
            elif path == "/api/log":
                self._handle_log(body)
            else:
                self._log(f"{path} -> 404")
                self._send(404, b"not found\n", "text/plain")
        except (BrokenPipeError, ConnectionResetError) as e:
            self._log(f"client dropped connection: {e}")


def sniff_image(data: bytes):
    """Return (content_type, extension) from the file's magic bytes."""
    if data[:2] == b"BM":
        return "image/bmp", "bmp"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png", "png"
    if data[:2] == b"\xff\xd8":
        return "image/jpeg", "jpg"
    return None, None


def make_handler(display_q, image_q, image_bytes, image_type, image_ext, refresh, setup_status,
                 quiet=False):
    return type("BoundMockHandler", (MockHandler,), {
        "quiet": quiet,
        "display_q": display_q,
        "image_q": image_q,
        "image_bytes": image_bytes,
        "image_type": image_type,
        "image_ext": image_ext,
        "refresh": refresh,
        "setup_status": setup_status,
    })


# ---- startup ---------------------------------------------------------------

def discover_local_ips() -> list:
    found = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        primary = s.getsockname()[0]
        s.close()
        if primary and not primary.startswith("127."):
            found.append(primary)
    except OSError:
        pass
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ip = info[4][0]
            if ip and not ip.startswith("127.") and ip not in found:
                found.append(ip)
    except OSError:
        pass
    return found


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="Interface to bind (default 0.0.0.0 so the device can reach it)")
    ap.add_argument("--display", metavar="SPEC", action="append", default=[],
                    type=lambda s: parse_spec(s, DISPLAY_KINDS, "display"),
                    help="failure to inject on GET /api/display (repeatable)")
    ap.add_argument("--image", metavar="SPEC", action="append", default=[],
                    type=lambda s: parse_spec(s, IMAGE_KINDS, "image"),
                    help="failure to inject on the image fetch (repeatable)")
    ap.add_argument("--image-file", default=DEFAULT_IMAGE,
                    help=f"image to serve (default {os.path.relpath(DEFAULT_IMAGE, REPO_ROOT)})")
    ap.add_argument("--refresh", type=int, default=60,
                    help="refresh_rate (seconds) returned to the device (default 60)")
    ap.add_argument("--setup-status", type=int, choices=(200, 404), default=200,
                    help="JSON status returned by /api/setup (default 200)")
    ap.add_argument("-q", "--quiet", action="store_true",
                    help="one line per request instead of dumping every header and body")
    args = ap.parse_args()

    try:
        with open(args.image_file, "rb") as f:
            image_bytes = f.read()
    except OSError as e:
        ap.error(f"cannot read --image-file: {e}")

    image_type, image_ext = sniff_image(image_bytes)
    if image_type is None:
        ap.error(f"--image-file {args.image_file}: not a BMP, PNG or JPEG")

    display_q = FailureQueue(args.display)
    image_q = FailureQueue(args.image)
    handler = make_handler(display_q, image_q, image_bytes, image_type, image_ext,
                           args.refresh, args.setup_status, quiet=args.quiet)

    server = ThreadingHTTPServer((args.bind, args.port), handler)
    server.daemon_threads = True

    print(f"mock TRMNL backend listening on http://{args.bind}:{args.port}")
    print(f"  image:    {args.image_file} ({len(image_bytes)} bytes, {image_type})")
    print(f"  refresh:  {args.refresh} s")
    print(f"  display failures: {display_q.remaining()}")
    print(f"  image failures:   {image_q.remaining()}")
    if any(s.kind == "status" and s.arg == 500 for s in args.display):
        print("  WARNING: status=500 makes the firmware erase its stored credentials")

    ips = discover_local_ips()
    if ips:
        print("\nPoint the device at one of these (captive portal custom server URL, "
              "or TEST_BACKEND_URL in test/integration/test_config.h):\n")
        for ip in ips:
            print(f"    http://{ip}:{args.port}")
            print(f'    #define TEST_BACKEND_URL "http://{ip}:{args.port}"')
        print()
    else:
        print(f"(couldn't auto-detect a LAN IP; use http://<dev-machine-LAN-IP>:{args.port})")
    sys.stdout.flush()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
        server.server_close()


if __name__ == "__main__":
    main()
