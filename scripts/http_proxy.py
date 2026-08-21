#!/usr/bin/env python3
"""Logging HTTP-to-HTTPS proxy for on-device integration tests.

Receives plain HTTP requests from the firmware, prints every detail to
stdout (method, path, all headers including HTTPClient implicit ones,
request body), then forwards the request to the real backend over HTTPS
and returns the response.

Use this to see *exactly* what the firmware put on the wire — including
headers HTTPClient adds for you under the hood (Content-Type,
User-Agent, Connection, etc.) — without changing what backend it talks
to. The firmware's WiFiClient (plain HTTP) is fine here because the
proxy upgrades to TLS for the upstream call.

Usage:
  scripts/http_proxy.py
  scripts/http_proxy.py --port 8888 --upstream https://trmnl.app
  scripts/http_proxy.py --upstream https://httpbin.org

Point the firmware at this proxy by overriding the API URL — e.g. in
test_config.h:

  #define TEST_BACKEND_URL "http://192.168.1.42:8888"

(use the dev machine's LAN IP — not localhost — since the firmware is
on a separate host)
"""
import argparse
import http.client
import socket
import ssl
import sys
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


UPSTREAM = "https://trmnl.app"   # overridable via --upstream


def _truncate(s: str, max_len: int = 4096) -> str:
    if len(s) <= max_len:
        return s
    return s[:max_len] + f"... <{len(s) - max_len} more bytes truncated>"


class ProxyHandler(BaseHTTPRequestHandler):
    upstream: str = UPSTREAM

    # ---- logging ---------------------------------------------------------

    def _log_request(self, body: bytes):
        peer = self.client_address[0]
        bar = "=" * 70
        print(f"\n{bar}")
        print(f">>> {self.command} {self.path}  from {peer}")
        # self.headers is an email.message.Message backed by an ordered list,
        # so .items() yields every header occurrence — duplicates included.
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

    def _log_response(self, status: int, headers: list, body: bytes):
        print(f"<<< {status}  ({len(body)} bytes)")
        for k, v in headers:
            print(f"    {k}: {v}")
        sys.stdout.flush()

    # ---- forwarding ------------------------------------------------------

    def _forward(self, body: bytes):
        parsed = urllib.parse.urlparse(self.upstream)
        is_https = parsed.scheme == "https"
        port = parsed.port or (443 if is_https else 80)

        if is_https:
            # Cert verification deliberately disabled — this proxy is a test
            # instrument, not a production HTTPS client. Avoids the macOS
            # "unable to get local issuer certificate" error without making
            # the user install certifi or run "Install Certificates.command".
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            conn = http.client.HTTPSConnection(parsed.hostname, port, context=ctx, timeout=20)
        else:
            conn = http.client.HTTPConnection(parsed.hostname, port, timeout=20)

        # Build outbound headers as a list of tuples (not a dict) so duplicate
        # header names from the firmware survive end-to-end instead of being
        # silently collapsed.
        # - skip Host + Content-Length here and re-set them below ourselves.
        #   NOTE: the low-level putrequest/putheader/endheaders API does NOT
        #   auto-add Content-Length the way the high-level conn.request() does
        #   (that magic lives in _send_request, which we bypass). If we drop
        #   the incoming Content-Length and don't re-add it, the upstream gets
        #   no body framing and reads a zero-length body -> the backend sees an
        #   empty request (e.g. /api/log stores `{}`). So recompute it from the
        #   actual bytes we're forwarding.
        # - force Accept-Encoding: identity so the upstream sends plain bytes
        #   (gzipped responses would confuse the firmware's JSON parser)
        skip = {"host", "content-length", "accept-encoding"}
        outbound_headers = [(k, v) for k, v in self.headers.items() if k.lower() not in skip]
        outbound_headers.append(("Host", parsed.hostname))
        outbound_headers.append(("Accept-Encoding", "identity"))
        outbound_headers.append(("Content-Length", str(len(body))))

        try:
            # Use the low-level API instead of conn.request(...): the high-level
            # call iterates `headers` as if it were a dict (it does
            # `for k in headers: k.lower()`), so passing a list of tuples
            # crashes with "'tuple' object has no attribute 'lower'", and a
            # dict would collapse duplicates. putheader() in a loop puts each
            # occurrence on the wire verbatim.
            conn.putrequest(self.command, self.path,
                            skip_host=True, skip_accept_encoding=True)
            for k, v in outbound_headers:
                conn.putheader(k, v)
            conn.endheaders(body)
            resp = conn.getresponse()
            resp_body = resp.read()
            resp_headers = resp.getheaders()
            self._log_response(resp.status, resp_headers, resp_body)

            self.send_response(resp.status)
            # Strip headers that would confuse the client given we may have
            # rewritten body framing. Recompute Content-Length from the
            # actual bytes we'll write.
            drop = {"transfer-encoding", "connection", "content-length", "content-encoding"}
            for k, v in resp_headers:
                if k.lower() in drop:
                    continue
                self.send_header(k, v)
            self.send_header("Content-Length", str(len(resp_body)))
            self.end_headers()
            self.wfile.write(resp_body)
        except Exception as e:
            print(f"!!! upstream request failed: {e}")
            self.send_error(502, f"upstream failed: {e}")
        finally:
            conn.close()

    # ---- HTTP methods ----------------------------------------------------

    def _read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", "0") or 0)
        return self.rfile.read(length) if length else b""

    def do_GET(self):    self._handle()
    def do_POST(self):   self._handle()
    def do_PUT(self):    self._handle()
    def do_DELETE(self): self._handle()
    def do_PATCH(self):  self._handle()
    def do_HEAD(self):   self._handle()

    def _handle(self):
        body = self._read_body()
        self._log_request(body)
        self._forward(body)

    # Silence the default access-log noise (we already print our own).
    def log_message(self, *_args):
        pass


def make_handler(upstream: str):
    return type("BoundHandler", (ProxyHandler,), {"upstream": upstream})


def discover_local_ips() -> list:
    """Return a sorted list of plausible LAN IPv4 addresses on this host,
    with the most likely outbound-default one first. Excludes loopback.
    """
    found = []
    # The reliable trick: open a UDP socket "to" a public address; the
    # kernel picks the outbound interface and we read its IP. No packets
    # actually leave the machine.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        primary = s.getsockname()[0]
        s.close()
        if primary and not primary.startswith("127."):
            found.append(primary)
    except OSError:
        pass

    # Also scan all interfaces via getaddrinfo on our hostname — picks up
    # multiple LAN addresses (wifi + ethernet + tailscale + etc).
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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="Interface to bind. Default 0.0.0.0 so the device on the LAN can reach it.")
    ap.add_argument("--upstream", default=UPSTREAM,
                    help=f"Upstream backend URL (default: {UPSTREAM})")
    args = ap.parse_args()

    server = ThreadingHTTPServer((args.bind, args.port), make_handler(args.upstream))
    print(f"logging proxy listening on http://{args.bind}:{args.port}  "
          f"-> {args.upstream}")

    ips = discover_local_ips()
    if ips:
        print()
        print("Paste ONE of these into test/integration/test_config.h "
              "(pick the LAN your device is on):")
        print()
        for ip in ips:
            print(f'    #define TEST_BACKEND_URL "http://{ip}:{args.port}"')
        print()
    else:
        print("(couldn't auto-detect a LAN IP — point firmware at "
              f"http://<dev-machine-LAN-IP>:{args.port})")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
        server.server_close()


if __name__ == "__main__":
    main()
