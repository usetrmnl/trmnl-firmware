#pragma once

#include <WiFiClientSecure.h>

// WiFiClientSecure that resumes the TLS session across deep sleep, skipping the
// ~1.7 s software ECDHE handshake each wake (the C3 has no ECC hardware). Stock
// WiFiClientSecure exposes no session resumption, so connect() runs a reduced fork
// of start_ssl_client() that adds mbedtls_ssl_set_session/get_session and keeps the
// session in RTC; read/write/stop are inherited unchanged.
//
// trmnl.app only: same host, insecure (VERIFY_NONE), 7-day tickets. NOT for S3 —
// different host, so a saved session is useless there.
class ResumableWiFiClientSecure : public WiFiClientSecure {
public:
  int connect(IPAddress ip, uint16_t port) override;
  int connect(const char *host, uint16_t port) override;

private:
  int connectResumable(IPAddress ip, uint16_t port, const char *host);
};