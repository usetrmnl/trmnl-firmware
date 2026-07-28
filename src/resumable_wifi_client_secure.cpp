#include "resumable_wifi_client_secure.h"

#include <WiFi.h>
#include <errno.h>
#include <esp32-hal-log.h>
#include <esp_arduino_version.h>
#include <lwip/sockets.h>
#include <mbedtls/error.h>
#include <mbedtls/ssl.h>

#include "Arduino.h"

#undef connect
#undef write
#undef read

// One session, in RTC so it survives deep sleep. The magic gates out the garbage
// RTC holds after a cold POWERON (RTC_NOINIT is only meaningful across a deep-sleep
// wake, not a power cycle).
#define RWCS_SESS_MAGIC 0x52435332u
#define RWCS_SESS_MAX   1536
RTC_NOINIT_ATTR static uint8_t s_sess[RWCS_SESS_MAX];
RTC_NOINIT_ATTR static uint32_t s_sess_len;
RTC_NOINIT_ATTR static uint32_t s_sess_magic;

static const char *rwcs_pers = "esp32-tls";

static int rwcs_err(int ret, int line) {
  char buf[100];
  mbedtls_strerror(ret, buf, sizeof(buf));
  log_e("rwcs_start[%d]: (%d) %s", line, ret, buf);
  return ret;
}
#define RWCS_ERR(e) rwcs_err(e, __LINE__)

// Reduced fork of the framework's start_ssl_client(): the insecure path trmnl runs,
// plus TLS session resume. The socket stays non-blocking so the inherited
// WiFiClientSecure read path drives the data phase. Returns the socket fd (>=0) or
// a negative error.
static int rwcs_start(sslclient_context *ssl_client, const IPAddress &ip, uint32_t port, const char *hostname,
                      int timeout, const char **alpn_protos, bool try_resume) {
  int ret;
  int enable = 1;

  ssl_client->socket = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (ssl_client->socket < 0) {
    log_e("ERROR opening socket");
    return ssl_client->socket;
  }

  fcntl(ssl_client->socket, F_SETFL, fcntl(ssl_client->socket, F_GETFL, 0) | O_NONBLOCK);
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = ip;
  serv_addr.sin_port = htons(port);

  if (timeout <= 0) {
    timeout = 30000;
  }
  ssl_client->socket_timeout = timeout;

  fd_set fdset;
  struct timeval tv;
  FD_ZERO(&fdset);
  FD_SET(ssl_client->socket, &fdset);
  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;

  int res = lwip_connect(ssl_client->socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  if (res < 0 && errno != EINPROGRESS) {
    log_e("connect on fd %d, errno: %d", ssl_client->socket, errno);
    lwip_close(ssl_client->socket);
    ssl_client->socket = -1;
    return -1;
  }

  // O_NONBLOCK means connect() returns immediately; select() applies the timeout.
  res = select(ssl_client->socket + 1, nullptr, &fdset, nullptr, &tv);
  int sockerr = 0;
  socklen_t len = sizeof(sockerr);
  if (res <= 0 || getsockopt(ssl_client->socket, SOL_SOCKET, SO_ERROR, &sockerr, &len) < 0 || sockerr != 0) {
    log_e("connect failed on fd %d (select %d, sockerr %d)", ssl_client->socket, res, sockerr);
    lwip_close(ssl_client->socket);
    ssl_client->socket = -1;
    return -1;
  }

  lwip_setsockopt(ssl_client->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  lwip_setsockopt(ssl_client->socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  lwip_setsockopt(ssl_client->socket, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
  lwip_setsockopt(ssl_client->socket, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));

  mbedtls_entropy_init(&ssl_client->entropy_ctx);
  ret = mbedtls_ctr_drbg_seed(&ssl_client->drbg_ctx, mbedtls_entropy_func, &ssl_client->entropy_ctx,
                              (const unsigned char *)rwcs_pers, strlen(rwcs_pers));
  if (ret < 0) return RWCS_ERR(ret);

  if ((ret = mbedtls_ssl_config_defaults(&ssl_client->ssl_conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM,
                                         MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
    return RWCS_ERR(ret);

  if (alpn_protos != NULL && (ret = mbedtls_ssl_conf_alpn_protocols(&ssl_client->ssl_conf, alpn_protos)) != 0)
    return RWCS_ERR(ret);

  mbedtls_ssl_conf_authmode(&ssl_client->ssl_conf, MBEDTLS_SSL_VERIFY_NONE);

  if ((ret = mbedtls_ssl_set_hostname(&ssl_client->ssl_ctx, hostname != NULL ? hostname : ip.toString().c_str())) != 0)
    return RWCS_ERR(ret);

  mbedtls_ssl_conf_rng(&ssl_client->ssl_conf, mbedtls_ctr_drbg_random, &ssl_client->drbg_ctx);

  if ((ret = mbedtls_ssl_setup(&ssl_client->ssl_ctx, &ssl_client->ssl_conf)) != 0) return RWCS_ERR(ret);

  mbedtls_ssl_set_bio(&ssl_client->ssl_ctx, &ssl_client->socket, mbedtls_net_send, mbedtls_net_recv, NULL);

  // Resume: mbedtls_ssl_set_session() deep-copies, so the local copy is freed at once.
  // A ticket the server declines degrades to a full handshake on its own — nothing to
  // handle here; the renewed session is captured below either way.
  if (try_resume) {
    mbedtls_ssl_session saved;
    mbedtls_ssl_session_init(&saved);
    if (mbedtls_ssl_session_load(&saved, s_sess, s_sess_len) == 0)
      mbedtls_ssl_set_session(&ssl_client->ssl_ctx, &saved);
    mbedtls_ssl_session_free(&saved);
  }

  unsigned long handshake_start = millis();
  while ((ret = mbedtls_ssl_handshake(&ssl_client->ssl_ctx)) != 0) {
    if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) return RWCS_ERR(ret);
    if ((millis() - handshake_start) > ssl_client->handshake_timeout) return -1;
    vTaskDelay(2);
  }

  mbedtls_ssl_session got;
  mbedtls_ssl_session_init(&got);
  if (mbedtls_ssl_get_session(&ssl_client->ssl_ctx, &got) == 0) {
    size_t olen = 0;
    if (mbedtls_ssl_session_save(&got, s_sess, sizeof(s_sess), &olen) == 0) {
      s_sess_len = (uint32_t)olen;
      s_sess_magic = RWCS_SESS_MAGIC;
    }
  }
  mbedtls_ssl_session_free(&got);

  return ssl_client->socket;
}

int ResumableWiFiClientSecure::connect(IPAddress ip, uint16_t port) { return connectResumable(ip, port, nullptr); }

int ResumableWiFiClientSecure::connect(const char *host, uint16_t port) {
  IPAddress address;
  if (!WiFi.hostByName(host, address)) return 0;
  return connectResumable(address, port, host);
}

int ResumableWiFiClientSecure::connectResumable(IPAddress ip, uint16_t port, const char *host) {
  bool try_resume = (s_sess_magic == RWCS_SESS_MAGIC && s_sess_len > 0 && s_sess_len <= RWCS_SESS_MAX);

  // &* bridges both cores: sslclient is a raw pointer on core 2.x, a shared_ptr on 3.x.
  int sock = rwcs_start(&*sslclient, ip, port, host, _timeout, _alpn_protos, try_resume);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  sslclient->last_error = sock;
#else
  _lastError = sock;
#endif

  if (sock < 0) {
    s_sess_magic = 0; // drop the saved session so a genuine failure can't loop on a bad ticket
    stop();
    return 0;
  }
  _connected = true;
  return 1;
}