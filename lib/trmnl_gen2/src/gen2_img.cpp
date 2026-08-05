// Image download + PNG/BMP decode pipeline for TRMNL Gen2 31.5" ACeP display.
// PNG decode via PNGdec (bitbank2/PNGdec) — already a dependency of the main firmware.

#include "gen2_img.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PNGdec.h>
#include <string.h>

#ifdef FUTURE
// ── PNG IDAT merger (workaround for PNGdec multi-IDAT bug) ────────────────
static uint8_t *pngMergeIdats(const uint8_t *src, uint32_t srcLen, uint32_t *outLen)
{
    if (srcLen < 8) return nullptr;
    int idatCount = 0;
    uint32_t totalIdat = 0, pos = 8;
    while (pos + 8 <= srcLen) {
        uint32_t clen = ((uint32_t)src[pos]<<24)|((uint32_t)src[pos+1]<<16)|
                        ((uint32_t)src[pos+2]<<8)|src[pos+3];
        if (memcmp(src+pos+4, "IDAT", 4) == 0) { idatCount++; totalIdat += clen; }
        if (memcmp(src+pos+4, "IEND", 4) == 0) break;
        pos += 8 + clen + 4;
    }
    if (idatCount <= 1) return nullptr;

    uint8_t *dst = (uint8_t *)ps_malloc(srcLen);
    if (!dst) return nullptr;

    uint32_t d = 0;
    memcpy(dst, src, 8); d = 8;
    bool written = false;
    pos = 8;
    while (pos + 8 <= srcLen) {
        uint32_t clen = ((uint32_t)src[pos]<<24)|((uint32_t)src[pos+1]<<16)|
                        ((uint32_t)src[pos+2]<<8)|src[pos+3];
        bool isIdat = memcmp(src+pos+4, "IDAT", 4) == 0;
        bool isIend = memcmp(src+pos+4, "IEND", 4) == 0;
        if (isIend) { memcpy(dst+d, src+pos, 12); d += 12; break; }
        if (isIdat) {
            if (!written) {
                dst[d]=(uint8_t)(totalIdat>>24); dst[d+1]=(uint8_t)(totalIdat>>16);
                dst[d+2]=(uint8_t)(totalIdat>>8); dst[d+3]=(uint8_t)totalIdat;
                dst[d+4]='I'; dst[d+5]='D'; dst[d+6]='A'; dst[d+7]='T'; d += 8;
                uint32_t p2 = 8;
                while (p2 + 8 <= srcLen) {
                    uint32_t cl2 = ((uint32_t)src[p2]<<24)|((uint32_t)src[p2+1]<<16)|
                                   ((uint32_t)src[p2+2]<<8)|src[p2+3];
                    if (memcmp(src+p2+4, "IDAT", 4) == 0) { memcpy(dst+d, src+p2+8, cl2); d += cl2; }
                    if (memcmp(src+p2+4, "IEND", 4) == 0) break;
                    p2 += 8 + cl2 + 4;
                }
                dst[d]=0; dst[d+1]=0; dst[d+2]=0; dst[d+3]=0; d += 4;
                written = true;
            }
            pos += 8 + clen + 4;
            continue;
        }
        if (d + 8 + clen + 4 <= srcLen) { memcpy(dst+d, src+pos, 8+clen+4); d += 8+clen+4; }
        pos += 8 + clen + 4;
    }
    *outLen = d;
    return dst;
}

#endif // FUTURE

// ── HTTP download helpers ──────────────────────────────────────────────────
static uint32_t downloadToBuffer(WiFiClient *stream, int contentLen,
                                  uint8_t *buf, uint32_t maxLen)
{
    uint32_t toRead = (contentLen > 0 && (uint32_t)contentLen < maxLen)
                      ? (uint32_t)contentLen : maxLen;
    uint32_t received = 0;
    unsigned long tLast = millis();

    while (received < toRead) {
        int avail = stream->available();
        if (avail > 0) {
            int n = stream->readBytes(buf + received,
                                      min(avail, (int)(toRead - received)));
            if (n > 0) { received += (uint32_t)n; tLast = millis(); }
        } else if (!stream->connected()) {
            break;
        } else if (millis() - tLast > 20000UL) {
            Serial.println("[GEN2_IMG] Download stall timeout");
            break;
        }
        delay(1);
    }
    return received;
}

bool gen2_imgFetchUrl(const char *url)
{
    bool isHttps = (strncmp(url, "https", 5) == 0);
    WiFiClientSecure *secClient = nullptr;
    WiFiClient       *client    = nullptr;
    if (isHttps) {
        secClient = new WiFiClientSecure();
        secClient->setInsecure();
        client = secClient;
    } else {
        client = new WiFiClient();
    }

    HTTPClient *http = new HTTPClient();
    if (!http->begin(*client, url)) {
        Serial.println("[GEN2_IMG] http.begin() failed");
        delete http; delete client; return false;
    }
    http->setReuse(false);
    http->setTimeout(30000);
    http->setConnectTimeout(15000);
    http->addHeader("Accept-Encoding", "identity");

    Serial.printf("[GEN2_IMG] GET %s\n", url);
    int code = http->GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[GEN2_IMG] HTTP error %d\n", code);
        http->end(); delete http; delete client; return false;
    }

    int contentLen = http->getSize();
    const uint32_t kMaxBuf = 1536u * 1024u;
    uint32_t allocLen = (contentLen > 0 && (uint32_t)contentLen < kMaxBuf)
                        ? (uint32_t)contentLen : kMaxBuf;
    uint8_t *imgBuf = (uint8_t *)ps_malloc(allocLen);
    if (!imgBuf) {
        http->end(); delete http; delete client; return false;
    }

    WiFiClient *stream = http->getStreamPtr();
    uint32_t received = downloadToBuffer(stream, contentLen, imgBuf, allocLen);
    stream->stop();
    http->end();
    delete http; delete client;

    bool ok = (received >= 4) && gen2_display_image(imgBuf, received);
    free(imgBuf);
    if (!ok) Serial.println("[GEN2_IMG] Decode failed — buffers stay white");
    return ok;
}
