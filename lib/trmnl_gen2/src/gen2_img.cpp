// Image download + PNG/BMP decode pipeline for TRMNL Gen2 31.5" ACeP display.
// Adapted from TRMNL_V3_C5 img_fetch.cpp.
// PNG decode via PNGdec (bitbank2/PNGdec) — already a dependency of the main firmware.

#include "gen2_img.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PNGdec.h>
#include <string.h>

// Section buffer pointers (public, read by epdWriteImage callers)
const uint8_t *gen2_imgSections[GEN2_IMG_SECTIONS] = { nullptr };
static uint8_t *_secBufs[GEN2_IMG_SECTIONS]        = { nullptr };

// ── 6-color ACeP palette (nearest-neighbour RGB → nibble) ─────────────────
static const struct { uint8_t r, g, b, nib; } kPal[] = {
    {  0,   0,   0, 0x0},  // Black
    {255, 255, 255, 0x1},  // White
    {255, 223,   0, 0x2},  // Yellow
    {196,   2,  51, 0x3},  // Red
    {  0,  86, 163, 0x5},  // Blue
    {  0, 155,  72, 0x6},  // Green
};

static uint8_t toNibble(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t best = UINT32_MAX;
    uint8_t  nib  = 0x1;
    for (auto &c : kPal) {
        int32_t dr = (int32_t)r - c.r, dg = (int32_t)g - c.g, db = (int32_t)b - c.b;
        uint32_t d = (uint32_t)(dr*dr + dg*dg + db*db);
        if (d < best) { best = d; nib = c.nib; }
    }
    return nib;
}

// ── Map display column → (section, local_x) and write nibble ──────────────
static void writePixel(uint32_t dc, uint32_t dr, uint8_t nib)
{
    uint32_t sec, lx;
    if      (dc < 2400u) { sec = dc / 800u;              lx = dc % 800u; }
    else if (dc < 2560u) { sec = 3u;                     lx = dc - 2400u; }
    else if (dc < 4960u) { sec = 4u + (dc-2560u)/800u;  lx = (dc-2560u) % 800u; }
    else if (dc < 5120u) { sec = 7u;                     lx = dc - 4960u; }
    else                 { return; }

    uint32_t byte = (dr * GEN2_IMG_SECTION_W + lx) >> 1u;
    if (lx & 1u)
        _secBufs[sec][byte] = (_secBufs[sec][byte] & 0xF0u) | nib;
    else
        _secBufs[sec][byte] = (uint8_t)((nib << 4u) | (_secBufs[sec][byte] & 0x0Fu));
}

void gen2_imgWritePixel(uint32_t dc, uint32_t dr, uint8_t nib) { writePixel(dc, dr, nib); }

// ── PNGdec context & draw callback ────────────────────────────────────────
struct PngCtx { int srcW, srcH; };

static inline void decodePix(int sx, uint8_t **ps, int iBpp, int iType,
                              const uint8_t *pPal, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t *s = *ps;
    *r = *g = *b = 0;
    switch (iBpp) {
        case 32: *r=s[0]; *g=s[1]; *b=s[2]; *ps+=4; break;
        case 24: *r=s[0]; *g=s[1]; *b=s[2]; *ps+=3; break;
        case 8:
            if (iType == PNG_PIXEL_INDEXED && pPal) { *r=pPal[s[0]*3]; *g=pPal[s[0]*3+1]; *b=pPal[s[0]*3+2]; }
            else { *r=*g=*b=s[0]; }
            *ps+=1; break;
        case 4: {
            uint8_t idx = (sx&1) ? (s[0]&0x0F) : (s[0]>>4);
            if (iType == PNG_PIXEL_INDEXED && pPal) { *r=pPal[idx*3]; *g=pPal[idx*3+1]; *b=pPal[idx*3+2]; }
            else { *r=*g=*b=(uint8_t)(idx|(idx<<4)); }
            if (sx&1) *ps+=1; break;
        }
        case 2: {
            uint8_t idx = (s[0]>>(6-((sx&3)*2)))&3;
            if (iType == PNG_PIXEL_INDEXED && pPal) { *r=pPal[idx*3]; *g=pPal[idx*3+1]; *b=pPal[idx*3+2]; }
            else { *r=*g=*b=(uint8_t)(idx*85); }
            if ((sx&3)==3) *ps+=1; break;
        }
        case 1: {
            uint8_t bit = (s[0]>>(7-(sx&7)))&1;
            if (iType == PNG_PIXEL_INDEXED && pPal) { *r=pPal[bit*3]; *g=pPal[bit*3+1]; *b=pPal[bit*3+2]; }
            else { *r=*g=*b=(uint8_t)(bit?255:0); }
            if ((sx&7)==7) *ps+=1; break;
        }
    }
}

static int pngDrawCb(PNGDRAW *pDraw)
{
    PngCtx *ctx = (PngCtx *)pDraw->pUser;
    // Nearest-neighbour Y downscale to GEN2_IMG_SECTION_H rows
    uint32_t dr = (uint32_t)((uint64_t)pDraw->y * GEN2_IMG_SECTION_H / (uint32_t)ctx->srcH);
    if (pDraw->y > 0) {
        uint32_t prev = (uint32_t)((uint64_t)(pDraw->y-1) * GEN2_IMG_SECTION_H / (uint32_t)ctx->srcH);
        if (prev == dr) return 1;
    }
    if (dr >= GEN2_IMG_SECTION_H) return 1;

    uint8_t *s   = pDraw->pPixels;
    int iBpp     = pDraw->iBpp;
    int iType    = pDraw->iPixelType;
    const uint8_t *pPal = pDraw->pPalette;
    if (iType == PNG_PIXEL_TRUECOLOR       && iBpp <= 8) iBpp *= 3;
    if (iType == PNG_PIXEL_TRUECOLOR_ALPHA && iBpp <= 8) iBpp *= 4;

    int width = pDraw->iWidth < ctx->srcW ? pDraw->iWidth : ctx->srcW;
    for (int sx = 0; sx < width; sx++) {
        uint8_t r, g, b;
        decodePix(sx, &s, iBpp, iType, pPal, &r, &g, &b);
        uint8_t nib  = toNibble(r, g, b);
        uint32_t dc0 = (uint32_t)((uint64_t)sx     * GEN2_DISPLAY_VISIBLE_W / (uint32_t)ctx->srcW);
        uint32_t dc1 = (uint32_t)((uint64_t)(sx+1) * GEN2_DISPLAY_VISIBLE_W / (uint32_t)ctx->srcW);
        if (dc1 <= dc0) dc1 = dc0 + 1;
        for (uint32_t dc = dc0; dc < dc1 && dc < GEN2_DISPLAY_VISIBLE_W; dc++)
            writePixel(dc, dr, nib);
    }
    return 1;
}

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

// ── Allocate / free section buffers ───────────────────────────────────────
static bool allocSections(bool fillWhite)
{
    uint8_t fill = fillWhite ? 0x11 : 0x00;
    for (int i = 0; i < GEN2_IMG_SECTIONS; i++) {
        _secBufs[i] = (uint8_t *)ps_malloc(GEN2_IMG_SECTION_BYTES);
        if (!_secBufs[i]) { gen2_imgFree(); return false; }
        memset(_secBufs[i], fill, GEN2_IMG_SECTION_BYTES);
        gen2_imgSections[i] = _secBufs[i];
    }
    return true;
}

void gen2_imgFree(void)
{
    for (int i = 0; i < GEN2_IMG_SECTIONS; i++) {
        free(_secBufs[i]);
        _secBufs[i]         = nullptr;
        gen2_imgSections[i] = nullptr;
    }
}

bool gen2_imgSolid(uint8_t nib)
{
    gen2_imgFree();
    uint8_t fill = (uint8_t)((nib << 4) | nib);
    for (int i = 0; i < GEN2_IMG_SECTIONS; i++) {
        _secBufs[i] = (uint8_t *)ps_malloc(GEN2_IMG_SECTION_BYTES);
        if (!_secBufs[i]) { gen2_imgFree(); return false; }
        memset(_secBufs[i], fill, GEN2_IMG_SECTION_BYTES);
        gen2_imgSections[i] = _secBufs[i];
    }
    return true;
}

// ── BMP decoder ───────────────────────────────────────────────────────────
static bool decodeBmp(const uint8_t *buf, uint32_t len)
{
    if (len < 54 || buf[0] != 'B' || buf[1] != 'M') return false;

    uint32_t pixOff; memcpy(&pixOff, buf+10, 4);
    int32_t  srcW;   memcpy(&srcW,   buf+18, 4);
    int32_t  srcH;   memcpy(&srcH,   buf+22, 4);
    uint16_t bpp;    memcpy(&bpp,    buf+28, 2);
    uint32_t comp;   memcpy(&comp,   buf+30, 4);

    if (comp != 0) return false;
    if (bpp != 1 && bpp != 8 && bpp != 24 && bpp != 32) return false;

    bool    topDown = (srcH < 0);
    int32_t rows    = topDown ? -srcH : srcH;

    uint8_t pal[256 * 4] = {};
    if (bpp <= 8) {
        uint32_t palBytes = (1u << bpp) * 4u;
        if (54u + palBytes > pixOff) palBytes = pixOff - 54u;
        if (palBytes > 0 && 54u + palBytes <= len) memcpy(pal, buf+54, palBytes);
    }

    uint32_t stride;
    switch (bpp) {
        case 1:  stride = ((uint32_t)srcW + 31u) / 32u * 4u; break;
        case 8:  stride = ((uint32_t)srcW +  3u) /  4u * 4u; break;
        case 24: stride = ((uint32_t)srcW * 3u + 3u) / 4u * 4u; break;
        default: stride = (uint32_t)srcW * 4u; break;
    }

    for (int32_t sy = 0; sy < rows; sy++) {
        int32_t  srcRow = topDown ? sy : (rows - 1 - sy);
        uint32_t rowOff = pixOff + (uint32_t)srcRow * stride;
        if (rowOff + stride > len) break;
        const uint8_t *row = buf + rowOff;

        uint32_t dr = (uint32_t)((uint64_t)sy * GEN2_IMG_SECTION_H / (uint32_t)rows);
        if (sy > 0) {
            uint32_t prev = (uint32_t)((uint64_t)(sy-1) * GEN2_IMG_SECTION_H / (uint32_t)rows);
            if (prev == dr) continue;
        }
        if (dr >= GEN2_IMG_SECTION_H) continue;

        for (int32_t sx = 0; sx < srcW; sx++) {
            uint8_t r=0, g=0, b=0;
            switch (bpp) {
                case 1:  { uint8_t bit=(row[sx>>3]>>(7-(sx&7)))&1u; b=pal[bit*4]; g=pal[bit*4+1]; r=pal[bit*4+2]; break; }
                case 8:  { uint8_t i=row[sx]; b=pal[i*4]; g=pal[i*4+1]; r=pal[i*4+2]; break; }
                case 24: b=row[sx*3]; g=row[sx*3+1]; r=row[sx*3+2]; break;
                case 32: b=row[sx*4]; g=row[sx*4+1]; r=row[sx*4+2]; break;
            }
            uint8_t  nib = toNibble(r, g, b);
            uint32_t dc0 = (uint32_t)((uint64_t)sx     * GEN2_DISPLAY_VISIBLE_W / (uint32_t)srcW);
            uint32_t dc1 = (uint32_t)((uint64_t)(sx+1) * GEN2_DISPLAY_VISIBLE_W / (uint32_t)srcW);
            if (dc1 <= dc0) dc1 = dc0 + 1;
            for (uint32_t dc = dc0; dc < dc1 && dc < GEN2_DISPLAY_VISIBLE_W; dc++)
                writePixel(dc, dr, nib);
        }
    }
    return true;
}

// ── Decode from an in-memory PNG or BMP buffer ────────────────────────────
bool gen2_imgDecodeBuffer(const uint8_t *buf, uint32_t len)
{
    if (!allocSections(true)) return false;

    static const uint8_t kPngMagic[] = { 0x89, 0x50, 0x4E, 0x47 };
    if (len >= 4 && memcmp(buf, kPngMagic, 4) == 0) {
        // Possibly merge multiple IDAT chunks (PNGdec bug workaround)
        const uint8_t *decodeBuf = buf;
        uint32_t       decodeLen = len;
        uint32_t mergedLen = 0;
        uint8_t *merged = pngMergeIdats(buf, len, &mergedLen);
        if (merged) { decodeBuf = merged; decodeLen = mergedLen; }

        PNG *png = new PNG();
        bool ok = false;
        if (png->openRAM((uint8_t *)decodeBuf, (int)decodeLen, pngDrawCb) == PNG_SUCCESS) {
            int srcW = png->getWidth(), srcH = png->getHeight();
            PngCtx ctx = { srcW, srcH };
            int rc = png->decode((void *)&ctx, 0);
            ok = (rc == PNG_SUCCESS || rc == PNG_QUIT_EARLY);
            Serial.printf("[GEN2_IMG] PNG %dx%d decode rc=%d\n", srcW, srcH, rc);
        }
        png->close();
        delete png;
        free(merged);
        return ok;
    } else if (len >= 2 && buf[0] == 'B' && buf[1] == 'M') {
        return decodeBmp(buf, len);
    }
    Serial.println("[GEN2_IMG] Unknown image format");
    return false;
}

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
    if (!allocSections(true)) return false;

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

    bool ok = (received >= 4) && gen2_imgDecodeBuffer(imgBuf, received);
    free(imgBuf);
    if (!ok) Serial.println("[GEN2_IMG] Decode failed — buffers stay white");
    return ok;
}
