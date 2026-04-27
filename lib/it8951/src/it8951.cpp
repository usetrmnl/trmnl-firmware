#include "it8951.h"
#include <cstring>
#include <trmnl_log.h>

static const char *TAG = "IT8951";

// ─── Low-level SPI helpers ──────────────────────────────────────────────────

void IT8951::spiSendWord(uint16_t word) {
    SPI.transfer16(word);
}

uint16_t IT8951::spiRecvWord() {
    return SPI.transfer16(0);
}

void IT8951::lcdWaitForReady() {
    const uint32_t start = millis();
    while (digitalRead(pins_.busy) == LOW) {
        if (millis() - start > 3000) {
            // Serial-only — HRDY timeouts are expected during the multi-attempt probe sequence
            Log_info_serial("%s: HRDY timeout", TAG);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void IT8951::lcdWriteCmdCode(uint16_t cmd) {
    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x6000); // command preamble
    lcdWaitForReady();
    spiSendWord(cmd);
    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
}

void IT8951::lcdWriteData(uint16_t data) {
    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x0000); // data preamble
    lcdWaitForReady();
    spiSendWord(data);
    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
}

void IT8951::lcdWriteNData(const uint16_t *buf, uint32_t word_count) {
    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x0000); // data preamble
    lcdWaitForReady();
    for (uint32_t i = 0; i < word_count; i++) {
        spiSendWord(buf[i]);
    }
    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
}

uint16_t IT8951::lcdReadData() {
    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x1000); // read preamble
    spiRecvWord();       // dummy read
    lcdWaitForReady();
    const uint16_t data = spiRecvWord();
    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
    return data;
}

void IT8951::lcdReadNData(uint16_t *buf, uint32_t word_count) {
    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x1000); // read preamble
    lcdWaitForReady();
    spiRecvWord();       // dummy read
    lcdWaitForReady();
    for (uint32_t i = 0; i < word_count; i++) {
        buf[i] = spiRecvWord();
    }
    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
}

void IT8951::lcdSendCmdArg(uint16_t cmd, uint16_t *args, uint16_t num_args) {
    lcdWriteCmdCode(cmd);
    for (uint16_t i = 0; i < num_args; i++) {
        lcdWriteData(args[i]);
    }
}

// ─── Hardware control ───────────────────────────────────────────────────────

void IT8951::hardwareReset() {
    digitalWrite(pins_.cs, HIGH);
    digitalWrite(pins_.rst, HIGH);
    digitalWrite(pins_.en, HIGH);
    digitalWrite(pins_.ite_en, HIGH);
    delay(50);
    digitalWrite(pins_.rst, LOW);
    delay(10);
    digitalWrite(pins_.rst, HIGH);
    delay(10);
}

void IT8951::powerCycle() {
    digitalWrite(pins_.cs, HIGH);
    digitalWrite(pins_.rst, HIGH);
    digitalWrite(pins_.en, LOW);
    digitalWrite(pins_.ite_en, LOW);
    delay(100);
    digitalWrite(pins_.en, HIGH);
    digitalWrite(pins_.ite_en, HIGH);
    delay(500);
    hardwareReset();
    delay(1500);
}

// ─── Register access ────────────────────────────────────────────────────────

uint16_t IT8951::readReg(uint16_t addr) {
    lcdWriteCmdCode(IT8951_TCON_REG_RD);
    lcdWriteData(addr);
    return lcdReadData();
}

void IT8951::writeReg(uint16_t addr, uint16_t val) {
    lcdWriteCmdCode(IT8951_TCON_REG_WR);
    lcdWriteData(addr);
    lcdWriteData(val);
}

// ─── IT8951 commands ────────────────────────────────────────────────────────

void IT8951::getSystemInfo() {
    memset(&dev_info_, 0, sizeof(dev_info_));
    lcdWriteCmdCode(USDEF_I80_CMD_GET_DEV_INFO);
    lcdReadNData(reinterpret_cast<uint16_t *>(&dev_info_), sizeof(IT8951DevInfo) / 2);
}

void IT8951::setImgBufBaseAddr(uint32_t addr) {
    const uint16_t hi = (addr >> 16) & 0xFFFF;
    const uint16_t lo = addr & 0xFFFF;
    writeReg(IT8951_REG_LISAR + 2, hi);
    writeReg(IT8951_REG_LISAR, lo);
}

void IT8951::loadImgAreaStart(uint16_t endian, uint16_t pix_fmt, uint16_t rotate,
                              uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint16_t args[5];
    args[0] = (endian << 8) | (pix_fmt << 4) | rotate;
    args[1] = x;
    args[2] = y;
    args[3] = w;
    args[4] = h;
    lcdSendCmdArg(IT8951_TCON_LD_IMG_AREA, args, 5);
}

void IT8951::writeVcom(uint16_t selector, uint16_t value) {
    lcdWriteCmdCode(USDEF_I80_CMD_VCOM);
    lcdWriteData(selector);
    lcdWriteData(value);
}

bool IT8951::hasValidDevInfo() const {
    return dev_info_.panel_width > 0 && dev_info_.panel_width < 10000 &&
           dev_info_.panel_height > 0 && dev_info_.panel_height < 10000;
}

bool IT8951::probeController(const char *label, bool send_sys_run, int vcom_selector) {
    memset(&dev_info_, 0, sizeof(dev_info_));

    if (send_sys_run) {
        Log_info("%s: [%s] Sending SYS_RUN wake command", TAG, label);
        lcdWriteCmdCode(IT8951_TCON_SYS_RUN);
        delay(10);
    }

    if (vcom_selector > 0) {
        Log_info("%s: [%s] Writing VCOM=%u with selector 0x%04X", TAG, label, vcom_, vcom_selector);
        writeVcom(vcom_selector, vcom_);
        lcdWriteCmdCode(USDEF_I80_CMD_VCOM);
        lcdWriteData(0x0000);
        uint16_t readback = lcdReadData();
        Log_info("%s: [%s] VCOM read-back: %u (0x%04X)", TAG, label, readback, readback);
        if (readback == vcom_) {
            vcom_write_selector_ = vcom_selector;
        }
    }

    getSystemInfo();
    Log_info("%s: [%s] DevInfo: W=%u H=%u BufL=0x%04X BufH=0x%04X",
             TAG, label, dev_info_.panel_width, dev_info_.panel_height,
             dev_info_.img_buf_addr_l, dev_info_.img_buf_addr_h);
    return hasValidDevInfo();
}

// ─── Bulk framebuffer upload ────────────────────────────────────────────────

void IT8951::writeFramebufferRows4bpp(const uint16_t *buf, uint16_t width_in_words, uint16_t height) {
    // Send row-by-row with word-reversal but preserve byte order within each word.
    // The IT8951 interprets the first received byte as the high byte of each 16-bit word,
    // so we must send the framebuffer bytes in their natural memory order (low address first)
    // to keep nibble/pixel alignment correct.
    uint8_t row_buffer[936];
    const uint32_t row_size_bytes = uint32_t(width_in_words) * 2;
    if (row_size_bytes > sizeof(row_buffer)) {
        Log_error("%s: Row buffer too small for %u-byte row", TAG, (unsigned)row_size_bytes);
        return;
    }

    const uint8_t *byte_buf = reinterpret_cast<const uint8_t *>(buf);

    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x0000); // data preamble
    lcdWaitForReady();

    for (uint16_t y = 0; y < height; y++) {
        const uint32_t row_byte_start = uint32_t(y) * row_size_bytes;
        for (uint16_t x = 0; x < width_in_words; x++) {
            const uint32_t src_byte_offset = row_byte_start + (width_in_words - 1 - x) * 2;
            const uint32_t dst_byte_index = uint32_t(x) * 2;
            row_buffer[dst_byte_index] = byte_buf[src_byte_offset];
            row_buffer[dst_byte_index + 1] = byte_buf[src_byte_offset + 1];
        }
        SPI.writeBytes(row_buffer, row_size_bytes);
        if ((y & 0x07) == 0) {
            yield();
        }
    }

    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
}

void IT8951::writeFramebufferRows1bpp(const uint8_t *buf_4bpp, uint16_t width, uint16_t height) {
    uint16_t row_words[117]; // 1872/16 = 117
    uint8_t row_buffer[234]; // 117 * 2
    const uint16_t width_in_words = width / 16;
    const uint32_t row_size_bytes = uint32_t(width_in_words) * 2;

    if (width_in_words > (sizeof(row_words) / sizeof(row_words[0])) ||
        row_size_bytes > sizeof(row_buffer)) {
        Log_error("%s: 1bpp row buffer too small for %u-byte row", TAG, (unsigned)row_size_bytes);
        return;
    }

    digitalWrite(pins_.cs, LOW);
    SPI.beginTransaction(SPISettings(spi_frequency_, MSBFIRST, SPI_MODE0));
    lcdWaitForReady();
    spiSendWord(0x0000); // data preamble
    lcdWaitForReady();

    for (uint16_t y = 0; y < height; y++) {
        memset(row_words, 0x00, row_size_bytes);

        for (uint16_t x = 0; x < width; x++) {
            // Read 4bpp nibble from the source buffer
            const uint32_t pos = (x + y * width) / 2;
            uint8_t nibble;
            if ((x & 1U) == 0) {
                nibble = buf_4bpp[pos] >> 4;
            } else {
                nibble = buf_4bpp[pos] & 0x0F;
            }
            // Threshold: nibble <= 7 → black (set bit)
            if (nibble <= 0x07) {
                row_words[x / 16] |= uint16_t(0x8000 >> (x & 0x0F));
            }
        }

        // Reverse words for IT8951 upload order
        for (uint16_t i = 0; i < width_in_words; i++) {
            const uint16_t word = row_words[width_in_words - 1 - i];
            const uint32_t byte_index = uint32_t(i) * 2;
            row_buffer[byte_index] = static_cast<uint8_t>(word >> 8);
            row_buffer[byte_index + 1] = static_cast<uint8_t>(word & 0xFF);
        }

        SPI.writeBytes(row_buffer, row_size_bytes);
        if ((y & 0x07) == 0) {
            yield();
        }
    }

    SPI.endTransaction();
    digitalWrite(pins_.cs, HIGH);
}

// ─── Public API ─────────────────────────────────────────────────────────────

bool IT8951::init(const IT8951Pins &pins, uint16_t vcom) {
    pins_ = pins;
    vcom_ = vcom;
    initialized_ = false;

    Log_info("%s: Setting up IT8951 reTerminal E1003...", TAG);

    pinMode(pins_.cs, OUTPUT);
    pinMode(pins_.en, OUTPUT);
    pinMode(pins_.ite_en, OUTPUT);
    pinMode(pins_.rst, OUTPUT);
    pinMode(pins_.busy, INPUT);

    digitalWrite(pins_.cs, HIGH);
    digitalWrite(pins_.en, HIGH);
    digitalWrite(pins_.ite_en, HIGH);
    digitalWrite(pins_.rst, HIGH);

    spi_frequency_ = IT8951_SPI_PROBE_FREQUENCY;
    SPI.begin(pins_.sck, pins_.miso, pins_.mosi, -1);
    Log_info("%s: SPI bus initialized at %u Hz probe speed", TAG, spi_frequency_);

    // Multi-attempt probe sequence
    struct ProbeAttempt {
        const char *label;
        bool send_sys_run;
        int vcom_selector;
    };
    static const ProbeAttempt attempts[] = {
        {"cold read", false, 0},
        {"wake then read", true, 0},
        {"wake + VCOM 0x0001", true, 0x0001},
        {"wake + VCOM 0x0002", true, 0x0002},
    };

    bool found_device = false;
    for (size_t i = 0; i < sizeof(attempts) / sizeof(attempts[0]); i++) {
        Log_info("%s: Probe attempt %u: %s", TAG, (unsigned)(i + 1), attempts[i].label);
        powerCycle();
        Log_info("%s: [%s] Power cycle complete, HRDY=%s",
                 TAG, attempts[i].label,
                 digitalRead(pins_.busy) ? "HIGH" : "LOW");
        lcdWaitForReady();
        if (probeController(attempts[i].label, attempts[i].send_sys_run, attempts[i].vcom_selector)) {
            found_device = true;
            break;
        }
    }

    if (!found_device) {
        Log_error("%s: SPI communication failed - IT8951 never returned valid device info", TAG);
        return false;
    }

    // If VCOM wasn't verified during probe, try preferred selectors
    if (vcom_write_selector_ == 0) {
        Log_info("%s: Panel answered before VCOM was verified, trying selector 0x0002", TAG);
        writeVcom(0x0002, vcom_);
        lcdWriteCmdCode(USDEF_I80_CMD_VCOM);
        lcdWriteData(0x0000);
        uint16_t readback = lcdReadData();
        if (readback == vcom_) {
            vcom_write_selector_ = 0x0002;
        } else {
            Log_info("%s: Selector 0x0002 read-back was %u, trying 0x0001", TAG, readback);
            writeVcom(0x0001, vcom_);
            lcdWriteCmdCode(USDEF_I80_CMD_VCOM);
            lcdWriteData(0x0000);
            readback = lcdReadData();
            if (readback == vcom_) {
                vcom_write_selector_ = 0x0001;
            }
        }
    }

    img_buf_addr_ = (uint32_t(dev_info_.img_buf_addr_h) << 16) | dev_info_.img_buf_addr_l;
    Log_info("%s: Panel: %dx%d, ImgBuf: 0x%08X", TAG,
             dev_info_.panel_width, dev_info_.panel_height, img_buf_addr_);

    // Enable packed write mode and set temperature
    writeReg(IT8951_REG_I80CPCR, 0x0001);
    lcdWriteCmdCode(USDEF_I80_CMD_TEMP);
    lcdWriteData(0x0001);
    lcdWriteData(14);

    // Switch to runtime SPI speed
    spi_frequency_ = IT8951_SPI_RUN_FREQUENCY;
    Log_info("%s: Switching SPI speed to %u Hz", TAG, spi_frequency_);

    initialized_ = true;
    Log_info("%s: IT8951 initialization complete", TAG);
    return true;
}

void IT8951::waitForDisplayReady() {
    const uint32_t start = millis();
    while (readReg(IT8951_REG_LUTAFSR) != 0) {
        if (millis() - start > 30000) {
            Log_error("%s: Display-ready timeout (LUTAFSR)", TAG);
            break;
        }
        yield();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void IT8951::uploadFramebuffer4bpp(const uint8_t *buf, uint16_t width, uint16_t height) {
    if (!initialized_) return;

    const uint16_t width_in_words = (width + 3) / 4;

    waitForDisplayReady();
    setImgBufBaseAddr(img_buf_addr_);

    // Disable 1bpp mode if it was previously enabled
    writeReg(IT8951_REG_UP1SR + 2, readReg(IT8951_REG_UP1SR + 2) & ~(1 << 2));

    loadImgAreaStart(IT8951_LDIMG_L_ENDIAN, IT8951_4BPP, 0, 0, 0, width, height);
    Log_info("%s: Uploading 4bpp: %u rows x %u words", TAG, height, width_in_words);
    writeFramebufferRows4bpp(reinterpret_cast<const uint16_t *>(buf), width_in_words, height);
    lcdWriteCmdCode(IT8951_TCON_LD_IMG_END);
}

void IT8951::uploadFramebuffer1bpp(const uint8_t *buf, uint16_t width, uint16_t height) {
    if (!initialized_) return;

    const uint16_t one_bpp_width_bytes = width / 8;

    waitForDisplayReady();
    setImgBufBaseAddr(img_buf_addr_);

    loadImgAreaStart(IT8951_LDIMG_L_ENDIAN, IT8951_8BPP, 0, 0, 0, one_bpp_width_bytes, height);
    Log_info("%s: Uploading 1bpp: %u rows x %u packed bytes", TAG, height, one_bpp_width_bytes);
    writeFramebufferRows1bpp(buf, width, height);
    lcdWriteCmdCode(IT8951_TCON_LD_IMG_END);
}

void IT8951::displayArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode) {
    if (!initialized_) return;
    lcdWriteCmdCode(USDEF_I80_CMD_DPY_AREA);
    lcdWriteData(x);
    lcdWriteData(y);
    lcdWriteData(w);
    lcdWriteData(h);
    lcdWriteData(mode);
}

void IT8951::displayArea1bpp(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                             uint16_t mode, uint8_t bg_gray, uint8_t fg_gray) {
    if (!initialized_) return;
    // Enable 1bpp mode
    writeReg(IT8951_REG_UP1SR + 2, readReg(IT8951_REG_UP1SR + 2) | (1 << 2));
    writeReg(IT8951_REG_BGVR, (uint16_t(bg_gray) << 8) | fg_gray);
    displayArea(x, y, w, h, mode);
    waitForDisplayReady();
}

void IT8951::sleep() {
    if (!initialized_) return;
    Log_info("%s: Entering sleep mode", TAG);
    lcdWriteCmdCode(IT8951_TCON_SLEEP);
}

bool IT8951::framebufferIsBinary(const uint8_t *buf, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        const uint8_t hi = buf[i] >> 4;
        const uint8_t lo = buf[i] & 0x0F;
        if ((hi != 0x00 && hi != 0x0F) || (lo != 0x00 && lo != 0x0F)) {
            return false;
        }
    }
    return true;
}
