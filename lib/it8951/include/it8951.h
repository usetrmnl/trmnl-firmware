#pragma once

#include <Arduino.h>
#include <SPI.h>

// IT8951 command codes
#define IT8951_TCON_SYS_RUN      0x0001
#define IT8951_TCON_STANDBY      0x0002
#define IT8951_TCON_SLEEP        0x0003
#define IT8951_TCON_REG_RD       0x0010
#define IT8951_TCON_REG_WR       0x0011
#define IT8951_TCON_LD_IMG       0x0020
#define IT8951_TCON_LD_IMG_AREA  0x0021
#define IT8951_TCON_LD_IMG_END   0x0022

// User-defined I80 command codes
#define USDEF_I80_CMD_DPY_AREA       0x0034
#define USDEF_I80_CMD_GET_DEV_INFO   0x0302
#define USDEF_I80_CMD_VCOM           0x0039
#define USDEF_I80_CMD_TEMP           0x0040

// Pixel format constants
#define IT8951_4BPP 2
#define IT8951_8BPP 3
#define IT8951_LDIMG_L_ENDIAN 0

// Register addresses
#define IT8951_REG_I80CPCR  0x0004
#define IT8951_REG_LISAR    0x0208
#define IT8951_REG_LUTAFSR  0x1224
#define IT8951_REG_UP1SR    0x1138
#define IT8951_REG_BGVR     0x1250

// SPI frequencies
#define IT8951_SPI_PROBE_FREQUENCY 1000000
#define IT8951_SPI_RUN_FREQUENCY   4000000

// IT8951 device info structure
typedef struct {
    uint16_t panel_width;
    uint16_t panel_height;
    uint16_t img_buf_addr_l;
    uint16_t img_buf_addr_h;
    uint16_t fw_version[8];
    uint16_t lut_version[8];
} IT8951DevInfo;

// Pin configuration
typedef struct {
    int8_t sck;
    int8_t miso;
    int8_t mosi;
    int8_t cs;
    int8_t busy;
    int8_t rst;
    int8_t en;
    int8_t ite_en;
} IT8951Pins;

class IT8951 {
public:
    /**
     * Initialize the IT8951 controller.
     * Performs hardware reset, SPI probe, VCOM configuration, and PSRAM check.
     * @param pins Pin configuration for the IT8951
     * @param vcom VCOM voltage value (default 1400 = -1.40V)
     * @return true if initialization succeeded
     */
    bool init(const IT8951Pins &pins, uint16_t vcom = 1400);

    /**
     * Upload a 4bpp framebuffer to the IT8951.
     * The buffer format is packed nibbles: 2 pixels per byte, high nibble first.
     * 0x00 = black, 0x0F = white.
     * @param buf Framebuffer data (width*height/2 bytes)
     * @param width Display width in pixels
     * @param height Display height in pixels
     */
    void uploadFramebuffer4bpp(const uint8_t *buf, uint16_t width, uint16_t height);

    /**
     * Upload a 1bpp framebuffer (derived from 4bpp data) to the IT8951.
     * Scans the 4bpp buffer and packs pixels into 1bpp for sharper text rendering.
     * @param buf 4bpp framebuffer data
     * @param width Display width in pixels
     * @param height Display height in pixels
     */
    void uploadFramebuffer1bpp(const uint8_t *buf, uint16_t width, uint16_t height);

    /**
     * Trigger a display refresh in the given area.
     * @param x X coordinate
     * @param y Y coordinate
     * @param w Width
     * @param h Height
     * @param mode Waveform mode (2 = GC16 for grayscale)
     */
    void displayArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t mode = 2);

    /**
     * Trigger a 1bpp display refresh with foreground/background gray levels.
     * @param x X coordinate
     * @param y Y coordinate
     * @param w Width
     * @param h Height
     * @param mode Waveform mode
     * @param bg_gray Background gray level (0xFF = white)
     * @param fg_gray Foreground gray level (0x00 = black)
     */
    void displayArea1bpp(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         uint16_t mode = 2, uint8_t bg_gray = 0xFF, uint8_t fg_gray = 0x00);

    /**
     * Put the IT8951 into sleep mode.
     */
    void sleep();

    /**
     * Wait until the display engine finishes the current operation.
     */
    void waitForDisplayReady();

    /**
     * Check if the framebuffer contains only pure black (0x00) and white (0x0F) pixels.
     * If true, the sharper 1bpp upload path can be used.
     * @param buf 4bpp framebuffer
     * @param size Buffer size in bytes
     * @return true if all pixels are binary
     */
    static bool framebufferIsBinary(const uint8_t *buf, uint32_t size);

    uint16_t getWidth() const { return dev_info_.panel_width; }
    uint16_t getHeight() const { return dev_info_.panel_height; }
    uint32_t getImgBufAddr() const { return img_buf_addr_; }
    bool isInitialized() const { return initialized_; }

private:
    // Low-level SPI helpers
    void spiSendWord(uint16_t word);
    uint16_t spiRecvWord();
    void lcdWaitForReady();
    void lcdWriteCmdCode(uint16_t cmd);
    void lcdWriteData(uint16_t data);
    void lcdWriteNData(const uint16_t *buf, uint32_t word_count);
    uint16_t lcdReadData();
    void lcdReadNData(uint16_t *buf, uint32_t word_count);
    void lcdSendCmdArg(uint16_t cmd, uint16_t *args, uint16_t num_args);

    // Hardware control
    void hardwareReset();
    void powerCycle();

    // Register access
    uint16_t readReg(uint16_t addr);
    void writeReg(uint16_t addr, uint16_t val);

    // IT8951 commands
    void getSystemInfo();
    void setImgBufBaseAddr(uint32_t addr);
    void loadImgAreaStart(uint16_t endian, uint16_t pix_fmt, uint16_t rotate,
                          uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    void writeVcom(uint16_t selector, uint16_t value);
    bool probeController(const char *label, bool send_sys_run, int vcom_selector);
    bool hasValidDevInfo() const;

    // Bulk framebuffer upload helpers
    void writeFramebufferRows4bpp(const uint16_t *buf, uint16_t width_in_words, uint16_t height);
    void writeFramebufferRows1bpp(const uint8_t *buf_4bpp, uint16_t width, uint16_t height);

    IT8951Pins pins_{};
    IT8951DevInfo dev_info_{};
    uint32_t img_buf_addr_{0};
    uint16_t vcom_{1400};
    uint32_t spi_frequency_{IT8951_SPI_PROBE_FREQUENCY};
    uint16_t vcom_write_selector_{0};
    bool initialized_{false};
};
