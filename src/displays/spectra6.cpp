#include "bb_epaper.h"
#include "config.h"
#include "DEV_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "spectra6.h"
#include "trmnl_log.h"
#include <Arduino.h>

extern BBEPAPER bbep;

static spi_device_handle_t spectra6Spi = nullptr;

static bool spectra6_wait_busy(const char *label)
{
    delay(10);
    uint32_t wait_count = 0;
    while (gpio_get_level((gpio_num_t)EPD_BUSY_PIN) == 0)
    {
        delay(10);
        if (++wait_count > 6000)
        {
            Log_error("spectra6 %s BUSY timeout", label);
            return false;
        }
    }
    return true;
}

static bool spectra6_transmit(const uint8_t *data, size_t len)
{
    spi_transaction_t transaction = {};
    transaction.length = len * 8;
    transaction.tx_buffer = data;
    return spi_device_polling_transmit(spectra6Spi, &transaction) == ESP_OK;
}

bool spectra6_init_spi()
{
    if (spectra6Spi)
        return true;

    gpio_config_t output_config = {};
    output_config.pin_bit_mask = (1ULL << EPD_RST_PIN) | (1ULL << EPD_DC_PIN) | (1ULL << EPD_CS_PIN);
    output_config.mode = GPIO_MODE_OUTPUT;
    output_config.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&output_config);

    gpio_config_t input_config = {};
    input_config.pin_bit_mask = (1ULL << EPD_BUSY_PIN);
    input_config.mode = GPIO_MODE_INPUT;
    input_config.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&input_config);

    gpio_set_level((gpio_num_t)EPD_CS_PIN, 1);
    gpio_set_level((gpio_num_t)EPD_DC_PIN, 0);

    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = EPD_MOSI_PIN;
    bus_config.miso_io_num = -1;
    bus_config.sclk_io_num = EPD_SCK_PIN;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        Log_error("spectra6 spi_bus_initialize failed: %s", esp_err_to_name(err));
        return false;
    }

    spi_device_interface_config_t device_config = {};
    device_config.clock_speed_hz = 20 * 1000 * 1000;
    device_config.mode = 0;
    device_config.spics_io_num = -1;
    device_config.queue_size = 1;
    device_config.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;

    err = spi_bus_add_device(SPI2_HOST, &device_config, &spectra6Spi);
    if (err != ESP_OK)
    {
        Log_error("spectra6 spi_bus_add_device failed: %s", esp_err_to_name(err));
        spectra6Spi = nullptr;
        return false;
    }

    Log_info("spectra6 SPI initialized");
    return true;
}

static bool spectra6_command_data(uint8_t command, const uint8_t *data, size_t len)
{
    if (!spectra6Spi)
        return false;

    spi_device_acquire_bus(spectra6Spi, portMAX_DELAY);
    gpio_set_level((gpio_num_t)EPD_DC_PIN, 0);
    gpio_set_level((gpio_num_t)EPD_CS_PIN, 0);

    spi_transaction_ext_t command_transaction = {};
    command_transaction.command_bits = 8;
    command_transaction.base.flags = SPI_TRANS_VARIABLE_CMD;
    command_transaction.base.cmd = command;
    esp_err_t err = spi_device_polling_transmit(spectra6Spi, &command_transaction.base);

    if (err == ESP_OK && data && len)
    {
        gpio_set_level((gpio_num_t)EPD_DC_PIN, 1);
        err = spectra6_transmit(data, len) ? ESP_OK : ESP_FAIL;
    }

    gpio_set_level((gpio_num_t)EPD_CS_PIN, 1);
    spi_device_release_bus(spectra6Spi);

    return err == ESP_OK;
}

static bool spectra6_command(uint8_t command)
{
    return spectra6_command_data(command, nullptr, 0);
}

static bool spectra6_send_buffer(const uint8_t *data, size_t len)
{
    if (!spectra6Spi)
        return false;

    const uint8_t *cursor = data;
    size_t remaining = len;
    while (remaining)
    {
        const size_t chunk = remaining > 128 ? 128 : remaining;

        spi_device_acquire_bus(spectra6Spi, portMAX_DELAY);
        gpio_set_level((gpio_num_t)EPD_DC_PIN, 1);
        gpio_set_level((gpio_num_t)EPD_CS_PIN, 0);
        const bool ok = spectra6_transmit(cursor, chunk);
        gpio_set_level((gpio_num_t)EPD_CS_PIN, 1);
        spi_device_release_bus(spectra6Spi);

        if (!ok)
            return false;
        cursor += chunk;
        remaining -= chunk;
    }

    return true;
}

static void spectra6_reset_panel()
{
    gpio_set_level((gpio_num_t)EPD_RST_PIN, 1);
    delay(50);
    gpio_set_level((gpio_num_t)EPD_RST_PIN, 0);
    delay(20);
    gpio_set_level((gpio_num_t)EPD_RST_PIN, 1);
    delay(50);
}

bool spectra6_update()
{
    Log_info("spectra6_update start");

    if (!spectra6_init_spi())
        return false;

    spectra6_reset_panel();
    if (!spectra6_wait_busy("reset"))
        return false;

    const uint8_t cmd_h[] = {0x49, 0x55, 0x20, 0x08, 0x09, 0x18};
    const uint8_t pwr[] = {0x3f};
    const uint8_t psr[] = {0x5f, 0x69};
    const uint8_t pfs[] = {0x00, 0x54, 0x00, 0x44};
    const uint8_t btst1[] = {0x40, 0x1f, 0x1f, 0x2c};
    const uint8_t btst2[] = {0x6f, 0x1f, 0x17, 0x49};
    const uint8_t btst3[] = {0x6f, 0x1f, 0x1f, 0x22};
    const uint8_t pll[] = {0x03};
    const uint8_t cdi[] = {0x3f};
    const uint8_t tcon[] = {0x02, 0x00};
    const uint8_t tres[] = {0x03, 0x20, 0x01, 0xe0};
    const uint8_t tvdcs[] = {0x01};
    const uint8_t pws[] = {0x2f};

    if (!spectra6_command_data(0xaa, cmd_h, sizeof(cmd_h)))
        return false;
    if (!spectra6_command_data(0x01, pwr, sizeof(pwr)))
        return false;
    if (!spectra6_command_data(0x00, psr, sizeof(psr)))
        return false;
    if (!spectra6_command_data(0x03, pfs, sizeof(pfs)))
        return false;
    if (!spectra6_command_data(0x05, btst1, sizeof(btst1)))
        return false;
    if (!spectra6_command_data(0x06, btst2, sizeof(btst2)))
        return false;
    if (!spectra6_command_data(0x08, btst3, sizeof(btst3)))
        return false;
    if (!spectra6_command_data(0x30, pll, sizeof(pll)))
        return false;
    if (!spectra6_command_data(0x50, cdi, sizeof(cdi)))
        return false;
    if (!spectra6_command_data(0x60, tcon, sizeof(tcon)))
        return false;
    if (!spectra6_command_data(0x61, tres, sizeof(tres)))
        return false;
    if (!spectra6_command_data(0x84, tvdcs, sizeof(tvdcs)))
        return false;
    if (!spectra6_command_data(0xe3, pws, sizeof(pws)))
        return false;
    if (!spectra6_wait_busy("init"))
        return false;

    if (!spectra6_command(0x10))
        return false;
    uint8_t *framebuffer = (uint8_t *)bbep.getBuffer();
    if (framebuffer)
    {
        if (!spectra6_send_buffer(framebuffer, 800 * 480 / 2))
            return false;
    }
    else
    {
        uint8_t white_row[800 / 2];
        memset(white_row, 0x11, sizeof(white_row));
        for (int y = 0; y < 480; y++)
        {
            if (!spectra6_send_buffer(white_row, sizeof(white_row)))
                return false;
        }
    }
    if (!spectra6_wait_busy("data"))
        return false;

    if (!spectra6_command(0x04))
        return false;
    if (!spectra6_wait_busy("power on"))
        return false;

    const uint8_t refresh_data[] = {0x00};
    if (!spectra6_command_data(0x12, refresh_data, sizeof(refresh_data)))
        return false;
    if (!spectra6_wait_busy("refresh"))
        return false;

    const uint8_t power_off_data[] = {0x00};
    const uint8_t deep_sleep_data[] = {0xa5};
    if (!spectra6_command_data(0x02, power_off_data, sizeof(power_off_data)))
        return false;
    if (!spectra6_wait_busy("power off"))
        return false;
    if (!spectra6_command_data(0x07, deep_sleep_data, sizeof(deep_sleep_data)))
        return false;

    Log_info("spectra6_update end");
    return true;
}