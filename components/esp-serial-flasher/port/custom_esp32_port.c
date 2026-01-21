/* Custom ESP32 port implementation for esp-serial-flasher
 *
 * Based on the original esp32_port.c but with custom reset/bootloader control
 * via IO expander instead of direct GPIO.
 */

#include "esp32_port.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <unistd.h>

// External wrapper functions for IO expander
extern void modem_enter_bootloader(void);
extern void modem_reset_target(void);

static int64_t s_time_end;
static int32_t s_uart_port;
static bool s_peripheral_needs_deinit;

esp_loader_error_t loader_port_esp32_init(const loader_esp32_config_t *config)
{
    s_uart_port = config->uart_port;

    // Initialize UART using ESP-IDF driver (same as original esp32_port.c)
    if (!config->dont_initialize_peripheral) {
        uart_config_t uart_config = {
            .baud_rate = config->baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        int rx_buffer_size = config->rx_buffer_size ? config->rx_buffer_size : 400;
        int tx_buffer_size = config->tx_buffer_size ? config->tx_buffer_size : 400;
        QueueHandle_t *uart_queue = config->uart_queue ? config->uart_queue : NULL;
        int queue_size = config->queue_size ? config->queue_size : 0;

        // Set TX pin drive strength
        if (gpio_set_drive_capability(config->uart_tx_pin, GPIO_DRIVE_CAP_3) != ESP_OK) {
            return ESP_LOADER_ERROR_FAIL;
        }

        if (uart_param_config(s_uart_port, &uart_config) != ESP_OK) {
            return ESP_LOADER_ERROR_FAIL;
        }

        if (uart_set_pin(s_uart_port, config->uart_tx_pin, config->uart_rx_pin,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
            return ESP_LOADER_ERROR_FAIL;
        }

        if (uart_driver_install(s_uart_port, rx_buffer_size, tx_buffer_size,
                                queue_size, uart_queue, 0) != ESP_OK) {
            return ESP_LOADER_ERROR_FAIL;
        }

        s_peripheral_needs_deinit = true;
    }

    ESP_LOGI("custom_port", "UART initialized on port %d at %d baud", s_uart_port, config->baud_rate);
    ESP_LOGI("custom_port", "Reset/Boot controlled via IO expander (custom functions)");

    return ESP_LOADER_SUCCESS;
}


void loader_port_esp32_deinit(void)
{
    if (s_peripheral_needs_deinit) {
        uart_driver_delete(s_uart_port);
        s_peripheral_needs_deinit = false;
    }
}

esp_loader_error_t loader_port_write(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    uart_write_bytes(s_uart_port, (const char *)data, size);
    esp_err_t err = uart_wait_tx_done(s_uart_port, pdMS_TO_TICKS(timeout));

    if (err == ESP_OK) {
        return ESP_LOADER_SUCCESS;
    } else if (err == ESP_ERR_TIMEOUT) {
        return ESP_LOADER_ERROR_TIMEOUT;
    } else {
        return ESP_LOADER_ERROR_FAIL;
    }
}

esp_loader_error_t loader_port_read(uint8_t *data, uint16_t size, uint32_t timeout)
{
    int read = uart_read_bytes(s_uart_port, data, size, pdMS_TO_TICKS(timeout));

    if (read < 0) {
        return ESP_LOADER_ERROR_FAIL;
    } else if (read < size) {
        return ESP_LOADER_ERROR_TIMEOUT;
    } else {
        return ESP_LOADER_SUCCESS;
    }
}

void loader_port_enter_bootloader(void)
{
    modem_enter_bootloader();
}

void loader_port_reset_target(void)
{
    modem_reset_target();
}

void loader_port_delay_ms(uint32_t ms)
{
    usleep(ms * 1000);
}

void loader_port_start_timer(uint32_t ms)
{
    s_time_end = esp_timer_get_time() + ms * 1000;
}

uint32_t loader_port_remaining_time(void)
{
    int64_t remaining = (s_time_end - esp_timer_get_time()) / 1000;
    return (remaining > 0) ? (uint32_t)remaining : 0;
}

esp_loader_error_t loader_port_change_transmission_rate(uint32_t baudrate)
{
    esp_err_t err = uart_set_baudrate(s_uart_port, baudrate);
    return (err == ESP_OK) ? ESP_LOADER_SUCCESS : ESP_LOADER_ERROR_FAIL;
}

void loader_port_debug_print(const char *str)
{
    ESP_LOGI("LOADER", "%s", str);
}
