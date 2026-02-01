#include "rtc_wake_stub_i2c.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_wake_stub.h"

static int sda_pin = 0;
static int scl_pin = 0;
static gpio_dev_t *gpio_dev;

static inline void wake_stub_feed_rtc_wdt(void)
{
    REG_WRITE(RTC_CNTL_WDTFEED_REG, RTC_CNTL_WDT_FEED);
}

// Use ROM delay for more accurate timing
static inline void delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static inline void sda_high(void)
{
    gpio_ll_set_level(gpio_dev, sda_pin, 1);
}

static inline void sda_low(void)
{
    gpio_ll_set_level(gpio_dev, sda_pin, 0);
}

static inline void scl_high(void)
{
    gpio_ll_set_level(gpio_dev, scl_pin, 1);
}

static inline void scl_low(void)
{
    gpio_ll_set_level(gpio_dev, scl_pin, 0);
}

static inline bool sda_read(void)
{
    return gpio_ll_get_level(gpio_dev, sda_pin);
}

void wake_stub_i2c_init(int sda, int scl)
{
    sda_pin = sda;
    scl_pin = scl;
    gpio_dev = GPIO_LL_GET_HW(GPIO_PORT_0);

    // Configure SDA pin
    gpio_ll_output_enable(gpio_dev, sda);
    gpio_ll_input_enable(gpio_dev, sda);
    gpio_ll_func_sel(gpio_dev, sda, PIN_FUNC_GPIO); 
    gpio_ll_od_enable(gpio_dev, sda);
    gpio_ll_pullup_en(gpio_dev, sda);
    sda_high();
    
    // Configure SCL pin
    gpio_ll_output_enable(gpio_dev, scl);
    gpio_ll_func_sel(gpio_dev, scl, PIN_FUNC_GPIO);
    gpio_ll_od_enable(gpio_dev, scl);
    gpio_ll_pullup_en(gpio_dev, scl);
    scl_high();

    // delay_us(100);
}

static void i2c_start(void)
{
    sda_high();
    scl_high();
    delay_us(10);
    sda_low();
    delay_us(10);
    scl_low();
    delay_us(10);
}

static void i2c_stop(void)
{
    sda_low();
    scl_low();
    delay_us(10);
    scl_high();
    delay_us(10);
    sda_high();
    delay_us(10);
}

static bool i2c_write_byte(uint8_t data)
{
    // Send 8 bits (MSB first)
    for (int i = 7; i >= 0; i--) {
        scl_low();
        delay_us(5);
        
        if (data & (1 << i)) {
            sda_high();
        } else {
            sda_low();
        }
        
        delay_us(5);
        scl_high();
        delay_us(10);
        wake_stub_feed_rtc_wdt();
    }
    
    // Read ACK bit
    scl_low();
    delay_us(5);
    sda_high();
    delay_us(5);
    scl_high();
    delay_us(5);
    
    bool ack = !sda_read();
    
    delay_us(5);
    scl_low();
    delay_us(10);
    
    return ack;
}

static uint8_t i2c_read_byte(bool send_ack)
{
    uint8_t data = 0;
    
    sda_high();
    
    // Read 8 bits (MSB first)
    for (int i = 7; i >= 0; i--) {
        scl_low();
        delay_us(10);
        scl_high();
        delay_us(5);
        
        if (sda_read()) {
            data |= (1 << i);
        }
        
        delay_us(5);
        wake_stub_feed_rtc_wdt();
    }
    
    // Send ACK/NACK
    scl_low();
    delay_us(5);
    
    if (send_ack) {
        sda_low();
    } else {
        sda_high();
    }
    
    delay_us(5);
    scl_high();
    delay_us(10);
    scl_low();
    delay_us(10);
    
    return data;
}

void wake_stub_i2c_write(uint8_t addr7, const uint8_t *data, size_t len)
{
    i2c_start();
    
    // Send 7-bit address with write bit (0)
    if (!i2c_write_byte((addr7 << 1) | 0)) {
        i2c_stop();
        return;
    }
    
    // Send data bytes
    for (size_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) {
            break;
        }
        wake_stub_feed_rtc_wdt();
    }
    
    i2c_stop();
}

void wake_stub_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *out, size_t len)
{
    // Write register address
    i2c_start();
    
    if (!i2c_write_byte((addr7 << 1) | 0)) {
        i2c_stop();
        return;
    }
    
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return;
    }
    
    // Restart and read data
    i2c_start();
    
    if (!i2c_write_byte((addr7 << 1) | 1)) {
        i2c_stop();
        return;
    }
    
    // Read data bytes
    for (size_t i = 0; i < len; i++) {
        out[i] = i2c_read_byte(i < (len - 1));
        wake_stub_feed_rtc_wdt();
    }
    
    i2c_stop();
}