//
// I/O wrapper functions for the ESP-IDF
// These functions are I/O wrapper functions
// suitable for all ESP32 MCUs
//
#ifndef __ESP_IDF_IO__
#define __ESP_IDF_IO__

#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define INPUT 0
#define INPUT_PULLUP 1
#define OUTPUT 2
#define DISABLED 3
#define HIGH 1
#define LOW 0

#define pgm_read_byte(a) *(uint8_t *)(a)
#define pgm_read_word(a) *(uint16_t *)(a)
#define pgm_read_dword(a) *(uint32_t *)(a)
#define memcpy_P memcpy

static spi_bus_config_t buscfg;
static spi_device_interface_config_t devcfg;
static spi_transaction_t trans;
static spi_device_handle_t spi;

#ifdef VSPI_HOST
#define ESP32_SPI_HOST VSPI_HOST
#else
#define ESP32_SPI_HOST SPI2_HOST
#endif // VSPI_HOST
// foreward references
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);

void digitalWrite(int iPin, int iState) {
   gpio_set_level((gpio_num_t)iPin, (uint32_t)iState);
}
void pinMode(int iPin, int iMode)
{
    gpio_config_t io_conf = {};

    gpio_reset_pin((gpio_num_t)iPin);
    if (iMode == DISABLED) return;
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << iPin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = (iMode == INPUT_PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE; // pull-up mode
    if (iMode == INPUT || iMode == INPUT_PULLUP) {
        io_conf.mode = GPIO_MODE_INPUT;
    } else { // must be output
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    gpio_config(&io_conf); //configure GPIO with the given settings
} /* pinMode() */
int digitalRead(int iPin)
{
  return (int)gpio_get_level((gpio_num_t)iPin);
} /* digitalRead() */

long millis(void)
{
    return (long)(esp_timer_get_time() / 1000L);
} /* millis() */

void delayMicroseconds(long l)
{
    l *= 40;
    while (l) {
        __asm__ __volatile__ ("nop");
        l--;
    }
}
void delay(long l)
{
    delayMicroseconds(l*1000);
}
void mymemset(void *d, uint8_t u8, int len)
{
    uint8_t *d8;
    if (((intptr_t)d & 3) == 0) { // dword aligned
        uint32_t u32, *d32 = (uint32_t *)d;
        u32 = u8 | (u8 << 8);
        u32 |= (u32 << 16);
        while (len >= 4) {
            *d32++ = u32;
            len -= 4;
        }
        d = d32;
    }
    d8 = (uint8_t *)d;
    while (len) {
        *d8++ = u8;
        len--;
    }
} /* mymemset() */
void mymemcpy(void *d, void *s, size_t iLen)
{
    uint8_t *d8 = (uint8_t *)d;
    uint8_t *s8 = (uint8_t *)s;
    while (iLen) {
        *d8++ = *s8++;
        iLen--;
    }
} /* mymemcpy() */
//
// Convert a number into a fixed length, zero-terminated string
//
void i2strf(char *pDest, int iVal, int iDigits)
{
        char *d = pDest;
        int i;
        pDest[iDigits] = 0;
        while (iDigits) {
                iDigits--;
            i = iVal % 10;
            d[iDigits] = '0' + (char)i;
            iVal /= 10;
        }
} /* i2strf() */
//
// Convert a number into a zero-terminated string
//
int i2str(char *pDest, int iVal)
{
        char *d = pDest;
        int i, iPlaceVal = 10000;
        int iDigits = 0;

        if (iVal < 0) {
                iDigits++;
                *d++ = '-';
                iVal = -iVal;
        }
        while (iPlaceVal) {
                if (iVal >= iPlaceVal) {
                        i = iVal / iPlaceVal;
                        *d++ = '0' + (char)i;
                        iVal -= (i*iPlaceVal);
                        iDigits++;
                } else if (iDigits != 0) {
                        *d++ = '0'; // non-zeros were already displayed
                }
                iPlaceVal /= 10;
        }
        if (d == pDest) // must be zero
                *d++ = '0';
        *d++ = 0; // terminator
        return (int)(d - pDest - 1); // string length
} /* i2str() */
void delayCycles(int i)
{
    while (i > 3) {
        __asm__ __volatile__ ("nop\n\t");
        i -= 3;
    }
}
void spi_write(BBEPDISP *pBBEP, uint8_t *pBuf, int iLen)
{
    esp_err_t ret;

    digitalWrite(pBBEP->iCSPin, LOW);
    memset(&trans, 0, sizeof(trans));       //Zero out the transaction
    while (iLen) {
        int l = iLen;
        if (l > 4000) { // transmit maximum length (full duplex mode)
            l = 4000;
        }
        trans.length=l*8;  // length in bits
        trans.rxlength = 0;
        trans.tx_buffer=pBuf;
        // Queue the transaction
        ret = spi_device_polling_transmit(spi, &trans);  //Transmit!
        assert(ret==ESP_OK);            //Should have had no issues.
        iLen -= l;
        pBuf += l;
    } // while (iLen)
    digitalWrite(pBBEP->iCSPin, HIGH);
} /* spi_write() */
//
// Set the second CS pin for dual-controller displays
//
void bbepSetCS2(BBEPDISP *pBBEP, uint8_t cs)
{
    pBBEP->iCS1Pin = pBBEP->iCSPin;
    pBBEP->iCS2Pin = cs;
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH); // disable second CS for now
} /* bbepSetCS2() */
//
// Write a single byte as a COMMAND (D/C set low)
//
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd)
{
    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    digitalWrite(pBBEP->iDCPin, LOW);
    delay(1);
    spi_write(pBBEP, &cmd, 1);
    digitalWrite(pBBEP->iDCPin, HIGH); // leave data mode as the default
} /* bbepWriteCmd() */
//
// Convenience function to write a command byte along with a data
// byte (it's single parameter)
//
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2)
{
    bbepWriteCmd(pBBEP, cmd1);
    bbepWriteData(pBBEP, &cmd2, 1);
} /* bbepCMD2() */
//
// Write 1 or more bytes as DATA (D/C set high)
//
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
    if (pBBEP->iFlags & BBEP_CS_EVERY_BYTE) {
        for (int i=0; i<iLen; i++) {
            spi_write(pBBEP, &pData[i], 1);
        }
    } else {
        spi_write(pBBEP, pData, iLen);
    }
} /* bbepWriteData() */

//
// Initialize the SPI bus and connections for e-paper displays
//
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed)
{
    esp_err_t ret;
    
    pBBEP->iDCPin = u8DC;
    pBBEP->iCSPin = u8CS;
    pBBEP->iMOSIPin = u8MOSI;
    pBBEP->iCLKPin = u8SCK;
    pBBEP->iRSTPin = u8RST;
    pBBEP->iBUSYPin = u8BUSY;

    pinMode(pBBEP->iDCPin, OUTPUT);
    pinMode(pBBEP->iRSTPin, OUTPUT);
    digitalWrite(pBBEP->iRSTPin, LOW);
    delay(100);
    digitalWrite(pBBEP->iRSTPin, HIGH);
    delay(100);
    if (pBBEP->iBUSYPin != 0xff) {
        pinMode(pBBEP->iBUSYPin, INPUT);
    }
    pBBEP->iSpeed = u32Speed;
    pinMode(pBBEP->iCSPin, OUTPUT);
    digitalWrite(pBBEP->iCSPin, HIGH); // manually control the CS pin

    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = -1; //u8MISO;
    buscfg.mosi_io_num = u8MOSI;
    buscfg.sclk_io_num = u8SCK;
    buscfg.max_transfer_sz=4096;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;
    //Initialize the SPI bus
    ret=spi_bus_initialize(ESP32_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    assert(ret==ESP_OK);

    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = u32Speed;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1; // we control the CS pin
    devcfg.queue_size = 2;                          //We want to be able to queue 2 transactions at a time
// These callbacks currently don't do anything
//    devcfg.pre_cb = spi_pre_transfer_callback;  //Specify pre-transfer callback to handle D/C line
//    devcfg.post_cb = spi_post_transfer_callback;
//    devcfg.flags = SPI_DEVICE_NO_DUMMY; // allow speeds > 26Mhz
    devcfg.flags = SPI_DEVICE_HALFDUPLEX; // this disables SD card access
    ret=spi_bus_add_device(ESP32_SPI_HOST, &devcfg, &spi); // attach to bus
    assert(ret==ESP_OK);
    
    if (pBBEP->iFlags & BBEP_7COLOR) { // need to send before you can send it data
        pBBEP->is_awake = 1;
        bbepSendCMDSequence(pBBEP, pBBEP->pInitFull);
//        if (pBBEP->iFlags & BBEP_SPLIT_BUFFER) { 
//           // Send the same sequence to the second controller
//           pBBEP->iCSPin = pBBEP->iCS2Pin;
//           bbepSendCMDSequence(pBBEP, pBBEP->pInitFull);
//           pBBEP->iCSPin = pBBEP->iCS1Pin;
//        }
    }
} /* bbepInitIO() */

#endif // __ESP_IDF_IO__
