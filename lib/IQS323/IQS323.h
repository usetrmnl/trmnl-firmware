/******************************************************************************
 *                                                                            *
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 * @file        IQS323.h                                                      *
 * @brief       This file contains all the necessary settings for the IQS323  *
 *              and this file can be changed from the GUI or edited here      *
 * @author      JN. Lochner - Azoteq PTY Ltd                                  *
 * @version     v1.5.2                                                        *
 * @date        2023                                                          *
 * ========================================================================== *
 * @attention  Makes use of the following standard Arduino libraries:         *
 * - Arduino.h -> Included in IQS323.h, comes standard with Arduino           *
 * - Wire.h    -> Included in IQS323.h, comes standard with Arduino           *
 ******************************************************************************/

#ifndef IQS323_INIT_H
#define IQS323_INIT_H

// Include Files
#include "Arduino.h"
#include "Wire.h"
#include "./inc/iqs323_addresses.h"

/* Select the EV-Kit below by changing the value of the define (default = 0):
 * 0: Inductive Options EV-Kit (AZP1212A4).
 * 1: Slider EV-Kit (AZP1209A4).
 * 2: 3-Projected Buttons EV-Kit (AZP1210A4).
 */
#define IQS323_EV_KIT   2

// Public Global Definitions
/* For use with Wire.h library. True argument with some functions closes the
I2C communication window. */
#define STOP    true
/* For use with Wire.h library. False argument with some functions keeps the
I2C communication window open.*/
#define RESTART false

// Device Info
#define IQS323_PRODUCT_NUM_001          1106
#define IQS323_PRODUCT_NUM_A01          1462

// Info Flags Byte Bits.
#define IQS323_SLIDER_EVENT_BIT	        2
#define IQS323_GESTURE_EVENT_BIT	6
#define IQS323_POWER_EVENT_BIT_0        6
#define IQS323_POWER_EVENT_BIT_1        7
#define IQS323_NORMAL_POWER_BIT		0b00
#define IQS323_LOW_POWER_BIT		0b01
#define IQS323_ULP_BIT		        0b10
#define IQS323_HALT_BIT		        0b11

// System event bits
#define IQS323_ATI_ACTIVE_BIT		5
#define IQS323_SHOW_RESET_BIT		7

// Channel proximity and touch bits
#define IQS323_CH0_PROX_BIT             0
#define IQS323_CH0_TOUCH_BIT            1
#define IQS323_CH1_PROX_BIT             2
#define IQS323_CH1_TOUCH_BIT            3
#define IQS323_CH2_PROX_BIT             4
#define IQS323_CH2_TOUCH_BIT            5

// Slider Event Bits
#define IQS323_GESTURE_TAP_BIT          0
#define IQS323_GESTURE_SWIPE_POS_BIT    1
#define IQS323_GESTURE_SWIPE_NEG_BIT    2
#define IQS323_GESTURE_FLICK_POS_BIT    3
#define IQS323_GESTURE_FLICK_NEG_BIT    4
#define IQS323_GESTURE_HOLD_BIT         5
#define IQS323_GESTURE_EVENT_BIT	6
#define IQS323_GESTURE_BUSY_BIT         7

// Utility Bits
#define IQS323_ACK_RESET_BIT		0
#define IQS323_SW_RESET_BIT		1
#define IQS323_RE_ATI_BIT		2
#define IQS323_RESEED_BIT		3

#define IQS323_EVENT_MODE_BIT           7

/* Defines and structs for IQS323 states */
/**
* @brief  iqs323 Init Enumeration.
*/
typedef enum {
        IQS323_INIT_NONE = (uint8_t) 0x00,
        IQS323_INIT_VERIFY_PRODUCT,
        IQS323_INIT_READ_RESET,
	IQS323_INIT_CHIP_RESET,
	IQS323_INIT_UPDATE_SETTINGS,
	IQS323_INIT_CHECK_RESET,
	IQS323_INIT_ACK_RESET,
	IQS323_INIT_ATI,
        IQS323_INIT_WAIT_FOR_ATI,
        IQS323_INIT_READ_DATA,
	IQS323_INIT_ACTIVATE_EVENT_MODE,
	IQS323_INIT_DONE
} iqs323_init_e;

typedef enum {
        IQS323_STATE_NONE = (uint8_t) 0x00,
        IQS323_STATE_START,
        IQS323_STATE_INIT,
        IQS323_STATE_SW_RESET,
        IQS323_STATE_CHECK_RESET,
	IQS323_STATE_RUN,
} iqs323_state_e;

typedef enum {
        IQS323_CH0 = (uint8_t) 0x00,
        IQS323_CH1,
        IQS323_CH2,
} iqs323_channel_e;
typedef enum
{
        IQS323_CH_NONE = (uint8_t) 0x00,
        IQS323_CH_PROX,
        IQS323_CH_TOUCH,
        IQS323_CH_UNKNOWN,
} iqs323_ch_states;
typedef enum
{
        IQS323_NORMAL_POWER = (uint8_t) 0x00,
        IQS323_LOW_POWER,
        IQS323_ULP,
        IQS323_HALT,
        IQS323_POWER_UNKNOWN
} iqs323_power_modes;
typedef enum
{
        IQS323_GESTURE_NONE = (uint8_t) 0x00,
        IQS323_GESTURE_TAP,
        IQS323_GESTURE_SWIPE_POSITIVE,
        IQS323_GESTURE_SWIPE_NEGATIVE,
        IQS323_GESTURE_FLICK_POSITIVE,
        IQS323_GESTURE_FLICK_NEGATIVE,
        IQS323_GESTURE_HOLD,
        IQS323_GESTURE_UNKNOWN,
} iqs323_gesture_events;

/* IQS323 Memory map data variables, only save the data that might be used
during program runtime */
#pragma pack(1)
typedef struct
{
	/* READ ONLY */			//  I2C Addresses:
	uint8_t VERSION_DETAILS[20]; 	// 	0x00 -> 0x09
	uint8_t SYSTEM_STATUS[2];       // 	0x10
	uint8_t GESTURES[2]; 		// 	0x11
	uint8_t SLIDER_COORDINATES[2]; 	// 	0x12
	uint8_t CH0_COUNTS_LTA[4]; 	// 	0x13 -> 0x14
	uint8_t CH1_COUNTS_LTA[4]; 	// 	0x15 -> 0x16
	uint8_t CH2_COUNTS_LTA[4]; 	// 	0x17 -> 0x18

	/* READ WRITE */		//  I2C Addresses:
	uint8_t SYSTEM_CONTROL[2]; 	// 	0xC0
} IQS323_MEMORY_MAP;
#pragma pack(4)

#pragma pack(1)
typedef struct {
        iqs323_state_e        state;
        iqs323_init_e         init_state;
}iqs323_s;
#pragma pack(4)

// Class Prototype
class IQS323
{
public:
        // Public Constructors
        IQS323();

        // Public Device States
        iqs323_s iqs323_state;

        // Public Variables
        IQS323_MEMORY_MAP IQSMemoryMap;
        bool new_data_available;

        // Public Methods
        void begin(uint8_t deviceAddressIn, int sdaPinIn, int sclPinIn, uint8_t readyPinIn, bool to_init);
        bool init(void);
        void run(void);
        void queueValueUpdates(void);
        bool readATIactive(void);
        uint16_t getProductNum(bool stopOrRestart);
        uint8_t getmajorVersion(bool stopOrRestart);
        uint8_t getminorVersion(bool stopOrRestart);
        void acknowledgeReset(bool stopOrRestart);
        void ReATI(bool stopOrRestart);
        void SW_Reset(bool stopOrRestart);
        void device_reset();
        void writeMM(bool stopOrRestart);
        void clearRDY(void);
        bool getRDYStatus(void);

        void setEventMode(bool stopOrRestart);

        void updateInfoFlags(bool stopOrRestart);
        iqs323_power_modes get_PowerMode(void);
        bool checkReset(void);

        bool channel_touchState(iqs323_channel_e channel);
        bool channel_proxState(iqs323_channel_e channel);
        uint16_t sliderCoordinate(void);
        bool getSliderEvent(void);
        bool getGestureEvent(void);
        iqs323_gesture_events getGestureType(void);
        uint16_t readChannelCounts(iqs323_channel_e channel);
        uint16_t readChannelLTA(iqs323_channel_e channel);

        void force_I2C_communication(void);
        void read_gesture(void);

private:
        // Private Variables
        uint8_t _deviceAddress;

        // Private Methods
        void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        bool getBit(uint8_t data, uint8_t bit_number);
        uint8_t setBit(uint8_t data, uint8_t bit_number);
        uint8_t clearBit(uint8_t data, uint8_t bit_number);
};
#endif // IQS323_h
