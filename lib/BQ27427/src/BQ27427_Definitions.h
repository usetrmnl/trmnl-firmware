/******************************************************************************
BQ27427_Definitions.h
BQ27427 Battery Fuel Gauge Definitions

Adapted BQ27427 Library based on SparkFun BQ27441 Arduino Library
Original Author: Jim Lindblom @ SparkFun Electronics
Original Date: May 9, 2016
Original Repo: https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Adapted by: Edrean Ernst
Adaptation Date: June 2025
Repository: https://github.com/edreanernst/BQ27427_Arduino_Library

BQ27441 hardware constants, register addresses, and bit positions.

This library modifies the original SparkFun BQ27441 library to support the
Texas Instruments BQ27427 fuel gauge.

Original License: MIT License
See LICENSE.md for full license terms.
******************************************************************************/

#define BQ27427_I2C_ADDRESS 0x55 // Default I2C address of the BQ27427-G1A

///////////////////////
// General Constants //
///////////////////////
#define BQ27427_UNSEAL_KEY	0x8000 // Secret code to unseal the BQ27427-G1A
#define BQ27427_DEVICE_ID	0x0427 // Default device ID

///////////////////////
// Standard Commands //
///////////////////////
// The fuel gauge uses a series of 2-byte standard commands to enable system 
// reading and writing of battery information. Each command has an associated
// sequential command-code pair.
#define BQ27427_COMMAND_CONTROL			0x00 // Control()
#define BQ27427_COMMAND_TEMP			0x02 // Temperature()
#define BQ27427_COMMAND_VOLTAGE			0x04 // Voltage()
#define BQ27427_COMMAND_FLAGS			0x06 // Flags()
#define BQ27427_COMMAND_NOM_CAPACITY	0x08 // NominalAvailableCapacity()
#define BQ27427_COMMAND_AVAIL_CAPACITY	0x0A // FullAvailableCapacity()
#define BQ27427_COMMAND_REM_CAPACITY	0x0C // RemainingCapacity()
#define BQ27427_COMMAND_FULL_CAPACITY	0x0E // FullChargeCapacity()
#define BQ27427_COMMAND_AVG_CURRENT		0x10 // AverageCurrent()
#define BQ27427_COMMAND_STDBY_CURRENT	0x12 // StandbyCurrent()
#define BQ27427_COMMAND_MAX_CURRENT		0x14 // MaxLoadCurrent()
#define BQ27427_COMMAND_AVG_POWER		0x18 // AveragePower()
#define BQ27427_COMMAND_SOC				0x1C // StateOfCharge()
#define BQ27427_COMMAND_INT_TEMP		0x1E // InternalTemperature()
#define BQ27427_COMMAND_SOH				0x20 // StateOfHealth()
#define BQ27427_COMMAND_REM_CAP_UNFL	0x28 // RemainingCapacityUnfiltered()
#define BQ27427_COMMAND_REM_CAP_FIL		0x2A // RemainingCapacityFiltered()
#define BQ27427_COMMAND_FULL_CAP_UNFL	0x2C // FullChargeCapacityUnfiltered()
#define BQ27427_COMMAND_FULL_CAP_FIL	0x2E // FullChargeCapacityFiltered()
#define BQ27427_COMMAND_SOC_UNFL		0x30 // StateOfChargeUnfiltered()

//////////////////////////
// Control Sub-commands //
//////////////////////////
// Issuing a Control() command requires a subsequent 2-byte subcommand. These
// additional bytes specify the particular control function desired. The 
// Control() command allows the system to control specific features of the fuel
// gauge during normal operation and additional features when the device is in 
// different access modes.
#define BQ27427_CONTROL_STATUS			0x00
#define BQ27427_CONTROL_DEVICE_TYPE		0x01
#define BQ27427_CONTROL_FW_VERSION		0x02
#define BQ27427_CONTROL_DM_CODE			0x04
#define BQ27427_CONTROL_PREV_MACWRITE	0x07
#define BQ27427_CONTROL_CHEM_ID			0x08
#define BQ27427_CONTROL_BAT_INSERT		0x0C
#define BQ27427_CONTROL_BAT_REMOVE		0x0D
#define BQ27427_CONTROL_SET_CFGUPDATE	0x13
#define BQ27427_CONTROL_SHUTDOWN_ENABLE	0x1B
#define BQ27427_CONTROL_SHUTDOWN		0x1C
#define BQ27427_CONTROL_SEALED			0x20
#define BQ27427_CONTROL_PULSE_SOC_INT	0x23
#define BQ27427_CONTROL_CHEM_A          0x30
#define BQ27427_CONTROL_CHEM_B          0x31
#define BQ27427_CONTROL_CHEM_C          0x32
#define BQ27427_CONTROL_RESET			0x41
#define BQ27427_CONTROL_SOFT_RESET		0x42

///////////////////////////////////////////
// Control Status Word - Bit Definitions //
///////////////////////////////////////////
// Bit positions for the 16-bit data of CONTROL_STATUS.
// CONTROL_STATUS instructs the fuel gauge to return status information to 
// Control() addresses 0x00 and 0x01. The read-only status word contains status
// bits that are set or cleared either automatically as conditions warrant or
// through using specified subcommands.
#define BQ27427_STATUS_SHUTDOWNEN	(1<<15)
#define BQ27427_STATUS_WDRESET		(1<<14)
#define BQ27427_STATUS_SS			(1<<13)
#define BQ27427_STATUS_CALMODE		(1<<12)
#define BQ27427_STATUS_CCA			(1<<11)
#define BQ27427_STATUS_BCA			(1<<10)
#define BQ27427_STATUS_QMAX_UP		(1<<9)
#define BQ27427_STATUS_RES_UP		(1<<8)
#define BQ27427_STATUS_INITCOMP		(1<<7)
#define BQ27427_STATUS_SLEEP		(1<<4)
#define BQ27427_STATUS_LDMD			(1<<3)
#define BQ27427_STATUS_RUP_DIS		(1<<2)
#define BQ27427_STATUS_VOK			(1<<1)

////////////////////////////////////
// Flag Command - Bit Definitions //
////////////////////////////////////
// Bit positions for the 16-bit data of Flags()
// This read-word function returns the contents of the fuel gauging status
// register, depicting the current operating status.
#define BQ27427_FLAG_OT			(1<<15)
#define BQ27427_FLAG_UT			(1<<14)
#define BQ27427_FLAG_FC			(1<<9)
#define BQ27427_FLAG_CHG		(1<<8)
#define BQ27427_FLAG_OCVTAKEN	(1<<7)
#define BQ27427_FLAG_DOD_CRRCT	(1<<6)
#define BQ27427_FLAG_ITPOR		(1<<5)
#define BQ27427_FLAG_CFGUPMODE	(1<<4)
#define BQ27427_FLAG_BAT_DET	(1<<3)
#define BQ27427_FLAG_SOC1		(1<<2)
#define BQ27427_FLAG_SOCF		(1<<1)
#define BQ27427_FLAG_DSG		(1<<0)

////////////////////////////
// Extended Data Commands //
////////////////////////////
// Extended data commands offer additional functionality beyond the standard
// set of commands. They are used in the same manner; however, unlike standard
// commands, extended commands are not limited to 2-byte words.
#define BQ27427_EXTENDED_DATACLASS	0x3E // DataClass()
#define BQ27427_EXTENDED_DATABLOCK	0x3F // DataBlock()
#define BQ27427_EXTENDED_BLOCKDATA	0x40 // BlockData()
#define BQ27427_EXTENDED_CHECKSUM	0x60 // BlockDataCheckSum()
#define BQ27427_EXTENDED_CONTROL	0x61 // BlockDataControl()

////////////////////////////////////////
// Configuration Class, Subclass ID's //
////////////////////////////////////////
// To access a subclass of the extended data, set the DataClass() function
// with one of these values.
// Configuration Classes
#define BQ27427_ID_SAFETY			2   // Safety
#define BQ27427_ID_CHG_TERMINATION	36  // Charge Termination
// #define BQ27427_ID_CONFIG_DATA		48  // Data
#define BQ27427_ID_DISCHARGE		49  // Discharge
#define BQ27427_ID_REGISTERS		64  // Registers
// #define BQ27427_ID_POWER			68  // Power
// Gas Gauging Classes
#define BQ27427_ID_IT_CFG			80  // IT Cfg
#define BQ27427_ID_CURRENT_THRESH	81  // Current Thresholds
#define BQ27427_ID_STATE			82  // State
// Ra Tables Classes
#define BQ27427_ID_R_A_RAM			89  // R_a RAM
// Chemistry Info Classes
#define BQ27427_ID_CHEM_DATA        109  // Chem Data
// Calibration Classes
#define BQ27427_ID_CALIB_DATA		104 // Data
#define BQ27427_ID_CC_CAL			105 // CC Cal
#define BQ27427_ID_CURRENT			107 // Current
// Security Classes
#define BQ27427_ID_CODES			112 // Codes

/////////////////////////////////////////
// OpConfig Register - Bit Definitions //
/////////////////////////////////////////
// Bit positions of the OpConfig Register
#define BQ27427_OPCONFIG_BIE        (1<<13)
#define BQ27427_OPCONFIG_GPIOPOL    (1<<11)
#define BQ27427_OPCONFIG_RS_FCT_STP (1<<6)
#define BQ27427_OPCONFIG_SLEEP      (1<<5)
#define BQ27427_OPCONFIG_RMFCC      (1<<4)
#define BQ27427_OPCONFIG_FASTCNV_EN (1<<3)
#define BQ27427_OPCONFIG_BATLOWEN   (1<<2)
#define BQ27427_OPCONFIG_TEMPS      (1<<0)