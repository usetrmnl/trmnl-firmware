/******************************************************************************
SparkFunBQ27427.cpp
BQ27427 Arduino Library Main Source File

Adapted BQ27427 Library based on SparkFun BQ27441 Arduino Library
Original Author: Jim Lindblom @ SparkFun Electronics
Original Date: May 9, 2016
Original Repo: https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library

Adapted by: Edrean Ernst
Adaptation Date: June 2025
Repository: https://github.com/edreanernst/BQ27427_Arduino_Library

Implementation of all features of the BQ27427 Battery Fuel Gauge.

This library modifies the original SparkFun BQ27441 library to support the
Texas Instruments BQ27427 fuel gauge.

Original License: MIT License
See LICENSE.txt for full license terms.
******************************************************************************/

#include "Arduino.h"
#include <Wire.h>
#include "BQ27427.h"
#include "BQ27427_Definitions.h"

/*****************************************************************************
 ************************** Initialization Functions *************************
 *****************************************************************************/
// Initializes class variables
BQ27427::BQ27427() : _deviceAddress(BQ27427_I2C_ADDRESS), _sealFlag(false), _userConfigControl(false)
{
}

// Initializes I2C and verifies communication with the BQ27427.
bool BQ27427::begin(int sda, int scl)
{
	uint16_t deviceID = 0;
	
	Wire.begin(sda, scl); // Initialize I2C master
	
	deviceID = deviceType(); // Read deviceType from BQ27427
	
	if (deviceID == BQ27427_DEVICE_ID)
	{
		return true; // If device ID is valid, return true
	}
	
	return false; // Otherwise return false
}

// Configures the design capacity of the connected battery.
bool BQ27427::setCapacity(uint16_t capacity)
{
	// Write to STATE subclass (82) of BQ27427 extended memory.
	// Offset 0x06 (6)
	// Design capacity is a 2-byte piece of data - MSB first
	// Unit: mAh
	uint8_t capMSB = capacity >> 8;
	uint8_t capLSB = capacity & 0x00FF;
	uint8_t capacityData[2] = {capMSB, capLSB};
	return writeExtendedData(BQ27427_ID_STATE, 6, capacityData, 2);
}

// Get the design energy of the connected battery.
uint16_t BQ27427::designEnergy(void)
{
	return (readExtendedData(BQ27427_ID_STATE, 8) << 8 | readExtendedData(BQ27427_ID_STATE, 9));
}

// Configures the design energy of the connected battery.
bool BQ27427::setDesignEnergy(uint16_t energy)
{
	// Write to STATE subclass (82) of BQ27427 extended memory.
	// Offset 0x08 (8)
	// Design energy is a 2-byte piece of data - MSB first
	// Unit: mWh
	uint8_t enMSB = energy >> 8;
	uint8_t enLSB = energy & 0x00FF;
	uint8_t energyData[2] = {enMSB, enLSB};
	return writeExtendedData(BQ27427_ID_STATE, 8, energyData, 2);
}

// Get the terminate voltage of the connected battery.
uint16_t BQ27427::terminateVoltage(void)
{
	return (readExtendedData(BQ27427_ID_STATE, 10) << 8 | readExtendedData(BQ27427_ID_STATE, 11));
}

// Configures the terminate voltage.
bool BQ27427::setTerminateVoltage(uint16_t voltage)
{
	// Write to STATE subclass (82) of BQ27427 extended memory.
	// Offset 0x0A (10)
	// Terminate voltage is a 2-byte piece of data - MSB first
	// Unit: mV
	// Min 2500, Max 3700
	if(voltage<2500) voltage=2500;
	if(voltage>3700) voltage=3700;
	
	uint8_t tvMSB = voltage >> 8;
	uint8_t tvLSB = voltage & 0x00FF;
	uint8_t tvData[2] = {tvMSB, tvLSB};
	return writeExtendedData(BQ27427_ID_STATE, 10, tvData, 2);
}

// Get the discharge current threshold.
uint16_t BQ27427::dischargeCurrentThreshold(void)
{
	return (readExtendedData(BQ27427_ID_CURRENT_THRESH, 0) << 8 | readExtendedData(BQ27427_ID_CURRENT_THRESH, 1));
}

// Configures the discharge current threshold.
bool BQ27427::setdischargeCurrentThreshold(uint16_t value)
{
	// Write to STATE subclass (81) of BQ27427 extended memory.
	// Offset 0x00 (0)
	// Discharge current threshold is a 2-byte piece of data - MSB first
	// Unit: 0.1h
	// Min 0, Max 2000
	if(value>2000) value=2000;
	
	uint8_t dctMSB = value >> 8;
	uint8_t dctLSB = value & 0x00FF;
	uint8_t dctData[2] = {dctMSB, dctLSB};
	return writeExtendedData(BQ27427_ID_CURRENT_THRESH, 0, dctData, 2);
}

// Get the taper voltage of the connected battery.
uint16_t BQ27427::taperVoltage(void)
{
	return (readExtendedData(BQ27427_ID_CHEM_DATA, 8) << 8 | readExtendedData(BQ27427_ID_CHEM_DATA, 9));
}

// Configures the taper voltage.
bool BQ27427::setTaperVoltage(uint16_t voltage)
{
	// Write to STATE subclass (109) of BQ27427 extended memory.
	// Offset 0x08 (8)
	// Taper voltage is a 2-byte piece of data - MSB first
	// Unit: mV
	// Min 0, Max 5000
	if(voltage>5000) voltage=5000;
	
	uint8_t tvMSB = voltage >> 8;
	uint8_t tvLSB = voltage & 0x00FF;
	uint8_t tvData[2] = {tvMSB, tvLSB};
	return writeExtendedData(BQ27427_ID_CHEM_DATA, 8, tvData, 2);
}

// Get the taper rate of the connected battery.
uint16_t BQ27427::taperRate(void)
{
	return (readExtendedData(BQ27427_ID_STATE, 21) << 8 | readExtendedData(BQ27427_ID_STATE, 22));
}

// Configures taper rate of connected battery.
bool BQ27427::setTaperRate(uint16_t rate)
{
	// Write to STATE subclass (82) of BQ27427 extended memory.
	// Offset 0x15 (21)
	// Termiante voltage is a 2-byte piece of data - MSB first
	// Unit: 0.1h
	// Max 2000
	if(rate>2000) rate=2000;
	uint8_t trMSB = rate >> 8;
	uint8_t trLSB = rate & 0x00FF;
	uint8_t trData[2] = {trMSB, trLSB};
	return writeExtendedData(BQ27427_ID_STATE, 21, trData, 2);
}

// Read the polarity of the BQ27427 current measurement.
bool BQ27427::currentPolarity(void)
{
	uint8_t calBit0 = readExtendedData(BQ27427_ID_CC_CAL, 5); // Read CC_CAL[0] value
	return (calBit0 & 0x80); // Return bit 7 (0x80) value
}

// Changes the current polarity of the BQ27427.
bool BQ27427::changeCurrentPolarity(void) 
{
	uint8_t calBit0 = readExtendedData(BQ27427_ID_CC_CAL, 5); // Read CC_CAL[0] value
	calBit0 ^= 0x80; // Toggle bit 7 (0x80)
	uint8_t calData[1] = {calBit0};
	return writeExtendedData(BQ27427_ID_CC_CAL, 5, calData, 1);
}

/*****************************************************************************
 ********************** Battery Characteristics Functions ********************
 *****************************************************************************/

// Reads and returns the battery voltage
uint16_t BQ27427::voltage(void)
{
	return readWord(BQ27427_COMMAND_VOLTAGE);
}

// Reads and returns the specified current measurement
int16_t BQ27427::current(current_measure type)
{
	int16_t current = 0;
	switch (type)
	{
	case AVG:
		current = (int16_t) readWord(BQ27427_COMMAND_AVG_CURRENT);
		break;
	case STBY:
		current = (int16_t) readWord(BQ27427_COMMAND_STDBY_CURRENT);
		break;
	case MAX:
		current = (int16_t) readWord(BQ27427_COMMAND_MAX_CURRENT);
		break;
	}
	
	return current;
}

// Reads and returns the specified capacity measurement
uint16_t BQ27427::capacity(capacity_measure type)
{
	uint16_t capacity = 0;
	switch (type)
	{
	case REMAIN:
		return readWord(BQ27427_COMMAND_REM_CAPACITY);
		break;
	case FULL:
		return readWord(BQ27427_COMMAND_FULL_CAPACITY);
		break;
	case AVAIL:
		capacity = readWord(BQ27427_COMMAND_NOM_CAPACITY);
		break;
	case AVAIL_FULL:
		capacity = readWord(BQ27427_COMMAND_AVAIL_CAPACITY);
		break;
	case REMAIN_F: 
		capacity = readWord(BQ27427_COMMAND_REM_CAP_FIL);
		break;
	case REMAIN_UF:
		capacity = readWord(BQ27427_COMMAND_REM_CAP_UNFL);
		break;
	case FULL_F:
		capacity = readWord(BQ27427_COMMAND_FULL_CAP_FIL);
		break;
	case FULL_UF:
		capacity = readWord(BQ27427_COMMAND_FULL_CAP_UNFL);
		break;
	case DESIGN:
		capacity = (readExtendedData(BQ27427_ID_STATE, 6) << 8 | readExtendedData(BQ27427_ID_STATE, 7));
	}
	
	return capacity;
}

// Reads and returns measured average power
int16_t BQ27427::power(void)
{
	return (int16_t) readWord(BQ27427_COMMAND_AVG_POWER);
}

// Reads and returns specified state of charge measurement
uint16_t BQ27427::soc(soc_measure type)
{
	uint16_t socRet = 0;
	switch (type)
	{
	case FILTERED:
		socRet = readWord(BQ27427_COMMAND_SOC);
		break;
	case UNFILTERED:
		socRet = readWord(BQ27427_COMMAND_SOC_UNFL);
		break;
	}
	
	return socRet;
}

// Reads and returns specified state of health measurement
uint8_t BQ27427::soh(soh_measure type)
{
	uint16_t sohRaw = readWord(BQ27427_COMMAND_SOH);
	uint8_t sohStatus = sohRaw >> 8;
	uint8_t sohPercent = sohRaw & 0x00FF;
	
	if (type == PERCENT)	
		return sohPercent;
	else
		return sohStatus;
}

// Reads and returns specified temperature measurement
uint16_t BQ27427::temperature(temp_measure type)
{
	uint16_t temp = 0;
	switch (type)
	{
	case BATTERY:
		temp = readWord(BQ27427_COMMAND_TEMP);
		break;
	case INTERNAL_TEMP:
		temp = readWord(BQ27427_COMMAND_INT_TEMP);
		break;
	}
	return temp;
}

/*****************************************************************************
 ************************** GPOUT Control Functions **************************
 *****************************************************************************/
// Get GPOUT polarity setting (active-high or active-low)
bool BQ27427::GPOUTPolarity(void)
{
	uint16_t opConfigRegister = opConfig();
	
	return (opConfigRegister & BQ27427_OPCONFIG_GPIOPOL);
}

// Set GPOUT polarity to active-high or active-low
bool BQ27427::setGPOUTPolarity(bool activeHigh)
{
	uint16_t oldOpConfig = opConfig();
	
	// Check to see if we need to update opConfig:
	if ((activeHigh && (oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)) ||
        (!activeHigh && !(oldOpConfig & BQ27427_OPCONFIG_GPIOPOL)))
		return true;
		
	uint16_t newOpConfig = oldOpConfig;
	if (activeHigh)
		newOpConfig |= BQ27427_OPCONFIG_GPIOPOL;
	else
		newOpConfig &= ~(BQ27427_OPCONFIG_GPIOPOL);
	
	return writeOpConfig(newOpConfig);	
}

// Get GPOUT function (BAT_LOW or SOC_INT)
bool BQ27427::GPOUTFunction(void)
{
	uint16_t opConfigRegister = opConfig();
	
	return (opConfigRegister & BQ27427_OPCONFIG_BATLOWEN);	
}

// Set GPOUT function to BAT_LOW or SOC_INT
bool BQ27427::setGPOUTFunction(gpout_function function)
{
	uint16_t oldOpConfig = opConfig();
	
	// Check to see if we need to update opConfig:
	if ((function && (oldOpConfig & BQ27427_OPCONFIG_BATLOWEN)) ||
        (!function && !(oldOpConfig & BQ27427_OPCONFIG_BATLOWEN)))
		return true;
	
	// Modify BATLOWN_EN bit of opConfig:
	uint16_t newOpConfig = oldOpConfig;
	if (function)
		newOpConfig |= BQ27427_OPCONFIG_BATLOWEN;
	else
		newOpConfig &= ~(BQ27427_OPCONFIG_BATLOWEN);

	// Write new opConfig
	return writeOpConfig(newOpConfig);	
}

// Get SOC1_Set Threshold - threshold to set the alert flag
uint8_t BQ27427::SOC1SetThreshold(void)
{
	return readExtendedData(BQ27427_ID_DISCHARGE, 0);
}

// Get SOC1_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27427::SOC1ClearThreshold(void)
{
	return readExtendedData(BQ27427_ID_DISCHARGE, 1);	
}

// Set the SOC1 set and clear thresholds to a percentage
bool BQ27427::setSOC1Thresholds(uint8_t set, uint8_t clear)
{
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(BQ27427_ID_DISCHARGE, 0, thresholds, 2);
}

// Get SOCF_Set Threshold - threshold to set the alert flag
uint8_t BQ27427::SOCFSetThreshold(void)
{
	return readExtendedData(BQ27427_ID_DISCHARGE, 2);
}

// Get SOCF_Clear Threshold - threshold to clear the alert flag
uint8_t BQ27427::SOCFClearThreshold(void)
{
	return readExtendedData(BQ27427_ID_DISCHARGE, 3);	
}

// Set the SOCF set and clear thresholds to a percentage
bool BQ27427::setSOCFThresholds(uint8_t set, uint8_t clear)
{
	uint8_t thresholds[2];
	thresholds[0] = constrain(set, 0, 100);
	thresholds[1] = constrain(clear, 0, 100);
	return writeExtendedData(BQ27427_ID_DISCHARGE, 2, thresholds, 2);
}

// Check if the SOC1 flag is set
bool BQ27427::socFlag(void)
{
	uint16_t flagState = flags();
	
	return flagState & BQ27427_FLAG_SOC1;
}

// Check if the SOCF flag is set
bool BQ27427::socfFlag(void)
{
	uint16_t flagState = flags();
	
	return flagState & BQ27427_FLAG_SOCF;
	
}

// Check if the ITPOR flag is set
bool BQ27427::itporFlag(void)
{
	uint16_t flagState = flags();
	
	return flagState & BQ27427_FLAG_ITPOR;
}

// Check if the FC flag is set
bool BQ27427::fcFlag(void)
{
	uint16_t flagState = flags();
	
	return flagState & BQ27427_FLAG_FC;
}

// Check if the CHG flag is set
bool BQ27427::chgFlag(void)
{
	uint16_t flagState = flags();
	
	return flagState & BQ27427_FLAG_CHG;
}

// Check if the DSG flag is set
bool BQ27427::dsgFlag(void)
{
	uint16_t flagState = flags();
	
	return flagState & BQ27427_FLAG_DSG;
}

// Get the SOC_INT interval delta
uint8_t BQ27427::sociDelta(void)
{
	return readExtendedData(BQ27427_ID_STATE, 26);
}

// Set the SOC_INT interval delta to a value between 1 and 100
bool BQ27427::setSOCIDelta(uint8_t delta)
{
	uint8_t soci = constrain(delta, 0, 100);
	return writeExtendedData(BQ27427_ID_STATE, 26, &soci, 1);
}

// Pulse the GPOUT pin - must be in SOC_INT mode
bool BQ27427::pulseGPOUT(void)
{
	return executeControlWord(BQ27427_CONTROL_PULSE_SOC_INT);
}

/*****************************************************************************
 *************************** Control Sub-Commands ****************************
 *****************************************************************************/

// Read the device type - should be 0x0427
uint16_t BQ27427::deviceType(void)
{
	return readControlWord(BQ27427_CONTROL_DEVICE_TYPE);
}

// Configures the chemistry profile of the connected battery.
bool BQ27427::setChemID(chemistry_profiles chem_id)
{
	// if (!_userConfigControl) 
	// {
	// 	if(!enterConfig())
	// 		return false; // If we can't enter config mode, return false
	// }

	if (sealed())
	{
		_sealFlag = true;
		unseal(); // Must be unsealed before making changes
	}

	uint16_t old_chem_id = readControlWord(BQ27427_CONTROL_CHEM_ID);

	if (executeControlWord(BQ27427_CONTROL_SET_CFGUPDATE))
	{
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && (!(flags() & BQ27427_FLAG_CFGUPMODE)))
			delay(1);
		
		if (timeout > 0)
		{
			if (executeControlWord(chem_id)) {
				delay(100); // Wait for the BQ27427 to process the command
				if (softReset())
				{
					int16_t timeout = BQ72441_I2C_TIMEOUT;
					while ((timeout--) && ((flags() & BQ27427_FLAG_CFGUPMODE)))
						delay(1);
					if (timeout > 0)
					{
						uint16_t new_chem_id = readControlWord(BQ27427_CONTROL_CHEM_ID);
						if (new_chem_id != old_chem_id) {
							if (_sealFlag) seal(); // Seal back up if we IC was sealed coming in
							return true;
						}
						return false;
					}
				}
				return false;
			} else {
				return false;
			}
		}
	}
	return false;
}

// Reads and returns the battery chemistry profile.
chemistry_profiles BQ27427::chemID(void) 
{
	uint16_t chem_id = readControlWord(BQ27427_CONTROL_CHEM_ID);
	return static_cast<chemistry_profiles>(chem_id);
}

// Enter configuration mode - set userControl if calling from an Arduino sketch
// and you want control over when to exitConfig
bool BQ27427::enterConfig(bool userControl)
{
	if (userControl) _userConfigControl = true;
	
	if (sealed())
	{
		_sealFlag = true;
		unseal(); // Must be unsealed before making changes
	}
	
	if (executeControlWord(BQ27427_CONTROL_SET_CFGUPDATE))
	{
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && (!(flags() & BQ27427_FLAG_CFGUPMODE)))
			delay(1);
		
		if (timeout > 0)
			return true;
	}
	
	return false;
}

// Exit configuration mode
bool BQ27427::exitConfig(bool userControl)
{
	if (userControl) _userConfigControl = false;

	if (softReset())
	{
		int16_t timeout = BQ72441_I2C_TIMEOUT;
		while ((timeout--) && ((flags() & BQ27427_FLAG_CFGUPMODE)))
			delay(1);
		if (timeout > 0)
		{
			if (_sealFlag) seal(); // Seal back up if we IC was sealed coming in
			return true;
		}
	}
	return false;
}

// Read the flags() command
uint16_t BQ27427::flags(void)
{
	return readWord(BQ27427_COMMAND_FLAGS);
}

// Read the CONTROL_STATUS subcommand of control()
uint16_t BQ27427::status(void)
{
	return readControlWord(BQ27427_CONTROL_STATUS);
}

// Issue a factory reset to the BQ27427
bool BQ27427::reset(void)
{
	if (!_userConfigControl) enterConfig(false); // Enter config mode if not already in it
	
	if (executeControlWord(BQ27427_CONTROL_RESET))
	{
		if (!_userConfigControl) exitConfig();
		return true;
	} 
	else 
	{
		return false;
	}
}

/***************************** Private Functions *****************************/

// Check if the BQ27427 is sealed or not.
bool BQ27427::sealed(void)
{
	uint16_t stat = status();
	return stat & BQ27427_STATUS_SS;
}

// Seal the BQ27427
bool BQ27427::seal(void)
{
	return readControlWord(BQ27427_CONTROL_SEALED);
}

// UNseal the BQ27427
bool BQ27427::unseal(void)
{
	// To unseal the BQ27427, write the key to the control
	// command. Then immediately write the same key to control again.
	if (readControlWord(BQ27427_UNSEAL_KEY))
	{
		return readControlWord(BQ27427_UNSEAL_KEY);
	}
	return false;
}

// Read the 16-bit opConfig register from extended data
uint16_t BQ27427::opConfig(void)
{
	return readExtendedData(BQ27427_ID_REGISTERS, 0);
}

// Write the 16-bit opConfig register in extended data
bool BQ27427::writeOpConfig(uint16_t value)
{
	uint8_t opConfigMSB = value >> 8;
	uint8_t opConfigLSB = value & 0x00FF;
	uint8_t opConfigData[2] = {opConfigMSB, opConfigLSB};
	
	// OpConfig register location: BQ27427_ID_REGISTERS id, offset 0
	return writeExtendedData(BQ27427_ID_REGISTERS, 0, opConfigData, 2);	
}

// Issue a soft-reset to the BQ27427
bool BQ27427::softReset(void)
{
	return executeControlWord(BQ27427_CONTROL_SOFT_RESET);
}

// Read a 16-bit command word from the BQ27427
uint16_t BQ27427::readWord(uint16_t subAddress)
{
	uint8_t data[2];
	i2cReadBytes(subAddress, data, 2);
	return ((uint16_t) data[1] << 8) | data[0];
}

// Read a 16-bit subcommand() from the BQ27427's control()
uint16_t BQ27427::readControlWord(uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};
	
	i2cWriteBytes((uint8_t) 0, command, 2);
	
	if (i2cReadBytes((uint8_t) 0, data, 2))
	{
		return ((uint16_t)data[1] << 8) | data[0];
	}
	
	return false;
}

// Execute a subcommand() from the BQ27427's control()
bool BQ27427::executeControlWord(uint16_t function)
{
	uint8_t subCommandMSB = (function >> 8);
	uint8_t subCommandLSB = (function & 0x00FF);
	uint8_t command[2] = {subCommandLSB, subCommandMSB};
	uint8_t data[2] = {0, 0};
	
	if (i2cWriteBytes((uint8_t) 0, command, 2))
		return true;
	
	return false;
}

/*****************************************************************************
 ************************** Extended Data Commands ***************************
 *****************************************************************************/
 
// Issue a BlockDataControl() command to enable BlockData access
bool BQ27427::blockDataControl(void)
{
	uint8_t enableByte = 0x00;
	return i2cWriteBytes(BQ27427_EXTENDED_CONTROL, &enableByte, 1);
}

// Issue a DataClass() command to set the data class to be accessed
bool BQ27427::blockDataClass(uint8_t id)
{
	return i2cWriteBytes(BQ27427_EXTENDED_DATACLASS, &id, 1);
}

// Issue a DataBlock() command to set the data block to be accessed
bool BQ27427::blockDataOffset(uint8_t offset)
{
	return i2cWriteBytes(BQ27427_EXTENDED_DATABLOCK, &offset, 1);
}

// Read the current checksum using BlockDataCheckSum()
uint8_t BQ27427::blockDataChecksum(void)
{
	uint8_t csum;
	i2cReadBytes(BQ27427_EXTENDED_CHECKSUM, &csum, 1);
	return csum;
}

// Use BlockData() to read a byte from the loaded extended data
uint8_t BQ27427::readBlockData(uint8_t offset)
{
	uint8_t ret;
	uint8_t address = offset + BQ27427_EXTENDED_BLOCKDATA;
	i2cReadBytes(address, &ret, 1);
	return ret;
}

// Use BlockData() to write a byte to an offset of the loaded data
bool BQ27427::writeBlockData(uint8_t offset, uint8_t data)
{
	uint8_t address = offset + BQ27427_EXTENDED_BLOCKDATA;
	return i2cWriteBytes(address, &data, 1);
}

// Read all 32 bytes of the loaded extended data and compute a 
// checksum based on the values.
uint8_t BQ27427::computeBlockChecksum(void)
{
	uint8_t data[32];
	i2cReadBytes(BQ27427_EXTENDED_BLOCKDATA, data, 32);

	uint8_t csum = 0;
	for (int i=0; i<32; i++)
	{
		csum += data[i];
	}
	csum = 255 - csum;
	
	return csum;
}

// Use the BlockDataCheckSum() command to write a checksum value
bool BQ27427::writeBlockChecksum(uint8_t csum)
{
	return i2cWriteBytes(BQ27427_EXTENDED_CHECKSUM, &csum, 1);	
}

// Read a byte from extended data specifying a class ID and position offset
uint8_t BQ27427::readExtendedData(uint8_t classID, uint8_t offset)
{
	uint8_t retData = 0;
	if (!_userConfigControl) enterConfig(false);
		
	if (!blockDataControl()) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)
	
	computeBlockChecksum(); // Compute checksum going in
	uint8_t oldCsum = blockDataChecksum();
	/*for (int i=0; i<32; i++)
		Serial.print(String(readBlockData(i)) + " ");*/
	retData = readBlockData(offset % 32); // Read from offset (limit to 0-31)
	
	if (!_userConfigControl) exitConfig();
	
	return retData;
}

// Write a specified number of bytes to extended data specifying a 
// class ID, position offset.
bool BQ27427::writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len)
{
	if (len > 32)
		return false;
	
	if (!_userConfigControl)
	{
		if (!enterConfig(false)) 
			return false; // Return false if enterConfig fails
	}
	
	if (!blockDataControl()) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(offset / 32); // Write 32-bit block offset (usually 0)
	computeBlockChecksum(); // Compute checksum going in
	uint8_t oldCsum = blockDataChecksum();

	// Write data bytes:
	for (int i = 0; i < len; i++)
	{
		// Write to offset, mod 32 if offset is greater than 32
		// The blockDataOffset above sets the 32-bit block
		if (!writeBlockData((offset % 32) + i, data[i]))
			return false; // Return false if writeBlockData fails
	}
	
	// Write new checksum using BlockDataChecksum (0x60)
	uint8_t newCsum = computeBlockChecksum(); // Compute the new checksum
	if (!writeBlockChecksum(newCsum))
		return false; // Return false if checksum write fails

	if (!_userConfigControl)
	{
		delay(10); // Wait for BQ27427 to process the write
		if (!exitConfig())
			return false; // Return false if exitConfig fails
	}
	
	return true;
}

/*****************************************************************************
 ************************ I2C Read and Write Routines ************************
 *****************************************************************************/

// Read a specified number of bytes over I2C at a given subAddress
int16_t BQ27427::i2cReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	int16_t timeout = BQ72441_I2C_TIMEOUT;	
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	Wire.endTransmission(true);
	
	Wire.requestFrom(_deviceAddress, count);
	
	for (int i=0; i<count; i++)
	{
		dest[i] = Wire.read();
	}
	
	return timeout;
}

// Write a specified number of bytes over I2C to a given subAddress
uint16_t BQ27427::i2cWriteBytes(uint8_t subAddress, uint8_t * src, uint8_t count)
{
	Wire.beginTransmission(_deviceAddress);
	Wire.write(subAddress);
	for (int i=0; i<count; i++)
	{
		Wire.write(src[i]);
	}	
	Wire.endTransmission(true);
	
	return true;	
}

BQ27427 lipo; // Use lipo.[] to interact with the library in an Arduino sketch