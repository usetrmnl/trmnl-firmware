//
//    FILE: TCA9555.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.4.3
// PURPOSE: Arduino library for I2C TCA9555 16 channel port expander
//    DATE: 2021-06-09
//     URL: https://github.com/RobTillaart/TCA9555

#include "TCA9555.h"

#define TCA9555_INPUT_PORT_REGISTER_0     0x00
#define TCA9555_INPUT_PORT_REGISTER_1     0x01
#define TCA9555_OUTPUT_PORT_REGISTER_0    0x02
#define TCA9555_OUTPUT_PORT_REGISTER_1    0x03
#define TCA9555_POLARITY_REGISTER_0       0x04
#define TCA9555_POLARITY_REGISTER_1       0x05
#define TCA9555_CONFIGURATION_PORT_0      0x06
#define TCA9555_CONFIGURATION_PORT_1      0x07

TCA9555::TCA9555(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire    = wire;
  _error   = TCA9555_OK;
  _type    = 55;
}

bool TCA9555::begin(uint8_t mode, uint16_t mask)
{
  if ((_address < 0x20) || (_address > 0x27)) return false;
  if (!isConnected()) return false;
  if (mode == OUTPUT) { pinMode16(0x0000); write16(mask); }
  else                { pinMode16(0xFFFF); }
  return true;
}

bool TCA9555::isConnected()
{
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}

uint8_t TCA9555::getAddress() { return _address; }

bool TCA9555::pinMode1(uint8_t pin, uint8_t mode)
{
  if (pin > 15) { _error = TCA9555_PIN_ERROR; return false; }
  if ((mode != INPUT) && (mode != OUTPUT)) { _error = TCA9555_VALUE_ERROR; return false; }
  uint8_t CONFREG = (pin > 7) ? TCA9555_CONFIGURATION_PORT_1 : TCA9555_CONFIGURATION_PORT_0;
  if (pin > 7) pin -= 8;
  uint8_t val = readRegister(CONFREG);
  uint8_t prev = val;
  uint8_t mask = 1 << pin;
  if (mode == INPUT) val |= mask; else val &= ~mask;
  if (val != prev) return writeRegister(CONFREG, val);
  _error = TCA9555_OK;
  return true;
}

bool TCA9555::write1(uint8_t pin, uint8_t value)
{
  if (pin > 15) { _error = TCA9555_PIN_ERROR; return false; }
  uint8_t OPR = (pin > 7) ? TCA9555_OUTPUT_PORT_REGISTER_1 : TCA9555_OUTPUT_PORT_REGISTER_0;
  if (pin > 7) pin -= 8;
  uint8_t val = readRegister(OPR);
  uint8_t prev = val;
  uint8_t mask = 1 << pin;
  if (value) val |= mask; else val &= ~mask;
  if (val != prev) return writeRegister(OPR, val);
  _error = TCA9555_OK;
  return true;
}

uint8_t TCA9555::read1(uint8_t pin)
{
  if (pin > 15) { _error = TCA9555_PIN_ERROR; return (uint8_t)TCA9555_INVALID_READ; }
  uint8_t IPR = (pin > 7) ? TCA9555_INPUT_PORT_REGISTER_1 : TCA9555_INPUT_PORT_REGISTER_0;
  if (pin > 7) pin -= 8;
  uint8_t val = readRegister(IPR);
  _error = TCA9555_OK;
  return (val & (1 << pin)) ? HIGH : LOW;
}

bool TCA9555::setPolarity(uint8_t pin, uint8_t value)
{
  if (pin > 15) { _error = TCA9555_PIN_ERROR; return false; }
  if ((value != LOW) && (value != HIGH)) { _error = TCA9555_VALUE_ERROR; return false; }
  uint8_t POLREG = (pin > 7) ? TCA9555_POLARITY_REGISTER_1 : TCA9555_POLARITY_REGISTER_0;
  if (pin > 7) pin -= 8;
  uint8_t val = readRegister(POLREG);
  uint8_t prev = val;
  uint8_t mask = 1 << pin;
  if (value == HIGH) val |= mask; else val &= ~mask;
  if (val != prev) return writeRegister(POLREG, val);
  _error = TCA9555_OK;
  return true;
}

uint8_t TCA9555::getPolarity(uint8_t pin)
{
  if (pin > 15) { _error = TCA9555_PIN_ERROR; return 0; }
  uint8_t POLREG = (pin > 7) ? TCA9555_POLARITY_REGISTER_1 : TCA9555_POLARITY_REGISTER_0;
  _error = TCA9555_OK;
  return (readRegister(POLREG) >> (pin & 7)) & 1;
}

bool TCA9555::pinMode8(uint8_t port, uint8_t mask)
{
  if (port > 1) { _error = TCA9555_PORT_ERROR; return false; }
  _error = TCA9555_OK;
  return writeRegister(port == 0 ? TCA9555_CONFIGURATION_PORT_0 : TCA9555_CONFIGURATION_PORT_1, mask);
}

bool TCA9555::write8(uint8_t port, uint8_t mask)
{
  if (port > 1) { _error = TCA9555_PORT_ERROR; return false; }
  _error = TCA9555_OK;
  return writeRegister(port == 0 ? TCA9555_OUTPUT_PORT_REGISTER_0 : TCA9555_OUTPUT_PORT_REGISTER_1, mask);
}

int TCA9555::read8(uint8_t port)
{
  if (port > 1) { _error = TCA9555_PORT_ERROR; return TCA9555_INVALID_READ; }
  _error = TCA9555_OK;
  return readRegister(port == 0 ? TCA9555_INPUT_PORT_REGISTER_0 : TCA9555_INPUT_PORT_REGISTER_1);
}

bool TCA9555::setPolarity8(uint8_t port, uint8_t mask)
{
  if (port > 1) { _error = TCA9555_PORT_ERROR; return false; }
  _error = TCA9555_OK;
  return writeRegister(port == 0 ? TCA9555_POLARITY_REGISTER_0 : TCA9555_POLARITY_REGISTER_1, mask);
}

uint8_t TCA9555::getPolarity8(uint8_t port)
{
  if (port > 1) { _error = TCA9555_PORT_ERROR; return 0; }
  _error = TCA9555_OK;
  return readRegister(port == 0 ? TCA9555_POLARITY_REGISTER_0 : TCA9555_POLARITY_REGISTER_1);
}

bool TCA9555::pinMode16(uint16_t mask)
{
  return pinMode8(0, mask & 0xFF) && pinMode8(1, mask >> 8);
}

bool TCA9555::write16(uint16_t mask)
{
  return write8(0, mask & 0xFF) && write8(1, mask >> 8);
}

uint16_t TCA9555::read16()
{
  return (uint16_t)read8(0) | ((uint16_t)read8(1) << 8);
}

bool TCA9555::setPolarity16(uint16_t mask)
{
  return setPolarity8(0, mask & 0xFF) && setPolarity8(1, mask >> 8);
}

uint8_t TCA9555::getPolarity16()
{
  return getPolarity8(0) | (getPolarity8(1) << 8);
}

int TCA9555::lastError()
{
  int e = _error;
  _error = TCA9555_OK;
  return e;
}

uint8_t TCA9555::getType() { return _type; }

bool TCA9555::writeRegister(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  if (_wire->endTransmission() != 0) { _error = TCA9555_I2C_ERROR; return false; }
  _error = TCA9555_OK;
  return true;
}

uint8_t TCA9555::readRegister(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  int rv = _wire->endTransmission();
  if (rv != 0) { _error = TCA9555_I2C_ERROR; return (uint8_t)rv; }
  _error = TCA9555_OK;
  _wire->requestFrom(_address, (uint8_t)1);
  return _wire->read();
}

TCA9535::TCA9535(uint8_t address, TwoWire *wire) : TCA9555(address, wire) { _type = 35; }
PCA9555::PCA9555(uint8_t address, TwoWire *wire) : TCA9555(address, wire) { _type = 55; }
PCA9535::PCA9535(uint8_t address, TwoWire *wire) : TCA9555(address, wire) { _type = 35; }
CAT9555::CAT9555(uint8_t address, TwoWire *wire) : TCA9555(address, wire) { _type = 55; }
