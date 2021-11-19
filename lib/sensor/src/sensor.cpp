#include "sensor.h"
#include <Arduino.h>
#include <Wire.h>

uint8_t sensor_readRegister(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* ptr)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  int retVal = Wire.endTransmission(false);
  Wire.requestFrom(address, bytes, true);
  for(int i = 0; i < bytes; i++)
    ptr[i] = Wire.read();
  return retVal;
}

uint8_t sensor_writeRegister(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* ptr)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  for(int i = 0; i < bytes; i++)
    Wire.write(ptr[i]);
  int retVal = Wire.endTransmission();
  return retVal;
}