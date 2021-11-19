#ifndef SENSOR_H_
#define SENSOR_H
#include <Arduino.h>


uint8_t sensor_readRegister(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* ptr);
uint8_t sensor_writeRegister(uint8_t address, uint8_t reg, uint8_t bytes, uint8_t* ptr);


#endif