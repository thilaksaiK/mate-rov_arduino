#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <sensor.h>

#define WHO_AM_I  0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define STATUS_REG 0x27
#define OUT_X_L 0x28

#define READ (1 << 7)

class gyroscope
{
private:
    /* data */
    uint8_t _address;
public:
    uint8_t init(uint8_t address);
    uint8_t readData(int16_t& x, int16_t& y, int16_t& z);
    uint8_t found();
};

#endif
