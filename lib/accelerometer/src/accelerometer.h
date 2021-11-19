#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H
#include <Arduino.h>


enum res_t
{
    RES_2G  = 0x00,
    RES_4G  = 0x01,
    RES_8G  = 0x02,
    RES_16G = 0x03
};

class accelerometer
{
private:
    uint8_t _address;
    res_t currentResolution;
public:
    uint8_t isFound();
    uint8_t setResolution(res_t resolution);
    uint8_t init(uint8_t address);
    uint8_t getData(int32_t& x, int32_t& y, int32_t& z);
    uint8_t calibration(int16_t x, int16_t y, int16_t z);
};
#endif