#include "accelerometer.h"
#include <sensor.h>

#define WHO_AM_I 0x00
#define OFSX 0x1E
#define OFSY 0x1F
#define OFSZ 0x20
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32

uint8_t accelerometer::isFound()
{
    // Who am I
    uint8_t data;
    sensor_readRegister(this->_address, WHO_AM_I, 1, &data);
    return (uint8_t)(data == 0xE5);
}

uint8_t accelerometer::setResolution(res_t res)
{
    // set resolution
    uint8_t data;
    sensor_readRegister(this->_address, DATA_FORMAT, 1, &data);
    uint8_t resolution = (uint8_t)res;
    data &= ~0x03;
    data |= resolution & 0x03;
    uint8_t retVal = sensor_writeRegister(this->_address, DATA_FORMAT, 1, &data);
    this->currentResolution = res;
    return retVal;
}

uint8_t accelerometer::init(uint8_t address)
{
    this->_address = address;
    uint8_t data = 0x08;
    uint8_t retVal = sensor_writeRegister(this->_address, POWER_CTL, 1, &data);
    return retVal;
}

uint8_t accelerometer::getData(int32_t& x, int32_t& y, int32_t& z)
{
    int32_t factor = 0x0100 >> (uint8_t)this->currentResolution;
    uint8_t data[6];
    uint8_t retVal = sensor_readRegister(this->_address, DATAX0, 6, data);
    x = ((data[0] | (data[1] << 8)));
    y = ((data[2] | (data[3] << 8)));
    z = ((data[4] | (data[5] << 8)));
    x = (x * 1000) / factor;
    y = (y * 1000) / factor;
    z = (z * 1000) / factor;
    return retVal;
}

    
uint8_t accelerometer::calibration(int16_t x, int16_t y, int16_t z)
{
    uint8_t data = (x*10)/156;
    uint8_t retVal = sensor_writeRegister(this->_address, OFSX, 1, &data);

    data = (y*10)/156;
    retVal = sensor_writeRegister(this->_address, OFSY, 1, &data);

    data = (z*10)/156;
    retVal = sensor_writeRegister(this->_address, OFSZ, 1, &data);
    return retVal;
}