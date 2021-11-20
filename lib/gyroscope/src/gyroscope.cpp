#include "gyroscope.h"

uint8_t gyroscope::init(uint8_t address)
{
    this->_address = address;
    uint8_t data = 0x6F;
    // DR = 100, BW = 32 enable all axes
    uint8_t retVal = sensor_writeRegister(address, CTRL_REG1, 1, &data);
    data = 0x00;
    retVal = sensor_writeRegister(address, CTRL_REG4, 1, &data);
    return retVal;
}

uint8_t gyroscope::readData(int16_t& x, int16_t& y, int16_t& z)
{
    uint8_t data[6];
    uint8_t status;
    uint8_t count = 0;
    do
    {
        sensor_readRegister(this->_address, STATUS_REG, 1, &status);
        ++count;
    } while (((status >> 3) & 0x01) == 0 && count < 10);

    uint8_t retVal = sensor_readRegister(this->_address, OUT_X_L | READ, 6, data);
    x = ((data[0] | (data[1] << 8)));
    y = ((data[2] | (data[3] << 8)));
    z = ((data[4] | (data[5] << 8)));
    return retVal;
}

uint8_t gyroscope::found()
{
    uint8_t data;
    sensor_readRegister(this->_address, WHO_AM_I | READ, 1, &data);
    return (data==0xD3);
}