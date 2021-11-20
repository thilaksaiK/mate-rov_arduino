#include <Wire.h>
#include <Arduino.h>
#include <sensor.h>

#define CTRL_REG1 0x20
#define STATUS_REG 0x27
#define OUT_X_L   0x28


#define READ    (1 << 7)
const uint8_t gyro_address = 0x69;

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    uint8_t data;
    sensor_readRegister(gyro_address, 0x0F, 1, &data);
    Serial.println(data, BIN);
    data = 0x0F;
    sensor_writeRegister(gyro_address, CTRL_REG1, 1, &data);
}

void loop()
{
    uint8_t data[6];
    int32_t x, y, z, factor = 1000;
    uint8_t status;
    sensor_readRegister(gyro_address, STATUS_REG, 1, &status);
    sensor_readRegister(gyro_address, OUT_X_L | READ, 6, data);
    x = ((data[0] | (data[1] << 8)));
    y = ((data[2] | (data[3] << 8)));
    z = ((data[4] | (data[5] << 8)));
    x = (x * 1000) / factor;
    y = (y * 1000) / factor;
    z = (z * 1000) / factor;
    Serial.print(status);
    Serial.print(" ");
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(z);
    Serial.println(" ");
    delay(9);
}
