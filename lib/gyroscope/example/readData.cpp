#include <Wire.h>
#include <Arduino.h>
#include <gyroscope.h>

gyroscope gyro;

const uint8_t gyro_address = 0x69;

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    uint8_t status = 0;
    status = gyro.init(gyro_address);
    Serial.print(String(status)+" ");
    status = gyro.found();
    Serial.println(String(status)+" ");
    delay(1000);
}

void loop()
{
    int16_t x, y, z;
    gyro.readData(x, y, z);
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(z);
    Serial.println(" ");
    delay(50);
}