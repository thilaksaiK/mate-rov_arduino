#include <Wire.h>
#include <Arduino.h>
#include <accelerometer.h>

const uint8_t accel_addr = 0x53;
accelerometer accel;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  uint8_t status = accel.init(accel_addr);
  Serial.println("init = "+String(status));
  status = accel.isFound();
  Serial.println("isFound = "+String(status));
  status = accel.setResolution(RES_4G);
  Serial.println("Resolution = "+String(status));
  status = accel.calibration(-62, 62, -431);
  Serial.println("calibration = "+String(status));
}

void loop()
{
  int32_t x, y, z;
  uint8_t status = accel.getData(x, y, z);
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.println(" ");
  delay(100);
}
