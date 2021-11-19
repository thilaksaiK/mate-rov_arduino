#include <Wire.h>
#include <Arduino.h>
#include <sensor.h>
#include <accelerometer.h>

uint8_t addresses[4] = {0x1E, 0x53, 0x69, 0x77};
const uint8_t accel_addr = 0x53;

uint8_t whoAmI(uint8_t address, uint8_t register);
uint8_t accel_readData(uint8_t address, float* data);
uint8_t accel_calibrate(uint8_t address, uint8_t values);
uint8_t accel_setResolution(uint8_t address, uint8_t resolution);
uint8_t accel_init(uint8_t address);
/*
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  accel_init(accel_addr);
  Serial.println(whoAmI(accel_addr, 0x00));
  accel_setResolution(accel_addr, 0x02);
  accel_calibrate(accel_addr, 206);
}

void loop()
{
  float data[3];
  accel_readData(accel_addr, data);
  Serial.print(data[0], 2);
  Serial.print(" ");
  Serial.print(data[1], 2);
  Serial.print(" ");
  Serial.print(data[2], 2);
  Serial.println(" ");
  //while(1);
  delay(1000);
}
*/

/*
int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out;  // Outputs
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y_out = Y_out/256;
  Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
  Z_out = Z_out/256;
  Serial.print("Xa= ");
  Serial.print(X_out);
  Serial.print("   Ya= ");
  Serial.print(Y_out);
  Serial.print("   Za= ");
  Serial.println(Z_out);
}

*/
uint8_t whoAmI(uint8_t address, uint8_t register)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(accel_addr, 1, true);
  uint8_t retVal = Wire.read();
  return retVal;
}

uint8_t accel_init(uint8_t address)
{
  Wire.beginTransmission(address);
  Wire.write(0x2D);
  Wire.write(8);
  return Wire.endTransmission();
}

uint8_t accel_readData(uint8_t address, float* data)
{
  float X_out, Y_out, Z_out;
  Wire.beginTransmission(address);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6, true);
  X_out = ( Wire.read()| Wire.read() << 8);
  data[0] = X_out/64.0; 
  Y_out = ( Wire.read()| Wire.read() << 8);
  data[1] = Y_out/64.0;
  Z_out = ( Wire.read()| Wire.read() << 8);
  data[2] = Z_out/64.0;
}

uint8_t accel_setResolution(uint8_t address, uint8_t resolution)
{
  Wire.beginTransmission(address);
  Wire.write(0x31);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 1, true);
  uint8_t retVal = Wire.read();
  retVal &= ~0x03;
  retVal |= resolution & 0x03;
  Serial.print("res = ");
  Serial.println(retVal);
  Wire.beginTransmission(address);
  Wire.write(0x31);
  Wire.write(retVal);
  Wire.endTransmission();
  return 0;
}

uint8_t accel_calibrate(uint8_t address, uint8_t values)
{
  Wire.beginTransmission(address);
  Wire.write(0x20);
  Wire.write(values);
  Wire.endTransmission();
}
/*

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  // Power on
  uint8_t data = 0x08;
  sensor_writeRegister(accel_addr, 0x2D, 1, &data);
  // Who am I
  sensor_readRegister(accel_addr, 0x00, 1, &data);
  Serial.print("Who am I = ");
  Serial.println(data);

  // set resolution
  sensor_readRegister(accel_addr, 0x31, 1, &data);
  uint8_t resolution = 0x02;
  data &= ~0x03;
  data |= resolution & 0x03;
  sensor_writeRegister(accel_addr, 0x31, 1, &data);
  accel_calibrate(accel_addr, 206);
}

void loop()
{
  uint8_t data[6];
  sensor_readRegister(accel_addr, 0x32, 6, data);
  int32_t x = ((data[0] | (data[1] << 8)));
  int32_t y = ((data[2] | (data[3] << 8)));
  int32_t z = ((data[4] | (data[5] << 8)));
  x = (x*1000)/64;
  y = (y*1000)/64;
  z = (z*1000)/64;
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.println(" ");
  delay(100);
}
*/

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
