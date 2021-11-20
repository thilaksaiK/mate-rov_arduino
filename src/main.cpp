// #include <Wire.h>
// #include <sensor.h>
// #include <Arduino.h>

// const uint8_t mag_address = 0x1E;
// void setup()
// {
//     Wire.begin();
//     Serial.begin(9600);
// }

// void loop()
// {

// }
/*
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
*/

#include <Wire.h>
#include <Arduino.h>
#include <accelerometer.h>
#include <gyroscope.h>

const uint8_t accel_addr = 0x53;
const uint8_t gyro_address = 0x69;
accelerometer accel;
gyroscope gyro;

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    uint8_t status = accel.init(accel_addr);
    Serial.println("init = " + String(status));
    status = accel.isFound();
    Serial.println("isFound = " + String(status));
    status = accel.setResolution(RES_4G);
    Serial.println("Resolution = " + String(status));
    status = accel.calibration(-62, 62, -431);
    Serial.println("calibration = " + String(status));
    status = gyro.init(gyro_address);
    Serial.print(String(status) + " ");
    status = gyro.found();
    Serial.println(String(status) + " ");
}

float previousTime, currentTime, elapsedTime;
float gyroAngleX, gyroAngleY, yaw, pitch, roll;
void loop()
{
    int32_t x, y, z;
    uint8_t status = accel.getData(x, y, z);
    float xf, yf, zf;
    xf = x / 1000.0;
    yf = y / 1000.0;
    zf = z / 1000.0;
    float accAngleX = (atan(yf / sqrt(pow(xf, 2) + pow(zf, 2))) * 180 / PI) - 0.58;      // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    float accAngleY = (atan(-1 * xf / sqrt(pow(yf, 2) + pow(zf, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
    int16_t xg, yg, zg;
    gyro.readData(xg, yg, zg);
    float GyroX = xg * 0.00875;
    float GyroY = yg * 0.00875;
    float GyroZ = zg * 0.00875;
    previousTime = currentTime;                        // Previous time is stored before the actual time read
    currentTime = millis();                            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
    gyroAngleX = gyroAngleX + GyroX * elapsedTime;     // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw = yaw + GyroZ * elapsedTime;
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    // Print the values on the serial monitor
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);
}

/*
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
*/