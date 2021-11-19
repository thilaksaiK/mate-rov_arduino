
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL345_U.h>

// /* Assign a unique ID to this sensor at the same time */
// Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// void displaySensorDetails(void)
// {
//   sensor_t sensor;
//   accel.getSensor(&sensor);
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
//   Serial.println("------------------------------------");
//   Serial.println("");
//   delay(500);
// }

// void displayDataRate(void)
// {
//   Serial.print  ("Data Rate:    "); 
  
//   switch(accel.getDataRate())
//   {
//     case ADXL345_DATARATE_3200_HZ:
//       Serial.print  ("3200 "); 
//       break;
//     case ADXL345_DATARATE_1600_HZ:
//       Serial.print  ("1600 "); 
//       break;
//     case ADXL345_DATARATE_800_HZ:
//       Serial.print  ("800 "); 
//       break;
//     case ADXL345_DATARATE_400_HZ:
//       Serial.print  ("400 "); 
//       break;
//     case ADXL345_DATARATE_200_HZ:
//       Serial.print  ("200 "); 
//       break;
//     case ADXL345_DATARATE_100_HZ:
//       Serial.print  ("100 "); 
//       break;
//     case ADXL345_DATARATE_50_HZ:
//       Serial.print  ("50 "); 
//       break;
//     case ADXL345_DATARATE_25_HZ:
//       Serial.print  ("25 "); 
//       break;
//     case ADXL345_DATARATE_12_5_HZ:
//       Serial.print  ("12.5 "); 
//       break;
//     case ADXL345_DATARATE_6_25HZ:
//       Serial.print  ("6.25 "); 
//       break;
//     case ADXL345_DATARATE_3_13_HZ:
//       Serial.print  ("3.13 "); 
//       break;
//     case ADXL345_DATARATE_1_56_HZ:
//       Serial.print  ("1.56 "); 
//       break;
//     case ADXL345_DATARATE_0_78_HZ:
//       Serial.print  ("0.78 "); 
//       break;
//     case ADXL345_DATARATE_0_39_HZ:
//       Serial.print  ("0.39 "); 
//       break;
//     case ADXL345_DATARATE_0_20_HZ:
//       Serial.print  ("0.20 "); 
//       break;
//     case ADXL345_DATARATE_0_10_HZ:
//       Serial.print  ("0.10 "); 
//       break;
//     default:
//       Serial.print  ("???? "); 
//       break;
//   }  
//   Serial.println(" Hz");  
// }

// void displayRange(void)
// {
//   Serial.print  ("Range:         +/- "); 
  
//   switch(accel.getRange())
//   {
//     case ADXL345_RANGE_16_G:
//       Serial.print  ("16 "); 
//       break;
//     case ADXL345_RANGE_8_G:
//       Serial.print  ("8 "); 
//       break;
//     case ADXL345_RANGE_4_G:
//       Serial.print  ("4 "); 
//       break;
//     case ADXL345_RANGE_2_G:
//       Serial.print  ("2 "); 
//       break;
//     default:
//       Serial.print  ("?? "); 
//       break;
//   }  
//   Serial.println(" g");  
// }

// void setup(void) 
// {
// #ifndef ESP8266
//   while (!Serial); // for Leonardo/Micro/Zero
// #endif
//   Serial.begin(9600);
//   Serial.println("Accelerometer Test"); Serial.println("");
  
//   /* Initialise the sensor */
//   if(!accel.begin())
//   {
//     /* There was a problem detecting the ADXL345 ... check your connections */
//     Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
//     while(1);
//   }

//   /* Set the range to whatever is appropriate for your project */
//   //accel.setRange(ADXL345_RANGE_16_G);
//   // accel.setRange(ADXL345_RANGE_8_G);
//    accel.setRange(ADXL345_RANGE_4_G);
//   // accel.setRange(ADXL345_RANGE_2_G);
  
//   /* Display some basic information on this sensor */
//   displaySensorDetails();
  
//   /* Display additional settings (outside the scope of sensor_t) */
//   displayDataRate();
//   displayRange();
//   Serial.println("");
// }

// void loop(void) 
// {
//   /* Get a new sensor event */ 
//   sensors_event_t event; 
//   accel.getEvent(&event);
 
//   /* Display the results (acceleration is measured in m/s^2) */
//   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
//   delay(500);
// }


// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL345_U.h>

// /* Assign a unique ID to this sensor at the same time */
// Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


// float AccelMinX = 0;
// float AccelMaxX = 0;
// float AccelMinY = 0;
// float AccelMaxY = 0;
// float AccelMinZ = 0;
// float AccelMaxZ = 0;


// void setup(void) 
// {
//   Serial.begin(9600);
//   Serial.println("ADXL345 Accelerometer Calibration"); 
//   Serial.println("");
  
//   /* Initialise the sensor */
//   if(!accel.begin())
//   {
//     /* There was a problem detecting the ADXL345 ... check your connections */
//     Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
//     while(1);
//   }
// }

// void loop(void)
// {
//     Serial.println("Type key when ready..."); 
//     while (!Serial.available()){}  // wait for a character
    
//     /* Get a new sensor event */ 
//     sensors_event_t accelEvent;  
//     accel.getEvent(&accelEvent);
    
//     if (accelEvent.acceleration.x < AccelMinX) AccelMinX = accelEvent.acceleration.x;
//     if (accelEvent.acceleration.x > AccelMaxX) AccelMaxX = accelEvent.acceleration.x;
    
//     if (accelEvent.acceleration.y < AccelMinY) AccelMinY = accelEvent.acceleration.y;
//     if (accelEvent.acceleration.y > AccelMaxY) AccelMaxY = accelEvent.acceleration.y;
  
//     if (accelEvent.acceleration.z < AccelMinZ) AccelMinZ = accelEvent.acceleration.z;
//     if (accelEvent.acceleration.z > AccelMaxZ) AccelMaxZ = accelEvent.acceleration.z;
  
//     Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
//     Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();

//     while (Serial.available())
//     {
//       Serial.read();  // clear the input buffer
//     }
// }

/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/
/*
#include <Wire.h>
#include <L3G.h>

L3G gyro;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
}

void loop() {
  gyro.read();

  Serial.print("G ");
  Serial.print("X: ");
  Serial.print((int)gyro.g.x);
  Serial.print(" Y: ");
  Serial.print((int)gyro.g.y);
  Serial.print(" Z: ");
  Serial.println((int)gyro.g.z);

  delay(100);
}
*/

/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746
 
  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/

// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_HMC5883_U.h>

// /* Assign a unique ID to this sensor at the same time */
// Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// void displaySensorDetails(void)
// {
//   sensor_t sensor;
//   mag.getSensor(&sensor);
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
//   Serial.println("------------------------------------");
//   Serial.println("");
//   delay(500);
// }

// void setup(void) 
// {
//   Serial.begin(9600);
//   Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
//   /* Initialise the sensor */
//   if(!mag.begin())
//   {
//     /* There was a problem detecting the HMC5883 ... check your connections */
//     Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
//     while(1);
//   }
  
//   /* Display some basic information on this sensor */
//   displaySensorDetails();
// }

// void loop(void) 
// {
//   /* Get a new sensor event */ 
//   sensors_event_t event; 
//   mag.getEvent(&event);
 
//   /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
//   Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
//   Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
//   Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

//   // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
//   // Calculate heading when the magnetometer is level, then correct for signs of axis.
//   float heading = atan2(event.magnetic.y, event.magnetic.x);
  
//   // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
//   // Find yours here: http://www.magnetic-declination.com/
//   // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
//   // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//   float declinationAngle = 0.22;
//   heading += declinationAngle;
  
//   // Correct for when signs are reversed.
//   if(heading < 0)
//     heading += 2*PI;
    
//   // Check for wrap due to addition of declination.
//   if(heading > 2*PI)
//     heading -= 2*PI;
   
//   // Convert radians to degrees for readability.
//   float headingDegrees = heading * 180/M_PI; 
  
//   Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  
//   delay(500);
// }

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These pressure and temperature sensors use I2C to communicate, 2 pins
  are required to interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
  
void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }
}
  
void loop() {
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();
    delay(500);
}
