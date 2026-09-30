#include <Arduino.h>
#include <Wire.h>             // I2C Library
#include <SPI.h>              // Required by SD library
#include <SD.h>               // Gives read/write capabilites for SD card
#include <Adafruit_Sensor.h>  // Main sensor library
#include <Adafruit_BMP3XX.h>  // Driver for Barometer
#include <Adafruit_BNO08x.h> //Driver for IMU

const char* LOG_FILE_NAME = "/flight-log.txt";
const int SAMPLE_RATE = 100;  // How often data is logged (in milliseconds)

// Global sensor values (read in loop, served on request)
float altitude    = 0, temperature = 0, pressure = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float pitch  = 0, roll   = 0, yaw    = 0;

Adafruit_BMP3XX barometer;
Adafruit_BNO08x IMU;
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200); // Initialize ESP32
  Wire.begin();         // Initialize I2C
  SD.begin();
  barometer.begin_I2C();
  IMU.begin_I2C();

  // Barometer configuration
  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);

  // IMU configuration
  IMU.enableReport(SH2_ACCELEROMETER);
  IMU.enableReport(SH2_GYROSCOPE_CALIBRATED);

  // Log file creation
  File logFile = SD.open(LOG_FILE_NAME, "w");
  logFile.println("Time | Altitude | Temperature | Pressure | X-Acceleration | Y-Acceleration | Z-Acceleration | Pitch | Roll | Yaw");
  logFile.close();
}

void loop() {
  // Get the barometer's data readings
  altitude = barometer.readAltitude(1013.25); // Standard sea level pressure in hPa
  temperature = barometer.temperature;
  pressure = barometer.pressure;

  // Local variables for IMU data
  //float accelX = 0, accelY = 0, accelZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;
  
  //Get IMU readings
  while(IMU.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {

      case SH2_ACCELEROMETER:
        accelX = sensorValue.un.accelerometer.x;
        accelY = sensorValue.un.accelerometer.y;
        accelZ = sensorValue.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyroX = sensorValue.un.gyroscope.x;
        gyroY = sensorValue.un.gyroscope.y;
        gyroZ = sensorValue.un.gyroscope.z;
        break;
    }
  }

  // Write the data into the flight log
  File logFile = SD.open(LOG_FILE_NAME, "a");
  
  logFile.printf(
    "%lu | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f\n",
    millis(),
    altitude,
    temperature,
    pressure,
    accelX,
    accelY,
    accelZ,
    gyroX,
    gyroY,
    gyroZ
  );

  logFile.close();

  // Wait before running the loop again
  delay(SAMPLE_RATE);
}