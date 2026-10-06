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
float altitude = 0, temperature = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float pitch  = 0, roll   = 0, yaw    = 0;
float altitudeBaseline = 0.0; 

Adafruit_BMP3XX barometer;
Adafruit_BNO08x IMU;
sh2_SensorValue_t sensorValue;

//Calculate gforce
void calculategforce(){
  
}

//Calibrate Roll Pitch Yaw
void calibrate() {
  //Wait until BNO08x reports good calibration status
  sh2_SensorValue_t val;
  while (true) {
    if (IMU.getSensorEvent(&val)) {
      if (val.status >= 2) break;  // 0=unreliable, 1=low, 2=medium, 3=high
    }
    delay(10);
  }

  Serial.println(F("[CAL] IMU ready, sampling altitude baseline..."));

  //Barometer sampling before calibration
  for (int i = 0; i < 5; i++) {
    barometer.performReading();
    delay(50);
  }

  float sum = 0;
  int   good = 0;
  for (int i = 0; i < 20; i++) {
    if (barometer.performReading()) {
      sum += barometer.readAltitude(1013.25);
      good++;
    }
    delay(50);
  }
  if (good > 0) {
    altitudeBaseline = sum / good;
    Serial.print(F("[CAL] Baseline set to "));
    Serial.print(altitudeBaseline, 2);
    Serial.println(F(" m"));
  } else {
    Serial.println(F("[CAL] ERROR: No valid BMP388 readings during calibration!"));
  }
}

// ─────────────────────────────────────────────
//  Quaternion to Euler (degrees)
//  Convention: ZYX (aerospace): Yaw/Pitch/Roll
// ─────────────────────────────────────────────
void quaternionToEuler(float qr, float qi, float qj, float qk,
                       float &outRoll, float &outPitch, float &outYaw) {
  // Rotation vector component layout from BNO08X:
  //   real=qw, i=qx, j=qy, k=qz
  float qw = qr, qx = qi, qy = qj, qz = qk;

  // Roll (X-axis rotation)
  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  outRoll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

  // Pitch (Y-axis rotation) — clamped to avoid gimbal singularity
  float sinp = 2.0f * (qw * qy - qz * qx);
  if (fabsf(sinp) >= 1.0f)
    outPitch = copysignf(90.0f, sinp);
  else
    outPitch = asinf(sinp) * RAD_TO_DEG;

  // Yaw (Z-axis rotation) — 0..360
  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  outYaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
  if (outYaw < 0) outYaw += 360.0f;
}

void setup() {
  Serial.begin(115200); // Initialize ESP32
  Wire.begin();         // Initialize I2C
  SD.begin();
  barometer.begin_I2C();
  IMU.begin_I2C();

  // Barometer configuration
  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);

  // IMU configuration
  IMU.enableReport(SH2_ACCELEROMETER);
  IMU.enableReport(SH2_GYROSCOPE_CALIBRATED);

  // Log file creation
  File logFile = SD.open(LOG_FILE_NAME, "w");
  logFile.println("Time | Altitude | Temperature | X-Acceleration | Y-Acceleration | Z-Acceleration | Pitch | Roll | Yaw");
  logFile.close();
}

void loop() {
  // Get the barometer's data readings
  altitude = barometer.readAltitude(1013.25); // Standard sea level pressure in hPa
  temperature = barometer.temperature;

  while (IMU.getSensorEvent(&sensorValue)){
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR){
      quaternionToEuler(
        sensorValue.un.rotationVector.real,
        sensorValue.un.rotationVector.i,
        sensorValue.un.rotationVector.j,
        sensorValue.un.rotationVector.k,
        roll, pitch, yaw
      );
    } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      accelX = sensorValue.un.linearAcceleration.x;
      accelY = sensorValue.un.linearAcceleration.y;
      accelZ = sensorValue.un.linearAcceleration.z;
    }

  // Write the data into the flight log
  File logFile = SD.open(LOG_FILE_NAME, "a");
  
  logFile.printf(
    "%lu | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f\n",
    millis(),
    altitude,
    temperature,
    accelX,
    accelY,
    accelZ,
    roll,
    pitch,
    yaw
  );

  logFile.close();

  // Wait before running the loop again
  delay(SAMPLE_RATE);
}