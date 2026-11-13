#include <Arduino.h>
#include <Wire.h>             // I2C Library
#include <SPI.h>              // Required by SD library
#include <SD.h>               // Gives read/write capabilites for SD card
#include <Adafruit_Sensor.h>  // Main sensor library
#include <Adafruit_BMP3XX.h>  // Driver for Barometer
#include <Adafruit_BNO08x.h>  // Driver for IMU
#include <TinyGPSPlus.h>      // Driver for GPS
#include <math.h>

const char* LOG_FILE_NAME = "/flight-log.txt";
const int SAMPLE_RATE = 100;  // How often data is logged (in milliseconds)

// Global sensor values (read in loop, served on request)
float altitude = 0, temperature = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float pitch  = 0, roll   = 0, yaw    = 0;
float altitudeBaseline = 0.0; 
float gravityConstant = 9.81;
float gForce = 0;
float totalGroundSpeed = 0.0, xGroundSpeed = 0.0, yGroundSpeed = 0.0, zGroundSpeed = 0.0;
bool isCalibrated = false;
bool GPSFix = false;
float pressureBaseline = 0.0;
float groundSpeed = 0.0;

//Barometer address: 0x76
//IMU address: 0x4A
#define SD_CS 5

Adafruit_BMP3XX barometer;
Adafruit_BNO08x IMU;
sh2_SensorValue_t sensorValue;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Use UART1 for GPS

void readGPS() {
    while (gpsSerial.available() > 0) {
        char incoming = gpsSerial.read();
        gps.encode(incoming);
    }
}

//Set report type
void setReportType(sh2_SensorId_t reportType, uint32_t interval_us) {
  if (!IMU.enableReport(reportType, interval_us)) {
    Serial.print(F("[ERR] Failed to enable report for sensor "));
    Serial.println(reportType);
  }
}

//Calibrate
 void calibrate() {
  Serial.println("[CAL] Calibrating Altitude...");
  float pressureSum = 0;
  int readings = 0;
  for (int i=0; i < 50; i++){
    if (barometer.performReading()) {
      pressureSum += barometer.readPressure();
      readings++;
    }
  }
  if (readings > 0) {
    pressureBaseline = pressureSum / readings;
    Serial.print("[CAL] Altitude baseline set to: ");
    Serial.println(altitudeBaseline);
    isCalibrated = true;
  } else {
    Serial.println("[ERR] Failed to read pressure for calibration.");
  }
}

//Calculate gforce
float calculategforce(float accelX, float accelY, float accelZ){
  float magnitudeAccel = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
  float Gforce = magnitudeAccel / gravityConstant;
  return Gforce;
}

//ground speed
float calculateGroundSpeed() {

    if (gps.speed.isValid() && gps.speed.age() < 2000) {
        return gps.speed.mps();
    }

    return NAN;
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
  Wire.begin(21, 22);  // SDA, SCL on ESP32 DevKit V1

  //initialize sensors
  if (!barometer.begin_I2C(0x76, &Wire)) {
    Serial.println(F("[ERR] BMP388 not found on I2C bus."));
  } else {
    Serial.println(F("[OK] BMP388 initialized."));
  }

  if (!IMU.begin_I2C(0x4A, &Wire)) {
    Serial.println(F("[ERR] BNO08x not found on I2C bus."));
  } else {
    Serial.println(F("[OK] BNO08x initialized."));
  }

  // Barometer configuration
  barometer.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  barometer.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  barometer.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  barometer.setOutputDataRate(BMP3_ODR_50_HZ);
  altitudeBaseline = barometer.readAltitude(1013.25);

  // IMU configuration
  setReportType(SH2_ARVR_STABILIZED_RV, 10000); // 100 Hz
  setReportType(SH2_ACCELEROMETER, 10000); // 100 Hz

  //Calibration
  Serial.println("Before");
  calibrate();
  Serial.println("After");

  // GPS configuration
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("[OK] GPS UART initialized.");

  // Log file creation
  SPI.begin(18, 19, 23, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("[ERR] SD card initialization failed!");
  } else {
    Serial.println("[OK] SD card initialized.");
  }
  File logFile = SD.open(LOG_FILE_NAME, "w");
  logFile.println("Time | Altitude | Temperature | X-Acceleration | Y-Acceleration | Z-Acceleration | G-Force | Ground Speed");
  logFile.close();
}

void loop() {
  bool receivedAccel = false;
  readGPS();

  // Get the barometer's data readings
  if (!barometer.performReading()) {
    Serial.println(F("[ERR] Failed to perform reading from BMP388."));
  } else {
    altitude = 44330.0f * (1.0f - powf(barometer.pressure / pressureBaseline, 0.19029495f));
    temperature = barometer.readTemperature();
  }

  //Check if IMU was reset
  if (IMU.wasReset()) {
    setReportType(SH2_ARVR_STABILIZED_RV, 10000);
    setReportType(SH2_ACCELEROMETER, 10000);
  }
  
  while (IMU.getSensorEvent(&sensorValue)){
    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      accelX = sensorValue.un.accelerometer.x;
      accelY = sensorValue.un.accelerometer.y;
      accelZ = sensorValue.un.accelerometer.z;
                  receivedAccel = true;
    }
  }


  Serial.print("Accelerometer event received: ");
  Serial.println(receivedAccel ? "YES" : "NO");
  
  gForce = calculategforce(accelX, accelY, accelZ);
  groundSpeed = calculateGroundSpeed();
  GPSFix = gps.location.isValid() && gps.location.age() < 2000;

  // Write the data into the flight log
  File logFile = SD.open(LOG_FILE_NAME, "a");
  
  logFile.printf(
    "%lu | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f\n",
    millis(),
    altitude,
    temperature,
    accelX,
    accelY,
    accelZ,
    gForce,
    groundSpeed
  );

  logFile.close();

  Serial.println("UMMM");
  Serial.println(accelX);
  Serial.println(accelY);
  Serial.println(accelZ);
  Serial.println(gForce);
  Serial.println(groundSpeed);
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  delay(SAMPLE_RATE);

}