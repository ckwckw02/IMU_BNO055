#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>

#define SDA_PIN D4  // D4
#define SCL_PIN D5  // D5

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Declare Preferences instance
Preferences preferences;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type
    */
/**************************************************************************/
void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void) {
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_y);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_z);
  Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_y);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_z);
  Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_y);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_z);
  Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

/**************************************************************************/
/*
    Arduino setup function
    */
/**************************************************************************/
void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  Wire.begin(SDA_PIN, SCL_PIN);

  /* Initialise the sensor */
  if (!bno.begin(OPERATION_MODE_NDOF_FMC_OFF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  // Open Preferences under the "bno055" namespace in read-only mode first
  preferences.begin("bno055", true);

  long storedBnoID = preferences.getLong("bnoID", 0);
  bool foundCalib = false;

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  bno.getSensor(&sensor);

  if (storedBnoID != sensor.sensor_id) {
    Serial.println("\nNo Calibration Data for this sensor exists in Preferences");
    delay(500);
  } else {
    Serial.println("\nFound Calibration for this sensor in Preferences.");
    
    // Read the raw offset struct bytes directly from NVS
    preferences.getBytes("calibData", &calibrationData, sizeof(calibrationData));

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }

  // Close read-only session
  preferences.end();

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  
  /* Always recal the mag as it goes out of calibration very often */
  if (foundCalib) {
    Serial.println("Move sensor slightly to calibrate magnetometers");
    while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  } else {
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);

      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);

      displayCalStatus();
      Serial.println("");

      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to Preferences...");

  bno.getSensor(&sensor);

  // Open Preferences in read-write mode (false) to update values
  preferences.begin("bno055", false);
  preferences.putLong("bnoID", sensor.sensor_id);
  preferences.putBytes("calibData", &newCalib, sizeof(newCalib));
  preferences.end();

  Serial.println("Data stored to Preferences.");

  Serial.println("\n--------------------------------\n");
  delay(500);
}

void loop() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  displayCalStatus();
  Serial.println("");

  delay(BNO055_SAMPLERATE_DELAY_MS);
}