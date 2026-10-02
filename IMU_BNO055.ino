#include <Wire.h>

#define SDA_PIN D4  // D4
#define SCL_PIN D5  // D5

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* System sample rate: 100 Hz (non-blocking, millis()-gated) */
static const uint32_t SAMPLE_PERIOD_MS = 10; // 1000 ms / 100 Hz

/* Serial baud rate - must match the Python receiver */
static const uint32_t SERIAL_BAUD = 921600;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(SERIAL_BAUD);

  while (!Serial) delay(10);  // wait for serial port to open!
  
  Wire.begin(SDA_PIN, SCL_PIN);
  


  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }

  delay(1000); // let the sensor settle before streaming starts

  // CSV header, printed exactly once. The Python receiver skips non-numeric lines.
  Serial.println(F("t_ms,euler_x,euler_y,euler_z,gyro_x,gyro_y,gyro_z,"
                   "accel_x,accel_y,accel_z,linacc_x,linacc_y,linacc_z,"
                   "mag_x,mag_y,mag_z,grav_x,grav_y,grav_z,temp_c,"
                   "cal_sys,cal_gyro,cal_accel,cal_mag"));
}

void loop(void)
{
  // Non-blocking 100 Hz gate. Catch-up style: after a stall we resync to the
  // 10 ms grid instead of bursting or drifting.
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < SAMPLE_PERIOD_MS) return;
  last += SAMPLE_PERIOD_MS;

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  const int8_t boardTemp = bno.getTemp(); // already in degC

  uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

  // One compact CSV line per sample (~150 bytes), single UART write.
  char buf[256];
  const int n = snprintf(buf, sizeof(buf),
    "%lu,"
    "%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,"
    "%.3f,%.3f,%.3f,"
    "%d,"
    "%u,%u,%u,%u\n",
    (unsigned long)now,
    orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z,
    angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z,
    accelerometerData.acceleration.x, accelerometerData.acceleration.y, accelerometerData.acceleration.z,
    linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z,
    magnetometerData.magnetic.x, magnetometerData.magnetic.y, magnetometerData.magnetic.z,
    gravityData.acceleration.x, gravityData.acceleration.y, gravityData.acceleration.z,
    boardTemp,
    calSys, calGyro, calAccel, calMag);
  Serial.write((const uint8_t *)buf, n);
}




