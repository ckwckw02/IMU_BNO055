#include <Wire.h>

#define SDA_PIN D4  // D4
#define SCL_PIN D5  // D5

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* System sample rate: 100 Hz, driven by two dedicated FreeRTOS tasks:
   imuReadTask samples over I2C on the tick grid, imuTxTask formats and
   transmits. Decoupling them keeps t_ms jitter-free even if the UART stalls. */
static const uint32_t SAMPLE_PERIOD_MS = 10; // 1000 ms / 100 Hz

/* Serial baud rate - must match the Python receiver. */
static const uint32_t SERIAL_BAUD = 115200;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/* One sample as handed from the read task to the TX task. */
struct ImuSample {
  uint32_t t_ms;                        // captured at the tick boundary in imuReadTask
  float euler[3], gyro[3], accel[3];    // deg, dps, m/s^2 (raw)
  float linacc[3], mag[3], grav[3];     // m/s^2, uT, m/s^2
  int8_t   temp_c;
  uint8_t  cal_sys, cal_gyro, cal_accel, cal_mag;
};

/* Decouples I2C sampling from UART backpressure: even if the serial link is
   momentarily slow (host paused, TX buffer full), the read task keeps hitting
   its 10 ms grid and t_ms stays jitter-free. */
static QueueHandle_t sampleQueue = nullptr;
static const uint8_t SAMPLE_QUEUE_LEN = 8;

/* ESP32-C3 is single-core: both tasks run on core 0. The read task gets the
   higher priority so it always preempts the TX task and owns the timing. */
static const UBaseType_t IMU_READ_TASK_PRIORITY = 5;
static const uint32_t    IMU_READ_TASK_STACK    = 4096;
static const UBaseType_t IMU_TX_TASK_PRIORITY   = 4;
static const uint32_t    IMU_TX_TASK_STACK      = 6144; // snprintf float formatting

/* Runs at exactly 100 Hz: vTaskDelayUntil wakes on absolute tick boundaries,
   so after any stall the task resyncs to the grid instead of bursting or
   drifting. Each iteration only does I2C reads (~0.5 ms at 400 kHz) and a
   queue push - no UART, no formatting - so it always finishes well inside its
   10 ms slot and t_ms jitter stays under ~1 ms. */
void imuReadTask(void *arg)
{
  (void)arg;

  TickType_t lastWake = xTaskGetTickCount();

  for (;;)
  {
    // Timestamp first: captured at the tick boundary, before any I2C work.
    ImuSample s;
    s.t_ms = millis();

    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    s.euler[0] = orientationData.orientation.x;
    s.euler[1] = orientationData.orientation.y;
    s.euler[2] = orientationData.orientation.z;
    s.gyro[0]  = angVelocityData.gyro.x;
    s.gyro[1]  = angVelocityData.gyro.y;
    s.gyro[2]  = angVelocityData.gyro.z;
    s.accel[0] = accelerometerData.acceleration.x;
    s.accel[1] = accelerometerData.acceleration.y;
    s.accel[2] = accelerometerData.acceleration.z;
    s.linacc[0] = linearAccelData.acceleration.x;
    s.linacc[1] = linearAccelData.acceleration.y;
    s.linacc[2] = linearAccelData.acceleration.z;
    s.mag[0]  = magnetometerData.magnetic.x;
    s.mag[1]  = magnetometerData.magnetic.y;
    s.mag[2]  = magnetometerData.magnetic.z;
    s.grav[0] = gravityData.acceleration.x;
    s.grav[1] = gravityData.acceleration.y;
    s.grav[2] = gravityData.acceleration.z;

    s.temp_c = bno.getTemp(); // already in degC
    bno.getCalibration(&s.cal_sys, &s.cal_gyro, &s.cal_accel, &s.cal_mag);

    // Never block on the UART side: if the TX task is behind, drop this sample
    // rather than delaying the next read. (At 921600 this never happens.)
    xQueueSend(sampleQueue, &s, 0);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

/* Formats and transmits samples as fast as the UART allows. At 921600 a full
   line shifts out in <2 ms, so this task keeps up with the 100 Hz stream; any
   blocking it does (e.g. host paused) can never disturb the read task's
   timing because it runs at lower priority and owns Serial exclusively. */
void imuTxTask(void *arg)
{
  (void)arg;

  char buf[256];
  ImuSample s;

  for (;;)
  {
    // Block until a sample is ready (timeout: nothing to send, just re-check).
    if (xQueueReceive(sampleQueue, &s, pdMS_TO_TICKS(20)) != pdTRUE) continue;

    // One compact CSV line per sample (~170 bytes), single UART write.
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
      (unsigned long)s.t_ms,
      s.euler[0], s.euler[1], s.euler[2],
      s.gyro[0], s.gyro[1], s.gyro[2],
      s.accel[0], s.accel[1], s.accel[2],
      s.linacc[0], s.linacc[1], s.linacc[2],
      s.mag[0], s.mag[1], s.mag[2],
      s.grav[0], s.grav[1], s.grav[2],
      (int)s.temp_c,
      s.cal_sys, s.cal_gyro, s.cal_accel, s.cal_mag);
    Serial.write((const uint8_t *)buf, n);
  }
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD);

  while (!Serial) delay(10);  // wait for serial port to open!
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL); // BNO055 supports up to 1 MHz; ~4x faster I2C reads
  


  /* Initialise the sensor */
  if (!bno.begin(OPERATION_MODE_IMUPLUS))
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

  /* Start the two streaming tasks. From this point on imuReadTask is the only
     code that touches bno and imuTxTask the only one that touches Serial, so
     no locking is needed. */
  sampleQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(ImuSample));
  xTaskCreatePinnedToCore(imuReadTask, "imu_read", IMU_READ_TASK_STACK, nullptr,
                          IMU_READ_TASK_PRIORITY, nullptr, 0);
  xTaskCreatePinnedToCore(imuTxTask, "imu_tx", IMU_TX_TASK_STACK, nullptr,
                          IMU_TX_TASK_PRIORITY, nullptr, 0);
}

void loop(void)
{
  // All sampling happens in imuReadTask / imuTxTask; nothing to do here.
  vTaskDelay(pdMS_TO_TICKS(100));
}




