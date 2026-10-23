#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA_PIN D4
#define SCL_PIN D5

static const uint32_t SAMPLE_PERIOD_MS = 10;
static const uint32_t SERIAL_BAUD = 115200;
static const char *WIFI_SSID = "MotorControl";
static const char *WIFI_PASSWORD = "12345678";
static const char *MOTOR_HOST = "192.168.4.1";
static const uint16_t MOTOR_PORT = 80;
static const uint32_t MOTOR_HTTP_TIMEOUT_MS = 300;

static const uint32_t STILL_CAL_MS = 5000;
static const uint32_t WALK_CAL_MS = 8000;
static const uint32_t CAL_STATUS_PERIOD_MS = 1000;

/* Right-shank mounting: X is along the shank, Y is forward/back. */
static const int SHANK_AXIS = 0;
static const int AP_AXIS = 1;

/* Metric = pitch excursion + weighted pitch speed. */
static const float OMEGA_WEIGHT = 0.15f;
static const float OFF_RANGE_FRACTION = 0.45f;
static const float ON_RANGE_FRACTION = 0.4f;
static const uint32_t MIN_STATE_TIME_MS = 60;

/* Mode 'a' (advanced capture): auto pitch-offset calibration window before
   full-rate logging starts. The log is post-processed by imu_adv_tune.py to
   back-calculate the four constants above. */
static const uint32_t ADV_STILL_MS = 3000;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

struct MotorCommand {
  bool start;
};

static QueueHandle_t motorQueue = nullptr;
static const uint8_t MOTOR_QUEUE_LEN = 4;

/* BNO055 sensor calibration (accel/gyro/mag) required before use. */
static bool imuCalibrated = false;

static const char *stateName(bool footOffGround)
{
  return footOffGround ? "FOOT_OFF_GROUND" : "FOOT_ON_GROUND";
}

static float wrapAngleDeg(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

static void queueMotorCommand(bool start)
{
  if (motorQueue == nullptr) return;
  MotorCommand command = {start};
  xQueueSend(motorQueue, &command, 0);
}

/* Print the raw BNO055 calibration offsets so the user can see exactly
   what is being loaded from / saved to Preferences. */
static void printCalibOffsets(const char *tag, const adafruit_bno055_offsets_t &d)
{
  Serial.printf("CALIB,%s,GYRO,%d,%d,%d\n", tag,
                (int)d.gyro_offset_x, (int)d.gyro_offset_y, (int)d.gyro_offset_z);
  Serial.printf("CALIB,%s,ACCEL,%d,%d,%d\n", tag,
                (int)d.accel_offset_x, (int)d.accel_offset_y, (int)d.accel_offset_z);
  Serial.printf("CALIB,%s,MAG,%d,%d,%d\n", tag,
                (int)d.mag_offset_x, (int)d.mag_offset_y, (int)d.mag_offset_z);
  Serial.printf("CALIB,%s,RADIUS,accel:%d,mag:%d\n", tag,
                (int)d.accel_radius, (int)d.mag_radius);
}

static void loadCalibrationOffsets()
{
  Preferences prefs;
  sensor_t sensor;
  bno.getSensor(&sensor);
  prefs.begin("bno055", true);
  const long storedBnoID = prefs.getLong("bnoID", 0);
  if (storedBnoID == (long)sensor.sensor_id)
  {
    adafruit_bno055_offsets_t calibData;
    const size_t n = prefs.getBytes("calibData", &calibData, sizeof(calibData));
    if (n == sizeof(calibData))
    {
      bno.setSensorOffsets(calibData);
      imuCalibrated = true;
      Serial.println(F("BNO055 calibration loaded."));
      printCalibOffsets("LOAD", calibData);
    }
    else Serial.println(F("Warning: calibration data missing."));
  }
  else Serial.println(F("Warning: no matching BNO055 calibration."));
  prefs.end();
}

static void saveCalibrationOffsets()
{
  adafruit_bno055_offsets_t calibData;
  sensor_t sensor;
  bno.getSensorOffsets(calibData);
  bno.getSensor(&sensor);
  Preferences prefs;
  prefs.begin("bno055", false);
  prefs.putLong("bnoID", (long)sensor.sensor_id);
  prefs.putBytes("calibData", &calibData, sizeof(calibData));
  prefs.end();
  Serial.println(F("BNO055 calibration saved."));
  printCalibOffsets("SAVE", calibData);
}

static void printImuCalStatus()
{
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.printf("CAL,IMU,SYS:%d,G:%d,A:%d,M:%d\n",
                system, gyro, accel, mag);
}

static bool hasImuCalibration()
{
  Preferences prefs;
  sensor_t sensor;
  bno.getSensor(&sensor);
  prefs.begin("bno055", true);
  const long storedBnoID = prefs.getLong("bnoID", 0);
  adafruit_bno055_offsets_t calibData;
  const size_t n = prefs.getBytes("calibData", &calibData, sizeof(calibData));
  prefs.end();
  return storedBnoID == (long)sensor.sensor_id && n == sizeof(calibData);
}

static bool motorHttpGet(const char *path)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.printf("MOTOR,ERROR,WIFI_DISCONNECTED,%s\n", path);
    return false;
  }

  WiFiClient client;
  client.setTimeout(MOTOR_HTTP_TIMEOUT_MS);
  if (!client.connect(MOTOR_HOST, MOTOR_PORT))
  {
    Serial.printf("MOTOR,ERROR,CONNECT_FAILED,%s\n", path);
    return false;
  }
  client.printf("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n",
                path, MOTOR_HOST);
  const uint32_t deadline = millis() + MOTOR_HTTP_TIMEOUT_MS;
  while (client.connected() && !client.available() && millis() < deadline)
    vTaskDelay(pdMS_TO_TICKS(1));

  String statusLine = client.readStringUntil('\n');
  client.stop();
  statusLine.trim();
  Serial.printf("MOTOR,%s,%s\n",
                strcmp(path, "/start") == 0 ? "START" : "STOP",
                statusLine.length() ? statusLine.c_str() : "NO_RESPONSE");
  return statusLine.startsWith("HTTP/1.1 2") || statusLine.startsWith("HTTP/1.0 2");
}

void motorTask(void *arg)
{
  (void)arg;
  MotorCommand command;
  for (;;)
  {
    if (xQueueReceive(motorQueue, &command, portMAX_DELAY) == pdTRUE)
      motorHttpGet(command.start ? "/start" : "/stop");
  }
}

static float gravityPitchDeg(const sensors_event_t &gravityData)
{
  const float gravity[3] = {
    gravityData.acceleration.x,
    gravityData.acceleration.y,
    gravityData.acceleration.z
  };
  return atan2f(gravity[AP_AXIS], gravity[SHANK_AXIS]) * 57.2957795f;
}

void imuTask(void *arg)
{
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  enum Stage { ST_IDLE, ST_IMU_CAL, ST_STILL, ST_WALK, ST_FSM, ST_ADV_CAL, ST_ADV } stage = ST_IDLE;
  uint32_t stageStart = 0;
  uint32_t lastCalStatus = 0;
  float stillPitchSum = 0.0f;
  uint32_t stillCount = 0;
  float walkMetricMin = 1.0e9f;
  float walkMetricMax = -1.0e9f;
  bool pitchCalibrated = false;
  bool thresholdsCalibrated = false;
  float previousPitch = 0.0f;
  float previousRawPitch = 0.0f;
  float unwrappedPitch = 0.0f;
  float filteredPitch = 0.0f;
  float pitchOffset = 0.0f;
  float offThreshold = 0.0f;
  float onThreshold = 0.0f;
  bool footOffGround = false;
  uint32_t lastStateChange = 0;
  uint32_t lastSample = 0;
  uint32_t lastDebug = 0;
  /* Mode 'a' advanced capture: per-sample raw signals + reference FSM. */
  uint32_t advSampleIndex = 0;
  uint32_t advLastPrint = 0;
  uint32_t advLastStateChange = 0;
  bool advFootOffGround = false;
  float advMetricMin = 1.0e9f;
  float advMetricMax = -1.0e9f;

  for (;;)
  {
    sensors_event_t gravityData;
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    const uint32_t now = millis();
    const float rawPitch = gravityPitchDeg(gravityData);
    // Re-seed the pitch filter on boot and after IMU calibration, because
    // the figure-8 mag motion corrupts the filtered pitch history.
    if (lastSample == 0 || stage == ST_IMU_CAL)
    {
      previousRawPitch = rawPitch;
      unwrappedPitch = rawPitch;
      filteredPitch = unwrappedPitch;
      previousPitch = filteredPitch;
      lastSample = now;
    }
    else
    {
      // atan2f() wraps at +/-180 degrees. Track the shortest change so
      // calibration does not average -179 and +179 as if they were far apart.
      unwrappedPitch += wrapAngleDeg(rawPitch - previousRawPitch);
      previousRawPitch = rawPitch;
    }
    const float dt = max(0.001f, (now - lastSample) * 0.001f);
    lastSample = now;
    filteredPitch += 0.20f * (unwrappedPitch - filteredPitch);
    const float omega = (filteredPitch - previousPitch) / dt;
    previousPitch = filteredPitch;

    // Serial commands: i = IMU calibration, s = still calibration,
    // w = walk calibration, f = FSM mode, a = advanced capture (tuning log),
    // d = dump current FSM parameters.
    while (Serial.available())
    {
      const char c = (char)Serial.read();
      if (c == 'i')
      {
        // Stop the motor if the FSM left it running before calibration.
        if (footOffGround)
        {
          footOffGround = false;
          queueMotorCommand(false);
        }
        stage = ST_IMU_CAL;
        stageStart = now;
        lastCalStatus = now;
        Serial.println(F("CAL,IMU_START,stand still then draw figure 8 with the sensor."));
      }
      else if (c == 'd')
      {
        Serial.printf("PARAM,OMEGA_WEIGHT,%.4f\n", OMEGA_WEIGHT);
        Serial.printf("PARAM,OFF_RANGE_FRACTION,%.4f\n", OFF_RANGE_FRACTION);
        Serial.printf("PARAM,ON_RANGE_FRACTION,%.4f\n", ON_RANGE_FRACTION);
        Serial.printf("PARAM,MIN_STATE_TIME_MS,%lu\n", (unsigned long)MIN_STATE_TIME_MS);
        Serial.printf("PARAM,PITCH_OFFSET,%.2f\n", pitchOffset);
        Serial.printf("PARAM,ON_THRESHOLD,%.2f\n", onThreshold);
        Serial.printf("PARAM,OFF_THRESHOLD,%.2f\n", offThreshold);
      }
      else if (c == 'a')
      {
        if (stage == ST_ADV || stage == ST_ADV_CAL)
        {
          stage = ST_IDLE;
          // The auto pitch offset taken here is capture-specific; do not let
          // the FSM reuse it without a fresh still calibration.
          thresholdsCalibrated = false;
          Serial.printf("ADV,STOP,samples:%lu,metric=%.2f..%.2f\n",
                        (unsigned long)advSampleIndex, advMetricMin, advMetricMax);
        }
        else if (!imuCalibrated)
        {
          Serial.println(F("REJECTED,run IMU calibration first (i)."));
        }
        else
        {
          stage = ST_ADV_CAL;
          stageStart = now;
          lastCalStatus = now;
          stillPitchSum = 0.0f;
          stillCount = 0;
          Serial.println(F("ADV,STILL_START,stand still for 3 seconds."));
        }
      }
      else if (c == 's' || c == 'w' || c == 'f')
      {
        if (!imuCalibrated)
        {
          Serial.println(F("REJECTED,run IMU calibration first (i)."));
          continue;
        }
        stageStart = now;
        lastCalStatus = now;
        if (c == 's')
        {
          stage = ST_STILL;
          stillPitchSum = 0.0f;
          stillCount = 0;
          pitchCalibrated = false;
          thresholdsCalibrated = false;
          Serial.println(F("CAL,STILL_START,stand still for 5 seconds."));
        }
        else if (c == 'w')
        {
          if (!pitchCalibrated)
            Serial.println(F("CAL,WALK_REJECTED,run still calibration first (s)."));
          else
          {
            stage = ST_WALK;
            walkMetricMin = 1.0e9f;
            walkMetricMax = -1.0e9f;
            Serial.println(F("CAL,WALK_START,walk naturally for 8 seconds."));
          }
        }
        else if (c == 'f')
        {
          if (!pitchCalibrated || !thresholdsCalibrated)
            Serial.println(F("FSM,REJECTED,calibrate first: s then w."));
          else
          {
            stage = ST_FSM;
            footOffGround = false;
            lastStateChange = now;
            Serial.println(F("FSM,FOOT_ON_GROUND,STARTED"));
            queueMotorCommand(false);
          }
        }
      }
    }

    if (stage == ST_IMU_CAL)
    {
      sensors_event_t event;
      bno.getEvent(&event);
      if (now - lastCalStatus >= CAL_STATUS_PERIOD_MS)
      {
        lastCalStatus = now;
        printImuCalStatus();
      }
      if (bno.isFullyCalibrated())
      {
        saveCalibrationOffsets();
        imuCalibrated = true;
        stage = ST_IDLE;
        footOffGround = false;
        Serial.println(F("CAL,IMU_DONE,calibration saved to Preferences."));
      }
    }
    else if (stage == ST_STILL)
    {
      stillPitchSum += filteredPitch;
      stillCount++;
      if (now - lastCalStatus >= CAL_STATUS_PERIOD_MS)
      {
        lastCalStatus = now;
        Serial.printf("CAL,STILL,%lu/%lu ms\n",
                      (unsigned long)(now - stageStart),
                      (unsigned long)STILL_CAL_MS);
      }
      if (now - stageStart >= STILL_CAL_MS && stillCount > 20)
      {
        pitchOffset = stillPitchSum / stillCount;
        pitchCalibrated = true;
        stage = ST_IDLE;
        Serial.printf("CAL,STILL_DONE,pitch_offset=%.2f\n", pitchOffset);
      }
    }
    else if (stage == ST_WALK)
    {
      const float theta = filteredPitch - pitchOffset;
      const float metric = fabsf(theta) + OMEGA_WEIGHT * fabsf(omega);
      walkMetricMin = min(walkMetricMin, metric);
      walkMetricMax = max(walkMetricMax, metric);
      if (now - lastCalStatus >= CAL_STATUS_PERIOD_MS)
      {
        lastCalStatus = now;
        Serial.printf("CAL,WALK,%lu/%lu ms,range=%.2f..%.2f\n",
                      (unsigned long)(now - stageStart),
                      (unsigned long)WALK_CAL_MS, walkMetricMin, walkMetricMax);
      }
      if (now - stageStart >= WALK_CAL_MS)
      {
        const float range = max(1.0f, walkMetricMax - walkMetricMin);
        offThreshold = walkMetricMin + OFF_RANGE_FRACTION * range;
        onThreshold = walkMetricMin + ON_RANGE_FRACTION * range;
        if (onThreshold >= offThreshold) onThreshold = 0.5f * offThreshold;
        thresholdsCalibrated = true;
        stage = ST_IDLE;
        Serial.printf("CAL,DONE,pitch_offset=%.2f,on=%.2f,off=%.2f\n",
                      pitchOffset, onThreshold, offThreshold);
      }
    }
    else if (stage == ST_ADV_CAL)
    {
      // Auto still calibration for mode 'a': same idea as 's' but shorter and
      // it does not mark the FSM as pitch-calibrated (capture is self-contained).
      stillPitchSum += filteredPitch;
      stillCount++;
      if (now - lastCalStatus >= CAL_STATUS_PERIOD_MS)
      {
        lastCalStatus = now;
        Serial.printf("ADV,STILL,%lu/%lu ms\n",
                      (unsigned long)(now - stageStart),
                      (unsigned long)ADV_STILL_MS);
      }
      if (now - stageStart >= ADV_STILL_MS && stillCount > 20)
      {
        pitchOffset = stillPitchSum / stillCount;
        stage = ST_ADV;
        advSampleIndex = 0;
        advLastPrint = 0;
        advLastStateChange = now;
        advFootOffGround = false;
        advMetricMin = 1.0e9f;
        advMetricMax = -1.0e9f;
        Serial.println(F("ADV,META,cols=t_ms,raw_pitch,filtpitch,omega,pitch_offset,theta,metric,ref_state"));
        Serial.printf("ADV,PARAM,OMEGA_WEIGHT,%.4f\n", OMEGA_WEIGHT);
        Serial.printf("ADV,PARAM,OFF_RANGE_FRACTION,%.4f\n", OFF_RANGE_FRACTION);
        Serial.printf("ADV,PARAM,ON_RANGE_FRACTION,%.4f\n", OFF_RANGE_FRACTION);
        Serial.printf("ADV,PARAM,MIN_STATE_TIME_MS,%lu\n", (unsigned long)MIN_STATE_TIME_MS);
        Serial.printf("ADV,START,pitch_offset=%.2f,walk at different speeds, press a to stop.\n",
                      pitchOffset);
      }
    }
    else if (stage == ST_ADV)
    {
      // Full-rate advanced capture: every sample is logged so the Python
      // post-processor can recompute theta/metric offline and search for the
      // best OMEGA_WEIGHT / OFF_RANGE_FRACTION / ON_RANGE_FRACTION /
      // MIN_STATE_TIME_MS. ref_state is the on-board FSM reference (loaded
      // with the current constants) for comparison only.
      const float theta = filteredPitch - pitchOffset;
      const float metric = fabsf(theta) + OMEGA_WEIGHT * fabsf(omega);
      advMetricMin = min(advMetricMin, metric);
      advMetricMax = max(advMetricMax, metric);

      bool refNext = advFootOffGround;
      if (thresholdsCalibrated)
      {
        if (!advFootOffGround && metric >= offThreshold &&
            now - advLastStateChange >= MIN_STATE_TIME_MS)
          refNext = true;
        else if (advFootOffGround && metric <= onThreshold &&
                 now - advLastStateChange >= MIN_STATE_TIME_MS)
          refNext = false;
      }
      if (refNext != advFootOffGround)
      {
        advFootOffGround = refNext;
        advLastStateChange = now;
      }

      Serial.printf("ADV,S,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
                    (unsigned long)now, rawPitch, filteredPitch, omega,
                    pitchOffset, theta, metric,
                    thresholdsCalibrated ? (advFootOffGround ? 1 : 0) : -1);
      advSampleIndex++;
      if (now - advLastPrint >= CAL_STATUS_PERIOD_MS)
      {
        advLastPrint = now;
        Serial.printf("ADV,STAT,samples:%lu,metric=%.2f..%.2f\n",
                      (unsigned long)advSampleIndex, advMetricMin, advMetricMax);
      }
    }
    else if (stage == ST_FSM)
    {
      const float theta = filteredPitch - pitchOffset;
      const float metric = fabsf(theta) + OMEGA_WEIGHT * fabsf(omega);
      bool nextState = footOffGround;
      if (!footOffGround && metric >= offThreshold &&
          now - lastStateChange >= MIN_STATE_TIME_MS)
        nextState = true;
      else if (footOffGround && metric <= onThreshold &&
               now - lastStateChange >= MIN_STATE_TIME_MS)
        nextState = false;

      if (nextState != footOffGround)
      {
        footOffGround = nextState;
        lastStateChange = now;
        Serial.printf("FSM,%s,theta=%.2f,omega=%.2f,metric=%.2f\n",
                      stateName(footOffGround), theta, omega, metric);
        queueMotorCommand(footOffGround);
      }
      if (now - lastDebug >= 100)
      {
        lastDebug = now;
        Serial.printf("FSM,%s,theta=%.2f,omega=%.2f,metric=%.2f\n",
                      stateName(footOffGround), theta, omega, metric);
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL);
  if (!bno.begin(OPERATION_MODE_IMUPLUS))
  {
    Serial.println(F("ERROR,BNO055_NOT_FOUND"));
    while (true) delay(1000);
  }
  delay(1000);
  if (hasImuCalibration())
    loadCalibrationOffsets();
  else
    Serial.println(F("CAL,IMU_REQUIRED,no calibration data in Preferences."));

  Serial.printf("WIFI,CONNECTING,%s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const uint32_t wifiDeadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < wifiDeadline)
  {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("WIFI,CONNECTED,%s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println(F("WIFI,ERROR,CONNECTION_TIMEOUT"));

  motorQueue = xQueueCreate(MOTOR_QUEUE_LEN, sizeof(MotorCommand));
  if (motorQueue == nullptr)
  {
    Serial.println(F("ERROR,MOTOR_QUEUE_CREATE_FAILED"));
    while (true) delay(1000);
  }
  if (imuCalibrated)
    Serial.println(F("READY,type s = still cal (5s), w = walk cal (8s), f = FSM mode, a = advanced capture, d = show params."));
  else
    Serial.println(F("READY,IMU calibration required. type i to start."));
  xTaskCreatePinnedToCore(imuTask, "imu_rt", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(motorTask, "motor_http", 4096, nullptr, 3, nullptr, 0);
}

void loop(void)
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
