#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "webpage.h"

#define SDA_PIN D4
#define SCL_PIN D5

static const uint32_t SAMPLE_PERIOD_MS = 10;
static const uint32_t SERIAL_BAUD = 115200;
static const char *WIFI_SSID = "MotorControl";
static const char *WIFI_PASSWORD = "12345678";
static const char *MOTOR_HOST = "192.168.4.1";
static const uint16_t MOTOR_PORT = 80;
static const uint32_t MOTOR_HTTP_TIMEOUT_MS = 300;
static const char *DEVICE_HOSTNAME = "RIMU";

static const uint32_t STILL_CAL_MS = 5000;
static const uint32_t WALK_CAL_MS = 8000;
static const uint32_t CAL_STATUS_PERIOD_MS = 1000;

/* Right-shank mounting: X is along the shank, Y is forward/back. */
static const int SHANK_AXIS = 0;
static const int AP_AXIS = 1;

/* Signed swing metric:  metric = polarity * (theta + OMEGA_WEIGHT * omega).
   The metric MUST stay signed. fabsf() would make it an even function, which
   produces two peaks per stride whenever the walking posture differs from the
   still-calibrated pitch offset, and the FSM then switches twice per step.
   SWING_POLARITY (+1/-1) is auto-detected while calibrating ('w') or capturing
   ('a'), so the swing rotation direction never has to be hard-coded. */
static const float OMEGA_WEIGHT = 0.10f;
static const float OFF_RANGE_FRACTION = 0.55f;
static const float ON_RANGE_FRACTION = 0.45f;
static const uint32_t MIN_STATE_TIME_MS = 60;

/* Mode 'a' (advanced capture): auto pitch-offset calibration window before
   full-rate logging starts. The log is post-processed by imu_adv_tune.py to
   back-calculate the four constants above. */
static const uint32_t ADV_STILL_MS = 3000;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/* Motor commands are HTTP paths. The FSM only triggers a one-shot swing on
   FOOT_OFF_GROUND; the motor controller ends the motion by itself, so no
   "stop" path is ever sent. */
static const char *MOTOR_PATH_ONESHOT_RIGHT = "/oneshot_right";
static const size_t MOTOR_PATH_MAX = 24;

struct MotorCommand {
  char path[MOTOR_PATH_MAX];
};

static QueueHandle_t motorQueue = nullptr;
static const uint8_t MOTOR_QUEUE_LEN = 4;
static QueueHandle_t webCommandQueue = nullptr;
static WebServer webServer(80);

struct WebTelemetry {
  float rawPitch;
  float filteredPitch;
  float omega;
  bool footOffGround;
};

static volatile WebTelemetry webTelemetry = {0.0f, 0.0f, 0.0f, false};
static volatile float webPitchOffset = 0.0f;
static volatile float webOnThreshold = 0.0f;
static volatile float webOffThreshold = 0.0f;

/* Status log for the webpage: every line that webLog() prints to the serial
   port is also kept in this small ring buffer, so the browser can show the
   same status the serial monitor shows. Only the 1 Hz status/event lines are
   logged; the high-rate FSM debug and ADV sample lines stay serial-only. */
static const uint8_t WEB_LOG_LINES = 12;
static const uint8_t WEB_LOG_LEN = 128;
static char webLogBuf[WEB_LOG_LINES][WEB_LOG_LEN];
static uint8_t webLogHead = 0;
static uint8_t webLogCount = 0;
static SemaphoreHandle_t webLogMutex = nullptr;

/* Mirrors the imuTask Stage enum, in the same order, for the status page. */
static volatile uint8_t webStageId = 0;
static const char *const WEB_STAGE_NAMES[] = {
  "IDLE", "IMU_CAL", "STILL", "WALK", "FSM", "ADV_CAL", "ADV"
};
static const uint8_t WEB_STAGE_COUNT =
  sizeof(WEB_STAGE_NAMES) / sizeof(WEB_STAGE_NAMES[0]);

static void webLogPush(const char *line)
{
  const bool locked = (webLogMutex != nullptr &&
                       xSemaphoreTake(webLogMutex, pdMS_TO_TICKS(20)) == pdTRUE);
  snprintf(webLogBuf[webLogHead], WEB_LOG_LEN, "%s", line);
  webLogHead = (uint8_t)((webLogHead + 1) % WEB_LOG_LINES);
  if (webLogCount < WEB_LOG_LINES) webLogCount++;
  if (locked) xSemaphoreGive(webLogMutex);
}

/* Print a status line to the serial port and keep it for the webpage. */
static void webLog(const char *fmt, ...)
            __attribute__((format(printf, 1, 2)));

static void webLog(const char *fmt, ...)
{
  char line[WEB_LOG_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  Serial.println(line);
  webLogPush(line);
}

static void jsonAppendEscaped(String &out, const char *s)
{
  for (; *s; s++)
  {
    const unsigned char c = (unsigned char)*s;
    if (c == '"' || c == '\\') { out += '\\'; out += (char)c; }
    else if (c < 0x20u) out += ' ';
    else out += (char)c;
  }
}

/* Append the log as a JSON array of strings, oldest entry first. */
static void appendWebLogJson(String &json)
{
  json += ",\"log\":[";
  const bool locked = (webLogMutex != nullptr &&
                       xSemaphoreTake(webLogMutex, pdMS_TO_TICKS(20)) == pdTRUE);
  const uint8_t head = webLogHead;
  const uint8_t count = webLogCount;
  for (uint8_t i = 0; i < count; i++)
  {
    const uint8_t idx = (uint8_t)((head - count + i + 2 * WEB_LOG_LINES) % WEB_LOG_LINES);
    if (i) json += ',';
    json += '"';
    jsonAppendEscaped(json, webLogBuf[idx]);
    json += '"';
  }
  if (locked) xSemaphoreGive(webLogMutex);
  json += ']';
}

/* BNO055 sensor calibration (accel/gyro/mag) required before use. */
static bool imuCalibrated = false;

/* +1 when the swing phase rotates theta positive, -1 when it rotates theta
   negative. Auto-detected from the walk calibration window (polarityFrom). */
static float swingPolarity = 1.0f;

/* Signed swing metric. theta + w*omega leads theta alone, which detects
   toe-off earlier; keeping it signed keeps one peak per stride. */
static float swingMetric(float theta, float omega)
{
  return swingPolarity * (theta + OMEGA_WEIGHT * omega);
}

/* omega*|omega| weights each sample by its own speed, so the sum over a walk
   window is dominated by the fast swing rotation and its sign is the polarity. */
static float polarityFrom(float polarityAccum)
{
  return (polarityAccum >= 0.0f) ? 1.0f : -1.0f;
}

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

static void queueMotorCommand(const char *path)
{
  if (motorQueue == nullptr) return;
  MotorCommand command;
  snprintf(command.path, sizeof(command.path), "%s", path);
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
      webLog("BNO055 calibration loaded.");
      printCalibOffsets("LOAD", calibData);
    }
    else webLog("Warning: calibration data missing.");
  }
  else webLog("Warning: no matching BNO055 calibration.");
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
  webLog("BNO055 calibration saved.");
  printCalibOffsets("SAVE", calibData);
}

static void printImuCalStatus()
{
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  webLog("CAL,IMU,SYS:%d,G:%d,A:%d,M:%d", system, gyro, accel, mag);
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

static void handleWebCommand()
{
  if (!webServer.hasArg("c") || webServer.arg("c").length() != 1)
  {
    webServer.send(400, "application/json",
                   "{\"error\":\"missing command\"}");
    return;
  }

  const char command = webServer.arg("c")[0];
  if (command != 'i' && command != 's' && command != 'w' &&
      command != 'f' && command != 'd')
  {
    webServer.send(400, "application/json",
                   "{\"error\":\"unsupported command\"}");
    return;
  }

  if (webCommandQueue == nullptr ||
      xQueueSend(webCommandQueue, &command, 0) != pdTRUE)
  {
    webServer.send(503, "application/json",
                   "{\"error\":\"command queue full\"}");
    return;
  }
  webServer.send(200, "application/json", "{\"message\":\"command sent\"}");
}

static void handleWebStatus()
{
  const bool connected = WiFi.status() == WL_CONNECTED;
  String json = "{\"connected\":";
  json += connected ? "true" : "false";
  json += ",\"wifi\":\"";
  json += connected ? "connected" : "disconnected";
  json += "\",\"ip\":\"";
  json += connected ? WiFi.localIP().toString() : "";
  json += "\",\"hostname\":\"http://RIMU.local/\"";
  json += ",\"rawPitch\":" + String(webTelemetry.rawPitch, 2);
  json += ",\"filteredPitch\":" + String(webTelemetry.filteredPitch, 2);
  json += ",\"omega\":" + String(webTelemetry.omega, 2);
  json += ",\"footOffGround\":";
  json += webTelemetry.footOffGround ? "true" : "false";
  const uint8_t stageId = webStageId;
  json += ",\"mode\":\"";
  json += stageId < WEB_STAGE_COUNT ? WEB_STAGE_NAMES[stageId] : "?";
  json += "\"";
  appendWebLogJson(json);
  json += "}";
  webServer.send(200, "application/json", json);
}

static void handleWebParams()
{
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  String json = "{\"OMEGA_WEIGHT\":" + String(OMEGA_WEIGHT, 4);
  json += ",\"OFF_RANGE_FRACTION\":" + String(OFF_RANGE_FRACTION, 4);
  json += ",\"ON_RANGE_FRACTION\":" + String(ON_RANGE_FRACTION, 4);
  json += ",\"MIN_STATE_TIME_MS\":" + String(MIN_STATE_TIME_MS);
  json += ",\"SWING_POLARITY\":";
  json += swingPolarity > 0.0f ? "1" : "-1";
  json += ",\"imuCalibrated\":";
  json += imuCalibrated ? "true" : "false";
  json += ",\"pitchOffset\":" + String(webPitchOffset, 2);
  json += ",\"onThreshold\":" + String(webOnThreshold, 2);
  json += ",\"offThreshold\":" + String(webOffThreshold, 2);
  json += ",\"calibration\":{\"system\":" + String(system);
  json += ",\"gyro\":" + String(gyro) + ",\"accel\":" + String(accel);
  json += ",\"mag\":" + String(mag) + "}}";
  webServer.send(200, "application/json", json);
}

static void startWebServer()
{
  webServer.on("/", HTTP_GET, []() {
    webServer.send_P(200, "text/html", WEBPAGE_HTML);
  });
  webServer.on("/api/command", HTTP_GET, handleWebCommand);
  webServer.on("/api/status", HTTP_GET, handleWebStatus);
  webServer.on("/api/params", HTTP_GET, handleWebParams);
  webServer.begin();
  webLog("WEB,READY,http://RIMU.local/");
}

/* Dump every value stored in Preferences (namespace "bno055") plus the live
   BNO055 calibration level, for the 'd' command. */
static void dumpStoredCalibration()
{
  Preferences prefs;
  sensor_t sensor;
  bno.getSensor(&sensor);
  prefs.begin("bno055", true);

  if (prefs.isKey("bnoID"))
  {
    const long storedBnoID = prefs.getLong("bnoID", 0);
    Serial.printf("PREF,bno055,bnoID,%ld,matching:%s\n",
                  storedBnoID, storedBnoID == (long)sensor.sensor_id ? "YES" : "NO");
  }
  else Serial.println(F("PREF,bno055,bnoID,MISSING"));

  if (prefs.isKey("calibData"))
  {
    adafruit_bno055_offsets_t calibData;
    const size_t n = prefs.getBytes("calibData", &calibData, sizeof(calibData));
    if (n == sizeof(calibData)) printCalibOffsets("PREF", calibData);
    else Serial.printf("PREF,bno055,calibData,SIZE_MISMATCH,%u/%u\n",
                       (unsigned)n, (unsigned)sizeof(calibData));
  }
  else Serial.println(F("PREF,bno055,calibData,MISSING"));

  prefs.end();
  printImuCalStatus();
}

static bool motorHttpGet(const char *path)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    webLog("MOTOR,ERROR,WIFI_DISCONNECTED,%s", path);
    return false;
  }

  WiFiClient client;
  client.setTimeout(MOTOR_HTTP_TIMEOUT_MS);
  if (!client.connect(MOTOR_HOST, MOTOR_PORT))
  {
    webLog("MOTOR,ERROR,CONNECT_FAILED,%s", path);
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
  webLog("MOTOR,%s,%s", path,
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
      motorHttpGet(command.path);
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
  /* Accumulates omega*|omega| over a walk window to auto-detect swing polarity. */
  float polarityAccum = 0.0f;
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
    webTelemetry.rawPitch = rawPitch;
    webTelemetry.filteredPitch = filteredPitch;
    webTelemetry.omega = omega;
    webTelemetry.footOffGround = footOffGround;

    // Serial commands: i = IMU calibration, s = still calibration,
    // w = walk calibration, f = FSM mode, a = advanced capture (tuning log),
    // d = dump current FSM parameters.
    while (Serial.available() || (webCommandQueue != nullptr &&
                                  uxQueueMessagesWaiting(webCommandQueue) > 0))
    {
      char c;
      if (Serial.available())
        c = (char)Serial.read();
      else if (xQueueReceive(webCommandQueue, &c, 0) != pdTRUE)
        break;
      if (c == 'i')
      {
        // Reset the FSM state if it was running before calibration. The motor
        // one-shot finishes on its own, so nothing has to be sent to stop it.
        footOffGround = false;
        stage = ST_IMU_CAL;
        stageStart = now;
        lastCalStatus = now;
        webLog("CAL,IMU_START,stand still then draw figure 8 with the sensor.");
      }
      else if (c == 'd')
      {
        Serial.printf("PARAM,OMEGA_WEIGHT,%.4f\n", OMEGA_WEIGHT);
        Serial.printf("PARAM,OFF_RANGE_FRACTION,%.4f\n", OFF_RANGE_FRACTION);
        Serial.printf("PARAM,ON_RANGE_FRACTION,%.4f\n", ON_RANGE_FRACTION);
        Serial.printf("PARAM,MIN_STATE_TIME_MS,%lu\n", (unsigned long)MIN_STATE_TIME_MS);
        Serial.printf("PARAM,SWING_POLARITY,%d\n", swingPolarity > 0.0f ? 1 : -1);
        Serial.printf("PARAM,PITCH_OFFSET,%.2f\n", pitchOffset);
        Serial.printf("PARAM,ON_THRESHOLD,%.2f\n", onThreshold);
        Serial.printf("PARAM,OFF_THRESHOLD,%.2f\n", offThreshold);
        dumpStoredCalibration();
      }
      else if (c == 'a')
      {
        if (stage == ST_ADV || stage == ST_ADV_CAL)
        {
          stage = ST_IDLE;
          // The auto pitch offset taken here is capture-specific; do not let
          // the FSM reuse it without a fresh still calibration.
          thresholdsCalibrated = false;
          webLog("ADV,STOP,samples:%lu,metric=%.2f..%.2f",
                 (unsigned long)advSampleIndex, advMetricMin, advMetricMax);
        }
        else if (!imuCalibrated)
        {
          webLog("REJECTED,run IMU calibration first (i).");
        }
        else
        {
          stage = ST_ADV_CAL;
          stageStart = now;
          lastCalStatus = now;
          stillPitchSum = 0.0f;
          stillCount = 0;
          webLog("ADV,STILL_START,stand still for 3 seconds.");
        }
      }
      else if (c == 's' || c == 'w' || c == 'f')
      {
        if (!imuCalibrated)
        {
          webLog("REJECTED,run IMU calibration first (i).");
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
          webLog("CAL,STILL_START,stand still for 5 seconds.");
        }
        else if (c == 'w')
        {
          if (!pitchCalibrated)
            webLog("CAL,WALK_REJECTED,run still calibration first (s).");
          else
          {
            stage = ST_WALK;
            walkMetricMin = 1.0e9f;
            walkMetricMax = -1.0e9f;
            polarityAccum = 0.0f;
            webLog("CAL,WALK_START,walk naturally for 8 seconds.");
          }
        }
        else if (c == 'f')
        {
          if (!pitchCalibrated || !thresholdsCalibrated)
            webLog("FSM,REJECTED,calibrate first: s then w.");
          else
          {
            stage = ST_FSM;
            footOffGround = false;
            lastStateChange = now;
            webLog("FSM,FOOT_ON_GROUND,STARTED");
          }
        }
      }
    }

    if (webStageId != (uint8_t)stage)
    {
      webStageId = (uint8_t)stage;
      webTelemetry.footOffGround = footOffGround;
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
        webLog("CAL,IMU_DONE,calibration saved to Preferences.");
      }
    }
    else if (stage == ST_STILL)
    {
      stillPitchSum += filteredPitch;
      stillCount++;
      if (now - lastCalStatus >= CAL_STATUS_PERIOD_MS)
      {
        lastCalStatus = now;
        webLog("CAL,STILL,%lu/%lu ms",
               (unsigned long)(now - stageStart),
               (unsigned long)STILL_CAL_MS);
      }
      if (now - stageStart >= STILL_CAL_MS && stillCount > 20)
      {
        pitchOffset = stillPitchSum / stillCount;
        webPitchOffset = pitchOffset;
        pitchCalibrated = true;
        stage = ST_IDLE;
        webLog("CAL,STILL_DONE,pitch_offset=%.2f", pitchOffset);
      }
    }
    else if (stage == ST_WALK)
    {
      const float theta = filteredPitch - pitchOffset;
      polarityAccum += omega * fabsf(omega);
      // Track the raw signed value s = theta + w*omega. The metric is
      // polarity*s, so a negative polarity just swaps and negates min/max.
      const float s = theta + OMEGA_WEIGHT * omega;
      walkMetricMin = min(walkMetricMin, s);
      walkMetricMax = max(walkMetricMax, s);
      if (now - lastCalStatus >= CAL_STATUS_PERIOD_MS)
      {
        lastCalStatus = now;
        webLog("CAL,WALK,%lu/%lu ms,range=%.2f..%.2f",
               (unsigned long)(now - stageStart),
               (unsigned long)WALK_CAL_MS, walkMetricMin, walkMetricMax);
      }
      if (now - stageStart >= WALK_CAL_MS)
      {
        // Detect the swing rotation direction before deriving the thresholds,
        // so a reversed mounting does not need any constant changed.
        swingPolarity = polarityFrom(polarityAccum);
        const float mMin = (swingPolarity > 0.0f) ? walkMetricMin : -walkMetricMax;
        const float mMax = (swingPolarity > 0.0f) ? walkMetricMax : -walkMetricMin;
        const float range = max(1.0f, mMax - mMin);
        offThreshold = mMin + OFF_RANGE_FRACTION * range;
        onThreshold = mMin + ON_RANGE_FRACTION * range;
        if (onThreshold >= offThreshold) onThreshold = 0.5f * offThreshold;
        webOnThreshold = onThreshold;
        webOffThreshold = offThreshold;
        thresholdsCalibrated = true;
        stage = ST_IDLE;
        webLog("CAL,POLARITY,%s", swingPolarity > 0.0f ? "POS" : "NEG");
        webLog("CAL,DONE,pitch_offset=%.2f,on=%.2f,off=%.2f",
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
        webLog("ADV,STILL,%lu/%lu ms",
               (unsigned long)(now - stageStart),
               (unsigned long)ADV_STILL_MS);
      }
      if (now - stageStart >= ADV_STILL_MS && stillCount > 20)
      {
        pitchOffset = stillPitchSum / stillCount;
        webPitchOffset = pitchOffset;
        stage = ST_ADV;
        advSampleIndex = 0;
        advLastPrint = 0;
        advLastStateChange = now;
        advFootOffGround = false;
        advMetricMin = 1.0e9f;
        advMetricMax = -1.0e9f;
        polarityAccum = 0.0f;
        Serial.println(F("ADV,META,cols=t_ms,raw_pitch,filtpitch,omega,pitch_offset,thetxa,signed_metric,ref_state"));
        Serial.printf("ADV,PARAM,OMEGA_WEIGHT,%.4f\n", OMEGA_WEIGHT);
        Serial.printf("ADV,PARAM,OFF_RANGE_FRACTION,%.4f\n", OFF_RANGE_FRACTION);
        Serial.printf("ADV,PARAM,ON_RANGE_FRACTION,%.4f\n", ON_RANGE_FRACTION);
        Serial.printf("ADV,PARAM,MIN_STATE_TIME_MS,%lu\n", (unsigned long)MIN_STATE_TIME_MS);
        Serial.printf("ADV,PARAM,SWING_POLARITY,%d\n", swingPolarity > 0.0f ? 1 : -1);
        webLog("ADV,START,pitch_offset=%.2f,walk at different speeds, press a to stop.",
               pitchOffset);
      }
    }
    else if (stage == ST_ADV)
    {
      // Full-rate advanced capture: every sample is logged so the Python
      // post-processor can recompute the metric offline and search for the
      // best OMEGA_WEIGHT / OFF_RANGE_FRACTION / ON_RANGE_FRACTION /
      // MIN_STATE_TIME_MS. The logged metric is theta + w*omega WITHOUT the
      // polarity factor, so the log stays valid for either mounting direction
      // and imu_adv_tune.py detects the polarity itself. ref_state is the
      // on-board FSM reference (current constants) for comparison only.
      const float theta = filteredPitch - pitchOffset;
      const float s = theta + OMEGA_WEIGHT * omega;
      polarityAccum += omega * fabsf(omega);
      // Live estimate, used only while no calibrated polarity exists yet. The
      // reference FSM must use the same polarity its thresholds came from.
      const float advPolarity = thresholdsCalibrated ? swingPolarity
                                                     : polarityFrom(polarityAccum);
      const float metric = advPolarity * s;
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
                    pitchOffset, theta, s,
                    thresholdsCalibrated ? (advFootOffGround ? 1 : 0) : -1);
      advSampleIndex++;
      if (now - advLastPrint >= CAL_STATUS_PERIOD_MS)
      {
        advLastPrint = now;
        webLog("ADV,STAT,samples:%lu,metric=%.2f..%.2f,polarity:%s",
               (unsigned long)advSampleIndex, advMetricMin, advMetricMax,
               advPolarity > 0.0f ? "POS" : "NEG");
      }
    }
    else if (stage == ST_FSM)
    {
      const float theta = filteredPitch - pitchOffset;
      const float metric = swingMetric(theta, omega);
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
        webTelemetry.footOffGround = footOffGround;
        lastStateChange = now;
        webLog("FSM,%s,theta=%.2f,omega=%.2f,metric=%.2f",
               stateName(footOffGround), theta, omega, metric);
        /* Trigger the one-shot swing on toe-off only. FOOT_ON_GROUND needs no
           command because the one-shot motion stops by itself. */
        if (footOffGround) queueMotorCommand(MOTOR_PATH_ONESHOT_RIGHT);
      }
      if (now - lastDebug >= 100)
      {
        lastDebug = now;
        webTelemetry.footOffGround = footOffGround;
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
  webLogMutex = xSemaphoreCreateMutex();
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
    webLog("CAL,IMU_REQUIRED,no calibration data in Preferences.");

  webLog("WIFI,CONNECTING,%s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  const uint32_t wifiDeadline = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < wifiDeadline)
  {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
    webLog("WIFI,CONNECTED,%s", WiFi.localIP().toString().c_str());
  else
    webLog("WIFI,ERROR,CONNECTION_TIMEOUT");

  webCommandQueue = xQueueCreate(8, sizeof(char));
  if (webCommandQueue == nullptr)
  {
    Serial.println(F("ERROR,WEB_COMMAND_QUEUE_CREATE_FAILED"));
    while (true) delay(1000);
  }
  startWebServer();

  motorQueue = xQueueCreate(MOTOR_QUEUE_LEN, sizeof(MotorCommand));
  if (motorQueue == nullptr)
  {
    Serial.println(F("ERROR,MOTOR_QUEUE_CREATE_FAILED"));
    while (true) delay(1000);
  }
  if (imuCalibrated)
    webLog("READY,type s = still cal (5s), w = walk cal (8s), f = FSM mode, a = advanced capture, d = show params.");
  else
    webLog("READY,IMU calibration required. type i to start.");
  xTaskCreatePinnedToCore(imuTask, "imu_rt", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(motorTask, "motor_http", 4096, nullptr, 3, nullptr, 0);
}

void loop(void)
{
  webServer.handleClient();
  vTaskDelay(pdMS_TO_TICKS(10));
}
