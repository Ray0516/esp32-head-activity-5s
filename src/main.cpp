#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include "auritus_activity_forest.h"

namespace {

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr uint32_t SERIAL_BAUD = 230400;
constexpr uint32_t FIFO_POLL_PERIOD_US = 50000;
constexpr uint32_t STATE_OUTPUT_PERIOD_MS = 5000;
constexpr uint32_t SAMPLE_STALE_MS = 300;
constexpr uint16_t WINDOW_SAMPLES = 1000;  // Source-compatible 10 s at 100 Hz.
constexpr uint16_t MODEL_UPDATE_SAMPLES = 100;
constexpr uint16_t MIN_MODEL_SAMPLES = 200;
constexpr uint8_t FIFO_FRAME_BYTES = 12;
constexpr uint8_t MAX_FIFO_FRAMES = 8;
constexpr float LPF_ALPHA = 0.239057f;  // 5 Hz low-pass at 100 Hz.
constexpr char FIRMWARE_ID[] = "head-activity-5s-v1";

// Screenshot values are retained as an internal baseline. API BIAS values
// start at zero, so zero means this exact behaviour.
constexpr int8_t BASELINE_BIAS[4] = {3, 2, 5, -3};

struct SensorReading {
  float ax, ay, az;
  float gx, gy, gz;
};

struct HeadFrame {
  float ax, ay, az;
  float gx, gy, gz;
};

MPU6050 imu(MPU_ADDRESS, &Wire);
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float filteredAx = 0.0f;
float filteredAy = 0.0f;
float filteredAz = 0.0f;
float filteredGx = 0.0f;
float filteredGy = 0.0f;
float filteredGz = 0.0f;
bool filterReady = false;

float accMagnitude[WINDOW_SAMPLES]{};
float gyroMagnitude[WINDOW_SAMPLES]{};
uint16_t historyIndex = 0;
uint16_t historyCount = 0;
uint16_t samplesSinceDecision = 0;

int8_t userBias[4] = {0, 0, 0, 0};
uint8_t currentClass = 0;
uint8_t pendingClass = 0;
uint8_t pendingCount = 0;
float confidence = 0.0f;
float lastFeatures[AURITUS_FEATURE_COUNT]{};
float classScores[4]{};
bool modelReady = false;

uint32_t lastPollUs = 0;
uint32_t lastOutputMs = 0;
uint32_t lastValidSampleMs = 0;
uint32_t imuReadyMs = 0;
uint32_t sequenceNumber = 0;
uint32_t i2cErrorCount = 0;
uint32_t recoveryCount = 0;
uint32_t fifoDropCount = 0;

int16_t signedValue(const uint8_t *raw, uint8_t index) {
  return static_cast<int16_t>((static_cast<uint16_t>(raw[index]) << 8) |
                              raw[index + 1]);
}

bool readRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
  return I2Cdev::readBytes(MPU_ADDRESS, reg, length, data, 20, &Wire) == length;
}

void clearHistory() {
  historyIndex = 0;
  historyCount = 0;
  samplesSinceDecision = 0;
  filterReady = false;
  modelReady = false;
  pendingClass = currentClass;
  pendingCount = 0;
  lastValidSampleMs = 0;
}

bool configureImu() {
  uint8_t deviceId = 0;
  if (!readRegisters(MPU6050_RA_WHO_AM_I, &deviceId, 1) ||
      (deviceId != 0x68 && deviceId != 0x72)) {
    return false;
  }
  imu.reset();
  delay(100);
  imu.initialize(ACCEL_FS::A4G, GYRO_FS::G500DPS);
  imu.setDLPFMode(MPU6050_DLPF_BW_20);
  imu.setRate(9);  // 100 Hz.
  imu.setFIFOEnabled(false);
  imu.setTempFIFOEnabled(false);
  imu.setAccelFIFOEnabled(true);
  imu.setXGyroFIFOEnabled(true);
  imu.setYGyroFIFOEnabled(true);
  imu.setZGyroFIFOEnabled(true);
  imu.resetFIFO();
  imu.setFIFOEnabled(true);
  delay(30);

  uint8_t fifoEnable = 0;
  uint8_t userControl = 0;
  return readRegisters(MPU6050_RA_FIFO_EN, &fifoEnable, 1) &&
         readRegisters(MPU6050_RA_USER_CTRL, &userControl, 1) &&
         (fifoEnable & 0x78) == 0x78 && (userControl & 0x40) != 0;
}

bool recoverImu() {
  Wire.end();
  delay(3);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL_PIN, HIGH);
  for (uint8_t i = 0; i < 9; ++i) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(6);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(6);
  }
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(6);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(6);
  digitalWrite(SDA_PIN, HIGH);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  I2Cdev::readTimeout = 20;
  clearHistory();
  const bool ready = configureImu();
  if (ready) imuReadyMs = millis();
  return ready;
}

void calibrateGyroscope() {
  gyroBiasX = gyroBiasY = gyroBiasZ = 0.0f;
  uint16_t valid = 0;
  uint8_t raw[14]{};
  for (uint16_t i = 0; i < 200; ++i) {
    if (readRegisters(MPU6050_RA_ACCEL_XOUT_H, raw, sizeof(raw))) {
      gyroBiasX += signedValue(raw, 8) / 65.5f;
      gyroBiasY += signedValue(raw, 10) / 65.5f;
      gyroBiasZ += signedValue(raw, 12) / 65.5f;
      ++valid;
    }
    delay(10);
  }
  if (valid) {
    gyroBiasX /= valid;
    gyroBiasY /= valid;
    gyroBiasZ /= valid;
  }
  uint8_t ignored = 0;
  readRegisters(MPU6050_RA_INT_STATUS, &ignored, 1);
  imu.resetFIFO();
  fifoDropCount = 0;
}

bool readFifo(SensorReading *frames, uint8_t &frameCount) {
  frameCount = 0;
  uint8_t status = 0;
  uint8_t countBytes[2]{};
  if (!readRegisters(MPU6050_RA_INT_STATUS, &status, 1) ||
      !readRegisters(MPU6050_RA_FIFO_COUNTH, countBytes, 2)) {
    return false;
  }
  uint16_t fifoCount = (static_cast<uint16_t>(countBytes[0]) << 8) | countBytes[1];
  if ((status & (1U << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    fifoDropCount += fifoCount / FIFO_FRAME_BYTES;
    imu.resetFIFO();
    clearHistory();
    return true;
  }
  uint16_t completeFrames = fifoCount / FIFO_FRAME_BYTES;
  if (!completeFrames) return true;

  uint8_t raw[MAX_FIFO_FRAMES * FIFO_FRAME_BYTES]{};
  while (completeFrames > MAX_FIFO_FRAMES) {
    const uint8_t discard = static_cast<uint8_t>(
        min<uint16_t>(completeFrames - MAX_FIFO_FRAMES, MAX_FIFO_FRAMES));
    if (!readRegisters(MPU6050_RA_FIFO_R_W, raw, discard * FIFO_FRAME_BYTES)) {
      return false;
    }
    fifoDropCount += discard;
    completeFrames -= discard;
  }
  const uint8_t byteCount = completeFrames * FIFO_FRAME_BYTES;
  if (!readRegisters(MPU6050_RA_FIFO_R_W, raw, byteCount)) return false;

  frameCount = static_cast<uint8_t>(completeFrames);
  for (uint8_t i = 0; i < frameCount; ++i) {
    const uint8_t *p = raw + i * FIFO_FRAME_BYTES;
    frames[i] = {
        signedValue(p, 0) / 8192.0f,
        signedValue(p, 2) / 8192.0f,
        signedValue(p, 4) / 8192.0f,
        signedValue(p, 6) / 65.5f,
        signedValue(p, 8) / 65.5f,
        signedValue(p, 10) / 65.5f};
  }
  return true;
}

HeadFrame translateToModel(const SensorReading &sensor) {
  HeadFrame frame{
      constrain(sensor.ax, -4.0f, 4.0f),
      constrain(sensor.ay, -4.0f, 4.0f),
      constrain(sensor.az, -4.0f, 4.0f),
      constrain(sensor.gx - gyroBiasX, -500.0f, 500.0f),
      constrain(sensor.gy - gyroBiasY, -500.0f, 500.0f),
      constrain(sensor.gz - gyroBiasZ, -500.0f, 500.0f)};
  if (!filterReady) {
    filteredAx = frame.ax; filteredAy = frame.ay; filteredAz = frame.az;
    filteredGx = frame.gx; filteredGy = frame.gy; filteredGz = frame.gz;
    filterReady = true;
  } else {
    filteredAx += LPF_ALPHA * (frame.ax - filteredAx);
    filteredAy += LPF_ALPHA * (frame.ay - filteredAy);
    filteredAz += LPF_ALPHA * (frame.az - filteredAz);
    filteredGx += LPF_ALPHA * (frame.gx - filteredGx);
    filteredGy += LPF_ALPHA * (frame.gy - filteredGy);
    filteredGz += LPF_ALPHA * (frame.gz - filteredGz);
  }
  return {filteredAx, filteredAy, filteredAz,
          filteredGx, filteredGy, filteredGz};
}

void magnitudeFeatures(const float *values, uint16_t count, float *features) {
  double sum = 0.0;
  double squares = 0.0;
  for (uint16_t i = 0; i < count; ++i) {
    sum += values[i];
    squares += static_cast<double>(values[i]) * values[i];
  }
  const float mean = static_cast<float>(sum / count);
  const float variance = count > 1
      ? max(0.0f, static_cast<float>((squares - sum * sum / count) / (count - 1)))
      : 0.0f;
  features[0] = mean;
  features[1] = sqrtf(variance);
  features[2] = variance;
}

uint8_t evaluateTree(uint16_t nodeIndex, const float *features) {
  while (nodeIndex < AURITUS_NODE_COUNT) {
    AuritusTreeNode node{};
    memcpy_P(&node, &AURITUS_NODES[nodeIndex], sizeof(node));
    if (node.feature < 0) return node.classId;
    nodeIndex = features[node.feature] <= node.threshold ? node.left : node.right;
  }
  return 0;
}

void applyHysteresis(uint8_t candidate) {
  if (candidate == currentClass) {
    pendingClass = candidate;
    pendingCount = 0;
  } else if (candidate != pendingClass) {
    pendingClass = candidate;
    pendingCount = 1;
  } else if (++pendingCount >= 2) {
    currentClass = candidate;
    pendingCount = 0;
  }
}

void classify() {
  if (historyCount < MIN_MODEL_SAMPLES) return;
  static float orderedAcc[WINDOW_SAMPLES];
  static float orderedGyro[WINDOW_SAMPLES];
  const uint16_t first = (historyIndex + WINDOW_SAMPLES - historyCount) % WINDOW_SAMPLES;
  for (uint16_t i = 0; i < historyCount; ++i) {
    const uint16_t source = (first + i) % WINDOW_SAMPLES;
    orderedAcc[i] = accMagnitude[source];
    orderedGyro[i] = gyroMagnitude[source];
  }
  magnitudeFeatures(orderedAcc, historyCount, lastFeatures);
  magnitudeFeatures(orderedGyro, historyCount, lastFeatures + 3);

  uint8_t votes[4]{};
  for (uint8_t tree = 0; tree < AURITUS_TREE_COUNT; ++tree) {
    const uint16_t root = pgm_read_word(&AURITUS_TREE_ROOTS[tree]);
    const uint8_t result = evaluateTree(root, lastFeatures);
    if (result < 4) ++votes[result];
  }

  uint8_t best = 0;
  float bestAdjusted = -100.0f;
  for (uint8_t candidate = 0; candidate < 4; ++candidate) {
    const float voteRatio = static_cast<float>(votes[candidate]) / AURITUS_TREE_COUNT;
    classScores[candidate] = voteRatio;
    const int8_t effectiveBias = BASELINE_BIAS[candidate] + userBias[candidate];
    const float adjusted = voteRatio + effectiveBias * 0.045f;
    if (adjusted > bestAdjusted) {
      bestAdjusted = adjusted;
      best = candidate;
    }
  }
  confidence = classScores[best];
  modelReady = true;
  applyHysteresis(best);
}

void processSample(const SensorReading &sensor) {
  const HeadFrame head = translateToModel(sensor);
  accMagnitude[historyIndex] = sqrtf(
      head.ax * head.ax + head.ay * head.ay + head.az * head.az);
  gyroMagnitude[historyIndex] = sqrtf(
      head.gx * head.gx + head.gy * head.gy + head.gz * head.gz);
  historyIndex = (historyIndex + 1) % WINDOW_SAMPLES;
  historyCount = min<uint16_t>(historyCount + 1, WINDOW_SAMPLES);
  if (++samplesSinceDecision >= MODEL_UPDATE_SAMPLES) {
    samplesSinceDecision = 0;
    classify();
  }
}

const char *className(uint8_t classId) {
  static const char *const names[4] = {"still", "light", "moderate", "high"};
  return names[classId < 4 ? classId : 0];
}

void emitState(uint32_t now, bool force = false) {
  if (!force && now - lastOutputMs < STATE_OUTPUT_PERIOD_MS) return;
  lastOutputMs = now;
  const bool sensorReady = lastValidSampleMs && now - lastValidSampleMs < 500;
  const bool valid = sensorReady && modelReady && historyCount == WINDOW_SAMPLES;
  Serial.printf(
      "{\"type\":\"activity_state\",\"version\":1,\"firmware\":\"%s\","
      "\"seq\":%lu,\"t\":%lu,\"sensor\":%s,\"valid\":%s,"
      "\"activity\":\"%s\",\"activityLevel\":%u,\"confidence\":%.3f,"
      "\"ttlMs\":6500}\n",
      FIRMWARE_ID, ++sequenceNumber, now,
      sensorReady ? "true" : "false", valid ? "true" : "false",
      className(currentClass), currentClass, confidence);
}

void processCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("API GET")) {
    Serial.printf(
        "{\"type\":\"config\",\"outputPeriodMs\":5000,\"model\":\"auritus-21-tree\","
        "\"windowSamples\":%u,\"sampleRateHz\":100,\"bias\":[%d,%d,%d,%d]}\n",
        WINDOW_SAMPLES, userBias[0], userBias[1], userBias[2], userBias[3]);
    return;
  }
  if (command.equalsIgnoreCase("API STATUS")) {
    emitState(millis(), true);
    return;
  }
  if (command.equalsIgnoreCase("API MODEL")) {
    Serial.printf(
        "{\"type\":\"model_state\",\"trees\":%u,\"nodes\":%u,"
        "\"features\":[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f],"
        "\"scores\":[%.3f,%.3f,%.3f,%.3f]}\n",
        AURITUS_TREE_COUNT, AURITUS_NODE_COUNT,
        lastFeatures[0], lastFeatures[1], lastFeatures[2],
        lastFeatures[3], lastFeatures[4], lastFeatures[5],
        classScores[0], classScores[1], classScores[2], classScores[3]);
    return;
  }
  if (command.equalsIgnoreCase("API RESTART")) {
    const bool recovered = recoverImu();
    Serial.printf("{\"type\":\"ack\",\"api\":\"restart\",\"recovered\":%s}\n",
                  recovered ? "true" : "false");
    return;
  }
  int classId = -1;
  int bias = 0;
  if (sscanf(command.c_str(), "API BIAS %d %d", &classId, &bias) == 2 &&
      classId >= 0 && classId < 4) {
    userBias[classId] = constrain(bias, -3, 3);
    Serial.printf("{\"type\":\"ack\",\"api\":\"bias\",\"class\":%d,\"value\":%d}\n",
                  classId, userBias[classId]);
  }
}

void handleCommands() {
  static char buffer[64];
  static uint8_t length = 0;
  static bool overflow = false;
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (!overflow) {
        buffer[length] = '\0';
        processCommand(String(buffer));
      }
      length = 0;
      overflow = false;
    } else if (c != '\r') {
      if (length < sizeof(buffer) - 1) buffer[length++] = c;
      else overflow = true;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  I2Cdev::readTimeout = 20;

  while (!configureImu()) {
    emitState(millis());
    handleCommands();
    recoverImu();
    delay(100);
  }
  calibrateGyroscope();
  imuReadyMs = millis();
  lastPollUs = micros();
}

void loop() {
  handleCommands();
  const uint32_t nowMs = millis();
  emitState(nowMs);
  const bool sampleStale = lastValidSampleMs
      ? nowMs - lastValidSampleMs > SAMPLE_STALE_MS
      : nowMs - imuReadyMs > 1000;
  if (sampleStale) {
    recoverImu();
    ++recoveryCount;
    lastPollUs = micros();
    return;
  }

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastPollUs) < FIFO_POLL_PERIOD_US) return;
  lastPollUs = nowUs;

  SensorReading frames[MAX_FIFO_FRAMES]{};
  uint8_t frameCount = 0;
  if (!readFifo(frames, frameCount)) {
    ++i2cErrorCount;
    recoverImu();
    ++recoveryCount;
    lastPollUs = micros();
    return;
  }
  if (!frameCount) return;
  lastValidSampleMs = nowMs;
  for (uint8_t i = 0; i < frameCount; ++i) processSample(frames[i]);
}
