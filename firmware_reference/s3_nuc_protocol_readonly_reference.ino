// KRAI NUC_S3_PROTOCOL_V1 reference snippet for ESP32-S3.
// Integrate this into the existing S3 firmware after the current BNO + Slave A telemetry baseline is stable.
// Milestone 1 is READ-ONLY: S3 sends health, odom, and base telemetry to NUC.

#define NUC_SERIAL Serial

#define NUC_SYNC1 0xAA
#define NUC_SYNC2 0x55
#define NUC_PROTOCOL_VERSION 0x01

#define NUC_MSG_ODOM_LOCAL       0x90
#define NUC_MSG_BASE_TELEMETRY   0x91
#define NUC_MSG_S3_HEALTH        0x92
#define NUC_MSG_PRIMITIVE_STATUS 0x93

uint8_t nucTxSeq = 0;
uint32_t nucRxCount = 0;
uint32_t nucCrcError = 0;
uint32_t lastNucCmdMs = 0;

// Match ROS bridge enum assumptions.
enum S3Mode : uint8_t {
  S3_MODE_IDLE = 0,
  S3_MODE_MANUAL = 1,
  S3_MODE_PRIMITIVE = 2,
  S3_MODE_SAFE_STOP = 3,
  S3_MODE_ESTOP = 4,
  S3_MODE_LOCAL_DEBUG = 5,
};

uint8_t s3Mode = S3_MODE_IDLE;
uint8_t primitiveStateForNuc = 0; // 0 idle, 1 running, 2 done, 3 failed, 4 canceled
uint16_t s3FaultFlags = 0;

struct __attribute__((packed)) OdomLocalPayload {
  int32_t x_cm_x10;
  int32_t y_cm_x10;
  int16_t heading_deg_x10;
  int32_t dist_cm_x10;
  int16_t linear_cm_s_x10;
  int16_t yaw_raw_deg_x10;
  uint8_t heading_ready;
  uint8_t reserved;
};

struct __attribute__((packed)) NucBaseTelemetryPayload {
  int32_t count_left;
  int32_t count_right;
  int16_t delta_left;
  int16_t delta_right;
  int16_t rpm_left_x10;
  int16_t rpm_right_x10;
  int16_t pwm_left;
  int16_t pwm_right;
  uint8_t slave_a_mode;
  uint8_t slave_a_fault_flags;
  uint16_t age_ms;
};

struct __attribute__((packed)) S3HealthPayload {
  uint32_t s3_time_ms;
  uint8_t mode;
  uint8_t heading_ready;
  uint8_t base_connected;
  uint8_t primitive_state;
  uint32_t bad_bno_frames;
  uint32_t yaw_jump_reject_count;
  uint32_t base_crc_error;
  uint32_t nuc_rx_count;
  uint32_t nuc_crc_error;
  uint16_t last_nuc_cmd_age_ms;
  uint16_t last_base_age_ms;
  uint16_t fault_flags;
};

uint16_t crc16CcittFalseUpdate(uint16_t crc, uint8_t data) {
  crc ^= ((uint16_t)data) << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

uint16_t crc16CcittFalse(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc16CcittFalseUpdate(crc, data[i]);
  }
  return crc;
}

int16_t floatToI16x10(float value) {
  long v = lroundf(value * 10.0f);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

int32_t floatToI32x10(float value) {
  double v = round((double)value * 10.0);
  if (v > 2147483647.0) v = 2147483647.0;
  if (v < -2147483648.0) v = -2147483648.0;
  return (int32_t)v;
}

void sendNucFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t seq = nucTxSeq++;
  uint8_t header[5];
  header[0] = NUC_PROTOCOL_VERSION;
  header[1] = type;
  header[2] = seq;
  header[3] = (uint8_t)(len & 0xFF);
  header[4] = (uint8_t)((len >> 8) & 0xFF);

  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < sizeof(header); i++) {
    crc = crc16CcittFalseUpdate(crc, header[i]);
  }
  for (uint16_t i = 0; i < len; i++) {
    crc = crc16CcittFalseUpdate(crc, payload[i]);
  }

  NUC_SERIAL.write(NUC_SYNC1);
  NUC_SERIAL.write(NUC_SYNC2);
  NUC_SERIAL.write(header, sizeof(header));
  if (payload != nullptr && len > 0) {
    NUC_SERIAL.write(payload, len);
  }
  NUC_SERIAL.write((uint8_t)(crc & 0xFF));
  NUC_SERIAL.write((uint8_t)((crc >> 8) & 0xFF));
}

void sendNucOdomLocal() {
  OdomLocalPayload p;
  p.x_cm_x10 = floatToI32x10(poseXcm);
  p.y_cm_x10 = floatToI32x10(poseYcm);
  p.heading_deg_x10 = floatToI16x10(getHeadingDeg());
  p.dist_cm_x10 = floatToI32x10(poseDistCm);
  p.linear_cm_s_x10 = floatToI16x10(poseLinearCmS);
  p.yaw_raw_deg_x10 = floatToI16x10(getYawRawDeg());
  p.heading_ready = isHeadingReady() ? 1 : 0;
  p.reserved = 0;

  sendNucFrame(NUC_MSG_ODOM_LOCAL, (const uint8_t *)&p, sizeof(p));
}

void sendNucBaseTelemetry() {
  if (!hasBaseTelemetry) return;

  uint32_t age = millis() - lastBaseRxMs;
  if (age > 65535) age = 65535;

  NucBaseTelemetryPayload p;
  p.count_left = baseTel.countLeft;
  p.count_right = baseTel.countRight;
  p.delta_left = baseTel.deltaLeft;
  p.delta_right = baseTel.deltaRight;
  p.rpm_left_x10 = baseTel.rpmLeft_x10;
  p.rpm_right_x10 = baseTel.rpmRight_x10;
  p.pwm_left = baseTel.pwmLeft;
  p.pwm_right = baseTel.pwmRight;
  p.slave_a_mode = baseTel.mode;
  p.slave_a_fault_flags = baseTel.faultFlags;
  p.age_ms = (uint16_t)age;

  sendNucFrame(NUC_MSG_BASE_TELEMETRY, (const uint8_t *)&p, sizeof(p));
}

void sendNucS3Health() {
  uint32_t baseAge = hasBaseTelemetry ? (millis() - lastBaseRxMs) : 65535;
  if (baseAge > 65535) baseAge = 65535;

  uint32_t nucAge = lastNucCmdMs == 0 ? 65535 : (millis() - lastNucCmdMs);
  if (nucAge > 65535) nucAge = 65535;

  S3HealthPayload p;
  p.s3_time_ms = millis();
  p.mode = s3Mode;
  p.heading_ready = isHeadingReady() ? 1 : 0;
  p.base_connected = (hasBaseTelemetry && baseAge < 300) ? 1 : 0;
  p.primitive_state = primitiveStateForNuc;
  p.bad_bno_frames = getBadFrameCount();
  p.yaw_jump_reject_count = getYawJumpRejectCount();
  p.base_crc_error = basePacketBadCrc;
  p.nuc_rx_count = nucRxCount;
  p.nuc_crc_error = nucCrcError;
  p.last_nuc_cmd_age_ms = (uint16_t)nucAge;
  p.last_base_age_ms = (uint16_t)baseAge;
  p.fault_flags = s3FaultFlags;

  sendNucFrame(NUC_MSG_S3_HEALTH, (const uint8_t *)&p, sizeof(p));
}

void updateNucTelemetryTx() {
  static uint32_t lastHealthMs = 0;
  static uint32_t lastOdomMs = 0;
  static uint32_t lastBaseMs = 0;

  uint32_t now = millis();

  // 10 Hz health
  if (now - lastHealthMs >= 100) {
    lastHealthMs = now;
    sendNucS3Health();
  }

  // 20 Hz odom
  if (now - lastOdomMs >= 50) {
    lastOdomMs = now;
    sendNucOdomLocal();
  }

  // 20 Hz forwarded Slave A telemetry
  if (now - lastBaseMs >= 50) {
    lastBaseMs = now;
    sendNucBaseTelemetry();
  }
}

// Integration notes:
// 1. In setup(), keep Serial.begin(115200) or Serial.begin(921600); USB CDC baud is usually not critical.
// 2. Remove or disable human debug prints on Serial while binary protocol is active.
// 3. In loop(), call updateNucTelemetryTx() after updateHeading() and updateBaseUART().
// 4. Do not enable NUC motion commands yet; milestone 1 is telemetry only.
