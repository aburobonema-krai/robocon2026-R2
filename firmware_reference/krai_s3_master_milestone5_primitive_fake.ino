#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>


// =====================================================
// KRAI NUC USB CDC LINK CONFIG - MILESTONE 5 PRIMITIVE EXECUTOR
// =====================================================
// IMPORTANT:
//   Serial    = USB CDC binary protocol to Intel NUC / ROS2.
//   Serial2   = UART link to Slave A.
//
// Human-readable Serial.print debug is disabled by default because it will
// corrupt the binary NUC_S3_PROTOCOL_V1 stream. Use ROS2 topics instead.
#define ENABLE_USB_HUMAN_DEBUG 0
#define NUC_SERIAL Serial
#define NUC_SERIAL_BAUD 115200

// =====================================================
// FAKE DATA MODE FOR BENCH TESTING
// =====================================================
// Set to 1 when ESP32-S3 is not installed on the robot and BNO055/Slave A
// are not connected. The board will send synthetic HEALTH, ODOM, BASE
// TELEMETRY, and PRIMITIVE_STATUS frames to ROS2 so the NUC bridge can be
// tested safely without motors or sensors. Milestone 5 accepts manual velocity and fake primitive commands (MOVE_REL / TURN_REL) without driving real motors while KRAI_FAKE_DATA_MODE is 1.
// Set to 0 when the S3 is back on the robot.
#define KRAI_FAKE_DATA_MODE 1

class NullDebugSerialClass {
public:
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }

  size_t print(const char *) { return 0; }
  size_t print(char) { return 0; }
  size_t print(unsigned char, int = DEC) { return 0; }
  size_t print(int, int = DEC) { return 0; }
  size_t print(unsigned int, int = DEC) { return 0; }
  size_t print(long, int = DEC) { return 0; }
  size_t print(unsigned long, int = DEC) { return 0; }
  size_t print(float, int = 2) { return 0; }
  size_t print(double, int = 2) { return 0; }

  size_t println() { return 0; }
  size_t println(const char *) { return 0; }
  size_t println(char) { return 0; }
  size_t println(unsigned char, int = DEC) { return 0; }
  size_t println(int, int = DEC) { return 0; }
  size_t println(unsigned int, int = DEC) { return 0; }
  size_t println(long, int = DEC) { return 0; }
  size_t println(unsigned long, int = DEC) { return 0; }
  size_t println(float, int = 2) { return 0; }
  size_t println(double, int = 2) { return 0; }

  template <typename... Args>
  size_t printf(const char *, Args...) { return 0; }
};

#if ENABLE_USB_HUMAN_DEBUG
  #define DBG_SERIAL Serial
#else
  NullDebugSerialClass DBG_SERIAL;
#endif

// =====================================================
// BNO055 MASTER HEADING CONFIG
// =====================================================
#define BNO_SDA 6
#define BNO_SCL 7
#define BNO_ADDRESS 0x28

// Baseline hasil test kamu: 25 kHz stabil.
#define I2C_CLOCK_HZ 25000

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO_ADDRESS, &Wire);
Preferences prefs;

// =====================================================
// UART SLAVE A LINK - MASTER
// Master UART2:
//   TX = GPIO15
//   RX = GPIO16
// Wiring:
//   Master TX16 -> Slave A RX17
//   Master RX15 <- Slave A TX16
//   GND common
// =====================================================
#define BASE_UART Serial2

#define BASE_UART_BAUD 921600
#define BASE_UART_TX 15
#define BASE_UART_RX 16

#define FRAME_SYNC1 0xAA
#define FRAME_SYNC2 0x55


// =====================================================
// NUC_S3_PROTOCOL_V1 - USB CDC BINARY FRAME
// =====================================================
#define NUC_PROTO_VERSION 0x01

// NUC -> S3 command IDs
#define NUC_CMD_PING            0x01
#define NUC_CMD_STOP            0x02
#define NUC_CMD_ESTOP           0x03
#define NUC_CMD_RESET_ODOM      0x04
#define NUC_CMD_RESET_HEADING   0x05
#define NUC_CMD_VEL_MANUAL      0x10
#define NUC_CMD_EXEC_MOVE_REL    0x20
#define NUC_CMD_EXEC_TURN_REL    0x21
#define NUC_CMD_CANCEL_PRIMITIVE 0x22

// S3 -> NUC message IDs
#define NUC_MSG_PONG             0x81
#define NUC_MSG_ACK              0x82
#define NUC_MSG_NACK             0x83
#define NUC_MSG_ODOM_LOCAL       0x90
#define NUC_MSG_BASE_TELEMETRY   0x91
#define NUC_MSG_S3_HEALTH        0x92
#define NUC_MSG_PRIMITIVE_STATUS 0x93
#define NUC_MSG_FAULT_STATUS     0x94

// S3 mode values reported to ROS2
#define S3_MODE_IDLE             0
#define S3_MODE_MANUAL           1
#define S3_MODE_PRIMITIVE        2
#define S3_MODE_SAFE_STOP        3
#define S3_MODE_ESTOP            4
#define S3_MODE_LOCAL_DEBUG      5

// Primitive state values reported to ROS2
#define PRIM_STATE_IDLE          0
#define PRIM_STATE_RUNNING       1
#define PRIM_STATE_DONE          2
#define PRIM_STATE_FAILED        3
#define PRIM_STATE_CANCELED      4

uint8_t nucTxSeq = 0;
uint32_t nucRxCount = 0;
uint32_t nucCrcError = 0;
uint32_t lastNucCmdMs = 0;

bool s3EstopLatched = false;
uint32_t s3SafeStopUntilMs = 0;
uint32_t s3StopCommandCount = 0;
uint32_t s3EstopCommandCount = 0;
uint32_t manualCmdRxCount = 0;
uint32_t lastManualCmdMs = 0;
bool manualControlActive = false;
const uint32_t MANUAL_CMD_TIMEOUT_MS = 300;
const float MAX_MANUAL_LINEAR_CM_S = 35.0f;
const float MAX_MANUAL_ANGULAR_DEG_S = 75.0f;

#define PRIM_TYPE_NONE       0
#define PRIM_TYPE_MOVE_REL   1
#define PRIM_TYPE_TURN_REL   2

uint16_t activePrimitiveId = 0;
uint8_t activePrimitiveType = PRIM_TYPE_NONE;
uint8_t activePrimitiveState = PRIM_STATE_IDLE;
bool fakePrimitiveActive = false;
float fakePrimitiveTargetAbs = 0.0f;
float fakePrimitiveProgress = 0.0f;
float fakePrimitiveRemaining = 0.0f;
float fakePrimitiveSpeedAbs = 0.0f;
float fakePrimitiveTolerance = 0.0f;
float fakePrimitiveSign = 1.0f;
float fakePrimitiveStartHeadingDeg = 0.0f;
float fakePrimitiveTargetHeadingDeg = 0.0f;
float fakePrimitiveHeadingErrorDeg = 0.0f;
uint32_t fakePrimitiveDoneMs = 0;

// Master -> Slave A
#define MSG_CMD_VEL        0x01
#define MSG_CMD_STOP       0x02

// Slave A -> Master
#define MSG_BASE_TELEMETRY 0x81

// Sama dengan Slave A
#define M4_COUNT_PER_REV 384.0f
#define M1_COUNT_PER_REV 382.0f
#define WHEEL_CIRCUMFERENCE_CM 31.4286f

// =====================================================
// HEADING STATE
// =====================================================
float yawZeroDeg = 0.0f;
float headingDeg = 0.0f;   // heading utama robot, range -180..180
float yawRawDeg = 0.0f;    // yaw absolute BNO, range 0..360

bool headingReady = false;

uint32_t okFrameCount = 0;
uint32_t badFrameCount = 0;

// Last valid Euler frame
float lastYaw = 0.0f;
float lastRoll = 0.0f;
float lastPitch = 0.0f;
bool hasLastValid = false;

// Accepted yaw state untuk jump rejection
bool hasAcceptedYaw = false;
float acceptedYawRawDeg = 0.0f;
uint32_t lastAcceptedYawMs = 0;

uint32_t yawJumpRejectCount = 0;
uint32_t yawRealJumpAcceptCount = 0;

// Pending real yaw jump detector.
// Spike palsu tetap ditolak, tapi yaw besar yang stabil
// akibat robot benar-benar diputar tetap diterima.
bool hasPendingJumpYaw = false;
float pendingJumpYawDeg = 0.0f;
uint32_t pendingJumpStartMs = 0;

const uint32_t REAL_JUMP_ACCEPT_MS = 120;
const float REAL_JUMP_STABLE_DEG = 3.0f;

// =====================================================
// STARTUP / JUMP FILTER CONFIG
// =====================================================
const uint32_t BNO_MIN_STARTUP_SETTLE_MS = 4000;
const uint32_t BNO_STABLE_REQUIRED_MS = 1500;
const uint32_t BNO_STARTUP_TIMEOUT_MS = 10000;

float BNO_STABLE_DIFF_DEG = 0.8f;
float MAX_YAW_RATE_DEG_S = 250.0f;
float YAW_JUMP_MARGIN_DEG = 6.0f;
float YAW_FILTER_MAX_DT_SEC = 0.10f;

// =====================================================
// Optional health monitor
// =====================================================
uint8_t calSys = 0;
uint8_t calGyro = 0;
uint8_t calAccel = 0;
uint8_t calMag = 0;

uint8_t systemStatus = 0;
uint8_t selfTestResult = 0;
uint8_t systemError = 0;

// =====================================================
// PACKET PAYLOADS
// =====================================================
struct __attribute__((packed)) BaseTelemetryPayload {
  uint32_t slaveTimeMs;

  int32_t countLeft;
  int32_t countRight;

  int16_t deltaLeft;
  int16_t deltaRight;

  int16_t rpmLeft_x10;
  int16_t rpmRight_x10;

  int16_t pwmLeft;
  int16_t pwmRight;

  uint8_t mode;
  uint8_t faultFlags;
};

struct __attribute__((packed)) CmdVelPayload {
  uint32_t masterTimeMs;

  int16_t linearCmS_x10;
  int16_t angularDegS_x10;

  uint8_t controlMode;
  uint8_t flags;
};


// =====================================================
// NUC_S3_PROTOCOL_V1 PAYLOADS - S3 -> NUC
// Must match krai_s3_bridge milestone 1 parser.
// =====================================================
struct __attribute__((packed)) NucOdomLocalPayload {
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

struct __attribute__((packed)) NucS3HealthPayload {
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

struct __attribute__((packed)) NucPrimitiveStatusPayload {
  uint16_t primitive_id;
  uint8_t primitive_type;
  uint8_t primitive_state;
  int32_t progress_x10;
  int32_t remaining_x10;
  int16_t heading_error_deg_x10;
  uint16_t fault_flags;
};

struct __attribute__((packed)) NucAckPayload {
  uint8_t request_type;
  uint8_t request_seq;
  uint8_t status_code;
};

struct __attribute__((packed)) NucPongPayload {
  uint8_t request_seq;
  uint32_t s3_time_ms;
};

struct __attribute__((packed)) NucCmdVelManualPayload {
  int16_t linear_cm_s_x10;
  int16_t angular_deg_s_x10;
  uint8_t control_mode;
  uint8_t flags;
};

struct __attribute__((packed)) NucExecMoveRelPayload {
  uint16_t primitive_id;
  int32_t distance_cm_x10;
  int16_t max_speed_cm_s_x10;
  int16_t tolerance_cm_x10;
  uint8_t heading_hold_enable;
  uint8_t reserved;
};

struct __attribute__((packed)) NucExecTurnRelPayload {
  uint16_t primitive_id;
  int32_t angle_deg_x10;
  int16_t max_speed_deg_s_x10;
  int16_t tolerance_deg_x10;
  uint16_t reserved;
};

struct __attribute__((packed)) NucCancelPrimitivePayload {
  uint16_t primitive_id;
};

// =====================================================
// BASE TELEMETRY STATE
// =====================================================
BaseTelemetryPayload baseTel;
bool hasBaseTelemetry = false;

uint32_t basePacketCount = 0;
uint32_t basePacketBadCrc = 0;
uint32_t basePacketBadLen = 0;
uint32_t basePacketUnknownType = 0;
uint32_t lastBaseRxMs = 0;
uint32_t lastBaseSlaveTimeMs = 0;

// Count baseline untuk Master pose.
// Pose dihitung dari count absolute, bukan deltaLeft/deltaRight payload.
bool hasLastBaseCount = false;
int32_t lastBaseCountLeft = 0;
int32_t lastBaseCountRight = 0;

// Pose resmi Master
float poseXcm = 0.0f;
float poseYcm = 0.0f;
float poseDistCm = 0.0f;
float poseThetaDeg = 0.0f;
float poseLinearCmS = 0.0f;

// Debug delta yang dihitung Master dari count absolute
int32_t masterDeltaLeftCount = 0;
int32_t masterDeltaRightCount = 0;

// =====================================================
// MASTER COMMAND STATE
// =====================================================
uint8_t masterCmdSeq = 0;

float masterCmdLinearCmS = 0.0f;
float masterCmdAngularDegS = 0.0f;
bool masterCmdActive = false;

uint32_t cmdVelTxCount = 0;
uint32_t cmdStopTxCount = 0;

// Command manual test
const float TEST_LINEAR_CM_S = 20.0f;
const float TEST_ANGULAR_DEG_S = 30.0f;

// =====================================================
// MASTER AUTO POSITION CONTROLLER
// Step ini: move 100 cm pakai pose Master + BNO heading hold.
// =====================================================
enum MasterAutoMode {
  MASTER_AUTO_IDLE = 0,
  MASTER_AUTO_MOVE = 1,
  MASTER_AUTO_TURN = 2,
  MASTER_AUTO_DONE = 3
};

MasterAutoMode masterAutoMode = MASTER_AUTO_IDLE;

float autoStartDistCm = 0.0f;
float autoTargetDistCm = 100.0f;
float autoTargetHeadingDeg = 0.0f;

float autoMoveMaxCmS = 25.0f;
float autoMoveMinCmS = 18.0f;
float autoMoveSlowdownCm = 35.0f;
float autoMoveStopCm = 1.5f;

// headingErrorDeg * headingHoldKp = angularDegS
float headingHoldKp = 2.0f;
float maxHeadingCorrectionDegS = 35.0f;

// Kalau heading correction malah makin melenceng, ubah jadi -1.0f.
float headingCorrectionSign = -1.0f;

// Auto rotate config
float autoTurnStartHeadingDeg = 0.0f;
float autoTurnTargetHeadingDeg = 0.0f;
float autoTurnMaxDegS = 45.0f;
float autoTurnMinDegS = 22.0f;
float autoTurnSlowdownDeg = 35.0f;
float autoTurnStopDeg = 2.0f;

// Untuk rotate, T/R debug diisi derajat, bukan cm.
float autoDebugTraveledCm = 0.0f;
float autoDebugRemainingCm = 0.0f;
float autoDebugHeadingErrDeg = 0.0f;

// =====================================================
// MASTER SEQUENCE TEST
// n = move 100 cm -> turn +90 -> move 100 cm -> turn -90
// =====================================================
enum MasterSeqMode {
  MASTER_SEQ_IDLE = 0,
  MASTER_SEQ_RUNNING = 1,
  MASTER_SEQ_DONE = 2
};

MasterSeqMode masterSeqMode = MASTER_SEQ_IDLE;
uint8_t masterSeqStep = 0;
uint32_t masterSeqStepDoneMs = 0;
bool masterSeqWaitingNext = false;

const uint32_t MASTER_SEQ_STEP_DELAY_MS = 500;

// =====================================================
// UART RX PARSER STATE
// =====================================================
enum RxState {
  RX_WAIT_SYNC1 = 0,
  RX_WAIT_SYNC2,
  RX_TYPE,
  RX_SEQ,
  RX_LEN,
  RX_PAYLOAD,
  RX_CRC
};

RxState rxState = RX_WAIT_SYNC1;

uint8_t rxType = 0;
uint8_t rxSeq = 0;
uint8_t rxLen = 0;
uint8_t rxBuf[64];
uint8_t rxIndex = 0;

// =====================================================
// FORWARD DECLARATION
// =====================================================
float getHeadingDeg();
uint32_t getBadFrameCount();
uint32_t getYawJumpRejectCount();

void setupBaseUART();
void updateBaseUART();
void resetMasterPose();
void updateMasterPoseFromBase();
void printMasterBaseDebug();

void sendCmdVel(float linearCmS, float angularDegS);
void sendCmdStop();
void setMasterCmd(float linearCmS, float angularDegS);
void stopMasterCmd();
void updateCmdVelTx50Hz();

void startAutoMove100();
void startAutoTurnDeg(float deltaDeg);
void stopMasterAuto();
void updateMasterAutoMove();
const char *getMasterAutoModeText();

void startSequenceTest();
void stopSequenceTest();
void updateSequenceTest();
void startSequenceStep(uint8_t step);
const char *getMasterSeqModeText();

bool setupBNOHeading();
bool updateHeading();
void updateBNOStatus();
void resetHeadingZero();

void setupNucSerial();
void updateNucRx();
void updateNucTelemetryTx();
void sendNucAck(uint8_t requestType, uint8_t requestSeq, uint8_t statusCode);
void sendNucNack(uint8_t requestType, uint8_t requestSeq, uint8_t errorCode);
void sendNucPong(uint8_t requestSeq);
void applyManualVelocityCommand(const NucCmdVelManualPayload &cmd);
void updateManualCommandWatchdog();
void setupFakeDataMode();
void updateFakeDataMode();

// Fake data reset helpers used by safe command milestones.
uint32_t fakeLastUpdateMs = 0;
int32_t fakeSimCountLeft = 0;
int32_t fakeSimCountRight = 0;
float fakePrimitiveProgressCm = 0.0f;
float fakeHeadingZeroRawDeg = 0.0f;

// =====================================================
// ANGLE HELPER
// =====================================================
float normalize360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

float normalize180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float angleDiffDeg(float current, float zero) {
  return normalize180(current - zero);
}

// =====================================================
// GETTER
// =====================================================
float getHeadingDeg() {
  return headingDeg;
}

float getYawRawDeg() {
  return yawRawDeg;
}

bool isHeadingReady() {
  return headingReady;
}

uint32_t getBadFrameCount() {
  return badFrameCount;
}

uint32_t getYawJumpRejectCount() {
  return yawJumpRejectCount;
}

// =====================================================
// UART CHECKSUM / TX FRAME
// =====================================================
uint8_t checksumXor(uint8_t type, uint8_t seq, uint8_t len, const uint8_t *payload) {
  uint8_t cs = 0;
  cs ^= type;
  cs ^= seq;
  cs ^= len;

  for (uint8_t i = 0; i < len; i++) {
    cs ^= payload[i];
  }

  return cs;
}

void sendBaseFrame(uint8_t type, const uint8_t *payload, uint8_t len) {
  uint8_t seq = masterCmdSeq++;
  uint8_t cs = checksumXor(type, seq, len, payload);

  BASE_UART.write(FRAME_SYNC1);
  BASE_UART.write(FRAME_SYNC2);
  BASE_UART.write(type);
  BASE_UART.write(seq);
  BASE_UART.write(len);

  if (len > 0 && payload != nullptr) {
    BASE_UART.write(payload, len);
  }

  BASE_UART.write(cs);
}

int16_t floatToInt16x10Master(float value) {
  long v = lroundf(value * 10.0f);
  v = constrain(v, -32768, 32767);
  return (int16_t)v;
}


// =====================================================
// NUC_S3_PROTOCOL_V1 TX HELPERS - READ ONLY MILESTONE
// =====================================================
void setupNucSerial() {
  NUC_SERIAL.begin(NUC_SERIAL_BAUD);
  delay(100);
}

uint16_t clampU16FromU32(uint32_t value) {
  if (value > 65535UL) return 65535;
  return (uint16_t)value;
}

int16_t floatToInt16x10Nuc(float value) {
  long v = lroundf(value * 10.0f);
  if (v < -32768L) v = -32768L;
  if (v > 32767L) v = 32767L;
  return (int16_t)v;
}

int32_t floatToInt32x10Nuc(float value) {
  double v = round((double)value * 10.0);
  if (v < -2147483648.0) v = -2147483648.0;
  if (v > 2147483647.0) v = 2147483647.0;
  return (int32_t)v;
}

uint16_t crc16CcittFalseUpdate(uint16_t crc, uint8_t data) {
  crc ^= ((uint16_t)data << 8);
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (uint16_t)((crc << 1) ^ 0x1021);
    } else {
      crc = (uint16_t)(crc << 1);
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

void sendNucFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
  uint8_t seq = nucTxSeq++;
  uint8_t header[5];
  header[0] = NUC_PROTO_VERSION;
  header[1] = type;
  header[2] = seq;
  header[3] = (uint8_t)(len & 0xFF);
  header[4] = (uint8_t)((len >> 8) & 0xFF);

  uint16_t crc = crc16CcittFalse(header, sizeof(header));
  if (payload != nullptr && len > 0) {
    for (uint16_t i = 0; i < len; i++) {
      crc = crc16CcittFalseUpdate(crc, payload[i]);
    }
  }

  NUC_SERIAL.write(FRAME_SYNC1);
  NUC_SERIAL.write(FRAME_SYNC2);
  NUC_SERIAL.write(header, sizeof(header));
  if (payload != nullptr && len > 0) {
    NUC_SERIAL.write(payload, len);
  }
  NUC_SERIAL.write((uint8_t)(crc & 0xFF));
  NUC_SERIAL.write((uint8_t)((crc >> 8) & 0xFF));
}

void sendNucAck(uint8_t requestType, uint8_t requestSeq, uint8_t statusCode) {
  NucAckPayload p;
  p.request_type = requestType;
  p.request_seq = requestSeq;
  p.status_code = statusCode;
  sendNucFrame(NUC_MSG_ACK, (const uint8_t *)&p, sizeof(p));
}

void sendNucNack(uint8_t requestType, uint8_t requestSeq, uint8_t errorCode) {
  NucAckPayload p;
  p.request_type = requestType;
  p.request_seq = requestSeq;
  p.status_code = errorCode;
  sendNucFrame(NUC_MSG_NACK, (const uint8_t *)&p, sizeof(p));
}

void sendNucPong(uint8_t requestSeq) {
  NucPongPayload p;
  p.request_seq = requestSeq;
  p.s3_time_ms = millis();
  sendNucFrame(NUC_MSG_PONG, (const uint8_t *)&p, sizeof(p));
}

#define NUC_RX_MAX_PAYLOAD 128
enum NucRxState {
  NUC_RX_WAIT_SYNC1 = 0,
  NUC_RX_WAIT_SYNC2,
  NUC_RX_HEADER,
  NUC_RX_PAYLOAD,
  NUC_RX_CRC
};

NucRxState nucRxState = NUC_RX_WAIT_SYNC1;
uint8_t nucRxHeader[5];
uint8_t nucRxHeaderIndex = 0;
uint8_t nucRxType = 0;
uint8_t nucRxSeq = 0;
uint16_t nucRxLen = 0;
uint8_t nucRxPayload[NUC_RX_MAX_PAYLOAD];
uint16_t nucRxPayloadIndex = 0;
uint8_t nucRxCrcBytes[2];
uint8_t nucRxCrcIndex = 0;

void resetNucRxParser() {
  nucRxState = NUC_RX_WAIT_SYNC1;
  nucRxHeaderIndex = 0;
  nucRxPayloadIndex = 0;
  nucRxCrcIndex = 0;
  nucRxLen = 0;
}

void applySafeResetOdom() {
  resetMasterPose();
#if KRAI_FAKE_DATA_MODE
  fakeLastUpdateMs = millis();
  fakeSimCountLeft = 0;
  fakeSimCountRight = 0;
  fakePrimitiveProgressCm = 0.0f;
  poseXcm = 0.0f;
  poseYcm = 0.0f;
  poseDistCm = 0.0f;
  poseLinearCmS = 0.0f;
  baseTel.countLeft = 0;
  baseTel.countRight = 0;
  baseTel.deltaLeft = 0;
  baseTel.deltaRight = 0;
#endif
}

void applySafeResetHeading() {
#if KRAI_FAKE_DATA_MODE
  float t = millis() / 1000.0f;
  fakeHeadingZeroRawDeg = 25.0f * sinf(t * 0.35f);
  headingDeg = 0.0f;
  poseThetaDeg = 0.0f;
  yawRawDeg = normalize360(fakeHeadingZeroRawDeg);
  headingReady = true;
#else
  resetHeadingZero();
#endif
}

bool isSafeStopActive() {
  return millis() < s3SafeStopUntilMs;
}

void applySafeStopCommand() {
  s3StopCommandCount++;
  s3SafeStopUntilMs = millis() + 1500;

  masterCmdLinearCmS = 0.0f;
  masterCmdAngularDegS = 0.0f;
  masterCmdActive = false;
  manualControlActive = false;
  lastManualCmdMs = 0;
  masterAutoMode = MASTER_AUTO_IDLE;
  stopSequenceTest();
  if (activePrimitiveState == PRIM_STATE_RUNNING) {
    cancelActivePrimitive(PRIM_STATE_CANCELED);
  }

#if !KRAI_FAKE_DATA_MODE
  for (int i = 0; i < 3; i++) {
    sendCmdStop();
    updateBaseUART();
    delay(5);
  }
#endif
}

void applyEmergencyStopCommand(bool enable) {
  s3EstopCommandCount++;

  if (enable) {
    s3EstopLatched = true;
    s3SafeStopUntilMs = millis() + 1500;

    masterCmdLinearCmS = 0.0f;
    masterCmdAngularDegS = 0.0f;
    masterCmdActive = false;
    manualControlActive = false;
    lastManualCmdMs = 0;
    masterAutoMode = MASTER_AUTO_IDLE;
    stopSequenceTest();
    if (activePrimitiveState == PRIM_STATE_RUNNING) {
      cancelActivePrimitive(PRIM_STATE_CANCELED);
    }

#if !KRAI_FAKE_DATA_MODE
    for (int i = 0; i < 5; i++) {
      sendCmdStop();
      updateBaseUART();
      delay(5);
    }
#endif
  } else {
    s3EstopLatched = false;
    s3SafeStopUntilMs = 0;
  }
}


void cancelActivePrimitive(uint8_t stateValue) {
  fakePrimitiveActive = false;
  activePrimitiveState = stateValue;
  fakePrimitiveRemaining = 0.0f;
  fakePrimitiveHeadingErrorDeg = 0.0f;
  masterAutoMode = MASTER_AUTO_IDLE;
  masterCmdLinearCmS = 0.0f;
  masterCmdAngularDegS = 0.0f;
  masterCmdActive = false;
  fakePrimitiveDoneMs = millis();
}

bool primitiveSafetyOk() {
  if (s3EstopLatched) return false;
  if (isSafeStopActive()) return false;
  return true;
}

void startFakeMovePrimitive(const NucExecMoveRelPayload &cmd) {
  activePrimitiveId = cmd.primitive_id;
  activePrimitiveType = PRIM_TYPE_MOVE_REL;
  activePrimitiveState = PRIM_STATE_RUNNING;
  fakePrimitiveActive = true;
  fakePrimitiveTargetAbs = fabs(cmd.distance_cm_x10 / 10.0f);
  fakePrimitiveProgress = 0.0f;
  fakePrimitiveRemaining = fakePrimitiveTargetAbs;
  fakePrimitiveSpeedAbs = fabs(cmd.max_speed_cm_s_x10 / 10.0f);
  if (fakePrimitiveSpeedAbs < 1.0f) fakePrimitiveSpeedAbs = 15.0f;
  fakePrimitiveTolerance = fabs(cmd.tolerance_cm_x10 / 10.0f);
  if (fakePrimitiveTolerance < 0.5f) fakePrimitiveTolerance = 1.5f;
  fakePrimitiveSign = (cmd.distance_cm_x10 >= 0) ? 1.0f : -1.0f;
  fakePrimitiveStartHeadingDeg = poseThetaDeg;
  fakePrimitiveTargetHeadingDeg = poseThetaDeg;
  fakePrimitiveHeadingErrorDeg = 0.0f;
  fakePrimitiveDoneMs = 0;
  manualControlActive = false;
  lastManualCmdMs = 0;
  masterCmdLinearCmS = 0.0f;
  masterCmdAngularDegS = 0.0f;
  masterCmdActive = false;
  masterAutoMode = MASTER_AUTO_MOVE;
}

void startFakeTurnPrimitive(const NucExecTurnRelPayload &cmd) {
  activePrimitiveId = cmd.primitive_id;
  activePrimitiveType = PRIM_TYPE_TURN_REL;
  activePrimitiveState = PRIM_STATE_RUNNING;
  fakePrimitiveActive = true;
  fakePrimitiveTargetAbs = fabs(cmd.angle_deg_x10 / 10.0f);
  fakePrimitiveProgress = 0.0f;
  fakePrimitiveRemaining = fakePrimitiveTargetAbs;
  fakePrimitiveSpeedAbs = fabs(cmd.max_speed_deg_s_x10 / 10.0f);
  if (fakePrimitiveSpeedAbs < 1.0f) fakePrimitiveSpeedAbs = 30.0f;
  fakePrimitiveTolerance = fabs(cmd.tolerance_deg_x10 / 10.0f);
  if (fakePrimitiveTolerance < 0.5f) fakePrimitiveTolerance = 2.0f;
  fakePrimitiveSign = (cmd.angle_deg_x10 >= 0) ? 1.0f : -1.0f;
  fakePrimitiveStartHeadingDeg = poseThetaDeg;
  fakePrimitiveTargetHeadingDeg = normalize180(fakePrimitiveStartHeadingDeg + (cmd.angle_deg_x10 / 10.0f));
  fakePrimitiveHeadingErrorDeg = angleDiffDeg(fakePrimitiveTargetHeadingDeg, poseThetaDeg);
  fakePrimitiveDoneMs = 0;
  manualControlActive = false;
  lastManualCmdMs = 0;
  masterCmdLinearCmS = 0.0f;
  masterCmdAngularDegS = 0.0f;
  masterCmdActive = false;
  masterAutoMode = MASTER_AUTO_TURN;
}

void applyManualVelocityCommand(const NucCmdVelManualPayload &cmd) {
  manualCmdRxCount++;
  lastManualCmdMs = millis();

  float linearCmS = cmd.linear_cm_s_x10 / 10.0f;
  float angularDegS = cmd.angular_deg_s_x10 / 10.0f;

  linearCmS = constrain(linearCmS, -MAX_MANUAL_LINEAR_CM_S, MAX_MANUAL_LINEAR_CM_S);
  angularDegS = constrain(angularDegS, -MAX_MANUAL_ANGULAR_DEG_S, MAX_MANUAL_ANGULAR_DEG_S);

  // Manual velocity cancels any local test sequence/primitive, but never overrides ESTOP.
  if (activePrimitiveState == PRIM_STATE_RUNNING) {
    cancelActivePrimitive(PRIM_STATE_CANCELED);
  }
  stopSequenceTest();
  masterAutoMode = MASTER_AUTO_IDLE;

  if (s3EstopLatched || isSafeStopActive()) {
    masterCmdLinearCmS = 0.0f;
    masterCmdAngularDegS = 0.0f;
    masterCmdActive = false;
    manualControlActive = false;
    return;
  }

  masterCmdLinearCmS = linearCmS;
  masterCmdAngularDegS = angularDegS;
  masterCmdActive = (fabs(linearCmS) > 0.01f || fabs(angularDegS) > 0.01f);
  manualControlActive = true;

#if !KRAI_FAKE_DATA_MODE
  if (!masterCmdActive) {
    sendCmdStop();
  }
#endif
}

void updateManualCommandWatchdog() {
  if (!manualControlActive) {
    return;
  }

  uint32_t nowMs = millis();
  if (lastManualCmdMs == 0) {
    return;
  }

  if (nowMs - lastManualCmdMs > MANUAL_CMD_TIMEOUT_MS) {
    masterCmdLinearCmS = 0.0f;
    masterCmdAngularDegS = 0.0f;
    masterCmdActive = false;
    manualControlActive = false;

#if !KRAI_FAKE_DATA_MODE
    sendCmdStop();
#endif
  }
}

void handleNucCommand(uint8_t type, uint8_t seq, uint16_t len, const uint8_t *payload) {
  (void)payload;
  nucRxCount++;
  lastNucCmdMs = millis();

  if (type == NUC_CMD_PING) {
    if (len != 0) {
      sendNucNack(type, seq, 1);
      return;
    }
    sendNucPong(seq);
    return;
  }

  if (type == NUC_CMD_STOP) {
    if (len != 0) {
      sendNucNack(type, seq, 1);
      return;
    }
    applySafeStopCommand();
    sendNucAck(type, seq, 0);
    return;
  }

  if (type == NUC_CMD_ESTOP) {
    if (len != 1) {
      sendNucNack(type, seq, 1);
      return;
    }
    bool enable = payload[0] != 0;
    applyEmergencyStopCommand(enable);
    sendNucAck(type, seq, 0);
    return;
  }

  if (type == NUC_CMD_RESET_ODOM) {
    if (len != 0) {
      sendNucNack(type, seq, 1);
      return;
    }
    applySafeResetOdom();
    sendNucAck(type, seq, 0);
    return;
  }

  if (type == NUC_CMD_RESET_HEADING) {
    if (len != 0) {
      sendNucNack(type, seq, 1);
      return;
    }
    applySafeResetHeading();
    sendNucAck(type, seq, 0);
    return;
  }

  if (type == NUC_CMD_VEL_MANUAL) {
    if (len != sizeof(NucCmdVelManualPayload)) {
      sendNucNack(type, seq, 1);
      return;
    }
    if (s3EstopLatched) {
      sendNucNack(type, seq, 3);
      return;
    }
    NucCmdVelManualPayload cmd;
    memcpy(&cmd, payload, sizeof(cmd));
    applyManualVelocityCommand(cmd);
    sendNucAck(type, seq, 0);
    return;
  }

  if (type == NUC_CMD_EXEC_MOVE_REL) {
    if (len != sizeof(NucExecMoveRelPayload)) {
      sendNucNack(type, seq, 1);
      return;
    }
    if (!primitiveSafetyOk()) {
      sendNucNack(type, seq, 3);
      return;
    }
    NucExecMoveRelPayload cmd;
    memcpy(&cmd, payload, sizeof(cmd));
#if KRAI_FAKE_DATA_MODE
    startFakeMovePrimitive(cmd);
    sendNucAck(type, seq, 0);
#else
    startMoveRelative(cmd.distance_cm_x10 / 10.0f, cmd.max_speed_cm_s_x10 / 10.0f, cmd.tolerance_cm_x10 / 10.0f);
    activePrimitiveId = cmd.primitive_id;
    activePrimitiveType = PRIM_TYPE_MOVE_REL;
    activePrimitiveState = PRIM_STATE_RUNNING;
    sendNucAck(type, seq, 0);
#endif
    return;
  }

  if (type == NUC_CMD_EXEC_TURN_REL) {
    if (len != sizeof(NucExecTurnRelPayload)) {
      sendNucNack(type, seq, 1);
      return;
    }
    if (!primitiveSafetyOk()) {
      sendNucNack(type, seq, 3);
      return;
    }
    NucExecTurnRelPayload cmd;
    memcpy(&cmd, payload, sizeof(cmd));
#if KRAI_FAKE_DATA_MODE
    startFakeTurnPrimitive(cmd);
    sendNucAck(type, seq, 0);
#else
    startAutoTurnDeg(cmd.angle_deg_x10 / 10.0f);
    activePrimitiveId = cmd.primitive_id;
    activePrimitiveType = PRIM_TYPE_TURN_REL;
    activePrimitiveState = PRIM_STATE_RUNNING;
    sendNucAck(type, seq, 0);
#endif
    return;
  }

  if (type == NUC_CMD_CANCEL_PRIMITIVE) {
    if (len != sizeof(NucCancelPrimitivePayload)) {
      sendNucNack(type, seq, 1);
      return;
    }
    NucCancelPrimitivePayload cmd;
    memcpy(&cmd, payload, sizeof(cmd));
    if (cmd.primitive_id == 0 || cmd.primitive_id == activePrimitiveId) {
      cancelActivePrimitive(PRIM_STATE_CANCELED);
      sendNucAck(type, seq, 0);
    } else {
      sendNucNack(type, seq, 4);
    }
    return;
  }

  sendNucNack(type, seq, 2);
}

void parseNucByte(uint8_t b) {
  switch (nucRxState) {
    case NUC_RX_WAIT_SYNC1:
      if (b == FRAME_SYNC1) {
        nucRxState = NUC_RX_WAIT_SYNC2;
      }
      break;

    case NUC_RX_WAIT_SYNC2:
      if (b == FRAME_SYNC2) {
        nucRxHeaderIndex = 0;
        nucRxState = NUC_RX_HEADER;
      } else if (b != FRAME_SYNC1) {
        resetNucRxParser();
      }
      break;

    case NUC_RX_HEADER:
      nucRxHeader[nucRxHeaderIndex++] = b;
      if (nucRxHeaderIndex >= 5) {
        uint8_t version = nucRxHeader[0];
        nucRxType = nucRxHeader[1];
        nucRxSeq = nucRxHeader[2];
        nucRxLen = (uint16_t)nucRxHeader[3] | ((uint16_t)nucRxHeader[4] << 8);

        if (version != NUC_PROTO_VERSION || nucRxLen > NUC_RX_MAX_PAYLOAD) {
          resetNucRxParser();
        } else if (nucRxLen == 0) {
          nucRxCrcIndex = 0;
          nucRxState = NUC_RX_CRC;
        } else {
          nucRxPayloadIndex = 0;
          nucRxState = NUC_RX_PAYLOAD;
        }
      }
      break;

    case NUC_RX_PAYLOAD:
      nucRxPayload[nucRxPayloadIndex++] = b;
      if (nucRxPayloadIndex >= nucRxLen) {
        nucRxCrcIndex = 0;
        nucRxState = NUC_RX_CRC;
      }
      break;

    case NUC_RX_CRC:
      nucRxCrcBytes[nucRxCrcIndex++] = b;
      if (nucRxCrcIndex >= 2) {
        uint16_t received = (uint16_t)nucRxCrcBytes[0] | ((uint16_t)nucRxCrcBytes[1] << 8);
        uint16_t computed = crc16CcittFalse(nucRxHeader, 5);
        for (uint16_t i = 0; i < nucRxLen; i++) {
          computed = crc16CcittFalseUpdate(computed, nucRxPayload[i]);
        }

        if (received == computed) {
          handleNucCommand(nucRxType, nucRxSeq, nucRxLen, nucRxPayload);
        } else {
          nucCrcError++;
        }

        resetNucRxParser();
      }
      break;
  }
}

void updateNucRx() {
  while (NUC_SERIAL.available()) {
    parseNucByte((uint8_t)NUC_SERIAL.read());
  }
}

uint8_t getS3ReportedMode() {
  if (s3EstopLatched) {
    return S3_MODE_ESTOP;
  }
  if (isSafeStopActive()) {
    return S3_MODE_SAFE_STOP;
  }
  if (activePrimitiveState == PRIM_STATE_RUNNING) {
    return S3_MODE_PRIMITIVE;
  }
  if (masterAutoMode == MASTER_AUTO_MOVE || masterAutoMode == MASTER_AUTO_TURN) {
    return S3_MODE_PRIMITIVE;
  }
  if (manualControlActive || masterCmdActive) {
    return S3_MODE_MANUAL;
  }
  return S3_MODE_IDLE;
}

uint8_t getPrimitiveReportedState() {
  return activePrimitiveState;
}

uint16_t getBasicFaultFlags() {
  uint16_t flags = 0;
  if (!headingReady) flags |= (1u << 0);
  if (!hasBaseTelemetry) flags |= (1u << 1);
  if (hasBaseTelemetry && (millis() - lastBaseRxMs > 300)) flags |= (1u << 2);
  if (systemError != 0) flags |= (1u << 3);
  if (s3EstopLatched) flags |= (1u << 4);
  if (isSafeStopActive()) flags |= (1u << 5);
  if (manualControlActive && lastManualCmdMs > 0 && (millis() - lastManualCmdMs > MANUAL_CMD_TIMEOUT_MS)) flags |= (1u << 6);
  return flags;
}

void sendNucS3Health() {
  uint32_t nowMs = millis();
  uint16_t lastBaseAge = hasBaseTelemetry ? clampU16FromU32(nowMs - lastBaseRxMs) : 65535;
  uint16_t lastNucAge = (lastNucCmdMs > 0) ? clampU16FromU32(nowMs - lastNucCmdMs) : 65535;

  NucS3HealthPayload p;
  p.s3_time_ms = nowMs;
  p.mode = getS3ReportedMode();
  p.heading_ready = headingReady ? 1 : 0;
  p.base_connected = (hasBaseTelemetry && lastBaseAge <= 300) ? 1 : 0;
  p.primitive_state = getPrimitiveReportedState();
  p.bad_bno_frames = badFrameCount;
  p.yaw_jump_reject_count = yawJumpRejectCount;
  p.base_crc_error = basePacketBadCrc;
  p.nuc_rx_count = nucRxCount;
  p.nuc_crc_error = nucCrcError;
  p.last_nuc_cmd_age_ms = lastNucAge;
  p.last_base_age_ms = lastBaseAge;
  p.fault_flags = getBasicFaultFlags();

  sendNucFrame(NUC_MSG_S3_HEALTH, (const uint8_t *)&p, sizeof(p));
}

void sendNucOdomLocal() {
  NucOdomLocalPayload p;
  p.x_cm_x10 = floatToInt32x10Nuc(poseXcm);
  p.y_cm_x10 = floatToInt32x10Nuc(poseYcm);
  p.heading_deg_x10 = floatToInt16x10Nuc(poseThetaDeg);
  p.dist_cm_x10 = floatToInt32x10Nuc(poseDistCm);
  p.linear_cm_s_x10 = floatToInt16x10Nuc(poseLinearCmS);
  p.yaw_raw_deg_x10 = floatToInt16x10Nuc(yawRawDeg);
  p.heading_ready = headingReady ? 1 : 0;
  p.reserved = 0;

  sendNucFrame(NUC_MSG_ODOM_LOCAL, (const uint8_t *)&p, sizeof(p));
}

void sendNucBaseTelemetry() {
  uint32_t nowMs = millis();
  uint16_t age = hasBaseTelemetry ? clampU16FromU32(nowMs - lastBaseRxMs) : 65535;

  NucBaseTelemetryPayload p;
  p.count_left = hasBaseTelemetry ? baseTel.countLeft : 0;
  p.count_right = hasBaseTelemetry ? baseTel.countRight : 0;
  p.delta_left = hasBaseTelemetry ? baseTel.deltaLeft : 0;
  p.delta_right = hasBaseTelemetry ? baseTel.deltaRight : 0;
  p.rpm_left_x10 = hasBaseTelemetry ? baseTel.rpmLeft_x10 : 0;
  p.rpm_right_x10 = hasBaseTelemetry ? baseTel.rpmRight_x10 : 0;
  p.pwm_left = hasBaseTelemetry ? baseTel.pwmLeft : 0;
  p.pwm_right = hasBaseTelemetry ? baseTel.pwmRight : 0;
  p.slave_a_mode = hasBaseTelemetry ? baseTel.mode : 0;
  p.slave_a_fault_flags = hasBaseTelemetry ? baseTel.faultFlags : 0;
  p.age_ms = age;

  sendNucFrame(NUC_MSG_BASE_TELEMETRY, (const uint8_t *)&p, sizeof(p));
}

void sendNucPrimitiveStatus() {
  NucPrimitiveStatusPayload p;
  p.primitive_id = activePrimitiveId;
  p.primitive_type = activePrimitiveType;
  p.primitive_state = getPrimitiveReportedState();
  p.progress_x10 = floatToInt32x10Nuc(fakePrimitiveProgress);
  p.remaining_x10 = floatToInt32x10Nuc(fakePrimitiveRemaining);
  p.heading_error_deg_x10 = floatToInt16x10Nuc(fakePrimitiveHeadingErrorDeg);
  p.fault_flags = getBasicFaultFlags();

  sendNucFrame(NUC_MSG_PRIMITIVE_STATUS, (const uint8_t *)&p, sizeof(p));
}

void updateNucTelemetryTx() {
  static uint32_t lastHealthTxMs = 0;
  static uint32_t lastOdomTxMs = 0;
  static uint32_t lastBaseTxMs = 0;
  static uint32_t lastPrimTxMs = 0;

  uint32_t nowMs = millis();

  // 10 Hz health
  if (nowMs - lastHealthTxMs >= 100) {
    lastHealthTxMs = nowMs;
    sendNucS3Health();
  }

  // 20 Hz local odometry
  if (nowMs - lastOdomTxMs >= 50) {
    lastOdomTxMs = nowMs;
    sendNucOdomLocal();
  }

  // 20 Hz base telemetry forwarded from Slave A
  if (nowMs - lastBaseTxMs >= 50) {
    lastBaseTxMs = nowMs;
    sendNucBaseTelemetry();
  }

  // 10 Hz primitive status for action feedback
  if (nowMs - lastPrimTxMs >= 100) {
    lastPrimTxMs = nowMs;
    sendNucPrimitiveStatus();
  }
}


// =====================================================
// FAKE DATA MODE - SAFE BENCH TEST WITHOUT ROBOT HARDWARE
// =====================================================
void setupFakeDataMode() {
#if KRAI_FAKE_DATA_MODE
  headingReady = true;
  hasAcceptedYaw = true;
  yawZeroDeg = 0.0f;
  headingDeg = 0.0f;
  yawRawDeg = 0.0f;
  poseXcm = 0.0f;
  poseYcm = 0.0f;
  poseDistCm = 0.0f;
  poseThetaDeg = 0.0f;
  poseLinearCmS = 0.0f;

  hasBaseTelemetry = true;
  lastBaseRxMs = millis();
  lastBaseSlaveTimeMs = millis();
  memset(&baseTel, 0, sizeof(baseTel));
  baseTel.slaveTimeMs = millis();
  baseTel.mode = 99;        // fake/simulated mode marker
  baseTel.faultFlags = 0;

  masterAutoMode = MASTER_AUTO_IDLE;
  autoDebugTraveledCm = 0.0f;
  autoDebugRemainingCm = 100.0f;
  autoDebugHeadingErrDeg = 0.0f;

  fakeLastUpdateMs = millis();
  fakeSimCountLeft = 0;
  fakeSimCountRight = 0;
  fakePrimitiveProgressCm = 0.0f;
  activePrimitiveId = 0;
  activePrimitiveType = PRIM_TYPE_NONE;
  activePrimitiveState = PRIM_STATE_IDLE;
  fakePrimitiveActive = false;
  fakePrimitiveProgress = 0.0f;
  fakePrimitiveRemaining = 0.0f;
  fakePrimitiveHeadingErrorDeg = 0.0f;
  fakeHeadingZeroRawDeg = 0.0f;
  s3EstopLatched = false;
  s3SafeStopUntilMs = 0;
  manualControlActive = false;
  lastManualCmdMs = 0;
#endif
}

void updateFakeDataMode() {
#if KRAI_FAKE_DATA_MODE
  uint32_t nowMs = millis();

  if (fakeLastUpdateMs == 0) {
    fakeLastUpdateMs = nowMs;
    lastBaseRxMs = nowMs;
    return;
  }

  float dtSec = (nowMs - fakeLastUpdateMs) / 1000.0f;
  if (dtSec <= 0.0f) {
    return;
  }
  if (dtSec > 0.10f) {
    dtSec = 0.10f;
  }
  fakeLastUpdateMs = nowMs;

  float t = nowMs / 1000.0f;

  if (s3EstopLatched || isSafeStopActive()) {
    poseLinearCmS = 0.0f;
    baseTel.rpmLeft_x10 = 0;
    baseTel.rpmRight_x10 = 0;
    baseTel.pwmLeft = 0;
    baseTel.pwmRight = 0;
    baseTel.mode = s3EstopLatched ? S3_MODE_ESTOP : S3_MODE_SAFE_STOP;
    lastBaseRxMs = nowMs;
    baseTel.slaveTimeMs = nowMs;
    return;
  }

  if (fakePrimitiveActive && activePrimitiveState == PRIM_STATE_RUNNING) {
    if (activePrimitiveType == PRIM_TYPE_MOVE_REL) {
      float step = fakePrimitiveSpeedAbs * dtSec;
      float allowed = max(0.0f, fakePrimitiveTargetAbs - fakePrimitiveProgress);
      if (step > allowed) step = allowed;
      fakePrimitiveProgress += step;
      fakePrimitiveRemaining = max(0.0f, fakePrimitiveTargetAbs - fakePrimitiveProgress);
      fakePrimitiveHeadingErrorDeg = angleDiffDeg(fakePrimitiveTargetHeadingDeg, poseThetaDeg);
      poseLinearCmS = fakePrimitiveSign * fakePrimitiveSpeedAbs;
      if (fakePrimitiveRemaining <= fakePrimitiveTolerance || fakePrimitiveRemaining <= 0.01f) {
        poseLinearCmS = 0.0f;
        fakePrimitiveActive = false;
        activePrimitiveState = PRIM_STATE_DONE;
        fakePrimitiveRemaining = 0.0f;
        fakePrimitiveDoneMs = nowMs;
        masterAutoMode = MASTER_AUTO_DONE;
      }
    } else if (activePrimitiveType == PRIM_TYPE_TURN_REL) {
      float step = fakePrimitiveSpeedAbs * dtSec;
      float allowed = max(0.0f, fakePrimitiveTargetAbs - fakePrimitiveProgress);
      if (step > allowed) step = allowed;
      fakePrimitiveProgress += step;
      fakePrimitiveRemaining = max(0.0f, fakePrimitiveTargetAbs - fakePrimitiveProgress);
      headingDeg = normalize180(headingDeg + fakePrimitiveSign * step);
      poseThetaDeg = headingDeg;
      yawRawDeg = normalize360(fakeHeadingZeroRawDeg + headingDeg);
      fakePrimitiveHeadingErrorDeg = angleDiffDeg(fakePrimitiveTargetHeadingDeg, poseThetaDeg);
      poseLinearCmS = 0.0f;
      if (fakePrimitiveRemaining <= fakePrimitiveTolerance || fakePrimitiveRemaining <= 0.01f) {
        headingDeg = fakePrimitiveTargetHeadingDeg;
        poseThetaDeg = headingDeg;
        yawRawDeg = normalize360(fakeHeadingZeroRawDeg + headingDeg);
        fakePrimitiveHeadingErrorDeg = 0.0f;
        fakePrimitiveActive = false;
        activePrimitiveState = PRIM_STATE_DONE;
        fakePrimitiveRemaining = 0.0f;
        fakePrimitiveDoneMs = nowMs;
        masterAutoMode = MASTER_AUTO_DONE;
      }
    }
  } else if (manualControlActive) {
    // In fake mode, manual commands from ROS2 drive the simulated odometry.
    headingDeg = normalize180(headingDeg + masterCmdAngularDegS * dtSec);
    yawRawDeg = normalize360(fakeHeadingZeroRawDeg + headingDeg);
    poseThetaDeg = headingDeg;
    poseLinearCmS = masterCmdLinearCmS;
  } else {
    // Idle fake mode stays still so primitive tests are easy to read.
    poseLinearCmS = 0.0f;
    yawRawDeg = normalize360(fakeHeadingZeroRawDeg + headingDeg);
    poseThetaDeg = headingDeg;
  }
  headingReady = true;

  float deltaCenterCm = poseLinearCmS * dtSec;
  float thetaRad = poseThetaDeg * DEG_TO_RAD;

  // Same convention as the real odometry: Y = forward, X = sideways.
  poseXcm += deltaCenterCm * sinf(thetaRad);
  poseYcm += deltaCenterCm * cosf(thetaRad);
  poseDistCm += deltaCenterCm;

  int32_t deltaLeftCount = (int32_t)lroundf((deltaCenterCm / WHEEL_CIRCUMFERENCE_CM) * M4_COUNT_PER_REV);
  int32_t deltaRightCount = (int32_t)lroundf((deltaCenterCm / WHEEL_CIRCUMFERENCE_CM) * M1_COUNT_PER_REV);

  fakeSimCountLeft += deltaLeftCount;
  fakeSimCountRight += deltaRightCount;

  float simWheelSpeedCmS = poseLinearCmS;
  if (manualControlActive && fabs(poseLinearCmS) < 0.01f && fabs(masterCmdAngularDegS) > 0.01f) {
    // Pseudo wheel speed for rotate-only fake telemetry.
    simWheelSpeedCmS = fabs(masterCmdAngularDegS) * 0.20f;
  }
  float wheelRevPerSec = simWheelSpeedCmS / WHEEL_CIRCUMFERENCE_CM;
  int16_t rpm_x10 = (int16_t)lroundf(wheelRevPerSec * 60.0f * 10.0f);

  masterDeltaLeftCount = deltaLeftCount;
  masterDeltaRightCount = deltaRightCount;

  baseTel.slaveTimeMs = nowMs;
  baseTel.countLeft = fakeSimCountLeft;
  baseTel.countRight = fakeSimCountRight;
  baseTel.deltaLeft = (int16_t)constrain(deltaLeftCount, -32768, 32767);
  baseTel.deltaRight = (int16_t)constrain(deltaRightCount, -32768, 32767);
  baseTel.rpmLeft_x10 = rpm_x10;
  baseTel.rpmRight_x10 = rpm_x10;
  baseTel.pwmLeft = 80 + (int16_t)lroundf(20.0f * sinf(t * 1.10f));
  baseTel.pwmRight = 80 + (int16_t)lroundf(20.0f * cosf(t * 1.10f));
  baseTel.mode = manualControlActive ? S3_MODE_MANUAL : 99;
  baseTel.faultFlags = 0;

  hasBaseTelemetry = true;
  lastBaseRxMs = nowMs;
  basePacketCount++;

  autoDebugTraveledCm = fakePrimitiveProgress;
  autoDebugRemainingCm = fakePrimitiveRemaining;
  autoDebugHeadingErrDeg = fakePrimitiveHeadingErrorDeg;
#endif
}

// =====================================================
// COMMAND TX MASTER -> SLAVE A
// =====================================================
void sendCmdVel(float linearCmS, float angularDegS) {
  CmdVelPayload p;
  p.masterTimeMs = millis();
  p.linearCmS_x10 = floatToInt16x10Master(linearCmS);
  p.angularDegS_x10 = floatToInt16x10Master(angularDegS);
  p.controlMode = 1;
  p.flags = 0;

  sendBaseFrame(MSG_CMD_VEL, (const uint8_t *)&p, sizeof(p));
  cmdVelTxCount++;
}

void sendCmdStop() {
  sendBaseFrame(MSG_CMD_STOP, nullptr, 0);
  cmdStopTxCount++;
}

void setMasterCmd(float linearCmS, float angularDegS) {
  masterCmdLinearCmS = linearCmS;
  masterCmdAngularDegS = angularDegS;
  masterCmdActive = true;
}

void stopMasterCmd() {
  masterCmdLinearCmS = 0.0f;
  masterCmdAngularDegS = 0.0f;
  masterCmdActive = false;

  // Kirim beberapa kali supaya pasti diterima.
  for (int i = 0; i < 3; i++) {
    sendCmdStop();
    updateBaseUART();
    delay(5);
  }
}

void updateCmdVelTx50Hz() {
  static unsigned long lastCmdTx = 0;

  if (millis() - lastCmdTx >= 20) {
    lastCmdTx = millis();

    if (masterCmdActive) {
      sendCmdVel(masterCmdLinearCmS, masterCmdAngularDegS);
    }
  }
}

// =====================================================
// MASTER AUTO MOVE 100 CM
// =====================================================
const char *getMasterAutoModeText() {
  if (masterAutoMode == MASTER_AUTO_MOVE) return "MOVE";
  if (masterAutoMode == MASTER_AUTO_TURN) return "TURN";
  if (masterAutoMode == MASTER_AUTO_DONE) return "DONE";
  return "IDLE";
}

const char *getMasterSeqModeText() {
  if (masterSeqMode == MASTER_SEQ_RUNNING) return "RUN";
  if (masterSeqMode == MASTER_SEQ_DONE) return "DONE";
  return "IDLE";
}

void startAutoMove100() {
  // Reset pose supaya target 100 cm selalu dari posisi start saat ini.
  resetMasterPose();

  autoStartDistCm = poseDistCm;
  autoTargetDistCm = 100.0f;
  autoTargetHeadingDeg = getHeadingDeg();

  autoDebugTraveledCm = 0.0f;
  autoDebugRemainingCm = autoTargetDistCm;
  autoDebugHeadingErrDeg = 0.0f;

  masterAutoMode = MASTER_AUTO_MOVE;

  // Mulai dari command 0, lalu updateMasterAutoMove akan isi speed.
  setMasterCmd(0.0f, 0.0f);

  DBG_SERIAL.println();
  DBG_SERIAL.println("MASTER AUTO MOVE 100cm START");
  DBG_SERIAL.print("Target heading = ");
  DBG_SERIAL.print(autoTargetHeadingDeg, 2);
  DBG_SERIAL.println(" deg");
  DBG_SERIAL.println();
}

void startAutoTurnDeg(float deltaDeg) {
  autoTurnStartHeadingDeg = getHeadingDeg();
  autoTurnTargetHeadingDeg = normalize180(autoTurnStartHeadingDeg + deltaDeg);

  autoDebugTraveledCm = 0.0f;
  autoDebugRemainingCm = abs(deltaDeg);
  autoDebugHeadingErrDeg = deltaDeg;

  masterAutoMode = MASTER_AUTO_TURN;

  setMasterCmd(0.0f, 0.0f);

  DBG_SERIAL.println();
  DBG_SERIAL.print("MASTER AUTO TURN START | delta=");
  DBG_SERIAL.print(deltaDeg, 2);
  DBG_SERIAL.print(" deg | start=");
  DBG_SERIAL.print(autoTurnStartHeadingDeg, 2);
  DBG_SERIAL.print(" deg | target=");
  DBG_SERIAL.print(autoTurnTargetHeadingDeg, 2);
  DBG_SERIAL.println(" deg");
  DBG_SERIAL.println();
}

void stopMasterAuto() {
  MasterAutoMode oldMode = masterAutoMode;
  masterAutoMode = MASTER_AUTO_IDLE;
  stopMasterCmd();

  if (oldMode != MASTER_AUTO_IDLE) {
    DBG_SERIAL.println("MASTER AUTO STOP");
  }
}

void stopSequenceTest() {
  masterSeqMode = MASTER_SEQ_IDLE;
  masterSeqStep = 0;
  masterSeqWaitingNext = false;
  masterSeqStepDoneMs = 0;
}

void startSequenceStep(uint8_t step) {
  masterSeqStep = step;
  masterSeqWaitingNext = false;
  masterSeqStepDoneMs = 0;

  DBG_SERIAL.println();
  DBG_SERIAL.print("MASTER SEQUENCE STEP ");
  DBG_SERIAL.println(step + 1);

  if (step == 0) {
    DBG_SERIAL.println("SEQ: MOVE 100 cm #1");
    startAutoMove100();
  } else if (step == 1) {
    DBG_SERIAL.println("SEQ: TURN +90 deg");
    startAutoTurnDeg(90.0f);
  } else if (step == 2) {
    DBG_SERIAL.println("SEQ: MOVE 100 cm #2");
    startAutoMove100();
  } else if (step == 3) {
    DBG_SERIAL.println("SEQ: TURN -90 deg");
    startAutoTurnDeg(-90.0f);
  } else {
    masterSeqMode = MASTER_SEQ_DONE;
    masterAutoMode = MASTER_AUTO_IDLE;
    stopMasterCmd();

    DBG_SERIAL.println();
    DBG_SERIAL.println("MASTER SEQUENCE DONE");
    DBG_SERIAL.println();
  }
}

void startSequenceTest() {
  stopMasterAuto();
  stopSequenceTest();

  masterSeqMode = MASTER_SEQ_RUNNING;
  masterSeqStep = 0;

  DBG_SERIAL.println();
  DBG_SERIAL.println("MASTER SEQUENCE START");
  DBG_SERIAL.println("Plan: move 100cm -> turn +90 -> move 100cm -> turn -90");
  DBG_SERIAL.println();

  startSequenceStep(0);
}

void updateSequenceTest() {
  if (masterSeqMode != MASTER_SEQ_RUNNING) {
    return;
  }

  if (masterAutoMode == MASTER_AUTO_DONE) {
    if (!masterSeqWaitingNext) {
      masterSeqWaitingNext = true;
      masterSeqStepDoneMs = millis();
      stopMasterCmd();
      return;
    }

    if (millis() - masterSeqStepDoneMs >= MASTER_SEQ_STEP_DELAY_MS) {
      startSequenceStep(masterSeqStep + 1);
    }
  }
}

void updateMasterAutoMove() {
  if (masterAutoMode == MASTER_AUTO_TURN) {
    float turnProgressDeg = abs(angleDiffDeg(getHeadingDeg(), autoTurnStartHeadingDeg));
    float turnErrorDeg = angleDiffDeg(autoTurnTargetHeadingDeg, getHeadingDeg());
    float remainingDeg = abs(turnErrorDeg);

    autoDebugTraveledCm = turnProgressDeg;
    autoDebugRemainingCm = remainingDeg;
    autoDebugHeadingErrDeg = turnErrorDeg;

    if (remainingDeg <= autoTurnStopDeg) {
      masterAutoMode = MASTER_AUTO_DONE;
      stopMasterCmd();

      DBG_SERIAL.println();
      DBG_SERIAL.print("MASTER AUTO TURN DONE | heading=");
      DBG_SERIAL.print(getHeadingDeg(), 2);
      DBG_SERIAL.print(" deg | target=");
      DBG_SERIAL.print(autoTurnTargetHeadingDeg, 2);
      DBG_SERIAL.print(" deg | error=");
      DBG_SERIAL.print(turnErrorDeg, 2);
      DBG_SERIAL.println(" deg");
      DBG_SERIAL.println();
      return;
    }

    float angularAbsDegS = autoTurnMaxDegS;

    if (remainingDeg < autoTurnSlowdownDeg) {
      float ratio = remainingDeg / autoTurnSlowdownDeg;
      ratio = constrain(ratio, 0.0f, 1.0f);
      angularAbsDegS = autoTurnMinDegS + (autoTurnMaxDegS - autoTurnMinDegS) * ratio;
    }

    angularAbsDegS = constrain(angularAbsDegS, autoTurnMinDegS, autoTurnMaxDegS);

    float turnDir = (turnErrorDeg >= 0.0f) ? 1.0f : -1.0f;
    float angularDegS = headingCorrectionSign * turnDir * angularAbsDegS;

    setMasterCmd(0.0f, angularDegS);
    return;
  }

  if (masterAutoMode != MASTER_AUTO_MOVE) {
    return;
  }

  float traveledCm = poseDistCm - autoStartDistCm;
  float remainingCm = autoTargetDistCm - traveledCm;

  autoDebugTraveledCm = traveledCm;
  autoDebugRemainingCm = remainingCm;

  if (remainingCm <= autoMoveStopCm) {
    masterAutoMode = MASTER_AUTO_DONE;
    stopMasterCmd();

    DBG_SERIAL.println();
    DBG_SERIAL.print("MASTER AUTO MOVE DONE | traveled=");
    DBG_SERIAL.print(traveledCm, 2);
    DBG_SERIAL.print(" cm | remaining=");
    DBG_SERIAL.print(remainingCm, 2);
    DBG_SERIAL.println(" cm");
    DBG_SERIAL.println();
    return;
  }

  // Speed profile: jauh cepat, dekat target turun.
  // Jangan terlalu rendah karena low RPM encoder kamu agak bergerigi.
  float linearCmS = autoMoveMaxCmS;

  if (remainingCm < autoMoveSlowdownCm) {
    float ratio = remainingCm / autoMoveSlowdownCm;
    ratio = constrain(ratio, 0.0f, 1.0f);

    linearCmS = autoMoveMinCmS + (autoMoveMaxCmS - autoMoveMinCmS) * ratio;
  }

  linearCmS = constrain(linearCmS, autoMoveMinCmS, autoMoveMaxCmS);

  // Heading hold dari BNO.
  // angleDiffDeg(target, current) = target - current dalam range -180..180.
  float headingErrorDeg = angleDiffDeg(autoTargetHeadingDeg, getHeadingDeg());
  autoDebugHeadingErrDeg = headingErrorDeg;

  float angularDegS = headingCorrectionSign * headingHoldKp * headingErrorDeg;
  angularDegS = constrain(
    angularDegS,
    -maxHeadingCorrectionDegS,
    maxHeadingCorrectionDegS
  );

  setMasterCmd(linearCmS, angularDegS);
}

// =====================================================
// BASE UART SETUP / POSE
// =====================================================
void setupBaseUART() {
  // Buffer besar supaya CRC/AGE spike berkurang saat Master sibuk I2C/DBG_SERIAL.
  BASE_UART.setRxBufferSize(4096);
  BASE_UART.begin(BASE_UART_BAUD, SERIAL_8N1, BASE_UART_RX, BASE_UART_TX);

  DBG_SERIAL.printf(
    "Base UART ready | baud=%d RX=%d TX=%d\n",
    BASE_UART_BAUD,
    BASE_UART_RX,
    BASE_UART_TX
  );
}

void resetMasterPose() {
  poseXcm = 0.0f;
  poseYcm = 0.0f;
  poseDistCm = 0.0f;
  poseThetaDeg = getHeadingDeg();
  poseLinearCmS = 0.0f;

  lastBaseSlaveTimeMs = 0;

  hasLastBaseCount = false;
  lastBaseCountLeft = 0;
  lastBaseCountRight = 0;

  masterDeltaLeftCount = 0;
  masterDeltaRightCount = 0;

  DBG_SERIAL.println("Master pose reset.");
}

void updateMasterPoseFromBase() {
  // Paket pertama hanya dijadikan baseline count.
  // Jangan langsung dihitung sebagai gerakan.
  if (!hasLastBaseCount) {
    lastBaseCountLeft = baseTel.countLeft;
    lastBaseCountRight = baseTel.countRight;
    lastBaseSlaveTimeMs = baseTel.slaveTimeMs;
    hasLastBaseCount = true;
    return;
  }

  masterDeltaLeftCount = baseTel.countLeft - lastBaseCountLeft;
  masterDeltaRightCount = baseTel.countRight - lastBaseCountRight;

  lastBaseCountLeft = baseTel.countLeft;
  lastBaseCountRight = baseTel.countRight;

  float deltaLeftCm =
    ((float)masterDeltaLeftCount / M4_COUNT_PER_REV) * WHEEL_CIRCUMFERENCE_CM;

  float deltaRightCm =
    ((float)masterDeltaRightCount / M1_COUNT_PER_REV) * WHEEL_CIRCUMFERENCE_CM;

  float deltaCenterCm = (deltaLeftCm + deltaRightCm) * 0.5f;

  float dtSec = 0.0f;

  if (lastBaseSlaveTimeMs != 0) {
    uint32_t dtMs = baseTel.slaveTimeMs - lastBaseSlaveTimeMs;
    dtSec = dtMs / 1000.0f;
  }

  lastBaseSlaveTimeMs = baseTel.slaveTimeMs;

  if (dtSec > 0.0f && dtSec < 0.5f) {
    poseLinearCmS = deltaCenterCm / dtSec;
  }

  // Heading utama dari BNO Master
  poseThetaDeg = getHeadingDeg();
  float thetaRad = poseThetaDeg * DEG_TO_RAD;

  // Konvensi:
  // Y = maju
  // X = samping
  poseXcm += deltaCenterCm * sin(thetaRad);
  poseYcm += deltaCenterCm * cos(thetaRad);

  poseDistCm += deltaCenterCm;
}

void handleBaseFrame(uint8_t type, uint8_t seq, uint8_t len, const uint8_t *payload) {
  (void)seq;

  if (type != MSG_BASE_TELEMETRY) {
    basePacketUnknownType++;
    return;
  }

  if (len != sizeof(BaseTelemetryPayload)) {
    basePacketBadLen++;
    return;
  }

  memcpy(&baseTel, payload, sizeof(BaseTelemetryPayload));

  hasBaseTelemetry = true;
  basePacketCount++;
  lastBaseRxMs = millis();

  updateMasterPoseFromBase();
}

void parseBaseByte(uint8_t b) {
  switch (rxState) {
    case RX_WAIT_SYNC1:
      if (b == FRAME_SYNC1) {
        rxState = RX_WAIT_SYNC2;
      }
      break;

    case RX_WAIT_SYNC2:
      if (b == FRAME_SYNC2) {
        rxState = RX_TYPE;
      } else {
        rxState = RX_WAIT_SYNC1;
      }
      break;

    case RX_TYPE:
      rxType = b;
      rxState = RX_SEQ;
      break;

    case RX_SEQ:
      rxSeq = b;
      rxState = RX_LEN;
      break;

    case RX_LEN:
      rxLen = b;
      rxIndex = 0;

      if (rxLen > sizeof(rxBuf)) {
        rxState = RX_WAIT_SYNC1;
      } else if (rxLen == 0) {
        rxState = RX_CRC;
      } else {
        rxState = RX_PAYLOAD;
      }
      break;

    case RX_PAYLOAD:
      rxBuf[rxIndex++] = b;

      if (rxIndex >= rxLen) {
        rxState = RX_CRC;
      }
      break;

    case RX_CRC: {
      uint8_t cs = checksumXor(rxType, rxSeq, rxLen, rxBuf);

      if (cs == b) {
        handleBaseFrame(rxType, rxSeq, rxLen, rxBuf);
      } else {
        basePacketBadCrc++;
      }

      rxState = RX_WAIT_SYNC1;
      break;
    }
  }
}

void updateBaseUART() {
  while (BASE_UART.available()) {
    parseBaseByte((uint8_t)BASE_UART.read());
  }
}

void printMasterBaseDebug() {
  float rpmLeft = baseTel.rpmLeft_x10 / 10.0f;
  float rpmRight = baseTel.rpmRight_x10 / 10.0f;

  uint32_t ageMs = millis() - lastBaseRxMs;

  DBG_SERIAL.printf(
    "SEQ=%s S=%u AUTO=%s T=%.1f R=%.1f HE=%.2f | "
    "H=%.2f JUMP=%lu REAL=%lu PEND=%u | X=%.1f Y=%.1f Dist=%.1fcm V=%.1f | "
    "dLM=%ld dRM=%ld dLS=%d dRS=%d RPM_L=%.1f RPM_R=%.1f PWM_L=%d PWM_R=%d | "
    "MODE=%u TXV=%.1f TXW=%.1f TX=%lu STOP=%lu | "
    "PKT=%lu CRC=%lu LEN=%lu UNK=%lu AGE=%lums\n",
    getMasterSeqModeText(),
    masterSeqStep,
    getMasterAutoModeText(),
    autoDebugTraveledCm,
    autoDebugRemainingCm,
    autoDebugHeadingErrDeg,
    getHeadingDeg(),
    (unsigned long)yawJumpRejectCount,
    (unsigned long)yawRealJumpAcceptCount,
    hasPendingJumpYaw ? 1 : 0,
    poseXcm,
    poseYcm,
    poseDistCm,
    poseLinearCmS,
    (long)masterDeltaLeftCount,
    (long)masterDeltaRightCount,
    baseTel.deltaLeft,
    baseTel.deltaRight,
    rpmLeft,
    rpmRight,
    baseTel.pwmLeft,
    baseTel.pwmRight,
    baseTel.mode,
    masterCmdLinearCmS,
    masterCmdAngularDegS,
    (unsigned long)cmdVelTxCount,
    (unsigned long)cmdStopTxCount,
    (unsigned long)basePacketCount,
    (unsigned long)basePacketBadCrc,
    (unsigned long)basePacketBadLen,
    (unsigned long)basePacketUnknownType,
    (unsigned long)ageMs
  );
}

// =====================================================
// BNO STATUS
// =====================================================
void updateBNOStatus() {
  bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
  bno.getSystemStatus(&systemStatus, &selfTestResult, &systemError);
}

// =====================================================
// PRINT OFFSETS
// =====================================================
void printOffsets(const adafruit_bno055_offsets_t &calibData) {
  DBG_SERIAL.println("Offset:");
  DBG_SERIAL.print("accel_offset_x: "); DBG_SERIAL.println(calibData.accel_offset_x);
  DBG_SERIAL.print("accel_offset_y: "); DBG_SERIAL.println(calibData.accel_offset_y);
  DBG_SERIAL.print("accel_offset_z: "); DBG_SERIAL.println(calibData.accel_offset_z);

  DBG_SERIAL.print("mag_offset_x: "); DBG_SERIAL.println(calibData.mag_offset_x);
  DBG_SERIAL.print("mag_offset_y: "); DBG_SERIAL.println(calibData.mag_offset_y);
  DBG_SERIAL.print("mag_offset_z: "); DBG_SERIAL.println(calibData.mag_offset_z);

  DBG_SERIAL.print("gyro_offset_x: "); DBG_SERIAL.println(calibData.gyro_offset_x);
  DBG_SERIAL.print("gyro_offset_y: "); DBG_SERIAL.println(calibData.gyro_offset_y);
  DBG_SERIAL.print("gyro_offset_z: "); DBG_SERIAL.println(calibData.gyro_offset_z);

  DBG_SERIAL.print("accel_radius: "); DBG_SERIAL.println(calibData.accel_radius);
  DBG_SERIAL.print("mag_radius: "); DBG_SERIAL.println(calibData.mag_radius);
}

// =====================================================
// LOAD + RESTORE OFFSET DARI NVS
// =====================================================
bool loadBNOOffsetsFromNVS() {
  prefs.begin("bno55", true);

  bool valid = prefs.getBool("valid", false);

  if (!valid) {
    prefs.end();
    DBG_SERIAL.println("ERROR: Tidak ada offset valid di NVS.");
    return false;
  }

  adafruit_bno055_offsets_t calibData;
  size_t len = prefs.getBytes("offsets", &calibData, sizeof(calibData));
  prefs.end();

  if (len != sizeof(calibData)) {
    DBG_SERIAL.println("ERROR: Ukuran offset di NVS salah.");
    return false;
  }

  DBG_SERIAL.println("Offset ditemukan di NVS.");
  printOffsets(calibData);

  DBG_SERIAL.println("Restore offset via CONFIG mode...");

  bno.setMode(OPERATION_MODE_CONFIG);
  delay(50);

  bno.setSensorOffsets(calibData);
  delay(50);

  bno.setMode(OPERATION_MODE_NDOF);
  delay(700);

  DBG_SERIAL.println("Offset berhasil direstore. Mode = NDOF.");

  updateBNOStatus();

  DBG_SERIAL.print("Status setelah restore: STATUS=");
  DBG_SERIAL.print(systemStatus);
  DBG_SERIAL.print(" SELFTEST=0x");
  DBG_SERIAL.print(selfTestResult, HEX);
  DBG_SERIAL.print(" ERROR=");
  DBG_SERIAL.println(systemError);

  return true;
}

// =====================================================
// ROBUST EULER READ
// Return true kalau frame valid.
// Kalau frame 0,0,0 muncul, pakai last valid.
// =====================================================
bool readEulerRobust(float &yaw, float &roll, float &pitch) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float y = normalize360(euler.x());
  float r = euler.y();
  float p = euler.z();

  bool allZero =
    abs(y) < 0.001f &&
    abs(r) < 0.001f &&
    abs(p) < 0.001f;

  if (hasLastValid && allZero) {
    yaw = lastYaw;
    roll = lastRoll;
    pitch = lastPitch;

    badFrameCount++;
    return false;
  }

  yaw = y;
  roll = r;
  pitch = p;

  lastYaw = yaw;
  lastRoll = roll;
  lastPitch = pitch;
  hasLastValid = true;

  okFrameCount++;
  return true;
}

// =====================================================
// ACCEPT YAW + JUMP REJECTION
// =====================================================
bool acceptYaw(float yaw) {
  uint32_t nowMs = millis();

  if (!hasAcceptedYaw) {
    acceptedYawRawDeg = yaw;
    lastAcceptedYawMs = nowMs;

    yawRawDeg = acceptedYawRawDeg;
    headingDeg = angleDiffDeg(yawRawDeg, yawZeroDeg);

    hasAcceptedYaw = true;
    headingReady = true;

    hasPendingJumpYaw = false;
    return true;
  }

  float dtSec = (nowMs - lastAcceptedYawMs) / 1000.0f;

  if (dtSec <= 0.0f) {
    dtSec = 0.001f;
  }

  if (dtSec > YAW_FILTER_MAX_DT_SEC) {
    dtSec = YAW_FILTER_MAX_DT_SEC;
  }

  float delta = angleDiffDeg(yaw, acceptedYawRawDeg);
  float maxAllowedDelta = (MAX_YAW_RATE_DEG_S * dtSec) + YAW_JUMP_MARGIN_DEG;

  // Normal yaw change: terima langsung.
  if (abs(delta) <= maxAllowedDelta) {
    hasPendingJumpYaw = false;

    acceptedYawRawDeg = yaw;
    lastAcceptedYawMs = nowMs;

    yawRawDeg = acceptedYawRawDeg;
    headingDeg = angleDiffDeg(yawRawDeg, yawZeroDeg);

    headingReady = true;
    return true;
  }

  // Sampai sini berarti yaw berubah terlalu besar untuk diterima langsung.
  // Jangan langsung dibuang total. Cek apakah nilai baru ini stabil.
  yawJumpRejectCount++;

  if (!hasPendingJumpYaw) {
    hasPendingJumpYaw = true;
    pendingJumpYawDeg = yaw;
    pendingJumpStartMs = nowMs;
  } else {
    float pendingDiff = abs(angleDiffDeg(yaw, pendingJumpYawDeg));

    if (pendingDiff <= REAL_JUMP_STABLE_DEG) {
      if (nowMs - pendingJumpStartMs >= REAL_JUMP_ACCEPT_MS) {
        // Yaw besar ini stabil cukup lama, jadi anggap gerakan fisik asli.
        acceptedYawRawDeg = yaw;
        lastAcceptedYawMs = nowMs;

        yawRawDeg = acceptedYawRawDeg;
        headingDeg = angleDiffDeg(yawRawDeg, yawZeroDeg);

        headingReady = true;
        hasPendingJumpYaw = false;
        yawRealJumpAcceptCount++;
        return true;
      }
    } else {
      // Kandidat berubah lagi, mulai ulang pending.
      pendingJumpYawDeg = yaw;
      pendingJumpStartMs = nowMs;
    }
  }

  // Sementara masih dianggap spike/pending, tahan heading lama.
  yawRawDeg = acceptedYawRawDeg;
  headingDeg = angleDiffDeg(yawRawDeg, yawZeroDeg);

  headingReady = true;
  return false;
}

// =====================================================
// UPDATE HEADING
// Panggil ini terus di loop Master.
// =====================================================
bool updateHeading() {
  float yaw, roll, pitch;
  bool frameValid = readEulerRobust(yaw, roll, pitch);

  if (!frameValid) {
    yawRawDeg = acceptedYawRawDeg;
    headingDeg = angleDiffDeg(yawRawDeg, yawZeroDeg);

    headingReady = hasAcceptedYaw;
    return false;
  }

  return acceptYaw(yaw);
}

// =====================================================
// WAIT BNO YAW STABLE
// Dipakai di startup setelah restore offset.
// =====================================================
bool waitBNOYawStable() {
  DBG_SERIAL.println();
  DBG_SERIAL.println("Menunggu BNO yaw stabil sebelum reset heading...");

  uint32_t startMs = millis();
  uint32_t stableSinceMs = 0;

  float prevYaw = 0.0f;
  bool hasPrev = false;

  while (millis() - startMs < BNO_STARTUP_TIMEOUT_MS) {
    float yaw, roll, pitch;
    bool valid = readEulerRobust(yaw, roll, pitch);

    if (valid) {
      if (!hasPrev) {
        prevYaw = yaw;
        hasPrev = true;
        stableSinceMs = millis();
      } else {
        float diff = abs(angleDiffDeg(yaw, prevYaw));

        if (diff <= BNO_STABLE_DIFF_DEG) {
          if ((millis() - stableSinceMs >= BNO_STABLE_REQUIRED_MS) &&
              (millis() - startMs >= BNO_MIN_STARTUP_SETTLE_MS)) {
            DBG_SERIAL.println("BNO yaw sudah stabil.");
            return true;
          }
        } else {
          stableSinceMs = millis();
        }

        prevYaw = yaw;
      }
    }

    delay(50);
  }

  DBG_SERIAL.println("WARNING: Timeout menunggu yaw stabil. Lanjut dengan nilai terakhir.");
  return false;
}

// =====================================================
// RESET HEADING KE 0
// Dipakai saat start autonomous / sebelum match.
// =====================================================
void resetHeadingZero() {
  float yaw = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;

  bool gotValid = false;

  for (int i = 0; i < 10; i++) {
    gotValid = readEulerRobust(yaw, roll, pitch);
    delay(20);
  }

  if (!gotValid && hasLastValid) {
    yaw = lastYaw;
  }

  yawZeroDeg = yaw;
  headingDeg = 0.0f;
  yawRawDeg = yaw;

  hasAcceptedYaw = false;
  headingReady = false;
  hasPendingJumpYaw = false;

  acceptYaw(yaw);

  DBG_SERIAL.println();
  DBG_SERIAL.print("Heading zero reset. YawZero=");
  DBG_SERIAL.print(yawZeroDeg, 2);
  DBG_SERIAL.println(" deg");
  DBG_SERIAL.println();
}

// =====================================================
// INIT BNO HEADING
// =====================================================
bool setupBNOHeading() {
  Wire.begin(BNO_SDA, BNO_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    DBG_SERIAL.println("ERROR: BNO055 tidak terdeteksi.");
    return false;
  }

  DBG_SERIAL.println("BNO055 begin OK.");
  delay(1000);

  bno.setExtCrystalUse(true);
  delay(1000);

  bno.setMode(OPERATION_MODE_NDOF);
  delay(500);

  if (!loadBNOOffsetsFromNVS()) {
    DBG_SERIAL.println("ERROR: Gagal load offset BNO.");
    DBG_SERIAL.println("Upload sketch kalibrasi dulu.");
    return false;
  }

  delay(1000);

  okFrameCount = 0;
  badFrameCount = 0;
  yawJumpRejectCount = 0;
  yawRealJumpAcceptCount = 0;

  hasLastValid = false;
  hasAcceptedYaw = false;
  headingReady = false;
  hasPendingJumpYaw = false;

  waitBNOYawStable();

  hasAcceptedYaw = false;
  headingReady = false;
  hasPendingJumpYaw = false;

  resetHeadingZero();

  float yaw, roll, pitch;
  if (readEulerRobust(yaw, roll, pitch)) {
    acceptYaw(yaw);
  }

  updateBNOStatus();

  return true;
}

// =====================================================
// SERIAL COMMAND HANDLER
// =====================================================
void handleSerialCommand(char c) {
  if (c == 'z' || c == 'Z') {
    resetHeadingZero();
  }

  else if (c == 'r' || c == 'R') {
    resetMasterPose();
  }

  else if (c == 'x' || c == 'X') {
    stopSequenceTest();
    startAutoMove100();
  }

  else if (c == 'n' || c == 'N') {
    startSequenceTest();
  }

  else if (c == 'o' || c == 'O') {
    stopSequenceTest();
    startAutoTurnDeg(90.0f);
  }

  else if (c == 'p' || c == 'P') {
    stopSequenceTest();
    startAutoTurnDeg(-90.0f);
  }

  else if (c == 'f' || c == 'F') {
    stopSequenceTest();
    masterAutoMode = MASTER_AUTO_IDLE;
    setMasterCmd(TEST_LINEAR_CM_S, 0.0f);
    DBG_SERIAL.println("CMD: forward 20 cm/s");
  }

  else if (c == 'b' || c == 'B') {
    stopSequenceTest();
    masterAutoMode = MASTER_AUTO_IDLE;
    setMasterCmd(-TEST_LINEAR_CM_S, 0.0f);
    DBG_SERIAL.println("CMD: backward 20 cm/s");
  }

  else if (c == 'l' || c == 'L') {
    stopSequenceTest();
    masterAutoMode = MASTER_AUTO_IDLE;
    setMasterCmd(0.0f, TEST_ANGULAR_DEG_S);
    DBG_SERIAL.println("CMD: rotate left 30 deg/s");
  }

  // r sudah dipakai reset pose, jadi rotate right pakai q.
  else if (c == 'q' || c == 'Q') {
    stopSequenceTest();
    masterAutoMode = MASTER_AUTO_IDLE;
    setMasterCmd(0.0f, -TEST_ANGULAR_DEG_S);
    DBG_SERIAL.println("CMD: rotate right 30 deg/s");
  }

  else if (c == 's' || c == 'S') {
    stopSequenceTest();
    stopMasterAuto();
    DBG_SERIAL.println("CMD: stop");
  }

  else if (c == '0') {
    stopSequenceTest();
    masterAutoMode = MASTER_AUTO_IDLE;
    setMasterCmd(0.0f, 0.0f);
    DBG_SERIAL.println("CMD: zero velocity keepalive");
  }

  else if (c == 'h' || c == 'H' || c == '?') {
    DBG_SERIAL.println();
    DBG_SERIAL.println("Commands:");
    DBG_SERIAL.println("  n = SEQUENCE: move 100 -> turn +90 -> move 100 -> turn -90");
    DBG_SERIAL.println("  x = AUTO move 100 cm pakai Master pose + BNO heading hold");
    DBG_SERIAL.println("  o = AUTO rotate kanan +90 deg pakai BNO heading");
    DBG_SERIAL.println("  p = AUTO rotate kiri -90 deg pakai BNO heading");
    DBG_SERIAL.println("  f = forward 20 cm/s manual CMD_VEL");
    DBG_SERIAL.println("  b = backward 20 cm/s manual CMD_VEL");
    DBG_SERIAL.println("  l = rotate left 30 deg/s manual CMD_VEL");
    DBG_SERIAL.println("  q = rotate right 30 deg/s manual CMD_VEL");
    DBG_SERIAL.println("  s = stop CMD / stop AUTO");
    DBG_SERIAL.println("  0 = zero velocity keepalive");
    DBG_SERIAL.println("  z = reset BNO heading zero");
    DBG_SERIAL.println("  r = reset Master pose");
    DBG_SERIAL.println();
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  setupNucSerial();
  DBG_SERIAL.begin(115200);
  delay(1000);

  DBG_SERIAL.println();
  DBG_SERIAL.println("====================================");
  DBG_SERIAL.println("MASTER ESP32-S3 - BNO055 + SLAVE A CMD_VEL + AUTO MOVE");
  DBG_SERIAL.println("BNO heading + encoder telemetry fusion + CMD_VEL sender");
  DBG_SERIAL.print("BNO SDA GPIO"); DBG_SERIAL.println(BNO_SDA);
  DBG_SERIAL.print("BNO SCL GPIO"); DBG_SERIAL.println(BNO_SCL);
  DBG_SERIAL.print("BNO I2C CLOCK "); DBG_SERIAL.println(I2C_CLOCK_HZ);
  DBG_SERIAL.print("UART RX GPIO"); DBG_SERIAL.println(BASE_UART_RX);
  DBG_SERIAL.print("UART TX GPIO"); DBG_SERIAL.println(BASE_UART_TX);
  DBG_SERIAL.println("====================================");

#if KRAI_FAKE_DATA_MODE
  setupFakeDataMode();
#else
  if (!setupBNOHeading()) {
    DBG_SERIAL.println("BNO heading init failed.");
    while (1) {
      delay(1000);
    }
  }

  setupBaseUART();
  resetMasterPose();
#endif

  DBG_SERIAL.println("Master ready.");
  DBG_SERIAL.println("Command:");
  DBG_SERIAL.println("  n = SEQUENCE: move 100 -> turn +90 -> move 100 -> turn -90");
  DBG_SERIAL.println("  x = AUTO move 100 cm pakai Master pose + BNO heading hold");
  DBG_SERIAL.println("  o = AUTO rotate kanan +90 deg pakai BNO heading");
  DBG_SERIAL.println("  p = AUTO rotate kiri -90 deg pakai BNO heading");
  DBG_SERIAL.println("  f = forward 20 cm/s");
  DBG_SERIAL.println("  b = backward 20 cm/s");
  DBG_SERIAL.println("  l = rotate left 30 deg/s");
  DBG_SERIAL.println("  q = rotate right 30 deg/s");
  DBG_SERIAL.println("  s = stop CMD / AUTO");
  DBG_SERIAL.println("  0 = zero velocity keepalive");
  DBG_SERIAL.println("  z = reset heading BNO ke 0");
  DBG_SERIAL.println("  r = reset master pose x/y/distance");
  DBG_SERIAL.println();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static unsigned long lastPrint = 0;
  static unsigned long lastStatus = 0;

#if KRAI_FAKE_DATA_MODE
  updateNucRx();
  updateManualCommandWatchdog();
  updateFakeDataMode();
  updateNucTelemetryTx();
  delay(1);
  return;
#else
  updateNucRx();
  updateManualCommandWatchdog();
  updateHeading();
  updateBaseUART();
  updateNucTelemetryTx();

  // Position controller Master: menghasilkan masterCmdLinearCmS/masterCmdAngularDegS.
  updateMasterAutoMove();
  updateSequenceTest();
  updateCmdVelTx50Hz();

  if (DBG_SERIAL.available()) {
    char c = DBG_SERIAL.read();
    handleSerialCommand(c);
  }

  updateBaseUART();
  updateNucRx();
  updateMasterAutoMove();
  updateSequenceTest();
  updateCmdVelTx50Hz();

  // Status BNO tidak perlu sering. Ini mengurangi blocking I2C dan AGE spike UART.
  if (millis() - lastStatus >= 5000) {
    lastStatus = millis();
    updateBNOStatus();
  }

  // Human-readable debug print is disabled when USB CDC is used for binary ROS2 protocol.
  if (ENABLE_USB_HUMAN_DEBUG && millis() - lastPrint >= 200) {
    lastPrint = millis();

    updateBaseUART();

    if (hasBaseTelemetry) {
      printMasterBaseDebug();
    } else {
      DBG_SERIAL.printf(
        "Waiting Slave A telemetry... AUTO=%s H=%.2f BAD=%lu JUMP=%lu TXV=%.1f TXW=%.1f TX=%lu STOP=%lu\n",
        getMasterAutoModeText(),
        getHeadingDeg(),
        (unsigned long)getBadFrameCount(),
        (unsigned long)getYawJumpRejectCount(),
        masterCmdLinearCmS,
        masterCmdAngularDegS,
        (unsigned long)cmdVelTxCount,
        (unsigned long)cmdStopTxCount
      );
    }

    updateBaseUART();
    updateMasterAutoMove();
    updateSequenceTest();
    updateCmdVelTx50Hz();
  }
#endif
}
