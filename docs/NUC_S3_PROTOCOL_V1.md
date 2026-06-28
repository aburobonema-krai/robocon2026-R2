# KRAI NUC_S3_PROTOCOL_V1

Milestone 1 is read-only: ESP32-S3 sends telemetry to the Intel NUC. The NUC/ROS2 bridge does not send motion commands yet.

## Physical links

```text
NUC USB <-> ESP32-S3 USB CDC Serial
ESP32-S3 UART2 <-> Slave A
```

The NUC only talks to S3. S3 only talks to Slave A.

## Frame format

All integer fields are little-endian.

```text
SYNC1       uint8   0xAA
SYNC2       uint8   0x55
VERSION     uint8   0x01
TYPE        uint8
SEQ         uint8
LEN_L       uint8
LEN_H       uint8
PAYLOAD     bytes
CRC16_L     uint8
CRC16_H     uint8
```

CRC is CRC-16/CCITT-FALSE over `VERSION, TYPE, SEQ, LEN_L, LEN_H, PAYLOAD`.

CRC parameters:

```text
poly   = 0x1021
init   = 0xFFFF
xorout = 0x0000
reflect input/output = false
```

## S3 -> NUC message IDs

```text
0x81 PONG
0x82 ACK
0x83 NACK
0x90 ODOM_LOCAL
0x91 BASE_TELEMETRY
0x92 S3_HEALTH
0x93 PRIMITIVE_STATUS
0x94 FAULT_STATUS
```

## ODOM_LOCAL payload, type 0x90, 20 bytes

```c
struct OdomLocalPayload {
  int32_t x_cm_x10;
  int32_t y_cm_x10;
  int16_t heading_deg_x10;
  int32_t dist_cm_x10;
  int16_t linear_cm_s_x10;
  int16_t yaw_raw_deg_x10;
  uint8_t heading_ready;
  uint8_t reserved;
} __attribute__((packed));
```

ROS topic: `/s3/odom_local` as `nav_msgs/Odometry`.

## BASE_TELEMETRY payload, type 0x91, 24 bytes

```c
struct NucBaseTelemetryPayload {
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
} __attribute__((packed));
```

ROS topic: `/base/telemetry` as `krai_interfaces/msg/BaseTelemetry`.

## S3_HEALTH payload, type 0x92, 34 bytes

```c
struct S3HealthPayload {
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
} __attribute__((packed));
```

ROS topic: `/s3/health` as `krai_interfaces/msg/S3Health`.

## PRIMITIVE_STATUS payload, type 0x93, 16 bytes

This is included now even though milestone 1 does not execute primitives yet.

```c
struct PrimitiveStatusPayload {
  uint16_t primitive_id;
  uint8_t primitive_type;
  uint8_t primitive_state;
  int32_t progress_x10;
  int32_t remaining_x10;
  int16_t heading_error_deg_x10;
  uint16_t fault_flags;
} __attribute__((packed));
```

ROS topic: `/motion/primitive_status` as `krai_interfaces/msg/PrimitiveStatus`.

## Recommended publish rates for milestone 1

```text
S3_HEALTH        10 Hz
ODOM_LOCAL       20 Hz
BASE_TELEMETRY   20 Hz
```

## NUC -> S3 command IDs added by milestones 2-4

```text
0x01 PING
0x02 CMD_STOP
0x03 CMD_ESTOP
0x04 RESET_ODOM
0x05 RESET_HEADING
0x10 CMD_VEL_MANUAL
```

## CMD_VEL_MANUAL payload, type 0x10, 6 bytes

```c
struct CmdVelManualPayload {
  int16_t linear_cm_s_x10;
  int16_t angular_deg_s_x10;
  uint8_t control_mode;
  uint8_t flags;
} __attribute__((packed));
```

ROS input topic: `/manual/cmd_vel` as `geometry_msgs/msg/Twist`.

Bridge conversion:

```text
Twist.linear.x  m/s   -> linear_cm_s_x10
Twist.angular.z rad/s -> angular_deg_s_x10
```

This command is for manual/debug motion only. Autonomous primitive commands are not enabled in milestone 4.
