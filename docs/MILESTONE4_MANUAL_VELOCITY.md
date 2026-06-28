# KRAI Milestone 4 - Manual Velocity Command

Milestone 4 adds ROS2 manual velocity control on top of the previous safe bridge milestones.

This milestone still does **not** include autonomous primitive execution (`MOVE_REL` / `TURN_REL`).
Those are intentionally reserved for a later milestone.

## Active communication path

```text
ROS2 /manual/cmd_vel
  -> krai_s3_bridge
  -> NUC_S3_PROTOCOL_V1 CMD_VEL_MANUAL
  -> ESP32-S3
  -> fake odom in bench mode
  -> Slave A CMD_VEL in real robot mode
```

## Services

```text
/s3_ping          std_srvs/srv/Trigger
/reset_odom       krai_interfaces/srv/ResetOdom
/reset_heading    krai_interfaces/srv/ResetHeading
/cmd_stop         std_srvs/srv/Trigger
/emergency_stop   krai_interfaces/srv/EmergencyStop
/manual_control   std_srvs/srv/SetBool
```

## Manual control flow

Enable manual control:

```bash
ros2 service call /manual_control std_srvs/srv/SetBool "{data: true}"
```

Publish a single forward command:

```bash
ros2 topic pub --once /manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Publish a short stream at 10 Hz:

```bash
ros2 topic pub -r 10 /manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Rotate in place:

```bash
ros2 topic pub -r 10 /manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.35}}"
```

Stop safely:

```bash
ros2 service call /cmd_stop std_srvs/srv/Trigger {}
```

Disable manual control:

```bash
ros2 service call /manual_control std_srvs/srv/SetBool "{data: false}"
```

## Safety gates

The bridge will not forward manual `/manual/cmd_vel` when manual control is disabled.

The bridge disables manual control if S3 reports ESTOP.

The bridge clamps manual commands before transmission:

```text
max_manual_linear_cm_s  = 25.0
max_manual_angular_deg_s = 45.0
```

The bridge sends zero velocity when manual command timeout is exceeded:

```text
manual_command_timeout_sec = 0.35
```

The S3 firmware also has a local watchdog:

```text
MANUAL_CMD_TIMEOUT_MS = 300
```

So safety is layered:

```text
ROS2 bridge timeout -> zero command
S3 timeout          -> stop motor command
Slave A watchdog    -> motor stop
```

## Fake mode behavior

When `KRAI_FAKE_DATA_MODE = 1`, manual velocity changes the fake odometry so the pipeline can be tested without robot hardware.

When `KRAI_FAKE_DATA_MODE = 0`, the same manual command updates S3 `masterCmdLinearCmS` / `masterCmdAngularDegS`, and the existing S3 50 Hz `CMD_VEL` sender forwards the command to Slave A.

## Definition of done

```text
/manual_control enable returns success
/manual/cmd_vel changes /s3/odom_local in fake mode
/s3/health mode becomes 1 while manual command is fresh
/cmd_stop stops odom movement
/emergency_stop blocks manual command
crc_errors remains 0
```
