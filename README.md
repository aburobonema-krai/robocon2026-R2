# KRAI Robot ROS2 Workspace

ROS2 Humble workspace for Intel NUC integration with ESP32-S3.

Current milestone: **Milestone 4 - manual velocity command**.

Architecture:

```text
NUC USB <-> ESP32-S3 USB CDC Serial
ESP32-S3 UART2 <-> Slave A

NUC only talks to S3.
S3 only talks to Slave A.
```

## Build

```bash
cd ~/krai_robot_ws
colcon build --symlink-install
source install/setup.bash
```

## Run S3 bridge

```bash
ros2 launch krai_bringup base_bringup.launch.py
```

Default serial settings:

```text
port: /dev/ttyACM0
baud: 115200
```

Override serial port if needed:

```bash
ros2 launch krai_bringup base_bringup.launch.py serial_port:=/dev/ttyACM1
```

## Monitor topics

```bash
ros2 topic echo /s3_bridge/debug
ros2 topic echo /s3/health
ros2 topic echo /s3/odom_local
ros2 topic echo /base/telemetry
```

## Safe services

```bash
ros2 service call /s3_ping std_srvs/srv/Trigger {}
ros2 service call /reset_odom krai_interfaces/srv/ResetOdom {}
ros2 service call /reset_heading krai_interfaces/srv/ResetHeading {}
ros2 service call /cmd_stop std_srvs/srv/Trigger {}
ros2 service call /emergency_stop krai_interfaces/srv/EmergencyStop "{enable: true}"
ros2 service call /emergency_stop krai_interfaces/srv/EmergencyStop "{enable: false}"
```

## Manual velocity test

Enable manual control:

```bash
ros2 service call /manual_control std_srvs/srv/SetBool "{data: true}"
```

Forward at 10 cm/s:

```bash
ros2 topic pub -r 10 /manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Rotate at about 20 deg/s:

```bash
ros2 topic pub -r 10 /manual/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.35}}"
```

Stop:

```bash
ros2 service call /cmd_stop std_srvs/srv/Trigger {}
```

Disable manual control:

```bash
ros2 service call /manual_control std_srvs/srv/SetBool "{data: false}"
```

## Protocol docs

See `docs/NUC_S3_PROTOCOL_V1.md` and `docs/MILESTONE4_MANUAL_VELOCITY.md`.

## Firmware

Use `krai_s3_master_milestone4_manual_velocity.ino` for the matching ESP32-S3 firmware.

`KRAI_FAKE_DATA_MODE = 1` is safe bench mode without robot hardware.

## Milestone 5

Milestone 5 adds `/execute_primitive` as a ROS2 action server in `krai_s3_bridge`.

Example:

```bash
ros2 action send_goal --feedback /execute_primitive krai_interfaces/action/ExecutePrimitive \
"{primitive_id: 101, primitive_type: 'move_rel', distance_cm: 50.0, angle_deg: 0.0, max_linear_cm_s: 20.0, max_angular_deg_s: 0.0, tolerance_cm: 2.0, tolerance_deg: 0.0, heading_hold_enable: true}"
```


## Milestone 6 - Mission YAML Executor

Run full stack:

```bash
ros2 launch krai_bringup full_system.launch.py
```

Run a mission:

```bash
ros2 action send_goal --feedback /run_mission krai_interfaces/action/RunMission "{mission_file: 'debug_forward_turn.yaml'}"
```
