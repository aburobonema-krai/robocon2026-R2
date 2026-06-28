# KRAI Milestone 5 - Primitive Executor Fake

Milestone 5 enables ROS2 action-based primitive execution while keeping the system safe for bench testing.

Enabled commands:
- `/s3_ping`
- `/reset_odom`
- `/reset_heading`
- `/cmd_stop`
- `/emergency_stop`
- `/manual_control`
- `/manual/cmd_vel`
- `/execute_primitive` action for `move_rel` and `turn_rel`

Still not included:
- Mission YAML executor
- Vision integration
- Real Slave B mechanism commands

Firmware default:
```cpp
#define KRAI_FAKE_DATA_MODE 1
```
This means MOVE_REL and TURN_REL are simulated on the ESP32-S3 and do not drive real motors.
