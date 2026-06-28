# KRAI Milestone 2: Safe Commands

Milestone 2 keeps all motion commands disabled. The NUC may only send safe non-motion commands to the ESP32-S3:

- `PING` (`0x01`) -> S3 replies `PONG` (`0x81`)
- `RESET_ODOM` (`0x04`) -> S3 resets local odometry and replies `ACK` (`0x82`)
- `RESET_HEADING` (`0x05`) -> S3 resets heading zero and replies `ACK` (`0x82`)

Default NUC<->S3 serial settings for this milestone:

```yaml
port: /dev/ttyACM0
baud: 115200
```

ROS2 services exposed by `krai_s3_bridge`:

```bash
ros2 service call /s3_ping std_srvs/srv/Trigger {}
ros2 service call /reset_odom krai_interfaces/srv/ResetOdom {}
ros2 service call /reset_heading krai_interfaces/srv/ResetHeading {}
```

Telemetry topics from milestone 1 remain available:

```bash
ros2 topic echo /s3_bridge/debug
ros2 topic echo /s3/health
ros2 topic echo /s3/odom_local
ros2 topic echo /base/telemetry
ros2 topic echo /motion/primitive_status
```

Motion commands such as `CMD_VEL`, `MOVE_REL`, and `TURN_REL` are intentionally not implemented yet.
