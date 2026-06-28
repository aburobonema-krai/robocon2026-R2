# KRAI Milestone 3 - Safety Commands

Milestone 3 adds safe command services on top of Milestone 2. Motion velocity and primitive movement commands remain disabled.

## Services

```bash
ros2 service call /s3_ping std_srvs/srv/Trigger {}
ros2 service call /reset_odom krai_interfaces/srv/ResetOdom {}
ros2 service call /reset_heading krai_interfaces/srv/ResetHeading {}
ros2 service call /cmd_stop std_srvs/srv/Trigger {}
ros2 service call /emergency_stop krai_interfaces/srv/EmergencyStop "{enable: true}"
ros2 service call /emergency_stop krai_interfaces/srv/EmergencyStop "{enable: false}"
```

## Command IDs

- `0x01` PING
- `0x02` CMD_STOP
- `0x03` CMD_ESTOP, payload `uint8 enable`
- `0x04` RESET_ODOM
- `0x05` RESET_HEADING

## Expected behavior

- `/cmd_stop` commands S3 to stop local motion and forward stop to Slave A in real mode. In fake mode, odom pauses briefly.
- `/emergency_stop {enable: true}` latches ESTOP. In fake mode, odom stops and health mode becomes `4`.
- `/emergency_stop {enable: false}` clears the ESTOP latch.
