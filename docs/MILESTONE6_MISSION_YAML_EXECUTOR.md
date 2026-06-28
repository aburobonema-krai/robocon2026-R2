# KRAI Milestone 6 - Mission YAML Executor

Milestone 6 adds `krai_mission` as a real mission sequencer.

## New ROS action

```bash
/run_mission [krai_interfaces/action/RunMission]
```

The mission manager loads a YAML file from `krai_config/missions/` and executes steps in order.

Supported step types:

- `reset_odom`
- `reset_heading`
- `move_rel`
- `turn_rel`
- `wait`
- `stop`
- `mechanism` reserved for Slave B and skipped for now

## Recommended launch

```bash
ros2 launch krai_bringup full_system.launch.py
```

This launches:

- `s3_bridge_node`
- `mission_manager_node`
- GUI placeholder

## Test mission

```bash
ros2 action send_goal --feedback /run_mission krai_interfaces/action/RunMission "{mission_file: 'debug_forward_turn.yaml'}"
```

Longer test:

```bash
ros2 action send_goal --feedback /run_mission krai_interfaces/action/RunMission "{mission_file: 'debug_square.yaml'}"
```

Monitor mission state:

```bash
ros2 topic echo /mission/state
```

## Safety

The mission manager never talks to serial directly. It calls tested bridge interfaces:

- `/reset_odom`
- `/reset_heading`
- `/cmd_stop`
- `/execute_primitive`

STOP/ESTOP safety stays in `krai_s3_bridge` and ESP32-S3.
