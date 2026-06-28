# KRAI Milestone 6.4 - Operator GUI

This workspace integrates a PyQt5 operator GUI with the KRAI ROS2 stack.

## Behavior

- CP1/CP2 must be selected first.
- Box buttons 1-12 select mission IDs like `cp1_box_01` or `cp2_box_12`.
- Weapon buttons 1-4 select mission IDs like `cp1_weapon_01` or `cp2_weapon_04`.
- START runs `/run_mission` from the beginning using the selected mission ID.
- STOP calls `/cmd_stop`. It does not resume mission progress; pressing START again starts the selected mission from the beginning.
- RESET GUI + ROBOT calls `/cmd_stop`, `/reset_odom`, `/reset_heading`, and clears GUI selection.
- ESTOP ON/CLEAR call `/emergency_stop`.

## Mission files

Mission catalog is in:

```bash
~/krai_robot_ws/src/krai_config/missions/index.yaml
```

Generated match placeholders are in:

```bash
~/krai_robot_ws/src/krai_config/missions/match/
```

Edit those YAML files to replace placeholder paths with real match strategies.

## Launch

```bash
ros2 launch krai_bringup full_system.launch.py
```

or GUI only after bridge/mission are already running:

```bash
ros2 launch krai_bringup gui_bringup.launch.py
```

Install GUI dependency if needed:

```bash
sudo apt install python3-pyqt5
```
