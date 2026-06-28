# KRAI GUI HMI 1024x600 Notes

This milestone keeps the same ROS2 backend as the operator GUI, but adjusts the PyQt5 layout for a 7 inch 1024x600 IPS display.

Changes:
- GUI starts full-screen by default.
- Press `Esc` to leave full-screen.
- Press `F11` to toggle full-screen/windowed mode.
- Window target size is 1024x600.
- Fonts, margins, title, cards, and buttons are compacted.
- Page navigation is moved near the top so it remains visible on the HMI.
- Index page uses 4 columns x 3 rows for 12 box buttons.
- Weapon buttons use 2 columns x 2 rows.

Recommended OS setup for the HMI:
- Hide the Ubuntu dock or run the GUI full-screen.
- If using only the HMI, auto-start `ros2 launch krai_bringup full_system.launch.py` after login.
