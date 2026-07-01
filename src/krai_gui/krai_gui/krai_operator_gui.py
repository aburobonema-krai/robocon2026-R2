#!/usr/bin/env python3
"""KRAI operator GUI.

PyQt5 dashboard adapted from the Robocon GUI concept, but wired to the
KRAI ROS2 stack instead of the old JSON command topic.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from PyQt5 import QtCore, QtGui, QtWidgets

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from krai_interfaces.action import RunMission
from krai_interfaces.msg import BaseTelemetry, MissionState, PrimitiveStatus, S3Health
from krai_interfaces.srv import EmergencyStop, ListMissions, ResetHeading, ResetOdom


@dataclass
class MissionEntry:
    mission_id: str
    name: str
    category: str
    file: str
    enabled: bool
    description: str


class KraiGuiRosNode(Node):
    """ROS2 adapter for the PyQt GUI.

    The Qt event loop calls rclpy.spin_once() periodically, so callbacks run in
    the GUI thread and can update state safely.
    """

    def __init__(self) -> None:
        super().__init__('krai_gui_node')

        self.latest_health: Optional[S3Health] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_base: Optional[BaseTelemetry] = None
        self.latest_primitive: Optional[PrimitiveStatus] = None
        self.latest_mission_state: Optional[MissionState] = None
        self.latest_debug: str = ''

        self.event_log: List[str] = []
        self.missions: Dict[str, MissionEntry] = {}
        self.categories: List[str] = []
        self.active_goal_handle = None
        self.active_mission_id: Optional[str] = None
        self.last_mission_feedback: str = ''
        self.last_mission_result: str = ''

        # Manual control removed

        self.create_subscription(S3Health, '/s3/health', self._on_health, 10)
        self.create_subscription(Odometry, '/s3/odom_local', self._on_odom, 10)
        self.create_subscription(BaseTelemetry, '/base/telemetry', self._on_base, 10)
        self.create_subscription(PrimitiveStatus, '/motion/primitive_status', self._on_primitive, 10)
        self.create_subscription(MissionState, '/mission/state', self._on_mission_state, 10)
        self.create_subscription(String, '/s3_bridge/debug', self._on_debug, 10)

        # Manual publisher removed

        self.s3_ping_client = self.create_client(Trigger, '/s3_ping')
        self.cmd_stop_client = self.create_client(Trigger, '/cmd_stop')
        self.reset_odom_client = self.create_client(ResetOdom, '/reset_odom')
        self.reset_heading_client = self.create_client(ResetHeading, '/reset_heading')
        self.estop_client = self.create_client(EmergencyStop, '/emergency_stop')
        # Manual client removed
        self.mission_list_client = self.create_client(ListMissions, '/mission/list')
        self.run_mission_client = ActionClient(self, RunMission, '/run_mission')

    # ----------------------------- subscribers -----------------------------
    def _on_health(self, msg: S3Health) -> None:
        self.latest_health = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_base(self, msg: BaseTelemetry) -> None:
        self.latest_base = msg

    def _on_primitive(self, msg: PrimitiveStatus) -> None:
        self.latest_primitive = msg

    def _on_mission_state(self, msg: MissionState) -> None:
        self.latest_mission_state = msg

    def _on_debug(self, msg: String) -> None:
        self.latest_debug = msg.data

    # ------------------------------- helpers -------------------------------
    def log(self, text: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        self.event_log.append(f'[{stamp}] {text}')
        self.event_log = self.event_log[-200:]
        self.get_logger().info(text)

    def _call_service(self, client, request, label: str, done_cb=None) -> None:
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.05)
        if not client.service_is_ready():
            self.log(f'{label}: service not available')
            return
        future = client.call_async(request)

        def _done(fut):
            try:
                res = fut.result()
                msg = getattr(res, 'message', '')
                ok = getattr(res, 'success', True)
                self.log(f'{label}: {ok} {msg}')
                if done_cb:
                    done_cb(res)
            except Exception as exc:  # noqa: BLE001
                self.log(f'{label}: failed {exc}')

        future.add_done_callback(_done)

    # ----------------------------- service calls ----------------------------
    def ping(self) -> None:
        self._call_service(self.s3_ping_client, Trigger.Request(), 'PING')

    def reset_odom(self) -> None:
        self._call_service(self.reset_odom_client, ResetOdom.Request(), 'RESET_ODOM')

    def reset_heading(self) -> None:
        self._call_service(self.reset_heading_client, ResetHeading.Request(), 'RESET_HEADING')

    def stop(self) -> None:
        self._call_service(self.cmd_stop_client, Trigger.Request(), 'STOP')

    def estop(self, enable: bool) -> None:
        req = EmergencyStop.Request()
        req.enable = bool(enable)
        self._call_service(self.estop_client, req, 'ESTOP_ON' if enable else 'ESTOP_CLEAR')

    def list_missions(self, category: str = '', include_disabled: bool = False) -> None:
        req = ListMissions.Request()
        req.category = category
        req.include_disabled = include_disabled

        def _done(res):
            if not getattr(res, 'success', False):
                self.log(f'MISSION_LIST failed: {getattr(res, "message", "")}'.strip())
                return
            self.missions.clear()
            cats = set()
            for i, mission_id in enumerate(res.mission_ids):
                entry = MissionEntry(
                    mission_id=mission_id,
                    name=res.names[i] if i < len(res.names) else mission_id,
                    category=res.categories[i] if i < len(res.categories) else '',
                    file=res.files[i] if i < len(res.files) else '',
                    enabled=res.enabled[i] if i < len(res.enabled) else True,
                    description=res.descriptions[i] if i < len(res.descriptions) else '',
                )
                self.missions[mission_id] = entry
                if entry.category:
                    cats.add(entry.category)
            self.categories = sorted(cats)
            self.log(f'MISSION_LIST: loaded {len(self.missions)} mission entries')

        self._call_service(self.mission_list_client, req, 'MISSION_LIST', _done)

    def run_mission(self, mission_id: str) -> None:
        if not mission_id:
            self.log('RUN_MISSION: no mission selected')
            return
        if not self.run_mission_client.server_is_ready():
            self.run_mission_client.wait_for_server(timeout_sec=0.05)
        if not self.run_mission_client.server_is_ready():
            self.log('RUN_MISSION: action server not available')
            return

        goal = RunMission.Goal()
        goal.mission_file = mission_id
        self.active_mission_id = mission_id
        self.last_mission_feedback = ''
        self.last_mission_result = ''
        self.log(f'RUN_MISSION: sending {mission_id}')

        future = self.run_mission_client.send_goal_async(goal, feedback_callback=self._on_mission_feedback)
        future.add_done_callback(self._on_mission_goal_response)

    def _on_mission_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.last_mission_feedback = f'{fb.current_step_id} {fb.current_step_type}: {fb.detail}'

    def _on_mission_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.log(f'RUN_MISSION: goal response failed {exc}')
            return
        if not goal_handle.accepted:
            self.last_mission_result = 'Goal rejected'
            self.log('RUN_MISSION: goal rejected')
            return
        self.active_goal_handle = goal_handle
        self.log('RUN_MISSION: goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_mission_result)

    def _on_mission_result(self, future) -> None:
        try:
            result = future.result().result
            self.last_mission_result = f'{result.success}: {result.message}'
            self.log(f'RUN_MISSION result: {result.success} {result.message}')
        except Exception as exc:  # noqa: BLE001
            self.last_mission_result = f'failed: {exc}'
            self.log(f'RUN_MISSION result failed: {exc}')
        finally:
            self.active_goal_handle = None


class KraiOperatorWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node: KraiGuiRosNode) -> None:
        super().__init__()
        self.ros_node = ros_node

        self.selected_team_color: str = 'merah'
        self.selected_checkpoint: Optional[int] = 1
        self.selected_box_indices: List[int] = []
        self.selected_weapon_index: Optional[int] = None
        self.selected_path: Optional[int] = None
        self.selected_mission_id: str = ''

        self.grid_buttons: List[QtWidgets.QPushButton] = []
        self.weapon_buttons: List[QtWidgets.QPushButton] = []
        self.cp_buttons: List[QtWidgets.QPushButton] = []
        self.path_buttons: List[QtWidgets.QPushButton] = []

        self._setup_window()
        self._refresh_selection_styles()
        self._setup_timers()
        self.ros_node.list_missions()
        self._append_log('KRAI operator GUI ready. Pilih Box/Weapon, lalu START.')

    # ----------------------------- setup -----------------------------
    def _setup_window(self) -> None:
        self.setWindowTitle('KRAI Operator GUI - HMI 1024x600')
        # 7 inch IPS HMI target: 1024x600. Run full-screen so Ubuntu dock/titlebar do not steal pixels.
        self.resize(1024, 600)
        self.setMinimumSize(900, 520)
        self.setStyleSheet(self._global_styles())

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(5, 5, 5, 5)
        root.setSpacing(4)

        title = QtWidgets.QLabel('KRAI ROBOT CONTROL - ROS2 / ESP32-S3')
        title.setObjectName('TitleBar')
        title.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(title)

        nav = QtWidgets.QHBoxLayout()
        nav.setSpacing(5)
        self.btn_page_monitor = QtWidgets.QPushButton('1 CONTROL')
        self.btn_page_index = QtWidgets.QPushButton('2 INDEX')
        self.btn_page_monitor.setMinimumHeight(45)
        self.btn_page_index.setMinimumHeight(45)
        self.btn_page_monitor.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.btn_page_index.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.btn_page_monitor.clicked.connect(lambda: self._set_page(0))
        self.btn_page_index.clicked.connect(lambda: self._set_page(1))
        nav.addWidget(self.btn_page_monitor)
        nav.addWidget(self.btn_page_index)
        root.addLayout(nav)

        self.pages = QtWidgets.QStackedWidget()
        self.pages.addWidget(self._build_monitor_page())
        self.pages.addWidget(self._build_index_page())
        root.addWidget(self.pages, 1)
        self._set_page(0)

    def _setup_timers(self) -> None:
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(100)

    # ----------------------------- builders -----------------------------
    def _card(self) -> QtWidgets.QFrame:
        frame = QtWidgets.QFrame()
        frame.setObjectName('Card')
        return frame

    def _title(self, text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setObjectName('SectionTitle')
        label.setAlignment(QtCore.Qt.AlignCenter)
        return label

    def _build_monitor_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        # Kiri: 33% full safety control
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(5)
        left.addWidget(self._build_safety_card(), 1)

        # Tengah: 33% Selected Index (top) & S3 Debug (bottom stretch)
        mid = QtWidgets.QVBoxLayout()
        mid.setSpacing(5)
        mid.addWidget(self._build_selected_card(), 0)
        mid.addWidget(self._build_debug_card(), 1)

        # Kanan: 33% Robot Status (top) & Event Log (bottom stretch)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(5)
        right.addWidget(self._build_status_card(), 0)
        right.addWidget(self._build_log_card(), 1)

        layout.addLayout(left, 33)
        layout.addLayout(mid, 33)
        layout.addLayout(right, 34)
        return page

    def _build_safety_card(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('ROBOT SAFETY / BASIC CONTROL'))

        row1 = QtWidgets.QGridLayout()
        self.btn_ping = self._make_button('PING', 'neutral')
        self.btn_reset_odom = self._make_button('RESET ODOM', 'neutral')
        self.btn_reset_heading = self._make_button('RESET HEADING', 'neutral')
        self.btn_reset_all = self._make_button('RESET GUI + ROBOT', 'reset')
        self.btn_stop = self._make_button('STOP', 'stop')
        self.btn_estop_on = self._make_button('ESTOP ON', 'estop')
        self.btn_estop_clear = self._make_button('ESTOP CLEAR', 'start')
        self.btn_start = self._make_button('START SELECTED', 'start')

        self.btn_ping.clicked.connect(self.ros_node.ping)
        self.btn_reset_odom.clicked.connect(self.ros_node.reset_odom)
        self.btn_reset_heading.clicked.connect(self.ros_node.reset_heading)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_estop_on.clicked.connect(lambda: self.ros_node.estop(True))
        self.btn_estop_clear.clicked.connect(lambda: self.ros_node.estop(False))
        self.btn_reset_all.clicked.connect(self._on_reset_all)
        self.btn_start.clicked.connect(self._on_start)

        buttons = [
            self.btn_ping, self.btn_reset_odom, self.btn_reset_heading, self.btn_reset_all,
            self.btn_start, self.btn_stop, self.btn_estop_on, self.btn_estop_clear,
        ]
        for i, b in enumerate(buttons):
            row1.addWidget(b, i // 2, i % 2)
        layout.addLayout(row1, 1)
        return card

    def _build_selected_card(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('SELECTED INDEX / MISSION STATE'))
        self.selected_label = QtWidgets.QLabel('-')
        self.selected_label.setWordWrap(True)
        self.selected_label.setObjectName('InfoBox')
        
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setFrameShadow(QtWidgets.QFrame.Sunken)
        line.setStyleSheet("background-color: #213853;")
        
        self.mission_state_label = QtWidgets.QLabel('-')
        self.mission_state_label.setWordWrap(True)
        self.mission_state_label.setObjectName('InfoBox')
        
        self.mission_feedback_label = QtWidgets.QLabel('-')
        self.mission_feedback_label.setWordWrap(True)
        self.mission_feedback_label.setObjectName('InfoBox')
        
        layout.addWidget(self.selected_label)
        layout.addWidget(line)
        layout.addWidget(self.mission_state_label)
        layout.addWidget(self.mission_feedback_label)
        layout.addStretch(1)
        return card

    def _build_status_card(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('ROBOT STATUS'))
        self.health_label = QtWidgets.QLabel('-')
        self.odom_label = QtWidgets.QLabel('-')
        self.base_label = QtWidgets.QLabel('-')
        self.primitive_label = QtWidgets.QLabel('-')
        for label in [self.health_label, self.odom_label, self.base_label, self.primitive_label]:
            label.setObjectName('InfoBox')
            label.setWordWrap(True)
            layout.addWidget(label)
        layout.addStretch(1)
        return card

    def _build_log_card(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('EVENT LOG'))
        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box, 1)
        return card

    def _build_debug_card(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('S3 BRIDGE DEBUG'))
        self.debug_box = QtWidgets.QPlainTextEdit()
        self.debug_box.setReadOnly(True)
        layout.addWidget(self.debug_box, 1)
        return card

    def _build_index_page(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        # Kiri: CP selector (18%)
        layout.addWidget(self._build_cp_selector(), 18)

        # Tengah: PATH di atas + BOX 12 di bawah (52%)
        center = QtWidgets.QVBoxLayout()
        center.setSpacing(5)
        center.addWidget(self._build_path_selector(), 1)
        center.addWidget(self._build_grid_selector(), 4)
        layout.addLayout(center, 52)

        # Kanan: tombol merah+biru di atas + Weapon di bawah (30%)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(5)
        right.addWidget(self._build_rb_buttons(), 1)
        right.addWidget(self._build_weapon_selector(), 2)
        layout.addLayout(right, 30)

        return page

    def _build_grid_selector(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('TARGET BOX - 12 INDEX'))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(6)
        for number in range(1, 13):
            btn = QtWidgets.QPushButton(str(number))
            btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            btn.setCheckable(True)
            btn.setMinimumSize(60, 50)
            btn.setCursor(QtCore.Qt.PointingHandCursor)
            btn.clicked.connect(lambda _=False, n=number: self._select_target('box', n))
            row, col = divmod(number - 1, 4)
            grid.addWidget(btn, row, col)
            self.grid_buttons.append(btn)
        layout.addLayout(grid, 1)
        return card

    def _build_weapon_selector(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('WEAPON - 4 INDEX'))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(6)
        for number in range(1, 5):
            btn = QtWidgets.QPushButton(str(number))
            btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            btn.setCheckable(True)
            btn.setMinimumSize(60, 50)
            btn.clicked.connect(lambda _=False, n=number: self._select_target('weapon', n))
            row, col = divmod(number - 1, 2)
            grid.addWidget(btn, row, col)
            self.weapon_buttons.append(btn)
        layout.addLayout(grid, 1)
        return card

    def _build_cp_selector(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('CHECKPOINT'))
        self.cp1 = self._make_button('CP 1', 'neutral')
        self.cp2 = self._make_button('CP 2', 'neutral')
        self.cp1.setCheckable(True)
        self.cp2.setCheckable(True)
        self.cp1.setMinimumHeight(130)
        self.cp2.setMinimumHeight(130)
        self.cp1.clicked.connect(lambda: self._select_checkpoint(1))
        self.cp2.clicked.connect(lambda: self._select_checkpoint(2))
        self.cp_buttons = [self.cp1, self.cp2]
        layout.addWidget(self.cp1)
        layout.addWidget(self.cp2)

        info = QtWidgets.QLabel(
            'FLOW:\n'
            '1 CP dulu.\n'
            '2 Pilih Box/Weapon.\n'
            '3 START dari awal.\n'
            'STOP tidak resume.\n'
            'RESET clear pilihan.'
        )
        info.setObjectName('InfoBox')
        info.setWordWrap(True)
        layout.addWidget(info, 1)
        return card

    def _build_path_selector(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('PATH PENGAMBILAN - 8'))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(6)
        for number in range(1, 9):
            btn = QtWidgets.QPushButton(str(number))
            btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            btn.setCheckable(True)
            btn.setMinimumSize(50, 40)
            btn.setCursor(QtCore.Qt.PointingHandCursor)
            btn.clicked.connect(lambda _=False, n=number: self._select_path(n))
            grid.addWidget(btn, 0, number - 1)
            self.path_buttons.append(btn)
        layout.addLayout(grid, 1)
        return card

    def _build_rb_buttons(self) -> QtWidgets.QFrame:
        card = self._card()
        layout = QtWidgets.QVBoxLayout(card)
        layout.addWidget(self._title('AKSI'))
        
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_red = self._make_button('MERAH', 'estop')
        self.btn_blue = self._make_button('BIRU', 'blue')
        self.btn_red.setMinimumHeight(45)
        self.btn_blue.setMinimumHeight(45)
        self.btn_red.clicked.connect(self._on_red_button)
        self.btn_blue.clicked.connect(self._on_blue_button)
        
        btn_layout.addWidget(self.btn_red, 1)
        btn_layout.addWidget(self.btn_blue, 1)
        layout.addLayout(btn_layout, 1)
        
        return card

    # ----------------------------- actions -----------------------------
    def _set_page(self, index: int) -> None:
        self.pages.setCurrentIndex(index)
        self._refresh_selection_styles()

    def _select_checkpoint(self, cp: int) -> None:
        self.selected_checkpoint = cp
        self._append_log(f'Selected CP{cp}')
        self._refresh_selection_styles()
        self._resolve_selected_mission()

    def _select_target(self, target_type: str, index: int) -> None:
        if self.selected_checkpoint is None:
            self._append_log('Pilih CP1 atau CP2 dulu sebelum memilih target.')
            self._show_warning('Pilih CP dulu', 'CP1 atau CP2 harus dipilih sebelum memilih box/senjata.')
            self._refresh_selection_styles()
            return
            
        if target_type == 'box':
            if index in self.selected_box_indices:
                self.selected_box_indices.remove(index)
            else:
                self.selected_box_indices.append(index)
                if len(self.selected_box_indices) > 2:
                    self.selected_box_indices.pop(0)
        else:
            if self.selected_weapon_index == index:
                self.selected_weapon_index = None
            else:
                self.selected_weapon_index = index
            
        self._append_log(f'Selected {target_type} {index} for CP{self.selected_checkpoint}')
        self._refresh_selection_styles()
        self._resolve_selected_mission()

    def _select_path(self, path: int) -> None:
        if self.selected_path == path:
            self.selected_path = None
        else:
            self.selected_path = path
        self._append_log(f'Selected Path {path}')
        self._refresh_selection_styles()
        self._resolve_selected_mission()

    def _on_red_button(self) -> None:
        self.selected_team_color = 'merah'
        self._append_log('Team set to MERAH')
        self._refresh_selection_styles()

    def _on_blue_button(self) -> None:
        self.selected_team_color = 'biru'
        self._append_log('Team set to BIRU')
        self._refresh_selection_styles()

    def _resolve_selected_mission(self) -> None:
        if self.selected_checkpoint is None:
            self.selected_mission_id = ''
            return
            
        parts = [f'cp{self.selected_checkpoint}']
        if self.selected_box_indices:
            for b in sorted(self.selected_box_indices):
                parts.append(f'box_{b:02d}')
        if self.selected_weapon_index is not None:
            parts.append(f'weapon_{self.selected_weapon_index:02d}')
            
        if len(parts) > 1:
            self.selected_mission_id = '_'.join(parts)
        else:
            self.selected_mission_id = ''

    def _on_start(self) -> None:
        self._resolve_selected_mission()
        if not self.selected_mission_id:
            self._show_warning('Mission belum dipilih', 'Pilih CP dan target box/senjata dulu.')
            return
        if self.ros_node.missions and self.selected_mission_id not in self.ros_node.missions:
            self._show_warning(
                'Mission ID belum ada di catalog',
                f'{self.selected_mission_id} belum ada di missions/index.yaml. Tambahkan atau pilih target lain.',
            )
            self._append_log(f'START blocked, mission missing: {self.selected_mission_id}')
            return
        self._append_log(f'START {self.selected_mission_id}')
        self.ros_node.run_mission(self.selected_mission_id)

    def _on_stop(self) -> None:
        self._append_log('STOP requested. Selection stays, next START restarts mission from beginning.')
        self.ros_node.stop()

    def _on_reset_all(self) -> None:
        self._append_log('RESET GUI + ROBOT requested')
        self.ros_node.stop()
        self.ros_node.reset_odom()
        self.ros_node.reset_heading()
        self.selected_checkpoint = 1
        self.selected_box_indices = []
        self.selected_weapon_index = None
        self.selected_mission_id = ''
        self.selected_path = None
        self._refresh_selection_styles()

    # ----------------------------- refresh -----------------------------
    def _refresh_ui(self) -> None:
        self._refresh_status_labels()
        self._refresh_mission_labels()
        self._refresh_log_boxes()
        self._refresh_selection_label()

    def _refresh_status_labels(self) -> None:
        h = self.ros_node.latest_health
        if h:
            self.health_label.setText(
                f'HEALTH mode={h.mode} ready={h.heading_ready} base={h.base_connected}\n'
                f'fault=0x{h.fault_flags:04x} rx={h.nuc_rx_count} crc={h.nuc_crc_error} cmd_age={h.last_nuc_cmd_age_ms}'
            )
        else:
            self.health_label.setText('HEALTH\nNo data')

        od = self.ros_node.latest_odom
        if od:
            q = od.pose.pose.orientation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            self.odom_label.setText(
                f'ODOM x={od.pose.pose.position.x:.2f}m y={od.pose.pose.position.y:.2f}m yaw={yaw:.1f}deg\n'
                f'v={od.twist.twist.linear.x:.2f}m/s wz={od.twist.twist.angular.z:.2f}rad/s'
            )
        else:
            self.odom_label.setText('ODOM LOCAL\nNo data')

        b = self.ros_node.latest_base
        if b:
            self.base_label.setText(
                f'BASE cnt={b.count_left}/{b.count_right} d={b.delta_left}/{b.delta_right}\n'
                f'rpm10={b.rpm_left_x10}/{b.rpm_right_x10} pwm={b.pwm_left}/{b.pwm_right} mode={b.slave_a_mode} age={b.age_ms}'
            )
        else:
            self.base_label.setText('BASE TELEMETRY\nNo data')

        p = self.ros_node.latest_primitive
        if p:
            self.primitive_label.setText(
                f'PRIM id={p.primitive_id} type={p.primitive_type} state={p.primitive_state}\n'
                f'prog={p.progress:.1f} rem={p.remaining:.1f} herr={p.heading_error_deg:.1f}'
            )
        else:
            self.primitive_label.setText('PRIMITIVE\nNo data')

    def _refresh_mission_labels(self) -> None:
        ms = self.ros_node.latest_mission_state
        if ms:
            self.mission_state_label.setText(
                f'MISSION STATE\n{ms.mission_name}\nstep={ms.current_step_id} type={ms.current_step_type} state={ms.state}\n{ms.detail}'
            )
        else:
            self.mission_state_label.setText('MISSION STATE\nnone\nstep=0 type=idle state=0\nmission manager ready')
        
        fb = self.ros_node.last_mission_feedback
        res = self.ros_node.last_mission_result
        self.mission_feedback_label.setText(f'RUN FEEDBACK\nResult: {res or "-"}')

    def _refresh_log_boxes(self) -> None:
        text = '\n'.join(self.ros_node.event_log[-120:])
        if self.log_box.toPlainText() != text:
            self.log_box.setPlainText(text)
            self.log_box.verticalScrollBar().setValue(self.log_box.verticalScrollBar().maximum())
        self.debug_box.setPlainText(self.ros_node.latest_debug or '-')

    def _refresh_selection_label(self) -> None:
        exists = self.selected_mission_id in self.ros_node.missions if self.ros_node.missions else False
        entry = self.ros_node.missions.get(self.selected_mission_id) if self.selected_mission_id else None
        
        target_str = []
        if self.selected_box_indices:
            for b in sorted(self.selected_box_indices):
                target_str.append(f"box {b}")
        if self.selected_weapon_index is not None:
            target_str.append(f"weapon {self.selected_weapon_index}")
            
        self.selected_label.setText(
            f'Team: {self.selected_team_color.upper()}\n'
            f'CP: {self.selected_checkpoint or "-"}\n'
            f'Path: {self.selected_path or "-"}\n'
            f'Target: {", ".join(target_str) or "-"}\n'
            f'Mission ID: {self.selected_mission_id or "-"}\n'
            f'Catalog: {"OK" if exists else "not found / not loaded"}\n'
            f'Name: {entry.name if entry else "-"}\n'
            f'Desc: {entry.description if entry else "-"}'
        )

    def _refresh_selection_styles(self) -> None:
        active_style = 'estop' if self.selected_team_color == 'merah' else 'blue'
        
        curr = self.pages.currentIndex()
        self.btn_page_monitor.setStyleSheet(self._nav_style('active' if curr == 0 else 'neutral'))
        self.btn_page_index.setStyleSheet(self._nav_style(active_style if curr == 1 else 'neutral'))
        
        self.btn_red.setStyleSheet(self._button_style('estop' if self.selected_team_color == 'merah' else 'neutral'))
        self.btn_blue.setStyleSheet(self._button_style('blue' if self.selected_team_color == 'biru' else 'neutral'))
        
        for i, btn in enumerate(self.cp_buttons, start=1):
            btn.setChecked(self.selected_checkpoint == i)
            btn.setStyleSheet(self._button_style(active_style if self.selected_checkpoint == i else 'neutral'))
            
        for i, btn in enumerate(self.path_buttons, start=1):
            active = self.selected_path == i
            btn.setChecked(active)
            btn.setStyleSheet(self._grid_style(active_style if active else 'neutral'))
            
        for i, btn in enumerate(self.grid_buttons, start=1):
            active = i in self.selected_box_indices
            btn.setChecked(active)
            btn.setStyleSheet(self._grid_style(active_style if active else 'neutral'))
            
        for i, btn in enumerate(self.weapon_buttons, start=1):
            active = self.selected_weapon_index == i
            btn.setChecked(active)
            btn.setStyleSheet(self._weapon_style(active_style if active else 'neutral'))

    # ----------------------------- utils/styles -----------------------------
    def _append_log(self, text: str) -> None:
        self.ros_node.log(text)

    def _show_warning(self, title: str, text: str) -> None:
        QtWidgets.QMessageBox.warning(self, title, text)

    def _make_button(self, text: str, kind: str) -> QtWidgets.QPushButton:
        btn = QtWidgets.QPushButton(text)
        btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        btn.setMinimumHeight(40)
        btn.setCursor(QtCore.Qt.PointingHandCursor)
        btn.setStyleSheet(self._button_style(kind))
        return btn

    def _global_styles(self) -> str:
        return """
        QWidget { background: #07111f; color: #e8f0fb; font-family: DejaVu Sans; font-size: 13px; }
        #TitleBar { background: #0f1c2f; border: 1px solid #243a5a; border-radius: 8px; padding: 6px; font-size: 20px; font-weight: 900; }
        #Card { background: #0b1626; border: 1px solid #223752; border-radius: 9px; }
        #SectionTitle { background: #12213a; border: 1px solid #2a4365; border-radius: 7px; padding: 4px; font-size: 14px; font-weight: 900; }
        #InfoBox { background: #07111f; border: 1px solid #213853; border-radius: 7px; padding: 4px; color: #e8f0fb; font-weight: 650; font-size: 13px; }
        QPlainTextEdit { background: #040b14; border: 1px solid #1f334c; border-radius: 7px; color: #d8e4f2; padding: 4px; font-size: 11px; }
        QComboBox { background: #0e1a2c; border: 1px solid #2a4365; border-radius: 6px; padding: 4px; min-height: 28px; }
        """

    def _button_style(self, kind: str) -> str:
        colors = {
            'neutral': ('#16243a', '#2d4566'),
            'start': ('#0f7a4f', '#18a66d'),
            'stop': ('#a16207', '#f59e0b'),
            'estop': ('#991b1b', '#ef4444'),
            'reset': ('#3730a3', '#6366f1'),
            'active': ('#0e7490', '#22d3ee'),
            'blue': ('#1e40af', '#3b82f6'),
        }
        bg, border = colors.get(kind, colors['neutral'])
        return f"""
        QPushButton {{ background: {bg}; border: 1px solid {border}; border-radius: 8px; padding: 5px; color: #ffffff; font-weight: 900; font-size: 15px; }}
        QPushButton:hover {{ border: 2px solid #e5f2ff; }}
        QPushButton:pressed {{ background: #ffffff; color: #07111f; }}
        """

    def _grid_style(self, kind: str) -> str:
        return self._button_style(kind) + 'QPushButton { font-size: 26px; }'

    def _weapon_style(self, kind: str) -> str:
        return self._button_style(kind) + 'QPushButton { font-size: 30px; }'

    def _nav_style(self, kind: str) -> str:
        return self._button_style(kind)


    def keyPressEvent(self, event) -> None:
        if event.key() == QtCore.Qt.Key_Escape:
            self.showNormal()
            return
        if event.key() == QtCore.Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            return
        super().keyPressEvent(event)


class SpinTimer(QtCore.QObject):
    def __init__(self, node: KraiGuiRosNode) -> None:
        super().__init__()
        self.node = node
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)

    def spin_once(self) -> None:
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error(f'GUI spin_once error: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    node = KraiGuiRosNode()
    spin_timer = SpinTimer(node)
    window = KraiOperatorWindow(node)
    # Fullscreen is the default for 7 inch 1024x600 HMI. Press Esc for windowed mode, F11 to toggle.
    window.showFullScreen()
    try:
        exit_code = app.exec_()
    finally:
        spin_timer.timer.stop()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
