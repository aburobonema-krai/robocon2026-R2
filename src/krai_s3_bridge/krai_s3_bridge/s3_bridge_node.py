#!/usr/bin/env python3

"""KRAI ESP32-S3 serial bridge, milestone 5.5 safety responsiveness primitive executor commands.

This node parses NUC_S3_PROTOCOL_V1 frames from ESP32-S3 over USB CDC serial,
publishes ROS2 telemetry topics, and supports safe services, manual CMD_VEL, and fake/embedded primitive execution.

Milestone 5.5 keeps primitive execution and adds deterministic local STOP/ESTOP latches, clear cooldown, and serial guards.
"""

import math
import struct
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from krai_interfaces.msg import S3Health, BaseTelemetry, PrimitiveStatus
from krai_interfaces.srv import ResetOdom, ResetHeading, EmergencyStop
from krai_interfaces.action import ExecutePrimitive
from geometry_msgs.msg import Twist

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled at runtime on robot
    serial = None

    class SerialException(Exception):
        pass


SYNC1 = 0xAA
SYNC2 = 0x55
PROTOCOL_VERSION = 0x01
MAX_PAYLOAD_LEN = 512

# NUC -> ESP32-S3 command ids
MSG_PING = 0x01
MSG_CMD_STOP = 0x02
MSG_CMD_ESTOP = 0x03
MSG_RESET_ODOM = 0x04
MSG_RESET_HEADING = 0x05
MSG_CMD_VEL_MANUAL = 0x10
MSG_EXEC_MOVE_REL = 0x20
MSG_EXEC_TURN_REL = 0x21
MSG_CANCEL_PRIMITIVE = 0x22

# ESP32-S3 -> NUC message ids
MSG_PONG = 0x81
MSG_ACK = 0x82
MSG_NACK = 0x83
MSG_ODOM_LOCAL = 0x90
MSG_BASE_TELEMETRY = 0x91
MSG_S3_HEALTH = 0x92
MSG_PRIMITIVE_STATUS = 0x93
MSG_FAULT_STATUS = 0x94

# Payload formats, little-endian, packed
FMT_ODOM_LOCAL = '<ii h i h h B B'.replace(' ', '')
FMT_BASE_TELEMETRY = '<ii h h h h h h B B H'.replace(' ', '')
FMT_S3_HEALTH = '<I B B B B I I I I I H H H'.replace(' ', '')
FMT_PRIMITIVE_STATUS = '<H B B i i h H'.replace(' ', '')
FMT_ACK = '<B B B'       # request_type, request_seq, status/error_code
FMT_PONG = '<B I'        # request_seq, s3_time_ms
FMT_ESTOP_CMD = '<B'     # enable
FMT_CMD_VEL_MANUAL = '<hhBB'  # linear_cm_s_x10, angular_deg_s_x10, control_mode, flags
FMT_EXEC_MOVE_REL = '<HihhBB'  # primitive_id, distance_cm_x10, max_speed_cm_s_x10, tolerance_cm_x10, heading_hold, reserved
FMT_EXEC_TURN_REL = '<HihhH'   # primitive_id, angle_deg_x10, max_speed_deg_s_x10, tolerance_deg_x10, reserved
FMT_CANCEL_PRIMITIVE = '<H'    # primitive_id, 0 cancels current primitive

PRIM_TYPE_NONE = 0
PRIM_TYPE_MOVE_REL = 1
PRIM_TYPE_TURN_REL = 2
PRIM_STATE_IDLE = 0
PRIM_STATE_RUNNING = 1
PRIM_STATE_DONE = 2
PRIM_STATE_FAILED = 3
PRIM_STATE_CANCELED = 4


class FrameParser:
    """Byte-by-byte parser for NUC_S3_PROTOCOL_V1 frames."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 'SYNC1'
        self.header = bytearray()
        self.payload = bytearray()
        self.msg_type = 0
        self.seq = 0
        self.length = 0
        self.crc_buf = bytearray()

    def feed(self, byte_value: int):
        """Feed one byte. Return (msg_type, seq, payload) when a frame completes."""
        b = byte_value & 0xFF

        if self.state == 'SYNC1':
            if b == SYNC1:
                self.state = 'SYNC2'
            return None

        if self.state == 'SYNC2':
            if b == SYNC2:
                self.header = bytearray()
                self.state = 'HEADER'
            elif b != SYNC1:
                self.state = 'SYNC1'
            return None

        if self.state == 'HEADER':
            self.header.append(b)
            if len(self.header) < 5:
                return None

            version = self.header[0]
            self.msg_type = self.header[1]
            self.seq = self.header[2]
            self.length = self.header[3] | (self.header[4] << 8)

            if version != PROTOCOL_VERSION or self.length > MAX_PAYLOAD_LEN:
                self.reset()
                return None

            self.payload = bytearray()
            if self.length == 0:
                self.crc_buf = bytearray()
                self.state = 'CRC'
            else:
                self.state = 'PAYLOAD'
            return None

        if self.state == 'PAYLOAD':
            self.payload.append(b)
            if len(self.payload) >= self.length:
                self.crc_buf = bytearray()
                self.state = 'CRC'
            return None

        if self.state == 'CRC':
            self.crc_buf.append(b)
            if len(self.crc_buf) < 2:
                return None

            received_crc = self.crc_buf[0] | (self.crc_buf[1] << 8)
            computed_crc = crc16_ccitt_false(bytes(self.header) + bytes(self.payload))

            if received_crc == computed_crc:
                frame = (self.msg_type, self.seq, bytes(self.payload))
                self.reset()
                return frame

            self.reset()
            raise ValueError('CRC mismatch')

        self.reset()
        return None


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def now_msg(node: Node) -> Time:
    return node.get_clock().now().to_msg()


class S3BridgeNode(Node):
    """NUC <-> ESP32-S3 bridge for milestone 2."""

    def __init__(self):
        super().__init__('s3_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_timer_sec', 0.005)
        self.declare_parameter('reconnect_timer_sec', 1.0)
        self.declare_parameter('command_timeout_sec', 0.8)
        self.declare_parameter('telemetry_timeout_ms', 300)
        self.declare_parameter('odom_frame_id', 'odom_local')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('log_unknown_frames', False)
        self.declare_parameter('max_manual_linear_cm_s', 25.0)
        self.declare_parameter('max_manual_angular_deg_s', 45.0)
        self.declare_parameter('manual_command_timeout_sec', 0.35)
        self.declare_parameter('manual_zero_on_timeout', True)
        self.declare_parameter('primitive_result_timeout_sec', 30.0)
        self.declare_parameter('stop_cooldown_sec', 0.30)
        self.declare_parameter('estop_clear_cooldown_sec', 0.50)
        self.declare_parameter('estop_clear_required_health_frames', 2)

        self.health_pub = self.create_publisher(S3Health, '/s3/health', 10)
        self.base_pub = self.create_publisher(BaseTelemetry, '/base/telemetry', 10)
        self.primitive_pub = self.create_publisher(PrimitiveStatus, '/motion/primitive_status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/s3/odom_local', 10)
        self.debug_pub = self.create_publisher(String, '/s3_bridge/debug', 10)

        # Milestone 5.3: allow STOP/ESTOP service callbacks to run while
        # /execute_primitive is blocking in its action execute loop.
        self.callback_group = ReentrantCallbackGroup()

        self.ping_srv = self.create_service(Trigger, '/s3_ping', self._handle_ping_service, callback_group=self.callback_group)
        self.reset_odom_srv = self.create_service(ResetOdom, '/reset_odom', self._handle_reset_odom_service, callback_group=self.callback_group)
        self.reset_heading_srv = self.create_service(ResetHeading, '/reset_heading', self._handle_reset_heading_service, callback_group=self.callback_group)
        self.cmd_stop_srv = self.create_service(Trigger, '/cmd_stop', self._handle_cmd_stop_service, callback_group=self.callback_group)
        self.estop_srv = self.create_service(EmergencyStop, '/emergency_stop', self._handle_emergency_stop_service, callback_group=self.callback_group)
        self.manual_enable_srv = self.create_service(SetBool, '/manual_control', self._handle_manual_control_service, callback_group=self.callback_group)
        self.manual_cmd_sub = self.create_subscription(Twist, '/manual/cmd_vel', self._handle_manual_cmd_vel, 10, callback_group=self.callback_group)
        self.primitive_action_server = ActionServer(
            self,
            ExecutePrimitive,
            '/execute_primitive',
            execute_callback=self._execute_primitive_action,
            goal_callback=self._handle_primitive_goal,
            cancel_callback=self._handle_primitive_cancel,
            callback_group=self.callback_group,
        )

        self.parser = FrameParser()
        self.serial_port = None
        # Milestone 5.4: MultiThreadedExecutor can run timers, services, and action
        # callbacks in parallel. Guard all pyserial access so read/write/close
        # never happen at the same time.
        self.serial_lock = threading.RLock()
        self.tx_seq = 0
        self.tx_frame_count = 0
        self.rx_frame_count = 0
        self.rx_crc_error_count = 0
        self.rx_unknown_count = 0
        self.last_frame_time = None
        self.last_health_time = None
        self.latest_s3_mode = None
        self.latest_fault_flags = 0
        self.manual_enabled = False
        self.manual_last_cmd_monotonic = None
        self.manual_zero_sent = True
        self.manual_tx_count = 0
        self.last_manual_cmd = (0.0, 0.0)
        self.active_primitive_id = 0
        self.latest_primitive_status = None

        # Milestone 5.5.1: initialize all local safety/action state before
        # any helper reads it. The previous 5.5 package referenced
        # local_estop_latched before assignment during node startup.
        self.local_estop_latched = False
        self.stop_cooldown_until = 0.0
        self.estop_clear_cooldown_until = 0.0
        self.estop_clear_health_frames = 0
        self.last_reject_reason = 'none'
        self.last_interrupt_reason = 'none'
        self.primitive_interrupt_reason = None
        self.primitive_interrupt_monotonic = 0.0

        self.control_responses = []  # list of dicts populated by PONG/ACK/NACK handlers

        read_period = float(self.get_parameter('read_timer_sec').value)
        reconnect_period = float(self.get_parameter('reconnect_timer_sec').value)
        self.read_timer = self.create_timer(read_period, self._read_serial_once, callback_group=self.callback_group)
        self.reconnect_timer = self.create_timer(reconnect_period, self._ensure_serial_open, callback_group=self.callback_group)
        self.status_timer = self.create_timer(1.0, self._publish_bridge_status, callback_group=self.callback_group)
        self.manual_watchdog_timer = self.create_timer(0.05, self._manual_watchdog_tick, callback_group=self.callback_group)

        self._ensure_serial_open()
        self.get_logger().info('KRAI S3 bridge started in MILESTONE 5.5 mode')
        self.get_logger().info('Safe services enabled: /s3_ping, /reset_odom, /reset_heading, /cmd_stop, /emergency_stop')
        self.get_logger().info('Manual velocity command enabled through /manual_control and /manual/cmd_vel')
        self.get_logger().info('Primitive executor enabled through /execute_primitive for move_rel and turn_rel; STOP/ESTOP immediate local interrupt + cooldown active')

    def _ensure_serial_open(self):
        if serial is None:
            self.get_logger().error_once(
                'python3-serial/pyserial is not installed. Install with: sudo apt install python3-serial'
            )
            return

        with self.serial_lock:
            try:
                if self.serial_port is not None and self.serial_port.is_open:
                    return
            except (SerialException, OSError, ValueError):
                self.serial_port = None

            port = str(self.get_parameter('serial_port').value)
            baud = int(self.get_parameter('baud_rate').value)

            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=baud,
                    timeout=0,
                    write_timeout=0.2,
                )
                self.parser.reset()
                self.get_logger().info(f'Opened S3 serial port {port} @ {baud}')
            except (SerialException, OSError) as exc:
                self.serial_port = None
                self.get_logger().warn(f'Waiting for S3 serial port {port}: {exc}', throttle_duration_sec=5.0)

    def _close_serial(self):
        with self.serial_lock:
            sp = self.serial_port
            self.serial_port = None
            if sp is not None:
                try:
                    sp.close()
                except Exception:
                    pass
            self.parser.reset()

    def _read_serial_once(self):
        with self.serial_lock:
            sp = self.serial_port
            if sp is None:
                return
            try:
                if not sp.is_open:
                    self.serial_port = None
                    return
                waiting = sp.in_waiting
                if waiting <= 0:
                    return
                data = sp.read(waiting)
            except (SerialException, OSError, ValueError) as exc:
                self.get_logger().warn(f'Serial read failed, closing port: {exc}', throttle_duration_sec=2.0)
                self.serial_port = None
                try:
                    sp.close()
                except Exception:
                    pass
                self.parser.reset()
                return

        # Parse outside the serial lock so ROS publishing does not block writes.
        for byte_value in data:
            try:
                frame = self.parser.feed(byte_value)
            except ValueError:
                self.rx_crc_error_count += 1
                continue

            if frame is not None:
                msg_type, seq, payload = frame
                self.rx_frame_count += 1
                self.last_frame_time = self.get_clock().now()
                self._handle_frame(msg_type, seq, payload)

    def _handle_frame(self, msg_type: int, seq: int, payload: bytes):
        try:
            if msg_type == MSG_S3_HEALTH:
                self._handle_s3_health(payload)
            elif msg_type == MSG_ODOM_LOCAL:
                self._handle_odom_local(payload)
            elif msg_type == MSG_BASE_TELEMETRY:
                self._handle_base_telemetry(payload)
            elif msg_type == MSG_PRIMITIVE_STATUS:
                self._handle_primitive_status(payload)
            elif msg_type == MSG_PONG:
                self._handle_pong(seq, payload)
            elif msg_type == MSG_ACK:
                self._handle_ack(seq, payload, ack=True)
            elif msg_type == MSG_NACK:
                self._handle_ack(seq, payload, ack=False)
            elif msg_type == MSG_FAULT_STATUS:
                self._publish_debug(f'rx FAULT_STATUS seq={seq} len={len(payload)}')
            else:
                self.rx_unknown_count += 1
                if bool(self.get_parameter('log_unknown_frames').value):
                    self.get_logger().warn(f'Unknown S3 frame type=0x{msg_type:02X} seq={seq} len={len(payload)}')
        except struct.error as exc:
            self.get_logger().warn(f'Bad payload for type=0x{msg_type:02X}: {exc}')

    def _send_frame(self, msg_type: int, payload: bytes = b'') -> Optional[int]:
        # Guard write/close/open because services and action callbacks run in parallel.
        with self.serial_lock:
            if self.serial_port is None or not self.serial_port.is_open:
                # Avoid recursive lock deadlock; RLock makes this safe.
                self._ensure_serial_open()
            if self.serial_port is None or not self.serial_port.is_open:
                return None

            seq = self.tx_seq & 0xFF
            self.tx_seq = (self.tx_seq + 1) & 0xFF
            header = bytes([
                PROTOCOL_VERSION,
                msg_type & 0xFF,
                seq,
                len(payload) & 0xFF,
                (len(payload) >> 8) & 0xFF,
            ])
            crc = crc16_ccitt_false(header + payload)
            frame = bytes([SYNC1, SYNC2]) + header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

            try:
                self.serial_port.write(frame)
                self.tx_frame_count += 1
                return seq
            except (SerialException, OSError, ValueError) as exc:
                self.get_logger().warn(f'Serial write failed, closing port: {exc}')
                self._close_serial()
                return None


    def _send_command_best_effort(self, msg_type: int, payload: bytes = b'', repeat: int = 3, gap_sec: float = 0.01) -> int:
        """Send a command without waiting for ACK. Used for safety-critical STOP/ESTOP.

        Returning immediately makes the ROS2 side deterministic. ACK/NACK frames are
        still parsed asynchronously by the read timer for diagnostics.
        """
        sent = 0
        for _ in range(max(1, int(repeat))):
            if self._send_frame(msg_type, payload=payload) is not None:
                sent += 1
            if gap_sec > 0:
                time.sleep(gap_sec)
        return sent

    def _set_local_stop_latch(self, reason: str = 'CMD_STOP/SAFE_STOP'):
        now = time.monotonic()
        self.primitive_interrupt_reason = reason
        self.primitive_interrupt_monotonic = now
        self.last_interrupt_reason = reason
        cooldown = float(self.get_parameter('stop_cooldown_sec').value)
        self.stop_cooldown_until = max(self.stop_cooldown_until, now + cooldown)
        self.manual_enabled = False
        self.manual_zero_sent = True
        self.latest_s3_mode = 3
        self.latest_fault_flags |= (1 << 5)
        if self.latest_primitive_status is not None:
            self.latest_primitive_status['primitive_state'] = PRIM_STATE_CANCELED

    def _set_local_estop_latch(self):
        now = time.monotonic()
        self.local_estop_latched = True
        self.primitive_interrupt_reason = 'ESTOP'
        self.primitive_interrupt_monotonic = now
        self.last_interrupt_reason = 'ESTOP'
        self.manual_enabled = False
        self.manual_zero_sent = True
        self.latest_s3_mode = 4
        self.latest_fault_flags |= (1 << 4)
        if self.latest_primitive_status is not None:
            self.latest_primitive_status['primitive_state'] = PRIM_STATE_CANCELED

    def _begin_estop_clear(self):
        self.local_estop_latched = False
        self.primitive_interrupt_reason = None
        self.last_interrupt_reason = 'ESTOP_CLEAR'
        self.estop_clear_health_frames = 0
        self.estop_clear_cooldown_until = time.monotonic() + float(self.get_parameter('estop_clear_cooldown_sec').value)
        # Do not instantly claim the S3 is healthy; wait for health frames in _handle_s3_health.
        self.latest_fault_flags &= ~(1 << 4)

    def _safety_ready(self) -> bool:
        now = time.monotonic()
        if self.local_estop_latched:
            self.last_reject_reason = 'local ESTOP latch active'
            return False
        if now < self.stop_cooldown_until:
            self.last_reject_reason = f'stop cooldown active {self.stop_cooldown_until - now:.2f}s'
            return False
        if now < self.estop_clear_cooldown_until:
            self.last_reject_reason = f'waiting after ESTOP clear {self.estop_clear_cooldown_until - now:.2f}s'
            return False
        if self.estop_clear_health_frames > 0:
            required = int(self.get_parameter('estop_clear_required_health_frames').value)
            if self.estop_clear_health_frames < required:
                self.last_reject_reason = f'waiting for healthy idle frames {self.estop_clear_health_frames}/{required}'
                return False
        if self.latest_s3_mode in (3, 4):
            self.last_reject_reason = f'S3 safety mode {self.latest_s3_mode}'
            return False
        if self.latest_fault_flags & ((1 << 4) | (1 << 5)):
            self.last_reject_reason = f'S3 safety fault flags 0x{self.latest_fault_flags:04X}'
            return False
        self.last_reject_reason = 'none'
        return True

    def _send_safe_command_and_wait(self, msg_type: int, payload: bytes = b'', timeout_sec: Optional[float] = None):
        """Send safe command and wait for matching ACK/NACK/PONG.

        Returns tuple: (success: bool, message: str)
        """
        if timeout_sec is None:
            timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        before = time.monotonic()
        seq = self._send_frame(msg_type, payload)
        if seq is None:
            return False, 'serial port is not open'

        deadline = before + timeout_sec
        command_name = {
            MSG_PING: 'PING',
            MSG_CMD_STOP: 'CMD_STOP',
            MSG_CMD_ESTOP: 'CMD_ESTOP',
            MSG_RESET_ODOM: 'RESET_ODOM',
            MSG_RESET_HEADING: 'RESET_HEADING',
            MSG_CMD_VEL_MANUAL: 'CMD_VEL_MANUAL',
            MSG_EXEC_MOVE_REL: 'EXEC_MOVE_REL',
            MSG_EXEC_TURN_REL: 'EXEC_TURN_REL',
            MSG_CANCEL_PRIMITIVE: 'CANCEL_PRIMITIVE',
        }.get(msg_type, f'0x{msg_type:02X}')

        while time.monotonic() < deadline:
            self._read_serial_once()

            # Keep list small and ignore responses created before this command was sent.
            recent = []
            for item in self.control_responses:
                if item['created_monotonic'] < before:
                    continue
                recent.append(item)
                if item['request_seq'] != seq:
                    continue

                if msg_type == MSG_PING and item['type'] == MSG_PONG:
                    return True, f'PONG from S3, s3_time_ms={item.get("s3_time_ms", 0)}'

                if item['type'] == MSG_ACK and item.get('request_type') == msg_type:
                    return True, f'{command_name} ACK from S3'

                if item['type'] == MSG_NACK and item.get('request_type') == msg_type:
                    code = item.get('code', 0)
                    return False, f'{command_name} NACK from S3, code={code}'

            self.control_responses = recent[-20:]
            time.sleep(0.005)

        return False, f'{command_name} timed out waiting for S3 response seq={seq}'

    def _handle_ping_service(self, request, response):
        ok, message = self._send_safe_command_and_wait(MSG_PING)
        response.success = bool(ok)
        response.message = message
        return response

    def _handle_reset_odom_service(self, request, response):
        ok, message = self._send_safe_command_and_wait(MSG_RESET_ODOM)
        response.success = bool(ok)
        response.message = message
        return response

    def _handle_reset_heading_service(self, request, response):
        ok, message = self._send_safe_command_and_wait(MSG_RESET_HEADING)
        response.success = bool(ok)
        response.message = message
        return response

    def _handle_cmd_stop_service(self, request, response):
        # Safety response is local-first. Do not wait for ACK before interrupting action.
        self._set_local_stop_latch('CMD_STOP/SAFE_STOP')
        sent_stop = self._send_command_best_effort(MSG_CMD_STOP, repeat=3, gap_sec=0.005)
        payload_cancel = struct.pack(FMT_CANCEL_PRIMITIVE, 0)
        sent_cancel = self._send_command_best_effort(MSG_CANCEL_PRIMITIVE, payload=payload_cancel, repeat=2, gap_sec=0.005)
        response.success = sent_stop > 0
        response.message = f'CMD_STOP local latch active; sent_stop={sent_stop} sent_cancel={sent_cancel}'
        return response

    def _handle_emergency_stop_service(self, request, response):
        payload = struct.pack(FMT_ESTOP_CMD, 1 if request.enable else 0)
        if request.enable:
            self._set_local_estop_latch()
            sent_estop = self._send_command_best_effort(MSG_CMD_ESTOP, payload=payload, repeat=3, gap_sec=0.005)
            payload_cancel = struct.pack(FMT_CANCEL_PRIMITIVE, 0)
            sent_cancel = self._send_command_best_effort(MSG_CANCEL_PRIMITIVE, payload=payload_cancel, repeat=2, gap_sec=0.005)
            response.success = sent_estop > 0
            response.message = f'ESTOP local latch active; sent_estop={sent_estop} sent_cancel={sent_cancel}'
            return response

        self._begin_estop_clear()
        sent_clear = self._send_command_best_effort(MSG_CMD_ESTOP, payload=payload, repeat=3, gap_sec=0.005)
        response.success = sent_clear > 0
        response.message = f'ESTOP clear sent; waiting for healthy idle state; sent_clear={sent_clear}'
        return response


    def _handle_manual_control_service(self, request, response):
        self.manual_enabled = bool(request.data)
        self.manual_last_cmd_monotonic = None
        self.manual_zero_sent = False
        if not self.manual_enabled:
            self._send_manual_velocity(0.0, 0.0, flags=0x01)
            self.manual_zero_sent = True
            response.success = True
            response.message = 'manual control disabled; zero velocity sent to S3'
            return response

        if not self._manual_safety_ok():
            self.manual_enabled = False
            response.success = False
            response.message = 'manual control rejected: S3 is not in a safe state'
            return response

        response.success = True
        response.message = 'manual control enabled; publish geometry_msgs/Twist to /manual/cmd_vel'
        return response

    def _manual_safety_ok(self) -> bool:
        return self._safety_ready()

    def _primitive_busy(self) -> bool:
        status = self.latest_primitive_status
        if status is not None and int(status.get('primitive_state', PRIM_STATE_IDLE)) == PRIM_STATE_RUNNING:
            return True
        # S3_MODE_PRIMITIVE = 2. If health says primitive is running but status has not arrived yet, treat as busy.
        if self.latest_s3_mode == 2:
            return True
        return False

    def _clamp_manual_cmd(self, linear_cm_s: float, angular_deg_s: float):
        max_linear = float(self.get_parameter('max_manual_linear_cm_s').value)
        max_angular = float(self.get_parameter('max_manual_angular_deg_s').value)
        linear_cm_s = max(-max_linear, min(max_linear, linear_cm_s))
        angular_deg_s = max(-max_angular, min(max_angular, angular_deg_s))
        return linear_cm_s, angular_deg_s

    def _send_manual_velocity(self, linear_cm_s: float, angular_deg_s: float, flags: int = 0) -> bool:
        linear_cm_s, angular_deg_s = self._clamp_manual_cmd(linear_cm_s, angular_deg_s)
        lin_x10 = int(round(linear_cm_s * 10.0))
        ang_x10 = int(round(angular_deg_s * 10.0))
        lin_x10 = max(-32768, min(32767, lin_x10))
        ang_x10 = max(-32768, min(32767, ang_x10))
        payload = struct.pack(FMT_CMD_VEL_MANUAL, lin_x10, ang_x10, 1, flags & 0xFF)
        seq = self._send_frame(MSG_CMD_VEL_MANUAL, payload=payload)
        if seq is None:
            return False
        self.manual_tx_count += 1
        self.last_manual_cmd = (linear_cm_s, angular_deg_s)
        return True

    def _handle_manual_cmd_vel(self, msg: Twist):
        if not self.manual_enabled:
            return
        if not self._manual_safety_ok():
            self.manual_enabled = False
            self._send_manual_velocity(0.0, 0.0, flags=0x02)
            self._publish_debug('manual disabled by safety gate; zero velocity sent')
            return

        linear_cm_s = float(msg.linear.x) * 100.0
        angular_deg_s = float(msg.angular.z) * 180.0 / math.pi
        ok = self._send_manual_velocity(linear_cm_s, angular_deg_s)
        if ok:
            self.manual_last_cmd_monotonic = time.monotonic()
            self.manual_zero_sent = False

    def _manual_watchdog_tick(self):
        if not self.manual_enabled:
            return
        if self.manual_last_cmd_monotonic is None:
            return
        timeout_sec = float(self.get_parameter('manual_command_timeout_sec').value)
        age = time.monotonic() - self.manual_last_cmd_monotonic
        if age > timeout_sec and bool(self.get_parameter('manual_zero_on_timeout').value):
            if not self.manual_zero_sent:
                self._send_manual_velocity(0.0, 0.0, flags=0x04)
                self.manual_zero_sent = True
                self._publish_debug(f'manual cmd timeout {age:.3f}s; zero velocity sent')


    def _handle_primitive_goal(self, goal_request):
        prim_type = self._primitive_type_from_string(goal_request.primitive_type)
        if prim_type not in (PRIM_TYPE_MOVE_REL, PRIM_TYPE_TURN_REL):
            self.get_logger().warn(f'Rejecting primitive goal with unsupported type: {goal_request.primitive_type}')
            return GoalResponse.REJECT
        if not self._manual_safety_ok():
            self.get_logger().warn(f'Rejecting primitive goal: {self.last_reject_reason}')
            return GoalResponse.REJECT
        if self._primitive_busy():
            self.get_logger().warn('Rejecting primitive goal because another primitive is still running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_primitive_cancel(self, goal_handle):
        primitive_id = int(goal_handle.request.primitive_id) & 0xFFFF
        payload = struct.pack(FMT_CANCEL_PRIMITIVE, primitive_id)
        self._send_safe_command_and_wait(MSG_CANCEL_PRIMITIVE, payload=payload, timeout_sec=0.5)
        return CancelResponse.ACCEPT

    def _primitive_type_from_string(self, text: str) -> int:
        normalized = (text or '').strip().lower()
        if normalized in ('move', 'move_rel', 'move_relative', 'forward', '1'):
            return PRIM_TYPE_MOVE_REL
        if normalized in ('turn', 'turn_rel', 'turn_relative', 'rotate', 'rotate_rel', '2'):
            return PRIM_TYPE_TURN_REL
        return PRIM_TYPE_NONE

    def _execute_primitive_action(self, goal_handle):
        goal = goal_handle.request
        result = ExecutePrimitive.Result()
        feedback = ExecutePrimitive.Feedback()

        primitive_id = int(goal.primitive_id) & 0xFFFF
        if primitive_id == 0:
            primitive_id = int(time.monotonic() * 1000.0) & 0xFFFF

        prim_type = self._primitive_type_from_string(goal.primitive_type)
        self.active_primitive_id = primitive_id
        self.latest_primitive_status = None
        if not self.local_estop_latched:
            self.primitive_interrupt_reason = None

        if prim_type == PRIM_TYPE_MOVE_REL:
            distance_x10 = int(round(float(goal.distance_cm) * 10.0))
            max_speed_x10 = int(round(float(goal.max_linear_cm_s or 20.0) * 10.0))
            tolerance_x10 = int(round(float(goal.tolerance_cm or 2.0) * 10.0))
            payload = struct.pack(
                FMT_EXEC_MOVE_REL,
                primitive_id,
                max(-2147483648, min(2147483647, distance_x10)),
                max(-32768, min(32767, max_speed_x10)),
                max(-32768, min(32767, tolerance_x10)),
                1 if bool(goal.heading_hold_enable) else 0,
                0,
            )
            ok, message = self._send_safe_command_and_wait(MSG_EXEC_MOVE_REL, payload=payload)
        elif prim_type == PRIM_TYPE_TURN_REL:
            angle_x10 = int(round(float(goal.angle_deg) * 10.0))
            max_speed_x10 = int(round(float(goal.max_angular_deg_s or 45.0) * 10.0))
            tolerance_x10 = int(round(float(goal.tolerance_deg or 2.0) * 10.0))
            payload = struct.pack(
                FMT_EXEC_TURN_REL,
                primitive_id,
                max(-2147483648, min(2147483647, angle_x10)),
                max(-32768, min(32767, max_speed_x10)),
                max(-32768, min(32767, tolerance_x10)),
                0,
            )
            ok, message = self._send_safe_command_and_wait(MSG_EXEC_TURN_REL, payload=payload)
        else:
            goal_handle.abort()
            result.success = False
            result.message = f'unsupported primitive type: {goal.primitive_type}'
            return result

        if not ok:
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        deadline = time.monotonic() + float(self.get_parameter('primitive_result_timeout_sec').value)
        final_state = PRIM_STATE_FAILED
        final_message = 'primitive timed out waiting for DONE/FAILED/CANCELED'

        while time.monotonic() < deadline:
            # Serial frames are read by the read timer in a parallel executor thread.
            # Do not read serial directly here, otherwise action/service callbacks can
            # race with the timer and close the same fd concurrently.

            # Local interrupt latch: service callbacks set this before any ACK wait,
            # so the action exits quickly even if S3 telemetry/status has not updated yet.
            if self.local_estop_latched or self.primitive_interrupt_reason == 'ESTOP':
                self.primitive_interrupt_reason = None
                goal_handle.abort()
                result.success = False
                result.message = 'primitive interrupted by ESTOP'
                return result

            if self.primitive_interrupt_reason:
                reason = self.primitive_interrupt_reason
                self.primitive_interrupt_reason = None
                goal_handle.canceled()
                result.success = False
                result.message = f'primitive canceled by {reason}'
                return result

            if time.monotonic() < self.stop_cooldown_until:
                goal_handle.canceled()
                result.success = False
                result.message = 'primitive canceled by CMD_STOP/SAFE_STOP cooldown'
                return result

            # Immediate safety interruption from S3 health/fault mirrors.
            if self.latest_s3_mode == 4 or (self.latest_fault_flags & (1 << 4)):
                goal_handle.abort()
                result.success = False
                result.message = 'primitive interrupted by ESTOP'
                return result

            if self.latest_s3_mode == 3 or (self.latest_fault_flags & (1 << 5)):
                goal_handle.canceled()
                result.success = False
                result.message = 'primitive canceled by CMD_STOP/SAFE_STOP'
                return result

            if goal_handle.is_cancel_requested:
                payload = struct.pack(FMT_CANCEL_PRIMITIVE, primitive_id)
                self._send_safe_command_and_wait(MSG_CANCEL_PRIMITIVE, payload=payload, timeout_sec=0.5)
                goal_handle.canceled()
                result.success = False
                result.message = 'primitive canceled by ROS2 client'
                return result

            status = self.latest_primitive_status
            if status is not None and status.get('primitive_id') == primitive_id:
                progress_x10 = int(status.get('progress_x10', 0))
                remaining_x10 = int(status.get('remaining_x10', 0))
                heading_err_x10 = int(status.get('heading_error_deg_x10', 0))
                state = int(status.get('primitive_state', PRIM_STATE_IDLE))

                feedback.progress = progress_x10 / 10.0
                feedback.remaining = remaining_x10 / 10.0
                feedback.heading_error_deg = heading_err_x10 / 10.0
                feedback.primitive_state = state
                goal_handle.publish_feedback(feedback)

                if state in (PRIM_STATE_DONE, PRIM_STATE_FAILED, PRIM_STATE_CANCELED):
                    final_state = state
                    if state == PRIM_STATE_DONE:
                        final_message = 'primitive DONE from S3'
                        goal_handle.succeed()
                        result.success = True
                        result.message = final_message
                        return result
                    if state == PRIM_STATE_CANCELED:
                        final_message = 'primitive CANCELED by S3'
                        goal_handle.canceled()
                        result.success = False
                        result.message = final_message
                        return result
                    final_message = 'primitive FAILED from S3'
                    goal_handle.abort()
                    result.success = False
                    result.message = final_message
                    return result

            time.sleep(0.02)

        goal_handle.abort()
        result.success = False
        result.message = final_message
        return result

    def _handle_pong(self, seq: int, payload: bytes):
        if len(payload) != struct.calcsize(FMT_PONG):
            self.get_logger().warn(f'PONG length mismatch: got={len(payload)} expected={struct.calcsize(FMT_PONG)}')
            return
        request_seq, s3_time_ms = struct.unpack(FMT_PONG, payload)
        self.control_responses.append({
            'type': MSG_PONG,
            'frame_seq': seq,
            'request_seq': request_seq,
            's3_time_ms': s3_time_ms,
            'created_monotonic': time.monotonic(),
        })
        self._publish_debug(f'rx PONG request_seq={request_seq} s3_time_ms={s3_time_ms}')

    def _handle_ack(self, seq: int, payload: bytes, ack: bool):
        if len(payload) != struct.calcsize(FMT_ACK):
            name = 'ACK' if ack else 'NACK'
            self.get_logger().warn(f'{name} length mismatch: got={len(payload)} expected={struct.calcsize(FMT_ACK)}')
            return
        request_type, request_seq, code = struct.unpack(FMT_ACK, payload)
        response_type = MSG_ACK if ack else MSG_NACK
        self.control_responses.append({
            'type': response_type,
            'frame_seq': seq,
            'request_type': request_type,
            'request_seq': request_seq,
            'code': code,
            'created_monotonic': time.monotonic(),
        })
        self._publish_debug(
            f'rx {"ACK" if ack else "NACK"} request_type=0x{request_type:02X} '
            f'request_seq={request_seq} code={code}'
        )

    def _handle_s3_health(self, payload: bytes):
        expected = struct.calcsize(FMT_S3_HEALTH)
        if len(payload) != expected:
            self.get_logger().warn(f'S3_HEALTH length mismatch: got={len(payload)} expected={expected}')
            return

        values = struct.unpack(FMT_S3_HEALTH, payload)
        msg = S3Health()
        msg.stamp = now_msg(self)
        (
            msg.s3_time_ms,
            msg.mode,
            msg.heading_ready,
            msg.base_connected,
            msg.primitive_state,
            msg.bad_bno_frames,
            msg.yaw_jump_reject_count,
            msg.base_crc_error,
            msg.nuc_rx_count,
            msg.nuc_crc_error,
            msg.last_nuc_cmd_age_ms,
            msg.last_base_age_ms,
            msg.fault_flags,
        ) = values
        self.last_health_time = self.get_clock().now()
        self.latest_s3_mode = int(msg.mode)
        self.latest_fault_flags = int(msg.fault_flags)
        # Clear local stop latch as soon as S3 reports it is idle/fault-free after the cooldown.
        if self.latest_s3_mode == 0 and (self.latest_fault_flags & ((1 << 4) | (1 << 5))) == 0:
            if time.monotonic() >= self.stop_cooldown_until:
                self.stop_cooldown_until = 0.0
            if time.monotonic() >= self.estop_clear_cooldown_until:
                required = int(self.get_parameter('estop_clear_required_health_frames').value)
                if self.estop_clear_health_frames < required:
                    self.estop_clear_health_frames += 1
        else:
            # If S3 is not clean/idle yet, require fresh healthy frames again.
            if self.estop_clear_health_frames > 0:
                self.estop_clear_health_frames = 0
        self.health_pub.publish(msg)

    def _handle_odom_local(self, payload: bytes):
        expected = struct.calcsize(FMT_ODOM_LOCAL)
        if len(payload) != expected:
            self.get_logger().warn(f'ODOM_LOCAL length mismatch: got={len(payload)} expected={expected}')
            return

        (
            x_cm_x10,
            y_cm_x10,
            heading_deg_x10,
            dist_cm_x10,
            linear_cm_s_x10,
            yaw_raw_deg_x10,
            heading_ready,
            _reserved,
        ) = struct.unpack(FMT_ODOM_LOCAL, payload)

        msg = Odometry()
        msg.header.stamp = now_msg(self)
        msg.header.frame_id = str(self.get_parameter('odom_frame_id').value)
        msg.child_frame_id = str(self.get_parameter('base_frame_id').value)

        x_m = (x_cm_x10 / 10.0) / 100.0
        y_m = (y_cm_x10 / 10.0) / 100.0
        heading_rad = (heading_deg_x10 / 10.0) * math.pi / 180.0
        linear_m_s = (linear_cm_s_x10 / 10.0) / 100.0

        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(heading_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(heading_rad / 2.0)
        msg.twist.twist.linear.x = linear_m_s
        msg.twist.twist.angular.z = 0.0

        # Keep covariance conservative because this is local odometry, not global localization.
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = 0.10
        msg.twist.covariance[35] = 0.20

        self.odom_pub.publish(msg)

        # Lightweight debug breadcrumb for human monitoring.
        self._last_odom_debug = (
            f'odom x={x_m:.3f}m y={y_m:.3f}m heading={heading_deg_x10 / 10.0:.1f}deg '
            f'dist={dist_cm_x10 / 10.0:.1f}cm v={linear_cm_s_x10 / 10.0:.1f}cm/s '
            f'yaw_raw={yaw_raw_deg_x10 / 10.0:.1f}deg ready={heading_ready}'
        )

    def _handle_base_telemetry(self, payload: bytes):
        expected = struct.calcsize(FMT_BASE_TELEMETRY)
        if len(payload) != expected:
            self.get_logger().warn(f'BASE_TELEMETRY length mismatch: got={len(payload)} expected={expected}')
            return

        values = struct.unpack(FMT_BASE_TELEMETRY, payload)
        msg = BaseTelemetry()
        msg.stamp = now_msg(self)
        (
            msg.count_left,
            msg.count_right,
            msg.delta_left,
            msg.delta_right,
            msg.rpm_left_x10,
            msg.rpm_right_x10,
            msg.pwm_left,
            msg.pwm_right,
            msg.slave_a_mode,
            msg.slave_a_fault_flags,
            msg.age_ms,
        ) = values
        self.base_pub.publish(msg)

    def _handle_primitive_status(self, payload: bytes):
        expected = struct.calcsize(FMT_PRIMITIVE_STATUS)
        if len(payload) != expected:
            self.get_logger().warn(f'PRIMITIVE_STATUS length mismatch: got={len(payload)} expected={expected}')
            return

        values = struct.unpack(FMT_PRIMITIVE_STATUS, payload)
        msg = PrimitiveStatus()
        msg.stamp = now_msg(self)
        (
            msg.primitive_id,
            msg.primitive_type,
            msg.primitive_state,
            msg.progress_x10,
            msg.remaining_x10,
            msg.heading_error_deg_x10,
            msg.fault_flags,
        ) = values
        self.latest_primitive_status = {
            'primitive_id': int(msg.primitive_id),
            'primitive_type': int(msg.primitive_type),
            'primitive_state': int(msg.primitive_state),
            'progress_x10': int(msg.progress_x10),
            'remaining_x10': int(msg.remaining_x10),
            'heading_error_deg_x10': int(msg.heading_error_deg_x10),
            'fault_flags': int(msg.fault_flags),
            'updated_monotonic': time.monotonic(),
        }
        self.primitive_pub.publish(msg)

    def _publish_bridge_status(self):
        port = str(self.get_parameter('serial_port').value)
        connected = self.serial_port is not None and self.serial_port.is_open
        last_age_ms = -1
        if self.last_frame_time is not None:
            dt = self.get_clock().now() - self.last_frame_time
            last_age_ms = int(dt.nanoseconds / 1_000_000)

        text = (
            f's3_bridge milestone5_5 | port={port} connected={connected} '
            f'rx_frames={self.rx_frame_count} tx_frames={self.tx_frame_count} '
            f'crc_errors={self.rx_crc_error_count} unknown={self.rx_unknown_count} '
            f'last_frame_age_ms={last_age_ms} manual_enabled={self.manual_enabled} '
            f'manual_tx={self.manual_tx_count} last_manual=({self.last_manual_cmd[0]:.1f}cm/s,{self.last_manual_cmd[1]:.1f}deg/s) '
            f'safety_ready={self._safety_ready()} mode={self.latest_s3_mode} faults=0x{self.latest_fault_flags:04X} '
            f'local_estop={self.local_estop_latched} stop_cd={max(0.0, self.stop_cooldown_until - time.monotonic()):.2f}s '
            f'clear_frames={self.estop_clear_health_frames} reject={self.last_reject_reason} interrupt={self.last_interrupt_reason}'
        )
        if hasattr(self, '_last_odom_debug'):
            text += ' | ' + self._last_odom_debug
        self._publish_debug(text)

    def _publish_debug(self, text: str):
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = S3BridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        try:
            node._close_serial()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
