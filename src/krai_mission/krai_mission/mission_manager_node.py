#!/usr/bin/env python3

import os
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from krai_interfaces.action import ExecutePrimitive, RunMission
from krai_interfaces.msg import MissionState
from krai_interfaces.srv import ListMissions, ResetHeading, ResetOdom


MISSION_IDLE = 0
MISSION_RUNNING = 1
MISSION_DONE = 2
MISSION_FAILED = 3
MISSION_CANCELED = 4


class MissionManagerNode(Node):
    """Milestone 6.2 mission catalog + YAML executor.

    New in 6.2:
    - missions/index.yaml catalog support
    - /mission/list service for GUI/dropdown use
    - /run_mission accepts either a mission id or a YAML filename/path
    - mission folders/categories are supported
    - speed_profile defaults can be shared across many missions

    Backward compatibility:
    - mission_file: 'debug_square.yaml' still works
    - mission_file: 'debug_square' now resolves through missions/index.yaml first
    """

    def __init__(self):
        super().__init__('mission_manager_node')

        self.cb_group = ReentrantCallbackGroup()
        self.mission_active = False
        self.current_mission_name = 'none'
        self.current_step_id = 0
        self.current_step_type = 'idle'
        self.current_detail = 'mission manager ready'
        self.current_state = MISSION_IDLE
        self.last_primitive_feedback = None

        self.config_share = Path(get_package_share_directory('krai_config'))
        self.missions_dir = self.config_share / 'missions'
        self.profiles_dir = self.config_share / 'profiles'
        self.catalog_path = self.missions_dir / 'index.yaml'
        self.catalog = self._load_catalog()
        self.speed_profiles = self._load_speed_profiles()
        self.default_mission_id = str(self.catalog.get('default_mission_id', 'debug_square'))

        self.state_pub = self.create_publisher(MissionState, '/mission/state', 10)
        self.state_timer = self.create_timer(0.2, self._publish_state, callback_group=self.cb_group)

        self.reset_odom_client = self.create_client(ResetOdom, '/reset_odom', callback_group=self.cb_group)
        self.reset_heading_client = self.create_client(ResetHeading, '/reset_heading', callback_group=self.cb_group)
        self.cmd_stop_client = self.create_client(Trigger, '/cmd_stop', callback_group=self.cb_group)
        self.primitive_client = ActionClient(self, ExecutePrimitive, '/execute_primitive', callback_group=self.cb_group)

        self.list_missions_service = self.create_service(
            ListMissions,
            '/mission/list',
            self._handle_list_missions,
            callback_group=self.cb_group,
        )

        self.run_mission_server = ActionServer(
            self,
            RunMission,
            '/run_mission',
            execute_callback=self._execute_run_mission,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('KRAI mission manager started in MILESTONE 6.2 mode')
        self.get_logger().info('Mission action enabled: /run_mission')
        self.get_logger().info('Mission catalog service enabled: /mission/list')
        self.get_logger().info(f'Mission catalog path: {self.catalog_path}')
        self.get_logger().info(f'Default mission id: {self.default_mission_id}')
        self.get_logger().info('Run mission by ID or file, e.g. mission_file: debug_square or debug/debug_square.yaml')

    # -------------------------
    # Catalog service
    # -------------------------
    def _handle_list_missions(self, request, response):
        category_filter = str(request.category).strip()
        include_disabled = bool(request.include_disabled)
        entries = self._catalog_entries()

        for entry in entries:
            enabled = bool(entry.get('enabled', True))
            category = str(entry.get('category', 'uncategorized'))
            if category_filter and category != category_filter:
                continue
            if not include_disabled and not enabled:
                continue
            response.mission_ids.append(str(entry.get('id', '')))
            response.names.append(str(entry.get('name', entry.get('id', ''))))
            response.categories.append(category)
            response.files.append(str(entry.get('file', '')))
            response.enabled.append(enabled)
            response.descriptions.append(str(entry.get('description', '')))

        response.success = True
        response.message = f'{len(response.mission_ids)} mission(s) returned'
        return response

    # -------------------------
    # Action callbacks
    # -------------------------
    def _goal_callback(self, goal_request):
        if self.mission_active:
            self.get_logger().warn('Rejecting mission goal: another mission is already active')
            return GoalResponse.REJECT

        mission_ref = goal_request.mission_file.strip() or self.default_mission_id
        mission_path, entry, reason = self._resolve_mission_reference(mission_ref)
        if mission_path is None:
            self.get_logger().warn(f'Rejecting mission goal: cannot resolve {mission_ref!r}: {reason}')
            return GoalResponse.REJECT

        mission_id = entry.get('id', mission_ref) if entry else mission_ref
        self.get_logger().info(f'Accepting mission goal: ref={mission_ref} resolved={mission_path} id={mission_id}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn('Mission cancel requested')
        return CancelResponse.ACCEPT

    def _execute_run_mission(self, goal_handle):
        self.mission_active = True
        result = RunMission.Result()

        try:
            mission_ref = goal_handle.request.mission_file.strip() or self.default_mission_id
            mission_path, catalog_entry, reason = self._resolve_mission_reference(mission_ref)
            if mission_path is None:
                msg = f'mission not found: {mission_ref} ({reason})'
                self._set_state(MISSION_FAILED, 'none', 0, 'load', msg)
                result.success = False
                result.message = msg
                goal_handle.abort()
                return result

            mission = self._load_yaml(mission_path)
            mission_name = str(mission.get('name', catalog_entry.get('id', Path(mission_path).stem) if catalog_entry else Path(mission_path).stem))
            steps = mission.get('steps', [])
            if not isinstance(steps, list) or not steps:
                msg = f'mission has no steps: {mission_path}'
                self._set_state(MISSION_FAILED, mission_name, 0, 'load', msg)
                result.success = False
                result.message = msg
                goal_handle.abort()
                return result

            profile_name, profile = self._resolve_speed_profile(mission)
            load_detail = f'loaded {len(steps)} steps from {mission_path} profile={profile_name}'
            if catalog_entry:
                load_detail += f' catalog_id={catalog_entry.get("id", "")}'

            self._set_state(MISSION_RUNNING, mission_name, 0, 'start', load_detail)
            self._publish_feedback(goal_handle, 0, 'start', f'running mission {mission_name}')
            self.get_logger().info(f'Running mission {mission_name}: {mission_path} speed_profile={profile_name}')

            if not self._wait_for_core_interfaces(goal_handle):
                msg = 'core bridge interfaces unavailable'
                self._set_state(MISSION_FAILED, mission_name, 0, 'precheck', msg)
                result.success = False
                result.message = msg
                goal_handle.abort()
                return result

            for idx, step in enumerate(steps, start=1):
                if goal_handle.is_cancel_requested:
                    self._best_effort_stop()
                    msg = f'mission canceled before step {idx}'
                    self._set_state(MISSION_CANCELED, mission_name, self._step_id(step, idx), self._step_type(step), msg)
                    result.success = False
                    result.message = msg
                    goal_handle.canceled()
                    return result

                step_id = self._step_id(step, idx)
                step_type = self._step_type(step)
                detail = f'step {idx}/{len(steps)} id={step_id} type={step_type}'
                self._set_state(MISSION_RUNNING, mission_name, step_id, step_type, detail)
                self._publish_feedback(goal_handle, step_id, step_type, detail)
                self.get_logger().info(detail)

                ok, message = self._execute_step(goal_handle, mission_name, step, step_id, step_type, profile)
                if goal_handle.is_cancel_requested:
                    self._best_effort_stop()
                    msg = f'mission canceled during step {step_id} ({step_type})'
                    self._set_state(MISSION_CANCELED, mission_name, step_id, step_type, msg)
                    result.success = False
                    result.message = msg
                    goal_handle.canceled()
                    return result

                if not ok:
                    self._best_effort_stop()
                    msg = f'mission failed at step {step_id} ({step_type}): {message}'
                    self._set_state(MISSION_FAILED, mission_name, step_id, step_type, msg)
                    result.success = False
                    result.message = msg
                    goal_handle.abort()
                    return result

            self._set_state(MISSION_DONE, mission_name, 0, 'done', 'mission complete')
            self._publish_feedback(goal_handle, 0, 'done', 'mission complete')
            result.success = True
            result.message = f'mission {mission_name} complete'
            goal_handle.succeed()
            return result

        except Exception as exc:  # keep the node alive and make failures visible
            self._best_effort_stop()
            msg = f'mission exception: {exc}'
            self.get_logger().exception(msg)
            self._set_state(MISSION_FAILED, self.current_mission_name, self.current_step_id, self.current_step_type, msg)
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result
        finally:
            self.mission_active = False
            # Keep DONE/FAILED/CANCELED visible in /mission/state instead of forcing IDLE immediately.

    # -------------------------
    # Step execution
    # -------------------------
    def _execute_step(self, goal_handle, mission_name, step, step_id, step_type, profile):
        if step_type == 'reset_odom':
            return self._call_reset_odom(goal_handle)

        if step_type == 'reset_heading':
            return self._call_reset_heading(goal_handle)

        if step_type == 'stop':
            return self._call_cmd_stop(goal_handle)

        if step_type == 'wait':
            duration = float(step.get('duration_s', step.get('seconds', step.get('time_s', 0.0))))
            if duration <= 0.0:
                duration = float(step.get('duration_ms', step.get('timeout_ms', 0))) / 1000.0
            duration = max(0.0, duration)
            return self._wait_duration(goal_handle, duration, step_id, step_type)

        if step_type == 'move_rel':
            primitive_goal = ExecutePrimitive.Goal()
            primitive_goal.primitive_id = int(step.get('primitive_id', step_id))
            primitive_goal.primitive_type = 'move_rel'
            primitive_goal.distance_cm = float(step.get('distance_cm', 0.0))
            primitive_goal.angle_deg = 0.0
            primitive_goal.max_linear_cm_s = float(step.get('max_linear_cm_s', step.get('max_speed_cm_s', profile.get('max_linear_cm_s', 20.0))))
            primitive_goal.max_angular_deg_s = 0.0
            primitive_goal.tolerance_cm = float(step.get('tolerance_cm', profile.get('move_tolerance_cm', 2.0)))
            primitive_goal.tolerance_deg = 0.0
            primitive_goal.heading_hold_enable = bool(step.get('heading_hold_enable', True))
            return self._run_primitive(goal_handle, mission_name, step_id, step_type, primitive_goal)

        if step_type == 'turn_rel':
            primitive_goal = ExecutePrimitive.Goal()
            primitive_goal.primitive_id = int(step.get('primitive_id', step_id))
            primitive_goal.primitive_type = 'turn_rel'
            primitive_goal.distance_cm = 0.0
            primitive_goal.angle_deg = float(step.get('angle_deg', 0.0))
            primitive_goal.max_linear_cm_s = 0.0
            primitive_goal.max_angular_deg_s = float(step.get('max_angular_deg_s', step.get('max_speed_deg_s', profile.get('max_angular_deg_s', 45.0))))
            primitive_goal.tolerance_cm = 0.0
            primitive_goal.tolerance_deg = float(step.get('tolerance_deg', profile.get('turn_tolerance_deg', 2.0)))
            primitive_goal.heading_hold_enable = False
            return self._run_primitive(goal_handle, mission_name, step_id, step_type, primitive_goal)

        if step_type == 'mechanism':
            # Reserved for Slave B. Skipped deliberately so mission format can include future steps now.
            target = step.get('target', 'slave_b')
            command = step.get('command', 'unknown')
            detail = f'mechanism step reserved, skipped for now: target={target} command={command}'
            self.get_logger().warn(detail)
            self._set_state(MISSION_RUNNING, mission_name, step_id, step_type, detail)
            self._publish_feedback(goal_handle, step_id, step_type, detail)
            return True, detail

        return False, f'unsupported step type: {step_type}'

    def _run_primitive(self, mission_goal_handle, mission_name, step_id, step_type, primitive_goal):
        if not self.primitive_client.wait_for_server(timeout_sec=5.0):
            return False, '/execute_primitive action server unavailable'

        self.last_primitive_feedback = None

        def feedback_cb(feedback_msg):
            self.last_primitive_feedback = feedback_msg.feedback

        # After one primitive finishes, s3_bridge/S3 may need a few fresh health/status
        # frames before accepting the next primitive. Retry short transient rejects instead
        # of failing the whole mission.
        accept_deadline = time.monotonic() + 3.0
        attempt = 0
        primitive_handle = None
        while rclpy.ok() and time.monotonic() < accept_deadline:
            if mission_goal_handle.is_cancel_requested:
                return False, 'mission canceled before primitive accepted'

            attempt += 1
            send_future = self.primitive_client.send_goal_async(primitive_goal, feedback_callback=feedback_cb)
            if not self._wait_for_future(send_future, timeout_s=1.0, mission_goal_handle=mission_goal_handle):
                detail = f'primitive send attempt {attempt} timed out, retrying'
                self._set_state(MISSION_RUNNING, mission_name, step_id, step_type, detail)
                self._publish_feedback(mission_goal_handle, step_id, step_type, detail)
                time.sleep(0.15)
                continue

            primitive_handle = send_future.result()
            if primitive_handle is not None and primitive_handle.accepted:
                if attempt > 1:
                    detail = f'primitive accepted after {attempt} attempts'
                    self._set_state(MISSION_RUNNING, mission_name, step_id, step_type, detail)
                    self._publish_feedback(mission_goal_handle, step_id, step_type, detail)
                break

            detail = f'primitive goal temporarily rejected by s3_bridge, retrying attempt={attempt}'
            self._set_state(MISSION_RUNNING, mission_name, step_id, step_type, detail)
            self._publish_feedback(mission_goal_handle, step_id, step_type, detail)
            time.sleep(0.20)

        if primitive_handle is None or not primitive_handle.accepted:
            return False, 'primitive goal rejected by s3_bridge after retry window'

        result_future = primitive_handle.get_result_async()
        last_feedback_pub = 0.0
        while rclpy.ok() and not result_future.done():
            if mission_goal_handle.is_cancel_requested:
                try:
                    primitive_handle.cancel_goal_async()
                except Exception:
                    pass
                self._best_effort_stop()
                return False, 'mission canceled while primitive running'

            now = time.monotonic()
            if now - last_feedback_pub >= 0.2:
                last_feedback_pub = now
                if self.last_primitive_feedback is not None:
                    pf = self.last_primitive_feedback
                    detail = (
                        f'primitive running progress={pf.progress:.1f} '
                        f'remaining={pf.remaining:.1f} heading_error={pf.heading_error_deg:.1f} '
                        f'state={pf.primitive_state}'
                    )
                else:
                    detail = 'primitive running, waiting for feedback'
                self._set_state(MISSION_RUNNING, mission_name, step_id, step_type, detail)
                self._publish_feedback(mission_goal_handle, step_id, step_type, detail)
            time.sleep(0.02)

        wrapper = result_future.result()
        if wrapper is None:
            return False, 'primitive result missing'
        primitive_result = wrapper.result
        if not primitive_result.success:
            return False, primitive_result.message

        # Short settle time so the next mission step sees bridge/S3 back in IDLE.
        time.sleep(0.35)
        return True, primitive_result.message

    # -------------------------
    # ROS service helpers
    # -------------------------
    def _wait_for_core_interfaces(self, goal_handle):
        ok = True
        ok = ok and self.reset_odom_client.wait_for_service(timeout_sec=5.0)
        ok = ok and self.reset_heading_client.wait_for_service(timeout_sec=5.0)
        ok = ok and self.cmd_stop_client.wait_for_service(timeout_sec=5.0)
        ok = ok and self.primitive_client.wait_for_server(timeout_sec=5.0)
        if goal_handle.is_cancel_requested:
            return False
        return ok

    def _call_reset_odom(self, goal_handle):
        req = ResetOdom.Request()
        future = self.reset_odom_client.call_async(req)
        if not self._wait_for_future(future, timeout_s=3.0, mission_goal_handle=goal_handle):
            return False, 'RESET_ODOM timeout'
        resp = future.result()
        return bool(resp.success), resp.message

    def _call_reset_heading(self, goal_handle):
        req = ResetHeading.Request()
        future = self.reset_heading_client.call_async(req)
        if not self._wait_for_future(future, timeout_s=3.0, mission_goal_handle=goal_handle):
            return False, 'RESET_HEADING timeout'
        resp = future.result()
        return bool(resp.success), resp.message

    def _call_cmd_stop(self, goal_handle):
        req = Trigger.Request()
        future = self.cmd_stop_client.call_async(req)
        if not self._wait_for_future(future, timeout_s=3.0, mission_goal_handle=goal_handle):
            return False, 'CMD_STOP timeout'
        resp = future.result()
        return bool(resp.success), resp.message

    def _best_effort_stop(self):
        try:
            if self.cmd_stop_client.service_is_ready():
                self.cmd_stop_client.call_async(Trigger.Request())
        except Exception:
            pass

    # -------------------------
    # Catalog / config helpers
    # -------------------------
    def _load_catalog(self):
        if not self.catalog_path.exists():
            self.get_logger().warn(f'Mission catalog missing: {self.catalog_path}')
            return {'missions': [], 'categories': [], 'default_mission_id': 'debug_square'}
        try:
            data = self._load_yaml(self.catalog_path)
            if not isinstance(data, dict):
                raise ValueError('index.yaml root must be a map')
            if 'missions' not in data or not isinstance(data.get('missions'), list):
                data['missions'] = []
            return data
        except Exception as exc:
            self.get_logger().error(f'Failed to load mission catalog {self.catalog_path}: {exc}')
            return {'missions': [], 'categories': [], 'default_mission_id': 'debug_square'}

    def _catalog_entries(self):
        entries = self.catalog.get('missions', [])
        return entries if isinstance(entries, list) else []

    def _find_catalog_entry(self, mission_id):
        key = str(mission_id).strip()
        for entry in self._catalog_entries():
            if str(entry.get('id', '')).strip() == key:
                return entry
        return None

    def _resolve_mission_reference(self, mission_ref):
        ref = str(mission_ref).strip() or self.default_mission_id

        # 1) Mission catalog ID has priority for short names.
        entry = self._find_catalog_entry(ref)
        if entry is not None:
            if not bool(entry.get('enabled', True)):
                return None, entry, 'catalog entry disabled'
            file_value = str(entry.get('file', '')).strip()
            if not file_value:
                return None, entry, 'catalog entry has empty file field'
            path = self._resolve_mission_file(file_value)
            if path is None:
                return None, entry, f'catalog file not found: {file_value}'
            return path, entry, 'catalog id'

        # 2) Direct path / relative file.
        path = self._resolve_mission_file(ref)
        if path is not None:
            return path, None, 'direct file'

        # 3) Convenience: strip .yaml and try catalog again.
        if ref.endswith('.yaml'):
            entry = self._find_catalog_entry(ref[:-5])
            if entry is not None:
                path, _, reason = self._resolve_mission_reference(str(entry.get('id')))
                return path, entry, reason

        return None, None, 'not found in catalog or mission files'

    def _resolve_mission_file(self, mission_file):
        candidates = []
        raw = Path(str(mission_file).strip()).expanduser()
        if raw.is_absolute():
            candidates.append(raw)
        else:
            candidates.append(Path.cwd() / raw)
            candidates.append(self.missions_dir / raw)
            if raw.suffix == '':
                candidates.append(self.missions_dir / f'{raw.name}.yaml')
            # Search one level of category folders for short filenames.
            if len(raw.parts) == 1:
                for child in self.missions_dir.iterdir() if self.missions_dir.exists() else []:
                    if child.is_dir():
                        candidates.append(child / raw)
                        if raw.suffix == '':
                            candidates.append(child / f'{raw.name}.yaml')

        for candidate in candidates:
            try:
                if candidate.exists() and candidate.is_file():
                    return str(candidate)
            except Exception:
                pass
        return None

    def _load_speed_profiles(self):
        path = self.profiles_dir / 'speed_profiles.yaml'
        if not path.exists():
            self.get_logger().warn(f'Speed profile file missing: {path}')
            return {}
        try:
            data = self._load_yaml(path)
            profiles = data.get('profiles', {}) if isinstance(data, dict) else {}
            if not isinstance(profiles, dict):
                return {}
            return profiles
        except Exception as exc:
            self.get_logger().warn(f'Failed to load speed profiles: {exc}')
            return {}

    def _resolve_speed_profile(self, mission):
        profile_name = str(mission.get('speed_profile', 'normal')).strip() or 'normal'
        profile = self.speed_profiles.get(profile_name)
        if not isinstance(profile, dict):
            self.get_logger().warn(f'Speed profile not found: {profile_name}, using built-in fallback')
            profile = {
                'max_linear_cm_s': 15.0,
                'max_angular_deg_s': 35.0,
                'move_tolerance_cm': 2.0,
                'turn_tolerance_deg': 2.5,
            }
        return profile_name, profile

    # -------------------------
    # General helpers
    # -------------------------
    def _wait_for_future(self, future, timeout_s, mission_goal_handle=None):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if mission_goal_handle is not None and mission_goal_handle.is_cancel_requested:
                return False
            if time.monotonic() - start > timeout_s:
                return False
            time.sleep(0.02)
        return future.done()

    def _wait_duration(self, goal_handle, duration_s, step_id, step_type):
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < duration_s:
            if goal_handle.is_cancel_requested:
                return False, 'mission canceled during wait'
            remaining = duration_s - (time.monotonic() - start)
            self._publish_feedback(goal_handle, step_id, step_type, f'waiting {remaining:.1f}s')
            time.sleep(min(0.2, max(0.02, remaining)))
        return True, f'waited {duration_s:.2f}s'

    @staticmethod
    def _load_yaml(path):
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data or {}

    @staticmethod
    def _step_id(step, fallback):
        try:
            return int(step.get('id', fallback))
        except Exception:
            return int(fallback)

    @staticmethod
    def _step_type(step):
        return str(step.get('type', 'unknown')).strip()

    def _set_state(self, state, mission_name, step_id, step_type, detail):
        self.current_state = int(state)
        self.current_mission_name = str(mission_name)
        self.current_step_id = int(step_id)
        self.current_step_type = str(step_type)
        self.current_detail = str(detail)
        self._publish_state()

    def _publish_state(self):
        msg = MissionState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mission_name = self.current_mission_name
        msg.current_step_id = int(self.current_step_id)
        msg.current_step_type = self.current_step_type
        msg.state = int(self.current_state)
        msg.detail = self.current_detail
        self.state_pub.publish(msg)

    def _publish_feedback(self, goal_handle, step_id, step_type, detail):
        fb = RunMission.Feedback()
        fb.current_step_id = int(step_id)
        fb.current_step_type = str(step_type)
        fb.detail = str(detail)
        goal_handle.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
