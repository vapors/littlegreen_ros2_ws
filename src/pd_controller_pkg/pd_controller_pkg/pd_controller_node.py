#!/usr/bin/env python3
"""Safety envelope, reference shaper, and outer-loop controller for LGH.

Canonical preserved interface::

    /desired_position or /desired_joint_position   Float64MultiArray[12], rad
        -> pd_controller_node
    /servo_target_radians                          Float64MultiArray[12], rad
        -> lgh_st3215_driver

Controller modes
----------------
``safety_only``
    Preserves the previous behavior: sanitize, clip, low-pass, velocity-limit,
    acceleration-limit, and publish a safe position target.

``outer_pd``
    Adds real measured-position and measured-velocity feedback.  A bounded
    joint-velocity command is computed from position and velocity error and
    integrated into the next ST3215 position target.

``outer_pid``
    Same as ``outer_pd`` with optional integral action and anti-windup.

The ST3215 remains a position servo.  This node does not treat the hardware as
if it were a direct torque actuator.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
from std_srvs.srv import Trigger

from pd_controller_pkg.core import OuterLoopPositionController, PDController

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None


class PDControllerNode(Node):
    """Safety-preserving position-command controller for the native servo path."""

    VALID_MODES = ('safety_only', 'outer_pd', 'outer_pid')

    def __init__(self) -> None:
        super().__init__('pd_controller_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('joint_map_path', '')
        self.declare_parameter('controller_mode', 'safety_only')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('feedback_timeout_sec', 0.15)
        self.declare_parameter('require_feedback_for_outer_loop', True)
        self.declare_parameter('initialize_output_from_feedback', True)
        self.declare_parameter('startup_publish_default_pose', False)

        self.declare_parameter('desired_position_topic', '/desired_position')
        self.declare_parameter(
            'desired_joint_position_topic', '/desired_joint_position'
        )
        self.declare_parameter('canonical_joint_state_topic', '/joint_states')
        self.declare_parameter('legacy_position_topic', '/joint_states_position')
        self.declare_parameter('legacy_velocity_topic', '/joint_states_velocity')
        self.declare_parameter('servo_target_topic', '/servo_target_radians')
        self.declare_parameter(
            'safe_target_joint_state_topic', '/safe_joint_targets'
        )
        self.declare_parameter('pd_torque_debug_topic', '/pd_torque_debug')
        self.declare_parameter(
            'outer_velocity_debug_topic', '/outer_controller/velocity_command'
        )
        self.declare_parameter(
            'outer_error_debug_topic', '/outer_controller/position_error'
        )
        self.declare_parameter(
            'outer_integral_debug_topic', '/outer_controller/integral_error'
        )
        self.declare_parameter('controller_status_topic', '/outer_controller/status')
        self.declare_parameter(
            'reset_to_feedback_service', '/pd_controller/reset_to_feedback'
        )

        # Reference shaping. Alpha=1 means no low-pass filtering.
        self.declare_parameter('command_filter_alpha', 0.7)
        self.declare_parameter('enable_rate_limit', True)
        self.declare_parameter('enable_accel_limit', True)
        self.declare_parameter('max_joint_speed_rad_s', [8.0] * 12)
        self.declare_parameter('max_joint_accel_rad_s2', [80.0] * 12)

        # Real outer-loop gains.  Units are appropriate for the velocity-form
        # position controller, not Isaac torque-domain actuator gains.
        self.declare_parameter('kp', [0.0] * 12)
        self.declare_parameter('kd', [0.0] * 12)
        self.declare_parameter('ki', [0.0] * 12)
        self.declare_parameter('max_controller_velocity_rad_s', [2.0] * 12)
        self.declare_parameter('max_controller_accel_rad_s2', [20.0] * 12)
        self.declare_parameter('integral_error_limit_rad_sec', [0.25] * 12)

        # Debug compatibility. /pd_torque_debug remains available but is never
        # used as the ST3215 command path.
        self.declare_parameter('publish_pd_torque_debug', False)
        self.declare_parameter('publish_outer_loop_debug', True)
        self.declare_parameter('torque_limit', 1.5)

        self.control_rate_hz = max(
            float(self.get_parameter('control_rate_hz').value), 1.0
        )
        self.control_dt = 1.0 / self.control_rate_hz
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.feedback_timeout_sec = float(
            self.get_parameter('feedback_timeout_sec').value
        )
        self.require_feedback_for_outer_loop = bool(
            self.get_parameter('require_feedback_for_outer_loop').value
        )
        self.initialize_output_from_feedback = bool(
            self.get_parameter('initialize_output_from_feedback').value
        )
        self.command_filter_alpha = self._clamp01(
            float(self.get_parameter('command_filter_alpha').value)
        )
        self.enable_rate_limit = bool(
            self.get_parameter('enable_rate_limit').value
        )
        self.enable_accel_limit = bool(
            self.get_parameter('enable_accel_limit').value
        )
        self.publish_pd_torque_debug = bool(
            self.get_parameter('publish_pd_torque_debug').value
        )
        self.publish_outer_loop_debug = bool(
            self.get_parameter('publish_outer_loop_debug').value
        )

        self.controller_mode = str(self.get_parameter('controller_mode').value)
        if self.controller_mode not in self.VALID_MODES:
            raise ValueError(
                f'controller_mode must be one of {self.VALID_MODES}, '
                f'got {self.controller_mode!r}'
            )

        self.joint_map_path = self._resolve_joint_map_path(
            str(self.get_parameter('joint_map_path').value)
        )
        self._load_joint_map(self.joint_map_path)

        self.max_joint_speed = self._expand_to_joint_array(
            self.get_parameter('max_joint_speed_rad_s').value,
            'max_joint_speed_rad_s',
            minimum=0.0,
        )
        self.max_joint_accel = self._expand_to_joint_array(
            self.get_parameter('max_joint_accel_rad_s2').value,
            'max_joint_accel_rad_s2',
            minimum=0.0,
        )
        self.kp = self._expand_to_joint_array(
            self.get_parameter('kp').value, 'kp', minimum=0.0
        )
        self.kd = self._expand_to_joint_array(
            self.get_parameter('kd').value, 'kd', minimum=0.0
        )
        self.ki = self._expand_to_joint_array(
            self.get_parameter('ki').value, 'ki', minimum=0.0
        )
        self.max_controller_velocity = self._expand_to_joint_array(
            self.get_parameter('max_controller_velocity_rad_s').value,
            'max_controller_velocity_rad_s',
            minimum=0.0,
        )
        self.max_controller_accel = self._expand_to_joint_array(
            self.get_parameter('max_controller_accel_rad_s2').value,
            'max_controller_accel_rad_s2',
            minimum=0.0,
        )
        self.integral_error_limit = self._expand_to_joint_array(
            self.get_parameter('integral_error_limit_rad_sec').value,
            'integral_error_limit_rad_sec',
            minimum=0.0,
        )

        self.outer_controller = OuterLoopPositionController(
            self.num_joints,
            kp=self.kp,
            kd=self.kd,
            ki=self.ki,
            max_velocity=self.max_controller_velocity,
            max_acceleration=self.max_controller_accel,
            integral_error_limit=self.integral_error_limit,
            lower_limits=self.lower_limits,
            upper_limits=self.upper_limits,
        )
        self.legacy_pd_debug = PDController(
            self.num_joints,
            kp=self.kp,
            kd=self.kd,
            torque_limit=float(self.get_parameter('torque_limit').value),
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.desired_position = self.default_joint_positions.copy()
        self.reference_position = self.default_joint_positions.copy()
        self.reference_velocity = np.zeros(self.num_joints, dtype=np.float64)
        self.target_position = self.default_joint_positions.copy()
        self.target_velocity = np.zeros(self.num_joints, dtype=np.float64)
        self.current_position = self.default_joint_positions.copy()
        self.current_velocity = np.zeros(self.num_joints, dtype=np.float64)

        self.position_seen = np.zeros(self.num_joints, dtype=bool)
        self.velocity_seen = np.zeros(self.num_joints, dtype=bool)
        self.last_feedback_msg_sec: Optional[float] = None
        self.last_desired_msg_sec: Optional[float] = None
        self.has_desired_position = False
        self.initialized_from_feedback = False
        self.warned_command_timeout = False
        self.warned_feedback_stale = False

        # ------------------------------------------------------------------
        # ROS interface
        # ------------------------------------------------------------------
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        desired_topic = str(self.get_parameter('desired_position_topic').value)
        desired_joint_topic = str(
            self.get_parameter('desired_joint_position_topic').value
        )
        self.create_subscription(
            Float64MultiArray,
            desired_topic,
            self.desired_position_callback,
            command_qos,
        )
        if desired_joint_topic and desired_joint_topic != desired_topic:
            self.create_subscription(
                Float64MultiArray,
                desired_joint_topic,
                self.desired_position_callback,
                command_qos,
            )

        self.create_subscription(
            JointState,
            str(self.get_parameter('canonical_joint_state_topic').value),
            self.joint_state_callback,
            sensor_qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('legacy_position_topic').value),
            self.legacy_position_callback,
            sensor_qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('legacy_velocity_topic').value),
            self.legacy_velocity_callback,
            sensor_qos,
        )

        self.servo_target_topic = str(
            self.get_parameter('servo_target_topic').value
        )
        self.servo_target_pub = self.create_publisher(
            Float64MultiArray, self.servo_target_topic, command_qos
        )
        self.safe_target_pub = self.create_publisher(
            JointState,
            str(self.get_parameter('safe_target_joint_state_topic').value),
            command_qos,
        )
        self.pd_torque_debug_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('pd_torque_debug_topic').value),
            debug_qos,
        )
        self.outer_velocity_debug_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('outer_velocity_debug_topic').value),
            debug_qos,
        )
        self.outer_error_debug_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('outer_error_debug_topic').value),
            debug_qos,
        )
        self.outer_integral_debug_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('outer_integral_debug_topic').value),
            debug_qos,
        )
        self.controller_status_pub = self.create_publisher(
            String,
            str(self.get_parameter('controller_status_topic').value),
            debug_qos,
        )

        self.reset_service = self.create_service(
            Trigger,
            str(self.get_parameter('reset_to_feedback_service').value),
            self.reset_to_feedback_callback,
        )

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        if bool(self.get_parameter('startup_publish_default_pose').value):
            self._publish_safe_target(self.target_position, self.target_velocity)

        self.get_logger().info(
            f'Outer-loop controller initialized: mode={self.controller_mode}, '
            f'joints={self.num_joints}, rate={self.control_rate_hz:.1f}Hz, '
            f'output={self.servo_target_topic}, map={self.joint_map_path}'
        )
        if self.controller_mode == 'safety_only':
            self.get_logger().warn(
                'controller_mode=safety_only: preserving legacy shaping/safety '
                'behavior; measured feedback is monitored but not yet used to '
                'correct the servo target.'
            )

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, value))

    @staticmethod
    def _now_sec() -> float:
        return time.monotonic()

    def _resolve_joint_map_path(self, configured_path: str) -> str:
        candidates: list[Path] = []
        if configured_path:
            candidates.append(Path(configured_path).expanduser())
        if get_package_share_directory is not None:
            try:
                candidates.append(
                    Path(get_package_share_directory('littlegreen_biped_pkg'))
                    / 'configs'
                    / 'joint_map.yaml'
                )
            except Exception:
                pass

        for candidate in candidates:
            if candidate.is_file():
                return str(candidate.resolve())

        raise FileNotFoundError(
            'Unable to resolve joint_map_path. Tried: '
            + ', '.join(str(path) for path in candidates)
        )

    def _load_joint_map(self, joint_map_path: str) -> None:
        with open(joint_map_path, 'r', encoding='utf-8') as stream:
            root = yaml.safe_load(stream)

        joints = root.get('joints', [])
        if not joints:
            raise ValueError(f'No joints found in joint map: {joint_map_path}')

        joints = sorted(
            joints,
            key=lambda joint: int(
                joint.get(
                    'policy_action_index',
                    joint.get('micro_ros_array_index', 0),
                )
            ),
        )
        expected_indices = list(range(len(joints)))
        actual_indices = [
            int(joint.get('policy_action_index', index))
            for index, joint in enumerate(joints)
        ]
        if actual_indices != expected_indices:
            raise ValueError(
                f'policy_action_index must be contiguous 0..N-1. '
                f'Found {actual_indices}'
            )

        self.joint_names = [str(joint['name']) for joint in joints]
        self.servo_ids = [
            int(joint.get('servo_id', index + 1))
            for index, joint in enumerate(joints)
        ]
        self.num_joints = len(joints)
        self.default_joint_positions = np.asarray(
            [float(joint.get('default_joint_rad', 0.0)) for joint in joints],
            dtype=np.float64,
        )
        self.lower_limits = np.asarray(
            [float(joint['limit_lower_rad']) for joint in joints],
            dtype=np.float64,
        )
        self.upper_limits = np.asarray(
            [float(joint['limit_upper_rad']) for joint in joints],
            dtype=np.float64,
        )
        self.joint_name_to_index = {
            name: index for index, name in enumerate(self.joint_names)
        }

        invalid = np.where(self.lower_limits >= self.upper_limits)[0]
        if invalid.size:
            bad = ', '.join(self.joint_names[index] for index in invalid)
            raise ValueError(f'Invalid joint limits for: {bad}')

        self.get_logger().info(
            f'Loaded joint map: {self.num_joints} joints, servo IDs={self.servo_ids}'
        )

    def _expand_to_joint_array(
        self,
        value,
        param_name: str,
        minimum: Optional[float] = None,
    ) -> np.ndarray:
        if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
            arr = np.asarray(list(value), dtype=np.float64)
            if arr.size != self.num_joints:
                raise ValueError(
                    f'{param_name} must have {self.num_joints} values, '
                    f'got {arr.size}'
                )
        else:
            arr = np.full(self.num_joints, float(value), dtype=np.float64)

        if minimum is not None:
            arr = np.maximum(arr, minimum)
        return arr

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def desired_position_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < self.num_joints:
            self.get_logger().warn(
                f'Desired position too short: got {len(msg.data)}, '
                f'expected {self.num_joints}'
            )
            return

        desired = np.asarray(msg.data[: self.num_joints], dtype=np.float64)
        desired = self._sanitize_and_clip(desired, source='desired_position')
        self.desired_position = desired
        self.last_desired_msg_sec = self._now_sec()
        self.has_desired_position = True
        self.warned_command_timeout = False

    def joint_state_callback(self, msg: JointState) -> None:
        self._update_joint_feedback_from_joint_state(
            msg, update_position=True, update_velocity=True
        )

    def legacy_position_callback(self, msg: JointState) -> None:
        self._update_joint_feedback_from_joint_state(
            msg, update_position=True, update_velocity=False
        )

    def legacy_velocity_callback(self, msg: JointState) -> None:
        self._update_joint_feedback_from_joint_state(
            msg, update_position=False, update_velocity=True
        )

    def _update_joint_feedback_from_joint_state(
        self,
        msg: JointState,
        *,
        update_position: bool,
        update_velocity: bool,
    ) -> None:
        updated_any = False

        if msg.name:
            name_to_src = {name: index for index, name in enumerate(msg.name)}
            for index, joint_name in enumerate(self.joint_names):
                src_index = name_to_src.get(joint_name)
                if src_index is None:
                    continue
                if update_position and src_index < len(msg.position):
                    value = float(msg.position[src_index])
                    if np.isfinite(value):
                        self.current_position[index] = value
                        self.position_seen[index] = True
                        updated_any = True
                if update_velocity and src_index < len(msg.velocity):
                    value = float(msg.velocity[src_index])
                    if np.isfinite(value):
                        self.current_velocity[index] = value
                        self.velocity_seen[index] = True
                        updated_any = True
        else:
            if update_position and len(msg.position) >= self.num_joints:
                values = np.asarray(msg.position[: self.num_joints], dtype=np.float64)
                finite = np.isfinite(values)
                self.current_position[finite] = values[finite]
                self.position_seen[finite] = True
                updated_any = updated_any or bool(np.any(finite))
            if update_velocity and len(msg.velocity) >= self.num_joints:
                values = np.asarray(msg.velocity[: self.num_joints], dtype=np.float64)
                finite = np.isfinite(values)
                self.current_velocity[finite] = values[finite]
                self.velocity_seen[finite] = True
                updated_any = updated_any or bool(np.any(finite))

        if updated_any:
            self.last_feedback_msg_sec = self._now_sec()

    def reset_to_feedback_callback(self, request, response):
        del request
        if not self._feedback_ready():
            response.success = False
            response.message = 'Cannot reset: complete fresh joint feedback is unavailable.'
            return response

        self._reset_controller_state_to_feedback()
        response.success = True
        response.message = (
            'Reference shaper and outer-loop controller reset to current feedback.'
        )
        return response

    # ------------------------------------------------------------------
    # Control path
    # ------------------------------------------------------------------
    def _feedback_ready(self) -> bool:
        if not np.all(self.position_seen) or not np.all(self.velocity_seen):
            return False
        if self.last_feedback_msg_sec is None:
            return False
        if self._now_sec() - self.last_feedback_msg_sec > self.feedback_timeout_sec:
            return False
        return bool(
            np.all(np.isfinite(self.current_position))
            and np.all(np.isfinite(self.current_velocity))
        )

    def _reset_controller_state_to_feedback(self) -> None:
        feedback = np.clip(
            self.current_position.copy(), self.lower_limits, self.upper_limits
        )
        self.reference_position = feedback.copy()
        self.reference_velocity.fill(0.0)
        self.target_position = feedback.copy()
        self.target_velocity.fill(0.0)
        self.outer_controller.reset(feedback)
        self.initialized_from_feedback = True
        self.warned_feedback_stale = False
        self.get_logger().info(
            'Controller/reference state aligned to current measured joint feedback.'
        )

    def control_loop(self) -> None:
        feedback_ready = self._feedback_ready()

        if (
            self.initialize_output_from_feedback
            and feedback_ready
            and not self.initialized_from_feedback
        ):
            self._reset_controller_state_to_feedback()

        desired = self._get_timeout_safe_desired()
        desired = self._sanitize_and_clip(desired, source='control_loop')

        reference_position, reference_velocity = self._shape_reference(desired)
        self.reference_position = reference_position
        self.reference_velocity = reference_velocity

        if self.controller_mode == 'safety_only':
            output_position = reference_position.copy()
            output_velocity = reference_velocity.copy()
            self.outer_controller.reset(output_position, output_velocity)
            position_error = reference_position - self.current_position
            integral_error = np.zeros(self.num_joints, dtype=np.float64)
            raw_velocity_command = reference_velocity.copy()
        else:
            if self.require_feedback_for_outer_loop and not feedback_ready:
                if not self.warned_feedback_stale:
                    self.get_logger().warn(
                        'Outer-loop feedback unavailable/stale; holding last safe target.'
                    )
                    self.warned_feedback_stale = True
                output_position = self.target_position.copy()
                output_velocity = np.zeros(self.num_joints, dtype=np.float64)
                position_error = reference_position - self.current_position
                integral_error = self.outer_controller.integral_error.copy()
                raw_velocity_command = np.zeros(self.num_joints, dtype=np.float64)
            else:
                self.warned_feedback_stale = False
                result = self.outer_controller.compute(
                    reference_position=reference_position,
                    reference_velocity=reference_velocity,
                    current_position=self.current_position,
                    current_velocity=self.current_velocity,
                    dt=self.control_dt,
                    mode=self.controller_mode,
                )
                output_position = result.position_command
                output_velocity = result.velocity_command
                position_error = result.position_error
                integral_error = result.integral_error
                raw_velocity_command = result.raw_velocity_command

        output_position = self._sanitize_and_clip(
            output_position, source='controller_output'
        )
        self.target_position = output_position.copy()
        self.target_velocity = output_velocity.copy()
        self._publish_safe_target(output_position, output_velocity)

        if self.publish_outer_loop_debug:
            self._publish_array(
                self.outer_velocity_debug_pub,
                raw_velocity_command,
                'qdot_cmd_raw[12]',
            )
            self._publish_array(
                self.outer_error_debug_pub,
                position_error,
                'position_error_rad[12]',
            )
            self._publish_array(
                self.outer_integral_debug_pub,
                integral_error,
                'integral_error_rad_sec[12]',
            )
            status = String()
            feedback_age = (
                -1.0
                if self.last_feedback_msg_sec is None
                else self._now_sec() - self.last_feedback_msg_sec
            )
            status.data = (
                f'mode={self.controller_mode} '
                f'feedback_ready={int(feedback_ready)} '
                f'feedback_age_sec={feedback_age:.6f} '
                f'command_age_sec={self._command_age_sec():.6f}'
            )
            self.controller_status_pub.publish(status)

        if self.publish_pd_torque_debug:
            torque_command = self.legacy_pd_debug.compute_command(
                desired_pos=reference_position,
                current_pos=self.current_position,
                current_vel=self.current_velocity,
                desired_vel=reference_velocity,
            )
            self._publish_array(
                self.pd_torque_debug_pub,
                torque_command,
                'legacy_pd_torque_like_debug[12]',
            )

    def _command_age_sec(self) -> float:
        if self.last_desired_msg_sec is None:
            return -1.0
        return self._now_sec() - self.last_desired_msg_sec

    def _get_timeout_safe_desired(self) -> np.ndarray:
        if not self.has_desired_position:
            if self.initialized_from_feedback:
                return self.reference_position.copy()
            return self.default_joint_positions.copy()

        age = self._command_age_sec()
        if age > self.command_timeout_sec:
            if not self.warned_command_timeout:
                self.get_logger().warn(
                    f'Desired command timeout ({age:.3f}s); holding last reference.'
                )
                self.warned_command_timeout = True
            return self.reference_position.copy()

        return self.desired_position.copy()

    def _sanitize_and_clip(self, values: np.ndarray, *, source: str) -> np.ndarray:
        if values.shape[0] != self.num_joints:
            raise ValueError(
                f'{source}: expected {self.num_joints} values, got {values.shape[0]}'
            )

        clean = values.astype(np.float64, copy=True)
        bad = ~np.isfinite(clean)
        if np.any(bad):
            bad_names = ', '.join(
                self.joint_names[index] for index in np.where(bad)[0]
            )
            self.get_logger().warn(
                f'Non-finite values from {source}; retaining prior target for: '
                f'{bad_names}'
            )
            clean[bad] = self.target_position[bad]

        return np.clip(clean, self.lower_limits, self.upper_limits)

    def _shape_reference(self, desired: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        filtered = (
            self.command_filter_alpha * desired
            + (1.0 - self.command_filter_alpha) * self.reference_position
        )

        delta = filtered - self.reference_position
        if self.enable_rate_limit:
            max_delta = self.max_joint_speed * self.control_dt
            delta = np.clip(delta, -max_delta, max_delta)

        requested_velocity = delta / self.control_dt
        if self.enable_accel_limit:
            max_dv = self.max_joint_accel * self.control_dt
            velocity = self.reference_velocity + np.clip(
                requested_velocity - self.reference_velocity,
                -max_dv,
                max_dv,
            )
        else:
            velocity = requested_velocity

        if self.enable_rate_limit:
            velocity = np.clip(
                velocity, -self.max_joint_speed, self.max_joint_speed
            )

        position = self.reference_position + velocity * self.control_dt
        position = np.clip(position, self.lower_limits, self.upper_limits)
        actual_velocity = (position - self.reference_position) / self.control_dt
        return position, actual_velocity

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------
    def _publish_array(self, publisher, values, label: str) -> None:
        msg = Float64MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label=label,
                size=self.num_joints,
                stride=self.num_joints,
            )
        )
        msg.data = np.asarray(values, dtype=np.float64).astype(float).tolist()
        publisher.publish(msg)

    def _publish_safe_target(
        self,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
    ) -> None:
        self._publish_array(
            self.servo_target_pub,
            target_position,
            'joints',
        )

        debug_msg = JointState()
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.name = list(self.joint_names)
        debug_msg.position = target_position.astype(float).tolist()
        debug_msg.velocity = target_velocity.astype(float).tolist()
        self.safe_target_pub.publish(debug_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PDControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
