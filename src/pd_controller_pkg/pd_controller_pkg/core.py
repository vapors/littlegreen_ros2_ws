"""Outer-loop position controller helpers for LittleGreen.

The ST3215 is commanded in position, so this module intentionally does not
pretend the hardware is a direct torque actuator.  The production controller
computes a bounded joint-velocity command from position/velocity error and
integrates that velocity into the next safe position target.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np


@dataclass
class OuterLoopResult:
    position_command: np.ndarray
    velocity_command: np.ndarray
    raw_velocity_command: np.ndarray
    position_error: np.ndarray
    velocity_error: np.ndarray
    integral_error: np.ndarray


class OuterLoopPositionController:
    """Vectorized PD/PID outer loop that outputs a position command.

    Control law::

        e       = q_ref - q
        e_dot   = qdot_ref - qdot
        qdot*   = kp*e + kd*e_dot + ki*integral(e)
        qdot    = velocity/acceleration limited(qdot*)
        q_cmd   = q_cmd_prev + qdot*dt

    The final q_cmd is clipped to the configured physical joint limits.
    Integral action is optional and disabled in ``outer_pd`` mode.
    """

    def __init__(
        self,
        num_joints: int,
        *,
        kp,
        kd,
        ki,
        max_velocity,
        max_acceleration,
        integral_error_limit,
        lower_limits,
        upper_limits,
    ) -> None:
        self.num_joints = int(num_joints)
        self.kp = self._as_joint_array(kp, 'kp')
        self.kd = self._as_joint_array(kd, 'kd')
        self.ki = self._as_joint_array(ki, 'ki')
        self.max_velocity = np.maximum(
            self._as_joint_array(max_velocity, 'max_velocity'), 0.0
        )
        self.max_acceleration = np.maximum(
            self._as_joint_array(max_acceleration, 'max_acceleration'), 0.0
        )
        self.integral_error_limit = np.maximum(
            self._as_joint_array(integral_error_limit, 'integral_error_limit'), 0.0
        )
        self.lower_limits = self._as_joint_array(lower_limits, 'lower_limits')
        self.upper_limits = self._as_joint_array(upper_limits, 'upper_limits')

        self.output_position = np.zeros(self.num_joints, dtype=np.float64)
        self.output_velocity = np.zeros(self.num_joints, dtype=np.float64)
        self.integral_error = np.zeros(self.num_joints, dtype=np.float64)
        self.initialized = False

    def _as_joint_array(self, value, name: str) -> np.ndarray:
        if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
            arr = np.asarray(list(value), dtype=np.float64)
            if arr.size != self.num_joints:
                raise ValueError(
                    f'{name} must have {self.num_joints} values, got {arr.size}'
                )
            return arr
        return np.full(self.num_joints, float(value), dtype=np.float64)

    def reset(self, position, velocity=None) -> None:
        position_arr = self._as_joint_array(position, 'reset position')
        self.output_position = np.clip(
            position_arr, self.lower_limits, self.upper_limits
        )
        if velocity is None:
            self.output_velocity.fill(0.0)
        else:
            self.output_velocity = np.clip(
                self._as_joint_array(velocity, 'reset velocity'),
                -self.max_velocity,
                self.max_velocity,
            )
        self.integral_error.fill(0.0)
        self.initialized = True

    def compute(
        self,
        *,
        reference_position,
        reference_velocity,
        current_position,
        current_velocity,
        dt: float,
        mode: str,
    ) -> OuterLoopResult:
        if mode not in ('outer_pd', 'outer_pid'):
            raise ValueError(f'Unsupported outer-loop mode: {mode}')
        if dt <= 0.0:
            raise ValueError('dt must be positive')

        q_ref = self._as_joint_array(reference_position, 'reference_position')
        qdot_ref = self._as_joint_array(reference_velocity, 'reference_velocity')
        q = self._as_joint_array(current_position, 'current_position')
        qdot = self._as_joint_array(current_velocity, 'current_velocity')

        if not self.initialized:
            self.reset(q)

        position_error = q_ref - q
        velocity_error = qdot_ref - qdot

        previous_integral = self.integral_error.copy()
        if mode == 'outer_pid':
            self.integral_error += position_error * dt
            self.integral_error = np.clip(
                self.integral_error,
                -self.integral_error_limit,
                self.integral_error_limit,
            )
        else:
            self.integral_error.fill(0.0)

        raw_velocity = (
            self.kp * position_error
            + self.kd * velocity_error
            + self.ki * self.integral_error
        )

        velocity_command = np.clip(
            raw_velocity, -self.max_velocity, self.max_velocity
        )

        max_dv = self.max_acceleration * dt
        velocity_command = self.output_velocity + np.clip(
            velocity_command - self.output_velocity,
            -max_dv,
            max_dv,
        )
        velocity_command = np.clip(
            velocity_command, -self.max_velocity, self.max_velocity
        )

        unclipped_position = self.output_position + velocity_command * dt
        position_command = np.clip(
            unclipped_position, self.lower_limits, self.upper_limits
        )

        # Simple conditional-integration anti-windup. If the integrated command
        # would push farther through a physical joint limit, discard that cycle's
        # new integral contribution for the affected joint.
        if mode == 'outer_pid':
            lower_push = (
                unclipped_position < self.lower_limits
            ) & (position_error < 0.0)
            upper_push = (
                unclipped_position > self.upper_limits
            ) & (position_error > 0.0)
            unwind = lower_push | upper_push
            self.integral_error[unwind] = previous_integral[unwind]

        # The actual integrated velocity may be smaller after joint clipping.
        actual_velocity = (position_command - self.output_position) / dt

        self.output_position = position_command.copy()
        self.output_velocity = actual_velocity.copy()

        return OuterLoopResult(
            position_command=position_command.copy(),
            velocity_command=actual_velocity.copy(),
            raw_velocity_command=raw_velocity.copy(),
            position_error=position_error.copy(),
            velocity_error=velocity_error.copy(),
            integral_error=self.integral_error.copy(),
        )


class PDController:
    """Legacy torque-like PD helper retained for /pd_torque_debug compatibility.

    This class does not command the ST3215 path.  It is retained only so the
    existing debug topic and downstream tooling remain available.
    """

    def __init__(
        self,
        num_joints: int,
        kp=20.0,
        kd=1.0,
        torque_limit: Optional[float] = None,
    ) -> None:
        self.num_joints = int(num_joints)
        self.kp = self._as_joint_array(kp, 'kp')
        self.kd = self._as_joint_array(kd, 'kd')
        self.torque_limit = torque_limit
        self.last_error = np.zeros(self.num_joints, dtype=np.float32)

    def _as_joint_array(self, value, name: str) -> np.ndarray:
        if isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
            arr = np.array(list(value), dtype=np.float32)
            if arr.size != self.num_joints:
                raise ValueError(
                    f'{name} must have {self.num_joints} values, got {arr.size}'
                )
            return arr
        return np.full(self.num_joints, float(value), dtype=np.float32)

    def compute_command(
        self,
        desired_pos,
        current_pos,
        current_vel,
        desired_vel=None,
    ) -> np.ndarray:
        desired_pos = np.asarray(desired_pos, dtype=np.float32)
        current_pos = np.asarray(current_pos, dtype=np.float32)
        current_vel = np.asarray(current_vel, dtype=np.float32)
        if desired_vel is None:
            desired_vel = np.zeros(self.num_joints, dtype=np.float32)
        else:
            desired_vel = np.asarray(desired_vel, dtype=np.float32)

        error = desired_pos - current_pos
        d_error = desired_vel - current_vel
        command = self.kp * error + self.kd * d_error

        if self.torque_limit is not None:
            command = np.clip(
                command,
                -float(self.torque_limit),
                float(self.torque_limit),
            )

        self.last_error = error.astype(np.float32)
        return command.astype(np.float32)
