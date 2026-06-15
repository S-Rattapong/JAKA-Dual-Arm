import math

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped

from jaka_coop_monitor.dual_object_trajectory_demo import quaternion_to_msg
from jaka_coop_monitor.dual_relative_motion_demo import (
    damped_pseudoinverse,
    orientation_error_rotvec,
    relative_quaternion,
)
from jaka_coop_monitor.dual_task_priority_joint_limit_demo import (
    DualTaskPriorityJointLimitDemo,
)


class DualSafetyPostureDemo(DualTaskPriorityJointLimitDemo):
    def __init__(self):
        self._demo_node_name = 'dual_safety_posture_demo'
        self._demo_title = 'Dual Safety Posture Demo'
        self._log_trajectory_params = False
        super().__init__()

        self.declare_parameter('object_min_x', 0.8)
        self.declare_parameter('object_max_x', 1.6)
        self.declare_parameter('object_min_y', -0.6)
        self.declare_parameter('object_max_y', 0.6)
        self.declare_parameter('object_min_z', 0.05)
        self.declare_parameter('object_max_z', 2.0)
        self.declare_parameter('max_object_target_speed', 0.20)

        self.declare_parameter('enable_home_posture_bias', True)
        self.declare_parameter('home_posture_gain', 0.10)
        self.declare_parameter('max_home_posture_velocity', 0.10)

        self.declare_parameter('enable_qdot_smoothing', True)
        self.declare_parameter('qdot_smoothing_alpha', 0.25)

        self.declare_parameter('enable_last_safe_target', True)
        self.declare_parameter('minimum_allowed_scale', 0.20)
        self.declare_parameter('maximum_raw_qdot_norm', 10.0)
        self.declare_parameter('joint_limit_hard_stop_margin', 0.05)
        self.declare_parameter('debug_joint_limit_raw_avoidance', False)

        self.object_min = np.array(
            [
                float(self.get_parameter('object_min_x').value),
                float(self.get_parameter('object_min_y').value),
                float(self.get_parameter('object_min_z').value),
            ],
            dtype=float,
        )
        self.object_max = np.array(
            [
                float(self.get_parameter('object_max_x').value),
                float(self.get_parameter('object_max_y').value),
                float(self.get_parameter('object_max_z').value),
            ],
            dtype=float,
        )
        self.max_object_target_speed = float(
            self.get_parameter('max_object_target_speed').value
        )

        self.enable_home_posture_bias = bool(
            self.get_parameter('enable_home_posture_bias').value
        )
        self.home_posture_gain = float(self.get_parameter('home_posture_gain').value)
        self.max_home_posture_velocity = float(
            self.get_parameter('max_home_posture_velocity').value
        )

        self.enable_qdot_smoothing = bool(
            self.get_parameter('enable_qdot_smoothing').value
        )
        self.qdot_smoothing_alpha = float(
            self.get_parameter('qdot_smoothing_alpha').value
        )

        self.enable_last_safe_target = bool(
            self.get_parameter('enable_last_safe_target').value
        )
        self.minimum_allowed_scale = float(
            self.get_parameter('minimum_allowed_scale').value
        )
        self.maximum_raw_qdot_norm = float(
            self.get_parameter('maximum_raw_qdot_norm').value
        )
        self.joint_limit_hard_stop_margin = float(
            self.get_parameter('joint_limit_hard_stop_margin').value
        )
        self.debug_joint_limit_raw_avoidance = bool(
            self.get_parameter('debug_joint_limit_raw_avoidance').value
        )

        self.raw_desired_object_position = None
        self.safe_desired_object_position = None
        self.last_safe_desired_object_position = None
        self.previous_q_dot = None
        self.home_q_vector = None
        self.target_was_clamped = False
        self.last_reject_log_time = None
        self.last_hard_stop_warning_time = None

        self.get_logger().info(
            'Safety posture demo: raw desired_object is the marker target; '
            'safe_desired_object is the filtered/clamped target used for control.'
        )

        if self.debug_joint_limit_raw_avoidance:
            self.get_logger().warn(
                'debug_joint_limit_raw_avoidance is enabled: joint-limit '
                'avoidance will bypass null-space projection.'
            )

    def apply_joint_limit_test_pose(self):
        test_joint = 'right_joint_6'
        if test_joint not in self.right_joint_names:
            return

        if test_joint not in self.joint_limits:
            self.get_logger().warn(
                f'Joint limit test pose requested, but no limits for {test_joint}.'
            )
            return

        index = self.right_joint_names.index(test_joint)
        upper = self.joint_limits[test_joint]['upper']
        test_margin = max(
            self.joint_limit_danger_margin + 0.16,
            self.joint_limit_warning_margin - 0.01,
            0.29,
        )
        target = upper - test_margin

        while len(self.initial_right_positions) <= index:
            self.initial_right_positions.append(0.0)

        self.initial_right_positions[index] = target
        self.get_logger().warn(
            f'Joint limit test pose enabled: {test_joint} initialized to '
            f'{target:.3f} rad (margin={test_margin:.3f} rad)'
        )

    def initialize_state(self):
        super().initialize_state()

        all_joint_names = self.left_joint_names + self.right_joint_names
        self.home_q_vector = np.array(
            [self.q_map.get(name, 0.0) for name in all_joint_names],
            dtype=float,
        )

        self.raw_desired_object_position = self.target_object_position.copy()
        clamped, was_clamped = self.clamp_object_target(
            self.raw_desired_object_position
        )
        self.safe_desired_object_position = self.initial_object_position.copy()
        self.last_safe_desired_object_position = self.initial_object_position.copy()
        self.latest_desired_object_position = self.raw_desired_object_position.copy()
        self.latest_safe_desired_object_position = (
            self.safe_desired_object_position.copy()
        )
        self.previous_q_dot = np.zeros(len(all_joint_names), dtype=float)

        if was_clamped:
            self.log_target_clamped(self.raw_desired_object_position, clamped)

        self.get_logger().info(
            f'workspace x=[{self.object_min[0]:.3f}, {self.object_max[0]:.3f}] '
            f'y=[{self.object_min[1]:.3f}, {self.object_max[1]:.3f}] '
            f'z=[{self.object_min[2]:.3f}, {self.object_max[2]:.3f}] m'
        )

    def process_marker_feedback(self, feedback):
        self.raw_desired_object_position = np.array(
            [
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z,
            ],
            dtype=float,
        )
        self.target_object_position = self.raw_desired_object_position.copy()

    def desired_object_position(self, elapsed):
        del elapsed

        if self.safe_desired_object_position is not None:
            return self.safe_desired_object_position.copy()

        if self.initial_object_position is not None:
            return self.initial_object_position.copy()

        return np.zeros(3, dtype=float)

    def control_step(self, elapsed, dt):
        del elapsed

        raw_target = self.current_raw_target()
        clamped_target, target_clamped = self.clamp_object_target(raw_target)

        if target_clamped and not self.target_was_clamped:
            self.log_target_clamped(raw_target, clamped_target)
        self.target_was_clamped = target_clamped

        desired_object = self.limit_target_speed(
            self.safe_desired_object_position,
            clamped_target,
            dt,
        )

        command = self.compute_safe_command(desired_object)
        q_dot_raw = command['q_dot']
        qdot_raw_norm = float(np.linalg.norm(q_dot_raw))

        max_abs_qdot_raw = (
            float(np.max(np.abs(q_dot_raw))) if q_dot_raw.size > 0 else 0.0
        )
        raw_scale = 1.0
        if max_abs_qdot_raw > self.max_joint_velocity:
            raw_scale = self.max_joint_velocity / max_abs_qdot_raw

        target_rejected = self.should_reject_target(q_dot_raw, qdot_raw_norm, raw_scale)
        if target_rejected and self.last_safe_desired_object_position is not None:
            desired_object = self.last_safe_desired_object_position.copy()
            command = self.compute_safe_command(desired_object)
            q_dot_raw = command['q_dot']
            qdot_raw_norm = float(np.linalg.norm(q_dot_raw))
        else:
            self.last_safe_desired_object_position = desired_object.copy()

        q_dot_smoothed = self.smooth_qdot(q_dot_raw)
        qdot_smoothed_norm = float(np.linalg.norm(q_dot_smoothed))

        q_dot, scale = self.scale_qdot(q_dot_smoothed)
        self.previous_q_dot = q_dot.copy()

        self.latest_qdot_map = {}
        all_joint_names = self.left_joint_names + self.right_joint_names
        for joint_name, qd in zip(all_joint_names, q_dot):
            self.latest_qdot_map[joint_name] = float(qd)
            self.q_map[joint_name] += float(qd) * dt
            self.apply_joint_limits(joint_name)

        left_pose = command['left_pose']
        right_pose = command['right_pose']
        actual_object = command['actual_object']
        current_relative_position = command['current_relative_position']
        current_relative_distance = float(np.linalg.norm(current_relative_position))

        desired_left = desired_object + self.left_grasp_offset
        desired_right = desired_object + self.right_grasp_offset

        self.safe_desired_object_position = desired_object.copy()
        self.latest_raw_desired_object_position = raw_target.copy()
        self.latest_safe_desired_object_position = desired_object.copy()
        self.latest_desired_object_position = raw_target.copy()
        self.latest_desired_left_position = desired_left.copy()
        self.latest_desired_right_position = desired_right.copy()
        self.latest_actual_object_position = actual_object.copy()

        self.latest_status = {
            'relative_position_error_m': float(
                np.linalg.norm(command['relative_position_error'])
            ),
            'relative_orientation_error_deg': math.degrees(
                command['relative_orientation_error_angle']
            ),
            'object_position_error_m': float(
                np.linalg.norm(desired_object - actual_object)
            ),
            'left_grasp_position_error_m': float(
                np.linalg.norm(desired_left - left_pose['position'])
            ),
            'right_grasp_position_error_m': float(
                np.linalg.norm(desired_right - right_pose['position'])
            ),
            'relative_distance_error_m': abs(
                current_relative_distance - self.initial_relative_distance
            ),
            'rel_rank': command['rel_rank'],
            'rel_condition_number': command['rel_condition_number'],
            'obj_rank': command['obj_rank'],
            'obj_condition_number': command['obj_condition_number'],
            'min_joint_margin': command['avoidance_status']['min_joint_margin'],
            'min_margin_joint': command['avoidance_status']['min_margin_joint'],
            'warning_count': command['avoidance_status']['warning_count'],
            'danger_count': command['avoidance_status']['danger_count'],
            'avoidance_active_count': command['avoidance_status'][
                'avoidance_active_count'
            ],
            'home_bias_norm': command['home_bias_norm'],
            'qavoid_raw_norm': command['qavoid_raw_norm'],
            'qavoid_projected_norm': command['qavoid_projected_norm'],
            'max_qavoid_raw': command['max_qavoid_raw'],
            'max_qavoid_projected': command['max_qavoid_projected'],
            'active_avoidance': command['avoidance_status']['active_avoidance'],
            'qdot_raw_norm': qdot_raw_norm,
            'qdot_smoothed_norm': qdot_smoothed_norm,
            'scale': scale,
            'target_clamped': target_clamped,
            'target_rejected': target_rejected,
        }

        self.warn_joint_limit_zone_changes(command['avoidance_status'])

    def compute_safe_command(self, desired_object):
        left_result = self.compute_fk_and_jacobian(self.left_chain)
        right_result = self.compute_fk_and_jacobian(self.right_chain)

        left_pose = left_result['pose']
        right_pose = right_result['pose']
        actual_object = 0.5 * (left_pose['position'] + right_pose['position'])

        current_relative_position = right_pose['position'] - left_pose['position']
        current_relative_quaternion = relative_quaternion(
            left_pose['quaternion'],
            right_pose['quaternion']
        )

        relative_position_error = (
            self.desired_relative_position - current_relative_position
        )
        relative_rotvec_error, relative_orientation_error_angle = (
            orientation_error_rotvec(
                current_relative_quaternion,
                self.desired_relative_quaternion
            )
        )

        J_left = left_result['J']
        J_right = right_result['J']

        if self.enable_relative_orientation_task:
            x_dot_rel = np.zeros(6)
            x_dot_rel[:3] = self.relative_position_gain * relative_position_error
            x_dot_rel[3:] = (
                self.relative_orientation_gain * relative_rotvec_error
            )
            J_rel = np.hstack((-J_left, J_right))
        else:
            x_dot_rel = self.relative_position_gain * relative_position_error
            J_rel = np.hstack((-J_left[:3, :], J_right[:3, :]))

        J_rel_pinv = damped_pseudoinverse(J_rel, self.damping_relative)
        q_dot_rel = J_rel_pinv @ x_dot_rel

        identity = np.eye(J_rel.shape[1])
        N_rel = identity - J_rel_pinv @ J_rel

        object_position_error = desired_object - actual_object
        x_dot_obj = self.object_position_gain * object_position_error

        J_obj = 0.5 * np.hstack((J_left[:3, :], J_right[:3, :]))
        J_obj_projected = J_obj @ N_rel
        J_obj_projected_pinv = damped_pseudoinverse(
            J_obj_projected,
            self.damping_object
        )
        q_dot_obj_projected = N_rel @ (J_obj_projected_pinv @ x_dot_obj)

        N_rel_obj = N_rel @ (identity - J_obj_projected_pinv @ J_obj_projected)
        q_dot_home_projected = self.compute_home_bias(N_rel_obj)

        q_dot_avoid_raw, avoidance_status = self.compute_joint_limit_avoidance()
        if self.debug_joint_limit_raw_avoidance:
            q_dot_avoid_projected = q_dot_avoid_raw
        else:
            # Joint-limit avoidance is the lowest-priority task.  It is only
            # allowed to use velocity left over by the relative and object
            # tasks, so normal object tracking keeps the same behavior.
            q_dot_avoid_projected = N_rel_obj @ q_dot_avoid_raw

        q_dot = (
            q_dot_rel
            + q_dot_obj_projected
            + q_dot_home_projected
            + q_dot_avoid_projected
        )

        rel_singular_values = np.linalg.svd(J_rel, compute_uv=False)
        obj_singular_values = np.linalg.svd(J_obj_projected, compute_uv=False)

        return {
            'q_dot': q_dot,
            'left_pose': left_pose,
            'right_pose': right_pose,
            'actual_object': actual_object,
            'current_relative_position': current_relative_position,
            'relative_position_error': relative_position_error,
            'relative_orientation_error_angle': relative_orientation_error_angle,
            'avoidance_status': avoidance_status,
            'home_bias_norm': float(np.linalg.norm(q_dot_home_projected)),
            'qavoid_raw_norm': float(np.linalg.norm(q_dot_avoid_raw)),
            'qavoid_projected_norm': float(np.linalg.norm(q_dot_avoid_projected)),
            'max_qavoid_raw': (
                float(np.max(np.abs(q_dot_avoid_raw)))
                if q_dot_avoid_raw.size > 0
                else 0.0
            ),
            'max_qavoid_projected': (
                float(np.max(np.abs(q_dot_avoid_projected)))
                if q_dot_avoid_projected.size > 0
                else 0.0
            ),
            'rel_rank': int(np.linalg.matrix_rank(J_rel, tol=1e-5)),
            'rel_condition_number': self.condition_number(rel_singular_values),
            'obj_rank': int(np.linalg.matrix_rank(J_obj_projected, tol=1e-5)),
            'obj_condition_number': self.condition_number(obj_singular_values),
        }

    def compute_joint_limit_avoidance(self):
        all_joint_names = self.left_joint_names + self.right_joint_names
        q_dot_avoid = np.zeros(len(all_joint_names), dtype=float)

        min_joint_margin = float('inf')
        min_margin_joint = 'none'
        warning_joints = []
        danger_joints = []
        active_avoidance = []

        if not self.enable_joint_limit_avoidance:
            return q_dot_avoid, {
                'min_joint_margin': min_joint_margin,
                'min_margin_joint': min_margin_joint,
                'warning_count': 0,
                'danger_count': 0,
                'avoidance_active_count': 0,
                'warning_joints': [],
                'danger_joints': [],
                'active_avoidance': [],
            }

        warning = self.joint_limit_warning_margin
        danger = self.joint_limit_danger_margin

        for i, joint_name in enumerate(all_joint_names):
            if joint_name not in self.joint_limits:
                continue

            lower = self.joint_limits[joint_name]['lower']
            upper = self.joint_limits[joint_name]['upper']
            q = self.q_map.get(joint_name, 0.0)

            margin_lower = q - lower
            margin_upper = upper - q
            min_margin = min(margin_lower, margin_upper)

            if min_margin < min_joint_margin:
                min_joint_margin = min_margin
                min_margin_joint = joint_name

            if min_margin < warning:
                warning_joints.append(joint_name)
            if min_margin < danger:
                danger_joints.append(joint_name)

            if min_margin <= self.joint_limit_hard_stop_margin:
                self.warn_hard_stop_margin(joint_name, min_margin)

            # Hard inactive zone: when the joint is outside the warning zone,
            # command exactly zero avoidance velocity.
            if min_margin >= warning:
                q_dot_avoid[i] = 0.0
                continue

            if min_margin <= danger:
                activation = 1.0
            else:
                denom = max(warning - danger, 1e-9)
                s = (warning - min_margin) / denom
                s = max(0.0, min(1.0, s))
                activation = s * s * (3.0 - 2.0 * s)

            direction = 1.0 if margin_lower < margin_upper else -1.0
            velocity = direction * activation * self.joint_limit_avoidance_gain
            velocity = max(
                -self.max_avoidance_velocity,
                min(self.max_avoidance_velocity, velocity),
            )

            q_dot_avoid[i] = velocity
            if abs(velocity) > 1e-12:
                active_avoidance.append(
                    {
                        'joint': joint_name,
                        'margin': float(min_margin),
                        'activation': float(activation),
                        'qavoid': float(velocity),
                    }
                )

        return q_dot_avoid, {
            'min_joint_margin': float(min_joint_margin),
            'min_margin_joint': min_margin_joint,
            'warning_count': len(warning_joints),
            'danger_count': len(danger_joints),
            'avoidance_active_count': len(active_avoidance),
            'warning_joints': warning_joints,
            'danger_joints': danger_joints,
            'active_avoidance': active_avoidance,
        }

    def warn_hard_stop_margin(self, joint_name, min_margin):
        now = self.get_clock().now()
        if self.last_hard_stop_warning_time is not None:
            elapsed = (now - self.last_hard_stop_warning_time).nanoseconds * 1e-9
            if elapsed < 1.0:
                return

        self.last_hard_stop_warning_time = now
        self.get_logger().error(
            f'{joint_name} is within joint_limit_hard_stop_margin '
            f'(margin={min_margin:.4f} rad). Joint position will still be '
            'clamped to URDF limits.'
        )

    def current_raw_target(self):
        if self.raw_desired_object_position is not None:
            return self.raw_desired_object_position.copy()

        if self.target_object_position is not None:
            return self.target_object_position.copy()

        return self.initial_object_position.copy()

    def clamp_object_target(self, target):
        clamped = np.minimum(np.maximum(target, self.object_min), self.object_max)
        was_clamped = bool(np.linalg.norm(clamped - target) > 1e-9)
        return clamped, was_clamped

    def log_target_clamped(self, raw_target, clamped_target):
        self.get_logger().warn(
            'desired object target clamped: '
            f'raw=[{raw_target[0]:.3f}, {raw_target[1]:.3f}, {raw_target[2]:.3f}] '
            f'safe=[{clamped_target[0]:.3f}, {clamped_target[1]:.3f}, '
            f'{clamped_target[2]:.3f}]'
        )

    def limit_target_speed(self, current, target, dt):
        if current is None:
            return target.copy()

        delta = target - current
        distance = float(np.linalg.norm(delta))
        max_step = max(0.0, self.max_object_target_speed * dt)

        if distance <= max_step or distance <= 1e-12:
            return target.copy()

        return current + delta * (max_step / distance)

    def compute_home_bias(self, null_space):
        size = len(self.left_joint_names) + len(self.right_joint_names)
        if (
            not self.enable_home_posture_bias
            or self.home_q_vector is None
            or self.home_q_vector.size != size
        ):
            return np.zeros(size, dtype=float)

        current = np.array(
            [
                self.q_map.get(name, 0.0)
                for name in self.left_joint_names + self.right_joint_names
            ],
            dtype=float,
        )
        q_dot_home = self.home_posture_gain * (self.home_q_vector - current)
        q_dot_home = np.clip(
            q_dot_home,
            -self.max_home_posture_velocity,
            self.max_home_posture_velocity,
        )
        return null_space @ q_dot_home

    def should_reject_target(self, q_dot_raw, qdot_raw_norm, scale):
        if not self.enable_last_safe_target:
            return False

        if not np.all(np.isfinite(q_dot_raw)):
            self.log_rejected_target('qdot contains NaN/inf.', error=True)
            return True

        if qdot_raw_norm > self.maximum_raw_qdot_norm:
            self.log_rejected_target(
                f'||qdot_raw||={qdot_raw_norm:.3f} exceeds '
                f'{self.maximum_raw_qdot_norm:.3f}.'
            )
            return True

        if scale < self.minimum_allowed_scale:
            self.log_rejected_target(
                f'scale={scale:.3f} below {self.minimum_allowed_scale:.3f}.'
            )
            return True

        return False

    def log_rejected_target(self, reason, error=False):
        now = self.get_clock().now()
        if self.last_reject_log_time is not None:
            elapsed = (now - self.last_reject_log_time).nanoseconds * 1e-9
            if elapsed < 1.0:
                return

        self.last_reject_log_time = now
        message = f'Rejecting target: {reason}'
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().warn(message)

    def smooth_qdot(self, q_dot_raw):
        if not self.enable_qdot_smoothing or self.previous_q_dot is None:
            return q_dot_raw.copy()

        alpha = max(0.0, min(1.0, self.qdot_smoothing_alpha))
        return alpha * q_dot_raw + (1.0 - alpha) * self.previous_q_dot

    def scale_qdot(self, q_dot):
        max_abs_qdot = float(np.max(np.abs(q_dot))) if q_dot.size > 0 else 0.0
        scale = 1.0

        if max_abs_qdot > self.max_joint_velocity:
            scale = self.max_joint_velocity / max_abs_qdot
            q_dot = q_dot * scale

        return q_dot, scale

    def publish_visualization_frames(self):
        if self.latest_desired_object_position is None:
            return

        stamp = self.get_clock().now().to_msg()
        safe_object = getattr(
            self,
            'latest_safe_desired_object_position',
            self.latest_desired_object_position,
        )

        transforms = [
            (
                'desired_object',
                self.latest_desired_object_position,
                np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            (
                'safe_desired_object',
                safe_object,
                np.array([0.0, 0.0, 0.0, 1.0]),
            ),
            (
                'desired_left_grasp',
                self.latest_desired_left_position,
                self.initial_left_quaternion,
            ),
            (
                'desired_right_grasp',
                self.latest_desired_right_position,
                self.initial_right_quaternion,
            ),
            (
                'actual_object',
                self.latest_actual_object_position,
                np.array([0.0, 0.0, 0.0, 1.0]),
            ),
        ]

        messages = []
        for child_frame_id, position, quaternion in transforms:
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'world'
            msg.child_frame_id = child_frame_id
            msg.transform.translation.x = float(position[0])
            msg.transform.translation.y = float(position[1])
            msg.transform.translation.z = float(position[2])

            q_msg = quaternion_to_msg(quaternion)
            msg.transform.rotation.x = q_msg['x']
            msg.transform.rotation.y = q_msg['y']
            msg.transform.rotation.z = q_msg['z']
            msg.transform.rotation.w = q_msg['w']
            messages.append(msg)

        self.tf_broadcaster.sendTransform(messages)

    def log_status_if_needed(self, now):
        if not hasattr(self, 'latest_status'):
            return

        elapsed_from_log = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed_from_log < 0.5:
            return

        self.last_log_time = now

        max_abs_qdot = 0.0
        if self.latest_qdot_map:
            max_abs_qdot = max(abs(v) for v in self.latest_qdot_map.values())

        s = self.latest_status
        active_avoidance = '[]'
        if s['active_avoidance']:
            active_avoidance = '[' + ', '.join(
                f'{entry["joint"]}: margin={entry["margin"]:.3f}, '
                f'activation={entry["activation"]:.2f}, '
                f'qavoid={entry["qavoid"]:.4f}'
                for entry in s['active_avoidance']
            ) + ']'

        self.get_logger().info(
            f'rel_pos_err={s["relative_position_error_m"]:.5f} m | '
            f'obj_err={s["object_position_error_m"]:.5f} m | '
            f'rel_dist_err={s["relative_distance_error_m"]:.5f} m | '
            f'min_joint_margin={s["min_joint_margin"]:.3f} rad '
            f'({s["min_margin_joint"]}) | '
            f'warning_count={s["warning_count"]} '
            f'danger_count={s["danger_count"]} '
            f'avoidance_active_count={s["avoidance_active_count"]} | '
            f'home_bias_norm={s["home_bias_norm"]:.4f} | '
            f'qavoid_raw_norm={s["qavoid_raw_norm"]:.4f} | '
            f'qavoid_projected_norm={s["qavoid_projected_norm"]:.4f} | '
            f'max_qavoid_raw={s["max_qavoid_raw"]:.4f} | '
            f'max_qavoid_projected={s["max_qavoid_projected"]:.4f} | '
            f'qdot_raw_norm={s["qdot_raw_norm"]:.4f} | '
            f'qdot_smoothed_norm={s["qdot_smoothed_norm"]:.4f} | '
            f'scale={s["scale"]:.3f} | '
            f'target_clamped={s["target_clamped"]} | '
            f'target_rejected={s["target_rejected"]} | '
            f'max_abs_qdot={max_abs_qdot:.4f} rad/s | '
            f'active_avoidance={active_avoidance}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualSafetyPostureDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
