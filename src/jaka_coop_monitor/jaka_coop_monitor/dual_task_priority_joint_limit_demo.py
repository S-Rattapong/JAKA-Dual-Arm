import math

import numpy as np

import rclpy

from jaka_coop_monitor.dual_relative_motion_demo import (
    damped_pseudoinverse,
    orientation_error_rotvec,
    relative_quaternion,
)
from jaka_coop_monitor.dual_task_priority_object_demo import (
    DualTaskPriorityObjectDemo,
)


class DualTaskPriorityJointLimitDemo(DualTaskPriorityObjectDemo):
    def __init__(self):
        self._demo_node_name = 'dual_task_priority_joint_limit_demo'
        self._demo_title = 'Dual Task Priority Joint Limit Demo'
        self._log_trajectory_params = False
        super().__init__()

        self.declare_parameter('joint_limit_warning_margin', 0.30)
        self.declare_parameter('joint_limit_danger_margin', 0.12)
        self.declare_parameter('joint_limit_avoidance_gain', 0.08)
        self.declare_parameter('max_avoidance_velocity', 0.05)
        self.declare_parameter('enable_joint_limit_avoidance', True)
        self.declare_parameter('enable_joint_limit_test_pose', False)

        self.joint_limit_warning_margin = float(
            self.get_parameter('joint_limit_warning_margin').value
        )
        self.joint_limit_danger_margin = float(
            self.get_parameter('joint_limit_danger_margin').value
        )
        self.joint_limit_avoidance_gain = float(
            self.get_parameter('joint_limit_avoidance_gain').value
        )
        self.max_avoidance_velocity = float(
            self.get_parameter('max_avoidance_velocity').value
        )
        self.enable_joint_limit_avoidance = bool(
            self.get_parameter('enable_joint_limit_avoidance').value
        )
        self.enable_joint_limit_test_pose = bool(
            self.get_parameter('enable_joint_limit_test_pose').value
        )

        self.joints_in_warning_zone = set()
        self.joints_in_danger_zone = set()

        self.get_logger().info(
            'Task 3 adds joint-limit avoidance in the relative/object null space.'
        )

    def initialize_state(self):
        if self.enable_joint_limit_test_pose:
            self.apply_joint_limit_test_pose()

        super().initialize_state()

    def apply_joint_limit_test_pose(self):
        test_joint = 'right_joint_2'
        if test_joint not in self.right_joint_names:
            return

        if test_joint not in self.joint_limits:
            self.get_logger().warn(
                f'Joint limit test pose requested, but no limits for {test_joint}.'
            )
            return

        index = self.right_joint_names.index(test_joint)
        upper = self.joint_limits[test_joint]['upper']
        target = upper - 0.12

        while len(self.initial_right_positions) <= index:
            self.initial_right_positions.append(0.0)

        self.initial_right_positions[index] = target
        self.get_logger().warn(
            f'Joint limit test pose enabled: {test_joint} initialized to '
            f'{target:.3f} rad'
        )

    def control_step(self, elapsed, dt):
        del elapsed

        left_result = self.compute_fk_and_jacobian(self.left_chain)
        right_result = self.compute_fk_and_jacobian(self.right_chain)

        left_pose = left_result['pose']
        right_pose = right_result['pose']

        desired_object = self.desired_object_position(0.0)
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

        q_dot_avoid, avoidance_status = self.compute_joint_limit_avoidance()

        N_rel_obj = N_rel @ (identity - J_obj_projected_pinv @ J_obj_projected)
        q_dot_avoid_projected = N_rel_obj @ q_dot_avoid

        q_dot = q_dot_rel + q_dot_obj_projected + q_dot_avoid_projected

        max_abs_qdot_raw = float(np.max(np.abs(q_dot))) if q_dot.size > 0 else 0.0
        scale = 1.0

        if max_abs_qdot_raw > self.max_joint_velocity:
            scale = self.max_joint_velocity / max_abs_qdot_raw
            q_dot = q_dot * scale

        self.latest_qdot_map = {}
        all_joint_names = self.left_joint_names + self.right_joint_names

        for joint_name, qd in zip(all_joint_names, q_dot):
            self.latest_qdot_map[joint_name] = float(qd)
            self.q_map[joint_name] += float(qd) * dt
            self.apply_joint_limits(joint_name)

        desired_left = desired_object + self.left_grasp_offset
        desired_right = desired_object + self.right_grasp_offset
        current_relative_distance = float(np.linalg.norm(current_relative_position))

        self.latest_desired_object_position = desired_object
        self.latest_desired_left_position = desired_left
        self.latest_desired_right_position = desired_right
        self.latest_actual_object_position = actual_object

        rel_singular_values = np.linalg.svd(J_rel, compute_uv=False)
        obj_singular_values = np.linalg.svd(J_obj_projected, compute_uv=False)

        self.latest_status = {
            'relative_position_error_m': float(np.linalg.norm(relative_position_error)),
            'relative_orientation_error_deg': math.degrees(
                relative_orientation_error_angle
            ),
            'object_position_error_m': float(np.linalg.norm(object_position_error)),
            'left_grasp_position_error_m': float(
                np.linalg.norm(desired_left - left_pose['position'])
            ),
            'right_grasp_position_error_m': float(
                np.linalg.norm(desired_right - right_pose['position'])
            ),
            'relative_distance_error_m': abs(
                current_relative_distance - self.initial_relative_distance
            ),
            'rel_rank': int(np.linalg.matrix_rank(J_rel, tol=1e-5)),
            'rel_condition_number': self.condition_number(rel_singular_values),
            'obj_rank': int(np.linalg.matrix_rank(J_obj_projected, tol=1e-5)),
            'obj_condition_number': self.condition_number(obj_singular_values),
            'q_dot_rel_norm': float(np.linalg.norm(q_dot_rel)),
            'q_dot_obj_projected_norm': float(np.linalg.norm(q_dot_obj_projected)),
            'q_dot_avoid_norm': float(np.linalg.norm(q_dot_avoid_projected)),
            'q_dot_norm': float(np.linalg.norm(q_dot)),
            'scale': scale,
            **avoidance_status,
        }

        self.warn_joint_limit_zone_changes(avoidance_status)

    def compute_joint_limit_avoidance(self):
        all_joint_names = self.left_joint_names + self.right_joint_names
        q_dot_avoid = np.zeros(len(all_joint_names))

        min_joint_margin = float('inf')
        min_margin_joint = ''
        warning_joints = []
        danger_joints = []
        active_count = 0

        if not self.enable_joint_limit_avoidance:
            return q_dot_avoid, {
                'min_joint_margin': min_joint_margin,
                'min_margin_joint': 'none',
                'warning_count': 0,
                'danger_count': 0,
                'avoidance_active_count': 0,
                'warning_joints': [],
                'danger_joints': [],
            }

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

            if min_margin < self.joint_limit_warning_margin:
                warning_joints.append(joint_name)
            if min_margin < self.joint_limit_danger_margin:
                danger_joints.append(joint_name)

            activation = self.limit_avoidance_activation(min_margin)
            if activation <= 0.0:
                continue

            direction = 1.0 if margin_lower <= margin_upper else -1.0
            velocity = direction * self.joint_limit_avoidance_gain * activation
            velocity = max(
                -self.max_avoidance_velocity,
                min(self.max_avoidance_velocity, velocity)
            )

            q_dot_avoid[i] = velocity
            active_count += 1

        if min_margin_joint == '':
            min_margin_joint = 'none'

        return q_dot_avoid, {
            'min_joint_margin': float(min_joint_margin),
            'min_margin_joint': min_margin_joint,
            'warning_count': len(warning_joints),
            'danger_count': len(danger_joints),
            'avoidance_active_count': active_count,
            'warning_joints': warning_joints,
            'danger_joints': danger_joints,
        }

    def limit_avoidance_activation(self, min_margin):
        warning = self.joint_limit_warning_margin
        danger = self.joint_limit_danger_margin

        if min_margin >= warning:
            return 0.0
        if min_margin <= danger:
            return 1.0
        if warning <= danger:
            return 1.0

        t = (warning - min_margin) / (warning - danger)
        return t * t * (3.0 - 2.0 * t)

    def warn_joint_limit_zone_changes(self, avoidance_status):
        warning_joints = set(avoidance_status['warning_joints'])
        danger_joints = set(avoidance_status['danger_joints'])

        new_warning_joints = warning_joints - self.joints_in_warning_zone
        new_danger_joints = danger_joints - self.joints_in_danger_zone

        for joint_name in sorted(new_warning_joints):
            margin = self.joint_margin(joint_name)
            self.get_logger().warn(
                f'{joint_name} entered joint-limit warning zone '
                f'(margin={margin:.3f} rad)'
            )

        for joint_name in sorted(new_danger_joints):
            margin = self.joint_margin(joint_name)
            self.get_logger().error(
                f'{joint_name} entered joint-limit danger zone '
                f'(margin={margin:.3f} rad)'
            )

        self.joints_in_warning_zone = warning_joints
        self.joints_in_danger_zone = danger_joints

    def joint_margin(self, joint_name):
        if joint_name not in self.joint_limits:
            return float('inf')

        lower = self.joint_limits[joint_name]['lower']
        upper = self.joint_limits[joint_name]['upper']
        q = self.q_map.get(joint_name, 0.0)
        return min(q - lower, upper - q)

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
        self.get_logger().info(
            f'rel_pos_err={s["relative_position_error_m"]:.5f} m | '
            f'obj_err={s["object_position_error_m"]:.5f} m | '
            f'rel_dist_err={s["relative_distance_error_m"]:.5f} m | '
            f'min_margin={s["min_joint_margin"]:.3f} rad '
            f'({s["min_margin_joint"]}) | '
            f'warn={s["warning_count"]} danger={s["danger_count"]} '
            f'avoid_active={s["avoidance_active_count"]} | '
            f'||qrel||={s["q_dot_rel_norm"]:.4f} | '
            f'||qobj_ns||={s["q_dot_obj_projected_norm"]:.4f} | '
            f'||qavoid||={s["q_dot_avoid_norm"]:.4f} | '
            f'||qdot||={s["q_dot_norm"]:.4f} | '
            f'scale={s["scale"]:.3f} | '
            f'max_abs_qdot={max_abs_qdot:.4f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualTaskPriorityJointLimitDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
