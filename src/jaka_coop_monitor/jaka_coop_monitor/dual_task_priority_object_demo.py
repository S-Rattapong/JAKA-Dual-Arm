import math

import numpy as np

import rclpy

from jaka_coop_monitor.dual_interactive_object_demo import DualInteractiveObjectDemo
from jaka_coop_monitor.dual_relative_motion_demo import (
    damped_pseudoinverse,
    orientation_error_rotvec,
    relative_quaternion,
)


class DualTaskPriorityObjectDemo(DualInteractiveObjectDemo):
    def __init__(self):
        if not hasattr(self, '_demo_node_name'):
            self._demo_node_name = 'dual_task_priority_object_demo'
        if not hasattr(self, '_demo_title'):
            self._demo_title = 'Dual Task Priority Object Demo'
        if not hasattr(self, '_log_trajectory_params'):
            self._log_trajectory_params = False
        super().__init__()

        self.declare_parameter('relative_position_gain', 1.5)
        self.declare_parameter('relative_orientation_gain', 1.0)
        self.declare_parameter('object_position_gain', 0.8)
        self.declare_parameter('damping_relative', 0.03)
        self.declare_parameter('damping_object', 0.03)
        self.declare_parameter('enable_relative_orientation_task', False)

        self.relative_position_gain = float(
            self.get_parameter('relative_position_gain').value
        )
        self.relative_orientation_gain = float(
            self.get_parameter('relative_orientation_gain').value
        )
        self.object_position_gain = float(
            self.get_parameter('object_position_gain').value
        )
        self.damping_relative = float(
            self.get_parameter('damping_relative').value
        )
        self.damping_object = float(
            self.get_parameter('damping_object').value
        )
        self.enable_relative_orientation_task = bool(
            self.get_parameter('enable_relative_orientation_task').value
        )

        self.desired_relative_position = None
        self.desired_relative_quaternion = None

        self.get_logger().info(
            'Task 1 preserves relative EE pose; Task 2 moves object center in '
            'the relative-task null space.'
        )

    def initialize_state(self):
        super().initialize_state()

        left_pose = self.compute_fk_and_jacobian(self.left_chain)['pose']
        right_pose = self.compute_fk_and_jacobian(self.right_chain)['pose']

        self.desired_relative_position = (
            right_pose['position'] - left_pose['position']
        )
        self.desired_relative_quaternion = relative_quaternion(
            left_pose['quaternion'],
            right_pose['quaternion']
        )

        self.get_logger().info(
            f'desired_relative_position = '
            f'[{self.desired_relative_position[0]:.5f}, '
            f'{self.desired_relative_position[1]:.5f}, '
            f'{self.desired_relative_position[2]:.5f}]'
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

        q_dot = q_dot_rel + q_dot_obj_projected

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
            'q_dot_norm': float(np.linalg.norm(q_dot)),
            'scale': scale,
        }

    def condition_number(self, singular_values):
        if len(singular_values) > 0 and singular_values[-1] > 1e-8:
            return float(singular_values[0] / singular_values[-1])

        return float('inf')

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
            f'rel_ori_err={s["relative_orientation_error_deg"]:.3f} deg | '
            f'obj_err={s["object_position_error_m"]:.5f} m | '
            f'L_err={s["left_grasp_position_error_m"]:.5f} m | '
            f'R_err={s["right_grasp_position_error_m"]:.5f} m | '
            f'rel_dist_err={s["relative_distance_error_m"]:.5f} m | '
            f'rel_rank={s["rel_rank"]} rel_cond={s["rel_condition_number"]:.3f} | '
            f'obj_rank={s["obj_rank"]} obj_cond={s["obj_condition_number"]:.3f} | '
            f'||qrel||={s["q_dot_rel_norm"]:.4f} | '
            f'||qobj_ns||={s["q_dot_obj_projected_norm"]:.4f} | '
            f'||qdot||={s["q_dot_norm"]:.4f} | '
            f'scale={s["scale"]:.3f} | '
            f'max_abs_qdot={max_abs_qdot:.4f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualTaskPriorityObjectDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
