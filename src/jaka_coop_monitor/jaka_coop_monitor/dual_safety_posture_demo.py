import math

import numpy as np

import rclpy
from geometry_msgs.msg import Point, Pose, TransformStamped
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

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

        self.declare_parameter('object_min_x', -0.5)
        self.declare_parameter('object_max_x', 2.0)
        self.declare_parameter('object_min_y', -1.0)
        self.declare_parameter('object_max_y', 1.0)
        self.declare_parameter('object_min_z', 0.05)
        self.declare_parameter('object_max_z', 2.0)
        self.declare_parameter('max_object_target_speed', 0.50)

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

        self.declare_parameter('enable_collision_avoidance', True)
        self.declare_parameter('collision_warning_distance', 0.25)
        self.declare_parameter('collision_danger_distance', 0.15)
        self.declare_parameter('collision_stop_distance', 0.08)
        self.declare_parameter('collision_link_radius', 0.04)
        self.declare_parameter('collision_scale_min', 0.10)
        self.declare_parameter('collision_reject_on_danger', True)
        self.declare_parameter('collision_debug_log', True)

        self.declare_parameter('enable_self_collision_avoidance', True)
        self.declare_parameter('self_collision_warning_distance', 0.20)
        self.declare_parameter('self_collision_danger_distance', 0.12)
        self.declare_parameter('self_collision_stop_distance', 0.07)
        self.declare_parameter('self_collision_link_radius', 0.035)
        self.declare_parameter('self_collision_scale_min', 0.10)
        self.declare_parameter('self_collision_reject_on_danger', True)
        self.declare_parameter('self_collision_min_index_gap', 2)
        self.declare_parameter('self_collision_debug_log', True)

        self.declare_parameter('collision_hard_stop_enabled', True)
        self.declare_parameter('reset_qdot_smoothing_on_collision_stop', True)
        self.declare_parameter('collision_recovery_enabled', True)
        self.declare_parameter('collision_recovery_scale', 0.15)
        self.declare_parameter('collision_recovery_max_qdot', 0.03)
        self.declare_parameter('collision_recovery_min_improvement', 0.002)
        self.declare_parameter('collision_recovery_debug_log', True)

        self.declare_parameter('enable_object_yaw', False)
        self.declare_parameter('object_yaw_deg', 0.0)
        self.declare_parameter('enable_object_yaw_sine', False)
        self.declare_parameter('object_yaw_amplitude_deg', 20.0)
        self.declare_parameter('object_yaw_frequency_hz', 0.05)
        self.declare_parameter('object_yaw_debug_log', True)

        self.declare_parameter('enable_interactive_object_yaw', True)
        self.declare_parameter('interactive_yaw_debug_log', True)
        self.declare_parameter('max_object_yaw_rate_deg_s', 30.0)
        self.declare_parameter('enable_object_yaw_rate_limit', True)

        self.declare_parameter('enable_ee_yaw_orientation_control', False)
        self.declare_parameter('ee_yaw_orientation_gain', 0.6)
        self.declare_parameter('max_ee_yaw_angular_speed', 0.25)
        self.declare_parameter('ee_yaw_task_damping', 0.05)
        self.declare_parameter('ee_yaw_debug_log', True)

        self.declare_parameter('enable_full_ee_orientation_control', False)
        self.declare_parameter('full_ee_orientation_gain', 0.35)
        self.declare_parameter('max_ee_angular_speed', 0.15)
        self.declare_parameter('full_ee_orientation_task_damping', 0.08)
        self.declare_parameter('full_ee_orientation_debug_log', True)
        self.declare_parameter('prefer_full_orientation_over_yaw_only', True)

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

        self.enable_collision_avoidance = bool(
            self.get_parameter('enable_collision_avoidance').value
        )
        self.collision_warning_distance = float(
            self.get_parameter('collision_warning_distance').value
        )
        self.collision_danger_distance = float(
            self.get_parameter('collision_danger_distance').value
        )
        self.collision_stop_distance = float(
            self.get_parameter('collision_stop_distance').value
        )
        self.collision_link_radius = max(
            0.0,
            float(self.get_parameter('collision_link_radius').value),
        )
        self.collision_scale_min = max(
            0.0,
            min(1.0, float(self.get_parameter('collision_scale_min').value)),
        )
        self.collision_reject_on_danger = bool(
            self.get_parameter('collision_reject_on_danger').value
        )
        self.collision_debug_log = bool(
            self.get_parameter('collision_debug_log').value
        )

        self.enable_self_collision_avoidance = bool(
            self.get_parameter('enable_self_collision_avoidance').value
        )
        self.self_collision_warning_distance = float(
            self.get_parameter('self_collision_warning_distance').value
        )
        self.self_collision_danger_distance = float(
            self.get_parameter('self_collision_danger_distance').value
        )
        self.self_collision_stop_distance = float(
            self.get_parameter('self_collision_stop_distance').value
        )
        self.self_collision_link_radius = max(
            0.0,
            float(self.get_parameter('self_collision_link_radius').value),
        )
        self.self_collision_scale_min = max(
            0.0,
            min(1.0, float(self.get_parameter('self_collision_scale_min').value)),
        )
        self.self_collision_reject_on_danger = bool(
            self.get_parameter('self_collision_reject_on_danger').value
        )
        self.self_collision_min_index_gap = max(
            0,
            int(self.get_parameter('self_collision_min_index_gap').value),
        )
        self.self_collision_debug_log = bool(
            self.get_parameter('self_collision_debug_log').value
        )

        self.collision_hard_stop_enabled = bool(
            self.get_parameter('collision_hard_stop_enabled').value
        )
        self.reset_qdot_smoothing_on_collision_stop = bool(
            self.get_parameter('reset_qdot_smoothing_on_collision_stop').value
        )
        self.collision_recovery_enabled = bool(
            self.get_parameter('collision_recovery_enabled').value
        )
        self.collision_recovery_scale = max(
            0.0,
            float(self.get_parameter('collision_recovery_scale').value),
        )
        self.collision_recovery_max_qdot = max(
            0.0,
            float(self.get_parameter('collision_recovery_max_qdot').value),
        )
        self.collision_recovery_min_improvement = max(
            0.0,
            float(self.get_parameter('collision_recovery_min_improvement').value),
        )
        self.collision_recovery_debug_log = bool(
            self.get_parameter('collision_recovery_debug_log').value
        )

        self.enable_object_yaw = bool(
            self.get_parameter('enable_object_yaw').value
        )
        self.object_yaw_deg = float(self.get_parameter('object_yaw_deg').value)
        self.enable_object_yaw_sine = bool(
            self.get_parameter('enable_object_yaw_sine').value
        )
        self.object_yaw_amplitude_deg = float(
            self.get_parameter('object_yaw_amplitude_deg').value
        )
        self.object_yaw_frequency_hz = float(
            self.get_parameter('object_yaw_frequency_hz').value
        )
        self.object_yaw_debug_log = bool(
            self.get_parameter('object_yaw_debug_log').value
        )

        self.enable_interactive_object_yaw = bool(
            self.get_parameter('enable_interactive_object_yaw').value
        )
        self.interactive_yaw_debug_log = bool(
            self.get_parameter('interactive_yaw_debug_log').value
        )
        self.max_object_yaw_rate_deg_s = max(
            0.0,
            float(self.get_parameter('max_object_yaw_rate_deg_s').value),
        )
        self.enable_object_yaw_rate_limit = bool(
            self.get_parameter('enable_object_yaw_rate_limit').value
        )
        self.enable_ee_yaw_orientation_control = bool(
            self.get_parameter('enable_ee_yaw_orientation_control').value
        )
        self.ee_yaw_orientation_gain = max(
            0.0,
            float(self.get_parameter('ee_yaw_orientation_gain').value),
        )
        self.max_ee_yaw_angular_speed = max(
            0.0,
            float(self.get_parameter('max_ee_yaw_angular_speed').value),
        )
        self.ee_yaw_task_damping = max(
            0.0,
            float(self.get_parameter('ee_yaw_task_damping').value),
        )
        self.ee_yaw_debug_log = bool(
            self.get_parameter('ee_yaw_debug_log').value
        )
        self.enable_full_ee_orientation_control = bool(
            self.get_parameter('enable_full_ee_orientation_control').value
        )
        self.full_ee_orientation_gain = max(
            0.0,
            float(self.get_parameter('full_ee_orientation_gain').value),
        )
        self.max_ee_angular_speed = max(
            0.0,
            float(self.get_parameter('max_ee_angular_speed').value),
        )
        self.full_ee_orientation_task_damping = max(
            0.0,
            float(self.get_parameter('full_ee_orientation_task_damping').value),
        )
        self.full_ee_orientation_debug_log = bool(
            self.get_parameter('full_ee_orientation_debug_log').value
        )
        self.prefer_full_orientation_over_yaw_only = bool(
            self.get_parameter('prefer_full_orientation_over_yaw_only').value
        )

        self.raw_desired_object_position = None
        self.safe_desired_object_position = None
        self.last_safe_desired_object_position = None
        self.previous_q_dot = None
        self.home_q_vector = None
        self.target_was_clamped = False
        self.last_reject_log_time = None
        self.last_hard_stop_warning_time = None
        self.latest_collision_status = self.empty_collision_status()
        self.initial_desired_relative_position_for_yaw = None
        self.latest_object_yaw_deg = 0.0
        self.latest_desired_relative_position = None
        self.raw_desired_object_yaw = 0.0
        self.safe_desired_object_yaw = 0.0
        self.last_safe_desired_object_yaw = 0.0
        self.latest_raw_object_yaw = 0.0
        self.latest_safe_object_yaw = 0.0
        self.latest_yaw_delta = 0.0
        self.object_yaw_rate_limited = False
        self.latest_desired_left_quaternion = None
        self.latest_desired_right_quaternion = None
        self.collision_marker_pub = self.create_publisher(
            MarkerArray,
            '/collision_debug_markers',
            10,
        )

        self.get_logger().info(
            'Safety posture demo: raw desired_object is the marker target; '
            'safe_desired_object is the filtered/clamped target used for control.'
        )

        if self.debug_joint_limit_raw_avoidance:
            self.get_logger().warn(
                'debug_joint_limit_raw_avoidance is enabled: joint-limit '
                'avoidance will bypass null-space projection.'
            )

        self.get_logger().info(
            'collision avoidance: '
            f'enabled={self.enable_collision_avoidance}, '
            f'warning={self.collision_warning_distance:.3f} m, '
            f'danger={self.collision_danger_distance:.3f} m, '
            f'stop={self.collision_stop_distance:.3f} m, '
            f'link_radius={self.collision_link_radius:.3f} m'
        )
        self.get_logger().info(
            'self-collision avoidance: '
            f'enabled={self.enable_self_collision_avoidance}, '
            f'warning={self.self_collision_warning_distance:.3f} m, '
            f'danger={self.self_collision_danger_distance:.3f} m, '
            f'stop={self.self_collision_stop_distance:.3f} m, '
            f'link_radius={self.self_collision_link_radius:.3f} m, '
            f'min_index_gap={self.self_collision_min_index_gap}'
        )
        self.get_logger().info(
            'collision hard stop/recovery: '
            f'hard_stop={self.collision_hard_stop_enabled}, '
            f'reset_smoothing={self.reset_qdot_smoothing_on_collision_stop}, '
            f'recovery={self.collision_recovery_enabled}, '
            f'recovery_scale={self.collision_recovery_scale:.3f}, '
            f'recovery_max_qdot={self.collision_recovery_max_qdot:.3f} rad/s, '
            f'min_improvement='
            f'{self.collision_recovery_min_improvement:.4f} m'
        )
        self.get_logger().info(
            'object yaw: '
            f'enabled={self.enable_object_yaw}, '
            f'static_yaw={self.object_yaw_deg:.3f} deg, '
            f'sine={self.enable_object_yaw_sine}, '
            f'amplitude={self.object_yaw_amplitude_deg:.3f} deg, '
            f'frequency={self.object_yaw_frequency_hz:.4f} Hz'
        )
        self.get_logger().info(
            'interactive object yaw: '
            f'enabled={self.enable_interactive_object_yaw}, '
            f'rate_limit={self.enable_object_yaw_rate_limit}, '
            f'max_rate={self.max_object_yaw_rate_deg_s:.3f} deg/s'
        )
        self.get_logger().info(
            'EE yaw orientation control: '
            f'enabled={self.enable_ee_yaw_orientation_control}, '
            f'gain={self.ee_yaw_orientation_gain:.3f}, '
            f'max_wz={self.max_ee_yaw_angular_speed:.3f} rad/s, '
            f'damping={self.ee_yaw_task_damping:.4f}'
        )
        self.get_logger().info(
            'full EE orientation control: '
            f'enabled={self.enable_full_ee_orientation_control}, '
            f'gain={self.full_ee_orientation_gain:.3f}, '
            f'max_omega={self.max_ee_angular_speed:.3f} rad/s, '
            f'damping={self.full_ee_orientation_task_damping:.4f}, '
            f'prefer_full_over_yaw='
            f'{self.prefer_full_orientation_over_yaw_only}'
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
        self.initial_desired_relative_position_for_yaw = (
            self.desired_relative_position.copy()
        )
        self.latest_desired_relative_position = (
            self.desired_relative_position.copy()
        )
        self.latest_object_yaw_deg = 0.0
        self.raw_desired_object_yaw = 0.0
        self.safe_desired_object_yaw = 0.0
        self.last_safe_desired_object_yaw = 0.0
        self.latest_raw_object_yaw = 0.0
        self.latest_safe_object_yaw = 0.0
        self.latest_yaw_delta = 0.0
        self.object_yaw_rate_limited = False
        self.latest_desired_left_quaternion = self.initial_left_quaternion.copy()
        self.latest_desired_right_quaternion = self.initial_right_quaternion.copy()

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
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return

        self.raw_desired_object_position = np.array(
            [
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z,
            ],
            dtype=float,
        )
        self.target_object_position = self.raw_desired_object_position.copy()
        if self.enable_interactive_object_yaw:
            self.raw_desired_object_yaw = self.yaw_from_quaternion_msg(
                feedback.pose.orientation
            )
            self.latest_raw_object_yaw = self.raw_desired_object_yaw

    def create_object_marker(self):
        if self.marker_created:
            return

        marker = InteractiveMarker()
        marker.header.frame_id = 'world'
        marker.name = 'desired_object'
        marker.description = 'desired_object'
        marker.scale = max(self.marker_scale * 2.5, 0.05)
        marker.pose = self.pose_from_position_and_yaw(
            self.target_object_position,
            self.raw_desired_object_yaw,
        )

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True

        object_marker = Marker()
        object_marker.type = Marker.SPHERE
        object_marker.scale.x = self.marker_scale
        object_marker.scale.y = self.marker_scale
        object_marker.scale.z = self.marker_scale
        object_marker.color.r = 0.1
        object_marker.color.g = 0.8
        object_marker.color.b = 1.0
        object_marker.color.a = 0.75

        visual_control.markers.append(object_marker)
        marker.controls.append(visual_control)

        marker.controls.append(self.make_move_axis_control('move_x', 1.0, 0.0, 0.0))
        marker.controls.append(self.make_move_axis_control('move_y', 0.0, 1.0, 0.0))
        marker.controls.append(self.make_move_axis_control('move_z', 0.0, 0.0, 1.0))

        if self.enable_interactive_object_yaw:
            marker.controls.append(self.make_rotate_z_control())

        self.marker_server.insert(marker, feedback_callback=self.process_marker_feedback)
        self.marker_server.applyChanges()
        self.marker_created = True

    def make_rotate_z_control(self):
        control = InteractiveMarkerControl()
        control.name = 'rotate_z_yaw'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        return control

    def pose_from_position_and_yaw(self, position, yaw):
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        q = self.yaw_quaternion(yaw)
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose

    def desired_object_position(self, elapsed):
        del elapsed

        if self.safe_desired_object_position is not None:
            return self.safe_desired_object_position.copy()

        if self.initial_object_position is not None:
            return self.initial_object_position.copy()

        return np.zeros(3, dtype=float)

    def current_object_yaw_rad(self, elapsed):
        if self.enable_interactive_object_yaw:
            return self.safe_desired_object_yaw

        if not self.enable_object_yaw:
            return 0.0

        yaw = math.radians(self.object_yaw_deg)
        if self.enable_object_yaw_sine:
            yaw += math.radians(self.object_yaw_amplitude_deg) * math.sin(
                2.0 * math.pi * self.object_yaw_frequency_hz * elapsed
            )
        return yaw

    def update_safe_object_yaw(self, elapsed, dt):
        if not self.enable_interactive_object_yaw:
            yaw = self.current_object_yaw_rad(elapsed)
            self.raw_desired_object_yaw = yaw
            self.safe_desired_object_yaw = yaw
            self.last_safe_desired_object_yaw = yaw
            self.latest_raw_object_yaw = yaw
            self.latest_safe_object_yaw = yaw
            self.latest_yaw_delta = 0.0
            self.object_yaw_rate_limited = False
            return

        raw_yaw = self.wrap_angle(self.raw_desired_object_yaw)
        current_safe_yaw = self.wrap_angle(self.safe_desired_object_yaw)
        delta = self.shortest_angle_delta(raw_yaw, current_safe_yaw)
        limited = False

        if self.enable_object_yaw_rate_limit:
            max_step = math.radians(self.max_object_yaw_rate_deg_s) * max(dt, 0.0)
            if abs(delta) > max_step:
                delta = self.clamp(delta, -max_step, max_step)
                limited = True

        self.safe_desired_object_yaw = self.wrap_angle(current_safe_yaw + delta)
        self.last_safe_desired_object_yaw = self.safe_desired_object_yaw
        self.latest_raw_object_yaw = raw_yaw
        self.latest_safe_object_yaw = self.safe_desired_object_yaw
        self.latest_yaw_delta = delta
        self.object_yaw_rate_limited = limited

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def shortest_angle_delta(self, target, current):
        return self.wrap_angle(target - current)

    def yaw_from_quaternion_msg(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_from_quaternion_array(self, q):
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_quaternion(self, yaw):
        half = 0.5 * yaw
        return np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=float)

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array(
            [
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ],
            dtype=float,
        )

    def limit_vector_norm(self, vector, max_norm):
        norm = float(np.linalg.norm(vector))
        if norm > max_norm and norm > 1e-12:
            return vector * (max_norm / norm)
        return vector

    def desired_grasp_quaternions(self):
        if (
            not self.enable_object_yaw
            and not self.enable_interactive_object_yaw
        ):
            return (
                self.initial_left_quaternion,
                self.initial_right_quaternion,
            )

        q_yaw = self.yaw_quaternion(self.safe_desired_object_yaw)
        return (
            self.quaternion_multiply(q_yaw, self.initial_left_quaternion),
            self.quaternion_multiply(q_yaw, self.initial_right_quaternion),
        )

    def current_desired_relative_position(self, elapsed):
        if self.initial_desired_relative_position_for_yaw is None:
            return self.desired_relative_position.copy()

        if not self.enable_object_yaw and not self.enable_interactive_object_yaw:
            self.latest_object_yaw_deg = 0.0
            self.latest_desired_relative_position = (
                self.desired_relative_position.copy()
            )
            return self.desired_relative_position.copy()

        yaw = self.current_object_yaw_rad(elapsed)
        c = math.cos(yaw)
        s = math.sin(yaw)
        rotation_z = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        desired_relative_position = (
            rotation_z @ self.initial_desired_relative_position_for_yaw
        )
        self.latest_object_yaw_deg = math.degrees(yaw)
        self.latest_desired_relative_position = desired_relative_position.copy()
        return desired_relative_position

    def desired_grasp_positions(self, desired_object, desired_relative_position):
        if self.enable_object_yaw or self.enable_interactive_object_yaw:
            return (
                desired_object - 0.5 * desired_relative_position,
                desired_object + 0.5 * desired_relative_position,
            )

        return (
            desired_object + self.left_grasp_offset,
            desired_object + self.right_grasp_offset,
        )

    def compute_ee_yaw_task(
        self,
        left_pose,
        right_pose,
        J_left,
        J_right,
        N_rel_obj,
    ):
        size = len(self.left_joint_names) + len(self.right_joint_names)
        desired_left_quaternion, desired_right_quaternion = (
            self.desired_grasp_quaternions()
        )
        status = {
            'ee_yaw_control_enabled': self.enable_ee_yaw_orientation_control,
            'left_ee_yaw_error_deg': 0.0,
            'right_ee_yaw_error_deg': 0.0,
            'left_ee_wz_des': 0.0,
            'right_ee_wz_des': 0.0,
            'qdot_ee_yaw_norm': 0.0,
            'desired_left_quaternion': desired_left_quaternion.copy(),
            'desired_right_quaternion': desired_right_quaternion.copy(),
        }

        yaw_suppressed_by_full_orientation = (
            self.enable_full_ee_orientation_control
            and self.prefer_full_orientation_over_yaw_only
        )
        if (
            not self.enable_ee_yaw_orientation_control
            or yaw_suppressed_by_full_orientation
        ):
            status['ee_yaw_control_enabled'] = False
            return np.zeros(size, dtype=float), status

        left_current_yaw = self.yaw_from_quaternion_array(left_pose['quaternion'])
        right_current_yaw = self.yaw_from_quaternion_array(right_pose['quaternion'])
        left_desired_yaw = self.yaw_from_quaternion_array(desired_left_quaternion)
        right_desired_yaw = self.yaw_from_quaternion_array(desired_right_quaternion)

        left_yaw_error = self.wrap_angle(left_desired_yaw - left_current_yaw)
        right_yaw_error = self.wrap_angle(right_desired_yaw - right_current_yaw)

        left_wz_des = self.ee_yaw_orientation_gain * left_yaw_error
        right_wz_des = self.ee_yaw_orientation_gain * right_yaw_error
        left_wz_des = float(
            np.clip(
                left_wz_des,
                -self.max_ee_yaw_angular_speed,
                self.max_ee_yaw_angular_speed,
            )
        )
        right_wz_des = float(
            np.clip(
                right_wz_des,
                -self.max_ee_yaw_angular_speed,
                self.max_ee_yaw_angular_speed,
            )
        )

        J_yaw = np.zeros((2, size), dtype=float)
        J_yaw[0, 0:len(self.left_joint_names)] = J_left[5, :]
        J_yaw[1, len(self.left_joint_names):size] = J_right[5, :]
        x_dot_yaw = np.array([left_wz_des, right_wz_des], dtype=float)

        J_yaw_projected = J_yaw @ N_rel_obj
        J_yaw_projected_pinv = damped_pseudoinverse(
            J_yaw_projected,
            self.ee_yaw_task_damping,
        )
        q_dot_yaw_projected = N_rel_obj @ (
            J_yaw_projected_pinv @ x_dot_yaw
        )

        status.update(
            {
                'left_ee_yaw_error_deg': math.degrees(left_yaw_error),
                'right_ee_yaw_error_deg': math.degrees(right_yaw_error),
                'left_ee_wz_des': left_wz_des,
                'right_ee_wz_des': right_wz_des,
                'qdot_ee_yaw_norm': float(np.linalg.norm(q_dot_yaw_projected)),
            }
        )
        return q_dot_yaw_projected, status

    def compute_full_ee_orientation_task(
        self,
        left_pose,
        right_pose,
        J_left,
        J_right,
        N_rel_obj,
    ):
        size = len(self.left_joint_names) + len(self.right_joint_names)
        desired_left_quaternion, desired_right_quaternion = (
            self.desired_grasp_quaternions()
        )
        status = {
            'full_ee_orientation_control_enabled': (
                self.enable_full_ee_orientation_control
            ),
            'left_full_ori_error_deg': 0.0,
            'right_full_ori_error_deg': 0.0,
            'left_omega_des_norm': 0.0,
            'right_omega_des_norm': 0.0,
            'qdot_full_ori_norm': 0.0,
            'full_ori_task_rank': 0,
            'full_ori_task_condition': float('inf'),
            'desired_left_quaternion': desired_left_quaternion.copy(),
            'desired_right_quaternion': desired_right_quaternion.copy(),
        }

        if not self.enable_full_ee_orientation_control:
            return np.zeros(size, dtype=float), status

        left_rotvec_error, left_orientation_error_angle = orientation_error_rotvec(
            left_pose['quaternion'],
            desired_left_quaternion,
        )
        right_rotvec_error, right_orientation_error_angle = orientation_error_rotvec(
            right_pose['quaternion'],
            desired_right_quaternion,
        )

        left_omega_des = self.full_ee_orientation_gain * left_rotvec_error
        right_omega_des = self.full_ee_orientation_gain * right_rotvec_error
        left_omega_des = self.limit_vector_norm(
            left_omega_des,
            self.max_ee_angular_speed,
        )
        right_omega_des = self.limit_vector_norm(
            right_omega_des,
            self.max_ee_angular_speed,
        )

        J_ori = np.zeros((6, size), dtype=float)
        J_ori[0:3, 0:len(self.left_joint_names)] = J_left[3:6, :]
        J_ori[3:6, len(self.left_joint_names):size] = J_right[3:6, :]
        x_dot_ori = np.hstack((left_omega_des, right_omega_des))

        J_ori_projected = J_ori @ N_rel_obj
        J_ori_projected_pinv = damped_pseudoinverse(
            J_ori_projected,
            self.full_ee_orientation_task_damping,
        )
        q_dot_full_ori_projected = N_rel_obj @ (
            J_ori_projected_pinv @ x_dot_ori
        )
        ori_singular_values = np.linalg.svd(
            J_ori_projected,
            compute_uv=False,
        )

        status.update(
            {
                'left_full_ori_error_deg': math.degrees(
                    left_orientation_error_angle
                ),
                'right_full_ori_error_deg': math.degrees(
                    right_orientation_error_angle
                ),
                'left_omega_des_norm': float(np.linalg.norm(left_omega_des)),
                'right_omega_des_norm': float(np.linalg.norm(right_omega_des)),
                'qdot_full_ori_norm': float(
                    np.linalg.norm(q_dot_full_ori_projected)
                ),
                'full_ori_task_rank': int(
                    np.linalg.matrix_rank(J_ori_projected, tol=1e-5)
                ),
                'full_ori_task_condition': self.condition_number(
                    ori_singular_values
                ),
            }
        )
        return q_dot_full_ori_projected, status

    def control_step(self, elapsed, dt):
        previous_safe_yaw = self.safe_desired_object_yaw
        self.update_safe_object_yaw(elapsed, dt)

        raw_target = self.current_raw_target()
        clamped_target, target_clamped = self.clamp_object_target(raw_target)
        desired_relative_position = self.current_desired_relative_position(elapsed)

        if target_clamped and not self.target_was_clamped:
            self.log_target_clamped(raw_target, clamped_target)
        self.target_was_clamped = target_clamped

        desired_object = self.limit_target_speed(
            self.safe_desired_object_position,
            clamped_target,
            dt,
        )

        command = self.compute_safe_command(
            desired_object,
            desired_relative_position,
        )
        q_dot_raw = command['q_dot']
        qdot_raw_norm = float(np.linalg.norm(q_dot_raw))
        collision_status = self.compute_collision_status()

        max_abs_qdot_raw = (
            float(np.max(np.abs(q_dot_raw))) if q_dot_raw.size > 0 else 0.0
        )
        raw_scale = 1.0
        if max_abs_qdot_raw > self.max_joint_velocity:
            raw_scale = self.max_joint_velocity / max_abs_qdot_raw

        target_rejected = self.should_reject_target(q_dot_raw, qdot_raw_norm, raw_scale)
        collision_recovery_needed = self.collision_recovery_needed(
            collision_status
        )
        allow_collision_recovery_candidate = (
            self.collision_recovery_enabled and collision_recovery_needed
        )
        if allow_collision_recovery_candidate:
            target_rejected_by_inter_collision = False
            target_rejected_by_self_collision = False
        else:
            target_rejected_by_inter_collision = (
                self.should_reject_for_inter_collision(collision_status)
            )
            target_rejected_by_self_collision = (
                self.should_reject_for_self_collision(collision_status)
            )
        target_rejected_by_collision = (
            target_rejected_by_inter_collision
            or target_rejected_by_self_collision
        )
        if (
            (target_rejected or target_rejected_by_collision)
            and self.last_safe_desired_object_position is not None
        ):
            desired_object = self.last_safe_desired_object_position.copy()
            command = self.compute_safe_command(
                desired_object,
                desired_relative_position,
            )
            q_dot_raw = command['q_dot']
            qdot_raw_norm = float(np.linalg.norm(q_dot_raw))
        elif not allow_collision_recovery_candidate:
            self.last_safe_desired_object_position = desired_object.copy()

        q_dot_smoothed = self.smooth_qdot(q_dot_raw)
        qdot_smoothed_norm = float(np.linalg.norm(q_dot_smoothed))

        q_dot, scale = self.scale_qdot(q_dot_smoothed)
        recovery_status = self.evaluate_collision_recovery(
            q_dot,
            collision_status,
            dt,
        )
        q_dot = self.apply_collision_safety(
            q_dot,
            collision_status,
            recovery_status,
        )
        self.previous_q_dot = q_dot.copy()

        self.latest_qdot_map = {}
        all_joint_names = self.left_joint_names + self.right_joint_names
        q_dot = self.apply_collision_final_guard(
            q_dot,
            collision_status,
            recovery_status,
        )
        self.previous_q_dot = q_dot.copy()
        for joint_name, qd in zip(all_joint_names, q_dot):
            self.latest_qdot_map[joint_name] = float(qd)
            self.q_map[joint_name] += float(qd) * dt
            self.apply_joint_limits(joint_name)

        if (
            recovery_status['collision_hard_stop_active']
            and not recovery_status['collision_recovery_allowed']
            and self.safe_desired_object_position is not None
        ):
            desired_object = self.safe_desired_object_position.copy()
            self.safe_desired_object_yaw = previous_safe_yaw
            self.latest_safe_object_yaw = previous_safe_yaw
            desired_relative_position = self.current_desired_relative_position(elapsed)
            command = self.compute_safe_command(
                desired_object,
                desired_relative_position,
            )
        else:
            self.last_safe_desired_object_position = desired_object.copy()
            self.last_safe_desired_object_yaw = self.safe_desired_object_yaw

        left_pose = command['left_pose']
        right_pose = command['right_pose']
        actual_object = command['actual_object']
        current_relative_position = command['current_relative_position']
        current_relative_distance = float(np.linalg.norm(current_relative_position))

        desired_left, desired_right = self.desired_grasp_positions(
            desired_object,
            command['desired_relative_position'],
        )
        ee_yaw_status = command['ee_yaw_status']
        full_ee_orientation_status = command['full_ee_orientation_status']
        desired_left_quaternion = (
            full_ee_orientation_status['desired_left_quaternion']
        )
        desired_right_quaternion = (
            full_ee_orientation_status['desired_right_quaternion']
        )

        self.safe_desired_object_position = desired_object.copy()
        self.latest_raw_desired_object_position = raw_target.copy()
        self.latest_safe_desired_object_position = desired_object.copy()
        self.latest_desired_object_position = raw_target.copy()
        self.latest_desired_left_position = desired_left.copy()
        self.latest_desired_right_position = desired_right.copy()
        self.latest_desired_left_quaternion = desired_left_quaternion.copy()
        self.latest_desired_right_quaternion = desired_right_quaternion.copy()
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
            'ee_yaw_control_enabled': ee_yaw_status['ee_yaw_control_enabled'],
            'left_ee_yaw_error_deg': ee_yaw_status['left_ee_yaw_error_deg'],
            'right_ee_yaw_error_deg': ee_yaw_status['right_ee_yaw_error_deg'],
            'left_ee_wz_des': ee_yaw_status['left_ee_wz_des'],
            'right_ee_wz_des': ee_yaw_status['right_ee_wz_des'],
            'qdot_ee_yaw_norm': ee_yaw_status['qdot_ee_yaw_norm'],
            'full_ee_orientation_control_enabled': (
                full_ee_orientation_status[
                    'full_ee_orientation_control_enabled'
                ]
            ),
            'left_full_ori_error_deg': (
                full_ee_orientation_status['left_full_ori_error_deg']
            ),
            'right_full_ori_error_deg': (
                full_ee_orientation_status['right_full_ori_error_deg']
            ),
            'left_omega_des_norm': (
                full_ee_orientation_status['left_omega_des_norm']
            ),
            'right_omega_des_norm': (
                full_ee_orientation_status['right_omega_des_norm']
            ),
            'qdot_full_ori_norm': (
                full_ee_orientation_status['qdot_full_ori_norm']
            ),
            'full_ori_task_rank': (
                full_ee_orientation_status['full_ori_task_rank']
            ),
            'full_ori_task_condition': (
                full_ee_orientation_status['full_ori_task_condition']
            ),
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
            'target_rejected_by_collision': target_rejected_by_collision,
            'target_rejected_by_self_collision': (
                target_rejected_by_self_collision
            ),
            'object_yaw_enabled': (
                self.enable_object_yaw or self.enable_interactive_object_yaw
            ),
            'interactive_object_yaw_enabled': self.enable_interactive_object_yaw,
            'object_yaw_deg_current': command['object_yaw_deg_current'],
            'raw_object_yaw_deg': math.degrees(self.latest_raw_object_yaw),
            'safe_object_yaw_deg': math.degrees(self.latest_safe_object_yaw),
            'object_yaw_delta_deg': math.degrees(self.latest_yaw_delta),
            'object_yaw_rate_limited': self.object_yaw_rate_limited,
            'desired_relative_position': command['desired_relative_position'],
            **self.collision_status_for_log(collision_status),
            **recovery_status,
        }

        self.latest_collision_status = collision_status
        self.publish_collision_debug_markers(collision_status)
        self.warn_joint_limit_zone_changes(command['avoidance_status'])

    def empty_collision_status(self):
        return {
            'enabled': False,
            'min_distance': float('inf'),
            'raw_distance': float('inf'),
            'closest_left': 'none',
            'closest_right': 'none',
            'closest_left_point': None,
            'closest_right_point': None,
            'warning_count': 0,
            'danger_count': 0,
            'stop_active': False,
            'scale': 1.0,
            'self_enabled': False,
            'self_min_distance': float('inf'),
            'self_raw_distance': float('inf'),
            'self_arm': 'none',
            'self_closest_a': 'none',
            'self_closest_b': 'none',
            'self_closest_a_point': None,
            'self_closest_b_point': None,
            'self_warning_count': 0,
            'self_danger_count': 0,
            'self_stop_active': False,
            'self_scale': 1.0,
        }

    def compute_collision_status(self, q_map=None):
        if (
            not self.enable_collision_avoidance
            and not self.enable_self_collision_avoidance
        ):
            return self.empty_collision_status()

        left_segments = self.build_arm_segments(self.left_chain, q_map)
        right_segments = self.build_arm_segments(self.right_chain, q_map)
        status = self.empty_collision_status()
        status['enabled'] = self.enable_collision_avoidance
        status['self_enabled'] = self.enable_self_collision_avoidance

        if self.enable_collision_avoidance:
            for left_name, left_start, left_end in left_segments:
                for right_name, right_start, right_end in right_segments:
                    if self.skip_collision_pair(left_name, right_name):
                        continue

                    raw_distance, left_point, right_point = self.segment_distance(
                        left_start,
                        left_end,
                        right_start,
                        right_end,
                    )
                    effective_distance = (
                        raw_distance - 2.0 * self.collision_link_radius
                    )

                    if effective_distance <= self.collision_warning_distance:
                        status['warning_count'] += 1
                    if effective_distance <= self.collision_danger_distance:
                        status['danger_count'] += 1

                    if effective_distance < status['min_distance']:
                        status['min_distance'] = float(effective_distance)
                        status['raw_distance'] = float(raw_distance)
                        status['closest_left'] = left_name
                        status['closest_right'] = right_name
                        status['closest_left_point'] = left_point.copy()
                        status['closest_right_point'] = right_point.copy()

            status['stop_active'] = (
                status['min_distance'] <= self.collision_stop_distance
            )
            status['scale'] = self.collision_scale(status['min_distance'])

        if self.enable_self_collision_avoidance:
            self.compute_self_collision_distance('left', left_segments, status)
            self.compute_self_collision_distance('right', right_segments, status)
            status['self_stop_active'] = (
                status['self_min_distance'] <= self.self_collision_stop_distance
            )
            status['self_scale'] = self.self_collision_scale(
                status['self_min_distance']
            )

        return status

    def compute_self_collision_distance(self, arm_name, segments, status):
        for i, (name_a, start_a, end_a) in enumerate(segments):
            for j in range(i + 1, len(segments)):
                if abs(i - j) <= self.self_collision_min_index_gap:
                    continue

                name_b, start_b, end_b = segments[j]
                raw_distance, point_a, point_b = self.segment_distance(
                    start_a,
                    end_a,
                    start_b,
                    end_b,
                )
                effective_distance = (
                    raw_distance - 2.0 * self.self_collision_link_radius
                )

                if effective_distance <= self.self_collision_warning_distance:
                    status['self_warning_count'] += 1
                if effective_distance <= self.self_collision_danger_distance:
                    status['self_danger_count'] += 1

                if effective_distance < status['self_min_distance']:
                    status['self_min_distance'] = float(effective_distance)
                    status['self_raw_distance'] = float(raw_distance)
                    status['self_arm'] = arm_name
                    status['self_closest_a'] = name_a
                    status['self_closest_b'] = name_b
                    status['self_closest_a_point'] = point_a.copy()
                    status['self_closest_b_point'] = point_b.copy()

    def skip_collision_pair(self, left_name, right_name):
        # The two base-to-J1 risers are intentionally mounted close together in
        # this dual-arm fixture. Treating that fixed pair as a collision would
        # keep the safety layer permanently active at startup.
        return (
            left_name == 'left_base_link-left_J1'
            and right_name == 'right_base_link-right_J1'
        )

    def build_arm_segments(self, chain, q_map=None):
        if q_map is None:
            q_map = self.q_map

        points = []
        transform = np.eye(4)

        for joint_info in chain:
            transform_origin = transform @ joint_info['origin_T']
            joint_type = joint_info['type']
            joint_name = joint_info['name']

            if joint_type in ['revolute', 'continuous']:
                q = q_map.get(joint_name, 0.0)
                transform = (
                    transform_origin
                    @ self.transform_from_axis_angle(joint_info['axis'], q)
                )
            elif joint_type == 'prismatic':
                q = q_map.get(joint_name, 0.0)
                transform_prismatic = np.eye(4)
                transform_prismatic[:3, 3] = joint_info['axis'] * q
                transform = transform_origin @ transform_prismatic
            else:
                transform = transform_origin

            points.append((joint_info['child'], transform[:3, 3].copy()))

        segments = []
        for (start_name, start_point), (end_name, end_point) in zip(
            points,
            points[1:],
        ):
            segments.append(
                (
                    f'{start_name}-{end_name}',
                    start_point,
                    end_point,
                )
            )

        return segments

    def transform_from_axis_angle(self, axis, angle):
        transform = np.eye(4)
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            return transform

        axis = np.array(axis, dtype=float) / norm
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        one_minus_c = 1.0 - c
        transform[:3, :3] = np.array(
            [
                [
                    c + x * x * one_minus_c,
                    x * y * one_minus_c - z * s,
                    x * z * one_minus_c + y * s,
                ],
                [
                    y * x * one_minus_c + z * s,
                    c + y * y * one_minus_c,
                    y * z * one_minus_c - x * s,
                ],
                [
                    z * x * one_minus_c - y * s,
                    z * y * one_minus_c + x * s,
                    c + z * z * one_minus_c,
                ],
            ]
        )
        return transform

    def segment_distance(self, p1, q1, p2, q2):
        p1 = np.array(p1, dtype=float)
        q1 = np.array(q1, dtype=float)
        p2 = np.array(p2, dtype=float)
        q2 = np.array(q2, dtype=float)

        d1 = q1 - p1
        d2 = q2 - p2
        r = p1 - p2
        a = float(np.dot(d1, d1))
        e = float(np.dot(d2, d2))
        f = float(np.dot(d2, r))
        epsilon = 1e-12

        if a <= epsilon and e <= epsilon:
            closest1 = p1
            closest2 = p2
            return float(np.linalg.norm(closest1 - closest2)), closest1, closest2

        if a <= epsilon:
            s = 0.0
            t = self.clamp(f / e, 0.0, 1.0)
        else:
            c = float(np.dot(d1, r))
            if e <= epsilon:
                t = 0.0
                s = self.clamp(-c / a, 0.0, 1.0)
            else:
                b = float(np.dot(d1, d2))
                denom = a * e - b * b
                if abs(denom) > epsilon:
                    s = self.clamp((b * f - c * e) / denom, 0.0, 1.0)
                else:
                    s = 0.0

                t_nom = b * s + f
                if t_nom < 0.0:
                    t = 0.0
                    s = self.clamp(-c / a, 0.0, 1.0)
                elif t_nom > e:
                    t = 1.0
                    s = self.clamp((b - c) / a, 0.0, 1.0)
                else:
                    t = t_nom / e

        closest1 = p1 + d1 * s
        closest2 = p2 + d2 * t
        return float(np.linalg.norm(closest1 - closest2)), closest1, closest2

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def collision_scale(self, distance):
        if not self.enable_collision_avoidance:
            return 1.0

        if distance <= self.collision_stop_distance:
            return 0.0
        if distance <= self.collision_danger_distance:
            return self.collision_scale_min
        if distance >= self.collision_warning_distance:
            return 1.0

        denom = max(
            self.collision_warning_distance - self.collision_danger_distance,
            1e-9,
        )
        t = (distance - self.collision_danger_distance) / denom
        t = self.clamp(t, 0.0, 1.0)
        smooth = t * t * (3.0 - 2.0 * t)
        return self.collision_scale_min + (1.0 - self.collision_scale_min) * smooth

    def self_collision_scale(self, distance):
        if not self.enable_self_collision_avoidance:
            return 1.0

        if distance <= self.self_collision_stop_distance:
            return 0.0
        if distance <= self.self_collision_danger_distance:
            return self.self_collision_scale_min
        if distance >= self.self_collision_warning_distance:
            return 1.0

        denom = max(
            self.self_collision_warning_distance
            - self.self_collision_danger_distance,
            1e-9,
        )
        t = (distance - self.self_collision_danger_distance) / denom
        t = self.clamp(t, 0.0, 1.0)
        smooth = t * t * (3.0 - 2.0 * t)
        return (
            self.self_collision_scale_min
            + (1.0 - self.self_collision_scale_min) * smooth
        )

    def should_reject_for_inter_collision(self, collision_status):
        if not self.enable_collision_avoidance:
            return False

        if collision_status['stop_active']:
            self.log_rejected_target(
                'collision stop distance reached: '
                f'{collision_status["closest_left"]} vs '
                f'{collision_status["closest_right"]}, '
                f'distance={collision_status["min_distance"]:.3f} m',
                error=True,
            )
            return True

        if (
            self.collision_reject_on_danger
            and collision_status['min_distance'] <= self.collision_danger_distance
        ):
            self.log_rejected_target(
                'collision danger distance reached: '
                f'{collision_status["closest_left"]} vs '
                f'{collision_status["closest_right"]}, '
                f'distance={collision_status["min_distance"]:.3f} m'
            )
            return True

        return False

    def should_reject_for_self_collision(self, collision_status):
        if not self.enable_self_collision_avoidance:
            return False

        if collision_status['self_stop_active']:
            self.log_rejected_target(
                'self-collision stop distance reached: '
                f'{collision_status["self_arm"]}:'
                f'{collision_status["self_closest_a"]} vs '
                f'{collision_status["self_closest_b"]}, '
                f'distance={collision_status["self_min_distance"]:.3f} m',
                error=True,
            )
            return True

        if (
            self.self_collision_reject_on_danger
            and collision_status['self_min_distance']
            <= self.self_collision_danger_distance
        ):
            self.log_rejected_target(
                'self-collision danger distance reached: '
                f'{collision_status["self_arm"]}:'
                f'{collision_status["self_closest_a"]} vs '
                f'{collision_status["self_closest_b"]}, '
                f'distance={collision_status["self_min_distance"]:.3f} m'
            )
            return True

        return False

    def should_reject_for_collision(self, collision_status):
        return (
            self.should_reject_for_inter_collision(collision_status)
            or self.should_reject_for_self_collision(collision_status)
        )

    def collision_stop_active_global(self, collision_status):
        return bool(
            (self.enable_collision_avoidance and collision_status['stop_active'])
            or (
                self.enable_self_collision_avoidance
                and collision_status['self_stop_active']
            )
        )

    def collision_danger_active_global(self, collision_status):
        inter_danger = (
            self.enable_collision_avoidance
            and self.collision_reject_on_danger
            and collision_status['danger_count'] > 0
        )
        self_danger = (
            self.enable_self_collision_avoidance
            and self.self_collision_reject_on_danger
            and collision_status['self_danger_count'] > 0
        )
        return bool(inter_danger or self_danger)

    def collision_recovery_needed(self, collision_status):
        return (
            self.collision_stop_active_global(collision_status)
            or self.collision_danger_active_global(collision_status)
        )

    def global_collision_distance(self, collision_status):
        distances = []
        if self.enable_collision_avoidance:
            distances.append(collision_status['min_distance'])
        if self.enable_self_collision_avoidance:
            distances.append(collision_status['self_min_distance'])

        if not distances:
            return float('inf')

        return float(min(distances))

    def collision_stop_reason(self, collision_status):
        if self.enable_collision_avoidance and collision_status['stop_active']:
            return 'inter_arm_stop'
        if (
            self.enable_self_collision_avoidance
            and collision_status['self_stop_active']
        ):
            return 'self_collision_stop'
        if (
            self.enable_collision_avoidance
            and self.collision_reject_on_danger
            and collision_status['danger_count'] > 0
        ):
            return 'inter_arm_danger'
        if (
            self.enable_self_collision_avoidance
            and self.self_collision_reject_on_danger
            and collision_status['self_danger_count'] > 0
        ):
            return 'self_collision_danger'
        return 'none'

    def empty_collision_recovery_status(self):
        return {
            'collision_hard_stop_active': False,
            'collision_recovery_active': False,
            'collision_recovery_allowed': False,
            'collision_recovery_blocked': False,
            'collision_current_global_distance': float('inf'),
            'collision_predicted_global_distance': float('inf'),
            'collision_recovery_qdot_norm': 0.0,
            'collision_stop_reason': 'none',
        }

    def evaluate_collision_recovery(self, q_dot_candidate, collision_status, dt):
        status = self.empty_collision_recovery_status()
        recovery_needed = self.collision_recovery_needed(collision_status)
        status['collision_current_global_distance'] = (
            self.global_collision_distance(collision_status)
        )
        status['collision_stop_reason'] = self.collision_stop_reason(
            collision_status
        )
        status['collision_hard_stop_active'] = bool(
            self.collision_hard_stop_enabled and recovery_needed
        )

        if not recovery_needed:
            status['collision_predicted_global_distance'] = (
                status['collision_current_global_distance']
            )
            return status

        if not self.collision_recovery_enabled or q_dot_candidate.size == 0:
            status['collision_recovery_blocked'] = True
            return status

        all_joint_names = self.left_joint_names + self.right_joint_names
        q_map_candidate = dict(self.q_map)
        for joint_name, qd in zip(all_joint_names, q_dot_candidate):
            q_map_candidate[joint_name] = (
                q_map_candidate.get(joint_name, 0.0) + float(qd) * dt
            )

        predicted_status = self.compute_collision_status(q_map_candidate)
        predicted_distance = self.global_collision_distance(predicted_status)
        status['collision_predicted_global_distance'] = predicted_distance

        improvement = (
            predicted_distance - status['collision_current_global_distance']
        )
        if improvement >= self.collision_recovery_min_improvement:
            recovery_q_dot = self.recovery_limited_qdot(q_dot_candidate)
            status['collision_recovery_allowed'] = True
            status['collision_recovery_active'] = True
            status['collision_recovery_qdot_norm'] = float(
                np.linalg.norm(recovery_q_dot)
            )
        else:
            status['collision_recovery_blocked'] = True

        if self.collision_recovery_debug_log and recovery_needed:
            self.log_collision_recovery_decision(status)

        return status

    def recovery_limited_qdot(self, q_dot_candidate):
        q_dot_recovery = q_dot_candidate * self.collision_recovery_scale
        return np.clip(
            q_dot_recovery,
            -self.collision_recovery_max_qdot,
            self.collision_recovery_max_qdot,
        )

    def log_collision_recovery_decision(self, recovery_status):
        now = self.get_clock().now()
        if not hasattr(self, 'last_collision_recovery_log_time'):
            self.last_collision_recovery_log_time = None

        if self.last_collision_recovery_log_time is not None:
            elapsed = (
                now - self.last_collision_recovery_log_time
            ).nanoseconds * 1e-9
            if elapsed < 0.5:
                return

        self.last_collision_recovery_log_time = now
        if recovery_status['collision_recovery_allowed']:
            self.get_logger().warn(
                'Collision recovery allowed: '
                f'current_dist='
                f'{recovery_status["collision_current_global_distance"]:.3f} m, '
                f'predicted_dist='
                f'{recovery_status["collision_predicted_global_distance"]:.3f} m, '
                f'qdot_norm='
                f'{recovery_status["collision_recovery_qdot_norm"]:.4f}, '
                f'reason={recovery_status["collision_stop_reason"]}'
            )
        else:
            self.get_logger().warn(
                'Collision recovery blocked: '
                f'current_dist='
                f'{recovery_status["collision_current_global_distance"]:.3f} m, '
                f'predicted_dist='
                f'{recovery_status["collision_predicted_global_distance"]:.3f} m, '
                f'reason={recovery_status["collision_stop_reason"]}'
            )

    def reset_qdot_smoothing_if_collision_stop(self, q_dot):
        if self.reset_qdot_smoothing_on_collision_stop:
            self.previous_q_dot = np.zeros_like(q_dot)

    def apply_collision_safety(
        self,
        q_dot,
        collision_status,
        recovery_status=None,
    ):
        if (
            not self.enable_collision_avoidance
            and not self.enable_self_collision_avoidance
        ):
            return q_dot

        if recovery_status is None:
            recovery_status = self.empty_collision_recovery_status()

        if recovery_status['collision_recovery_allowed']:
            return self.recovery_limited_qdot(q_dot)

        if recovery_status['collision_hard_stop_active']:
            self.reset_qdot_smoothing_if_collision_stop(q_dot)
            return np.zeros_like(q_dot)

        if self.collision_stop_active_global(collision_status):
            self.reset_qdot_smoothing_if_collision_stop(q_dot)
            return np.zeros_like(q_dot)

        inter_scale = (
            collision_status['scale']
            if self.enable_collision_avoidance
            else 1.0
        )
        self_scale = (
            collision_status['self_scale']
            if self.enable_self_collision_avoidance
            else 1.0
        )
        return q_dot * min(inter_scale, self_scale)

    def apply_collision_final_guard(self, q_dot, collision_status, recovery_status):
        if not self.collision_hard_stop_enabled:
            return q_dot

        if not self.collision_stop_active_global(collision_status):
            return q_dot

        if recovery_status['collision_recovery_allowed']:
            return q_dot

        self.reset_qdot_smoothing_if_collision_stop(q_dot)
        return np.zeros_like(q_dot)

    def collision_status_for_log(self, collision_status):
        return {
            'collision_min_distance': collision_status['min_distance'],
            'collision_raw_distance': collision_status['raw_distance'],
            'collision_closest_left': collision_status['closest_left'],
            'collision_closest_right': collision_status['closest_right'],
            'collision_warning_count': collision_status['warning_count'],
            'collision_danger_count': collision_status['danger_count'],
            'collision_stop_active': collision_status['stop_active'],
            'collision_scale': collision_status['scale'],
            'self_collision_min_distance': collision_status['self_min_distance'],
            'self_collision_raw_distance': collision_status['self_raw_distance'],
            'self_collision_arm': collision_status['self_arm'],
            'self_collision_closest_a': collision_status['self_closest_a'],
            'self_collision_closest_b': collision_status['self_closest_b'],
            'self_collision_warning_count': collision_status[
                'self_warning_count'
            ],
            'self_collision_danger_count': collision_status['self_danger_count'],
            'self_collision_stop_active': collision_status['self_stop_active'],
            'self_collision_scale': collision_status['self_scale'],
        }

    def publish_collision_debug_markers(self, collision_status):
        if (
            not self.enable_collision_avoidance
            and not self.enable_self_collision_avoidance
        ):
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if (
            self.enable_collision_avoidance
            and self.collision_debug_log
            and collision_status['closest_left_point'] is not None
            and collision_status['closest_right_point'] is not None
        ):
            left_point = collision_status['closest_left_point']
            right_point = collision_status['closest_right_point']
            color = self.collision_marker_color(collision_status)
            marker_array.markers.append(
                self.make_collision_point_marker(
                    marker_id=0,
                    name='closest_left',
                    point=left_point,
                    color=color,
                    stamp=stamp,
                    namespace='dual_arm_collision',
                )
            )
            marker_array.markers.append(
                self.make_collision_point_marker(
                    marker_id=1,
                    name='closest_right',
                    point=right_point,
                    color=color,
                    stamp=stamp,
                    namespace='dual_arm_collision',
                )
            )
            marker_array.markers.append(
                self.make_collision_line_marker(
                    marker_id=2,
                    left_point=left_point,
                    right_point=right_point,
                    color=color,
                    stamp=stamp,
                    namespace='dual_arm_collision',
                )
            )

        if (
            self.enable_self_collision_avoidance
            and self.self_collision_debug_log
            and collision_status['self_closest_a_point'] is not None
            and collision_status['self_closest_b_point'] is not None
        ):
            point_a = collision_status['self_closest_a_point']
            point_b = collision_status['self_closest_b_point']
            color = self.self_collision_marker_color(collision_status)
            marker_array.markers.append(
                self.make_collision_point_marker(
                    marker_id=10,
                    name='self_collision_a',
                    point=point_a,
                    color=color,
                    stamp=stamp,
                    namespace='self_collision',
                )
            )
            marker_array.markers.append(
                self.make_collision_point_marker(
                    marker_id=11,
                    name='self_collision_b',
                    point=point_b,
                    color=color,
                    stamp=stamp,
                    namespace='self_collision',
                )
            )
            marker_array.markers.append(
                self.make_collision_line_marker(
                    marker_id=12,
                    left_point=point_a,
                    right_point=point_b,
                    color=color,
                    stamp=stamp,
                    namespace='self_collision',
                )
            )

        if not marker_array.markers:
            return

        self.collision_marker_pub.publish(marker_array)

    def collision_marker_color(self, collision_status):
        if collision_status['stop_active']:
            return (1.0, 0.0, 0.0, 1.0)
        if collision_status['danger_count'] > 0:
            return (1.0, 0.25, 0.0, 1.0)
        if collision_status['warning_count'] > 0:
            return (1.0, 0.85, 0.0, 1.0)
        return (0.0, 0.8, 0.2, 0.8)

    def self_collision_marker_color(self, collision_status):
        if collision_status['self_stop_active']:
            return (1.0, 0.0, 0.0, 1.0)
        if collision_status['self_danger_count'] > 0:
            return (1.0, 0.25, 0.0, 1.0)
        if collision_status['self_warning_count'] > 0:
            return (1.0, 0.85, 0.0, 1.0)
        return (0.0, 0.65, 1.0, 0.8)

    def make_collision_point_marker(
        self,
        marker_id,
        name,
        point,
        color,
        stamp,
        namespace,
    ):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.scale.z = 0.045
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = name
        return marker

    def make_collision_line_marker(
        self,
        marker_id,
        left_point,
        right_point,
        color,
        stamp,
        namespace,
    ):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        p_left = Point()
        p_left.x = float(left_point[0])
        p_left.y = float(left_point[1])
        p_left.z = float(left_point[2])
        p_right = Point()
        p_right.x = float(right_point[0])
        p_right.y = float(right_point[1])
        p_right.z = float(right_point[2])
        marker.points = [p_left, p_right]
        return marker

    def compute_safe_command(self, desired_object, desired_relative_position=None):
        if desired_relative_position is None:
            desired_relative_position = self.desired_relative_position.copy()

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
            desired_relative_position - current_relative_position
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
        q_dot_full_ori_projected, full_ee_orientation_status = (
            self.compute_full_ee_orientation_task(
                left_pose,
                right_pose,
                J_left,
                J_right,
                N_rel_obj,
            )
        )
        q_dot_yaw_projected, ee_yaw_status = self.compute_ee_yaw_task(
            left_pose,
            right_pose,
            J_left,
            J_right,
            N_rel_obj,
        )
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
            + q_dot_full_ori_projected
            + q_dot_yaw_projected
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
            'desired_relative_position': desired_relative_position.copy(),
            'object_yaw_deg_current': self.latest_object_yaw_deg,
            'ee_yaw_status': ee_yaw_status,
            'full_ee_orientation_status': full_ee_orientation_status,
            'avoidance_status': avoidance_status,
            'home_bias_norm': float(np.linalg.norm(q_dot_home_projected)),
            'qdot_ee_yaw_norm': float(np.linalg.norm(q_dot_yaw_projected)),
            'qdot_full_ori_norm': float(np.linalg.norm(q_dot_full_ori_projected)),
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
        raw_object_yaw = getattr(self, 'latest_raw_object_yaw', 0.0)
        safe_object_yaw = getattr(self, 'latest_safe_object_yaw', 0.0)
        desired_left_quaternion = getattr(
            self,
            'latest_desired_left_quaternion',
            self.initial_left_quaternion,
        )
        desired_right_quaternion = getattr(
            self,
            'latest_desired_right_quaternion',
            self.initial_right_quaternion,
        )

        transforms = [
            (
                'desired_object',
                self.latest_desired_object_position,
                self.yaw_quaternion(raw_object_yaw),
            ),
            (
                'safe_desired_object',
                safe_object,
                self.yaw_quaternion(safe_object_yaw),
            ),
            (
                'desired_left_grasp',
                self.latest_desired_left_position,
                desired_left_quaternion,
            ),
            (
                'desired_right_grasp',
                self.latest_desired_right_position,
                desired_right_quaternion,
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

        object_yaw_status = ''
        if self.object_yaw_debug_log or self.interactive_yaw_debug_log:
            object_yaw_status = (
                f'object_yaw_enabled={s["object_yaw_enabled"]} | '
                f'interactive_object_yaw_enabled='
                f'{s["interactive_object_yaw_enabled"]} | '
                f'object_yaw={s["object_yaw_deg_current"]:.3f} deg | '
                f'raw_yaw={s["raw_object_yaw_deg"]:.3f} deg | '
                f'safe_yaw={s["safe_object_yaw_deg"]:.3f} deg | '
                f'yaw_delta={s["object_yaw_delta_deg"]:.3f} deg | '
                f'yaw_limited={s["object_yaw_rate_limited"]} | '
                f'desired_rel=['
                f'{s["desired_relative_position"][0]:.3f}, '
                f'{s["desired_relative_position"][1]:.3f}, '
                f'{s["desired_relative_position"][2]:.3f}] | '
            )

        ee_yaw_status = ''
        if self.ee_yaw_debug_log:
            ee_yaw_status = (
                f'ee_yaw_control_enabled={s["ee_yaw_control_enabled"]} | '
                f'left_yaw_err={s["left_ee_yaw_error_deg"]:.3f} deg | '
                f'right_yaw_err={s["right_ee_yaw_error_deg"]:.3f} deg | '
                f'left_wz_des={s["left_ee_wz_des"]:.4f} rad/s | '
                f'right_wz_des={s["right_ee_wz_des"]:.4f} rad/s | '
                f'qdot_yaw_norm={s["qdot_ee_yaw_norm"]:.4f} | '
            )

        full_ori_status = ''
        if self.full_ee_orientation_debug_log:
            full_ori_status = (
                f'full_ori='
                f'{s["full_ee_orientation_control_enabled"]} | '
                f'left_ori_err={s["left_full_ori_error_deg"]:.3f} deg | '
                f'right_ori_err={s["right_full_ori_error_deg"]:.3f} deg | '
                f'left_omega_norm={s["left_omega_des_norm"]:.4f} | '
                f'right_omega_norm={s["right_omega_des_norm"]:.4f} | '
                f'qdot_full_ori_norm={s["qdot_full_ori_norm"]:.4f} | '
                f'full_ori_rank={s["full_ori_task_rank"]} | '
                f'full_ori_cond={s["full_ori_task_condition"]:.3f} | '
            )

        self.get_logger().info(
            f'rel_pos_err={s["relative_position_error_m"]:.5f} m | '
            f'obj_err={s["object_position_error_m"]:.5f} m | '
            f'rel_dist_err={s["relative_distance_error_m"]:.5f} m | '
            f'{object_yaw_status}'
            f'{ee_yaw_status}'
            f'{full_ori_status}'
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
            f'target_rejected_by_collision={s["target_rejected_by_collision"]} | '
            f'target_rejected_by_self_collision='
            f'{s["target_rejected_by_self_collision"]} | '
            f'collision_min_distance={s["collision_min_distance"]:.3f} m | '
            f'collision_raw_distance={s["collision_raw_distance"]:.3f} m | '
            f'closest={s["collision_closest_left"]} vs '
            f'{s["collision_closest_right"]} | '
            f'collision_warning_count={s["collision_warning_count"]} | '
            f'collision_danger_count={s["collision_danger_count"]} | '
            f'collision_stop_active={s["collision_stop_active"]} | '
            f'collision_scale={s["collision_scale"]:.3f} | '
            f'self_collision_min_distance='
            f'{s["self_collision_min_distance"]:.3f} m | '
            f'self_collision_raw_distance='
            f'{s["self_collision_raw_distance"]:.3f} m | '
            f'self_collision={s["self_collision_arm"]}:'
            f'{s["self_collision_closest_a"]} vs '
            f'{s["self_collision_closest_b"]} | '
            f'self_collision_warning_count='
            f'{s["self_collision_warning_count"]} | '
            f'self_collision_danger_count={s["self_collision_danger_count"]} | '
            f'self_collision_stop_active={s["self_collision_stop_active"]} | '
            f'self_collision_scale={s["self_collision_scale"]:.3f} | '
            f'collision_hard_stop_active='
            f'{s["collision_hard_stop_active"]} | '
            f'collision_recovery_active='
            f'{s["collision_recovery_active"]} | '
            f'collision_recovery_allowed='
            f'{s["collision_recovery_allowed"]} | '
            f'collision_recovery_blocked='
            f'{s["collision_recovery_blocked"]} | '
            f'collision_current_global_distance='
            f'{s["collision_current_global_distance"]:.3f} m | '
            f'collision_predicted_global_distance='
            f'{s["collision_predicted_global_distance"]:.3f} m | '
            f'collision_recovery_qdot_norm='
            f'{s["collision_recovery_qdot_norm"]:.4f} | '
            f'collision_stop_reason={s["collision_stop_reason"]} | '
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
