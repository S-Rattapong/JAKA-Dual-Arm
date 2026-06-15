import math
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from jaka_coop_monitor.dual_relative_motion_demo import (
    damped_pseudoinverse,
    matrix_to_quaternion,
    orientation_error_rotvec,
    transform_from_axis_angle,
    transform_from_xyz_rpy,
)


def quaternion_to_msg(q):
    q = np.array(q, dtype=float)
    return {
        'x': float(q[0]),
        'y': float(q[1]),
        'z': float(q[2]),
        'w': float(q[3]),
    }


class DualObjectTrajectoryDemo(Node):
    def __init__(self):
        super().__init__(
            getattr(self, '_demo_node_name', 'dual_object_trajectory_demo')
        )

        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.declare_parameter('left_base_link', 'world')
        self.declare_parameter('right_base_link', 'world')
        self.declare_parameter('left_ee_link', 'left_J6')
        self.declare_parameter('right_ee_link', 'right_J6')

        self.declare_parameter('trajectory_mode', 'sine_x')
        self.declare_parameter('amplitude', 0.10)
        self.declare_parameter('frequency', 0.10)
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('position_gain', 1.0)
        self.declare_parameter('orientation_gain', 0.6)
        self.declare_parameter('damping', 0.03)
        self.declare_parameter('max_joint_velocity', 0.30)

        self.declare_parameter(
            'initial_left_positions',
            [0.0, 0.52124, -0.800072, 0.0, 0.798816, 0.0]
        )
        self.declare_parameter(
            'initial_right_positions',
            [3.14, 2.617504, 0.798816, 0.0, 2.339928, 0.0]
        )

        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        self.left_base_link = self.get_parameter('left_base_link').value
        self.right_base_link = self.get_parameter('right_base_link').value
        self.left_ee_link = self.get_parameter('left_ee_link').value
        self.right_ee_link = self.get_parameter('right_ee_link').value

        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.amplitude = float(self.get_parameter('amplitude').value)
        self.frequency = float(self.get_parameter('frequency').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.position_gain = float(self.get_parameter('position_gain').value)
        self.orientation_gain = float(self.get_parameter('orientation_gain').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_velocity = float(self.get_parameter('max_joint_velocity').value)

        self.initial_left_positions = list(
            self.get_parameter('initial_left_positions').value
        )
        self.initial_right_positions = list(
            self.get_parameter('initial_right_positions').value
        )

        self.joints = {}
        self.joint_limits = {}
        self.children_map = {}
        self.left_chain = []
        self.right_chain = []
        self.left_joint_names = []
        self.right_joint_names = []

        self.q_map = {}
        self.latest_qdot_map = {}

        self.robot_loaded = False
        self.initialized = False

        self.initial_object_position = None
        self.left_grasp_offset = None
        self.right_grasp_offset = None
        self.initial_left_quaternion = None
        self.initial_right_quaternion = None
        self.initial_relative_distance = None

        self.latest_desired_object_position = None
        self.latest_desired_left_position = None
        self.latest_desired_right_position = None
        self.latest_actual_object_position = None

        self.start_time = None
        self.last_time = None
        self.last_log_time = None

        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_states_topic,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        robot_description_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            String,
            self.robot_description_topic,
            self.robot_description_callback,
            robot_description_qos
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.timer_callback
        )

        demo_title = getattr(self, '_demo_title', 'Dual Object Trajectory Demo')
        self.get_logger().info(f'{demo_title} started')
        self.get_logger().info('This node publishes /joint_states directly.')
        self.get_logger().info('Do NOT run joint_state_publisher_gui at the same time.')
        if getattr(self, '_log_trajectory_params', True):
            self.get_logger().info(
                f'trajectory_mode={self.trajectory_mode}, '
                f'amplitude={self.amplitude:.3f} m, '
                f'frequency={self.frequency:.3f} Hz'
            )

    def robot_description_callback(self, msg):
        if self.robot_loaded:
            return

        try:
            root = ET.fromstring(msg.data)

            for joint in root.findall('joint'):
                joint_name = joint.attrib.get('name', '')
                joint_type = joint.attrib.get('type', '')

                parent_tag = joint.find('parent')
                child_tag = joint.find('child')

                if parent_tag is None or child_tag is None:
                    continue

                parent_link = parent_tag.attrib.get('link', '')
                child_link = child_tag.attrib.get('link', '')

                origin_tag = joint.find('origin')
                if origin_tag is not None:
                    xyz = [float(v) for v in origin_tag.attrib.get('xyz', '0 0 0').split()]
                    rpy = [float(v) for v in origin_tag.attrib.get('rpy', '0 0 0').split()]
                else:
                    xyz = [0.0, 0.0, 0.0]
                    rpy = [0.0, 0.0, 0.0]

                axis_tag = joint.find('axis')
                if axis_tag is not None:
                    axis = [float(v) for v in axis_tag.attrib.get('xyz', '0 0 1').split()]
                else:
                    axis = [0.0, 0.0, 1.0]

                lower = None
                upper = None
                limit_tag = joint.find('limit')
                if limit_tag is not None:
                    lower_text = limit_tag.attrib.get('lower')
                    upper_text = limit_tag.attrib.get('upper')
                    if lower_text is not None and upper_text is not None:
                        lower = float(lower_text)
                        upper = float(upper_text)

                joint_info = {
                    'name': joint_name,
                    'type': joint_type,
                    'parent': parent_link,
                    'child': child_link,
                    'origin_T': transform_from_xyz_rpy(xyz, rpy),
                    'axis': np.array(axis, dtype=float),
                    'lower': lower,
                    'upper': upper,
                }

                self.joints[joint_name] = joint_info

                if lower is not None and upper is not None:
                    self.joint_limits[joint_name] = {
                        'lower': lower,
                        'upper': upper,
                    }

                self.children_map.setdefault(parent_link, []).append(joint_info)

            self.left_chain = self.find_chain(self.left_base_link, self.left_ee_link)
            self.right_chain = self.find_chain(self.right_base_link, self.right_ee_link)

            self.left_joint_names = self.get_active_joint_names(self.left_chain)
            self.right_joint_names = self.get_active_joint_names(self.right_chain)

            self.robot_loaded = True

            self.get_logger().info('Loaded robot model successfully.')
            self.get_logger().info(f'Left joints : {self.left_joint_names}')
            self.get_logger().info(f'Right joints: {self.right_joint_names}')

        except Exception as ex:
            self.get_logger().error(f'Failed to parse robot_description: {ex}')

    def find_chain(self, base_link, ee_link):
        visited = set()

        def dfs(current_link, path):
            if current_link == ee_link:
                return path

            if current_link in visited:
                return None

            visited.add(current_link)

            for joint_info in self.children_map.get(current_link, []):
                result = dfs(joint_info['child'], path + [joint_info])
                if result is not None:
                    return result

            return None

        chain = dfs(base_link, [])

        if chain is None:
            raise RuntimeError(f'Could not find chain: {base_link} -> {ee_link}')

        return chain

    def get_active_joint_names(self, chain):
        return [
            joint_info['name']
            for joint_info in chain
            if joint_info['type'] in ['revolute', 'continuous', 'prismatic']
        ]

    def initialize_state(self):
        for i, joint_name in enumerate(self.left_joint_names):
            self.q_map[joint_name] = (
                float(self.initial_left_positions[i])
                if i < len(self.initial_left_positions)
                else 0.0
            )

        for i, joint_name in enumerate(self.right_joint_names):
            self.q_map[joint_name] = (
                float(self.initial_right_positions[i])
                if i < len(self.initial_right_positions)
                else 0.0
            )

        left_pose = self.compute_fk_and_jacobian(self.left_chain)['pose']
        right_pose = self.compute_fk_and_jacobian(self.right_chain)['pose']

        self.initial_object_position = 0.5 * (
            left_pose['position'] + right_pose['position']
        )
        self.left_grasp_offset = left_pose['position'] - self.initial_object_position
        self.right_grasp_offset = right_pose['position'] - self.initial_object_position

        self.initial_left_quaternion = left_pose['quaternion']
        self.initial_right_quaternion = right_pose['quaternion']
        self.initial_relative_distance = float(
            np.linalg.norm(right_pose['position'] - left_pose['position'])
        )

        self.latest_desired_object_position = self.initial_object_position.copy()
        self.latest_desired_left_position = left_pose['position'].copy()
        self.latest_desired_right_position = right_pose['position'].copy()
        self.latest_actual_object_position = self.initial_object_position.copy()

        now = self.get_clock().now()
        self.start_time = now
        self.last_time = now
        self.last_log_time = now

        self.initialized = True

        self.get_logger().info('Initial object and grasp offsets initialized.')
        self.get_logger().info(
            f'initial_object_position = '
            f'[{self.initial_object_position[0]:.5f}, '
            f'{self.initial_object_position[1]:.5f}, '
            f'{self.initial_object_position[2]:.5f}]'
        )
        self.get_logger().info(
            f'initial_relative_distance = {self.initial_relative_distance:.5f} m'
        )

    def timer_callback(self):
        if not self.robot_loaded:
            self.get_logger().warn('Waiting for /robot_description.')
            return

        if not self.initialized:
            self.initialize_state()
            self.publish_joint_state()
            self.publish_visualization_frames()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.control_rate

        elapsed = (now - self.start_time).nanoseconds * 1e-9

        self.control_step(elapsed, dt)
        self.publish_joint_state()
        self.publish_visualization_frames()
        self.log_status_if_needed(now)

    def desired_object_position(self, elapsed):
        omega_t = 2.0 * math.pi * self.frequency * elapsed
        delta = np.zeros(3)

        if self.trajectory_mode == 'sine_x':
            delta[0] = self.amplitude * math.sin(omega_t)
        elif self.trajectory_mode == 'sine_y':
            delta[1] = self.amplitude * math.sin(omega_t)
        elif self.trajectory_mode == 'sine_z':
            delta[2] = self.amplitude * math.sin(omega_t)
        elif self.trajectory_mode == 'circle_xy':
            delta[0] = self.amplitude * math.cos(omega_t) - self.amplitude
            delta[1] = self.amplitude * math.sin(omega_t)
        else:
            self.get_logger().warn(
                f'Unknown trajectory_mode={self.trajectory_mode}; using sine_x.'
            )
            delta[0] = self.amplitude * math.sin(omega_t)

        return self.initial_object_position + delta

    def control_step(self, elapsed, dt):
        left_result = self.compute_fk_and_jacobian(self.left_chain)
        right_result = self.compute_fk_and_jacobian(self.right_chain)

        left_pose = left_result['pose']
        right_pose = right_result['pose']

        desired_object = self.desired_object_position(elapsed)
        desired_left = desired_object + self.left_grasp_offset
        desired_right = desired_object + self.right_grasp_offset

        left_qdot, left_status = self.solve_arm_velocity(
            left_result['J'],
            left_pose,
            desired_left,
            self.initial_left_quaternion,
        )
        right_qdot, right_status = self.solve_arm_velocity(
            right_result['J'],
            right_pose,
            desired_right,
            self.initial_right_quaternion,
        )

        q_dot = np.concatenate((left_qdot, right_qdot))
        max_abs_qdot = float(np.max(np.abs(q_dot))) if q_dot.size > 0 else 0.0
        scale = 1.0

        if max_abs_qdot > self.max_joint_velocity:
            scale = self.max_joint_velocity / max_abs_qdot
            q_dot = q_dot * scale

        self.latest_qdot_map = {}
        all_joint_names = self.left_joint_names + self.right_joint_names

        for joint_name, qd in zip(all_joint_names, q_dot):
            self.latest_qdot_map[joint_name] = float(qd)
            self.q_map[joint_name] += float(qd) * dt
            self.apply_joint_limits(joint_name)

        actual_object = 0.5 * (left_pose['position'] + right_pose['position'])
        current_relative_distance = float(
            np.linalg.norm(right_pose['position'] - left_pose['position'])
        )

        self.latest_desired_object_position = desired_object
        self.latest_desired_left_position = desired_left
        self.latest_desired_right_position = desired_right
        self.latest_actual_object_position = actual_object

        self.latest_status = {
            'object_position_error_m': float(np.linalg.norm(desired_object - actual_object)),
            'left_grasp_position_error_m': left_status['position_error_norm'],
            'right_grasp_position_error_m': right_status['position_error_norm'],
            'relative_distance_error_m': abs(
                current_relative_distance - self.initial_relative_distance
            ),
            'left_rank': left_status['rank'],
            'right_rank': right_status['rank'],
            'left_condition_number': left_status['condition_number'],
            'right_condition_number': right_status['condition_number'],
            'scale': scale,
        }

    def solve_arm_velocity(self, J, current_pose, desired_position, desired_quaternion):
        position_error = desired_position - current_pose['position']
        rotvec_error, _ = orientation_error_rotvec(
            current_pose['quaternion'],
            desired_quaternion
        )

        x_dot = np.zeros(6)
        x_dot[:3] = self.position_gain * position_error
        x_dot[3:] = self.orientation_gain * rotvec_error

        q_dot = damped_pseudoinverse(J, self.damping) @ x_dot

        singular_values = np.linalg.svd(J, compute_uv=False)
        rank = int(np.linalg.matrix_rank(J, tol=1e-5))

        if len(singular_values) > 0 and singular_values[-1] > 1e-8:
            condition_number = float(singular_values[0] / singular_values[-1])
        else:
            condition_number = float('inf')

        return q_dot, {
            'position_error_norm': float(np.linalg.norm(position_error)),
            'rank': rank,
            'condition_number': condition_number,
        }

    def compute_fk_and_jacobian(self, chain):
        T = np.eye(4)

        active_joint_names = []
        joint_positions = []
        joint_axes_world = []
        joint_types = []

        for joint_info in chain:
            T_origin = T @ joint_info['origin_T']

            joint_type = joint_info['type']
            joint_name = joint_info['name']

            if joint_type in ['revolute', 'continuous']:
                q = self.q_map.get(joint_name, 0.0)

                p_joint = T_origin[:3, 3].copy()
                axis_world = T_origin[:3, :3] @ joint_info['axis']

                norm = np.linalg.norm(axis_world)
                if norm > 1e-12:
                    axis_world = axis_world / norm

                active_joint_names.append(joint_name)
                joint_positions.append(p_joint)
                joint_axes_world.append(axis_world)
                joint_types.append(joint_type)

                T = T_origin @ transform_from_axis_angle(joint_info['axis'], q)

            elif joint_type == 'prismatic':
                q = self.q_map.get(joint_name, 0.0)

                axis_world = T_origin[:3, :3] @ joint_info['axis']
                norm = np.linalg.norm(axis_world)
                if norm > 1e-12:
                    axis_world = axis_world / norm

                p_joint = T_origin[:3, 3].copy()

                active_joint_names.append(joint_name)
                joint_positions.append(p_joint)
                joint_axes_world.append(axis_world)
                joint_types.append(joint_type)

                T_prismatic = np.eye(4)
                T_prismatic[:3, 3] = joint_info['axis'] * q
                T = T_origin @ T_prismatic

            else:
                T = T_origin

        p_ee = T[:3, 3].copy()
        R_ee = T[:3, :3].copy()
        q_ee = matrix_to_quaternion(R_ee)

        J = np.zeros((6, len(active_joint_names)))

        for i in range(len(active_joint_names)):
            axis_world = joint_axes_world[i]
            p_joint = joint_positions[i]
            joint_type = joint_types[i]

            if joint_type in ['revolute', 'continuous']:
                J[:3, i] = np.cross(axis_world, p_ee - p_joint)
                J[3:, i] = axis_world
            elif joint_type == 'prismatic':
                J[:3, i] = axis_world
                J[3:, i] = np.zeros(3)

        return {
            'pose': {
                'position': p_ee,
                'quaternion': q_ee,
            },
            'J': J,
            'joint_names': active_joint_names,
        }

    def apply_joint_limits(self, joint_name):
        if joint_name not in self.joint_limits:
            return

        lower = self.joint_limits[joint_name]['lower']
        upper = self.joint_limits[joint_name]['upper']

        self.q_map[joint_name] = max(lower, min(upper, self.q_map[joint_name]))

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        joint_names = self.left_joint_names + self.right_joint_names

        msg.name = joint_names
        msg.position = [self.q_map.get(name, 0.0) for name in joint_names]
        msg.velocity = [self.latest_qdot_map.get(name, 0.0) for name in joint_names]
        msg.effort = []

        self.joint_state_pub.publish(msg)

    def publish_visualization_frames(self):
        if self.latest_desired_object_position is None:
            return

        stamp = self.get_clock().now().to_msg()
        transforms = [
            (
                'desired_object',
                self.latest_desired_object_position,
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
        self.get_logger().info(
            f'object_err={s["object_position_error_m"]:.5f} m | '
            f'left_err={s["left_grasp_position_error_m"]:.5f} m | '
            f'right_err={s["right_grasp_position_error_m"]:.5f} m | '
            f'rel_dist_err={s["relative_distance_error_m"]:.5f} m | '
            f'L_rank={s["left_rank"]} R_rank={s["right_rank"]} | '
            f'L_cond={s["left_condition_number"]:.3f} '
            f'R_cond={s["right_condition_number"]:.3f} | '
            f'scale={s["scale"]:.3f} | '
            f'max_abs_qdot={max_abs_qdot:.4f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectTrajectoryDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
