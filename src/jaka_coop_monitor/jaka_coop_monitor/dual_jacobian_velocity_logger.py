import math
import xml.etree.ElementTree as ET

import csv
from pathlib import Path
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr],
    ])

    ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ])

    rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ])

    return rz @ ry @ rx


def transform_from_xyz_rpy(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = np.array(xyz, dtype=float)
    return T


def rotation_matrix_from_axis_angle(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)

    if norm < 1e-12:
        return np.eye(3)

    axis = axis / norm
    x, y, z = axis

    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c

    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C],
    ])


def transform_from_axis_angle(axis, angle):
    T = np.eye(4)
    T[:3, :3] = rotation_matrix_from_axis_angle(axis, angle)
    return T


def normalize_quaternion(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)

    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])

    return q / n


def quat_inverse(q):
    q = normalize_quaternion(q)
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def orientation_error_rotvec(actual_q, desired_q):
    actual_q = normalize_quaternion(actual_q)
    desired_q = normalize_quaternion(desired_q)

    q_err = quat_multiply(desired_q, quat_inverse(actual_q))
    q_err = normalize_quaternion(q_err)

    # ใช้ shortest rotation
    if q_err[3] < 0.0:
        q_err = -q_err

    w = max(min(q_err[3], 1.0), -1.0)
    angle = 2.0 * math.acos(w)

    s = math.sqrt(max(1.0 - w*w, 0.0))

    if s < 1e-8:
        return np.zeros(3), 0.0

    axis = q_err[:3] / s
    rotvec = axis * angle

    return rotvec, angle


def damped_pseudoinverse(J, damping):
    J = np.array(J, dtype=float)
    m, n = J.shape

    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(m))


class DualJacobianVelocityLogger(Node):
    def __init__(self):
        super().__init__('dual_jacobian_velocity_logger')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('sample_rate', 2.0)

        self.declare_parameter('left_actual_frame', 'left_J6')
        self.declare_parameter('right_actual_frame', 'right_J6')
        self.declare_parameter('left_desired_frame', 'desired_left_grasp')
        self.declare_parameter('right_desired_frame', 'desired_right_grasp')

        self.declare_parameter('left_base_link', 'world')
        self.declare_parameter('right_base_link', 'world')
        self.declare_parameter('left_ee_link', 'left_J6')
        self.declare_parameter('right_ee_link', 'right_J6')

        self.declare_parameter('position_gain', 0.8)
        self.declare_parameter('orientation_gain', 0.8)
        self.declare_parameter('damping', 0.02)
        self.declare_parameter('max_joint_velocity', 0.30)

        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        self.left_actual_frame = self.get_parameter('left_actual_frame').value
        self.right_actual_frame = self.get_parameter('right_actual_frame').value
        self.left_desired_frame = self.get_parameter('left_desired_frame').value
        self.right_desired_frame = self.get_parameter('right_desired_frame').value

        self.left_base_link = self.get_parameter('left_base_link').value
        self.right_base_link = self.get_parameter('right_base_link').value
        self.left_ee_link = self.get_parameter('left_ee_link').value
        self.right_ee_link = self.get_parameter('right_ee_link').value

        self.position_gain = float(self.get_parameter('position_gain').value)
        self.orientation_gain = float(self.get_parameter('orientation_gain').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_velocity = float(self.get_parameter('max_joint_velocity').value)
        self.declare_parameter('log_dir', str(Path.home() / 'jaka_ws' / 'logs'))

        self.log_dir = Path(self.get_parameter('log_dir').value)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f'dual_jacobian_velocity_{timestamp}.csv'

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time_sec',

            'left_pos_error_m',
            'left_ori_error_deg',
            'left_rank',
            'left_condition_number',
            'left_scale',
            'left_qdot_1', 'left_qdot_2', 'left_qdot_3',
            'left_qdot_4', 'left_qdot_5', 'left_qdot_6',

            'right_pos_error_m',
            'right_ori_error_deg',
            'right_rank',
            'right_condition_number',
            'right_scale',
            'right_qdot_1', 'right_qdot_2', 'right_qdot_3',
            'right_qdot_4', 'right_qdot_5', 'right_qdot_6',
        ])
        self.csv_file.flush()

        self.start_time = self.get_clock().now()

        self.get_logger().info(f'Logging CSV to: {self.csv_path}')

        self.latest_joint_state = None
        self.robot_loaded = False

        self.joints = {}
        self.children_map = {}
        self.left_chain = []
        self.right_chain = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10
        )

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
            1.0 / self.sample_rate,
            self.timer_callback
        )

        self.get_logger().info('Dual Jacobian Velocity Monitor started')
        self.get_logger().info('This node computes q_dot only. It does NOT command the robot.')
        self.get_logger().info(
            f'position_gain={self.position_gain:.3f}, '
            f'orientation_gain={self.orientation_gain:.3f}, '
            f'damping={self.damping:.4f}, '
            f'max_joint_velocity={self.max_joint_velocity:.3f} rad/s'
        )

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

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
                    xyz_text = origin_tag.attrib.get('xyz', '0 0 0')
                    rpy_text = origin_tag.attrib.get('rpy', '0 0 0')
                    xyz = [float(v) for v in xyz_text.split()]
                    rpy = [float(v) for v in rpy_text.split()]
                else:
                    xyz = [0.0, 0.0, 0.0]
                    rpy = [0.0, 0.0, 0.0]

                axis_tag = joint.find('axis')
                if axis_tag is not None:
                    axis_text = axis_tag.attrib.get('xyz', '0 0 1')
                    axis = [float(v) for v in axis_text.split()]
                else:
                    axis = [0.0, 0.0, 1.0]

                joint_info = {
                    'name': joint_name,
                    'type': joint_type,
                    'parent': parent_link,
                    'child': child_link,
                    'origin_T': transform_from_xyz_rpy(xyz, rpy),
                    'axis': np.array(axis, dtype=float),
                }

                self.joints[joint_name] = joint_info

                if parent_link not in self.children_map:
                    self.children_map[parent_link] = []

                self.children_map[parent_link].append(joint_info)

            self.left_chain = self.find_chain(
                self.left_base_link,
                self.left_ee_link
            )

            self.right_chain = self.find_chain(
                self.right_base_link,
                self.right_ee_link
            )

            self.robot_loaded = True

            self.get_logger().info(
                f'Loaded robot model: {len(self.joints)} joints'
            )
            self.get_logger().info(
                f'Left chain joints: {[j["name"] for j in self.left_chain]}'
            )
            self.get_logger().info(
                f'Right chain joints: {[j["name"] for j in self.right_chain]}'
            )

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

    def timer_callback(self):
        if not self.robot_loaded:
            self.get_logger().warn('Waiting for /robot_description.')
            return

        if self.latest_joint_state is None:
            self.get_logger().warn('Waiting for /joint_states.')
            return

        try:
            joint_map = dict(
                zip(self.latest_joint_state.name, self.latest_joint_state.position)
            )

            left_result = self.compute_arm_velocity_command(
                label='Left',
                chain=self.left_chain,
                joint_map=joint_map,
                actual_frame=self.left_actual_frame,
                desired_frame=self.left_desired_frame
            )

            right_result = self.compute_arm_velocity_command(
                label='Right',
                chain=self.right_chain,
                joint_map=joint_map,
                actual_frame=self.right_actual_frame,
                desired_frame=self.right_desired_frame
            )

            self.log_result('Left', left_result)
            self.log_result('Right', right_result)
            self.write_csv(left_result, right_result)

        except TransformException as ex:
            self.get_logger().warn(f'Waiting for TF: {ex}')
        except Exception as ex:
            self.get_logger().error(f'Failed to compute velocity command: {ex}')

    def compute_arm_velocity_command(self, label, chain, joint_map, actual_frame, desired_frame):
        J_result = self.compute_geometric_jacobian(chain, joint_map)
        J = J_result['J']

        actual_pose = self.lookup_pose(actual_frame)
        desired_pose = self.lookup_pose(desired_frame)

        pos_error = desired_pose['position'] - actual_pose['position']

        rotvec_error, ori_error_angle = orientation_error_rotvec(
            actual_pose['quaternion'],
            desired_pose['quaternion']
        )

        x_dot_cmd = np.zeros(6)
        x_dot_cmd[:3] = self.position_gain * pos_error
        x_dot_cmd[3:] = self.orientation_gain * rotvec_error

        J_pinv = damped_pseudoinverse(J, self.damping)
        q_dot = J_pinv @ x_dot_cmd

        q_dot_scaled = q_dot.copy()
        max_abs_qdot = float(np.max(np.abs(q_dot_scaled))) if q_dot_scaled.size > 0 else 0.0
        scale = 1.0

        if max_abs_qdot > self.max_joint_velocity:
            scale = self.max_joint_velocity / max_abs_qdot
            q_dot_scaled = q_dot_scaled * scale

        return {
            'joint_names': J_result['joint_names'],
            'J': J,
            'rank': J_result['rank'],
            'condition_number': J_result['condition_number'],
            'pos_error_norm': float(np.linalg.norm(pos_error)),
            'ori_error_deg': math.degrees(ori_error_angle),
            'x_dot_cmd_norm': float(np.linalg.norm(x_dot_cmd)),
            'q_dot': q_dot,
            'q_dot_scaled': q_dot_scaled,
            'scale': scale,
            'max_abs_qdot': max_abs_qdot,
        }

    def lookup_pose(self, frame_name):
        tf = self.tf_buffer.lookup_transform(
            self.reference_frame,
            frame_name,
            rclpy.time.Time()
        )

        t = tf.transform.translation
        q = tf.transform.rotation

        return {
            'position': np.array([t.x, t.y, t.z], dtype=float),
            'quaternion': np.array([q.x, q.y, q.z, q.w], dtype=float),
        }

    def compute_geometric_jacobian(self, chain, joint_map):
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
                q = joint_map.get(joint_name, 0.0)

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
                q = joint_map.get(joint_name, 0.0)

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

        singular_values = np.linalg.svd(J, compute_uv=False)
        rank = np.linalg.matrix_rank(J, tol=1e-5)

        if len(singular_values) > 0 and singular_values[-1] > 1e-8:
            condition_number = singular_values[0] / singular_values[-1]
        else:
            condition_number = float('inf')

        return {
            'J': J,
            'joint_names': active_joint_names,
            'rank': rank,
            'condition_number': condition_number,
        }

    def write_csv(self, left_result, right_result):
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds * 1e-9

        left_qdot = list(left_result['q_dot_scaled'])
        right_qdot = list(right_result['q_dot_scaled'])

        while len(left_qdot) < 6:
            left_qdot.append(float('nan'))

        while len(right_qdot) < 6:
            right_qdot.append(float('nan'))

        row = [
            elapsed,

            left_result['pos_error_norm'],
            left_result['ori_error_deg'],
            left_result['rank'],
            left_result['condition_number'],
            left_result['scale'],
            left_qdot[0], left_qdot[1], left_qdot[2],
            left_qdot[3], left_qdot[4], left_qdot[5],

            right_result['pos_error_norm'],
            right_result['ori_error_deg'],
            right_result['rank'],
            right_result['condition_number'],
            right_result['scale'],
            right_qdot[0], right_qdot[1], right_qdot[2],
            right_qdot[3], right_qdot[4], right_qdot[5],
        ]

        self.writer.writerow(row)
        self.csv_file.flush()

    def log_result(self, label, result):
        joint_names = result['joint_names']
        q_dot_scaled = result['q_dot_scaled']

        qdot_pairs = []
        for name, qd in zip(joint_names, q_dot_scaled):
            qdot_pairs.append(f'{name}={qd:+.4f}')

        qdot_text = ', '.join(qdot_pairs)

        self.get_logger().info(
            f'{label} velocity command | '
            f'rank={result["rank"]} | '
            f'cond={result["condition_number"]:.3f} | '
            f'pos_err={result["pos_error_norm"]:.5f} m | '
            f'ori_err={result["ori_error_deg"]:.3f} deg | '
            f'x_dot_norm={result["x_dot_cmd_norm"]:.5f} | '
            f'scale={result["scale"]:.3f} | '
            f'q_dot_scaled=[{qdot_text}]'
        )

        def destroy_node(self):
            if hasattr(self, 'csv_file') and not self.csv_file.closed:
                self.csv_file.close()
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualJacobianVelocityLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
