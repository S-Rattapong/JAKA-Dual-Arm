import math
import xml.etree.ElementTree as ET

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


def relative_quaternion(left_q, right_q):
    return quat_multiply(right_q, quat_inverse(left_q))


def orientation_error_rotvec(current_q, desired_q):
    current_q = normalize_quaternion(current_q)
    desired_q = normalize_quaternion(desired_q)

    q_err = quat_multiply(desired_q, quat_inverse(current_q))
    q_err = normalize_quaternion(q_err)

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
    m, _ = J.shape

    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * np.eye(m))


class DualRelativeJacobianMonitor(Node):
    def __init__(self):
        super().__init__('dual_relative_jacobian_monitor')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('sample_rate', 2.0)

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

        self.left_base_link = self.get_parameter('left_base_link').value
        self.right_base_link = self.get_parameter('right_base_link').value
        self.left_ee_link = self.get_parameter('left_ee_link').value
        self.right_ee_link = self.get_parameter('right_ee_link').value

        self.position_gain = float(self.get_parameter('position_gain').value)
        self.orientation_gain = float(self.get_parameter('orientation_gain').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_velocity = float(self.get_parameter('max_joint_velocity').value)

        self.latest_joint_state = None
        self.robot_loaded = False

        self.joints = {}
        self.children_map = {}
        self.left_chain = []
        self.right_chain = []

        self.desired_relative_position = None
        self.desired_relative_quaternion = None

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

        self.get_logger().info('Dual Relative Jacobian Monitor started')
        self.get_logger().info('This node computes relative q_dot only. It does NOT command the robot.')
        self.get_logger().info('Using approximate world-frame relative Jacobian: J_rel = [-J_left, J_right]')

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

            left_pose = self.lookup_pose(self.left_ee_link)
            right_pose = self.lookup_pose(self.right_ee_link)

            current_relative_position = right_pose['position'] - left_pose['position']
            current_relative_quaternion = relative_quaternion(
                left_pose['quaternion'],
                right_pose['quaternion']
            )

            if self.desired_relative_position is None:
                self.desired_relative_position = current_relative_position.copy()
                self.desired_relative_quaternion = current_relative_quaternion.copy()

                self.get_logger().info('Desired relative pose initialized from current dual-arm pose.')
                self.get_logger().info(
                    f'desired_relative_position = '
                    f'[{self.desired_relative_position[0]:.5f}, '
                    f'{self.desired_relative_position[1]:.5f}, '
                    f'{self.desired_relative_position[2]:.5f}]'
                )
                return

            left_result = self.compute_geometric_jacobian(self.left_chain, joint_map)
            right_result = self.compute_geometric_jacobian(self.right_chain, joint_map)

            J_left = left_result['J']
            J_right = right_result['J']

            J_rel = np.hstack((-J_left, J_right))

            position_error = self.desired_relative_position - current_relative_position

            rotvec_error, orientation_error_angle = orientation_error_rotvec(
                current_relative_quaternion,
                self.desired_relative_quaternion
            )

            x_dot_rel = np.zeros(6)
            x_dot_rel[:3] = self.position_gain * position_error
            x_dot_rel[3:] = self.orientation_gain * rotvec_error

            J_rel_pinv = damped_pseudoinverse(J_rel, self.damping)
            q_dot_12 = J_rel_pinv @ x_dot_rel

            q_dot_12_scaled = q_dot_12.copy()
            max_abs_qdot = float(np.max(np.abs(q_dot_12_scaled))) if q_dot_12_scaled.size > 0 else 0.0
            scale = 1.0

            if max_abs_qdot > self.max_joint_velocity:
                scale = self.max_joint_velocity / max_abs_qdot
                q_dot_12_scaled = q_dot_12_scaled * scale

            singular_values = np.linalg.svd(J_rel, compute_uv=False)
            rank = np.linalg.matrix_rank(J_rel, tol=1e-5)

            if len(singular_values) > 0 and singular_values[-1] > 1e-8:
                condition_number = singular_values[0] / singular_values[-1]
            else:
                condition_number = float('inf')

            left_qdot = q_dot_12_scaled[:len(left_result['joint_names'])]
            right_qdot = q_dot_12_scaled[len(left_result['joint_names']):]

            self.log_result(
                J_rel=J_rel,
                rank=rank,
                condition_number=condition_number,
                position_error=position_error,
                orientation_error_angle=orientation_error_angle,
                x_dot_rel=x_dot_rel,
                scale=scale,
                left_joint_names=left_result['joint_names'],
                right_joint_names=right_result['joint_names'],
                left_qdot=left_qdot,
                right_qdot=right_qdot,
            )

        except TransformException as ex:
            self.get_logger().warn(f'Waiting for TF: {ex}')
        except Exception as ex:
            self.get_logger().error(f'Failed to compute relative Jacobian: {ex}')

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

        return {
            'J': J,
            'joint_names': active_joint_names,
        }

    def log_result(
        self,
        J_rel,
        rank,
        condition_number,
        position_error,
        orientation_error_angle,
        x_dot_rel,
        scale,
        left_joint_names,
        right_joint_names,
        left_qdot,
        right_qdot,
    ):
        left_pairs = []
        for name, qd in zip(left_joint_names, left_qdot):
            left_pairs.append(f'{name}={qd:+.4f}')

        right_pairs = []
        for name, qd in zip(right_joint_names, right_qdot):
            right_pairs.append(f'{name}={qd:+.4f}')

        pos_err_norm = float(np.linalg.norm(position_error))
        ori_err_deg = math.degrees(orientation_error_angle)
        x_dot_norm = float(np.linalg.norm(x_dot_rel))

        self.get_logger().info(
            f'Relative Jacobian | '
            f'shape={J_rel.shape} | '
            f'rank={rank} | '
            f'cond={condition_number:.3f} | '
            f'rel_pos_err={pos_err_norm:.5f} m | '
            f'rel_ori_err={ori_err_deg:.3f} deg | '
            f'x_dot_norm={x_dot_norm:.5f} | '
            f'scale={scale:.3f}'
        )

        self.get_logger().info(
            f'Left relative q_dot=[{", ".join(left_pairs)}]'
        )

        self.get_logger().info(
            f'Right relative q_dot=[{", ".join(right_pairs)}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualRelativeJacobianMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
