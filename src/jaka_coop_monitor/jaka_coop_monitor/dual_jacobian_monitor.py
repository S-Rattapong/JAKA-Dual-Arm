import math
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String


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


class DualJacobianMonitor(Node):
    def __init__(self):
        super().__init__('dual_jacobian_monitor')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('sample_rate', 2.0)

        self.declare_parameter('left_base_link', 'left_base_link')
        self.declare_parameter('left_ee_link', 'left_J6')
        self.declare_parameter('right_base_link', 'right_base_link')
        self.declare_parameter('right_ee_link', 'right_J6')

        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        self.left_base_link = self.get_parameter('left_base_link').value
        self.left_ee_link = self.get_parameter('left_ee_link').value
        self.right_base_link = self.get_parameter('right_base_link').value
        self.right_ee_link = self.get_parameter('right_ee_link').value

        self.latest_joint_state = None
        self.robot_loaded = False

        self.joints = {}
        self.children_map = {}

        self.left_chain = []
        self.right_chain = []

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

        self.get_logger().info('Dual Jacobian Monitor started')
        self.get_logger().info(
            f'Left chain : {self.left_base_link} -> {self.left_ee_link}'
        )
        self.get_logger().info(
            f'Right chain: {self.right_base_link} -> {self.right_ee_link}'
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

        joint_map = dict(
            zip(self.latest_joint_state.name, self.latest_joint_state.position)
        )

        left_result = self.compute_geometric_jacobian(self.left_chain, joint_map)
        right_result = self.compute_geometric_jacobian(self.right_chain, joint_map)

        self.log_jacobian_result('Left', left_result)
        self.log_jacobian_result('Right', right_result)

    def compute_geometric_jacobian(self, chain, joint_map):
        T = np.eye(4)

        active_joint_names = []
        joint_positions = []
        joint_axes_world = []

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

                T_prismatic = np.eye(4)
                T_prismatic[:3, 3] = joint_info['axis'] * q
                T = T_origin @ T_prismatic

            else:
                T = T_origin

        p_ee = T[:3, 3].copy()

        J = np.zeros((6, len(active_joint_names)))

        for i, joint_name in enumerate(active_joint_names):
            joint_info = None
            for j in chain:
                if j['name'] == joint_name:
                    joint_info = j
                    break

            axis_world = joint_axes_world[i]
            p_joint = joint_positions[i]

            if joint_info['type'] in ['revolute', 'continuous']:
                J[:3, i] = np.cross(axis_world, p_ee - p_joint)
                J[3:, i] = axis_world
            elif joint_info['type'] == 'prismatic':
                J[:3, i] = axis_world
                J[3:, i] = np.zeros(3)

        singular_values = np.linalg.svd(J, compute_uv=False)
        rank = np.linalg.matrix_rank(J, tol=1e-5)

        if len(singular_values) > 0 and singular_values[-1] > 1e-8:
            condition_number = singular_values[0] / singular_values[-1]
        else:
            condition_number = float('inf')

        manipulability = float(np.prod(singular_values))

        return {
            'J': J,
            'joint_names': active_joint_names,
            'ee_position': p_ee,
            'rank': rank,
            'singular_values': singular_values,
            'condition_number': condition_number,
            'manipulability': manipulability,
        }

    def log_jacobian_result(self, label, result):
        J = result['J']
        s = result['singular_values']

        min_sv = float(s[-1]) if len(s) > 0 else 0.0
        max_sv = float(s[0]) if len(s) > 0 else 0.0

        self.get_logger().info(
            f'{label} Jacobian | '
            f'shape={J.shape} | '
            f'rank={result["rank"]} | '
            f'min_sv={min_sv:.6f} | '
            f'max_sv={max_sv:.6f} | '
            f'cond={result["condition_number"]:.3f} | '
            f'manip={result["manipulability"]:.8f} | '
            f'ee_pos=[{result["ee_position"][0]:.3f}, '
            f'{result["ee_position"][1]:.3f}, '
            f'{result["ee_position"][2]:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualJacobianMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
