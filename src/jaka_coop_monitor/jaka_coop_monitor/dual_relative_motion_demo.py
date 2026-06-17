import ast
import math
import xml.etree.ElementTree as ET

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String


DEFAULT_INITIAL_LEFT_POSITIONS = [
    0.0,
    0.52124,
    -0.800072,
    0.0,
    0.798816,
    0.0,
]
DEFAULT_INITIAL_RIGHT_POSITIONS = [
    0.0,
    2.617504,
    0.798816,
    0.0,
    2.339928,
    0.0,
]


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


def matrix_to_quaternion(R):
    tr = np.trace(R)

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return normalize_quaternion(np.array([qx, qy, qz, qw], dtype=float))


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


class DualRelativeMotionDemo(Node):
    def __init__(self):
        super().__init__('dual_relative_motion_demo')

        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.declare_parameter('left_base_link', 'world')
        self.declare_parameter('right_base_link', 'world')
        self.declare_parameter('left_ee_link', 'left_J6')
        self.declare_parameter('right_ee_link', 'right_J6')

        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('position_gain', 1.2)
        self.declare_parameter('orientation_gain', 1.2)
        self.declare_parameter('damping', 0.03)
        self.declare_parameter('max_joint_velocity', 0.35)

        self.declare_parameter('enable_disturbance', True)
        self.declare_parameter('disturbance_time', 3.0)
        self.declare_parameter('disturbance_joint', 'left_joint_2')
        self.declare_parameter('disturbance_amount', 0.12)
        self.declare_parameter('startup_hold_seconds', 2.0)
        self.declare_parameter('initial_pose_only', False)

        self.declare_parameter(
            'initial_left_positions',
            DEFAULT_INITIAL_LEFT_POSITIONS
        )
        self.declare_parameter(
            'initial_right_positions',
            DEFAULT_INITIAL_RIGHT_POSITIONS
        )
        for i in range(6):
            self.declare_parameter(f'initial_left_joint_{i + 1}', math.nan)
            self.declare_parameter(f'initial_right_joint_{i + 1}', math.nan)

        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        self.left_base_link = self.get_parameter('left_base_link').value
        self.right_base_link = self.get_parameter('right_base_link').value
        self.left_ee_link = self.get_parameter('left_ee_link').value
        self.right_ee_link = self.get_parameter('right_ee_link').value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.position_gain = float(self.get_parameter('position_gain').value)
        self.orientation_gain = float(self.get_parameter('orientation_gain').value)
        self.damping = float(self.get_parameter('damping').value)
        self.max_joint_velocity = float(self.get_parameter('max_joint_velocity').value)

        self.enable_disturbance = bool(self.get_parameter('enable_disturbance').value)
        self.disturbance_time = float(self.get_parameter('disturbance_time').value)
        self.disturbance_joint = self.get_parameter('disturbance_joint').value
        self.disturbance_amount = float(self.get_parameter('disturbance_amount').value)
        self.startup_hold_seconds = max(
            0.0,
            float(self.get_parameter('startup_hold_seconds').value)
        )
        self.initial_pose_only = bool(
            self.get_parameter('initial_pose_only').value
        )

        self.initial_left_positions, self.initial_left_source = (
            self.read_initial_positions(
                'left',
                DEFAULT_INITIAL_LEFT_POSITIONS,
            )
        )
        self.initial_right_positions, self.initial_right_source = (
            self.read_initial_positions(
                'right',
                DEFAULT_INITIAL_RIGHT_POSITIONS,
            )
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
        self.disturbance_applied = False
        self.hold_complete_logged = False
        self.first_joint_state_logged = False

        self.desired_relative_position = None
        self.desired_relative_quaternion = None

        self.start_time = None
        self.last_time = None
        self.last_log_time = None

        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_states_topic,
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
            1.0 / self.control_rate,
            self.timer_callback
        )

        self.get_logger().info('Dual Relative Motion Demo started')
        self.get_logger().info('This node publishes /joint_states directly.')
        self.get_logger().info('Do NOT run joint_state_publisher_gui at the same time.')
        self.get_logger().info(
            f'controller_enabled={not self.initial_pose_only}, '
            f'startup_hold_seconds={self.startup_hold_seconds:.2f}s, '
            f'initial_pose_only={self.initial_pose_only}'
        )
        self.get_logger().info(
            f'disturbance: enabled={self.enable_disturbance}, '
            f'joint={self.disturbance_joint}, '
            f'amount={self.disturbance_amount:.3f} rad, '
            f'delay_after_hold={self.disturbance_time:.2f}s'
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

                limit_tag = joint.find('limit')
                lower = None
                upper = None
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
                    'origin_xyz': xyz,
                    'origin_rpy': rpy,
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

                if parent_link not in self.children_map:
                    self.children_map[parent_link] = []

                self.children_map[parent_link].append(joint_info)

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
        names = []
        for joint_info in chain:
            if joint_info['type'] in ['revolute', 'continuous', 'prismatic']:
                names.append(joint_info['name'])
        return names

    def read_initial_positions(self, side, defaults):
        raw_positions = self.get_parameter(f'initial_{side}_positions').value
        positions = self.parse_initial_position_array(raw_positions)
        source = 'default parameters'

        if not self.same_positions(positions, defaults):
            source = f'initial_{side}_positions launch override'

        overridden_joints = []
        for i in range(6):
            param_name = f'initial_{side}_joint_{i + 1}'
            value = float(self.get_parameter(param_name).value)
            if math.isnan(value):
                continue

            while len(positions) <= i:
                positions.append(0.0)
            positions[i] = value
            overridden_joints.append(param_name)

        if overridden_joints:
            source = 'individual launch override: ' + ', '.join(overridden_joints)

        return positions, source

    def parse_initial_position_array(self, value):
        if isinstance(value, str):
            value = ast.literal_eval(value)

        return [float(v) for v in value]

    def same_positions(self, values, defaults):
        if len(values) != len(defaults):
            return False

        return all(
            abs(float(value) - float(default)) <= 1e-9
            for value, default in zip(values, defaults)
        )

    def joint_names(self):
        return self.left_joint_names + self.right_joint_names

    def log_initial_state(self):
        joint_names = self.joint_names()
        positions = [self.q_map.get(name, 0.0) for name in joint_names]

        self.get_logger().info(
            'Initial joint state used by dual_relative_motion_demo:'
        )
        self.get_logger().info(f'joint_names = {joint_names}')
        self.get_logger().info(
            f'initial_left_positions source: {self.initial_left_source}'
        )
        self.get_logger().info(
            f'initial_right_positions source: {self.initial_right_source}'
        )
        self.get_logger().info(
            f'combined joint state vector = {[float(v) for v in positions]}'
        )
        self.get_logger().info(
            f'controller_enabled={not self.initial_pose_only}, '
            f'startup_hold_seconds={self.startup_hold_seconds:.2f}s, '
            f'disturbance_delay_after_hold={self.disturbance_time:.2f}s'
        )

        for name in joint_names:
            radians = float(self.q_map.get(name, 0.0))
            self.get_logger().info(
                f'{name} = {radians:.6f} rad = '
                f'{math.degrees(radians):.2f} deg'
            )

        self.log_base_transform_assumption()

    def log_base_transform_assumption(self):
        for label, chain in [
            ('left', self.left_chain),
            ('right', self.right_chain),
        ]:
            if not chain:
                continue

            base_joint = chain[0]
            if base_joint['type'] != 'fixed':
                continue

            xyz = base_joint.get('origin_xyz', [0.0, 0.0, 0.0])
            rpy = base_joint.get('origin_rpy', [0.0, 0.0, 0.0])
            self.get_logger().info(
                f'{label} base transform assumption: '
                f'{base_joint["parent"]} -> {base_joint["child"]}, '
                f'xyz=[{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}], '
                f'rpy=[{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}] rad '
                f'= [{math.degrees(rpy[0]):.2f}, '
                f'{math.degrees(rpy[1]):.2f}, '
                f'{math.degrees(rpy[2]):.2f}] deg'
            )

            yaw = rpy[2]
            if abs(abs(yaw) - math.pi) < 1e-3:
                self.get_logger().info(
                    f'{label.capitalize()} base already includes about '
                    f'180 deg yaw in the URDF/Xacro; {label}_joint_1 '
                    'defaults to 0.0 rad to avoid an extra yaw rotation.'
                )

    def log_first_published_joint_state(self, joint_names, positions):
        if self.first_joint_state_logged:
            return

        self.first_joint_state_logged = True
        self.get_logger().info(
            'First published /joint_states sample '
            '(compare with: ros2 topic echo /joint_states --once):'
        )
        for name, radians in zip(joint_names, positions):
            self.get_logger().info(
                f'published {name} = {radians:.6f} rad = '
                f'{math.degrees(radians):.2f} deg'
            )

    def initialize_state(self):
        for i, joint_name in enumerate(self.left_joint_names):
            if i < len(self.initial_left_positions):
                self.q_map[joint_name] = float(self.initial_left_positions[i])
            else:
                self.q_map[joint_name] = 0.0

        for i, joint_name in enumerate(self.right_joint_names):
            if i < len(self.initial_right_positions):
                self.q_map[joint_name] = float(self.initial_right_positions[i])
            else:
                self.q_map[joint_name] = 0.0

        left_pose = self.compute_fk_and_jacobian(self.left_chain)['pose']
        right_pose = self.compute_fk_and_jacobian(self.right_chain)['pose']

        self.desired_relative_position = (
            right_pose['position'] - left_pose['position']
        )

        self.desired_relative_quaternion = relative_quaternion(
            left_pose['quaternion'],
            right_pose['quaternion']
        )

        now = self.get_clock().now()
        self.start_time = now
        self.last_time = now
        self.last_log_time = now

        self.initialized = True

        self.log_initial_state()
        self.get_logger().info('Initial state published and desired relative pose initialized.')
        self.get_logger().info(
            f'desired_relative_position = '
            f'[{self.desired_relative_position[0]:.5f}, '
            f'{self.desired_relative_position[1]:.5f}, '
            f'{self.desired_relative_position[2]:.5f}]'
        )

    def timer_callback(self):
        if not self.robot_loaded:
            self.get_logger().warn('Waiting for /robot_description.')
            return

        if not self.initialized:
            self.initialize_state()
            self.publish_joint_state()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.control_rate

        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if self.initial_pose_only:
            self.last_time = now
            self.latest_qdot_map = {}
            self.publish_joint_state()
            return

        if elapsed < self.startup_hold_seconds:
            self.last_time = now
            self.latest_qdot_map = {}
            self.publish_joint_state()
            return

        if not self.hold_complete_logged:
            self.hold_complete_logged = True
            self.get_logger().info(
                'Startup hold complete; relative motion controller enabled.'
            )

        if (
            self.enable_disturbance
            and not self.disturbance_applied
            and elapsed >= self.startup_hold_seconds + self.disturbance_time
        ):
            if self.disturbance_joint in self.q_map:
                self.q_map[self.disturbance_joint] += self.disturbance_amount
                self.apply_joint_limits(self.disturbance_joint)
                self.disturbance_applied = True
                self.get_logger().warn(
                    f'Injected disturbance: {self.disturbance_joint} '
                    f'+= {self.disturbance_amount:.4f} rad'
                )
            else:
                self.disturbance_applied = True
                self.get_logger().error(
                    f'Disturbance joint not found: {self.disturbance_joint}'
                )

        self.control_step(dt)
        self.publish_joint_state()
        self.log_status_if_needed(now)

    def control_step(self, dt):
        left_result = self.compute_fk_and_jacobian(self.left_chain)
        right_result = self.compute_fk_and_jacobian(self.right_chain)

        left_pose = left_result['pose']
        right_pose = right_result['pose']

        current_relative_position = right_pose['position'] - left_pose['position']
        current_relative_quaternion = relative_quaternion(
            left_pose['quaternion'],
            right_pose['quaternion']
        )

        position_error = (
            self.desired_relative_position - current_relative_position
        )

        rotvec_error, orientation_error_angle = orientation_error_rotvec(
            current_relative_quaternion,
            self.desired_relative_quaternion
        )

        x_dot_rel = np.zeros(6)
        x_dot_rel[:3] = self.position_gain * position_error
        x_dot_rel[3:] = self.orientation_gain * rotvec_error

        J_left = left_result['J']
        J_right = right_result['J']

        J_rel = np.hstack((-J_left, J_right))
        J_rel_pinv = damped_pseudoinverse(J_rel, self.damping)

        q_dot_12 = J_rel_pinv @ x_dot_rel

        max_abs_qdot = float(np.max(np.abs(q_dot_12))) if q_dot_12.size > 0 else 0.0
        scale = 1.0

        if max_abs_qdot > self.max_joint_velocity:
            scale = self.max_joint_velocity / max_abs_qdot
            q_dot_12 = q_dot_12 * scale

        all_joint_names = self.left_joint_names + self.right_joint_names

        self.latest_qdot_map = {}

        for joint_name, qd in zip(all_joint_names, q_dot_12):
            self.latest_qdot_map[joint_name] = float(qd)
            self.q_map[joint_name] += float(qd) * dt
            self.apply_joint_limits(joint_name)

        singular_values = np.linalg.svd(J_rel, compute_uv=False)
        rank = np.linalg.matrix_rank(J_rel, tol=1e-5)

        if len(singular_values) > 0 and singular_values[-1] > 1e-8:
            condition_number = singular_values[0] / singular_values[-1]
        else:
            condition_number = float('inf')

        self.latest_status = {
            'relative_position_error_m': float(np.linalg.norm(position_error)),
            'relative_orientation_error_deg': math.degrees(orientation_error_angle),
            'x_dot_norm': float(np.linalg.norm(x_dot_rel)),
            'rank': rank,
            'condition_number': condition_number,
            'scale': scale,
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

        self.q_map[joint_name] = max(
            lower,
            min(upper, self.q_map[joint_name])
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        joint_names = self.joint_names()

        msg.name = joint_names
        msg.position = [self.q_map.get(name, 0.0) for name in joint_names]
        msg.velocity = [self.latest_qdot_map.get(name, 0.0) for name in joint_names]
        msg.effort = []

        self.joint_state_pub.publish(msg)
        self.log_first_published_joint_state(joint_names, msg.position)

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
            f'rank={s["rank"]} | '
            f'cond={s["condition_number"]:.3f} | '
            f'x_dot_norm={s["x_dot_norm"]:.5f} | '
            f'scale={s["scale"]:.3f} | '
            f'max_abs_qdot={max_abs_qdot:.4f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DualRelativeMotionDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
