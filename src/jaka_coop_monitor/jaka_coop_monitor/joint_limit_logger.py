import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String


class JointLimitLogger(Node):
    def __init__(self):
        super().__init__('joint_limit_logger')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('sample_rate', 5.0)
        self.declare_parameter('warning_margin_rad', 0.30)
        self.declare_parameter('danger_margin_rad', 0.10)
        self.declare_parameter('log_dir', str(Path.home() / 'jaka_ws' / 'logs'))

        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.warning_margin_rad = float(self.get_parameter('warning_margin_rad').value)
        self.danger_margin_rad = float(self.get_parameter('danger_margin_rad').value)
        self.log_dir = Path(self.get_parameter('log_dir').value)

        self.joint_limits = {}
        self.joint_names = []
        self.latest_joint_state = None
        self.received_robot_description = False

        self.csv_file = None
        self.writer = None

        self.start_time = self.get_clock().now()

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

        self.get_logger().info('Joint Limit Logger started')
        self.get_logger().info(f'Listening joint states from: {self.joint_states_topic}')
        self.get_logger().info(f'Listening robot description from: {self.robot_description_topic}')
        self.get_logger().info(
            f'warning_margin={self.warning_margin_rad:.3f} rad | '
            f'danger_margin={self.danger_margin_rad:.3f} rad'
        )

    def robot_description_callback(self, msg):
        if self.received_robot_description:
            return

        try:
            root = ET.fromstring(msg.data)
            limits = {}

            for joint in root.findall('joint'):
                joint_name = joint.attrib.get('name', '')
                joint_type = joint.attrib.get('type', '')

                if joint_type in ['fixed', 'continuous']:
                    continue

                limit_tag = joint.find('limit')
                if limit_tag is None:
                    continue

                lower = limit_tag.attrib.get('lower')
                upper = limit_tag.attrib.get('upper')

                if lower is None or upper is None:
                    continue

                limits[joint_name] = {
                    'lower': float(lower),
                    'upper': float(upper),
                }

            self.joint_limits = limits
            self.joint_names = sorted(self.joint_limits.keys())
            self.received_robot_description = True

            self.setup_csv()

            self.get_logger().info(
                f'Loaded joint limits from robot_description: {len(self.joint_names)} joints'
            )
            self.get_logger().info(f'Logging CSV to: {self.csv_path}')

        except Exception as ex:
            self.get_logger().warn(f'Failed to parse robot_description: {ex}')

    def setup_csv(self):
        self.log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f'joint_limit_log_{timestamp}.csv'

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        header = ['time_sec']

        for joint_name in self.joint_names:
            header.extend([
                f'{joint_name}_position_rad',
                f'{joint_name}_lower_limit_rad',
                f'{joint_name}_upper_limit_rad',
                f'{joint_name}_margin_rad',
                f'{joint_name}_margin_percent',
                f'{joint_name}_activation',
                f'{joint_name}_status',
            ])

        header.extend([
            'warning_count',
            'danger_count',
            'minimum_margin_rad',
            'minimum_margin_joint',
        ])

        self.writer.writerow(header)
        self.csv_file.flush()

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def timer_callback(self):
        if not self.received_robot_description:
            self.get_logger().warn('Waiting for /robot_description.')
            return

        if self.latest_joint_state is None:
            self.get_logger().warn('Waiting for /joint_states.')
            return

        if self.writer is None:
            return

        joint_map = dict(
            zip(self.latest_joint_state.name, self.latest_joint_state.position)
        )

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds * 1e-9

        row = [elapsed]

        warning_count = 0
        danger_count = 0
        minimum_margin = float('inf')
        minimum_margin_joint = ''

        for joint_name in self.joint_names:
            lower = self.joint_limits[joint_name]['lower']
            upper = self.joint_limits[joint_name]['upper']

            if joint_name not in joint_map:
                row.extend([
                    math.nan,
                    lower,
                    upper,
                    math.nan,
                    math.nan,
                    math.nan,
                    'MISSING',
                ])
                continue

            q = joint_map[joint_name]

            margin_lower = q - lower
            margin_upper = upper - q
            min_margin = min(margin_lower, margin_upper)

            joint_range = upper - lower
            margin_percent = 100.0 * min_margin / joint_range if joint_range > 0.0 else 0.0

            status = 'OK'
            if min_margin <= self.danger_margin_rad:
                status = 'DANGER'
                danger_count += 1
            elif min_margin <= self.warning_margin_rad:
                status = 'WARNING'
                warning_count += 1

            activation = self.compute_activation(min_margin)

            if min_margin < minimum_margin:
                minimum_margin = min_margin
                minimum_margin_joint = joint_name

            row.extend([
                q,
                lower,
                upper,
                min_margin,
                margin_percent,
                activation,
                status,
            ])

        row.extend([
            warning_count,
            danger_count,
            minimum_margin,
            minimum_margin_joint,
        ])

        self.writer.writerow(row)
        self.csv_file.flush()

        self.get_logger().info(
            f't={elapsed:.2f}s | '
            f'warning={warning_count} | '
            f'danger={danger_count} | '
            f'min_margin={minimum_margin:.4f} rad at {minimum_margin_joint}'
        )

    def compute_activation(self, margin):
        if margin >= self.warning_margin_rad:
            return 0.0

        if margin <= self.danger_margin_rad:
            return 1.0

        denominator = self.warning_margin_rad - self.danger_margin_rad
        return (self.warning_margin_rad - margin) / denominator

    def destroy_node(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointLimitLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
