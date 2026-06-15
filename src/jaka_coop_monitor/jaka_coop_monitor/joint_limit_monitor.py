import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String


class JointLimitMonitor(Node):
    def __init__(self):
        super().__init__('joint_limit_monitor')

        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('publish_rate', 2.0)

        self.declare_parameter('warning_margin_rad', 0.30)
        self.declare_parameter('danger_margin_rad', 0.10)

        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.robot_description_topic = self.get_parameter('robot_description_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.warning_margin_rad = float(
            self.get_parameter('warning_margin_rad').value
        )
        self.danger_margin_rad = float(
            self.get_parameter('danger_margin_rad').value
        )

        self.joint_limits = {}
        self.latest_joint_state = None
        self.received_robot_description = False

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
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info('Joint Limit Monitor started')
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
            self.received_robot_description = True

            self.get_logger().info(
                f'Loaded joint limits from robot_description: {len(self.joint_limits)} joints'
            )

            for name, lim in sorted(self.joint_limits.items()):
                self.get_logger().info(
                    f'{name}: lower={lim["lower"]:.4f}, upper={lim["upper"]:.4f}'
                )

        except Exception as ex:
            self.get_logger().warn(f'Failed to parse robot_description: {ex}')

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def timer_callback(self):
        if not self.received_robot_description:
            self.get_logger().warn(
                'Waiting for /robot_description. '
                'Make sure dual_jaka_a12_tf_demo.launch.py is running.'
            )
            return

        if self.latest_joint_state is None:
            self.get_logger().warn(
                'Waiting for /joint_states. '
                'Make sure joint_state_publisher_gui is running.'
            )
            return

        joint_map = dict(
            zip(self.latest_joint_state.name, self.latest_joint_state.position)
        )

        monitored_count = 0
        warning_count = 0
        danger_count = 0

        lines = []

        for joint_name in sorted(joint_map.keys()):
            if joint_name not in self.joint_limits:
                continue

            q = joint_map[joint_name]
            lower = self.joint_limits[joint_name]['lower']
            upper = self.joint_limits[joint_name]['upper']

            margin_lower = q - lower
            margin_upper = upper - q
            min_margin = min(margin_lower, margin_upper)

            joint_range = upper - lower
            margin_percent = 100.0 * min_margin / joint_range if joint_range > 0 else 0.0

            status = 'OK'

            if min_margin <= self.danger_margin_rad:
                status = 'DANGER'
                danger_count += 1
            elif min_margin <= self.warning_margin_rad:
                status = 'WARNING'
                warning_count += 1

            monitored_count += 1

            lines.append(
                f'{joint_name}: '
                f'q={q:+.4f} rad | '
                f'limit=[{lower:+.4f}, {upper:+.4f}] | '
                f'margin={min_margin:.4f} rad ({margin_percent:.1f}%) | '
                f'{status}'
            )

        self.get_logger().info(
            f'Joint limit summary: '
            f'monitored={monitored_count}, '
            f'warning={warning_count}, '
            f'danger={danger_count}'
        )

        for line in lines:
            if 'DANGER' in line:
                self.get_logger().error(line)
            elif 'WARNING' in line:
                self.get_logger().warn(line)
            else:
                self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = JointLimitMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
