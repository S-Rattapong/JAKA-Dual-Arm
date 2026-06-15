import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class PoseErrorMonitor(Node):
    def __init__(self):
        super().__init__('pose_error_monitor')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('ee_frame', 'J6')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('auto_set_desired', True)

        self.declare_parameter('desired_x', 0.0)
        self.declare_parameter('desired_y', 0.0)
        self.declare_parameter('desired_z', 0.0)
        self.declare_parameter('desired_qx', 0.0)
        self.declare_parameter('desired_qy', 0.0)
        self.declare_parameter('desired_qz', 0.0)
        self.declare_parameter('desired_qw', 1.0)

        self.reference_frame = self.get_parameter('reference_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.auto_set_desired = bool(self.get_parameter('auto_set_desired').value)

        self.desired_position = [
            float(self.get_parameter('desired_x').value),
            float(self.get_parameter('desired_y').value),
            float(self.get_parameter('desired_z').value),
        ]

        self.desired_quaternion = [
            float(self.get_parameter('desired_qx').value),
            float(self.get_parameter('desired_qy').value),
            float(self.get_parameter('desired_qz').value),
            float(self.get_parameter('desired_qw').value),
        ]

        self.desired_initialized = not self.auto_set_desired

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Pose Error Monitor started: {self.reference_frame} -> {self.ee_frame}'
        )

        if self.auto_set_desired:
            self.get_logger().info(
                'auto_set_desired=True: first valid EE pose will be used as desired pose.'
            )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.ee_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            current_position = [t.x, t.y, t.z]
            current_quaternion = [q.x, q.y, q.z, q.w]

            if not self.desired_initialized:
                self.desired_position = current_position.copy()
                self.desired_quaternion = current_quaternion.copy()
                self.desired_initialized = True

                self.get_logger().info(
                    'Desired pose initialized from current pose: '
                    f'pos = [{t.x:.4f}, {t.y:.4f}, {t.z:.4f}], '
                    f'quat = [{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]'
                )
                return

            position_error = self.compute_position_error(
                current_position,
                self.desired_position
            )

            orientation_error = self.compute_orientation_error(
                current_quaternion,
                self.desired_quaternion
            )

            self.get_logger().info(
                f'actual pos = [{current_position[0]:.4f}, '
                f'{current_position[1]:.4f}, {current_position[2]:.4f}] | '
                f'pos_error = {position_error:.5f} m | '
                f'ori_error = {math.degrees(orientation_error):.3f} deg'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.reference_frame} to {self.ee_frame}: {ex}'
            )

    @staticmethod
    def compute_position_error(current, desired):
        dx = current[0] - desired[0]
        dy = current[1] - desired[1]
        dz = current[2] - desired[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def compute_orientation_error(current_q, desired_q):
        current_q = PoseErrorMonitor.normalize_quaternion(current_q)
        desired_q = PoseErrorMonitor.normalize_quaternion(desired_q)

        dot = (
            current_q[0] * desired_q[0]
            + current_q[1] * desired_q[1]
            + current_q[2] * desired_q[2]
            + current_q[3] * desired_q[3]
        )

        dot = abs(dot)
        dot = max(min(dot, 1.0), -1.0)

        return 2.0 * math.acos(dot)

    @staticmethod
    def normalize_quaternion(q):
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if norm < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]


def main(args=None):
    rclpy.init(args=args)
    node = PoseErrorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
