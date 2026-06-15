import csv
import math
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class PoseErrorLogger(Node):
    def __init__(self):
        super().__init__('pose_error_logger')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('ee_frame', 'J6')
        self.declare_parameter('sample_rate', 10.0)
        self.declare_parameter('auto_set_desired', True)
        self.declare_parameter('log_dir', str(Path.home() / 'jaka_ws' / 'logs'))

        self.reference_frame = self.get_parameter('reference_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.auto_set_desired = bool(self.get_parameter('auto_set_desired').value)
        self.log_dir = Path(self.get_parameter('log_dir').value)

        self.desired_position = None
        self.desired_quaternion = None

        self.start_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f'single_jaka_a12_pose_error_{timestamp}.csv'

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time_sec',
            'x', 'y', 'z',
            'qx', 'qy', 'qz', 'qw',
            'desired_x', 'desired_y', 'desired_z',
            'desired_qx', 'desired_qy', 'desired_qz', 'desired_qw',
            'position_error_m',
            'orientation_error_deg'
        ])

        self.timer = self.create_timer(1.0 / self.sample_rate, self.timer_callback)

        self.get_logger().info(
            f'Pose Error Logger started: {self.reference_frame} -> {self.ee_frame}'
        )
        self.get_logger().info(f'Logging CSV to: {self.csv_path}')

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.ee_frame,
                rclpy.time.Time()
            )

            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9

            t = transform.transform.translation
            q = transform.transform.rotation

            current_position = [t.x, t.y, t.z]
            current_quaternion = [q.x, q.y, q.z, q.w]

            if self.desired_position is None or self.desired_quaternion is None:
                self.desired_position = current_position.copy()
                self.desired_quaternion = current_quaternion.copy()

                self.get_logger().info(
                    'Desired pose initialized from current EE pose.'
                )
                return

            position_error = self.compute_position_error(
                current_position,
                self.desired_position
            )

            orientation_error_rad = self.compute_orientation_error(
                current_quaternion,
                self.desired_quaternion
            )

            orientation_error_deg = math.degrees(orientation_error_rad)

            self.writer.writerow([
                elapsed,
                current_position[0], current_position[1], current_position[2],
                current_quaternion[0], current_quaternion[1],
                current_quaternion[2], current_quaternion[3],
                self.desired_position[0], self.desired_position[1],
                self.desired_position[2],
                self.desired_quaternion[0], self.desired_quaternion[1],
                self.desired_quaternion[2], self.desired_quaternion[3],
                position_error,
                orientation_error_deg
            ])

            self.csv_file.flush()

            self.get_logger().info(
                f't={elapsed:.2f}s | '
                f'pos=[{current_position[0]:.4f}, '
                f'{current_position[1]:.4f}, {current_position[2]:.4f}] | '
                f'pos_error={position_error:.5f} m | '
                f'ori_error={orientation_error_deg:.3f} deg'
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
        current_q = PoseErrorLogger.normalize_quaternion(current_q)
        desired_q = PoseErrorLogger.normalize_quaternion(desired_q)

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
        norm = math.sqrt(
            q[0] * q[0]
            + q[1] * q[1]
            + q[2] * q[2]
            + q[3] * q[3]
        )

        if norm < 1e-12:
            return [0.0, 0.0, 0.0, 1.0]

        return [
            q[0] / norm,
            q[1] / norm,
            q[2] / norm,
            q[3] / norm
        ]

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseErrorLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
