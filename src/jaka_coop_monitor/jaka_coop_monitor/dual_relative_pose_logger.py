import csv
import math
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class DualRelativePoseLogger(Node):
    def __init__(self):
        super().__init__('dual_relative_pose_logger')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('left_ee_frame', 'left_J6')
        self.declare_parameter('right_ee_frame', 'right_J6')
        self.declare_parameter('sample_rate', 10.0)
        self.declare_parameter('log_dir', str(Path.home() / 'jaka_ws' / 'logs'))

        self.reference_frame = self.get_parameter('reference_frame').value
        self.left_ee_frame = self.get_parameter('left_ee_frame').value
        self.right_ee_frame = self.get_parameter('right_ee_frame').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.log_dir = Path(self.get_parameter('log_dir').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.desired_distance = None
        self.desired_object_center = None
        self.desired_relative_quaternion = None

        self.start_time = self.get_clock().now()

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f'dual_jaka_a12_relative_pose_{timestamp}.csv'

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time_sec',

            'left_x', 'left_y', 'left_z',
            'left_qx', 'left_qy', 'left_qz', 'left_qw',

            'right_x', 'right_y', 'right_z',
            'right_qx', 'right_qy', 'right_qz', 'right_qw',

            'object_center_x', 'object_center_y', 'object_center_z',
            'desired_center_x', 'desired_center_y', 'desired_center_z',

            'relative_distance_m',
            'desired_relative_distance_m',
            'relative_distance_error_m',
            'object_center_error_m',
            'relative_orientation_error_deg'
        ])

        self.timer = self.create_timer(1.0 / self.sample_rate, self.timer_callback)

        self.get_logger().info(
            f'Dual Relative Pose Logger started: '
            f'{self.left_ee_frame} <-> {self.right_ee_frame} in {self.reference_frame}'
        )
        self.get_logger().info(f'Logging CSV to: {self.csv_path}')

    def timer_callback(self):
        try:
            left_tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.left_ee_frame,
                rclpy.time.Time()
            )

            right_tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.right_ee_frame,
                rclpy.time.Time()
            )

            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9

            p_left = [
                left_tf.transform.translation.x,
                left_tf.transform.translation.y,
                left_tf.transform.translation.z,
            ]

            p_right = [
                right_tf.transform.translation.x,
                right_tf.transform.translation.y,
                right_tf.transform.translation.z,
            ]

            q_left = [
                left_tf.transform.rotation.x,
                left_tf.transform.rotation.y,
                left_tf.transform.rotation.z,
                left_tf.transform.rotation.w,
            ]

            q_right = [
                right_tf.transform.rotation.x,
                right_tf.transform.rotation.y,
                right_tf.transform.rotation.z,
                right_tf.transform.rotation.w,
            ]

            relative_distance = self.distance(p_left, p_right)

            object_center = [
                0.5 * (p_left[0] + p_right[0]),
                0.5 * (p_left[1] + p_right[1]),
                0.5 * (p_left[2] + p_right[2]),
            ]

            relative_quaternion = self.relative_quaternion(q_left, q_right)

            if self.desired_distance is None:
                self.desired_distance = relative_distance
                self.desired_object_center = object_center.copy()
                self.desired_relative_quaternion = relative_quaternion.copy()

                self.get_logger().info(
                    'Desired relative pose initialized from current dual-arm pose.'
                )
                self.get_logger().info(
                    f'desired_distance={self.desired_distance:.5f} m | '
                    f'desired_center=[{object_center[0]:.4f}, '
                    f'{object_center[1]:.4f}, {object_center[2]:.4f}]'
                )
                return

            distance_error = relative_distance - self.desired_distance
            abs_distance_error = abs(distance_error)

            center_error = self.distance(object_center, self.desired_object_center)

            relative_orientation_error_rad = self.quaternion_angle_error(
                relative_quaternion,
                self.desired_relative_quaternion
            )

            relative_orientation_error_deg = math.degrees(relative_orientation_error_rad)

            self.writer.writerow([
                elapsed,

                p_left[0], p_left[1], p_left[2],
                q_left[0], q_left[1], q_left[2], q_left[3],

                p_right[0], p_right[1], p_right[2],
                q_right[0], q_right[1], q_right[2], q_right[3],

                object_center[0], object_center[1], object_center[2],
                self.desired_object_center[0],
                self.desired_object_center[1],
                self.desired_object_center[2],

                relative_distance,
                self.desired_distance,
                abs_distance_error,
                center_error,
                relative_orientation_error_deg
            ])

            self.csv_file.flush()

            self.get_logger().info(
                f't={elapsed:.2f}s | '
                f'dist={relative_distance:.5f} m | '
                f'dist_err={abs_distance_error:.5f} m | '
                f'center=[{object_center[0]:.3f}, '
                f'{object_center[1]:.3f}, {object_center[2]:.3f}] | '
                f'center_err={center_error:.5f} m | '
                f'rel_ori_err={relative_orientation_error_deg:.3f} deg'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not read dual EE transforms: {ex}'
            )

    @staticmethod
    def distance(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

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

    @staticmethod
    def quaternion_inverse(q):
        q = DualRelativePoseLogger.normalize_quaternion(q)
        return [-q[0], -q[1], -q[2], q[3]]

    @staticmethod
    def quaternion_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]

    @staticmethod
    def relative_quaternion(q_left, q_right):
        q_left_inv = DualRelativePoseLogger.quaternion_inverse(q_left)
        q_rel = DualRelativePoseLogger.quaternion_multiply(q_left_inv, q_right)
        return DualRelativePoseLogger.normalize_quaternion(q_rel)

    @staticmethod
    def quaternion_angle_error(q_current, q_desired):
        q_current = DualRelativePoseLogger.normalize_quaternion(q_current)
        q_desired = DualRelativePoseLogger.normalize_quaternion(q_desired)

        dot = (
            q_current[0] * q_desired[0]
            + q_current[1] * q_desired[1]
            + q_current[2] * q_desired[2]
            + q_current[3] * q_desired[3]
        )

        dot = abs(dot)
        dot = max(min(dot, 1.0), -1.0)

        return 2.0 * math.acos(dot)

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualRelativePoseLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
