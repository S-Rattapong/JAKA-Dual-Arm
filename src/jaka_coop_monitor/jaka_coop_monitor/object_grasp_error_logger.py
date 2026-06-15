import csv
import math
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class ObjectGraspErrorLogger(Node):
    def __init__(self):
        super().__init__('object_grasp_error_logger')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('left_actual_frame', 'left_J6')
        self.declare_parameter('right_actual_frame', 'right_J6')
        self.declare_parameter('left_desired_frame', 'desired_left_grasp')
        self.declare_parameter('right_desired_frame', 'desired_right_grasp')
        self.declare_parameter('sample_rate', 10.0)
        self.declare_parameter('log_dir', str(Path.home() / 'jaka_ws' / 'logs'))

        self.reference_frame = self.get_parameter('reference_frame').value
        self.left_actual_frame = self.get_parameter('left_actual_frame').value
        self.right_actual_frame = self.get_parameter('right_actual_frame').value
        self.left_desired_frame = self.get_parameter('left_desired_frame').value
        self.right_desired_frame = self.get_parameter('right_desired_frame').value
        self.sample_rate = float(self.get_parameter('sample_rate').value)
        self.log_dir = Path(self.get_parameter('log_dir').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = self.log_dir / f'object_grasp_error_{timestamp}.csv'

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time_sec',

            'left_actual_x', 'left_actual_y', 'left_actual_z',
            'right_actual_x', 'right_actual_y', 'right_actual_z',

            'left_desired_x', 'left_desired_y', 'left_desired_z',
            'right_desired_x', 'right_desired_y', 'right_desired_z',

            'left_position_error_m',
            'right_position_error_m',
            'mean_position_error_m',
            'max_position_error_m',

            'left_orientation_error_deg',
            'right_orientation_error_deg',
            'mean_orientation_error_deg',
            'max_orientation_error_deg'
        ])

        self.timer = self.create_timer(1.0 / self.sample_rate, self.timer_callback)

        self.get_logger().info('Object Grasp Error Logger started')
        self.get_logger().info(f'Logging CSV to: {self.csv_path}')

    def timer_callback(self):
        try:
            left_actual = self.lookup_pose(self.left_actual_frame)
            right_actual = self.lookup_pose(self.right_actual_frame)
            left_desired = self.lookup_pose(self.left_desired_frame)
            right_desired = self.lookup_pose(self.right_desired_frame)

            elapsed = (
                self.get_clock().now() - self.start_time
            ).nanoseconds * 1e-9

            left_pos_error = self.position_error(
                left_actual['position'],
                left_desired['position']
            )

            right_pos_error = self.position_error(
                right_actual['position'],
                right_desired['position']
            )

            left_ori_error = self.orientation_error(
                left_actual['quaternion'],
                left_desired['quaternion']
            )

            right_ori_error = self.orientation_error(
                right_actual['quaternion'],
                right_desired['quaternion']
            )

            left_ori_deg = math.degrees(left_ori_error)
            right_ori_deg = math.degrees(right_ori_error)

            mean_pos_error = 0.5 * (left_pos_error + right_pos_error)
            max_pos_error = max(left_pos_error, right_pos_error)

            mean_ori_deg = 0.5 * (left_ori_deg + right_ori_deg)
            max_ori_deg = max(left_ori_deg, right_ori_deg)

            self.writer.writerow([
                elapsed,

                left_actual['position'][0],
                left_actual['position'][1],
                left_actual['position'][2],

                right_actual['position'][0],
                right_actual['position'][1],
                right_actual['position'][2],

                left_desired['position'][0],
                left_desired['position'][1],
                left_desired['position'][2],

                right_desired['position'][0],
                right_desired['position'][1],
                right_desired['position'][2],

                left_pos_error,
                right_pos_error,
                mean_pos_error,
                max_pos_error,

                left_ori_deg,
                right_ori_deg,
                mean_ori_deg,
                max_ori_deg
            ])

            self.csv_file.flush()

            self.get_logger().info(
                f't={elapsed:.2f}s | '
                f'L_pos_err={left_pos_error:.5f} m | '
                f'R_pos_err={right_pos_error:.5f} m | '
                f'mean_pos_err={mean_pos_error:.5f} m | '
                f'L_ori_err={left_ori_deg:.3f} deg | '
                f'R_ori_err={right_ori_deg:.3f} deg'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not log object grasp error: {ex}'
            )

    def lookup_pose(self, frame_name):
        tf = self.tf_buffer.lookup_transform(
            self.reference_frame,
            frame_name,
            rclpy.time.Time()
        )

        t = tf.transform.translation
        q = tf.transform.rotation

        return {
            'position': [t.x, t.y, t.z],
            'quaternion': [q.x, q.y, q.z, q.w],
        }

    @staticmethod
    def position_error(actual, desired):
        dx = actual[0] - desired[0]
        dy = actual[1] - desired[1]
        dz = actual[2] - desired[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def orientation_error(actual_q, desired_q):
        actual_q = ObjectGraspErrorLogger.normalize_quaternion(actual_q)
        desired_q = ObjectGraspErrorLogger.normalize_quaternion(desired_q)

        dot = (
            actual_q[0] * desired_q[0]
            + actual_q[1] * desired_q[1]
            + actual_q[2] * desired_q[2]
            + actual_q[3] * desired_q[3]
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
            q[3] / norm,
        ]

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGraspErrorLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
