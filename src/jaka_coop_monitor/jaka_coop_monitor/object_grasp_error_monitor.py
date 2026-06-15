import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class ObjectGraspErrorMonitor(Node):
    def __init__(self):
        super().__init__('object_grasp_error_monitor')

        self.declare_parameter('reference_frame', 'world')

        self.declare_parameter('left_actual_frame', 'left_J6')
        self.declare_parameter('right_actual_frame', 'right_J6')

        self.declare_parameter('left_desired_frame', 'desired_left_grasp')
        self.declare_parameter('right_desired_frame', 'desired_right_grasp')

        self.declare_parameter('publish_rate', 2.0)

        self.reference_frame = self.get_parameter('reference_frame').value

        self.left_actual_frame = self.get_parameter('left_actual_frame').value
        self.right_actual_frame = self.get_parameter('right_actual_frame').value

        self.left_desired_frame = self.get_parameter('left_desired_frame').value
        self.right_desired_frame = self.get_parameter('right_desired_frame').value

        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('Object Grasp Error Monitor started')
        self.get_logger().info(
            f'Left : {self.left_actual_frame} vs {self.left_desired_frame}'
        )
        self.get_logger().info(
            f'Right: {self.right_actual_frame} vs {self.right_desired_frame}'
        )

    def timer_callback(self):
        try:
            left_actual = self.lookup_pose(self.left_actual_frame)
            right_actual = self.lookup_pose(self.right_actual_frame)

            left_desired = self.lookup_pose(self.left_desired_frame)
            right_desired = self.lookup_pose(self.right_desired_frame)

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

            mean_pos_error = 0.5 * (left_pos_error + right_pos_error)
            max_pos_error = max(left_pos_error, right_pos_error)

            mean_ori_error = 0.5 * (left_ori_error + right_ori_error)
            max_ori_error = max(left_ori_error, right_ori_error)

            self.get_logger().info(
                f'L_pos_err={left_pos_error:.5f} m | '
                f'R_pos_err={right_pos_error:.5f} m | '
                f'mean_pos_err={mean_pos_error:.5f} m | '
                f'max_pos_err={max_pos_error:.5f} m | '
                f'L_ori_err={math.degrees(left_ori_error):.3f} deg | '
                f'R_ori_err={math.degrees(right_ori_error):.3f} deg | '
                f'mean_ori_err={math.degrees(mean_ori_error):.3f} deg | '
                f'max_ori_err={math.degrees(max_ori_error):.3f} deg'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not compute object grasp error: {ex}'
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
        actual_q = ObjectGraspErrorMonitor.normalize_quaternion(actual_q)
        desired_q = ObjectGraspErrorMonitor.normalize_quaternion(desired_q)

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


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGraspErrorMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
