import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster


class VirtualObjectFramePublisher(Node):
    def __init__(self):
        super().__init__('virtual_object_frame_publisher')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('left_ee_frame', 'left_J6')
        self.declare_parameter('right_ee_frame', 'right_J6')
        self.declare_parameter('object_frame', 'virtual_object')
        self.declare_parameter('publish_rate', 30.0)

        self.reference_frame = self.get_parameter('reference_frame').value
        self.left_ee_frame = self.get_parameter('left_ee_frame').value
        self.right_ee_frame = self.get_parameter('right_ee_frame').value
        self.object_frame = self.get_parameter('object_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Virtual Object Frame Publisher started: '
            f'{self.left_ee_frame}, {self.right_ee_frame} -> {self.object_frame}'
        )

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

            center = [
                0.5 * (p_left[0] + p_right[0]),
                0.5 * (p_left[1] + p_right[1]),
                0.5 * (p_left[2] + p_right[2]),
            ]

            x_axis = [
                p_right[0] - p_left[0],
                p_right[1] - p_left[1],
                p_right[2] - p_left[2],
            ]

            x_axis = self.normalize_vector(x_axis)

            # Use world Z as reference. If nearly parallel, use world Y instead.
            z_ref = [0.0, 0.0, 1.0]
            if abs(self.dot(x_axis, z_ref)) > 0.95:
                z_ref = [0.0, 1.0, 0.0]

            y_axis = self.normalize_vector(self.cross(z_ref, x_axis))
            z_axis = self.normalize_vector(self.cross(x_axis, y_axis))

            q = self.rotation_matrix_to_quaternion(x_axis, y_axis, z_axis)

            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.reference_frame
            msg.child_frame_id = self.object_frame

            msg.transform.translation.x = center[0]
            msg.transform.translation.y = center[1]
            msg.transform.translation.z = center[2]

            msg.transform.rotation.x = q[0]
            msg.transform.rotation.y = q[1]
            msg.transform.rotation.z = q[2]
            msg.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(msg)

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not publish virtual object frame: {ex}'
            )

    @staticmethod
    def normalize_vector(v):
        norm = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if norm < 1e-12:
            return [1.0, 0.0, 0.0]
        return [v[0] / norm, v[1] / norm, v[2] / norm]

    @staticmethod
    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def cross(a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    @staticmethod
    def rotation_matrix_to_quaternion(x_axis, y_axis, z_axis):
        # Rotation matrix columns are x_axis, y_axis, z_axis
        r00, r01, r02 = x_axis[0], y_axis[0], z_axis[0]
        r10, r11, r12 = x_axis[1], y_axis[1], z_axis[1]
        r20, r21, r22 = x_axis[2], y_axis[2], z_axis[2]

        trace = r00 + r11 + r22

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (r21 - r12) / s
            qy = (r02 - r20) / s
            qz = (r10 - r01) / s
        elif r00 > r11 and r00 > r22:
            s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
            qw = (r21 - r12) / s
            qx = 0.25 * s
            qy = (r01 + r10) / s
            qz = (r02 + r20) / s
        elif r11 > r22:
            s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
            qw = (r02 - r20) / s
            qx = (r01 + r10) / s
            qy = 0.25 * s
            qz = (r12 + r21) / s
        else:
            s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
            qw = (r10 - r01) / s
            qx = (r02 + r20) / s
            qy = (r12 + r21) / s
            qz = 0.25 * s

        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = VirtualObjectFramePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
