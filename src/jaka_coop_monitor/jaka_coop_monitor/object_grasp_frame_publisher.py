import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
    StaticTransformBroadcaster,
)


class ObjectGraspFramePublisher(Node):
    def __init__(self):
        super().__init__('object_grasp_frame_publisher')

        self.declare_parameter('object_frame', 'virtual_object')
        self.declare_parameter('left_actual_frame', 'left_J6')
        self.declare_parameter('right_actual_frame', 'right_J6')
        self.declare_parameter('left_grasp_frame', 'desired_left_grasp')
        self.declare_parameter('right_grasp_frame', 'desired_right_grasp')
        self.declare_parameter('publish_rate', 5.0)

        self.object_frame = self.get_parameter('object_frame').value
        self.left_actual_frame = self.get_parameter('left_actual_frame').value
        self.right_actual_frame = self.get_parameter('right_actual_frame').value
        self.left_grasp_frame = self.get_parameter('left_grasp_frame').value
        self.right_grasp_frame = self.get_parameter('right_grasp_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.calibrated = False

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info('Object Grasp Frame Publisher started')
        self.get_logger().info(
            'Auto-calibration mode: using current EE poses as desired grasp frames'
        )
        self.get_logger().info(
            f'Waiting for transforms: '
            f'{self.object_frame} -> {self.left_actual_frame}, '
            f'{self.object_frame} -> {self.right_actual_frame}'
        )

    def timer_callback(self):
        if self.calibrated:
            return

        try:
            left_tf = self.tf_buffer.lookup_transform(
                self.object_frame,
                self.left_actual_frame,
                rclpy.time.Time()
            )

            right_tf = self.tf_buffer.lookup_transform(
                self.object_frame,
                self.right_actual_frame,
                rclpy.time.Time()
            )

            left_grasp_tf = self.create_grasp_transform(
                source_tf=left_tf,
                child_frame=self.left_grasp_frame
            )

            right_grasp_tf = self.create_grasp_transform(
                source_tf=right_tf,
                child_frame=self.right_grasp_frame
            )

            self.static_broadcaster.sendTransform([
                left_grasp_tf,
                right_grasp_tf
            ])

            self.calibrated = True

            self.get_logger().info('Grasp frames calibrated successfully.')
            self.get_logger().info(
                f'{self.object_frame} -> {self.left_grasp_frame} '
                f'calibrated from {self.left_actual_frame}'
            )
            self.get_logger().info(
                f'{self.object_frame} -> {self.right_grasp_frame} '
                f'calibrated from {self.right_actual_frame}'
            )

            self.log_transform('left grasp', left_grasp_tf)
            self.log_transform('right grasp', right_grasp_tf)

        except TransformException as ex:
            self.get_logger().warn(
                f'Waiting for calibration transforms: {ex}'
            )

    def create_grasp_transform(self, source_tf, child_frame):
        msg = TransformStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.object_frame
        msg.child_frame_id = child_frame

        msg.transform.translation.x = source_tf.transform.translation.x
        msg.transform.translation.y = source_tf.transform.translation.y
        msg.transform.translation.z = source_tf.transform.translation.z

        msg.transform.rotation.x = source_tf.transform.rotation.x
        msg.transform.rotation.y = source_tf.transform.rotation.y
        msg.transform.rotation.z = source_tf.transform.rotation.z
        msg.transform.rotation.w = source_tf.transform.rotation.w

        return msg

    def log_transform(self, label, tf_msg):
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        self.get_logger().info(
            f'{label}: '
            f'xyz=[{t.x:.5f}, {t.y:.5f}, {t.z:.5f}], '
            f'quat=[{q.x:.5f}, {q.y:.5f}, {q.z:.5f}, {q.w:.5f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGraspFramePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()