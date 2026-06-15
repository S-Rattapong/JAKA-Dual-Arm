import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class EEPoseMonitor(Node):
    def __init__(self):
        super().__init__('ee_pose_monitor')

        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('ee_frame', 'J6')
        self.declare_parameter('publish_rate', 2.0)

        self.reference_frame = self.get_parameter('reference_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'EE Pose Monitor started: {self.reference_frame} -> {self.ee_frame}'
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.ee_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            self.get_logger().info(
                f'{self.ee_frame} in {self.reference_frame}: '
                f'pos = [{t.x:.4f}, {t.y:.4f}, {t.z:.4f}], '
                f'quat = [{r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f}]'
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.reference_frame} to {self.ee_frame}: {ex}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = EEPoseMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
