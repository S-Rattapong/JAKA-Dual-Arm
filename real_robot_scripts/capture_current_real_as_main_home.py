#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CaptureMainHome(Node):
    def __init__(self):
        super().__init__("capture_current_real_as_main_home")
        self.left = None
        self.right = None
        self.create_subscription(JointState, "/left_jaka_driver/joint_position", self.left_cb, 10)
        self.create_subscription(JointState, "/right_jaka_driver/joint_position", self.right_cb, 10)

    def left_cb(self, msg):
        if len(msg.position) >= 6:
            self.left = [float(x) for x in msg.position[:6]]

    def right_cb(self, msg):
        if len(msg.position) >= 6:
            self.right = [float(x) for x in msg.position[:6]]

    def run(self):
        self.get_logger().warn("Waiting for current real robot joint positions...")
        while rclpy.ok() and (self.left is None or self.right is None):
            rclpy.spin_once(self, timeout_sec=0.1)

        data = {
            "description": "Main Real Home captured from current real JAKA A12 positions",
            "left_joint": self.left,
            "right_joint": self.right,
        }

        path = Path.home() / "jaka_ws" / "real_robot_configs" / "main_real_home.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2))

        self.get_logger().warn(f"Saved Main Real Home to: {path}")
        self.get_logger().warn("LEFT  = " + str(self.left))
        self.get_logger().warn("RIGHT = " + str(self.right))


def main():
    rclpy.init()
    node = CaptureMainHome()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
