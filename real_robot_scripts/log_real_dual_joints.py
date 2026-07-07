#!/usr/bin/env python3
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DualJointLogger(Node):
    def __init__(self):
        super().__init__("dual_real_joint_logger")
        self.left = None
        self.right = None
        self.start = time.time()

        path = Path.home() / "jaka_ws" / "real_robot_logs" / "dual_real_motion_log.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "w", newline="")
        self.writer = csv.writer(self.f)
        self.writer.writerow([
            "time_s",
            "left_j1","left_j2","left_j3","left_j4","left_j5","left_j6",
            "right_j1","right_j2","right_j3","right_j4","right_j5","right_j6",
        ])

        self.create_subscription(JointState, "/left_jaka_driver/joint_position", self.left_cb, 10)
        self.create_subscription(JointState, "/right_jaka_driver/joint_position", self.right_cb, 10)
        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().warn(f"Logging to {path}")

    def left_cb(self, msg):
        self.left = list(msg.position)

    def right_cb(self, msg):
        self.right = list(msg.position)

    def tick(self):
        if self.left is None or self.right is None:
            return
        t = time.time() - self.start
        self.writer.writerow([f"{t:.3f}"] + self.left + self.right)
        self.f.flush()

def main():
    rclpy.init()
    node = DualJointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
