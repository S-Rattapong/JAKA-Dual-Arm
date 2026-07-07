#!/usr/bin/env python3
import argparse
import re
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def joint_num(name):
    n = name.lower()
    m = re.search(r"(?:joint[_-]?|j)([1-6])\b", n)
    if not m:
        return 999
    return int(m.group(1))


class RealTwinJointStatePublisher(Node):
    def __init__(self, args):
        super().__init__("real_twin_joint_state_publisher")

        self.left_real = None
        self.right_real = None

        self.left_names, self.right_names = self.load_twin_joint_names(args.urdf)

        self.get_logger().warn("Real twin left joints : " + str(self.left_names))
        self.get_logger().warn("Real twin right joints: " + str(self.right_names))

        self.create_subscription(JointState, "/left_jaka_driver/joint_position", self.left_cb, 10)
        self.create_subscription(JointState, "/right_jaka_driver/joint_position", self.right_cb, 10)

        self.pub = self.create_publisher(JointState, "/real_twin/joint_states", 10)
        self.timer = self.create_timer(0.05, self.tick)

    def load_twin_joint_names(self, urdf_path):
        path = Path(urdf_path).expanduser()
        root = ET.parse(path).getroot()

        movable = []
        for joint in root.findall("joint"):
            if joint.attrib.get("type", "") == "fixed":
                continue
            name = joint.attrib.get("name", "")
            movable.append(name)

        left = [n for n in movable if "real_left" in n.lower()]
        right = [n for n in movable if "real_right" in n.lower()]

        left = sorted(left, key=joint_num)
        right = sorted(right, key=joint_num)

        if len(left) < 6 or len(right) < 6:
            raise RuntimeError(f"Cannot find 6 left/right joints in URDF. left={left}, right={right}")

        return left[:6], right[:6]

    def left_cb(self, msg):
        if len(msg.position) >= 6:
            self.left_real = list(msg.position)[:6]

    def right_cb(self, msg):
        if len(msg.position) >= 6:
            self.right_real = list(msg.position)[:6]

    def tick(self):
        if self.left_real is None or self.right_real is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.left_names + self.right_names
        msg.position = self.left_real + self.right_real
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        default="~/jaka_ws/real_robot_configs/dual_jaka_a12_real_twin.urdf",
    )
    args = parser.parse_args()

    rclpy.init()
    node = RealTwinJointStatePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
