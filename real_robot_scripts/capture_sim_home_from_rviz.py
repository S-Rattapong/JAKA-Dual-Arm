#!/usr/bin/env python3
import json
import re
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def find_joint_number(name: str, side: str) -> Optional[int]:
    n = name.lower()
    if not n.startswith(side.lower()):
        return None
    for p in [r"(?:joint[_-]?)([1-6])\b", r"(?:j)([1-6])\b"]:
        m = re.search(p, n)
        if m:
            return int(m.group(1))
    return None


class CaptureSimHome(Node):
    def __init__(self):
        super().__init__("capture_sim_home_from_rviz")
        self.msg = None
        self.create_subscription(JointState, "/joint_states", self.cb, 10)

    def cb(self, msg):
        self.msg = msg

    def capture(self):
        self.get_logger().warn("Waiting for /joint_states...")
        while rclpy.ok() and self.msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        msg = self.msg
        left_map = {}
        right_map = {}

        for i, name in enumerate(msg.name):
            ln = find_joint_number(name, "left")
            rn = find_joint_number(name, "right")
            if ln is not None:
                left_map[ln] = i
            if rn is not None:
                right_map[rn] = i

        if len(left_map) != 6 or len(right_map) != 6:
            raise RuntimeError(f"Cannot find 6 left/right joints. Names: {list(msg.name)}")

        left_indices = [left_map[i] for i in range(1, 7)]
        right_indices = [right_map[i] for i in range(1, 7)]

        data = {
            "left_names": [msg.name[i] for i in left_indices],
            "right_names": [msg.name[i] for i in right_indices],
            "left_joint": [float(msg.position[i]) for i in left_indices],
            "right_joint": [float(msg.position[i]) for i in right_indices],
        }

        path = Path.home() / "jaka_ws" / "real_robot_configs" / "sim_home_from_rviz.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2))

        self.get_logger().warn(f"Saved Sim Home to: {path}")
        self.get_logger().warn("LEFT  = " + str(data["left_joint"]))
        self.get_logger().warn("RIGHT = " + str(data["right_joint"]))


def main():
    rclpy.init()
    node = CaptureSimHome()
    try:
        node.capture()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
