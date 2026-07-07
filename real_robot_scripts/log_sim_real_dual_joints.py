#!/usr/bin/env python3
import csv
import time
import re
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg


def find_joint_number(name: str, side: str) -> Optional[int]:
    n = name.lower()
    if not n.startswith(side.lower()):
        return None
    for p in [r"(?:joint[_-]?)([1-6])\b", r"(?:j)([1-6])\b"]:
        m = re.search(p, n)
        if m:
            return int(m.group(1))
    return None


class SimRealDualJointLogger(Node):
    def __init__(self):
        super().__init__("sim_real_dual_joint_logger")

        self.start = time.time()

        self.sim_left = None
        self.sim_right = None
        self.sim_left_base = None
        self.sim_right_base = None
        self.left_indices = None
        self.right_indices = None

        self.real_left = None
        self.real_right = None
        self.real_left_base = None
        self.real_right_base = None

        self.left_state = None
        self.right_state = None

        path = Path.home() / "jaka_ws" / "real_robot_logs" / "sim_real_dual_motion_log.csv"
        path.parent.mkdir(parents=True, exist_ok=True)

        self.f = open(path, "w", newline="")
        self.writer = csv.writer(self.f)

        header = ["time_s"]
        for side in ["sim_left", "sim_right", "real_left", "real_right", "sim_delta_left", "sim_delta_right", "real_delta_left", "real_delta_right", "error_left", "error_right"]:
            for i in range(1, 7):
                header.append(f"{side}_j{i}")
        header += [
            "left_motion_state", "left_power_state", "left_servo_state", "left_collision_state",
            "right_motion_state", "right_power_state", "right_servo_state", "right_collision_state",
        ]
        self.writer.writerow(header)

        self.create_subscription(JointState, "/joint_states", self.sim_cb, 10)
        self.create_subscription(JointState, "/left_jaka_driver/joint_position", self.real_left_cb, 10)
        self.create_subscription(JointState, "/right_jaka_driver/joint_position", self.real_right_cb, 10)
        self.create_subscription(RobotMsg, "/left_jaka_driver/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, "/right_jaka_driver/robot_states", self.right_state_cb, 10)

        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().warn(f"Logging Sim vs Real to {path}")

    def sim_cb(self, msg: JointState):
        if self.left_indices is None or self.right_indices is None:
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
                return

            self.left_indices = [left_map[i] for i in range(1, 7)]
            self.right_indices = [right_map[i] for i in range(1, 7)]

        self.sim_left = [float(msg.position[i]) for i in self.left_indices]
        self.sim_right = [float(msg.position[i]) for i in self.right_indices]

        if self.sim_left_base is None:
            self.sim_left_base = self.sim_left[:]
            self.sim_right_base = self.sim_right[:]
            self.get_logger().warn("Captured SIM baseline.")

    def real_left_cb(self, msg):
        self.real_left = list(msg.position)
        if self.real_left_base is None and len(self.real_left) >= 6:
            self.real_left_base = self.real_left[:6]
            self.get_logger().warn("Captured REAL LEFT baseline.")

    def real_right_cb(self, msg):
        self.real_right = list(msg.position)
        if self.real_right_base is None and len(self.real_right) >= 6:
            self.real_right_base = self.real_right[:6]
            self.get_logger().warn("Captured REAL RIGHT baseline.")

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def delta(self, cur, base):
        return [c - b for c, b in zip(cur[:6], base[:6])]

    def tick(self):
        if None in [self.sim_left, self.sim_right, self.real_left, self.real_right,
                    self.sim_left_base, self.sim_right_base, self.real_left_base, self.real_right_base]:
            return

        sim_delta_left = self.delta(self.sim_left, self.sim_left_base)
        sim_delta_right = self.delta(self.sim_right, self.sim_right_base)
        real_delta_left = self.delta(self.real_left, self.real_left_base)
        real_delta_right = self.delta(self.real_right, self.real_right_base)

        error_left = [r - s for r, s in zip(real_delta_left, sim_delta_left)]
        error_right = [r - s for r, s in zip(real_delta_right, sim_delta_right)]

        ls = self.left_state
        rs = self.right_state

        row = [f"{time.time() - self.start:.3f}"]
        row += self.sim_left[:6] + self.sim_right[:6]
        row += self.real_left[:6] + self.real_right[:6]
        row += sim_delta_left + sim_delta_right
        row += real_delta_left + real_delta_right
        row += error_left + error_right

        if ls is not None:
            row += [ls.motion_state, ls.power_state, ls.servo_state, ls.collision_state]
        else:
            row += ["", "", "", ""]

        if rs is not None:
            row += [rs.motion_state, rs.power_state, rs.servo_state, rs.collision_state]
        else:
            row += ["", "", "", ""]

        self.writer.writerow(row)
        self.f.flush()


def main():
    rclpy.init()
    node = SimRealDualJointLogger()
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
