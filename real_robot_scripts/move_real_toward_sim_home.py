#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import Move


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class MoveTowardSimHome(Node):
    def __init__(self, args):
        super().__init__("move_real_toward_sim_home")
        self.args = args

        path = Path(args.config).expanduser()
        data = json.loads(path.read_text())
        self.left_target = data["left_joint"]
        self.right_target = data["right_joint"]

        self.left_joint = None
        self.right_joint = None
        self.left_state = None
        self.right_state = None

        self.create_subscription(JointState, "/left_jaka_driver/joint_position", self.left_joint_cb, 10)
        self.create_subscription(JointState, "/right_jaka_driver/joint_position", self.right_joint_cb, 10)
        self.create_subscription(RobotMsg, "/left_jaka_driver/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, "/right_jaka_driver/robot_states", self.right_state_cb, 10)

        self.left_client = self.create_client(Move, "/left_jaka_driver/joint_move")
        self.right_client = self.create_client(Move, "/right_jaka_driver/joint_move")

    def left_joint_cb(self, msg):
        self.left_joint = list(msg.position)[:6]

    def right_joint_cb(self, msg):
        self.right_joint = list(msg.position)[:6]

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def wait_data(self):
        self.get_logger().warn("Waiting for real robot joint/state topics...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(x is not None for x in [self.left_joint, self.right_joint, self.left_state, self.right_state]):
                return

    def check_ready(self):
        for label, s in [("LEFT", self.left_state), ("RIGHT", self.right_state)]:
            if s.power_state != 1 or s.servo_state != 1:
                raise RuntimeError(f"{label} not enabled: power={s.power_state}, servo={s.servo_state}")
            if s.collision_state != 0:
                raise RuntimeError(f"{label} collision_state={s.collision_state}")
            if s.motion_state != 0:
                self.get_logger().warn(f"{label} motion_state={s.motion_state}; robot may still be moving.")

    def step_target(self, current, target):
        step = []
        delta = []
        for c, t in zip(current, target):
            d = t - c
            delta.append(d)
            step.append(c + clamp(d, -self.args.max_step_per_run, self.args.max_step_per_run))
        return step, delta

    def call_move(self, client, joints):
        req = Move.Request()
        req.pose = [float(x) for x in joints]
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = float(self.args.vel)
        req.mvacc = float(self.args.acc)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        return client.call_async(req)

    def run_once(self):
        self.left_client.wait_for_service()
        self.right_client.wait_for_service()
        self.wait_data()
        self.check_ready()

        left_step, left_delta = self.step_target(self.left_joint, self.left_target)
        right_step, right_delta = self.step_target(self.right_joint, self.right_target)

        max_left = max(abs(x) for x in left_delta)
        max_right = max(abs(x) for x in right_delta)
        max_err = max(max_left, max_right)

        self.get_logger().warn("Current LEFT  = " + str(self.left_joint))
        self.get_logger().warn("Target  LEFT  = " + str(self.left_target))
        self.get_logger().warn("Delta   LEFT  = " + str(left_delta))
        self.get_logger().warn("Step    LEFT  = " + str(left_step))

        self.get_logger().warn("Current RIGHT = " + str(self.right_joint))
        self.get_logger().warn("Target  RIGHT = " + str(self.right_target))
        self.get_logger().warn("Delta   RIGHT = " + str(right_delta))
        self.get_logger().warn("Step    RIGHT = " + str(right_step))
        self.get_logger().warn(f"Max remaining joint error = {max_err:.6f} rad")

        if max_err < self.args.tolerance:
            self.get_logger().warn("Already close to Sim Home.")
            return

        if not self.args.execute:
            self.get_logger().warn("DRY RUN ONLY: not moving real robot.")
            return

        self.get_logger().warn(
            f"EXECUTE: moving one safe step toward Sim Home, "
            f"max_step_per_run={self.args.max_step_per_run}, vel={self.args.vel}, acc={self.args.acc}"
        )

        lf = self.call_move(self.left_client, left_step)
        rf = self.call_move(self.right_client, right_step)

        while rclpy.ok() and (not lf.done() or not rf.done()):
            rclpy.spin_once(self, timeout_sec=0.1)

        lres = lf.result()
        rres = rf.result()
        self.get_logger().warn(f"LEFT ret={lres.ret}, msg={lres.message}")
        self.get_logger().warn(f"RIGHT ret={rres.ret}, msg={rres.message}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="~/jaka_ws/real_robot_configs/sim_home_from_rviz.json")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-step-per-run", type=float, default=0.10)
    parser.add_argument("--vel", type=float, default=0.06)
    parser.add_argument("--acc", type=float, default=0.12)
    parser.add_argument("--tolerance", type=float, default=0.015)
    args = parser.parse_args()

    rclpy.init()
    node = MoveTowardSimHome(args)
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
