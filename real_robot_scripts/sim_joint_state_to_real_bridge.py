#!/usr/bin/env python3
import argparse
import math
import re
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import Move


LEFT_BASE = [
    2.8181028366088863,
    0.6005005240440368,
    -1.7843303680419924,
    1.3221564292907713,
    0.2156650573015213,
    1.0896085500717163,
]

RIGHT_BASE = [
    0.3300157189369201,
    2.253940582275391,
    1.5918420553207397,
    -1.3463665246963503,
    3.300674200057982,
    0.7286149859428406,
]


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def find_joint_number(name: str, side: str) -> Optional[int]:
    n = name.lower()
    if not n.startswith(side.lower()):
        return None

    patterns = [
        r"(?:joint[_-]?)([1-6])\b",
        r"(?:j)([1-6])\b",
    ]
    for p in patterns:
        m = re.search(p, n)
        if m:
            return int(m.group(1))
    return None


class SimJointStateToRealBridge(Node):
    def __init__(self, args):
        super().__init__("sim_joint_state_to_real_bridge")

        self.execute = args.execute
        self.max_abs_delta = args.max_abs_delta
        self.max_step = args.max_step
        self.period = args.period
        self.vel = args.vel
        self.acc = args.acc

        self.left_state = None
        self.right_state = None

        self.left_indices = None
        self.right_indices = None

        self.sim_base_left = None
        self.sim_base_right = None
        self.sim_cur_left = None
        self.sim_cur_right = None

        self.last_cmd_left = LEFT_BASE[:]
        self.last_cmd_right = RIGHT_BASE[:]

        self.busy = False

        self.create_subscription(JointState, args.source, self.sim_cb, 10)
        self.create_subscription(RobotMsg, "/left_jaka_driver/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, "/right_jaka_driver/robot_states", self.right_state_cb, 10)

        self.left_client = None
        self.right_client = None

        if self.execute:
            self.left_client = self.create_client(Move, "/left_jaka_driver/joint_move")
            self.right_client = self.create_client(Move, "/right_jaka_driver/joint_move")

            self.get_logger().warn("Waiting for /left_jaka_driver/joint_move ...")
            self.left_client.wait_for_service()
            self.get_logger().warn("Waiting for /right_jaka_driver/joint_move ...")
            self.right_client.wait_for_service()
        else:
            self.get_logger().warn("DRY RUN: skipping real robot service wait.")

        mode = "EXECUTE REAL ROBOT MOTION" if self.execute else "DRY RUN ONLY"
        self.get_logger().warn(f"Bridge started: {mode}")
        self.get_logger().warn(f"source={args.source}")
        self.get_logger().warn(f"max_abs_delta={self.max_abs_delta}, max_step={self.max_step}, period={self.period}")
        self.get_logger().warn("Move the simulation marker slowly. Commands are relative to the captured simulation baseline.")

        self.timer = self.create_timer(self.period, self.timer_cb)

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

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
                self.get_logger().warn("Cannot infer left/right 6 joint names from /joint_states yet.")
                self.get_logger().warn("Available names: " + str(list(msg.name)))
                return

            self.left_indices = [left_map[i] for i in range(1, 7)]
            self.right_indices = [right_map[i] for i in range(1, 7)]

            self.sim_base_left = [msg.position[i] for i in self.left_indices]
            self.sim_base_right = [msg.position[i] for i in self.right_indices]
            self.sim_cur_left = self.sim_base_left[:]
            self.sim_cur_right = self.sim_base_right[:]

            self.get_logger().warn("Captured simulation baseline.")
            self.get_logger().warn("Left sim joints : " + str(self.sim_base_left))
            self.get_logger().warn("Right sim joints: " + str(self.sim_base_right))
            self.get_logger().warn("Left names : " + str([msg.name[i] for i in self.left_indices]))
            self.get_logger().warn("Right names: " + str([msg.name[i] for i in self.right_indices]))
            return

        self.sim_cur_left = [msg.position[i] for i in self.left_indices]
        self.sim_cur_right = [msg.position[i] for i in self.right_indices]

    def real_ready(self) -> bool:
        if self.left_state is None or self.right_state is None:
            self.get_logger().warn("Waiting for real robot states...")
            return False

        for label, s in [("LEFT", self.left_state), ("RIGHT", self.right_state)]:
            if s.power_state != 1 or s.servo_state != 1:
                self.get_logger().error(f"{label} not enabled: power={s.power_state}, servo={s.servo_state}")
                return False
            if s.collision_state != 0:
                self.get_logger().error(f"{label} collision_state={s.collision_state}; refusing command")
                return False
            if s.motion_state != 0:
                self.get_logger().warn(f"{label} is moving: motion_state={s.motion_state}; waiting")
                return False

        return True

    def compute_target(self, base_real: List[float], base_sim: List[float], cur_sim: List[float], last_cmd: List[float]):
        raw_target = []
        step_target = []

        for br, bs, cs, lc in zip(base_real, base_sim, cur_sim, last_cmd):
            sim_delta = cs - bs
            safe_delta = clamp(sim_delta, -self.max_abs_delta, self.max_abs_delta)
            desired = br + safe_delta
            step = lc + clamp(desired - lc, -self.max_step, self.max_step)
            raw_target.append(desired)
            step_target.append(step)

        return raw_target, step_target

    def call_move(self, client, joints):
        req = Move.Request()
        req.pose = [float(x) for x in joints]
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = float(self.vel)
        req.mvacc = float(self.acc)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        return client.call_async(req)

    def timer_cb(self):
        if self.busy:
            return

        if self.sim_cur_left is None or self.sim_cur_right is None:
            self.get_logger().warn("Waiting for simulation /joint_states baseline...")
            return

        _, left_target = self.compute_target(
            LEFT_BASE, self.sim_base_left, self.sim_cur_left, self.last_cmd_left
        )
        _, right_target = self.compute_target(
            RIGHT_BASE, self.sim_base_right, self.sim_cur_right, self.last_cmd_right
        )

        max_change = max(
            max(abs(a - b) for a, b in zip(left_target, self.last_cmd_left)),
            max(abs(a - b) for a, b in zip(right_target, self.last_cmd_right)),
        )

        if max_change < 1e-5:
            return

        if self.execute and not self.real_ready():
            return

        self.get_logger().warn(f"Target LEFT : {left_target}")
        self.get_logger().warn(f"Target RIGHT: {right_target}")

        if not self.execute:
            self.get_logger().warn("DRY RUN: not sending command to real robot.")
            self.last_cmd_left = left_target
            self.last_cmd_right = right_target
            return

        self.busy = True
        lf = self.call_move(self.left_client, left_target)
        rf = self.call_move(self.right_client, right_target)

        def done_cb(_):
            if lf.done() and rf.done():
                try:
                    lres = lf.result()
                    rres = rf.result()
                    self.get_logger().warn(f"LEFT response ret={lres.ret}, msg={lres.message}")
                    self.get_logger().warn(f"RIGHT response ret={rres.ret}, msg={rres.message}")
                    self.last_cmd_left = left_target
                    self.last_cmd_right = right_target
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {e}")
                self.busy = False

        lf.add_done_callback(done_cb)
        rf.add_done_callback(done_cb)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="/joint_states")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-abs-delta", type=float, default=0.010)
    parser.add_argument("--max-step", type=float, default=0.002)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--vel", type=float, default=0.02)
    parser.add_argument("--acc", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = SimJointStateToRealBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
