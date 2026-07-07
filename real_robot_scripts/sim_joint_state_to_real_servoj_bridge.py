#!/usr/bin/env python3
import argparse
import math
import re
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import ServoMove, ServoMoveEnable
from std_srvs.srv import Empty


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def find_joint_number(name: str, side: str) -> Optional[int]:
    n = name.lower()
    if not n.startswith(side.lower()):
        return None
    for p in [r"(?:joint[_-]?)([1-6])\b", r"(?:j)([1-6])\b"]:
        m = re.search(p, n)
        if m:
            return int(m.group(1))
    return None


class ServoJBridge(Node):
    def __init__(self, args):
        super().__init__("sim_joint_state_to_real_servoj_bridge")

        self.args = args
        self.execute = args.execute
        self.max_abs_delta = args.max_abs_delta
        self.max_step = args.max_step
        self.deadband = args.deadband
        self.timeout = args.watchdog_timeout

        self.left_state = None
        self.right_state = None

        self.left_indices = None
        self.right_indices = None
        self.sim_base_left = None
        self.sim_base_right = None
        self.sim_cur_left = None
        self.sim_cur_right = None
        self.last_sim_time = 0.0

        # These are commanded offsets from the captured baseline, not absolute joint positions.
        self.cmd_offset_left = [0.0] * 6
        self.cmd_offset_right = [0.0] * 6

        self.busy = False
        self.servo_enabled = False

        self.create_subscription(JointState, args.source, self.sim_cb, 10)
        self.create_subscription(RobotMsg, "/left_jaka_driver/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, "/right_jaka_driver/robot_states", self.right_state_cb, 10)

        self.left_servo = self.create_client(ServoMove, "/left_jaka_driver/servo_j")
        self.right_servo = self.create_client(ServoMove, "/right_jaka_driver/servo_j")
        self.left_enable = self.create_client(ServoMoveEnable, "/left_jaka_driver/servo_move_enable")
        self.right_enable = self.create_client(ServoMoveEnable, "/right_jaka_driver/servo_move_enable")
        self.left_stop = self.create_client(Empty, "/left_jaka_driver/stop_move")
        self.right_stop = self.create_client(Empty, "/right_jaka_driver/stop_move")

        if self.execute:
            for name, cli in [
                ("left servo_j", self.left_servo),
                ("right servo_j", self.right_servo),
                ("left servo_move_enable", self.left_enable),
                ("right servo_move_enable", self.right_enable),
                ("left stop_move", self.left_stop),
                ("right stop_move", self.right_stop),
            ]:
                self.get_logger().warn(f"Waiting for {name} service...")
                cli.wait_for_service()

            self.enable_servo(True)
            self.servo_enabled = True
        else:
            self.get_logger().warn("DRY RUN: not waiting for real robot services.")

        mode = "EXECUTE SERVO_J REAL ROBOT MOTION" if self.execute else "DRY RUN ONLY"
        self.get_logger().warn(f"ServoJ bridge started: {mode}")
        self.get_logger().warn(f"source={args.source}")
        self.get_logger().warn(
            f"max_abs_delta={self.max_abs_delta}, max_step={self.max_step}, "
            f"period={args.period}, watchdog_timeout={self.timeout}"
        )
        self.get_logger().warn("IMPORTANT: servo_j is INCREMENTAL. This bridge sends tiny joint increments only.")

        self.timer = self.create_timer(args.period, self.timer_cb)

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def sim_cb(self, msg: JointState):
        self.last_sim_time = time.time()

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

            self.sim_base_left = [float(msg.position[i]) for i in self.left_indices]
            self.sim_base_right = [float(msg.position[i]) for i in self.right_indices]
            self.sim_cur_left = self.sim_base_left[:]
            self.sim_cur_right = self.sim_base_right[:]

            self.get_logger().warn("Captured simulation baseline.")
            self.get_logger().warn("Left sim joints : " + str(self.sim_base_left))
            self.get_logger().warn("Right sim joints: " + str(self.sim_base_right))
            self.get_logger().warn("Left names : " + str([msg.name[i] for i in self.left_indices]))
            self.get_logger().warn("Right names: " + str([msg.name[i] for i in self.right_indices]))
            return

        self.sim_cur_left = [float(msg.position[i]) for i in self.left_indices]
        self.sim_cur_right = [float(msg.position[i]) for i in self.right_indices]

    def real_ready(self):
        if not self.execute:
            return True

        if self.left_state is None or self.right_state is None:
            self.get_logger().warn("Waiting for real robot states...")
            return False

        for label, s in [("LEFT", self.left_state), ("RIGHT", self.right_state)]:
            if s.power_state != 1 or s.servo_state != 1:
                self.get_logger().error(f"{label} not enabled: power={s.power_state}, servo={s.servo_state}")
                return False
            if s.collision_state != 0:
                self.get_logger().error(f"{label} collision_state={s.collision_state}; stopping.")
                self.stop_and_disable()
                return False

        return True

    def enable_servo(self, enable: bool):
        for label, cli in [("LEFT", self.left_enable), ("RIGHT", self.right_enable)]:
            req = ServoMoveEnable.Request()
            req.enable = bool(enable)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.done():
                res = fut.result()
                self.get_logger().warn(f"{label} servo_move_enable({enable}) ret={res.ret}, msg={res.message}")
            else:
                self.get_logger().error(f"{label} servo_move_enable({enable}) timeout")

    def stop_and_disable(self):
        if not self.execute:
            return

        for cli in [self.left_stop, self.right_stop]:
            try:
                cli.call_async(Empty.Request())
            except Exception:
                pass

        try:
            self.enable_servo(False)
        except Exception:
            pass

        self.servo_enabled = False

    def compute_increment(self, sim_base, sim_cur, cmd_offset):
        desired_offset = []
        increment = []

        for bs, cs, co in zip(sim_base, sim_cur, cmd_offset):
            raw_delta = cs - bs
            safe_delta = clamp(raw_delta, -self.max_abs_delta, self.max_abs_delta)
            next_offset = co + clamp(safe_delta - co, -self.max_step, self.max_step)

            inc = next_offset - co
            if abs(inc) < self.deadband:
                inc = 0.0

            desired_offset.append(next_offset)
            increment.append(inc)

        return desired_offset, increment

    def call_servoj(self, cli, inc: List[float]):
        req = ServoMove.Request()
        req.pose = [float(x) for x in inc]
        req.speed = [0.0] * 6
        return cli.call_async(req)

    def timer_cb(self):
        if self.busy:
            return

        if self.sim_cur_left is None or self.sim_cur_right is None:
            self.get_logger().warn("Waiting for simulation /joint_states baseline...")
            return

        if time.time() - self.last_sim_time > self.timeout:
            self.get_logger().error("Watchdog timeout: /joint_states stopped. Disabling servo mode.")
            self.stop_and_disable()
            return

        if not self.real_ready():
            return

        next_left, inc_left = self.compute_increment(
            self.sim_base_left, self.sim_cur_left, self.cmd_offset_left
        )
        next_right, inc_right = self.compute_increment(
            self.sim_base_right, self.sim_cur_right, self.cmd_offset_right
        )

        max_inc = max(max(abs(x) for x in inc_left), max(abs(x) for x in inc_right))
        if max_inc <= 0.0:
            return

        self.get_logger().warn(f"INC LEFT : {inc_left}")
        self.get_logger().warn(f"INC RIGHT: {inc_right}")

        if not self.execute:
            self.cmd_offset_left = next_left
            self.cmd_offset_right = next_right
            self.get_logger().warn("DRY RUN: not sending servo_j.")
            return

        self.busy = True
        lf = self.call_servoj(self.left_servo, inc_left)
        rf = self.call_servoj(self.right_servo, inc_right)

        def done_cb(_):
            if lf.done() and rf.done():
                try:
                    lres = lf.result()
                    rres = rf.result()
                    self.get_logger().warn(f"LEFT servo_j ret={lres.ret}, msg={lres.message}")
                    self.get_logger().warn(f"RIGHT servo_j ret={rres.ret}, msg={rres.message}")

                    if lres.ret == 1 and rres.ret == 1:
                        self.cmd_offset_left = next_left
                        self.cmd_offset_right = next_right
                    else:
                        self.get_logger().error("servo_j returned error. Stopping and disabling.")
                        self.stop_and_disable()
                except Exception as e:
                    self.get_logger().error(f"servo_j service failed: {e}")
                    self.stop_and_disable()
                self.busy = False

        lf.add_done_callback(done_cb)
        rf.add_done_callback(done_cb)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="/joint_states")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-abs-delta", type=float, default=0.020)
    parser.add_argument("--max-step", type=float, default=0.0005)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--deadband", type=float, default=0.00005)
    parser.add_argument("--watchdog-timeout", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = ServoJBridge(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt: stopping and disabling servo mode.")
    finally:
        node.stop_and_disable()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
