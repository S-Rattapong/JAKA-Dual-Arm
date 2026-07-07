#!/usr/bin/env python3
import argparse
import math
import re
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import ServoMove, ServoMoveEnable


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm_inf(v):
    if not v:
        return 0.0
    return max(abs(x) for x in v)


def find_joint_number(name: str, side: str) -> Optional[int]:
    n = name.lower()
    if not n.startswith(side.lower()):
        return None
    for p in [r"(?:joint[_-]?)([1-6])\b", r"(?:j)([1-6])\b"]:
        m = re.search(p, n)
        if m:
            return int(m.group(1))
    return None


class VelocityLimitedServoJBridge(Node):
    def __init__(self, args):
        super().__init__("sim_to_real_velocity_limited_servoj_bridge")
        self.args = args

        self.sim_msg = None
        self.left_real = None
        self.right_real = None
        self.left_state = None
        self.right_state = None

        self.left_indices = None
        self.right_indices = None

        self.sim_left_base = None
        self.sim_right_base = None
        self.real_left_base = None
        self.real_right_base = None

        # Internal commanded joint estimate.
        # This reduces servo_j jitter caused by slow/noisy real joint feedback.
        self.cmd_left = None
        self.cmd_right = None

        self.last_sim_time = None
        self.last_print = 0.0

        self.create_subscription(JointState, args.source, self.sim_cb, 20)
        self.create_subscription(JointState, "/left_jaka_driver/joint_position", self.left_real_cb, 20)
        self.create_subscription(JointState, "/right_jaka_driver/joint_position", self.right_real_cb, 20)
        self.create_subscription(RobotMsg, "/left_jaka_driver/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, "/right_jaka_driver/robot_states", self.right_state_cb, 10)

        self.left_servo = self.create_client(ServoMove, "/left_jaka_driver/servo_j")
        self.right_servo = self.create_client(ServoMove, "/right_jaka_driver/servo_j")
        self.left_enable = self.create_client(ServoMoveEnable, "/left_jaka_driver/servo_move_enable")
        self.right_enable = self.create_client(ServoMoveEnable, "/right_jaka_driver/servo_move_enable")

        self.timer = self.create_timer(args.period, self.tick)

        self.get_logger().warn("Velocity-limited ServoJ bridge started.")
        self.get_logger().warn("NO max_abs_delta clamp. Motion is limited by max_step and period only.")
        self.get_logger().warn(f"source={args.source}")
        self.get_logger().warn(f"execute={args.execute}")
        self.get_logger().warn(f"max_step={args.max_step} rad/cycle, period={args.period} s")
        self.get_logger().warn(f"approx joint speed limit = {args.max_step / args.period:.4f} rad/s")
        self.get_logger().warn(f"deadband={args.deadband}, watchdog_timeout={args.watchdog_timeout}")
        self.get_logger().warn(f"panic_if_error_over={args.panic_if_error_over} rad")

    def sim_cb(self, msg):
        self.sim_msg = msg
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

            if len(left_map) == 6 and len(right_map) == 6:
                self.left_indices = [left_map[i] for i in range(1, 7)]
                self.right_indices = [right_map[i] for i in range(1, 7)]
                self.get_logger().warn("Detected sim joint names.")
                self.get_logger().warn("LEFT  sim names = " + str([msg.name[i] for i in self.left_indices]))
                self.get_logger().warn("RIGHT sim names = " + str([msg.name[i] for i in self.right_indices]))

    def left_real_cb(self, msg):
        if len(msg.position) >= 6:
            self.left_real = list(msg.position)[:6]

    def right_real_cb(self, msg):
        if len(msg.position) >= 6:
            self.right_real = list(msg.position)[:6]

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def get_sim_joints(self):
        if self.sim_msg is None or self.left_indices is None or self.right_indices is None:
            return None, None
        left = [float(self.sim_msg.position[i]) for i in self.left_indices]
        right = [float(self.sim_msg.position[i]) for i in self.right_indices]
        return left, right

    def ready(self):
        return (
            self.sim_msg is not None
            and self.left_indices is not None
            and self.right_indices is not None
            and self.left_real is not None
            and self.right_real is not None
            and self.left_state is not None
            and self.right_state is not None
        )

    def enable_servo_mode(self):
        if not self.args.execute:
            return True

        self.left_enable.wait_for_service(timeout_sec=2.0)
        self.right_enable.wait_for_service(timeout_sec=2.0)

        req = ServoMoveEnable.Request()
        req.enable = True

        lf = self.left_enable.call_async(req)
        rf = self.right_enable.call_async(req)

        rclpy.spin_until_future_complete(self, lf, timeout_sec=2.0)
        rclpy.spin_until_future_complete(self, rf, timeout_sec=2.0)

        return True

    def disable_servo_mode(self):
        if not self.args.execute:
            return

        req = ServoMoveEnable.Request()
        req.enable = False

        try:
            self.left_enable.call_async(req)
            self.right_enable.call_async(req)
        except Exception:
            pass

    def check_robot_state(self):
        for label, s in [("LEFT", self.left_state), ("RIGHT", self.right_state)]:
            if s.power_state != 1 or s.servo_state != 1:
                self.get_logger().error(f"{label} not enabled: power={s.power_state}, servo={s.servo_state}")
                return False
            if s.collision_state != 0:
                self.get_logger().error(f"{label} collision_state={s.collision_state}")
                return False
        return True

    def send_servoj(self, client, inc):
        req = ServoMove.Request()
        req.pose = [float(x) for x in inc]
        req.speed = []
        return client.call_async(req)

    def tick(self):
        if not self.ready():
            return

        now = time.time()

        # Some simulation nodes publish /joint_states only when the interactive target changes.
        # Allow watchdog_timeout <= 0 to disable the source-topic silence watchdog.
        if self.args.watchdog_timeout > 0.0:
            if self.last_sim_time is None or now - self.last_sim_time > self.args.watchdog_timeout:
                self.get_logger().error("Watchdog timeout: /joint_states stopped. Disabling servo mode.")
                self.disable_servo_mode()
                rclpy.shutdown()
                return

        if not self.check_robot_state():
            self.get_logger().error("Robot state not safe. Disabling servo mode.")
            self.disable_servo_mode()
            rclpy.shutdown()
            return

        sim_left, sim_right = self.get_sim_joints()

        if self.sim_left_base is None:
            self.sim_left_base = sim_left[:]
            self.sim_right_base = sim_right[:]
            self.real_left_base = self.left_real[:]
            self.real_right_base = self.right_real[:]

            # Start command estimate from the current real joint feedback.
            self.cmd_left = self.real_left_base[:]
            self.cmd_right = self.real_right_base[:]

            self.get_logger().warn("Captured baseline.")
            self.get_logger().warn("SIM LEFT BASE  = " + str(self.sim_left_base))
            self.get_logger().warn("SIM RIGHT BASE = " + str(self.sim_right_base))
            self.get_logger().warn("REAL LEFT BASE = " + str(self.real_left_base))
            self.get_logger().warn("REAL RIGHT BASE= " + str(self.real_right_base))

            self.enable_servo_mode()
            return

        # Full delta follow: no max_abs_delta clamp.
        desired_left = [
            rb + (sc - sb)
            for rb, sc, sb in zip(self.real_left_base, sim_left, self.sim_left_base)
        ]
        desired_right = [
            rb + (sc - sb)
            for rb, sc, sb in zip(self.real_right_base, sim_right, self.sim_right_base)
        ]

        # Use internal command estimate for control, not raw real feedback.
        # Raw feedback may update slower than servo_j commands and can cause repeated increments.
        control_left = self.cmd_left if self.cmd_left is not None else self.left_real
        control_right = self.cmd_right if self.cmd_right is not None else self.right_real

        err_left = [d - c for d, c in zip(desired_left, control_left)]
        err_right = [d - c for d, c in zip(desired_right, control_right)]

        max_err = max(norm_inf(err_left), norm_inf(err_right))

        if max_err > self.args.panic_if_error_over:
            self.get_logger().error(
                f"PANIC: target error too large ({max_err:.3f} rad). "
                "This may indicate wrong baseline/joint order/IK jump. Disabling servo mode."
            )
            self.disable_servo_mode()
            rclpy.shutdown()
            return

        inc_left = []
        inc_right = []

        for e in err_left:
            if abs(e) < self.args.deadband:
                inc_left.append(0.0)
            else:
                inc_left.append(clamp(e, -self.args.max_step, self.args.max_step))

        for e in err_right:
            if abs(e) < self.args.deadband:
                inc_right.append(0.0)
            else:
                inc_right.append(clamp(e, -self.args.max_step, self.args.max_step))

        if norm_inf(inc_left) == 0.0 and norm_inf(inc_right) == 0.0:
            return

        if now - self.last_print > 0.5:
            self.last_print = now
            self.get_logger().warn(f"max_err={max_err:.5f} rad")
            self.get_logger().warn("INC LEFT  = " + str(inc_left))
            self.get_logger().warn("INC RIGHT = " + str(inc_right))

        # Update internal commanded state immediately.
        # servo_j is incremental, so this represents what we asked the robot to do.
        self.cmd_left = [c + u for c, u in zip(self.cmd_left, inc_left)]
        self.cmd_right = [c + u for c, u in zip(self.cmd_right, inc_right)]

        if not self.args.execute:
            return

        lf = self.send_servoj(self.left_servo, inc_left)
        rf = self.send_servoj(self.right_servo, inc_right)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="/joint_states")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-step", type=float, default=0.0010)
    parser.add_argument("--period", type=float, default=0.04)
    parser.add_argument("--deadband", type=float, default=0.00020)
    parser.add_argument("--watchdog-timeout", type=float, default=0.5)
    parser.add_argument("--panic-if-error-over", type=float, default=1.2)
    args = parser.parse_args()

    rclpy.init()
    node = VelocityLimitedServoJBridge(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disable_servo_mode()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
