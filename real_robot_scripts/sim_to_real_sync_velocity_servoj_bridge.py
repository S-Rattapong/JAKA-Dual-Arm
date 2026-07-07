#!/usr/bin/env python3
import argparse
import re
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import ServoMove, ServoMoveEnable


def norm_inf(v):
    return max((abs(x) for x in v), default=0.0)


def find_joint_number(name: str, side: str) -> Optional[int]:
    n = name.lower()
    if not n.startswith(side.lower()):
        return None
    for p in [r"(?:joint[_-]?)([1-6])\b", r"(?:j)([1-6])\b"]:
        m = re.search(p, n)
        if m:
            return int(m.group(1))
    return None


class SyncVelocityServoJBridge(Node):
    def __init__(self, args):
        super().__init__("sim_to_real_sync_velocity_servoj_bridge")
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

        self.cmd_left = None
        self.cmd_right = None

        self.filtered_left_target = None
        self.filtered_right_target = None

        self.last_sim_time = None
        self.last_print = 0.0
        self.servo_enabled = False

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

        self.get_logger().warn("SYNC Velocity-limited ServoJ bridge started.")
        self.get_logger().warn("This bridge scales the full 12-joint error vector with ONE common scale.")
        self.get_logger().warn("This should reduce joint groups moving out of sync.")
        self.get_logger().warn(f"execute={args.execute}")
        self.get_logger().warn(f"max_step={args.max_step} rad/cycle, period={args.period} s")
        self.get_logger().warn(f"approx max joint speed = {args.max_step / args.period:.4f} rad/s")
        self.get_logger().warn(f"deadband={args.deadband}, target_alpha={args.target_alpha}")
        self.get_logger().warn(f"panic_if_error_over={args.panic_if_error_over} rad")
        self.get_logger().warn(f"watchdog_timeout={args.watchdog_timeout}; <=0 disables source silence watchdog")

    def sim_cb(self, msg):
        self.sim_msg = msg
        self.last_sim_time = time.time()

        if self.left_indices is not None and self.right_indices is not None:
            return

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

    def check_robot_state(self):
        for label, s in [("LEFT", self.left_state), ("RIGHT", self.right_state)]:
            if s.power_state != 1 or s.servo_state != 1:
                self.get_logger().error(f"{label} not enabled: power={s.power_state}, servo={s.servo_state}")
                return False
            if s.collision_state != 0:
                self.get_logger().error(f"{label} collision_state={s.collision_state}")
                return False
        return True

    def set_servo_enable(self, enable: bool):
        if not self.args.execute:
            return

        self.left_enable.wait_for_service(timeout_sec=2.0)
        self.right_enable.wait_for_service(timeout_sec=2.0)

        req = ServoMoveEnable.Request()
        req.enable = bool(enable)

        lf = self.left_enable.call_async(req)
        rf = self.right_enable.call_async(req)

        rclpy.spin_until_future_complete(self, lf, timeout_sec=2.0)
        rclpy.spin_until_future_complete(self, rf, timeout_sec=2.0)

        self.servo_enabled = enable

    def disable_servo_mode(self):
        try:
            self.set_servo_enable(False)
        except Exception:
            pass

    def send_servoj(self, client, inc):
        req = ServoMove.Request()
        req.pose = [float(x) for x in inc]
        req.speed = []
        return client.call_async(req)

    def tick(self):
        if not self.ready():
            return

        now = time.time()

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

            self.cmd_left = self.real_left_base[:]
            self.cmd_right = self.real_right_base[:]

            self.filtered_left_target = self.real_left_base[:]
            self.filtered_right_target = self.real_right_base[:]

            self.get_logger().warn("Captured baseline.")
            self.get_logger().warn("SIM LEFT BASE   = " + str(self.sim_left_base))
            self.get_logger().warn("SIM RIGHT BASE  = " + str(self.sim_right_base))
            self.get_logger().warn("REAL LEFT BASE  = " + str(self.real_left_base))
            self.get_logger().warn("REAL RIGHT BASE = " + str(self.real_right_base))

            self.set_servo_enable(True)
            return

        raw_left_target = [
            rb + (sc - sb)
            for rb, sc, sb in zip(self.real_left_base, sim_left, self.sim_left_base)
        ]
        raw_right_target = [
            rb + (sc - sb)
            for rb, sc, sb in zip(self.real_right_base, sim_right, self.sim_right_base)
        ]

        a = self.args.target_alpha
        self.filtered_left_target = [
            old + a * (new - old)
            for old, new in zip(self.filtered_left_target, raw_left_target)
        ]
        self.filtered_right_target = [
            old + a * (new - old)
            for old, new in zip(self.filtered_right_target, raw_right_target)
        ]

        err_left = [d - c for d, c in zip(self.filtered_left_target, self.cmd_left)]
        err_right = [d - c for d, c in zip(self.filtered_right_target, self.cmd_right)]

        err12 = err_left + err_right
        max_err = norm_inf(err12)

        if max_err > self.args.panic_if_error_over:
            self.get_logger().error(
                f"PANIC: target error too large ({max_err:.3f} rad). "
                "Possible IK jump / wrong baseline / unsafe target. Disabling servo mode."
            )
            self.disable_servo_mode()
            rclpy.shutdown()
            return

        # Apply deadband first.
        err12_db = [0.0 if abs(e) < self.args.deadband else e for e in err12]
        max_err_db = norm_inf(err12_db)

        if max_err_db == 0.0:
            return

        # Critical sync logic:
        # ONE common scale for all 12 joints, so they move proportionally together.
        scale = min(1.0, self.args.max_step / max_err_db)
        inc12 = [e * scale for e in err12_db]

        inc_left = inc12[:6]
        inc_right = inc12[6:]

        self.cmd_left = [c + u for c, u in zip(self.cmd_left, inc_left)]
        self.cmd_right = [c + u for c, u in zip(self.cmd_right, inc_right)]

        if now - self.last_print > 0.5:
            self.last_print = now
            self.get_logger().warn(
                f"max_err={max_err:.5f}, max_err_db={max_err_db:.5f}, vector_scale={scale:.5f}"
            )
            self.get_logger().warn("INC LEFT  = " + str(inc_left))
            self.get_logger().warn("INC RIGHT = " + str(inc_right))

        if not self.args.execute:
            return

        self.send_servoj(self.left_servo, inc_left)
        self.send_servoj(self.right_servo, inc_right)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="/joint_states")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--max-step", type=float, default=0.0004)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--deadband", type=float, default=0.00050)
    parser.add_argument("--target-alpha", type=float, default=0.20)
    parser.add_argument("--watchdog-timeout", type=float, default=0.0)
    parser.add_argument("--panic-if-error-over", type=float, default=1.2)
    args = parser.parse_args()

    rclpy.init()
    node = SyncVelocityServoJBridge(args)

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
