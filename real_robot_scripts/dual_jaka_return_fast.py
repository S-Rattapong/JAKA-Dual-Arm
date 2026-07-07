#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from jaka_msgs.srv import Move
from jaka_msgs.msg import RobotMsg

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

class DualReturnFast(Node):
    def __init__(self, vel, acc):
        super().__init__("dual_jaka_return_fast")
        self.vel = vel
        self.acc = acc
        self.left_state = None
        self.right_state = None

        self.create_subscription(RobotMsg, "/left_jaka_driver/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, "/right_jaka_driver/robot_states", self.right_state_cb, 10)

        self.left_client = self.create_client(Move, "/left_jaka_driver/joint_move")
        self.right_client = self.create_client(Move, "/right_jaka_driver/joint_move")

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def wait_ready(self):
        self.get_logger().warn("Waiting for robot states...")
        end_time = self.get_clock().now().nanoseconds + int(5e9)

        while rclpy.ok() and self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.left_state is not None and self.right_state is not None:
                break

        if self.left_state is None or self.right_state is None:
            raise RuntimeError("Cannot read robot states.")

        for label, s in [("LEFT", self.left_state), ("RIGHT", self.right_state)]:
            if s.power_state != 1 or s.servo_state != 1:
                raise RuntimeError(f"{label} not enabled: power={s.power_state}, servo={s.servo_state}")
            if s.collision_state != 0:
                raise RuntimeError(f"{label} collision_state={s.collision_state}")
            if s.motion_state != 0:
                self.get_logger().warn(f"{label} motion_state={s.motion_state}; still sending return cautiously.")

    def call_joint_move(self, client, joints):
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

    def run(self):
        self.get_logger().warn("Waiting for joint_move services...")
        self.left_client.wait_for_service()
        self.right_client.wait_for_service()

        self.wait_ready()

        self.get_logger().warn(f"Returning both arms to baseline with vel={self.vel}, acc={self.acc}")
        lf = self.call_joint_move(self.left_client, LEFT_BASE)
        rf = self.call_joint_move(self.right_client, RIGHT_BASE)

        while rclpy.ok() and (not lf.done() or not rf.done()):
            rclpy.spin_once(self, timeout_sec=0.1)

        lres = lf.result()
        rres = rf.result()

        self.get_logger().warn(f"LEFT ret={lres.ret}, msg={lres.message}")
        self.get_logger().warn(f"RIGHT ret={rres.ret}, msg={rres.message}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vel", type=float, default=0.10)
    parser.add_argument("--acc", type=float, default=0.20)
    args = parser.parse_args()

    rclpy.init()
    node = DualReturnFast(args.vel, args.acc)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
