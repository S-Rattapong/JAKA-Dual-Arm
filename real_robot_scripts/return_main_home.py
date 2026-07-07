#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from jaka_msgs.srv import Move


class ReturnMainHome(Node):
    def __init__(self, args):
        super().__init__("return_main_home")
        self.args = args

        path = Path(args.config).expanduser()
        data = json.loads(path.read_text())

        self.left_home = data["left_joint"]
        self.right_home = data["right_joint"]

        self.left_client = self.create_client(Move, "/left_jaka_driver/joint_move")
        self.right_client = self.create_client(Move, "/right_jaka_driver/joint_move")

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

    def run(self):
        self.get_logger().warn("Returning both real robots to MAIN REAL HOME.")
        self.get_logger().warn("LEFT HOME  = " + str(self.left_home))
        self.get_logger().warn("RIGHT HOME = " + str(self.right_home))
        self.get_logger().warn(f"vel={self.args.vel}, acc={self.args.acc}")

        self.left_client.wait_for_service()
        self.right_client.wait_for_service()

        lf = self.call_move(self.left_client, self.left_home)
        rf = self.call_move(self.right_client, self.right_home)

        while rclpy.ok() and (not lf.done() or not rf.done()):
            rclpy.spin_once(self, timeout_sec=0.1)

        lres = lf.result()
        rres = rf.result()

        self.get_logger().warn(f"LEFT ret={lres.ret}, msg={lres.message}")
        self.get_logger().warn(f"RIGHT ret={rres.ret}, msg={rres.message}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="~/jaka_ws/real_robot_configs/main_real_home.json")
    parser.add_argument("--vel", type=float, default=0.6)
    parser.add_argument("--acc", type=float, default=0.30)
    args = parser.parse_args()

    rclpy.init()
    node = ReturnMainHome(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
