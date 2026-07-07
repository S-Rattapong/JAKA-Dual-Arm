#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import Move


COORD_MODE = {
    "joint": 0,
    "base": 1,
    "tool": 2,
}

AXIS_INDEX = {
    "j1": 0, "joint1": 0, "1": 0,
    "j2": 1, "joint2": 1, "2": 1,
    "j3": 2, "joint3": 2, "3": 2,
    "j4": 3, "joint4": 3, "4": 3,
    "j5": 4, "joint5": 4, "5": 4,
    "j6": 5, "joint6": 5, "6": 5,

    "x": 0,
    "y": 1,
    "z": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
}


class DualJakaJogCli(Node):
    def __init__(self, cfg):
        super().__init__("dual_jaka_jog_cli")
        self.cfg = cfg

        self.left_state = None
        self.right_state = None

        self.create_subscription(
            RobotMsg,
            f"{cfg['left']['prefix']}/robot_states",
            self.left_state_cb,
            10,
        )
        self.create_subscription(
            RobotMsg,
            f"{cfg['right']['prefix']}/robot_states",
            self.right_state_cb,
            10,
        )

        self.left_jog = self.create_client(Move, f"{cfg['left']['prefix']}/jog")
        self.right_jog = self.create_client(Move, f"{cfg['right']['prefix']}/jog")
        self.left_stop = self.create_client(Empty, f"{cfg['left']['prefix']}/stop_move")
        self.right_stop = self.create_client(Empty, f"{cfg['right']['prefix']}/stop_move")

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def wait_state(self, timeout=5.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.left_state is not None and self.right_state is not None:
                return True
        return False

    def check_safe(self, side):
        self.wait_state()

        selected = []
        if side in ("left", "both"):
            selected.append(("LEFT", self.left_state))
        if side in ("right", "both"):
            selected.append(("RIGHT", self.right_state))

        for label, state in selected:
            if state is None:
                raise RuntimeError(f"{label}: no robot state")
            if state.power_state != 1:
                raise RuntimeError(f"{label}: power_state={state.power_state}, expected 1")
            if state.servo_state != 1:
                raise RuntimeError(f"{label}: servo_state={state.servo_state}, expected 1")
            if state.collision_state != 0:
                raise RuntimeError(f"{label}: collision_state={state.collision_state}, expected 0")

    def wait_services(self, side):
        pairs = []
        if side in ("left", "both"):
            pairs += [("left jog", self.left_jog), ("left stop", self.left_stop)]
        if side in ("right", "both"):
            pairs += [("right jog", self.right_jog), ("right stop", self.right_stop)]

        for name, client in pairs:
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(f"Service unavailable: {name}")

    def axis_to_index(self, axis, direction):
        axis = axis.lower()
        if axis not in AXIS_INDEX:
            raise RuntimeError(f"Unknown axis/joint: {axis}")

        base = AXIS_INDEX[axis] * 2
        if direction == "+":
            return base
        if direction == "-":
            return base + 1
        raise RuntimeError("direction must be + or -")

    def invert_index_direction(self, jog_index):
        if jog_index % 2 == 0:
            return jog_index + 1
        return jog_index - 1

    def make_jog_request(self, coord, jog_index, speed):
        req = Move.Request()

        # Driver jog_callback only uses coord_mode, index, and mvacc.
        # The other fields are filled safely for the Move service schema.
        req.pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = 0.0
        req.mvacc = float(speed)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = int(COORD_MODE[coord])
        req.index = int(jog_index)
        return req

    def get_side_signs(self, side, coord):
        motion = self.cfg.setdefault("motion", {})

        if side == "left":
            if coord == "joint":
                return motion.get("left_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "base":
                return motion.get("left_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "tool":
                return motion.get("left_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

        if side == "right":
            if coord == "joint":
                return motion.get("right_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "base":
                return motion.get("right_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
            if coord == "tool":
                return motion.get("right_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

        return [1, 1, 1, 1, 1, 1]

    def apply_sign_to_index(self, jog_index, factor):
        if factor < 0:
            return self.invert_index_direction(jog_index)
        return jog_index

    def compute_indices(self, side, coord, axis, direction, sync_mode):
        base_index = self.axis_to_index(axis, direction)

        if sync_mode == "raw":
            return base_index, base_index

        axis_i = AXIS_INDEX[axis.lower()]

        left_signs = self.get_side_signs("left", coord)
        right_signs = self.get_side_signs("right", coord)

        if len(left_signs) != 6:
            raise RuntimeError("left sign map must contain 6 values")
        if len(right_signs) != 6:
            raise RuntimeError("right sign map must contain 6 values")

        left_factor = float(left_signs[axis_i])
        right_factor = float(right_signs[axis_i])

        if side == "both" and sync_mode == "mirror":
            right_factor *= -1.0
        elif sync_mode != "same":
            raise RuntimeError("sync-mode must be raw, same, or mirror")

        left_index = self.apply_sign_to_index(base_index, left_factor)
        right_index = self.apply_sign_to_index(base_index, right_factor)

        return left_index, right_index

    def call_jog_once(self, side, coord, left_index, right_index, speed, dry_run=False):
        if dry_run:
            print(
                f"DRY RUN jog: side={side}, coord={coord}, "
                f"left_index={left_index}, right_index={right_index}, speed={speed}"
            )
            return

        futures = []

        if side in ("left", "both"):
            futures.append(("LEFT", self.left_jog.call_async(self.make_jog_request(coord, left_index, speed))))

        if side in ("right", "both"):
            futures.append(("RIGHT", self.right_jog.call_async(self.make_jog_request(coord, right_index, speed))))

        for label, fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
            if fut.done() and fut.result() is not None:
                res = fut.result()
                if res.ret not in (0, 1):
                    print(f"{label} jog ret={res.ret}, msg={res.message}")

    def stop(self, side):
        req = Empty.Request()
        futures = []

        if side in ("left", "both"):
            self.left_stop.wait_for_service(timeout_sec=2.0)
            futures.append(("LEFT", self.left_stop.call_async(req)))

        if side in ("right", "both"):
            self.right_stop.wait_for_service(timeout_sec=2.0)
            futures.append(("RIGHT", self.right_stop.call_async(req)))

        for label, fut in futures:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

        print(f"STOP sent: {side}")

    def jog_hold(self, side, coord, axis, direction, speed, duration, repeat, sync_mode, dry_run):
        self.check_safe(side)
        self.wait_services(side)

        if coord not in COORD_MODE:
            raise RuntimeError("coord must be joint, base, or tool")

        axis = axis.lower()
        if coord == "joint" and axis not in ("j1", "j2", "j3", "j4", "j5", "j6", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "1", "2", "3", "4", "5", "6"):
            raise RuntimeError("joint coord requires axis j1-j6")

        if coord in ("base", "tool") and axis not in ("x", "y", "z", "rx", "ry", "rz"):
            raise RuntimeError("base/tool coord requires axis x, y, z, rx, ry, or rz")

        left_index, right_index = self.compute_indices(
            side=side,
            coord=coord,
            axis=axis,
            direction=direction,
            sync_mode=sync_mode,
        )

        print("=== Jog Hold ===")
        print(f"side      : {side}")
        print(f"coord     : {coord}")
        print(f"axis      : {axis}")
        print(f"direction : {direction}")
        print(f"sync_mode : {sync_mode}")
        print(f"speed     : {speed}")
        print(f"duration  : {duration}")
        print(f"repeat    : {repeat}")
        print(f"left_idx  : {left_index}")
        print(f"right_idx : {right_index}")

        start = time.time()
        try:
            while rclpy.ok() and time.time() - start < duration:
                self.call_jog_once(side, coord, left_index, right_index, speed, dry_run=dry_run)
                time.sleep(repeat)
        finally:
            if not dry_run:
                self.stop(side)


def load_config():
    p = Path.home() / "jaka_ws/dual_arm_app/config/robots.yaml"
    if not p.exists():
        raise FileNotFoundError(p)

    cfg = yaml.safe_load(p.read_text())

    cfg.setdefault("motion", {})
    cfg["motion"].setdefault("left_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("left_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("left_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

    return cfg


def main():
    parser = argparse.ArgumentParser(description="Dual JAKA jog CLI using /jog service")
    sub = parser.add_subparsers(dest="cmd", required=True)

    jog = sub.add_parser("jog")
    jog.add_argument("side", choices=["left", "right", "both"])
    jog.add_argument("coord", choices=["joint", "base", "tool"])
    jog.add_argument("axis")
    jog.add_argument("direction", choices=["+", "-"])
    jog.add_argument("--speed", type=float, default=None)
    jog.add_argument("--duration", type=float, default=0.4)
    jog.add_argument("--repeat", type=float, default=0.15)
    jog.add_argument("--sync-mode", choices=["raw", "same", "mirror"], default="same")
    jog.add_argument("--dry-run", action="store_true")

    stop = sub.add_parser("stop")
    stop.add_argument("side", choices=["left", "right", "both"])

    args = parser.parse_args()

    cfg = load_config()

    if args.cmd == "jog":
        if args.speed is None:
            if args.coord == "joint":
                args.speed = 0.03
            elif args.axis.lower() in ("rx", "ry", "rz"):
                args.speed = 2.0
            else:
                args.speed = 5.0

    rclpy.init()
    node = DualJakaJogCli(cfg)

    try:
        if args.cmd == "jog":
            node.jog_hold(
                side=args.side,
                coord=args.coord,
                axis=args.axis,
                direction=args.direction,
                speed=args.speed,
                duration=args.duration,
                repeat=args.repeat,
                sync_mode=args.sync_mode,
                dry_run=args.dry_run,
            )
        elif args.cmd == "stop":
            node.stop(args.side)

    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        try:
            node.stop("both")
        except Exception:
            pass
        sys.exit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
