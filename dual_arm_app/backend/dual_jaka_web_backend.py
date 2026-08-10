from typing import List, Literal, Optional
#!/usr/bin/env python3
import json
import re
import threading
import time
from pathlib import Path
from typing import Optional, Dict, Any

import yaml
import rclpy
from rclpy.node import Node

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import Move, GetFK, GetIK

try:
    from dual_arm_app.backend.joint_feedback import (
        build_digital_twin_joint_status,
        expected_joint_name_aliases,
        normalize_joint_state_message,
        wall_clock_ms,
    )
except ImportError:
    try:
        from .joint_feedback import (
            build_digital_twin_joint_status,
            expected_joint_name_aliases,
            normalize_joint_state_message,
            wall_clock_ms,
        )
    except ImportError:
        from joint_feedback import (
            build_digital_twin_joint_status,
            expected_joint_name_aliases,
            normalize_joint_state_message,
            wall_clock_ms,
        )

try:
    from dual_arm_app.backend.sampled_path_validation import (
        SampledPathValidationInputError,
        normalize_sampled_path_options,
    )
except ImportError:
    try:
        from .sampled_path_validation import (
            SampledPathValidationInputError,
            normalize_sampled_path_options,
        )
    except ImportError:
        from sampled_path_validation import (
            SampledPathValidationInputError,
            normalize_sampled_path_options,
        )

try:
    from dual_arm_app.backend.moveit_state_validation import (
        MoveItStateValidationBridge,
        TrajectoryValidationInputError,
        empty_validation_result,
    )
except ImportError:
    try:
        from .moveit_state_validation import (
            MoveItStateValidationBridge,
            TrajectoryValidationInputError,
            empty_validation_result,
        )
    except ImportError:
        from moveit_state_validation import (
            MoveItStateValidationBridge,
            TrajectoryValidationInputError,
            empty_validation_result,
        )


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

class JogStartRequest(BaseModel):
    side: str
    coord: str
    axis: str
    direction: str
    speed: float
    sync_mode: str = "same"



class ProgramStepRequest(BaseModel):
    name: str
    side: Optional[str] = None
    vel: Optional[float] = None
    acc: Optional[float] = None
    delay: float = 0.0


class ProgramRunRequest(BaseModel):
    steps: list[ProgramStepRequest]
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None
    loop_mode: Literal["once", "count", "forever"] = "once"
    loop_count: int = Field(default=1, ge=1)


class ProgramSaveRequest(BaseModel):
    name: str
    steps: list[ProgramStepRequest]


class ProgramNameRequest(BaseModel):
    name: str


class DigitalTwinTrajectoryValidationRequest(BaseModel):
    trajectory: Dict[str, Any]
    sampled_path: Optional[Dict[str, Any]] = None


class StopRequest(BaseModel):
    side: str = "both"


class HomeRequest(BaseModel):
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None
    vel: Optional[float] = None
    acc: Optional[float] = None


class WaypointSaveRequest(BaseModel):
    name: str
    side: str = "both"


class WaypointRunRequest(BaseModel):
    name: str
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None


class SequenceRunRequest(BaseModel):
    names: list[str]
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None


class DualJakaWebNode(Node):
    def __init__(self, cfg):
        super().__init__("dual_jaka_web_backend")
        self.cfg = cfg

        self.left_joint = None
        self.right_joint = None
        self.left_state = None
        self.right_state = None

        # Read-only Digital Twin feedback is kept separate from other robot
        # state so the endpoint never needs FK, services, or publishers.
        self.digital_twin_joint_cache_lock = threading.Lock()
        self.digital_twin_expected_joint_names = {
            side: expected_joint_name_aliases(side)
            for side in ("left", "right")
        }
        self.digital_twin_joint_cache = {
            side: {
                "joint": None,
                "received_at_ms": None,
                "mapping": None,
                "error": "No valid JointState received",
                "names": [],
            }
            for side in ("left", "right")
        }
        self.moveit_state_validation_bridge = None
        self.moveit_state_validation_error = None
        try:
            self.moveit_state_validation_bridge = MoveItStateValidationBridge.from_node(
                self,
                service_name="/check_state_validity",
                group_name="dual_arm",
                timeout_s=2.0,
            )
        except Exception as error:
            self.moveit_state_validation_error = str(error)

        self.active_jog: Optional[Dict[str, Any]] = None
        self.active_sequence: Optional[Dict[str, Any]] = None
        self.sequence_cancel_requested = False
        self.stop_generation = 0
        self.active_motion: Optional[Dict[str, Any]] = None
        self.active_lock = threading.Lock()
        self.last_jog_time = 0.0
        self.deadman_timeout = 0.45
        self.repeat_period = 0.12

        self.waypoint_path = Path.home() / "jaka_ws/dual_arm_app/tasks/waypoints.json"
        self.waypoint_path.parent.mkdir(parents=True, exist_ok=True)

        self.create_subscription(JointState, f"{cfg['left']['prefix']}/joint_position", self.left_joint_cb, 10)
        self.create_subscription(JointState, f"{cfg['right']['prefix']}/joint_position", self.right_joint_cb, 10)
        self.create_subscription(RobotMsg, f"{cfg['left']['prefix']}/robot_states", self.left_state_cb, 10)
        self.create_subscription(RobotMsg, f"{cfg['right']['prefix']}/robot_states", self.right_state_cb, 10)

        self.left_jog = self.create_client(Move, f"{cfg['left']['prefix']}/jog")
        self.right_jog = self.create_client(Move, f"{cfg['right']['prefix']}/jog")
        self.left_move = self.create_client(Move, f"{cfg['left']['prefix']}/joint_move")
        self.right_move = self.create_client(Move, f"{cfg['right']['prefix']}/joint_move")
        self.left_linear_move = self.create_client(Move, f"{cfg['left']['prefix']}/linear_move")
        self.right_linear_move = self.create_client(Move, f"{cfg['right']['prefix']}/linear_move")
        self.left_get_fk = self.create_client(GetFK, f"{cfg['left']['prefix']}/get_fk")
        self.right_get_fk = self.create_client(GetFK, f"{cfg['right']['prefix']}/get_fk")
        self.left_get_ik = self.create_client(GetIK, f"{cfg['left']['prefix']}/get_ik")
        self.right_get_ik = self.create_client(GetIK, f"{cfg['right']['prefix']}/get_ik")

        self.left_stop = self.create_client(Empty, f"{cfg['left']['prefix']}/stop_move")
        self.right_stop = self.create_client(Empty, f"{cfg['right']['prefix']}/stop_move")

        self.jog_thread = threading.Thread(target=self.jog_loop, daemon=True)
        self.jog_thread.start()

    def left_joint_cb(self, msg):
        self._update_digital_twin_joint_cache("left", msg)

    def right_joint_cb(self, msg):
        self._update_digital_twin_joint_cache("right", msg)

    def _update_digital_twin_joint_cache(self, side, msg):
        received_at_ms = wall_clock_ms()
        result = normalize_joint_state_message(
            getattr(msg, "name", []),
            getattr(msg, "position", None),
            self.digital_twin_expected_joint_names[side],
        )
        with self.digital_twin_joint_cache_lock:
            cache = self.digital_twin_joint_cache[side]
            cache["names"] = list(result["names"])
            cache["error"] = result["error"]
            if result["valid"]:
                normalized_joint = list(result["joint"])
                cache["joint"] = normalized_joint
                cache["received_at_ms"] = received_at_ms
                cache["mapping"] = result["mapping"]
                if side == "left":
                    self.left_joint = list(normalized_joint)
                else:
                    self.right_joint = list(normalized_joint)

    def left_state_cb(self, msg):
        self.left_state = msg

    def right_state_cb(self, msg):
        self.right_state = msg

    def robot_state_dict(self, state):
        if state is None:
            return None
        return {
            "power_state": int(state.power_state),
            "servo_state": int(state.servo_state),
            "motion_state": int(state.motion_state),
            "collision_state": int(state.collision_state),
        }

    def status(self):
        def to_builtin(obj):
            if obj is None:
                return None

            if isinstance(obj, (str, int, float, bool)):
                return obj

            if isinstance(obj, (list, tuple)):
                return [to_builtin(x) for x in obj]

            if isinstance(obj, dict):
                return {str(k): to_builtin(v) for k, v in obj.items()}

            # ROS2 message: use get_fields_and_field_types() if available
            if hasattr(obj, "get_fields_and_field_types"):
                out = {}
                try:
                    for name in obj.get_fields_and_field_types().keys():
                        out[name] = to_builtin(getattr(obj, name))
                    return out
                except Exception:
                    pass

            # ROS2 message fallback: use __slots__
            slots = getattr(obj, "__slots__", None)
            if slots:
                out = {}
                for slot in slots:
                    clean = slot[1:] if str(slot).startswith("_") else slot

                    if hasattr(obj, clean):
                        out[clean] = to_builtin(getattr(obj, clean))
                    elif hasattr(obj, slot):
                        out[clean] = to_builtin(getattr(obj, slot))

                if out:
                    return out

            # Last fallback: avoid FastAPI JSON encoder crash
            return str(obj)

        left_tcp = self.get_fk_pose("left")
        right_tcp = self.get_fk_pose("right")

        return {
            "left": {
                "joint": to_builtin(self.left_joint),
                "tcp": to_builtin(left_tcp),
                "state": to_builtin(self.left_state),
            },
            "right": {
                "joint": to_builtin(self.right_joint),
                "tcp": to_builtin(right_tcp),
                "state": to_builtin(self.right_state),
            },
            "active_jog": to_builtin(self.active_jog),
            "active_motion": to_builtin(self.active_motion),
            "active_sequence": to_builtin(self.active_sequence),
            "waypoints": to_builtin(self.load_waypoints()),
        }

    def digital_twin_joint_status(self):
        """Return only cached joint feedback; never invoke robot operations."""
        with self.digital_twin_joint_cache_lock:
            cache_snapshot = {
                side: {
                    "joint": (
                        list(values["joint"])
                        if values["joint"] is not None
                        else None
                    ),
                    "received_at_ms": values["received_at_ms"],
                    "mapping": values["mapping"],
                    "error": values["error"],
                    "names": list(values["names"]),
                }
                for side, values in self.digital_twin_joint_cache.items()
            }
        return build_digital_twin_joint_status(cache_snapshot)

    def validate_digital_twin_trajectory(self, trajectory, sampled_path=None):
        """Check stored and optional sampled states without invoking motion APIs."""
        normalize_sampled_path_options(sampled_path)
        if self.moveit_state_validation_bridge is None:
            return empty_validation_result(
                "UNAVAILABLE",
                self.moveit_state_validation_error
                or "MoveIt state-validity client is unavailable",
            )
        return self.moveit_state_validation_bridge.validate_trajectory(
            trajectory, sampled_path
        )

    def safe_state_ok(self, side):
        selected = []
        if side in ("left", "both"):
            selected.append(("LEFT", self.left_state))
        if side in ("right", "both"):
            selected.append(("RIGHT", self.right_state))

        for label, s in selected:
            if s is None:
                return False, f"{label}: no state"
            if s.power_state != 1:
                return False, f"{label}: power_state={s.power_state}"
            if s.servo_state != 1:
                return False, f"{label}: servo_state={s.servo_state}"
            if s.collision_state != 0:
                return False, f"{label}: collision_state={s.collision_state}"

        return True, "ok"

    def axis_to_index(self, axis, direction):
        axis = axis.lower()
        if axis not in AXIS_INDEX:
            raise ValueError(f"Unknown axis: {axis}")

        base = AXIS_INDEX[axis] * 2
        if direction == "+":
            return base
        if direction == "-":
            return base + 1
        raise ValueError("direction must be + or -")

    def invert_index_direction(self, idx):
        return idx + 1 if idx % 2 == 0 else idx - 1

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

    def apply_sign_to_index(self, idx, factor):
        return self.invert_index_direction(idx) if factor < 0 else idx

    def compute_indices(self, side, coord, axis, direction, sync_mode):
        base_idx = self.axis_to_index(axis, direction)

        if sync_mode == "raw":
            return base_idx, base_idx

        axis_i = AXIS_INDEX[axis.lower()]
        left_signs = self.get_side_signs("left", coord)
        right_signs = self.get_side_signs("right", coord)

        if len(left_signs) != 6:
            raise ValueError("left sign map must contain 6 values")
        if len(right_signs) != 6:
            raise ValueError("right sign map must contain 6 values")

        left_factor = float(left_signs[axis_i])
        right_factor = float(right_signs[axis_i])

        if side == "both" and sync_mode == "mirror":
            right_factor *= -1.0
        elif sync_mode != "same":
            raise ValueError("sync_mode must be raw, same, or mirror")

        left_idx = self.apply_sign_to_index(base_idx, left_factor)
        right_idx = self.apply_sign_to_index(base_idx, right_factor)

        return left_idx, right_idx

    def make_jog_request(self, coord, index, speed):
        req = Move.Request()
        req.pose = [0.0] * 6
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = 0.0
        req.mvacc = float(speed)  # driver jog ใช้ mvacc เป็น jog velocity
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = int(COORD_MODE[coord])
        req.index = int(index)
        return req

    def wait_future_result(self, future, timeout=0.4):
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None


    def get_fk_pose(self, side_name):
        if side_name == "left":
            joint = self.left_joint
            client = self.left_get_fk
        else:
            joint = self.right_joint
            client = self.right_get_fk

        if joint is None:
            return None

        if not client.service_is_ready():
            return None

        req = GetFK.Request()
        req.joint = [float(x) for x in joint]

        resp = self.wait_future_result(client.call_async(req), timeout=0.4)
        if resp is None:
            return None

        pose = list(getattr(resp, "cartesian_pose", []) or [])
        if len(pose) < 6:
            return None

        return [float(x) for x in pose[:6]]


    def make_linear_move_request(self, pose, vel, acc, ref_joint=None):
        req = Move.Request()
        req.pose = [float(x) for x in pose]

        # สำคัญมาก:
        # Cartesian pose เดียวกันอาจมีหลาย IK solution
        # ถ้าไม่ส่ง ref_joint หุ่นอาจ flip joint ไปอีก posture ได้
        if ref_joint is not None:
            req.has_ref = True
            req.ref_joint = [float(x) for x in ref_joint]
        else:
            req.has_ref = False
            req.ref_joint = []

        req.mvvelo = float(vel)
        req.mvacc = float(acc)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        return req

    def direct_tcp_move(self, side="both", left_pose=None, right_pose=None, vel=None, acc=None):
        # SAFE TCP TARGET:
        # Do NOT use /linear_move directly.
        # Convert TCP target -> IK with current joint as ref_joint -> execute by joint_move.
        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        ok, msg = self.safe_state_ok(side)
        if not ok:
            return {"ok": False, "error": msg}

        joint_vel = float(vel if vel is not None else self.cfg["motion"].get("default_joint_vel", 0.12))
        joint_acc = float(acc if acc is not None else self.cfg["motion"].get("default_joint_acc", 0.20))

        pos_eps = float(self.cfg["motion"].get("tcp_noop_position_epsilon", 0.01))
        rot_eps = float(self.cfg["motion"].get("tcp_noop_rotation_epsilon", 0.001))

        def validate_pose(vals, label):
            if vals is None:
                return None
            if len(vals) != 6:
                return {"error": f"{label} must contain 6 values: X Y Z Rx Ry Rz"}
            try:
                return [float(v) for v in vals]
            except Exception as e:
                return {"error": f"{label} parse error: {e}"}

        def pose_delta(current, target):
            if current is None or target is None:
                return None
            dp = max(abs(float(target[i]) - float(current[i])) for i in range(3))
            dr = max(abs(float(target[i]) - float(current[i])) for i in range(3, 6))
            return dp, dr

        def solve_ik(side_name, pose, ref_joint):
            if side_name == "left":
                client = self.left_get_ik
            else:
                client = self.right_get_ik

            if ref_joint is None:
                return None, f"{side_name} joint feedback missing"

            if not client.service_is_ready():
                return None, f"{side_name} get_ik service not ready"

            req = GetIK.Request()
            req.ref_joint = [float(x) for x in ref_joint]
            req.cartesian_pose = [float(x) for x in pose]

            resp = self.wait_future_result(client.call_async(req), timeout=1.0)
            if resp is None:
                return None, f"{side_name} get_ik timeout/no response"

            joint = list(getattr(resp, "joint", []) or [])
            message = str(getattr(resp, "message", "") or "")

            if len(joint) < 6:
                return None, f"{side_name} IK failed: {message}"

            return [float(x) for x in joint[:6]], message

        left_target_pose = None
        right_target_pose = None

        if side in ("left", "both"):
            left_target_pose = validate_pose(left_pose, "left_pose")
            if isinstance(left_target_pose, dict):
                return {"ok": False, **left_target_pose}

        if side in ("right", "both"):
            right_target_pose = validate_pose(right_pose, "right_pose")
            if isinstance(right_target_pose, dict):
                return {"ok": False, **right_target_pose}

        left_current_tcp = self.get_fk_pose("left") if side in ("left", "both") else None
        right_current_tcp = self.get_fk_pose("right") if side in ("right", "both") else None

        left_delta = None
        right_delta = None
        left_joint_target = None
        right_joint_target = None
        ik_messages = {}

        if side in ("left", "both"):
            left_delta = pose_delta(left_current_tcp, left_target_pose)
            if left_delta is None:
                return {"ok": False, "error": "left TCP feedback/FK missing"}

            if left_delta[0] > pos_eps or left_delta[1] > rot_eps:
                left_joint_target, msg = solve_ik("left", left_target_pose, self.left_joint)
                ik_messages["left"] = msg
                if left_joint_target is None:
                    return {"ok": False, "error": msg}
            else:
                left_joint_target = self.left_joint

        if side in ("right", "both"):
            right_delta = pose_delta(right_current_tcp, right_target_pose)
            if right_delta is None:
                return {"ok": False, "error": "right TCP feedback/FK missing"}

            if right_delta[0] > pos_eps or right_delta[1] > rot_eps:
                right_joint_target, msg = solve_ik("right", right_target_pose, self.right_joint)
                ik_messages["right"] = msg
                if right_joint_target is None:
                    return {"ok": False, "error": msg}
            else:
                right_joint_target = self.right_joint

        left_send = side in ("left", "both") and left_delta is not None and (left_delta[0] > pos_eps or left_delta[1] > rot_eps)
        right_send = side in ("right", "both") and right_delta is not None and (right_delta[0] > pos_eps or right_delta[1] > rot_eps)

        if not left_send and not right_send:
            return {
                "ok": True,
                "accepted": False,
                "message": "TCP target is already at current pose; no move sent",
                "method": "get_ik_to_joint_move",
                "left_delta": left_delta,
                "right_delta": right_delta,
            }

        result = self.move_both_joint(
            left_target=left_joint_target,
            right_target=right_joint_target,
            side=side,
            vel=joint_vel,
            acc=joint_acc,
        )

        result["method"] = "get_ik_to_joint_move"
        result["ik_messages"] = ik_messages
        result["left_tcp_delta"] = left_delta
        result["right_tcp_delta"] = right_delta
        result["note"] = "TCP target solved by IK and executed using joint_move, not linear_move"

        return result

    def make_joint_move_request(self, joints, vel, acc):
        req = Move.Request()
        req.pose = [float(x) for x in joints]
        req.has_ref = False
        req.ref_joint = []
        req.mvvelo = float(vel)
        req.mvacc = float(acc)
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0
        return req

    def send_jog_once(self, cmd):
        side = cmd["side"]
        coord = cmd["coord"]
        axis = cmd["axis"]
        direction = cmd["direction"]
        speed = cmd["speed"]
        sync_mode = cmd["sync_mode"]

        ok, msg = self.safe_state_ok(side)
        if not ok:
            self.get_logger().error(f"Unsafe state during jog: {msg}")
            self.stop_motion("both")
            with self.active_lock:
                self.active_jog = None
            return

        left_idx, right_idx = self.compute_indices(
            side=side,
            coord=coord,
            axis=axis,
            direction=direction,
            sync_mode=sync_mode,
        )

        if side in ("left", "both"):
            self.left_jog.call_async(self.make_jog_request(coord, left_idx, speed))

        if side in ("right", "both"):
            self.right_jog.call_async(self.make_jog_request(coord, right_idx, speed))

    def jog_loop(self):
        while True:
            time.sleep(self.repeat_period)
            with self.active_lock:
                cmd = dict(self.active_jog) if self.active_jog else None

            if cmd is None:
                continue

            if time.time() - self.last_jog_time > self.deadman_timeout:
                self.get_logger().warn("Jog deadman timeout. Stop both.")
                self.stop_motion("both")
                with self.active_lock:
                    self.active_jog = None
                continue

            self.send_jog_once(cmd)

    def start_jog(self, req: JogStartRequest):
        if req.side not in ("left", "right", "both"):
            return {"ok": False, "error": "side must be left, right, or both"}
        if req.coord not in ("joint", "base", "tool"):
            return {"ok": False, "error": "coord must be joint, base, or tool"}
        if req.direction not in ("+", "-"):
            return {"ok": False, "error": "direction must be + or -"}
        if req.sync_mode not in ("raw", "same", "mirror"):
            return {"ok": False, "error": "sync_mode must be raw, same, or mirror"}

        ok, msg = self.safe_state_ok(req.side)
        if not ok:
            return {"ok": False, "error": msg}

        cmd = {
            "side": req.side,
            "coord": req.coord,
            "axis": req.axis.lower(),
            "direction": req.direction,
            "speed": float(req.speed),
            "sync_mode": req.sync_mode,
        }

        with self.active_lock:
            self.active_jog = cmd
            self.last_jog_time = time.time()

        self.send_jog_once(cmd)

        return {"ok": True, "active_jog": cmd}

    def heartbeat_jog(self):
        with self.active_lock:
            if self.active_jog is None:
                return {"ok": False, "error": "no active jog"}
            self.last_jog_time = time.time()
            return {"ok": True, "active_jog": self.active_jog}

    def stop_jog(self, side="both"):
        with self.active_lock:
            self.active_jog = None
            self.active_motion = None
        self.stop_motion(side)
        return {"ok": True, "stopped": side}

    def stop_motion(self, side="both"):
        req = Empty.Request()
        if side in ("left", "both"):
            self.left_stop.call_async(req)
        if side in ("right", "both"):
            self.right_stop.call_async(req)

    def move_both_joint(self, left_target=None, right_target=None, side="both", vel=None, acc=None):
        # Dual-arm synchronized joint move:
        # Treat incoming vel/acc as maximum values, then scale left/right velocity
        # based on each arm's joint distance so both arms finish closer together.
        ok, msg = self.safe_state_ok(side)
        if not ok:
            return {"ok": False, "error": msg}

        max_vel = float(vel if vel is not None else self.cfg["motion"].get("default_joint_vel", 0.18))
        max_acc = float(acc if acc is not None else self.cfg["motion"].get("default_joint_acc", 0.30))

        sync_enabled = bool(self.cfg["motion"].get("sync_joint_move_enabled", True))
        min_duration = float(self.cfg["motion"].get("sync_min_duration", 1.0))
        min_vel = float(self.cfg["motion"].get("sync_min_joint_vel", 0.005))
        min_acc = float(self.cfg["motion"].get("sync_min_joint_acc", 0.02))
        eps = float(self.cfg["motion"].get("sync_delta_epsilon", 0.0003))

        if side in ("left", "both") and left_target is None:
            return {"ok": False, "error": "left target missing"}
        if side in ("right", "both") and right_target is None:
            return {"ok": False, "error": "right target missing"}

        if side in ("left", "both") and self.left_joint is None:
            return {"ok": False, "error": "left joint feedback missing"}
        if side in ("right", "both") and self.right_joint is None:
            return {"ok": False, "error": "right joint feedback missing"}

        def max_abs_delta(current, target):
            return max(abs(float(t) - float(c)) for c, t in zip(current, target))

        left_delta = 0.0
        right_delta = 0.0

        if side in ("left", "both"):
            left_delta = max_abs_delta(self.left_joint, left_target)

        if side in ("right", "both"):
            right_delta = max_abs_delta(self.right_joint, right_target)

        left_send = side in ("left", "both") and left_delta > eps
        right_send = side in ("right", "both") and right_delta > eps

        if not left_send and not right_send:
            return {
                "ok": True,
                "accepted": False,
                "message": "already at target within epsilon",
                "left_delta": left_delta,
                "right_delta": right_delta,
            }

        if sync_enabled and side == "both":
            max_delta = max(left_delta, right_delta)
            target_duration = max(min_duration, max_delta / max(max_vel, 1e-9))

            left_vel = left_delta / target_duration if left_send else 0.0
            right_vel = right_delta / target_duration if right_send else 0.0

            if left_send:
                left_vel = min(max_vel, max(min_vel, left_vel))
            if right_send:
                right_vel = min(max_vel, max(min_vel, right_vel))

            # Scale acceleration roughly with velocity ratio.
            # This helps the shorter-motion arm avoid snapping too aggressively.
            left_acc = max_acc * (left_vel / max(max_vel, 1e-9)) if left_send else 0.0
            right_acc = max_acc * (right_vel / max(max_vel, 1e-9)) if right_send else 0.0

            if left_send:
                left_acc = min(max_acc, max(min_acc, left_acc))
            if right_send:
                right_acc = min(max_acc, max(min_acc, right_acc))

        else:
            target_duration = None
            left_vel = max_vel if left_send else 0.0
            right_vel = max_vel if right_send else 0.0
            left_acc = max_acc if left_send else 0.0
            right_acc = max_acc if right_send else 0.0

        self.motion_cancel_requested = False
        self.active_motion = {
            "side": side,
            "type": "sync_joint_move" if sync_enabled and side == "both" else "single_joint_move",
            "status": "sent",
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "sync_enabled": sync_enabled,
            "max_vel": max_vel,
            "max_acc": max_acc,
            "target_duration": target_duration,
            "left_delta": left_delta,
            "right_delta": right_delta,
            "left_vel": left_vel,
            "right_vel": right_vel,
            "left_acc": left_acc,
            "right_acc": right_acc,
            "left_send": left_send,
            "right_send": right_send,
        }

        if left_send:
            self.left_move.call_async(
                self.make_joint_move_request(left_target, left_vel, left_acc)
            )

        if right_send:
            self.right_move.call_async(
                self.make_joint_move_request(right_target, right_vel, right_acc)
            )

        return {
            "ok": True,
            "accepted": True,
            "message": "sync joint_move sent",
            "side": side,
            "sync_enabled": sync_enabled,
            "target_duration": target_duration,
            "left_delta": left_delta,
            "right_delta": right_delta,
            "left_vel": left_vel,
            "right_vel": right_vel,
            "left_acc": left_acc,
            "right_acc": right_acc,
            "left_send": left_send,
            "right_send": right_send,
        }

    def request_stop_all(self, side="both"):
        # Global stop latch:
        # กด STOP ครั้งเดียวต้องหยุด motion ปัจจุบัน + ยกเลิก sequence ทั้งหมด
        self.stop_generation = getattr(self, "stop_generation", 0) + 1
        self.motion_cancel_requested = True
        self.sequence_cancel_requested = True

        with self.active_lock:
            self.active_jog = None

            if self.active_motion is not None:
                self.active_motion["status"] = "stop_requested"
                self.active_motion["stop_generation"] = self.stop_generation

            if self.active_sequence is not None:
                self.active_sequence["status"] = "stop_requested"
                self.active_sequence["stop_generation"] = self.stop_generation
                self.active_sequence["reason"] = "STOP BOTH pressed"

        self.stop_motion("both")

        return {
            "ok": True,
            "stop_requested": side,
            "stop_generation": self.stop_generation,
            "message": "STOP BOTH: current motion and full sequence cancelled",
        }

    def home(self, side="both", vel=None, acc=None):
        home_cfg = self.cfg.get("home", {})
        left_home = home_cfg.get("left")
        right_home = home_cfg.get("right")

        return self.move_both_joint(
            left_target=left_home,
            right_target=right_home,
            side=side,
            vel=vel,
            acc=acc,
        )

    def load_waypoints(self):
        if not self.waypoint_path.exists():
            return {}
        try:
            return json.loads(self.waypoint_path.read_text())
        except Exception:
            return {}

    def save_waypoints(self, data):
        self.waypoint_path.write_text(json.dumps(data, indent=2))

    def save_waypoint(self, name, side="both"):
        name = name.strip()
        if not name:
            return {"ok": False, "error": "waypoint name is empty"}

        if self.left_joint is None or self.right_joint is None:
            return {"ok": False, "error": "joint feedback missing"}

        data = self.load_waypoints()
        data[name] = {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "side": side,
        }

        if side in ("left", "both"):
            data[name]["left"] = self.left_joint
        if side in ("right", "both"):
            data[name]["right"] = self.right_joint

        self.save_waypoints(data)
        return {"ok": True, "name": name, "waypoint": data[name]}

    def direct_joint_move(self, side="both", left_deg=None, right_deg=None, vel=None, acc=None):
        # Direct joint input from Web UI.
        # UI uses degrees for readability, backend converts to radians.
        import math

        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        def deg_to_rad(vals, label):
            if vals is None:
                return None
            if len(vals) != 6:
                return {"error": f"{label} must contain 6 joint values"}
            try:
                return [float(v) * math.pi / 180.0 for v in vals]
            except Exception as e:
                return {"error": f"{label} parse error: {e}"}

        left_target = None
        right_target = None

        if side in ("left", "both"):
            left_target = deg_to_rad(left_deg, "left_deg")
            if isinstance(left_target, dict):
                return {"ok": False, **left_target}

        if side in ("right", "both"):
            right_target = deg_to_rad(right_deg, "right_deg")
            if isinstance(right_target, dict):
                return {"ok": False, **right_target}

        return self.move_both_joint(
            left_target=left_target,
            right_target=right_target,
            side=side,
            vel=vel,
            acc=acc,
        )


    def delete_waypoint(self, name):
        name = (name or "").strip()
        if not name:
            return {"ok": False, "error": "waypoint name is empty"}

        data = self.load_waypoints()
        if name not in data:
            return {"ok": False, "error": f"waypoint not found: {name}"}

        del data[name]

        wp_file = Path.home() / "jaka_ws/dual_arm_app/tasks/waypoints.json"
        wp_file.parent.mkdir(parents=True, exist_ok=True)
        wp_file.write_text(json.dumps(data, indent=2, ensure_ascii=False))

        return {
            "ok": True,
            "deleted": name,
            "remaining": len(data),
        }


    def run_waypoint(self, name, side="both", vel=None, acc=None):
        data = self.load_waypoints()
        if name not in data:
            return {"ok": False, "error": f"waypoint not found: {name}"}

        wp = data[name]
        left = wp.get("left")
        right = wp.get("right")

        return self.move_both_joint(
            left_target=left,
            right_target=right,
            side=side,
            vel=vel,
            acc=acc,
        )

    def is_motion_idle(self, side="both"):
        selected = []
        if side in ("left", "both"):
            selected.append(self.left_state)
        if side in ("right", "both"):
            selected.append(self.right_state)

        for st in selected:
            if st is None:
                return False
            if int(st.motion_state) != 0:
                return False
        return True

    def wait_motion_done(self, side="both", timeout=120.0, stop_generation_at_start=None):
        start = time.time()

        def should_cancel():
            if self.sequence_cancel_requested:
                return True
            if self.motion_cancel_requested:
                return True
            if stop_generation_at_start is not None:
                if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                    return True
            return False

        def motion_state_of(state):
            if state is None:
                return None

            if isinstance(state, dict):
                return state.get("motion_state", None)

            try:
                return getattr(state, "motion_state")
            except Exception:
                return None

        def is_moving_state(v):
            # JAKA motion_state = 0 means idle / not moving
            if v is None:
                return False
            try:
                return int(v) != 0
            except Exception:
                return False

        active = self.active_motion if isinstance(self.active_motion, dict) else {}

        track_left = side in ("left", "both")
        track_right = side in ("right", "both")

        # ถ้า move_both_joint ระบุว่าแขนนั้นไม่ได้ถูกส่งคำสั่ง ก็ไม่ต้อง track
        if side == "both":
            if active.get("left_send") is False:
                track_left = False
            if active.get("right_send") is False:
                track_right = False

        left_started_at = None
        right_started_at = None
        left_finished_at = None
        right_finished_at = None

        # ถ้าไม่ได้ track ให้ถือว่า finished ตั้งแต่ start
        if not track_left:
            left_finished_at = start
        if not track_right:
            right_finished_at = start

        while time.time() - start < timeout:
            now = time.time()

            if should_cancel():
                self.stop_motion("both")
                self._update_motion_finish_log(
                    start,
                    left_started_at,
                    right_started_at,
                    left_finished_at,
                    right_finished_at,
                    status="cancelled",
                )
                return False, "cancelled"

            left_state = motion_state_of(self.left_state)
            right_state = motion_state_of(self.right_state)

            left_moving = is_moving_state(left_state)
            right_moving = is_moving_state(right_state)

            if track_left:
                if left_started_at is None and left_moving:
                    left_started_at = now

                if left_started_at is not None and not left_moving and left_finished_at is None:
                    left_finished_at = now

                # motion สั้นมากจนไม่ทันเห็น moving state
                if left_started_at is None and left_finished_at is None and now - start > 1.0 and not left_moving:
                    left_finished_at = now

            if track_right:
                if right_started_at is None and right_moving:
                    right_started_at = now

                if right_started_at is not None and not right_moving and right_finished_at is None:
                    right_finished_at = now

                # motion สั้นมากจนไม่ทันเห็น moving state
                if right_started_at is None and right_finished_at is None and now - start > 1.0 and not right_moving:
                    right_finished_at = now

            if left_finished_at is not None and right_finished_at is not None:
                self._update_motion_finish_log(
                    start,
                    left_started_at,
                    right_started_at,
                    left_finished_at,
                    right_finished_at,
                    status="done",
                )
                return True, "done"

            time.sleep(0.05)

        self._update_motion_finish_log(
            start,
            left_started_at,
            right_started_at,
            left_finished_at,
            right_finished_at,
            status="timeout",
        )
        return False, "timeout"


    def _update_motion_finish_log(
        self,
        start,
        left_started_at,
        right_started_at,
        left_finished_at,
        right_finished_at,
        status="unknown",
    ):
        def rel(t):
            if t is None:
                return None
            return round(float(t - start), 3)

        finish_delta = None
        if left_finished_at is not None and right_finished_at is not None:
            finish_delta = round(abs(left_finished_at - right_finished_at), 3)

        log = {
            "status": status,
            "left_started_sec": rel(left_started_at),
            "right_started_sec": rel(right_started_at),
            "left_finished_sec": rel(left_finished_at),
            "right_finished_sec": rel(right_finished_at),
            "finish_delta_sec": finish_delta,
        }

        if isinstance(self.active_motion, dict):
            self.active_motion["finish_log"] = log
            if status in ("done", "cancelled", "timeout"):
                self.active_motion["finish_status"] = status

        return log

    def _programs_dir(self):
        d = Path(__file__).resolve().parents[1] / "programs"
        d.mkdir(parents=True, exist_ok=True)
        return d


    def _safe_program_filename(self, name):
        raw = str(name or "").strip()
        raw = raw.replace(" ", "_")
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw).strip("._")

        if not safe:
            raise ValueError("program name is empty")

        if not safe.endswith(".json"):
            safe += ".json"

        # กัน path traversal
        if "/" in safe or "\\" in safe or ".." in safe:
            raise ValueError("invalid program name")

        return safe


    def _program_step_to_dict(self, step, default_side="both", default_vel=None, default_acc=None):
        def get(key, default=None):
            if isinstance(step, dict):
                return step.get(key, default)
            return getattr(step, key, default)

        name = get("name", None)
        if not name:
            raise ValueError("program step missing waypoint name")

        side = str(get("side", default_side) or default_side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            raise ValueError(f"invalid step side: {side}")

        vel = get("vel", default_vel)
        acc = get("acc", default_acc)
        delay = get("delay", 0.0)

        return {
            "name": str(name),
            "side": side,
            "vel": float(vel) if vel is not None else None,
            "acc": float(acc) if acc is not None else None,
            "delay": max(0.0, float(delay or 0.0)),
        }


    def normalize_program_steps(self, steps, default_side="both", default_vel=None, default_acc=None):
        return [
            self._program_step_to_dict(
                step,
                default_side=default_side,
                default_vel=default_vel,
                default_acc=default_acc,
            )
            for step in steps
        ]


    def save_program_file(self, name, steps):
        try:
            filename = self._safe_program_filename(name)
            path = self._programs_dir() / filename

            normalized_steps = self.normalize_program_steps(steps)

            payload = {
                "name": filename[:-5],
                "file": filename,
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "steps": normalized_steps,
            }

            path.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )

            return {
                "ok": True,
                "message": "program saved",
                "file": filename,
                "path": str(path),
                "steps": normalized_steps,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def list_program_files(self):
        try:
            d = self._programs_dir()
            items = []

            for path in sorted(d.glob("*.json")):
                try:
                    stat = path.stat()
                    items.append({
                        "name": path.stem,
                        "file": path.name,
                        "size": stat.st_size,
                        "modified_at": time.strftime(
                            "%Y-%m-%d %H:%M:%S",
                            time.localtime(stat.st_mtime),
                        ),
                    })
                except Exception:
                    pass

            return {"ok": True, "programs": items}

        except Exception as e:
            return {"ok": False, "error": str(e), "programs": []}


    def load_program_file(self, name):
        try:
            filename = self._safe_program_filename(name)
            path = self._programs_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"program not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))
            steps = self.normalize_program_steps(payload.get("steps", []))

            return {
                "ok": True,
                "message": "program loaded",
                "name": payload.get("name", path.stem),
                "file": filename,
                "steps": steps,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def delete_program_file(self, name):
        try:
            filename = self._safe_program_filename(name)
            path = self._programs_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"program not found: {filename}"}

            path.unlink()
            return {"ok": True, "message": "program deleted", "file": filename}

        except Exception as e:
            return {"ok": False, "error": str(e)}



    def run_program(self, steps, side="both", vel=None, acc=None, loop_mode="once", loop_count=1):
        if not steps:
            return {"ok": False, "error": "program is empty"}

        loop_mode = str(loop_mode or "once").lower().strip()
        if loop_mode not in ("once", "count", "forever"):
            return {"ok": False, "error": f"invalid loop mode: {loop_mode}"}

        loop_count = int(loop_count if loop_count is not None else 1)
        if loop_mode == "count" and loop_count < 1:
            return {"ok": False, "error": "loop_count must be at least 1 for count mode"}
        if loop_mode == "once":
            loop_count = 1

        if self.active_sequence is not None and self.active_sequence.get("status") == "running":
            return {
                "ok": False,
                "error": "sequence/program already running",
                "active_sequence": self.active_sequence,
            }

        def step_get(step, key, default=None):
            if isinstance(step, dict):
                return step.get(key, default)
            return getattr(step, key, default)

        normalized_steps = []
        for i, step in enumerate(steps):
            name = step_get(step, "name", None)
            if not name:
                return {"ok": False, "error": f"step {i} missing waypoint name"}

            step_side = (step_get(step, "side", None) or side or "both").lower().strip()
            if step_side not in ("left", "right", "both"):
                return {"ok": False, "error": f"step {i} invalid side: {step_side}"}

            step_vel = step_get(step, "vel", None)
            step_acc = step_get(step, "acc", None)
            step_delay = step_get(step, "delay", 0.0)

            step_vel = float(step_vel if step_vel is not None else vel) if (step_vel is not None or vel is not None) else None
            step_acc = float(step_acc if step_acc is not None else acc) if (step_acc is not None or acc is not None) else None
            step_delay = max(0.0, float(step_delay or 0.0))

            normalized_steps.append({
                "name": str(name),
                "side": step_side,
                "vel": step_vel,
                "acc": step_acc,
                "delay": step_delay,
            })

        self.sequence_cancel_requested = False
        self.motion_cancel_requested = False
        stop_generation_at_start = getattr(self, "stop_generation", 0)
        status_loop_count = None if loop_mode == "forever" else loop_count

        self.active_sequence = {
            "type": "program",
            "status": "running",
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "current_index": 0,
            "current_name": None,
            "steps": normalized_steps,
            "side": side,
            "default_vel": vel,
            "default_acc": acc,
            "stop_generation": stop_generation_at_start,
            "loop_mode": loop_mode,
            "loop_count": status_loop_count,
            "current_cycle": 1,
            "completed_cycles": 0,
            "total_steps": len(normalized_steps),
        }

        th = threading.Thread(
            target=self._program_thread,
            args=(normalized_steps, stop_generation_at_start, loop_mode, loop_count),
            daemon=True,
        )
        th.start()

        return {
            "ok": True,
            "accepted": True,
            "message": "program started",
            "steps": normalized_steps,
            "stop_generation": stop_generation_at_start,
            "loop_mode": loop_mode,
            "loop_count": status_loop_count,
        }


    def _program_thread(self, steps, stop_generation_at_start=0, loop_mode="once", loop_count=1):
        def should_cancel():
            if self.sequence_cancel_requested:
                return True
            if self.motion_cancel_requested:
                return True
            if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                return True
            return False

        def update_state(status, **fields):
            if self.active_sequence is None:
                self.active_sequence = {}
            self.active_sequence["type"] = "program"
            self.active_sequence["status"] = status
            self.active_sequence.update(fields)

        try:
            requested_cycles = None if loop_mode == "forever" else loop_count
            completed_cycles = 0

            while requested_cycles is None or completed_cycles < requested_cycles:
                current_cycle = completed_cycles + 1
                update_state("running", current_cycle=current_cycle)

                for i, step in enumerate(steps):
                    if should_cancel():
                        update_state(
                            "cancelled",
                            cancelled_at_index=i,
                            reason="cancel requested before step",
                        )
                        return

                    name = step["name"]
                    step_side = step.get("side", "both")
                    step_vel = step.get("vel", None)
                    step_acc = step.get("acc", None)
                    step_delay = float(step.get("delay", 0.0) or 0.0)

                    update_state(
                        "running",
                        current_index=i,
                        current_name=name,
                        current_step=step,
                        progress=f"{i + 1}/{len(steps)}",
                    )

                    result = self.run_waypoint(
                        name,
                        side=step_side,
                        vel=step_vel,
                        acc=step_acc,
                    )

                    if should_cancel():
                        update_state(
                            "cancelled",
                            cancelled_at_index=i,
                            reason="cancel requested after waypoint command",
                        )
                        return

                    if not result.get("ok", False):
                        update_state(
                            "error",
                            failed_index=i,
                            failed_name=name,
                            result=result,
                        )
                        return

                    done, msg = self.wait_motion_done(
                        side=step_side,
                        timeout=120.0,
                        stop_generation_at_start=stop_generation_at_start,
                    )

                    if not done:
                        update_state(
                            "cancelled" if msg == "cancelled" else "error",
                            failed_index=i,
                            failed_name=name,
                            reason=msg,
                        )
                        return

                    if should_cancel():
                        update_state(
                            "cancelled",
                            cancelled_at_index=i,
                            reason="cancel requested after current motion",
                        )
                        return

                    if step_delay > 0:
                        update_state("delay", delay_sec=step_delay)

                        delay_start = time.time()
                        while time.time() - delay_start < step_delay:
                            if should_cancel():
                                update_state(
                                    "cancelled",
                                    cancelled_at_index=i,
                                    reason="cancel requested during delay",
                                )
                                return
                            time.sleep(0.05)

                completed_cycles += 1
                update_state("running", completed_cycles=completed_cycles)

                if requested_cycles is not None and completed_cycles >= requested_cycles:
                    break

                if should_cancel():
                    update_state("cancelled", reason="cancel requested between cycles")
                    return

            update_state(
                "done",
                finished_at=time.strftime("%Y-%m-%d %H:%M:%S"),
                count=len(steps),
            )

        except Exception as e:
            update_state("exception", error=str(e))


    def run_sequence(self, names, side="both", vel=None, acc=None):
        if not names:
            return {"ok": False, "error": "sequence is empty"}

        if self.active_sequence is not None and self.active_sequence.get("status") == "running":
            return {
                "ok": False,
                "error": "sequence already running",
                "active_sequence": self.active_sequence,
            }

        data = self.load_waypoints()
        missing = [n for n in names if n not in data]
        if missing:
            return {"ok": False, "error": f"missing waypoints: {missing}"}

        self.sequence_cancel_requested = False
        self.motion_cancel_requested = False

        stop_generation_at_start = getattr(self, "stop_generation", 0)

        self.active_sequence = {
            "status": "running",
            "names": names,
            "current_index": 0,
            "current_name": None,
            "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "side": side,
            "vel": vel,
            "acc": acc,
            "stop_generation_at_start": stop_generation_at_start,
        }

        th = threading.Thread(
            target=self._sequence_thread,
            args=(names, side, vel, acc, stop_generation_at_start),
            daemon=True,
        )
        th.start()

        return {
            "ok": True,
            "accepted": True,
            "message": "sequence started",
            "names": names,
            "side": side,
            "vel": vel,
            "acc": acc,
            "stop_generation_at_start": stop_generation_at_start,
        }

    def _sequence_thread(self, names, side="both", vel=None, acc=None, stop_generation_at_start=0):
        try:
            def should_cancel():
                return (
                    self.sequence_cancel_requested
                    or self.motion_cancel_requested
                    or getattr(self, "stop_generation", 0) != stop_generation_at_start
                )

            for i, name in enumerate(names):
                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested before step",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                if self.active_sequence is not None:
                    self.active_sequence["status"] = "running"
                    self.active_sequence["current_index"] = i
                    self.active_sequence["current_name"] = name

                result = self.run_waypoint(name, side=side, vel=vel, acc=acc)

                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested after sending waypoint",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                if not result.get("ok", False):
                    self.active_sequence = {
                        "status": "error",
                        "error": result,
                        "failed_at": name,
                        "index": i,
                    }
                    return

                ok, msg = self.wait_motion_done(
                    side=side,
                    timeout=120.0,
                    stop_generation_at_start=stop_generation_at_start,
                )

                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested after current motion",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                if not ok:
                    self.active_sequence = {
                        "status": "cancelled" if msg == "cancelled" else "error",
                        "message": msg,
                        "at": name,
                        "index": i,
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

                time.sleep(0.15)

                if should_cancel():
                    self.stop_motion("both")
                    self.active_sequence = {
                        "status": "cancelled",
                        "cancelled_at": name,
                        "index": i,
                        "reason": "stop requested before next step",
                        "stop_generation": getattr(self, "stop_generation", 0),
                    }
                    return

            self.active_sequence = {
                "status": "done",
                "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "names": names,
            }

        except Exception as e:
            self.active_sequence = {
                "status": "error",
                "error": str(e),
            }



def load_config():
    p = Path.home() / "jaka_ws/dual_arm_app/config/robots.yaml"
    if not p.exists():
        raise FileNotFoundError(f"Config file not found: {p}")

    cfg = yaml.safe_load(p.read_text())

    cfg.setdefault("motion", {})
    cfg["motion"].setdefault("default_joint_vel", 0.18)
    cfg["motion"].setdefault("default_joint_acc", 0.30)

    cfg["motion"].setdefault("left_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_joint_sign_same_physical", [1, 1, 1, 1, 1, 1])

    cfg["motion"].setdefault("left_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_tcp_base_sign_same_physical", [1, 1, 1, 1, 1, 1])

    cfg["motion"].setdefault("left_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])
    cfg["motion"].setdefault("right_tcp_tool_sign_same_physical", [1, 1, 1, 1, 1, 1])

    cfg.setdefault("home", {})
    cfg.setdefault("left", {})
    cfg.setdefault("right", {})

    return cfg




class DirectTcpMoveRequest(BaseModel):
    side: str = "both"
    left_pose: Optional[List[float]] = None
    right_pose: Optional[List[float]] = None
    vel: Optional[float] = None
    acc: Optional[float] = None

class DirectJointMoveRequest(BaseModel):
    side: str = "both"
    left_deg: Optional[List[float]] = None
    right_deg: Optional[List[float]] = None
    vel: Optional[float] = None
    acc: Optional[float] = None


class WaypointDeleteRequest(BaseModel):
    name: str


# ---------------------------------------------------------------------
# FastAPI app bootstrap
# ---------------------------------------------------------------------

import threading
import rclpy
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path

if not rclpy.ok():
    rclpy.init()

node = DualJakaWebNode(load_config())

spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
spin_thread.start()

app = FastAPI(title="Dual JAKA A12 Web Backend")

_BACKEND_DIR = Path(__file__).resolve().parent
_DUAL_ARM_APP_DIR = _BACKEND_DIR.parent
_REPOSITORY_ROOT = _DUAL_ARM_APP_DIR.parent
_WEB_DIR = _DUAL_ARM_APP_DIR / "web"
_DIGITAL_TWIN_ASSETS_DIR = _WEB_DIR / "assets"
_A12_MESH_DIR = (
    _REPOSITORY_ROOT
    / "src/jaka_ros2/src/jaka_description/meshes/jaka_a12_meshes"
)

# Static-only Digital Twin resources. These mounts do not initialize clients or
# add any robot command surface.
app.mount(
    "/digital-twin/assets",
    StaticFiles(directory=str(_DIGITAL_TWIN_ASSETS_DIR), check_dir=True),
    name="digital-twin-assets",
)
app.mount(
    "/digital-twin/meshes",
    StaticFiles(directory=str(_A12_MESH_DIR), check_dir=True),
    name="digital-twin-meshes",
)
app.mount(
    "/web-assets",
    StaticFiles(directory=str(_WEB_DIR), check_dir=True),
    name="web-assets",
)


# D33.3B Backend Robot Activity State
# Tracks whether a web/API motion command is currently being handled by backend.
import time as _d33_activity_time
from threading import Lock as _D33ActivityLock

_D33_ACTIVITY_LOCK = _D33ActivityLock()
_D33_ACTIVE_COMMANDS = 0
# D33.3C Backend Settling Hold
_D33_SETTLE_HOLD_SEC = 3.0
_D33_HOLD_UNTIL = 0.0
_D33_LAST_ACTIVITY = {
    "state": "idle",
    "active": False,
    "active_commands": 0,
    "last_path": None,
    "last_started_at": None,
    "last_finished_at": None,
    "last_duration_sec": None,
    "last_error": None,
}

_D33_MOTION_EXACT_PATHS = set()

_D33_MOTION_KEYWORDS = (
    "/home",
    "/home_both",
    "/jog",
    "/stop",
    "/run",
    "/program/run",
    "/joint_move",
    "/tcp_move",
    "/direct_move",
    "/direct_joint",
    "/move_both",
)

_D33_NO_MOTION_PATHS = (
    "/api/status",
    "/api/robot/activity",
    "/api/waypoints",
    "/api/program/list",
    "/api/program/load",
    "/api/digital-twin/validate-trajectory",
)

def _d33_is_motion_command(path: str, method: str) -> bool:
    method = (method or "GET").upper()
    path = str(path or "")

    if method == "GET":
        return False

    for no_motion in _D33_NO_MOTION_PATHS:
        if path.startswith(no_motion):
            return False

    if path in _D33_MOTION_EXACT_PATHS:
        return True

    return any(k in path for k in _D33_MOTION_KEYWORDS)

def _d33_activity_start(path: str):
    global _D33_ACTIVE_COMMANDS
    now = _d33_activity_time.time()
    with _D33_ACTIVITY_LOCK:
        _D33_ACTIVE_COMMANDS += 1
        _D33_LAST_ACTIVITY.update({
            "state": "running",
            "active": True,
            "active_commands": _D33_ACTIVE_COMMANDS,
            "last_path": path,
            "last_started_at": now,
            "last_finished_at": None,
            "last_duration_sec": None,
            "last_error": None,
        })
    return now

def _d33_activity_finish(path: str, started_at: float, error: str = None):
    global _D33_ACTIVE_COMMANDS, _D33_HOLD_UNTIL
    now = _d33_activity_time.time()
    duration = now - started_at if started_at else None

    with _D33_ACTIVITY_LOCK:
        _D33_ACTIVE_COMMANDS = max(0, _D33_ACTIVE_COMMANDS - 1)

        hold_sec = 0.0
        if error is None:
            low_path = str(path or "").lower()

            # Do not keep RUNNING after an explicit stop command.
            if "/stop" not in low_path:
                hold_sec = _D33_SETTLE_HOLD_SEC
                _D33_HOLD_UNTIL = max(_D33_HOLD_UNTIL, now + hold_sec)

        if error:
            state = "error"
        elif _D33_ACTIVE_COMMANDS > 0:
            state = "running"
        elif hold_sec > 0:
            state = "settling"
        else:
            state = "idle"

        _D33_LAST_ACTIVITY.update({
            "state": state,
            "active": (_D33_ACTIVE_COMMANDS > 0) or (state == "settling"),
            "active_commands": _D33_ACTIVE_COMMANDS,
            "last_path": path,
            "last_finished_at": now,
            "last_duration_sec": duration,
            "last_error": error,
            "settling_until": _D33_HOLD_UNTIL,
            "settling_hold_sec": hold_sec,
        })


def _d33_activity_snapshot():
    now = _d33_activity_time.time()
    with _D33_ACTIVITY_LOCK:
        data = dict(_D33_LAST_ACTIVITY)

        settling_until = float(data.get("settling_until") or _D33_HOLD_UNTIL or 0.0)
        settling_remaining = max(0.0, settling_until - now)

        if _D33_ACTIVE_COMMANDS > 0:
            data["state"] = "running"
            data["active"] = True
            data["active_commands"] = _D33_ACTIVE_COMMANDS
        elif settling_remaining > 0 and data.get("state") != "error":
            data["state"] = "settling"
            data["active"] = True
            data["active_commands"] = 0
            data["settling_remaining_sec"] = settling_remaining
        elif data.get("state") != "error":
            data["state"] = "idle"
            data["active"] = False
            data["active_commands"] = 0
            data["settling_remaining_sec"] = 0.0

        data["now"] = now
        return data


@app.middleware("http")
async def d33_robot_activity_middleware(request, call_next):
    path = str(request.url.path)
    motion = _d33_is_motion_command(path, request.method)
    started_at = None

    if motion:
        started_at = _d33_activity_start(path)

    try:
        response = await call_next(request)
        if motion and response.status_code >= 400:
            _d33_activity_finish(path, started_at, error=f"HTTP {response.status_code}")
        elif motion:
            _d33_activity_finish(path, started_at, error=None)
        return response
    except Exception as exc:
        if motion:
            _d33_activity_finish(path, started_at, error=str(exc))
        raise

@app.get("/api/robot/activity")
async def api_robot_activity():
    return _d33_activity_snapshot()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
def index():
    return FileResponse(str(Path.home() / "jaka_ws/dual_arm_app/web/index.html"))


@app.get("/api/status")
def api_status():
    return node.status()


@app.get("/api/digital-twin/joints")
def api_digital_twin_joints():
    return node.digital_twin_joint_status()


@app.post("/api/digital-twin/validate-trajectory")
def api_digital_twin_validate_trajectory(req: DigitalTwinTrajectoryValidationRequest):
    try:
        validation = node.validate_digital_twin_trajectory(
            req.trajectory, req.sampled_path
        )
    except (TrajectoryValidationInputError, SampledPathValidationInputError) as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    sampled_status = (validation.get("sampled_path") or {}).get("status")
    return {
        "ok": (
            validation["status"] in {"PASS", "FAIL"}
            and sampled_status not in {"UNAVAILABLE", "TIMEOUT", "ERROR"}
        ),
        "validation": validation,
    }


@app.post("/api/jog/start")
def api_jog_start(req: JogStartRequest):
    return node.start_jog(req)


@app.post("/api/jog/heartbeat")
def api_jog_heartbeat():
    return node.heartbeat_jog()


@app.post("/api/jog/stop")
def api_jog_stop(req: StopRequest):
    return node.stop_jog(req.side)


@app.post("/api/stop")
def api_stop(req: StopRequest):
    return node.request_stop_all(req.side)


@app.post("/api/home")
def api_home(req: HomeRequest):
    return node.home(req.side, req.vel, req.acc)


@app.get("/api/waypoints")
def api_waypoints():
    return node.load_waypoints()


@app.post("/api/waypoint/save")
def api_waypoint_save(req: WaypointSaveRequest):
    return node.save_waypoint(req.name, req.side)


@app.post("/api/waypoint/run")
def api_waypoint_run(req: WaypointRunRequest):
    return node.run_waypoint(req.name, req.side, req.vel, req.acc)


@app.post("/api/sequence/run")
def api_sequence_run(req: SequenceRunRequest):
    return node.run_sequence(req.names, req.side, req.vel, req.acc)









@app.get("/api/program/list")
def api_program_list():
    return node.list_program_files()


@app.post("/api/program/save")
def api_program_save(req: ProgramSaveRequest):
    return node.save_program_file(req.name, req.steps)


@app.post("/api/program/load")
def api_program_load(req: ProgramNameRequest):
    return node.load_program_file(req.name)


@app.post("/api/program/delete")
def api_program_delete(req: ProgramNameRequest):
    return node.delete_program_file(req.name)

@app.post("/api/program/run")
def api_program_run(req: ProgramRunRequest):
    return node.run_program(
        req.steps,
        req.side,
        req.vel,
        req.acc,
        req.loop_mode,
        req.loop_count,
    )

@app.post("/api/sequence/stop")
def api_sequence_stop(req: StopRequest):
    return node.request_stop_all(req.side)


@app.post("/api/direct/joint_move")
def api_direct_joint_move(req: DirectJointMoveRequest):
    return node.direct_joint_move(req.side, req.left_deg, req.right_deg, req.vel, req.acc)


@app.post("/api/waypoint/delete")
def api_waypoint_delete(req: WaypointDeleteRequest):
    return node.delete_waypoint(req.name)


@app.post("/api/direct/tcp_move")
def api_direct_tcp_move(req: DirectTcpMoveRequest):
    return node.direct_tcp_move(req.side, req.left_pose, req.right_pose, req.vel, req.acc)
