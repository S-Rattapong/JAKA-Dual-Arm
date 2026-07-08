from typing import List, Optional
#!/usr/bin/env python3
import json
import math
import re
import threading
import time
from pathlib import Path
from typing import Optional, Dict, Any

import yaml
import rclpy
from rclpy.node import Node

from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from jaka_msgs.msg import RobotMsg
from jaka_msgs.srv import Move, GetFK, GetIK, ServoMoveEnable, ServoMove


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

DEFAULT_OBJECT_MAX_TRANSLATION_MM = 30.0
DEFAULT_OBJECT_MAX_ROTATION_RAD = 0.10
MAX_IK_JOINT_JUMP_RAD = math.radians(25.0)
MAX_IK_TOTAL_JUMP_NORM_RAD = math.radians(80.0)
HARD_MAX_CALIBRATED_WORLD_TRANSLATION_MM = 10.0
HARD_MAX_CALIBRATED_WORLD_ROTATION_RAD = 0.05
HARD_MAX_CALIBRATED_WORLD_JOINT_DELTA_RAD = 0.2
HARD_MAX_CALIBRATED_WORLD_JOINT_DELTA_SUM_RAD = 0.6
HARD_MAX_CALIBRATED_WORLD_SPEED_SCALE = 0.05
HARD_LARGE_MAX_TRANSLATION_MM = 100.0
HARD_LARGE_MAX_ROTATION_RAD = 0.5
HARD_LARGE_MAX_SEGMENT_TRANSLATION_MM = 10.0
HARD_LARGE_MAX_SEGMENT_ROTATION_RAD = 0.05
HARD_LARGE_MAX_SEGMENT_TCP_ROTATION_RAD = 0.15
HARD_LARGE_MAX_JOINT_DELTA_RAD_PER_SEGMENT = 0.2
HARD_LARGE_MAX_JOINT_DELTA_SUM_RAD_PER_SEGMENT = 0.6
HARD_LARGE_MAX_SPEED_SCALE = 0.05


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


class ProgramSaveRequest(BaseModel):
    name: str
    steps: list[ProgramStepRequest]


class ProgramNameRequest(BaseModel):
    name: str


class ObjectFrameCaptureRequest(BaseModel):
    name: str
    object_pose: Optional[List[float]] = None






class ServoEnableRequest(BaseModel):
    side: str = "both"
    enable: bool = True


class ServoTinyTestRequest(BaseModel):
    side: str = "left"
    joint_index: int = 5
    delta: float = 0.0005
    repeats: int = 1
    interval: float = 0.05
    auto_enable: bool = True
    auto_disable: bool = True


class ObjectServoStreamRequest(BaseModel):
    name: str
    target_object_pose: List[float]
    side: str = "both"
    steps: int = 120
    interval: float = 0.025
    auto_enable: bool = True
    auto_disable: bool = True

class ObjectSetCurrentPoseRequest(BaseModel):
    name: str
    current_object_pose: List[float]


class ObjectExecuteInterpolatedRequest(BaseModel):
    name: str
    target_object_pose: List[float]
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None
    steps: int = 10
    delay: float = 0.0

class ObjectExecuteMoveRequest(BaseModel):
    name: str
    target_object_pose: List[float]
    side: str = "both"
    vel: Optional[float] = None
    acc: Optional[float] = None

class ObjectPreviewRequest(BaseModel):
    name: str
    target_object_pose: List[float]

class ObjectValidateMotionRequest(BaseModel):
    name: str
    target_object_pose: List[float]

class ObjectValidateIkJumpRequest(BaseModel):
    name: str
    target_object_pose: List[float]
    side: str = "both"

class ObjectCalibrationCapturePointRequest(BaseModel):
    name: str
    point_type: str
    source: str = "left_tcp"

class ObjectCalibrationAxesPreviewRequest(BaseModel):
    name: str
    axis_length_mm: float = 100.0

class ObjectCalibratedRigidCompareRequest(BaseModel):
    name: str
    target_delta: List[float]

class ObjectCalibratedWorldIkValidateRequest(BaseModel):
    name: str
    target_delta: List[float]
    max_joint_delta_rad: Optional[float] = 0.35
    max_joint_delta_sum_rad: Optional[float] = 1.2

class ObjectCalibratedWorldRigidExecuteRequest(BaseModel):
    name: str
    target_delta: List[float]
    dry_run: bool = True
    confirm_execute: str = ""
    max_translation_mm: Optional[float] = 5.0
    max_rotation_rad: Optional[float] = 0.02
    max_joint_delta_rad: Optional[float] = 0.12
    max_joint_delta_sum_rad: Optional[float] = 0.35
    speed_scale: Optional[float] = 0.03

class ObjectCalibratedWorldRigidLargeExecuteRequest(BaseModel):
    name: str
    target_delta: List[float]
    dry_run: bool = True
    confirm_execute: str = ""
    max_translation_mm: Optional[float] = 100.0
    max_rotation_rad: Optional[float] = 0.5
    max_segment_translation_mm: Optional[float] = 10.0
    max_segment_rotation_rad: Optional[float] = 0.05
    max_segment_tcp_rotation_rad: Optional[float] = 0.12
    max_joint_delta_rad_per_segment: Optional[float] = 0.12
    max_joint_delta_sum_rad_per_segment: Optional[float] = 0.35
    speed_scale: Optional[float] = 0.03
    auto_commit: bool = False

class ObjectPendingCalibratedExecuteCommitStatusRequest(BaseModel):
    name: str

class ObjectVerifyPendingCalibratedExecuteCommitRequest(BaseModel):
    name: str
    position_tolerance_mm: Optional[float] = 1.5
    orientation_tolerance_rad: Optional[float] = 0.05

class ObjectCommitPendingCalibratedExecutePoseRequest(BaseModel):
    name: str
    confirm_commit: str = ""
    position_tolerance_mm: Optional[float] = 1.5
    orientation_tolerance_rad: Optional[float] = 0.05

class ObjectRecoverPendingCalibratedExecuteCommitRequest(BaseModel):
    name: str
    target_delta: List[float]
    confirm_recover: str = ""
    position_tolerance_mm: Optional[float] = 1.5
    orientation_tolerance_rad: Optional[float] = 0.05

class WorldCalibrationCapturePointRequest(BaseModel):
    point_type: str
    side: str

class ObjectNameRequest(BaseModel):
    name: str

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

        self.left_servo_enable = self.create_client(ServoMoveEnable, "/left_jaka_driver/servo_move_enable")
        self.right_servo_enable = self.create_client(ServoMoveEnable, "/right_jaka_driver/servo_move_enable")
        self.left_servo_j = self.create_client(ServoMove, "/left_jaka_driver/servo_j")
        self.right_servo_j = self.create_client(ServoMove, "/right_jaka_driver/servo_j")
        self.left_servo_p = self.create_client(ServoMove, "/left_jaka_driver/servo_p")
        self.right_servo_p = self.create_client(ServoMove, "/right_jaka_driver/servo_p")
        self.left_stop = self.create_client(Empty, f"{cfg['left']['prefix']}/stop_move")
        self.right_stop = self.create_client(Empty, f"{cfg['right']['prefix']}/stop_move")

        self.jog_thread = threading.Thread(target=self.jog_loop, daemon=True)
        self.jog_thread.start()

    def left_joint_cb(self, msg):
        self.left_joint = list(msg.position)[:6]

    def right_joint_cb(self, msg):
        self.right_joint = list(msg.position)[:6]

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

    def _objects_dir(self):
        d = Path(__file__).resolve().parents[1] / "objects"
        d.mkdir(parents=True, exist_ok=True)
        return d


    def _safe_object_filename(self, name):
        raw = str(name or "").strip().replace(" ", "_")
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw).strip("._")

        if not safe:
            raise ValueError("object name is empty")

        if not safe.endswith(".json"):
            safe += ".json"

        if "/" in safe or "\\" in safe or ".." in safe:
            raise ValueError("invalid object name")

        return safe


    def _validate_pose6(self, pose, label="pose"):
        if pose is None:
            return None

        if len(pose) != 6:
            raise ValueError(f"{label} must contain 6 values: X Y Z Rx Ry Rz")

        return [float(x) for x in pose]


    def _angle_mean2(self, a, b):
        import math as _math
        return _math.atan2(_math.sin(a) + _math.sin(b), _math.cos(a) + _math.cos(b))


    def _angle_diff(self, a, b):
        import math as _math
        # shortest angular difference a - b
        return _math.atan2(_math.sin(a - b), _math.cos(a - b))


    def _pose_offset(self, tcp_pose, object_pose):
        # MVP offset:
        # position = simple difference in mm
        # rotation = shortest difference in rad
        return [
            float(tcp_pose[0] - object_pose[0]),
            float(tcp_pose[1] - object_pose[1]),
            float(tcp_pose[2] - object_pose[2]),
            float(self._angle_diff(tcp_pose[3], object_pose[3])),
            float(self._angle_diff(tcp_pose[4], object_pose[4])),
            float(self._angle_diff(tcp_pose[5], object_pose[5])),
        ]

    def _object_pose_delta(self, start_pose, target_pose):
        start_pose = self._validate_pose6(start_pose, "start_object_pose")
        target_pose = self._validate_pose6(target_pose, "target_object_pose")
        return [
            float(target_pose[0] - start_pose[0]),
            float(target_pose[1] - start_pose[1]),
            float(target_pose[2] - start_pose[2]),
            float(self._wrap_angle(target_pose[3] - start_pose[3])),
            float(self._wrap_angle(target_pose[4] - start_pose[4])),
            float(self._wrap_angle(target_pose[5] - start_pose[5])),
        ]

    def _object_motion_delta_metrics(self, object_delta):
        import math
        object_delta = [float(x) for x in object_delta]
        return (
            float(math.sqrt(sum(x * x for x in object_delta[:3]))),
            float(max(abs(x) for x in object_delta[3:6])),
        )

    def validate_object_motion_limits(
        self,
        name,
        target_object_pose,
        max_translation_mm=DEFAULT_OBJECT_MAX_TRANSLATION_MM,
        max_rotation_rad=DEFAULT_OBJECT_MAX_ROTATION_RAD,
    ):
        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {
                    "ok": False,
                    "code": "object_file_not_found",
                    "motion_blocked": True,
                    "error": f"object frame not found: {filename}",
                }

            payload = json.loads(path.read_text(encoding="utf-8"))
            start_object_pose = self._validate_pose6(
                payload.get("current_object_pose") or payload.get("object_pose"),
                "current_object_pose",
            )
            target_object_pose = self._validate_pose6(
                target_object_pose,
                "target_object_pose",
            )
            object_delta = self._object_pose_delta(start_object_pose, target_object_pose)
            translation_delta_mm, rotation_delta_rad = self._object_motion_delta_metrics(object_delta)
            max_translation_mm = float(max_translation_mm)
            max_rotation_rad = float(max_rotation_rad)
            within_limits = (
                translation_delta_mm <= max_translation_mm
                and rotation_delta_rad <= max_rotation_rad
            )

            result = {
                "ok": within_limits,
                "code": (
                    "object_motion_within_limits"
                    if within_limits
                    else "object_motion_limit_exceeded"
                ),
                "start_object_pose": start_object_pose,
                "target_object_pose": target_object_pose,
                "object_delta": object_delta,
                "translation_delta_mm": translation_delta_mm,
                "rotation_delta_rad": rotation_delta_rad,
                "max_translation_mm": max_translation_mm,
                "max_rotation_rad": max_rotation_rad,
            }

            if not within_limits:
                result.update({
                    "motion_blocked": True,
                    "error": "Requested object motion exceeds safe per-command limit.",
                })

            return result

        except Exception as e:
            return {
                "ok": False,
                "code": "object_motion_limit_validation_error",
                "motion_blocked": True,
                "error": str(e),
            }

    def validate_object_motion_request(self, name, target_object_pose):
        motion_limit_check = self.validate_object_motion_limits(name, target_object_pose)
        result = dict(motion_limit_check)
        result["motion_limit_check"] = motion_limit_check

        if not motion_limit_check.get("ok", False):
            result["ok"] = False
            result["motion_blocked"] = True
            result["freshness_check"] = {
                "ok": None,
                "code": "skipped_motion_limit_failed",
                "reason": "Motion limit failed before any live robot state check.",
            }
            return result

        freshness_check = self.validate_current_object_pose_fresh(name)
        result["freshness_check"] = freshness_check

        if not freshness_check.get("ok", False):
            result.update({
                "ok": False,
                "code": freshness_check.get("code", "current_object_pose_not_fresh"),
                "motion_blocked": True,
                "error": freshness_check.get("error", "Current object pose freshness check failed."),
            })
            return result

        result["ok"] = True
        result["code"] = "object_motion_request_valid"
        return result

    def validate_ik_joint_jump(
        self,
        side,
        current_joint,
        target_joint,
        max_joint_jump_rad=MAX_IK_JOINT_JUMP_RAD,
        max_total_norm_rad=MAX_IK_TOTAL_JUMP_NORM_RAD,
    ):
        side = str(side).lower().strip()
        current_joint = [float(x) for x in list(current_joint or [])[:6]]
        target_joint = [float(x) for x in list(target_joint or [])[:6]]

        if len(current_joint) != 6 or len(target_joint) != 6:
            return {
                "ok": False,
                "code": "robot_state_unavailable",
                "motion_blocked": True,
                "side": side,
                "error": "Cannot validate IK joint jump because current robot joint state is unavailable.",
            }

        def angular_diff(a, b):
            return (a - b + math.pi) % (2.0 * math.pi) - math.pi

        joint_delta_rad = [
            float(angular_diff(target_joint[i], current_joint[i]))
            for i in range(6)
        ]
        joint_delta_deg = [float(math.degrees(x)) for x in joint_delta_rad]
        abs_delta = [abs(x) for x in joint_delta_rad]
        max_joint_delta_rad = max(abs_delta)
        max_joint_index = int(abs_delta.index(max_joint_delta_rad))
        total_joint_delta_norm_rad = float(math.sqrt(sum(x * x for x in joint_delta_rad)))
        max_joint_jump_rad = float(max_joint_jump_rad)
        max_total_norm_rad = float(max_total_norm_rad)
        within_limits = (
            max_joint_delta_rad <= max_joint_jump_rad
            and total_joint_delta_norm_rad <= max_total_norm_rad
        )

        result = {
            "ok": within_limits,
            "code": (
                "ik_joint_jump_within_limits"
                if within_limits
                else "ik_joint_jump_limit_exceeded"
            ),
            "side": side,
            "current_joint": current_joint,
            "target_joint": target_joint,
            "joint_delta_rad": joint_delta_rad,
            "joint_delta_deg": joint_delta_deg,
            "max_joint_delta_rad": float(max_joint_delta_rad),
            "max_joint_delta_deg": float(math.degrees(max_joint_delta_rad)),
            "max_joint_index": max_joint_index,
            "total_joint_delta_norm_rad": total_joint_delta_norm_rad,
            "total_joint_delta_norm_deg": float(math.degrees(total_joint_delta_norm_rad)),
            "max_allowed_joint_jump_rad": max_joint_jump_rad,
            "max_allowed_joint_jump_deg": float(math.degrees(max_joint_jump_rad)),
            "max_allowed_total_norm_rad": max_total_norm_rad,
            "max_allowed_total_norm_deg": float(math.degrees(max_total_norm_rad)),
        }

        if not within_limits:
            result.update({
                "motion_blocked": True,
                "error": "IK target requires a large joint jump from the current robot posture.",
            })

        return result

    def _object_target_tcp_poses(self, name, target_object_pose):
        filename = self._safe_object_filename(name)
        path = self._objects_dir() / filename

        if not path.exists():
            return None, {
                "ok": False,
                "code": "object_file_not_found",
                "motion_blocked": True,
                "error": f"object frame not found: {filename}",
            }

        payload = json.loads(path.read_text(encoding="utf-8"))
        captured_object_pose = self._validate_pose6(payload.get("object_pose"), "object_pose")
        start_object_pose = self._validate_pose6(
            payload.get("current_object_pose") or captured_object_pose,
            "current_object_pose",
        )
        target_object_pose = self._validate_pose6(target_object_pose, "target_object_pose")

        left_target_tcp = self._cooperative_tcp_target_from_object(
            captured_object_pose,
            target_object_pose,
            payload.get("left_tcp_at_capture"),
        )
        right_target_tcp = self._cooperative_tcp_target_from_object(
            captured_object_pose,
            target_object_pose,
            payload.get("right_tcp_at_capture"),
        )

        return {
            "filename": filename,
            "payload": payload,
            "captured_object_pose": captured_object_pose,
            "start_object_pose": start_object_pose,
            "target_object_pose": target_object_pose,
            "left_target_tcp": left_target_tcp,
            "right_target_tcp": right_target_tcp,
        }, None

    def validate_object_target_ik_jump(self, name, target_object_pose, side="both"):
        side = (side or "both").lower().strip()
        sides, side_error = self._servo_sides(side)
        if side_error:
            return {
                "ok": False,
                "code": "invalid_side",
                "motion_blocked": True,
                "error": side_error,
            }

        try:
            target_info, error = self._object_target_tcp_poses(name, target_object_pose)
            if error is not None:
                return error

            checks = {}
            target_tcp = {}
            ik_messages = {}

            def get_target_tcp(sd):
                tcp_by_side = target_info.get("target_tcp")
                if isinstance(tcp_by_side, dict) and tcp_by_side.get(sd) is not None:
                    return tcp_by_side.get(sd)
                return target_info.get(f"{sd}_target_tcp")

            for sd in sides:
                current_joint = self.left_joint if sd == "left" else self.right_joint
                if current_joint is None:
                    return {
                        "ok": False,
                        "code": "robot_state_unavailable",
                        "motion_blocked": True,
                        "side": sd,
                        "error": "Cannot validate IK joint jump because current robot joint state is unavailable.",
                    }

                tcp = get_target_tcp(sd)
                if tcp is None:
                    return {
                        "ok": False,
                        "code": "target_tcp_unavailable",
                        "motion_blocked": True,
                        "side": sd,
                        "target_tcp": target_tcp,
                        "error": "Cannot validate IK jump because target TCP pose is unavailable.",
                    }

                target_tcp[sd] = tcp
                target_joint, ik_msg = self.solve_ik_for_tcp_target(
                    sd,
                    tcp,
                    ref_joint=current_joint,
                    timeout=1.0,
                )
                ik_messages[sd] = ik_msg

                if target_joint is None:
                    checks[sd] = {
                        "ok": False,
                        "code": "ik_solution_unavailable",
                        "motion_blocked": True,
                        "side": sd,
                        "target_tcp": tcp,
                        "error": ik_msg,
                    }
                    continue

                checks[sd] = self.validate_ik_joint_jump(sd, current_joint, target_joint)

            ok = all(checks.get(sd, {}).get("ok", False) for sd in sides)
            result = {
                "ok": ok,
                "code": (
                    "ik_joint_jump_within_limits"
                    if ok
                    else "ik_joint_jump_limit_exceeded"
                ),
                "side": side,
                "target_tcp": target_tcp,
                "ik_messages": ik_messages,
                **checks,
            }

            if not ok:
                result.update({
                    "motion_blocked": True,
                    "error": "One or more IK targets require a large joint jump.",
                })

            return result

        except Exception as e:
            return {
                "ok": False,
                "code": "ik_joint_jump_validation_error",
                "motion_blocked": True,
                "error": str(e),
            }

    def _motion_debug_fields(
        self,
        start_pose,
        target_pose,
        object_delta,
        freshness_check,
    ):
        return {
            "start_object_pose": start_pose,
            "target_object_pose": target_pose,
            "object_delta": object_delta,
            "live_object_pose_before_execute": (
                freshness_check.get("live_object_pose")
                if isinstance(freshness_check, dict)
                else None
            ),
            "stored_current_object_pose_before_execute": (
                freshness_check.get("stored_current_object_pose")
                if isinstance(freshness_check, dict)
                else start_pose
            ),
            "freshness_check": freshness_check,
        }


    def estimate_object_pose_from_current_tcps(self, left_tcp, right_tcp):
        # ใช้ midpoint ระหว่าง TCP ซ้าย/ขวาเป็น object center แบบ provisional
        # เหมาะสำหรับเริ่มทำ demo object-frame ก่อน
        return [
            float((left_tcp[0] + right_tcp[0]) / 2.0),
            float((left_tcp[1] + right_tcp[1]) / 2.0),
            float((left_tcp[2] + right_tcp[2]) / 2.0),
            float(self._angle_mean2(left_tcp[3], right_tcp[3])),
            float(self._angle_mean2(left_tcp[4], right_tcp[4])),
            float(self._angle_mean2(left_tcp[5], right_tcp[5])),
        ]


    def capture_object_frame_current(self, name, object_pose=None):
        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            left_tcp = self.get_fk_pose("left")
            right_tcp = self.get_fk_pose("right")

            if left_tcp is None:
                return {"ok": False, "error": "left TCP/FK missing"}
            if right_tcp is None:
                return {"ok": False, "error": "right TCP/FK missing"}

            left_tcp = self._validate_pose6(left_tcp, "left_tcp")
            right_tcp = self._validate_pose6(right_tcp, "right_tcp")

            if object_pose is None:
                object_pose = self.estimate_object_pose_from_current_tcps(left_tcp, right_tcp)
                object_pose_source = "estimated_midpoint_from_current_tcps"
            else:
                object_pose = self._validate_pose6(object_pose, "object_pose")
                object_pose_source = "manual"

            left_offset = self._pose_offset(left_tcp, object_pose)
            right_offset = self._pose_offset(right_tcp, object_pose)

            payload = {
                "name": filename[:-5],
                "file": filename,
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "units": {
                    "position": "mm",
                    "rotation": "rad"
                },
                "object_pose": object_pose,
                "current_object_pose": object_pose,
                "current_pose_updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "object_pose_source": object_pose_source,
                "left_tcp_at_capture": left_tcp,
                "right_tcp_at_capture": right_tcp,
                "left_offset_from_object": left_offset,
                "right_offset_from_object": right_offset,
                "note": (
                    "D32.1/D32.2 MVP capture. Position offsets are relative to the "
                    "captured object pose. Full SE(3) interpolation for rotation will be added in D32.4/D32.5."
                )
            }

            path.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )

            return {
                "ok": True,
                "message": "object frame captured",
                "file": filename,
                "path": str(path),
                **payload,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def _wrap_angle(self, a):
        import math as _math
        return _math.atan2(_math.sin(a), _math.cos(a))


    def _rpy_to_matrix(self, rx, ry, rz):
        # MVP convention: R = Rz * Ry * Rx
        # D34 note: this assumes JAKA pose uses fixed RPY with Rz*Ry*Rx.
        # Verify against the real controller before enabling rigid execute paths.
        import math as _math

        cx, sx = _math.cos(rx), _math.sin(rx)
        cy, sy = _math.cos(ry), _math.sin(ry)
        cz, sz = _math.cos(rz), _math.sin(rz)

        return [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy,     cy * sx,                  cy * cx],
        ]

    def rotation_matrix_to_rpy(self, R):
        # Inverse of _rpy_to_matrix(), using R = Rz * Ry * Rx.
        # D34 caution: near pitch +/-90 deg, RPY is singular/ambiguous.
        sy = -float(R[2][0])
        sy = max(-1.0, min(1.0, sy))
        ry = math.asin(sy)
        cy = math.cos(ry)

        if abs(cy) > 1e-9:
            rx = math.atan2(float(R[2][1]), float(R[2][2]))
            rz = math.atan2(float(R[1][0]), float(R[0][0]))
        else:
            rx = 0.0
            rz = math.atan2(-float(R[0][1]), float(R[1][1]))

        return [
            float(self._wrap_angle(rx)),
            float(self._wrap_angle(ry)),
            float(self._wrap_angle(rz)),
        ]

    def pose6_to_transform(self, pose6):
        pose6 = self._validate_pose6(pose6, "pose6")
        R = self._rpy_to_matrix(pose6[3], pose6[4], pose6[5])
        return [
            [float(R[0][0]), float(R[0][1]), float(R[0][2]), float(pose6[0])],
            [float(R[1][0]), float(R[1][1]), float(R[1][2]), float(pose6[1])],
            [float(R[2][0]), float(R[2][1]), float(R[2][2]), float(pose6[2])],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def transform_to_pose6(self, T):
        R = [
            [float(T[0][0]), float(T[0][1]), float(T[0][2])],
            [float(T[1][0]), float(T[1][1]), float(T[1][2])],
            [float(T[2][0]), float(T[2][1]), float(T[2][2])],
        ]
        rx, ry, rz = self.rotation_matrix_to_rpy(R)
        return [
            float(T[0][3]),
            float(T[1][3]),
            float(T[2][3]),
            rx,
            ry,
            rz,
        ]

    def invert_transform(self, T):
        R = [
            [float(T[0][0]), float(T[0][1]), float(T[0][2])],
            [float(T[1][0]), float(T[1][1]), float(T[1][2])],
            [float(T[2][0]), float(T[2][1]), float(T[2][2])],
        ]
        Rt = self._mat_T(R)
        p = [float(T[0][3]), float(T[1][3]), float(T[2][3])]
        inv_p = self._mat_vec(Rt, [-p[0], -p[1], -p[2]])
        return [
            [Rt[0][0], Rt[0][1], Rt[0][2], inv_p[0]],
            [Rt[1][0], Rt[1][1], Rt[1][2], inv_p[1]],
            [Rt[2][0], Rt[2][1], Rt[2][2], inv_p[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def compose_transform(self, A, B):
        out = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                out[i][j] = float(sum(float(A[i][k]) * float(B[k][j]) for k in range(4)))
        return out

    def transform_point(self, T, p):
        return [
            float(T[0][0] * p[0] + T[0][1] * p[1] + T[0][2] * p[2] + T[0][3]),
            float(T[1][0] * p[0] + T[1][1] * p[1] + T[1][2] * p[2] + T[1][3]),
            float(T[2][0] * p[0] + T[2][1] * p[1] + T[2][2] * p[2] + T[2][3]),
        ]


    def _mat_vec(self, R, v):
        return [
            R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
        ]


    def _mat_T(self, R):
        return [
            [R[0][0], R[1][0], R[2][0]],
            [R[0][1], R[1][1], R[2][1]],
            [R[0][2], R[1][2], R[2][2]],
        ]


    def _cooperative_tcp_target_from_object(self, old_object_pose, new_object_pose, tcp_at_capture):
        old_object_pose = self._validate_pose6(old_object_pose, "old_object_pose")
        new_object_pose = self._validate_pose6(new_object_pose, "new_object_pose")
        tcp_at_capture = self._validate_pose6(tcp_at_capture, "tcp_at_capture")

        old_R = self._rpy_to_matrix(
            old_object_pose[3],
            old_object_pose[4],
            old_object_pose[5],
        )

        new_R = self._rpy_to_matrix(
            new_object_pose[3],
            new_object_pose[4],
            new_object_pose[5],
        )

        # world offset ตอน capture
        world_offset = [
            tcp_at_capture[0] - old_object_pose[0],
            tcp_at_capture[1] - old_object_pose[1],
            tcp_at_capture[2] - old_object_pose[2],
        ]

        # แปลง world offset กลับเป็น local offset ของ object frame
        local_offset = self._mat_vec(self._mat_T(old_R), world_offset)

        # หมุน local offset ตาม object pose ใหม่
        new_world_offset = self._mat_vec(new_R, local_offset)

        # rotation offset แบบ MVP: ใช้ผลต่าง RPY จากตอน capture
        rot_offset = [
            self._wrap_angle(tcp_at_capture[3] - old_object_pose[3]),
            self._wrap_angle(tcp_at_capture[4] - old_object_pose[4]),
            self._wrap_angle(tcp_at_capture[5] - old_object_pose[5]),
        ]

        return [
            float(new_object_pose[0] + new_world_offset[0]),
            float(new_object_pose[1] + new_world_offset[1]),
            float(new_object_pose[2] + new_world_offset[2]),
            float(self._wrap_angle(new_object_pose[3] + rot_offset[0])),
            float(self._wrap_angle(new_object_pose[4] + rot_offset[1])),
            float(self._wrap_angle(new_object_pose[5] + rot_offset[2])),
        ]

    def ensure_rigid_grasp_transforms(self, obj):
        if (
            obj.get("left_grasp_transform_in_object") is not None
            and obj.get("right_grasp_transform_in_object") is not None
        ):
            return obj

        object_pose = self._validate_pose6(obj.get("object_pose"), "object_pose")
        left_tcp = self._validate_pose6(obj.get("left_tcp_at_capture"), "left_tcp_at_capture")
        right_tcp = self._validate_pose6(obj.get("right_tcp_at_capture"), "right_tcp_at_capture")

        T_world_object = self.pose6_to_transform(object_pose)
        T_object_world = self.invert_transform(T_world_object)
        T_world_left = self.pose6_to_transform(left_tcp)
        T_world_right = self.pose6_to_transform(right_tcp)

        obj = dict(obj)
        obj["left_grasp_transform_in_object"] = self.compose_transform(T_object_world, T_world_left)
        obj["right_grasp_transform_in_object"] = self.compose_transform(T_object_world, T_world_right)
        obj["rigid_transform_schema_version"] = 1
        return obj

    def _cooperative_tcp_target_from_object_rigid(self, obj, target_object_pose):
        obj = self.ensure_rigid_grasp_transforms(obj)
        target_object_pose = self._validate_pose6(target_object_pose, "target_object_pose")
        T_world_object_target = self.pose6_to_transform(target_object_pose)

        T_world_left_target = self.compose_transform(
            T_world_object_target,
            obj["left_grasp_transform_in_object"],
        )
        T_world_right_target = self.compose_transform(
            T_world_object_target,
            obj["right_grasp_transform_in_object"],
        )

        return {
            "left_target_tcp_rigid": self.transform_to_pose6(T_world_left_target),
            "right_target_tcp_rigid": self.transform_to_pose6(T_world_right_target),
            "rigid_schema_version": int(obj.get("rigid_transform_schema_version", 1)),
        }

    def _pose6_difference_metrics(self, old_pose, new_pose):
        old_pose = self._validate_pose6(old_pose, "old_pose")
        new_pose = self._validate_pose6(new_pose, "new_pose")

        dp = [
            float(new_pose[0] - old_pose[0]),
            float(new_pose[1] - old_pose[1]),
            float(new_pose[2] - old_pose[2]),
        ]
        translation_mm = float(math.sqrt(sum(x * x for x in dp)))

        # Use true SO(3) geodesic rotation distance instead of comparing RPY/Euler
        # components directly. Direct RPY subtraction can report false jumps near
        # gimbal-lock-like poses or around +/-pi wrapping.
        T_old = self.pose6_to_transform(old_pose)
        T_new = self.pose6_to_transform(new_pose)

        R_old = [[float(T_old[r][c]) for c in range(3)] for r in range(3)]
        R_new = [[float(T_new[r][c]) for c in range(3)] for r in range(3)]

        # R_delta = R_old^T * R_new
        R_delta = [
            [
                sum(R_old[k][i] * R_new[k][j] for k in range(3))
                for j in range(3)
            ]
            for i in range(3)
        ]

        trace = R_delta[0][0] + R_delta[1][1] + R_delta[2][2]
        cos_angle = (trace - 1.0) / 2.0
        cos_angle = max(-1.0, min(1.0, cos_angle))
        rotation_rad = float(math.acos(cos_angle))

        return translation_mm, rotation_rad

    def preview_object_targets_rigid_compare(self, name, target_object_pose):
        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            obj = json.loads(path.read_text(encoding="utf-8"))
            captured_object_pose = self._validate_pose6(obj.get("object_pose"), "object_pose")
            start_object_pose = self._validate_pose6(
                obj.get("current_object_pose") or captured_object_pose,
                "current_object_pose",
            )
            target_object_pose = self._validate_pose6(target_object_pose, "target_object_pose")

            old_left = self._cooperative_tcp_target_from_object(
                captured_object_pose,
                target_object_pose,
                obj.get("left_tcp_at_capture"),
            )
            old_right = self._cooperative_tcp_target_from_object(
                captured_object_pose,
                target_object_pose,
                obj.get("right_tcp_at_capture"),
            )
            rigid = self._cooperative_tcp_target_from_object_rigid(obj, target_object_pose)
            rigid_left = rigid["left_target_tcp_rigid"]
            rigid_right = rigid["right_target_tcp_rigid"]
            left_pos_delta, left_rot_delta = self._pose6_difference_metrics(old_left, rigid_left)
            right_pos_delta, right_rot_delta = self._pose6_difference_metrics(old_right, rigid_right)
            object_delta = self._object_pose_delta(start_object_pose, target_object_pose)
            _, object_rotation_delta = self._object_motion_delta_metrics(object_delta)

            warnings = [
                "D34 dry-run only. Execute paths still use the legacy target TCP calculation.",
                "JAKA pose convention is assumed to be RPY with R = Rz * Ry * Rx; verify before rigid execute.",
            ]
            if object_rotation_delta > 0.05:
                warnings.append("Target object rotation is large enough that legacy Euler addition may diverge from rigid SE(3).")
            if abs(math.cos(target_object_pose[4])) < 0.10 or abs(math.cos(captured_object_pose[4])) < 0.10:
                warnings.append("Object pitch is near RPY gimbal lock; transform_to_pose6 may be ambiguous.")

            return {
                "ok": True,
                "name": filename[:-5],
                "target_object_pose": target_object_pose,
                "object_delta": object_delta,
                "old": {
                    "left_target_tcp": old_left,
                    "right_target_tcp": old_right,
                },
                "rigid": {
                    "left_target_tcp": rigid_left,
                    "right_target_tcp": rigid_right,
                    "rigid_schema_version": rigid["rigid_schema_version"],
                },
                "difference": {
                    "left_position_delta_mm": left_pos_delta,
                    "left_rotation_delta_rad": left_rot_delta,
                    "right_position_delta_mm": right_pos_delta,
                    "right_rotation_delta_rad": right_rot_delta,
                },
                "warnings": warnings,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def preview_object_targets(self, name, target_object_pose):
        try:
            motion_limit_check = self.validate_object_motion_limits(name, target_object_pose)
            if motion_limit_check.get("ok", False):
                try:
                    freshness_check = self.validate_current_object_pose_fresh(name)
                except Exception as e:
                    freshness_check = {
                        "ok": False,
                        "code": "freshness_check_unavailable",
                        "error": str(e),
                    }
            else:
                freshness_check = {
                    "ok": None,
                    "code": "skipped_motion_limit_failed",
                    "reason": "Motion limit failed before any live robot state check.",
                }
            if motion_limit_check.get("ok", False):
                ik_joint_jump_check = self.validate_object_target_ik_jump(
                    name,
                    target_object_pose,
                    side="both",
                )
            else:
                ik_joint_jump_check = {
                    "ok": None,
                    "code": "skipped_motion_limit_failed",
                    "reason": "Motion limit failed before IK jump validation.",
                }

            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))

            captured_object_pose = payload.get("object_pose")
            start_object_pose = payload.get("current_object_pose") or captured_object_pose
            left_tcp_at_capture = payload.get("left_tcp_at_capture")
            right_tcp_at_capture = payload.get("right_tcp_at_capture")

            captured_object_pose = self._validate_pose6(
                captured_object_pose,
                "object_pose",
            )
            start_object_pose = self._validate_pose6(
                start_object_pose,
                "current_object_pose",
            )
            target_object_pose = self._validate_pose6(target_object_pose, "target_object_pose")

            left_target_tcp = self._cooperative_tcp_target_from_object(
                captured_object_pose,
                target_object_pose,
                left_tcp_at_capture,
            )

            right_target_tcp = self._cooperative_tcp_target_from_object(
                captured_object_pose,
                target_object_pose,
                right_tcp_at_capture,
            )

            object_delta = self._object_pose_delta(
                start_object_pose,
                target_object_pose,
            )
            captured_relative_object_delta = self._object_pose_delta(
                captured_object_pose,
                target_object_pose,
            )

            return {
                "ok": True,
                "message": "cooperative TCP targets preview generated",
                "object": filename[:-5],
                "method": "object_frame_offset_preview",
                "units": {
                    "position": "mm",
                    "rotation": "rad",
                },
                "old_object_pose": captured_object_pose,
                "captured_object_pose": captured_object_pose,
                "start_object_pose": start_object_pose,
                "target_object_pose": target_object_pose,
                "object_delta": object_delta,
                "captured_relative_object_delta": captured_relative_object_delta,
                "motion_limit_check": motion_limit_check,
                "freshness_check": freshness_check,
                "ik_joint_jump_check": ik_joint_jump_check,
                "left_target_tcp": left_target_tcp,
                "right_target_tcp": right_target_tcp,
                "note": (
                    "Preview only. No robot motion was sent. "
                    "D32.4 will execute these targets through IK -> joint_move with interpolation."
                ),
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}



    def solve_ik_for_tcp_target(self, side_name, tcp_pose, ref_joint=None, timeout=1.0):
        side_name = str(side_name).lower().strip()

        if side_name == "left":
            client = self.left_get_ik
            if ref_joint is None:
                ref_joint = self.left_joint
        elif side_name == "right":
            client = self.right_get_ik
            if ref_joint is None:
                ref_joint = self.right_joint
        else:
            return None, f"invalid IK side: {side_name}"

        if ref_joint is None:
            return None, f"{side_name} joint feedback missing"

        if not client.service_is_ready():
            return None, f"{side_name} get_ik service not ready"

        req = GetIK.Request()
        req.ref_joint = [float(x) for x in ref_joint]
        req.cartesian_pose = [float(x) for x in tcp_pose]

        resp = self.wait_future_result(client.call_async(req), timeout=timeout)
        if resp is None:
            return None, f"{side_name} get_ik timeout/no response"

        joint = list(getattr(resp, "joint", []) or [])
        message = str(getattr(resp, "message", "") or "")

        if len(joint) < 6:
            return None, f"{side_name} IK failed: {message}"

        return [float(x) for x in joint[:6]], message


    def execute_object_move(self, name, target_object_pose, side="both", vel=None, acc=None):
        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        filename = self._safe_object_filename(name)
        path = self._objects_dir() / filename
        if not path.exists():
            return {"ok": False, "error": f"object frame not found: {filename}"}
        payload = json.loads(path.read_text(encoding="utf-8"))

        motion_limit_check = self.validate_object_motion_limits(name, target_object_pose)
        if not motion_limit_check.get("ok", False):
            return {
                **motion_limit_check,
                "motion_limit_check": motion_limit_check,
            }

        # D32.5C-S1 safety guard:
        # Do not execute object motion when stored current_object_pose differs from live TCP midpoint.
        freshness_check = self.validate_current_object_pose_fresh(name)
        if not freshness_check.get("ok", False):
            return freshness_check

        ik_joint_jump_check = self.validate_object_target_ik_jump(name, target_object_pose, side)
        if not ik_joint_jump_check.get("ok", False):
            return ik_joint_jump_check

        ok, msg = self.safe_state_ok(side)
        if not ok:
            return {"ok": False, "error": msg}

        start_object_pose = motion_limit_check["start_object_pose"]
        target_object_pose = motion_limit_check["target_object_pose"]
        object_delta = motion_limit_check["object_delta"]
        debug_fields = self._motion_debug_fields(
            start_object_pose,
            target_object_pose,
            object_delta,
            freshness_check,
        )
        debug_fields["motion_limit_check"] = motion_limit_check
        debug_fields["ik_joint_jump_check"] = ik_joint_jump_check

        preview = self.preview_object_targets(name, target_object_pose)
        if not preview.get("ok", False):
            return preview

        translation_delta = motion_limit_check["translation_delta_mm"]
        rotation_delta = motion_limit_check["rotation_delta_rad"]

        left_target_joint = None
        right_target_joint = None
        ik_messages = {}

        if side in ("left", "both"):
            left_target_tcp = preview.get("left_target_tcp")
            left_target_joint, msg = self.solve_ik_for_tcp_target(
                "left",
                left_target_tcp,
                ref_joint=self.left_joint,
                timeout=1.0,
            )
            ik_messages["left"] = msg

            if left_target_joint is None:
                return {"ok": False, "error": msg, "preview": preview}

        if side in ("right", "both"):
            right_target_tcp = preview.get("right_target_tcp")
            right_target_joint, msg = self.solve_ik_for_tcp_target(
                "right",
                right_target_tcp,
                ref_joint=self.right_joint,
                timeout=1.0,
            )
            ik_messages["right"] = msg

            if right_target_joint is None:
                return {"ok": False, "error": msg, "preview": preview}

        result = self.move_both_joint(
            left_target=left_target_joint,
            right_target=right_target_joint,
            side=side,
            vel=vel,
            acc=acc,
        )

        result["method"] = "execute_object_move_ik_to_joint_move"
        result["object"] = name
        result.update(debug_fields)
        result["translation_delta"] = translation_delta
        result["rotation_delta"] = rotation_delta
        result["ik_messages"] = ik_messages
        result["preview"] = preview
        result["note"] = (
            "Object move executed by preview targets -> IK -> synchronized joint_move. "
            "No linear_move was used."
        )

        return result



    def _servo_sides(self, side):
        side = (side or "both").lower().strip()
        if side == "both":
            return ["left", "right"], None
        if side in ("left", "right"):
            return [side], None
        return [], f"invalid side: {side}"


    def set_servo_enable(self, side="both", enable=True):
        sides, err = self._servo_sides(side)
        if err:
            return {"ok": False, "error": err}

        results = {}

        for sd in sides:
            client = self.left_servo_enable if sd == "left" else self.right_servo_enable

            if not client.service_is_ready():
                results[sd] = {
                    "ok": False,
                    "error": f"{sd} servo_move_enable service not ready",
                }
                continue

            req = ServoMoveEnable.Request()
            req.enable = bool(enable)

            resp = self.wait_future_result(client.call_async(req), timeout=2.0)

            if resp is None:
                results[sd] = {
                    "ok": False,
                    "error": f"{sd} servo_move_enable timeout/no response",
                }
            else:
                ret = int(getattr(resp, "ret", -999))
                msg = str(getattr(resp, "message", "") or "")
                results[sd] = {
                    "ok": (ret == 1 and not msg.lower().startswith("error occurred")),
                    "ret": ret,
                    "message": msg,
                }

        overall_ok = all(v.get("ok", False) for v in results.values())

        return {
            "ok": overall_ok,
            "side": side,
            "enable": bool(enable),
            "results": results,
            "message": "servo mode command sent",
        }


    def call_servo_j_increment(self, side="left", pose=None, speed=None):
        sides, err = self._servo_sides(side)
        if err:
            return {"ok": False, "error": err}

        if pose is None:
            pose = [0.0] * 6

        pose = [float(x) for x in pose]

        if len(pose) != 6:
            return {"ok": False, "error": "servo_j pose must have 6 values"}

        if speed is None:
            speed = [0.0] * 6

        speed = [float(x) for x in speed]

        if len(speed) != 6:
            speed = [0.0] * 6

        results = {}

        for sd in sides:
            client = self.left_servo_j if sd == "left" else self.right_servo_j

            if not client.service_is_ready():
                results[sd] = {
                    "ok": False,
                    "error": f"{sd} servo_j service not ready",
                }
                continue

            req = ServoMove.Request()
            req.pose = pose
            req.speed = speed

            resp = self.wait_future_result(client.call_async(req), timeout=1.0)

            if resp is None:
                results[sd] = {
                    "ok": False,
                    "error": f"{sd} servo_j timeout/no response",
                }
            else:
                ret = int(getattr(resp, "ret", -999))
                msg = str(getattr(resp, "message", "") or "")
                results[sd] = {
                    "ok": (ret == 1 and not msg.lower().startswith("error occurred")),
                    "ret": ret,
                    "message": msg,
                }

        overall_ok = all(v.get("ok", False) for v in results.values())

        return {
            "ok": overall_ok,
            "side": side,
            "pose_increment": pose,
            "speed": speed,
            "results": results,
        }


    def servo_tiny_test(self, side="left", joint_index=5, delta=0.0005, repeats=1, interval=0.05, auto_enable=True, auto_disable=True):
        side = (side or "left").lower().strip()

        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        try:
            joint_index = int(joint_index)
            delta = float(delta)
            repeats = int(repeats)
            interval = float(interval)
        except Exception as e:
            return {"ok": False, "error": f"invalid servo tiny test parameter: {e}"}

        if joint_index < 0 or joint_index > 5:
            return {"ok": False, "error": "joint_index must be 0..5"}

        if abs(delta) > 0.005:
            return {
                "ok": False,
                "error": "delta too large for tiny test; max abs(delta) is 0.005 rad",
            }

        if repeats < 1:
            repeats = 1

        if repeats > 20:
            return {"ok": False, "error": "repeats too large; max is 20"}

        interval = max(0.0, min(interval, 1.0))

        ok, msg = self.safe_state_ok(side)
        if not ok:
            return {"ok": False, "error": msg}

        enable_result = None
        disable_result = None
        step_results = []

        try:
            if auto_enable:
                enable_result = self.set_servo_enable(side, True)
                if not enable_result.get("ok", False):
                    return {
                        "ok": False,
                        "error": "failed to enable servo mode",
                        "enable_result": enable_result,
                    }

                time.sleep(0.10)

            for i in range(repeats):
                pose = [0.0] * 6
                pose[joint_index] = delta

                r = self.call_servo_j_increment(side, pose=pose, speed=[0.0] * 6)
                step_results.append(r)

                if not r.get("ok", False):
                    return {
                        "ok": False,
                        "error": "servo_j tiny test failed",
                        "failed_step": i + 1,
                        "step_result": r,
                        "enable_result": enable_result,
                    }

                time.sleep(interval)

            if auto_disable:
                time.sleep(0.05)
                disable_result = self.set_servo_enable(side, False)

            return {
                "ok": True,
                "message": "servo tiny test completed",
                "side": side,
                "joint_index": joint_index,
                "delta": delta,
                "repeats": repeats,
                "interval": interval,
                "auto_enable": bool(auto_enable),
                "auto_disable": bool(auto_disable),
                "enable_result": enable_result,
                "step_results": step_results,
                "disable_result": disable_result,
                "note": "servo_j uses incremental joint motion in the current driver",
            }

        except Exception as e:
            if auto_disable:
                try:
                    disable_result = self.set_servo_enable(side, False)
                except Exception:
                    pass

            return {
                "ok": False,
                "error": str(e),
                "enable_result": enable_result,
                "step_results": step_results,
                "disable_result": disable_result,
            }



    def _servo_success(self, resp):
        if resp is None:
            return False, -999, "timeout/no response"
        ret = int(getattr(resp, "ret", -999))
        msg = str(getattr(resp, "message", "") or "")
        ok = (ret == 1 and not msg.lower().startswith("error occurred"))
        return ok, ret, msg


    def _call_servo_j_one(self, side, pose_increment, speed=None, timeout=0.5):
        side = str(side).lower().strip()
        if side == "left":
            client = self.left_servo_j
        elif side == "right":
            client = self.right_servo_j
        else:
            return {"ok": False, "error": f"invalid servo side: {side}"}

        if not client.service_is_ready():
            return {"ok": False, "error": f"{side} servo_j service not ready"}

        if speed is None:
            speed = [0.0] * 6

        req = ServoMove.Request()
        req.pose = [float(x) for x in pose_increment]
        req.speed = [float(x) for x in speed]

        resp = self.wait_future_result(client.call_async(req), timeout=timeout)
        ok, ret, msg = self._servo_success(resp)

        return {
            "ok": ok,
            "side": side,
            "ret": ret,
            "message": msg,
            "pose_increment": req.pose,
        }



    # ------------------------------------------------------------------
    # D32.5C-S1: Live TCP midpoint sync + stale current_object_pose guard
    # ------------------------------------------------------------------
    def _angle_wrap_pi(self, a):
        import math
        return (float(a) + math.pi) % (2.0 * math.pi) - math.pi

    def _angle_mean_safe(self, a, b):
        import math
        return math.atan2(math.sin(float(a)) + math.sin(float(b)),
                          math.cos(float(a)) + math.cos(float(b)))

    def _angle_diff_abs(self, a, b):
        return abs(self._angle_wrap_pi(float(a) - float(b)))

    def _normalize_pose6(self, pose, label="pose"):
        if pose is None:
            raise ValueError(f"{label} is missing")
        if isinstance(pose, dict):
            for key in ("cartesian_pose", "tcp", "tcp_pose", "pose"):
                if key in pose:
                    pose = pose[key]
                    break
        pose = list(pose)
        if len(pose) < 6:
            raise ValueError(f"{label} must have 6 values, got {pose}")
        return [float(x) for x in pose[:6]]

    def _get_live_tcp_pose_for_object_guard(self, side):
        """
        D32.5C-S3:
        Read the same live TCP source used by /api/status / Live Position first.
        get_fk_pose() is only fallback because FK may use a different TCP/tool/reference
        and was observed to differ by about +20 mm in Z.
        """
        side = side.lower().strip()

        def _try_pose(cand, label):
            if cand is None:
                return None
            try:
                return self._normalize_pose6(cand, label)
            except Exception:
                return None

        # 1) Preferred: status() output, same source as Web UI Live Position.
        try:
            st = self.status()
        except Exception:
            st = None

        candidates = []

        if isinstance(st, dict):
            # Common direct fields.
            candidates.extend([
                st.get(f"{side}_tcp"),
                st.get(f"{side}_tcp_pose"),
                st.get(f"{side}_cartesian_pose"),
                st.get(f"{side}_pose"),
            ])

            # Common nested fields.
            for key in [side, f"{side}_robot", f"{side}_jaka", f"{side}_state"]:
                sd = st.get(key)
                if isinstance(sd, dict):
                    candidates.extend([
                        sd.get("tcp"),
                        sd.get("tcp_pose"),
                        sd.get("cartesian_pose"),
                        sd.get("pose"),
                        sd.get("tcp_position"),
                    ])

            # Deep search fallback inside status:
            # choose pose-like lists under a path containing side + tcp/cartesian.
            deep = []

            def walk(obj, path=""):
                if isinstance(obj, dict):
                    for k, v in obj.items():
                        pp = f"{path}.{k}" if path else str(k)
                        walk(v, pp)
                elif isinstance(obj, list):
                    if len(obj) >= 6 and all(isinstance(x, (int, float)) for x in obj[:6]):
                        low = path.lower()
                        if side in low and any(word in low for word in ["tcp", "cartesian", "pose"]):
                            deep.append((path, obj))
                    for i, v in enumerate(obj):
                        walk(v, f"{path}[{i}]")

            walk(st)

            for path, obj in deep:
                candidates.append(obj)

        for idx, cand in enumerate(candidates):
            pose = _try_pose(cand, f"{side} live tcp from status candidate {idx}")
            if pose is not None:
                return pose

        # 2) Fallback only: FK helper. This may not match displayed TCP if tool/reference differs.
        try:
            pose = self.get_fk_pose(side)
            return self._normalize_pose6(pose, f"{side} live tcp fallback from get_fk_pose")
        except Exception as e:
            raise RuntimeError(
                f"cannot read live TCP pose for {side}; status had no usable TCP pose "
                f"and get_fk_pose failed: {e}"
            )


    def _live_object_pose_from_tcp_midpoint(self):
        """
        Estimate current object pose from live left/right TCP.
        This matches the current object-frame prototype assumption:
        object center = midpoint of two TCPs.
        """
        import time

        left = self._get_live_tcp_pose_for_object_guard("left")
        right = self._get_live_tcp_pose_for_object_guard("right")

        live = [
            0.5 * (left[0] + right[0]),
            0.5 * (left[1] + right[1]),
            0.5 * (left[2] + right[2]),
            self._angle_mean_safe(left[3], right[3]),
            self._angle_mean_safe(left[4], right[4]),
            self._angle_mean_safe(left[5], right[5]),
        ]

        return {
            "ok": True,
            "live_object_pose": [float(x) for x in live],
            "left_tcp_live": left,
            "right_tcp_live": right,
            "computed_at": time.time(),
            "source": "live_tcp_midpoint",
        }

    def _object_file_path_s1(self, name):
        from pathlib import Path
        safe = Path(str(name)).name.strip()
        if not safe:
            raise ValueError("object name is empty")
        if safe.endswith(".json"):
            filename = safe
        else:
            filename = safe + ".json"

        # Use existing self.objects_dir when available, otherwise default to ../objects
        obj_dir = getattr(self, "objects_dir", None)
        if obj_dir is None:
            obj_dir = Path(__file__).resolve().parents[1] / "objects"
        else:
            obj_dir = Path(obj_dir)

        return obj_dir / filename

    def _load_object_record_s1(self, name):
        import json
        path = self._object_file_path_s1(name)
        if not path.exists():
            return None, path
        return json.loads(path.read_text()), path

    def _save_object_record_s1(self, path, data):
        import json
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))

    def get_live_calibration_point(self, source):
        source = str(source or "").lower().strip()
        if source not in ("left_tcp", "right_tcp", "midpoint"):
            raise ValueError("source must be one of: left_tcp, right_tcp, midpoint")

        if source == "left_tcp":
            return self._get_live_tcp_pose_for_object_guard("left")[:3]
        if source == "right_tcp":
            return self._get_live_tcp_pose_for_object_guard("right")[:3]

        left = self._get_live_tcp_pose_for_object_guard("left")
        right = self._get_live_tcp_pose_for_object_guard("right")
        return [
            float((left[0] + right[0]) / 2.0),
            float((left[1] + right[1]) / 2.0),
            float((left[2] + right[2]) / 2.0),
        ]

    def get_live_tcp_for_side(self, side):
        side = str(side or "").lower().strip()
        if side not in ("left", "right"):
            raise ValueError("side must be one of: left, right")
        return self._get_live_tcp_pose_for_object_guard(side)

    def _default_calibration_points(self):
        return {
            "origin": None,
            "x_axis": None,
            "y_helper": None,
        }

    def _ensure_object_frame_calibration(self, data):
        cal = data.get("object_frame_calibration")
        if not isinstance(cal, dict):
            cal = {}
        points = cal.get("points")
        if not isinstance(points, dict):
            points = {}

        default_points = self._default_calibration_points()
        default_points.update({
            key: points.get(key)
            for key in default_points
            if points.get(key) is not None
        })
        cal["mode"] = "manual_3_point"
        cal["points"] = default_points
        cal.setdefault("computed", None)
        return cal

    def capture_object_calibration_point(self, name, point_type, source="left_tcp"):
        point_type = str(point_type or "").lower().strip()
        if point_type not in ("origin", "x_axis", "y_helper"):
            return {
                "ok": False,
                "code": "invalid_point_type",
                "error": "point_type must be one of: origin, x_axis, y_helper",
            }

        data, path = self._load_object_record_s1(name)
        if data is None:
            return {"ok": False, "code": "object_file_not_found", "error": f"object file not found: {path}"}

        try:
            point = [float(x) for x in self.get_live_calibration_point(source)]
        except Exception as e:
            return {
                "ok": False,
                "code": "calibration_point_unavailable",
                "error": str(e),
            }

        cal = self._ensure_object_frame_calibration(data)
        cal["points"][point_type] = point
        cal["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        data["object_frame_calibration"] = cal
        self._save_object_record_s1(path, data)

        return {
            "ok": True,
            "name": Path(path).stem,
            "point_type": point_type,
            "source": source,
            "point": point,
            "calibration_points": cal["points"],
        }

    def _vec_sub3(self, a, b):
        return [float(a[i] - b[i]) for i in range(3)]

    def _vec_add3(self, a, b):
        return [float(a[i] + b[i]) for i in range(3)]

    def _vec_scale3(self, v, s):
        return [float(v[i] * s) for i in range(3)]

    def _vec_dot3(self, a, b):
        return float(sum(float(a[i]) * float(b[i]) for i in range(3)))

    def _vec_cross3(self, a, b):
        return [
            float(a[1] * b[2] - a[2] * b[1]),
            float(a[2] * b[0] - a[0] * b[2]),
            float(a[0] * b[1] - a[1] * b[0]),
        ]

    def _vec_norm3(self, v):
        return float(math.sqrt(sum(float(x) * float(x) for x in v)))

    def _vec_normalize3(self, v, label):
        n = self._vec_norm3(v)
        if n < 1e-9:
            raise ValueError(f"{label} vector is too small")
        return [float(x / n) for x in v], n

    def _identity_transform4(self):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _world_calibration_path(self):
        d = Path(__file__).resolve().parents[1] / "config"
        d.mkdir(parents=True, exist_ok=True)
        return d / "world_frame_calibration.json"

    def _default_world_frame_calibration(self):
        return {
            "enabled": False,
            "world_frame": "left_base",
            "points": {
                "origin": {"left": None, "right": None},
                "x_axis": {"left": None, "right": None},
                "y_helper": {"left": None, "right": None},
            },
            "T_world_left_base": self._identity_transform4(),
            "T_world_right_base": None,
            "T_left_base_right_base": None,
            "computed_at": None,
            "notes": "",
        }

    def _load_world_frame_calibration(self):
        path = self._world_calibration_path()
        if path.exists():
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                data = {}
        else:
            data = {}

        cal = self._default_world_frame_calibration()
        if isinstance(data, dict):
            cal.update({k: v for k, v in data.items() if k != "points"})
            points = data.get("points")
            if isinstance(points, dict):
                for point_type in ("origin", "x_axis", "y_helper"):
                    src = points.get(point_type)
                    if isinstance(src, dict):
                        cal["points"][point_type]["left"] = src.get("left")
                        cal["points"][point_type]["right"] = src.get("right")
        return cal

    def _save_world_frame_calibration(self, cal):
        path = self._world_calibration_path()
        path.write_text(json.dumps(cal, indent=2, ensure_ascii=False), encoding="utf-8")
        return path

    def _world_calibration_has_all_points(self, cal):
        points = cal.get("points", {})
        for point_type in ("origin", "x_axis", "y_helper"):
            item = points.get(point_type, {})
            if item.get("left") is None or item.get("right") is None:
                return False
        return True

    def build_frame_from_3_points(self, origin, x_axis, y_helper):
        origin = [float(x) for x in origin[:3]]
        x_point = [float(x) for x in x_axis[:3]]
        y_point = [float(x) for x in y_helper[:3]]

        x_vec = self._vec_sub3(x_point, origin)
        y_temp = self._vec_sub3(y_point, origin)
        x_unit, x_dist = self._vec_normalize3(x_vec, "origin to x_axis")
        _, y_dist = self._vec_normalize3(y_temp, "origin to y_helper")
        if x_dist <= 20.0:
            raise ValueError("origin -> x_axis distance must be > 20 mm")
        if y_dist <= 20.0:
            raise ValueError("origin -> y_helper distance must be > 20 mm")

        z_raw = self._vec_cross3(x_unit, y_temp)
        z_unit, cross_norm = self._vec_normalize3(z_raw, "calibration z_axis")
        if cross_norm <= 1e-6:
            raise ValueError("origin, x_axis, and y_helper points are nearly collinear")
        y_unit = self._vec_cross3(z_unit, x_unit)
        y_unit, _ = self._vec_normalize3(y_unit, "calibration y_axis")

        dots = {
            "dot_xy": self._vec_dot3(x_unit, y_unit),
            "dot_yz": self._vec_dot3(y_unit, z_unit),
            "dot_zx": self._vec_dot3(z_unit, x_unit),
        }
        norms = {
            "x": self._vec_norm3(x_unit),
            "y": self._vec_norm3(y_unit),
            "z": self._vec_norm3(z_unit),
        }
        max_abs_dot = max(abs(v) for v in dots.values())
        max_norm_error = max(abs(v - 1.0) for v in norms.values())
        if max_abs_dot > 1e-5:
            raise ValueError("calibration axes are not orthogonal enough")
        if max_norm_error > 1e-5:
            raise ValueError("calibration axes are not normalized enough")

        T = [
            [x_unit[0], y_unit[0], z_unit[0], origin[0]],
            [x_unit[1], y_unit[1], z_unit[1], origin[1]],
            [x_unit[2], y_unit[2], z_unit[2], origin[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]
        return {
            "transform": T,
            "origin": origin,
            "axes": {"x": x_unit, "y": y_unit, "z": z_unit},
            "distances_mm": {
                "origin_to_x_axis": x_dist,
                "origin_to_y_helper": y_dist,
            },
            "dot_products": dots,
            "axis_norms": norms,
            "max_abs_dot": max_abs_dot,
            "max_norm_error": max_norm_error,
        }

    def _transform_pose6(self, T_parent_child, pose_child):
        T_child_pose = self.pose6_to_transform(pose_child)
        return self.transform_to_pose6(
            self.compose_transform(T_parent_child, T_child_pose)
        )

    def world_calibration_capture_point(self, point_type, side):
        point_type = str(point_type or "").lower().strip()
        side = str(side or "").lower().strip()
        if point_type not in ("origin", "x_axis", "y_helper"):
            return {
                "ok": False,
                "code": "invalid_point_type",
                "error": "point_type must be one of: origin, x_axis, y_helper",
            }
        if side not in ("left", "right"):
            return {
                "ok": False,
                "code": "invalid_side",
                "error": "side must be one of: left, right",
            }
        try:
            pose = self.get_live_tcp_for_side(side)
        except Exception as e:
            return {"ok": False, "code": "live_tcp_unavailable", "error": str(e)}

        cal = self._load_world_frame_calibration()
        cal["points"][point_type][side] = [float(x) for x in pose]
        cal["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        self._save_world_frame_calibration(cal)
        return {
            "ok": True,
            "point_type": point_type,
            "side": side,
            "captured_pose": [float(x) for x in pose],
            "captured_position": [float(x) for x in pose[:3]],
            "warnings": ["No motion was sent."],
        }

    def world_calibration_compute_transform(self):
        cal = self._load_world_frame_calibration()
        if not self._world_calibration_has_all_points(cal):
            return {
                "ok": False,
                "code": "world_calibration_points_incomplete",
                "error": "Capture left and right TCP for origin, x_axis, and y_helper first.",
                "world_frame_calibration": cal,
                "warnings": ["No motion was sent."],
            }

        try:
            points = cal["points"]
            left_frame = self.build_frame_from_3_points(
                points["origin"]["left"][:3],
                points["x_axis"]["left"][:3],
                points["y_helper"]["left"][:3],
            )
            right_frame = self.build_frame_from_3_points(
                points["origin"]["right"][:3],
                points["x_axis"]["right"][:3],
                points["y_helper"]["right"][:3],
            )
            T_left_base_calibration_frame = left_frame["transform"]
            T_right_base_calibration_frame = right_frame["transform"]
            T_left_base_right_base = self.compose_transform(
                T_left_base_calibration_frame,
                self.invert_transform(T_right_base_calibration_frame),
            )
            computed_at = time.strftime("%Y-%m-%dT%H:%M:%S")
            cal["world_frame"] = "left_base"
            cal["enabled"] = False
            cal["T_world_left_base"] = self._identity_transform4()
            cal["T_world_right_base"] = T_left_base_right_base
            cal["T_left_base_right_base"] = T_left_base_right_base
            cal["T_left_base_calibration_frame"] = T_left_base_calibration_frame
            cal["T_right_base_calibration_frame"] = T_right_base_calibration_frame
            cal["computed_at"] = computed_at
            cal["updated_at"] = computed_at
            self._save_world_frame_calibration(cal)
            return {
                "ok": True,
                "world_frame": "left_base",
                "T_left_base_right_base": T_left_base_right_base,
                "left_axes": left_frame,
                "right_axes": right_frame,
                "validation": {
                    "left": {
                        "distances_mm": left_frame["distances_mm"],
                        "dot_products": left_frame["dot_products"],
                        "axis_norms": left_frame["axis_norms"],
                    },
                    "right": {
                        "distances_mm": right_frame["distances_mm"],
                        "dot_products": right_frame["dot_products"],
                        "axis_norms": right_frame["axis_norms"],
                    },
                },
                "enabled": False,
                "warnings": [
                    "No motion was sent.",
                    "World transform is computed but not used for execute paths yet.",
                ],
            }
        except Exception as e:
            return {
                "ok": False,
                "code": "world_calibration_compute_failed",
                "error": str(e),
                "warnings": ["No motion was sent."],
            }

    def world_calibration_status(self):
        cal = self._load_world_frame_calibration()
        return {
            "ok": True,
            "world_frame_calibration": cal,
            "has_all_points": self._world_calibration_has_all_points(cal),
            "has_transform": cal.get("T_left_base_right_base") is not None,
            "enabled": bool(cal.get("enabled", False)),
            "warnings": ["No motion was sent."],
        }

    def world_calibration_preview_live(self):
        warnings = ["No motion was sent."]
        cal = self._load_world_frame_calibration()
        try:
            left_raw = self.get_live_tcp_for_side("left")
            right_raw = self.get_live_tcp_for_side("right")
        except Exception as e:
            return {"ok": False, "code": "live_tcp_unavailable", "error": str(e), "warnings": warnings}

        T_left_base_right_base = cal.get("T_left_base_right_base")
        left_world = [float(x) for x in left_raw]
        if T_left_base_right_base is None:
            right_world = [float(x) for x in right_raw]
            warnings.append("World/base transform has not been computed; right_world_tcp is shown as raw right TCP.")
        else:
            right_world = self._transform_pose6(T_left_base_right_base, right_raw)

        dx = left_world[0] - right_world[0]
        dy = left_world[1] - right_world[1]
        dz = left_world[2] - right_world[2]
        return {
            "ok": True,
            "left_raw_tcp": left_raw,
            "right_raw_tcp": right_raw,
            "left_world_tcp": left_world,
            "right_world_tcp": right_world,
            "left_right_distance_world_mm": float(math.sqrt(dx * dx + dy * dy + dz * dz)),
            "world_calibration_enabled": bool(cal.get("enabled", False)),
            "warnings": warnings,
        }

    def compute_object_calibration_frame(self, name):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return {"ok": False, "code": "object_file_not_found", "error": f"object file not found: {path}"}

        try:
            cal = self._ensure_object_frame_calibration(data)
            points = cal["points"]
            origin = points.get("origin")
            x_point = points.get("x_axis")
            y_helper = points.get("y_helper")
            if origin is None or x_point is None or y_helper is None:
                return {
                    "ok": False,
                    "code": "calibration_points_incomplete",
                    "error": "origin, x_axis, and y_helper points are required",
                    "calibration_points": points,
                }

            origin = [float(x) for x in origin[:3]]
            x_point = [float(x) for x in x_point[:3]]
            y_helper = [float(x) for x in y_helper[:3]]
            x_vec = self._vec_sub3(x_point, origin)
            y_temp = self._vec_sub3(y_helper, origin)
            x_axis, x_dist = self._vec_normalize3(x_vec, "origin to x_axis")
            _, y_dist = self._vec_normalize3(y_temp, "origin to y_helper")
            if x_dist <= 20.0:
                return {"ok": False, "code": "x_axis_point_too_close", "error": "origin -> x_axis distance must be > 20 mm", "distance_mm": x_dist}
            if y_dist <= 20.0:
                return {"ok": False, "code": "y_helper_point_too_close", "error": "origin -> y_helper distance must be > 20 mm", "distance_mm": y_dist}

            z_raw = self._vec_cross3(x_axis, y_temp)
            z_axis, cross_norm = self._vec_normalize3(z_raw, "object z_axis")
            if cross_norm <= 1e-6:
                return {"ok": False, "code": "calibration_points_collinear", "error": "origin, x_axis, and y_helper points are nearly collinear"}
            y_axis = self._vec_cross3(z_axis, x_axis)
            y_axis, _ = self._vec_normalize3(y_axis, "object y_axis")

            T = [
                [x_axis[0], y_axis[0], z_axis[0], origin[0]],
                [x_axis[1], y_axis[1], z_axis[1], origin[1]],
                [x_axis[2], y_axis[2], z_axis[2], origin[2]],
                [0.0, 0.0, 0.0, 1.0],
            ]
            pose6 = self.transform_to_pose6(T)
            computed_at = time.strftime("%Y-%m-%dT%H:%M:%S")
            computed = {
                "x_axis": x_axis,
                "y_axis": y_axis,
                "z_axis": z_axis,
                "origin": origin,
                "pose6": pose6,
                "transform": T,
                "computed_at": computed_at,
            }
            cal["computed"] = computed
            cal["updated_at"] = computed_at
            data["object_frame_calibration"] = cal
            data["calibrated_object_pose"] = pose6
            data["calibrated_object_transform"] = T
            self._save_object_record_s1(path, data)

            warnings = [
                "No motion was sent. Existing Execute still uses legacy frame until D35.6.",
                "JAKA pose convention is assumed to be RPY with R = Rz * Ry * Rx.",
            ]
            if abs(math.cos(pose6[4])) < 0.10:
                warnings.append("Calibrated pose pitch is near RPY gimbal lock.")

            return {
                "ok": True,
                "name": Path(path).stem,
                "calibrated_object_pose": pose6,
                "axes": {"x": x_axis, "y": y_axis, "z": z_axis},
                "warnings": warnings,
            }

        except Exception as e:
            return {"ok": False, "code": "calibration_compute_failed", "error": str(e)}

    def object_calibration_status(self, name):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return {"ok": False, "code": "object_file_not_found", "error": f"object file not found: {path}"}
        cal = self._ensure_object_frame_calibration(data)
        points = cal.get("points", {})
        complete = all(points.get(k) is not None for k in ("origin", "x_axis", "y_helper"))
        computed = cal.get("computed")
        return {
            "ok": True,
            "name": Path(path).stem,
            "calibration_points": points,
            "complete": bool(complete),
            "computed": computed,
            "calibrated_object_pose": data.get("calibrated_object_pose"),
            "object_frame_calibration": cal,
        }

    def preview_calibrated_axes(self, name, axis_length_mm=100.0):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return {"ok": False, "code": "object_file_not_found", "error": f"object file not found: {path}"}
        cal = self._ensure_object_frame_calibration(data)
        computed = cal.get("computed")
        if not isinstance(computed, dict):
            return {"ok": False, "code": "calibrated_frame_missing", "error": "Compute calibrated object frame first."}

        origin = [float(x) for x in computed["origin"][:3]]
        x_axis = [float(x) for x in computed["x_axis"][:3]]
        y_axis = [float(x) for x in computed["y_axis"][:3]]
        z_axis = [float(x) for x in computed["z_axis"][:3]]
        length = float(axis_length_mm)
        return {
            "ok": True,
            "name": Path(path).stem,
            "origin": origin,
            "x_axis_end": self._vec_add3(origin, self._vec_scale3(x_axis, length)),
            "y_axis_end": self._vec_add3(origin, self._vec_scale3(y_axis, length)),
            "z_axis_end": self._vec_add3(origin, self._vec_scale3(z_axis, length)),
            "axes": {"x": x_axis, "y": y_axis, "z": z_axis},
            "axis_length_mm": length,
        }

    def preview_calibrated_rigid_compare(self, name, target_delta):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return {"ok": False, "code": "object_file_not_found", "error": f"object file not found: {path}"}
        start_pose = data.get("calibrated_object_pose")
        if start_pose is None:
            return {"ok": False, "code": "calibrated_frame_missing", "error": "Compute calibrated object frame first."}
        target_delta = self._validate_pose6(target_delta, "target_delta")
        start_pose = self._validate_pose6(start_pose, "calibrated_object_pose")
        target_pose = [
            float(start_pose[0] + target_delta[0]),
            float(start_pose[1] + target_delta[1]),
            float(start_pose[2] + target_delta[2]),
            float(self._wrap_angle(start_pose[3] + target_delta[3])),
            float(self._wrap_angle(start_pose[4] + target_delta[4])),
            float(self._wrap_angle(start_pose[5] + target_delta[5])),
        ]
        obj = dict(data)
        obj["object_pose"] = start_pose
        obj.pop("left_grasp_transform_in_object", None)
        obj.pop("right_grasp_transform_in_object", None)
        rigid = self._cooperative_tcp_target_from_object_rigid(obj, target_pose)
        return {
            "ok": True,
            "name": Path(path).stem,
            "start_object_pose": start_pose,
            "target_delta": target_delta,
            "target_object_pose": target_pose,
            "rigid": {
                "left_target_tcp": rigid["left_target_tcp_rigid"],
                "right_target_tcp": rigid["right_target_tcp_rigid"],
                "rigid_schema_version": rigid["rigid_schema_version"],
            },
            "warnings": [
                "No motion was sent. Existing Execute still uses legacy frame until D35.6.",
            ],
        }

    def preview_calibrated_rigid_compare_world_internal(self, name, target_delta):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return {"ok": False, "code": "object_file_not_found", "error": f"object file not found: {path}"}

        warnings = [
            "No motion was sent.",
            "Existing Execute still uses legacy frame until D35.6.",
        ]
        cal = self._load_world_frame_calibration()
        T_left_base_right_base = cal.get("T_left_base_right_base")
        if T_left_base_right_base is None:
            return {
                "ok": False,
                "code": "world_calibration_transform_missing",
                "error": "Compute world/base calibration transform first.",
                "warnings": warnings,
                "world_calibration_enabled": bool(cal.get("enabled", False)),
            }

        start_pose = data.get("calibrated_object_pose")
        if start_pose is None:
            return {
                "ok": False,
                "code": "calibrated_frame_missing",
                "error": "Compute calibrated object frame first.",
                "warnings": warnings,
            }

        try:
            target_delta = self._validate_pose6(target_delta, "target_delta")
            start_pose = self._validate_pose6(start_pose, "calibrated_object_pose")
            left_base = self.get_live_tcp_for_side("left")
            right_base = self.get_live_tcp_for_side("right")

            left_world = [float(x) for x in left_base]
            right_world = self._transform_pose6(T_left_base_right_base, right_base)

            target_pose = [
                float(start_pose[0] + target_delta[0]),
                float(start_pose[1] + target_delta[1]),
                float(start_pose[2] + target_delta[2]),
                float(self._wrap_angle(start_pose[3] + target_delta[3])),
                float(self._wrap_angle(start_pose[4] + target_delta[4])),
                float(self._wrap_angle(start_pose[5] + target_delta[5])),
            ]
            T_world_object_start = self.pose6_to_transform(start_pose)
            T_object_world = self.invert_transform(T_world_object_start)
            T_object_left_grasp = self.compose_transform(
                T_object_world,
                self.pose6_to_transform(left_world),
            )
            T_object_right_grasp = self.compose_transform(
                T_object_world,
                self.pose6_to_transform(right_world),
            )
            T_world_object_target = self.pose6_to_transform(target_pose)
            T_world_left_target = self.compose_transform(
                T_world_object_target,
                T_object_left_grasp,
            )
            T_world_right_target = self.compose_transform(
                T_world_object_target,
                T_object_right_grasp,
            )

            left_target_world = self.transform_to_pose6(T_world_left_target)
            right_target_world = self.transform_to_pose6(T_world_right_target)
            T_right_base_left_base = self.invert_transform(T_left_base_right_base)
            left_target_base = [float(x) for x in left_target_world]
            right_target_base = self._transform_pose6(
                T_right_base_left_base,
                right_target_world,
            )

            left_world_disp, _ = self._pose6_difference_metrics(left_world, left_target_world)
            right_world_disp, _ = self._pose6_difference_metrics(right_world, right_target_world)
            left_base_disp, _ = self._pose6_difference_metrics(left_base, left_target_base)
            right_base_disp, _ = self._pose6_difference_metrics(right_base, right_target_base)

            return {
                "ok": True,
                "name": Path(path).stem,
                "start_object_pose_world": start_pose,
                "target_object_pose_world": target_pose,
                "target_delta": target_delta,
                "world_calibration_enabled": bool(cal.get("enabled", False)),
                "current": {
                    "left_tcp_base": left_base,
                    "right_tcp_base": right_base,
                    "left_tcp_world": left_world,
                    "right_tcp_world": right_world,
                },
                "rigid_world": {
                    "left_target_tcp_world": left_target_world,
                    "right_target_tcp_world": right_target_world,
                },
                "target_for_ik_base": {
                    "left": left_target_base,
                    "right": right_target_base,
                },
                "displacement_mm": {
                    "left_world": left_world_disp,
                    "right_world": right_world_disp,
                    "left_base": left_base_disp,
                    "right_base": right_base_disp,
                },
                "warnings": warnings,
            }
        except Exception as e:
            return {
                "ok": False,
                "code": "preview_calibrated_rigid_compare_world_failed",
                "error": str(e),
                "warnings": warnings,
            }

    def preview_calibrated_rigid_compare_world(self, name, target_delta):
        return self.preview_calibrated_rigid_compare_world_internal(name, target_delta)

    def _validate_calibrated_world_ik_side(
        self,
        side,
        target_tcp_base,
        max_joint_delta_rad,
        max_joint_delta_sum_rad,
    ):
        try:
            target_tcp_base = self._validate_pose6(target_tcp_base, f"{side}_target_tcp_base")
        except Exception as e:
            return {
                "ok": False,
                "current_joint": None,
                "ik_joint": None,
                "joint_delta": None,
                "max_abs_joint_delta_rad": None,
                "sum_abs_joint_delta_rad": None,
                "reason": "invalid_ik_response",
                "error": str(e),
            }

        current_joint = self.left_joint if side == "left" else self.right_joint
        if current_joint is None:
            return {
                "ok": False,
                "current_joint": None,
                "ik_joint": None,
                "joint_delta": None,
                "max_abs_joint_delta_rad": None,
                "sum_abs_joint_delta_rad": None,
                "reason": "current_joint_unavailable",
            }

        current_joint = [float(x) for x in list(current_joint or [])[:6]]
        if len(current_joint) != 6:
            return {
                "ok": False,
                "current_joint": current_joint,
                "ik_joint": None,
                "joint_delta": None,
                "max_abs_joint_delta_rad": None,
                "sum_abs_joint_delta_rad": None,
                "reason": "current_joint_unavailable",
            }

        ik_joint, ik_msg = self.solve_ik_for_tcp_target(
            side,
            target_tcp_base,
            ref_joint=current_joint,
            timeout=1.0,
        )
        if ik_joint is None:
            return {
                "ok": False,
                "current_joint": current_joint,
                "ik_joint": None,
                "joint_delta": None,
                "max_abs_joint_delta_rad": None,
                "sum_abs_joint_delta_rad": None,
                "reason": "ik_failed",
                "ik_message": ik_msg,
            }

        ik_joint = [float(x) for x in list(ik_joint or [])[:6]]
        if len(ik_joint) != 6:
            return {
                "ok": False,
                "current_joint": current_joint,
                "ik_joint": ik_joint,
                "joint_delta": None,
                "max_abs_joint_delta_rad": None,
                "sum_abs_joint_delta_rad": None,
                "reason": "invalid_ik_response",
                "ik_message": ik_msg,
            }

        joint_delta = [
            float(self._wrap_angle(ik_joint[i] - current_joint[i]))
            for i in range(6)
        ]
        abs_delta = [abs(x) for x in joint_delta]
        max_abs_joint_delta_rad = float(max(abs_delta))
        sum_abs_joint_delta_rad = float(sum(abs_delta))

        reason = "ok"
        ok = True
        if max_abs_joint_delta_rad > max_joint_delta_rad:
            ok = False
            reason = "joint_delta_limit_exceeded"
        elif sum_abs_joint_delta_rad > max_joint_delta_sum_rad:
            ok = False
            reason = "joint_delta_sum_limit_exceeded"

        return {
            "ok": ok,
            "current_joint": current_joint,
            "ik_joint": ik_joint,
            "joint_delta": joint_delta,
            "max_abs_joint_delta_rad": max_abs_joint_delta_rad,
            "sum_abs_joint_delta_rad": sum_abs_joint_delta_rad,
            "reason": reason,
            "ik_message": ik_msg,
        }

    def validate_calibrated_world_ik(
        self,
        name,
        target_delta,
        max_joint_delta_rad=0.35,
        max_joint_delta_sum_rad=1.2,
    ):
        max_joint_delta_rad = float(max_joint_delta_rad if max_joint_delta_rad is not None else 0.35)
        max_joint_delta_sum_rad = float(
            max_joint_delta_sum_rad if max_joint_delta_sum_rad is not None else 1.2
        )
        warnings = [
            "No motion was sent.",
            "This endpoint only validates IK and joint deltas.",
            "Existing Execute still uses legacy frame until D35.6.",
        ]
        preview = self.preview_calibrated_rigid_compare_world_internal(name, target_delta)
        if not preview.get("ok", False):
            return {
                "ok": False,
                "name": name,
                "target_delta": target_delta,
                "preview": preview,
                "ik_validation": {},
                "thresholds": {
                    "max_joint_delta_rad": max_joint_delta_rad,
                    "max_joint_delta_sum_rad": max_joint_delta_sum_rad,
                },
                "ready_for_execute_preview": False,
                "warnings": warnings,
            }

        targets = preview.get("target_for_ik_base") or {}
        ik_validation = {
            "left": self._validate_calibrated_world_ik_side(
                "left",
                targets.get("left"),
                max_joint_delta_rad,
                max_joint_delta_sum_rad,
            ),
            "right": self._validate_calibrated_world_ik_side(
                "right",
                targets.get("right"),
                max_joint_delta_rad,
                max_joint_delta_sum_rad,
            ),
        }
        ok = bool(ik_validation["left"].get("ok") and ik_validation["right"].get("ok"))
        return {
            "ok": ok,
            "name": preview.get("name", name),
            "target_delta": preview.get("target_delta", target_delta),
            "preview": {
                "start_object_pose_world": preview.get("start_object_pose_world"),
                "target_object_pose_world": preview.get("target_object_pose_world"),
                "current": preview.get("current"),
                "rigid_world": preview.get("rigid_world"),
                "target_for_ik_base": preview.get("target_for_ik_base"),
                "displacement_mm": preview.get("displacement_mm"),
            },
            "ik_validation": ik_validation,
            "thresholds": {
                "max_joint_delta_rad": max_joint_delta_rad,
                "max_joint_delta_sum_rad": max_joint_delta_sum_rad,
            },
            "ready_for_execute_preview": ok,
            "warnings": warnings,
        }

    def _reject_calibrated_world_execute(self, reason, warnings, **extra):
        result = {
            "ok": False,
            "executed": False,
            "reason": reason,
            "warnings": warnings,
        }
        result.update(extra)
        return result

    def _now_iso(self):
        return time.strftime("%Y-%m-%dT%H:%M:%S")

    def _calibrated_world_target_pose_from_delta(self, start_pose, target_delta):
        start_pose = self._validate_pose6(start_pose, "start_object_pose_world")
        target_delta = self._validate_pose6(target_delta, "target_delta")
        return [
            float(start_pose[0] + target_delta[0]),
            float(start_pose[1] + target_delta[1]),
            float(start_pose[2] + target_delta[2]),
            float(self._wrap_angle(start_pose[3] + target_delta[3])),
            float(self._wrap_angle(start_pose[4] + target_delta[4])),
            float(self._wrap_angle(start_pose[5] + target_delta[5])),
        ]

    def _get_live_world_tcp_pair_for_calibrated_commit(self):
        cal = self._load_world_frame_calibration()
        T_left_base_right_base = cal.get("T_left_base_right_base")
        if T_left_base_right_base is None:
            raise RuntimeError("world calibration transform is missing")

        left_base = self.get_live_tcp_for_side("left")
        right_base = self.get_live_tcp_for_side("right")
        left_world = [float(x) for x in left_base]
        right_world = self._transform_pose6(T_left_base_right_base, right_base)
        return {
            "left_tcp_base": left_base,
            "right_tcp_base": right_base,
            "left_tcp_world": left_world,
            "right_tcp_world": right_world,
            "world_calibration_enabled": bool(cal.get("enabled", False)),
        }

    def _build_pending_calibrated_execute_commit(
        self,
        name,
        target_delta,
        validation,
        left_target_joint,
        right_target_joint,
        motion_result,
    ):
        preview = validation.get("preview", {}) if isinstance(validation, dict) else {}
        rigid_world = preview.get("rigid_world", {}) if isinstance(preview, dict) else {}
        previous_pose = preview.get("start_object_pose_world")
        target_pose = preview.get("target_object_pose_world")
        if previous_pose is None:
            data, _ = self._load_object_record_s1(name)
            if data is not None:
                previous_pose = data.get("calibrated_object_pose")
        if target_pose is None and previous_pose is not None:
            target_pose = self._calibrated_world_target_pose_from_delta(previous_pose, target_delta)

        if previous_pose is None or target_pose is None:
            raise RuntimeError("validated object start/target pose is unavailable")
        expected_left = rigid_world.get("left_target_tcp_world")
        expected_right = rigid_world.get("right_target_tcp_world")
        if expected_left is None or expected_right is None:
            raise RuntimeError("validated expected target TCP world pose is unavailable")

        now = self._now_iso()
        return {
            "id": f"d35_6a_{int(time.time() * 1000)}",
            "name": str(name),
            "created_at": now,
            "updated_at": now,
            "status": "pending_verification",
            "source": "D35.6 execute_calibrated_world_rigid",
            "target_delta": [float(x) for x in target_delta],
            "previous_object_pose_world": self._validate_pose6(previous_pose, "previous_object_pose_world"),
            "target_object_pose_world": self._validate_pose6(target_pose, "target_object_pose_world"),
            "expected_left_tcp_world": self._validate_pose6(
                expected_left,
                "expected_left_tcp_world",
            ),
            "expected_right_tcp_world": self._validate_pose6(
                expected_right,
                "expected_right_tcp_world",
            ),
            "left_target_joint": [float(x) for x in left_target_joint],
            "right_target_joint": [float(x) for x in right_target_joint],
            "motion_result": motion_result,
        }

    def _store_pending_calibrated_execute_commit(self, name, pending):
        data, path = self._load_object_record_s1(name)
        if data is None:
            raise RuntimeError(f"object file not found: {path}")
        data["pending_calibrated_execute_commit"] = pending
        data["pending_calibrated_execute_commit_updated_at"] = self._now_iso()
        self._save_object_record_s1(path, data)
        return pending

    def _load_pending_calibrated_execute_commit(self, name):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return None, data, path
        pending = data.get("pending_calibrated_execute_commit")
        if not isinstance(pending, dict) or pending.get("status") not in (
            "pending_verification",
            "verification_failed",
            "verified",
        ):
            return None, data, path
        if str(pending.get("name", name)) != str(name):
            return None, data, path
        return pending, data, path

    def pending_calibrated_execute_commit_status(self, name):
        pending, data, path = self._load_pending_calibrated_execute_commit(name)
        if data is None:
            return {
                "ok": False,
                "name": name,
                "has_pending_commit": False,
                "reason": "object_file_not_found",
                "error": f"object file not found: {path}",
                "warnings": ["No motion was sent."],
            }
        return {
            "ok": True,
            "name": Path(path).stem,
            "has_pending_commit": pending is not None,
            "pending": pending,
            "warnings": ["No motion was sent."],
        }

    def recover_pending_calibrated_execute_commit(
        self,
        name,
        target_delta,
        confirm_recover,
        position_tolerance_mm=1.5,
        orientation_tolerance_rad=0.05,
    ):
        warnings = [
            "No motion was sent.",
            "Recovered pending commit from verified live TCP after a previous execute.",
        ]
        if str(confirm_recover or "") != "RECOVER_PENDING_CALIBRATED_COMMIT":
            return {
                "ok": False,
                "recovered": False,
                "name": name,
                "reason": "missing_recover_confirmation",
                "error": "confirm_recover must be RECOVER_PENDING_CALIBRATED_COMMIT",
                "warnings": ["No motion was sent."],
            }

        try:
            target_delta = self._validate_pose6(target_delta, "target_delta")
            position_tolerance_mm = float(
                position_tolerance_mm if position_tolerance_mm is not None else 1.5
            )
            orientation_tolerance_rad = float(
                orientation_tolerance_rad if orientation_tolerance_rad is not None else 0.05
            )
        except Exception as e:
            return {
                "ok": False,
                "recovered": False,
                "name": name,
                "reason": "invalid_recovery_request",
                "error": str(e),
                "warnings": ["No motion was sent."],
            }

        data, path = self._load_object_record_s1(name)
        if data is None:
            return {
                "ok": False,
                "recovered": False,
                "name": name,
                "reason": "object_file_not_found",
                "error": f"object file not found: {path}",
                "warnings": ["No motion was sent."],
            }

        start_pose = data.get("calibrated_object_pose")
        if start_pose is None:
            return {
                "ok": False,
                "recovered": False,
                "name": Path(path).stem,
                "reason": "calibrated_frame_missing",
                "error": "Compute calibrated object frame first.",
                "warnings": ["No motion was sent."],
            }

        try:
            start_pose = self._validate_pose6(start_pose, "calibrated_object_pose")
            target_pose = self._calibrated_world_target_pose_from_delta(start_pose, target_delta)
            preview = self.preview_calibrated_rigid_compare_world_internal(name, target_delta)
            if not preview.get("ok", False):
                return {
                    "ok": False,
                    "recovered": False,
                    "name": Path(path).stem,
                    "reason": preview.get("code", "preview_calibrated_world_failed"),
                    "preview": preview,
                    "warnings": ["No motion was sent."],
                }

            live = self._get_live_world_tcp_pair_for_calibrated_commit()
            live_left_world = self._validate_pose6(live.get("left_tcp_world"), "left_tcp_world")
            live_right_world = self._validate_pose6(live.get("right_tcp_world"), "right_tcp_world")

            # Recovery is called after motion already happened, so current live TCP is the
            # only trustworthy target-grasp observation. Estimate the pre-execute grasps by
            # applying inverse object delta, then reapply the target pose to get the expected
            # target TCP. This keeps the recovered pending compatible with the D35.6A verify
            # and commit flow without sending any motion.
            T_world_object_start = self.pose6_to_transform(start_pose)
            T_world_object_target = self.pose6_to_transform(target_pose)
            T_object_target_world = self.invert_transform(T_world_object_target)
            T_object_target_left = self.compose_transform(
                T_object_target_world,
                self.pose6_to_transform(live_left_world),
            )
            T_object_target_right = self.compose_transform(
                T_object_target_world,
                self.pose6_to_transform(live_right_world),
            )
            estimated_left_tcp_world_before = self.transform_to_pose6(
                self.compose_transform(T_world_object_start, T_object_target_left)
            )
            estimated_right_tcp_world_before = self.transform_to_pose6(
                self.compose_transform(T_world_object_start, T_object_target_right)
            )
            recovered_left_expected = self.transform_to_pose6(
                self.compose_transform(T_world_object_target, T_object_target_left)
            )
            recovered_right_expected = self.transform_to_pose6(
                self.compose_transform(T_world_object_target, T_object_target_right)
            )

            left_position_error_mm, left_orientation_error_rad = self._pose6_difference_metrics(
                recovered_left_expected,
                live_left_world,
            )
            right_position_error_mm, right_orientation_error_rad = self._pose6_difference_metrics(
                recovered_right_expected,
                live_right_world,
            )
            errors = {
                "left_position_error_mm": left_position_error_mm,
                "right_position_error_mm": right_position_error_mm,
                "left_orientation_error_rad": left_orientation_error_rad,
                "right_orientation_error_rad": right_orientation_error_rad,
            }
            ready = bool(
                left_position_error_mm <= position_tolerance_mm
                and right_position_error_mm <= position_tolerance_mm
                and left_orientation_error_rad <= orientation_tolerance_rad
                and right_orientation_error_rad <= orientation_tolerance_rad
            )
            if not ready:
                return {
                    "ok": False,
                    "recovered": False,
                    "name": Path(path).stem,
                    "reason": "recovery_live_tcp_not_near_expected",
                    "errors": errors,
                    "tolerances": {
                        "position_tolerance_mm": position_tolerance_mm,
                        "orientation_tolerance_rad": orientation_tolerance_rad,
                    },
                    "live": {
                        "left_tcp_world": live_left_world,
                        "right_tcp_world": live_right_world,
                    },
                    "expected": {
                        "left_tcp_world": recovered_left_expected,
                        "right_tcp_world": recovered_right_expected,
                    },
                    "warnings": ["No motion was sent."],
                }

            preview_rigid = preview.get("rigid_world", {}) if isinstance(preview, dict) else {}
            now = self._now_iso()
            pending = {
                "id": f"d35_6ar_{int(time.time() * 1000)}",
                "name": Path(path).stem,
                "created_at": now,
                "updated_at": now,
                "status": "pending_verification",
                "source": "D35.6A-R recovery",
                "target_delta": [float(x) for x in target_delta],
                "previous_object_pose_world": start_pose,
                "target_object_pose_world": target_pose,
                "expected_left_tcp_world": recovered_left_expected,
                "expected_right_tcp_world": recovered_right_expected,
                "live_left_tcp_world_at_recovery": live_left_world,
                "live_right_tcp_world_at_recovery": live_right_world,
                "estimated_left_tcp_world_before_recovery_motion": estimated_left_tcp_world_before,
                "estimated_right_tcp_world_before_recovery_motion": estimated_right_tcp_world_before,
                "preview_expected_from_current_live": {
                    "left_tcp_world": preview_rigid.get("left_target_tcp_world"),
                    "right_tcp_world": preview_rigid.get("right_target_tcp_world"),
                    "note": "Preview helper uses live TCP at call time; for recovery after motion this may represent an extra target_delta and is kept for audit only.",
                },
                "recovery_errors": errors,
                "recovery_tolerances": {
                    "position_tolerance_mm": position_tolerance_mm,
                    "orientation_tolerance_rad": orientation_tolerance_rad,
                },
            }
            pending = self._store_pending_calibrated_execute_commit(Path(path).stem, pending)
            return {
                "ok": True,
                "recovered": True,
                "name": Path(path).stem,
                "pending": pending,
                "errors": errors,
                "warnings": warnings,
            }
        except Exception as e:
            return {
                "ok": False,
                "recovered": False,
                "name": Path(path).stem,
                "reason": "recover_pending_commit_failed",
                "error": str(e),
                "warnings": ["No motion was sent."],
            }

    def verify_pending_calibrated_execute_commit(
        self,
        name,
        position_tolerance_mm=1.5,
        orientation_tolerance_rad=0.05,
        update_status=True,
    ):
        warnings = [
            "No motion was sent.",
            "This only verifies whether the last calibrated execute reached the expected target.",
        ]
        pending, data, path = self._load_pending_calibrated_execute_commit(name)
        if data is None:
            return {
                "ok": False,
                "name": name,
                "ready_to_commit": False,
                "reason": "object_file_not_found",
                "error": f"object file not found: {path}",
                "warnings": warnings,
            }
        if pending is None:
            return {
                "ok": False,
                "name": Path(path).stem,
                "ready_to_commit": False,
                "reason": "no_pending_commit",
                "warnings": warnings,
            }

        try:
            position_tolerance_mm = float(
                position_tolerance_mm if position_tolerance_mm is not None else 1.5
            )
            orientation_tolerance_rad = float(
                orientation_tolerance_rad if orientation_tolerance_rad is not None else 0.05
            )
            live = self._get_live_world_tcp_pair_for_calibrated_commit()
            expected_left = self._validate_pose6(
                pending.get("expected_left_tcp_world"),
                "expected_left_tcp_world",
            )
            expected_right = self._validate_pose6(
                pending.get("expected_right_tcp_world"),
                "expected_right_tcp_world",
            )

            left_position_error_mm, left_orientation_error_rad = self._pose6_difference_metrics(
                expected_left,
                live["left_tcp_world"],
            )
            right_position_error_mm, right_orientation_error_rad = self._pose6_difference_metrics(
                expected_right,
                live["right_tcp_world"],
            )
            errors = {
                "left_position_error_mm": left_position_error_mm,
                "right_position_error_mm": right_position_error_mm,
                "left_orientation_error_rad": left_orientation_error_rad,
                "right_orientation_error_rad": right_orientation_error_rad,
            }
            tolerances = {
                "position_tolerance_mm": position_tolerance_mm,
                "orientation_tolerance_rad": orientation_tolerance_rad,
            }
            ready = bool(
                left_position_error_mm <= position_tolerance_mm
                and right_position_error_mm <= position_tolerance_mm
                and left_orientation_error_rad <= orientation_tolerance_rad
                and right_orientation_error_rad <= orientation_tolerance_rad
            )

            if update_status:
                pending = dict(pending)
                pending["last_verified_at"] = self._now_iso()
                pending["status"] = "verified" if ready else "verification_failed"
                pending["last_verification"] = {
                    "ready_to_commit": ready,
                    "live": {
                        "left_tcp_world": live["left_tcp_world"],
                        "right_tcp_world": live["right_tcp_world"],
                    },
                    "expected": {
                        "left_tcp_world": expected_left,
                        "right_tcp_world": expected_right,
                    },
                    "errors": errors,
                    "tolerances": tolerances,
                }
                data["pending_calibrated_execute_commit"] = pending
                data["pending_calibrated_execute_commit_updated_at"] = self._now_iso()
                self._save_object_record_s1(path, data)

            return {
                "ok": True,
                "name": Path(path).stem,
                "pending_id": pending.get("id"),
                "ready_to_commit": ready,
                "reason": None if ready else "verification_failed",
                "pending": pending,
                "live": {
                    "left_tcp_world": live["left_tcp_world"],
                    "right_tcp_world": live["right_tcp_world"],
                },
                "expected": {
                    "left_tcp_world": expected_left,
                    "right_tcp_world": expected_right,
                },
                "errors": errors,
                "tolerances": tolerances,
                "warnings": warnings,
            }
        except Exception as e:
            return {
                "ok": False,
                "name": Path(path).stem,
                "ready_to_commit": False,
                "reason": "pending_commit_verification_failed",
                "error": str(e),
                "pending": pending,
                "warnings": warnings,
            }

    def _commit_calibrated_object_pose_from_pending(self, data, pending):
        target_pose = self._validate_pose6(
            pending.get("target_object_pose_world"),
            "target_object_pose_world",
        )
        previous_pose = data.get("calibrated_object_pose")
        if previous_pose is not None:
            previous_pose = self._validate_pose6(previous_pose, "calibrated_object_pose")

        now = self._now_iso()
        T = self.pose6_to_transform(target_pose)
        x_axis = [float(T[0][0]), float(T[1][0]), float(T[2][0])]
        y_axis = [float(T[0][1]), float(T[1][1]), float(T[2][1])]
        z_axis = [float(T[0][2]), float(T[1][2]), float(T[2][2])]
        origin = [float(target_pose[0]), float(target_pose[1]), float(target_pose[2])]

        cal = self._ensure_object_frame_calibration(data)
        computed = cal.get("computed")
        if not isinstance(computed, dict):
            computed = {}
        computed.update({
            "x_axis": x_axis,
            "y_axis": y_axis,
            "z_axis": z_axis,
            "origin": origin,
            "pose6": target_pose,
            "transform": T,
            "computed_at": computed.get("computed_at") or now,
            "last_committed_at": now,
        })
        cal["computed"] = computed
        cal["updated_at"] = now

        data["object_frame_calibration"] = cal
        data["calibrated_object_pose"] = target_pose
        data["calibrated_object_transform"] = T
        data["current_object_pose"] = target_pose
        data["current_pose_source"] = "D35.6A_verified_calibrated_world_commit"
        data["current_pose_updated_at"] = now

        committed_pending = dict(pending)
        committed_pending["status"] = "committed"
        committed_pending["committed_at"] = now
        data["last_calibrated_execute_commit"] = committed_pending
        history = data.get("calibrated_execute_commit_history")
        if not isinstance(history, list):
            history = []
        history.append(committed_pending)
        data["calibrated_execute_commit_history"] = history[-10:]
        data.pop("pending_calibrated_execute_commit", None)
        data["pending_calibrated_execute_commit_updated_at"] = now

        return previous_pose, target_pose, committed_pending

    def commit_pending_calibrated_execute_pose(
        self,
        name,
        confirm_commit,
        position_tolerance_mm=1.5,
        orientation_tolerance_rad=0.05,
    ):
        warnings = [
            "No motion was sent.",
            "Object pose state was updated after verified execute.",
        ]
        if str(confirm_commit or "") != "COMMIT_CALIBRATED_OBJECT_POSE":
            return {
                "ok": False,
                "committed": False,
                "name": name,
                "reason": "missing_commit_confirmation",
                "error": "confirm_commit must be COMMIT_CALIBRATED_OBJECT_POSE",
                "warnings": ["No motion was sent."],
            }

        verification = self.verify_pending_calibrated_execute_commit(
            name,
            position_tolerance_mm,
            orientation_tolerance_rad,
            update_status=True,
        )
        if not verification.get("ok", False) or not verification.get("ready_to_commit", False):
            return {
                "ok": False,
                "committed": False,
                "name": name,
                "reason": verification.get("reason", "verification_not_ready"),
                "verification": verification,
                "warnings": ["No motion was sent."],
            }

        pending, data, path = self._load_pending_calibrated_execute_commit(name)
        if data is None:
            return {
                "ok": False,
                "committed": False,
                "name": name,
                "reason": "object_file_not_found",
                "error": f"object file not found: {path}",
                "warnings": ["No motion was sent."],
            }
        if pending is None:
            return {
                "ok": False,
                "committed": False,
                "name": Path(path).stem,
                "reason": "no_pending_commit",
                "warnings": ["No motion was sent."],
            }

        previous_pose, target_pose, committed_pending = self._commit_calibrated_object_pose_from_pending(
            data,
            pending,
        )
        self._save_object_record_s1(path, data)
        return {
            "ok": True,
            "committed": True,
            "name": Path(path).stem,
            "previous_object_pose_world": previous_pose,
            "new_object_pose_world": target_pose,
            "verification": verification,
            "committed_pending": committed_pending,
            "updated_fields": [
                "calibrated_object_pose",
                "calibrated_object_transform",
                "object_frame_calibration.computed",
                "current_object_pose",
            ],
            "warnings": warnings,
        }

    def _effective_large_calibrated_world_limits(
        self,
        max_translation_mm=100.0,
        max_rotation_rad=0.5,
        max_segment_translation_mm=10.0,
        max_segment_rotation_rad=0.05,
        max_segment_tcp_rotation_rad=0.12,
        max_joint_delta_rad_per_segment=0.12,
        max_joint_delta_sum_rad_per_segment=0.35,
        speed_scale=0.03,
    ):
        requested = {
            "max_translation_mm": max(0.0, float(max_translation_mm if max_translation_mm is not None else 100.0)),
            "max_rotation_rad": max(0.0, float(max_rotation_rad if max_rotation_rad is not None else 0.5)),
            "max_segment_translation_mm": max(1e-9, float(max_segment_translation_mm if max_segment_translation_mm is not None else 10.0)),
            "max_segment_rotation_rad": max(1e-9, float(max_segment_rotation_rad if max_segment_rotation_rad is not None else 0.05)),
            "max_segment_tcp_rotation_rad": max(1e-9, float(max_segment_tcp_rotation_rad if max_segment_tcp_rotation_rad is not None else 0.12)),
            "max_joint_delta_rad_per_segment": max(0.0, float(max_joint_delta_rad_per_segment if max_joint_delta_rad_per_segment is not None else 0.12)),
            "max_joint_delta_sum_rad_per_segment": max(0.0, float(max_joint_delta_sum_rad_per_segment if max_joint_delta_sum_rad_per_segment is not None else 0.35)),
            "speed_scale": max(0.0, float(speed_scale if speed_scale is not None else 0.03)),
        }
        effective = {
            "max_translation_mm": min(requested["max_translation_mm"], HARD_LARGE_MAX_TRANSLATION_MM),
            "max_rotation_rad": min(requested["max_rotation_rad"], HARD_LARGE_MAX_ROTATION_RAD),
            "max_segment_translation_mm": min(requested["max_segment_translation_mm"], HARD_LARGE_MAX_SEGMENT_TRANSLATION_MM),
            "max_segment_rotation_rad": min(requested["max_segment_rotation_rad"], HARD_LARGE_MAX_SEGMENT_ROTATION_RAD),
            "max_segment_tcp_rotation_rad": min(requested["max_segment_tcp_rotation_rad"], HARD_LARGE_MAX_SEGMENT_TCP_ROTATION_RAD),
            "max_joint_delta_rad_per_segment": min(
                requested["max_joint_delta_rad_per_segment"],
                HARD_LARGE_MAX_JOINT_DELTA_RAD_PER_SEGMENT,
            ),
            "max_joint_delta_sum_rad_per_segment": min(
                requested["max_joint_delta_sum_rad_per_segment"],
                HARD_LARGE_MAX_JOINT_DELTA_SUM_RAD_PER_SEGMENT,
            ),
            "speed_scale": min(requested["speed_scale"], HARD_LARGE_MAX_SPEED_SCALE),
        }
        return requested, effective

    def _pose_add_delta(self, pose, delta):
        pose = self._validate_pose6(pose, "pose")
        delta = self._validate_pose6(delta, "delta")
        return [
            float(pose[0] + delta[0]),
            float(pose[1] + delta[1]),
            float(pose[2] + delta[2]),
            float(self._wrap_angle(pose[3] + delta[3])),
            float(self._wrap_angle(pose[4] + delta[4])),
            float(self._wrap_angle(pose[5] + delta[5])),
        ]

    def _plan_large_calibrated_world_segments(self, name, target_delta, effective_limits):
        data, path = self._load_object_record_s1(name)
        if data is None:
            return {
                "ok": False,
                "reason": "object_file_not_found",
                "error": f"object file not found: {path}",
                "warnings": ["No motion was sent."],
            }

        start_object_pose = data.get("calibrated_object_pose")
        if start_object_pose is None:
            return {
                "ok": False,
                "reason": "calibrated_frame_missing",
                "error": "Compute calibrated object frame first.",
                "warnings": ["No motion was sent."],
            }

        try:
            target_delta = self._validate_pose6(target_delta, "target_delta")
            start_object_pose = self._validate_pose6(start_object_pose, "calibrated_object_pose")
            live = self._get_live_world_tcp_pair_for_calibrated_commit()
        except Exception as e:
            return {
                "ok": False,
                "reason": "large_plan_initial_state_failed",
                "error": str(e),
                "warnings": ["No motion was sent."],
            }

        translation_mm, rotation_rad = self._object_motion_delta_metrics(target_delta)
        if translation_mm > effective_limits["max_translation_mm"]:
            return {
                "ok": False,
                "reason": "target_translation_limit_exceeded",
                "target_translation_mm": translation_mm,
                "limit_mm": effective_limits["max_translation_mm"],
                "warnings": ["No motion was sent."],
            }
        if rotation_rad > effective_limits["max_rotation_rad"]:
            return {
                "ok": False,
                "reason": "target_rotation_limit_exceeded",
                "target_rotation_rad": rotation_rad,
                "limit_rad": effective_limits["max_rotation_rad"],
                "warnings": ["No motion was sent."],
            }

        trans_segments = int(math.ceil(translation_mm / max(effective_limits["max_segment_translation_mm"], 1e-9)))
        rot_segments = int(math.ceil(rotation_rad / max(effective_limits["max_segment_rotation_rad"], 1e-9)))
        segment_count = max(1, trans_segments, rot_segments)
        segment_delta = [float(x / segment_count) for x in target_delta]

        cal = self._load_world_frame_calibration()
        T_left_base_right_base = cal.get("T_left_base_right_base")
        if T_left_base_right_base is None:
            return {
                "ok": False,
                "reason": "world_calibration_transform_missing",
                "error": "Compute world/base calibration transform first.",
                "warnings": ["No motion was sent."],
            }
        T_right_base_left_base = self.invert_transform(T_left_base_right_base)

        T_world_object_start = self.pose6_to_transform(start_object_pose)
        T_object_world_start = self.invert_transform(T_world_object_start)
        T_object_left_grasp = self.compose_transform(
            T_object_world_start,
            self.pose6_to_transform(live["left_tcp_world"]),
        )
        T_object_right_grasp = self.compose_transform(
            T_object_world_start,
            self.pose6_to_transform(live["right_tcp_world"]),
        )

        left_ref_joint = list(self.left_joint or [])
        right_ref_joint = list(self.right_joint or [])
        if len(left_ref_joint) < 6 or len(right_ref_joint) < 6:
            return {
                "ok": False,
                "reason": "joint_feedback_missing",
                "error": "left/right joint feedback is required for segmented IK validation.",
                "warnings": ["No motion was sent."],
            }

        segments = []
        current_object_pose = start_object_pose
        prev_left_tcp_world = live["left_tcp_world"]
        prev_right_tcp_world = live["right_tcp_world"]
        max_left_joint_jump = 0.0
        max_right_joint_jump = 0.0
        max_left_joint_sum = 0.0
        max_right_joint_sum = 0.0

        for idx in range(1, segment_count + 1):
            segment_start_pose = current_object_pose
            segment_target_pose = self._pose_add_delta(segment_start_pose, segment_delta)
            T_world_object_target = self.pose6_to_transform(segment_target_pose)
            left_target_world = self.transform_to_pose6(
                self.compose_transform(T_world_object_target, T_object_left_grasp)
            )
            right_target_world = self.transform_to_pose6(
                self.compose_transform(T_world_object_target, T_object_right_grasp)
            )
            left_target_base = [float(x) for x in left_target_world]
            right_target_base = self._transform_pose6(T_right_base_left_base, right_target_world)

            left_disp_mm, left_disp_rot = self._pose6_difference_metrics(prev_left_tcp_world, left_target_world)
            right_disp_mm, right_disp_rot = self._pose6_difference_metrics(prev_right_tcp_world, right_target_world)
            if max(left_disp_mm, right_disp_mm) > effective_limits["max_segment_translation_mm"]:
                return {
                    "ok": False,
                    "reason": "segment_tcp_translation_limit_exceeded",
                    "segment_index": idx,
                    "left_displacement_mm": left_disp_mm,
                    "right_displacement_mm": right_disp_mm,
                    "limit_mm": effective_limits["max_segment_translation_mm"],
                    "segments": segments,
                    "warnings": ["No motion was sent."],
                }
            if max(left_disp_rot, right_disp_rot) > effective_limits["max_segment_tcp_rotation_rad"]:
                return {
                    "ok": False,
                    "reason": "segment_tcp_rotation_limit_exceeded",
                    "segment_index": idx,
                    "left_rotation_rad": left_disp_rot,
                    "right_rotation_rad": right_disp_rot,
                    "limit_rad": effective_limits["max_segment_tcp_rotation_rad"],
                    "object_segment_rotation_limit_rad": effective_limits["max_segment_rotation_rad"],
                    "segments": segments,
                    "warnings": ["No motion was sent."],
                }

            left_joint, left_msg = self.solve_ik_for_tcp_target(
                "left",
                left_target_base,
                ref_joint=left_ref_joint,
                timeout=1.0,
            )
            right_joint, right_msg = self.solve_ik_for_tcp_target(
                "right",
                right_target_base,
                ref_joint=right_ref_joint,
                timeout=1.0,
            )
            if left_joint is None or right_joint is None:
                return {
                    "ok": False,
                    "reason": "segment_ik_failed",
                    "segment_index": idx,
                    "left_ik_message": left_msg,
                    "right_ik_message": right_msg,
                    "segments": segments,
                    "warnings": ["No motion was sent."],
                }

            left_delta = [float(t - c) for c, t in zip(left_ref_joint, left_joint)]
            right_delta = [float(t - c) for c, t in zip(right_ref_joint, right_joint)]
            left_max_jump = max(abs(x) for x in left_delta)
            right_max_jump = max(abs(x) for x in right_delta)
            left_sum_jump = sum(abs(x) for x in left_delta)
            right_sum_jump = sum(abs(x) for x in right_delta)
            max_left_joint_jump = max(max_left_joint_jump, left_max_jump)
            max_right_joint_jump = max(max_right_joint_jump, right_max_jump)
            max_left_joint_sum = max(max_left_joint_sum, left_sum_jump)
            max_right_joint_sum = max(max_right_joint_sum, right_sum_jump)

            if max(left_max_jump, right_max_jump) > effective_limits["max_joint_delta_rad_per_segment"]:
                return {
                    "ok": False,
                    "reason": "segment_joint_delta_limit_exceeded",
                    "segment_index": idx,
                    "left_max_joint_delta_rad": left_max_jump,
                    "right_max_joint_delta_rad": right_max_jump,
                    "limit_rad": effective_limits["max_joint_delta_rad_per_segment"],
                    "segments": segments,
                    "warnings": ["No motion was sent."],
                }
            if max(left_sum_jump, right_sum_jump) > effective_limits["max_joint_delta_sum_rad_per_segment"]:
                return {
                    "ok": False,
                    "reason": "segment_joint_delta_sum_limit_exceeded",
                    "segment_index": idx,
                    "left_sum_joint_delta_rad": left_sum_jump,
                    "right_sum_joint_delta_rad": right_sum_jump,
                    "limit_rad": effective_limits["max_joint_delta_sum_rad_per_segment"],
                    "segments": segments,
                    "warnings": ["No motion was sent."],
                }

            segments.append({
                "segment_index": idx,
                "segment_delta": segment_delta,
                "start_object_pose_world": segment_start_pose,
                "target_object_pose_world": segment_target_pose,
                "left_target_tcp_world": left_target_world,
                "right_target_tcp_world": right_target_world,
                "target_for_ik_base": {
                    "left": left_target_base,
                    "right": right_target_base,
                },
                "left_target_joint": left_joint,
                "right_target_joint": right_joint,
                "ik_validation": {
                    "left": {
                        "ok": True,
                        "ik_message": left_msg,
                        "joint_delta": left_delta,
                        "max_abs_joint_delta_rad": left_max_jump,
                        "sum_abs_joint_delta_rad": left_sum_jump,
                    },
                    "right": {
                        "ok": True,
                        "ik_message": right_msg,
                        "joint_delta": right_delta,
                        "max_abs_joint_delta_rad": right_max_jump,
                        "sum_abs_joint_delta_rad": right_sum_jump,
                    },
                },
                "tcp_displacement": {
                    "left_translation_mm": left_disp_mm,
                    "right_translation_mm": right_disp_mm,
                    "left_rotation_rad": left_disp_rot,
                    "right_rotation_rad": right_disp_rot,
                },
            })
            current_object_pose = segment_target_pose
            prev_left_tcp_world = left_target_world
            prev_right_tcp_world = right_target_world
            left_ref_joint = left_joint
            right_ref_joint = right_joint

        return {
            "ok": True,
            "mode": "large_segmented_joint_move",
            "name": Path(path).stem,
            "target_delta": target_delta,
            "target_delta_check": {
                "translation_mm": translation_mm,
                "rotation_rad": rotation_rad,
            },
            "segment_count": segment_count,
            "segment_delta": segment_delta,
            "start_object_pose_world": start_object_pose,
            "final_target_object_pose_world": current_object_pose,
            "initial_live_world": {
                "left_tcp_world": live["left_tcp_world"],
                "right_tcp_world": live["right_tcp_world"],
            },
            "segments": segments,
            "summary": {
                "max_left_joint_jump_rad": max_left_joint_jump,
                "max_right_joint_jump_rad": max_right_joint_jump,
                "max_left_joint_delta_sum_rad": max_left_joint_sum,
                "max_right_joint_delta_sum_rad": max_right_joint_sum,
            },
            "warnings": ["No motion was sent."],
        }

    def _build_large_pending_calibrated_execute_commit(self, name, target_delta, plan, motion_results):
        final_segment = plan["segments"][-1]
        now = self._now_iso()
        return {
            "id": f"d35_7_{int(time.time() * 1000)}",
            "name": str(name),
            "created_at": now,
            "updated_at": now,
            "status": "pending_verification",
            "source": "D35.7 large segmented execute",
            "target_delta": [float(x) for x in target_delta],
            "previous_object_pose_world": plan.get("start_object_pose_world"),
            "target_object_pose_world": plan.get("final_target_object_pose_world"),
            "expected_left_tcp_world": final_segment.get("left_target_tcp_world"),
            "expected_right_tcp_world": final_segment.get("right_target_tcp_world"),
            "left_target_joint": final_segment.get("left_target_joint"),
            "right_target_joint": final_segment.get("right_target_joint"),
            "segment_count": plan.get("segment_count"),
            "motion_results": motion_results,
        }

    def execute_calibrated_world_rigid_large(
        self,
        name,
        target_delta,
        dry_run=True,
        confirm_execute="",
        max_translation_mm=100.0,
        max_rotation_rad=0.5,
        max_segment_translation_mm=10.0,
        max_segment_rotation_rad=0.05,
        max_segment_tcp_rotation_rad=0.12,
        max_joint_delta_rad_per_segment=0.12,
        max_joint_delta_sum_rad_per_segment=0.35,
        speed_scale=0.03,
        auto_commit=False,
    ):
        warnings = [
            "D35.7 large calibrated world rigid execute endpoint.",
            "Large motion will be executed as segmented joint_move.",
        ]
        pending, data, path = self._load_pending_calibrated_execute_commit(name)
        if data is None:
            return {
                "ok": False,
                "executed": False,
                "dry_run": bool(dry_run),
                "mode": "large_segmented_joint_move",
                "name": name,
                "reason": "object_file_not_found",
                "error": f"object file not found: {path}",
                "warnings": ["No motion was sent."],
            }
        if pending is not None:
            return {
                "ok": False,
                "executed": False,
                "dry_run": bool(dry_run),
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "reason": "pending_commit_exists",
                "pending": pending,
                "warnings": [
                    "No motion was sent.",
                    "Verify and commit the existing pending calibrated object pose before running another calibrated execute.",
                ],
            }
        if bool(auto_commit):
            return {
                "ok": False,
                "executed": False,
                "dry_run": bool(dry_run),
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "reason": "auto_commit_not_supported",
                "warnings": ["No motion was sent."],
            }

        try:
            target_delta = self._validate_pose6(target_delta, "target_delta")
            requested_limits, effective_limits = self._effective_large_calibrated_world_limits(
                max_translation_mm,
                max_rotation_rad,
                max_segment_translation_mm,
                max_segment_rotation_rad,
                max_segment_tcp_rotation_rad,
                max_joint_delta_rad_per_segment,
                max_joint_delta_sum_rad_per_segment,
                speed_scale,
            )
        except Exception as e:
            return {
                "ok": False,
                "executed": False,
                "dry_run": bool(dry_run),
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "reason": "invalid_large_execute_request",
                "error": str(e),
                "warnings": ["No motion was sent."],
            }

        plan = self._plan_large_calibrated_world_segments(
            Path(path).stem,
            target_delta,
            effective_limits,
        )
        limits = {
            "requested": requested_limits,
            "effective": effective_limits,
            "hard_caps": {
                "max_translation_mm": HARD_LARGE_MAX_TRANSLATION_MM,
                "max_rotation_rad": HARD_LARGE_MAX_ROTATION_RAD,
                "max_segment_translation_mm": HARD_LARGE_MAX_SEGMENT_TRANSLATION_MM,
                "max_segment_rotation_rad": HARD_LARGE_MAX_SEGMENT_ROTATION_RAD,
                "max_segment_tcp_rotation_rad": HARD_LARGE_MAX_SEGMENT_TCP_ROTATION_RAD,
                "max_joint_delta_rad_per_segment": HARD_LARGE_MAX_JOINT_DELTA_RAD_PER_SEGMENT,
                "max_joint_delta_sum_rad_per_segment": HARD_LARGE_MAX_JOINT_DELTA_SUM_RAD_PER_SEGMENT,
                "max_speed_scale": HARD_LARGE_MAX_SPEED_SCALE,
            },
        }
        if not plan.get("ok", False):
            plan["limits"] = limits
            return {
                "ok": False,
                "executed": False,
                "dry_run": bool(dry_run),
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "reason": plan.get("reason", "large_segment_plan_failed"),
                "plan": plan,
                "warnings": ["No motion was sent."],
            }

        if bool(dry_run):
            return {
                "ok": True,
                "executed": False,
                "dry_run": True,
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "target_delta": target_delta,
                "segment_count": plan["segment_count"],
                "segment_delta": plan["segment_delta"],
                "final_target_object_pose_world": plan["final_target_object_pose_world"],
                "segments": plan["segments"],
                "limits": limits,
                "summary": plan.get("summary"),
                "warnings": [
                    "Dry run only. No motion was sent.",
                    "Large motion will be executed as segmented joint_move.",
                ],
            }

        if str(confirm_execute or "") != "EXECUTE_LARGE_CALIBRATED_WORLD_RIGID":
            return {
                "ok": False,
                "executed": False,
                "dry_run": False,
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "reason": "missing_execute_confirmation",
                "warnings": [
                    "No motion was sent.",
                    "Set confirm_execute exactly to EXECUTE_LARGE_CALIBRATED_WORLD_RIGID.",
                ],
            }

        ok_state, state_msg = self.safe_state_ok("both")
        if not ok_state:
            return {
                "ok": False,
                "executed": False,
                "dry_run": False,
                "mode": "large_segmented_joint_move",
                "name": Path(path).stem,
                "reason": "robot_state_not_ready",
                "error": state_msg,
                "warnings": ["No motion was sent."],
            }

        base_vel = float(self.cfg["motion"].get("default_joint_vel", 0.18))
        base_acc = float(self.cfg["motion"].get("default_joint_acc", 0.30))
        vel = max(0.001, base_vel * effective_limits["speed_scale"])
        acc = max(0.001, base_acc * effective_limits["speed_scale"])
        motion_results = []
        stop_generation_at_start = getattr(self, "stop_generation", 0)
        for segment in plan["segments"]:
            motion_result = self.move_both_joint(
                left_target=segment["left_target_joint"],
                right_target=segment["right_target_joint"],
                side="both",
                vel=vel,
                acc=acc,
            )
            done = False
            done_msg = "not_started"
            if motion_result.get("ok", False) and motion_result.get("accepted", False):
                done, done_msg = self.wait_motion_done(
                    "both",
                    timeout=10.0,
                    stop_generation_at_start=stop_generation_at_start,
                )
            elif motion_result.get("ok", False) and not motion_result.get("accepted", False):
                done = True
                done_msg = "already_at_target"
            motion_results.append({
                "segment_index": segment["segment_index"],
                "motion_result": motion_result,
                "wait_done": done,
                "wait_message": done_msg,
            })
            if not motion_result.get("ok", False) or not done:
                return {
                    "ok": False,
                    "executed": False,
                    "dry_run": False,
                    "mode": "large_segmented_joint_move",
                    "name": Path(path).stem,
                    "segment_count": plan["segment_count"],
                    "segments_executed": max(0, len(motion_results) - 1),
                    "failed_segment": segment["segment_index"],
                    "reason": "segment_motion_failed",
                    "motion_results": motion_results,
                    "warnings": warnings + [
                        "Real robot motion was requested.",
                        "Segmented execution stopped before completion.",
                    ],
                }

        pending_commit = None
        pending_commit_error = None
        try:
            pending_commit = self._build_large_pending_calibrated_execute_commit(
                Path(path).stem,
                target_delta,
                plan,
                motion_results,
            )
            pending_commit = self._store_pending_calibrated_execute_commit(Path(path).stem, pending_commit)
        except Exception as e:
            pending_commit_error = str(e)

        response_warnings = warnings + [
            "Real robot motion was requested.",
            "Large motion was executed as segmented joint_move.",
            "Verify and commit object pose before next calibrated execute.",
        ]
        if pending_commit_error is not None:
            response_warnings.append(
                f"Motion completed, but pending calibrated object pose commit could not be stored: {pending_commit_error}"
            )
        return {
            "ok": pending_commit_error is None,
            "executed": True,
            "dry_run": False,
            "mode": "large_segmented_joint_move",
            "name": Path(path).stem,
            "target_delta": target_delta,
            "segment_count": plan["segment_count"],
            "segments_executed": len(motion_results),
            "final_target_object_pose_world": plan["final_target_object_pose_world"],
            "motion_results": motion_results,
            "pending_calibrated_execute_commit": pending_commit,
            "pending_calibrated_execute_commit_error": pending_commit_error,
            "limits": limits,
            "summary": plan.get("summary"),
            "warnings": response_warnings,
        }

    def execute_calibrated_world_rigid(
        self,
        name,
        target_delta,
        dry_run=True,
        confirm_execute="",
        max_translation_mm=5.0,
        max_rotation_rad=0.02,
        max_joint_delta_rad=0.12,
        max_joint_delta_sum_rad=0.35,
        speed_scale=0.03,
    ):
        warnings = [
            "D35.6 calibrated world rigid execute endpoint.",
            "Only small calibrated world rigid motion should be used in this phase.",
        ]
        dry_run = bool(dry_run)
        try:
            target_delta = self._validate_pose6(target_delta, "target_delta")
        except Exception as e:
            return self._reject_calibrated_world_execute(
                "invalid_target_delta",
                warnings + ["No motion was sent."],
                error=str(e),
            )

        requested_limits = {
            "max_translation_mm": max(0.0, float(max_translation_mm if max_translation_mm is not None else 5.0)),
            "max_rotation_rad": max(0.0, float(max_rotation_rad if max_rotation_rad is not None else 0.02)),
            "max_joint_delta_rad": max(0.0, float(max_joint_delta_rad if max_joint_delta_rad is not None else 0.12)),
            "max_joint_delta_sum_rad": max(0.0, float(max_joint_delta_sum_rad if max_joint_delta_sum_rad is not None else 0.35)),
            "speed_scale": float(speed_scale if speed_scale is not None else 0.03),
        }
        effective_limits = {
            "max_translation_mm": min(
                requested_limits["max_translation_mm"],
                HARD_MAX_CALIBRATED_WORLD_TRANSLATION_MM,
            ),
            "max_rotation_rad": min(
                requested_limits["max_rotation_rad"],
                HARD_MAX_CALIBRATED_WORLD_ROTATION_RAD,
            ),
            "max_joint_delta_rad": min(
                requested_limits["max_joint_delta_rad"],
                HARD_MAX_CALIBRATED_WORLD_JOINT_DELTA_RAD,
            ),
            "max_joint_delta_sum_rad": min(
                requested_limits["max_joint_delta_sum_rad"],
                HARD_MAX_CALIBRATED_WORLD_JOINT_DELTA_SUM_RAD,
            ),
            "speed_scale": min(
                max(0.0, requested_limits["speed_scale"]),
                HARD_MAX_CALIBRATED_WORLD_SPEED_SCALE,
            ),
        }

        translation_mm, rotation_rad = self._object_motion_delta_metrics(target_delta)
        target_delta_check = {
            "translation_mm": translation_mm,
            "rotation_rad": rotation_rad,
            "requested_limits": requested_limits,
            "effective_limits": effective_limits,
            "hard_caps": {
                "max_translation_mm": HARD_MAX_CALIBRATED_WORLD_TRANSLATION_MM,
                "max_rotation_rad": HARD_MAX_CALIBRATED_WORLD_ROTATION_RAD,
                "max_joint_delta_rad": HARD_MAX_CALIBRATED_WORLD_JOINT_DELTA_RAD,
                "max_joint_delta_sum_rad": HARD_MAX_CALIBRATED_WORLD_JOINT_DELTA_SUM_RAD,
                "max_speed_scale": HARD_MAX_CALIBRATED_WORLD_SPEED_SCALE,
            },
        }
        if translation_mm > effective_limits["max_translation_mm"]:
            return self._reject_calibrated_world_execute(
                "target_translation_limit_exceeded",
                warnings + ["No motion was sent."],
                target_delta=target_delta,
                target_delta_check=target_delta_check,
            )
        if rotation_rad > effective_limits["max_rotation_rad"]:
            return self._reject_calibrated_world_execute(
                "target_rotation_limit_exceeded",
                warnings + ["No motion was sent."],
                target_delta=target_delta,
                target_delta_check=target_delta_check,
            )

        validation = self.validate_calibrated_world_ik(
            name,
            target_delta,
            effective_limits["max_joint_delta_rad"],
            effective_limits["max_joint_delta_sum_rad"],
        )
        if not validation.get("ok", False):
            return self._reject_calibrated_world_execute(
                "calibrated_world_ik_validation_failed",
                warnings + ["No motion was sent."],
                name=name,
                target_delta=target_delta,
                target_delta_check=target_delta_check,
                validation=validation,
            )

        left_target_joint = (
            validation.get("ik_validation", {})
            .get("left", {})
            .get("ik_joint")
        )
        right_target_joint = (
            validation.get("ik_validation", {})
            .get("right", {})
            .get("ik_joint")
        )
        if left_target_joint is None or right_target_joint is None:
            return self._reject_calibrated_world_execute(
                "validated_ik_joint_missing",
                warnings + ["No motion was sent."],
                name=name,
                target_delta=target_delta,
                target_delta_check=target_delta_check,
                validation=validation,
            )

        if dry_run:
            return {
                "ok": True,
                "executed": False,
                "dry_run": True,
                "name": name,
                "target_delta": target_delta,
                "target_delta_check": target_delta_check,
                "validation": validation,
                "left_target_joint": left_target_joint,
                "right_target_joint": right_target_joint,
                "warnings": [
                    "Dry run only. No motion was sent.",
                    "Set dry_run=false and confirm_execute exactly to execute real robot motion.",
                ],
            }

        if str(confirm_execute or "") != "EXECUTE_CALIBRATED_WORLD_RIGID":
            return self._reject_calibrated_world_execute(
                "missing_execute_confirmation",
                warnings + [
                    "No motion was sent.",
                    "Set confirm_execute exactly to EXECUTE_CALIBRATED_WORLD_RIGID.",
                ],
                name=name,
                target_delta=target_delta,
                target_delta_check=target_delta_check,
                validation=validation,
            )

        ok_state, state_msg = self.safe_state_ok("both")
        if not ok_state:
            return self._reject_calibrated_world_execute(
                "robot_state_not_ready",
                warnings + ["No motion was sent."],
                name=name,
                target_delta=target_delta,
                target_delta_check=target_delta_check,
                validation=validation,
                error=state_msg,
            )

        base_vel = float(self.cfg["motion"].get("default_joint_vel", 0.18))
        base_acc = float(self.cfg["motion"].get("default_joint_acc", 0.30))
        vel = max(0.001, base_vel * effective_limits["speed_scale"])
        acc = max(0.001, base_acc * effective_limits["speed_scale"])
        motion_result = self.move_both_joint(
            left_target=left_target_joint,
            right_target=right_target_joint,
            side="both",
            vel=vel,
            acc=acc,
        )
        executed = bool(motion_result.get("ok", False) and motion_result.get("accepted", False))
        pending_commit = None
        pending_commit_error = None
        if executed:
            try:
                pending_commit = self._build_pending_calibrated_execute_commit(
                    name,
                    target_delta,
                    validation,
                    left_target_joint,
                    right_target_joint,
                    motion_result,
                )
                pending_commit = self._store_pending_calibrated_execute_commit(name, pending_commit)
            except Exception as e:
                pending_commit_error = str(e)

        response_warnings = warnings + [
            "Real robot motion was requested.",
            "joint_move helper may enforce configured minimum velocity/acceleration.",
            "No linear_move or servo_j was used.",
        ]
        if pending_commit is not None:
            response_warnings.append(
                "Pending calibrated object pose commit was created. Verify and explicitly commit before the next calibrated object execute."
            )
        if pending_commit_error is not None:
            response_warnings.append(
                f"Motion result was accepted, but pending object pose commit could not be stored: {pending_commit_error}"
            )
        return {
            "ok": bool(motion_result.get("ok", False)),
            "executed": executed,
            "dry_run": False,
            "name": name,
            "target_delta": target_delta,
            "target_delta_check": target_delta_check,
            "validation": validation,
            "motion": {
                "mode": "joint_move",
                "sync": True,
                "speed_scale": effective_limits["speed_scale"],
                "vel": vel,
                "acc": acc,
                "left_target_joint": left_target_joint,
                "right_target_joint": right_target_joint,
                "left_result": {
                    "sent": bool(motion_result.get("left_send", False)),
                    "delta": motion_result.get("left_delta"),
                    "vel": motion_result.get("left_vel"),
                    "acc": motion_result.get("left_acc"),
                },
                "right_result": {
                    "sent": bool(motion_result.get("right_send", False)),
                    "delta": motion_result.get("right_delta"),
                    "vel": motion_result.get("right_vel"),
                    "acc": motion_result.get("right_acc"),
                },
                "result": motion_result,
            },
            "pending_calibrated_execute_commit": pending_commit,
            "pending_calibrated_execute_commit_error": pending_commit_error,
            "warnings": response_warnings,
        }

    def sync_current_object_pose_from_live_tcp(self, name):
        """
        Non-motion command:
        update stored current_object_pose from live TCP midpoint.
        Use this after Home/Jog/manual reposition/Stop-before-retry.
        """
        import time

        data, path = self._load_object_record_s1(name)
        if data is None:
            return {
                "ok": False,
                "error": f"object file not found: {path}",
                "name": name,
            }

        try:
            live = self._live_object_pose_from_tcp_midpoint()
        except Exception as e:
            return {
                "ok": False,
                "code": "live_object_pose_unavailable",
                "motion_blocked": True,
                "name": name,
                "error": (
                    "Cannot compute live object pose from current TCP midpoint. "
                    f"No motion was sent: {e}"
                ),
            }
        old_current = data.get("current_object_pose", data.get("object_pose"))

        data["current_object_pose"] = live["live_object_pose"]
        data["current_pose_updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        data["current_pose_source"] = "live_tcp_midpoint_sync"
        data["last_live_sync"] = {
            "left_tcp_live": live["left_tcp_live"],
            "right_tcp_live": live["right_tcp_live"],
            "live_object_pose": live["live_object_pose"],
            "synced_at": data["current_pose_updated_at"],
        }

        self._save_object_record_s1(path, data)

        return {
            "ok": True,
            "message": "current_object_pose synced from live TCP midpoint",
            "name": Path(path).stem,
            "file": Path(path).name,
            "path": str(path),
            "old_current_object_pose": old_current,
            "live_object_pose": live["live_object_pose"],
            "left_tcp_live": live["left_tcp_live"],
            "right_tcp_live": live["right_tcp_live"],
            "current_object_pose": data["current_object_pose"],
            "current_pose_source": data["current_pose_source"],
            "current_pose_updated_at": data["current_pose_updated_at"],
            "updated_object_frame": data,
            **data,
        }

    def validate_current_object_pose_fresh(self, name, max_translation_mm=3.0, max_rotation_rad=0.08):
        """
        Non-motion safety guard:
        Compare stored current_object_pose with live TCP midpoint before executing motion.
        If stale, abort before sending any robot command.
        """
        import math

        data, path = self._load_object_record_s1(name)
        if data is None:
            return {
                "ok": False,
                "error": f"object file not found: {path}",
                "code": "object_file_not_found",
                "name": name,
            }

        stored = data.get("current_object_pose", data.get("object_pose"))
        if stored is None:
            return {
                "ok": False,
                "error": "object has no current_object_pose or object_pose",
                "code": "missing_current_object_pose",
                "name": name,
            }

        stored = self._normalize_pose6(stored, "stored current_object_pose")
        try:
            live = self._live_object_pose_from_tcp_midpoint()
        except Exception as e:
            return {
                "ok": False,
                "code": "live_object_pose_unavailable",
                "motion_blocked": True,
                "name": Path(path).stem,
                "stored_current_object_pose": stored,
                "live_object_pose": None,
                "translation_error_mm": None,
                "rotation_error_rad": None,
                "error": (
                    "Cannot compute live object pose from current TCP midpoint. "
                    f"Motion blocked: {e}"
                ),
            }

        lp = live["live_object_pose"]

        dx = lp[0] - stored[0]
        dy = lp[1] - stored[1]
        dz = lp[2] - stored[2]
        translation_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        drx = self._angle_diff_abs(lp[3], stored[3])
        dry = self._angle_diff_abs(lp[4], stored[4])
        drz = self._angle_diff_abs(lp[5], stored[5])
        rotation_error = max(drx, dry, drz)

        fresh = translation_error <= float(max_translation_mm) and rotation_error <= float(max_rotation_rad)

        result = {
            "ok": fresh,
            "code": "current_object_pose_fresh" if fresh else "stale_current_object_pose",
            "name": Path(path).stem,
            "stored_current_object_pose": stored,
            "live_object_pose": lp,
            "translation_error_mm": translation_error,
            "rotation_error_rad": rotation_error,
            "max_translation_mm": float(max_translation_mm),
            "max_rotation_rad": float(max_rotation_rad),
            "left_tcp_live": live["left_tcp_live"],
            "right_tcp_live": live["right_tcp_live"],
        }

        if not fresh:
            result["error"] = (
                "Stored current_object_pose is stale. "
                "Please sync current from live TCP before executing motion. "
                f"translation_error={translation_error:.3f} mm, "
                f"rotation_error={rotation_error:.4f} rad."
            )
            result["motion_blocked"] = True

        return result

    def _call_servo_j_parallel(self, side, left_increment=None, right_increment=None, speed=None, timeout=0.5):
        side = (side or "both").lower().strip()

        if speed is None:
            speed = [0.0] * 6

        speed = [float(x) for x in speed]
        if len(speed) != 6:
            speed = [0.0] * 6

        requests = {}

        # Create both requests first, then call_async both before waiting.
        # This reduces left-right command skew compared with left->wait->right->wait.
        if side in ("left", "both"):
            if left_increment is None:
                return {"ok": False, "error": "left_increment missing"}

            if not self.left_servo_j.service_is_ready():
                return {"ok": False, "error": "left servo_j service not ready"}

            req = ServoMove.Request()
            req.pose = [float(x) for x in left_increment]
            req.speed = speed
            requests["left"] = {
                "client": self.left_servo_j,
                "request": req,
                "future": None,
            }

        if side in ("right", "both"):
            if right_increment is None:
                return {"ok": False, "error": "right_increment missing"}

            if not self.right_servo_j.service_is_ready():
                return {"ok": False, "error": "right servo_j service not ready"}

            req = ServoMove.Request()
            req.pose = [float(x) for x in right_increment]
            req.speed = speed
            requests["right"] = {
                "client": self.right_servo_j,
                "request": req,
                "future": None,
            }

        if not requests:
            return {"ok": False, "error": f"invalid side for parallel servo_j: {side}"}

        t_send = time.time()

        # Dispatch all servo requests first.
        for sd, item in requests.items():
            item["future"] = item["client"].call_async(item["request"])

        results = {}

        # Wait after dispatch. The requests are already in flight.
        for sd, item in requests.items():
            resp = self.wait_future_result(item["future"], timeout=timeout)
            ok, ret, msg = self._servo_success(resp)

            results[sd] = {
                "ok": ok,
                "ret": ret,
                "message": msg,
                "pose_increment": list(item["request"].pose),
            }

        elapsed = time.time() - t_send
        overall_ok = all(v.get("ok", False) for v in results.values())

        return {
            "ok": overall_ok,
            "side": side,
            "elapsed_sec": elapsed,
            "results": results,
        }



    def _joint_delta(self, target, current):
        # สำหรับ joint ธรรมดาใช้ target-current ได้เลย
        return [float(target[i] - current[i]) for i in range(6)]


    def execute_object_servo_stream(
        self,
        name,
        target_object_pose,
        side="both",
        steps=120,
        interval=0.025,
        auto_enable=True,
        auto_disable=True,
    ):
        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        try:
            steps = int(steps)
            interval = float(interval)
        except Exception as e:
            return {"ok": False, "error": f"invalid steps/interval: {e}"}

        if steps < 10:
            return {"ok": False, "error": "steps too small; use at least 10"}
        if steps > 500:
            return {"ok": False, "error": "steps too large; max is 500"}
        if interval < 0.008:
            return {"ok": False, "error": "interval too small; min is 0.008 s"}
        if interval > 0.20:
            return {"ok": False, "error": "interval too large; max is 0.20 s"}

        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))
            motion_limit_check = self.validate_object_motion_limits(name, target_object_pose)
            if not motion_limit_check.get("ok", False):
                return {
                    **motion_limit_check,
                    "motion_limit_check": motion_limit_check,
                }

            freshness_check = self.validate_current_object_pose_fresh(name)
            if not freshness_check.get("ok", False):
                return freshness_check

            ik_joint_jump_check = self.validate_object_target_ik_jump(name, target_object_pose, side)
            if not ik_joint_jump_check.get("ok", False):
                return ik_joint_jump_check

            ok, msg = self.safe_state_ok(side)
            if not ok:
                return {"ok": False, "error": msg}

            start_pose = motion_limit_check["start_object_pose"]
            target_object_pose = motion_limit_check["target_object_pose"]
            object_delta = motion_limit_check["object_delta"]
            debug_fields = self._motion_debug_fields(
                start_pose,
                target_object_pose,
                object_delta,
                freshness_check,
            )
            debug_fields["motion_limit_check"] = motion_limit_check
            debug_fields["ik_joint_jump_check"] = ik_joint_jump_check
            translation_delta = motion_limit_check["translation_delta_mm"]
            rotation_delta = motion_limit_check["rotation_delta_rad"]

            # Current joints used as base for incremental servo_j
            if side in ("left", "both") and self.left_joint is None:
                return {"ok": False, "error": "left joint feedback missing"}
            if side in ("right", "both") and self.right_joint is None:
                return {"ok": False, "error": "right joint feedback missing"}

            left_base_joint = [float(x) for x in self.left_joint] if self.left_joint is not None else None
            right_base_joint = [float(x) for x in self.right_joint] if self.right_joint is not None else None

            # Precompute IK path before enabling servo
            left_ref = left_base_joint[:] if left_base_joint is not None else None
            right_ref = right_base_joint[:] if right_base_joint is not None else None
            path_joints = []

            for i in range(1, steps + 1):
                ratio = i / float(steps)
                step_pose = self._object_pose_lerp(start_pose, target_object_pose, ratio)
                preview = self.preview_object_targets(name, step_pose)

                if not preview.get("ok", False):
                    return {
                        "ok": False,
                        "error": "preview failed during precompute",
                        "failed_step": i,
                        "preview": preview,
                    }

                item = {
                    "index": i,
                    "object_pose": step_pose,
                    "left_joint": None,
                    "right_joint": None,
                }

                if side in ("left", "both"):
                    lj, ik_msg = self.solve_ik_for_tcp_target(
                        "left",
                        preview.get("left_target_tcp"),
                        ref_joint=left_ref,
                        timeout=1.0,
                    )
                    if lj is None:
                        return {
                            "ok": False,
                            "error": f"left IK failed at step {i}: {ik_msg}",
                            "preview": preview,
                        }
                    item["left_joint"] = lj
                    left_ref = lj[:]

                if side in ("right", "both"):
                    rj, ik_msg = self.solve_ik_for_tcp_target(
                        "right",
                        preview.get("right_target_tcp"),
                        ref_joint=right_ref,
                        timeout=1.0,
                    )
                    if rj is None:
                        return {
                            "ok": False,
                            "error": f"right IK failed at step {i}: {ik_msg}",
                            "preview": preview,
                        }
                    item["right_joint"] = rj
                    right_ref = rj[:]

                path_joints.append(item)

            enable_result = None
            disable_result = None
            stream_log = []
            timing_sample = []
            step_periods = []
            service_elapsed_values = []
            timing_overrun_count = 0
            max_timing_overrun_sec = 0.0
            stream_start = None
            stream_end = None

            self.sequence_cancel_requested = False
            self.motion_cancel_requested = False
            stop_generation_at_start = getattr(self, "stop_generation", 0)

            self.active_sequence = {
                "type": "object_servo_stream",
                "status": "running",
                "object": filename[:-5],
                "side": side,
                "steps": steps,
                "interval": interval,
                "current_index": 0,
                **debug_fields,
                "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            }

            def should_cancel():
                if self.sequence_cancel_requested:
                    return True
                if self.motion_cancel_requested:
                    return True
                if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                    return True
                return False

            try:
                if auto_enable:
                    enable_result = self.set_servo_enable(side, True)
                    if not enable_result.get("ok", False):
                        self.active_sequence = {
                            "type": "object_servo_stream",
                            "status": "error",
                            "error": "failed to enable servo",
                            "enable_result": enable_result,
                        }
                        return self.active_sequence
                    time.sleep(0.10)

                last_left = left_base_joint[:] if left_base_joint is not None else None
                last_right = right_base_joint[:] if right_base_joint is not None else None

                stream_start = time.monotonic()
                next_tick = stream_start

                for item in path_joints:
                    i = item["index"]
                    step_start = time.monotonic()

                    if should_cancel():
                        self.active_sequence = {
                            "type": "object_servo_stream",
                            "status": "cancelled",
                            "object": filename[:-5],
                            "cancelled_at_step": i,
                        }
                        return self.active_sequence

                    step_result = {"index": i, "ok": True}

                    left_inc = None
                    right_inc = None

                    if side in ("left", "both"):
                        left_inc = self._joint_delta(item["left_joint"], last_left)

                    if side in ("right", "both"):
                        right_inc = self._joint_delta(item["right_joint"], last_right)

                    service_start = time.monotonic()
                    parallel_result = self._call_servo_j_parallel(
                        side=side,
                        left_increment=left_inc,
                        right_increment=right_inc,
                        speed=[0.0] * 6,
                        timeout=0.5,
                    )
                    service_elapsed = time.monotonic() - service_start
                    service_elapsed_values.append(service_elapsed)

                    step_result["parallel"] = parallel_result
                    step_result["service_elapsed_sec"] = service_elapsed

                    if not parallel_result.get("ok", False):
                        self.active_sequence = {
                            "type": "object_servo_stream",
                            "status": "error",
                            "object": filename[:-5],
                            "failed_step": i,
                            "error": "parallel servo_j failed",
                            "result": parallel_result,
                        }
                        return self.active_sequence

                    if side in ("left", "both"):
                        step_result["left"] = parallel_result.get("results", {}).get("left")
                        last_left = item["left_joint"][:]

                    if side in ("right", "both"):
                        step_result["right"] = parallel_result.get("results", {}).get("right")
                        last_right = item["right_joint"][:]

                    if i <= 3 or i == steps:
                        stream_log.append(step_result)

                    self.active_sequence["current_index"] = i
                    self.active_sequence["progress"] = f"{i}/{steps}"
                    self.active_sequence["current_object_pose_target"] = item["object_pose"]

                    next_tick += interval
                    sleep_time = next_tick - time.monotonic()
                    overrun_sec = 0.0
                    if sleep_time > 0.0:
                        time.sleep(sleep_time)
                    else:
                        overrun_sec = abs(sleep_time)
                        timing_overrun_count += 1
                        max_timing_overrun_sec = max(max_timing_overrun_sec, overrun_sec)
                        sleep_time = 0.0

                    step_period = time.monotonic() - step_start
                    step_periods.append(step_period)

                    timing_item = {
                        "index": i,
                        "step_start_offset_sec": step_start - stream_start,
                        "service_elapsed_sec": service_elapsed,
                        "step_period_sec": step_period,
                        "sleep_sec": sleep_time,
                        "overrun_sec": overrun_sec,
                    }
                    if i <= 3 or i == steps:
                        timing_sample.append(timing_item)

                stream_end = time.monotonic()

                if auto_disable:
                    time.sleep(0.05)
                    disable_result = self.set_servo_enable(side, False)

                payload["current_object_pose"] = target_object_pose
                payload["current_pose_updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
                path.write_text(
                    json.dumps(payload, indent=2, ensure_ascii=False),
                    encoding="utf-8",
                )

                actual_duration_sec = (
                    float(stream_end - stream_start)
                    if stream_start is not None and stream_end is not None
                    else 0.0
                )
                timing_report = {
                    "actual_duration_sec": actual_duration_sec,
                    "duration_estimate_sec": steps * interval,
                    "average_step_period_sec": (
                        sum(step_periods) / len(step_periods)
                        if step_periods
                        else 0.0
                    ),
                    "min_step_period_sec": min(step_periods) if step_periods else 0.0,
                    "max_step_period_sec": max(step_periods) if step_periods else 0.0,
                    "average_service_elapsed_sec": (
                        sum(service_elapsed_values) / len(service_elapsed_values)
                        if service_elapsed_values
                        else 0.0
                    ),
                    "max_service_elapsed_sec": (
                        max(service_elapsed_values)
                        if service_elapsed_values
                        else 0.0
                    ),
                    "timing_overrun_count": timing_overrun_count,
                    "max_timing_overrun_sec": max_timing_overrun_sec,
                    "timing_mode": "fixed_rate_monotonic",
                    "timing_sample": timing_sample,
                }

                self.active_sequence = {
                    "type": "object_servo_stream",
                    "status": "done",
                    "object": filename[:-5],
                    "side": side,
                    "steps": steps,
                    "interval": interval,
                    "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "current_object_pose": target_object_pose,
                    **timing_report,
                }

                return {
                    "ok": True,
                    "message": "servo object stream completed",
                    "object": filename[:-5],
                    "side": side,
                    "steps": steps,
                    "interval": interval,
                    **timing_report,
                    **debug_fields,
                    "translation_delta": translation_delta,
                    "rotation_delta": rotation_delta,
                    "enable_result": enable_result,
                    "disable_result": disable_result,
                    "stream_log_sample": stream_log,
                    "note": "Smooth motion executed by servo_j incremental streaming. No joint_move wait between steps.",
                }

            finally:
                if auto_disable:
                    try:
                        self.set_servo_enable(side, False)
                    except Exception:
                        pass

        except Exception as e:
            try:
                if auto_disable:
                    self.set_servo_enable(side, False)
            except Exception:
                pass
            self.active_sequence = {
                "type": "object_servo_stream",
                "status": "exception",
                "error": str(e),
            }
            return {"ok": False, "error": str(e)}



    def set_current_object_pose_file(self, name, current_object_pose):
        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))
            current_object_pose = self._validate_pose6(current_object_pose, "current_object_pose")

            payload["current_object_pose"] = current_object_pose
            payload["current_pose_updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")

            path.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )

            return {
                "ok": True,
                "message": "current object pose updated",
                "file": filename,
                "current_object_pose": current_object_pose,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def _object_pose_lerp(self, start_pose, target_pose, ratio):
        start_pose = self._validate_pose6(start_pose, "start_pose")
        target_pose = self._validate_pose6(target_pose, "target_pose")
        r = float(ratio)

        out = []
        for i in range(3):
            out.append(float(start_pose[i] + (target_pose[i] - start_pose[i]) * r))

        for i in range(3, 6):
            d = self._wrap_angle(target_pose[i] - start_pose[i])
            out.append(float(self._wrap_angle(start_pose[i] + d * r)))

        return out


    def execute_object_move_interpolated(
        self,
        name,
        target_object_pose,
        side="both",
        vel=None,
        acc=None,
        steps=10,
        delay=0.0,
    ):
        side = (side or "both").lower().strip()
        if side not in ("left", "right", "both"):
            return {"ok": False, "error": f"invalid side: {side}"}

        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))

            motion_limit_check = self.validate_object_motion_limits(name, target_object_pose)
            if not motion_limit_check.get("ok", False):
                return {
                    **motion_limit_check,
                    "motion_limit_check": motion_limit_check,
                }

            freshness_check = self.validate_current_object_pose_fresh(name)
            if not freshness_check.get("ok", False):
                return freshness_check

            ik_joint_jump_check = self.validate_object_target_ik_jump(name, target_object_pose, side)
            if not ik_joint_jump_check.get("ok", False):
                return ik_joint_jump_check

            if self.active_sequence is not None and self.active_sequence.get("status") in ("running", "delay"):
                return {
                    "ok": False,
                    "error": "sequence/program/object motion already running",
                    "active_sequence": self.active_sequence,
                    "motion_limit_check": motion_limit_check,
                    "ik_joint_jump_check": ik_joint_jump_check,
                }

            start_pose = motion_limit_check["start_object_pose"]
            target_object_pose = motion_limit_check["target_object_pose"]
            object_delta = motion_limit_check["object_delta"]
            debug_fields = self._motion_debug_fields(
                start_pose,
                target_object_pose,
                object_delta,
                freshness_check,
            )
            debug_fields["motion_limit_check"] = motion_limit_check
            debug_fields["ik_joint_jump_check"] = ik_joint_jump_check

            steps = int(steps)
            if steps < 1:
                steps = 1
            if steps > 100:
                return {"ok": False, "error": "steps too large; max is 100"}

            delay = max(0.0, float(delay or 0.0))

            ok, msg = self.safe_state_ok(side)
            if not ok:
                return {"ok": False, "error": msg}

            self.sequence_cancel_requested = False
            self.motion_cancel_requested = False
            stop_generation_at_start = getattr(self, "stop_generation", 0)

            self.active_sequence = {
                "type": "object_interpolated_move",
                "status": "running",
                "object": filename[:-5],
                "side": side,
                "steps": steps,
                "current_index": 0,
                **debug_fields,
                "vel": vel,
                "acc": acc,
                "delay": delay,
                "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "stop_generation": stop_generation_at_start,
            }

            th = threading.Thread(
                target=self._object_interpolated_thread,
                args=(filename, start_pose, target_object_pose, side, vel, acc, steps, delay, stop_generation_at_start),
                daemon=True,
            )
            th.start()

            return {
                "ok": True,
                "accepted": True,
                "message": "interpolated object move started",
                "object": filename[:-5],
                "side": side,
                "steps": steps,
                **debug_fields,
                "vel": vel,
                "acc": acc,
                "delay": delay,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def _object_interpolated_thread(
        self,
        filename,
        start_pose,
        target_pose,
        side,
        vel,
        acc,
        steps,
        delay,
        stop_generation_at_start,
    ):
        def should_cancel():
            if self.sequence_cancel_requested:
                return True
            if self.motion_cancel_requested:
                return True
            if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                return True
            return False

        try:
            name = filename[:-5]

            for i in range(1, steps + 1):
                if should_cancel():
                    self.active_sequence = {
                        "type": "object_interpolated_move",
                        "status": "cancelled",
                        "object": name,
                        "cancelled_at_step": i,
                        "reason": "cancel requested before step",
                    }
                    return

                ratio = i / float(steps)
                step_pose = self._object_pose_lerp(start_pose, target_pose, ratio)

                if self.active_sequence is not None:
                    self.active_sequence["status"] = "running"
                    self.active_sequence["current_index"] = i
                    self.active_sequence["progress"] = f"{i}/{steps}"
                    self.active_sequence["current_object_pose_target"] = step_pose

                preview = self.preview_object_targets(name, step_pose)
                if not preview.get("ok", False):
                    self.active_sequence = {
                        "type": "object_interpolated_move",
                        "status": "error",
                        "object": name,
                        "failed_step": i,
                        "error": preview,
                    }
                    return

                left_joint = None
                right_joint = None

                if side in ("left", "both"):
                    left_joint, msg = self.solve_ik_for_tcp_target(
                        "left",
                        preview.get("left_target_tcp"),
                        ref_joint=self.left_joint,
                        timeout=1.0,
                    )
                    if left_joint is None:
                        self.active_sequence = {
                            "type": "object_interpolated_move",
                            "status": "error",
                            "object": name,
                            "failed_step": i,
                            "error": msg,
                            "preview": preview,
                        }
                        return

                if side in ("right", "both"):
                    right_joint, msg = self.solve_ik_for_tcp_target(
                        "right",
                        preview.get("right_target_tcp"),
                        ref_joint=self.right_joint,
                        timeout=1.0,
                    )
                    if right_joint is None:
                        self.active_sequence = {
                            "type": "object_interpolated_move",
                            "status": "error",
                            "object": name,
                            "failed_step": i,
                            "error": msg,
                            "preview": preview,
                        }
                        return

                result = self.move_both_joint(
                    left_target=left_joint,
                    right_target=right_joint,
                    side=side,
                    vel=vel,
                    acc=acc,
                )

                if not result.get("ok", False):
                    self.active_sequence = {
                        "type": "object_interpolated_move",
                        "status": "error",
                        "object": name,
                        "failed_step": i,
                        "error": result,
                    }
                    return

                done, msg = self.wait_motion_done(
                    side=side,
                    timeout=120.0,
                    stop_generation_at_start=stop_generation_at_start,
                )

                if not done:
                    self.active_sequence = {
                        "type": "object_interpolated_move",
                        "status": "cancelled" if msg == "cancelled" else "error",
                        "object": name,
                        "failed_step": i,
                        "reason": msg,
                    }
                    return

                if delay > 0:
                    t0 = time.time()
                    while time.time() - t0 < delay:
                        if should_cancel():
                            self.active_sequence = {
                                "type": "object_interpolated_move",
                                "status": "cancelled",
                                "object": name,
                                "cancelled_at_step": i,
                                "reason": "cancel requested during delay",
                            }
                            return
                        time.sleep(0.03)

            # update current_object_pose after successful completion
            path = self._objects_dir() / filename
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["current_object_pose"] = target_pose
            payload["current_pose_updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            path.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )

            self.active_sequence = {
                "type": "object_interpolated_move",
                "status": "done",
                "object": name,
                "steps": steps,
                "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "current_object_pose": target_pose,
            }

        except Exception as e:
            self.active_sequence = {
                "type": "object_interpolated_move",
                "status": "exception",
                "error": str(e),
            }



    def list_object_frames(self):
        try:
            d = self._objects_dir()
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

            return {"ok": True, "objects": items}

        except Exception as e:
            return {"ok": False, "error": str(e), "objects": []}


    def load_object_frame(self, name):
        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            payload = json.loads(path.read_text(encoding="utf-8"))
            return {
                "ok": True,
                "message": "object frame loaded",
                **payload,
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}


    def delete_object_frame(self, name):
        try:
            filename = self._safe_object_filename(name)
            path = self._objects_dir() / filename

            if not path.exists():
                return {"ok": False, "error": f"object frame not found: {filename}"}

            path.unlink()
            return {"ok": True, "message": "object frame deleted", "file": filename}

        except Exception as e:
            return {"ok": False, "error": str(e)}



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



    def run_program(self, steps, side="both", vel=None, acc=None):
        if not steps:
            return {"ok": False, "error": "program is empty"}

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
        }

        th = threading.Thread(
            target=self._program_thread,
            args=(normalized_steps, stop_generation_at_start),
            daemon=True,
        )
        th.start()

        return {
            "ok": True,
            "accepted": True,
            "message": "program started",
            "steps": normalized_steps,
            "stop_generation": stop_generation_at_start,
        }


    def _program_thread(self, steps, stop_generation_at_start=0):
        def should_cancel():
            if self.sequence_cancel_requested:
                return True
            if self.motion_cancel_requested:
                return True
            if getattr(self, "stop_generation", 0) != stop_generation_at_start:
                return True
            return False

        try:
            for i, step in enumerate(steps):
                if should_cancel():
                    self.active_sequence = {
                        "type": "program",
                        "status": "cancelled",
                        "cancelled_at_index": i,
                        "steps": steps,
                        "reason": "cancel requested before step",
                    }
                    return

                name = step["name"]
                step_side = step.get("side", "both")
                step_vel = step.get("vel", None)
                step_acc = step.get("acc", None)
                step_delay = float(step.get("delay", 0.0) or 0.0)

                if self.active_sequence is not None:
                    self.active_sequence["type"] = "program"
                    self.active_sequence["status"] = "running"
                    self.active_sequence["current_index"] = i
                    self.active_sequence["current_name"] = name
                    self.active_sequence["current_step"] = step
                    self.active_sequence["progress"] = f"{i + 1}/{len(steps)}"

                result = self.run_waypoint(
                    name,
                    side=step_side,
                    vel=step_vel,
                    acc=step_acc,
                )

                if not result.get("ok", False):
                    self.active_sequence = {
                        "type": "program",
                        "status": "error",
                        "failed_index": i,
                        "failed_name": name,
                        "steps": steps,
                        "result": result,
                    }
                    return

                done, msg = self.wait_motion_done(
                    side=step_side,
                    timeout=120.0,
                    stop_generation_at_start=stop_generation_at_start,
                )

                if not done:
                    self.active_sequence = {
                        "type": "program",
                        "status": "cancelled" if msg == "cancelled" else "error",
                        "failed_index": i,
                        "failed_name": name,
                        "steps": steps,
                        "reason": msg,
                    }
                    return

                if step_delay > 0:
                    if self.active_sequence is not None:
                        self.active_sequence["status"] = "delay"
                        self.active_sequence["delay_sec"] = step_delay

                    delay_start = time.time()
                    while time.time() - delay_start < step_delay:
                        if should_cancel():
                            self.active_sequence = {
                                "type": "program",
                                "status": "cancelled",
                                "cancelled_at_index": i,
                                "steps": steps,
                                "reason": "cancel requested during delay",
                            }
                            return
                        time.sleep(0.05)

            self.active_sequence = {
                "type": "program",
                "status": "done",
                "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "steps": steps,
                "count": len(steps),
            }

        except Exception as e:
            self.active_sequence = {
                "type": "program",
                "status": "exception",
                "error": str(e),
                "steps": steps,
            }


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

_D33_MOTION_EXACT_PATHS = {
    "/api/object/execute_move",
    "/api/object/execute_interpolated",
    "/api/object/servo_stream",
    "/api/object/execute_calibrated_world_rigid",
    "/api/object/execute_calibrated_world_rigid_large",
}

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
    "/api/object/list",
    "/api/object/load",
    "/api/object/validate_motion",
    "/api/object/validate_ik_jump",
    "/api/object/preview_targets",
    "/api/object/sync_current_from_live",
    "/api/world_calibration",
    "/api/object/preview_calibrated_rigid_compare_world",
    "/api/object/validate_calibrated_world_ik",
    "/api/object/verify_pending_calibrated_execute_commit",
    "/api/object/commit_pending_calibrated_execute_pose",
    "/api/object/pending_calibrated_execute_commit_status",
    "/api/object/recover_pending_calibrated_execute_commit",
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


class ObjectSyncCurrentFromLiveRequest(BaseModel):
    name: str

@app.get("/")
def index():
    return FileResponse(str(Path.home() / "jaka_ws/dual_arm_app/web/index.html"))


@app.get("/api/status")
def api_status():
    return node.status()


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









@app.post("/api/servo/enable")
def api_servo_enable(req: ServoEnableRequest):
    return node.set_servo_enable(req.side, req.enable)


@app.post("/api/servo/tiny_test")
def api_servo_tiny_test(req: ServoTinyTestRequest):
    return node.servo_tiny_test(
        req.side,
        req.joint_index,
        req.delta,
        req.repeats,
        req.interval,
        req.auto_enable,
        req.auto_disable,
    )


@app.post("/api/object/servo_stream")
def api_object_servo_stream(req: ObjectServoStreamRequest):
    return node.execute_object_servo_stream(
        req.name,
        req.target_object_pose,
        req.side,
        req.steps,
        req.interval,
        req.auto_enable,
        req.auto_disable,
    )


@app.post("/api/object/sync_current_from_live")
def api_object_sync_current_from_live(req: ObjectSyncCurrentFromLiveRequest):
    return node.sync_current_object_pose_from_live_tcp(req.name)

@app.post("/api/object/set_current_pose")
def api_object_set_current_pose(req: ObjectSetCurrentPoseRequest):
    return node.set_current_object_pose_file(req.name, req.current_object_pose)


@app.post("/api/object/execute_interpolated")
def api_object_execute_interpolated(req: ObjectExecuteInterpolatedRequest):
    return node.execute_object_move_interpolated(
        req.name,
        req.target_object_pose,
        req.side,
        req.vel,
        req.acc,
        req.steps,
        req.delay,
    )

@app.post("/api/object/execute_move")
def api_object_execute_move(req: ObjectExecuteMoveRequest):
    return node.execute_object_move(
        req.name,
        req.target_object_pose,
        req.side,
        req.vel,
        req.acc,
    )

@app.post("/api/object/validate_motion")
def api_object_validate_motion(req: ObjectValidateMotionRequest):
    return node.validate_object_motion_request(req.name, req.target_object_pose)

@app.post("/api/object/validate_ik_jump")
def api_object_validate_ik_jump(req: ObjectValidateIkJumpRequest):
    return node.validate_object_target_ik_jump(req.name, req.target_object_pose, req.side)

@app.post("/api/object/preview_targets")
def api_object_preview_targets(req: ObjectPreviewRequest):
    return node.preview_object_targets(req.name, req.target_object_pose)

@app.post("/api/object/preview_targets_rigid_compare")
def api_object_preview_targets_rigid_compare(req: ObjectPreviewRequest):
    return node.preview_object_targets_rigid_compare(req.name, req.target_object_pose)

@app.post("/api/object/calibration/capture_point")
def api_object_calibration_capture_point(req: ObjectCalibrationCapturePointRequest):
    return node.capture_object_calibration_point(req.name, req.point_type, req.source)

@app.post("/api/object/calibration/compute_frame")
def api_object_calibration_compute_frame(req: ObjectNameRequest):
    return node.compute_object_calibration_frame(req.name)

@app.post("/api/object/calibration/status")
def api_object_calibration_status(req: ObjectNameRequest):
    return node.object_calibration_status(req.name)

@app.post("/api/object/preview_calibrated_axes")
def api_object_preview_calibrated_axes(req: ObjectCalibrationAxesPreviewRequest):
    return node.preview_calibrated_axes(req.name, req.axis_length_mm)

@app.post("/api/object/preview_calibrated_rigid_compare")
def api_object_preview_calibrated_rigid_compare(req: ObjectCalibratedRigidCompareRequest):
    return node.preview_calibrated_rigid_compare(req.name, req.target_delta)

@app.post("/api/world_calibration/capture_point")
def api_world_calibration_capture_point(req: WorldCalibrationCapturePointRequest):
    return node.world_calibration_capture_point(req.point_type, req.side)

@app.post("/api/world_calibration/compute_transform")
def api_world_calibration_compute_transform():
    return node.world_calibration_compute_transform()

@app.post("/api/world_calibration/status")
def api_world_calibration_status():
    return node.world_calibration_status()

@app.post("/api/world_calibration/preview_live")
def api_world_calibration_preview_live():
    return node.world_calibration_preview_live()

@app.post("/api/object/preview_calibrated_rigid_compare_world")
def api_object_preview_calibrated_rigid_compare_world(req: ObjectCalibratedRigidCompareRequest):
    return node.preview_calibrated_rigid_compare_world(req.name, req.target_delta)

@app.post("/api/object/validate_calibrated_world_ik")
def api_object_validate_calibrated_world_ik(req: ObjectCalibratedWorldIkValidateRequest):
    return node.validate_calibrated_world_ik(
        req.name,
        req.target_delta,
        req.max_joint_delta_rad,
        req.max_joint_delta_sum_rad,
    )

@app.post("/api/object/execute_calibrated_world_rigid")
def api_object_execute_calibrated_world_rigid(req: ObjectCalibratedWorldRigidExecuteRequest):
    return node.execute_calibrated_world_rigid(
        req.name,
        req.target_delta,
        req.dry_run,
        req.confirm_execute,
        req.max_translation_mm,
        req.max_rotation_rad,
        req.max_joint_delta_rad,
        req.max_joint_delta_sum_rad,
        req.speed_scale,
    )

@app.post("/api/object/execute_calibrated_world_rigid_large")
def api_object_execute_calibrated_world_rigid_large(req: ObjectCalibratedWorldRigidLargeExecuteRequest):
    return node.execute_calibrated_world_rigid_large(
        req.name,
        req.target_delta,
        req.dry_run,
        req.confirm_execute,
        req.max_translation_mm,
        req.max_rotation_rad,
        req.max_segment_translation_mm,
        req.max_segment_rotation_rad,
        req.max_segment_tcp_rotation_rad,
        req.max_joint_delta_rad_per_segment,
        req.max_joint_delta_sum_rad_per_segment,
        req.speed_scale,
        req.auto_commit,
    )

@app.post("/api/object/pending_calibrated_execute_commit_status")
def api_object_pending_calibrated_execute_commit_status(req: ObjectPendingCalibratedExecuteCommitStatusRequest):
    return node.pending_calibrated_execute_commit_status(req.name)

@app.post("/api/object/verify_pending_calibrated_execute_commit")
def api_object_verify_pending_calibrated_execute_commit(req: ObjectVerifyPendingCalibratedExecuteCommitRequest):
    return node.verify_pending_calibrated_execute_commit(
        req.name,
        req.position_tolerance_mm,
        req.orientation_tolerance_rad,
    )

@app.post("/api/object/commit_pending_calibrated_execute_pose")
def api_object_commit_pending_calibrated_execute_pose(req: ObjectCommitPendingCalibratedExecutePoseRequest):
    return node.commit_pending_calibrated_execute_pose(
        req.name,
        req.confirm_commit,
        req.position_tolerance_mm,
        req.orientation_tolerance_rad,
    )

@app.post("/api/object/recover_pending_calibrated_execute_commit")
def api_object_recover_pending_calibrated_execute_commit(req: ObjectRecoverPendingCalibratedExecuteCommitRequest):
    return node.recover_pending_calibrated_execute_commit(
        req.name,
        req.target_delta,
        req.confirm_recover,
        req.position_tolerance_mm,
        req.orientation_tolerance_rad,
    )

@app.get("/api/object/list")
def api_object_list():
    return node.list_object_frames()


@app.post("/api/object/capture_current")
def api_object_capture_current(req: ObjectFrameCaptureRequest):
    return node.capture_object_frame_current(req.name, req.object_pose)


@app.post("/api/object/load")
def api_object_load(req: ObjectNameRequest):
    return node.load_object_frame(req.name)


@app.post("/api/object/delete")
def api_object_delete(req: ObjectNameRequest):
    return node.delete_object_frame(req.name)

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
    return node.run_program(req.steps, req.side, req.vel, req.acc)

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
