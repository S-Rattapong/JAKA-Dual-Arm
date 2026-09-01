"""PRE-P5.A3 one-time three-point physical World calibration. Robot reads only."""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from jaka_msgs.srv import GetFrameState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

from dual_arm_app.backend.physical_world_calibration import (
    METHOD, WORLD_ORIGIN_SEMANTICS, build_physical_calibration_payload,
)
from dual_arm_app.backend.world_frame_calibration import load_world_calibration

ROOT = Path(__file__).resolve().parents[2]
CAPTURE_PATH = ROOT / "dual_arm_app/config/world_frame_calibration_capture.json"
CALIBRATION_PATH = ROOT / "dual_arm_app/config/world_frame_calibration.json"
CANDIDATE_PATH = ROOT / "dual_arm_app/config/world_frame_calibration_candidate.json"
SAMPLE_COUNT = 10
MAX_JOINT_SPREAD_RAD = 0.002
MAX_ORIENTATION_DRIFT_RAD = 0.01
MIN_AXIS_BASELINE_M = 0.10
MIN_Y_ORTHOGONAL_M = 0.08
POINT_LABELS = ("origin", "x_plus", "y_plus")

def _call(node, client, request, timeout_s=4.0):
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"service call failed or timed out: {client.srv_name}")
    return future.result()


def _stable_joint_capture(node, topic):
    samples = []
    def callback(msg):
        if len(msg.position) >= 6:
            samples.append([float(v) for v in msg.position[:6]])
    sub = node.create_subscription(JointState, topic, callback, 10)
    deadline = time.monotonic() + 5.0
    while len(samples) < SAMPLE_COUNT and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if len(samples) < SAMPLE_COUNT:
        raise RuntimeError(f"not enough fresh joint samples on {topic}: {len(samples)}")
    spread = [max(s[j] for s in samples) - min(s[j] for s in samples) for j in range(6)]
    if max(spread) > MAX_JOINT_SPREAD_RAD:
        raise RuntimeError(f"robot not stationary enough; max joint spread={max(spread):.6f} rad")
    median = [statistics.median(s[j] for s in samples) for j in range(6)]
    return median, spread

def _frame_state(node, side):
    client = node.create_client(GetFrameState, f"/{side}_jaka_driver/get_frame_state")
    response = _call(node, client, GetFrameState.Request())
    if int(response.ret) != 1:
        raise RuntimeError(f"{side} frame-state read failed: {response.message}")
    tool_pose = [float(v) for v in response.tool_pose]
    user_pose = [float(v) for v in response.user_frame_pose]
    installation = [float(v) for v in response.installation_rpy]
    if int(response.tool_id) != 0 or int(response.user_frame_id) != 0:
        raise RuntimeError(f"{side} requires Tool0/User0 for A3 capture")
    if any(abs(v) > 1e-6 for v in tool_pose + user_pose + installation):
        raise RuntimeError(f"{side} Tool0/User0/installation RPY must remain zero")
    return {"tool_id": int(response.tool_id), "tool_pose": tool_pose,
            "user_frame_id": int(response.user_frame_id), "user_frame_pose": user_pose,
            "installation_rpy": installation}


def _moveit_j6_in_base(node, side, joints):
    client = node.create_client(GetPositionFK, "/compute_fk")
    request = GetPositionFK.Request()
    request.header.frame_id = f"{side}_base_link"
    request.fk_link_names = [f"{side}_J6"]
    state = RobotState()
    state.joint_state.name = [f"{side}_joint_{i}" for i in range(1, 7)]
    state.joint_state.position = [float(v) for v in joints]
    request.robot_state = state
    response = _call(node, client, request)
    if int(response.error_code.val) != 1 or len(response.pose_stamped) != 1:
        raise RuntimeError(f"MoveIt FK failed for {side}: error={response.error_code.val}")
    pose = response.pose_stamped[0].pose
    return {"translation_m": [pose.position.x, pose.position.y, pose.position.z],
            "quaternion_xyzw": [pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w]}

def _empty_evidence():
    return {"schema_version": 2, "method": METHOD,
            "world_origin_semantics": WORLD_ORIGIN_SEMANTICS,
            "captures": {"left": {}, "right": {}}}


def _load_capture_file():
    if not CAPTURE_PATH.exists():
        return _empty_evidence()
    data = json.loads(CAPTURE_PATH.read_text(encoding="utf-8"))
    if data.get("schema_version") == 2 and data.get("method") == METHOD:
        return data
    if data.get("schema_version") == 1 and data.get("method") == "ONE_POINT_VIRTUAL_J6_TOUCH":
        migrated = _empty_evidence()
        old = data.get("captures", {})
        for side in ("left", "right"):
            if side in old:
                migrated["captures"][side]["origin"] = old[side]
        migrated["migration_note"] = "Schema-1 one-point capture preserved as Origin for three-point A3."
        return migrated
    raise RuntimeError("existing A3 capture evidence has incompatible schema/method")


def _quat_angle(a, b):
    na = math.sqrt(sum(float(x)*float(x) for x in a))
    nb = math.sqrt(sum(float(x)*float(x) for x in b))
    if na <= 0.0 or nb <= 0.0:
        raise RuntimeError("invalid zero quaternion in FK evidence")
    dot = abs(sum(float(a[i])*float(b[i]) for i in range(4)) / (na*nb))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def capture(side, label):
    rclpy.init()
    node = rclpy.create_node(f"pre_p5_a3_capture_{side}_{label}")
    try:
        joints, spread = _stable_joint_capture(node, f"/{side}_jaka_driver/joint_position")
        frame = _frame_state(node, side)
        fk = _moveit_j6_in_base(node, side, joints)
        evidence = _load_capture_file()
        evidence["captures"].setdefault(side, {})[label] = {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "joint_rad": joints, "joint_spread_rad": spread,
            "frame_state": frame, "j6_in_model_base": fk,
        }
        CAPTURE_PATH.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
        print(f"CAPTURE_{side.upper()}_{label.upper()}_PASS")
        print("j6_in_base_m=", [round(v, 9) for v in fk["translation_m"]])
        print("max_joint_spread_rad=", max(spread))
    finally:
        node.destroy_node()
        rclpy.shutdown()

def _points_for_side(captures, side):
    side_caps = captures.get(side, {})
    missing = [label for label in POINT_LABELS if label not in side_caps]
    if missing:
        raise RuntimeError(f"{side} is missing captures: {missing}")
    origin = side_caps["origin"]["j6_in_model_base"]
    for label in ("x_plus", "y_plus"):
        angle = _quat_angle(origin["quaternion_xyzw"], side_caps[label]["j6_in_model_base"]["quaternion_xyzw"])
        if angle > MAX_ORIENTATION_DRIFT_RAD:
            raise RuntimeError(
                f"{side} J6 orientation drift Origin->{label} is {angle:.6f} rad; "
                f"limit is {MAX_ORIENTATION_DRIFT_RAD:.6f} rad"
            )
    return {label: side_caps[label]["j6_in_model_base"]["translation_m"] for label in POINT_LABELS}


def solve_candidate():
    evidence = _load_capture_file()
    captures = evidence.get("captures", {})
    left_points = _points_for_side(captures, "left")
    right_points = _points_for_side(captures, "right")
    current_raw = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    solved, diagnostics = build_physical_calibration_payload(current_raw, left_points, right_points)
    for side, diag in diagnostics.items():
        if diag["origin_to_x_m"] < MIN_AXIS_BASELINE_M or diag["origin_to_y_m"] < MIN_AXIS_BASELINE_M:
            raise RuntimeError(f"{side} O->X/O->Y baseline is too short: {diag}")
        if diag["y_orthogonal_component_m"] < MIN_Y_ORTHOGONAL_M:
            raise RuntimeError(f"{side} X/Y geometry is too close to collinear: {diag}")
    CANDIDATE_PATH.write_text(json.dumps(solved, indent=2) + "\n", encoding="utf-8")
    load_world_calibration(CANDIDATE_PATH)
    evidence["candidate_revision"] = solved["revision"]
    evidence["candidate_diagnostics"] = diagnostics
    evidence["candidate_transforms"] = solved["transforms"]
    CAPTURE_PATH.write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    print("SOLVE_CANDIDATE_PASS")
    print("revision=", solved["revision"])
    print("diagnostics=", json.dumps(diagnostics, sort_keys=True))
    print("candidate=", CANDIDATE_PATH)

def commit_candidate():
    candidate = json.loads(CANDIDATE_PATH.read_text(encoding="utf-8"))
    load_world_calibration(CANDIDATE_PATH)
    current = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    backup = CALIBRATION_PATH.with_name(
        f"world_frame_calibration.before_{current['revision'].split(':',1)[-1][:12]}.json"
    )
    if not backup.exists():
        backup.write_text(json.dumps(current, indent=2) + "\n", encoding="utf-8")
    CALIBRATION_PATH.write_text(json.dumps(candidate, indent=2) + "\n", encoding="utf-8")
    loaded = load_world_calibration(CALIBRATION_PATH)
    print("COMMIT_PASS")
    print("revision=", loaded.revision)
    print("backup=", backup)


def main():
    parser = argparse.ArgumentParser()
    choices = [f"capture-{side}-{label}" for side in ("left", "right") for label in POINT_LABELS]
    choices += ["solve-candidate", "commit-candidate"]
    parser.add_argument("action", choices=choices)
    args = parser.parse_args()
    if args.action.startswith("capture-"):
        _, side, label = args.action.split("-", 2)
        capture(side, label)
    elif args.action == "solve-candidate":
        solve_candidate()
    else:
        commit_candidate()


if __name__ == "__main__":
    main()
