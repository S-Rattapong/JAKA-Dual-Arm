"""Pure offline EXP-2 rigid-grasp preservation analysis.

Consumes persisted EXP-0 evidence plus the calibrated Web URDF.  This module has
no ROS, SDK, motion, fault, or safety authority.  Encoder+URDF FK is model-based
and is explicitly not external metrology.
"""
from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
import math
from bisect import bisect_left
from pathlib import Path
import xml.etree.ElementTree as ET
from typing import Any

EXP2_SCHEMA_VERSION = "jaka-dual-arm-exp2-rigid-grasp/v1"
EXP2_SEMANTIC = (
    "ANALYSIS ONLY — ENCODER+URDF MODEL BASED — NOT EXTERNAL METROLOGY — "
    "NOT PHASE6 TIMING, FAULT, OR SAFETY AUTHORITY"
)
CENTER_SIGN = "CENTER_FROM_LEFT - CENTER_FROM_RIGHT"
SIDES = ("left", "right")
TERMINAL_LINKS = {"left": "left_J6", "right": "right_J6"}
JOINT_NAMES = {
    side: tuple(f"{side}_joint_{index}" for index in range(1, 7))
    for side in SIDES
}


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _vector(value: Any, count: int) -> list[float] | None:
    if isinstance(value, (str, bytes, Mapping)) or not isinstance(value, Sequence) or len(value) != count:
        return None
    checked = [_finite(item) for item in value]
    return None if any(item is None for item in checked) else [float(item) for item in checked]


def _identity() -> list[list[float]]:
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def _matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[math.fsum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rpy_rotation(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    # Canonical ROS/URDF fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _transform(xyz: Sequence[float], rpy: Sequence[float]) -> list[list[float]]:
    matrix = _identity()
    rotation = _rpy_rotation(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    for i in range(3):
        for j in range(3):
            matrix[i][j] = rotation[i][j]
        matrix[i][3] = float(xyz[i])
    return matrix


def _axis_motion(axis: Sequence[float], value: float, joint_type: str) -> list[list[float]]:
    x, y, z = (float(axis[0]), float(axis[1]), float(axis[2]))
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-15:
        raise ValueError("URDF joint axis must be non-zero")
    x, y, z = x / norm, y / norm, z / norm
    matrix = _identity()
    if joint_type in {"revolute", "continuous"}:
        c, s, v = math.cos(value), math.sin(value), 1.0 - math.cos(value)
        rotation = [
            [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
        ]
        for i in range(3):
            for j in range(3):
                matrix[i][j] = rotation[i][j]
    elif joint_type == "prismatic":
        matrix[0][3], matrix[1][3], matrix[2][3] = x * value, y * value, z * value
    return matrix


def _inverse_rigid(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    result = _identity()
    for i in range(3):
        for j in range(3):
            result[i][j] = float(matrix[j][i])
    translation = [float(matrix[i][3]) for i in range(3)]
    for i in range(3):
        result[i][3] = -math.fsum(result[i][j] * translation[j] for j in range(3))
    return result


def _translation(matrix: Sequence[Sequence[float]]) -> list[float]:
    return [float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3])]


def _rotation_angle(reference: Sequence[Sequence[float]], actual: Sequence[Sequence[float]]) -> float:
    # R_err = R_ref^T * R_actual, no Euler-angle subtraction.
    trace = 0.0
    for i in range(3):
        trace += math.fsum(reference[k][i] * actual[k][i] for k in range(3))
    cosine = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.acos(cosine)


def _pose_matrix(pose: Any) -> list[list[float]] | None:
    if not isinstance(pose, Mapping):
        return None
    xyz = _vector(pose.get("translation_m"), 3)
    rpy = _vector(pose.get("rpy_rad"), 3)
    return _transform(xyz, rpy) if xyz is not None and rpy is not None else None


def _location(sample: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "time_from_start_s": sample.get("time_from_start_s"),
        "observation_index": sample.get("observation_index"),
    }


def _signed_metric(values: list[tuple[float, dict[str, Any]]]) -> dict[str, Any]:
    if not values:
        return {"sample_count": 0, "mean": None, "mae": None, "rmse": None, "max_abs": None, "max_abs_location": {"time_from_start_s": None, "observation_index": None}}
    scalars = [item[0] for item in values]
    mean = math.fsum(scalars) / len(scalars)
    mae = math.fsum(abs(value) for value in scalars) / len(scalars)
    rmse = math.sqrt(math.fsum(value * value for value in scalars) / len(scalars))
    maximum = max(values, key=lambda item: abs(item[0]))
    return {"sample_count": len(values), "mean": mean, "mae": mae, "rmse": rmse, "max_abs": abs(maximum[0]), "max_abs_location": dict(maximum[1])}


def _magnitude_metric(values: list[tuple[float, dict[str, Any]]], *, radians: bool = False) -> dict[str, Any]:
    if not values:
        result = {"sample_count": 0, "mean": None, "rms": None, "max": None, "max_location": {"time_from_start_s": None, "observation_index": None}}
    else:
        scalars = [item[0] for item in values]
        maximum = max(values, key=lambda item: item[0])
        result = {"sample_count": len(values), "mean": math.fsum(scalars) / len(scalars), "rms": math.sqrt(math.fsum(value * value for value in scalars) / len(scalars)), "max": maximum[0], "max_location": dict(maximum[1])}
    if radians:
        for key in ("mean", "rms", "max"):
            result[f"{key}_deg"] = math.degrees(result[key]) if result[key] is not None else None
    return result


class UrdfFkModel:
    """Small deterministic URDF FK reader sufficient for the calibrated Web model."""

    def __init__(self, urdf_path: str | Path):
        self.path = Path(urdf_path).resolve()
        root = ET.parse(self.path).getroot()
        self.joints_by_child: dict[str, dict[str, Any]] = {}
        for joint in root.findall("joint"):
            name, joint_type = joint.get("name"), joint.get("type", "fixed")
            parent_node, child_node = joint.find("parent"), joint.find("child")
            if not name or parent_node is None or child_node is None:
                continue
            parent, child = parent_node.get("link"), child_node.get("link")
            if not parent or not child:
                continue
            origin = joint.find("origin")
            xyz = [float(item) for item in (origin.get("xyz", "0 0 0").split() if origin is not None else [0, 0, 0])]
            rpy = [float(item) for item in (origin.get("rpy", "0 0 0").split() if origin is not None else [0, 0, 0])]
            axis_node = joint.find("axis")
            axis = [float(item) for item in (axis_node.get("xyz", "1 0 0").split() if axis_node is not None else [1, 0, 0])]
            if len(xyz) != 3 or len(rpy) != 3 or len(axis) != 3 or not all(math.isfinite(item) for item in xyz + rpy + axis):
                raise ValueError(f"Invalid URDF joint transform: {name}")
            self.joints_by_child[child] = {"name": name, "type": joint_type, "parent": parent, "origin": _transform(xyz, rpy), "axis": axis}

    def fk(self, tip_link: str, joint_positions: Mapping[str, float]) -> list[list[float]]:
        chain = []
        link = tip_link
        seen = set()
        while link in self.joints_by_child:
            if link in seen:
                raise ValueError("URDF kinematic cycle")
            seen.add(link)
            joint = self.joints_by_child[link]
            chain.append(joint)
            link = joint["parent"]
        matrix = _identity()
        for joint in reversed(chain):
            matrix = _matmul(matrix, joint["origin"])
            if joint["type"] in {"revolute", "continuous", "prismatic"}:
                value = _finite(joint_positions.get(joint["name"]))
                if value is None:
                    raise ValueError(f"Missing finite joint position for {joint['name']}")
                matrix = _matmul(matrix, _axis_motion(joint["axis"], value, joint["type"]))
            elif joint["type"] != "fixed":
                raise ValueError(f"Unsupported URDF joint type {joint['type']}")
        return matrix


def _source_semantics(raw: Mapping[str, Any], manifest: Mapping[str, Any]) -> dict[str, Any]:
    sources = set()
    for observation in raw.get("observations", []) if isinstance(raw.get("observations"), list) else []:
        actual = observation.get("actual") if isinstance(observation, Mapping) else None
        if not isinstance(actual, Mapping):
            continue
        for side in SIDES:
            item = actual.get(side)
            source = item.get("source") if isinstance(item, Mapping) else None
            if isinstance(source, str) and source.strip():
                sources.add(source.strip())
    return {
        "description": manifest.get("actual_source_semantics") or "Recorded encoder/model joint evidence",
        "observed_sources": sorted(sources),
        "fk_source": "OFFLINE CALIBRATED WEB URDF FK",
        "external_metrology": False,
    }


def analyze_rigid_grasp(raw: Mapping[str, Any], manifest: Mapping[str, Any], urdf_path: str | Path) -> dict[str, Any]:
    """Compute EXP-2 relative-pose preservation from persisted Actual joint evidence."""
    raw = raw if isinstance(raw, Mapping) else {}
    manifest = manifest if isinstance(manifest, Mapping) else {}
    run_id = raw.get("run_id") or manifest.get("run_id")
    locked = manifest.get("locked_grasp") if isinstance(manifest.get("locked_grasp"), Mapping) else {}
    object_to_left = _pose_matrix(locked.get("left"))
    object_to_right = _pose_matrix(locked.get("right"))
    base = {
        "schema_version": EXP2_SCHEMA_VERSION,
        "experiment": "EXP-2 Rigid-Grasp Preservation",
        "run_id": run_id,
        "semantic": EXP2_SEMANTIC,
        "center_disagreement_definition": CENTER_SIGN,
        "actual_source_semantics": _source_semantics(raw, manifest),
        "model_identity": {
            "urdf": Path(urdf_path).name,
            "calibration_revision": manifest.get("calibration_revision"),
            "model_calibration_revision": manifest.get("model_calibration_revision"),
            "grasp_content_revision": manifest.get("grasp_content_revision"),
            "grasp_lock_revision": manifest.get("grasp_lock_revision"),
        },
        "coverage": {},
        "samples": [],
    }
    if object_to_left is None or object_to_right is None:
        base.update({"status": "UNAVAILABLE", "error": "Locked grasp left/right poses are missing or invalid"})
        return base
    reference = _matmul(_inverse_rigid(object_to_left), object_to_right)
    base["reference_relative_pose"] = {"definition": "L_T_R_ref = inverse(O_T_L) * O_T_R", "translation_m": _translation(reference), "matrix": reference}
    try:
        model = UrdfFkModel(urdf_path)
    except (OSError, ET.ParseError, ValueError) as error:
        base.update({"status": "UNAVAILABLE", "error": f"Offline URDF FK unavailable: {error}"})
        return base

    observations = raw.get("observations", []) if isinstance(raw.get("observations"), list) else []
    execution = raw.get("execution", {}) if isinstance(raw.get("execution"), Mapping) else {}
    start_ns = execution.get("start_time_unix_ns")
    duration = _finite(execution.get("duration_s"))
    reasons = Counter()
    translation_axes = [[] for _ in range(3)]
    translation_magnitude = []
    orientation_values = []
    center_axes = [[] for _ in range(3)]
    center_magnitude = []
    center_orientation_values = []

    if not isinstance(start_ns, int) or isinstance(start_ns, bool) or start_ns <= 0 or duration is None:
        base.update({"status": "UNAVAILABLE", "error": "Execution start/duration is missing or invalid"})
        return base

    # Build one unique receive-time Actual stream per arm.  EXP-2 must compare
    # both arms at the same physical time; pairing latest snapshots directly can
    # introduce a false relative-pose error when Port10000 packets arrive tens of
    # milliseconds apart.  We therefore linearly interpolate each recorded joint
    # stream onto the union of both in-range receive-time stamps.  This is offline
    # analysis only and does not modify controller timing or raw evidence.
    streams: dict[str, list[dict[str, Any]]] = {side: [] for side in SIDES}
    seen_received: dict[str, set[float]] = {side: set() for side in SIDES}
    for observation in observations:
        if not isinstance(observation, Mapping):
            reasons["INVALID_OBSERVATION"] += 1
            continue
        actual = observation.get("actual")
        if not isinstance(actual, Mapping):
            reasons["MISSING_ACTUAL"] += 1
            continue
        for side in SIDES:
            item = actual.get(side) if isinstance(actual.get(side), Mapping) else {}
            if item.get("valid") is not True or item.get("fresh") is not True:
                continue
            joints = _vector(item.get("joints_rad"), 6)
            received = _finite(item.get("received_at_ms"))
            if joints is None or received is None or received in seen_received[side]:
                continue
            seen_received[side].add(received)
            streams[side].append({
                "received_at_ms": received,
                "time_from_start_s": (received - start_ns / 1e6) / 1000.0,
                "joints_rad": joints,
            })
    for side in SIDES:
        streams[side].sort(key=lambda item: item["time_from_start_s"])

    if any(len(streams[side]) < 2 for side in SIDES):
        base.update({
            "status": "UNAVAILABLE",
            "error": "Both arms need at least two unique fresh Actual samples for synchronized interpolation",
            "coverage": {
                "total_observations": len(observations),
                "source_unique_actual_samples": {side: len(streams[side]) for side in SIDES},
            },
        })
        return base

    overlap_start = max(0.0, streams["left"][0]["time_from_start_s"], streams["right"][0]["time_from_start_s"])
    overlap_end = min(duration, streams["left"][-1]["time_from_start_s"], streams["right"][-1]["time_from_start_s"])
    if overlap_end < overlap_start:
        base.update({"status": "UNAVAILABLE", "error": "Left/right Actual streams do not overlap inside execution"})
        return base
    common_times = sorted({
        item["time_from_start_s"]
        for side in SIDES
        for item in streams[side]
        if overlap_start <= item["time_from_start_s"] <= overlap_end
    })

    def interpolate(side: str, target: float) -> list[float] | None:
        stream = streams[side]
        times = [item["time_from_start_s"] for item in stream]
        upper = bisect_left(times, target)
        if upper < len(times) and math.isclose(times[upper], target, abs_tol=1e-12, rel_tol=0.0):
            return list(stream[upper]["joints_rad"])
        if upper == 0 or upper >= len(stream):
            return None
        lower = upper - 1
        span = times[upper] - times[lower]
        if span <= 0.0:
            return None
        ratio = (target - times[lower]) / span
        return [
            stream[lower]["joints_rad"][index]
            + ratio * (stream[upper]["joints_rad"][index] - stream[lower]["joints_rad"][index])
            for index in range(6)
        ]

    reference_translation = _translation(reference)
    for sample_index, time_s in enumerate(common_times):
        left_joints = interpolate("left", time_s)
        right_joints = interpolate("right", time_s)
        if left_joints is None or right_joints is None:
            reasons["INTERPOLATION_UNAVAILABLE"] += 1
            continue
        try:
            left_fk = model.fk(TERMINAL_LINKS["left"], dict(zip(JOINT_NAMES["left"], left_joints)))
            right_fk = model.fk(TERMINAL_LINKS["right"], dict(zip(JOINT_NAMES["right"], right_joints)))
        except ValueError:
            reasons["FK_FAILED"] += 1
            continue
        actual_relative = _matmul(_inverse_rigid(left_fk), right_fk)
        actual_translation = _translation(actual_relative)
        delta = [actual_translation[index] - reference_translation[index] for index in range(3)]
        magnitude = math.sqrt(math.fsum(value * value for value in delta))
        orientation = _rotation_angle(reference, actual_relative)
        center_left = _matmul(left_fk, _inverse_rigid(object_to_left))
        center_right = _matmul(right_fk, _inverse_rigid(object_to_right))
        center_left_translation = _translation(center_left)
        center_right_translation = _translation(center_right)
        center_delta = [center_left_translation[index] - center_right_translation[index] for index in range(3)]
        center_mag = math.sqrt(math.fsum(value * value for value in center_delta))
        center_orientation = _rotation_angle(center_right, center_left)
        location = {"time_from_start_s": time_s, "observation_index": None, "synchronized_sample_index": sample_index}
        for index in range(3):
            translation_axes[index].append((delta[index], location))
            center_axes[index].append((center_delta[index], location))
        translation_magnitude.append((magnitude, location))
        orientation_values.append((orientation, location))
        center_magnitude.append((center_mag, location))
        center_orientation_values.append((center_orientation, location))
        base["samples"].append({
            **location,
            "relative_translation_actual_m": actual_translation,
            "relative_translation_error_m": delta,
            "relative_translation_error_magnitude_m": magnitude,
            "relative_orientation_error_rad": orientation,
            "relative_orientation_error_deg": math.degrees(orientation),
            "center_translation_disagreement_m": center_delta,
            "center_translation_disagreement_magnitude_m": center_mag,
            "center_orientation_disagreement_rad": center_orientation,
            "center_orientation_disagreement_deg": math.degrees(center_orientation),
        })

    usable = len(base["samples"])
    total = len(observations)
    status = "UNAVAILABLE" if usable == 0 else ("AVAILABLE" if usable == len(common_times) else "PARTIAL")
    base["status"] = status
    base["coverage"] = {
        "total_observations": total,
        "source_unique_actual_samples": {side: len(streams[side]) for side in SIDES},
        "synchronized_candidate_samples": len(common_times),
        "usable_synchronized_samples": usable,
        "skipped_synchronized_samples": len(common_times) - usable,
        "usable_fraction": usable / len(common_times) if common_times else None,
        "skipped_status_counts": dict(sorted(reasons.items())),
        "overlap_start_s": overlap_start,
        "overlap_end_s": overlap_end,
        "time_alignment": "LINEAR INTERPOLATION OF EACH RECORDED ACTUAL JOINT STREAM TO A COMMON UNION RECEIVE-TIME TIMELINE — ANALYSIS ONLY",
    }
    axis_names = ("x", "y", "z")
    base["relative_translation"] = {
        "error_definition": "translation(L_T_R_actual) - translation(L_T_R_ref)",
        "axes_m": {axis_names[index]: _signed_metric(translation_axes[index]) for index in range(3)},
        "magnitude_m": _magnitude_metric(translation_magnitude),
    }
    base["relative_orientation"] = {
        "error_definition": "angle(R_ref^T * R_actual); no Euler subtraction",
        "angle": _magnitude_metric(orientation_values, radians=True),
    }
    base["center_consistency"] = {
        "definition": CENTER_SIGN,
        "ground_truth": False,
        "averaged_center": False,
        "translation_axes_m": {axis_names[index]: _signed_metric(center_axes[index]) for index in range(3)},
        "translation_magnitude_m": _magnitude_metric(center_magnitude),
        "orientation_angle": _magnitude_metric(center_orientation_values, radians=True),
    }
    if usable == 0:
        base["error"] = "No paired in-range fresh Actual samples were usable for offline FK"
    return base


__all__ = ["EXP2_SCHEMA_VERSION", "EXP2_SEMANTIC", "UrdfFkModel", "analyze_rigid_grasp"]
