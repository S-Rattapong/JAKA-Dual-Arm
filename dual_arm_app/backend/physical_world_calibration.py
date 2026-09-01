"""Pure math for PRE-P5.A3 three-point virtual-J6 World calibration."""
from __future__ import annotations

import copy
import math
from typing import Any, Mapping, Sequence

from dual_arm_app.backend.world_frame_calibration import compute_calibration_revision

METHOD = "THREE_POINT_VIRTUAL_J6_TOUCH"
WORLD_ORIGIN_SEMANTICS = "VIRTUAL_COMMON_J6_AT_EQUIVALENT_TOOL_REFERENCE_ORIGIN"


def _v(values: Sequence[float]) -> list[float]:
    if len(values) != 3:
        raise ValueError("expected a 3-vector")
    out = [float(x) for x in values]
    if not all(math.isfinite(x) for x in out):
        raise ValueError("vector must be finite")
    return out


def _sub(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [float(a[i]) - float(b[i]) for i in range(3)]


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def _cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def _norm(a: Sequence[float]) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Sequence[float], label: str) -> list[float]:
    n = _norm(a)
    if n <= 1e-12:
        raise ValueError(f"{label} has zero length")
    return [float(x) / n for x in a]


def matrix_to_rpy(r: Sequence[Sequence[float]]) -> list[float]:
    """Return extrinsic XYZ RPY for R = Rz * Ry * Rx."""
    sy = max(-1.0, min(1.0, -float(r[2][0])))
    ry = math.asin(sy)
    cy = math.cos(ry)
    if abs(cy) > 1e-9:
        rx = math.atan2(float(r[2][1]), float(r[2][2]))
        rz = math.atan2(float(r[1][0]), float(r[0][0]))
    else:
        rx = math.atan2(-float(r[1][2]), float(r[1][1]))
        rz = 0.0
    return [rx, ry, rz]


def rpy_to_matrix(rpy: Sequence[float]) -> list[list[float]]:
    rx, ry, rz = (float(v) for v in rpy)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
            [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
            [-sy, cy*sx, cy*cx]]

def solve_world_to_base_from_three_points(
    origin_in_base_m: Sequence[float],
    x_plus_in_base_m: Sequence[float],
    y_plus_in_base_m: Sequence[float],
) -> dict[str, Any]:
    o = _v(origin_in_base_m)
    vx = _sub(_v(x_plus_in_base_m), o)
    vy = _sub(_v(y_plus_in_base_m), o)
    x_len = _norm(vx)
    y_len = _norm(vy)
    x_axis_b = _unit(vx, "O->X")
    y_rejected = [vy[i] - _dot(vy, x_axis_b) * x_axis_b[i] for i in range(3)]
    y_axis_b = _unit(y_rejected, "Y component orthogonal to X")
    z_axis_b = _unit(_cross(x_axis_b, y_axis_b), "X cross Y")
    y_axis_b = _unit(_cross(z_axis_b, x_axis_b), "orthogonalized Y")
    # Rows map a base-frame vector into World coordinates.
    r_world_base = [x_axis_b, y_axis_b, z_axis_b]
    t_world_base = [-_dot(row, o) for row in r_world_base]
    cosine = max(-1.0, min(1.0, _dot(vx, vy) / (x_len * y_len)))
    return {
        "translation_m": t_world_base,
        "rotation_rpy_rad": matrix_to_rpy(r_world_base),
        "diagnostics": {
            "origin_to_x_m": x_len,
            "origin_to_y_m": y_len,
            "xy_angle_rad": math.acos(cosine),
            "y_orthogonal_component_m": _norm(y_rejected),
        },
    }


def world_point(base_translation_m: Sequence[float], base_rotation_rpy_rad: Sequence[float],
                point_in_base_m: Sequence[float]) -> list[float]:
    r = rpy_to_matrix(base_rotation_rpy_rad)
    t, p = _v(base_translation_m), _v(point_in_base_m)
    return [t[row] + sum(r[row][col] * p[col] for col in range(3)) for row in range(3)]

def build_physical_calibration_payload(
    current: Mapping[str, Any], left_points: Mapping[str, Sequence[float]],
    right_points: Mapping[str, Sequence[float]],
) -> tuple[dict[str, Any], dict[str, Any]]:
    result = copy.deepcopy(dict(current))
    diagnostics: dict[str, Any] = {}
    for side, points in (("left", left_points), ("right", right_points)):
        solved = solve_world_to_base_from_three_points(
            points["origin"], points["x_plus"], points["y_plus"]
        )
        diagnostics[side] = solved.pop("diagnostics")
        result["transforms"][f"world_to_{side}_base"] = solved
    result["calibration_state"] = "PHYSICAL_CALIBRATED"
    result["physical_calibration"] = "CALIBRATED"
    result["source"] = {
        "method": METHOD,
        "description": (
            "Three-point physical World calibration using virtual J6 positions at "
            "repeatable rigid-tool contacts for Origin, +X and +Y; full base pose "
            "is solved from measured physical directions without a Web setter."
        ),
    }
    result["provenance"] = {
        "status": "PHYSICAL_TOUCH_CALIBRATION",
        "verification_status": "PHYSICALLY_CALIBRATED_SINGLE_PASS_NOT_REPEATABILITY_VERIFIED",
    }
    result["tcp_tool_contract"]["status"] = "TOOL0_J6_KINEMATICALLY_VERIFIED"
    result["revision"] = "sha256:pending"
    result["revision"] = compute_calibration_revision(result)
    return result, diagnostics
