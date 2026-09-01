"""Pure normalization for Digital Twin robot-state and TCP read paths.

This module intentionally imports no ROS, FastAPI, or JAKA runtime packages so
the contracts can be exercised offline without connecting to robot hardware.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any


ROBOT_STATE_FIELDS = (
    "power_state",
    "servo_state",
    "motion_state",
    "collision_state",
)
DEFAULT_STALE_TIMEOUT_MS = 1500
TCP_SOURCE = "jaka_get_fk_from_current_joint_feedback"


def _finite_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite integer")
    converted = float(value)
    if not math.isfinite(converted) or not converted.is_integer():
        raise ValueError(f"{label} must be a finite integer")
    return int(converted)


def _field(source: Any, name: str) -> Any:
    if isinstance(source, Mapping):
        if name not in source:
            raise ValueError(f"RobotMsg is missing {name}")
        return source[name]
    if not hasattr(source, name):
        raise ValueError(f"RobotMsg is missing {name}")
    return getattr(source, name)


def normalize_robot_state_message(message: Any) -> dict[str, Any]:
    """Copy only the four Phase-1 RobotMsg fields into JSON-safe integers."""
    try:
        if message is None:
            raise ValueError("RobotMsg is missing")
        state = {
            name: _finite_integer(_field(message, name), name)
            for name in ROBOT_STATE_FIELDS
        }
        return {"valid": True, "state": state, "error": None}
    except (TypeError, ValueError) as error:
        return {"valid": False, "state": None, "error": str(error)}


def _valid_timestamp(value: Any) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _age_ms(received_at_ms: Any, now_ms: int) -> int | None:
    if not _valid_timestamp(received_at_ms):
        return None
    return max(0, now_ms - int(received_at_ms))


def _joint_cache_valid(cache: Mapping[str, Any]) -> bool:
    joint = cache.get("joint")
    return (
        isinstance(joint, (list, tuple))
        and len(joint) == 6
        and all(
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(float(value))
            for value in joint
        )
        and _valid_timestamp(cache.get("received_at_ms"))
    )


def build_digital_twin_robot_status(
    state_cache: Mapping[str, Mapping[str, Any]],
    joint_cache: Mapping[str, Mapping[str, Any]],
    *,
    server_time_ms: int,
    stale_timeout_ms: int = DEFAULT_STALE_TIMEOUT_MS,
) -> dict[str, Any]:
    """Build cache-only per-arm state with deterministic feedback freshness."""
    now_ms = int(server_time_ms)
    timeout_ms = int(stale_timeout_ms)
    if timeout_ms < 0:
        raise ValueError("stale_timeout_ms must be non-negative")

    response: dict[str, Any] = {
        "ok": False,
        "source": "ros_robot_state_and_joint_feedback_cache",
        "cache_only": True,
        "server_time_ms": now_ms,
        "stale_timeout_ms": timeout_ms,
        "connection_semantics": "feedback_freshness_not_transport_connection",
    }
    both_live = True

    for side in ("left", "right"):
        cached_state = state_cache.get(side, {})
        cached_joint = joint_cache.get(side, {})
        normalized_state = normalize_robot_state_message(cached_state.get("state"))
        robot_valid = (
            cached_state.get("valid") is True
            and normalized_state["valid"]
            and _valid_timestamp(cached_state.get("received_at_ms"))
        )
        joint_valid = _joint_cache_valid(cached_joint)
        robot_age = _age_ms(cached_state.get("received_at_ms"), now_ms)
        joint_age = _age_ms(cached_joint.get("received_at_ms"), now_ms)

        state_error = cached_state.get("error")
        joint_error = cached_joint.get("error")
        state_invalid = cached_state.get("status") == "INVALID"
        joint_invalid = cached_joint.get("status") == "INVALID"
        if state_invalid or joint_invalid:
            feedback_status = "INVALID"
        elif not robot_valid or not joint_valid:
            feedback_status = "MISSING"
        elif robot_age > timeout_ms or joint_age > timeout_ms:
            feedback_status = "STALE"
        else:
            feedback_status = "LIVE"

        both_live = both_live and feedback_status == "LIVE"
        response[side] = {
            "feedback_status": feedback_status,
            "robot_state_valid": robot_valid,
            "robot_state_received_at_ms": (
                int(cached_state["received_at_ms"]) if robot_valid else None
            ),
            "robot_state_age_ms": robot_age if robot_valid else None,
            "joint_feedback_valid": joint_valid,
            "joint_feedback_received_at_ms": (
                int(cached_joint["received_at_ms"]) if joint_valid else None
            ),
            "joint_feedback_age_ms": joint_age if joint_valid else None,
            "state": dict(normalized_state["state"]) if robot_valid else None,
            "robot_state_error": state_error or normalized_state["error"],
            "joint_feedback_error": joint_error,
        }

    response["ok"] = both_live
    return response


def normalize_tcp_pose(pose: Any) -> dict[str, Any]:
    """Validate a raw JAKA CartesianPose array without converting its units."""
    try:
        if isinstance(pose, (str, bytes)) or not isinstance(pose, Sequence):
            raise ValueError("TCP pose must contain exactly six finite values")
        values = list(pose)
        if len(values) != 6:
            raise ValueError("TCP pose must contain exactly six finite values")
        normalized: list[float] = []
        for index, value in enumerate(values):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"TCP pose value {index} must be finite")
            converted = float(value)
            if not math.isfinite(converted):
                raise ValueError(f"TCP pose value {index} must be finite")
            normalized.append(converted)
        if normalized == [9999.0] * 6:
            raise ValueError("JAKA GetFK returned the documented driver failure sentinel 9999")
        return {"valid": True, "tcp": normalized, "error": None}
    except (TypeError, ValueError) as error:
        return {"valid": False, "tcp": None, "error": str(error)}


def build_digital_twin_tcp_status(
    poses: Mapping[str, Any],
    *,
    server_time_ms: int,
    errors: Mapping[str, str | None] | None = None,
) -> dict[str, Any]:
    """Normalize isolated FK results; one failed arm never hides the other."""
    supplied_errors = errors or {}
    response: dict[str, Any] = {
        "ok": False,
        "source": TCP_SOURCE,
        "cache_only": False,
        "server_time_ms": int(server_time_ms),
        "pose_order": ["x", "y", "z", "rx", "ry", "rz"],
        "units": {"translation": "millimeter", "orientation": "radian"},
        "frame": "jaka_controller_current_user_coordinate",
        "orientation_convention": "JAKA CartesianPose RPY rx_ry_rz",
        "measurement_semantics": "forward_kinematics_not_external_metrology",
    }
    both_valid = True
    for side in ("left", "right"):
        normalized = normalize_tcp_pose(poses.get(side))
        if not normalized["valid"] and supplied_errors.get(side):
            normalized["error"] = supplied_errors[side]
        both_valid = both_valid and normalized["valid"]
        response[side] = normalized
    response["ok"] = both_valid
    return response
