"""Pure Phase-5 motion-quality configuration and actual-start interlock.

This module has no ROS, SDK, FastAPI, or motion dependency.  It accepts copied
cache/artifact data so all fail-closed rules can be exercised offline.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any


REPLAN_FROM_CURRENT_SOURCE = (
    "LIVE ACTUAL JOINT FEEDBACK — REPLAN FROM CURRENT"
)
DEFAULT_START_MATCH_THRESHOLD_RAD = 0.01
DEFAULT_ACTUAL_FEEDBACK_MAX_AGE_MS = 500
DEFAULT_RECOVERY_JOINT_VELOCITY_RAD_S = 0.10
DEFAULT_RECOVERY_JOINT_ACCELERATION_RAD_S2 = 0.20
DEFAULT_SERVO_STEP_NUM = 1
DEFAULT_LEGACY_FORESIGHT_MAX_BUF = 15
DEFAULT_LEGACY_FORESIGHT_KP = 0.03
FILTER_MODES = frozenset({"NONE", "LPF", "NLF", "LEGACY_FORESIGHT"})


class Phase5MotionQualityConfigError(ValueError):
    """Raised when a Phase-5 motion setting is not conservatively valid."""


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Phase5MotionQualityConfigError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise Phase5MotionQualityConfigError(f"{label} must be a finite number")
    return result


def normalize_phase5_motion_settings(motion: Any) -> dict[str, Any]:
    """Validate additive Phase-5 settings from the robots.yaml motion section."""
    if not isinstance(motion, Mapping):
        raise Phase5MotionQualityConfigError("motion configuration must be an object")
    threshold = _finite_number(
        motion.get(
            "phase5_start_match_threshold_rad",
            DEFAULT_START_MATCH_THRESHOLD_RAD,
        ),
        "phase5_start_match_threshold_rad",
    )
    if not 0.0 < threshold <= 0.1:
        raise Phase5MotionQualityConfigError(
            "phase5_start_match_threshold_rad must be within (0, 0.1] rad"
        )
    max_age_raw = motion.get(
        "phase5_actual_feedback_max_age_ms",
        DEFAULT_ACTUAL_FEEDBACK_MAX_AGE_MS,
    )
    if isinstance(max_age_raw, bool) or not isinstance(max_age_raw, int):
        raise Phase5MotionQualityConfigError(
            "phase5_actual_feedback_max_age_ms must be an integer"
        )
    max_age_ms = int(max_age_raw)
    if not 1 <= max_age_ms <= 5000:
        raise Phase5MotionQualityConfigError(
            "phase5_actual_feedback_max_age_ms must be within [1, 5000]"
        )
    recovery_velocity = _finite_number(
        motion.get(
            "phase5_recovery_joint_vel",
            DEFAULT_RECOVERY_JOINT_VELOCITY_RAD_S,
        ),
        "phase5_recovery_joint_vel",
    )
    recovery_acceleration = _finite_number(
        motion.get(
            "phase5_recovery_joint_acc",
            DEFAULT_RECOVERY_JOINT_ACCELERATION_RAD_S2,
        ),
        "phase5_recovery_joint_acc",
    )
    if not 0.0 < recovery_velocity <= 0.5:
        raise Phase5MotionQualityConfigError(
            "phase5_recovery_joint_vel must be within (0, 0.5] rad/s"
        )
    if not 0.0 < recovery_acceleration <= 1.0:
        raise Phase5MotionQualityConfigError(
            "phase5_recovery_joint_acc must be within (0, 1.0] rad/s^2"
        )

    servo_step_num_raw = motion.get("phase5_servo_step_num", DEFAULT_SERVO_STEP_NUM)
    if (
        isinstance(servo_step_num_raw, bool)
        or not isinstance(servo_step_num_raw, int)
        or not 1 <= int(servo_step_num_raw) <= 4
    ):
        raise Phase5MotionQualityConfigError(
            "phase5_servo_step_num must be integer within [1, 4]"
        )
    servo_step_num = int(servo_step_num_raw)

    filter_raw = motion.get(
        "phase5_servo_filter", {"mode": "LEGACY_FORESIGHT"}
    )
    if not isinstance(filter_raw, Mapping):
        raise Phase5MotionQualityConfigError(
            "phase5_servo_filter must be an object"
        )
    mode = str(filter_raw.get("mode", "LEGACY_FORESIGHT")).strip().upper()
    if mode not in FILTER_MODES:
        raise Phase5MotionQualityConfigError(
            "phase5_servo_filter.mode must be LEGACY_FORESIGHT, NONE, LPF, or NLF"
        )
    filter_config: dict[str, Any] = {
        "mode": mode,
        "legacy_max_buf": 0,
        "legacy_kp": 0.0,
        "lpf_cutoff_hz": 0.0,
        "nlf_max_velocity_deg_s": 0.0,
        "nlf_max_acceleration_deg_s2": 0.0,
        "nlf_max_jerk_deg_s3": 0.0,
    }
    if mode == "LEGACY_FORESIGHT":
        max_buf = filter_raw.get(
            "max_buf", DEFAULT_LEGACY_FORESIGHT_MAX_BUF
        )
        if (
            isinstance(max_buf, bool)
            or not isinstance(max_buf, int)
            or int(max_buf) != DEFAULT_LEGACY_FORESIGHT_MAX_BUF
        ):
            raise Phase5MotionQualityConfigError(
                "LEGACY_FORESIGHT max_buf must be 15"
            )
        kp = _finite_number(
            filter_raw.get("kp", DEFAULT_LEGACY_FORESIGHT_KP),
            "LEGACY_FORESIGHT kp",
        )
        if kp != DEFAULT_LEGACY_FORESIGHT_KP:
            raise Phase5MotionQualityConfigError(
                "LEGACY_FORESIGHT kp must be 0.03"
            )
        filter_config["legacy_max_buf"] = int(max_buf)
        filter_config["legacy_kp"] = kp
    elif mode == "LPF":
        cutoff = _finite_number(filter_raw.get("cutoff_hz"), "LPF cutoff_hz")
        if not 0.1 <= cutoff <= 100.0:
            raise Phase5MotionQualityConfigError(
                "LPF cutoff_hz must be within [0.1, 100] Hz"
            )
        filter_config["lpf_cutoff_hz"] = cutoff
    elif mode == "NLF":
        limits = (
            ("max_velocity_deg_s", "nlf_max_velocity_deg_s", 0.1, 2000.0),
            (
                "max_acceleration_deg_s2",
                "nlf_max_acceleration_deg_s2",
                0.1,
                20000.0,
            ),
            ("max_jerk_deg_s3", "nlf_max_jerk_deg_s3", 0.1, 200000.0),
        )
        for yaml_name, transport_name, minimum, maximum in limits:
            value = _finite_number(filter_raw.get(yaml_name), f"NLF {yaml_name}")
            if not minimum <= value <= maximum:
                raise Phase5MotionQualityConfigError(
                    f"NLF {yaml_name} must be within [{minimum}, {maximum}]"
                )
            filter_config[transport_name] = value
    return {
        "valid": True,
        "start_match_threshold_rad": threshold,
        "actual_feedback_max_age_ms": max_age_ms,
        "recovery_joint_vel_rad_s": recovery_velocity,
        "recovery_joint_acc_rad_s2": recovery_acceleration,
        "servo_step_num": servo_step_num,
        "servo_filter": filter_config,
    }


def _fresh_actual_side(
    actual_feedback: Any,
    side: str,
    max_age_ms: int,
) -> tuple[tuple[float, ...] | None, str | None]:
    if not isinstance(actual_feedback, Mapping):
        return None, "ACTUAL_FEEDBACK_UNAVAILABLE"
    side_data = actual_feedback.get(side)
    if not isinstance(side_data, Mapping) or side_data.get("valid") is not True:
        return None, f"{side.upper()}_ACTUAL_FEEDBACK_INVALID"
    joints = side_data.get("joint")
    if (
        not isinstance(joints, Sequence)
        or isinstance(joints, (str, bytes, bool))
        or len(joints) != 6
    ):
        return None, f"{side.upper()}_ACTUAL_JOINT_SHAPE_INVALID"
    normalized = []
    for value in joints:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            return None, f"{side.upper()}_ACTUAL_JOINT_NONFINITE"
        normalized.append(float(value))
    age_ms = side_data.get("age_ms")
    if (
        isinstance(age_ms, bool)
        or not isinstance(age_ms, (int, float))
        or not math.isfinite(float(age_ms))
        or float(age_ms) < 0.0
    ):
        return None, f"{side.upper()}_ACTUAL_FEEDBACK_AGE_INVALID"
    if float(age_ms) > float(max_age_ms):
        return None, (
            f"{side.upper()}_ACTUAL_FEEDBACK_STALE: "
            f"age_ms={float(age_ms):.3f} > {max_age_ms}"
        )
    return tuple(normalized), None


def fresh_actual_joint_state(
    actual_feedback: Any,
    *,
    max_age_ms: int = DEFAULT_ACTUAL_FEEDBACK_MAX_AGE_MS,
) -> dict[str, Any]:
    """Return a copy-safe fresh dual-arm snapshot or a fail-closed reason."""
    sides: dict[str, tuple[float, ...]] = {}
    reasons = []
    for side in ("left", "right"):
        joints, reason = _fresh_actual_side(actual_feedback, side, int(max_age_ms))
        if reason is not None:
            reasons.append(reason)
        elif joints is not None:
            sides[side] = joints
    return {
        "ok": not reasons,
        "reason": "; ".join(reasons) if reasons else "FRESH_ACTUAL_FEEDBACK",
        "max_age_ms": int(max_age_ms),
        "initial_joint_state_rad": (
            {side: list(sides[side]) for side in ("left", "right")}
            if not reasons
            else None
        ),
        "source": REPLAN_FROM_CURRENT_SOURCE if not reasons else None,
    }


def evaluate_actual_start_match(
    actual_feedback: Any,
    artifact: Any,
    *,
    threshold_rad: float = DEFAULT_START_MATCH_THRESHOLD_RAD,
    max_age_ms: int = DEFAULT_ACTUAL_FEEDBACK_MAX_AGE_MS,
) -> dict[str, Any]:
    """Compare fresh cached actual joints with frozen sample zero."""
    result: dict[str, Any] = {
        "match": False,
        "state": "BLOCKED",
        "reason": None,
        "max_delta_rad": None,
        "threshold_rad": float(threshold_rad),
        "actual_feedback_max_age_ms": int(max_age_ms),
        "joint_deltas_rad": None,
        "authoritative_initial_source": "FROZEN_ARTIFACT_TRAJECTORY_SAMPLE_0",
    }
    if (
        not isinstance(threshold_rad, (int, float))
        or isinstance(threshold_rad, bool)
        or not math.isfinite(float(threshold_rad))
        or float(threshold_rad) <= 0.0
    ):
        result["reason"] = "START_MATCH_THRESHOLD_INVALID"
        return result
    if artifact is None:
        result["reason"] = "FROZEN_ARTIFACT_UNAVAILABLE"
        return result
    fresh = fresh_actual_joint_state(actual_feedback, max_age_ms=int(max_age_ms))
    if fresh["ok"] is not True:
        result["reason"] = fresh["reason"]
        return result
    try:
        initial = {
            "left": tuple(float(value) for value in artifact.left_positions_rad[0]),
            "right": tuple(float(value) for value in artifact.right_positions_rad[0]),
        }
        if any(len(initial[side]) != 6 for side in ("left", "right")):
            raise ValueError("sample zero must contain six joints per arm")
        if any(
            not math.isfinite(value)
            for side in ("left", "right")
            for value in initial[side]
        ):
            raise ValueError("sample zero contains nonfinite joints")
    except (AttributeError, IndexError, TypeError, ValueError):
        result["reason"] = "FROZEN_ARTIFACT_SAMPLE_0_INVALID"
        return result
    actual = fresh["initial_joint_state_rad"]
    deltas = {
        side: [
            abs(float(actual[side][index]) - initial[side][index])
            for index in range(6)
        ]
        for side in ("left", "right")
    }
    max_delta = max(value for side in deltas.values() for value in side)
    matches = max_delta <= float(threshold_rad)
    result.update(
        {
            "match": matches,
            "state": "READY" if matches else "MISMATCH",
            "reason": (
                "ACTUAL_START_MATCHES_FROZEN_SAMPLE_0"
                if matches
                else "ACTUAL_START_DELTA_EXCEEDS_THRESHOLD"
            ),
            "max_delta_rad": max_delta,
            "joint_deltas_rad": deltas,
        }
    )
    return result
