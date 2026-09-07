"""P6.1 joint tracking monitor — read-only, threshold-free.

Compares the latest joint command successfully accepted by each Phase-5
``servo_j`` stream with the latest cached Actual joint feedback.  This module
never authorizes motion, applies thresholds, or requests STOP.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any


MONITOR_VERSION = "PHASE6_JOINT_TRACKING_V1"
MONITOR_SEMANTIC = (
    "LATEST SUCCESSFUL PHASE5 SERVO_J COMMAND VS LATEST CACHED ACTUAL JOINTS — "
    "MONITORING ONLY; NO THRESHOLD OR STOP AUTHORITY"
)
SIDES = ("left", "right")
JOINT_COUNT = 6


def _finite_joint_vector(value: Any, label: str) -> list[float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{label} must contain exactly {JOINT_COUNT} joints")
    if len(value) != JOINT_COUNT:
        raise ValueError(f"{label} must contain exactly {JOINT_COUNT} joints")
    result: list[float] = []
    for index, raw in enumerate(value):
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{label}[{index}] must be finite")
        number = float(raw)
        if not math.isfinite(number):
            raise ValueError(f"{label}[{index}] must be finite")
        result.append(number)
    return result


def _joint_names(side: str) -> list[str]:
    return [f"{side}_joint_{index}" for index in range(1, JOINT_COUNT + 1)]


def _empty_side(side: str, reason: str) -> dict[str, Any]:
    return {
        "available": False,
        "side": side,
        "reason": reason,
        "joint_order": _joint_names(side),
        "commanded_joints_rad": None,
        "actual_joints_rad": None,
        "error_rad": None,
        "abs_error_rad": None,
        "max_abs_error_rad": None,
        "max_error_joint_index": None,
        "max_error_joint_name": None,
        "commanded_sample_index": None,
        "commanded_time_from_start_s": None,
        "commanded_dispatch_unix_ns": None,
        "actual_received_at_ms": None,
        "actual_age_ms": None,
    }


def _side_tracking(
    side: str,
    driver: Mapping[str, Any] | None,
    actual: Mapping[str, Any] | None,
) -> dict[str, Any]:
    if not isinstance(driver, Mapping) or driver.get("valid") is not True:
        return _empty_side(side, "DRIVER_STATUS_UNAVAILABLE")
    command = driver.get("commanded_sample")
    if not isinstance(command, Mapping) or command.get("valid") is not True:
        return _empty_side(side, "COMMAND_SAMPLE_UNAVAILABLE")
    if not isinstance(actual, Mapping) or actual.get("valid") is not True:
        return _empty_side(side, "ACTUAL_JOINT_FEEDBACK_UNAVAILABLE")

    try:
        commanded = _finite_joint_vector(
            command.get("joints_rad"), f"{side}.commanded_joints_rad"
        )
        measured = _finite_joint_vector(
            actual.get("joint"), f"{side}.actual_joints_rad"
        )
    except ValueError as error:
        return _empty_side(side, str(error))

    errors = [measured[i] - commanded[i] for i in range(JOINT_COUNT)]
    absolute = [abs(value) for value in errors]
    max_index = max(range(JOINT_COUNT), key=absolute.__getitem__)
    names = _joint_names(side)
    return {
        "available": True,
        "side": side,
        "reason": None,
        "joint_order": names,
        "commanded_joints_rad": commanded,
        "actual_joints_rad": measured,
        "error_rad": errors,
        "abs_error_rad": absolute,
        "max_abs_error_rad": absolute[max_index],
        "max_error_joint_index": max_index,
        "max_error_joint_name": names[max_index],
        "commanded_sample_index": command.get("sample_index"),
        "commanded_time_from_start_s": command.get("time_from_start_s"),
        "commanded_dispatch_unix_ns": (
            (driver.get("servo_stream") or {}).get("last_dispatch_unix_ns")
        ),
        "actual_received_at_ms": actual.get("received_at_ms"),
        "actual_age_ms": actual.get("age_ms"),
    }


def compute_joint_tracking_error(
    driver_feedback: Mapping[str, Any] | None,
    actual_joint_status: Mapping[str, Any] | None,
) -> dict[str, Any]:
    feedback = driver_feedback if isinstance(driver_feedback, Mapping) else {}
    actual = actual_joint_status if isinstance(actual_joint_status, Mapping) else {}
    drivers = feedback.get("drivers") if isinstance(feedback.get("drivers"), Mapping) else {}

    sides = {
        side: _side_tracking(
            side,
            drivers.get(side) if isinstance(drivers, Mapping) else None,
            actual.get(side) if isinstance(actual, Mapping) else None,
        )
        for side in SIDES
    }
    available = [side for side in SIDES if sides[side]["available"]]
    status = "AVAILABLE" if len(available) == 2 else ("PARTIAL" if available else "NOT_EVALUATED")

    combined: dict[str, Any] = {
        "max_abs_error_rad": None,
        "max_error_side": None,
        "max_error_joint_index": None,
        "max_error_joint_name": None,
    }
    if available:
        side = max(available, key=lambda item: sides[item]["max_abs_error_rad"])
        combined = {
            "max_abs_error_rad": sides[side]["max_abs_error_rad"],
            "max_error_side": side,
            "max_error_joint_index": sides[side]["max_error_joint_index"],
            "max_error_joint_name": sides[side]["max_error_joint_name"],
        }
    return {
        "monitor_version": MONITOR_VERSION,
        "task": "P6.1",
        "status": status,
        "unit": "radian",
        "trajectory_id": feedback.get("trajectory_id"),
        "execution_state": feedback.get("combined_state"),
        "actual_source": actual.get("source"),
        "monitoring_only": True,
        "threshold_applied": False,
        "coordinated_stop_enabled": False,
        "time_alignment": (
            "LATEST COMMAND VS LATEST ACTUAL; P6.6 OWNS TIMING/PHASE ERROR"
        ),
        "left": sides["left"],
        "right": sides["right"],
        "combined": combined,
        "semantic": MONITOR_SEMANTIC,
    }
