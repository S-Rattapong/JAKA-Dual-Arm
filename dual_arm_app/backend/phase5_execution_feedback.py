"""Pure P5.11-P5.15 aggregation of read-only driver execution snapshots."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any


DRIVER_STATES = frozenset(
    {"IDLE", "ARMED", "RUNNING", "COMPLETED", "ABORTED", "FAILED"}
)
TERMINAL_STATES = frozenset({"COMPLETED", "ABORTED", "FAILED"})


def _field(source: Any, name: str, default: Any = None) -> Any:
    return source.get(name, default) if isinstance(source, Mapping) else getattr(
        source, name, default
    )


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _joint_vector(value: Any, name: str) -> list[float]:
    # ROS2 float64[] fields are commonly exposed as array.array rather than a
    # built-in list/tuple. Copy any non-text iterable into a plain list first,
    # then apply the same exact-length and finite-number validation.
    if isinstance(value, (str, bytes, bytearray, Mapping)):
        raise ValueError(f"{name} must contain exactly 6 joints")
    try:
        items = list(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must contain exactly 6 joints") from None
    if len(items) != 6:
        raise ValueError(f"{name} must contain exactly 6 joints")
    return [_finite_float(item, f"{name}[{index}]") for index, item in enumerate(items)]


def normalize_driver_execution_status(
    response: Any,
    *,
    side: str,
    unavailable_error: str | None = None,
) -> dict[str, Any]:
    """Normalize one ROS response without any ROS/runtime dependency."""
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    if response is None:
        return {
            "available": False,
            "valid": False,
            "side": side,
            "error": unavailable_error or f"{side.upper()}_STATUS_UNAVAILABLE",
        }
    try:
        valid = _field(response, "valid") is True
        ret = _integer(_field(response, "ret"), "ret")
        state = str(_field(response, "state", ""))
        if not valid or ret != 1:
            raise ValueError(
                str(_field(response, "message", "driver marked status invalid"))
            )
        if state not in DRIVER_STATES:
            raise ValueError(f"unknown driver state: {state}")
        trajectory_id = str(_field(response, "trajectory_id", ""))
        start_ns = _integer(_field(response, "start_time_unix_ns"), "start_time_unix_ns")
        terminal_ns = _integer(
            _field(response, "terminal_time_unix_ns"), "terminal_time_unix_ns"
        )
        duration_s = max(0.0, _finite_float(_field(response, "duration_s"), "duration_s"))
        elapsed_s = max(0.0, _finite_float(_field(response, "elapsed_s"), "elapsed_s"))
        progress = min(
            1.0,
            max(
                0.0,
                _finite_float(
                    _field(response, "progress_0_to_1"), "progress_0_to_1"
                ),
            ),
        )
        sample_index = _integer(_field(response, "sample_index"), "sample_index")
        sample_count = _integer(_field(response, "sample_count"), "sample_count")
        if sample_index < -1 or sample_count < 0:
            raise ValueError("sample index/count are outside the supported range")
        servo_step_num = _integer(
            _field(response, "servo_step_num", 1), "servo_step_num"
        )
        if not 1 <= servo_step_num <= 4:
            raise ValueError("servo_step_num must be within [1, 4]")
        command_period_ms = _finite_float(
            _field(response, "command_period_ms", servo_step_num * 8.0),
            "command_period_ms",
        )
        if command_period_ms != servo_step_num * 8.0:
            raise ValueError("command_period_ms does not match servo_step_num")
        telemetry_mode = str(_field(response, "telemetry_mode", "NORMAL"))
        if telemetry_mode not in {
            "NORMAL",
            "PHASE5_PORT10004_SNAPSHOT",
            "PHASE5_SDK_EXCLUSIVE_TELEMETRY_FROZEN",
        }:
            raise ValueError("unknown telemetry_mode")
        commanded_sample_valid = bool(_field(response, "commanded_sample_valid", False))
        commanded_time_from_start_s = None
        commanded_joints_rad: list[float] | None = None
        if commanded_sample_valid:
            commanded_time_from_start_s = max(0.0, _finite_float(
                _field(response, "commanded_time_from_start_s"),
                "commanded_time_from_start_s",
            ))
            commanded_joints_rad = _joint_vector(
                _field(response, "commanded_joints_rad"),
                "commanded_joints_rad",
            )
        return {
            "available": True,
            "valid": True,
            "side": side,
            "ret": ret,
            "message": str(_field(response, "message", "")),
            "trajectory_id": trajectory_id,
            "state": state,
            "active": bool(_field(response, "active", False)),
            "start_time_unix_ns": start_ns,
            "terminal_time_unix_ns": terminal_ns,
            "duration_s": duration_s,
            "elapsed_s": min(elapsed_s, duration_s) if duration_s else elapsed_s,
            "progress_0_to_1": progress,
            "sample_index": sample_index,
            "sample_count": sample_count,
            "commanded_sample": {
                "valid": commanded_sample_valid,
                "sample_index": sample_index if commanded_sample_valid else None,
                "time_from_start_s": commanded_time_from_start_s,
                "joints_rad": commanded_joints_rad,
                "source": "LATEST SUCCESSFUL PHASE5 SERVO_J COMMAND — DRIVER MEMORY",
            },
            "terminal_reason": str(_field(response, "terminal_reason", "")),
            "motion_quality": {
                "sample_count": sample_count,
                "max_abs_velocity_rad_s": _finite_float(
                    _field(response, "max_abs_velocity_rad_s", 0.0),
                    "max_abs_velocity_rad_s",
                ),
                "rms_velocity_rad_s": _finite_float(
                    _field(response, "rms_velocity_rad_s", 0.0),
                    "rms_velocity_rad_s",
                ),
                "max_abs_acceleration_rad_s2": _finite_float(
                    _field(response, "max_abs_acceleration_rad_s2", 0.0),
                    "max_abs_acceleration_rad_s2",
                ),
                "rms_acceleration_rad_s2": _finite_float(
                    _field(response, "rms_acceleration_rad_s2", 0.0),
                    "rms_acceleration_rad_s2",
                ),
                "max_abs_jerk_rad_s3": _finite_float(
                    _field(response, "max_abs_jerk_rad_s3", 0.0),
                    "max_abs_jerk_rad_s3",
                ),
                "rms_jerk_rad_s3": _finite_float(
                    _field(response, "rms_jerk_rad_s3", 0.0),
                    "rms_jerk_rad_s3",
                ),
            },
            "dispatch_timing": {
                "sample_count": _integer(
                    _field(response, "dispatch_sample_count", 0),
                    "dispatch_sample_count",
                ),
                "mean_abs_lateness_ms": _finite_float(
                    _field(response, "mean_abs_lateness_ms", 0.0),
                    "mean_abs_lateness_ms",
                ),
                "p95_abs_lateness_ms": _finite_float(
                    _field(response, "p95_abs_lateness_ms", 0.0),
                    "p95_abs_lateness_ms",
                ),
                "p99_abs_lateness_ms": _finite_float(
                    _field(response, "p99_abs_lateness_ms", 0.0),
                    "p99_abs_lateness_ms",
                ),
                "max_abs_lateness_ms": _finite_float(
                    _field(response, "max_abs_lateness_ms", 0.0),
                    "max_abs_lateness_ms",
                ),
                "mean_abs_jitter_ms": _finite_float(
                    _field(response, "mean_abs_jitter_ms", 0.0),
                    "mean_abs_jitter_ms",
                ),
                "p95_abs_jitter_ms": _finite_float(
                    _field(response, "p95_abs_jitter_ms", 0.0),
                    "p95_abs_jitter_ms",
                ),
                "p99_abs_jitter_ms": _finite_float(
                    _field(response, "p99_abs_jitter_ms", 0.0),
                    "p99_abs_jitter_ms",
                ),
                "max_abs_jitter_ms": _finite_float(
                    _field(response, "max_abs_jitter_ms", 0.0),
                    "max_abs_jitter_ms",
                ),
                "missed_cycle_count": _integer(
                    _field(response, "missed_cycle_count", 0),
                    "missed_cycle_count",
                ),
            },
            "servo_j_call_duration": {
                "sample_count": _integer(
                    _field(response, "servo_j_call_sample_count", 0),
                    "servo_j_call_sample_count",
                ),
                "overrun_count": _integer(
                    _field(response, "servo_j_overrun_count", 0),
                    "servo_j_overrun_count",
                ),
                "mean_ms": _finite_float(
                    _field(response, "mean_servo_j_call_duration_ms", 0.0),
                    "mean_servo_j_call_duration_ms",
                ),
                "p95_ms": _finite_float(
                    _field(response, "p95_servo_j_call_duration_ms", 0.0),
                    "p95_servo_j_call_duration_ms",
                ),
                "p99_ms": _finite_float(
                    _field(response, "p99_servo_j_call_duration_ms", 0.0),
                    "p99_servo_j_call_duration_ms",
                ),
                "max_ms": _finite_float(
                    _field(response, "max_servo_j_call_duration_ms", 0.0),
                    "max_servo_j_call_duration_ms",
                ),
            },
            "servo_stream": {
                "servo_step_num": servo_step_num,
                "command_period_ms": command_period_ms,
                "telemetry_mode": telemetry_mode,
                "telemetry_suppressed_poll_count": _integer(
                    _field(response, "telemetry_suppressed_poll_count", 0),
                    "telemetry_suppressed_poll_count",
                ),
                "first_dispatch_unix_ns": _integer(
                    _field(response, "first_dispatch_unix_ns", 0),
                    "first_dispatch_unix_ns",
                ),
                "first_servo_return_unix_ns": _integer(
                    _field(response, "first_servo_return_unix_ns", 0),
                    "first_servo_return_unix_ns",
                ),
                "last_dispatch_unix_ns": _integer(
                    _field(response, "last_dispatch_unix_ns", 0),
                    "last_dispatch_unix_ns",
                ),
                "last_servo_return_unix_ns": _integer(
                    _field(response, "last_servo_return_unix_ns", 0),
                    "last_servo_return_unix_ns",
                ),
                "guard_limit_rad_s": _finite_float(
                    _field(response, "stream_guard_limit_rad_s", 0.0),
                    "stream_guard_limit_rad_s",
                ),
                "guard_observed_max_rad_s": _finite_float(
                    _field(response, "stream_guard_observed_max_rad_s", 0.0),
                    "stream_guard_observed_max_rad_s",
                ),
            },
            "servo_filter": {
                "mode": str(
                    _field(response, "servo_filter_mode", "LEGACY_FORESIGHT")
                ),
                "legacy_max_buf": _integer(
                    _field(response, "servo_filter_legacy_max_buf", 15),
                    "servo_filter_legacy_max_buf",
                ),
                "legacy_kp": _finite_float(
                    _field(response, "servo_filter_legacy_kp", 0.03),
                    "servo_filter_legacy_kp",
                ),
                "lpf_cutoff_hz": _finite_float(
                    _field(response, "servo_filter_lpf_cutoff_hz", 0.0),
                    "servo_filter_lpf_cutoff_hz",
                ),
                "nlf_max_velocity_deg_s": _finite_float(
                    _field(response, "servo_filter_nlf_max_velocity_deg_s", 0.0),
                    "servo_filter_nlf_max_velocity_deg_s",
                ),
                "nlf_max_acceleration_deg_s2": _finite_float(
                    _field(response, "servo_filter_nlf_max_acceleration_deg_s2", 0.0),
                    "servo_filter_nlf_max_acceleration_deg_s2",
                ),
                "nlf_max_jerk_deg_s3": _finite_float(
                    _field(response, "servo_filter_nlf_max_jerk_deg_s3", 0.0),
                    "servo_filter_nlf_max_jerk_deg_s3",
                ),
            },
            "error": None,
        }
    except (TypeError, ValueError) as error:
        return {
            "available": True,
            "valid": False,
            "side": side,
            "error": str(error),
        }


def _reason_for_terminal(state: str, drivers: Mapping[str, Mapping[str, Any]]) -> str:
    if state == "COMPLETED":
        return "BOTH_DRIVERS_COMPLETED"
    details = []
    for side in ("left", "right"):
        driver = drivers[side]
        driver_state = driver.get("state")
        if driver_state in TERMINAL_STATES:
            detail = driver.get("terminal_reason") or driver.get("message") or driver_state
            details.append(f"{side.upper()}_{driver_state}: {detail}")
    return "; ".join(details) or state


def build_phase5_execution_feedback(
    *,
    expected_trajectory_id: str | None,
    left_response: Any,
    right_response: Any,
    left_error: str | None = None,
    right_error: str | None = None,
    actual_joint_feedback: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Combine two driver snapshots; unavailable/mismatched data stays nonterminal."""
    expected = str(expected_trajectory_id or "")
    drivers = {
        "left": normalize_driver_execution_status(
            left_response, side="left", unavailable_error=left_error
        ),
        "right": normalize_driver_execution_status(
            right_response, side="right", unavailable_error=right_error
        ),
    }
    result: dict[str, Any] = {
        "read_only": True,
        "authoritative": False,
        "feedback_status": "FEEDBACK_UNAVAILABLE",
        "combined_state": "FEEDBACK_UNAVAILABLE",
        "terminal": False,
        "result": None,
        "reason": None,
        "trajectory_id": expected or None,
        "drivers": drivers,
        "common_timeline": {
            "start_time_unix_ns": None,
            "terminal_time_unix_ns": None,
            "duration_s": None,
            "elapsed_s": None,
            "progress_0_to_1": None,
            "sample_index": None,
            "sample_count": None,
        },
        "actual_joints": dict(actual_joint_feedback or {}),
        "semantic": "READ-ONLY AUTHORITATIVE DRIVER MEMORY + CACHED JOINT FEEDBACK",
    }
    invalid_sides = [
        side for side, driver in drivers.items() if driver.get("valid") is not True
    ]
    if not expected:
        result["reason"] = "NO_CURRENT_TRAJECTORY"
        return result
    if invalid_sides:
        result["reason"] = "; ".join(
            f"{side.upper()}: {drivers[side].get('error') or 'STATUS_UNAVAILABLE'}"
            for side in invalid_sides
        )
        return result

    mismatches = [
        side
        for side, driver in drivers.items()
        if driver.get("trajectory_id") != expected
    ]
    if mismatches:
        result["feedback_status"] = "MISMATCH"
        result["combined_state"] = "MISMATCH"
        result["reason"] = "; ".join(
            f"{side.upper()}: expected {expected}, got "
            f"{drivers[side].get('trajectory_id') or '<empty>'}"
            for side in mismatches
        )
        return result

    # Same trajectory id is necessary but not sufficient: the two driver
    # snapshots must describe the same common execution contract.
    left = drivers["left"]
    right = drivers["right"]
    contract_mismatches = []
    if left["start_time_unix_ns"] != right["start_time_unix_ns"]:
        contract_mismatches.append("COMMON_START_TIME_MISMATCH")
    if abs(left["duration_s"] - right["duration_s"]) > 1e-9:
        contract_mismatches.append("DURATION_MISMATCH")
    if left["sample_count"] != right["sample_count"]:
        contract_mismatches.append("SAMPLE_COUNT_MISMATCH")
    if left["servo_stream"]["servo_step_num"] != right["servo_stream"]["servo_step_num"]:
        contract_mismatches.append("SERVO_STEP_NUM_MISMATCH")
    if abs(
        left["servo_stream"]["command_period_ms"]
        - right["servo_stream"]["command_period_ms"]
    ) > 1e-9:
        contract_mismatches.append("COMMAND_PERIOD_MISMATCH")
    if left["servo_stream"]["telemetry_mode"] != right["servo_stream"]["telemetry_mode"]:
        contract_mismatches.append("TELEMETRY_MODE_MISMATCH")
    if left["servo_filter"] != right["servo_filter"]:
        contract_mismatches.append("SERVO_FILTER_MISMATCH")
    if contract_mismatches:
        result["feedback_status"] = "MISMATCH"
        result["combined_state"] = "MISMATCH"
        result["reason"] = "; ".join(contract_mismatches)
        return result

    states = {side: driver["state"] for side, driver in drivers.items()}
    if any(state == "IDLE" for state in states.values()):
        result["reason"] = "MATCHED_TRAJECTORY_REPORTED_IDLE"
        return result

    all_terminal = all(state in TERMINAL_STATES for state in states.values())
    if any(state == "FAILED" for state in states.values()):
        combined = "FAILED" if all_terminal else "ABORT_REQUESTED"
    elif any(state == "ABORTED" for state in states.values()):
        combined = "ABORTED" if all_terminal else "ABORT_REQUESTED"
    elif all(state == "COMPLETED" for state in states.values()):
        combined = "COMPLETED"
    elif any(state in {"RUNNING", "COMPLETED"} for state in states.values()):
        combined = "RUNNING"
    else:
        combined = "ARMED"

    start_values = [driver["start_time_unix_ns"] for driver in drivers.values()]
    terminal_values = [driver["terminal_time_unix_ns"] for driver in drivers.values()]
    duration_values = [driver["duration_s"] for driver in drivers.values()]
    elapsed_values = [driver["elapsed_s"] for driver in drivers.values()]
    progress_values = [driver["progress_0_to_1"] for driver in drivers.values()]
    sample_indices = [driver["sample_index"] for driver in drivers.values()]
    sample_counts = [driver["sample_count"] for driver in drivers.values()]
    terminal = combined in TERMINAL_STATES
    reason = (
        _reason_for_terminal(combined if terminal else "ABORTED", drivers)
        if terminal or combined == "ABORT_REQUESTED"
        else None
    )
    result.update(
        {
            "authoritative": True,
            "feedback_status": "AUTHORITATIVE",
            "combined_state": combined,
            "terminal": terminal,
            "result": combined if terminal else None,
            "reason": reason,
            "common_timeline": {
                "start_time_unix_ns": max(start_values),
                "terminal_time_unix_ns": max(terminal_values) if terminal else 0,
                "duration_s": max(duration_values),
                "elapsed_s": min(elapsed_values),
                "progress_0_to_1": min(1.0, max(0.0, min(progress_values))),
                "sample_index": min(sample_indices),
                "sample_count": min(sample_counts),
            },
        }
    )
    return result
