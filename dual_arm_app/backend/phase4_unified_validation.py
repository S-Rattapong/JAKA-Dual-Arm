"""Pure Phase-4C unified validation report and fail-closed gate policy.

OFFLINE / VALIDATION ONLY.  This module combines already-produced Phase-3,
Phase-4A, and Phase-4B artifacts.  It has no ROS, driver, service, publisher,
controller, endpoint, or robot-command dependency.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import re
from typing import Any, Mapping, Sequence

from dual_arm_app.backend.object_trajectory_ik import (
    DUAL_ARM_JOINT_ORDER,
    CanonicalJointPositionLimits,
)


REPORT_VERSION = "PHASE4C_UNIFIED_VALIDATION_V3_GRASP_CALIBRATION_IDENTITY"
SAFETY_SEMANTIC = (
    "CURRENT-SCOPE SOFTWARE VALIDATION ONLY — DEFERRED CHECKS REMAIN "
    "UNVALIDATED — NOT SAFETY CERTIFICATION, FULL INDUSTRIAL COLLISION "
    "VALIDATION, OR PHYSICAL CALIBRATION"
)
EXECUTION_SCOPE_SEMANTIC = (
    "READY UNDER CURRENT PROJECT VALIDATION SCOPE — "
    "DEFERRED CHECKS REMAIN UNVALIDATED"
)
TIMELINE_PASS = "COMMON_TIMELINE_STRUCTURALLY_VALID"
INCOMPLETE_STATUSES = frozenset({
    "NOT_EVALUATED",
    "NOT_CONFIGURED",
    "LIMIT_UNAVAILABLE",
    "UNAVAILABLE",
    "TIMEOUT",
    "ERROR",
    "STALE",
    "INCOMPLETE",
    "NOT_VALIDATED",
})
REQUIRED_CHECKS = (
    "plan_identity",
    "authoritative_inputs",
    "ik_complete",
    "joint_position_limits",
    "start_transition",
    "joint_continuity",
    "joint_jump",
    "fk_target",
    "relative_pose",
    "fixed_grasp",
    "common_timeline",
    "velocity",
    "self_collision",
    "inter_arm_collision",
    "sampled_path_collision",
)
CHECK_PRIORITY = REQUIRED_CHECKS
DEFERRED_CHECK_POLICY = (
    {
        "check": "object_collision",
        "roadmap_item": "P4.5",
        "reason": "DEFERRED UNTIL MAIN PROJECT COMPLETION",
    },
    {
        "check": "environment_collision",
        "roadmap_item": "P4.6",
        "reason": "DEFERRED UNTIL MAIN PROJECT COMPLETION",
    },
    {
        "check": "acceleration",
        "roadmap_item": "P4.20",
        "reason": (
            "ACCELERATION LIMIT VALIDATION DEFERRED; DIAGNOSTIC RETAINED"
        ),
    },
)
DEFERRED_CHECKS = tuple(item["check"] for item in DEFERRED_CHECK_POLICY)


class Phase4UnifiedValidationInputError(ValueError):
    """Raised when a unified validation request is structurally invalid."""


def _canonical(value: Any, path: str = "value") -> Any:
    if value is None or isinstance(value, (bool, str, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise Phase4UnifiedValidationInputError(f"{path} must be finite")
        return value
    if isinstance(value, Mapping):
        output = {}
        for key in sorted(value):
            if not isinstance(key, str):
                raise Phase4UnifiedValidationInputError(
                    f"{path} keys must be strings"
                )
            output[key] = _canonical(value[key], f"{path}.{key}")
        return output
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [_canonical(item, f"{path}[{index}]") for index, item in enumerate(value)]
    raise Phase4UnifiedValidationInputError(
        f"{path} contains unsupported {type(value).__name__} data"
    )


def compute_plan_fingerprint(
    plan: Any,
    *,
    start_state: Any = None,
    position_tolerance_m: Any = 0.001,
    orientation_tolerance_rad: Any = 0.001,
    max_joint_step_rad: Any = 0.05,
    validation_context: Any = None,
) -> str:
    """Return a deterministic SHA-256 identity for execution-relevant input."""
    if not isinstance(plan, Mapping):
        raise Phase4UnifiedValidationInputError("plan must be an object")
    identity = {
        "global_path": plan.get("global_path"),
        "combined_path": plan.get("combined_path"),
        "object_samples": plan.get("object_samples"),
        "combined_object_samples": plan.get("combined_object_samples"),
        "approach": plan.get("approach"),
        "approach_path": plan.get("approach_path"),
        "approach_samples": plan.get("approach_samples"),
        "grasp": plan.get("grasp"),
        "calibration": plan.get("calibration"),
        "waypoints": plan.get("waypoints"),
        "fixed_orientation_rpy_rad": plan.get("fixed_orientation_rpy_rad"),
        "common_timestamps_s": plan.get("common_timestamps_s"),
        "duration_s": plan.get("duration_s"),
        "combined_duration_s": plan.get("combined_duration_s"),
        "combined_timestamps_s": plan.get("combined_timestamps_s"),
        "trajectory_name": plan.get("trajectory_name"),
        "planning_start_state_rad": plan.get("planning_start_state_rad"),
        "planning_start_state_source": plan.get("planning_start_state_source"),
        "start_state": start_state,
        "position_tolerance_m": position_tolerance_m,
        "orientation_tolerance_rad": orientation_tolerance_rad,
        "max_joint_step_rad": max_joint_step_rad,
        "validation_context": validation_context,
    }
    serialized = json.dumps(
        _canonical(identity, "plan_identity"),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(serialized).hexdigest()


def _finite_joint_vector(value: Any, count: int) -> bool:
    return bool(
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes, bool))
        and len(value) == count
        and all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in value
        )
    )


def evaluate_ik_completeness(plan: Any) -> dict[str, Any]:
    """Map the accepted Phase-3 artifact to P4.1 without rerunning IK."""
    if not isinstance(plan, Mapping):
        return {
            "status": "FAIL",
            "reason_code": "MALFORMED_PLAN",
            "reason": "Phase-3 plan is not an object",
            "first_failure": {"sample_index": None},
        }
    if plan.get("ok") is not True or plan.get("planner_status") != "READY":
        failure = plan.get("failure") if isinstance(plan.get("failure"), Mapping) else {}
        return {
            "status": "FAIL",
            "reason_code": "PHASE3_PLAN_NOT_READY",
            "reason": str(failure.get("message") or "Phase-3 planner did not produce a READY plan"),
            "first_failure": deepcopy(dict(failure)),
        }
    object_samples = plan.get("object_samples")
    global_path = plan.get("global_path")
    if not isinstance(object_samples, list) or not isinstance(global_path, list):
        return {
            "status": "FAIL",
            "reason_code": "PHASE3_SAMPLE_ARRAY_MISSING",
            "reason": "object_samples and global_path must both exist",
            "first_failure": {"sample_index": None},
        }
    if not global_path or len(global_path) != len(object_samples):
        return {
            "status": "FAIL",
            "reason_code": "PHASE3_GLOBAL_PATH_INCOMPLETE",
            "reason": "Selected joint sample count does not equal required Object sample count",
            "required_object_sample_count": len(object_samples),
            "selected_joint_sample_count": len(global_path),
            "first_failure": {"sample_index": len(global_path)},
        }
    for index, point in enumerate(global_path):
        if not isinstance(point, Mapping):
            valid = False
        else:
            left, right, combined = point.get("left"), point.get("right"), point.get("combined")
            valid = (
                _finite_joint_vector(left, 6)
                and _finite_joint_vector(right, 6)
                and _finite_joint_vector(combined, 12)
                and tuple(float(item) for item in combined)
                == tuple(float(item) for item in (*left, *right))
            )
        if not valid:
            return {
                "status": "FAIL",
                "reason_code": "PHASE3_SELECTED_SAMPLE_INVALID",
                "reason": f"global_path[{index}] must contain finite canonical Left[6] + Right[6]",
                "first_failure": {"sample_index": index, "layer_index": index},
            }
    combined_path = plan.get("combined_path")
    combined_objects = plan.get("combined_object_samples")
    if combined_path is not None or combined_objects is not None:
        if not isinstance(combined_path, list) or not isinstance(combined_objects, list) or not combined_path or len(combined_path) != len(combined_objects):
            return {
                "status": "FAIL",
                "reason_code": "PHASE3_COMBINED_PATH_INCOMPLETE",
                "reason": "Combined approach + rigid path must have matching joint and target samples",
                "first_failure": {"sample_index": None},
            }
        for index, point in enumerate(combined_path):
            if not isinstance(point, Mapping):
                valid = False
            else:
                left, right, combined = point.get("left"), point.get("right"), point.get("combined")
                valid = (
                    _finite_joint_vector(left, 6) and _finite_joint_vector(right, 6)
                    and _finite_joint_vector(combined, 12)
                    and tuple(float(item) for item in combined)
                    == tuple(float(item) for item in (*left, *right))
                )
            if not valid:
                return {
                    "status": "FAIL",
                    "reason_code": "PHASE3_COMBINED_SAMPLE_INVALID",
                    "reason": f"combined_path[{index}] must contain finite canonical Left[6] + Right[6]",
                    "first_failure": {"sample_index": index},
                }
    checked_path = combined_path if isinstance(combined_path, list) else global_path
    return {
        "status": "PASS",
        "source": "PHASE3_COMBINED_SELECTED_PATH" if isinstance(combined_path, list) else "PHASE3_GLOBAL_SELECTED_PATH",
        "required_object_sample_count": len(combined_objects) if isinstance(combined_objects, list) else len(object_samples),
        "selected_joint_sample_count": len(checked_path),
        "checked_joint_count": len(checked_path) * 12,
        "first_failure": None,
    }


def validate_all_joint_positions(
    plan: Any,
    limits: CanonicalJointPositionLimits | None,
) -> dict[str, Any]:
    """Apply existing canonical model position limits to every selected sample."""
    if limits is None:
        return {
            "status": "UNAVAILABLE",
            "reason_code": "JOINT_POSITION_LIMITS_UNAVAILABLE",
            "reason": "Canonical joint-position limits are unavailable",
            "limit_source": "UNAVAILABLE",
            "first_failure": None,
        }
    if not isinstance(limits, CanonicalJointPositionLimits):
        raise TypeError("limits must be CanonicalJointPositionLimits or None")
    path = (
        plan.get("combined_path")
        if isinstance(plan, Mapping) and isinstance(plan.get("combined_path"), list)
        else (plan.get("global_path") if isinstance(plan, Mapping) else None)
    )
    if not isinstance(path, list):
        return {
            "status": "FAIL",
            "reason_code": "GLOBAL_PATH_MISSING",
            "reason": "selected motion path is unavailable for joint-position validation",
            "limit_source": "CANONICAL_MOVEIT_MODEL_POSITION_LIMITS",
            "first_failure": None,
        }
    checked = 0
    violations = []
    for sample_index, point in enumerate(path):
        combined = point.get("combined") if isinstance(point, Mapping) else None
        if not _finite_joint_vector(combined, 12):
            violations.append({
                "sample_index": sample_index,
                "reason_code": "INVALID_JOINT_VECTOR",
                "reason": "Selected sample does not contain 12 finite joints",
            })
            continue
        for joint_index, value in enumerate(combined):
            checked += 1
            lower = limits.lower_rad[joint_index]
            upper = limits.upper_rad[joint_index]
            if float(value) < lower or float(value) > upper:
                violations.append({
                    "sample_index": sample_index,
                    "joint_index": joint_index,
                    "joint_name": DUAL_ARM_JOINT_ORDER[joint_index],
                    "actual_value_rad": float(value),
                    "lower_limit_rad": lower,
                    "upper_limit_rad": upper,
                    "reason_code": "JOINT_POSITION_LIMIT_VIOLATION",
                    "reason": "Joint position is outside the canonical model limit",
                })
    return {
        "status": "FAIL" if violations else "PASS",
        "limit_source": "CANONICAL_MOVEIT_MODEL_POSITION_LIMITS",
        "checked_sample_count": len(path),
        "checked_joint_count": checked,
        "first_failure": deepcopy(violations[0]) if violations else None,
        "violations": violations,
    }


def _mapping(value: Any) -> dict[str, Any]:
    return deepcopy(dict(value)) if isinstance(value, Mapping) else {}


def _status_check(status: Any, evidence: Any, **extra: Any) -> dict[str, Any]:
    normalized = str(status or "NOT_EVALUATED")
    return {"status": normalized, "evidence": deepcopy(evidence), **extra}


def _timeline_check(phase4a: Mapping[str, Any]) -> dict[str, Any]:
    evidence = _mapping(phase4a.get("timeline"))
    status = "PASS" if evidence.get("status") == TIMELINE_PASS else evidence.get("status")
    return _status_check(status, evidence)


def _start_check(
    phase4a: Mapping[str, Any], start_state: Any
) -> dict[str, Any]:
    evidence = _mapping(phase4a.get("start_transition"))
    if start_state is None:
        return _status_check(
            "NOT_EVALUATED",
            evidence,
            reason="Explicit start state is required and has not been evaluated",
        )
    status = evidence.get("status")
    if status == "EVALUATED":
        suspicious = _mapping(evidence.get("suspicious_jumps"))
        status = "PASS" if suspicious.get("suspicious_transition_count", 0) == 0 else "FAIL"
    return _status_check(status, evidence)


def _jump_check(phase4a: Mapping[str, Any]) -> dict[str, Any]:
    continuity = _mapping(phase4a.get("continuity"))
    jumps = _mapping(continuity.get("suspicious_jumps"))
    if jumps.get("analysis_completed") is True:
        status = "PASS" if jumps.get("suspicious_transition_count") == 0 else "FAIL"
    else:
        status = "NOT_EVALUATED"
    return _status_check(status, {"summary": continuity.get("summary"), **jumps})


def _acceleration_check(phase4a: Mapping[str, Any]) -> dict[str, Any]:
    evidence = _mapping(phase4a.get("acceleration"))
    if evidence.get("limit_status") == "LIMIT_UNAVAILABLE":
        status = "LIMIT_UNAVAILABLE"
    else:
        status = evidence.get("limit_validation") or "NOT_EVALUATED"
    return _status_check(status, evidence)


def _collision_parts(phase4b: Mapping[str, Any]) -> tuple[dict[str, Any], ...]:
    robot = _mapping(phase4b.get("robot_collision"))
    object_scene = _mapping(phase4b.get("object_scene"))
    environment_scene = _mapping(phase4b.get("environment_scene"))
    object_collision = _mapping(phase4b.get("object_collision"))
    environment_collision = _mapping(phase4b.get("environment_collision"))
    self_collision = _mapping(robot.get("self_collision"))
    inter_arm = _mapping(robot.get("inter_arm_collision"))
    sampled = _mapping(robot.get("sampled_path"))
    object_status = (
        "NOT_CONFIGURED"
        if object_scene.get("status") == "NOT_CONFIGURED"
        else object_collision.get("status")
    )
    environment_status = (
        "NOT_CONFIGURED"
        if environment_scene.get("status") == "NOT_CONFIGURED"
        else environment_collision.get("status")
    )
    return (
        _status_check(self_collision.get("status"), self_collision),
        _status_check(inter_arm.get("status"), inter_arm),
        _status_check(
            sampled.get("status"),
            sampled,
            max_joint_step_rad=_mapping(phase4b.get("synchronized_sampling")).get("max_joint_step_rad"),
        ),
        _status_check(object_status, object_collision, scene=object_scene),
        _status_check(environment_status, environment_collision, scene=environment_scene),
    )


def _slug(value: str) -> str:
    return re.sub(r"[^A-Z0-9]+", "_", value.upper()).strip("_")


def _blocking_reason(name: str, check: Mapping[str, Any]) -> str:
    status = str(check.get("status") or "NOT_EVALUATED")
    exact = {
        ("object_collision", "NOT_CONFIGURED"): "OBJECT_COLLISION_GEOMETRY_NOT_CONFIGURED",
        ("environment_collision", "NOT_CONFIGURED"): "ENVIRONMENT_COLLISION_SCENE_NOT_CONFIGURED",
        ("acceleration", "LIMIT_UNAVAILABLE"): "ACCELERATION_LIMIT_UNAVAILABLE",
        ("start_transition", "NOT_EVALUATED"): "START_TRANSITION_NOT_EVALUATED",
        ("plan_identity", "STALE"): "STALE_VALIDATION",
    }
    return exact.get((name, status), f"{_slug(name)}_{_slug(status)}")


def _failure_evidence(name: str, check: Mapping[str, Any]) -> dict[str, Any]:
    evidence = _mapping(check.get("evidence"))
    first = check.get("first_failure")
    if not isinstance(first, Mapping):
        first = evidence.get("first_failure")
    if not isinstance(first, Mapping):
        failures = evidence.get("failures")
        first = failures[0] if isinstance(failures, list) and failures else {}
    if not isinstance(first, Mapping) or not first:
        violations = evidence.get("violations")
        first = violations[0] if isinstance(violations, list) and violations else {}
    if not isinstance(first, Mapping) or not first:
        samples = evidence.get("samples")
        first = next(
            (
                sample for sample in samples
                if isinstance(sample, Mapping) and sample.get("status") == "FAIL"
            ),
            {},
        ) if isinstance(samples, list) else {}
    if not isinstance(first, Mapping) or not first:
        transitions = evidence.get("transitions")
        first = transitions[0] if isinstance(transitions, list) and transitions else {}
    if not isinstance(first, Mapping):
        first = {}
    reason_code = _blocking_reason(name, check)
    reason = (
        check.get("reason")
        or first.get("reason")
        or evidence.get("reason")
        or reason_code.replace("_", " ").title()
    )
    location_keys = (
        "sample_index", "point_index", "layer_index", "from_sample_index",
        "to_sample_index", "segment_index", "interior_sample_index", "alpha",
        "time_from_start_s", "joint_index", "joint_name", "category",
        "body_1", "body_2", "depth_m", "location_kind",
    )
    return {
        "check": name,
        "status": check.get("status"),
        "reason_code": reason_code,
        "reason": str(reason),
        "location": {key: first.get(key) for key in location_keys if key in first},
        "evidence": deepcopy(dict(first)),
    }


def build_phase4_unified_report(
    plan: Any,
    phase4a_result: Any,
    phase4b_result: Any,
    *,
    joint_position_limits: CanonicalJointPositionLimits | None,
    start_state: Any = None,
    position_tolerance_m: Any = 0.001,
    orientation_tolerance_rad: Any = 0.001,
    max_joint_step_rad: Any = 0.05,
    validation_context: Any = None,
    phase4a_plan_fingerprint: str | None = None,
    phase4b_plan_fingerprint: str | None = None,
    authoritative_input_validation: Any = None,
) -> dict[str, Any]:
    """Combine one plan's accepted validators into one deterministic report."""
    if not isinstance(phase4a_result, Mapping) or not isinstance(phase4b_result, Mapping):
        raise Phase4UnifiedValidationInputError(
            "Phase-4A and Phase-4B results must be objects"
        )
    fingerprint = compute_plan_fingerprint(
        plan,
        start_state=start_state,
        position_tolerance_m=position_tolerance_m,
        orientation_tolerance_rad=orientation_tolerance_rad,
        max_joint_step_rad=max_joint_step_rad,
        validation_context=validation_context,
    )
    component_fingerprints = {
        "phase4a": phase4a_plan_fingerprint or fingerprint,
        "phase4b": phase4b_plan_fingerprint or fingerprint,
    }
    identity_ok = all(value == fingerprint for value in component_fingerprints.values())
    phase4a = _mapping(phase4a_result)
    phase4b = _mapping(phase4b_result)
    self_collision, inter_arm, sampled, object_collision, environment_collision = (
        _collision_parts(phase4b)
    )
    continuity = _mapping(phase4a.get("continuity"))
    checks = {
        "plan_identity": _status_check(
            "PASS" if identity_ok else "STALE",
            component_fingerprints,
            reason=(None if identity_ok else "Component result fingerprint does not match the current plan"),
        ),
        "authoritative_inputs": _status_check(
            (
                authoritative_input_validation.get("status")
                if isinstance(authoritative_input_validation, Mapping)
                else "NOT_EVALUATED"
            ),
            authoritative_input_validation,
            reason=(
                authoritative_input_validation.get("reason")
                if isinstance(authoritative_input_validation, Mapping)
                else "Current authoritative grasp/calibration was not checked"
            ),
        ),
        "ik_complete": evaluate_ik_completeness(plan),
        "joint_position_limits": validate_all_joint_positions(
            plan, joint_position_limits
        ),
        "self_collision": self_collision,
        "inter_arm_collision": inter_arm,
        "object_collision": object_collision,
        "environment_collision": environment_collision,
        "sampled_path_collision": sampled,
        "start_transition": _start_check(phase4a, start_state),
        "joint_continuity": _status_check(
            continuity.get("status"), continuity
        ),
        "joint_jump": _jump_check(phase4a),
        "fk_target": _status_check(
            _mapping(phase4a.get("fk")).get("status"), phase4a.get("fk")
        ),
        "relative_pose": _status_check(
            _mapping(phase4a.get("relative_pose")).get("status"),
            phase4a.get("relative_pose"),
        ),
        "fixed_grasp": _status_check(
            _mapping(phase4a.get("fixed_grasp")).get("status"),
            phase4a.get("fixed_grasp"),
        ),
        "common_timeline": _timeline_check(phase4a),
        "velocity": _status_check(
            _mapping(phase4a.get("velocity")).get("status"),
            phase4a.get("velocity"),
        ),
        "acceleration": _acceleration_check(phase4a),
    }
    required_nonpass = [
        name for name in REQUIRED_CHECKS if checks[name]["status"] != "PASS"
    ]
    required_failed = [
        name for name in required_nonpass if checks[name]["status"] == "FAIL"
    ]
    overall = (
        "FAIL" if required_failed
        else ("INCOMPLETE" if required_nonpass else "PASS")
    )
    blocking_reasons = [
        _blocking_reason(name, checks[name]) for name in required_nonpass
    ]
    first_failure = (
        _failure_evidence(required_nonpass[0], checks[required_nonpass[0]])
        if required_nonpass else None
    )
    execution_ready = overall == "PASS" and not blocking_reasons
    deferred_findings = [
        {
            **deepcopy(policy),
            "status": checks[policy["check"]]["status"],
            "validated": checks[policy["check"]]["status"] == "PASS",
        }
        for policy in DEFERRED_CHECK_POLICY
    ]
    collision_summary = _mapping(phase4b.get("collision_summary"))
    report = {
        "report_version": REPORT_VERSION,
        "plan_fingerprint": fingerprint,
        "component_fingerprints": component_fingerprints,
        "plan_only": True,
        "overall_status": overall,
        "scope_policy": {
            "policy": "CURRENT_MAIN_PROJECT_SCOPE",
            "required_checks": list(REQUIRED_CHECKS),
            "deferred_checks": deepcopy(deferred_findings),
        },
        "checks": checks,
        "first_failure": first_failure,
        "diagnostics": {
            "authoritative_inputs": deepcopy(
                checks["authoritative_inputs"]["evidence"]
            ),
            "collision": {
                "summary": collision_summary,
                "self_collision": deepcopy(checks["self_collision"]),
                "inter_arm_collision": deepcopy(checks["inter_arm_collision"]),
                "object_collision": deepcopy(checks["object_collision"]),
                "environment_collision": deepcopy(checks["environment_collision"]),
                "sampled_path_collision": deepcopy(checks["sampled_path_collision"]),
            },
            "joint_jump": deepcopy(checks["joint_jump"]["evidence"]),
            "relative_pose": {
                "fk_target": deepcopy(checks["fk_target"]["evidence"]),
                "relative_pose": deepcopy(checks["relative_pose"]["evidence"]),
                "fixed_grasp": deepcopy(checks["fixed_grasp"]["evidence"]),
            },
            "timing_dynamics": {
                "timeline": deepcopy(checks["common_timeline"]["evidence"]),
                "velocity": deepcopy(checks["velocity"]["evidence"]),
                "acceleration": deepcopy(checks["acceleration"]["evidence"]),
            },
        },
        "execution_gate": {
            "execution_ready": execution_ready,
            "execution_ready_label": "YES" if execution_ready else "NO",
            "status": "READY" if execution_ready else "BLOCKED",
            "blocking_reasons": blocking_reasons,
            "semantic": EXECUTION_SCOPE_SEMANTIC,
        },
        "deferred_warnings": deepcopy(deferred_findings),
        "components": {
            "phase4a": phase4a,
            "phase4b": phase4b,
        },
        "warnings": [
            SAFETY_SEMANTIC,
            (
                "FAIL-CLOSED FOR REQUIRED CURRENT-SCOPE CHECKS: INCOMPLETE, "
                "ERROR, TIMEOUT, UNAVAILABLE, OR STALE IS NOT PASS"
            ),
            (
                "P4.5 OBJECT COLLISION, P4.6 ENVIRONMENT COLLISION, AND "
                "P4.20 ACCELERATION HARD-LIMIT VALIDATION ARE DEFERRED"
            ),
        ],
        "first_failure_priority": list(CHECK_PRIORITY),
    }
    return report


def evaluate_execution_gate(
    report: Any,
    current_plan_fingerprint: Any,
    *,
    validation_running: bool = False,
) -> dict[str, Any]:
    """Re-evaluate the authoritative fail-closed gate at the point of use."""
    reasons: list[str] = []
    validated = report.get("plan_fingerprint") if isinstance(report, Mapping) else None
    if validation_running:
        reasons.append("VALIDATION_RUNNING")
    if not isinstance(report, Mapping):
        reasons.append("UNIFIED_VALIDATION_REPORT_MISSING")
    else:
        if report.get("overall_status") != "PASS":
            reasons.append(f"OVERALL_STATUS_{_slug(str(report.get('overall_status') or 'MISSING'))}")
        scope_policy = report.get("scope_policy")
        if (
            report.get("report_version") != REPORT_VERSION
            or not isinstance(scope_policy, Mapping)
            or scope_policy.get("required_checks") != list(REQUIRED_CHECKS)
        ):
            reasons.append("REPORT_SCOPE_POLICY_MISMATCH")
        checks = report.get("checks")
        if not isinstance(checks, Mapping) or any(
            not isinstance(checks.get(name), Mapping)
            or checks[name].get("status") != "PASS"
            for name in REQUIRED_CHECKS
        ):
            reasons.append("REQUIRED_VALIDATION_NOT_ALL_PASS")
        embedded = report.get("execution_gate")
        if (
            not isinstance(embedded, Mapping)
            or embedded.get("execution_ready") is not True
            or embedded.get("execution_ready_label") != "YES"
        ):
            reasons.append("REPORT_EXECUTION_READY_NOT_YES")
    if not isinstance(current_plan_fingerprint, str) or not current_plan_fingerprint:
        reasons.append("CURRENT_PLAN_FINGERPRINT_MISSING")
    elif current_plan_fingerprint != validated:
        reasons.append("PLAN_FINGERPRINT_MISMATCH")
    reasons = list(dict.fromkeys(reasons))
    ready = not reasons
    return {
        "execution_ready": ready,
        "execution_ready_label": "YES" if ready else "NO",
        "status": "READY" if ready else "BLOCKED",
        "blocking_reasons": reasons,
        "validated_plan_fingerprint": validated,
        "current_plan_fingerprint": current_plan_fingerprint,
        "stale": "PLAN_FINGERPRINT_MISMATCH" in reasons,
        "semantic": EXECUTION_SCOPE_SEMANTIC,
    }
