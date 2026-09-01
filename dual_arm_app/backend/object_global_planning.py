"""Pure Phase-3B orchestration and JSON contracts for global object planning.

PLAN ONLY. This module has no ROS, FastAPI, JAKA driver, publisher, action, or
motion dependency. MoveIt calls remain behind ``ObjectTrajectoryIkAdapter``.
The returned optimum is exact over the generated layered candidate graph only.
"""

from __future__ import annotations

import json
import logging
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Mapping, Sequence

from dual_arm_app.backend.object_grasp_model import (
    ObjectGraspModel,
    RigidTransform,
    compose,
    inverse,
)
from dual_arm_app.backend.object_trajectory import (
    ObjectTrajectory,
    ObjectTrajectorySample,
    ObjectWaypoint,
    generate_multi_waypoint_object_trajectory,
)
from dual_arm_app.backend.planning_start_state_config import (
    PLANNING_START_STATE_COMBINED_RAD,
    PLANNING_START_STATE_SOURCE,
    PlanningStartStateConfigError,
    normalize_planning_start_state_rad,
)
from dual_arm_app.backend.object_trajectory_ik import (
    DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG,
    DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    DUAL_ARM_JOINT_ORDER,
    GLOBAL_GRAPH_OPTIMALITY_SCOPE,
    CanonicalJointPositionLimits,
    IkCandidateExplorationConfig,
    ObjectTrajectoryIkAdapter,
    TrajectoryCandidateNode,
    TrajectoryEdgeFeasibility,
    analyze_joint_transition,
    analyze_suspicious_joint_jumps,
    build_object_trajectory_candidate_graph,
    compare_greedy_and_global,
    default_graph_edge_feasibility,
    graph_edge_between,
    joint_delta_rad,
)


WEB_OPTIMALITY_SCOPE = "Exact optimum over generated layered candidate graph"
PLANNER_SOURCE = "GLOBAL GRAPH SEARCH — GENERATED GRAPH OPTIMUM"
LOCKED_OPERATOR_GRASP_SOURCE = "LOCKED OPERATOR GRASP"
PLAN_ONLY_WARNINGS = (
    "PLAN ONLY",
    "NO ROBOT EXECUTION",
    "NOT PHASE-4 VALIDATED",
    "NOT SAFE-TO-EXECUTE CLAIM",
)
WEB_GLOBAL_CANDIDATE_EXPLORATION_PROFILE = (
    "WEB RUNTIME — BOUNDED IK SEED EXPLORATION"
)
PHASE3_IK_DEBUG_CAPTURE_ENV = "JAKA_PHASE3_IK_DEBUG_CAPTURE_PATH"
PHASE3_IK_DEBUG_CAPTURE_SCHEMA = "phase3_ik_branch_capture_v1"
PHASE3_DEBUG_SUSPICIOUS_RAW_STEP_RAD = 1.0
PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD = 1.0
PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD = 1.0
PHASE3_CONTINUITY_GATE_REASON = "IK_BRANCH_CONTINUITY_GATE"
RELATIVE_WAYPOINT_PROFILE_TIMING_SEMANTIC = (
    "Wn profile applies to Wn -> Wn+1; 100% is the existing baseline; "
    "lower values only slow/stretch; relative planning scales, not manufacturer "
    "hard limits or Phase-4 acceleration certification"
)
LOGGER = logging.getLogger(__name__)
WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG = IkCandidateExplorationConfig(
    perturbation_offset_rad=(
        DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.perturbation_offset_rad
    ),
    max_attempts_per_arm=3,
    duplicate_tolerance_rad=(
        DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.duplicate_tolerance_rad
    ),
    ranking_tie_tolerance=(
        DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.ranking_tie_tolerance
    ),
)


class ObjectGlobalPlanInputError(ValueError):
    """Raised for an invalid or ambiguous Web planning request."""


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ObjectGlobalPlanInputError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ObjectGlobalPlanInputError(f"{label} must be a finite number")
    return converted


def normalize_waypoint_profile_percent(value: Any, label: str) -> float:
    """Normalize one relative planning scale without silently clamping it."""
    checked = 100.0 if value is None else _finite_number(value, label)
    if checked <= 0.0 or checked > 100.0:
        raise ObjectGlobalPlanInputError(f"{label} must be within (0, 100]")
    return checked


def relative_segment_duration_s(
    base_duration_s: Any,
    speed_percent: Any,
    acceleration_percent: Any,
) -> float:
    """Stretch a baseline segment using conservative normalized path scaling."""
    base = _finite_number(base_duration_s, "base_duration_s")
    if base <= 0.0:
        raise ObjectGlobalPlanInputError("base_duration_s must be greater than zero")
    speed = normalize_waypoint_profile_percent(speed_percent, "speed_percent")
    acceleration = normalize_waypoint_profile_percent(
        acceleration_percent, "acceleration_percent"
    )
    time_scale = max(100.0 / speed, math.sqrt(100.0 / acceleration))
    return base * time_scale


def _pose6(value: Any, label: str) -> dict[str, tuple[float, float, float]]:
    if not isinstance(value, Mapping):
        raise ObjectGlobalPlanInputError(f"{label} must be an object")
    return {
        "translation_m": _vector3(value.get("translation_m"), f"{label}.translation_m"),
        "rpy_rad": _vector3(value.get("rpy_rad"), f"{label}.rpy_rad"),
    }


def _normalize_approach_waypoints(value: Any) -> tuple[dict[str, Any], ...]:
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ObjectGlobalPlanInputError("approach_waypoints must be an array")
    result = []
    seen = set()
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise ObjectGlobalPlanInputError(f"approach_waypoints[{index}] must be an object")
        name = _nonempty_string(raw.get("identifier"), f"approach_waypoints[{index}].identifier")
        folded = name.casefold()
        if folded in seen:
            raise ObjectGlobalPlanInputError("approach waypoint identifiers must be unique")
        seen.add(folded)
        result.append({
            "identifier": name,
            "left": _pose6(raw.get("left"), f"approach_waypoints[{index}].left"),
            "right": _pose6(raw.get("right"), f"approach_waypoints[{index}].right"),
        })
    return tuple(result)


def _vector3(value: Any, label: str) -> tuple[float, float, float]:
    if (
        isinstance(value, (str, bytes, bool))
        or not isinstance(value, Sequence)
        or len(value) != 3
    ):
        raise ObjectGlobalPlanInputError(f"{label} must contain exactly 3 values")
    return tuple(
        _finite_number(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    )  # type: ignore[return-value]


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ObjectGlobalPlanInputError(f"{label} must be a non-empty string")
    return value.strip()


def _lock_generation(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ObjectGlobalPlanInputError(f"{label} must be an integer >= 1")
    return value


def normalize_object_global_plan_request(payload: Any) -> dict[str, Any]:
    """Validate and defensively copy one 6D Center planning request."""
    if not isinstance(payload, dict):
        raise ObjectGlobalPlanInputError("planning request must be an object")
    raw_name = payload.get("name")
    if not isinstance(raw_name, str) or not raw_name.strip():
        raise ObjectGlobalPlanInputError("name must be a non-empty string")
    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ObjectGlobalPlanInputError("at least 2 ordered Object Waypoints are required")
    waypoints: list[ObjectWaypoint] = []
    waypoint_profiles: list[dict[str, float]] = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, dict):
            raise ObjectGlobalPlanInputError(f"waypoints[{index}] must be an object")
        try:
            waypoint_profiles.append({
                "speed_percent": normalize_waypoint_profile_percent(
                    raw_waypoint.get("speed_percent"),
                    f"waypoints[{index}].speed_percent",
                ),
                "acceleration_percent": normalize_waypoint_profile_percent(
                    raw_waypoint.get("acceleration_percent"),
                    f"waypoints[{index}].acceleration_percent",
                ),
            })
            waypoints.append(ObjectWaypoint(
                raw_waypoint.get("identifier"),
                raw_waypoint.get("translation_m"),
                _vector3(raw_waypoint.get("rpy_rad"), f"waypoints[{index}].rpy_rad")
                if raw_waypoint.get("rpy_rad") is not None else None,
            ))
        except (TypeError, ValueError) as error:
            raise ObjectGlobalPlanInputError(f"waypoints[{index}]: {error}") from error
    identifiers = tuple(waypoint.identifier.casefold() for waypoint in waypoints)
    if len(set(identifiers)) != len(identifiers):
        raise ObjectGlobalPlanInputError("Object Waypoint identifiers must be unique")

    orientation = _vector3(
        payload.get("fixed_orientation_rpy_rad"),
        "fixed_orientation_rpy_rad",
    )
    segment_duration_s = _finite_number(
        payload.get("segment_duration_s"),
        "segment_duration_s",
    )
    if segment_duration_s <= 0.0:
        raise ObjectGlobalPlanInputError("segment_duration_s must be greater than zero")
    segment_durations_s = tuple(
        relative_segment_duration_s(
            segment_duration_s,
            waypoint_profiles[index]["speed_percent"],
            waypoint_profiles[index]["acceleration_percent"],
        )
        for index in range(len(waypoints) - 1)
    )
    samples_per_segment = payload.get("samples_per_segment")
    if (
        isinstance(samples_per_segment, bool)
        or not isinstance(samples_per_segment, int)
        or samples_per_segment < 2
    ):
        raise ObjectGlobalPlanInputError("samples_per_segment must be an integer >= 2")
    if "candidate_attempts_per_arm" in payload:
        candidate_attempts_per_arm = payload["candidate_attempts_per_arm"]
        if (
            isinstance(candidate_attempts_per_arm, bool)
            or not isinstance(candidate_attempts_per_arm, int)
            or not 1 <= candidate_attempts_per_arm <= 13
        ):
            raise ObjectGlobalPlanInputError(
                "candidate_attempts_per_arm must be an integer within [1, 13]"
            )
    else:
        candidate_attempts_per_arm = (
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.max_attempts_per_arm
        )
    if "initial_joint_state_rad" in payload:
        try:
            initial_joint_state_rad = normalize_planning_start_state_rad(
                payload["initial_joint_state_rad"]
            )
        except PlanningStartStateConfigError as error:
            raise ObjectGlobalPlanInputError(
                f"initial_joint_state_rad: {error}"
            ) from error
        raw_start_source = payload.get(
            "planning_start_state_source",
            "EXPLICIT REQUEST OVERRIDE",
        )
    else:
        initial_joint_state_rad = tuple(PLANNING_START_STATE_COMBINED_RAD)
        raw_start_source = PLANNING_START_STATE_SOURCE
    if not isinstance(raw_start_source, str) or not raw_start_source.strip():
        raise ObjectGlobalPlanInputError(
            "planning_start_state_source must be a non-empty string"
        )
    return {
        "name": raw_name.strip(),
        "waypoints": tuple(waypoints),
        "waypoint_profiles": tuple(waypoint_profiles),
        "approach_waypoints": _normalize_approach_waypoints(payload.get("approach_waypoints", [])),
        "fixed_orientation_rpy_rad": orientation,
        "segment_duration_s": segment_duration_s,
        "segment_durations_s": segment_durations_s,
        "samples_per_segment": samples_per_segment,
        "candidate_attempts_per_arm": candidate_attempts_per_arm,
        "initial_joint_state_rad": initial_joint_state_rad,
        "planning_start_state_source": raw_start_source.strip(),
        "expected_grasp_content_revision": _nonempty_string(
            payload.get("expected_grasp_content_revision"),
            "expected_grasp_content_revision",
        ),
        "expected_lock_generation": _lock_generation(
            payload.get("expected_lock_generation"),
            "expected_lock_generation",
        ),
        "expected_lock_revision": _nonempty_string(
            payload.get("expected_lock_revision"),
            "expected_lock_revision",
        ),
        "expected_calibration_revision": _nonempty_string(
            payload.get("expected_calibration_revision"),
            "expected_calibration_revision",
        ),
        "expected_model_calibration_revision": _nonempty_string(
            payload.get("expected_model_calibration_revision"),
            "expected_model_calibration_revision",
        ),
    }


def _normalize_planning_authority(
    authority: Any,
    grasp_model: ObjectGraspModel,
) -> dict[str, Any]:
    """Validate backend-produced authority metadata against the supplied model."""
    if not isinstance(authority, dict):
        raise ObjectGlobalPlanInputError("planning_authority must be an object")
    if authority.get("grasp_status") != "GRASP_LOCKED":
        raise ObjectGlobalPlanInputError("planning authority grasp must be GRASP_LOCKED")
    if authority.get("grasp_source") != LOCKED_OPERATOR_GRASP_SOURCE:
        raise ObjectGlobalPlanInputError(
            f"planning authority grasp source must be {LOCKED_OPERATOR_GRASP_SOURCE}"
        )
    left = authority.get("left")
    right = authority.get("right")
    if not isinstance(left, dict) or not isinstance(right, dict):
        raise ObjectGlobalPlanInputError("planning authority must contain Left and Right grasp poses")
    left_translation = _vector3(left.get("translation_m"), "planning_authority.left.translation_m")
    left_rpy = _vector3(left.get("rpy_rad"), "planning_authority.left.rpy_rad")
    right_translation = _vector3(right.get("translation_m"), "planning_authority.right.translation_m")
    right_rpy = _vector3(right.get("rpy_rad"), "planning_authority.right.rpy_rad")
    left_transform = RigidTransform.from_translation_rpy(left_translation, left_rpy)
    right_transform = RigidTransform.from_translation_rpy(right_translation, right_rpy)
    if not left_transform.almost_equal(grasp_model.object_T_left):
        raise ObjectGlobalPlanInputError("planning authority Left transform does not match grasp_model")
    if not right_transform.almost_equal(grasp_model.object_T_right):
        raise ObjectGlobalPlanInputError("planning authority Right transform does not match grasp_model")
    calibration_revision = _nonempty_string(
        authority.get("calibration_revision"), "planning_authority.calibration_revision"
    )
    model_revision = _nonempty_string(
        authority.get("model_calibration_revision"),
        "planning_authority.model_calibration_revision",
    )
    if calibration_revision != model_revision or authority.get("calibration_revision_status") != "MATCH":
        raise ObjectGlobalPlanInputError("planning authority model/calibration revision must MATCH")
    expected_relative = compose(inverse(left_transform), right_transform)
    return {
        "grasp_status": "GRASP_LOCKED",
        "grasp_source": LOCKED_OPERATOR_GRASP_SOURCE,
        "grasp_content_revision": _nonempty_string(
            authority.get("grasp_content_revision"),
            "planning_authority.grasp_content_revision",
        ),
        "lock_generation": _lock_generation(
            authority.get("lock_generation"), "planning_authority.lock_generation"
        ),
        "lock_revision": _nonempty_string(
            authority.get("lock_revision"), "planning_authority.lock_revision"
        ),
        "left": _transform_payload(left_transform, rpy_rad=left_rpy),
        "right": _transform_payload(right_transform, rpy_rad=right_rpy),
        "expected_left_T_right": _transform_payload(expected_relative),
        "calibration_revision": calibration_revision,
        "model_calibration_revision": model_revision,
        "calibration_revision_status": "MATCH",
        "calibration_state": _nonempty_string(
            authority.get("calibration_state"), "planning_authority.calibration_state"
        ),
        "physical_calibration": _nonempty_string(
            authority.get("physical_calibration"), "planning_authority.physical_calibration"
        ),
        "physically_calibrated": authority.get("physically_calibrated") is True,
        "tcp_tool_contract_status": _nonempty_string(
            authority.get("tcp_tool_contract_status"),
            "planning_authority.tcp_tool_contract_status",
        ),
    }


def _web_candidate_exploration_config(
    candidate_attempts_per_arm: int,
) -> IkCandidateExplorationConfig:
    """Copy the accepted Web profile with only its explicit attempt budget changed."""
    return IkCandidateExplorationConfig(
        perturbation_offset_rad=(
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.perturbation_offset_rad
        ),
        max_attempts_per_arm=candidate_attempts_per_arm,
        duplicate_tolerance_rad=(
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.duplicate_tolerance_rad
        ),
        ranking_tie_tolerance=(
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.ranking_tie_tolerance
        ),
    )


def _transform_payload(
    transform: RigidTransform,
    *,
    rpy_rad: Sequence[float] | None = None,
) -> dict[str, Any]:
    payload = {
        "translation_m": list(transform.translation_m),
        "matrix": [list(row) for row in transform.matrix],
    }
    if rpy_rad is not None:
        payload["rpy_rad"] = list(rpy_rad)
    return payload


def _failure_payload(failure: Any) -> dict[str, Any] | None:
    if failure is None:
        return None
    return {
        "reason": failure.reason,
        "failed_sample_index": failure.failed_sample_index,
        "message": failure.message,
        "left_candidate_count": failure.left_candidate_count,
        "right_candidate_count": failure.right_candidate_count,
        "candidate_pair_count": failure.candidate_pair_count,
        "valid_pair_count": failure.valid_pair_count,
    }


def _maximum_path_transition(path: Sequence[Any]) -> dict[str, Any]:
    maximum_value = 0.0
    maximum_index = 0
    from_sample = None
    to_sample = None
    for previous, current in zip(path, path[1:]):
        delta = joint_delta_rad(
            current.combined_joint_positions_rad,
            previous.combined_joint_positions_rad,
        )
        candidate_index = max(range(12), key=lambda index: abs(delta[index]))
        candidate_value = abs(delta[candidate_index])
        if candidate_value > maximum_value:
            maximum_value = candidate_value
            maximum_index = candidate_index
            from_sample = previous.sample_index
            to_sample = current.sample_index
    return {
        "value_rad": maximum_value,
        "joint_index": maximum_index if len(path) > 1 else None,
        "joint_name": DUAL_ARM_JOINT_ORDER[maximum_index] if len(path) > 1 else None,
        "from_sample_index": from_sample,
        "to_sample_index": to_sample,
    }


def phase3_preview_continuity_edge_feasibility(
    from_node: TrajectoryCandidateNode,
    to_node: TrajectoryCandidateNode,
) -> TrajectoryEdgeFeasibility:
    """Reject visually radical graph edges without changing IK solutions.

    This is an offline preview-quality gate, not a robot safety limit. Raw
    displacement is checked because the Web preview interpolates the explicit
    returned joint coordinates. The shortest-angular check is retained in the
    diagnostic so a coordinate-wrap alias can be distinguished from a genuine
    multi-joint IK branch switch.
    """
    structural = default_graph_edge_feasibility(from_node, to_node)
    if not structural.feasible:
        return structural
    transition = analyze_joint_transition(
        from_sample_index=from_node.sample_index,
        to_sample_index=to_node.sample_index,
        previous_joint_positions_rad=from_node.combined_joint_positions_rad,
        current_joint_positions_rad=to_node.combined_joint_positions_rad,
    )
    raw_maximum = transition.max_abs_joint_step_rad
    shortest_maximum = transition.max_shortest_abs_joint_step_rad
    if (
        raw_maximum > PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD
        or shortest_maximum
        > PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD
    ):
        return TrajectoryEdgeFeasibility(
            False,
            (
                f"{PHASE3_CONTINUITY_GATE_REASON}: raw maximum "
                f"{raw_maximum:.9f} rad, shortest-angular maximum "
                f"{shortest_maximum:.9f} rad; limits are "
                f"{PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD:.9f} raw and "
                f"{PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD:.9f} "
                "shortest-angular rad per sampled transition; "
                "OFFLINE PREVIEW QUALITY GATE — NOT A ROBOT SAFETY LIMIT"
            ),
        )
    return structural


def _phase3_planning_failure_payload(global_result: Any, graph: Any) -> dict[str, Any] | None:
    failure = _failure_payload(global_result.failure or graph.failure)
    if failure is None or global_result.completed:
        return failure
    failed_sample_index = global_result.failed_sample_index
    rejected_edges = tuple(
        edge
        for edge in global_result.evaluated_edges
        if (
            not edge.feasible
            and edge.to_sample_index == failed_sample_index
            and PHASE3_CONTINUITY_GATE_REASON in edge.feasibility_diagnostic
        )
    )
    if not rejected_edges:
        return failure
    rejected_transitions = tuple(
        analyze_joint_transition(
            from_sample_index=edge.from_sample_index,
            to_sample_index=edge.to_sample_index,
            previous_joint_positions_rad=graph.layers[
                edge.from_sample_index
            ].nodes[edge.from_node_index].combined_joint_positions_rad,
            current_joint_positions_rad=graph.layers[
                edge.to_sample_index
            ].nodes[edge.to_node_index].combined_joint_positions_rad,
        )
        for edge in rejected_edges
    )
    failure["search_reason"] = failure["reason"]
    failure["reason"] = PHASE3_CONTINUITY_GATE_REASON
    failure["message"] = (
        "Phase-3 rejected an abrupt IK branch transition before preview. "
        "Increase Samples / Segment, adjust the waypoint path away from the "
        "singularity, or increase IK Attempts / Arm; no discontinuous Planned "
        "Ghost trajectory was accepted."
    )
    failure["continuity_gate"] = {
        "rejected_edge_count": len(rejected_edges),
        "maximum_raw_joint_step_rad": max(
            edge.max_raw_joint_step_rad for edge in rejected_edges
        ),
        "maximum_shortest_angular_joint_step_rad": max(
            transition.max_shortest_abs_joint_step_rad
            for transition in rejected_transitions
        ),
        "raw_step_limit_rad": PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD,
        "shortest_angular_step_limit_rad": (
            PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD
        ),
        "semantic": "OFFLINE PREVIEW QUALITY GATE — NOT A ROBOT SAFETY LIMIT",
    }
    return failure


def _candidate_provenance_payload(provenance: Any) -> dict[str, Any]:
    return {
        "parent_node_id": provenance.parent_node_id,
        "left_candidate_index": provenance.left_candidate_index,
        "left_source_seed_index": provenance.left_source_seed_index,
        "left_source_seed_rad": (
            list(provenance.left_source_seed_rad)
            if provenance.left_source_seed_rad is not None else None
        ),
        "left_diagnostic": provenance.left_diagnostic,
        "right_candidate_index": provenance.right_candidate_index,
        "right_source_seed_index": provenance.right_source_seed_index,
        "right_source_seed_rad": (
            list(provenance.right_source_seed_rad)
            if provenance.right_source_seed_rad is not None else None
        ),
        "right_diagnostic": provenance.right_diagnostic,
    }


def _graph_node_payload(node: Any) -> dict[str, Any]:
    return {
        "node_id": node.node_id,
        "node_index": node.node_index,
        "left_joint_positions_rad": list(node.left_joint_positions_rad),
        "right_joint_positions_rad": list(node.right_joint_positions_rad),
        "combined_joint_positions_rad": list(node.combined_joint_positions_rad),
        "candidate_provenance": [
            _candidate_provenance_payload(item) for item in node.provenance
        ],
        "state_validity_provenance": list(node.state_validity_provenance),
    }


def _candidate_attempt_payload(attempt: Any) -> dict[str, Any]:
    if not attempt.solver_success:
        disposition = "IK_FAILURE"
    elif attempt.duplicate_of_candidate_index is not None:
        disposition = "RAW_COORDINATE_DUPLICATE"
    else:
        disposition = "UNIQUE_SUCCESS"
    return {
        "parent_node_id": attempt.parent_node_id,
        "group_name": attempt.group_name,
        "candidate_index": attempt.candidate_index,
        "source_seed_index": attempt.source_seed_index,
        "source_seed_rad": list(attempt.source_seed_rad),
        "solver_success": attempt.solver_success,
        "joint_positions_rad": (
            list(attempt.joint_positions_rad)
            if attempt.joint_positions_rad is not None else None
        ),
        "duplicate_of_candidate_index": attempt.duplicate_of_candidate_index,
        "disposition": disposition,
        "diagnostic": attempt.diagnostic or "UNAVAILABLE",
    }


def _rejected_pair_payload(rejection: Any) -> dict[str, Any]:
    return {
        "parent_node_id": rejection.parent_node_id,
        "rejection_type": rejection.rejection_type,
        "left_candidate_index": rejection.left_candidate_index,
        "left_source_seed_index": rejection.left_source_seed_index,
        "right_candidate_index": rejection.right_candidate_index,
        "right_source_seed_index": rejection.right_source_seed_index,
        "combined_joint_positions_rad": list(
            rejection.combined_joint_positions_rad
        ),
        "diagnostic": rejection.diagnostic or "UNAVAILABLE",
    }


def _selected_path_payload(path: Sequence[Any]) -> list[dict[str, Any]]:
    return [
        {
            "sample_index": point.sample_index,
            "time_from_start_s": point.time_from_start_s,
            "node_id": point.graph_node_id,
            "node_index": point.graph_node_index,
            "left_joint_positions_rad": list(point.left_joint_positions_rad),
            "right_joint_positions_rad": list(point.right_joint_positions_rad),
            "combined_joint_positions_rad": list(
                point.combined_joint_positions_rad
            ),
            "local_edge_cost_rad2": point.edge_cost_from_predecessor_rad2,
            "cumulative_cost_rad2": point.cumulative_cost_rad2,
            "candidate_provenance": [
                _candidate_provenance_payload(item)
                for item in point.candidate_provenance
            ],
        }
        for point in path
    ]


def _path_transition_payloads(path: Sequence[Any]) -> list[dict[str, Any]]:
    transitions = tuple(
        analyze_joint_transition(
            from_sample_index=previous.sample_index,
            to_sample_index=current.sample_index,
            previous_joint_positions_rad=previous.combined_joint_positions_rad,
            current_joint_positions_rad=current.combined_joint_positions_rad,
        )
        for previous, current in zip(path, path[1:])
    )
    jump_analysis = analyze_suspicious_joint_jumps(
        transitions,
        DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    )
    payloads = []
    for index, (previous, current, transition) in enumerate(
        zip(path, path[1:], transitions)
    ):
        raw_deltas = tuple(
            record.delta_rad for record in transition.joint_deltas
        )
        shortest_deltas = tuple(
            record.shortest_delta_rad for record in transition.joint_deltas
        )
        jump = jump_analysis.transitions[index]
        reasons = []
        if transition.max_abs_joint_step_rad > PHASE3_DEBUG_SUSPICIOUS_RAW_STEP_RAD:
            reasons.append("RAW_ABSOLUTE_STEP_GT_1_RAD")
        if jump.has_suspicious_jump:
            reasons.append("EXISTING_SHORTEST_ANGULAR_JUMP_HEURISTIC")
        payloads.append({
            "from_layer": previous.sample_index,
            "to_layer": current.sample_index,
            "from_node_index": previous.graph_node_index,
            "to_node_index": current.graph_node_index,
            "from_joint_positions_rad": list(
                previous.combined_joint_positions_rad
            ),
            "to_joint_positions_rad": list(current.combined_joint_positions_rad),
            "raw_joint_deltas_rad": list(raw_deltas),
            "shortest_angular_joint_deltas_rad": list(shortest_deltas),
            "raw_squared_l2_edge_cost_rad2": sum(
                value * value for value in raw_deltas
            ),
            "selected_local_edge_cost_rad2": (
                current.edge_cost_from_predecessor_rad2
            ),
            "maximum_absolute_raw_delta_rad": (
                transition.max_abs_joint_step_rad
            ),
            "maximum_absolute_shortest_angular_delta_rad": (
                transition.max_shortest_abs_joint_step_rad
            ),
            "wraparound_adjusted_record_count": (
                transition.wraparound_adjusted_record_count
            ),
            "suspicious": bool(reasons),
            "suspicious_reasons": reasons,
            "existing_jump_heuristic_joint_names": list(
                jump.suspicious_joint_names
            ),
        })
    return payloads


def _edge_payload(edge: Any) -> dict[str, Any]:
    return {
        "from_layer": edge.from_sample_index,
        "from_node_index": edge.from_node_index,
        "to_layer": edge.to_sample_index,
        "to_node_index": edge.to_node_index,
        "feasible": edge.feasible,
        "feasibility_diagnostic": edge.feasibility_diagnostic,
        "raw_squared_l2_edge_cost_rad2": edge.raw_displacement_cost_rad2,
        "maximum_absolute_raw_delta_rad": edge.max_raw_joint_step_rad,
        "maximum_raw_joint_index": edge.max_raw_joint_step_index,
        "maximum_raw_joint_name": edge.max_raw_joint_step_name,
    }


def _suspicious_alternative_payloads(
    graph: Any,
    global_result: Any,
    global_transitions: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    dp_by_node = {
        (item.sample_index, item.node_index): item
        for item in global_result.node_search_diagnostics
    }
    summaries = []
    for transition in global_transitions:
        if not transition["suspicious"]:
            continue
        from_layer = graph.layers[transition["from_layer"]]
        to_layer = graph.layers[transition["to_layer"]]
        from_node = from_layer.nodes[transition["from_node_index"]]
        candidates = []
        for node in to_layer.nodes:
            edge = graph_edge_between(from_node, node)
            continuity = analyze_joint_transition(
                from_sample_index=from_layer.sample_index,
                to_sample_index=to_layer.sample_index,
                previous_joint_positions_rad=from_node.combined_joint_positions_rad,
                current_joint_positions_rad=node.combined_joint_positions_rad,
            )
            dp = dp_by_node.get((to_layer.sample_index, node.node_index))
            candidates.append({
                "node_index": node.node_index,
                "selected": node.node_index == transition["to_node_index"],
                "combined_joint_positions_rad": list(
                    node.combined_joint_positions_rad
                ),
                "edge_from_selected_previous_node": _edge_payload(edge),
                "shortest_angular_joint_deltas_rad": [
                    record.shortest_delta_rad
                    for record in continuity.joint_deltas
                ],
                "global_dp": ({
                    "reachable": dp.reachable,
                    "selected_predecessor_node_index": (
                        dp.selected_predecessor_node_index
                    ),
                    "local_edge_cost_rad2": dp.local_edge_cost_rad2,
                    "cumulative_cost_rad2": dp.cumulative_cost_rad2,
                } if dp is not None else None),
            })
        summaries.append({
            "selected_transition": dict(transition),
            "destination_layer_candidates": candidates,
        })
    return summaries


def _build_phase3_ik_debug_capture(
    *,
    request: Mapping[str, Any],
    authority: Mapping[str, Any],
    trajectory: Any,
    graph: Any,
    comparison: Any,
    candidate_exploration_config: IkCandidateExplorationConfig,
    joint_limits: CanonicalJointPositionLimits,
) -> dict[str, Any]:
    global_result = comparison.global_search
    greedy_result = comparison.greedy
    global_transitions = _path_transition_payloads(global_result.selected_path)
    greedy_transitions = _path_transition_payloads(greedy_result.selected_path)
    dp_by_node = {
        (item.sample_index, item.node_index): item
        for item in global_result.node_search_diagnostics
    }
    layers = []
    for layer, sample in zip(graph.layers, trajectory.samples):
        layers.append({
            "layer_index": layer.sample_index,
            "sample_index": sample.sample_index,
            "time_from_start_s": sample.time_from_start_s,
            "center_target": _transform_payload(sample.world_T_object),
            "left_target": _transform_payload(sample.world_T_left),
            "right_target": _transform_payload(sample.world_T_right),
            "source_parent_node_ids": list(layer.source_parent_node_ids),
            "candidate_counts": {
                "left_unique": layer.left_unique_candidate_count,
                "right_unique": layer.right_unique_candidate_count,
                "pairs": layer.candidate_pair_count,
                "valid_pairs": layer.valid_candidate_pair_count,
            },
            "candidate_attempts": [
                _candidate_attempt_payload(item)
                for item in layer.candidate_attempt_diagnostics
            ],
            "rejected_candidate_pairs": [
                _rejected_pair_payload(item)
                for item in layer.rejected_candidate_pairs
            ],
            "valid_nodes": [
                {
                    **_graph_node_payload(node),
                    "global_dp": ({
                        "reachable": dp_by_node[(layer.sample_index, node.node_index)].reachable,
                        "selected_predecessor_node_index": dp_by_node[(layer.sample_index, node.node_index)].selected_predecessor_node_index,
                        "local_edge_cost_rad2": dp_by_node[(layer.sample_index, node.node_index)].local_edge_cost_rad2,
                        "cumulative_cost_rad2": dp_by_node[(layer.sample_index, node.node_index)].cumulative_cost_rad2,
                    } if (layer.sample_index, node.node_index) in dp_by_node else None),
                }
                for node in layer.nodes
            ],
        })
    return {
        "schema": PHASE3_IK_DEBUG_CAPTURE_SCHEMA,
        "capture_mode": "OFFLINE_DIAGNOSTIC_ONLY",
        "input_authority": {
            "trajectory_name": request["name"],
            "waypoints": [
                {
                    "identifier": waypoint.identifier,
                    "translation_m": list(waypoint.translation_m),
                    "rpy_rad": list(waypoint.rpy_rad or request["fixed_orientation_rpy_rad"]),
                }
                for waypoint in request["waypoints"]
            ],
            "legacy_fallback_center_orientation_rpy_rad": list(
                request["fixed_orientation_rpy_rad"]
            ),
            "segment_duration_s": request["segment_duration_s"],
            "segment_durations_s": list(request["segment_durations_s"]),
            "waypoint_profiles": [dict(item) for item in request["waypoint_profiles"]],
            "timing_semantic": RELATIVE_WAYPOINT_PROFILE_TIMING_SEMANTIC,
            "samples_per_segment": request["samples_per_segment"],
            "planning_start_state_rad": {
                "left": list(request["initial_joint_state_rad"][:6]),
                "right": list(request["initial_joint_state_rad"][6:]),
            },
            "planning_start_state_source": request[
                "planning_start_state_source"
            ],
            "candidate_attempts_per_arm": request[
                "candidate_attempts_per_arm"
            ],
            "locked_grasp": {
                "center_T_left": authority["left"],
                "center_T_right": authority["right"],
                "content_revision": authority["grasp_content_revision"],
                "lock_generation": authority["lock_generation"],
                "lock_revision": authority["lock_revision"],
            },
            "calibration_revision": authority["calibration_revision"],
            "model_calibration_revision": authority[
                "model_calibration_revision"
            ],
        },
        "planner_settings": {
            "edge_cost": "sum((q_to-q_from)^2) over raw 12-joint coordinates",
            "layer_zero_cost": "0 for every valid layer-zero node",
            "preview_continuity_gate": {
                "maximum_raw_joint_step_rad": (
                    PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD
                ),
                "maximum_shortest_angular_joint_step_rad": (
                    PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD
                ),
                "semantic": (
                    "OFFLINE PREVIEW QUALITY GATE — NOT A ROBOT SAFETY LIMIT"
                ),
            },
            "candidate_perturbation_offset_rad": (
                candidate_exploration_config.perturbation_offset_rad
            ),
            "candidate_attempts_per_arm": (
                candidate_exploration_config.max_attempts_per_arm
            ),
            "raw_duplicate_tolerance_rad": (
                candidate_exploration_config.duplicate_tolerance_rad
            ),
            "ranking_tie_tolerance": (
                candidate_exploration_config.ranking_tie_tolerance
            ),
            "joint_position_limits_rad": {
                "lower": list(joint_limits.lower_rad),
                "upper": list(joint_limits.upper_rad),
            },
            "suspicious_capture_criteria": {
                "maximum_raw_step_strictly_greater_than_rad": (
                    PHASE3_DEBUG_SUSPICIOUS_RAW_STEP_RAD
                ),
                "also_uses_existing_shortest_angular_jump_heuristic": True,
            },
        },
        "graph": {
            "completed": graph.completed,
            "failure": _failure_payload(graph.failure),
            "layers": layers,
            "evaluated_edges": [
                _edge_payload(edge) for edge in global_result.evaluated_edges
            ],
        },
        "global_search": {
            "completed": global_result.completed,
            "total_cost_rad2": (
                global_result.cumulative_raw_displacement_cost_rad2
            ),
            "selected_node_indices": [
                point.graph_node_index for point in global_result.selected_path
            ],
            "selected_path": _selected_path_payload(
                global_result.selected_path
            ),
            "transitions": global_transitions,
            "node_dp": [
                {
                    "layer_index": item.sample_index,
                    "node_index": item.node_index,
                    "reachable": item.reachable,
                    "selected_predecessor_node_index": (
                        item.selected_predecessor_node_index
                    ),
                    "local_edge_cost_rad2": item.local_edge_cost_rad2,
                    "cumulative_cost_rad2": item.cumulative_cost_rad2,
                }
                for item in global_result.node_search_diagnostics
            ],
        },
        "greedy_search": {
            "completed": greedy_result.completed,
            "total_cost_rad2": (
                greedy_result.cumulative_raw_displacement_cost_rad2
            ),
            "selected_node_indices": [
                point.graph_node_index for point in greedy_result.selected_path
            ],
            "selected_path": _selected_path_payload(greedy_result.selected_path),
            "transitions": greedy_transitions,
        },
        "suspicious_global_transitions": _suspicious_alternative_payloads(
            graph,
            global_result,
            global_transitions,
        ),
    }


def _atomic_write_phase3_debug_capture(
    target_path: str,
    payload: Mapping[str, Any],
) -> None:
    target = Path(target_path).expanduser()
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.",
        suffix=".tmp",
        dir=str(target.parent),
    )
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, target)
    except Exception:
        try:
            os.unlink(temporary_name)
        except OSError:
            pass
        raise


def _maybe_capture_phase3_ik_debug(**capture_inputs: Any) -> None:
    target_path = os.environ.get(PHASE3_IK_DEBUG_CAPTURE_ENV)
    if not target_path:
        return
    try:
        capture = _build_phase3_ik_debug_capture(**capture_inputs)
        _atomic_write_phase3_debug_capture(target_path, capture)
    except Exception as error:
        LOGGER.warning(
            "Phase-3 IK diagnostic capture failed for %s: %s",
            target_path,
            error,
        )


def planning_unavailable_result(
    error: str,
    candidate_attempts_per_arm: int = (
        WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.max_attempts_per_arm
    ),
    planning_start_state_rad: Sequence[float] = PLANNING_START_STATE_COMBINED_RAD,
    planning_start_state_source: str = PLANNING_START_STATE_SOURCE,
) -> dict[str, Any]:
    """Return stable failure metadata while leaving robot-control paths usable."""
    start = tuple(planning_start_state_rad)
    return {
        "ok": False,
        "planner_status": "UNAVAILABLE",
        "planner_source": PLANNER_SOURCE,
        "plan_only": True,
        "optimality_scope": WEB_OPTIMALITY_SCOPE,
        "core_optimality_scope": GLOBAL_GRAPH_OPTIMALITY_SCOPE,
        "candidate_attempts_per_arm": candidate_attempts_per_arm,
        "candidate_exploration_profile": (
            WEB_GLOBAL_CANDIDATE_EXPLORATION_PROFILE
        ),
        "candidate_pruning_applied": False,
        "planning_start_state_rad": {
            "left": list(start[:6]),
            "right": list(start[6:]),
        },
        "planning_start_state_source": planning_start_state_source,
        "first_selected_joint_state_rad": None,
        "start_to_first_raw_joint_delta_rad": None,
        "start_to_first_maximum_raw_joint_delta": None,
        "failure": {
            "reason": "MOVEIT_PLANNING_UNAVAILABLE",
            "failed_sample_index": None,
            "message": str(error),
        },
        "warnings": list(PLAN_ONLY_WARNINGS),
        "units": {
            "translation": "meter",
            "orientation": "radian",
            "joint": "radian",
            "time": "second",
            "edge_cost": "radian^2",
        },
    }


def planning_authority_rejected_result(
    reason_code: str,
    message: str,
    normalized_request: Mapping[str, Any],
    *,
    diagnostics: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a structured pre-IK rejection without fabricating a plan."""
    start = normalized_request["initial_joint_state_rad"]
    return {
        "ok": False,
        "planner_status": "REJECTED",
        "planner_source": PLANNER_SOURCE,
        "planner_grasp_source": "AUTHORITATIVE BACKEND LOCKED SNAPSHOT REQUIRED",
        "plan_only": True,
        "failure": {
            "reason": str(reason_code),
            "failed_sample_index": None,
            "message": str(message),
            "stage": "AUTHORITATIVE_INPUT_GATE_BEFORE_IK",
            "diagnostics": dict(diagnostics or {}),
        },
        "candidate_attempts_per_arm": normalized_request[
            "candidate_attempts_per_arm"
        ],
        "planning_start_state_rad": {
            "left": list(start[:6]),
            "right": list(start[6:]),
        },
        "planning_start_state_source": normalized_request[
            "planning_start_state_source"
        ],
        "warnings": list(PLAN_ONLY_WARNINGS),
    }


def _matrix_rpy(transform: RigidTransform) -> tuple[float, float, float]:
    m = transform.matrix
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:
        roll = math.atan2(-m[0][1], m[1][1])
        yaw = 0.0
    return (roll, pitch, yaw)


def _angle_lerp(a: float, b: float, alpha: float) -> float:
    delta = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + alpha * delta


def _approach_pose_transform(pose: Mapping[str, Any]) -> RigidTransform:
    return RigidTransform.from_translation_rpy(pose["translation_m"], pose["rpy_rad"])


def _build_independent_approach_trajectory(
    request: Mapping[str, Any],
    grasp_model: ObjectGraspModel,
) -> ObjectTrajectory | None:
    approach = request["approach_waypoints"]
    if not approach:
        return None
    first_waypoint = request["waypoints"][0]
    first_center = RigidTransform.from_translation_rpy(
        first_waypoint.translation_m,
        first_waypoint.rpy_rad or request["fixed_orientation_rpy_rad"],
    )
    final_targets = grasp_model.compute_world_grasp_targets(first_center)
    sequence = list(approach) + [{
        "identifier": "FINAL_GRASP",
        "left": {"translation_m": final_targets.world_T_left.translation_m, "rpy_rad": _matrix_rpy(final_targets.world_T_left)},
        "right": {"translation_m": final_targets.world_T_right.translation_m, "rpy_rad": _matrix_rpy(final_targets.world_T_right)},
    }]
    count = request["samples_per_segment"]
    duration = request["segment_duration_s"]
    samples: list[ObjectTrajectorySample] = []
    elapsed = 0.0
    for segment_index, (start, end) in enumerate(zip(sequence, sequence[1:])):
        for local_index in range(0 if segment_index == 0 else 1, count):
            alpha = local_index / (count - 1)
            def interp(side: str) -> RigidTransform:
                a, b = start[side], end[side]
                translation = tuple((1-alpha)*a["translation_m"][i] + alpha*b["translation_m"][i] for i in range(3))
                rpy = tuple(_angle_lerp(a["rpy_rad"][i], b["rpy_rad"][i], alpha) for i in range(3))
                return RigidTransform.from_translation_rpy(translation, rpy)
            left = interp("left")
            right = interp("right")
            midpoint = tuple((left.translation_m[i] + right.translation_m[i]) * 0.5 for i in range(3))
            object_pose = RigidTransform.from_translation_rpy(midpoint, (0.0, 0.0, 0.0))
            samples.append(ObjectTrajectorySample(
                sample_index=len(samples),
                time_from_start_s=elapsed + alpha * duration,
                alpha=(elapsed + alpha * duration) / (duration * (len(sequence)-1)),
                world_T_object=object_pose,
                world_T_left=left,
                world_T_right=right,
            ))
        elapsed += duration
    return ObjectTrajectory(name=f"{request['name']}_Independent_Approach", duration_s=elapsed, samples=tuple(samples))


def _approach_selected_path_payload(path: Sequence[Any]) -> list[dict[str, Any]]:
    return [{
        "sample_index": point.sample_index,
        "time_from_start_s": point.time_from_start_s,
        "left": list(point.left_joint_positions_rad),
        "right": list(point.right_joint_positions_rad),
        "combined": list(point.combined_joint_positions_rad),
        "graph_node_id": point.graph_node_id,
        "graph_node_index": point.graph_node_index,
        "edge_cost_rad2": point.edge_cost_from_predecessor_rad2,
        "cumulative_cost_rad2": point.cumulative_cost_rad2,
    } for point in path]


def plan_object_global(
    payload: Any,
    adapter: ObjectTrajectoryIkAdapter,
    joint_limits: CanonicalJointPositionLimits,
    *,
    grasp_model: ObjectGraspModel,
    planning_authority: Any,
) -> dict[str, Any]:
    """Plan with one explicit backend-authoritative locked rigid grasp."""
    request = normalize_object_global_plan_request(payload)
    planning_started = time.perf_counter()
    if not callable(getattr(adapter, "solve_arm_ik", None)) or not callable(
        getattr(adapter, "check_combined_state", None)
    ):
        raise TypeError("adapter must implement the planning-only IK boundary")
    if not isinstance(joint_limits, CanonicalJointPositionLimits):
        raise TypeError("joint_limits must be canonical model limits")
    if not isinstance(grasp_model, ObjectGraspModel):
        raise TypeError("grasp_model must be an explicit ObjectGraspModel")
    authority = _normalize_planning_authority(planning_authority, grasp_model)
    planning_start = request["initial_joint_state_rad"]
    if not joint_limits.contains(planning_start):
        offending = next(
            index
            for index, (value, lower, upper) in enumerate(zip(
                planning_start,
                joint_limits.lower_rad,
                joint_limits.upper_rad,
            ))
            if value < lower or value > upper
        )
        raise ObjectGlobalPlanInputError(
            f"initial_joint_state_rad {DUAL_ARM_JOINT_ORDER[offending]} is outside "
            "the canonical position limits"
        )
    segment_count = len(request["waypoints"]) - 1
    segment_timing = []
    for segment_index in range(segment_count):
        profile = request["waypoint_profiles"][segment_index]
        duration_s = request["segment_durations_s"][segment_index]
        segment_timing.append({
            "segment_index": segment_index,
            "from_waypoint": request["waypoints"][segment_index].identifier,
            "to_waypoint": request["waypoints"][segment_index + 1].identifier,
            "profile_source_waypoint": request["waypoints"][segment_index].identifier,
            "speed_percent": profile["speed_percent"],
            "acceleration_percent": profile["acceleration_percent"],
            "base_duration_s": request["segment_duration_s"],
            "time_scale": duration_s / request["segment_duration_s"],
            "duration_s": duration_s,
            "semantic": "OUTGOING SEGMENT PROFILE (OPTION B)",
        })
    orientation_transform = RigidTransform.from_translation_rpy(
        (0.0, 0.0, 0.0),
        request["fixed_orientation_rpy_rad"],
    )
    trajectory = generate_multi_waypoint_object_trajectory(
        name=request["name"],
        waypoints=request["waypoints"],
        fixed_object_orientation=orientation_transform,
        segment_durations_s=request["segment_durations_s"],
        segment_sample_counts=(request["samples_per_segment"],) * segment_count,
        grasp_model=grasp_model,
    )
    candidate_exploration_config = _web_candidate_exploration_config(
        request["candidate_attempts_per_arm"]
    )
    debug_capture_enabled = bool(os.environ.get(PHASE3_IK_DEBUG_CAPTURE_ENV))
    approach_trajectory = _build_independent_approach_trajectory(request, grasp_model)
    approach_path: list[dict[str, Any]] = []
    rigid_planning_start = planning_start
    approach_summary = {"enabled": False, "status": "NOT_CONFIGURED", "waypoint_count": 0}
    if approach_trajectory is not None:
        approach_graph = build_object_trajectory_candidate_graph(
            approach_trajectory, adapter,
            initial_seed_joint_positions_rad=planning_start,
            candidate_exploration_config=candidate_exploration_config,
            joint_limits=joint_limits,
            retain_debug_diagnostics=False,
        )
        approach_comparison = compare_greedy_and_global(
            approach_graph,
            tie_tolerance=candidate_exploration_config.ranking_tie_tolerance,
            edge_feasibility=phase3_preview_continuity_edge_feasibility,
            retain_debug_diagnostics=False,
        )
        approach_result = approach_comparison.global_search
        if not approach_result.completed:
            return {
                **planning_unavailable_result(
                    f"Independent approach planning failed: {approach_result.failure}",
                    request["candidate_attempts_per_arm"], planning_start,
                    request["planning_start_state_source"],
                ),
                "planner_status": "FAILED",
                "failure": {
                    "reason": "INDEPENDENT_APPROACH_FAILED",
                    "failed_sample_index": approach_result.failed_sample_index,
                    "message": "Independent Left/Right approach path has no continuous IK solution",
                },
            }
        first_approach = approach_result.selected_path[0].combined_joint_positions_rad
        approach_start_transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=planning_start,
            current_joint_positions_rad=first_approach,
        )
        if (
            approach_start_transition.max_abs_joint_step_rad
            > PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD
            or approach_start_transition.max_shortest_abs_joint_step_rad
            > PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD
        ):
            return {
                **planning_unavailable_result(
                    "Independent approach start transition exceeds the Phase-3 continuity gate",
                    request["candidate_attempts_per_arm"], planning_start,
                    request["planning_start_state_source"],
                ),
                "planner_status": "FAILED",
                "failure": {
                    "reason": "INDEPENDENT_APPROACH_START_TRANSITION",
                    "failed_sample_index": 0,
                    "message": (
                        "Planning Start → first Independent Approach target changes "
                        "joint branch too abruptly; capture the first approach target "
                        "from the current displayed robot pose or add a nearer approach target"
                    ),
                    "maximum_raw_joint_step_rad": (
                        approach_start_transition.max_abs_joint_step_rad
                    ),
                    "maximum_shortest_angular_joint_step_rad": (
                        approach_start_transition.max_shortest_abs_joint_step_rad
                    ),
                },
            }
        approach_path = _approach_selected_path_payload(approach_result.selected_path)
        rigid_planning_start = approach_result.selected_path[-1].combined_joint_positions_rad
        approach_summary = {
            "enabled": True, "status": "READY",
            "waypoint_count": len(request["approach_waypoints"]),
            "sample_count": len(approach_path),
            "duration_s": approach_trajectory.duration_s,
            "semantic": "PRE-RIGID INDEPENDENT LEFT/RIGHT APPROACH — PLAN ONLY",
        }
    graph = build_object_trajectory_candidate_graph(
        trajectory,
        adapter,
        initial_seed_joint_positions_rad=rigid_planning_start,
        candidate_exploration_config=candidate_exploration_config,
        joint_limits=joint_limits,
        retain_debug_diagnostics=debug_capture_enabled,
    )
    comparison = compare_greedy_and_global(
        graph,
        tie_tolerance=(
            candidate_exploration_config.ranking_tie_tolerance
        ),
        edge_feasibility=phase3_preview_continuity_edge_feasibility,
        retain_debug_diagnostics=debug_capture_enabled,
    )
    global_result = comparison.global_search
    greedy_result = comparison.greedy
    object_samples = [
        {
            "sample_index": sample.sample_index,
            "time_from_start_s": sample.time_from_start_s,
            "object_pose": _transform_payload(
                sample.world_T_object,
                rpy_rad=_matrix_rpy(sample.world_T_object),
            ),
            "left_target": _transform_payload(sample.world_T_left),
            "right_target": _transform_payload(sample.world_T_right),
        }
        for sample in trajectory.samples
    ]
    global_path = [
        {
            "sample_index": point.sample_index,
            "time_from_start_s": point.time_from_start_s,
            "left": list(point.left_joint_positions_rad),
            "right": list(point.right_joint_positions_rad),
            "combined": list(point.combined_joint_positions_rad),
            "graph_node_id": point.graph_node_id,
            "graph_node_index": point.graph_node_index,
            "edge_cost_rad2": point.edge_cost_from_predecessor_rad2,
            "cumulative_cost_rad2": point.cumulative_cost_rad2,
        }
        for point in global_result.selected_path
    ]
    approach_duration_s = float(approach_summary.get("duration_s", 0.0) or 0.0)
    if approach_path:
        combined_path = [dict(point) for point in approach_path[:-1]]
        for point in global_path:
            shifted = dict(point)
            shifted["sample_index"] = len(combined_path)
            shifted["time_from_start_s"] = float(point["time_from_start_s"]) + approach_duration_s
            combined_path.append(shifted)
        approach_samples = []
        for sample in approach_trajectory.samples if approach_trajectory is not None else ():
            approach_samples.append({
                "sample_index": sample.sample_index,
                "time_from_start_s": sample.time_from_start_s,
                "left_target": _transform_payload(sample.world_T_left),
                "right_target": _transform_payload(sample.world_T_right),
            })
        first_object = object_samples[0]
        combined_object_samples = []
        for index, point in enumerate(approach_path[:-1]):
            target = approach_samples[index]
            combined_object_samples.append({
                "sample_index": index,
                "time_from_start_s": point["time_from_start_s"],
                "object_pose": first_object["object_pose"],
                "left_target": target["left_target"],
                "right_target": target["right_target"],
                "phase": "APPROACH",
            })
        for sample in object_samples:
            shifted = dict(sample)
            shifted["sample_index"] = len(combined_object_samples)
            shifted["time_from_start_s"] = float(sample["time_from_start_s"]) + approach_duration_s
            shifted["phase"] = "RIGID"
            combined_object_samples.append(shifted)
    else:
        approach_samples = []
        combined_path = [dict(point) for point in global_path]
        combined_object_samples = [{**sample, "phase": "RIGID"} for sample in object_samples]
    combined_duration_s = approach_duration_s + trajectory.duration_s
    rigid_first_selected = (
        global_result.selected_path[0].combined_joint_positions_rad
        if global_result.selected_path
        else None
    )
    if approach_path and rigid_first_selected is not None:
        boundary = analyze_joint_transition(
            from_sample_index=0, to_sample_index=1,
            previous_joint_positions_rad=rigid_planning_start,
            current_joint_positions_rad=rigid_first_selected,
        )
        if (
            boundary.max_abs_joint_step_rad > PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD
            or boundary.max_shortest_abs_joint_step_rad > PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD
        ):
            return {
                **planning_unavailable_result(
                    "Approach to rigid-grasp boundary exceeds the Phase-3 continuity gate",
                    request["candidate_attempts_per_arm"], planning_start,
                    request["planning_start_state_source"],
                ),
                "planner_status": "FAILED",
                "failure": {
                    "reason": "APPROACH_TO_RIGID_CONTINUITY_GATE",
                    "failed_sample_index": 0,
                    "message": "Final Approach state to first rigid Center sample is discontinuous",
                    "maximum_raw_joint_step_rad": boundary.max_abs_joint_step_rad,
                    "maximum_shortest_angular_joint_step_rad": boundary.max_shortest_abs_joint_step_rad,
                },
            }
    first_selected = (
        approach_result.selected_path[0].combined_joint_positions_rad
        if approach_trajectory is not None and approach_path
        else rigid_first_selected
    )
    start_to_first_delta = (
        joint_delta_rad(first_selected, planning_start)
        if first_selected is not None
        else None
    )
    start_to_first_maximum = None
    if start_to_first_delta is not None:
        maximum_index = max(
            range(12), key=lambda index: abs(start_to_first_delta[index])
        )
        start_to_first_maximum = {
            "value_rad": abs(start_to_first_delta[maximum_index]),
            "joint_index": maximum_index,
            "joint_name": DUAL_ARM_JOINT_ORDER[maximum_index],
        }
    result = {
        "ok": global_result.completed,
        "planner_status": "READY" if global_result.completed else "FAILED",
        "planner_source": PLANNER_SOURCE,
        "plan_only": True,
        "trajectory_name": trajectory.name,
        "object_waypoint_count": len(trajectory.waypoints),
        "object_sample_count": trajectory.sample_count,
        "duration_s": trajectory.duration_s,
        "segment_duration_s": request["segment_duration_s"],
        "segment_durations_s": list(request["segment_durations_s"]),
        "segment_timing": segment_timing,
        "timing_semantic": RELATIVE_WAYPOINT_PROFILE_TIMING_SEMANTIC,
        "fixed_orientation_rpy_rad": list(request["fixed_orientation_rpy_rad"]),
        "orientation_source": "PER-WAYPOINT 6D CENTER POSE — SHORTEST-ARC QUATERNION SLERP",
        "grasp": {
            "status": authority["grasp_status"],
            "source": authority["grasp_source"],
            "content_revision": authority["grasp_content_revision"],
            "lock_generation": authority["lock_generation"],
            "lock_revision": authority["lock_revision"],
            "left": authority["left"],
            "right": authority["right"],
            "expected_left_T_right": authority["expected_left_T_right"],
            "semantic": (
                "LOCKED RIGID GRASP APPLIES TO RIGID CENTER PATH; "
                "APPROACH IS PRE-RIGID"
                if approach_path else
                "ONE USER-LOCKED RIGID GRASP FOR EVERY TRAJECTORY SAMPLE"
            ),
        },
        "calibration": {
            "revision": authority["calibration_revision"],
            "model_revision": authority["model_calibration_revision"],
            "revision_status": authority["calibration_revision_status"],
            "calibration_state": authority["calibration_state"],
            "physical_calibration": authority["physical_calibration"],
            "physically_calibrated": authority["physically_calibrated"],
            "tcp_tool_contract_status": authority["tcp_tool_contract_status"],
            "semantic": "SOFTWARE MODEL REVISION MATCH — NOT PHYSICAL CALIBRATION",
        },
        "candidate_attempts_per_arm": (
            candidate_exploration_config.max_attempts_per_arm
        ),
        "candidate_exploration_profile": (
            WEB_GLOBAL_CANDIDATE_EXPLORATION_PROFILE
        ),
        "candidate_pruning_applied": graph.candidate_pruning_applied,
        "pre_approach_start_state_rad": {
            "left": list(planning_start[:6]),
            "right": list(planning_start[6:]),
        },
        "planning_start_state_rad": {
            "left": list(planning_start[:6]),
            "right": list(planning_start[6:]),
        },
        "rigid_planning_start_state_rad": {
            "left": list(rigid_planning_start[:6]),
            "right": list(rigid_planning_start[6:]),
        },
        "approach": approach_summary,
        "approach_path": approach_path,
        "planning_start_state_source": request["planning_start_state_source"],
        "first_selected_joint_state_rad": (
            {
                "left": list(first_selected[:6]),
                "right": list(first_selected[6:]),
            }
            if first_selected is not None
            else None
        ),
        "start_to_first_raw_joint_delta_rad": (
            list(start_to_first_delta) if start_to_first_delta is not None else None
        ),
        "start_to_first_maximum_raw_joint_delta": start_to_first_maximum,
        "planning_elapsed_s": time.perf_counter() - planning_started,
        "waypoints": [
            {
                "identifier": waypoint.identifier,
                "translation_m": list(waypoint.translation_m),
                "rpy_rad": list(waypoint.rpy_rad or request["fixed_orientation_rpy_rad"]),
                "speed_percent": request["waypoint_profiles"][index]["speed_percent"],
                "acceleration_percent": request["waypoint_profiles"][index]["acceleration_percent"],
                "has_outgoing_segment": index < len(trajectory.waypoints) - 1,
            }
            for index, waypoint in enumerate(trajectory.waypoints)
        ],
        "object_samples": object_samples,
        "common_timestamps_s": [
            sample["time_from_start_s"] for sample in object_samples
        ],
        "approach_samples": approach_samples,
        "combined_path": combined_path,
        "combined_object_samples": combined_object_samples,
        "combined_timestamps_s": [point["time_from_start_s"] for point in combined_path],
        "combined_duration_s": combined_duration_s,
        "global_path": global_path,
        "global": {
            "completed": global_result.completed,
            "total_cost_rad2": global_result.cumulative_raw_displacement_cost_rad2,
            "maximum_raw_single_joint_transition": _maximum_path_transition(
                global_result.selected_path
            ),
            "failed_sample_index": global_result.failed_sample_index,
        },
        "greedy": {
            "completed": greedy_result.completed,
            "total_cost_rad2": greedy_result.cumulative_raw_displacement_cost_rad2,
            "maximum_raw_single_joint_transition_rad": (
                greedy_result.maximum_raw_single_joint_transition_rad
            ),
            "failed_sample_index": greedy_result.failed_sample_index,
        },
        "comparison": {
            "global_not_worse": comparison.global_not_worse_on_equivalent_graph,
            "cost_difference_rad2": (
                greedy_result.cumulative_raw_displacement_cost_rad2
                - global_result.cumulative_raw_displacement_cost_rad2
                if greedy_result.completed and global_result.completed
                else None
            ),
        },
        "graph": {
            "completed": graph.completed,
            "layer_count": len(graph.layers),
            "requested_layer_count": graph.requested_layer_count,
            "node_counts_per_layer": [len(layer.nodes) for layer in graph.layers],
            "left_candidate_counts_per_layer": [
                layer.left_unique_candidate_count for layer in graph.layers
            ],
            "right_candidate_counts_per_layer": [
                layer.right_unique_candidate_count for layer in graph.layers
            ],
            "candidate_pair_counts_per_layer": [
                layer.candidate_pair_count for layer in graph.layers
            ],
            "valid_pair_counts_per_layer": [
                layer.valid_candidate_pair_count for layer in graph.layers
            ],
            "selected_node_ids": [point.graph_node_id for point in global_result.selected_path],
            "candidate_pruning_applied": graph.candidate_pruning_applied,
        },
        "failure": _phase3_planning_failure_payload(global_result, graph),
        "optimality_scope": WEB_OPTIMALITY_SCOPE,
        "core_optimality_scope": GLOBAL_GRAPH_OPTIMALITY_SCOPE,
        "warnings": list(PLAN_ONLY_WARNINGS),
        "units": {
            "translation": "meter",
            "orientation": "radian",
            "joint": "radian",
            "time": "second",
            "edge_cost": "radian^2",
        },
    }
    _maybe_capture_phase3_ik_debug(
        request=request,
        authority=authority,
        trajectory=trajectory,
        graph=graph,
        comparison=comparison,
        candidate_exploration_config=candidate_exploration_config,
        joint_limits=joint_limits,
    )
    return result
