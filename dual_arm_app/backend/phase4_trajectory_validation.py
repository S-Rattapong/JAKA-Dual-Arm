"""Pure Phase-4A trajectory, kinematic-model, and dynamics validation.

OFFLINE VALIDATION ONLY.  This module has no ROS, JAKA driver, publisher,
controller, action, endpoint, or motion dependency.  It validates a successful
Phase-3 global plan and delegates model FK through a small injected adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence

import yaml

from dual_arm_app.backend.object_grasp_model import RigidTransform, compose, inverse
from dual_arm_app.backend.object_trajectory_ik import (
    DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    DUAL_ARM_JOINT_ORDER,
    CanonicalJointPositionLimits,
    TransitionContinuity,
    analyze_joint_transition,
    analyze_suspicious_joint_jumps,
    summarize_trajectory_continuity,
)


DEFAULT_POSITION_TOLERANCE_M = 0.001
DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.001
MODEL_CONSISTENCY_TOLERANCE_SEMANTIC = (
    "MODEL CONSISTENCY VALIDATION TOLERANCE — NOT MANUFACTURER SAFETY LIMIT"
)
COMMON_TIMELINE_SUCCESS = "COMMON_TIMELINE_STRUCTURALLY_VALID"
COMMON_TIMELINE_MESSAGE = "COMMON PLAN TIMELINE VERIFIED"
DYNAMIC_TIMELINE_WARNING = (
    "DYNAMIC EXECUTION PARAMETERIZATION NOT YET FULLY VALIDATED"
)
VELOCITY_LIMIT_SOURCE = "MOVEIT_JOINT_LIMITS_YAML"
ACCELERATION_ESTIMATE_SEMANTIC = (
    "DISCRETE ACCELERATION ESTIMATE FROM THE CURRENT COMMON TIMESTAMPS"
)
NUMERICAL_LIMIT_EPSILON = 1e-12
MOVEIT_JOINT_LIMITS_PATH = (
    Path(__file__).resolve().parents[2]
    / "src/jaka_ros2/src/jaka_a12_moveit_config/config/joint_limits.yaml"
)


class Phase4ValidationInputError(ValueError):
    """Raised when a Phase-4A request or successful Phase-3 plan is malformed."""


class Phase4FkAdapter(Protocol):
    def compute_combined_fk(
        self,
        joint_positions_rad: Sequence[float],
        timeout_s: float = 2.0,
    ) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class MoveItDynamicsLimits:
    velocity_limits_rad_s: tuple[float, ...]
    velocity_limit_source: str
    source_path: str
    default_velocity_scaling_factor: float | None
    acceleration_limits_rad_s2: tuple[float, ...] | None
    acceleration_limit_status: str
    acceleration_limit_validation: str
    acceleration_limit_source: str


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Phase4ValidationInputError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise Phase4ValidationInputError(f"{label} must be a finite number")
    return converted


def _nonnegative_tolerance(value: Any, label: str) -> float:
    checked = _finite_number(value, label)
    if checked < 0.0:
        raise Phase4ValidationInputError(f"{label} must be non-negative")
    return checked


def _joint_vector(value: Any, label: str, count: int = 12) -> tuple[float, ...]:
    if (
        isinstance(value, (str, bytes, bool))
        or not isinstance(value, Sequence)
        or len(value) != count
    ):
        raise Phase4ValidationInputError(
            f"{label} must contain exactly {count} joint values"
        )
    return tuple(_finite_number(item, f"{label}[{index}]") for index, item in enumerate(value))


def _transform(value: Any, label: str) -> RigidTransform:
    matrix = value.get("matrix") if isinstance(value, Mapping) else None
    if matrix is None:
        raise Phase4ValidationInputError(f"{label}.matrix is required")
    try:
        return RigidTransform.from_matrix(matrix)
    except (TypeError, ValueError) as error:
        raise Phase4ValidationInputError(f"{label}.matrix: {error}") from error


def load_moveit_joint_dynamics_limits(
    path: Path = MOVEIT_JOINT_LIMITS_PATH,
) -> MoveItDynamicsLimits:
    """Load the active MoveIt override and map six limits to canonical 12."""
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Unable to load MoveIt joint limits: {error}") from error
    if not isinstance(payload, Mapping):
        raise RuntimeError("MoveIt joint-limit YAML root must be a mapping")
    joint_limits = payload.get("joint_limits")
    if not isinstance(joint_limits, Mapping):
        raise RuntimeError("MoveIt joint-limit YAML has no joint_limits mapping")

    source_velocities: list[float] = []
    configured_accelerations: list[float] = []
    all_accelerations_configured = True
    for index in range(1, 7):
        name = f"joint_{index}"
        metadata = joint_limits.get(name)
        if not isinstance(metadata, Mapping):
            raise RuntimeError(f"MoveIt joint-limit YAML is missing {name}")
        if metadata.get("has_velocity_limits") is not True:
            raise RuntimeError(f"{name}.has_velocity_limits must be true")
        velocity = metadata.get("max_velocity")
        if isinstance(velocity, bool) or not isinstance(velocity, (int, float)):
            raise RuntimeError(f"{name}.max_velocity must be finite and positive")
        velocity = float(velocity)
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise RuntimeError(f"{name}.max_velocity must be finite and positive")
        source_velocities.append(velocity)

        has_acceleration = metadata.get("has_acceleration_limits") is True
        acceleration = metadata.get("max_acceleration")
        if (
            not has_acceleration
            or isinstance(acceleration, bool)
            or not isinstance(acceleration, (int, float))
            or not math.isfinite(float(acceleration))
            or float(acceleration) <= 0.0
        ):
            all_accelerations_configured = False
        else:
            configured_accelerations.append(float(acceleration))

    scaling = payload.get("default_velocity_scaling_factor")
    if isinstance(scaling, bool) or not isinstance(scaling, (int, float)):
        scaling_value = None
    else:
        scaling_value = float(scaling)
        if not math.isfinite(scaling_value):
            scaling_value = None

    acceleration_limits = (
        tuple(configured_accelerations + configured_accelerations)
        if all_accelerations_configured
        else None
    )
    return MoveItDynamicsLimits(
        velocity_limits_rad_s=tuple(source_velocities + source_velocities),
        velocity_limit_source=VELOCITY_LIMIT_SOURCE,
        source_path=str(path),
        default_velocity_scaling_factor=scaling_value,
        acceleration_limits_rad_s2=acceleration_limits,
        acceleration_limit_status=("AVAILABLE" if acceleration_limits else "LIMIT_UNAVAILABLE"),
        acceleration_limit_validation=("PENDING" if acceleration_limits else "NOT_EVALUATED"),
        acceleration_limit_source=(
            VELOCITY_LIMIT_SOURCE if acceleration_limits else "NOT_CONFIGURED"
        ),
    )


def rotation_angle_error_rad(target: RigidTransform, actual: RigidTransform) -> float:
    """Return acos(clamp((trace(R_target^T R_actual)-1)/2,-1,+1))."""
    if not isinstance(target, RigidTransform) or not isinstance(actual, RigidTransform):
        raise TypeError("rotation_angle_error_rad requires RigidTransform values")
    target_matrix = target.matrix
    actual_matrix = actual.matrix
    trace = sum(
        target_matrix[row][column] * actual_matrix[row][column]
        for row in range(3)
        for column in range(3)
    )
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.acos(cosine)


def transform_error(target: RigidTransform, actual: RigidTransform) -> tuple[float, float]:
    translation_error = math.sqrt(sum(
        (actual.translation_m[index] - target.translation_m[index]) ** 2
        for index in range(3)
    ))
    return translation_error, rotation_angle_error_rad(target, actual)


def _locked_grasp_transforms(plan: Mapping[str, Any]) -> tuple[
    Mapping[str, Any], RigidTransform, RigidTransform, RigidTransform
]:
    grasp = plan.get("grasp")
    if not isinstance(grasp, Mapping):
        raise Phase4ValidationInputError("plan.grasp locked metadata is required")
    if grasp.get("status") != "GRASP_LOCKED":
        raise Phase4ValidationInputError("plan.grasp.status must be GRASP_LOCKED")
    for field in ("content_revision", "lock_revision"):
        if not isinstance(grasp.get(field), str) or not grasp[field].strip():
            raise Phase4ValidationInputError(f"plan.grasp.{field} must be a non-empty string")
    generation = grasp.get("lock_generation")
    if isinstance(generation, bool) or not isinstance(generation, int) or generation < 1:
        raise Phase4ValidationInputError("plan.grasp.lock_generation must be an integer >= 1")
    object_T_left = _transform(grasp.get("left"), "plan.grasp.left")
    object_T_right = _transform(grasp.get("right"), "plan.grasp.right")
    expected_relative = compose(inverse(object_T_left), object_T_right)
    embedded_relative = _transform(
        grasp.get("expected_left_T_right"), "plan.grasp.expected_left_T_right"
    )
    translation_error, orientation_error = transform_error(
        expected_relative, embedded_relative
    )
    if translation_error > 1e-9 or orientation_error > 1e-9:
        raise Phase4ValidationInputError(
            "plan.grasp.expected_left_T_right is inconsistent with locked Left/Right transforms"
        )
    return grasp, object_T_left, object_T_right, expected_relative


def validate_locked_grasp_targets(
    plan: Mapping[str, Any],
    object_samples: Sequence[Mapping[str, Any]],
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> tuple[dict[str, Any], RigidTransform]:
    """Prove every stored target was derived from the embedded locked grasp."""
    grasp, object_T_left, object_T_right, expected_relative = (
        _locked_grasp_transforms(plan)
    )
    samples: list[dict[str, Any]] = []
    for index, sample in enumerate(object_samples):
        if not isinstance(sample, Mapping):
            raise Phase4ValidationInputError(f"object_samples[{index}] must be an object")
        world_T_object = _transform(
            sample.get("object_pose"), f"object_samples[{index}].object_pose"
        )
        stored_left = _transform(
            sample.get("left_target"), f"object_samples[{index}].left_target"
        )
        stored_right = _transform(
            sample.get("right_target"), f"object_samples[{index}].right_target"
        )
        expected_left = compose(world_T_object, object_T_left)
        expected_right = compose(world_T_object, object_T_right)
        left_position, left_orientation = transform_error(expected_left, stored_left)
        right_position, right_orientation = transform_error(expected_right, stored_right)
        stored_relative = compose(inverse(stored_left), stored_right)
        relative_position, relative_orientation = transform_error(
            expected_relative, stored_relative
        )
        passed = (
            left_position <= position_tolerance_m
            and left_orientation <= orientation_tolerance_rad
            and right_position <= position_tolerance_m
            and right_orientation <= orientation_tolerance_rad
            and relative_position <= position_tolerance_m
            and relative_orientation <= orientation_tolerance_rad
        )
        samples.append({
            "sample_index": index,
            "status": "PASS" if passed else "FAIL",
            "left_target_position_error_m": left_position,
            "left_target_orientation_error_rad": left_orientation,
            "right_target_position_error_m": right_position,
            "right_target_orientation_error_rad": right_orientation,
            "stored_relative_translation_error_m": relative_position,
            "stored_relative_orientation_error_rad": relative_orientation,
        })
    metric_names = (
        "left_target_position_error_m",
        "left_target_orientation_error_rad",
        "right_target_position_error_m",
        "right_target_orientation_error_rad",
        "stored_relative_translation_error_m",
        "stored_relative_orientation_error_rad",
    )
    maxima = {}
    for metric in metric_names:
        maximum = max(samples, key=lambda sample: sample[metric])
        maxima[f"max_{metric}"] = maximum[metric]
        maxima[f"max_{metric}_sample_index"] = maximum["sample_index"]
    first_failure = next(
        (sample for sample in samples if sample["status"] == "FAIL"), None
    )
    return ({
        "status": "PASS" if first_failure is None else "FAIL",
        "semantic": "USER-LOCKED OPERATOR RIGID GRASP — CONSTANT FOR RIGID CENTER SAMPLES",
        "grasp_content_revision": grasp["content_revision"],
        "lock_generation": grasp["lock_generation"],
        "lock_revision": grasp["lock_revision"],
        "expected_relative_translation_m": list(expected_relative.translation_m),
        "expected_relative_rotation_matrix": [
            list(row[:3]) for row in expected_relative.matrix[:3]
        ],
        "position_tolerance_m": position_tolerance_m,
        "orientation_tolerance_rad": orientation_tolerance_rad,
        "samples": samples,
        "first_failure": first_failure,
        **maxima,
    }, expected_relative)


def validate_common_timeline(plan: Mapping[str, Any]) -> dict[str, Any]:
    rigid_objects = plan.get("object_samples")
    rigid_path = plan.get("global_path")
    motion_path = plan.get("combined_path") if isinstance(plan.get("combined_path"), list) else rigid_path
    motion_objects = plan.get("combined_object_samples") if isinstance(plan.get("combined_object_samples"), list) else rigid_objects
    failures: list[dict[str, Any]] = []
    if not isinstance(rigid_objects, list) or not isinstance(rigid_path, list):
        return {"status": "FAIL", "code": "MISSING_SAMPLE_ARRAYS", "failures": [{"reason": "object_samples and global_path must be arrays"}], "message": DYNAMIC_TIMELINE_WARNING}
    if not isinstance(motion_path, list) or not isinstance(motion_objects, list):
        return {"status": "FAIL", "code": "MISSING_COMBINED_SAMPLE_ARRAYS", "failures": [{"reason": "combined path arrays must be arrays"}], "message": DYNAMIC_TIMELINE_WARNING}
    if len(rigid_objects) != len(rigid_path):
        failures.append({"reason": "OBJECT_JOINT_SAMPLE_COUNT_MISMATCH", "object_sample_count": len(rigid_objects), "joint_sample_count": len(rigid_path)})
    if len(motion_objects) != len(motion_path):
        failures.append({"reason": "COMBINED_OBJECT_JOINT_SAMPLE_COUNT_MISMATCH", "object_sample_count": len(motion_objects), "joint_sample_count": len(motion_path)})

    def times(samples: Sequence[Mapping[str, Any]], label: str) -> list[float]:
        output: list[float] = []
        for index, sample in enumerate(samples):
            try:
                output.append(_finite_number(sample.get("time_from_start_s"), f"{label}[{index}].time_from_start_s"))
            except (AttributeError, Phase4ValidationInputError) as error:
                failures.append({"reason": "INVALID_TIMESTAMP", "location": f"{label}[{index}]", "detail": str(error)})
                output.append(float("nan"))
        return output

    rigid_object_times = times(rigid_objects, "object_samples")
    rigid_joint_times = times(rigid_path, "global_path")
    motion_object_times = times(motion_objects, "combined_object_samples")
    motion_joint_times = times(motion_path, "combined_path")
    for label, left, right in (("RIGID", rigid_object_times, rigid_joint_times), ("COMBINED", motion_object_times, motion_joint_times)):
        for index in range(min(len(left), len(right))):
            if left[index] != right[index]:
                failures.append({"reason": "OBJECT_JOINT_TIMESTAMP_MISMATCH", "timeline_scope": label, "sample_index": index, "object_time_s": left[index], "joint_time_s": right[index]})
    if motion_joint_times and motion_joint_times[0] != 0.0:
        failures.append({"reason": "FIRST_TIMESTAMP_NOT_ZERO", "time_s": motion_joint_times[0]})
    for index in range(len(motion_joint_times) - 1):
        dt = motion_joint_times[index + 1] - motion_joint_times[index]
        if not math.isfinite(dt) or dt <= 0.0:
            failures.append({"reason": "TIMESTAMPS_NOT_STRICTLY_INCREASING", "from_sample_index": index, "to_sample_index": index + 1, "dt_s": dt})
    try:
        rigid_duration = _finite_number(plan.get("duration_s"), "duration_s")
        motion_duration = _finite_number(plan.get("combined_duration_s", rigid_duration), "combined_duration_s")
    except Phase4ValidationInputError as error:
        rigid_duration = motion_duration = None
        failures.append({"reason": "INVALID_DECLARED_DURATION", "detail": str(error)})
    if rigid_joint_times and rigid_duration is not None and rigid_joint_times[-1] != rigid_duration:
        failures.append({"reason": "RIGID_FINAL_TIMESTAMP_DURATION_MISMATCH", "final_time_s": rigid_joint_times[-1], "declared_duration_s": rigid_duration})
    if motion_joint_times and motion_duration is not None and motion_joint_times[-1] != motion_duration:
        failures.append({"reason": "FINAL_TIMESTAMP_DURATION_MISMATCH", "final_time_s": motion_joint_times[-1], "declared_duration_s": motion_duration})
    return {
        "status": COMMON_TIMELINE_SUCCESS if not failures else "FAIL",
        "message": COMMON_TIMELINE_MESSAGE if not failures else "COMMON PLAN TIMELINE INVALID",
        "dynamic_parameterization": DYNAMIC_TIMELINE_WARNING,
        "object_sample_count": len(motion_objects),
        "joint_sample_count": len(motion_path),
        "rigid_object_sample_count": len(rigid_objects),
        "rigid_joint_sample_count": len(rigid_path),
        "common_timestamp_for_both_arms": not failures,
        "first_time_s": motion_joint_times[0] if motion_joint_times else None,
        "final_time_s": motion_joint_times[-1] if motion_joint_times else None,
        "declared_duration_s": motion_duration,
        "failures": failures,
    }

def _serialize_transition(transition: TransitionContinuity) -> dict[str, Any]:
    return {
        "from_sample_index": transition.from_sample_index,
        "to_sample_index": transition.to_sample_index,
        "raw_deltas_rad": [record.delta_rad for record in transition.joint_deltas],
        "shortest_angular_deltas_rad": [record.shortest_delta_rad for record in transition.joint_deltas],
        "wraparound_records": [
            {
                "joint_index": record.joint_index,
                "joint_name": record.joint_name,
                "raw_delta_rad": record.delta_rad,
                "shortest_delta_rad": record.shortest_delta_rad,
            }
            for record in transition.joint_deltas if record.wraparound_adjusted
        ],
        "maximum_raw_step_rad": transition.max_abs_joint_step_rad,
        "maximum_raw_joint": transition.max_joint_name,
        "maximum_shortest_angular_step_rad": transition.max_shortest_abs_joint_step_rad,
        "maximum_shortest_joint": transition.max_shortest_joint_name,
    }


def _jump_payload(transitions: Sequence[TransitionContinuity]) -> dict[str, Any]:
    analysis = analyze_suspicious_joint_jumps(
        transitions,
        DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    )
    return {
        "analysis_completed": analysis.analysis_completed,
        "suspicious_transition_count": analysis.suspicious_transition_count,
        "suspicious_joint_record_count": analysis.suspicious_joint_record_count,
        "transitions": [
            {
                "from_sample_index": item.from_sample_index,
                "to_sample_index": item.to_sample_index,
                "suspicious_joint_names": list(item.suspicious_joint_names),
                "records": [
                    {
                        "joint_index": record.joint_index,
                        "joint_name": record.joint_name,
                        "suspicious": record.suspicious,
                        "reasons": list(record.reasons),
                    }
                    for record in item.assessments if record.suspicious
                ],
            }
            for item in analysis.transitions
        ],
    }


def _continuity_payload(global_path: Sequence[Mapping[str, Any]]) -> tuple[dict[str, Any], tuple[TransitionContinuity, ...]]:
    transitions = tuple(
        analyze_joint_transition(
            from_sample_index=index,
            to_sample_index=index + 1,
            previous_joint_positions_rad=global_path[index]["combined"],
            current_joint_positions_rad=global_path[index + 1]["combined"],
        )
        for index in range(len(global_path) - 1)
    )
    summary = summarize_trajectory_continuity(transitions)
    return ({
        "status": "PASS",
        "algorithm_source": (
            "object_trajectory_ik.analyze_joint_transition / "
            "summarize_trajectory_continuity / analyze_suspicious_joint_jumps"
        ),
        "transitions": [_serialize_transition(item) for item in transitions],
        "summary": {
            "transition_count": summary.transition_count,
            "maximum_abs_joint_step_rad": summary.maximum_abs_joint_step_rad,
            "maximum_joint_name": summary.maximum_joint_name,
            "maximum_shortest_abs_joint_step_rad": summary.maximum_shortest_abs_joint_step_rad,
            "maximum_shortest_joint_name": summary.maximum_shortest_joint_name,
            "wraparound_adjusted_transition_count": summary.wraparound_adjusted_transition_count,
            "wraparound_adjusted_record_count": summary.wraparound_adjusted_record_count,
        },
        "suspicious_jumps": _jump_payload(transitions),
    }, transitions)


def _normalize_start_state(start_state: Any) -> dict[str, Any] | None:
    if start_state is None:
        return None
    if not isinstance(start_state, Mapping):
        raise Phase4ValidationInputError("start_state must be an object")
    left = _joint_vector(start_state.get("left"), "start_state.left", 6)
    right = _joint_vector(start_state.get("right"), "start_state.right", 6)
    source = start_state.get("source")
    if not isinstance(source, str) or not source.strip():
        raise Phase4ValidationInputError("start_state.source must be a non-empty string")
    return {"left": left, "right": right, "combined": left + right, "source": source.strip()}


def _start_transition_payload(start_state: dict[str, Any] | None, first: Mapping[str, Any]) -> dict[str, Any]:
    if start_state is None:
        return {
            "status": "NOT_EVALUATED",
            "source": "UNAVAILABLE",
            "reason": "Explicit validation start state was not supplied",
        }
    # Existing continuity types require monotonically increasing numeric sample
    # indices.  Internal 0->1 is relabelled below as START_STATE->sample 0.
    transition = analyze_joint_transition(
        from_sample_index=0,
        to_sample_index=1,
        previous_joint_positions_rad=start_state["combined"],
        current_joint_positions_rad=first["combined"],
    )
    payload = _serialize_transition(transition)
    payload.update({
        "status": "EVALUATED",
        "source": start_state["source"],
        "from_state": "START_STATE",
        "from_sample_index": None,
        "to_sample_index": 0,
        "suspicious_jumps": _jump_payload((transition,)),
    })
    return payload


def compute_joint_velocities(global_path: Sequence[Mapping[str, Any]], limits: MoveItDynamicsLimits) -> dict[str, Any]:
    segments: list[dict[str, Any]] = []
    violations: list[dict[str, Any]] = []
    maximum: dict[str, Any] | None = None
    for index in range(len(global_path) - 1):
        dt = float(global_path[index + 1]["time_from_start_s"]) - float(global_path[index]["time_from_start_s"])
        if not math.isfinite(dt) or dt <= 0.0:
            return {"status": "NOT_EVALUATED", "reason": "Common timeline is invalid", "segments": []}
        previous = global_path[index]["combined"]
        current = global_path[index + 1]["combined"]
        values = [(current[joint] - previous[joint]) / dt for joint in range(12)]
        segment = {
            "from_sample_index": index,
            "to_sample_index": index + 1,
            "dt_s": dt,
            "velocity_rad_s": values,
            "absolute_velocity_rad_s": [abs(value) for value in values],
        }
        segments.append(segment)
        for joint, value in enumerate(values):
            absolute = abs(value)
            candidate = {
                "value_rad_s": absolute,
                "signed_value_rad_s": value,
                "joint_index": joint,
                "joint_name": DUAL_ARM_JOINT_ORDER[joint],
                "from_sample_index": index,
                "to_sample_index": index + 1,
            }
            if maximum is None or absolute > maximum["value_rad_s"]:
                maximum = candidate
            if absolute > limits.velocity_limits_rad_s[joint] + NUMERICAL_LIMIT_EPSILON:
                violations.append({**candidate, "limit_rad_s": limits.velocity_limits_rad_s[joint]})
    return {
        "status": "PASS" if not violations else "FAIL",
        "coordinate_policy": "RAW EXPLICIT JOINT COORDINATES — NO SHORTEST-ANGLE NORMALIZATION",
        "limit_source": limits.velocity_limit_source,
        "limit_source_path": limits.source_path,
        "limits_rad_s": list(limits.velocity_limits_rad_s),
        "default_velocity_scaling_factor": limits.default_velocity_scaling_factor,
        "scaling_factor_is_hard_limit": False,
        "segments": segments,
        "violations": violations,
        "max_abs_velocity_rad_s": maximum["value_rad_s"] if maximum else None,
        "maximum": maximum,
    }


def compute_discrete_accelerations(velocity: Mapping[str, Any], limits: MoveItDynamicsLimits) -> dict[str, Any]:
    segments = velocity.get("segments")
    if not isinstance(segments, list) or len(segments) < 2:
        return {
            "compute_status": "NOT_APPLICABLE",
            "semantic": ACCELERATION_ESTIMATE_SEMANTIC,
            "limit_status": limits.acceleration_limit_status,
            "limit_validation": "NOT_EVALUATED",
            "limit_source": limits.acceleration_limit_source,
            "samples": [],
            "max_abs_discrete_acceleration_rad_s2": None,
            "maximum": None,
        }
    samples: list[dict[str, Any]] = []
    maximum: dict[str, Any] | None = None
    for index in range(len(segments) - 1):
        previous = segments[index]
        following = segments[index + 1]
        denominator = previous["dt_s"] + following["dt_s"]
        values = [
            2.0 * (following["velocity_rad_s"][joint] - previous["velocity_rad_s"][joint]) / denominator
            for joint in range(12)
        ]
        sample_index = index + 1
        samples.append({"sample_index": sample_index, "discrete_acceleration_rad_s2": values})
        for joint, value in enumerate(values):
            candidate = {
                "value_rad_s2": abs(value),
                "signed_value_rad_s2": value,
                "joint_index": joint,
                "joint_name": DUAL_ARM_JOINT_ORDER[joint],
                "sample_index": sample_index,
            }
            if maximum is None or candidate["value_rad_s2"] > maximum["value_rad_s2"]:
                maximum = candidate
    limit_validation = "NOT_EVALUATED"
    if limits.acceleration_limits_rad_s2 is not None:
        limit_validation = "PASS"
        for sample in samples:
            if any(
                abs(value) > limits.acceleration_limits_rad_s2[joint] + NUMERICAL_LIMIT_EPSILON
                for joint, value in enumerate(sample["discrete_acceleration_rad_s2"])
            ):
                limit_validation = "FAIL"
                break
    return {
        "compute_status": "COMPUTED",
        "semantic": ACCELERATION_ESTIMATE_SEMANTIC,
        "continuity_claim": "PIECEWISE-LINEAR PREVIEW IS NOT CLAIMED PHYSICALLY CONTINUOUS",
        "limit_status": limits.acceleration_limit_status,
        "limit_validation": limit_validation,
        "limit_source": limits.acceleration_limit_source,
        "limits_rad_s2": list(limits.acceleration_limits_rad_s2) if limits.acceleration_limits_rad_s2 else None,
        "samples": samples,
        "max_abs_discrete_acceleration_rad_s2": maximum["value_rad_s2"] if maximum else None,
        "maximum": maximum,
    }


def _fk_unavailable(status: str, error: str, position_tolerance_m: float, orientation_tolerance_rad: float) -> tuple[dict[str, Any], dict[str, Any]]:
    common = {
        "status": status,
        "error": error,
        "samples": [],
        "position_tolerance_m": position_tolerance_m,
        "orientation_tolerance_rad": orientation_tolerance_rad,
        "tolerance_semantic": MODEL_CONSISTENCY_TOLERANCE_SEMANTIC,
    }
    relative = {
        "status": "NOT_EVALUATED",
        "error": error,
        "samples": [],
        "max_translation_error_m": None,
        "max_orientation_error_rad": None,
    }
    return common, relative


def _validate_fk(
    object_samples: Sequence[Mapping[str, Any]],
    global_path: Sequence[Mapping[str, Any]],
    adapter: Phase4FkAdapter | None,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
    expected_locked_relative: RigidTransform,
    unavailable_error: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    if adapter is None:
        return _fk_unavailable(
            "UNAVAILABLE",
            unavailable_error or "MoveIt /compute_fk adapter is unavailable",
            position_tolerance_m,
            orientation_tolerance_rad,
        )
    fk_samples: list[dict[str, Any]] = []
    relative_samples: list[dict[str, Any]] = []
    for index, (target_sample, path_point) in enumerate(zip(object_samples, global_path)):
        try:
            result = adapter.compute_combined_fk(path_point["combined"], timeout_s=2.0)
        except TimeoutError as error:
            return _fk_unavailable("TIMEOUT", str(error), position_tolerance_m, orientation_tolerance_rad)
        except Exception as error:
            return _fk_unavailable("ERROR", str(error), position_tolerance_m, orientation_tolerance_rad)
        status = result.get("status")
        left_fk = result.get("left")
        right_fk = result.get("right")
        if status != "PASS" or not isinstance(left_fk, RigidTransform) or not isinstance(right_fk, RigidTransform):
            return _fk_unavailable(str(status or "ERROR"), str(result.get("error") or "MoveIt FK response is malformed"), position_tolerance_m, orientation_tolerance_rad)
        left_target = _transform(target_sample.get("left_target"), f"object_samples[{index}].left_target")
        right_target = _transform(target_sample.get("right_target"), f"object_samples[{index}].right_target")
        left_position, left_orientation = transform_error(left_target, left_fk)
        right_position, right_orientation = transform_error(right_target, right_fk)
        sample_pass = (
            left_position <= position_tolerance_m
            and right_position <= position_tolerance_m
            and left_orientation <= orientation_tolerance_rad
            and right_orientation <= orientation_tolerance_rad
        )
        fk_samples.append({
            "sample_index": index,
            "status": "PASS" if sample_pass else "FAIL",
            "left_position_error_m": left_position,
            "left_orientation_error_rad": left_orientation,
            "right_position_error_m": right_position,
            "right_orientation_error_rad": right_orientation,
        })
        fk_relative = compose(inverse(left_fk), right_fk)
        translation_error, orientation_error = transform_error(
            expected_locked_relative, fk_relative
        )
        relative_samples.append({
            "sample_index": index,
            "status": "PASS" if translation_error <= position_tolerance_m and orientation_error <= orientation_tolerance_rad else "FAIL",
            "translation_error_m": translation_error,
            "orientation_error_rad": orientation_error,
            "expected_locked_left_T_right": [
                list(row) for row in expected_locked_relative.matrix
            ],
            "fk_left_T_right": [list(row) for row in fk_relative.matrix],
        })

    max_fields = {}
    for arm in ("left", "right"):
        for metric, suffix in (("position", "m"), ("orientation", "rad")):
            key = f"{arm}_{metric}_error_{suffix}"
            maximum = max(fk_samples, key=lambda sample: sample[key])
            max_fields[f"max_{key}"] = maximum[key]
            max_fields[f"max_{arm}_{metric}_error_sample_index"] = maximum["sample_index"]
    max_translation = max(relative_samples, key=lambda sample: sample["translation_error_m"])
    max_orientation = max(relative_samples, key=lambda sample: sample["orientation_error_rad"])
    fk_status = "PASS" if all(sample["status"] == "PASS" for sample in fk_samples) else "FAIL"
    relative_status = "PASS" if all(sample["status"] == "PASS" for sample in relative_samples) else "FAIL"
    return ({
        "status": fk_status,
        "service": "/compute_fk",
        "position_tolerance_m": position_tolerance_m,
        "orientation_tolerance_rad": orientation_tolerance_rad,
        "tolerance_semantic": MODEL_CONSISTENCY_TOLERANCE_SEMANTIC,
        "samples": fk_samples,
        **max_fields,
    }, {
        "status": relative_status,
        "semantic": "KINEMATIC PLANNING MODEL ONLY — NOT PHYSICAL GRASP CALIBRATION",
        "expected_locked_relative_translation_m": list(
            expected_locked_relative.translation_m
        ),
        "expected_locked_relative_rotation_matrix": [
            list(row[:3]) for row in expected_locked_relative.matrix[:3]
        ],
        "samples": relative_samples,
        "max_translation_error_m": max_translation["translation_error_m"],
        "max_translation_error_sample_index": max_translation["sample_index"],
        "max_orientation_error_rad": max_orientation["orientation_error_rad"],
        "max_orientation_error_sample_index": max_orientation["sample_index"],
    })


def validate_phase4_trajectory(
    plan: Any,
    *,
    fk_adapter: Phase4FkAdapter | None = None,
    joint_position_limits: CanonicalJointPositionLimits | None = None,
    start_state: Any = None,
    position_tolerance_m: Any = DEFAULT_POSITION_TOLERANCE_M,
    orientation_tolerance_rad: Any = DEFAULT_ORIENTATION_TOLERANCE_RAD,
    dynamics_limits: MoveItDynamicsLimits | None = None,
    fk_unavailable_error: str | None = None,
) -> dict[str, Any]:
    """Validate one successful Phase-3 selected path without executing it."""
    if not isinstance(plan, Mapping):
        raise Phase4ValidationInputError("plan must be an object")
    if plan.get("ok") is not True or plan.get("planner_status") != "READY":
        raise Phase4ValidationInputError("plan must be a successful Phase-3 Global Plan")
    object_samples = plan.get("object_samples")
    global_path = plan.get("global_path")
    if (
        not isinstance(object_samples, list)
        or not isinstance(global_path, list)
        or not object_samples
        or not global_path
    ):
        raise Phase4ValidationInputError("plan must contain non-empty object_samples and global_path arrays")
    normalized_path: list[dict[str, Any]] = []
    for index, point in enumerate(global_path):
        if not isinstance(point, Mapping):
            raise Phase4ValidationInputError(f"global_path[{index}] must be an object")
        combined = _joint_vector(point.get("combined"), f"global_path[{index}].combined")
        left = _joint_vector(point.get("left"), f"global_path[{index}].left", 6)
        right = _joint_vector(point.get("right"), f"global_path[{index}].right", 6)
        if combined != left + right:
            raise Phase4ValidationInputError(f"global_path[{index}] combined joints do not match left + right")
        normalized_path.append({**point, "left": left, "right": right, "combined": combined})
    raw_motion_path = plan.get("combined_path") if isinstance(plan.get("combined_path"), list) else global_path
    normalized_motion_path: list[dict[str, Any]] = []
    for index, point in enumerate(raw_motion_path):
        if not isinstance(point, Mapping):
            raise Phase4ValidationInputError(f"combined_path[{index}] must be an object")
        combined = _joint_vector(point.get("combined"), f"combined_path[{index}].combined")
        left = _joint_vector(point.get("left"), f"combined_path[{index}].left", 6)
        right = _joint_vector(point.get("right"), f"combined_path[{index}].right", 6)
        if combined != left + right:
            raise Phase4ValidationInputError(f"combined_path[{index}] combined joints do not match left + right")
        normalized_motion_path.append({**point, "left": left, "right": right, "combined": combined})
    checked_start = _normalize_start_state(start_state)
    position_tolerance = _nonnegative_tolerance(position_tolerance_m, "position_tolerance_m")
    orientation_tolerance = _nonnegative_tolerance(orientation_tolerance_rad, "orientation_tolerance_rad")
    limits = dynamics_limits or load_moveit_joint_dynamics_limits()

    fixed_grasp, expected_locked_relative = validate_locked_grasp_targets(
        plan,
        object_samples,
        position_tolerance,
        orientation_tolerance,
    )

    timeline = validate_common_timeline(plan)
    continuity, _ = _continuity_payload(normalized_motion_path)
    start_transition = _start_transition_payload(checked_start, normalized_motion_path[0])
    velocity = compute_joint_velocities(normalized_motion_path, limits) if timeline["status"] == COMMON_TIMELINE_SUCCESS else {"status": "NOT_EVALUATED", "reason": "Common timeline is invalid", "segments": []}
    acceleration = compute_discrete_accelerations(velocity, limits)
    if timeline["status"] == COMMON_TIMELINE_SUCCESS:
        fk, relative_pose = _validate_fk(
            object_samples,
            normalized_path,
            fk_adapter,
            position_tolerance,
            orientation_tolerance,
            expected_locked_relative,
            fk_unavailable_error,
        )
    else:
        fk, relative_pose = _fk_unavailable(
            "NOT_EVALUATED",
            "Common timeline is invalid",
            position_tolerance,
            orientation_tolerance,
        )

    final_index = len(normalized_motion_path) - 1
    final = normalized_motion_path[final_index]
    position_limit_status = "NOT_EVALUATED"
    if joint_position_limits is not None:
        position_limit_status = "PASS" if joint_position_limits.contains(final["combined"]) else "FAIL"
    end_state = {
        "status": "REPORTED",
        "sample_index": final_index,
        "time_from_start_s": final.get("time_from_start_s"),
        "left": list(final["left"]),
        "right": list(final["right"]),
        "combined": list(final["combined"]),
        "joint_position_limits": position_limit_status,
        "fk": (fk.get("samples") or [{}])[-1].get("status", fk["status"]),
        "relative_pose": (relative_pose.get("samples") or [{}])[-1].get("status", relative_pose["status"]),
    }
    if timeline["status"] != COMMON_TIMELINE_SUCCESS or velocity["status"] == "FAIL" or fk["status"] == "FAIL" or fixed_grasp["status"] == "FAIL" or position_limit_status == "FAIL":
        overall = "FAIL"
    elif fk["status"] != "PASS" or start_transition["status"] == "NOT_EVALUATED" or acceleration["limit_validation"] == "NOT_EVALUATED":
        overall = "INCOMPLETE"
    else:
        overall = "PASS"
    return {
        "status": overall,
        "plan_only": True,
        "execution_ready": None,
        "warnings": [
            "MODEL-CONSISTENCY CHECK",
            "NOT PHYSICAL METROLOGY",
            DYNAMIC_TIMELINE_WARNING,
            "NOT EXECUTION READY",
        ] + (["ACCELERATION LIMIT UNAVAILABLE"] if acceleration["limit_status"] == "LIMIT_UNAVAILABLE" else []),
        "timeline": timeline,
        "start_transition": start_transition,
        "continuity": continuity,
        "end_state": end_state,
        "fk": fk,
        "relative_pose": relative_pose,
        "fixed_grasp": fixed_grasp,
        "velocity": velocity,
        "acceleration": acceleration,
    }
