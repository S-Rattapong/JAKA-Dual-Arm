"""Immutable Phase-5 execution artifact for P5.1-P5.3.

OFFLINE DATA CONTRACT ONLY. This module freezes a Phase-4 validated
Phase-3 trajectory, splits the canonical 12-joint samples into Left/Right
tracks, and preserves one shared timestamp authority. It has no ROS,
driver, controller, endpoint, or robot-motion dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Any, Mapping, Sequence

from dual_arm_app.backend.object_trajectory_ik import (
    DUAL_ARM_JOINT_ORDER,
    LEFT_JOINT_ORDER,
    RIGHT_JOINT_ORDER,
)
from dual_arm_app.backend.phase4_unified_validation import (
    REPORT_VERSION,
    REQUIRED_CHECKS,
    evaluate_execution_gate,
)


ARTIFACT_VERSION = "PHASE5_EXECUTION_ARTIFACT_V1"
ARTIFACT_STATUS = "FROZEN"
TIMELINE_SEMANTIC = "ONE SHARED COMMON TIMELINE FOR LEFT AND RIGHT"


class Phase5ExecutionArtifactError(ValueError):
    """Raised when validated execution data cannot be frozen safely."""


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Phase5ExecutionArtifactError(f"{path} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise Phase5ExecutionArtifactError(f"{path} must be finite")
    return number


def _joint_tuple(value: Any, count: int, path: str) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bool))
        or len(value) != count
    ):
        raise Phase5ExecutionArtifactError(
            f"{path} must contain exactly {count} joint values"
        )
    return tuple(
        _finite_number(item, f"{path}[{index}]")
        for index, item in enumerate(value)
    )


def _nonempty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise Phase5ExecutionArtifactError(f"{path} must be a non-empty string")
    return value.strip()


def _optional_start_state(value: Any, path: str):
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise Phase5ExecutionArtifactError(f"{path} must be an object or null")
    left = _joint_tuple(value.get("left"), 6, f"{path}.left")
    right = _joint_tuple(value.get("right"), 6, f"{path}.right")
    return (left, right)


def _canonical_json(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _fingerprint(identity: Mapping[str, Any]) -> str:
    return "sha256:" + hashlib.sha256(_canonical_json(identity)).hexdigest()


@dataclass(frozen=True, slots=True)
class Phase5ExecutionArtifact:
    artifact_fingerprint: str
    plan_fingerprint: str
    validation_report_version: str
    trajectory_name: str
    calibration_revision: str
    model_calibration_revision: str
    grasp_content_revision: str
    grasp_lock_generation: int
    grasp_lock_revision: str
    common_timestamps_s: tuple[float, ...]
    combined_positions_rad: tuple[tuple[float, ...], ...]
    left_positions_rad: tuple[tuple[float, ...], ...]
    right_positions_rad: tuple[tuple[float, ...], ...]
    planning_start_left_rad: tuple[float, ...]
    planning_start_right_rad: tuple[float, ...]
    validation_start_left_rad: tuple[float, ...] | None
    validation_start_right_rad: tuple[float, ...] | None
    duration_s: float

    @property
    def sample_count(self) -> int:
        return len(self.common_timestamps_s)

    def public_payload(self) -> dict[str, Any]:
        validation_start = None
        if self.validation_start_left_rad is not None:
            validation_start = {
                "left": list(self.validation_start_left_rad),
                "right": list(self.validation_start_right_rad or ()),
            }
        return {
            "artifact_version": ARTIFACT_VERSION,
            "artifact_status": ARTIFACT_STATUS,
            "artifact_fingerprint": self.artifact_fingerprint,
            "source": {
                "phase4_plan_fingerprint": self.plan_fingerprint,
                "phase4_report_version": self.validation_report_version,
                "frozen_from_phase4_pass": True,
            },
            "authority": {
                "calibration_revision": self.calibration_revision,
                "model_calibration_revision": self.model_calibration_revision,
                "grasp_content_revision": self.grasp_content_revision,
                "grasp_lock_generation": self.grasp_lock_generation,
                "grasp_lock_revision": self.grasp_lock_revision,
            },
            "trajectory": {
                "name": self.trajectory_name,
                "sample_count": self.sample_count,
                "duration_s": self.duration_s,
                "joint_order": list(DUAL_ARM_JOINT_ORDER),
                "common_timestamps_s": list(self.common_timestamps_s),
                "timestamp_authority": TIMELINE_SEMANTIC,
                "combined_positions_rad": [
                    list(sample) for sample in self.combined_positions_rad
                ],
                "left": {
                    "joint_order": list(LEFT_JOINT_ORDER),
                    "positions_rad": [
                        list(sample) for sample in self.left_positions_rad
                    ],
                },
                "right": {
                    "joint_order": list(RIGHT_JOINT_ORDER),
                    "positions_rad": [
                        list(sample) for sample in self.right_positions_rad
                    ],
                },
            },
            "planning_start_state_rad": {
                "left": list(self.planning_start_left_rad),
                "right": list(self.planning_start_right_rad),
            },
            "validation_start_state_rad": validation_start,
            "semantic": (
                "IMMUTABLE VALIDATED TRAJECTORY DATA ONLY — "
                "NO ROBOT MOTION COMMAND"
            ),
        }


def _require_phase4_pass(report: Any) -> str:
    if not isinstance(report, Mapping):
        raise Phase5ExecutionArtifactError("Phase-4 report must be an object")
    if report.get("report_version") != REPORT_VERSION:
        raise Phase5ExecutionArtifactError("Phase-4 report version is not authoritative")
    if report.get("overall_status") != "PASS":
        raise Phase5ExecutionArtifactError("Phase-4 overall status must be PASS")
    fingerprint = _nonempty_string(
        report.get("plan_fingerprint"), "report.plan_fingerprint"
    )
    checks = report.get("checks")
    if not isinstance(checks, Mapping) or any(
        not isinstance(checks.get(name), Mapping)
        or checks[name].get("status") != "PASS"
        for name in REQUIRED_CHECKS
    ):
        raise Phase5ExecutionArtifactError(
            "All required current-scope Phase-4 checks must be PASS"
        )
    gate = evaluate_execution_gate(report, fingerprint)
    if gate.get("execution_ready") is not True:
        raise Phase5ExecutionArtifactError(
            "Phase-4 execution gate is not READY for its validated fingerprint"
        )
    return fingerprint


def _freeze_motion_path(plan: Mapping[str, Any]):
    raw_path = plan.get("combined_path")
    raw_times = plan.get("combined_timestamps_s")
    if not isinstance(raw_path, list) or not raw_path:
        raise Phase5ExecutionArtifactError("plan.combined_path must be non-empty")
    if not isinstance(raw_times, list) or len(raw_times) != len(raw_path):
        raise Phase5ExecutionArtifactError(
            "combined_timestamps_s must match combined_path sample count"
        )

    timestamps: list[float] = []
    combined_samples: list[tuple[float, ...]] = []
    left_samples: list[tuple[float, ...]] = []
    right_samples: list[tuple[float, ...]] = []
    for index, point in enumerate(raw_path):
        if not isinstance(point, Mapping):
            raise Phase5ExecutionArtifactError(
                f"combined_path[{index}] must be an object"
            )
        if point.get("sample_index") != index:
            raise Phase5ExecutionArtifactError(
                f"combined_path[{index}] sample_index must equal {index}"
            )
        point_time = _finite_number(
            point.get("time_from_start_s"), f"combined_path[{index}].time_from_start_s"
        )
        listed_time = _finite_number(
            raw_times[index], f"combined_timestamps_s[{index}]"
        )
        if not math.isclose(point_time, listed_time, rel_tol=0.0, abs_tol=1e-12):
            raise Phase5ExecutionArtifactError(
                f"timestamp mismatch at combined sample {index}"
            )
        if point_time < 0.0 or (timestamps and point_time <= timestamps[-1]):
            raise Phase5ExecutionArtifactError(
                "combined timestamps must be nonnegative and strictly increasing"
            )

        left = _joint_tuple(point.get("left"), 6, f"combined_path[{index}].left")
        right = _joint_tuple(point.get("right"), 6, f"combined_path[{index}].right")
        combined = _joint_tuple(
            point.get("combined"), 12, f"combined_path[{index}].combined"
        )
        if combined != left + right:
            raise Phase5ExecutionArtifactError(
                f"combined_path[{index}] is not canonical Left[6] + Right[6]"
            )
        timestamps.append(point_time)
        left_samples.append(left)
        right_samples.append(right)
        combined_samples.append(combined)

    raw_objects = plan.get("combined_object_samples")
    if not isinstance(raw_objects, list) or len(raw_objects) != len(timestamps):
        raise Phase5ExecutionArtifactError(
            "combined_object_samples must match the frozen joint sample count"
        )
    for index, sample in enumerate(raw_objects):
        if not isinstance(sample, Mapping):
            raise Phase5ExecutionArtifactError(
                f"combined_object_samples[{index}] must be an object"
            )
        object_time = _finite_number(
            sample.get("time_from_start_s"),
            f"combined_object_samples[{index}].time_from_start_s",
        )
        if not math.isclose(
            object_time, timestamps[index], rel_tol=0.0, abs_tol=1e-12
        ):
            raise Phase5ExecutionArtifactError(
                f"object/joint timeline mismatch at sample {index}"
            )

    duration = _finite_number(plan.get("combined_duration_s"), "combined_duration_s")
    if not math.isclose(duration, timestamps[-1], rel_tol=0.0, abs_tol=1e-9):
        raise Phase5ExecutionArtifactError(
            "combined_duration_s must equal the final common timestamp"
        )

    return (
        tuple(timestamps),
        tuple(combined_samples),
        tuple(left_samples),
        tuple(right_samples),
        duration,
    )


def freeze_validated_execution_artifact(
    plan: Any,
    phase4_report: Any,
    *,
    validation_start_state: Any = None,
) -> Phase5ExecutionArtifact:
    """Freeze one Phase-4 PASS plan into the P5.1-P5.3 contract."""
    if not isinstance(plan, Mapping):
        raise Phase5ExecutionArtifactError("Phase-3 plan must be an object")
    if plan.get("ok") is not True or plan.get("planner_status") != "READY":
        raise Phase5ExecutionArtifactError(
            "Only a successful READY Phase-3 plan can become executable data"
        )
    if plan.get("plan_only") is not True:
        raise Phase5ExecutionArtifactError("Phase-3 plan must retain plan_only=true")

    plan_fingerprint = _require_phase4_pass(phase4_report)
    trajectory_name = _nonempty_string(plan.get("trajectory_name"), "trajectory_name")
    calibration = plan.get("calibration")
    grasp = plan.get("grasp")
    if not isinstance(calibration, Mapping) or not isinstance(grasp, Mapping):
        raise Phase5ExecutionArtifactError(
            "Plan must carry calibration and locked-grasp authority metadata"
        )
    calibration_revision = _nonempty_string(
        calibration.get("revision"), "calibration.revision"
    )
    model_calibration_revision = _nonempty_string(
        calibration.get("model_revision"), "calibration.model_revision"
    )
    grasp_content_revision = _nonempty_string(
        grasp.get("content_revision"), "grasp.content_revision"
    )
    grasp_lock_revision = _nonempty_string(
        grasp.get("lock_revision"), "grasp.lock_revision"
    )
    lock_generation = grasp.get("lock_generation")
    if isinstance(lock_generation, bool) or not isinstance(lock_generation, int):
        raise Phase5ExecutionArtifactError("grasp.lock_generation must be an integer")

    planning_start = _optional_start_state(
        plan.get("planning_start_state_rad"), "planning_start_state_rad"
    )
    if planning_start is None:
        raise Phase5ExecutionArtifactError("planning_start_state_rad is required")
    validation_start = _optional_start_state(
        validation_start_state, "validation_start_state"
    )
    (
        timestamps,
        combined_samples,
        left_samples,
        right_samples,
        duration,
    ) = _freeze_motion_path(plan)

    identity = {
        "artifact_version": ARTIFACT_VERSION,
        "plan_fingerprint": plan_fingerprint,
        "validation_report_version": phase4_report["report_version"],
        "trajectory_name": trajectory_name,
        "authority": {
            "calibration_revision": calibration_revision,
            "model_calibration_revision": model_calibration_revision,
            "grasp_content_revision": grasp_content_revision,
            "grasp_lock_generation": lock_generation,
            "grasp_lock_revision": grasp_lock_revision,
        },
        "joint_order": list(DUAL_ARM_JOINT_ORDER),
        "common_timestamps_s": list(timestamps),
        "combined_positions_rad": [list(sample) for sample in combined_samples],
        "planning_start_state_rad": {
            "left": list(planning_start[0]),
            "right": list(planning_start[1]),
        },
        "validation_start_state_rad": (
            {
                "left": list(validation_start[0]),
                "right": list(validation_start[1]),
            }
            if validation_start is not None
            else None
        ),
        "duration_s": duration,
    }
    artifact_fingerprint = _fingerprint(identity)
    return Phase5ExecutionArtifact(
        artifact_fingerprint=artifact_fingerprint,
        plan_fingerprint=plan_fingerprint,
        validation_report_version=phase4_report["report_version"],
        trajectory_name=trajectory_name,
        calibration_revision=calibration_revision,
        model_calibration_revision=model_calibration_revision,
        grasp_content_revision=grasp_content_revision,
        grasp_lock_generation=lock_generation,
        grasp_lock_revision=grasp_lock_revision,
        common_timestamps_s=timestamps,
        combined_positions_rad=combined_samples,
        left_positions_rad=left_samples,
        right_positions_rad=right_samples,
        planning_start_left_rad=planning_start[0],
        planning_start_right_rad=planning_start[1],
        validation_start_left_rad=(validation_start[0] if validation_start else None),
        validation_start_right_rad=(validation_start[1] if validation_start else None),
        duration_s=duration,
    )
