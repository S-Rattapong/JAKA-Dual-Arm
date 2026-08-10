"""Pure adaptive joint-space sampling for stored dual-arm trajectories."""

from __future__ import annotations

import math
from typing import Any, Sequence


DEFAULT_MAX_JOINT_STEP_RAD = 0.05
MAX_INTERIOR_SAMPLE_COUNT = 5000
JOINT_COUNT = 12


class SampledPathValidationInputError(ValueError):
    """Raised when sampled-path configuration or input is unsafe."""


def validate_max_joint_step(value: Any) -> float:
    """Return a finite positive maximum step in radians."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SampledPathValidationInputError(
            "sampled_path.max_joint_step_rad must be a finite number > 0"
        )
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0:
        raise SampledPathValidationInputError(
            "sampled_path.max_joint_step_rad must be a finite number > 0"
        )
    return converted


def normalize_sampled_path_options(options: Any) -> dict[str, Any]:
    """Normalize optional endpoint configuration without changing its source."""
    if options is None:
        return {
            "enabled": False,
            "max_joint_step_rad": DEFAULT_MAX_JOINT_STEP_RAD,
        }
    if not isinstance(options, dict):
        raise SampledPathValidationInputError("sampled_path must be an object")
    enabled = options.get("enabled", False)
    if not isinstance(enabled, bool):
        raise SampledPathValidationInputError("sampled_path.enabled must be boolean")
    step = validate_max_joint_step(
        options.get("max_joint_step_rad", DEFAULT_MAX_JOINT_STEP_RAD)
    )
    return {"enabled": enabled, "max_joint_step_rad": step}


def _vector(values: Sequence[Any], label: str) -> list[float]:
    if isinstance(values, (str, bytes, bool)) or not isinstance(values, Sequence):
        raise SampledPathValidationInputError(
            f"{label} must contain exactly {JOINT_COUNT} joint values"
        )
    if len(values) != JOINT_COUNT:
        raise SampledPathValidationInputError(
            f"{label} must contain exactly {JOINT_COUNT} joint values"
        )
    result: list[float] = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SampledPathValidationInputError(
                f"{label}[{index}] must be a finite number"
            )
        converted = float(value)
        if not math.isfinite(converted):
            raise SampledPathValidationInputError(
                f"{label}[{index}] must be a finite number"
            )
        result.append(converted)
    return result


def flatten_point(point: dict[str, Any]) -> list[float]:
    """Flatten one six-left/six-right point in canonical joint order."""
    if not isinstance(point, dict):
        raise SampledPathValidationInputError("trajectory point must be an object")
    left = point.get("left")
    right = point.get("right")
    if not isinstance(left, list) or len(left) != 6:
        raise SampledPathValidationInputError("point.left must contain six joints")
    if not isinstance(right, list) or len(right) != 6:
        raise SampledPathValidationInputError("point.right must contain six joints")
    return _vector([*left, *right], "trajectory point")


def max_joint_delta(start: Sequence[Any], end: Sequence[Any]) -> float:
    """Return max_i |end_i - start_i| for two 12-DOF vectors."""
    start_vector = _vector(start, "segment start")
    end_vector = _vector(end, "segment end")
    delta = max(abs(end_value - start_value) for start_value, end_value in zip(
        start_vector, end_vector
    ))
    if not math.isfinite(delta):
        raise SampledPathValidationInputError(
            "segment maximum joint displacement must be finite"
        )
    return delta


def segment_subdivision_count(
    start: Sequence[Any],
    end: Sequence[Any],
    max_joint_step_rad: Any,
) -> int:
    """Compute max(1, ceil(max joint displacement / configured step))."""
    step = validate_max_joint_step(max_joint_step_rad)
    ratio = max_joint_delta(start, end) / step
    if not math.isfinite(ratio):
        raise SampledPathValidationInputError(
            "sampled path subdivision count is not finite"
        )
    return max(1, math.ceil(ratio))


def _timestamp(point: dict[str, Any], label: str) -> float:
    value = point.get("time_from_start_s")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SampledPathValidationInputError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise SampledPathValidationInputError(f"{label} must be a finite number")
    return converted


def generate_segment_interior_samples(
    start_point: dict[str, Any],
    end_point: dict[str, Any],
    segment_index: int,
    max_joint_step_rad: Any,
) -> dict[str, Any]:
    """Generate k=1..N-1 samples for one trajectory segment."""
    if not isinstance(segment_index, int) or isinstance(segment_index, bool) or segment_index < 0:
        raise SampledPathValidationInputError("segment_index must be a non-negative integer")
    step = validate_max_joint_step(max_joint_step_rad)
    start = flatten_point(start_point)
    end = flatten_point(end_point)
    start_time = _timestamp(start_point, "segment start time")
    end_time = _timestamp(end_point, "segment end time")
    subdivisions = segment_subdivision_count(start, end, step)
    samples: list[dict[str, Any]] = []
    for sample_index in range(1, subdivisions):
        alpha = sample_index / subdivisions
        positions = [
            (1.0 - alpha) * start_value + alpha * end_value
            for start_value, end_value in zip(start, end)
        ]
        sample_time = (1.0 - alpha) * start_time + alpha * end_time
        samples.append(
            {
                "segment_index": segment_index,
                "start_point_index": segment_index,
                "end_point_index": segment_index + 1,
                "sample_index": sample_index,
                "subdivision_count": subdivisions,
                "alpha": alpha,
                "time_from_start_s": sample_time,
                "sample_time_from_start_s": sample_time,
                "left": positions[:6],
                "right": positions[6:],
            }
        )
    return {
        "segment_index": segment_index,
        "start_point_index": segment_index,
        "end_point_index": segment_index + 1,
        "max_joint_delta_rad": max_joint_delta(start, end),
        "subdivision_count": subdivisions,
        "interior_sample_count": len(samples),
        "samples": samples,
    }


def generate_trajectory_interior_samples(
    trajectory: dict[str, Any],
    max_joint_step_rad: Any = DEFAULT_MAX_JOINT_STEP_RAD,
    *,
    max_interior_sample_count: int = MAX_INTERIOR_SAMPLE_COUNT,
) -> dict[str, Any]:
    """Create bounded adaptive samples without mutating a normalized trajectory."""
    step = validate_max_joint_step(max_joint_step_rad)
    if (
        not isinstance(max_interior_sample_count, int)
        or isinstance(max_interior_sample_count, bool)
        or max_interior_sample_count < 0
    ):
        raise SampledPathValidationInputError(
            "max_interior_sample_count must be a non-negative integer"
        )
    if not isinstance(trajectory, dict) or not isinstance(trajectory.get("points"), list):
        raise SampledPathValidationInputError("trajectory.points must be an array")

    points = trajectory["points"]
    segments: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    for segment_index in range(max(0, len(points) - 1)):
        start = flatten_point(points[segment_index])
        end = flatten_point(points[segment_index + 1])
        subdivisions = segment_subdivision_count(start, end, step)
        projected_count = len(samples) + max(0, subdivisions - 1)
        if projected_count > max_interior_sample_count:
            raise SampledPathValidationInputError(
                "sampled path would generate "
                f"{projected_count} interior samples, exceeding cap "
                f"{max_interior_sample_count}"
            )
        segment = generate_segment_interior_samples(
            points[segment_index], points[segment_index + 1], segment_index, step
        )
        samples.extend(segment.pop("samples"))
        segments.append(segment)
    return {
        "max_joint_step_rad": step,
        "segment_count": len(segments),
        "generated_interior_sample_count": len(samples),
        "segments": segments,
        "samples": samples,
    }
