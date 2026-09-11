"""Offline shared time-parameterization of a selected dual-arm joint polyline.

This module does not solve IK or alter the selected spatial path.  It applies
one scalar progress value to all 12 raw joint coordinates and to the matching
user Object segment.  The resulting samples are references for the existing
piecewise-linear command stream; they are not polynomial joint commands.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any, Sequence

from dual_arm_app.backend.object_grasp_model import ObjectGraspModel, RigidTransform
from dual_arm_app.backend.object_trajectory import (
    MultiWaypointObjectTrajectory,
    ObjectTrajectorySample,
)


VELOCITY_SHAPING_PROFILE = "SHARED_POLYLINE_LOCAL_S_CURVE_V1"
VELOCITY_SHAPING_SCOPE = "RIGID_USER_WAYPOINT_PATH_ONLY"
VELOCITY_SHAPING_SEMANTIC = (
    "OFFLINE SHARED TIME-PARAMETERIZATION; DRIVER REMAINS LINEAR-V2"
)
DEFAULT_CADENCE_S = 0.024
DEFAULT_DESIRED_RAMP_S = 0.36
DEFAULT_MAX_DENSE_RIGID_SAMPLES = 5000
USER_WAYPOINT_ADJACENT_SPEED_GATE_RAD_S = 0.005
MAX_ADAPTIVE_RAMP_ITERATIONS = 16
_ROUNDING_EPSILON = 1e-12
_PEAK_SPEED_EPSILON = 1e-10


class WaypointVelocityShapingError(ValueError):
    """Raised when a selected coarse path cannot be safely shaped."""


@dataclass(frozen=True)
class SegmentShape:
    """Ideal scalar profile plus its coarse-edge-preserving command schedule."""

    requested_duration_s: float
    base_ticks: int
    ramp_ticks: int
    total_ticks: int
    cadence_s: float
    edge_ticks: tuple[int, ...] = ()
    vertex_ideal_times_s: tuple[float, ...] = ()

    @property
    def reference_duration_s(self) -> float:
        """Duration of the analytic S-curve used as the ideal reference."""
        return self.total_ticks * self.cadence_s

    @property
    def effective_duration_s(self) -> float:
        """Duration emitted after independently rounding every coarse edge up."""
        ticks = sum(self.edge_ticks) if self.edge_ticks else self.total_ticks
        return ticks * self.cadence_s

    @property
    def effective_total_ticks(self) -> int:
        return sum(self.edge_ticks) if self.edge_ticks else self.total_ticks

    @property
    def effective_ramp_s(self) -> float:
        return self.ramp_ticks * self.cadence_s

    @property
    def cruise_scalar_speed(self) -> float:
        return 1.0 / (self.reference_duration_s - self.effective_ramp_s)


@dataclass(frozen=True)
class ScalarProgressState:
    progress: float
    velocity_per_s: float
    acceleration_per_s2: float
    jerk_per_s3: float


@dataclass(frozen=True)
class DenseJointSample:
    sample_index: int
    time_from_start_s: float
    combined_joint_positions_rad: tuple[float, ...]
    user_segment_index: int
    local_tick: int
    progress: float
    coarse_lower_sample_index: int
    coarse_upper_sample_index: int
    coarse_fraction: float


@dataclass(frozen=True)
class WaypointVelocityShapingResult:
    joint_samples: tuple[DenseJointSample, ...]
    object_samples: tuple[ObjectTrajectorySample, ...]
    segment_shapes: tuple[SegmentShape, ...]
    waypoint_ticks: tuple[int, ...]
    diagnostics: dict[str, Any]

    @property
    def duration_s(self) -> float:
        return self.joint_samples[-1].time_from_start_s

    @property
    def effective_segment_durations_s(self) -> tuple[float, ...]:
        return tuple(shape.effective_duration_s for shape in self.segment_shapes)


def _finite_positive(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointVelocityShapingError(f"{label} must be a finite positive number")
    checked = float(value)
    if not math.isfinite(checked) or checked <= 0.0:
        raise WaypointVelocityShapingError(f"{label} must be a finite positive number")
    return checked


def _ceil_ticks(value_s: float, cadence_s: float) -> int:
    ratio = value_s / cadence_s
    nearest = round(ratio)
    tolerance = _ROUNDING_EPSILON * max(1.0, abs(ratio))
    ticks = int(nearest) if abs(ratio - nearest) <= tolerance else math.ceil(ratio)
    ticks = max(1, ticks)
    # A value microscopically above a tick because of ordinary binary residue is
    # treated as exact; any material shortfall is always rounded upward.
    if ticks * cadence_s + _ROUNDING_EPSILON * max(1.0, value_s) < value_s:
        ticks += 1
    return ticks


def build_segment_shape(
    duration_s: Any,
    *,
    cadence_s: Any = DEFAULT_CADENCE_S,
    desired_ramp_s: Any = DEFAULT_DESIRED_RAMP_S,
) -> SegmentShape:
    """Quantize one duration and add endpoint ramps without shortening it."""
    duration = _finite_positive(duration_s, "duration_s")
    cadence = _finite_positive(cadence_s, "cadence_s")
    desired_ramp = _finite_positive(desired_ramp_s, "desired_ramp_s")
    base_ticks = _ceil_ticks(duration, cadence)
    ramp_ticks = _ceil_ticks(desired_ramp, cadence)
    total_ticks = max(base_ticks + ramp_ticks, 2 * ramp_ticks)
    shape = SegmentShape(
        requested_duration_s=duration,
        base_ticks=base_ticks,
        ramp_ticks=ramp_ticks,
        total_ticks=total_ticks,
        cadence_s=cadence,
    )
    if shape.effective_duration_s + _ROUNDING_EPSILON < duration:
        raise WaypointVelocityShapingError("effective segment duration shortened the request")
    if shape.effective_ramp_s + _ROUNDING_EPSILON < desired_ramp:
        raise WaypointVelocityShapingError("effective ramp is shorter than requested")
    if shape.cruise_scalar_speed > 1.0 / duration + _ROUNDING_EPSILON:
        raise WaypointVelocityShapingError("scalar cruise speed exceeds the coarse baseline")
    return shape


def _smoothstep5(u: float) -> float:
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _smoothstep5_d1(u: float) -> float:
    return 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4


def _smoothstep5_d2(u: float) -> float:
    return 60.0 * u - 180.0 * u**2 + 120.0 * u**3


def _smoothstep5_integral(u: float) -> float:
    return 2.5 * u**4 - 3.0 * u**5 + u**6


def evaluate_scalar_progress(shape: SegmentShape, time_s: Any) -> ScalarProgressState:
    """Evaluate analytic progress and its first three time derivatives."""
    if not isinstance(shape, SegmentShape):
        raise TypeError("shape must be a SegmentShape")
    if isinstance(time_s, bool) or not isinstance(time_s, (int, float)):
        raise WaypointVelocityShapingError("time_s must be finite")
    t = float(time_s)
    if not math.isfinite(t) or t < 0.0 or t > shape.reference_duration_s:
        raise WaypointVelocityShapingError("time_s must be within the segment")
    total = shape.reference_duration_s
    ramp = shape.effective_ramp_s
    c = shape.cruise_scalar_speed
    if t == 0.0:
        return ScalarProgressState(0.0, 0.0, 0.0, 0.0)
    if t == total:
        return ScalarProgressState(1.0, 0.0, 0.0, 0.0)
    if t <= ramp:
        u = t / ramp
        state = ScalarProgressState(
            c * ramp * _smoothstep5_integral(u),
            c * _smoothstep5(u),
            c * _smoothstep5_d1(u) / ramp,
            c * _smoothstep5_d2(u) / (ramp * ramp),
        )
    elif t < total - ramp:
        state = ScalarProgressState(c * (t - ramp / 2.0), c, 0.0, 0.0)
    else:
        u = (total - t) / ramp
        state = ScalarProgressState(
            1.0 - c * ramp * _smoothstep5_integral(u),
            c * _smoothstep5(u),
            -c * _smoothstep5_d1(u) / ramp,
            c * _smoothstep5_d2(u) / (ramp * ramp),
        )
    progress = state.progress
    if -_ROUNDING_EPSILON <= progress < 0.0:
        progress = 0.0
    elif 1.0 < progress <= 1.0 + _ROUNDING_EPSILON:
        progress = 1.0
    if not 0.0 <= progress <= 1.0:
        raise WaypointVelocityShapingError("scalar progress escaped [0, 1]")
    return ScalarProgressState(
        progress,
        state.velocity_per_s,
        state.acceleration_per_s2,
        state.jerk_per_s3,
    )


def _inverse_progress_time(shape: SegmentShape, progress: float) -> float:
    """Invert the monotone ideal S-curve without changing its analytic law."""
    if progress <= 0.0:
        return 0.0
    if progress >= 1.0:
        return shape.reference_duration_s
    lower = 0.0
    upper = shape.reference_duration_s
    for _ in range(80):
        middle = (lower + upper) / 2.0
        if evaluate_scalar_progress(shape, middle).progress < progress:
            lower = middle
        else:
            upper = middle
    return (lower + upper) / 2.0


def _schedule_coarse_edges(shape: SegmentShape, coarse_count: int) -> SegmentShape:
    if coarse_count < 2:
        raise WaypointVelocityShapingError(
            "each user segment must contain at least 2 coarse vertices"
        )
    denominator = coarse_count - 1
    vertex_times = tuple(
        _inverse_progress_time(shape, index / denominator)
        for index in range(coarse_count)
    )
    edge_ticks = tuple(
        max(1, _ceil_ticks(upper - lower, shape.cadence_s))
        for lower, upper in zip(vertex_times, vertex_times[1:])
    )
    scheduled = replace(
        shape,
        edge_ticks=edge_ticks,
        vertex_ideal_times_s=vertex_times,
    )
    if any(
        ticks * shape.cadence_s + _ROUNDING_EPSILON < upper - lower
        for ticks, lower, upper in zip(edge_ticks, vertex_times, vertex_times[1:])
    ):
        raise WaypointVelocityShapingError("a coarse edge compressed its ideal interval")
    if (
        scheduled.effective_duration_s + _ROUNDING_EPSILON
        < scheduled.reference_duration_s
    ):
        raise WaypointVelocityShapingError(
            "edge-rounded segment duration compressed its ideal reference"
        )
    return scheduled


def cadence_aligned(time_s: Any, cadence_s: Any = DEFAULT_CADENCE_S) -> bool:
    """Return whether a boundary time lies on the command cadence grid."""
    time_value = _finite_positive(time_s, "time_s")
    cadence = _finite_positive(cadence_s, "cadence_s")
    ratio = time_value / cadence
    return math.isclose(ratio, round(ratio), rel_tol=0.0, abs_tol=_ROUNDING_EPSILON)


def _rotation_quaternion_xyzw(source: RigidTransform) -> tuple[float, float, float, float]:
    m = source.matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m[2][1] - m[1][2]) / scale,
            (m[0][2] - m[2][0]) / scale,
            (m[1][0] - m[0][1]) / scale,
            0.25 * scale,
        )
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        quaternion = (
            0.25 * scale,
            (m[0][1] + m[1][0]) / scale,
            (m[0][2] + m[2][0]) / scale,
            (m[2][1] - m[1][2]) / scale,
        )
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        quaternion = (
            (m[0][1] + m[1][0]) / scale,
            0.25 * scale,
            (m[1][2] + m[2][1]) / scale,
            (m[0][2] - m[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        quaternion = (
            (m[0][2] + m[2][0]) / scale,
            (m[1][2] + m[2][1]) / scale,
            0.25 * scale,
            (m[1][0] - m[0][1]) / scale,
        )
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _quaternion_transform(
    quaternion: Sequence[float], translation_m: Sequence[float]
) -> RigidTransform:
    x, y, z, w = quaternion
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return RigidTransform.from_matrix((
        (1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), translation_m[0]),
        (2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), translation_m[1]),
        (2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), translation_m[2]),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _interpolate_object_pose(
    start: RigidTransform,
    end: RigidTransform,
    progress: float,
) -> RigidTransform:
    translation = tuple(
        start.translation_m[axis]
        if start.translation_m[axis] == end.translation_m[axis]
        else (
            (1.0 - progress) * start.translation_m[axis]
            + progress * end.translation_m[axis]
        )
        for axis in range(3)
    )
    q0 = _rotation_quaternion_xyzw(start)
    q1 = _rotation_quaternion_xyzw(end)
    dot = sum(left * right for left, right in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        quaternion = tuple(
            (1.0 - progress) * left + progress * right
            for left, right in zip(q0, q1)
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        quaternion = tuple(value / norm for value in quaternion)
    else:
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)
        theta = theta0 * progress
        scale0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
        scale1 = math.sin(theta) / sin_theta0
        quaternion = tuple(
            scale0 * left + scale1 * right for left, right in zip(q0, q1)
        )
    return _quaternion_transform(quaternion, translation)


def _joint_vector(value: Any, label: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, bool)) or not isinstance(value, Sequence) or len(value) != 12:
        raise WaypointVelocityShapingError(f"{label} must contain exactly 12 joints")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise WaypointVelocityShapingError(f"{label} must contain finite joints")
    return result


def _maximum_per_joint(
    vectors: Sequence[Sequence[float]],
) -> tuple[list[float], list[int | None]]:
    maximum = [0.0] * 12
    locations: list[int | None] = [None] * 12
    for index, vector in enumerate(vectors):
        for joint_index, value in enumerate(vector):
            magnitude = abs(value)
            if magnitude > maximum[joint_index]:
                maximum[joint_index] = magnitude
                locations[joint_index] = index
    return maximum, locations


def _diagnostics(
    coarse_segments: Sequence[Sequence[tuple[float, ...]]],
    requested_durations: Sequence[float],
    dense: Sequence[DenseJointSample],
    waypoint_ticks: Sequence[int],
    cadence_s: float,
    coarse_vertex_dense_indices: Sequence[int],
    transition_coarse_edges: Sequence[tuple[int, int]],
) -> dict[str, Any]:
    coarse_peaks = [0.0] * 12
    for segment, duration in zip(coarse_segments, requested_durations):
        dt = duration / (len(segment) - 1)
        for previous, current in zip(segment, segment[1:]):
            for joint_index in range(12):
                speed = abs(current[joint_index] - previous[joint_index]) / dt
                coarse_peaks[joint_index] = max(coarse_peaks[joint_index], speed)

    velocities = [
        tuple(
            (current.combined_joint_positions_rad[joint_index]
             - previous.combined_joint_positions_rad[joint_index]) / cadence_s
            for joint_index in range(12)
        )
        for previous, current in zip(dense, dense[1:])
    ]
    accelerations = [
        tuple(
            (current[joint_index] - previous[joint_index]) / cadence_s
            for joint_index in range(12)
        )
        for previous, current in zip(velocities, velocities[1:])
    ]
    jerks = [
        tuple(
            (current[joint_index] - previous[joint_index]) / cadence_s
            for joint_index in range(12)
        )
        for previous, current in zip(accelerations, accelerations[1:])
    ]
    dense_peaks, velocity_locations = _maximum_per_joint(velocities)
    acceleration_peaks, acceleration_locations = _maximum_per_joint(accelerations)
    jerk_peaks, jerk_locations = _maximum_per_joint(jerks)

    waypoint_speed = []
    for waypoint_index, tick in enumerate(waypoint_ticks[1:-1], start=1):
        signed_before = list(velocities[tick - 1])
        signed_after = list(velocities[tick])
        before = [abs(value) for value in signed_before]
        after = [abs(value) for value in signed_after]
        jump = [
            abs(signed_after[index] - signed_before[index])
            for index in range(12)
        ]
        waypoint_speed.append({
            "waypoint_index": waypoint_index,
            "tick": tick,
            "time_s": tick * cadence_s,
            "joint_velocity_before_rad_s": signed_before,
            "joint_velocity_after_rad_s": signed_after,
            "joint_speed_before_rad_s": before,
            "joint_speed_after_rad_s": after,
            "joint_velocity_jump_rad_s": jump,
            "max_before_rad_s": max(before),
            "max_after_rad_s": max(after),
            "max_jump_rad_s": max(jump),
        })

    repeated = []
    for index, (previous, current) in enumerate(zip(dense, dense[1:])):
        if previous.combined_joint_positions_rad == current.combined_joint_positions_rad:
            repeated.append(index)
    flattened_coarse = list(coarse_segments[0]) if coarse_segments else []
    for segment in coarse_segments[1:]:
        flattened_coarse.extend(segment[1:])
    repeated_on_moving = [
        index
        for index in repeated
        if (
            index < len(transition_coarse_edges)
            and flattened_coarse[transition_coarse_edges[index][0]]
            != flattened_coarse[transition_coarse_edges[index][1]]
        )
    ]
    velocity_finite = all(
        math.isfinite(value) for vector in velocities for value in vector
    )
    acceleration_finite = all(
        math.isfinite(value) for vector in accelerations for value in vector
    )
    jerk_finite = all(
        math.isfinite(value) for vector in jerks for value in vector
    )
    vertex_emission_counts = [
        sum(
            assigned_dense_index == dense_index
            for assigned_dense_index in coarse_vertex_dense_indices
        )
        for dense_index in coarse_vertex_dense_indices
    ]
    vertices_exact = (
        len(flattened_coarse) == len(coarse_vertex_dense_indices)
        and len(set(coarse_vertex_dense_indices)) == len(coarse_vertex_dense_indices)
        and all(count == 1 for count in vertex_emission_counts)
        and all(
            0 <= dense_index < len(dense)
            and dense[dense_index].combined_joint_positions_rad == vertex
            for dense_index, vertex in zip(
                coarse_vertex_dense_indices, flattened_coarse
            )
        )
    )
    transition_edges_valid = (
        len(transition_coarse_edges) == max(0, len(dense) - 1)
        and all(
            0 <= lower and upper == lower + 1 and upper < len(flattened_coarse)
            for lower, upper in transition_coarse_edges
        )
    )
    if transition_edges_valid:
        for transition_index, (lower_index, upper_index) in enumerate(
            transition_coarse_edges
        ):
            previous = dense[transition_index]
            current = dense[transition_index + 1]
            lower = flattened_coarse[lower_index]
            upper = flattened_coarse[upper_index]

            def expected_at(fraction: float) -> tuple[float, ...]:
                if fraction == 0.0:
                    return lower
                if fraction == 1.0:
                    return upper
                return tuple(
                    (1.0 - fraction) * lower[joint]
                    + fraction * upper[joint]
                    for joint in range(12)
                )

            current_is_on_edge = (
                current.coarse_lower_sample_index == lower_index
                and current.coarse_upper_sample_index == upper_index
                and current.combined_joint_positions_rad
                == expected_at(current.coarse_fraction)
            )
            previous_is_on_edge = (
                previous.combined_joint_positions_rad == lower
                or (
                    previous.coarse_lower_sample_index == lower_index
                    and previous.coarse_upper_sample_index == upper_index
                    and previous.combined_joint_positions_rad
                    == expected_at(previous.coarse_fraction)
                )
            )
            if not current_is_on_edge or not previous_is_on_edge:
                transition_edges_valid = False
                break
    cadence_grid_passed = all(
        sample.sample_index == index
        and sample.time_from_start_s == index * cadence_s
        for index, sample in enumerate(dense)
    )
    strict_timestamps_passed = all(
        current.time_from_start_s > previous.time_from_start_s
        for previous, current in zip(dense, dense[1:])
    )
    waypoint_speed_gate_passed = all(
        item["max_before_rad_s"]
        <= USER_WAYPOINT_ADJACENT_SPEED_GATE_RAD_S + _ROUNDING_EPSILON
        and item["max_after_rad_s"]
        <= USER_WAYPOINT_ADJACENT_SPEED_GATE_RAD_S + _ROUNDING_EPSILON
        for item in waypoint_speed
    )
    pass_conditions = {
        "cadence_grid_passed": cadence_grid_passed,
        "strict_timestamps_passed": strict_timestamps_passed,
        "discrete_velocity_finite": velocity_finite,
        "discrete_acceleration_finite": acceleration_finite,
        "discrete_jerk_finite": jerk_finite,
        "dense_peak_speed_not_above_coarse": all(
            dense <= coarse + _PEAK_SPEED_EPSILON
            for dense, coarse in zip(dense_peaks, coarse_peaks)
        ),
        "no_repeated_moving_transition_passed": not repeated_on_moving,
        "coarse_vertices_preserved": vertices_exact,
        "consecutive_samples_stay_on_one_coarse_edge": transition_edges_valid,
        "waypoint_speed_gate_passed": waypoint_speed_gate_passed,
    }
    return {
        "engineering_diagnostics_only": True,
        "manufacturer_or_safety_limit_claim": False,
        "coarse_baseline_peak_joint_speed_rad_s": coarse_peaks,
        "dense_peak_joint_speed_rad_s": dense_peaks,
        "dense_peak_joint_speed_transition_indices": velocity_locations,
        "dense_peak_speed_not_above_coarse": pass_conditions[
            "dense_peak_speed_not_above_coarse"
        ],
        "peak_speed_comparison_epsilon_rad_s": _PEAK_SPEED_EPSILON,
        "internal_waypoint_adjacent_speeds": waypoint_speed,
        "user_waypoint_adjacent_speed_gate_rad_s": (
            USER_WAYPOINT_ADJACENT_SPEED_GATE_RAD_S
        ),
        "waypoint_speed_gate_passed": waypoint_speed_gate_passed,
        "discrete_velocity_finite": velocity_finite,
        "discrete_acceleration_finite": acceleration_finite,
        "discrete_jerk_finite": jerk_finite,
        "peak_discrete_acceleration_rad_s2": acceleration_peaks,
        "peak_discrete_acceleration_sample_indices": [
            None if index is None else index + 1 for index in acceleration_locations
        ],
        "peak_discrete_jerk_rad_s3": jerk_peaks,
        "peak_discrete_jerk_sample_indices": [
            None if index is None else index + 2 for index in jerk_locations
        ],
        "repeated_dense_transition_count": len(repeated),
        "repeated_dense_transition_indices": repeated,
        "repeated_transition_on_moving_coarse_segment_count": len(repeated_on_moving),
        "no_repeated_moving_transition_passed": not repeated_on_moving,
        "cadence_grid_passed": cadence_grid_passed,
        "strict_timestamps_passed": strict_timestamps_passed,
        "coarse_vertex_dense_sample_indices": list(coarse_vertex_dense_indices),
        "coarse_vertex_emission_counts": vertex_emission_counts,
        "coarse_vertices_preserved": vertices_exact,
        "consecutive_samples_stay_on_one_coarse_edge": transition_edges_valid,
        "coarse_polyline_preserved": vertices_exact and transition_edges_valid,
        "quality_check_pass_conditions": pass_conditions,
        "quality_check_status": (
            "PASS" if all(pass_conditions.values()) else "FAIL"
        ),
    }


def _build_vertex_preserving_joint_schedule(
    coarse: Sequence[tuple[float, ...]],
    coarse_counts: Sequence[int],
    shapes: Sequence[SegmentShape],
) -> tuple[
    list[DenseJointSample],
    list[tuple[tuple[float, ...], ...]],
    list[int],
    list[int],
    list[tuple[int, int]],
]:
    dense: list[DenseJointSample] = []
    coarse_segments: list[tuple[tuple[float, ...], ...]] = []
    waypoint_ticks = [0]
    vertex_dense_indices = [-1] * len(coarse)
    transition_edges: list[tuple[int, int]] = []
    coarse_offset = 0
    for segment_index, (shape, coarse_count) in enumerate(zip(shapes, coarse_counts)):
        segment = tuple(coarse[coarse_offset:coarse_offset + coarse_count])
        coarse_segments.append(segment)
        local_edge_start_tick = 0
        for edge_index, edge_ticks in enumerate(shape.edge_ticks):
            alpha_lower = edge_index / (coarse_count - 1)
            alpha_upper = (edge_index + 1) / (coarse_count - 1)
            ideal_lower = shape.vertex_ideal_times_s[edge_index]
            ideal_upper = shape.vertex_ideal_times_s[edge_index + 1]
            lower = segment[edge_index]
            upper = segment[edge_index + 1]
            lower_global = coarse_offset + edge_index
            upper_global = lower_global + 1
            first_command_tick = 0 if not dense else 1
            if first_command_tick == 1 and vertex_dense_indices[lower_global] < 0:
                vertex_dense_indices[lower_global] = len(dense) - 1
            for edge_tick in range(first_command_tick, edge_ticks + 1):
                if edge_tick == 0:
                    progress = alpha_lower
                    fraction = 0.0
                    combined = lower
                elif edge_tick == edge_ticks:
                    progress = alpha_upper
                    fraction = 1.0
                    combined = upper
                else:
                    actual_fraction = edge_tick / edge_ticks
                    ideal_time = ideal_lower + actual_fraction * (
                        ideal_upper - ideal_lower
                    )
                    progress = evaluate_scalar_progress(shape, ideal_time).progress
                    fraction = (progress - alpha_lower) / (
                        alpha_upper - alpha_lower
                    )
                    if -_ROUNDING_EPSILON <= fraction < 0.0:
                        fraction = 0.0
                    elif 1.0 < fraction <= 1.0 + _ROUNDING_EPSILON:
                        fraction = 1.0
                    if not 0.0 <= fraction <= 1.0:
                        raise WaypointVelocityShapingError(
                            "coarse-edge interpolation fraction escaped [0, 1]"
                        )
                    combined = tuple(
                        (1.0 - fraction) * lower[joint]
                        + fraction * upper[joint]
                        for joint in range(12)
                    )
                sample_index = len(dense)
                if sample_index:
                    transition_edges.append((lower_global, upper_global))
                dense.append(DenseJointSample(
                    sample_index=sample_index,
                    time_from_start_s=sample_index * shape.cadence_s,
                    combined_joint_positions_rad=combined,
                    user_segment_index=segment_index,
                    local_tick=local_edge_start_tick + edge_tick,
                    progress=progress,
                    coarse_lower_sample_index=lower_global,
                    coarse_upper_sample_index=upper_global,
                    coarse_fraction=fraction,
                ))
                if edge_tick == 0:
                    vertex_dense_indices[lower_global] = sample_index
                if edge_tick == edge_ticks:
                    vertex_dense_indices[upper_global] = sample_index
            local_edge_start_tick += edge_ticks
        waypoint_ticks.append(len(dense) - 1)
        coarse_offset += coarse_count - 1
    if any(index < 0 for index in vertex_dense_indices):
        raise WaypointVelocityShapingError("a coarse vertex was not emitted")
    return (
        dense,
        coarse_segments,
        waypoint_ticks,
        vertex_dense_indices,
        transition_edges,
    )


def retime_selected_waypoint_polyline(
    coarse_joint_positions_rad: Sequence[Sequence[float]],
    coarse_object_trajectory: MultiWaypointObjectTrajectory,
    grasp_model: ObjectGraspModel,
    *,
    cadence_s: Any = DEFAULT_CADENCE_S,
    desired_ramp_s: Any = DEFAULT_DESIRED_RAMP_S,
    max_dense_samples: int = DEFAULT_MAX_DENSE_RIGID_SAMPLES,
) -> WaypointVelocityShapingResult:
    """Retime an already-selected 12-joint polyline without resolving IK."""
    if not isinstance(coarse_object_trajectory, MultiWaypointObjectTrajectory):
        raise TypeError("coarse_object_trajectory must be a MultiWaypointObjectTrajectory")
    if not isinstance(grasp_model, ObjectGraspModel):
        raise TypeError("grasp_model must be an ObjectGraspModel")
    if isinstance(max_dense_samples, bool) or not isinstance(max_dense_samples, int) or max_dense_samples < 2:
        raise WaypointVelocityShapingError("max_dense_samples must be an integer >= 2")
    cadence = _finite_positive(cadence_s, "cadence_s")
    desired_ramp = _finite_positive(desired_ramp_s, "desired_ramp_s")
    coarse = tuple(
        _joint_vector(value, f"coarse_joint_positions_rad[{index}]")
        for index, value in enumerate(coarse_joint_positions_rad)
    )
    if len(coarse) != coarse_object_trajectory.sample_count:
        raise WaypointVelocityShapingError(
            "coarse joint and Object sample counts must match"
        )
    fixed_orientation = coarse_object_trajectory.fixed_object_orientation
    waypoint_poses = tuple(
        RigidTransform.from_translation_rpy(
            waypoint.translation_m,
            waypoint.rpy_rad
            if waypoint.rpy_rad is not None
            else (0.0, 0.0, 0.0),
        )
        if waypoint.rpy_rad is not None
        else RigidTransform.from_matrix((
            (*fixed_orientation.matrix[0][:3], waypoint.translation_m[0]),
            (*fixed_orientation.matrix[1][:3], waypoint.translation_m[1]),
            (*fixed_orientation.matrix[2][:3], waypoint.translation_m[2]),
            (0.0, 0.0, 0.0, 1.0),
        ))
        for waypoint in coarse_object_trajectory.waypoints
    )

    requested_ramp_ticks = _ceil_ticks(desired_ramp, cadence)
    chosen_ramp_ticks = requested_ramp_ticks
    diagnostics: dict[str, Any] | None = None
    shapes: tuple[SegmentShape, ...] = ()
    dense_joint_samples: list[DenseJointSample] = []
    coarse_segments: list[tuple[tuple[float, ...], ...]] = []
    waypoint_ticks: list[int] = []
    adaptive_iteration = 0
    adaptive_attempts: list[dict[str, Any]] = []
    for adaptive_iteration in range(1, MAX_ADAPTIVE_RAMP_ITERATIONS + 1):
        shapes = tuple(
            _schedule_coarse_edges(
                build_segment_shape(
                    duration,
                    cadence_s=cadence,
                    desired_ramp_s=chosen_ramp_ticks * cadence,
                ),
                coarse_count,
            )
            for duration, coarse_count in zip(
                coarse_object_trajectory.segment_durations_s,
                coarse_object_trajectory.segment_sample_counts,
            )
        )
        dense_count = 1 + sum(shape.effective_total_ticks for shape in shapes)
        if dense_count > max_dense_samples:
            raise WaypointVelocityShapingError(
                "dense rigid sample budget exceeded while enforcing the USER-waypoint "
                f"speed gate: {dense_count} > {max_dense_samples}"
            )
        (
            dense_joint_samples,
            coarse_segments,
            waypoint_ticks,
            coarse_vertex_dense_indices,
            transition_coarse_edges,
        ) = _build_vertex_preserving_joint_schedule(
            coarse,
            coarse_object_trajectory.segment_sample_counts,
            shapes,
        )
        diagnostics = _diagnostics(
            coarse_segments,
            coarse_object_trajectory.segment_durations_s,
            dense_joint_samples,
            waypoint_ticks,
            cadence,
            coarse_vertex_dense_indices,
            transition_coarse_edges,
        )
        diagnostics.update({
            "requested_ramp_s": desired_ramp,
            "effective_ramp_s": chosen_ramp_ticks * cadence,
            "adaptive_ramp_iteration_count": adaptive_iteration,
            "adaptive_ramp_increased": chosen_ramp_ticks > requested_ramp_ticks,
            "requested_duration_baseline_cruise_scalar_speed_per_s": [
                1.0 / duration
                for duration in coarse_object_trajectory.segment_durations_s
            ],
            "effective_segment_cruise_scalar_speed_per_s": [
                shape.cruise_scalar_speed for shape in shapes
            ],
        })
        cruise_speed_passed = all(
            shape.cruise_scalar_speed
            <= 1.0 / duration + _ROUNDING_EPSILON
            for shape, duration in zip(
                shapes, coarse_object_trajectory.segment_durations_s
            )
        )
        diagnostics["cruise_speed_not_above_requested_baseline"] = (
            cruise_speed_passed
        )
        diagnostics["quality_check_pass_conditions"][
            "cruise_speed_not_above_requested_baseline"
        ] = cruise_speed_passed
        diagnostics["quality_check_status"] = (
            "PASS"
            if all(diagnostics["quality_check_pass_conditions"].values())
            else "FAIL"
        )
        adaptive_attempts.append({
            "iteration": adaptive_iteration,
            "effective_ramp_s": chosen_ramp_ticks * cadence,
            "dense_sample_count": dense_count,
            "waypoint_speed_gate_passed": diagnostics[
                "waypoint_speed_gate_passed"
            ],
            "internal_waypoint_maxima_rad_s": [
                {
                    "waypoint_index": item["waypoint_index"],
                    "max_before_rad_s": item["max_before_rad_s"],
                    "max_after_rad_s": item["max_after_rad_s"],
                    "max_jump_rad_s": item["max_jump_rad_s"],
                }
                for item in diagnostics["internal_waypoint_adjacent_speeds"]
            ],
        })
        diagnostics["adaptive_ramp_attempts"] = list(adaptive_attempts)
        if diagnostics["waypoint_speed_gate_passed"]:
            break
        chosen_ramp_ticks = max(
            chosen_ramp_ticks + 1,
            math.ceil(chosen_ramp_ticks * 1.5),
        )
    else:
        raise WaypointVelocityShapingError(
            "USER-waypoint adjacent speed gate was not met within "
            f"{MAX_ADAPTIVE_RAMP_ITERATIONS} adaptive ramp iterations"
        )

    pending_objects: list[
        tuple[int, float, RigidTransform, RigidTransform, RigidTransform]
    ] = []
    coarse_offsets = []
    offset = 0
    for count in coarse_object_trajectory.segment_sample_counts:
        coarse_offsets.append(offset)
        offset += count - 1
    for sample in dense_joint_samples:
        segment_index = sample.user_segment_index
        segment_offset = coarse_offsets[segment_index]
        segment_count = coarse_object_trajectory.segment_sample_counts[segment_index]
        coarse_object_start = coarse_object_trajectory.samples[segment_offset]
        coarse_object_end = coarse_object_trajectory.samples[
            segment_offset + segment_count - 1
        ]
        if sample.progress == 0.0:
            object_pose = coarse_object_start.world_T_object
            left_target = coarse_object_start.world_T_left
            right_target = coarse_object_start.world_T_right
        elif sample.progress == 1.0:
            object_pose = coarse_object_end.world_T_object
            left_target = coarse_object_end.world_T_left
            right_target = coarse_object_end.world_T_right
        else:
            object_pose = _interpolate_object_pose(
                waypoint_poses[segment_index],
                waypoint_poses[segment_index + 1],
                sample.progress,
            )
            targets = grasp_model.compute_world_grasp_targets(object_pose)
            left_target = targets.world_T_left
            right_target = targets.world_T_right
        pending_objects.append((
            sample.sample_index,
            sample.time_from_start_s,
            object_pose,
            left_target,
            right_target,
        ))

    total_duration = waypoint_ticks[-1] * cadence
    object_samples = tuple(
        ObjectTrajectorySample(
            sample_index=sample_index,
            time_from_start_s=timestamp,
            alpha=timestamp / total_duration,
            world_T_object=object_pose,
            world_T_left=left_target,
            world_T_right=right_target,
        )
        for sample_index, timestamp, object_pose, left_target, right_target
        in pending_objects
    )
    if diagnostics is None:
        raise WaypointVelocityShapingError("velocity shaping did not build a schedule")
    if any(
        sample.time_from_start_s != sample.sample_index * cadence
        for sample in dense_joint_samples
    ):
        raise WaypointVelocityShapingError("dense timestamps left the cadence grid")
    if any(
        current.time_from_start_s <= previous.time_from_start_s
        for previous, current in zip(dense_joint_samples, dense_joint_samples[1:])
    ):
        raise WaypointVelocityShapingError("dense timestamps are not strictly increasing")
    if not diagnostics["dense_peak_speed_not_above_coarse"]:
        raise WaypointVelocityShapingError(
            "dense joint speed exceeded the corresponding coarse baseline peak"
        )
    if diagnostics["repeated_transition_on_moving_coarse_segment_count"]:
        raise WaypointVelocityShapingError(
            "dense shaping produced a repeated 12-joint transition on a moving segment"
        )
    if not diagnostics["cadence_grid_passed"]:
        raise WaypointVelocityShapingError("dense timestamps left the cadence grid")
    if not diagnostics["strict_timestamps_passed"]:
        raise WaypointVelocityShapingError(
            "dense timestamps are not strictly increasing"
        )
    if not diagnostics["coarse_polyline_preserved"]:
        raise WaypointVelocityShapingError("dense shaping did not preserve the coarse polyline")
    if not all(
        diagnostics[key]
        for key in (
            "discrete_velocity_finite",
            "discrete_acceleration_finite",
            "discrete_jerk_finite",
        )
    ):
        raise WaypointVelocityShapingError("dense derivative diagnostics are not finite")
    if not diagnostics["waypoint_speed_gate_passed"]:
        raise WaypointVelocityShapingError("USER-waypoint adjacent speed gate failed")
    if diagnostics["quality_check_status"] != "PASS":
        raise WaypointVelocityShapingError(
            "dense trajectory did not pass every required shaping diagnostic"
        )
    return WaypointVelocityShapingResult(
        joint_samples=tuple(dense_joint_samples),
        object_samples=object_samples,
        segment_shapes=shapes,
        waypoint_ticks=tuple(waypoint_ticks),
        diagnostics=diagnostics,
    )
