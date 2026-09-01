"""Translation-only object-centric Cartesian trajectory core.

This module uses the Phase 1E transform convention without redefining it:
``^A T_B`` is frame B expressed in frame A. For sample index ``i`` from zero
through ``N - 1``, where ``N`` is the sample count, the generator applies:

``alpha_i = i / (N - 1)`` (unitless path fraction),
``t_i = alpha_i * T`` (seconds from start for duration ``T``), and
``p_i = (1 - alpha_i) * p0 + alpha_i * p1`` (meters in the world frame).

Only translation is interpolated. Every ``^W T_O(i)`` copies the start
object's 3x3 rotation matrix directly. Left and right Cartesian planning-frame
targets are derived, never independently interpolated:
``^W T_L(i) = ^W T_O(i) * ^O T_L`` and
``^W T_R(i) = ^W T_O(i) * ^O T_R``.

PHASE 1F.1 OUTPUT IS NOT A ROBOT JOINT TRAJECTORY. It is an object Cartesian
trajectory plus derived left/right Cartesian grasp targets. It proves neither
IK feasibility, joint continuity/limits, collision freedom, velocity or
acceleration feasibility, hardware synchronization, physical grasp validity,
nor executable robot motion.

Phase 3A adds planning-only :class:`ObjectWaypoint` values and deterministic
multi-segment interpolation. Its timeline is for planning/preview only; it is
not dynamically executable timing and has no velocity or acceleration claim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

from dual_arm_app.backend.object_grasp_model import (
    DEFAULT_COMPARISON_TOLERANCE,
    SYNTHETIC_OBJECT_GRASP_FIXTURE,
    ObjectGraspModel,
    RigidTransform,
    compose,
    inverse,
)


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be finite")
    return converted


def _translation3(values: Any, label: str) -> Vector3:
    if (
        isinstance(values, (str, bytes, bool))
        or not isinstance(values, Sequence)
        or len(values) != 3
    ):
        raise TypeError(f"{label} must contain exactly three finite meter values")
    return tuple(
        _finite_number(value, f"{label}[{index}]")
        for index, value in enumerate(values)
    )  # type: ignore[return-value]


def _comparison_tolerance(value: Any) -> float:
    tolerance = _finite_number(value, "tolerance")
    if tolerance < 0:
        raise ValueError("tolerance must be non-negative")
    return tolerance


def _rotation_is_close(
    left: RigidTransform,
    right: RigidTransform,
    tolerance: float,
) -> bool:
    left_matrix = left.matrix
    right_matrix = right.matrix
    return all(
        abs(left_matrix[row][column] - right_matrix[row][column]) <= tolerance
        for row in range(3)
        for column in range(3)
    )


def _rotation_quaternion_xyzw(source: RigidTransform) -> Quaternion:
    m = source.matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        x = (m[2][1] - m[1][2]) / scale
        y = (m[0][2] - m[2][0]) / scale
        z = (m[1][0] - m[0][1]) / scale
        w = 0.25 * scale
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        x = 0.25 * scale
        y = (m[0][1] + m[1][0]) / scale
        z = (m[0][2] + m[2][0]) / scale
        w = (m[2][1] - m[1][2]) / scale
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        x = (m[0][1] + m[1][0]) / scale
        y = 0.25 * scale
        z = (m[1][2] + m[2][1]) / scale
        w = (m[0][2] - m[2][0]) / scale
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        x = (m[0][2] + m[2][0]) / scale
        y = (m[1][2] + m[2][1]) / scale
        z = 0.25 * scale
        w = (m[1][0] - m[0][1]) / scale
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/norm, y/norm, z/norm, w/norm)


def _quaternion_transform(quaternion: Quaternion, translation_m: Vector3) -> RigidTransform:
    x, y, z, w = quaternion
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return RigidTransform.from_matrix((
        (1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy), translation_m[0]),
        (2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx), translation_m[1]),
        (2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy), translation_m[2]),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _slerp_rotation(start: RigidTransform, end: RigidTransform, alpha: float, translation_m: Vector3) -> RigidTransform:
    if _rotation_is_close(start, end, 1e-15):
        return _with_translation(start, translation_m)
    q0 = _rotation_quaternion_xyzw(start)
    q1 = _rotation_quaternion_xyzw(end)
    dot = sum(a*b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-v for v in q1)  # shortest rotational arc
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        q = tuple((1.0-alpha)*a + alpha*b for a, b in zip(q0, q1))
        norm = math.sqrt(sum(v*v for v in q))
        q = tuple(v/norm for v in q)
    else:
        theta0 = math.acos(dot)
        sin_theta0 = math.sin(theta0)
        theta = theta0 * alpha
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta0
        s1 = math.sin(theta) / sin_theta0
        q = tuple(s0*a + s1*b for a, b in zip(q0, q1))
    return _quaternion_transform(q, translation_m)  # type: ignore[arg-type]


def _with_translation(
    source: RigidTransform,
    translation_m: Vector3,
) -> RigidTransform:
    """Copy ``source`` rotation directly and replace only translation in meters."""
    matrix = source.matrix
    return RigidTransform.from_matrix(
        (
            (*matrix[0][:3], translation_m[0]),
            (*matrix[1][:3], translation_m[1]),
            (*matrix[2][:3], translation_m[2]),
            (0.0, 0.0, 0.0, 1.0),
        )
    )


@dataclass(frozen=True)
class ObjectTrajectorySample:
    """One immutable object pose and its derived Cartesian grasp targets.

    ``time_from_start_s`` is seconds and ``alpha`` is unitless in ``[0, 1]``.
    All transforms use the existing ``^A T_B`` convention and meter units.
    """

    sample_index: int
    time_from_start_s: float
    alpha: float
    world_T_object: RigidTransform
    world_T_left: RigidTransform
    world_T_right: RigidTransform

    def __post_init__(self) -> None:
        if (
            isinstance(self.sample_index, bool)
            or not isinstance(self.sample_index, int)
            or self.sample_index < 0
        ):
            raise ValueError("sample_index must be a non-negative integer")
        time_s = _finite_number(self.time_from_start_s, "time_from_start_s")
        alpha = _finite_number(self.alpha, "alpha")
        if time_s < 0:
            raise ValueError("time_from_start_s must be non-negative")
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be within [0, 1]")
        for label, transform in (
            ("world_T_object", self.world_T_object),
            ("world_T_left", self.world_T_left),
            ("world_T_right", self.world_T_right),
        ):
            if not isinstance(transform, RigidTransform):
                raise TypeError(f"{label} must be a RigidTransform")
        object.__setattr__(self, "time_from_start_s", time_s)
        object.__setattr__(self, "alpha", alpha)


@dataclass(frozen=True)
class ObjectTrajectory:
    """Immutable translation-only object and Cartesian grasp-target path."""

    name: str
    duration_s: float
    samples: tuple[ObjectTrajectorySample, ...]
    translation_only: bool = field(default=True, init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.name, str):
            raise TypeError("trajectory name must be a non-empty string")
        checked_name = self.name.strip()
        if not checked_name:
            raise ValueError("trajectory name must be a non-empty string")
        duration_s = _finite_number(self.duration_s, "duration_s")
        if duration_s <= 0:
            raise ValueError("duration_s must be greater than zero")
        try:
            samples = tuple(self.samples)
        except TypeError as error:
            raise TypeError("samples must be an iterable of ObjectTrajectorySample") from error
        if len(samples) < 2:
            raise ValueError("trajectory must contain at least two samples")
        if not all(isinstance(sample, ObjectTrajectorySample) for sample in samples):
            raise TypeError("samples must contain only ObjectTrajectorySample values")
        if not math.isclose(samples[0].time_from_start_s, 0.0, abs_tol=1e-12):
            raise ValueError("first sample time must be zero")
        if not math.isclose(
            samples[-1].time_from_start_s,
            duration_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("last sample time must equal duration_s")
        if any(
            current.time_from_start_s >= following.time_from_start_s
            for current, following in zip(samples, samples[1:])
        ):
            raise ValueError("sample times must be strictly increasing")
        if tuple(sample.sample_index for sample in samples) != tuple(range(len(samples))):
            raise ValueError("sample_index values must be chronological from zero")
        object.__setattr__(self, "name", checked_name)
        object.__setattr__(self, "duration_s", duration_s)
        object.__setattr__(self, "samples", samples)

    @property
    def sample_count(self) -> int:
        """Return the exact number of endpoint-inclusive samples."""
        return len(self.samples)

    def has_constant_object_orientation(
        self,
        reference: RigidTransform | None = None,
        tolerance: float = DEFAULT_COMPARISON_TOLERANCE,
    ) -> bool:
        """Check that every object rotation matches ``reference`` within tolerance."""
        if reference is not None and not isinstance(reference, RigidTransform):
            raise TypeError("reference must be a RigidTransform or None")
        checked_tolerance = _comparison_tolerance(tolerance)
        expected = reference or self.samples[0].world_T_object
        return all(
            _rotation_is_close(
                sample.world_T_object,
                expected,
                checked_tolerance,
            )
            for sample in self.samples
        )


@dataclass(frozen=True)
class ObjectWaypoint:
    """One immutable object-space planning waypoint.

    ``translation_m`` is a world-frame translation in meters. ``rpy_rad`` is
    optional for backward compatibility; when omitted the containing trajectory
    fallback orientation is used. Waypoints contain no joint/execution semantics.
    """

    identifier: str
    translation_m: Vector3
    rpy_rad: Vector3 | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.identifier, str):
            raise TypeError("waypoint identifier must be a non-empty string")
        identifier = self.identifier.strip()
        if not identifier:
            raise ValueError("waypoint identifier must be a non-empty string")
        if not identifier[0].isalnum() or any(
            not (character.isalnum() or character in "._-")
            for character in identifier
        ):
            raise ValueError(
                "waypoint identifier must start with an alphanumeric character "
                "and contain only alphanumeric, '.', '_', or '-' characters"
            )
        object.__setattr__(self, "identifier", identifier)
        object.__setattr__(
            self,
            "translation_m",
            _translation3(self.translation_m, "translation_m"),
        )
        if self.rpy_rad is not None:
            object.__setattr__(self, "rpy_rad", _translation3(self.rpy_rad, "rpy_rad"))


@dataclass(frozen=True)
class MultiWaypointObjectTrajectory(ObjectTrajectory):
    """Immutable 6D Center timeline through ordered object waypoints."""

    translation_only: bool = field(default=False, init=False)
    waypoints: tuple[ObjectWaypoint, ...]
    fixed_object_orientation: RigidTransform
    segment_durations_s: tuple[float, ...]
    segment_sample_counts: tuple[int, ...]
    timing_semantic: str = field(
        default="PLANNING/PREVIEW TIMING — NOT DYNAMICALLY EXECUTABLE",
        init=False,
    )

    def __post_init__(self) -> None:
        super().__post_init__()
        waypoints = tuple(self.waypoints)
        if len(waypoints) < 2:
            raise ValueError("multi-waypoint trajectory requires at least 2 waypoints")
        if any(not isinstance(item, ObjectWaypoint) for item in waypoints):
            raise TypeError("waypoints must contain only ObjectWaypoint values")
        identifiers = tuple(item.identifier.casefold() for item in waypoints)
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("waypoint identifiers must be unique")
        if not isinstance(self.fixed_object_orientation, RigidTransform):
            raise TypeError("fixed_object_orientation must be a RigidTransform")
        durations = tuple(self.segment_durations_s)
        counts = tuple(self.segment_sample_counts)
        if len(durations) != len(waypoints) - 1:
            raise ValueError("segment_durations_s must have one value per segment")
        if len(counts) != len(waypoints) - 1:
            raise ValueError("segment_sample_counts must have one value per segment")
        checked_durations = tuple(
            _finite_number(value, f"segment_durations_s[{index}]")
            for index, value in enumerate(durations)
        )
        if any(value <= 0.0 for value in checked_durations):
            raise ValueError("every segment duration must be greater than zero")
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 2
            for value in counts
        ):
            raise ValueError("every segment sample count must be an integer >= 2")
        expected_sample_count = 1 + sum(value - 1 for value in counts)
        if len(self.samples) != expected_sample_count:
            raise ValueError("samples do not match endpoint-deduplicated segment counts")
        object.__setattr__(self, "waypoints", waypoints)
        object.__setattr__(self, "segment_durations_s", checked_durations)
        object.__setattr__(self, "segment_sample_counts", counts)


def generate_translation_only_object_trajectory(
    name: str,
    start_world_T_object: RigidTransform,
    end_translation_m: Sequence[float],
    duration_s: float,
    sample_count: int,
    grasp_model: ObjectGraspModel,
) -> ObjectTrajectory:
    """Generate endpoint-inclusive object and derived grasp-frame targets.

    For each ``i = 0 .. N-1``, ``alpha_i = i/(N-1)`` and
    ``t_i = alpha_i*T`` seconds. World translation in meters is
    ``p_i = (1-alpha_i)*p0 + alpha_i*p1``. The start rotation is copied without
    RPY conversion. Assumptions are constant object orientation and fixed
    ``^O T_L``/``^O T_R``. Output is Cartesian planning data only; it provides
    none of the robot feasibility or execution guarantees listed above.
    """
    if not isinstance(name, str):
        raise TypeError("trajectory name must be a non-empty string")
    checked_name = name.strip()
    if not checked_name:
        raise ValueError("trajectory name must be a non-empty string")
    if not isinstance(start_world_T_object, RigidTransform):
        raise TypeError("start_world_T_object must be a RigidTransform")
    if not isinstance(grasp_model, ObjectGraspModel):
        raise TypeError("grasp_model must be an ObjectGraspModel")
    checked_duration_s = _finite_number(duration_s, "duration_s")
    if checked_duration_s <= 0:
        raise ValueError("duration_s must be greater than zero")
    if isinstance(sample_count, bool) or not isinstance(sample_count, int):
        raise TypeError("sample_count must be an integer")
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2")
    end_translation = _translation3(end_translation_m, "end_translation_m")

    start_translation = start_world_T_object.translation_m
    expected_left_T_right = grasp_model.left_T_right()
    samples: list[ObjectTrajectorySample] = []
    for index in range(sample_count):
        # index is 0..N-1; alpha is unitless and time is seconds from start.
        alpha = index / (sample_count - 1)
        time_from_start_s = alpha * checked_duration_s
        # p0/p1 and the result are world-frame translations measured in meters.
        translation = tuple(
            (1.0 - alpha) * start_translation[axis]
            + alpha * end_translation[axis]
            for axis in range(3)
        )
        world_T_object = _with_translation(
            start_world_T_object,
            translation,  # type: ignore[arg-type]
        )
        targets = grasp_model.compute_world_grasp_targets(world_T_object)

        # Both targets must remain derived from the same object pose and fixed
        # object-relative fixture; they are never interpolated independently.
        actual_left_T_right = compose(
            inverse(targets.world_T_left),
            targets.world_T_right,
        )
        if not actual_left_T_right.almost_equal(
            expected_left_T_right,
            DEFAULT_COMPARISON_TOLERANCE,
        ):
            raise RuntimeError("derived grasp targets violate rigid-grasp invariance")

        samples.append(
            ObjectTrajectorySample(
                sample_index=index,
                time_from_start_s=time_from_start_s,
                alpha=alpha,
                world_T_object=world_T_object,
                world_T_left=targets.world_T_left,
                world_T_right=targets.world_T_right,
            )
        )

    trajectory = ObjectTrajectory(
        name=checked_name,
        duration_s=checked_duration_s,
        samples=tuple(samples),
    )
    if not trajectory.has_constant_object_orientation(start_world_T_object):
        raise RuntimeError("generated object orientation changed along translation-only path")
    return trajectory


def generate_multi_waypoint_object_trajectory(
    *,
    name: str,
    waypoints: Sequence[ObjectWaypoint],
    fixed_object_orientation: RigidTransform,
    segment_durations_s: Sequence[float],
    segment_sample_counts: Sequence[int],
    grasp_model: ObjectGraspModel,
) -> MultiWaypointObjectTrajectory:
    """Generate one endpoint-deduplicated 6D Center timeline for ``W0..Wn``.

    Translation is linearly interpolated and orientation uses shortest-arc quaternion
    SLERP between per-waypoint RPY poses (or the legacy fallback orientation).
    Each segment is endpoint-inclusive locally, but the first sample of every
    segment after segment zero is omitted. Therefore intermediate waypoint
    boundary poses and their exact cumulative times appear once. Object pose and
    both TCP targets share each sample index/time. Left/right targets are always
    derived from the same object pose through ``ObjectGraspModel``.
    """
    if not isinstance(name, str):
        raise TypeError("trajectory name must be a non-empty string")
    if not name.strip():
        raise ValueError("trajectory name must be a non-empty string")
    try:
        checked_waypoints = tuple(waypoints)
    except TypeError as error:
        raise TypeError("waypoints must be an iterable of ObjectWaypoint values") from error
    if len(checked_waypoints) < 2:
        raise ValueError("multi-waypoint trajectory requires at least 2 waypoints")
    if any(not isinstance(item, ObjectWaypoint) for item in checked_waypoints):
        raise TypeError("waypoints must contain only ObjectWaypoint values")
    identifiers = tuple(item.identifier.casefold() for item in checked_waypoints)
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("waypoint identifiers must be unique")
    if not isinstance(fixed_object_orientation, RigidTransform):
        raise TypeError("fixed_object_orientation must be a RigidTransform")
    if not isinstance(grasp_model, ObjectGraspModel):
        raise TypeError("grasp_model must be an ObjectGraspModel")
    try:
        durations = tuple(segment_durations_s)
        counts = tuple(segment_sample_counts)
    except TypeError as error:
        raise TypeError("segment timing and sample counts must be iterable") from error
    if len(durations) != len(checked_waypoints) - 1:
        raise ValueError("segment_durations_s must have one value per segment")
    if len(counts) != len(checked_waypoints) - 1:
        raise ValueError("segment_sample_counts must have one value per segment")
    checked_durations = tuple(
        _finite_number(value, f"segment_durations_s[{index}]")
        for index, value in enumerate(durations)
    )
    if any(value <= 0.0 for value in checked_durations):
        raise ValueError("every segment duration must be greater than zero")
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 2
        for value in counts
    ):
        raise ValueError("every segment sample count must be an integer >= 2")

    total_duration = sum(checked_durations)
    fixed_orientation = _with_translation(fixed_object_orientation, (0.0, 0.0, 0.0))
    expected_left_T_right = grasp_model.left_T_right()
    samples: list[ObjectTrajectorySample] = []
    elapsed_s = 0.0
    for segment_index, (duration_s, sample_count) in enumerate(
        zip(checked_durations, counts)
    ):
        start_waypoint = checked_waypoints[segment_index]
        end_waypoint = checked_waypoints[segment_index + 1]
        start_translation = start_waypoint.translation_m
        end_translation = end_waypoint.translation_m
        start_orientation = (
            RigidTransform.from_translation_rpy((0.0, 0.0, 0.0), start_waypoint.rpy_rad)
            if start_waypoint.rpy_rad is not None else fixed_orientation
        )
        end_orientation = (
            RigidTransform.from_translation_rpy((0.0, 0.0, 0.0), end_waypoint.rpy_rad)
            if end_waypoint.rpy_rad is not None else fixed_orientation
        )
        first_local_index = 0 if segment_index == 0 else 1
        for local_index in range(first_local_index, sample_count):
            local_alpha = local_index / (sample_count - 1)
            time_from_start_s = elapsed_s + local_alpha * duration_s
            if local_index == 0:
                translation = start_translation
            elif local_index == sample_count - 1:
                translation = end_translation
                time_from_start_s = elapsed_s + duration_s
            else:
                translation = tuple(
                    (1.0 - local_alpha) * start_translation[axis]
                    + local_alpha * end_translation[axis]
                    for axis in range(3)
                )
            world_T_object = _slerp_rotation(
                start_orientation, end_orientation, local_alpha, translation  # type: ignore[arg-type]
            )
            targets = grasp_model.compute_world_grasp_targets(world_T_object)
            actual_left_T_right = compose(
                inverse(targets.world_T_left),
                targets.world_T_right,
            )
            if not actual_left_T_right.almost_equal(
                expected_left_T_right,
                DEFAULT_COMPARISON_TOLERANCE,
            ):
                raise RuntimeError("derived grasp targets violate rigid-grasp invariance")
            samples.append(ObjectTrajectorySample(
                sample_index=len(samples),
                time_from_start_s=time_from_start_s,
                alpha=time_from_start_s / total_duration,
                world_T_object=world_T_object,
                world_T_left=targets.world_T_left,
                world_T_right=targets.world_T_right,
            ))
        elapsed_s += duration_s

    return MultiWaypointObjectTrajectory(
        name=name.strip(),
        duration_s=total_duration,
        samples=tuple(samples),
        waypoints=checked_waypoints,
        fixed_object_orientation=fixed_orientation,
        segment_durations_s=checked_durations,
        segment_sample_counts=counts,
    )


SYNTHETIC_OBJECT_TRAJECTORY_NOTICE = (
    "OFFLINE SYNTHETIC OBJECT TRAJECTORY — NOT FOR ROBOT EXECUTION"
)
SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY = (
    generate_translation_only_object_trajectory(
        name=SYNTHETIC_OBJECT_TRAJECTORY_NOTICE,
        start_world_T_object=RigidTransform.from_translation_rpy(
            translation_m=(0.0, 0.0, 0.8),
            rpy_rad=(0.1, -0.2, 0.3),
        ),
        end_translation_m=(0.4, 0.0, 1.0),
        duration_s=4.0,
        sample_count=5,
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
    )
)
