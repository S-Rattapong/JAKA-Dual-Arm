"""Translation-only object-centric Cartesian trajectory core for Phase 1F.1.

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

    time_from_start_s: float
    alpha: float
    world_T_object: RigidTransform
    world_T_left: RigidTransform
    world_T_right: RigidTransform

    def __post_init__(self) -> None:
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
