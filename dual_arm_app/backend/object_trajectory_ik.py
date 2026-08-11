"""ROS-independent sequential dual-arm IK bookkeeping for Phase 1G.2A.

OFFLINE PLANNING ONLY — NO PHYSICAL ROBOT, NO JAKA DRIVER, NO MOTION.

For object-trajectory sample ``i``, independent left/right IK candidates are
requested for the already-derived ``^W T_L(i)`` and ``^W T_R(i)`` targets. The
pair is ordered as ``q(i) = [q_L1..q_L6, q_R1..q_R6]`` and accepted only after
combined dual-arm state validation. Sample zero uses an explicit 12-joint seed;
later samples use only the previous accepted pair. Joint changes are recorded
as ``Delta q_j(i) = q_j(i) - q_j(i-1)`` in radians. This phase records jumps but
does not reject them, optimize them, or produce an executable trajectory.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Protocol, Sequence

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory import ObjectTrajectory


LEFT_JOINT_ORDER = tuple(f"left_joint_{index}" for index in range(1, 7))
RIGHT_JOINT_ORDER = tuple(f"right_joint_{index}" for index in range(1, 7))
DUAL_ARM_JOINT_ORDER = (*LEFT_JOINT_ORDER, *RIGHT_JOINT_ORDER)

LEFT_GROUP_NAME = "left_arm"
RIGHT_GROUP_NAME = "right_arm"
DUAL_ARM_GROUP_NAME = "dual_arm"
LEFT_IK_LINK_NAME = "left_J6"
RIGHT_IK_LINK_NAME = "right_J6"

DEFAULT_IK_TIMEOUT_S = 1.0
OFFLINE_MODEL_SEED_NOTICE = (
    "OFFLINE SYNTHETIC / MODEL SEED — NOT PHYSICAL ROBOT CALIBRATION"
)
DEFAULT_INITIAL_DUAL_ARM_SEED_RAD = (
    3.1399999999999997,
    0.5187280000000003,
    -0.8000720000000001,
    0.0,
    0.7988159999999995,
    0.0,
    0.0,
    2.6162480000000015,
    0.7988159999999995,
    0.0,
    2.3399279999999996,
    0.0,
)

JointVector6 = tuple[float, float, float, float, float, float]
JointVector12 = tuple[
    float, float, float, float, float, float,
    float, float, float, float, float, float,
]
QuaternionXyzw = tuple[float, float, float, float]


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be finite")
    return converted


def _joint_vector(values: Any, expected_size: int, label: str) -> tuple[float, ...]:
    if (
        isinstance(values, (str, bytes, bool))
        or not isinstance(values, Sequence)
        or len(values) != expected_size
    ):
        raise ValueError(f"{label} must contain exactly {expected_size} joint values")
    return tuple(
        _finite_number(value, f"{label}[{index}]")
        for index, value in enumerate(values)
    )


def combine_arm_joint_solutions(
    left_joint_positions_rad: Sequence[float],
    right_joint_positions_rad: Sequence[float],
) -> JointVector12:
    """Return canonical ``[left 1..6, right 1..6]`` positions in radians."""
    left = _joint_vector(left_joint_positions_rad, 6, "left_joint_positions_rad")
    right = _joint_vector(right_joint_positions_rad, 6, "right_joint_positions_rad")
    return (*left, *right)  # type: ignore[return-value]


def joint_delta_rad(
    current_joint_positions_rad: Sequence[float],
    previous_joint_positions_rad: Sequence[float],
) -> JointVector12:
    """Return ``Delta q_j = q_j(current) - q_j(previous)`` in radians."""
    current = _joint_vector(current_joint_positions_rad, 12, "current_joint_positions_rad")
    previous = _joint_vector(
        previous_joint_positions_rad,
        12,
        "previous_joint_positions_rad",
    )
    return tuple(
        current[index] - previous[index]
        for index in range(12)
    )  # type: ignore[return-value]


def max_abs_joint_step_rad(delta_rad: Sequence[float]) -> float:
    """Return ``max_j |Delta q_j|`` in radians without applying a threshold."""
    checked = _joint_vector(delta_rad, 12, "joint_delta_rad")
    return max(abs(value) for value in checked)


def rotation_matrix_to_quaternion_xyzw(transform: RigidTransform) -> QuaternionXyzw:
    """Convert a proper rotation to a normalized deterministic ROS ``x,y,z,w``.

    The standard trace/major-diagonal branches avoid numerical cancellation.
    Quaternion sign is canonicalized to non-negative ``w`` (then the first
    non-zero vector component when ``w`` is zero), because ``q`` and ``-q`` are
    the same orientation.
    """
    if not isinstance(transform, RigidTransform):
        raise TypeError("transform must be a RigidTransform")
    matrix = transform.matrix
    m00, m01, m02 = matrix[0][:3]
    m10, m11, m12 = matrix[1][:3]
    m20, m21, m22 = matrix[2][:3]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) * 2.0
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
        w = (m21 - m12) / scale
    elif m11 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m11 - m00 - m22)) * 2.0
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
        w = (m02 - m20) / scale
    else:
        scale = math.sqrt(max(0.0, 1.0 + m22 - m00 - m11)) * 2.0
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
        w = (m10 - m01) / scale

    quaternion = (x, y, z, w)
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError("rotation produced a non-finite quaternion")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-15:
        raise ValueError("rotation produced a degenerate quaternion")
    normalized = tuple(value / norm for value in quaternion)
    sign_reference = normalized[3]
    if abs(sign_reference) <= 1e-15:
        sign_reference = next(
            (value for value in normalized[:3] if abs(value) > 1e-15),
            1.0,
        )
    if sign_reference < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


@dataclass(frozen=True)
class ArmIkSolution:
    """One planning-only six-joint IK response."""

    success: bool
    joint_positions_rad: JointVector6 | None = None
    diagnostic: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.success, bool):
            raise TypeError("success must be boolean")
        if self.success and self.joint_positions_rad is None:
            raise ValueError("successful IK must provide six joint positions")
        if self.joint_positions_rad is not None:
            checked = _joint_vector(
                self.joint_positions_rad,
                6,
                "joint_positions_rad",
            )
            object.__setattr__(self, "joint_positions_rad", checked)
        if not isinstance(self.diagnostic, str):
            raise TypeError("diagnostic must be a string")


@dataclass(frozen=True)
class CombinedStateValidity:
    """Planning-only combined 12-joint state validity response."""

    valid: bool
    diagnostic: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.valid, bool):
            raise TypeError("valid must be boolean")
        if not isinstance(self.diagnostic, str):
            raise TypeError("diagnostic must be a string")


class ObjectTrajectoryIkAdapter(Protocol):
    """Boundary implemented by a planning-only MoveIt service adapter or fake."""

    def solve_arm_ik(
        self,
        *,
        group_name: str,
        ik_link_name: str,
        target_world_T_tip: RigidTransform,
        seed_joint_positions_rad: JointVector12,
        timeout_s: float,
        avoid_collisions: bool,
    ) -> ArmIkSolution: ...

    def check_combined_state(
        self,
        *,
        joint_positions_rad: JointVector12,
        group_name: str,
    ) -> CombinedStateValidity: ...


@dataclass(frozen=True)
class ObjectTrajectoryIkSampleResult:
    sample_index: int
    time_from_start_s: float
    alpha: float
    left_ik_success: bool
    right_ik_success: bool
    combined_valid: bool
    accepted: bool
    left_joint_positions_rad: JointVector6 | None
    right_joint_positions_rad: JointVector6 | None
    combined_joint_positions_rad: JointVector12 | None
    joint_delta_from_previous_rad: JointVector12 | None
    max_abs_joint_step_rad: float | None
    failure_reason: str | None
    diagnostic_message: str


@dataclass(frozen=True)
class ObjectTrajectoryIkResult:
    trajectory_name: str
    requested_sample_count: int
    processed_sample_count: int
    accepted_sample_count: int
    completed: bool
    failed_sample_index: int | None
    samples: tuple[ObjectTrajectoryIkSampleResult, ...]

    @property
    def maximum_observed_joint_step_rad(self) -> float | None:
        steps = tuple(
            sample.max_abs_joint_step_rad
            for sample in self.samples
            if sample.max_abs_joint_step_rad is not None
        )
        return max(steps) if steps else None


def _failure_sample(
    *,
    sample_index: int,
    time_from_start_s: float,
    alpha: float,
    left_success: bool,
    right_success: bool,
    combined_valid: bool,
    left_positions: JointVector6 | None,
    right_positions: JointVector6 | None,
    combined_positions: JointVector12 | None,
    reason: str,
    diagnostic: str,
) -> ObjectTrajectoryIkSampleResult:
    return ObjectTrajectoryIkSampleResult(
        sample_index=sample_index,
        time_from_start_s=time_from_start_s,
        alpha=alpha,
        left_ik_success=left_success,
        right_ik_success=right_success,
        combined_valid=combined_valid,
        accepted=False,
        left_joint_positions_rad=left_positions,
        right_joint_positions_rad=right_positions,
        combined_joint_positions_rad=combined_positions,
        joint_delta_from_previous_rad=None,
        max_abs_joint_step_rad=None,
        failure_reason=reason,
        diagnostic_message=diagnostic,
    )


def solve_sequential_object_trajectory_ik(
    trajectory: ObjectTrajectory,
    adapter: ObjectTrajectoryIkAdapter,
    *,
    initial_seed_joint_positions_rad: Sequence[float] = (
        DEFAULT_INITIAL_DUAL_ARM_SEED_RAD
    ),
    ik_timeout_s: float = DEFAULT_IK_TIMEOUT_S,
) -> ObjectTrajectoryIkResult:
    """Solve ordered samples with previous-accepted-state seed propagation.

    Both independent arm requests at sample ``i`` receive the same pair seed.
    The newly solved left candidate is deliberately not inserted into the right
    request seed. Failed or combined-invalid candidates stop processing and are
    never propagated. ``avoid_collisions`` is explicitly false; combined state
    validity is the acceptance gate. Joint step values are recorded only and no
    jump threshold is applied in Phase 1G.2A.
    """
    if not isinstance(trajectory, ObjectTrajectory):
        raise TypeError("trajectory must be an ObjectTrajectory")
    if not callable(getattr(adapter, "solve_arm_ik", None)):
        raise TypeError("adapter must provide solve_arm_ik")
    if not callable(getattr(adapter, "check_combined_state", None)):
        raise TypeError("adapter must provide check_combined_state")
    pair_seed = _joint_vector(
        initial_seed_joint_positions_rad,
        12,
        "initial_seed_joint_positions_rad",
    )
    timeout_s = _finite_number(ik_timeout_s, "ik_timeout_s")
    if timeout_s <= 0:
        raise ValueError("ik_timeout_s must be greater than zero")

    results: list[ObjectTrajectoryIkSampleResult] = []
    previous_accepted: JointVector12 | None = None
    failed_sample_index: int | None = None

    for sample_index, sample in enumerate(trajectory.samples):
        try:
            left = adapter.solve_arm_ik(
                group_name=LEFT_GROUP_NAME,
                ik_link_name=LEFT_IK_LINK_NAME,
                target_world_T_tip=sample.world_T_left,
                seed_joint_positions_rad=pair_seed,  # type: ignore[arg-type]
                timeout_s=timeout_s,
                avoid_collisions=False,
            )
        except Exception as error:  # adapter/runtime boundary becomes a result
            left = ArmIkSolution(False, diagnostic=f"Left IK exception: {error}")
        if not isinstance(left, ArmIkSolution):
            left = ArmIkSolution(False, diagnostic="Left IK adapter returned invalid result")
        if not left.success:
            failed_sample_index = sample_index
            results.append(_failure_sample(
                sample_index=sample_index,
                time_from_start_s=sample.time_from_start_s,
                alpha=sample.alpha,
                left_success=False,
                right_success=False,
                combined_valid=False,
                left_positions=left.joint_positions_rad,
                right_positions=None,
                combined_positions=None,
                reason="LEFT_IK_FAILED",
                diagnostic=left.diagnostic or "Left IK failed",
            ))
            break

        try:
            right = adapter.solve_arm_ik(
                group_name=RIGHT_GROUP_NAME,
                ik_link_name=RIGHT_IK_LINK_NAME,
                target_world_T_tip=sample.world_T_right,
                seed_joint_positions_rad=pair_seed,  # type: ignore[arg-type]
                timeout_s=timeout_s,
                avoid_collisions=False,
            )
        except Exception as error:
            right = ArmIkSolution(False, diagnostic=f"Right IK exception: {error}")
        if not isinstance(right, ArmIkSolution):
            right = ArmIkSolution(False, diagnostic="Right IK adapter returned invalid result")
        if not right.success:
            failed_sample_index = sample_index
            results.append(_failure_sample(
                sample_index=sample_index,
                time_from_start_s=sample.time_from_start_s,
                alpha=sample.alpha,
                left_success=True,
                right_success=False,
                combined_valid=False,
                left_positions=left.joint_positions_rad,
                right_positions=right.joint_positions_rad,
                combined_positions=None,
                reason="RIGHT_IK_FAILED",
                diagnostic=right.diagnostic or "Right IK failed",
            ))
            break

        combined = combine_arm_joint_solutions(
            left.joint_positions_rad,  # type: ignore[arg-type]
            right.joint_positions_rad,  # type: ignore[arg-type]
        )
        try:
            validity = adapter.check_combined_state(
                joint_positions_rad=combined,
                group_name=DUAL_ARM_GROUP_NAME,
            )
        except Exception as error:
            validity = CombinedStateValidity(
                False,
                diagnostic=f"Combined validity exception: {error}",
            )
        if not isinstance(validity, CombinedStateValidity):
            validity = CombinedStateValidity(
                False,
                diagnostic="State-validity adapter returned invalid result",
            )
        if not validity.valid:
            failed_sample_index = sample_index
            results.append(_failure_sample(
                sample_index=sample_index,
                time_from_start_s=sample.time_from_start_s,
                alpha=sample.alpha,
                left_success=True,
                right_success=True,
                combined_valid=False,
                left_positions=left.joint_positions_rad,
                right_positions=right.joint_positions_rad,
                combined_positions=combined,
                reason="COMBINED_STATE_INVALID",
                diagnostic=validity.diagnostic or "Combined dual-arm state is invalid",
            ))
            break

        delta = (
            joint_delta_rad(combined, previous_accepted)
            if previous_accepted is not None
            else None
        )
        maximum_step = max_abs_joint_step_rad(delta) if delta is not None else None
        results.append(ObjectTrajectoryIkSampleResult(
            sample_index=sample_index,
            time_from_start_s=sample.time_from_start_s,
            alpha=sample.alpha,
            left_ik_success=True,
            right_ik_success=True,
            combined_valid=True,
            accepted=True,
            left_joint_positions_rad=left.joint_positions_rad,
            right_joint_positions_rad=right.joint_positions_rad,
            combined_joint_positions_rad=combined,
            joint_delta_from_previous_rad=delta,
            max_abs_joint_step_rad=maximum_step,
            failure_reason=None,
            diagnostic_message="ACCEPTED — PLANNING MODEL STATE VALID",
        ))
        previous_accepted = combined
        pair_seed = combined

    accepted_count = sum(1 for result in results if result.accepted)
    completed = failed_sample_index is None and len(results) == trajectory.sample_count
    return ObjectTrajectoryIkResult(
        trajectory_name=trajectory.name,
        requested_sample_count=trajectory.sample_count,
        processed_sample_count=len(results),
        accepted_sample_count=accepted_count,
        completed=completed,
        failed_sample_index=failed_sample_index,
        samples=tuple(results),
    )
