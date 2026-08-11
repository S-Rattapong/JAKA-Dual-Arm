"""ROS-independent sequential dual-arm IK and continuity analysis.

OFFLINE PLANNING ONLY — NO PHYSICAL ROBOT, NO JAKA DRIVER, NO MOTION.

For object-trajectory sample ``i``, independent left/right IK candidates are
requested for the already-derived ``^W T_L(i)`` and ``^W T_R(i)`` targets. The
pair is ordered as ``q(i) = [q_L1..q_L6, q_R1..q_R6]`` and accepted only after
combined dual-arm state validation. Sample zero uses an explicit 12-joint seed;
later samples use only the previous accepted pair. Phase 1G.3A records every
joint change as the raw subtraction ``Delta q_j(i) = q_j(i) - q_j(i-1)`` in
radians and preserves it unchanged. Phase 1G.3B additionally computes the
shortest angular analysis delta ``atan2(sin(Delta q), cos(Delta q))`` in the
canonical interval ``(-pi, +pi]``. Only the analysis delta is wrapped: stored
joint positions, IK results, and seeds are never normalized or modified. This
phase does not reject jumps, optimize them, or produce an executable trajectory.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
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
RAW_JOINT_DELTA_NOTICE = "RAW JOINT DELTA — WRAPAROUND NOT YET NORMALIZED"
RAW_JOINT_DELTA_PRESERVED_NOTICE = "RAW JOINT DELTA IS PRESERVED"
SHORTEST_ANGULAR_ANALYSIS_NOTICE = (
    "SHORTEST ANGULAR DELTA IS AN ANALYSIS METRIC ONLY"
)
JOINT_POSITIONS_UNCHANGED_NOTICE = (
    "JOINT POSITIONS ARE NOT NORMALIZED OR MODIFIED"
)
ANGULAR_ENDPOINT_TOLERANCE_RAD = 1e-12
WRAPAROUND_ADJUSTMENT_TOLERANCE_RAD = 1e-9
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


def shortest_angular_delta_rad(raw_delta_rad: float) -> float:
    """Return ``atan2(sin(dq), cos(dq))`` in canonical ``(-pi, +pi]``.

    ``-pi`` is the excluded endpoint. Results numerically within
    ``ANGULAR_ENDPOINT_TOLERANCE_RAD`` of it are represented as ``+pi``.
    This is a continuity metric only and never normalizes a joint position.
    """
    raw = _finite_number(raw_delta_rad, "raw_delta_rad")
    shortest = math.atan2(math.sin(raw), math.cos(raw))
    if math.isclose(
        shortest,
        -math.pi,
        rel_tol=0.0,
        abs_tol=ANGULAR_ENDPOINT_TOLERANCE_RAD,
    ):
        return math.pi
    return shortest


@dataclass(frozen=True)
class JointDeltaRecord:
    """One joint's preserved raw and analysis-only shortest angular changes."""

    joint_index: int
    previous_position_rad: float
    current_position_rad: float
    joint_name: str = field(init=False)
    delta_rad: float = field(init=False)
    abs_delta_rad: float = field(init=False)
    shortest_delta_rad: float = field(init=False)
    shortest_abs_delta_rad: float = field(init=False)
    wraparound_adjusted: bool = field(init=False)

    def __post_init__(self) -> None:
        if (
            isinstance(self.joint_index, bool)
            or not isinstance(self.joint_index, int)
            or not 0 <= self.joint_index < len(DUAL_ARM_JOINT_ORDER)
        ):
            raise ValueError("joint_index must identify a canonical dual-arm joint")
        previous = _finite_number(
            self.previous_position_rad,
            "previous_position_rad",
        )
        current = _finite_number(self.current_position_rad, "current_position_rad")
        delta = current - previous
        shortest = shortest_angular_delta_rad(delta)
        object.__setattr__(self, "previous_position_rad", previous)
        object.__setattr__(self, "current_position_rad", current)
        object.__setattr__(self, "joint_name", DUAL_ARM_JOINT_ORDER[self.joint_index])
        object.__setattr__(self, "delta_rad", delta)
        object.__setattr__(self, "abs_delta_rad", abs(delta))
        object.__setattr__(self, "shortest_delta_rad", shortest)
        object.__setattr__(self, "shortest_abs_delta_rad", abs(shortest))
        object.__setattr__(
            self,
            "wraparound_adjusted",
            abs(delta - shortest) > WRAPAROUND_ADJUSTMENT_TOLERANCE_RAD,
        )


@dataclass(frozen=True)
class TransitionContinuity:
    """Raw and shortest-angular continuity for an accepted transition."""

    from_sample_index: int
    to_sample_index: int
    joint_deltas: tuple[JointDeltaRecord, ...]
    max_abs_joint_step_rad: float = field(init=False)
    max_joint_index: int = field(init=False)
    max_joint_name: str = field(init=False)
    max_joint_delta_rad: float = field(init=False)
    max_shortest_abs_joint_step_rad: float = field(init=False)
    max_shortest_joint_index: int = field(init=False)
    max_shortest_joint_name: str = field(init=False)
    max_shortest_joint_delta_rad: float = field(init=False)
    wraparound_adjusted_record_count: int = field(init=False)

    def __post_init__(self) -> None:
        for label, value in (
            ("from_sample_index", self.from_sample_index),
            ("to_sample_index", self.to_sample_index),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        if self.to_sample_index <= self.from_sample_index:
            raise ValueError("to_sample_index must follow from_sample_index")
        try:
            records = tuple(self.joint_deltas)
        except TypeError as error:
            raise TypeError("joint_deltas must be an iterable") from error
        if len(records) != len(DUAL_ARM_JOINT_ORDER):
            raise ValueError("joint_deltas must contain exactly 12 records")
        if any(not isinstance(record, JointDeltaRecord) for record in records):
            raise TypeError("joint_deltas must contain JointDeltaRecord values")
        if tuple(record.joint_index for record in records) != tuple(range(12)):
            raise ValueError("joint_deltas must use canonical joint index order")

        # max() preserves the first record on an exact tie, so canonical order
        # deterministically selects the earliest joint index.
        maximum_record = max(records, key=lambda record: record.abs_delta_rad)
        maximum_shortest_record = max(
            records,
            key=lambda record: record.shortest_abs_delta_rad,
        )
        object.__setattr__(self, "joint_deltas", records)
        object.__setattr__(
            self,
            "max_abs_joint_step_rad",
            maximum_record.abs_delta_rad,
        )
        object.__setattr__(self, "max_joint_index", maximum_record.joint_index)
        object.__setattr__(self, "max_joint_name", maximum_record.joint_name)
        object.__setattr__(self, "max_joint_delta_rad", maximum_record.delta_rad)
        object.__setattr__(
            self,
            "max_shortest_abs_joint_step_rad",
            maximum_shortest_record.shortest_abs_delta_rad,
        )
        object.__setattr__(
            self,
            "max_shortest_joint_index",
            maximum_shortest_record.joint_index,
        )
        object.__setattr__(
            self,
            "max_shortest_joint_name",
            maximum_shortest_record.joint_name,
        )
        object.__setattr__(
            self,
            "max_shortest_joint_delta_rad",
            maximum_shortest_record.shortest_delta_rad,
        )
        object.__setattr__(
            self,
            "wraparound_adjusted_record_count",
            sum(record.wraparound_adjusted for record in records),
        )


@dataclass(frozen=True)
class TrajectoryContinuitySummary:
    """Raw and shortest-angular maxima across accepted transitions."""

    transition_count: int
    maximum_abs_joint_step_rad: float | None
    maximum_joint_name: str | None
    maximum_joint_index: int | None
    from_sample_index: int | None
    to_sample_index: int | None
    signed_delta_rad: float | None
    maximum_shortest_abs_joint_step_rad: float | None = None
    maximum_shortest_joint_name: str | None = None
    maximum_shortest_joint_index: int | None = None
    shortest_from_sample_index: int | None = None
    shortest_to_sample_index: int | None = None
    signed_shortest_delta_rad: float | None = None
    wraparound_adjusted_transition_count: int = 0
    wraparound_adjusted_record_count: int = 0


def analyze_joint_transition(
    *,
    from_sample_index: int,
    to_sample_index: int,
    previous_joint_positions_rad: Sequence[float],
    current_joint_positions_rad: Sequence[float],
) -> TransitionContinuity:
    """Build 12 records preserving raw subtraction plus shortest analysis."""
    previous = _joint_vector(
        previous_joint_positions_rad,
        12,
        "previous_joint_positions_rad",
    )
    current = _joint_vector(
        current_joint_positions_rad,
        12,
        "current_joint_positions_rad",
    )
    records = tuple(
        JointDeltaRecord(
            joint_index=index,
            previous_position_rad=previous[index],
            current_position_rad=current[index],
        )
        for index in range(12)
    )
    return TransitionContinuity(
        from_sample_index=from_sample_index,
        to_sample_index=to_sample_index,
        joint_deltas=records,
    )


def summarize_trajectory_continuity(
    transitions: Sequence[TransitionContinuity],
) -> TrajectoryContinuitySummary:
    """Summarize accepted transitions, retaining earliest exact maximum tie."""
    if isinstance(transitions, (str, bytes, bool)) or not isinstance(
        transitions,
        Sequence,
    ):
        raise TypeError("transitions must be a sequence")
    checked = tuple(transitions)
    if any(not isinstance(item, TransitionContinuity) for item in checked):
        raise TypeError("transitions must contain TransitionContinuity values")
    if not checked:
        return TrajectoryContinuitySummary(
            transition_count=0,
            maximum_abs_joint_step_rad=None,
            maximum_joint_name=None,
            maximum_joint_index=None,
            from_sample_index=None,
            to_sample_index=None,
            signed_delta_rad=None,
            maximum_shortest_abs_joint_step_rad=None,
            maximum_shortest_joint_name=None,
            maximum_shortest_joint_index=None,
            shortest_from_sample_index=None,
            shortest_to_sample_index=None,
            signed_shortest_delta_rad=None,
            wraparound_adjusted_transition_count=0,
            wraparound_adjusted_record_count=0,
        )

    # Transition order and each transition's canonical joint order make exact
    # trajectory-level ties deterministic without inventing another ordering.
    maximum = max(checked, key=lambda item: item.max_abs_joint_step_rad)
    maximum_shortest = max(
        checked,
        key=lambda item: item.max_shortest_abs_joint_step_rad,
    )
    return TrajectoryContinuitySummary(
        transition_count=len(checked),
        maximum_abs_joint_step_rad=maximum.max_abs_joint_step_rad,
        maximum_joint_name=maximum.max_joint_name,
        maximum_joint_index=maximum.max_joint_index,
        from_sample_index=maximum.from_sample_index,
        to_sample_index=maximum.to_sample_index,
        signed_delta_rad=maximum.max_joint_delta_rad,
        maximum_shortest_abs_joint_step_rad=(
            maximum_shortest.max_shortest_abs_joint_step_rad
        ),
        maximum_shortest_joint_name=maximum_shortest.max_shortest_joint_name,
        maximum_shortest_joint_index=maximum_shortest.max_shortest_joint_index,
        shortest_from_sample_index=maximum_shortest.from_sample_index,
        shortest_to_sample_index=maximum_shortest.to_sample_index,
        signed_shortest_delta_rad=maximum_shortest.max_shortest_joint_delta_rad,
        wraparound_adjusted_transition_count=sum(
            transition.wraparound_adjusted_record_count > 0
            for transition in checked
        ),
        wraparound_adjusted_record_count=sum(
            transition.wraparound_adjusted_record_count
            for transition in checked
        ),
    )


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
    continuity_from_previous: TransitionContinuity | None
    failure_reason: str | None
    diagnostic_message: str

    @property
    def joint_delta_from_previous_rad(self) -> JointVector12 | None:
        """Compatibility view derived from richer canonical delta records."""
        if self.continuity_from_previous is None:
            return None
        return tuple(
            record.delta_rad
            for record in self.continuity_from_previous.joint_deltas
        )  # type: ignore[return-value]

    @property
    def max_abs_joint_step_rad(self) -> float | None:
        """Compatibility maximum derived from the continuity transition."""
        if self.continuity_from_previous is None:
            return None
        return self.continuity_from_previous.max_abs_joint_step_rad


@dataclass(frozen=True)
class ObjectTrajectoryIkResult:
    trajectory_name: str
    requested_sample_count: int
    processed_sample_count: int
    accepted_sample_count: int
    completed: bool
    failed_sample_index: int | None
    samples: tuple[ObjectTrajectoryIkSampleResult, ...]
    continuity_summary: TrajectoryContinuitySummary = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "continuity_summary",
            summarize_trajectory_continuity(self.continuity_transitions),
        )

    @property
    def continuity_transitions(self) -> tuple[TransitionContinuity, ...]:
        """Return ordered continuity records for accepted transitions only."""
        return tuple(
            sample.continuity_from_previous
            for sample in self.samples
            if sample.accepted and sample.continuity_from_previous is not None
        )

    @property
    def maximum_observed_joint_step_rad(self) -> float | None:
        """Compatibility view of the trajectory raw-continuity maximum."""
        return self.continuity_summary.maximum_abs_joint_step_rad


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
        continuity_from_previous=None,
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
    previous_accepted_sample_index: int | None = None
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

        continuity = None
        if previous_accepted is not None:
            if previous_accepted_sample_index is None:
                raise RuntimeError("accepted seed is missing its sample index")
            continuity = analyze_joint_transition(
                from_sample_index=previous_accepted_sample_index,
                to_sample_index=sample_index,
                previous_joint_positions_rad=previous_accepted,
                current_joint_positions_rad=combined,
            )
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
            continuity_from_previous=continuity,
            failure_reason=None,
            diagnostic_message="ACCEPTED — PLANNING MODEL STATE VALID",
        ))
        previous_accepted = combined
        previous_accepted_sample_index = sample_index
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
