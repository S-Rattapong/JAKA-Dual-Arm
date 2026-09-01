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

Phase 1G.3C may flag suspicious changes using configurable absolute and relative
heuristics over shortest-angular deltas. These are offline analysis heuristics,
not manufacturer, controller, velocity, or physical robot safety limits. Flags
never change IK acceptance, trajectory completion, joint values, or seeds.

Phase 1G.4 performs greedy multi-candidate selection. Deterministic perturbed
seed banks produce independent arm candidates, raw-coordinate duplicates are
removed, every unique Left×Right pair is state-validity checked, and the valid
pair minimizing ``sum_j (q_candidate,j - q_previous,j)^2`` is selected. This is
a continuity preference in explicit coordinates, not a dynamic, energy, time,
or safety metric. No shortest-angle remapping is used for execution cost.

Phase 3A keeps that Greedy behavior as a baseline and adds a separate layered
candidate graph. Every later layer explores from all valid nodes in the prior
layer, without pruning, before exact dynamic-programming search and deterministic
backtracking. "Global" means the exact optimum over that generated graph only;
it does not mean an optimum over continuous configuration space or every IK root.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Protocol, Sequence

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory import ObjectTrajectory
from dual_arm_app.backend.planning_start_state_config import (
    PLANNING_START_STATE_COMBINED_RAD,
)


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
ABSOLUTE_STEP_REASON = "ABSOLUTE_STEP"
RELATIVE_GROWTH_REASON = "RELATIVE_GROWTH"
JUMP_HEURISTIC_WARNING = "ANALYSIS HEURISTIC ONLY — NOT A ROBOT SAFETY LIMIT"
CANDIDATE_EXPLORATION_WARNING = (
    "OFFLINE IK EXPLORATION PARAMETERS — NOT ROBOT SAFETY LIMITS"
)
NO_LEFT_IK_CANDIDATE = "NO_LEFT_IK_CANDIDATE"
NO_RIGHT_IK_CANDIDATE = "NO_RIGHT_IK_CANDIDATE"
NO_VALID_DUAL_ARM_PAIR = "NO_VALID_DUAL_ARM_PAIR"
EMPTY_GRAPH_LAYER = "EMPTY_GRAPH_LAYER"
NO_FEASIBLE_INCOMING_EDGE = "NO_FEASIBLE_INCOMING_EDGE"
NO_COMPLETE_GLOBAL_PATH = "NO_COMPLETE_GLOBAL_PATH"
GLOBAL_GRAPH_OPTIMALITY_SCOPE = (
    "EXACT GLOBAL OPTIMUM OVER THE GENERATED LAYERED CANDIDATE GRAPH ONLY"
)
RAW_DISPLACEMENT_COST_NOTICE = (
    "JOINT-SPACE DISPLACEMENT PROXY IN RADIAN^2 — NOT ENERGY, TORQUE, "
    "EXECUTION TIME, VELOCITY, ACCELERATION, MANUFACTURER SAFETY, OR COLLISION"
)
OFFLINE_MODEL_SEED_NOTICE = (
    "OFFLINE SYNTHETIC / MODEL SEED — NOT PHYSICAL ROBOT CALIBRATION"
)
# Backward-compatible name; the values live only in planning_start_state_config.py.
DEFAULT_INITIAL_DUAL_ARM_SEED_RAD = PLANNING_START_STATE_COMBINED_RAD

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


@dataclass(frozen=True)
class JointJumpDetectionConfig:
    """Configurable offline heuristic thresholds, never physical safety limits."""

    absolute_step_threshold_rad: float | None = None
    relative_step_ratio_threshold: float | None = None
    relative_reference_floor_rad: float = 0.0
    enabled: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise TypeError("enabled must be boolean")
        for label in (
            "absolute_step_threshold_rad",
            "relative_step_ratio_threshold",
        ):
            value = getattr(self, label)
            if value is None:
                continue
            checked = _finite_number(value, label)
            if checked <= 0.0:
                raise ValueError(f"{label} must be greater than zero")
            object.__setattr__(self, label, checked)
        floor = _finite_number(
            self.relative_reference_floor_rad,
            "relative_reference_floor_rad",
        )
        if floor < 0.0:
            raise ValueError("relative_reference_floor_rad must be non-negative")
        object.__setattr__(self, "relative_reference_floor_rad", floor)


DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG = JointJumpDetectionConfig(
    absolute_step_threshold_rad=1.0,
    relative_step_ratio_threshold=3.0,
    relative_reference_floor_rad=0.05,
)


@dataclass(frozen=True)
class JointJumpAssessment:
    """One joint's config-dependent, analysis-only suspicious-jump result."""

    joint_index: int
    joint_name: str
    from_sample_index: int
    to_sample_index: int
    shortest_delta_rad: float
    shortest_abs_delta_rad: float
    previous_shortest_abs_delta_rad: float | None
    relative_step_ratio: float | None
    absolute_flag: bool
    relative_flag: bool
    suspicious: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class TransitionJumpAssessment:
    """All 12 joint assessments for one accepted continuity transition."""

    from_sample_index: int
    to_sample_index: int
    assessments: tuple[JointJumpAssessment, ...]
    suspicious_joint_count: int = field(init=False)
    suspicious_joint_names: tuple[str, ...] = field(init=False)
    has_suspicious_jump: bool = field(init=False)

    def __post_init__(self) -> None:
        try:
            assessments = tuple(self.assessments)
        except TypeError as error:
            raise TypeError("assessments must be an iterable") from error
        if len(assessments) != len(DUAL_ARM_JOINT_ORDER):
            raise ValueError("assessments must contain exactly 12 joint records")
        if any(not isinstance(item, JointJumpAssessment) for item in assessments):
            raise TypeError("assessments must contain JointJumpAssessment values")
        if tuple(item.joint_index for item in assessments) != tuple(range(12)):
            raise ValueError("assessments must use canonical joint index order")
        if any(
            item.from_sample_index != self.from_sample_index
            or item.to_sample_index != self.to_sample_index
            for item in assessments
        ):
            raise ValueError("assessment sample indices must match the transition")
        suspicious_names = tuple(
            item.joint_name for item in assessments if item.suspicious
        )
        object.__setattr__(self, "assessments", assessments)
        object.__setattr__(self, "suspicious_joint_count", len(suspicious_names))
        object.__setattr__(self, "suspicious_joint_names", suspicious_names)
        object.__setattr__(self, "has_suspicious_jump", bool(suspicious_names))


@dataclass(frozen=True)
class TrajectoryJumpAnalysis:
    """Separate diagnostic result that never changes IK completion semantics."""

    config: JointJumpDetectionConfig
    transitions: tuple[TransitionJumpAssessment, ...]
    analysis_completed: bool = field(default=True, init=False)
    suspicious_transition_count: int = field(init=False)
    suspicious_joint_record_count: int = field(init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.config, JointJumpDetectionConfig):
            raise TypeError("config must be a JointJumpDetectionConfig")
        try:
            transitions = tuple(self.transitions)
        except TypeError as error:
            raise TypeError("transitions must be an iterable") from error
        if any(not isinstance(item, TransitionJumpAssessment) for item in transitions):
            raise TypeError("transitions must contain TransitionJumpAssessment values")
        object.__setattr__(self, "transitions", transitions)
        object.__setattr__(
            self,
            "suspicious_transition_count",
            sum(item.has_suspicious_jump for item in transitions),
        )
        object.__setattr__(
            self,
            "suspicious_joint_record_count",
            sum(item.suspicious_joint_count for item in transitions),
        )

    @property
    def suspicious_transitions(self) -> tuple[TransitionJumpAssessment, ...]:
        """Return suspicious transitions in their original chronological order."""
        return tuple(item for item in self.transitions if item.has_suspicious_jump)


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


def analyze_suspicious_joint_jumps(
    transitions: Sequence[TransitionContinuity],
    config: JointJumpDetectionConfig = DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
) -> TrajectoryJumpAnalysis:
    """Flag suspicious shortest-angular steps without rejecting any sample.

    For each joint, the absolute criterion is strictly
    ``current_shortest_abs > absolute_threshold``. Starting with the second
    transition, the relative ratio is available only when the preceding
    accepted transition's same-joint step is positive and at least the
    configured reference floor. Its criterion is also strict ``>``.
    """
    if isinstance(transitions, (str, bytes, bool)) or not isinstance(
        transitions,
        Sequence,
    ):
        raise TypeError("transitions must be a sequence")
    if not isinstance(config, JointJumpDetectionConfig):
        raise TypeError("config must be a JointJumpDetectionConfig")
    checked = tuple(transitions)
    if any(not isinstance(item, TransitionContinuity) for item in checked):
        raise TypeError("transitions must contain TransitionContinuity values")
    for previous, current in zip(checked, checked[1:]):
        if current.from_sample_index != previous.to_sample_index:
            raise ValueError("transitions must form a chronological accepted sequence")

    transition_results: list[TransitionJumpAssessment] = []
    for transition_index, transition in enumerate(checked):
        previous_transition = (
            checked[transition_index - 1] if transition_index > 0 else None
        )
        assessments: list[JointJumpAssessment] = []
        for record in transition.joint_deltas:
            previous_step = None
            ratio = None
            if previous_transition is not None:
                previous_step = previous_transition.joint_deltas[
                    record.joint_index
                ].shortest_abs_delta_rad
                if (
                    previous_step > 0.0
                    and previous_step >= config.relative_reference_floor_rad
                ):
                    ratio = record.shortest_abs_delta_rad / previous_step

            absolute_flag = bool(
                config.enabled
                and config.absolute_step_threshold_rad is not None
                and record.shortest_abs_delta_rad
                > config.absolute_step_threshold_rad
            )
            relative_flag = bool(
                config.enabled
                and config.relative_step_ratio_threshold is not None
                and ratio is not None
                and ratio > config.relative_step_ratio_threshold
            )
            reasons = tuple(
                reason
                for reason, flagged in (
                    (ABSOLUTE_STEP_REASON, absolute_flag),
                    (RELATIVE_GROWTH_REASON, relative_flag),
                )
                if flagged
            )
            assessments.append(JointJumpAssessment(
                joint_index=record.joint_index,
                joint_name=record.joint_name,
                from_sample_index=transition.from_sample_index,
                to_sample_index=transition.to_sample_index,
                shortest_delta_rad=record.shortest_delta_rad,
                shortest_abs_delta_rad=record.shortest_abs_delta_rad,
                previous_shortest_abs_delta_rad=previous_step,
                relative_step_ratio=ratio,
                absolute_flag=absolute_flag,
                relative_flag=relative_flag,
                suspicious=absolute_flag or relative_flag,
                reasons=reasons,
            ))
        transition_results.append(TransitionJumpAssessment(
            from_sample_index=transition.from_sample_index,
            to_sample_index=transition.to_sample_index,
            assessments=tuple(assessments),
        ))
    return TrajectoryJumpAnalysis(config=config, transitions=tuple(transition_results))


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


@dataclass(frozen=True)
class CanonicalJointPositionLimits:
    """Canonical model position limits supplied to deterministic seed exploration."""

    lower_rad: JointVector12
    upper_rad: JointVector12

    def __post_init__(self) -> None:
        lower = _joint_vector(self.lower_rad, 12, "lower_rad")
        upper = _joint_vector(self.upper_rad, 12, "upper_rad")
        if any(low >= high for low, high in zip(lower, upper)):
            raise ValueError("every lower joint limit must be less than its upper limit")
        object.__setattr__(self, "lower_rad", lower)
        object.__setattr__(self, "upper_rad", upper)

    def contains(self, positions_rad: Sequence[float]) -> bool:
        checked = _joint_vector(positions_rad, 12, "positions_rad")
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(checked, self.lower_rad, self.upper_rad)
        )

    def clamp(self, positions_rad: Sequence[float]) -> JointVector12:
        """Clamp an exploratory seed only; returned IK solutions are untouched."""
        checked = _joint_vector(positions_rad, 12, "positions_rad")
        return tuple(
            min(max(value, lower), upper)
            for value, lower, upper in zip(checked, self.lower_rad, self.upper_rad)
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class IkCandidateExplorationConfig:
    """Deterministic offline IK exploration and ranking parameters."""

    perturbation_offset_rad: float = 0.35
    max_attempts_per_arm: int = 13
    duplicate_tolerance_rad: float = 1e-5
    ranking_tie_tolerance: float = 1e-12

    def __post_init__(self) -> None:
        offset = _finite_number(self.perturbation_offset_rad, "perturbation_offset_rad")
        duplicate = _finite_number(self.duplicate_tolerance_rad, "duplicate_tolerance_rad")
        tie = _finite_number(self.ranking_tie_tolerance, "ranking_tie_tolerance")
        if offset <= 0.0:
            raise ValueError("perturbation_offset_rad must be greater than zero")
        if duplicate < 0.0:
            raise ValueError("duplicate_tolerance_rad must be non-negative")
        if tie < 0.0:
            raise ValueError("ranking_tie_tolerance must be non-negative")
        if (
            isinstance(self.max_attempts_per_arm, bool)
            or not isinstance(self.max_attempts_per_arm, int)
            or not 1 <= self.max_attempts_per_arm <= 13
        ):
            raise ValueError("max_attempts_per_arm must be an integer within [1, 13]")
        object.__setattr__(self, "perturbation_offset_rad", offset)
        object.__setattr__(self, "duplicate_tolerance_rad", duplicate)
        object.__setattr__(self, "ranking_tie_tolerance", tie)


DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG = IkCandidateExplorationConfig()
SINGLE_CANDIDATE_EXPLORATION_CONFIG = IkCandidateExplorationConfig(
    max_attempts_per_arm=1,
)


def generate_arm_seed_bank(
    base_seed_joint_positions_rad: Sequence[float],
    group_name: str,
    config: IkCandidateExplorationConfig,
    joint_limits: CanonicalJointPositionLimits | None,
) -> tuple[JointVector12, ...]:
    """Generate base, then ``+offset, -offset`` for each arm joint.

    Exploratory seeds use deterministic CLAMP against the supplied model limits.
    The base seed remains exact and must already satisfy limits when exploration
    is enabled. Clamping never changes a solver-returned IK solution.
    """
    base = _joint_vector(
        base_seed_joint_positions_rad,
        12,
        "base_seed_joint_positions_rad",
    )
    if group_name not in (LEFT_GROUP_NAME, RIGHT_GROUP_NAME):
        raise ValueError("group_name must be left_arm or right_arm")
    if not isinstance(config, IkCandidateExplorationConfig):
        raise TypeError("config must be an IkCandidateExplorationConfig")
    if config.max_attempts_per_arm > 1:
        if not isinstance(joint_limits, CanonicalJointPositionLimits):
            raise TypeError("joint_limits are required for exploratory seed generation")
        if not joint_limits.contains(base):
            raise ValueError("base seed must satisfy canonical model joint limits")

    seeds: list[JointVector12] = [base]  # type: ignore[list-item]
    first_joint_index = 0 if group_name == LEFT_GROUP_NAME else 6
    for local_joint_index in range(6):
        canonical_index = first_joint_index + local_joint_index
        for direction in (1.0, -1.0):
            perturbed = list(base)
            perturbed[canonical_index] += direction * config.perturbation_offset_rad
            seed = (
                joint_limits.clamp(perturbed)
                if joint_limits is not None
                else tuple(perturbed)
            )
            seeds.append(seed)  # type: ignore[arg-type]
    return tuple(seeds[:config.max_attempts_per_arm])


def raw_joint_vectors_are_duplicates(
    left_positions_rad: Sequence[float],
    right_positions_rad: Sequence[float],
    tolerance_rad: float,
) -> bool:
    """Compare explicit raw coordinates; no shortest-angle equivalence is used."""
    left = _joint_vector(left_positions_rad, 6, "left_positions_rad")
    right = _joint_vector(right_positions_rad, 6, "right_positions_rad")
    tolerance = _finite_number(tolerance_rad, "tolerance_rad")
    if tolerance < 0.0:
        raise ValueError("tolerance_rad must be non-negative")
    return max(abs(a - b) for a, b in zip(left, right)) <= tolerance


@dataclass(frozen=True)
class ArmIkCandidate:
    """One deterministic IK attempt and its raw-coordinate deduplication status."""

    candidate_index: int
    source_seed_index: int
    source_seed_rad: JointVector12
    solver_success: bool
    joint_positions_rad: JointVector6 | None
    duplicate_of_candidate_index: int | None
    diagnostic: str = ""

    @property
    def is_unique_success(self) -> bool:
        return self.solver_success and self.duplicate_of_candidate_index is None


@dataclass(frozen=True)
class ArmCandidateGenerationResult:
    group_name: str
    attempts: tuple[ArmIkCandidate, ...]

    @property
    def attempt_count(self) -> int:
        return len(self.attempts)

    @property
    def unique_candidates(self) -> tuple[ArmIkCandidate, ...]:
        return tuple(item for item in self.attempts if item.is_unique_success)

    @property
    def unique_candidate_count(self) -> int:
        return len(self.unique_candidates)


@dataclass(frozen=True)
class DualArmIkCandidatePair:
    """One scored explicit-coordinate Left×Right candidate pair."""

    pair_index: int
    left_candidate_index: int
    right_candidate_index: int
    combined_joint_positions_rad: JointVector12
    state_valid: bool
    raw_continuity_cost: float
    max_raw_joint_step_rad: float
    max_raw_joint_step_index: int
    max_raw_joint_step_name: str
    selected: bool = False
    validation_diagnostic: str = ""


def score_dual_arm_candidate_pair(
    *,
    pair_index: int,
    left_candidate: ArmIkCandidate,
    right_candidate: ArmIkCandidate,
    previous_joint_positions_rad: Sequence[float],
    state_valid: bool,
    validation_diagnostic: str = "",
) -> DualArmIkCandidatePair:
    """Score with ``sum((q_candidate-q_previous)^2)`` in raw radian coordinates."""
    if left_candidate.joint_positions_rad is None:
        raise ValueError("left candidate must provide joint positions")
    if right_candidate.joint_positions_rad is None:
        raise ValueError("right candidate must provide joint positions")
    previous = _joint_vector(
        previous_joint_positions_rad,
        12,
        "previous_joint_positions_rad",
    )
    combined = combine_arm_joint_solutions(
        left_candidate.joint_positions_rad,
        right_candidate.joint_positions_rad,
    )
    raw_delta = joint_delta_rad(combined, previous)
    cost = sum(value * value for value in raw_delta)
    maximum_index = max(range(12), key=lambda index: abs(raw_delta[index]))
    return DualArmIkCandidatePair(
        pair_index=pair_index,
        left_candidate_index=left_candidate.candidate_index,
        right_candidate_index=right_candidate.candidate_index,
        combined_joint_positions_rad=combined,
        state_valid=bool(state_valid),
        raw_continuity_cost=cost,
        max_raw_joint_step_rad=abs(raw_delta[maximum_index]),
        max_raw_joint_step_index=maximum_index,
        max_raw_joint_step_name=DUAL_ARM_JOINT_ORDER[maximum_index],
        validation_diagnostic=validation_diagnostic,
    )


def select_best_dual_arm_candidate_pair(
    pairs: Sequence[DualArmIkCandidatePair],
    tie_tolerance: float,
) -> tuple[DualArmIkCandidatePair | None, tuple[DualArmIkCandidatePair, ...]]:
    """Select valid minimum raw cost using explicit deterministic tie rules."""
    checked = tuple(pairs)
    if any(not isinstance(pair, DualArmIkCandidatePair) for pair in checked):
        raise TypeError("pairs must contain DualArmIkCandidatePair values")
    tolerance = _finite_number(tie_tolerance, "tie_tolerance")
    if tolerance < 0.0:
        raise ValueError("tie_tolerance must be non-negative")
    valid = tuple(pair for pair in checked if pair.state_valid)
    if not valid:
        return None, checked

    def better(candidate: DualArmIkCandidatePair, incumbent: DualArmIkCandidatePair) -> bool:
        cost_delta = candidate.raw_continuity_cost - incumbent.raw_continuity_cost
        if cost_delta < -tolerance:
            return True
        if abs(cost_delta) > tolerance:
            return False
        step_delta = candidate.max_raw_joint_step_rad - incumbent.max_raw_joint_step_rad
        if step_delta < -tolerance:
            return True
        if abs(step_delta) > tolerance:
            return False
        return (
            candidate.left_candidate_index,
            candidate.right_candidate_index,
            candidate.pair_index,
        ) < (
            incumbent.left_candidate_index,
            incumbent.right_candidate_index,
            incumbent.pair_index,
        )

    selected = valid[0]
    for candidate in valid[1:]:
        if better(candidate, selected):
            selected = candidate
    marked = tuple(
        replace(pair, selected=pair.pair_index == selected.pair_index)
        for pair in checked
    )
    return next(pair for pair in marked if pair.selected), marked


@dataclass(frozen=True)
class SampleCandidateSelectionDiagnostics:
    left_generation: ArmCandidateGenerationResult
    right_generation: ArmCandidateGenerationResult
    candidate_pairs: tuple[DualArmIkCandidatePair, ...]
    selected_pair_index: int | None

    @property
    def candidate_pair_count(self) -> int:
        return len(self.candidate_pairs)

    @property
    def valid_candidate_pair_count(self) -> int:
        return sum(pair.state_valid for pair in self.candidate_pairs)

    @property
    def selected_pair(self) -> DualArmIkCandidatePair | None:
        return next((pair for pair in self.candidate_pairs if pair.selected), None)


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


def _generate_arm_candidates(
    *,
    adapter: ObjectTrajectoryIkAdapter,
    group_name: str,
    ik_link_name: str,
    target_world_T_tip: RigidTransform,
    base_seed: JointVector12,
    timeout_s: float,
    config: IkCandidateExplorationConfig,
    joint_limits: CanonicalJointPositionLimits | None,
) -> ArmCandidateGenerationResult:
    seed_bank = generate_arm_seed_bank(base_seed, group_name, config, joint_limits)
    attempts: list[ArmIkCandidate] = []
    for seed_index, source_seed in enumerate(seed_bank):
        try:
            solution = adapter.solve_arm_ik(
                group_name=group_name,
                ik_link_name=ik_link_name,
                target_world_T_tip=target_world_T_tip,
                seed_joint_positions_rad=source_seed,
                timeout_s=timeout_s,
                avoid_collisions=False,
            )
        except Exception as error:
            solution = ArmIkSolution(
                False,
                diagnostic=f"IK candidate exception: {error}",
            )
        if not isinstance(solution, ArmIkSolution):
            solution = ArmIkSolution(
                False,
                diagnostic="IK candidate adapter returned invalid result",
            )

        duplicate_of = None
        if solution.success and solution.joint_positions_rad is not None:
            for earlier in attempts:
                if (
                    earlier.is_unique_success
                    and earlier.joint_positions_rad is not None
                    and raw_joint_vectors_are_duplicates(
                        solution.joint_positions_rad,
                        earlier.joint_positions_rad,
                        config.duplicate_tolerance_rad,
                    )
                ):
                    duplicate_of = earlier.candidate_index
                    break
        attempts.append(ArmIkCandidate(
            candidate_index=seed_index,
            source_seed_index=seed_index,
            source_seed_rad=source_seed,
            solver_success=solution.success,
            joint_positions_rad=solution.joint_positions_rad,
            duplicate_of_candidate_index=duplicate_of,
            diagnostic=solution.diagnostic,
        ))
    return ArmCandidateGenerationResult(group_name=group_name, attempts=tuple(attempts))


def _empty_arm_generation(group_name: str) -> ArmCandidateGenerationResult:
    return ArmCandidateGenerationResult(group_name=group_name, attempts=())


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
    candidate_selection: SampleCandidateSelectionDiagnostics | None = None

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

    @property
    def left_ik_attempt_count(self) -> int:
        return (
            self.candidate_selection.left_generation.attempt_count
            if self.candidate_selection is not None
            else int(self.left_ik_success)
        )

    @property
    def right_ik_attempt_count(self) -> int:
        return (
            self.candidate_selection.right_generation.attempt_count
            if self.candidate_selection is not None
            else int(self.right_ik_success)
        )

    @property
    def left_unique_candidate_count(self) -> int:
        return (
            self.candidate_selection.left_generation.unique_candidate_count
            if self.candidate_selection is not None
            else int(self.left_ik_success)
        )

    @property
    def right_unique_candidate_count(self) -> int:
        return (
            self.candidate_selection.right_generation.unique_candidate_count
            if self.candidate_selection is not None
            else int(self.right_ik_success)
        )

    @property
    def candidate_pair_count(self) -> int:
        return (
            self.candidate_selection.candidate_pair_count
            if self.candidate_selection is not None
            else int(self.combined_joint_positions_rad is not None)
        )

    @property
    def valid_candidate_pair_count(self) -> int:
        return (
            self.candidate_selection.valid_candidate_pair_count
            if self.candidate_selection is not None
            else int(self.combined_valid)
        )

    @property
    def selected_pair(self) -> DualArmIkCandidatePair | None:
        return (
            self.candidate_selection.selected_pair
            if self.candidate_selection is not None
            else None
        )

    @property
    def selected_left_candidate_index(self) -> int | None:
        return self.selected_pair.left_candidate_index if self.selected_pair else None

    @property
    def selected_right_candidate_index(self) -> int | None:
        return self.selected_pair.right_candidate_index if self.selected_pair else None

    @property
    def selected_pair_index(self) -> int | None:
        return self.selected_pair.pair_index if self.selected_pair else None

    @property
    def selected_raw_continuity_cost(self) -> float | None:
        return self.selected_pair.raw_continuity_cost if self.selected_pair else None

    @property
    def selected_max_raw_joint_step_rad(self) -> float | None:
        return self.selected_pair.max_raw_joint_step_rad if self.selected_pair else None

    @property
    def selected_max_raw_joint_step_index(self) -> int | None:
        return self.selected_pair.max_raw_joint_step_index if self.selected_pair else None

    @property
    def selected_max_raw_joint_step_name(self) -> str | None:
        return self.selected_pair.max_raw_joint_step_name if self.selected_pair else None


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

    def analyze_suspicious_jumps(
        self,
        config: JointJumpDetectionConfig = DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    ) -> TrajectoryJumpAnalysis:
        """Return separate flags without changing acceptance or completion."""
        return analyze_suspicious_joint_jumps(self.continuity_transitions, config)


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
    candidate_selection: SampleCandidateSelectionDiagnostics | None = None,
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
        candidate_selection=candidate_selection,
    )


def solve_sequential_object_trajectory_ik(
    trajectory: ObjectTrajectory,
    adapter: ObjectTrajectoryIkAdapter,
    *,
    initial_seed_joint_positions_rad: Sequence[float] = (
        DEFAULT_INITIAL_DUAL_ARM_SEED_RAD
    ),
    ik_timeout_s: float = DEFAULT_IK_TIMEOUT_S,
    candidate_exploration_config: IkCandidateExplorationConfig = (
        SINGLE_CANDIDATE_EXPLORATION_CONFIG
    ),
    joint_limits: CanonicalJointPositionLimits | None = None,
) -> ObjectTrajectoryIkResult:
    """Greedily select the minimum-cost valid candidate pair at each sample.

    Both arm seed banks derive from the same previous selected pair. Every unique
    Left×Right pair is checked with combined state validity, scored using raw
    explicit-coordinate squared displacement, and ranked deterministically.
    Only the selected pair propagates. The default one-attempt config preserves
    the prior single-candidate caller behavior; the runtime explicitly enables
    the 13-attempt offline exploration profile.
    """
    if not isinstance(trajectory, ObjectTrajectory):
        raise TypeError("trajectory must be an ObjectTrajectory")
    if not callable(getattr(adapter, "solve_arm_ik", None)):
        raise TypeError("adapter must provide solve_arm_ik")
    if not callable(getattr(adapter, "check_combined_state", None)):
        raise TypeError("adapter must provide check_combined_state")
    if not isinstance(candidate_exploration_config, IkCandidateExplorationConfig):
        raise TypeError(
            "candidate_exploration_config must be an IkCandidateExplorationConfig"
        )
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
        left_generation = _generate_arm_candidates(
            adapter=adapter,
            group_name=LEFT_GROUP_NAME,
            ik_link_name=LEFT_IK_LINK_NAME,
            target_world_T_tip=sample.world_T_left,
            base_seed=pair_seed,  # type: ignore[arg-type]
            timeout_s=timeout_s,
            config=candidate_exploration_config,
            joint_limits=joint_limits,
        )
        if not left_generation.unique_candidates:
            failed_sample_index = sample_index
            diagnostics = SampleCandidateSelectionDiagnostics(
                left_generation=left_generation,
                right_generation=_empty_arm_generation(RIGHT_GROUP_NAME),
                candidate_pairs=(),
                selected_pair_index=None,
            )
            results.append(_failure_sample(
                sample_index=sample_index,
                time_from_start_s=sample.time_from_start_s,
                alpha=sample.alpha,
                left_success=False,
                right_success=False,
                combined_valid=False,
                left_positions=None,
                right_positions=None,
                combined_positions=None,
                reason=NO_LEFT_IK_CANDIDATE,
                diagnostic="No unique successful Left IK candidate",
                candidate_selection=diagnostics,
            ))
            break

        right_generation = _generate_arm_candidates(
            adapter=adapter,
            group_name=RIGHT_GROUP_NAME,
            ik_link_name=RIGHT_IK_LINK_NAME,
            target_world_T_tip=sample.world_T_right,
            base_seed=pair_seed,  # type: ignore[arg-type]
            timeout_s=timeout_s,
            config=candidate_exploration_config,
            joint_limits=joint_limits,
        )
        if not right_generation.unique_candidates:
            failed_sample_index = sample_index
            diagnostics = SampleCandidateSelectionDiagnostics(
                left_generation=left_generation,
                right_generation=right_generation,
                candidate_pairs=(),
                selected_pair_index=None,
            )
            results.append(_failure_sample(
                sample_index=sample_index,
                time_from_start_s=sample.time_from_start_s,
                alpha=sample.alpha,
                left_success=True,
                right_success=False,
                combined_valid=False,
                left_positions=left_generation.unique_candidates[0].joint_positions_rad,
                right_positions=None,
                combined_positions=None,
                reason=NO_RIGHT_IK_CANDIDATE,
                diagnostic="No unique successful Right IK candidate",
                candidate_selection=diagnostics,
            ))
            break

        pair_records: list[DualArmIkCandidatePair] = []
        for left_candidate in left_generation.unique_candidates:
            for right_candidate in right_generation.unique_candidates:
                combined_candidate = combine_arm_joint_solutions(
                    left_candidate.joint_positions_rad,  # type: ignore[arg-type]
                    right_candidate.joint_positions_rad,  # type: ignore[arg-type]
                )
                try:
                    validity = adapter.check_combined_state(
                        joint_positions_rad=combined_candidate,
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
                pair_records.append(score_dual_arm_candidate_pair(
                    pair_index=len(pair_records),
                    left_candidate=left_candidate,
                    right_candidate=right_candidate,
                    previous_joint_positions_rad=pair_seed,
                    state_valid=validity.valid,
                    validation_diagnostic=validity.diagnostic,
                ))

        selected_pair, marked_pairs = select_best_dual_arm_candidate_pair(
            pair_records,
            candidate_exploration_config.ranking_tie_tolerance,
        )
        diagnostics = SampleCandidateSelectionDiagnostics(
            left_generation=left_generation,
            right_generation=right_generation,
            candidate_pairs=marked_pairs,
            selected_pair_index=(
                selected_pair.pair_index if selected_pair is not None else None
            ),
        )
        if selected_pair is None:
            failed_sample_index = sample_index
            results.append(_failure_sample(
                sample_index=sample_index,
                time_from_start_s=sample.time_from_start_s,
                alpha=sample.alpha,
                left_success=True,
                right_success=True,
                combined_valid=False,
                left_positions=left_generation.unique_candidates[0].joint_positions_rad,
                right_positions=right_generation.unique_candidates[0].joint_positions_rad,
                combined_positions=(
                    marked_pairs[0].combined_joint_positions_rad
                    if marked_pairs
                    else None
                ),
                reason=NO_VALID_DUAL_ARM_PAIR,
                diagnostic="No Left×Right candidate pair passed dual-arm validity",
                candidate_selection=diagnostics,
            ))
            break

        selected_left = next(
            candidate
            for candidate in left_generation.unique_candidates
            if candidate.candidate_index == selected_pair.left_candidate_index
        )
        selected_right = next(
            candidate
            for candidate in right_generation.unique_candidates
            if candidate.candidate_index == selected_pair.right_candidate_index
        )
        combined = selected_pair.combined_joint_positions_rad
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
            left_joint_positions_rad=selected_left.joint_positions_rad,
            right_joint_positions_rad=selected_right.joint_positions_rad,
            combined_joint_positions_rad=combined,
            continuity_from_previous=continuity,
            failure_reason=None,
            diagnostic_message="ACCEPTED — SELECTED VALID MINIMUM RAW-COST PAIR",
            candidate_selection=diagnostics,
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


@dataclass(frozen=True)
class TrajectoryCandidateProvenance:
    """One valid Left×Right generation route into a graph node."""

    parent_node_id: str | None
    left_candidate_index: int
    left_source_seed_index: int
    left_diagnostic: str
    right_candidate_index: int
    right_source_seed_index: int
    right_diagnostic: str
    left_source_seed_rad: JointVector12 | None = None
    right_source_seed_rad: JointVector12 | None = None


@dataclass(frozen=True)
class TrajectoryCandidateAttemptDiagnostic:
    """Diagnostics-only record for one arm IK attempt during graph generation."""

    sample_index: int
    parent_node_id: str | None
    group_name: str
    candidate_index: int
    source_seed_index: int
    source_seed_rad: JointVector12
    solver_success: bool
    joint_positions_rad: JointVector6 | None
    duplicate_of_candidate_index: int | None
    diagnostic: str


@dataclass(frozen=True)
class TrajectoryCandidatePairRejection:
    """Diagnostics-only record for a candidate pair rejected before node insertion."""

    sample_index: int
    parent_node_id: str | None
    rejection_type: str
    left_candidate_index: int
    left_source_seed_index: int
    right_candidate_index: int
    right_source_seed_index: int
    combined_joint_positions_rad: JointVector12
    diagnostic: str


@dataclass(frozen=True)
class TrajectoryCandidateNode:
    """One state-valid 12-joint node in a deterministic sample layer."""

    sample_index: int
    node_index: int
    combined_joint_positions_rad: JointVector12
    provenance: tuple[TrajectoryCandidateProvenance, ...]
    state_validity_provenance: tuple[str, ...]
    node_id: str = field(init=False)
    state_valid: bool = field(default=True, init=False)

    def __post_init__(self) -> None:
        for label, value in (
            ("sample_index", self.sample_index),
            ("node_index", self.node_index),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        positions = _joint_vector(
            self.combined_joint_positions_rad,
            12,
            "combined_joint_positions_rad",
        )
        provenance = tuple(self.provenance)
        validity = tuple(self.state_validity_provenance)
        if not provenance:
            raise ValueError("graph node provenance must not be empty")
        if any(not isinstance(item, TrajectoryCandidateProvenance) for item in provenance):
            raise TypeError("provenance must contain TrajectoryCandidateProvenance")
        if not validity or any(not isinstance(item, str) for item in validity):
            raise ValueError("state-validity provenance must contain diagnostics")
        object.__setattr__(self, "combined_joint_positions_rad", positions)
        object.__setattr__(self, "provenance", provenance)
        object.__setattr__(self, "state_validity_provenance", validity)
        object.__setattr__(self, "node_id", f"L{self.sample_index}:N{self.node_index}")

    @property
    def left_joint_positions_rad(self) -> JointVector6:
        return self.combined_joint_positions_rad[:6]  # type: ignore[return-value]

    @property
    def right_joint_positions_rad(self) -> JointVector6:
        return self.combined_joint_positions_rad[6:]  # type: ignore[return-value]


@dataclass(frozen=True)
class TrajectoryCandidateLayer:
    """All unioned, raw-coordinate-deduplicated valid nodes for one sample."""

    sample_index: int
    time_from_start_s: float
    nodes: tuple[TrajectoryCandidateNode, ...]
    source_parent_node_ids: tuple[str, ...]
    left_unique_candidate_count: int = 0
    right_unique_candidate_count: int = 0
    candidate_pair_count: int = 0
    valid_candidate_pair_count: int = 0
    candidate_attempt_diagnostics: tuple[
        TrajectoryCandidateAttemptDiagnostic, ...
    ] = ()
    rejected_candidate_pairs: tuple[TrajectoryCandidatePairRejection, ...] = ()

    def __post_init__(self) -> None:
        if (
            isinstance(self.sample_index, bool)
            or not isinstance(self.sample_index, int)
            or self.sample_index < 0
        ):
            raise ValueError("sample_index must be a non-negative integer")
        time_s = _finite_number(self.time_from_start_s, "time_from_start_s")
        if time_s < 0.0:
            raise ValueError("time_from_start_s must be non-negative")
        nodes = tuple(self.nodes)
        if any(not isinstance(node, TrajectoryCandidateNode) for node in nodes):
            raise TypeError("nodes must contain TrajectoryCandidateNode values")
        if any(node.sample_index != self.sample_index for node in nodes):
            raise ValueError("every node must belong to the layer sample index")
        if tuple(node.node_index for node in nodes) != tuple(range(len(nodes))):
            raise ValueError("node indices must be deterministic from zero")
        parents = tuple(self.source_parent_node_ids)
        if any(not isinstance(item, str) for item in parents):
            raise TypeError("source_parent_node_ids must contain strings")
        for label in (
            "left_unique_candidate_count",
            "right_unique_candidate_count",
            "candidate_pair_count",
            "valid_candidate_pair_count",
        ):
            value = getattr(self, label)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{label} must be a non-negative integer")
        object.__setattr__(self, "time_from_start_s", time_s)
        object.__setattr__(self, "nodes", nodes)
        object.__setattr__(self, "source_parent_node_ids", parents)
        object.__setattr__(
            self,
            "candidate_attempt_diagnostics",
            tuple(self.candidate_attempt_diagnostics),
        )
        object.__setattr__(
            self,
            "rejected_candidate_pairs",
            tuple(self.rejected_candidate_pairs),
        )


@dataclass(frozen=True)
class PlannerFailureDiagnostic:
    reason: str
    failed_sample_index: int
    message: str
    left_candidate_count: int = 0
    right_candidate_count: int = 0
    candidate_pair_count: int = 0
    valid_pair_count: int = 0


@dataclass(frozen=True)
class TrajectoryCandidateGraph:
    """Immutable generated graph; incomplete graphs retain failure diagnostics."""

    trajectory_name: str
    requested_layer_count: int
    layers: tuple[TrajectoryCandidateLayer, ...]
    completed: bool
    failure: PlannerFailureDiagnostic | None = None
    candidate_pruning_applied: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        layers = tuple(self.layers)
        if any(not isinstance(layer, TrajectoryCandidateLayer) for layer in layers):
            raise TypeError("layers must contain TrajectoryCandidateLayer values")
        if tuple(layer.sample_index for layer in layers) != tuple(range(len(layers))):
            raise ValueError("graph layers must be chronological from zero")
        if any(
            current.time_from_start_s >= following.time_from_start_s
            for current, following in zip(layers, layers[1:])
        ):
            raise ValueError("graph layer times must be strictly increasing")
        if self.completed:
            if len(layers) != self.requested_layer_count or self.failure is not None:
                raise ValueError("completed graph must contain every requested layer")
        elif self.failure is None:
            raise ValueError("incomplete graph requires a failure diagnostic")
        object.__setattr__(self, "layers", layers)


@dataclass(frozen=True)
class TrajectoryEdgeFeasibility:
    feasible: bool
    diagnostic: str

    def __post_init__(self) -> None:
        if not isinstance(self.feasible, bool):
            raise TypeError("feasible must be boolean")
        if not isinstance(self.diagnostic, str) or not self.diagnostic:
            raise ValueError("edge feasibility diagnostic must be non-empty")


@dataclass(frozen=True)
class TrajectoryGraphEdge:
    from_sample_index: int
    from_node_index: int
    to_sample_index: int
    to_node_index: int
    feasible: bool
    feasibility_diagnostic: str
    raw_displacement_cost_rad2: float
    max_raw_joint_step_rad: float
    max_raw_joint_step_index: int
    max_raw_joint_step_name: str

    def __post_init__(self) -> None:
        if self.to_sample_index != self.from_sample_index + 1:
            raise ValueError("graph edges may connect consecutive layers only")
        _finite_number(self.raw_displacement_cost_rad2, "raw_displacement_cost_rad2")
        _finite_number(self.max_raw_joint_step_rad, "max_raw_joint_step_rad")


@dataclass(frozen=True)
class SelectedTrajectoryGraphPoint:
    sample_index: int
    time_from_start_s: float
    graph_node_id: str
    graph_node_index: int
    combined_joint_positions_rad: JointVector12
    left_joint_positions_rad: JointVector6
    right_joint_positions_rad: JointVector6
    edge_cost_from_predecessor_rad2: float
    cumulative_cost_rad2: float
    candidate_provenance: tuple[TrajectoryCandidateProvenance, ...]


@dataclass(frozen=True)
class GlobalNodeSearchDiagnostic:
    """The exact DP value/backpointer retained for one generated graph node."""

    sample_index: int
    node_index: int
    reachable: bool
    selected_predecessor_node_index: int | None
    local_edge_cost_rad2: float | None
    cumulative_cost_rad2: float | None


@dataclass(frozen=True)
class GlobalTrajectorySearchResult:
    completed: bool
    selected_path: tuple[SelectedTrajectoryGraphPoint, ...]
    cumulative_raw_displacement_cost_rad2: float | None
    maximum_raw_single_joint_transition_rad: float | None
    failed_sample_index: int | None
    failure: PlannerFailureDiagnostic | None
    evaluated_edges: tuple[TrajectoryGraphEdge, ...]
    node_search_diagnostics: tuple[GlobalNodeSearchDiagnostic, ...] = ()
    optimality_scope: str = field(default=GLOBAL_GRAPH_OPTIMALITY_SCOPE, init=False)


@dataclass(frozen=True)
class GreedyTrajectorySearchResult:
    completed: bool
    selected_path: tuple[SelectedTrajectoryGraphPoint, ...]
    cumulative_raw_displacement_cost_rad2: float | None
    maximum_raw_single_joint_transition_rad: float | None
    failed_sample_index: int | None
    failure: PlannerFailureDiagnostic | None


@dataclass(frozen=True)
class GreedyGlobalComparison:
    greedy: GreedyTrajectorySearchResult
    global_search: GlobalTrajectorySearchResult
    global_not_worse_on_equivalent_graph: bool | None
    tie_tolerance: float


EdgeFeasibilityCallback = Callable[
    [TrajectoryCandidateNode, TrajectoryCandidateNode],
    TrajectoryEdgeFeasibility,
]


def _combined_raw_duplicates(
    left: Sequence[float],
    right: Sequence[float],
    tolerance_rad: float,
) -> bool:
    checked_left = _joint_vector(left, 12, "left_combined_joint_positions_rad")
    checked_right = _joint_vector(right, 12, "right_combined_joint_positions_rad")
    return max(abs(a - b) for a, b in zip(checked_left, checked_right)) <= tolerance_rad


def build_object_trajectory_candidate_graph(
    trajectory: ObjectTrajectory,
    adapter: ObjectTrajectoryIkAdapter,
    *,
    initial_seed_joint_positions_rad: Sequence[float] = DEFAULT_INITIAL_DUAL_ARM_SEED_RAD,
    ik_timeout_s: float = DEFAULT_IK_TIMEOUT_S,
    candidate_exploration_config: IkCandidateExplorationConfig = (
        DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG
    ),
    joint_limits: CanonicalJointPositionLimits | None = None,
    retain_debug_diagnostics: bool = False,
) -> TrajectoryCandidateGraph:
    """Generate every layer from all valid nodes in the preceding layer.

    No candidate pruning is performed. Each parent node independently feeds the
    accepted multi-seed arm machinery; valid Left×Right results are unioned and
    raw-coordinate-deduplicated within the layer. This preserves branch diversity
    and is independent of the existing greedy winner propagation.
    """
    if not isinstance(trajectory, ObjectTrajectory):
        raise TypeError("trajectory must be an ObjectTrajectory")
    if not callable(getattr(adapter, "solve_arm_ik", None)):
        raise TypeError("adapter must provide solve_arm_ik")
    if not callable(getattr(adapter, "check_combined_state", None)):
        raise TypeError("adapter must provide check_combined_state")
    if not isinstance(candidate_exploration_config, IkCandidateExplorationConfig):
        raise TypeError("candidate_exploration_config must be valid")
    if not isinstance(retain_debug_diagnostics, bool):
        raise TypeError("retain_debug_diagnostics must be boolean")
    initial_seed = _joint_vector(
        initial_seed_joint_positions_rad,
        12,
        "initial_seed_joint_positions_rad",
    )
    timeout_s = _finite_number(ik_timeout_s, "ik_timeout_s")
    if timeout_s <= 0.0:
        raise ValueError("ik_timeout_s must be greater than zero")

    layers: list[TrajectoryCandidateLayer] = []
    for sample_index, sample in enumerate(trajectory.samples):
        parent_seeds: tuple[tuple[str | None, JointVector12], ...]
        if sample_index == 0:
            parent_seeds = ((None, initial_seed),)  # type: ignore[assignment]
        else:
            parent_seeds = tuple(
                (node.node_id, node.combined_joint_positions_rad)
                for node in layers[-1].nodes
            )

        nodes: list[TrajectoryCandidateNode] = []
        left_count = 0
        right_count = 0
        pair_count = 0
        valid_pair_count = 0
        attempt_diagnostics: list[TrajectoryCandidateAttemptDiagnostic] = []
        rejected_pairs: list[TrajectoryCandidatePairRejection] = []
        for parent_node_id, base_seed in parent_seeds:
            left_generation = _generate_arm_candidates(
                adapter=adapter,
                group_name=LEFT_GROUP_NAME,
                ik_link_name=LEFT_IK_LINK_NAME,
                target_world_T_tip=sample.world_T_left,
                base_seed=base_seed,
                timeout_s=timeout_s,
                config=candidate_exploration_config,
                joint_limits=joint_limits,
            )
            right_generation = _generate_arm_candidates(
                adapter=adapter,
                group_name=RIGHT_GROUP_NAME,
                ik_link_name=RIGHT_IK_LINK_NAME,
                target_world_T_tip=sample.world_T_right,
                base_seed=base_seed,
                timeout_s=timeout_s,
                config=candidate_exploration_config,
                joint_limits=joint_limits,
            )
            if retain_debug_diagnostics:
                for generation in (left_generation, right_generation):
                    attempt_diagnostics.extend(
                        TrajectoryCandidateAttemptDiagnostic(
                            sample_index=sample_index,
                            parent_node_id=parent_node_id,
                            group_name=generation.group_name,
                            candidate_index=attempt.candidate_index,
                            source_seed_index=attempt.source_seed_index,
                            source_seed_rad=attempt.source_seed_rad,
                            solver_success=attempt.solver_success,
                            joint_positions_rad=attempt.joint_positions_rad,
                            duplicate_of_candidate_index=(
                                attempt.duplicate_of_candidate_index
                            ),
                            diagnostic=attempt.diagnostic,
                        )
                        for attempt in generation.attempts
                    )
            left_count += left_generation.unique_candidate_count
            right_count += right_generation.unique_candidate_count
            for left_candidate in left_generation.unique_candidates:
                for right_candidate in right_generation.unique_candidates:
                    pair_count += 1
                    combined = combine_arm_joint_solutions(
                        left_candidate.joint_positions_rad,  # type: ignore[arg-type]
                        right_candidate.joint_positions_rad,  # type: ignore[arg-type]
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
                        if retain_debug_diagnostics:
                            rejected_pairs.append(TrajectoryCandidatePairRejection(
                                sample_index=sample_index,
                                parent_node_id=parent_node_id,
                                rejection_type="STATE_VALIDITY_REJECTION",
                                left_candidate_index=left_candidate.candidate_index,
                                left_source_seed_index=left_candidate.source_seed_index,
                                right_candidate_index=right_candidate.candidate_index,
                                right_source_seed_index=right_candidate.source_seed_index,
                                combined_joint_positions_rad=combined,
                                diagnostic=(
                                    validity.diagnostic
                                    or "State validity rejected candidate pair without detail"
                                ),
                            ))
                        continue
                    valid_pair_count += 1
                    provenance = TrajectoryCandidateProvenance(
                        parent_node_id=parent_node_id,
                        left_candidate_index=left_candidate.candidate_index,
                        left_source_seed_index=left_candidate.source_seed_index,
                        left_diagnostic=left_candidate.diagnostic,
                        right_candidate_index=right_candidate.candidate_index,
                        right_source_seed_index=right_candidate.source_seed_index,
                        right_diagnostic=right_candidate.diagnostic,
                        left_source_seed_rad=(
                            left_candidate.source_seed_rad
                            if retain_debug_diagnostics else None
                        ),
                        right_source_seed_rad=(
                            right_candidate.source_seed_rad
                            if retain_debug_diagnostics else None
                        ),
                    )
                    duplicate_index = next((
                        index
                        for index, node in enumerate(nodes)
                        if _combined_raw_duplicates(
                            combined,
                            node.combined_joint_positions_rad,
                            candidate_exploration_config.duplicate_tolerance_rad,
                        )
                    ), None)
                    if duplicate_index is not None:
                        duplicate = nodes[duplicate_index]
                        nodes[duplicate_index] = replace(
                            duplicate,
                            provenance=(*duplicate.provenance, provenance),
                            state_validity_provenance=(
                                *duplicate.state_validity_provenance,
                                validity.diagnostic,
                            ),
                        )
                        continue
                    nodes.append(TrajectoryCandidateNode(
                        sample_index=sample_index,
                        node_index=len(nodes),
                        combined_joint_positions_rad=combined,
                        provenance=(provenance,),
                        state_validity_provenance=(validity.diagnostic,),
                    ))

        if not nodes:
            if left_count == 0:
                reason = NO_LEFT_IK_CANDIDATE
                message = "No unique successful Left IK candidate from any parent"
            elif right_count == 0:
                reason = NO_RIGHT_IK_CANDIDATE
                message = "No unique successful Right IK candidate from any parent"
            elif pair_count > 0 and valid_pair_count == 0:
                reason = NO_VALID_DUAL_ARM_PAIR
                message = "No Left×Right pair passed combined dual-arm validity"
            else:
                reason = EMPTY_GRAPH_LAYER
                message = "Candidate union produced an empty graph layer"
            failure = PlannerFailureDiagnostic(
                reason=reason,
                failed_sample_index=sample_index,
                message=message,
                left_candidate_count=left_count,
                right_candidate_count=right_count,
                candidate_pair_count=pair_count,
                valid_pair_count=valid_pair_count,
            )
            return TrajectoryCandidateGraph(
                trajectory_name=trajectory.name,
                requested_layer_count=trajectory.sample_count,
                layers=tuple(layers),
                completed=False,
                failure=failure,
            )

        layers.append(TrajectoryCandidateLayer(
            sample_index=sample_index,
            time_from_start_s=sample.time_from_start_s,
            nodes=tuple(nodes),
            source_parent_node_ids=tuple(
                parent_id if parent_id is not None else "INITIAL_SEED"
                for parent_id, _seed in parent_seeds
            ),
            left_unique_candidate_count=left_count,
            right_unique_candidate_count=right_count,
            candidate_pair_count=pair_count,
            valid_candidate_pair_count=valid_pair_count,
            candidate_attempt_diagnostics=tuple(attempt_diagnostics),
            rejected_candidate_pairs=tuple(rejected_pairs),
        ))

    return TrajectoryCandidateGraph(
        trajectory_name=trajectory.name,
        requested_layer_count=trajectory.sample_count,
        layers=tuple(layers),
        completed=True,
    )


def default_graph_edge_feasibility(
    from_node: TrajectoryCandidateNode,
    to_node: TrajectoryCandidateNode,
) -> TrajectoryEdgeFeasibility:
    """Apply structural Phase-3 feasibility only, with no invented threshold."""
    if not isinstance(from_node, TrajectoryCandidateNode) or not isinstance(
        to_node,
        TrajectoryCandidateNode,
    ):
        return TrajectoryEdgeFeasibility(False, "Both endpoints must be graph nodes")
    if to_node.sample_index != from_node.sample_index + 1:
        return TrajectoryEdgeFeasibility(False, "Layers are not consecutive")
    if not from_node.state_valid or not to_node.state_valid:
        return TrajectoryEdgeFeasibility(False, "Both endpoints must be valid nodes")
    try:
        _joint_vector(from_node.combined_joint_positions_rad, 12, "from_node joints")
        _joint_vector(to_node.combined_joint_positions_rad, 12, "to_node joints")
    except (TypeError, ValueError) as error:
        return TrajectoryEdgeFeasibility(False, f"Invalid edge endpoint: {error}")
    return TrajectoryEdgeFeasibility(
        True,
        "FEASIBLE — VALID NODES IN CONSECUTIVE LAYERS; NO PHASE-4 LIMIT APPLIED",
    )


def graph_edge_between(
    from_node: TrajectoryCandidateNode,
    to_node: TrajectoryCandidateNode,
    edge_feasibility: EdgeFeasibilityCallback = default_graph_edge_feasibility,
) -> TrajectoryGraphEdge:
    """Return structural feasibility plus raw displacement proxy diagnostics."""
    feasibility = edge_feasibility(from_node, to_node)
    if not isinstance(feasibility, TrajectoryEdgeFeasibility):
        raise TypeError("edge feasibility callback must return TrajectoryEdgeFeasibility")
    delta = joint_delta_rad(
        to_node.combined_joint_positions_rad,
        from_node.combined_joint_positions_rad,
    )
    maximum_index = max(range(12), key=lambda index: abs(delta[index]))
    return TrajectoryGraphEdge(
        from_sample_index=from_node.sample_index,
        from_node_index=from_node.node_index,
        to_sample_index=to_node.sample_index,
        to_node_index=to_node.node_index,
        feasible=feasibility.feasible,
        feasibility_diagnostic=feasibility.diagnostic,
        raw_displacement_cost_rad2=sum(value * value for value in delta),
        max_raw_joint_step_rad=abs(delta[maximum_index]),
        max_raw_joint_step_index=maximum_index,
        max_raw_joint_step_name=DUAL_ARM_JOINT_ORDER[maximum_index],
    )


def build_consecutive_graph_edges(
    graph: TrajectoryCandidateGraph,
    edge_feasibility: EdgeFeasibilityCallback = default_graph_edge_feasibility,
) -> tuple[TrajectoryGraphEdge, ...]:
    """Build the complete consecutive-layer bipartite edge set without pruning."""
    if not isinstance(graph, TrajectoryCandidateGraph):
        raise TypeError("graph must be a TrajectoryCandidateGraph")
    edges: list[TrajectoryGraphEdge] = []
    for from_layer, to_layer in zip(graph.layers, graph.layers[1:]):
        for from_node in from_layer.nodes:
            for to_node in to_layer.nodes:
                edges.append(graph_edge_between(from_node, to_node, edge_feasibility))
    return tuple(edges)


def _path_point(
    layer: TrajectoryCandidateLayer,
    node: TrajectoryCandidateNode,
    edge_cost: float,
    cumulative_cost: float,
) -> SelectedTrajectoryGraphPoint:
    return SelectedTrajectoryGraphPoint(
        sample_index=layer.sample_index,
        time_from_start_s=layer.time_from_start_s,
        graph_node_id=node.node_id,
        graph_node_index=node.node_index,
        combined_joint_positions_rad=node.combined_joint_positions_rad,
        left_joint_positions_rad=node.left_joint_positions_rad,
        right_joint_positions_rad=node.right_joint_positions_rad,
        edge_cost_from_predecessor_rad2=edge_cost,
        cumulative_cost_rad2=cumulative_cost,
        candidate_provenance=node.provenance,
    )


def search_global_candidate_graph(
    graph: TrajectoryCandidateGraph,
    *,
    tie_tolerance: float = 1e-12,
    edge_feasibility: EdgeFeasibilityCallback = default_graph_edge_feasibility,
    retain_debug_diagnostics: bool = False,
) -> GlobalTrajectorySearchResult:
    """Exact dynamic programming shortest path over the generated layered DAG.

    Costs within ``tie_tolerance`` tie; the lower predecessor node index wins,
    then the lower final node index wins. This is exact only over ``graph`` and
    never claims optimality over continuous configuration space or all IK roots.
    """
    if not isinstance(graph, TrajectoryCandidateGraph):
        raise TypeError("graph must be a TrajectoryCandidateGraph")
    tolerance = _finite_number(tie_tolerance, "tie_tolerance")
    if tolerance < 0.0:
        raise ValueError("tie_tolerance must be non-negative")
    if not isinstance(retain_debug_diagnostics, bool):
        raise TypeError("retain_debug_diagnostics must be boolean")
    if not graph.completed:
        return GlobalTrajectorySearchResult(
            completed=False,
            selected_path=(),
            cumulative_raw_displacement_cost_rad2=None,
            maximum_raw_single_joint_transition_rad=None,
            failed_sample_index=graph.failure.failed_sample_index,
            failure=graph.failure,
            evaluated_edges=(),
        )
    for layer in graph.layers:
        if not layer.nodes:
            failure = PlannerFailureDiagnostic(
                EMPTY_GRAPH_LAYER,
                layer.sample_index,
                "Global search encountered an empty graph layer",
            )
            return GlobalTrajectorySearchResult(
                False, (), None, None, layer.sample_index, failure, (),
            )
    if not graph.layers:
        failure = PlannerFailureDiagnostic(
            EMPTY_GRAPH_LAYER,
            0,
            "Global search requires at least one graph layer",
        )
        return GlobalTrajectorySearchResult(False, (), None, None, 0, failure, ())

    edges = build_consecutive_graph_edges(graph, edge_feasibility)
    edges_by_target: dict[tuple[int, int], list[TrajectoryGraphEdge]] = {}
    for edge in edges:
        edges_by_target.setdefault(
            (edge.to_sample_index, edge.to_node_index),
            [],
        ).append(edge)

    costs: dict[tuple[int, int], float] = {
        (0, node.node_index): 0.0 for node in graph.layers[0].nodes
    }
    predecessors: dict[tuple[int, int], int] = {}
    incoming_costs: dict[tuple[int, int], float] = {}

    def node_search_diagnostics() -> tuple[GlobalNodeSearchDiagnostic, ...]:
        if not retain_debug_diagnostics:
            return ()
        diagnostics = []
        for diagnostic_layer in graph.layers:
            for diagnostic_node in diagnostic_layer.nodes:
                key = (diagnostic_layer.sample_index, diagnostic_node.node_index)
                diagnostics.append(GlobalNodeSearchDiagnostic(
                    sample_index=diagnostic_layer.sample_index,
                    node_index=diagnostic_node.node_index,
                    reachable=key in costs,
                    selected_predecessor_node_index=predecessors.get(key),
                    local_edge_cost_rad2=(
                        0.0 if diagnostic_layer.sample_index == 0
                        else incoming_costs.get(key)
                    ),
                    cumulative_cost_rad2=costs.get(key),
                ))
        return tuple(diagnostics)

    for layer in graph.layers[1:]:
        reachable_count = 0
        for node in layer.nodes:
            target_key = (layer.sample_index, node.node_index)
            best_total: float | None = None
            best_predecessor: int | None = None
            best_edge_cost: float | None = None
            for edge in edges_by_target.get(target_key, ()):
                previous_key = (edge.from_sample_index, edge.from_node_index)
                if not edge.feasible or previous_key not in costs:
                    continue
                candidate_total = costs[previous_key] + edge.raw_displacement_cost_rad2
                if (
                    best_total is None
                    or candidate_total < best_total - tolerance
                    or (
                        abs(candidate_total - best_total) <= tolerance
                        and edge.from_node_index < best_predecessor  # type: ignore[operator]
                    )
                ):
                    best_total = candidate_total
                    best_predecessor = edge.from_node_index
                    best_edge_cost = edge.raw_displacement_cost_rad2
            if best_total is not None:
                costs[target_key] = best_total
                predecessors[target_key] = best_predecessor  # type: ignore[assignment]
                incoming_costs[target_key] = best_edge_cost  # type: ignore[assignment]
                reachable_count += 1
        if reachable_count == 0:
            reason = (
                NO_COMPLETE_GLOBAL_PATH
                if layer.sample_index == len(graph.layers) - 1
                else NO_FEASIBLE_INCOMING_EDGE
            )
            failure = PlannerFailureDiagnostic(
                reason,
                layer.sample_index,
                "No graph node has a feasible edge from a reachable predecessor",
            )
            return GlobalTrajectorySearchResult(
                False,
                (),
                None,
                None,
                layer.sample_index,
                failure,
                edges,
                node_search_diagnostics(),
            )

    final_layer = graph.layers[-1]
    reachable_final_nodes = tuple(
        node for node in final_layer.nodes
        if (final_layer.sample_index, node.node_index) in costs
    )
    final_node = reachable_final_nodes[0]
    for candidate in reachable_final_nodes[1:]:
        candidate_cost = costs[(final_layer.sample_index, candidate.node_index)]
        selected_cost = costs[(final_layer.sample_index, final_node.node_index)]
        if candidate_cost < selected_cost - tolerance or (
            abs(candidate_cost - selected_cost) <= tolerance
            and candidate.node_index < final_node.node_index
        ):
            final_node = candidate
    selected_indices = [final_node.node_index]
    for sample_index in range(final_layer.sample_index, 0, -1):
        selected_indices.append(predecessors[(sample_index, selected_indices[-1])])
    selected_indices.reverse()

    path: list[SelectedTrajectoryGraphPoint] = []
    maximum_step = 0.0
    for layer, node_index in zip(graph.layers, selected_indices):
        node = layer.nodes[node_index]
        key = (layer.sample_index, node_index)
        edge_cost = incoming_costs.get(key, 0.0)
        path.append(_path_point(layer, node, edge_cost, costs[key]))
        if layer.sample_index > 0:
            predecessor_index = selected_indices[layer.sample_index - 1]
            edge = next(
                item for item in edges
                if item.from_sample_index == layer.sample_index - 1
                and item.from_node_index == predecessor_index
                and item.to_sample_index == layer.sample_index
                and item.to_node_index == node_index
            )
            maximum_step = max(maximum_step, edge.max_raw_joint_step_rad)
    return GlobalTrajectorySearchResult(
        completed=True,
        selected_path=tuple(path),
        cumulative_raw_displacement_cost_rad2=path[-1].cumulative_cost_rad2,
        maximum_raw_single_joint_transition_rad=maximum_step,
        failed_sample_index=None,
        failure=None,
        evaluated_edges=edges,
        node_search_diagnostics=node_search_diagnostics(),
    )


def search_greedy_candidate_graph(
    graph: TrajectoryCandidateGraph,
    *,
    tie_tolerance: float = 1e-12,
    edge_feasibility: EdgeFeasibilityCallback = default_graph_edge_feasibility,
) -> GreedyTrajectorySearchResult:
    """Select the locally cheapest feasible outgoing edge as a baseline."""
    tolerance = _finite_number(tie_tolerance, "tie_tolerance")
    if tolerance < 0.0:
        raise ValueError("tie_tolerance must be non-negative")
    if not graph.completed or not graph.layers or any(not layer.nodes for layer in graph.layers):
        failure = graph.failure or PlannerFailureDiagnostic(
            EMPTY_GRAPH_LAYER,
            next((layer.sample_index for layer in graph.layers if not layer.nodes), 0),
            "Greedy search requires a complete non-empty graph",
        )
        return GreedyTrajectorySearchResult(
            False, (), None, None, failure.failed_sample_index, failure,
        )
    selected = graph.layers[0].nodes[0]
    cumulative = 0.0
    maximum_step = 0.0
    path = [_path_point(graph.layers[0], selected, 0.0, 0.0)]
    for layer in graph.layers[1:]:
        candidates = tuple(
            graph_edge_between(selected, node, edge_feasibility)
            for node in layer.nodes
        )
        feasible = tuple(edge for edge in candidates if edge.feasible)
        if not feasible:
            failure = PlannerFailureDiagnostic(
                NO_FEASIBLE_INCOMING_EDGE,
                layer.sample_index,
                "Greedy winner has no feasible outgoing edge",
            )
            return GreedyTrajectorySearchResult(
                False, tuple(path), None, maximum_step, layer.sample_index, failure,
            )
        best = feasible[0]
        for candidate in feasible[1:]:
            delta = candidate.raw_displacement_cost_rad2 - best.raw_displacement_cost_rad2
            if delta < -tolerance or (
                abs(delta) <= tolerance
                and candidate.to_node_index < best.to_node_index
            ):
                best = candidate
        selected = layer.nodes[best.to_node_index]
        cumulative += best.raw_displacement_cost_rad2
        maximum_step = max(maximum_step, best.max_raw_joint_step_rad)
        path.append(_path_point(layer, selected, best.raw_displacement_cost_rad2, cumulative))
    return GreedyTrajectorySearchResult(
        True, tuple(path), cumulative, maximum_step, None, None,
    )


def compare_greedy_and_global(
    graph: TrajectoryCandidateGraph,
    *,
    tie_tolerance: float = 1e-12,
    edge_feasibility: EdgeFeasibilityCallback = default_graph_edge_feasibility,
    retain_debug_diagnostics: bool = False,
) -> GreedyGlobalComparison:
    """Compare local Greedy and exact DP over the identical generated graph."""
    tolerance = _finite_number(tie_tolerance, "tie_tolerance")
    greedy = search_greedy_candidate_graph(
        graph,
        tie_tolerance=tolerance,
        edge_feasibility=edge_feasibility,
    )
    global_search = search_global_candidate_graph(
        graph,
        tie_tolerance=tolerance,
        edge_feasibility=edge_feasibility,
        retain_debug_diagnostics=retain_debug_diagnostics,
    )
    not_worse = None
    if greedy.completed and global_search.completed:
        not_worse = (
            global_search.cumulative_raw_displacement_cost_rad2
            <= greedy.cumulative_raw_displacement_cost_rad2 + tolerance
        )
    return GreedyGlobalComparison(greedy, global_search, not_worse, tolerance)
