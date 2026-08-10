"""ROS-independent rigid-transform core for offline dual-arm grasp planning.

Transform notation follows ``^A T_B``: the pose of frame B expressed in frame
A. It maps a point from B coordinates to A coordinates as
``^A p = ^A T_B * ^B p``. Translation is always meters and rotation is stored
as a 3x3 proper rotation matrix inside a 4x4 homogeneous transform.

The current offline planning/grasp tips are conceptually ``left_J6`` and
``right_J6``. They are not asserted to be calibrated physical hardware TCPs;
tooling and installation calibration remain outside this mathematical core.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence


HOMOGENEOUS_ROW_TOLERANCE = 1e-9
ROTATION_ORTHONORMAL_TOLERANCE = 1e-6
ROTATION_DETERMINANT_TOLERANCE = 1e-6
DEFAULT_COMPARISON_TOLERANCE = 1e-9
# Separate from acceptance tolerances: avoids division by a numerically zero
# axis during Gram-Schmidt without making invalid rotations easier to accept.
ORTHONORMALIZATION_AXIS_EPSILON = 1e-12

Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]
Vector3 = tuple[float, float, float]


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be a finite number")
    return converted


def _vector3(values: Any, label: str) -> Vector3:
    if (
        isinstance(values, (str, bytes, bool))
        or not isinstance(values, Sequence)
        or len(values) != 3
    ):
        raise ValueError(f"{label} must contain exactly three finite numbers")
    return tuple(
        _finite_number(value, f"{label}[{index}]")
        for index, value in enumerate(values)
    )  # type: ignore[return-value]


def _determinant3(rotation: Sequence[Sequence[float]]) -> float:
    return (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )


def _dot3(left: Vector3, right: Vector3) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _normalize3(vector: Vector3, label: str) -> Vector3:
    norm = math.sqrt(_dot3(vector, vector))
    if not math.isfinite(norm) or norm <= ORTHONORMALIZATION_AXIS_EPSILON:
        raise ValueError(f"rotation {label} axis is degenerate")
    return tuple(value / norm for value in vector)  # type: ignore[return-value]


def _cross3(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _orthonormalize_rotation_columns(
    rotation: Sequence[Sequence[float]],
) -> tuple[Vector3, Vector3, Vector3]:
    """Canonicalize an already-accepted near-SO(3) rotation.

    For the column-vector convention, matrix columns are the rotated X/Y/Z
    basis axes. The first column is normalized; the second is Gram-Schmidt
    orthogonalized against it; and the third is their normalized cross product,
    which enforces a right-handed determinant of +1. This helper is called only
    after the original matrix passes the existing near-orthonormal and positive
    determinant checks, so it never repairs a grossly invalid rotation.
    """
    original_x: Vector3 = tuple(rotation[row][0] for row in range(3))  # type: ignore[assignment]
    original_y: Vector3 = tuple(rotation[row][1] for row in range(3))  # type: ignore[assignment]
    original_z: Vector3 = tuple(rotation[row][2] for row in range(3))  # type: ignore[assignment]
    axis_x = _normalize3(original_x, "first")
    projection = _dot3(original_y, axis_x)
    orthogonal_y: Vector3 = tuple(
        original_y[index] - projection * axis_x[index]
        for index in range(3)
    )  # type: ignore[assignment]
    axis_y = _normalize3(orthogonal_y, "second")
    axis_z = _normalize3(_cross3(axis_x, axis_y), "third")
    if _dot3(axis_z, original_z) <= ORTHONORMALIZATION_AXIS_EPSILON:
        raise ValueError("rotation third axis is inconsistent with a right-handed basis")
    return axis_x, axis_y, axis_z


def _validated_matrix(matrix: Any) -> Matrix4:
    if (
        isinstance(matrix, (str, bytes, bool))
        or not isinstance(matrix, Sequence)
        or len(matrix) != 4
    ):
        raise ValueError("homogeneous transform must have exactly four rows")

    rows: list[tuple[float, float, float, float]] = []
    for row_index, row in enumerate(matrix):
        if (
            isinstance(row, (str, bytes, bool))
            or not isinstance(row, Sequence)
            or len(row) != 4
        ):
            raise ValueError(
                f"homogeneous transform row {row_index} must have four values"
            )
        rows.append(
            tuple(
                _finite_number(value, f"matrix[{row_index}][{column_index}]")
                for column_index, value in enumerate(row)
            )  # type: ignore[arg-type]
        )

    expected_last_row = (0.0, 0.0, 0.0, 1.0)
    if any(
        abs(actual - expected) > HOMOGENEOUS_ROW_TOLERANCE
        for actual, expected in zip(rows[3], expected_last_row)
    ):
        raise ValueError("homogeneous transform last row must be [0, 0, 0, 1]")
    rows[3] = expected_last_row

    rotation = tuple(tuple(rows[row][column] for column in range(3)) for row in range(3))
    for row in range(3):
        for column in range(3):
            dot_product = sum(
                rotation[index][row] * rotation[index][column]
                for index in range(3)
            )
            expected = 1.0 if row == column else 0.0
            if abs(dot_product - expected) > ROTATION_ORTHONORMAL_TOLERANCE:
                raise ValueError("rotation matrix must be orthonormal")

    determinant = _determinant3(rotation)
    if abs(determinant - 1.0) > ROTATION_DETERMINANT_TOLERANCE:
        raise ValueError("rotation matrix determinant must be approximately +1")

    # Store a canonical proper rotation rather than accepted numerical noise.
    # Translation values are copied unchanged from the validated input.
    axis_x, axis_y, axis_z = _orthonormalize_rotation_columns(rotation)
    canonical_rotation = tuple(
        (axis_x[row], axis_y[row], axis_z[row])
        for row in range(3)
    )
    canonical_determinant = _determinant3(canonical_rotation)
    if abs(canonical_determinant - 1.0) > ROTATION_DETERMINANT_TOLERANCE:
        raise ValueError("canonical rotation determinant must be approximately +1")
    for row in range(3):
        rows[row] = (*canonical_rotation[row], rows[row][3])
    return tuple(rows)  # type: ignore[return-value]


def _comparison_tolerance(value: Any) -> float:
    tolerance = _finite_number(value, "tolerance")
    if tolerance < 0:
        raise ValueError("tolerance must be non-negative")
    return tolerance


@dataclass(frozen=True)
class RigidTransform:
    """Effectively immutable 4x4 proper rigid transform.

    Matrix translation entries are meters. Input rotations may contain small
    floating-point noise, but only matrices already within the acceptance
    tolerances are canonicalized. Stored rotations are proper orthonormal
    matrices, which keeps inverse and composition closed over accepted inputs.
    Arbitrary invalid matrices are never repaired. Use
    :meth:`from_translation_rpy` for ROS-URDF-style RPY input.
    """

    _matrix: Matrix4 = field(repr=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "_matrix", _validated_matrix(self._matrix))

    @classmethod
    def identity(cls) -> "RigidTransform":
        """Return ``^A T_A``, with zero translation and identity rotation."""
        return cls(
            (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )

    @classmethod
    def from_matrix(cls, matrix: Any) -> "RigidTransform":
        """Validate, canonicalize, and defensively copy a near-rigid matrix."""
        return cls(matrix)  # type: ignore[arg-type]

    @classmethod
    def from_translation_rpy(
        cls,
        translation_m: Sequence[float],
        rpy_rad: Sequence[float],
    ) -> "RigidTransform":
        """Construct a transform from meters and ROS URDF RPY radians.

        ``roll``, ``pitch``, and ``yaw`` are fixed-axis rotations about X, Y,
        and Z. For column vectors the exact composition is
        ``R = Rz(yaw) * Ry(pitch) * Rx(roll)``. This convention is used
        consistently; no intrinsic Euler convention is mixed into the core.
        """
        x, y, z = _vector3(translation_m, "translation_m")
        roll, pitch, yaw = _vector3(rpy_rad, "rpy_rad")
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
        return cls(
            (
                (*rotation[0], x),
                (*rotation[1], y),
                (*rotation[2], z),
                (0.0, 0.0, 0.0, 1.0),
            )
        )

    @property
    def matrix(self) -> Matrix4:
        """Return an immutable defensive copy of the homogeneous matrix."""
        return tuple(tuple(row) for row in self._matrix)  # type: ignore[return-value]

    @property
    def translation_m(self) -> Vector3:
        """Return translation in meters."""
        return (self._matrix[0][3], self._matrix[1][3], self._matrix[2][3])

    def almost_equal(
        self,
        other: "RigidTransform",
        tolerance: float = DEFAULT_COMPARISON_TOLERANCE,
    ) -> bool:
        """Compare all homogeneous-matrix entries within an absolute tolerance."""
        if not isinstance(other, RigidTransform):
            return False
        checked_tolerance = _comparison_tolerance(tolerance)
        return all(
            abs(self._matrix[row][column] - other._matrix[row][column])
            <= checked_tolerance
            for row in range(4)
            for column in range(4)
        )


def compose(A_T_B: RigidTransform, B_T_C: RigidTransform) -> RigidTransform:
    """Return ``^A T_C = ^A T_B * ^B T_C``.

    Inputs and output use meters for translation and rotation matrices. Frame
    compatibility is a caller naming/semantic responsibility because runtime
    frame identifiers are intentionally not embedded in the math type.
    """
    if not isinstance(A_T_B, RigidTransform) or not isinstance(B_T_C, RigidTransform):
        raise TypeError("compose requires two RigidTransform instances")
    left, right = A_T_B._matrix, B_T_C._matrix
    product = tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        )
        for row in range(4)
    )
    return RigidTransform(product)  # type: ignore[arg-type]


def inverse(A_T_B: RigidTransform) -> RigidTransform:
    """Return ``^B T_A``, the rigid inverse of input ``^A T_B``.

    The proper rotation is transposed and translation is ``-R^T t`` in meters.
    """
    if not isinstance(A_T_B, RigidTransform):
        raise TypeError("inverse requires a RigidTransform")
    matrix = A_T_B._matrix
    rotation_transpose = tuple(
        tuple(matrix[column][row] for column in range(3))
        for row in range(3)
    )
    translation = (matrix[0][3], matrix[1][3], matrix[2][3])
    inverse_translation = tuple(
        -sum(rotation_transpose[row][index] * translation[index] for index in range(3))
        for row in range(3)
    )
    return RigidTransform(
        (
            (*rotation_transpose[0], inverse_translation[0]),
            (*rotation_transpose[1], inverse_translation[1]),
            (*rotation_transpose[2], inverse_translation[2]),
            (0.0, 0.0, 0.0, 1.0),
        )
    )


def transform_point(A_T_B: RigidTransform, point_B_m: Sequence[float]) -> Vector3:
    """Return ``^A p = ^A T_B * ^B p`` for a 3D point measured in meters."""
    if not isinstance(A_T_B, RigidTransform):
        raise TypeError("transform_point requires a RigidTransform")
    x, y, z = _vector3(point_B_m, "point_B_m")
    point = (x, y, z)
    return tuple(
        sum(A_T_B._matrix[row][column] * point[column] for column in range(3))
        + A_T_B._matrix[row][3]
        for row in range(3)
    )  # type: ignore[return-value]


@dataclass(frozen=True)
class WorldGraspTargets:
    """Derived world poses for the left and right planning/grasp tip frames."""

    world_T_left: RigidTransform
    world_T_right: RigidTransform


@dataclass(frozen=True)
class ObjectGraspModel:
    """Constant object-relative left/right grasp relationships.

    ``object_T_left`` is ``^O T_L`` and ``object_T_right`` is ``^O T_R``.
    They are mathematical planning-frame relationships, not physical TCP
    calibration measurements.
    """

    object_T_left: RigidTransform
    object_T_right: RigidTransform

    def __post_init__(self) -> None:
        if not isinstance(self.object_T_left, RigidTransform):
            raise TypeError("object_T_left must be a RigidTransform")
        if not isinstance(self.object_T_right, RigidTransform):
            raise TypeError("object_T_right must be a RigidTransform")

    def compute_world_grasp_targets(
        self,
        world_T_object: RigidTransform,
    ) -> WorldGraspTargets:
        """Derive world planning-tip targets from an object pose.

        Input ``world_T_object`` is ``^W T_O``. The returned transforms apply
        exactly ``^W T_L = ^W T_O * ^O T_L`` and
        ``^W T_R = ^W T_O * ^O T_R``. Translation is meters. This does not
        perform IK, collision checking, trajectory generation, or execution.
        """
        if not isinstance(world_T_object, RigidTransform):
            raise TypeError("world_T_object must be a RigidTransform")
        return WorldGraspTargets(
            world_T_left=compose(world_T_object, self.object_T_left),
            world_T_right=compose(world_T_object, self.object_T_right),
        )

    def left_T_right(self) -> RigidTransform:
        """Derive constant ``^L T_R = inverse(^O T_L) * ^O T_R``.

        The relative grasp is derived rather than stored, so it cannot diverge
        from the two object-relative sources of truth.
        """
        return compose(inverse(self.object_T_left), self.object_T_right)


def check_rigid_grasp_invariance(
    model: ObjectGraspModel,
    world_T_object_poses: Iterable[RigidTransform],
    tolerance: float = DEFAULT_COMPARISON_TOLERANCE,
) -> bool:
    """Check rigid-grasp invariance for multiple object poses.

    For every supplied ``^W T_O``, this verifies
    ``inverse(^W T_L) * ^W T_R == inverse(^O T_L) * ^O T_R`` within tolerance.
    Therefore the result is independent of world object pose. This is a
    transform-consistency check only, not IK, collision, calibration, or motion
    validation.
    """
    if not isinstance(model, ObjectGraspModel):
        raise TypeError("model must be an ObjectGraspModel")
    checked_tolerance = _comparison_tolerance(tolerance)
    object_relative = model.left_T_right()
    pose_count = 0
    for world_T_object in world_T_object_poses:
        pose_count += 1
        targets = model.compute_world_grasp_targets(world_T_object)
        world_relative = compose(
            inverse(targets.world_T_left), targets.world_T_right
        )
        if not world_relative.almost_equal(object_relative, checked_tolerance):
            return False
    if pose_count == 0:
        raise ValueError("at least one world_T_object pose is required")
    return True


SYNTHETIC_OBJECT_GRASP_FIXTURE_NOTICE = (
    "OFFLINE SYNTHETIC FIXTURE — NOT CALIBRATED FROM PHYSICAL ROBOT"
)
SYNTHETIC_OBJECT_GRASP_FIXTURE = ObjectGraspModel(
    object_T_left=RigidTransform.from_translation_rpy(
        translation_m=(0.0, 0.25, 0.0),
        rpy_rad=(0.0, 0.0, 0.0),
    ),
    object_T_right=RigidTransform.from_translation_rpy(
        translation_m=(0.0, -0.25, 0.0),
        rpy_rad=(0.0, 0.0, 0.0),
    ),
)
