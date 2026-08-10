"""Deterministic offline tests for the Phase 1E.1 transform core."""

from __future__ import annotations

import ast
import math
import unittest
from pathlib import Path

from dual_arm_app.backend.object_grasp_model import (
    HOMOGENEOUS_ROW_TOLERANCE,
    ORTHONORMALIZATION_AXIS_EPSILON,
    ROTATION_DETERMINANT_TOLERANCE,
    ROTATION_ORTHONORMAL_TOLERANCE,
    SYNTHETIC_OBJECT_GRASP_FIXTURE,
    SYNTHETIC_OBJECT_GRASP_FIXTURE_NOTICE,
    ObjectGraspModel,
    RigidTransform,
    check_rigid_grasp_invariance,
    compose,
    inverse,
    transform_point,
)


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "dual_arm_app/backend/object_grasp_model.py"


class TransformAssertionsMixin:
    def assertTransformAlmostEqual(
        self,
        actual: RigidTransform,
        expected: RigidTransform,
        tolerance: float = 1e-9,
    ) -> None:
        self.assertTrue(
            actual.almost_equal(expected, tolerance),
            f"{actual.matrix!r} != {expected.matrix!r}",
        )


class RigidTransformTests(TransformAssertionsMixin, unittest.TestCase):

    def test_identity_is_immutable_and_defensively_copied(self) -> None:
        identity = RigidTransform.identity()
        self.assertEqual(identity.matrix, (
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ))
        source = [list(row) for row in identity.matrix]
        copied = RigidTransform.from_matrix(source)
        source[0][3] = 99
        self.assertEqual(copied.translation_m, (0.0, 0.0, 0.0))
        self.assertIsInstance(copied.matrix, tuple)
        self.assertTrue(all(isinstance(row, tuple) for row in copied.matrix))

    def test_translation_only_and_point_transformation(self) -> None:
        transform = RigidTransform.from_translation_rpy(
            translation_m=(1.0, -2.0, 0.5),
            rpy_rad=(0.0, 0.0, 0.0),
        )
        self.assertEqual(transform.translation_m, (1.0, -2.0, 0.5))
        self.assertEqual(transform_point(transform, (0.25, 1.0, -0.5)), (
            1.25, -1.0, 0.0
        ))

    def test_ros_urdf_rpy_uses_rz_ry_rx_composition(self) -> None:
        yaw = RigidTransform.from_translation_rpy(
            translation_m=(0.0, 0.0, 0.0),
            rpy_rad=(0.0, 0.0, math.pi / 2),
        )
        transformed = transform_point(yaw, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(transformed[0], 0.0)
        self.assertAlmostEqual(transformed[1], 1.0)
        self.assertAlmostEqual(transformed[2], 0.0)

        roll, pitch, yaw_angle = 0.3, -0.4, 0.7
        combined = RigidTransform.from_translation_rpy(
            (0.0, 0.0, 0.0), (roll, pitch, yaw_angle)
        )
        separate = compose(
            RigidTransform.from_translation_rpy((0, 0, 0), (0, 0, yaw_angle)),
            compose(
                RigidTransform.from_translation_rpy((0, 0, 0), (0, pitch, 0)),
                RigidTransform.from_translation_rpy((0, 0, 0), (roll, 0, 0)),
            ),
        )
        self.assertTransformAlmostEqual(combined, separate)

    def test_composition_frame_order_and_inverse(self) -> None:
        A_T_B = RigidTransform.from_translation_rpy(
            (1.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2)
        )
        B_T_C = RigidTransform.from_translation_rpy(
            (1.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        )
        A_T_C = compose(A_T_B, B_T_C)
        expected = RigidTransform.from_translation_rpy(
            (1.0, 1.0, 0.0), (0.0, 0.0, math.pi / 2)
        )
        self.assertTransformAlmostEqual(A_T_C, expected)

        B_T_A = inverse(A_T_B)
        self.assertTransformAlmostEqual(compose(A_T_B, B_T_A), RigidTransform.identity())
        point_A = (0.2, -0.5, 1.7)
        point_B = transform_point(B_T_A, point_A)
        recovered = transform_point(A_T_B, point_B)
        for actual, wanted in zip(recovered, point_A):
            self.assertAlmostEqual(actual, wanted)

    def test_invalid_matrix_shapes_and_nonfinite_values_are_rejected(self) -> None:
        invalid = [
            [],
            [[1, 0, 0, 0]] * 3,
            [[1, 0, 0]] * 4,
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, math.nan],
                [0, 0, 0, 1],
            ],
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, math.inf],
                [0, 0, 0, 1],
            ],
            [
                [True, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
        ]
        for matrix in invalid:
            with self.subTest(matrix=matrix):
                with self.assertRaises(ValueError):
                    RigidTransform.from_matrix(matrix)

    def test_invalid_homogeneous_row_rotation_and_reflection_are_rejected(self) -> None:
        bad_last_row = [
            [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0.01, 0, 1]
        ]
        scaled_rotation = [
            [2, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]
        ]
        reflection = [
            [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]
        ]
        for matrix in (bad_last_row, scaled_rotation, reflection):
            with self.subTest(matrix=matrix):
                with self.assertRaises(ValueError):
                    RigidTransform.from_matrix(matrix)

    def test_small_normal_floating_noise_is_accepted(self) -> None:
        noisy = [
            [1.0 + 4e-7, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1.0 + HOMOGENEOUS_ROW_TOLERANCE * 0.5],
        ]
        transform = RigidTransform.from_matrix(noisy)
        self.assertAlmostEqual(transform.matrix[0][0], 1.0)
        self.assertEqual(ROTATION_ORTHONORMAL_TOLERANCE, 1e-6)
        self.assertEqual(ROTATION_DETERMINANT_TOLERANCE, 1e-6)
        self.assertEqual(ORTHONORMALIZATION_AXIS_EPSILON, 1e-12)

    def test_reproduced_accepted_noise_is_canonical_and_closed_under_inverse(self) -> None:
        transform = RigidTransform.from_matrix([
            [1.0000004, 0, 0, 0.25],
            [0, 1, 0, -0.10],
            [0, 0, 1, 0.50],
            [0, 0, 0, 1],
        ])
        self.assertEqual(transform.translation_m, (0.25, -0.10, 0.50))
        self.assertEqual(transform.matrix[0][0], 1.0)
        inverse_transform = inverse(transform)
        identity = compose(transform, inverse_transform)
        self.assertTransformAlmostEqual(identity, RigidTransform.identity(), 1e-12)

    def test_repeated_compose_inverse_chain_remains_stable(self) -> None:
        transform = RigidTransform.from_translation_rpy(
            (0.37, -0.22, 1.15),
            (0.43, -0.71, 1.08),
        )
        inverse_transform = inverse(transform)
        accumulated = RigidTransform.identity()
        for _ in range(100):
            accumulated = compose(accumulated, transform)
            accumulated = compose(accumulated, inverse_transform)
        self.assertTransformAlmostEqual(accumulated, RigidTransform.identity(), 1e-9)

        round_trip = transform
        for _ in range(50):
            round_trip = inverse(inverse(round_trip))
        self.assertTransformAlmostEqual(round_trip, transform, 1e-9)

    def test_nearly_collinear_rotation_axes_remain_invalid(self) -> None:
        nearly_collinear = [
            [1.0, 1.0, 0.0, 0.0],
            [0.0, 1e-14, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        with self.assertRaises(ValueError):
            RigidTransform.from_matrix(nearly_collinear)

    def test_invalid_points_and_ambiguous_operations_are_rejected(self) -> None:
        transform = RigidTransform.identity()
        for point in ((1, 2), (1, 2, 3, 4), (1, True, 3), (1, math.nan, 3)):
            with self.subTest(point=point):
                with self.assertRaises(ValueError):
                    transform_point(transform, point)
        with self.assertRaises(TypeError):
            compose(transform, "not a transform")
        with self.assertRaises(TypeError):
            inverse(None)


class ObjectGraspModelTests(TransformAssertionsMixin, unittest.TestCase):
    def setUp(self) -> None:
        self.object_T_left = RigidTransform.from_translation_rpy(
            (0.1, 0.3, 0.0), (0.0, 0.0, math.pi / 2)
        )
        self.object_T_right = RigidTransform.from_translation_rpy(
            (0.1, -0.3, 0.0), (0.0, 0.0, -math.pi / 2)
        )
        self.model = ObjectGraspModel(self.object_T_left, self.object_T_right)

    def test_model_stores_grasps_and_computes_both_world_target_equations(self) -> None:
        self.assertIs(self.model.object_T_left, self.object_T_left)
        self.assertIs(self.model.object_T_right, self.object_T_right)
        world_T_object = RigidTransform.from_translation_rpy(
            (1.0, 2.0, 3.0), (0.2, -0.1, 0.4)
        )
        targets = self.model.compute_world_grasp_targets(world_T_object)
        self.assertTransformAlmostEqual(
            targets.world_T_left,
            compose(world_T_object, self.object_T_left),
        )
        self.assertTransformAlmostEqual(
            targets.world_T_right,
            compose(world_T_object, self.object_T_right),
        )

    def test_left_to_right_is_derived_and_world_pose_invariant(self) -> None:
        expected = compose(inverse(self.object_T_left), self.object_T_right)
        self.assertTransformAlmostEqual(self.model.left_T_right(), expected)
        world_poses = (
            RigidTransform.identity(),
            RigidTransform.from_translation_rpy(
                (0.4, -1.2, 2.0), (0.1, 0.2, 0.3)
            ),
            RigidTransform.from_translation_rpy(
                (-2.5, 0.7, 0.25), (-0.8, 0.45, 1.7)
            ),
        )
        self.assertTrue(check_rigid_grasp_invariance(self.model, world_poses))
        for world_T_object in world_poses:
            targets = self.model.compute_world_grasp_targets(world_T_object)
            world_left_T_right = compose(
                inverse(targets.world_T_left), targets.world_T_right
            )
            self.assertTransformAlmostEqual(world_left_T_right, expected)

    def test_synthetic_fixture_is_simple_and_explicitly_not_calibrated(self) -> None:
        self.assertEqual(
            SYNTHETIC_OBJECT_GRASP_FIXTURE_NOTICE,
            "OFFLINE SYNTHETIC FIXTURE — NOT CALIBRATED FROM PHYSICAL ROBOT",
        )
        self.assertEqual(
            SYNTHETIC_OBJECT_GRASP_FIXTURE.object_T_left.translation_m,
            (0.0, 0.25, 0.0),
        )
        self.assertEqual(
            SYNTHETIC_OBJECT_GRASP_FIXTURE.object_T_right.translation_m,
            (0.0, -0.25, 0.0),
        )
        self.assertEqual(
            SYNTHETIC_OBJECT_GRASP_FIXTURE.left_T_right().translation_m,
            (0.0, -0.5, 0.0),
        )

    def test_module_has_only_standard_library_and_no_runtime_stack_imports(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imported_roots = {
            alias.name.split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        imported_roots.update(
            node.module.split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        )
        self.assertLessEqual(imported_roots, {"__future__", "math", "dataclasses", "typing"})
        for forbidden in ("rclpy", "moveit_msgs", "jaka_msgs", "numpy"):
            self.assertNotIn(forbidden, imported_roots)
        self.assertIn("R = Rz(yaw) * Ry(pitch) * Rx(roll)", source)
        self.assertIn("left_J6", source)
        self.assertIn("right_J6", source)
        self.assertIn("not asserted to be calibrated physical hardware TCPs", source)


if __name__ == "__main__":
    unittest.main()
