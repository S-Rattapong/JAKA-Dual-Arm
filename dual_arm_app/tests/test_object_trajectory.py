"""Deterministic offline tests for the Phase 1F.1 object trajectory core."""

from __future__ import annotations

import ast
import math
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

from dual_arm_app.backend.object_grasp_model import (
    SYNTHETIC_OBJECT_GRASP_FIXTURE,
    RigidTransform,
    compose,
    inverse,
)
from dual_arm_app.backend.object_trajectory import (
    SYNTHETIC_OBJECT_TRAJECTORY_NOTICE,
    SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
    ObjectTrajectory,
    ObjectTrajectorySample,
    generate_translation_only_object_trajectory,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "dual_arm_app/backend/object_trajectory.py"


class ObjectTrajectoryTests(unittest.TestCase):
    def generate(
        self,
        *,
        start: RigidTransform | None = None,
        end_translation_m=(1.0, 2.0, 3.0),
        duration_s=4.0,
        sample_count=5,
        name="Deterministic Translation",
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
    ) -> ObjectTrajectory:
        return generate_translation_only_object_trajectory(
            name=name,
            start_world_T_object=start or RigidTransform.identity(),
            end_translation_m=end_translation_m,
            duration_s=duration_s,
            sample_count=sample_count,
            grasp_model=grasp_model,
        )

    def test_endpoint_inclusive_alphas_times_and_linear_translations(self) -> None:
        trajectory = self.generate()
        self.assertEqual(trajectory.sample_count, 5)
        self.assertEqual(
            [sample.alpha for sample in trajectory.samples],
            [0.0, 0.25, 0.5, 0.75, 1.0],
        )
        self.assertEqual(
            [sample.time_from_start_s for sample in trajectory.samples],
            [0.0, 1.0, 2.0, 3.0, 4.0],
        )
        self.assertEqual(trajectory.samples[0].world_T_object.translation_m, (0, 0, 0))
        self.assertEqual(trajectory.samples[2].world_T_object.translation_m, (0.5, 1, 1.5))
        self.assertEqual(trajectory.samples[-1].world_T_object.translation_m, (1, 2, 3))
        self.assertTrue(all(
            current.time_from_start_s < following.time_from_start_s
            for current, following in zip(trajectory.samples, trajectory.samples[1:])
        ))

    def test_nonzero_start_orientation_is_constant_and_copied_directly(self) -> None:
        start = RigidTransform.from_translation_rpy(
            (0.2, -0.4, 0.7),
            (0.31, -0.47, 0.82),
        )
        trajectory = self.generate(
            start=start,
            end_translation_m=(-0.6, 0.8, 1.4),
            sample_count=9,
        )
        start_rotation = tuple(row[:3] for row in start.matrix[:3])
        for sample in trajectory.samples:
            self.assertEqual(
                tuple(row[:3] for row in sample.world_T_object.matrix[:3]),
                start_rotation,
            )
        self.assertTrue(trajectory.has_constant_object_orientation(start, 1e-12))
        self.assertFalse(trajectory.has_constant_object_orientation(
            RigidTransform.identity(),
            1e-12,
        ))

    def test_left_and_right_targets_are_derived_from_each_object_sample(self) -> None:
        start = RigidTransform.from_translation_rpy(
            (-0.2, 0.3, 0.9),
            (0.2, -0.1, 0.4),
        )
        trajectory = self.generate(start=start, end_translation_m=(0.7, -0.5, 1.1))
        for sample in trajectory.samples:
            expected = SYNTHETIC_OBJECT_GRASP_FIXTURE.compute_world_grasp_targets(
                sample.world_T_object
            )
            self.assertTrue(sample.world_T_left.almost_equal(expected.world_T_left))
            self.assertTrue(sample.world_T_right.almost_equal(expected.world_T_right))

    def test_left_right_relative_transform_is_invariant_for_every_sample(self) -> None:
        trajectory = self.generate(
            start=RigidTransform.from_translation_rpy(
                (0.1, 0.2, 0.3),
                (-0.4, 0.25, 0.6),
            ),
            end_translation_m=(0.9, -0.7, 1.2),
            sample_count=21,
        )
        expected = SYNTHETIC_OBJECT_GRASP_FIXTURE.left_T_right()
        for sample in trajectory.samples:
            actual = compose(inverse(sample.world_T_left), sample.world_T_right)
            self.assertTrue(actual.almost_equal(expected))

    def test_zero_displacement_is_valid_and_keeps_every_position_constant(self) -> None:
        start = RigidTransform.from_translation_rpy(
            (0.25, -0.5, 1.25),
            (0.2, 0.3, -0.4),
        )
        trajectory = self.generate(
            start=start,
            end_translation_m=start.translation_m,
            duration_s=2.5,
            sample_count=6,
        )
        self.assertEqual(trajectory.sample_count, 6)
        self.assertTrue(all(
            sample.world_T_object.translation_m == start.translation_m
            for sample in trajectory.samples
        ))
        self.assertTrue(trajectory.has_constant_object_orientation(start))

    def test_duration_validation_rejects_nonpositive_nonfinite_bool_and_wrong_type(self) -> None:
        for duration in (0, -1, math.nan, math.inf, -math.inf, True, False, "4"):
            with self.subTest(duration=duration):
                with self.assertRaises((TypeError, ValueError)):
                    self.generate(duration_s=duration)

    def test_sample_count_validation_rejects_small_noninteger_and_bool(self) -> None:
        for sample_count in (0, 1, -2, 2.0, 3.5, True, False, "5"):
            with self.subTest(sample_count=sample_count):
                with self.assertRaises((TypeError, ValueError)):
                    self.generate(sample_count=sample_count)

    def test_end_translation_validation_rejects_shape_type_and_nonfinite_values(self) -> None:
        invalid_translations = (
            (),
            (1, 2),
            (1, 2, 3, 4),
            "1 2 3",
            (1, 2, True),
            (1, 2, "3"),
            (1, math.nan, 3),
            (1, math.inf, 3),
            (1, -math.inf, 3),
        )
        for translation in invalid_translations:
            with self.subTest(translation=translation):
                with self.assertRaises((TypeError, ValueError)):
                    self.generate(end_translation_m=translation)

    def test_wrong_transform_grasp_model_and_name_are_rejected(self) -> None:
        with self.assertRaises(TypeError):
            generate_translation_only_object_trajectory(
                "Bad Start", None, (1, 2, 3), 1, 2, SYNTHETIC_OBJECT_GRASP_FIXTURE
            )
        with self.assertRaises(TypeError):
            self.generate(grasp_model=None)
        for name in ("", "   ", None, 123, True):
            with self.subTest(name=name):
                with self.assertRaises((TypeError, ValueError)):
                    self.generate(name=name)

    def test_data_structures_are_frozen_and_samples_are_an_immutable_tuple(self) -> None:
        trajectory = self.generate()
        self.assertIsInstance(trajectory.samples, tuple)
        self.assertTrue(trajectory.translation_only)
        self.assertIsInstance(trajectory.samples[0], ObjectTrajectorySample)
        with self.assertRaises(FrozenInstanceError):
            trajectory.duration_s = 99  # type: ignore[misc]
        with self.assertRaises(FrozenInstanceError):
            trajectory.samples[0].alpha = 99  # type: ignore[misc]

    def test_synthetic_fixture_is_deterministic_and_explicitly_non_executable(self) -> None:
        trajectory = SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY
        self.assertEqual(
            SYNTHETIC_OBJECT_TRAJECTORY_NOTICE,
            "OFFLINE SYNTHETIC OBJECT TRAJECTORY — NOT FOR ROBOT EXECUTION",
        )
        self.assertEqual(trajectory.name, SYNTHETIC_OBJECT_TRAJECTORY_NOTICE)
        self.assertEqual(trajectory.duration_s, 4.0)
        self.assertEqual(trajectory.sample_count, 5)
        self.assertEqual(
            trajectory.samples[0].world_T_object.translation_m,
            (0.0, 0.0, 0.8),
        )
        self.assertEqual(
            trajectory.samples[-1].world_T_object.translation_m,
            (0.4, 0.0, 1.0),
        )
        self.assertTrue(trajectory.has_constant_object_orientation())

    def test_module_is_dependency_free_and_documents_equations_and_limitations(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imports = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        imports.update(
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        )
        for forbidden in ("rclpy", "moveit_msgs", "jaka_msgs", "numpy"):
            with self.subTest(forbidden=forbidden):
                self.assertFalse(any(
                    name == forbidden or name.startswith(f"{forbidden}.")
                    for name in imports
                ))
        for documentation in (
            "alpha_i = i / (N - 1)",
            "t_i = alpha_i * T",
            "p_i = (1 - alpha_i) * p0 + alpha_i * p1",
            "^W T_L(i) = ^W T_O(i) * ^O T_L",
            "^W T_R(i) = ^W T_O(i) * ^O T_R",
            "PHASE 1F.1 OUTPUT IS NOT A ROBOT JOINT TRAJECTORY",
            "IK feasibility",
            "collision freedom",
            "executable robot motion",
        ):
            with self.subTest(documentation=documentation):
                self.assertIn(documentation, source)


if __name__ == "__main__":
    unittest.main()
