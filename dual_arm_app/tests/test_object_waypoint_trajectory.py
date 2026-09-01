"""Phase 3A tests for immutable object waypoints and common timeline generation."""

from __future__ import annotations

import math
import unittest
from dataclasses import FrozenInstanceError, fields

from dual_arm_app.backend.object_grasp_model import (
    SYNTHETIC_OBJECT_GRASP_FIXTURE,
    RigidTransform,
    compose,
    inverse,
)
from dual_arm_app.backend.object_trajectory import (
    MultiWaypointObjectTrajectory,
    ObjectWaypoint,
    generate_multi_waypoint_object_trajectory,
)


class ObjectWaypointTrajectoryTests(unittest.TestCase):
    @staticmethod
    def waypoints() -> tuple[ObjectWaypoint, ...]:
        return (
            ObjectWaypoint("W0", (0.0, 0.0, 0.8)),
            ObjectWaypoint("W1", (0.6, 0.0, 1.0)),
            ObjectWaypoint("W2", (0.6, -0.9, 1.2)),
            ObjectWaypoint("W3", (-0.2, -0.9, 1.1)),
        )

    def generate(self) -> MultiWaypointObjectTrajectory:
        return generate_multi_waypoint_object_trajectory(
            name="Phase 3A Multi-Waypoint",
            waypoints=self.waypoints(),
            fixed_object_orientation=RigidTransform.from_translation_rpy(
                (99.0, 98.0, 97.0),
                (0.2, -0.3, 0.4),
            ),
            segment_durations_s=(2.0, 3.0, 1.0),
            segment_sample_counts=(3, 4, 2),
            grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        )

    def test_waypoint_is_validated_immutable_and_contains_no_robot_joints(self) -> None:
        waypoint = ObjectWaypoint("object_pick-1", [1, -2.5, 3])
        self.assertEqual(waypoint.identifier, "object_pick-1")
        self.assertEqual(waypoint.translation_m, (1.0, -2.5, 3.0))
        self.assertEqual(
            tuple(item.name for item in fields(ObjectWaypoint)),
            ("identifier", "translation_m", "rpy_rad"),
        )
        with self.assertRaises(FrozenInstanceError):
            waypoint.translation_m = (0, 0, 0)  # type: ignore[misc]

    def test_waypoint_accepts_optional_orientation_and_rejects_invalid_rpy(self) -> None:
        waypoint = ObjectWaypoint("W6D", (1, 2, 3), (0.1, -0.2, 0.3))
        self.assertEqual(waypoint.rpy_rad, (0.1, -0.2, 0.3))
        self.assertIsNone(ObjectWaypoint("legacy", (0, 0, 0)).rpy_rad)
        for invalid in ((1, 2), (1, 2, 3, 4), (0, math.nan, 0), (0, True, 0)):
            with self.subTest(invalid=invalid):
                with self.assertRaises((TypeError, ValueError)):
                    ObjectWaypoint("bad", (0, 0, 0), invalid)  # type: ignore[arg-type]

    def test_waypoint_rejects_invalid_identifier_shape_bool_and_nonfinite(self) -> None:
        invalid_identifiers = ("", "   ", "bad id", "@bad", None, True, 3)
        for identifier in invalid_identifiers:
            with self.subTest(identifier=identifier):
                with self.assertRaises((TypeError, ValueError)):
                    ObjectWaypoint(identifier, (0, 0, 0))  # type: ignore[arg-type]
        invalid_translations = (
            (), (1, 2), (1, 2, 3, 4), "1,2,3", (1, True, 3),
            (1, math.nan, 3), (1, math.inf, 3), (1, -math.inf, 3),
        )
        for translation in invalid_translations:
            with self.subTest(translation=translation):
                with self.assertRaises((TypeError, ValueError)):
                    ObjectWaypoint("W", translation)  # type: ignore[arg-type]

    def test_multi_waypoint_timeline_is_chronological_and_deduplicates_boundaries(self) -> None:
        trajectory = self.generate()
        self.assertEqual(trajectory.sample_count, 7)
        self.assertEqual(
            [sample.sample_index for sample in trajectory.samples],
            list(range(7)),
        )
        self.assertEqual(
            [sample.time_from_start_s for sample in trajectory.samples],
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        )
        self.assertTrue(all(
            current.time_from_start_s < following.time_from_start_s
            for current, following in zip(trajectory.samples, trajectory.samples[1:])
        ))
        translations = [sample.world_T_object.translation_m for sample in trajectory.samples]
        self.assertEqual(translations.count(self.waypoints()[1].translation_m), 1)
        self.assertEqual(translations.count(self.waypoints()[2].translation_m), 1)
        self.assertEqual(translations[2], self.waypoints()[1].translation_m)
        self.assertEqual(translations[5], self.waypoints()[2].translation_m)

    def test_exact_waypoint_boundary_poses_and_cumulative_times_are_preserved(self) -> None:
        trajectory = self.generate()
        boundary_indices = (0, 2, 5, 6)
        boundary_times = (0.0, 2.0, 5.0, 6.0)
        for waypoint, sample_index, expected_time in zip(
            self.waypoints(), boundary_indices, boundary_times
        ):
            sample = trajectory.samples[sample_index]
            self.assertEqual(sample.world_T_object.translation_m, waypoint.translation_m)
            self.assertEqual(sample.time_from_start_s, expected_time)

    def test_constant_nonzero_orientation_and_common_derived_targets(self) -> None:
        trajectory = self.generate()
        expected_rotation = tuple(
            row[:3] for row in trajectory.fixed_object_orientation.matrix[:3]
        )
        expected_relative = SYNTHETIC_OBJECT_GRASP_FIXTURE.left_T_right()
        for sample in trajectory.samples:
            self.assertEqual(
                tuple(row[:3] for row in sample.world_T_object.matrix[:3]),
                expected_rotation,
            )
            targets = SYNTHETIC_OBJECT_GRASP_FIXTURE.compute_world_grasp_targets(
                sample.world_T_object
            )
            self.assertTrue(sample.world_T_left.almost_equal(targets.world_T_left))
            self.assertTrue(sample.world_T_right.almost_equal(targets.world_T_right))
            self.assertTrue(compose(
                inverse(sample.world_T_left), sample.world_T_right
            ).almost_equal(expected_relative))
        self.assertTrue(trajectory.has_constant_object_orientation())

    def test_6d_waypoint_orientation_uses_shortest_arc_slerp_and_preserves_rigid_grasp(self) -> None:
        deg = math.pi / 180.0
        waypoints = (
            ObjectWaypoint("W0", (0.0, 0.0, 0.8), (0.0, 0.0, 170.0 * deg)),
            ObjectWaypoint("W1", (0.2, 0.0, 0.8), (0.0, 0.0, -170.0 * deg)),
        )
        trajectory = generate_multi_waypoint_object_trajectory(
            name="6D shortest arc",
            waypoints=waypoints,
            fixed_object_orientation=RigidTransform.identity(),
            segment_durations_s=(2.0,),
            segment_sample_counts=(3,),
            grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        )
        self.assertFalse(trajectory.translation_only)
        midpoint = trajectory.samples[1].world_T_object.matrix
        self.assertAlmostEqual(midpoint[0][0], -1.0, places=9)
        self.assertAlmostEqual(midpoint[1][1], -1.0, places=9)
        self.assertAlmostEqual(midpoint[0][1], 0.0, places=9)
        expected_relative = SYNTHETIC_OBJECT_GRASP_FIXTURE.left_T_right()
        for sample in trajectory.samples:
            actual = compose(inverse(sample.world_T_left), sample.world_T_right)
            self.assertTrue(actual.almost_equal(expected_relative, 1e-9))
        self.assertFalse(trajectory.has_constant_object_orientation())

    def test_6d_waypoint_endpoints_preserve_requested_rotations_exactly(self) -> None:
        waypoints = (
            ObjectWaypoint("W0", (0.0, 0.0, 0.8), (0.1, -0.2, 0.3)),
            ObjectWaypoint("W1", (0.1, 0.2, 0.9), (-0.4, 0.2, -0.1)),
        )
        trajectory = generate_multi_waypoint_object_trajectory(
            name="6D endpoints", waypoints=waypoints,
            fixed_object_orientation=RigidTransform.identity(),
            segment_durations_s=(1.0,), segment_sample_counts=(3,),
            grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        )
        for sample, waypoint in ((trajectory.samples[0], waypoints[0]), (trajectory.samples[-1], waypoints[-1])):
            expected = RigidTransform.from_translation_rpy(waypoint.translation_m, waypoint.rpy_rad)
            self.assertTrue(sample.world_T_object.almost_equal(expected, 1e-12))

    def test_trajectory_metadata_is_immutable_and_explicitly_preview_only(self) -> None:
        trajectory = self.generate()
        self.assertEqual(trajectory.segment_durations_s, (2.0, 3.0, 1.0))
        self.assertEqual(trajectory.segment_sample_counts, (3, 4, 2))
        self.assertEqual(trajectory.waypoints, self.waypoints())
        self.assertIn("PLANNING/PREVIEW", trajectory.timing_semantic)
        self.assertIn("NOT DYNAMICALLY EXECUTABLE", trajectory.timing_semantic)
        with self.assertRaises(FrozenInstanceError):
            trajectory.duration_s = 9  # type: ignore[misc]

    def test_duplicate_identifiers_and_invalid_segment_configuration_are_rejected(self) -> None:
        base = dict(
            name="Invalid",
            fixed_object_orientation=RigidTransform.identity(),
            grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        )
        with self.assertRaises(ValueError):
            generate_multi_waypoint_object_trajectory(
                **base,
                waypoints=(ObjectWaypoint("W0", (0, 0, 0)), ObjectWaypoint("w0", (1, 0, 0))),
                segment_durations_s=(1,),
                segment_sample_counts=(2,),
            )
        invalid_configs = (
            ((1,), (2, 2)),
            ((1, 2), (2,)),
            ((0,), (2,)),
            ((math.inf,), (2,)),
            ((1,), (1,)),
            ((1,), (2.5,)),
            ((1,), (True,)),
        )
        for durations, counts in invalid_configs:
            with self.subTest(durations=durations, counts=counts):
                with self.assertRaises((TypeError, ValueError)):
                    generate_multi_waypoint_object_trajectory(
                        **base,
                        waypoints=(
                            ObjectWaypoint("W0", (0, 0, 0)),
                            ObjectWaypoint("W1", (1, 0, 0)),
                        ),
                        segment_durations_s=durations,
                        segment_sample_counts=counts,
                    )


if __name__ == "__main__":
    unittest.main()
