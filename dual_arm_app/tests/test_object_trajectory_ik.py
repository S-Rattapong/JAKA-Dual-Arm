"""Offline tests for sequential IK and Phase 1G.3A raw continuity analysis."""

from __future__ import annotations

import ast
import math
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory import (
    SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
)
from dual_arm_app.backend.object_trajectory_ik import (
    ABSOLUTE_STEP_REASON,
    ANGULAR_ENDPOINT_TOLERANCE_RAD,
    DEFAULT_INITIAL_DUAL_ARM_SEED_RAD,
    DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    DUAL_ARM_GROUP_NAME,
    DUAL_ARM_JOINT_ORDER,
    LEFT_GROUP_NAME,
    LEFT_IK_LINK_NAME,
    LEFT_JOINT_ORDER,
    JOINT_POSITIONS_UNCHANGED_NOTICE,
    JUMP_HEURISTIC_WARNING,
    OFFLINE_MODEL_SEED_NOTICE,
    RAW_JOINT_DELTA_NOTICE,
    RAW_JOINT_DELTA_PRESERVED_NOTICE,
    RIGHT_GROUP_NAME,
    RIGHT_IK_LINK_NAME,
    RIGHT_JOINT_ORDER,
    RELATIVE_GROWTH_REASON,
    SHORTEST_ANGULAR_ANALYSIS_NOTICE,
    WRAPAROUND_ADJUSTMENT_TOLERANCE_RAD,
    ArmIkSolution,
    CombinedStateValidity,
    JointDeltaRecord,
    JointJumpDetectionConfig,
    TransitionContinuity,
    analyze_joint_transition,
    analyze_suspicious_joint_jumps,
    combine_arm_joint_solutions,
    joint_delta_rad,
    max_abs_joint_step_rad,
    rotation_matrix_to_quaternion_xyzw,
    shortest_angular_delta_rad,
    solve_sequential_object_trajectory_ik,
    summarize_trajectory_continuity,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CORE_PATH = REPOSITORY_ROOT / "dual_arm_app/backend/object_trajectory_ik.py"
ADAPTER_PATH = REPOSITORY_ROOT / "dual_arm_app/backend/moveit_object_trajectory_ik.py"


class FakeMoveItAdapter:
    """Deterministic fake that never imports ROS or touches a ROS graph."""

    def __init__(
        self,
        *,
        left_fail_at: int | None = None,
        right_fail_at: int | None = None,
        invalid_at: int | None = None,
        joint_offsets_by_sample: tuple[tuple[float, ...], ...] | None = None,
    ) -> None:
        self.left_fail_at = left_fail_at
        self.right_fail_at = right_fail_at
        self.invalid_at = invalid_at
        self.joint_offsets_by_sample = joint_offsets_by_sample
        self.ik_calls: list[dict[str, object]] = []
        self.validity_calls: list[dict[str, object]] = []
        self.current_sample_index = -1

    def solve_arm_ik(
        self,
        *,
        group_name,
        ik_link_name,
        target_world_T_tip,
        seed_joint_positions_rad,
        timeout_s,
        avoid_collisions,
    ) -> ArmIkSolution:
        if group_name == LEFT_GROUP_NAME:
            self.current_sample_index += 1
        sample_index = self.current_sample_index
        self.ik_calls.append({
            "sample_index": sample_index,
            "group_name": group_name,
            "ik_link_name": ik_link_name,
            "target": target_world_T_tip,
            "seed": tuple(seed_joint_positions_rad),
            "timeout_s": timeout_s,
            "avoid_collisions": avoid_collisions,
        })
        offsets = (
            self.joint_offsets_by_sample[sample_index]
            if self.joint_offsets_by_sample is not None
            else (0.1,) * 6 + (0.2,) * 6
        )
        if group_name == LEFT_GROUP_NAME:
            if sample_index == self.left_fail_at:
                return ArmIkSolution(False, diagnostic="synthetic left failure")
            positions = tuple(
                value + offset
                for value, offset in zip(seed_joint_positions_rad[:6], offsets[:6])
            )
            return ArmIkSolution(True, positions)
        if group_name == RIGHT_GROUP_NAME:
            if sample_index == self.right_fail_at:
                return ArmIkSolution(False, diagnostic="synthetic right failure")
            positions = tuple(
                value + offset
                for value, offset in zip(seed_joint_positions_rad[6:], offsets[6:])
            )
            return ArmIkSolution(True, positions)
        raise AssertionError(f"unexpected group: {group_name}")

    def check_combined_state(
        self,
        *,
        joint_positions_rad,
        group_name,
    ) -> CombinedStateValidity:
        self.validity_calls.append({
            "sample_index": self.current_sample_index,
            "group_name": group_name,
            "positions": tuple(joint_positions_rad),
        })
        valid = self.current_sample_index != self.invalid_at
        diagnostic = "synthetic valid" if valid else "synthetic collision"
        return CombinedStateValidity(valid, diagnostic)


class JointAndQuaternionTests(unittest.TestCase):
    def test_canonical_joint_order_is_fixed(self) -> None:
        self.assertEqual(
            LEFT_JOINT_ORDER,
            tuple(f"left_joint_{index}" for index in range(1, 7)),
        )
        self.assertEqual(
            RIGHT_JOINT_ORDER,
            tuple(f"right_joint_{index}" for index in range(1, 7)),
        )
        self.assertEqual(DUAL_ARM_JOINT_ORDER, LEFT_JOINT_ORDER + RIGHT_JOINT_ORDER)
        self.assertEqual(len(DEFAULT_INITIAL_DUAL_ARM_SEED_RAD), 12)

    def test_arm_solutions_combine_left_then_right(self) -> None:
        combined = combine_arm_joint_solutions(range(6), range(6, 12))
        self.assertEqual(combined, tuple(float(value) for value in range(12)))

    def test_invalid_vector_sizes_are_rejected(self) -> None:
        invalid_pairs = (((), range(6)), (range(5), range(6)), (range(6), range(7)))
        for left, right in invalid_pairs:
            with self.subTest(left=left, right=right):
                with self.assertRaises(ValueError):
                    combine_arm_joint_solutions(left, right)
        with self.assertRaises(ValueError):
            joint_delta_rad(range(11), range(12))
        with self.assertRaises(ValueError):
            max_abs_joint_step_rad(range(13))

    def test_nonfinite_and_boolean_joint_values_are_rejected(self) -> None:
        for value in (math.nan, math.inf, -math.inf, True, False):
            with self.subTest(value=value):
                left = [0.0] * 6
                left[3] = value
                with self.assertRaises((TypeError, ValueError)):
                    combine_arm_joint_solutions(left, [0.0] * 6)

    def test_identity_rotation_produces_normalized_identity_quaternion(self) -> None:
        quaternion = rotation_matrix_to_quaternion_xyzw(RigidTransform.identity())
        self.assertEqual(quaternion, (0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in quaternion)), 1.0)

    def test_phase_1f_rpy_rotation_produces_expected_xyzw_quaternion(self) -> None:
        transform = RigidTransform.from_translation_rpy(
            (0.0, 0.0, 0.0),
            (0.1, -0.2, 0.3),
        )
        actual = rotation_matrix_to_quaternion_xyzw(transform)
        expected = (0.06407135, -0.09115755, 0.15343930, 0.98185617)
        direct_error = max(abs(left - right) for left, right in zip(actual, expected))
        negated_error = max(abs(left + right) for left, right in zip(actual, expected))
        self.assertLess(min(direct_error, negated_error), 1e-7)
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in actual)), 1.0)

    def test_quaternion_rejects_non_transform_input(self) -> None:
        with self.assertRaises(TypeError):
            rotation_matrix_to_quaternion_xyzw(None)  # type: ignore[arg-type]


class SequentialObjectTrajectoryIkTests(unittest.TestCase):
    trajectory = SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY

    def test_sample_zero_uses_supplied_seed_for_both_independent_solves(self) -> None:
        seed = tuple(float(index) / 10.0 for index in range(12))
        adapter = FakeMoveItAdapter()
        result = solve_sequential_object_trajectory_ik(
            self.trajectory,
            adapter,
            initial_seed_joint_positions_rad=seed,
        )
        self.assertTrue(result.completed)
        first_left, first_right = adapter.ik_calls[:2]
        self.assertEqual(first_left["seed"], seed)
        self.assertEqual(first_right["seed"], seed)
        self.assertEqual(first_left["group_name"], LEFT_GROUP_NAME)
        self.assertEqual(first_left["ik_link_name"], LEFT_IK_LINK_NAME)
        self.assertEqual(first_right["group_name"], RIGHT_GROUP_NAME)
        self.assertEqual(first_right["ik_link_name"], RIGHT_IK_LINK_NAME)
        self.assertFalse(first_left["avoid_collisions"])
        self.assertFalse(first_right["avoid_collisions"])

    def test_sample_one_uses_only_accepted_sample_zero_pair_as_both_seeds(self) -> None:
        adapter = FakeMoveItAdapter()
        result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
        accepted_zero = result.samples[0].combined_joint_positions_rad
        self.assertIsNotNone(accepted_zero)
        sample_one_calls = adapter.ik_calls[2:4]
        self.assertEqual(sample_one_calls[0]["seed"], accepted_zero)
        self.assertEqual(sample_one_calls[1]["seed"], accepted_zero)

    def test_joint_deltas_and_maximum_steps_are_recorded_without_rejection(self) -> None:
        adapter = FakeMoveItAdapter()
        result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
        self.assertIsNone(result.samples[0].joint_delta_from_previous_rad)
        self.assertIsNone(result.samples[0].max_abs_joint_step_rad)
        expected_delta = (0.1,) * 6 + (0.2,) * 6
        for sample in result.samples[1:]:
            self.assertEqual(len(sample.joint_delta_from_previous_rad or ()), 12)
            for actual, expected in zip(
                sample.joint_delta_from_previous_rad or (),
                expected_delta,
            ):
                self.assertAlmostEqual(actual, expected)
            self.assertAlmostEqual(sample.max_abs_joint_step_rad or 0.0, 0.2)
        self.assertAlmostEqual(result.maximum_observed_joint_step_rad or 0.0, 0.2)
        self.assertTrue(result.completed)

    def test_left_ik_failure_records_sample_and_stops_immediately(self) -> None:
        adapter = FakeMoveItAdapter(left_fail_at=2)
        result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
        self.assertFalse(result.completed)
        self.assertEqual(result.failed_sample_index, 2)
        self.assertEqual(result.processed_sample_count, 3)
        self.assertEqual(result.accepted_sample_count, 2)
        self.assertEqual(result.samples[-1].failure_reason, "LEFT_IK_FAILED")
        self.assertEqual(len(adapter.ik_calls), 5)
        self.assertEqual(len(adapter.validity_calls), 2)

    def test_right_ik_failure_records_sample_and_stops(self) -> None:
        adapter = FakeMoveItAdapter(right_fail_at=1)
        result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
        self.assertFalse(result.completed)
        self.assertEqual(result.failed_sample_index, 1)
        self.assertEqual(result.processed_sample_count, 2)
        self.assertEqual(result.accepted_sample_count, 1)
        self.assertEqual(result.samples[-1].failure_reason, "RIGHT_IK_FAILED")
        self.assertEqual(len(adapter.validity_calls), 1)

    def test_combined_invalid_state_stops_and_is_never_propagated(self) -> None:
        adapter = FakeMoveItAdapter(invalid_at=1)
        result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
        rejected = result.samples[-1].combined_joint_positions_rad
        self.assertFalse(result.completed)
        self.assertEqual(result.failed_sample_index, 1)
        self.assertEqual(result.samples[-1].failure_reason, "COMBINED_STATE_INVALID")
        self.assertIsNotNone(rejected)
        self.assertEqual(len(adapter.ik_calls), 4)
        self.assertEqual(len(adapter.validity_calls), 2)
        self.assertNotIn(rejected, [call["seed"] for call in adapter.ik_calls])

    def test_successful_five_sample_trajectory_completes_in_order(self) -> None:
        adapter = FakeMoveItAdapter()
        result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
        self.assertTrue(result.completed)
        self.assertIsNone(result.failed_sample_index)
        self.assertEqual(result.requested_sample_count, 5)
        self.assertEqual(result.processed_sample_count, 5)
        self.assertEqual(result.accepted_sample_count, 5)
        self.assertEqual(
            tuple(sample.sample_index for sample in result.samples),
            tuple(range(5)),
        )
        self.assertTrue(all(sample.accepted for sample in result.samples))
        self.assertEqual(len(adapter.ik_calls), 10)
        self.assertEqual(len(adapter.validity_calls), 5)
        self.assertTrue(all(
            call["group_name"] == DUAL_ARM_GROUP_NAME
            for call in adapter.validity_calls
        ))

    def test_initial_seed_and_timeout_are_validated_before_adapter_calls(self) -> None:
        for seed in ((0.0,) * 11, (0.0,) * 11 + (math.nan,)):
            adapter = FakeMoveItAdapter()
            with self.subTest(seed=seed):
                with self.assertRaises(ValueError):
                    solve_sequential_object_trajectory_ik(
                        self.trajectory,
                        adapter,
                        initial_seed_joint_positions_rad=seed,
                    )
                self.assertEqual(adapter.ik_calls, [])
        for timeout in (0.0, -1.0, math.nan, math.inf, True):
            adapter = FakeMoveItAdapter()
            with self.subTest(timeout=timeout):
                with self.assertRaises((TypeError, ValueError)):
                    solve_sequential_object_trajectory_ik(
                        self.trajectory,
                        adapter,
                        ik_timeout_s=timeout,
                    )
                self.assertEqual(adapter.ik_calls, [])

    def test_results_and_solutions_are_immutable(self) -> None:
        result = solve_sequential_object_trajectory_ik(
            self.trajectory,
            FakeMoveItAdapter(),
        )
        with self.assertRaises(FrozenInstanceError):
            result.completed = False  # type: ignore[misc]
        with self.assertRaises(FrozenInstanceError):
            result.samples[0].accepted = False  # type: ignore[misc]


class ContinuityAnalysisTests(unittest.TestCase):
    trajectory = SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY

    def test_shortest_delta_zero_and_ordinary_signed_changes(self) -> None:
        cases = (
            (0.0, 0.0, 0.0),
            (0.1, 0.2, 0.1),
            (0.2, 0.1, -0.1),
        )
        for previous, current, expected in cases:
            with self.subTest(previous=previous, current=current):
                record = JointDeltaRecord(0, previous, current)
                self.assertAlmostEqual(record.delta_rad, current - previous)
                self.assertAlmostEqual(record.abs_delta_rad, abs(current - previous))
                self.assertAlmostEqual(record.shortest_delta_rad, expected)
                self.assertAlmostEqual(record.shortest_abs_delta_rad, abs(expected))
                self.assertFalse(record.wraparound_adjusted)

    def test_shortest_delta_handles_both_pi_boundary_crossing_directions(self) -> None:
        positive_crossing = JointDeltaRecord(0, 3.13, -3.13)
        self.assertAlmostEqual(positive_crossing.delta_rad, -6.26)
        self.assertAlmostEqual(
            positive_crossing.shortest_delta_rad,
            math.atan2(math.sin(-6.26), math.cos(-6.26)),
        )
        self.assertGreater(positive_crossing.shortest_delta_rad, 0.0)
        self.assertTrue(positive_crossing.wraparound_adjusted)

        reverse_crossing = JointDeltaRecord(0, -3.13, 3.13)
        self.assertAlmostEqual(reverse_crossing.delta_rad, 6.26)
        self.assertAlmostEqual(
            reverse_crossing.shortest_delta_rad,
            math.atan2(math.sin(6.26), math.cos(6.26)),
        )
        self.assertLess(reverse_crossing.shortest_delta_rad, 0.0)
        self.assertTrue(reverse_crossing.wraparound_adjusted)

    def test_shortest_delta_handles_raw_change_greater_than_two_pi(self) -> None:
        raw = 4.0 * math.pi + 0.25
        record = JointDeltaRecord(0, 0.0, raw)
        self.assertEqual(record.delta_rad, raw)
        self.assertAlmostEqual(record.shortest_delta_rad, 0.25)
        self.assertAlmostEqual(record.shortest_abs_delta_rad, 0.25)
        self.assertTrue(record.wraparound_adjusted)

    def test_shortest_delta_uses_deterministic_open_negative_pi_interval(self) -> None:
        self.assertEqual(shortest_angular_delta_rad(math.pi), math.pi)
        self.assertEqual(shortest_angular_delta_rad(-math.pi), math.pi)
        near_excluded_endpoint = -math.pi + ANGULAR_ENDPOINT_TOLERANCE_RAD / 2.0
        self.assertEqual(shortest_angular_delta_rad(near_excluded_endpoint), math.pi)
        for raw in (-9.0 * math.pi, -math.pi, math.pi, 9.0 * math.pi):
            with self.subTest(raw=raw):
                shortest = shortest_angular_delta_rad(raw)
                self.assertGreater(shortest, -math.pi)
                self.assertLessEqual(shortest, math.pi)
        boundary_record = JointDeltaRecord(0, 0.0, -math.pi)
        self.assertEqual(boundary_record.delta_rad, -math.pi)
        self.assertEqual(boundary_record.shortest_delta_rad, math.pi)
        self.assertTrue(boundary_record.wraparound_adjusted)

    def test_shortest_helper_rejects_nonfinite_and_boolean_values(self) -> None:
        for value in (math.nan, math.inf, -math.inf, True, False):
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    shortest_angular_delta_rad(value)

    def test_transition_preserves_raw_max_and_adds_distinct_shortest_max(self) -> None:
        current = [0.0] * 12
        current[0] = 2.0 * math.pi + 0.1
        current[4] = -0.5
        transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=current,
        )
        self.assertEqual(transition.max_joint_index, 0)
        self.assertEqual(transition.max_joint_name, "left_joint_1")
        self.assertAlmostEqual(transition.max_joint_delta_rad, 2.0 * math.pi + 0.1)
        self.assertAlmostEqual(
            transition.max_abs_joint_step_rad,
            2.0 * math.pi + 0.1,
        )
        self.assertEqual(transition.max_shortest_joint_index, 4)
        self.assertEqual(transition.max_shortest_joint_name, "left_joint_5")
        self.assertAlmostEqual(transition.max_shortest_joint_delta_rad, -0.5)
        self.assertAlmostEqual(transition.max_shortest_abs_joint_step_rad, 0.5)
        self.assertEqual(transition.wraparound_adjusted_record_count, 1)

    def test_shortest_maximum_tie_uses_earliest_canonical_joint(self) -> None:
        current = [0.0] * 12
        current[2] = 2.0 * math.pi + 0.4
        current[8] = -0.4
        transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=current,
        )
        self.assertEqual(transition.max_shortest_joint_index, 2)
        self.assertEqual(transition.max_shortest_joint_name, "left_joint_3")
        self.assertAlmostEqual(transition.max_shortest_abs_joint_step_rad, 0.4)

    def test_trajectory_raw_and_shortest_maxima_and_wrap_counts_are_independent(self) -> None:
        first_current = [0.0] * 12
        first_current[0] = 2.0 * math.pi + 0.1
        first = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=first_current,
        )
        second_current = [0.0] * 12
        second_current[5] = -0.7
        second = analyze_joint_transition(
            from_sample_index=1,
            to_sample_index=2,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=second_current,
        )
        summary = summarize_trajectory_continuity((first, second))
        self.assertEqual(summary.from_sample_index, 0)
        self.assertEqual(summary.to_sample_index, 1)
        self.assertEqual(summary.maximum_joint_index, 0)
        self.assertAlmostEqual(summary.signed_delta_rad or 0.0, 2.0 * math.pi + 0.1)
        self.assertEqual(summary.shortest_from_sample_index, 1)
        self.assertEqual(summary.shortest_to_sample_index, 2)
        self.assertEqual(summary.maximum_shortest_joint_index, 5)
        self.assertEqual(summary.maximum_shortest_joint_name, "left_joint_6")
        self.assertAlmostEqual(summary.signed_shortest_delta_rad or 0.0, -0.7)
        self.assertAlmostEqual(
            summary.maximum_shortest_abs_joint_step_rad or 0.0,
            0.7,
        )
        self.assertEqual(summary.wraparound_adjusted_transition_count, 1)
        self.assertEqual(summary.wraparound_adjusted_record_count, 1)

    def test_transition_has_canonical_names_values_deltas_and_maximum(self) -> None:
        previous = tuple(index / 10.0 for index in range(12))
        expected_deltas = (
            0.01,
            -0.02,
            0.03,
            -0.04,
            0.05,
            -0.06,
            0.07,
            -0.08,
            0.09,
            -0.7,
            0.11,
            -0.12,
        )
        current = tuple(
            position + delta
            for position, delta in zip(previous, expected_deltas)
        )
        transition = analyze_joint_transition(
            from_sample_index=2,
            to_sample_index=3,
            previous_joint_positions_rad=previous,
            current_joint_positions_rad=current,
        )

        self.assertEqual(len(transition.joint_deltas), 12)
        for index, record in enumerate(transition.joint_deltas):
            with self.subTest(index=index):
                self.assertEqual(record.joint_index, index)
                self.assertEqual(record.joint_name, DUAL_ARM_JOINT_ORDER[index])
                self.assertAlmostEqual(record.previous_position_rad, previous[index])
                self.assertAlmostEqual(record.current_position_rad, current[index])
                self.assertAlmostEqual(record.delta_rad, expected_deltas[index])
                self.assertAlmostEqual(record.abs_delta_rad, abs(expected_deltas[index]))
        self.assertEqual(transition.max_joint_index, 9)
        self.assertEqual(transition.max_joint_name, "right_joint_4")
        self.assertAlmostEqual(transition.max_joint_delta_rad, -0.7)
        self.assertAlmostEqual(transition.max_abs_joint_step_rad, 0.7)

    def test_exact_maximum_tie_uses_earliest_canonical_joint(self) -> None:
        deltas = [0.0] * 12
        deltas[2] = 0.5
        deltas[8] = -0.5
        transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=deltas,
        )
        self.assertEqual(transition.max_joint_index, 2)
        self.assertEqual(transition.max_joint_name, "left_joint_3")
        self.assertEqual(transition.max_joint_delta_rad, 0.5)

    def test_delta_is_raw_subtraction_without_wraparound_normalization(self) -> None:
        previous = [0.0] * 12
        current = [0.0] * 12
        previous[0] = 3.1
        current[0] = -3.1
        transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=previous,
            current_joint_positions_rad=current,
        )
        self.assertAlmostEqual(transition.joint_deltas[0].delta_rad, -6.2)
        self.assertAlmostEqual(transition.joint_deltas[0].abs_delta_rad, 6.2)

    def test_richer_analysis_agrees_with_legacy_delta_and_maximum_views(self) -> None:
        result = solve_sequential_object_trajectory_ik(
            self.trajectory,
            FakeMoveItAdapter(),
        )
        self.assertEqual(len(result.continuity_transitions), 4)
        for sample in result.samples[1:]:
            transition = sample.continuity_from_previous
            self.assertIsNotNone(transition)
            self.assertEqual(
                sample.joint_delta_from_previous_rad,
                tuple(record.delta_rad for record in transition.joint_deltas),
            )
            self.assertEqual(
                sample.max_abs_joint_step_rad,
                transition.max_abs_joint_step_rad,
            )
        self.assertEqual(
            result.maximum_observed_joint_step_rad,
            result.continuity_summary.maximum_abs_joint_step_rad,
        )

    def test_trajectory_summary_identifies_transition_joint_and_signed_delta(self) -> None:
        offsets = (
            (0.01,) * 12,
            (0.02,) * 12,
            (0.03,) * 9 + (-0.8,) + (0.03,) * 2,
            (0.04,) * 12,
            (0.05,) * 12,
        )
        result = solve_sequential_object_trajectory_ik(
            self.trajectory,
            FakeMoveItAdapter(joint_offsets_by_sample=offsets),
        )
        summary = result.continuity_summary
        self.assertEqual(summary.transition_count, 4)
        self.assertEqual(summary.from_sample_index, 1)
        self.assertEqual(summary.to_sample_index, 2)
        self.assertEqual(summary.maximum_joint_index, 9)
        self.assertEqual(summary.maximum_joint_name, "right_joint_4")
        self.assertAlmostEqual(summary.signed_delta_rad or 0.0, -0.8)
        self.assertAlmostEqual(summary.maximum_abs_joint_step_rad or 0.0, 0.8)
        self.assertEqual(summary.shortest_from_sample_index, 1)
        self.assertEqual(summary.shortest_to_sample_index, 2)
        self.assertEqual(summary.maximum_shortest_joint_index, 9)
        self.assertEqual(summary.maximum_shortest_joint_name, "right_joint_4")
        self.assertAlmostEqual(summary.signed_shortest_delta_rad or 0.0, -0.8)
        self.assertAlmostEqual(
            summary.maximum_shortest_abs_joint_step_rad or 0.0,
            0.8,
        )
        self.assertEqual(summary.wraparound_adjusted_transition_count, 0)
        self.assertEqual(summary.wraparound_adjusted_record_count, 0)

    def test_one_accepted_sample_has_no_transition_or_fabricated_maximum(self) -> None:
        result = solve_sequential_object_trajectory_ik(
            self.trajectory,
            FakeMoveItAdapter(invalid_at=1),
        )
        summary = result.continuity_summary
        self.assertEqual(result.accepted_sample_count, 1)
        self.assertEqual(summary.transition_count, 0)
        self.assertEqual(result.continuity_transitions, ())
        self.assertIsNone(summary.maximum_abs_joint_step_rad)
        self.assertIsNone(summary.maximum_joint_name)
        self.assertIsNone(summary.maximum_joint_index)
        self.assertIsNone(summary.from_sample_index)
        self.assertIsNone(summary.to_sample_index)
        self.assertIsNone(summary.signed_delta_rad)
        self.assertIsNone(summary.maximum_shortest_abs_joint_step_rad)
        self.assertIsNone(summary.maximum_shortest_joint_name)
        self.assertIsNone(summary.maximum_shortest_joint_index)
        self.assertIsNone(summary.shortest_from_sample_index)
        self.assertIsNone(summary.shortest_to_sample_index)
        self.assertIsNone(summary.signed_shortest_delta_rad)
        self.assertEqual(summary.wraparound_adjusted_transition_count, 0)
        self.assertEqual(summary.wraparound_adjusted_record_count, 0)
        self.assertIsNone(result.maximum_observed_joint_step_rad)
        self.assertIsNone(result.samples[-1].continuity_from_previous)

    def test_failed_or_rejected_sample_is_not_a_continuity_transition(self) -> None:
        for adapter in (
            FakeMoveItAdapter(left_fail_at=2),
            FakeMoveItAdapter(right_fail_at=2),
            FakeMoveItAdapter(invalid_at=2),
        ):
            with self.subTest(adapter=adapter):
                result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
                self.assertEqual(result.accepted_sample_count, 2)
                self.assertEqual(result.continuity_summary.transition_count, 1)
                self.assertEqual(len(result.continuity_transitions), 1)
                self.assertFalse(result.samples[-1].accepted)
                self.assertIsNone(result.samples[-1].continuity_from_previous)

    def test_continuity_inputs_remain_finite_controlled_and_immutable(self) -> None:
        for value in (math.nan, math.inf, -math.inf, True):
            current = [0.0] * 12
            current[5] = value
            with self.subTest(value=value):
                with self.assertRaises((TypeError, ValueError)):
                    analyze_joint_transition(
                        from_sample_index=0,
                        to_sample_index=1,
                        previous_joint_positions_rad=(0.0,) * 12,
                        current_joint_positions_rad=current,
                    )
        record = JointDeltaRecord(0, 1.0, 1.5)
        with self.assertRaises(FrozenInstanceError):
            record.delta_rad = 99.0  # type: ignore[misc]

    def test_summary_helper_uses_earliest_transition_on_exact_tie(self) -> None:
        first = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=(0.4,) + (0.0,) * 11,
        )
        second = analyze_joint_transition(
            from_sample_index=1,
            to_sample_index=2,
            previous_joint_positions_rad=(0.0,) * 12,
            current_joint_positions_rad=(0.0,) * 11 + (-0.4,),
        )
        summary = summarize_trajectory_continuity((first, second))
        self.assertEqual(summary.from_sample_index, 0)
        self.assertEqual(summary.to_sample_index, 1)
        self.assertEqual(summary.maximum_joint_index, 0)
        self.assertEqual(summary.signed_delta_rad, 0.4)

    def test_transition_model_rejects_noncanonical_record_order(self) -> None:
        records = tuple(JointDeltaRecord(index, 0.0, 0.0) for index in range(12))
        with self.assertRaises(ValueError):
            TransitionContinuity(0, 1, tuple(reversed(records)))


class SuspiciousJointJumpAnalysisTests(unittest.TestCase):
    trajectory = SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY

    @staticmethod
    def transitions_from_steps(*steps: tuple[float, ...]) -> tuple[TransitionContinuity, ...]:
        position = (0.0,) * 12
        transitions = []
        for index, step in enumerate(steps):
            if len(step) != 12:
                raise ValueError("synthetic step must contain 12 joints")
            current = tuple(left + right for left, right in zip(position, step))
            transitions.append(analyze_joint_transition(
                from_sample_index=index,
                to_sample_index=index + 1,
                previous_joint_positions_rad=position,
                current_joint_positions_rad=current,
            ))
            position = current
        return tuple(transitions)

    @staticmethod
    def config(
        *,
        absolute=1.0,
        relative=3.0,
        floor=0.05,
        enabled=True,
    ) -> JointJumpDetectionConfig:
        return JointJumpDetectionConfig(
            absolute_step_threshold_rad=absolute,
            relative_step_ratio_threshold=relative,
            relative_reference_floor_rad=floor,
            enabled=enabled,
        )

    def test_config_accepts_valid_thresholds_and_is_immutable(self) -> None:
        config = self.config(absolute=1.25, relative=2.5, floor=0.1)
        self.assertEqual(config.absolute_step_threshold_rad, 1.25)
        self.assertEqual(config.relative_step_ratio_threshold, 2.5)
        self.assertEqual(config.relative_reference_floor_rad, 0.1)
        self.assertTrue(config.enabled)
        disabled_criteria = self.config(absolute=None, relative=None, floor=0.0)
        self.assertIsNone(disabled_criteria.absolute_step_threshold_rad)
        self.assertIsNone(disabled_criteria.relative_step_ratio_threshold)
        with self.assertRaises(FrozenInstanceError):
            config.absolute_step_threshold_rad = 99.0  # type: ignore[misc]

    def test_config_rejects_nonfinite_zero_negative_and_boolean_numbers(self) -> None:
        for value in (math.nan, math.inf, -math.inf, 0.0, -0.1, True, False):
            with self.subTest(field="absolute", value=value):
                with self.assertRaises((TypeError, ValueError)):
                    self.config(absolute=value)
            with self.subTest(field="relative", value=value):
                with self.assertRaises((TypeError, ValueError)):
                    self.config(relative=value)
        for value in (math.nan, math.inf, -math.inf, -0.1, True, False):
            with self.subTest(field="floor", value=value):
                with self.assertRaises((TypeError, ValueError)):
                    self.config(floor=value)
        for enabled in (0, 1, "yes", None):
            with self.subTest(enabled=enabled):
                with self.assertRaises(TypeError):
                    self.config(enabled=enabled)

    def test_absolute_criterion_uses_strict_greater_than_boundary(self) -> None:
        for step, expected in ((0.25, False), (0.5, False), (0.500001, True)):
            values = (step,) + (0.0,) * 11
            analysis = analyze_suspicious_joint_jumps(
                self.transitions_from_steps(values),
                self.config(absolute=0.5, relative=None),
            )
            assessment = analysis.transitions[0].assessments[0]
            with self.subTest(step=step):
                self.assertEqual(assessment.absolute_flag, expected)
                self.assertFalse(assessment.relative_flag)
                self.assertEqual(assessment.suspicious, expected)

    def test_first_transition_has_no_relative_reference_or_ratio(self) -> None:
        analysis = analyze_suspicious_joint_jumps(
            self.transitions_from_steps((0.8,) + (0.0,) * 11),
            self.config(absolute=None, relative=2.0, floor=0.05),
        )
        assessment = analysis.transitions[0].assessments[0]
        self.assertIsNone(assessment.previous_shortest_abs_delta_rad)
        self.assertIsNone(assessment.relative_step_ratio)
        self.assertFalse(assessment.relative_flag)

    def test_previous_step_below_floor_does_not_evaluate_relative_ratio(self) -> None:
        transitions = self.transitions_from_steps(
            (0.04,) + (0.0,) * 11,
            (0.9,) + (0.0,) * 11,
        )
        analysis = analyze_suspicious_joint_jumps(
            transitions,
            self.config(absolute=None, relative=2.0, floor=0.05),
        )
        assessment = analysis.transitions[1].assessments[0]
        self.assertAlmostEqual(assessment.previous_shortest_abs_delta_rad or 0.0, 0.04)
        self.assertIsNone(assessment.relative_step_ratio)
        self.assertFalse(assessment.relative_flag)

    def test_relative_criterion_uses_strict_greater_than_boundary(self) -> None:
        for current_step, expected in ((0.25, False), (0.375, False), (0.375001, True)):
            transitions = self.transitions_from_steps(
                (0.125,) + (0.0,) * 11,
                (current_step,) + (0.0,) * 11,
            )
            analysis = analyze_suspicious_joint_jumps(
                transitions,
                self.config(absolute=None, relative=3.0, floor=0.05),
            )
            assessment = analysis.transitions[1].assessments[0]
            with self.subTest(current_step=current_step):
                self.assertAlmostEqual(
                    assessment.relative_step_ratio or 0.0,
                    current_step / 0.125,
                )
                self.assertEqual(assessment.relative_flag, expected)

    def test_absolute_relative_and_combined_reasons_are_independent_and_ordered(self) -> None:
        absolute_only = analyze_suspicious_joint_jumps(
            self.transitions_from_steps((0.6,) + (0.0,) * 11),
            self.config(absolute=0.5, relative=2.0),
        ).transitions[0].assessments[0]
        self.assertTrue(absolute_only.absolute_flag)
        self.assertFalse(absolute_only.relative_flag)
        self.assertEqual(absolute_only.reasons, (ABSOLUTE_STEP_REASON,))

        relative_only = analyze_suspicious_joint_jumps(
            self.transitions_from_steps(
                (0.1,) + (0.0,) * 11,
                (0.31,) + (0.0,) * 11,
            ),
            self.config(absolute=1.0, relative=3.0),
        ).transitions[1].assessments[0]
        self.assertFalse(relative_only.absolute_flag)
        self.assertTrue(relative_only.relative_flag)
        self.assertEqual(relative_only.reasons, (RELATIVE_GROWTH_REASON,))

        both = analyze_suspicious_joint_jumps(
            self.transitions_from_steps(
                (0.2,) + (0.0,) * 11,
                (0.8,) + (0.0,) * 11,
            ),
            self.config(absolute=0.5, relative=3.0),
        ).transitions[1].assessments[0]
        self.assertTrue(both.absolute_flag)
        self.assertTrue(both.relative_flag)
        self.assertTrue(both.suspicious)
        self.assertEqual(
            both.reasons,
            (ABSOLUTE_STEP_REASON, RELATIVE_GROWTH_REASON),
        )

    def test_assessments_preserve_canonical_joint_identity(self) -> None:
        step = (0.0,) * 9 + (1.1,) + (0.0,) * 2
        analysis = analyze_suspicious_joint_jumps(
            self.transitions_from_steps(step),
            self.config(absolute=1.0, relative=None),
        )
        assessment = analysis.transitions[0].assessments[9]
        self.assertEqual(assessment.joint_index, 9)
        self.assertEqual(assessment.joint_name, "right_joint_4")
        self.assertEqual(analysis.transitions[0].suspicious_joint_names, ("right_joint_4",))

    def test_wraparound_detection_uses_shortest_not_large_raw_delta(self) -> None:
        previous = (3.13,) + (0.0,) * 11
        current = (-3.13,) + (0.0,) * 11
        transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=previous,
            current_joint_positions_rad=current,
        )
        raw_before = transition.joint_deltas[0].delta_rad
        shortest_before = transition.joint_deltas[0].shortest_delta_rad
        analysis = analyze_suspicious_joint_jumps(
            (transition,),
            self.config(absolute=1.0, relative=None),
        )
        assessment = analysis.transitions[0].assessments[0]
        self.assertAlmostEqual(raw_before, -6.26)
        self.assertLess(abs(shortest_before), 0.03)
        self.assertEqual(assessment.shortest_delta_rad, shortest_before)
        self.assertFalse(assessment.absolute_flag)
        self.assertFalse(assessment.suspicious)
        self.assertEqual(transition.joint_deltas[0].delta_rad, raw_before)
        self.assertEqual(transition.joint_deltas[0].shortest_delta_rad, shortest_before)

    def test_trajectory_counts_and_chronological_suspicious_transitions(self) -> None:
        transitions = self.transitions_from_steps(
            (0.1, 0.1) + (0.0,) * 10,
            (0.4, 1.2) + (0.0,) * 10,
            (1.3, 0.2, 1.1) + (0.0,) * 9,
        )
        analysis = analyze_suspicious_joint_jumps(
            transitions,
            self.config(absolute=1.0, relative=3.0, floor=0.05),
        )
        self.assertTrue(analysis.analysis_completed)
        self.assertEqual(analysis.suspicious_transition_count, 2)
        self.assertEqual(analysis.suspicious_joint_record_count, 4)
        self.assertEqual(
            tuple(item.to_sample_index for item in analysis.suspicious_transitions),
            (2, 3),
        )

    def test_zero_transitions_produces_completed_empty_analysis(self) -> None:
        analysis = analyze_suspicious_joint_jumps((), self.config())
        self.assertTrue(analysis.analysis_completed)
        self.assertEqual(analysis.transitions, ())
        self.assertEqual(analysis.suspicious_transitions, ())
        self.assertEqual(analysis.suspicious_transition_count, 0)
        self.assertEqual(analysis.suspicious_joint_record_count, 0)

        one_accepted = solve_sequential_object_trajectory_ik(
            self.trajectory,
            FakeMoveItAdapter(invalid_at=1),
        )
        result_analysis = one_accepted.analyze_suspicious_jumps(self.config())
        self.assertEqual(one_accepted.accepted_sample_count, 1)
        self.assertEqual(result_analysis.transitions, ())
        self.assertEqual(result_analysis.suspicious_transition_count, 0)
        self.assertEqual(result_analysis.suspicious_joint_record_count, 0)

    def test_failed_or_rejected_sample_is_excluded_from_jump_sequence(self) -> None:
        for adapter in (
            FakeMoveItAdapter(left_fail_at=2),
            FakeMoveItAdapter(right_fail_at=2),
            FakeMoveItAdapter(invalid_at=2),
        ):
            with self.subTest(adapter=adapter):
                result = solve_sequential_object_trajectory_ik(self.trajectory, adapter)
                analysis = result.analyze_suspicious_jumps(self.config())
                self.assertEqual(result.accepted_sample_count, 2)
                self.assertEqual(len(result.continuity_transitions), 1)
                self.assertEqual(len(analysis.transitions), 1)
                self.assertEqual(analysis.transitions[0].from_sample_index, 0)
                self.assertEqual(analysis.transitions[0].to_sample_index, 1)
                self.assertFalse(result.samples[-1].accepted)

    def test_disabled_detector_retains_diagnostics_but_never_flags(self) -> None:
        transitions = self.transitions_from_steps(
            (0.1,) + (0.0,) * 11,
            (2.0,) + (0.0,) * 11,
        )
        analysis = analyze_suspicious_joint_jumps(
            transitions,
            self.config(absolute=0.5, relative=2.0, enabled=False),
        )
        assessment = analysis.transitions[1].assessments[0]
        self.assertAlmostEqual(assessment.relative_step_ratio or 0.0, 20.0)
        self.assertFalse(assessment.absolute_flag)
        self.assertFalse(assessment.relative_flag)
        self.assertFalse(assessment.suspicious)
        self.assertEqual(assessment.reasons, ())

    def test_suspicion_does_not_change_ik_acceptance_or_completion(self) -> None:
        offsets = (
            (0.1,) * 12,
            (1.5,) + (0.1,) * 11,
            (0.1,) * 12,
            (0.1,) * 12,
            (0.1,) * 12,
        )
        result = solve_sequential_object_trajectory_ik(
            self.trajectory,
            FakeMoveItAdapter(joint_offsets_by_sample=offsets),
        )
        accepted_before = tuple(sample.accepted for sample in result.samples)
        completed_before = result.completed
        analysis = result.analyze_suspicious_jumps(self.config())
        self.assertGreater(analysis.suspicious_transition_count, 0)
        self.assertEqual(tuple(sample.accepted for sample in result.samples), accepted_before)
        self.assertTrue(all(accepted_before))
        self.assertEqual(result.completed, completed_before)
        self.assertTrue(result.completed)
        self.assertEqual(result.accepted_sample_count, 5)


class ArchitectureAndSafetyTests(unittest.TestCase):
    def test_core_has_no_ros_moveit_driver_or_numpy_dependency(self) -> None:
        source = CORE_PATH.read_text(encoding="utf-8")
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
        for forbidden in ("rclpy", "moveit_msgs", "jaka_msgs", "numpy", "scipy"):
            self.assertFalse(any(
                name == forbidden or name.startswith(f"{forbidden}.")
                for name in imports
            ))

    def test_sources_document_offline_safety_and_model_seed(self) -> None:
        combined_source = "\n".join((
            CORE_PATH.read_text(encoding="utf-8"),
            ADAPTER_PATH.read_text(encoding="utf-8"),
        ))
        for warning in (
            "OFFLINE OBJECT TRAJECTORY IK",
            "MOVEIT PLANNING ONLY",
            "NO JAKA DRIVER",
            "NO ROBOT CONNECTION",
            "NO MOTION EXECUTION",
            "OFFLINE SYNTHETIC / MODEL SEED — NOT PHYSICAL ROBOT CALIBRATION",
        ):
            self.assertIn(warning, combined_source)
        self.assertEqual(
            OFFLINE_MODEL_SEED_NOTICE,
            "OFFLINE SYNTHETIC / MODEL SEED — NOT PHYSICAL ROBOT CALIBRATION",
        )

    def test_raw_delta_wraparound_limitation_is_in_core_and_runtime_report(self) -> None:
        core_source = CORE_PATH.read_text(encoding="utf-8")
        adapter_source = ADAPTER_PATH.read_text(encoding="utf-8")
        self.assertEqual(
            RAW_JOINT_DELTA_NOTICE,
            "RAW JOINT DELTA — WRAPAROUND NOT YET NORMALIZED",
        )
        self.assertIn(RAW_JOINT_DELTA_NOTICE, core_source)
        self.assertIn("print(RAW_JOINT_DELTA_NOTICE)", adapter_source)
        self.assertEqual(RAW_JOINT_DELTA_PRESERVED_NOTICE, "RAW JOINT DELTA IS PRESERVED")
        self.assertEqual(
            SHORTEST_ANGULAR_ANALYSIS_NOTICE,
            "SHORTEST ANGULAR DELTA IS AN ANALYSIS METRIC ONLY",
        )
        self.assertEqual(
            JOINT_POSITIONS_UNCHANGED_NOTICE,
            "JOINT POSITIONS ARE NOT NORMALIZED OR MODIFIED",
        )
        for notice, symbol_name in (
            (RAW_JOINT_DELTA_PRESERVED_NOTICE, "RAW_JOINT_DELTA_PRESERVED_NOTICE"),
            (SHORTEST_ANGULAR_ANALYSIS_NOTICE, "SHORTEST_ANGULAR_ANALYSIS_NOTICE"),
            (JOINT_POSITIONS_UNCHANGED_NOTICE, "JOINT_POSITIONS_UNCHANGED_NOTICE"),
        ):
            self.assertIn(notice, core_source)
            self.assertIn(f"print({symbol_name})", adapter_source)
        self.assertEqual(ANGULAR_ENDPOINT_TOLERANCE_RAD, 1e-12)
        self.assertEqual(WRAPAROUND_ADJUSTMENT_TOLERANCE_RAD, 1e-9)

    def test_jump_detector_is_explicitly_offline_heuristic_not_safety_limit(self) -> None:
        core_source = CORE_PATH.read_text(encoding="utf-8")
        adapter_source = ADAPTER_PATH.read_text(encoding="utf-8")
        self.assertEqual(
            JUMP_HEURISTIC_WARNING,
            "ANALYSIS HEURISTIC ONLY — NOT A ROBOT SAFETY LIMIT",
        )
        self.assertIn(JUMP_HEURISTIC_WARNING, core_source)
        self.assertIn("OFFLINE JUMP HEURISTIC PROFILE", adapter_source)
        self.assertIn("print_jump_profile(DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG)", adapter_source)
        self.assertNotIn("SAFE FOR ROBOT EXECUTION", adapter_source)
        self.assertEqual(
            DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG.absolute_step_threshold_rad,
            1.0,
        )
        self.assertEqual(
            DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG.relative_step_ratio_threshold,
            3.0,
        )
        self.assertEqual(
            DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG.relative_reference_floor_rad,
            0.05,
        )
        for not_yet_allowed in (
            "shortest_angular_distance",
            "math.remainder",
            "jump_threshold",
            "velocity_rad",
            "acceleration_rad",
        ):
            self.assertNotIn(not_yet_allowed, core_source)

    def test_runtime_adapter_contains_only_allowed_planning_interfaces(self) -> None:
        source = ADAPTER_PATH.read_text(encoding="utf-8")
        self.assertIn('COMPUTE_IK_SERVICE = "/compute_ik"', source)
        self.assertIn(
            'CHECK_STATE_VALIDITY_SERVICE = "/check_state_validity"',
            source,
        )
        for forbidden in (
            "/left_jaka_driver/get_ik",
            "/right_jaka_driver/get_ik",
            "ExecuteTrajectory",
            "JointTrajectory",
            "create_publisher",
            "create_subscription",
            "create_action_client",
            "ActionClient",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
