"""Pure offline tests for the Phase-4A validation core and FK boundary."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from dual_arm_app.backend.moveit_fk_validation import (
    COMPUTE_FK_SERVICE,
    FK_LINK_NAMES,
    WORLD_FRAME,
    MoveItFkValidationAdapter,
)
from dual_arm_app.backend.moveit_object_trajectory_ik import load_canonical_joint_limits
from dual_arm_app.backend.object_grasp_model import RigidTransform, compose, inverse
from dual_arm_app.backend.object_trajectory_ik import DUAL_ARM_JOINT_ORDER
from dual_arm_app.backend.phase4_trajectory_validation import (
    ACCELERATION_ESTIMATE_SEMANTIC,
    COMMON_TIMELINE_SUCCESS,
    DEFAULT_ORIENTATION_TOLERANCE_RAD,
    DEFAULT_POSITION_TOLERANCE_M,
    DYNAMIC_TIMELINE_WARNING,
    MODEL_CONSISTENCY_TOLERANCE_SEMANTIC,
    MOVEIT_JOINT_LIMITS_PATH,
    Phase4ValidationInputError,
    compute_discrete_accelerations,
    compute_joint_velocities,
    load_moveit_joint_dynamics_limits,
    rotation_angle_error_rad,
    transform_error,
    validate_common_timeline,
    validate_phase4_trajectory,
)


def matrix_payload(transform: RigidTransform) -> dict:
    return {"matrix": [list(row) for row in transform.matrix]}


def grasp_payload(left: RigidTransform, right: RigidTransform) -> dict:
    return {
        "status": "GRASP_LOCKED",
        "source": "LOCKED OPERATOR GRASP",
        "content_revision": "sha256:test-grasp",
        "lock_generation": 1,
        "lock_revision": "sha256:test-lock",
        "left": matrix_payload(left),
        "right": matrix_payload(right),
        "expected_left_T_right": matrix_payload(compose(inverse(left), right)),
    }


def make_plan(joints=None, times=None, targets=None):
    times = [0.0, 1.0, 2.0] if times is None else list(times)
    joints = [[0.1 * i] * 12 for i in range(len(times))] if joints is None else joints
    identity = RigidTransform.identity()
    targets = [(identity, identity)] * len(times) if targets is None else targets
    return {
        "ok": True,
        "planner_status": "READY",
        "duration_s": times[-1],
        "grasp": grasp_payload(targets[0][0], targets[0][1]),
        "object_samples": [
            {
                "sample_index": index,
                "time_from_start_s": time_s,
                "object_pose": matrix_payload(identity),
                "left_target": matrix_payload(targets[index][0]),
                "right_target": matrix_payload(targets[index][1]),
            }
            for index, time_s in enumerate(times)
        ],
        "global_path": [
            {
                "sample_index": index,
                "time_from_start_s": time_s,
                "left": list(joints[index][:6]),
                "right": list(joints[index][6:]),
                "combined": list(joints[index]),
            }
            for index, time_s in enumerate(times)
        ],
    }


class EchoFk:
    def __init__(self, poses=None):
        self.poses = poses
        self.calls = []

    def compute_combined_fk(self, joint_positions_rad, timeout_s=2.0):
        self.calls.append((tuple(joint_positions_rad), timeout_s))
        left, right = self.poses or (RigidTransform.identity(), RigidTransform.identity())
        return {"status": "PASS", "left": left, "right": right}


class TimeoutFk:
    def compute_combined_fk(self, joint_positions_rad, timeout_s=2.0):
        del joint_positions_rad, timeout_s
        raise TimeoutError("synthetic timeout")


class Phase4TrajectoryValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.position_limits = load_canonical_joint_limits()
        cls.dynamics = load_moveit_joint_dynamics_limits()

    def validate(self, plan=None, adapter=None, start=True, **kwargs):
        return validate_phase4_trajectory(
            plan or make_plan(),
            fk_adapter=adapter if adapter is not None else EchoFk(),
            joint_position_limits=self.position_limits,
            start_state={
                "left": [0.0] * 6,
                "right": [0.0] * 6,
                "source": "EXPLICIT_OFFLINE_TEST",
            } if start else None,
            dynamics_limits=self.dynamics,
            **kwargs,
        )

    def test_continuity_reuses_existing_three_functions(self):
        result = self.validate()
        source = result["continuity"]["algorithm_source"]
        self.assertIn("analyze_joint_transition", source)
        self.assertIn("summarize_trajectory_continuity", source)
        self.assertIn("analyze_suspicious_joint_jumps", source)

    def test_raw_and_shortest_wraparound_diagnostics_are_both_retained(self):
        plan = make_plan(
            joints=[[math.pi - 0.1] * 12, [-math.pi + 0.1] * 12],
            times=[0.0, 1.0],
        )
        result = self.validate(plan)
        transition = result["continuity"]["transitions"][0]
        self.assertAlmostEqual(transition["raw_deltas_rad"][0], -2 * math.pi + 0.2)
        self.assertAlmostEqual(transition["shortest_angular_deltas_rad"][0], 0.2)
        self.assertEqual(len(transition["wraparound_records"]), 12)

    def test_explicit_start_to_first_is_evaluated(self):
        result = self.validate()
        self.assertEqual(result["start_transition"]["status"], "EVALUATED")
        self.assertEqual(result["start_transition"]["from_state"], "START_STATE")
        self.assertEqual(result["start_transition"]["to_sample_index"], 0)
        self.assertIn("suspicious_jumps", result["start_transition"])

    def test_missing_start_is_not_silently_replaced(self):
        result = self.validate(start=False)
        self.assertEqual(result["start_transition"]["status"], "NOT_EVALUATED")
        self.assertEqual(result["start_transition"]["source"], "UNAVAILABLE")

    def test_end_state_reports_time_joints_and_three_states(self):
        result = self.validate()
        end = result["end_state"]
        self.assertEqual(end["sample_index"], 2)
        self.assertEqual(end["time_from_start_s"], 2.0)
        self.assertEqual(len(end["combined"]), 12)
        self.assertEqual(end["joint_position_limits"], "PASS")
        self.assertEqual(end["fk"], "PASS")
        self.assertEqual(end["relative_pose"], "PASS")

    def test_common_timeline_success_has_required_wording(self):
        timeline = validate_common_timeline(make_plan())
        self.assertEqual(timeline["status"], COMMON_TIMELINE_SUCCESS)
        self.assertEqual(timeline["message"], "COMMON PLAN TIMELINE VERIFIED")
        self.assertEqual(timeline["dynamic_parameterization"], DYNAMIC_TIMELINE_WARNING)

    def test_timeline_requires_zero_strictly_increasing_and_duration(self):
        plan = make_plan(times=[0.1, 0.1, 1.0])
        plan["duration_s"] = 2.0
        result = validate_common_timeline(plan)
        reasons = {item["reason"] for item in result["failures"]}
        self.assertIn("FIRST_TIMESTAMP_NOT_ZERO", reasons)
        self.assertIn("TIMESTAMPS_NOT_STRICTLY_INCREASING", reasons)
        self.assertIn("FINAL_TIMESTAMP_DURATION_MISMATCH", reasons)

    def test_object_and_joint_timestamps_must_match(self):
        plan = make_plan()
        plan["object_samples"][1]["time_from_start_s"] = 1.25
        result = validate_common_timeline(plan)
        self.assertIn("OBJECT_JOINT_TIMESTAMP_MISMATCH", {x["reason"] for x in result["failures"]})

    def test_fk_called_once_per_point_for_both_arms(self):
        adapter = EchoFk()
        result = self.validate(adapter=adapter)
        self.assertEqual(len(adapter.calls), 3)
        self.assertTrue(all(len(call[0]) == 12 for call in adapter.calls))
        self.assertEqual(len(result["fk"]["samples"]), 3)

    def test_fk_position_error_and_exact_tolerance_boundary(self):
        moved = RigidTransform.from_translation_rpy((0.001, 0.0, 0.0), (0.0, 0.0, 0.0))
        result = self.validate(adapter=EchoFk((moved, RigidTransform.identity())))
        self.assertEqual(result["fk"]["status"], "PASS")
        self.assertAlmostEqual(result["fk"]["max_left_position_error_m"], 0.001)

    def test_above_fk_tolerance_fails(self):
        moved = RigidTransform.from_translation_rpy((0.001001, 0.0, 0.0), (0.0, 0.0, 0.0))
        result = self.validate(adapter=EchoFk((moved, RigidTransform.identity())))
        self.assertEqual(result["fk"]["status"], "FAIL")

    def test_rotation_angle_formula_not_euler_subtraction(self):
        identity = RigidTransform.identity()
        quarter_turn = RigidTransform.from_translation_rpy((0, 0, 0), (0, 0, math.pi / 2))
        self.assertAlmostEqual(rotation_angle_error_rad(identity, quarter_turn), math.pi / 2)
        self.assertEqual(transform_error(identity, quarter_turn)[0], 0.0)

    def test_default_tolerances_and_semantic_are_explicit(self):
        result = self.validate()
        self.assertEqual(result["fk"]["position_tolerance_m"], DEFAULT_POSITION_TOLERANCE_M)
        self.assertEqual(result["fk"]["orientation_tolerance_rad"], DEFAULT_ORIENTATION_TOLERANCE_RAD)
        self.assertEqual(result["fk"]["tolerance_semantic"], MODEL_CONSISTENCY_TOLERANCE_SEMANTIC)

    def test_boolean_or_negative_tolerances_are_rejected(self):
        with self.assertRaises(Phase4ValidationInputError):
            self.validate(position_tolerance_m=True)
        with self.assertRaises(Phase4ValidationInputError):
            self.validate(orientation_tolerance_rad=-0.1)

    def test_fk_timeout_and_unavailable_are_structured(self):
        timeout = self.validate(adapter=TimeoutFk())
        unavailable = validate_phase4_trajectory(
            make_plan(), dynamics_limits=self.dynamics, fk_adapter=None
        )
        self.assertEqual(timeout["fk"]["status"], "TIMEOUT")
        self.assertEqual(timeout["relative_pose"]["status"], "NOT_EVALUATED")
        self.assertEqual(unavailable["fk"]["status"], "UNAVAILABLE")

    def test_relative_transform_errors_and_fixed_grasp_max_sample(self):
        right_target = RigidTransform.from_translation_rpy((0.5, 0, 0), (0, 0, 0))
        plan = make_plan(targets=[(RigidTransform.identity(), right_target)] * 3)
        right_fk = RigidTransform.from_translation_rpy((0.502, 0, 0), (0, 0, 0))
        result = self.validate(plan, adapter=EchoFk((RigidTransform.identity(), right_fk)))
        self.assertAlmostEqual(result["relative_pose"]["max_translation_error_m"], 0.002)
        self.assertEqual(result["relative_pose"]["max_translation_error_sample_index"], 0)
        self.assertEqual(result["fixed_grasp"]["status"], "PASS")
        self.assertEqual(result["relative_pose"]["status"], "FAIL")

    def test_velocity_uses_raw_dq_over_dt(self):
        plan = make_plan(
            joints=[[math.pi - 0.1] * 12, [-math.pi + 0.1] * 12],
            times=[0.0, 2.0],
        )
        velocity = compute_joint_velocities(plan["global_path"], self.dynamics)
        self.assertAlmostEqual(velocity["segments"][0]["velocity_rad_s"][0], (-2 * math.pi + 0.2) / 2)
        self.assertIn("NO SHORTEST-ANGLE", velocity["coordinate_policy"])

    def test_moveit_velocity_limit_is_157_and_scaling_not_hard_limit(self):
        self.assertTrue(MOVEIT_JOINT_LIMITS_PATH.is_file())
        self.assertEqual(self.dynamics.velocity_limits_rad_s, (1.57,) * 12)
        self.assertEqual(self.dynamics.velocity_limit_source, "MOVEIT_JOINT_LIMITS_YAML")
        self.assertEqual(self.dynamics.default_velocity_scaling_factor, 0.1)
        velocity = compute_joint_velocities(make_plan()["global_path"], self.dynamics)
        self.assertFalse(velocity["scaling_factor_is_hard_limit"])

    def test_exact_velocity_boundary_passes_and_above_fails(self):
        exact = make_plan(joints=[[0.0] * 12, [1.57] * 12], times=[0.0, 1.0])
        above = make_plan(joints=[[0.0] * 12, [1.571] * 12], times=[0.0, 1.0])
        exact_result = compute_joint_velocities(exact["global_path"], self.dynamics)
        above_result = compute_joint_velocities(above["global_path"], self.dynamics)
        self.assertEqual(exact_result["status"], "PASS")
        self.assertEqual(above_result["status"], "FAIL")
        self.assertEqual(above_result["maximum"]["joint_name"], DUAL_ARM_JOINT_ORDER[0])
        self.assertEqual(above_result["maximum"]["from_sample_index"], 0)

    def test_central_discrete_acceleration_formula(self):
        plan = make_plan(joints=[[0.0] * 12, [1.0] * 12, [4.0] * 12], times=[0.0, 1.0, 3.0])
        velocity = compute_joint_velocities(plan["global_path"], self.dynamics)
        acceleration = compute_discrete_accelerations(velocity, self.dynamics)
        # v_prev=1, v_next=1.5; 2*(1.5-1)/(1+2) = 1/3.
        self.assertAlmostEqual(acceleration["samples"][0]["discrete_acceleration_rad_s2"][0], 1 / 3)
        self.assertEqual(acceleration["semantic"], ACCELERATION_ESTIMATE_SEMANTIC)

    def test_fewer_than_three_points_is_not_applicable(self):
        plan = make_plan(joints=[[0.0] * 12, [0.1] * 12], times=[0.0, 1.0])
        acceleration = compute_discrete_accelerations(
            compute_joint_velocities(plan["global_path"], self.dynamics), self.dynamics
        )
        self.assertEqual(acceleration["compute_status"], "NOT_APPLICABLE")
        self.assertEqual(acceleration["samples"], [])

    def test_unconfigured_acceleration_is_not_zero_limit(self):
        self.assertIsNone(self.dynamics.acceleration_limits_rad_s2)
        self.assertEqual(self.dynamics.acceleration_limit_status, "LIMIT_UNAVAILABLE")
        result = self.validate()
        self.assertEqual(result["acceleration"]["limit_validation"], "NOT_EVALUATED")
        self.assertNotEqual(result["acceleration"].get("limits_rad_s2"), [0.0] * 12)
        self.assertNotIn("MANUFACTURER", result["acceleration"]["limit_source"])

    def test_phase4_never_claims_execution_ready(self):
        result = self.validate()
        self.assertIsNone(result["execution_ready"])
        self.assertIn("NOT EXECUTION READY", result["warnings"])


class _Request:
    def __init__(self):
        self.header = SimpleNamespace(frame_id=None, stamp=None)
        self.fk_link_names = []
        self.robot_state = SimpleNamespace(
            joint_state=SimpleNamespace(name=[], position=[]),
            is_diff=True,
        )


def _pose(x=0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


class MoveItFkAdapterTests(unittest.TestCase):
    def test_adapter_builds_one_canonical_both_tip_world_request(self):
        client = SimpleNamespace(service_is_ready=lambda: True)
        clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))
        captured = []

        def call_and_wait(given_client, request, timeout_s):
            captured.append((given_client, request, timeout_s))
            return SimpleNamespace(
                fk_link_names=list(FK_LINK_NAMES),
                pose_stamped=[SimpleNamespace(pose=_pose()), SimpleNamespace(pose=_pose(0.5))],
            )

        adapter = MoveItFkValidationAdapter(client, clock, call_and_wait, _Request)
        result = adapter.compute_combined_fk([0.0] * 12)
        request = captured[0][1]
        self.assertEqual(COMPUTE_FK_SERVICE, "/compute_fk")
        self.assertEqual(request.header.frame_id, WORLD_FRAME)
        self.assertEqual(request.fk_link_names, list(FK_LINK_NAMES))
        self.assertEqual(request.robot_state.joint_state.name, list(DUAL_ARM_JOINT_ORDER))
        self.assertEqual(result["status"], "PASS")
        self.assertAlmostEqual(result["right"].translation_m[0], 0.5)

    def test_adapter_unavailable_and_missing_link_are_structured(self):
        clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))
        unavailable = MoveItFkValidationAdapter(
            SimpleNamespace(service_is_ready=lambda: False), clock, lambda *_: None, _Request
        )
        self.assertEqual(unavailable.compute_combined_fk([0.0] * 12)["status"], "UNAVAILABLE")
        missing = MoveItFkValidationAdapter(
            SimpleNamespace(service_is_ready=lambda: True),
            clock,
            lambda *_: SimpleNamespace(
                fk_link_names=[FK_LINK_NAMES[0]],
                pose_stamped=[SimpleNamespace(pose=_pose())],
            ),
            _Request,
        )
        self.assertEqual(missing.compute_combined_fk([0.0] * 12)["status"], "ERROR")


if __name__ == "__main__":
    unittest.main()
