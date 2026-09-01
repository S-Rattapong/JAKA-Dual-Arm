"""Focused offline tests for Phase-4C report identity and gate policy."""

from __future__ import annotations

from copy import deepcopy
import unittest

from dual_arm_app.backend.object_trajectory_ik import CanonicalJointPositionLimits
from dual_arm_app.backend.phase4_unified_validation import (
    CHECK_PRIORITY,
    DEFERRED_CHECKS,
    EXECUTION_SCOPE_SEMANTIC,
    REPORT_VERSION,
    REQUIRED_CHECKS,
    compute_plan_fingerprint,
    build_phase4_unified_report,
    evaluate_execution_gate,
    evaluate_ik_completeness,
    validate_all_joint_positions,
)


IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]
START = {"left": [0.0] * 6, "right": [0.0] * 6, "source": "TEST"}
LIMITS = CanonicalJointPositionLimits((-6.28,) * 12, (6.28,) * 12)
AUTHORITY_PASS = {
    "status": "PASS",
    "grasp_content_revision": "sha256:test-grasp",
    "lock_generation": 1,
    "lock_revision": "sha256:test-lock",
    "calibration_revision": "sha256:test-calibration",
    "model_calibration_revision": "sha256:test-calibration",
    "model_calibration_status": "MATCH",
}


def plan() -> dict:
    times = [0.0, 1.0, 2.0]
    return {
        "ok": True,
        "planner_status": "READY",
        "trajectory_name": "Phase4C_Test",
        "duration_s": 2.0,
        "planning_start_state_rad": {"left": [0.0] * 6, "right": [0.0] * 6},
        "planning_start_state_source": "TEST",
        "object_samples": [
            {
                "sample_index": index,
                "time_from_start_s": time_s,
                "object_pose": {"matrix": deepcopy(IDENTITY)},
                "left_target": {"matrix": deepcopy(IDENTITY)},
                "right_target": {"matrix": deepcopy(IDENTITY)},
            }
            for index, time_s in enumerate(times)
        ],
        "global_path": [
            {
                "sample_index": index,
                "time_from_start_s": time_s,
                "left": [index * 0.1] * 6,
                "right": [-index * 0.1] * 6,
                "combined": [index * 0.1] * 6 + [-index * 0.1] * 6,
            }
            for index, time_s in enumerate(times)
        ],
    }


def phase4a_all_pass() -> dict:
    return {
        "status": "PASS",
        "timeline": {
            "status": "COMMON_TIMELINE_STRUCTURALLY_VALID",
            "joint_sample_count": 3,
            "object_sample_count": 3,
            "declared_duration_s": 2.0,
            "failures": [],
        },
        "start_transition": {
            "status": "EVALUATED",
            "source": "TEST",
            "to_sample_index": 0,
            "suspicious_jumps": {
                "analysis_completed": True,
                "suspicious_transition_count": 0,
            },
        },
        "continuity": {
            "status": "PASS",
            "summary": {
                "maximum_abs_joint_step_rad": 0.1,
                "maximum_joint_name": "left_joint_1",
                "maximum_shortest_abs_joint_step_rad": 0.1,
                "maximum_shortest_joint_name": "left_joint_1",
                "wraparound_adjusted_record_count": 0,
            },
            "suspicious_jumps": {
                "analysis_completed": True,
                "suspicious_transition_count": 0,
                "suspicious_joint_record_count": 0,
                "transitions": [],
            },
        },
        "fk": {
            "status": "PASS",
            "max_left_position_error_m": 0.0001,
            "max_right_position_error_m": 0.0002,
            "max_left_orientation_error_rad": 0.0001,
            "max_right_orientation_error_rad": 0.0002,
            "position_tolerance_m": 0.001,
            "orientation_tolerance_rad": 0.001,
        },
        "relative_pose": {
            "status": "PASS",
            "max_translation_error_m": 0.0002,
            "max_translation_error_sample_index": 1,
            "max_orientation_error_rad": 0.0003,
            "max_orientation_error_sample_index": 2,
        },
        "fixed_grasp": {"status": "PASS", "max_translation_error_m": 0.0002},
        "velocity": {
            "status": "PASS",
            "max_abs_velocity_rad_s": 0.1,
            "maximum": {"joint_name": "left_joint_1", "from_sample_index": 0, "to_sample_index": 1},
            "limits_rad_s": [1.57] * 12,
            "limit_source": "MOVEIT_JOINT_LIMITS_YAML",
        },
        "acceleration": {
            "compute_status": "COMPUTED",
            "limit_status": "LIMIT_UNAVAILABLE",
            "limit_validation": "NOT_EVALUATED",
            "limit_source": "UNAVAILABLE",
            "max_abs_discrete_acceleration_rad_s2": 0.2,
            "maximum": {"joint_name": "right_joint_1", "sample_index": 1},
        },
    }


def phase4b_all_pass() -> dict:
    collision_pass = {"status": "PASS", "first_failure": None, "collision_pairs": []}
    return {
        "status": "PASS",
        "robot_collision": {
            "stored_points": {"status": "PASS", "points": []},
            "sampled_path": {"status": "PASS", "samples": []},
            "self_collision": deepcopy(collision_pass),
            "inter_arm_collision": deepcopy(collision_pass),
        },
        "object_scene": {"status": "NOT_CONFIGURED", "source": "UNAVAILABLE"},
        "object_collision": {"status": "NOT_EVALUATED", "collision_pairs": []},
        "environment_scene": {"status": "NOT_CONFIGURED", "source": "UNAVAILABLE"},
        "environment_collision": {"status": "NOT_EVALUATED", "collision_pairs": []},
        "synchronized_sampling": {"max_joint_step_rad": 0.05},
        "collision_summary": {
            "first_failure": None,
            "collision_categories": {},
            "collision_pairs": [],
        },
    }


def report(a=None, b=None, selected_plan=None, **kwargs):
    start_state = kwargs.pop("start_state", START)
    authority = kwargs.pop("authoritative_input_validation", AUTHORITY_PASS)
    return build_phase4_unified_report(
        selected_plan or plan(),
        a or phase4a_all_pass(),
        b or phase4b_all_pass(),
        joint_position_limits=LIMITS,
        start_state=start_state,
        authoritative_input_validation=authority,
        **kwargs,
    )


class Phase4UnifiedValidationTests(unittest.TestCase):
    def test_all_required_sections_appear_once_and_all_pass_can_unlock(self):
        result = report()
        self.assertEqual(tuple(result["checks"]), (
            "plan_identity", "authoritative_inputs", "ik_complete", "joint_position_limits",
            "self_collision", "inter_arm_collision", "object_collision",
            "environment_collision", "sampled_path_collision",
            "start_transition", "joint_continuity", "joint_jump",
            "fk_target", "relative_pose", "fixed_grasp", "common_timeline",
            "velocity", "acceleration",
        ))
        self.assertEqual(result["overall_status"], "PASS")
        self.assertTrue(result["execution_gate"]["execution_ready"])
        self.assertEqual(result["execution_gate"]["execution_ready_label"], "YES")
        self.assertEqual(result["report_version"], REPORT_VERSION)
        self.assertEqual(
            result["scope_policy"]["required_checks"], list(REQUIRED_CHECKS)
        )
        self.assertEqual(
            tuple(item["check"] for item in result["scope_policy"]["deferred_checks"]),
            DEFERRED_CHECKS,
        )
        self.assertEqual(
            result["execution_gate"]["semantic"], EXECUTION_SCOPE_SEMANTIC
        )

    def test_p4_1_maps_complete_and_incomplete_phase3_without_ik(self):
        complete = evaluate_ik_completeness(plan())
        self.assertEqual(complete["status"], "PASS")
        self.assertEqual(complete["checked_joint_count"], 36)
        incomplete = plan()
        incomplete["global_path"].pop()
        failed = evaluate_ik_completeness(incomplete)
        self.assertEqual(failed["status"], "FAIL")
        self.assertEqual(failed["first_failure"]["sample_index"], 2)

    def test_p4_2_checks_every_joint_and_reports_exact_first_violation(self):
        passed = validate_all_joint_positions(plan(), LIMITS)
        self.assertEqual(passed["checked_sample_count"], 3)
        self.assertEqual(passed["checked_joint_count"], 36)
        bad = plan()
        bad["global_path"][1]["left"][2] = 7.0
        bad["global_path"][1]["combined"][2] = 7.0
        failed = validate_all_joint_positions(bad, LIMITS)
        self.assertEqual(failed["status"], "FAIL")
        self.assertEqual(failed["checked_joint_count"], 36)
        self.assertEqual(failed["first_failure"]["sample_index"], 1)
        self.assertEqual(failed["first_failure"]["joint_name"], "left_joint_3")
        self.assertEqual(failed["first_failure"]["actual_value_rad"], 7.0)

    def test_phase4a_and_phase4b_results_are_reused_and_defensively_copied(self):
        a, b = phase4a_all_pass(), phase4b_all_pass()
        result = report(a, b)
        self.assertEqual(result["components"]["phase4a"], a)
        self.assertEqual(result["components"]["phase4b"], b)
        result["components"]["phase4a"]["status"] = "MUTATED"
        self.assertEqual(a["status"], "PASS")

    def test_deterministic_priority_selects_joint_limits_before_fk_and_collision(self):
        selected = plan()
        selected["global_path"][0]["left"][0] = 7.0
        selected["global_path"][0]["combined"][0] = 7.0
        a = phase4a_all_pass()
        a["fk"] = {"status": "FAIL", "first_failure": {"sample_index": 2}}
        b = phase4b_all_pass()
        b["robot_collision"]["self_collision"] = {
            "status": "FAIL", "first_failure": {"sample_index": 1}
        }
        result = report(a, b, selected)
        self.assertEqual(result["overall_status"], "FAIL")
        self.assertEqual(result["first_failure"]["check"], "joint_position_limits")
        self.assertEqual(result["first_failure_priority"], list(CHECK_PRIORITY))

    def test_collision_pair_category_location_and_depth_are_retained(self):
        b = phase4b_all_pass()
        failure = {
            "category": "INTER_ARM", "body_1": "left_J5", "body_2": "right_J5",
            "segment_index": 1, "sample_index": 2, "alpha": 0.5,
            "time_from_start_s": 1.5, "depth_m": 0.004,
        }
        b["robot_collision"]["inter_arm_collision"] = {
            "status": "FAIL", "first_failure": failure,
            "collision_pairs": [{"body_1": "left_J5", "body_2": "right_J5", "category": "INTER_ARM"}],
        }
        b["collision_summary"]["first_failure"] = failure
        result = report(b=b)
        detail = result["diagnostics"]["collision"]["inter_arm_collision"]
        self.assertEqual(detail["evidence"]["first_failure"]["category"], "INTER_ARM")
        self.assertEqual(detail["evidence"]["first_failure"]["depth_m"], 0.004)

    def test_jump_relative_and_timing_diagnostics_are_retained(self):
        result = report()
        jump = result["diagnostics"]["joint_jump"]
        relative = result["diagnostics"]["relative_pose"]
        timing = result["diagnostics"]["timing_dynamics"]
        self.assertEqual(jump["summary"]["maximum_joint_name"], "left_joint_1")
        self.assertEqual(relative["relative_pose"]["max_translation_error_sample_index"], 1)
        self.assertEqual(timing["velocity"]["maximum"]["joint_name"], "left_joint_1")
        self.assertEqual(timing["acceleration"]["maximum"]["sample_index"], 1)

    def test_any_fail_incomplete_or_error_status_is_fail_closed(self):
        scenarios = (
            ("fk", "FAIL", "FAIL"),
            ("relative_pose", "NOT_EVALUATED", "INCOMPLETE"),
            ("fixed_grasp", "UNAVAILABLE", "INCOMPLETE"),
            ("velocity", "TIMEOUT", "INCOMPLETE"),
            ("velocity", "ERROR", "INCOMPLETE"),
        )
        for field, status, overall in scenarios:
            a = phase4a_all_pass()
            a[field]["status"] = status
            with self.subTest(field=field, status=status):
                result = report(a=a)
                self.assertEqual(result["overall_status"], overall)
                self.assertFalse(result["execution_gate"]["execution_ready"])

    def test_deferred_incomplete_checks_remain_truthful_but_do_not_block(self):
        a, b = phase4a_all_pass(), phase4b_all_pass()
        result = report(a, b)
        self.assertEqual(result["overall_status"], "PASS")
        self.assertTrue(result["execution_gate"]["execution_ready"])
        reasons = result["execution_gate"]["blocking_reasons"]
        self.assertNotIn("OBJECT_COLLISION_GEOMETRY_NOT_CONFIGURED", reasons)
        self.assertNotIn("ENVIRONMENT_COLLISION_SCENE_NOT_CONFIGURED", reasons)
        self.assertNotIn("ACCELERATION_LIMIT_UNAVAILABLE", reasons)
        self.assertEqual(result["checks"]["object_collision"]["status"], "NOT_CONFIGURED")
        self.assertEqual(result["checks"]["environment_collision"]["status"], "NOT_CONFIGURED")
        self.assertEqual(result["checks"]["acceleration"]["status"], "LIMIT_UNAVAILABLE")
        acceleration = result["checks"]["acceleration"]["evidence"]
        self.assertEqual(acceleration["compute_status"], "COMPUTED")
        self.assertEqual(acceleration["limit_validation"], "NOT_EVALUATED")
        self.assertEqual(len(result["deferred_warnings"]), 3)

    def test_missing_explicit_start_state_remains_required_blocker(self):
        result = report(start_state=None)
        self.assertEqual(result["checks"]["start_transition"]["status"], "NOT_EVALUATED")
        self.assertEqual(result["overall_status"], "INCOMPLETE")
        self.assertIn(
            "START_TRANSITION_NOT_EVALUATED",
            result["execution_gate"]["blocking_reasons"],
        )
        self.assertFalse(result["execution_gate"]["execution_ready"])

    def test_every_required_failure_class_remains_fail_closed(self):
        scenarios = (
            ("joint_position_limits", "plan", None),
            ("self_collision", "phase4b", "self_collision"),
            ("inter_arm_collision", "phase4b", "inter_arm_collision"),
            ("sampled_path_collision", "phase4b", "sampled_path"),
            ("fk_target", "phase4a", "fk"),
            ("relative_pose", "phase4a", "relative_pose"),
            ("velocity", "phase4a", "velocity"),
        )
        for expected_check, component, field in scenarios:
            selected, a, b = plan(), phase4a_all_pass(), phase4b_all_pass()
            if component == "plan":
                selected["global_path"][0]["left"][0] = 7.0
                selected["global_path"][0]["combined"][0] = 7.0
            elif component == "phase4a":
                a[field]["status"] = "FAIL"
            elif field == "sampled_path":
                b["robot_collision"][field] = {"status": "FAIL", "first_failure": {}}
            else:
                b["robot_collision"][field] = {"status": "FAIL", "first_failure": {}}
            with self.subTest(check=expected_check):
                result = report(a, b, selected)
                self.assertEqual(result["overall_status"], "FAIL")
                self.assertFalse(result["execution_gate"]["execution_ready"])
                self.assertEqual(result["first_failure"]["check"], expected_check)

    def test_fingerprint_is_deterministic_and_sensitive_to_required_data(self):
        selected = plan()
        base = compute_plan_fingerprint(selected, start_state=START)
        self.assertEqual(base, compute_plan_fingerprint(deepcopy(selected), start_state=deepcopy(START)))
        mutations = []
        joint = deepcopy(selected)
        joint["global_path"][1]["left"][0] += 0.01
        mutations.append(joint)
        timestamp = deepcopy(selected)
        timestamp["global_path"][1]["time_from_start_s"] += 0.01
        mutations.append(timestamp)
        object_target = deepcopy(selected)
        object_target["object_samples"][1]["left_target"]["matrix"][0][3] = 0.01
        mutations.append(object_target)
        for changed in mutations:
            with self.subTest():
                self.assertNotEqual(base, compute_plan_fingerprint(changed, start_state=START))
        changed_start = deepcopy(START)
        changed_start["left"][0] = 0.01
        self.assertNotEqual(base, compute_plan_fingerprint(selected, start_state=changed_start))

    def test_component_fingerprint_mismatch_is_stale_and_blocks(self):
        result = report(phase4a_plan_fingerprint="old")
        self.assertEqual(result["checks"]["plan_identity"]["status"], "STALE")
        self.assertEqual(result["overall_status"], "INCOMPLETE")
        self.assertIn("STALE_VALIDATION", result["execution_gate"]["blocking_reasons"])

    def test_authoritative_gate_rechecks_missing_running_and_fingerprint(self):
        accepted = report()
        fingerprint = accepted["plan_fingerprint"]
        self.assertTrue(evaluate_execution_gate(accepted, fingerprint)["execution_ready"])
        self.assertFalse(evaluate_execution_gate(None, fingerprint)["execution_ready"])
        self.assertFalse(evaluate_execution_gate(accepted, "different")["execution_ready"])
        self.assertTrue(evaluate_execution_gate(accepted, "different")["stale"])
        self.assertFalse(evaluate_execution_gate(
            accepted, fingerprint, validation_running=True
        )["execution_ready"])
        legacy = deepcopy(accepted)
        legacy.pop("scope_policy")
        self.assertIn(
            "REPORT_SCOPE_POLICY_MISMATCH",
            evaluate_execution_gate(legacy, fingerprint)["blocking_reasons"],
        )


if __name__ == "__main__":
    unittest.main()
