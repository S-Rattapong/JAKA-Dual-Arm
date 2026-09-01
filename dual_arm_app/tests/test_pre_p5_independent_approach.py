from __future__ import annotations

import unittest
from pathlib import Path

from dual_arm_app.backend.object_global_planning import normalize_object_global_plan_request
from dual_arm_app.tests.test_digital_twin_phase3_planning import (
    EchoPlanningAdapter, request_payload, run_test_plan,
)
from dual_arm_app.backend.object_trajectory_ik import CanonicalJointPositionLimits
from dual_arm_app.backend.phase4_trajectory_validation import validate_common_timeline
from dual_arm_app.backend.phase4_collision_validation import phase3_plan_to_robot_trajectory
from dual_arm_app.backend.phase4_unified_validation import (
    compute_plan_fingerprint,
    evaluate_ik_completeness,
    validate_all_joint_positions,
)

ROOT = Path(__file__).resolve().parents[2]
HTML = (ROOT / "dual_arm_app/web/index.html").read_text(encoding="utf-8")
CONTROLLER = (ROOT / "dual_arm_app/web/digital_twin.js").read_text(encoding="utf-8")
PLANNING = (ROOT / "dual_arm_app/web/digital_twin_phase3_planning.js").read_text(encoding="utf-8")

APPROACH = [{
    "identifier": "A0",
    "left": {"translation_m": [0.0, 0.30, 0.8], "rpy_rad": [0.0, 0.0, 0.0]},
    "right": {"translation_m": [0.0, -0.30, 0.8], "rpy_rad": [0.0, 0.0, 0.0]},
}]

def approach_request():
    payload = request_payload()
    payload["initial_joint_state_rad"] = {"left": [0.0] * 6, "right": [0.0] * 6}
    payload["planning_start_state_source"] = "TEST CURRENT STATE"
    payload["approach_waypoints"] = APPROACH
    return payload


class IndependentApproachContractTests(unittest.TestCase):
    def test_request_normalizes_independent_left_right_targets(self):
        payload = approach_request()
        normalized = normalize_object_global_plan_request(payload)
        self.assertEqual(len(normalized["approach_waypoints"]), 1)
        self.assertEqual(normalized["approach_waypoints"][0]["identifier"], "A0")
        self.assertEqual(normalized["approach_waypoints"][0]["left"]["translation_m"][1], 0.30)
        self.assertEqual(normalized["approach_waypoints"][0]["right"]["translation_m"][1], -0.30)

    def test_planner_embeds_pre_rigid_approach_and_uses_its_end_as_rigid_start(self):
        payload = approach_request()
        payload["initial_joint_state_rad"] = {"left": [0.0] * 6, "right": [0.0] * 6}
        payload["planning_start_state_source"] = "EXPLICIT TEST START"
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        result = run_test_plan(payload, EchoPlanningAdapter(), limits)
        self.assertTrue(result["ok"])
        self.assertTrue(result["approach"]["enabled"])
        self.assertEqual(result["approach"]["status"], "READY")
        self.assertGreaterEqual(len(result["approach_path"]), 2)
        self.assertIn("pre_approach_start_state_rad", result)
        self.assertEqual(result["planning_start_state_rad"]["left"], result["pre_approach_start_state_rad"]["left"])
        self.assertEqual(result["planning_start_state_rad"]["right"], result["pre_approach_start_state_rad"]["right"])
        self.assertEqual(result["rigid_planning_start_state_rad"]["left"], result["approach_path"][-1]["left"])
        self.assertEqual(result["rigid_planning_start_state_rad"]["right"], result["approach_path"][-1]["right"])
        self.assertGreater(len(result["combined_path"]), len(result["global_path"]))
        self.assertEqual(len(result["combined_path"]), len(result["combined_object_samples"]))
        self.assertEqual(result["combined_path"][0]["time_from_start_s"], 0.0)
        self.assertEqual(result["combined_path"][-1]["time_from_start_s"], result["combined_duration_s"])


    def test_first_approach_joint_branch_must_be_continuous_from_planning_start(self):
        class JumpAdapter(EchoPlanningAdapter):
            def solve_arm_ik(self, **kwargs):
                self.ik_calls += 1
                from dual_arm_app.backend.object_trajectory_ik import ArmIkSolution
                return ArmIkSolution(True, (2.0,) * 6, "synthetic first-approach branch jump")

        payload = approach_request()
        payload["initial_joint_state_rad"] = {"left": [0.0] * 6, "right": [0.0] * 6}
        payload["planning_start_state_source"] = "EXPLICIT TEST START"
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        result = run_test_plan(payload, JumpAdapter(), limits)
        self.assertFalse(result["ok"])
        self.assertEqual(result["planner_status"], "FAILED")
        self.assertEqual(result["failure"]["reason"], "INDEPENDENT_APPROACH_START_TRANSITION")
        self.assertGreater(result["failure"]["maximum_shortest_angular_joint_step_rad"], 1.0)

    def test_phase4_uses_complete_approach_plus_rigid_motion_authority(self):
        payload = approach_request()
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        result = run_test_plan(payload, EchoPlanningAdapter(), limits)
        timeline = validate_common_timeline(result)
        self.assertEqual(timeline["status"], "COMMON_TIMELINE_STRUCTURALLY_VALID")
        self.assertEqual(timeline["joint_sample_count"], len(result["combined_path"]))
        self.assertEqual(timeline["rigid_joint_sample_count"], len(result["global_path"]))
        completeness = evaluate_ik_completeness(result)
        self.assertEqual(completeness["status"], "PASS")
        self.assertEqual(completeness["source"], "PHASE3_COMBINED_SELECTED_PATH")
        positions = validate_all_joint_positions(result, limits)
        self.assertEqual(positions["status"], "PASS")
        self.assertEqual(positions["checked_sample_count"], len(result["combined_path"]))
        collision_path = phase3_plan_to_robot_trajectory(result)
        self.assertEqual(len(collision_path["points"]), len(result["combined_path"]))
        self.assertEqual(collision_path["points"][0]["time_from_start_s"], 0.0)
        self.assertEqual(
            collision_path["points"][-1]["time_from_start_s"],
            result["combined_duration_s"],
        )

    def test_center_waypoint_orientation_is_execution_relevant_in_phase4_fingerprint(self):
        payload = approach_request()
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        baseline = run_test_plan(payload, EchoPlanningAdapter(), limits)
        rotated_payload = approach_request()
        rotated_payload["waypoints"][1]["rpy_rad"] = [0.0, 0.0, 0.2]
        rotated = run_test_plan(rotated_payload, EchoPlanningAdapter(), limits)
        self.assertNotEqual(compute_plan_fingerprint(baseline), compute_plan_fingerprint(rotated))

    def test_approach_is_execution_relevant_in_phase4_fingerprint(self):
        payload = request_payload()
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        without_approach = run_test_plan(payload, EchoPlanningAdapter(), limits)
        payload["approach_waypoints"] = APPROACH
        with_approach = run_test_plan(payload, EchoPlanningAdapter(), limits)
        self.assertNotEqual(
            compute_plan_fingerprint(without_approach),
            compute_plan_fingerprint(with_approach),
        )

    def test_start_to_first_approach_is_fail_closed_when_not_near_current_state(self):
        payload = request_payload()
        payload["approach_waypoints"] = APPROACH
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        result = run_test_plan(payload, EchoPlanningAdapter(), limits)
        self.assertFalse(result["ok"])
        self.assertEqual(result["planner_status"], "FAILED")
        self.assertEqual(result["failure"]["reason"], "INDEPENDENT_APPROACH_START_TRANSITION")
        self.assertEqual(result["failure"]["failed_sample_index"], 0)

    def test_preview_and_planned_trajectory_use_complete_combined_path(self):
        self.assertIn("plan.combined_path", PLANNING)
        self.assertIn("plan.combined_object_samples", PLANNING)
        sampler = PLANNING.split("export function sampleIntegratedObjectGlobalPlan", 1)[1].split(
            "export function globalPlanToPlannedTrajectory", 1
        )[0]
        self.assertIn("const path = Array.isArray(plan.combined_path)", sampler)
        self.assertIn("interpolate(firstPath.left, secondPath.left, alpha)", sampler)
        planned = PLANNING.split("export function globalPlanToPlannedTrajectory", 1)[1].split(
            "export async function requestObjectGlobalPlan", 1
        )[0]
        self.assertIn("plan.combined_path", planned)

    def test_operator_ui_captures_approach_before_lock(self):
        for element_id in (
            "digitalTwinApproachWaypointNewId",
            "digitalTwinApproachWaypointAddCurrent",
            "digitalTwinApproachWaypointClear",
            "digitalTwinApproachWaypointList",
        ):
            self.assertIn(f'id="{element_id}"', HTML)
        capture = CONTROLLER.split("function addCurrentIndependentApproachWaypoint", 1)[1].split(
            "function clearIndependentApproachWaypoints", 1
        )[0]
        self.assertIn('rigidGraspState.state !== "GRASP_UNLOCKED"', capture)
        self.assertIn("computeWorldGraspFrameMatrices", capture)
        self.assertIn("poseFromRigidMatrix4", capture)


    def test_start_new_grasp_clears_stale_approach_targets(self):
        start_new = CONTROLLER.split("async function startNewGraspFromCurrentRobotPose", 1)[1].split(
            "function setObjectPreviewVisibility", 1
        )[0]
        self.assertIn("objectWaypointPlanningState.approachWaypoints = [];", start_new)
        self.assertIn("updateApproachWaypointUi();", start_new)

    def test_plan_request_carries_approach_without_changing_locked_center_waypoints(self):
        self.assertIn("approachWaypoints: objectWaypointPlanningState.approachWaypoints", CONTROLLER)
        self.assertIn("approach_waypoints: normalizeApproachWaypoints(approachWaypoints)", PLANNING)
        self.assertIn("Center waypoints remain rigid-only", HTML)

    def test_approach_waypoints_render_as_full_editable_pose_cards(self):
        renderer = CONTROLLER.split("function updateApproachWaypointUi", 1)[1].split(
            "function moveIndependentApproachWaypoint", 1
        )[0]
        self.assertIn('card.className = "digital-twin-approach-waypoint-card"', renderer)
        self.assertIn('"Left Target"', renderer)
        self.assertIn('"Right Target"', renderer)
        self.assertIn('`Roll [${rotationUnit.shortLabel}]`', renderer)
        self.assertIn("writeOperatorRotationInput", renderer)
        self.assertIn("readOperatorRotationInput", renderer)
        self.assertIn('button.textContent = label', renderer)
        self.assertIn('setIndependentApproachWaypoints(next', renderer)
        self.assertIn("A-last → Final Grasp is an implicit pre-rigid segment", HTML)

    def test_integrated_progress_uses_complete_approach_plus_rigid_duration(self):
        self.assertIn("function integratedPlanDurationS(plan)", CONTROLLER)
        state = CONTROLLER.split("function integratedPlanStateSnapshot", 1)[1].split(
            "function getIntegratedPlanPreviewState", 1
        )[0]
        self.assertIn("durationS: integratedPlanDurationS(plan)", state)
        self.assertIn("sampleCount: path.length", state)
        ui = CONTROLLER.split("function updateObjectGlobalPlanUi", 1)[1].split(
            "function deactivateIntegratedPlanPreview", 1
        )[0]
        self.assertIn("digitalTwinGlobalPlanDuration: plan", ui)
        self.assertIn("integratedPlanDurationS(plan).toFixed(2)", ui)
        self.assertIn("scrubber.max = String(integratedPlanDurationS(plan))", ui)
        self.assertIn("integratedPlanPath(plan).length - 1", ui)

    def test_web_normalizer_fail_closes_on_malformed_combined_timeline(self):
        normalizer = PLANNING.split("export function normalizeObjectGlobalPlanResponse", 1)[1].split(
            "function interpolate", 1
        )[0]
        self.assertIn("combinedPath.length < 2", normalizer)
        self.assertIn("combinedSamples.length !== combinedPath.length", normalizer)
        self.assertIn("combined object and joint timestamps must match exactly", normalizer)
        self.assertIn("combined plan timestamps must be strictly increasing", normalizer)
        self.assertIn("combined_duration_s must equal the final combined timestamp", normalizer)

if __name__ == "__main__":
    unittest.main()
