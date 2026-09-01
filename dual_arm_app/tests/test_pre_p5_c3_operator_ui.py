"""Focused PRE-P5.C3 operator-workflow UI contract tests."""

from __future__ import annotations

from html.parser import HTMLParser
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
HTML_PATH = ROOT / "dual_arm_app/web/index.html"
CONTROLLER_PATH = ROOT / "dual_arm_app/web/digital_twin.js"
PHASE4A_PATH = ROOT / "dual_arm_app/web/digital_twin_phase4_validation.js"
PHASE4B_PATH = ROOT / "dual_arm_app/web/digital_twin_phase4_collision.js"
UNIFIED_PATH = ROOT / "dual_arm_app/web/digital_twin_phase4_unified_validation.js"


def run_start_state_contract(controller: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="pre-p5-c3-start-state-") as temporary:
        directory = Path(temporary)
        (directory / "phase4a.mjs").write_text(
            PHASE4A_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "phase4b.mjs").write_text(
            PHASE4B_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        unified = UNIFIED_PATH.read_text(encoding="utf-8").replace(
            '"./digital_twin_phase4_validation.js"', '"./phase4a.mjs"'
        ).replace(
            '"./digital_twin_phase4_collision.js"', '"./phase4b.mjs"'
        )
        (directory / "unified.mjs").write_text(unified, encoding="utf-8")
        effective_rule = controller[
            controller.index("function normalizeResolvedPhase4StartState"):
            controller.index("function currentPlanMatchesPhase3Authority")
        ]
        (directory / "harness.mjs").write_text(effective_rule + r'''
import {
  buildPhase4UnifiedValidationRequest,
} from "./unified.mjs";
const planStartA = {
  left: [0.1,0.2,0.3,0.4,0.5,0.6],
  right: [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6],
};
const planStartB = {
  left: [1.1,1.2,1.3,1.4,1.5,1.6],
  right: [-1.1,-1.2,-1.3,-1.4,-1.5,-1.6],
};
const firstSample = {
  left: [9,9,9,9,9,9], right: [-9,-9,-9,-9,-9,-9],
};
const planA = {
  ok: true,
  planning_start_state_rad: planStartA,
  planning_start_state_source: "CODE CONFIG A",
  global_path: [firstSample],
};
const planB = {
  ok: true,
  planning_start_state_rad: planStartB,
  planning_start_state_source: "CODE CONFIG B",
  global_path: [firstSample],
};
const explicit = {
  left: [2,2,2,2,2,2], right: [-2,-2,-2,-2,-2,-2], source: "EXPLICIT TEST",
};
const automaticA = resolvePhase4ValidationStartState({plan: planA});
const automaticB = resolvePhase4ValidationStartState({plan: planB});
const overridden = resolvePhase4ValidationStartState({
  plan: planA, explicitStartState: explicit, explicitStartStateKind: "EXPLICIT OFFLINE",
});
const missing = phase4ValidationReadiness({
  planStatus: "READY", plan: {ok:true, global_path:[firstSample]}, authorityMatches: true,
});
const ready = phase4ValidationReadiness({
  planStatus: "READY", plan: planA, authorityMatches: true,
});
const request = buildPhase4UnifiedValidationRequest({
  plan: planA, startState: automaticA.startState,
});
console.log(JSON.stringify({
  automaticA, automaticB, overridden, missing, ready, request, firstSample,
}));
''', encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            raise AssertionError(result.stderr)
        return json.loads(result.stdout)


class UiStructureParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids: list[str] = []
        self.details_depth = 0
        self.id_details_depth: dict[str, int] = {}

    def handle_starttag(self, tag, attrs):
        if tag == "details":
            self.details_depth += 1
        attributes = dict(attrs)
        element_id = attributes.get("id")
        if element_id:
            self.ids.append(element_id)
            self.id_details_depth[element_id] = self.details_depth

    def handle_endtag(self, tag):
        if tag == "details":
            self.details_depth -= 1


class OperatorWorkflowLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.unified = UNIFIED_PATH.read_text(encoding="utf-8")
        cls.start_state_contract = run_start_state_contract(cls.controller)
        cls.parser = UiStructureParser()
        cls.parser.feed(cls.html)

    def test_main_workflow_has_three_ordered_operator_stages(self):
        stage_ids = (
            "digitalTwinOperatorGraspStage",
            "digitalTwinOperatorWaypointStage",
            "digitalTwinPhase5ExecutionSection",
        )
        positions = [self.html.index(f'id="{stage_id}"') for stage_id in stage_ids]
        self.assertEqual(positions, sorted(positions))
        for label in (
            "1. Grasp Setup",
            "2. Center Waypoints",
            "3. Preview &amp; Execute",
        ):
            self.assertIn(label, self.html)

        for technical_stage_id in (
            "digitalTwinOperatorPlanStage",
            "digitalTwinOperatorValidateStage",
        ):
            self.assertGreater(self.parser.id_details_depth[technical_stage_id], 0)

    def test_revision_and_engineering_fields_live_under_details(self):
        for element_id in (
            "digitalTwinGraspRevision",
            "digitalTwinGraspLockGeneration",
            "digitalTwinBackendCalibrationRevision",
            "digitalTwinModelCalibrationRevision",
            "digitalTwinGlobalPlanSource",
            "digitalTwinPhase4CPlanFingerprint",
            "digitalTwinPhase4CGraspRevision",
            "digitalTwinPhase4CCalibrationRevision",
        ):
            with self.subTest(element_id=element_id):
                self.assertGreater(self.parser.id_details_depth[element_id], 0)
        self.assertIn("Advanced / Diagnostics", self.html)

    def test_grasp_editor_is_operator_facing_and_responsive(self):
        for label in (
            "Left Arm Grasp",
            "Right Arm Grasp",
            "Position",
            "Roll [deg]",
            "Pitch [deg]",
            "Yaw [deg]",
            "Lock Grasp",
            "Edit Current Grasp",
            "Start New Grasp from Current Robot Pose",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html)
        self.assertIn("digital-twin-grasp-editors", self.html)
        self.assertIn("grid-template-columns: repeat(2", self.html)

    def test_locked_and_unlocked_states_switch_editor_and_summary(self):
        update = self.controller.split(
            "function updateRigidGraspConfigurationUi()", 1
        )[1].split("function applyBackendRigidGraspState", 1)[0]
        self.assertIn('rigidGraspState.state === "GRASP_UNLOCKED"', update)
        self.assertIn('rigidGraspState.state === "GRASP_LOCKED"', update)
        self.assertIn("lockedSummary.hidden = !locked", update)
        self.assertIn("unlockedEditor.hidden = locked", update)
        self.assertIn("resetFramesButton.hidden = locked", update)
        self.assertIn("element.disabled = !editable", update)
        self.assertIn("button.disabled = frame !== GRASP_SELECTED_FRAMES.CENTER && !editable", update)

    def test_lock_and_edit_use_existing_backend_authority_requests(self):
        self.assertIn("requestLockGraspConfiguration(rigidGraspState.draft, fetchImpl)", self.controller)
        self.assertIn("requestUnlockGraspConfiguration(fetchImpl)", self.controller)
        self.assertIn("applyBackendRigidGraspState", self.controller)
        self.assertIn("invalidatePlanningForGraspChange", self.controller)
        edit = self.controller.split("function editCurrentGrasp", 1)[1].split(
            "async function startNewGraspFromCurrentRobotPose", 1
        )[0]
        self.assertIn("unlockRigidGraspConfiguration(fetchImpl)", edit)
        self.assertNotIn("resetGraspFramesToCurrentRobotPose", edit)

    def test_new_grasp_setup_is_explicit_and_never_auto_locks(self):
        start_new = self.controller.split(
            "async function startNewGraspFromCurrentRobotPose", 1
        )[1].split("function setObjectPreviewVisibility", 1)[0]
        self.assertLess(
            start_new.index("await unlockRigidGraspConfiguration(fetchImpl)"),
            start_new.index('sessionAction: "NEW_GRASP"'),
        )
        self.assertIn('unlockedState.state !== "GRASP_UNLOCKED"', start_new)
        self.assertNotIn("await lockRigidGraspConfiguration", start_new)
        self.assertNotIn("return lockRigidGraspConfiguration", start_new)
        self.assertNotIn("requestLockGraspConfiguration", start_new)

    def test_unlock_does_not_install_a_global_reset_requirement(self):
        unlock = self.controller.split(
            "async function unlockRigidGraspConfiguration", 1
        )[1].split("function setObjectPreviewUiError", 1)[0]
        lock = self.controller.split(
            "async function lockRigidGraspConfiguration", 1
        )[1].split("async function unlockRigidGraspConfiguration", 1)[0]
        for forbidden in ("resetRequired", "RESET_REQUIRED", "alignmentStatus"):
            self.assertNotIn(forbidden, unlock)
            self.assertNotIn(forbidden, lock)

    def test_fallback_selection_and_move_rotate_edit_only_selected_side(self):
        for element_id in (
            "digitalTwinSelectObjectFrame",
            "digitalTwinSelectLeftGraspFrame",
            "digitalTwinSelectRightGraspFrame",
            "digitalTwinGraspGizmoTranslate",
            "digitalTwinGraspGizmoRotate",
        ):
            self.assertIn(f'id="{element_id}"', self.html)
        self.assertIn("setRigidGraspSelectedFrame(frame)", self.controller)
        self.assertIn("setObjectDragEnabled(true)", self.controller)
        self.assertIn("setRigidGraspGizmoMode(\"translate\")", self.controller)
        self.assertIn("setRigidGraspGizmoMode(\"rotate\")", self.controller)
        self.assertIn("[side]: copyGraspPose(normalized)", self.controller)

    def test_locked_object_remains_movable_without_changing_local_grasp(self):
        selection = self.controller.split(
            "function setRigidGraspSelectedFrame", 1
        )[1].split("function setRigidGraspGizmoMode", 1)[0]
        self.assertIn("frame = GRASP_SELECTED_FRAMES.CENTER", selection)
        object_change = self.controller.split(
            'objectTransformControls.addEventListener("objectChange"', 1
        )[1].split('objectTransformControls.addEventListener("change"', 1)[0]
        self.assertIn("applyObjectPreviewPose", object_change)
        self.assertIn("updateRigidGraspDraftSide", object_change)
        self.assertIn('rigidGraspState.state !== "GRASP_UNLOCKED"', object_change)
        self.assertIn("computeWorldGraspFrameMatrices(pose, content)", self.controller)

    def test_waypoint_and_plan_controls_remain_authority_gated(self):
        waypoint_ui = self.controller.split(
            "function updateObjectWaypointPlanningUi()", 1
        )[1].split("function setObjectPlanningWaypoints", 1)[0]
        self.assertIn('button.disabled = !locked', waypoint_ui)
        self.assertIn("phase3PlanningAuthorityIdentity()", waypoint_ui)
        self.assertIn("planButton.disabled = state.planning || !prerequisitesReady", waypoint_ui)
        self.assertIn(">Save Current Center as Waypoint</button>", self.html)
        self.assertIn(">Generate Trajectory</button>", self.html)

    def test_plan_still_drives_planned_ghost_and_existing_validation_path(self):
        planner = self.controller.split("async function planObjectGlobal", 1)[1].split(
            "function bindObjectWaypointPlanningControls", 1
        )[0]
        self.assertIn("loadObjectGlobalPlanPreview(response)", planner)
        self.assertIn("globalPlanToPlannedTrajectory(plan)", self.controller)
        validation_bind = self.controller.split(
            "function bindPhase4UnifiedValidationControls", 1
        )[1].split("function diagnosticValue", 1)[0]
        self.assertIn("validateCurrentGlobalPlanPhase4Unified()", validation_bind)
        self.assertIn("Validate Trajectory", self.html)

    def test_calibration_summary_is_concise_and_truthful(self):
        self.assertIn("Model Alignment:", self.html)
        self.assertIn(
            "Physical world calibration not completed — offline planning only.",
            self.html,
        )
        self.assertIn('worldCalibrationState.matches ? "OK"', self.controller)
        self.assertIn('worldCalibrationState.physicallyCalibrated', self.controller)
        self.assertEqual(
            self.parser.id_details_depth["digitalTwinOperatorModelAlignment"], 0
        )

    def test_existing_invalidation_and_backend_identity_are_not_duplicated(self):
        invalidation = self.controller.split(
            "function invalidatePlanningForGraspChange", 1
        )[1].split("function updateRigidGraspDraftSide", 1)[0]
        self.assertIn("clearPlannedTrajectory()", invalidation)
        self.assertIn("invalidateAllPhase4Validation(reason)", invalidation)
        self.assertIn("requestGraspConfiguration(fetchImpl)", self.controller)
        self.assertIn("phase3PlanningAuthorityIdentity()", self.controller)

    def test_html_ids_are_unique(self):
        self.assertEqual(len(self.parser.ids), len(set(self.parser.ids)))
        self.assertEqual(self.parser.details_depth, 0)

    def test_ready_plan_with_embedded_planning_start_enables_validate(self):
        self.assertTrue(self.start_state_contract["ready"]["ready"])

    def test_plan_planning_start_is_effective_without_override(self):
        effective = self.start_state_contract["automaticA"]
        self.assertTrue(effective["available"])
        self.assertEqual(effective["kind"], "PLANNING START STATE")
        self.assertEqual(effective["startState"]["source"], "CODE CONFIG A")

    def test_explicit_phase4_override_has_priority(self):
        effective = self.start_state_contract["overridden"]
        self.assertEqual(effective["kind"], "EXPLICIT OFFLINE")
        self.assertEqual(effective["startState"]["source"], "EXPLICIT TEST")
        self.assertEqual(effective["startState"]["left"], [2] * 6)

    def test_first_planned_sample_is_not_an_implicit_fallback(self):
        contract = self.start_state_contract
        self.assertNotEqual(
            contract["automaticA"]["startState"]["left"],
            contract["firstSample"]["left"],
        )
        resolver = self.controller.split(
            "function resolvePhase4ValidationStartState", 1
        )[1].split("function phase4ValidationReadiness", 1)[0]
        self.assertNotIn("global_path", resolver)

    def test_missing_start_keeps_validate_readiness_false(self):
        self.assertFalse(self.start_state_contract["missing"]["ready"])

    def test_missing_start_has_accurate_operator_reason(self):
        self.assertEqual(
            self.start_state_contract["missing"]["reason"],
            "Validation start state unavailable.",
        )

    def test_ready_message_and_button_share_one_readiness_predicate(self):
        self.assertEqual(
            self.start_state_contract["ready"]["reason"],
            "Trajectory ready for validation.",
        )
        ui = self.controller.split(
            "function updatePhase4UnifiedValidationUi", 1
        )[1].split("function invalidatePhase4UnifiedValidation", 1)[0]
        self.assertIn("const readiness = currentPhase4ValidationReadiness()", ui)
        self.assertIn("reason = readiness.reason", ui)
        self.assertIn("validateButton.disabled = !readiness.ready", ui)

    def test_unified_request_contains_resolved_explicit_start_state(self):
        request = self.start_state_contract["request"]
        automatic = self.start_state_contract["automaticA"]["startState"]
        self.assertEqual(request["start_state"], automatic)
        handler = self.controller.split(
            "async function validateCurrentGlobalPlanPhase4Unified", 1
        )[1].split("function bindPhase4UnifiedValidationControls", 1)[0]
        self.assertIn("startState: readiness.effectiveStart.startState", handler)

    def test_plan_replacement_resolves_start_from_new_current_plan(self):
        first = self.start_state_contract["automaticA"]["startState"]
        second = self.start_state_contract["automaticB"]["startState"]
        self.assertNotEqual(first["left"], second["left"])
        self.assertEqual(second["source"], "CODE CONFIG B")
        loader = self.controller.split(
            "function loadObjectGlobalPlanPreview", 1
        )[1].split("async function planObjectGlobal", 1)[0]
        self.assertIn('invalidateAllPhase4Validation("NEW GLOBAL PLAN LOADED")', loader)

    def test_planning_start_change_invalidates_current_plan_and_phase4(self):
        loader = self.controller.split(
            "async function loadPlanningStartState", 1
        )[1].split("function objectWaypointPlanningStateSnapshot", 1)[0]
        self.assertIn("previousIdentity !== currentIdentity", loader)
        self.assertIn(
            'invalidatePlanningForGraspChange("PLANNING START STATE CHANGED")',
            loader,
        )

    def test_grasp_change_still_invalidates_phase4(self):
        invalidator = self.controller.split(
            "function invalidatePlanningForGraspChange", 1
        )[1].split("function updateRigidGraspDraftSide", 1)[0]
        self.assertIn("invalidateAllPhase4Validation(reason)", invalidator)

    def test_calibration_change_still_invalidates_phase4(self):
        loader = self.controller.split(
            "async function loadWorldCalibrationState", 1
        )[1].split("function unifiedCheckStatus", 1)[0]
        self.assertIn("invalidatePlanningForGraspChange", loader)
        self.assertIn("WORLD CALIBRATION REVISION CHANGED", loader)

    def test_start_transition_remains_required(self):
        self.assertIn('"start_transition"', self.unified)
        required = self.unified.split(
            "export const PHASE4_REQUIRED_CHECKS", 1
        )[1].split("]);", 1)[0]
        self.assertIn('"start_transition"', required)

    def test_unified_endpoint_remains_unchanged(self):
        self.assertIn(
            '"/api/digital-twin/validate-phase4"',
            self.unified,
        )
        binding = self.controller.split(
            "function bindPhase4UnifiedValidationControls", 1
        )[1].split("function diagnosticValue", 1)[0]
        self.assertIn("validateCurrentGlobalPlanPhase4Unified()", binding)

    def test_stop_and_execution_semantics_are_unchanged(self):
        stop = self.html.split("async function stopBoth()", 1)[1].split(
            "function makeHoldButton", 1
        )[0]
        self.assertNotIn("getExecutionGateState", stop)
        self.assertNotIn("runProgramButton", stop)
        self.assertNotIn("/api/program/run", self.unified)


if __name__ == "__main__":
    unittest.main()
