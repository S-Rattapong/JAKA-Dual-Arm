"""Focused PRE-P5.C3 Center/grasp-frame snapshot initialization tests."""

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
TRANSFORMS_PATH = ROOT / "dual_arm_app/web/digital_twin_object_grasp_preview.js"
PHASE4_VALIDATION_PATH = ROOT / "dual_arm_app/backend/phase4_trajectory_validation.py"


def run_geometry_contract() -> dict:
    with tempfile.TemporaryDirectory(prefix="pre-p5-c3-frame-reset-") as temporary:
        directory = Path(temporary)
        (directory / "transforms.mjs").write_text(
            TRANSFORMS_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "harness.mjs").write_text(
            r'''
import {
  deriveGraspFrameResetFromWorldTips,
  matrix4FromTranslationRpy,
  multiplyMatrix4,
  verifyGraspFrameAlignment,
} from "./transforms.mjs";

// Arbitrary non-zero model-tip transforms, representative of a non-zero
// dual-arm joint configuration after the visible URDF has updated its FK.
const worldTLeft = matrix4FromTranslationRpy({
  translationM: [-1.18, 0.42, 1.31], rpyRad: [0.23, -0.31, 0.47],
});
const worldTRight = matrix4FromTranslationRpy({
  translationM: [-1.44, -0.36, 1.09], rpyRad: [-0.41, 0.28, -0.19],
});
const reset = deriveGraspFrameResetFromWorldTips(worldTLeft, worldTRight);
const alignment = verifyGraspFrameAlignment({
  centerPose: reset.centerPose,
  draft: reset.draft,
  worldTLeft,
  worldTRight,
});
const shiftedWorldTLeft = worldTLeft.map((row) => [...row]);
shiftedWorldTLeft[0][3] += 0.000001;
const rejectedAlignment = verifyGraspFrameAlignment({
  centerPose: reset.centerPose,
  draft: reset.draft,
  worldTLeft: shiftedWorldTLeft,
  worldTRight,
});
console.log(JSON.stringify({
  worldTLeft,
  worldTRight,
  reset,
  alignment,
  rejectedAlignment,
  reconstructedLeft: multiplyMatrix4(reset.worldTCenter, reset.centerTLeft),
  reconstructedRight: multiplyMatrix4(reset.worldTCenter, reset.centerTRight),
}));
''',
            encoding="utf-8",
        )
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            raise AssertionError(result.stderr)
        return json.loads(result.stdout)


def assert_matrix_almost_equal(
    case: unittest.TestCase, actual: list[list[float]], expected: list[list[float]]
) -> None:
    for row in range(4):
        for column in range(4):
            case.assertAlmostEqual(actual[row][column], expected[row][column], places=10)


class IdParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: list[str] = []

    def handle_starttag(self, _tag, attrs):
        attributes = dict(attrs)
        if attributes.get("id"):
            self.ids.append(attributes["id"])


class FrameInitializationGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = run_geometry_contract()

    def test_center_is_tip_midpoint_and_world_aligned(self):
        center = self.result["reset"]["centerPose"]
        self.assertEqual(center["rpyRad"], [0, 0, 0])
        for actual, expected in zip(center["translationM"], [-1.31, 0.03, 1.2]):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_left_and_right_drafts_reconstruct_displayed_tips(self):
        assert_matrix_almost_equal(
            self, self.result["reconstructedLeft"], self.result["worldTLeft"]
        )
        assert_matrix_almost_equal(
            self, self.result["reconstructedRight"], self.result["worldTRight"]
        )

    def test_arbitrary_nonzero_model_pose_preserves_tip_orientation(self):
        self.assertNotEqual(self.result["reset"]["draft"]["left"]["rpyRad"], [0, 0, 0])
        self.assertNotEqual(self.result["reset"]["draft"]["right"]["rpyRad"], [0, 0, 0])

    def test_alignment_self_check_accepts_reconstruction_and_reports_residuals(self):
        alignment = self.result["alignment"]
        self.assertTrue(alignment["ok"])
        self.assertLessEqual(
            alignment["maxTranslationM"], alignment["translationToleranceM"]
        )
        self.assertLessEqual(
            alignment["maxOrientationRad"], alignment["orientationToleranceRad"]
        )

    def test_alignment_self_check_fails_closed_for_shifted_snapshot(self):
        alignment = self.result["rejectedAlignment"]
        self.assertFalse(alignment["ok"])
        self.assertGreater(
            alignment["maxTranslationM"], alignment["translationToleranceM"]
        )


class FrameInitializationControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.reset_block = cls.controller.split(
            "function resetGraspFramesToCurrentRobotPose", 1
        )[1].split(
            "function initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady", 1
        )[0]
        cls.initialization_block = cls.controller.split(
            "function initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady", 1
        )[1].split("function resetObjectPreview", 1)[0]
        cls.edit_block = cls.controller.split(
            "function editCurrentGrasp", 1
        )[1].split("async function startNewGraspFromCurrentRobotPose", 1)[0]
        cls.start_new_block = cls.controller.split(
            "async function startNewGraspFromCurrentRobotPose", 1
        )[1].split("function setObjectPreviewVisibility", 1)[0]

    def test_snapshot_reads_visible_left_and_right_model_tips(self):
        source = self.controller.split(
            "function currentDisplayedModelTipWorldMatrices", 1
        )[1].split("function resetGraspFramesToCurrentRobotPose", 1)[0]
        self.assertIn("robot.links[MODEL_TCP_LINKS.left]", source)
        self.assertIn("robot.links[MODEL_TCP_LINKS.right]", source)
        self.assertIn("robot.updateMatrixWorld(true)", source)
        self.assertIn("threeMatrixToRows(leftTip.matrixWorld)", source)
        self.assertIn("threeMatrixToRows(rightTip.matrixWorld)", source)

    def test_live_and_offline_main_pose_sources_are_accepted_without_commands(self):
        source = self.controller.split(
            "function currentDisplayedModelTipWorldMatrices", 1
        )[1].split("function resetGraspFramesToCurrentRobotPose", 1)[0]
        self.assertIn('"LIVE JOINT FEEDBACK"', source)
        self.assertIn('"OFFLINE CODE CONFIG"', source)
        self.assertIn("deriveGraspFrameResetFromWorldTips(tips.left, tips.right)", self.reset_block)
        for forbidden in ("fetch(", "joint_move", "linear_move", "/api/program/run"):
            self.assertNotIn(forbidden, self.reset_block)

    def test_fresh_unlocked_load_is_one_shot_and_locked_load_is_preserved(self):
        backend = self.controller.split("function applyBackendRigidGraspState", 1)[1].split(
            "async function loadRigidGraspConfiguration", 1
        )[0]
        self.assertIn("initialGraspBackendStateObserved", backend)
        self.assertIn("initialGraspFrameSnapshotPending = true", backend)
        self.assertIn('rigidGraspState.state === "GRASP_LOCKED"', self.initialization_block)
        self.assertIn('rigidGraspState.state !== "GRASP_UNLOCKED"', self.initialization_block)
        self.assertIn("!initialGraspFrameSnapshotPending", self.initialization_block)
        self.assertIn("initialGraspFrameSnapshotComplete = true", self.initialization_block)
        locked_placement = self.initialization_block.split(
            'if (rigidGraspState.state === "GRASP_LOCKED")', 1
        )[1].split('if (rigidGraspState.state !== "GRASP_UNLOCKED"', 1)[0]
        self.assertNotIn("rigidGraspState.draft =", locked_placement)
        self.assertNotIn("rigidGraspState.lockedSnapshot =", locked_placement)
        set_joints = self.controller.split("function setJointValues", 1)[1].split(
            "function dualArmPoseMatches", 1
        )[0]
        self.assertNotIn("resetGraspFramesToCurrentRobotPose", set_joints)

    def test_reset_updates_draft_editors_visuals_and_invalidates_authority(self):
        for required in (
            "verifyGraspFrameAlignment",
            "rigidGraspState.draft = copyGraspContent(resetGeometry.draft)",
            "applyObjectPreviewPose(resetGeometry.centerPose)",
            "writeRigidGraspEditorFields()",
            '"GRASP FRAMES RESET FROM CURRENT ROBOT POSE"',
        ):
            self.assertIn(required, self.reset_block)

    def test_alignment_failure_happens_before_draft_mutation(self):
        self.assertLess(
            self.reset_block.index("if (!alignment.ok)"),
            self.reset_block.index(
                "rigidGraspState.draft = copyGraspContent(resetGeometry.draft)"
            ),
        )
        self.assertIn("Grasp frame alignment self-check failed", self.reset_block)
        self.assertIn('rigidGraspState.alignmentStatus = "ALIGNMENT ERROR"', self.reset_block)

    def test_edit_current_grasp_preserves_existing_unlock_semantics(self):
        self.assertIn("unlockRigidGraspConfiguration(fetchImpl)", self.edit_block)
        self.assertNotIn("resetGraspFramesToCurrentRobotPose", self.edit_block)
        self.assertNotIn("currentDisplayedModelTipWorldMatrices", self.edit_block)

    def test_start_new_grasp_unlocks_then_resets_and_remains_unlocked(self):
        unlock_position = self.start_new_block.index(
            "await unlockRigidGraspConfiguration(fetchImpl)"
        )
        reset_position = self.start_new_block.index(
            'resetGraspFramesToCurrentRobotPose({ sessionAction: "NEW_GRASP" })'
        )
        self.assertLess(unlock_position, reset_position)
        self.assertIn('unlockedState.state !== "GRASP_UNLOCKED"', self.start_new_block)
        self.assertNotIn("await lockRigidGraspConfiguration", self.start_new_block)
        self.assertNotIn("return lockRigidGraspConfiguration", self.start_new_block)
        self.assertNotIn("requestLockGraspConfiguration", self.start_new_block)

    def test_start_new_and_reset_use_actual_model_snapshot_only(self):
        self.assertIn("currentDisplayedModelTipWorldMatrices()", self.reset_block)
        self.assertNotIn("plannedRobot", self.reset_block)
        self.assertNotIn("plannedRobot", self.start_new_block)

    def test_reset_is_disabled_and_rejected_while_locked(self):
        ui = self.controller.split("function updateRigidGraspConfigurationUi", 1)[1].split(
            "function applyBackendRigidGraspState", 1
        )[0]
        self.assertIn("resetFramesButton.disabled = !editable", ui)
        self.assertIn('rigidGraspState.state !== "GRASP_UNLOCKED"', self.reset_block)
        self.assertIn("Edit/Unlock the grasp before resetting frames", self.reset_block)
        self.assertIn(
            'id="digitalTwinResetObjectPreview" type="button" disabled', self.html
        )
        self.assertIn("resetFramesButton.hidden = locked", ui)
        self.assertIn("Reset Frames is available after unlocking", self.html)

    def test_locked_ui_has_distinct_edit_and_new_setup_actions(self):
        self.assertIn("Edit Current Grasp", self.html)
        self.assertIn("Start New Grasp from Current Robot Pose", self.html)
        self.assertIn(
            "This grasp configuration is preserved from the current planning state.",
            self.html,
        )
        self.assertIn('id="digitalTwinStartNewGraspFromCurrentRobotPose"', self.html)
        self.assertIn("editCurrentGrasp()", self.controller)
        self.assertIn("startNewGraspFromCurrentRobotPose()", self.controller)

    def test_synthetic_mesh_is_absent_but_all_three_frames_remain(self):
        create = self.controller.split("function createObjectGraspPreview", 1)[1].split(
            "function objectDragStateSnapshot", 1
        )[0]
        self.assertNotIn("BoxGeometry", create)
        self.assertNotIn("Synthetic Workpiece", create)
        self.assertIn('objectFrame.name = "Center / Coordination Frame C"', create)
        self.assertIn('createGraspFrame("Left Grasp Frame L"', create)
        self.assertIn('createGraspFrame("Right Grasp Frame R"', create)
        self.assertNotIn('id="digitalTwinShowObject"', self.html)
        self.assertIn("Show Center Frame", self.html)
        self.assertIn("Show Grasp Frames", self.html)

    def test_planned_ghost_and_phase4_start_transition_are_not_modified_by_reset(self):
        self.assertNotIn("plannedRobot", self.reset_block)
        self.assertNotIn("plannedPreviewState", self.reset_block)
        self.assertIn("invalidateAllPhase4Validation(reason)", self.controller)
        validation = PHASE4_VALIDATION_PATH.read_text(encoding="utf-8")
        self.assertIn("def _start_transition_payload", validation)
        self.assertIn("analyze_joint_transition", validation)
        self.assertNotIn("start_transition", self.reset_block)

    def test_center_waypoint_workflow_and_operator_terms_are_present(self):
        for text in (
            "2. Center Waypoints",
            "Save Current Center as Waypoint",
            "Reset Frames to Current Robot Pose",
            "Apply Center Pose",
        ):
            self.assertIn(text, self.html)
        self.assertIn('id="digitalTwinObjectWaypointAddCurrent"', self.html)
        self.assertIn("addObjectPlanningWaypoint", self.controller)

    def test_no_physical_tcp_calibration_claim_or_motion_path_is_added(self):
        self.assertIn("Digital Twin left_J6/right_J6 model-tip transforms", self.controller)
        self.assertNotIn("physically calibrated TCP", self.reset_block.lower())
        self.assertNotIn("controller TCP", self.reset_block)
        for forbidden in ("/api/program/run", "joint_move(", "linear_move("):
            self.assertNotIn(forbidden, self.reset_block)

    def test_html_ids_remain_unique(self):
        parser = IdParser()
        parser.feed(self.html)
        duplicates = sorted({value for value in parser.ids if parser.ids.count(value) > 1})
        self.assertEqual(duplicates, [])


if __name__ == "__main__":
    unittest.main()
