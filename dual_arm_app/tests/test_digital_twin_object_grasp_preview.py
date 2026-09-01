"""Offline tests for Phase 1E.2 object and grasp-frame visualization."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = REPOSITORY_ROOT / "dual_arm_app" / "web"
HELPER_PATH = WEB_ROOT / "digital_twin_object_grasp_preview.js"
CONTROLLER_PATH = WEB_ROOT / "digital_twin.js"
HTML_PATH = WEB_ROOT / "index.html"


def _run_node(helper_source: str, harness_source: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1e2-") as temporary_directory:
        directory = Path(temporary_directory)
        (directory / "preview.mjs").write_text(helper_source, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness_source, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            check=True,
            capture_output=True,
            text=True,
        )
    return json.loads(result.stdout)


class _IdCollector(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: list[str] = []

    def handle_starttag(self, _tag: str, attrs: list[tuple[str, str | None]]) -> None:
        for name, value in attrs:
            if name == "id" and value is not None:
                self.ids.append(value)


class ObjectGraspPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.helper = HELPER_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_pure_transform_module_identity_yaw_and_combined_rpy(self) -> None:
        harness = r'''
import {
  matrix4FromTranslationRpy,
} from "./preview.mjs";

function near(actual, expected, tolerance = 1e-12) {
  return actual.every((row, i) => row.every(
    (value, j) => Math.abs(value - expected[i][j]) <= tolerance,
  ));
}

const identity = matrix4FromTranslationRpy({
  translationM: [0, 0, 0], rpyRad: [0, 0, 0],
});
const yaw90 = matrix4FromTranslationRpy({
  translationM: [0, 0, 0], rpyRad: [0, 0, Math.PI / 2],
});
const combined = matrix4FromTranslationRpy({
  translationM: [0.4, -0.2, 1.1], rpyRad: [0.3, -0.4, 0.7],
});

console.log(JSON.stringify({
  identity: near(identity, [
    [1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1],
  ]),
  yaw90: near(yaw90, [
    [0,-1,0,0], [1,0,0,0], [0,0,1,0], [0,0,0,1],
  ]),
  combined: near(combined, [
    [0.7044663052755917,-0.7034634588974241,-0.09416149280586425,0.4],
    [0.5933637833613874,0.6565444413589879,-0.465691761915191,-0.2],
    [0.3894183423086505,0.2721921352954314,0.879923176281257,1.1],
    [0,0,0,1],
  ]),
}));
'''
        result = _run_node(self.helper, harness)
        self.assertEqual(result, {"identity": True, "yaw90": True, "combined": True})

    def test_fixed_grasp_offsets_move_rigidly_without_mutation(self) -> None:
        harness = r'''
import {
  INITIAL_SYNTHETIC_OBJECT_POSE,
  SYNTHETIC_OBJECT_DIMENSIONS_M,
  SYNTHETIC_OBJECT_T_LEFT_POSE,
  SYNTHETIC_OBJECT_T_RIGHT_POSE,
  computeWorldGraspFrameMatrices,
} from "./preview.mjs";

const beforeLeft = JSON.stringify(SYNTHETIC_OBJECT_T_LEFT_POSE);
const beforeRight = JSON.stringify(SYNTHETIC_OBJECT_T_RIGHT_POSE);
const identity = computeWorldGraspFrameMatrices({
  translationM: [0,0,0], rpyRad: [0,0,0],
});
const moved = computeWorldGraspFrameMatrices({
  translationM: [1,2,3], rpyRad: [0,0,Math.PI / 2],
});
const translation = (matrix) => [matrix[0][3], matrix[1][3], matrix[2][3]];

console.log(JSON.stringify({
  dimensions: SYNTHETIC_OBJECT_DIMENSIONS_M,
  initialPose: INITIAL_SYNTHETIC_OBJECT_POSE,
  objectTLeft: SYNTHETIC_OBJECT_T_LEFT_POSE,
  objectTRight: SYNTHETIC_OBJECT_T_RIGHT_POSE,
  identityLeft: translation(identity.worldTLeft),
  identityRight: translation(identity.worldTRight),
  movedObject: translation(moved.worldTObject),
  movedLeft: translation(moved.worldTLeft),
  movedRight: translation(moved.worldTRight),
  fixturesUnchanged: beforeLeft === JSON.stringify(SYNTHETIC_OBJECT_T_LEFT_POSE)
    && beforeRight === JSON.stringify(SYNTHETIC_OBJECT_T_RIGHT_POSE),
  fixturesFrozen: Object.isFrozen(SYNTHETIC_OBJECT_T_LEFT_POSE)
    && Object.isFrozen(SYNTHETIC_OBJECT_T_LEFT_POSE.translationM)
    && Object.isFrozen(SYNTHETIC_OBJECT_T_RIGHT_POSE)
    && Object.isFrozen(SYNTHETIC_OBJECT_T_RIGHT_POSE.translationM),
}));
'''
        result = _run_node(self.helper, harness)
        self.assertEqual(
            result["dimensions"],
            {"length": 0.5, "width": 0.15, "height": 0.1},
        )
        self.assertEqual(result["initialPose"]["translationM"], [-1.32, 0, 1.2])
        self.assertEqual(result["objectTLeft"]["translationM"], [0, 0.25, 0])
        self.assertEqual(result["objectTRight"]["translationM"], [0, -0.25, 0])
        self.assertEqual(result["identityLeft"], [0, 0.25, 0])
        self.assertEqual(result["identityRight"], [0, -0.25, 0])
        self.assertEqual(result["movedObject"], [1, 2, 3])
        for actual, expected in zip(result["movedLeft"], [0.75, 2, 3]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(result["movedRight"], [1.25, 2, 3]):
            self.assertAlmostEqual(actual, expected)
        self.assertTrue(result["fixturesUnchanged"])
        self.assertTrue(result["fixturesFrozen"])

    def test_helper_is_frontend_only_and_explicit_about_transform_convention(self) -> None:
        self.assertIn("^A T_B", self.helper)
        self.assertIn("Rz(yaw) * Ry(pitch) * Rx(roll)", self.helper)
        self.assertIn("^W T_L = ^W T_O * ^O T_L", self.helper)
        self.assertIn("^W T_R = ^W T_O * ^O T_R", self.helper)
        for forbidden in (
            "fetch(",
            "/api/",
            "MoveIt",
            "setJointValue",
            "trajectory",
            "robot",
            "window",
            "document",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.helper)

    def test_scene_omits_synthetic_mesh_and_keeps_three_distinct_frames(self) -> None:
        create_preview = self.controller.split(
            "function createObjectGraspPreview()", 1
        )[1].split("function applyObjectPreviewPose", 1)[0]
        self.assertNotIn("new THREE.BoxGeometry(", create_preview)
        self.assertNotIn("Synthetic Workpiece", create_preview)
        self.assertIn('objectFrame.name = "Center / Coordination Frame C"', create_preview)
        self.assertIn('"Left Grasp Frame L"', create_preview)
        self.assertIn('"Right Grasp Frame R"', create_preview)
        self.assertIn('axes.name = "World Frame W axes"', self.controller)
        self.assertIn("new THREE.AxesHelper(0.24)", create_preview)
        self.assertIn("new THREE.AxesHelper(0.18)", self.controller)
        self.assertEqual(self.controller.count("scene.add(objectFrame)"), 1)
        self.assertEqual(self.controller.count("scene.add(leftGraspFrame)"), 1)
        self.assertEqual(self.controller.count("scene.add(rightGraspFrame)"), 1)

    def test_apply_and_reset_are_isolated_from_robot_joint_state(self) -> None:
        apply_block = self.controller.split(
            "function applyObjectPreviewPose(pose)", 1
        )[1].split("function objectPoseFromControls", 1)[0]
        reset_block = self.controller.split(
            "function resetGraspFramesToCurrentRobotPose", 1
        )[1].split(
            "function initializeUnlockedGraspFramesFromCurrentRobotPoseWhenReady", 1
        )[0]
        self.assertIn(
            "computeWorldGraspFrameMatrices(normalizedPose, graspContent)",
            apply_block,
        )
        self.assertIn("currentRigidGraspContent()", apply_block)
        self.assertIn("applyMatrixToFrame(objectFrame, worldTObject)", apply_block)
        self.assertIn("transforms.worldTLeft", apply_block)
        self.assertIn("transforms.worldTRight", apply_block)
        self.assertIn("deriveGraspFrameResetFromWorldTips", reset_block)
        self.assertIn("currentDisplayedModelTipWorldMatrices", reset_block)
        self.assertIn("objectVisible: false", reset_block)
        self.assertIn("objectFrameVisible: true", reset_block)
        self.assertIn("graspFramesVisible: true", reset_block)
        for source in (apply_block, reset_block):
            for forbidden in (
                "setJointValues(",
                "applyJointValuesToModel(",
                "planned",
                "requestMoveIt",
                "fetch(",
                "/api/",
                "IK",
            ):
                with self.subTest(forbidden=forbidden):
                    self.assertNotIn(forbidden, source)

    def test_object_pose_controls_units_actions_visibility_and_warnings(self) -> None:
        for label in (
            "Grasp Setup",
            "Center X [m]",
            "Center Y [m]",
            "Center Z [m]",
            "Rotation Unit",
            "Degrees (°)",
            "Radians (rad)",
            "Roll [deg]",
            "Pitch [deg]",
            "Yaw [deg]",
            "Apply Center Pose",
            "Reset Frames to Current Robot Pose",
            "Show Center Frame",
            "Show Grasp Frames",
            "Offline Center/grasp preview",
            "Physical world calibration not completed",
            "no physical execution",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html)

        for control_id in (
            "digitalTwinObjectX",
            "digitalTwinObjectY",
            "digitalTwinObjectZ",
            "digitalTwinObjectRoll",
            "digitalTwinObjectPitch",
            "digitalTwinObjectYaw",
            "digitalTwinApplyObjectPose",
            "digitalTwinResetObjectPreview",
            "digitalTwinShowObjectFrame",
            "digitalTwinShowGraspFrames",
        ):
            with self.subTest(control=control_id):
                self.assertIn(f'id="{control_id}"', self.html)

        parser = _IdCollector()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))

    def test_existing_zoom_and_sampled_path_contracts_remain_present(self) -> None:
        self.assertIn("const NORMALIZED_WHEEL_ZOOM_RATE = 0.035", self.controller)
        self.assertEqual(self.controller.count('addEventListener("wheel"'), 1)
        self.assertIn("function applySampledPathValidation", self.controller)
        self.assertIn("DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD", self.controller)


if __name__ == "__main__":
    unittest.main()
