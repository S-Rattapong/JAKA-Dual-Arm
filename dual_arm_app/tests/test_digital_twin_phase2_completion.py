"""Offline contract tests for Phase 2 object/grasp visualization completion."""

from __future__ import annotations

import json
import re
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = REPOSITORY_ROOT / "dual_arm_app" / "web"
HELPER_PATH = WEB_ROOT / "digital_twin_object_grasp_preview.js"
ANGLE_HELPER_PATH = WEB_ROOT / "digital_twin_angle_units.js"
CONTROLLER_PATH = WEB_ROOT / "digital_twin.js"
HTML_PATH = WEB_ROOT / "index.html"


def _run_node(helper_source: str, harness_source: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase2-completion-") as temporary_directory:
        directory = Path(temporary_directory)
        (directory / "preview.mjs").write_text(helper_source, encoding="utf-8")
        (directory / "angle_units.mjs").write_text(
            ANGLE_HELPER_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "harness.mjs").write_text(harness_source, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise AssertionError(result.stderr)
    return json.loads(result.stdout)


class _HtmlContractParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: list[str] = []
        self.inputs: dict[str, dict[str, str | None]] = {}

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attributes = dict(attrs)
        element_id = attributes.get("id")
        if element_id is not None:
            self.ids.append(element_id)
            if tag == "input":
                self.inputs[element_id] = attributes


class DigitalTwinPhase2CompletionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.helper = HELPER_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_relative_state_is_derived_has_expected_fixture_and_returns_copies(self) -> None:
        harness = r'''
import { computeObjectGraspRelativeState } from "./preview.mjs";

const first = computeObjectGraspRelativeState();
const originalLeftX = first.objectTLeft.translationM[0];
first.objectTLeft.translationM[0] = 99;
first.leftTRight.rpyRad[2] = 99;
const second = computeObjectGraspRelativeState();

console.log(JSON.stringify({
  objectTLeft: second.objectTLeft,
  objectTRight: second.objectTRight,
  leftTRight: second.leftTRight,
  translationUnit: second.translationUnit,
  orientationUnit: second.orientationUnit,
  frameNotation: second.frameNotation,
  semanticSource: second.semanticSource,
  copiesAreIsolated: originalLeftX === second.objectTLeft.translationM[0]
    && second.leftTRight.rpyRad[2] !== 99,
}));
'''
        result = _run_node(self.helper, harness)
        self.assertEqual(result["objectTLeft"]["translationM"], [0, 0.25, 0])
        self.assertEqual(result["objectTRight"]["translationM"], [0, -0.25, 0])
        for actual, expected in zip(
            result["leftTRight"]["translationM"], [0, -0.5, 0]
        ):
            self.assertAlmostEqual(actual, expected, places=12)
        for angle in result["leftTRight"]["rpyRad"]:
            self.assertAlmostEqual(angle, 0, places=12)
        self.assertEqual(result["translationUnit"], "meter")
        self.assertEqual(result["orientationUnit"], "radian")
        self.assertIn("^O T_L", result["frameNotation"]["objectTLeft"])
        self.assertIn("^O T_R", result["frameNotation"]["objectTRight"])
        self.assertIn("inverse(^O T_L)", result["frameNotation"]["leftTRight"])
        self.assertTrue(result["semanticSource"])
        self.assertTrue(result["copiesAreIsolated"])

        relative_block = self.helper.split(
            "export function computeObjectGraspRelativeState(", 1
        )[1].split("export function computeWorldGraspFrameMatricesFromWorldTObject", 1)[0]
        self.assertIn("configuration = SYNTHETIC_DEFAULT_GRASP_DRAFT", relative_block)
        self.assertIn("normalizedGraspMatrices(", relative_block)
        self.assertIn("inverseRigidMatrix4(objectTLeft)", relative_block)
        self.assertIn("objectTRight", relative_block)
        self.assertIn("multiplyMatrix4(", relative_block)
        self.assertNotIn("-0.5", relative_block)

    def test_left_to_right_pose_is_invariant_under_arbitrary_world_object_pose(self) -> None:
        harness = r'''
import {
  computeObjectGraspRelativeState,
  computeWorldGraspFrameMatrices,
  inverseRigidMatrix4,
  multiplyMatrix4,
  poseFromRigidMatrix4,
} from "./preview.mjs";

const expected = computeObjectGraspRelativeState().leftTRight;
const world = computeWorldGraspFrameMatrices({
  translationM: [1.4, -0.8, 0.65],
  rpyRad: [0.31, -0.42, 0.73],
});
const measured = poseFromRigidMatrix4(multiplyMatrix4(
  inverseRigidMatrix4(world.worldTLeft),
  world.worldTRight,
));
const near = (left, right) => left.every(
  (value, index) => Math.abs(value - right[index]) < 1e-11,
);
console.log(JSON.stringify({
  sameTranslation: near(measured.translationM, expected.translationM),
  sameOrientation: near(measured.rpyRad, expected.rpyRad),
}));
'''
        self.assertEqual(
            _run_node(self.helper, harness),
            {"sameTranslation": True, "sameOrientation": True},
        )

    def test_relative_numeric_displays_exist_and_start_without_hardcoded_values(self) -> None:
        for label in (
            "Object-relative Left Grasp (^O T_L)",
            "Object-relative Right Grasp (^O T_R)",
            "Left Grasp → Right Grasp Relative Pose (^L T_R)",
            "Frame notation: ^L T_R = inverse(^O T_L) * ^O T_R",
        ):
            self.assertIn(label, self.html)

        for prefix in (
            "digitalTwinObjectTLeft",
            "digitalTwinObjectTRight",
            "digitalTwinLeftTRight",
        ):
            self.assertIn(f'"{prefix}"', self.controller)
            for suffix in ("X", "Y", "Z", "Rx", "Ry", "Rz"):
                element_id = f"{prefix}{suffix}"
                self.assertRegex(
                    self.html,
                    rf'id="{element_id}">UNAVAILABLE</strong>',
                )
        self.assertIn('["X", pose.translationM[0]]', self.controller)
        self.assertIn('["Rz", pose.rpyRad[2]]', self.controller)

    def test_translation_controls_are_editable_and_orientation_is_locked(self) -> None:
        parser = _HtmlContractParser()
        parser.feed(self.html)
        for element_id in (
            "digitalTwinObjectX",
            "digitalTwinObjectY",
            "digitalTwinObjectZ",
        ):
            self.assertNotIn("readonly", parser.inputs[element_id])
            self.assertNotIn("disabled", parser.inputs[element_id])
        for element_id in (
            "digitalTwinObjectRoll",
            "digitalTwinObjectPitch",
            "digitalTwinObjectYaw",
        ):
            self.assertIn(element_id, parser.inputs)
        self.assertIn("field.disabled = !locked", self.controller)
        self.assertIn("Center orientation is editable after Lock Grasp", self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))

    def test_manual_six_d_pose_updates_orientation_and_all_frames(self) -> None:
        object_block = "function copyObjectPose" + self.controller.split(
            "function copyObjectPose", 1
        )[1].split("function setObjectPreviewVisibility", 1)[0]
        harness = r'''
import {
  INITIAL_SYNTHETIC_OBJECT_POSE,
  SYNTHETIC_OBJECT_DIMENSIONS_M,
  computeObjectGraspRelativeState,
  computeWorldGraspFrameMatrices,
  matrix4FromTranslationRpy,
  normalizeObjectPreviewPose,
} from "./preview.mjs";
import {
  angleUnitMetadata,
  formatRotationRadians,
  normalizeAngleUnit,
  rotationDisplayToRadians,
  rotationRadiansToDisplay,
} from "./angle_units.mjs";

const angleUnitState = { unit: "degrees" };

class Matrix4 {
  set(...elements) { this.elements = elements; return this; }
}
const THREE = { Matrix4 };
const frame = () => ({
  matrixAutoUpdate: true,
  matrix: { copy(value) { this.elements = [...value.elements]; } },
  matrixWorldNeedsUpdate: false,
  updateCount: 0,
  updateMatrixWorld() { this.updateCount += 1; },
});
let objectFrame = frame();
let leftGraspFrame = frame();
let rightGraspFrame = frame();
let objectFrameAxes = null;
let syntheticWorkpiece = null;
const objectPreviewState = {
  pose: null,
  objectVisible: true,
  objectFrameVisible: true,
  graspFramesVisible: true,
  error: null,
};
const rigidGraspState = {
  state: "GRASP_UNLOCKED",
  draft: {
    left: { translationM: [0, 0.25, 0], rpyRad: [0, 0, 0] },
    right: { translationM: [0, -0.25, 0], rpyRad: [0, 0, 0] },
  },
  lockedSnapshot: null,
  selectedFrame: "CENTER",
};
const values = {
  digitalTwinObjectX: -1.32,
  digitalTwinObjectY: 0,
  digitalTwinObjectZ: 1.2,
  digitalTwinObjectRoll: 0,
  digitalTwinObjectPitch: 0,
  digitalTwinObjectYaw: 0,
};
const elements = Object.fromEntries(Object.entries(values).map(([id, value]) => [
  id,
  {
    value: String(value),
    get valueAsNumber() { return Number(this.value); },
  },
]));
elements.digitalTwinObjectPreviewError = { textContent: "NONE" };
const document = { getElementById: (id) => elements[id] || null };
let renderCount = 0;
let pauseCount = 0;
let jointState = [0.11, 0.22, 0.33];
function render() { renderCount += 1; }
function pauseObjectTrajectoryForManualPreview() { pauseCount += 1; }
function setObjectPreviewVisibility() {}
function invalidateAllPhase4Validation() {}
'''
        harness += object_block
        harness += r'''

applyObjectPreviewPose({
  translationM: [0.2, -0.3, 0.9],
  rpyRad: [0.1, -0.2, 0.3],
});
elements.digitalTwinObjectX.value = "1.1";
elements.digitalTwinObjectY.value = "-1.2";
elements.digitalTwinObjectZ.value = "1.3";
// Center orientation is an editable part of the six-dimensional W pose.
elements.digitalTwinObjectRoll.value = "5";
elements.digitalTwinObjectPitch.value = "4";
elements.digitalTwinObjectYaw.value = "3";
const result = applyObjectPoseFromControls();
console.log(JSON.stringify({
  pose: result.pose,
  orientationControls: [
    Number(elements.digitalTwinObjectRoll.value),
    Number(elements.digitalTwinObjectPitch.value),
    Number(elements.digitalTwinObjectYaw.value),
  ],
  objectUpdated: objectFrame.updateCount,
  leftUpdated: leftGraspFrame.updateCount,
  rightUpdated: rightGraspFrame.updateCount,
  pauseCount,
  renderCount,
  jointState,
}));
'''
        result = _run_node(self.helper, harness)
        self.assertEqual(result["pose"]["translationM"], [1.1, -1.2, 1.3])
        expected_rpy = [value * 3.141592653589793 / 180.0 for value in (5, 4, 3)]
        for actual, expected in zip(result["pose"]["rpyRad"], expected_rpy):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(result["orientationControls"], [5, 4, 3]):
            self.assertAlmostEqual(actual, expected, places=8)
        self.assertEqual(result["objectUpdated"], 2)
        self.assertEqual(result["leftUpdated"], 2)
        self.assertEqual(result["rightUpdated"], 2)
        self.assertEqual(result["pauseCount"], 1)
        self.assertEqual(result["renderCount"], 2)
        self.assertEqual(result["jointState"], [0.11, 0.22, 0.33])

    def test_scene_public_api_and_existing_phase_contracts_remain_intact(self) -> None:
        create_block = self.controller.split(
            "function createObjectGraspPreview()", 1
        )[1].split("function applyObjectPreviewPose", 1)[0]
        self.assertNotIn("new THREE.BoxGeometry(", create_block)
        self.assertIn('objectFrame.name = "Center / Coordination Frame C"', create_block)
        self.assertEqual(self.controller.count("scene.add(objectFrame)"), 1)
        self.assertEqual(self.controller.count("scene.add(leftGraspFrame)"), 1)
        self.assertEqual(self.controller.count("scene.add(rightGraspFrame)"), 1)
        self.assertIn("getObjectGraspRelativeState,", self.controller)
        self.assertIn("applyObjectPreviewPose(sample.objectPose)", self.controller)
        for phase1_contract in (
            "getModelTcpState,",
            "getPlannedPreviewState,",
            "getTrajectoryValidationState,",
            'id="digitalTwinViewer"',
        ):
            self.assertIn(
                phase1_contract,
                self.controller if phase1_contract.endswith(",") else self.html,
            )

        manual_block = self.controller.split(
            "function objectPoseFromControls()", 1
        )[1].split("function writeObjectPoseControls", 1)[0]
        for orientation_control in (
            "digitalTwinObjectRoll",
            "digitalTwinObjectPitch",
            "digitalTwinObjectYaw",
        ):
            with self.subTest(orientation_control=orientation_control):
                self.assertIn(orientation_control, manual_block)
        for forbidden in (
            "setJointValues(",
            "fetch(",
            "/api/",
            "requestMoveIt",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, manual_block)
        self.assertNotRegex(self.html, re.compile(r">\s*Execute\s*<", re.IGNORECASE))


if __name__ == "__main__":
    unittest.main()
