"""Offline tests for Phase 1F.2 object Cartesian trajectory preview."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = REPOSITORY_ROOT / "dual_arm_app" / "web"
GRASP_HELPER_PATH = WEB_ROOT / "digital_twin_object_grasp_preview.js"
TRAJECTORY_HELPER_PATH = WEB_ROOT / "digital_twin_object_trajectory_preview.js"
CONTROLLER_PATH = WEB_ROOT / "digital_twin.js"
HTML_PATH = WEB_ROOT / "index.html"


def _run_node(grasp_source: str, trajectory_source: str, harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1f2-") as temporary_directory:
        directory = Path(temporary_directory)
        (directory / "grasp.mjs").write_text(grasp_source, encoding="utf-8")
        rewritten_trajectory = trajectory_source.replace(
            '"./digital_twin_object_grasp_preview.js"',
            '"./grasp.mjs"',
        )
        (directory / "trajectory.mjs").write_text(
            rewritten_trajectory,
            encoding="utf-8",
        )
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
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


class ObjectTrajectoryPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.grasp_helper = GRASP_HELPER_PATH.read_text(encoding="utf-8")
        cls.trajectory_helper = TRAJECTORY_HELPER_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def run_helper(self, harness: str) -> dict:
        return _run_node(self.grasp_helper, self.trajectory_helper, harness)

    def test_synthetic_samples_match_phase_1f1_values_and_constant_orientation(self) -> None:
        harness = r'''
import {
  SYNTHETIC_OBJECT_TRAJECTORY_DEFINITION,
  SYNTHETIC_OBJECT_TRAJECTORY_NOTICE,
  SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
} from "./trajectory.mjs";

const trajectory = SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY;
const rotation = (matrix) => matrix.slice(0, 3).map((row) => row.slice(0, 3));
const startRotation = JSON.stringify(rotation(trajectory.samples[0].worldTObject));
console.log(JSON.stringify({
  notice: SYNTHETIC_OBJECT_TRAJECTORY_NOTICE,
  definition: SYNTHETIC_OBJECT_TRAJECTORY_DEFINITION,
  sampleCount: trajectory.sampleCount,
  times: trajectory.samples.map((sample) => sample.timeFromStartS),
  alphas: trajectory.samples.map((sample) => sample.alpha),
  translations: trajectory.samples.map((sample) => sample.objectPose.translationM),
  rpyValues: trajectory.samples.map((sample) => sample.objectPose.rpyRad),
  constantRotation: trajectory.samples.every(
    (sample) => JSON.stringify(rotation(sample.worldTObject)) === startRotation,
  ),
  frozen: Object.isFrozen(trajectory)
    && Object.isFrozen(trajectory.samples)
    && trajectory.samples.every(Object.isFrozen),
}));
'''
        result = self.run_helper(harness)
        self.assertEqual(
            result["notice"],
            "OFFLINE SYNTHETIC OBJECT TRAJECTORY — NOT FOR ROBOT EXECUTION",
        )
        self.assertEqual(result["definition"]["startTranslationM"], [0, 0, 0.8])
        self.assertEqual(result["definition"]["startRpyRad"], [0.1, -0.2, 0.3])
        self.assertEqual(result["definition"]["endTranslationM"], [0.4, 0, 1])
        self.assertEqual(result["definition"]["durationS"], 4)
        self.assertEqual(result["sampleCount"], 5)
        self.assertEqual(result["times"], [0, 1, 2, 3, 4])
        self.assertEqual(result["alphas"], [0, 0.25, 0.5, 0.75, 1])
        self.assertEqual(result["translations"][0], [0, 0, 0.8])
        self.assertEqual(result["translations"][2], [0.2, 0, 0.9])
        self.assertEqual(result["translations"][-1], [0.4, 0, 1])
        self.assertTrue(all(rpy == [0.1, -0.2, 0.3] for rpy in result["rpyValues"]))
        self.assertTrue(result["constantRotation"])
        self.assertTrue(result["frozen"])

    def test_grasp_targets_are_derived_and_relative_transform_is_invariant(self) -> None:
        harness = r'''
import {
  computeWorldGraspFrameMatrices,
  multiplyMatrix4,
} from "./grasp.mjs";
import { SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY as trajectory } from "./trajectory.mjs";

function close(left, right, tolerance = 1e-12) {
  return left.every((row, i) => row.every(
    (value, j) => Math.abs(value - right[i][j]) <= tolerance,
  ));
}
function rigidInverse(matrix) {
  const result = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]];
  for (let row = 0; row < 3; row += 1) {
    for (let column = 0; column < 3; column += 1) {
      result[row][column] = matrix[column][row];
    }
    result[row][3] = -result[row].slice(0, 3).reduce(
      (total, value, index) => total + value * matrix[index][3],
      0,
    );
  }
  return result;
}
const relativeTransforms = trajectory.samples.map((sample) => (
  multiplyMatrix4(rigidInverse(sample.worldTLeft), sample.worldTRight)
));
const derived = trajectory.samples.every((sample) => {
  const expected = computeWorldGraspFrameMatrices(sample.objectPose);
  return close(sample.worldTLeft, expected.worldTLeft)
    && close(sample.worldTRight, expected.worldTRight);
});
console.log(JSON.stringify({
  derived,
  invariant: relativeTransforms.every(
    (relative) => close(relative, relativeTransforms[0]),
  ),
  relativeTranslation: relativeTransforms[0].slice(0, 3).map((row) => row[3]),
}));
'''
        result = self.run_helper(harness)
        self.assertTrue(result["derived"])
        self.assertTrue(result["invariant"])
        for actual, expected in zip(result["relativeTranslation"], [0, -0.5, 0]):
            self.assertAlmostEqual(actual, expected)

    def test_scrubber_uses_clamped_nearest_sample_with_earlier_tie_break(self) -> None:
        harness = r'''
import {
  SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY as trajectory,
  nearestObjectTrajectorySample,
  objectTrajectorySampleAtIndex,
} from "./trajectory.mjs";
const times = [-5, 0.49, 0.5, 0.51, 2.5, 3.6, 99];
console.log(JSON.stringify({
  selected: times.map(
    (time) => nearestObjectTrajectorySample(trajectory, time).index,
  ),
  direct: objectTrajectorySampleAtIndex(trajectory, 3).timeFromStartS,
}));
'''
        result = self.run_helper(harness)
        self.assertEqual(result["selected"], [0, 0, 0, 1, 2, 4, 4])
        self.assertEqual(result["direct"], 3)

    def test_controller_preserves_requested_time_while_applying_discrete_sample(self) -> None:
        state_source = "const objectTrajectoryPreviewState = {" + self.controller.split(
            "const objectTrajectoryPreviewState = {", 1
        )[1].split("const trajectoryValidationState", 1)[0]
        controller_source = "function objectTrajectoryStateSnapshot" + self.controller.split(
            "function objectTrajectoryStateSnapshot", 1
        )[1].split("const publicApi", 1)[0]
        harness = r'''
import {
  SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
  nearestObjectTrajectorySample,
  objectTrajectorySampleAtIndex,
} from "./trajectory.mjs";
globalThis.document = { getElementById() { return null; } };
globalThis.cancelAnimationFrame = () => {};
globalThis.requestAnimationFrame = () => 1;
const appliedPoses = [];
function applyObjectPreviewPose(pose) {
  appliedPoses.push(JSON.parse(JSON.stringify(pose)));
}
''' + state_source + controller_source + r'''
loadSyntheticObjectTrajectory();
function capture(requestedTimeS) {
  const state = setObjectTrajectoryTime(requestedTimeS);
  return {
    requestedTimeS,
    currentTimeS: state.currentTimeS,
    currentSampleIndex: state.currentSampleIndex,
    alpha: state.alpha,
    appliedTranslationM: appliedPoses[appliedPoses.length - 1].translationM,
  };
}
console.log(JSON.stringify({
  samples: [capture(0.49), capture(0.51), capture(0.50), capture(2.50), capture(3.60)],
}));
'''
        result = self.run_helper(harness)
        expected = (
            (0.49, 0, 0.0, [0, 0, 0.8]),
            (0.51, 1, 0.25, [0.1, 0, 0.85]),
            (0.50, 0, 0.0, [0, 0, 0.8]),
            (2.50, 2, 0.5, [0.2, 0, 0.9]),
            (3.60, 4, 1.0, [0.4, 0, 1.0]),
        )
        for actual, (time_s, index, alpha, translation) in zip(
            result["samples"], expected
        ):
            with self.subTest(requested_time=time_s):
                self.assertEqual(actual["currentTimeS"], time_s)
                self.assertEqual(actual["currentSampleIndex"], index)
                self.assertEqual(actual["alpha"], alpha)
                for value, wanted in zip(actual["appliedTranslationM"], translation):
                    self.assertAlmostEqual(value, wanted)

    def test_helper_is_pure_and_documents_equations_units_and_limitations(self) -> None:
        self.assertTrue(TRAJECTORY_HELPER_PATH.is_file())
        for forbidden in ("window", "document", "fetch(", "/api/", "MoveIt"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.trajectory_helper)
        for documentation in (
            "alpha_i = i/(N-1)",
            "t_i = alpha_i T",
            "p_i = (1-alpha_i)p0 + alpha_i p1",
            "^W T_L(i) = ^W T_O(i) ^O T_L",
            "^W T_R(i) = ^W T_O(i) ^O T_R",
            "world translation in meters",
            "orientation is constant",
            "Cartesian visualization",
            "no IK",
            "joint-trajectory",
            "execution guarantee",
        ):
            with self.subTest(documentation=documentation):
                self.assertIn(documentation, self.trajectory_helper)

    def test_controller_uses_separate_object_trajectory_state(self) -> None:
        state_block = self.controller.split(
            "const objectTrajectoryPreviewState = {", 1
        )[1].split("};", 1)[0]
        for field in (
            "trajectory",
            "currentTimeS",
            "currentSampleIndex",
            "playing",
            "playbackRate",
            "error",
            "previousFrameTimeMs",
            "animationFrameId",
        ):
            self.assertIn(field, state_block)
        self.assertIn("const trajectoryPreviewState = {", self.controller)
        self.assertIn("const plannedPreviewState = {", self.controller)
        self.assertIn("const mirrorState = {", self.controller)

    def test_sample_application_is_isolated_from_joint_and_transport_systems(self) -> None:
        apply_block = self.controller.split(
            "function applyObjectTrajectorySample", 1
        )[1].split("function loadSyntheticObjectTrajectory", 1)[0]
        self.assertIn("applyObjectPreviewPose(sample.objectPose)", apply_block)
        for forbidden in (
            "setJointValues(",
            "applyJointValuesToModel(",
            "robot",
            "plannedRobot",
            "trajectoryPreviewState",
            "plannedPreviewState",
            "mirrorState",
            "requestMoveIt",
            "fetch(",
            "/api/",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, apply_block)

    def test_playback_stop_clear_and_manual_ownership_semantics_are_explicit(self) -> None:
        stop_block = self.controller.split(
            "function stopObjectTrajectory()", 1
        )[1].split("function clearObjectTrajectory", 1)[0]
        clear_block = self.controller.split(
            "function clearObjectTrajectory()", 1
        )[1].split("function setObjectTrajectoryPlaybackRate", 1)[0]
        manual_apply = self.controller.split(
            "function applyObjectPoseFromControls()", 1
        )[1].split("function writeObjectPoseControls", 1)[0]
        manual_reset = self.controller.split(
            "function resetObjectPreview()", 1
        )[1].split("function setObjectPreviewVisibility", 1)[0]
        ownership = self.controller.split(
            "function pauseObjectTrajectoryForManualPreview()", 1
        )[1].split("function stopObjectTrajectory", 1)[0]
        self.assertIn('applyObjectTrajectorySample(0, "READY")', stop_block)
        self.assertNotIn("applyObjectPreviewPose", clear_block)
        self.assertIn("Preserve the last visible Object/Grasp pose", clear_block)
        self.assertIn("pauseObjectTrajectoryForManualPreview()", manual_apply)
        self.assertIn("pauseObjectTrajectoryForManualPreview()", manual_reset)
        self.assertIn('status = "MANUAL_OVERRIDE"', ownership)
        self.assertIn("requestAnimationFrame", self.controller)
        self.assertIn('"FINISHED"', self.controller)

    def test_ui_panel_controls_units_warnings_and_unique_ids_exist(self) -> None:
        for label in (
            "Object Trajectory Preview — OFFLINE ONLY",
            "Object Trajectory State",
            "Trajectory Name",
            "Sample Count",
            "Current Sample",
            "Current Time",
            "Duration",
            "Alpha",
            "Playback Rate",
            "Translation Unit",
            "meters",
            "Orientation Unit",
            "radians",
            "Load Synthetic Object Trajectory",
            "Previous Sample",
            "Next Sample",
            "Play",
            "Pause",
            "Stop",
            "Clear Object Trajectory",
            "snaps to nearest sample",
            "OFFLINE SYNTHETIC OBJECT TRAJECTORY",
            "TRANSLATION-ONLY OBJECT MOTION",
            "OBJECT ORIENTATION HELD CONSTANT",
            "NO IK APPLIED",
            "NOT A ROBOT JOINT TRAJECTORY",
            "NO PHYSICAL EXECUTION",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html)

        for control_id in (
            "digitalTwinLoadSyntheticObjectTrajectory",
            "digitalTwinObjectTrajectoryPrevious",
            "digitalTwinObjectTrajectoryNext",
            "digitalTwinObjectTrajectoryPlay",
            "digitalTwinObjectTrajectoryPause",
            "digitalTwinObjectTrajectoryStop",
            "digitalTwinObjectTrajectoryClear",
            "digitalTwinObjectTrajectoryPlaybackRate",
            "digitalTwinObjectTrajectoryScrubber",
        ):
            with self.subTest(control=control_id):
                self.assertIn(f'id="{control_id}"', self.html)

        parser = _IdCollector()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))

        panel = self.html.split(
            'class="digital-twin-object-trajectory-panel"', 1
        )[1].split('class="digital-twin-planned-panel"', 1)[0]
        self.assertIn('id="digitalTwinObjectTrajectoryScrubber"', panel)
        self.assertIn('step="0.01"', panel)
        self.assertIn("preserve the requested continuous time", panel)
        self.assertIn("No Cartesian", panel)
        update_ui = self.controller.split(
            "function updateObjectTrajectoryPreviewUi()", 1
        )[1].split("function cancelObjectTrajectoryAnimation", 1)[0]
        self.assertIn("state.currentTimeS.toFixed(2)", update_ui)
        set_time = self.controller.split(
            "function setObjectTrajectoryTime(timeSeconds)", 1
        )[1].split("function scheduleObjectTrajectoryFrame", 1)[0]
        self.assertIn("clampedRequestedTimeS", set_time)
        self.assertIn(
            'applyObjectTrajectorySample(\n      sample.index,\n      "PAUSED",\n      clampedRequestedTimeS,',
            set_time,
        )

    def test_existing_object_preview_zoom_and_sampled_path_contracts_remain(self) -> None:
        self.assertIn("Object / Grasp Frame Preview", self.html)
        self.assertIn("function applyObjectPreviewPose", self.controller)
        self.assertIn("const NORMALIZED_WHEEL_ZOOM_RATE = 0.035", self.controller)
        self.assertEqual(self.controller.count('addEventListener("wheel"'), 1)
        self.assertIn("function applySampledPathValidation", self.controller)
        self.assertIn("DEFAULT_SAMPLED_PATH_MAX_JOINT_STEP_RAD", self.controller)


if __name__ == "__main__":
    unittest.main()
