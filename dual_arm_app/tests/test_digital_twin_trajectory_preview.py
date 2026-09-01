"""Offline tests for Phase 1D.2 Planned Joint Trajectory Preview."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = REPOSITORY_ROOT / "dual_arm_app" / "web"
TRAJECTORY_PATH = WEB_ROOT / "digital_twin_trajectory_preview.js"
PLANNED_PATH = WEB_ROOT / "digital_twin_planned_preview.js"
ADAPTER_PATH = WEB_ROOT / "digital_twin_status_adapter.js"
SMOOTHING_PATH = WEB_ROOT / "digital_twin_live_smoothing.js"
DIGITAL_TWIN_PATH = WEB_ROOT / "digital_twin.js"
HTML_PATH = WEB_ROOT / "index.html"


def _run_node(files: dict[str, str], harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1d2-") as temporary_directory:
        directory = Path(temporary_directory)
        for name, source in files.items():
            (directory / name).write_text(source, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            check=True,
            capture_output=True,
            text=True,
        )
    return json.loads(result.stdout)


class TrajectoryPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.trajectory = TRAJECTORY_PATH.read_text(encoding="utf-8")
        cls.planned = PLANNED_PATH.read_text(encoding="utf-8")
        cls.adapter = ADAPTER_PATH.read_text(encoding="utf-8")
        cls.digital_twin = DIGITAL_TWIN_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_pure_module_validation_copy_metadata_and_contract(self) -> None:
        self.assertTrue(TRAJECTORY_PATH.is_file())
        for forbidden in (
            "fetch",
            "window",
            "document",
            "Three.js",
            "ROS",
            "backend",
            "/api/",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.trajectory)

        harness = r'''
import {
  flattenTrajectoryPoint,
  normalizeDualArmTrajectory,
  trajectoryDurationSeconds,
  trajectoryMetadata,
  trajectoryPointAtIndex,
} from "./trajectory.mjs";

function rejects(callback) {
  try { callback(); return false; } catch (_error) { return true; }
}
const zeros = [0,0,0,0,0,0];
const source = {
  name: " Copy Test ",
  points: [
    { time_from_start_s: 0, left: [...zeros], right: [...zeros] },
    { time_from_start_s: 2.5, left: [1,2,3,4,5,6], right: [-1,-2,-3,-4,-5,-6] },
  ],
};
const before = JSON.stringify(source);
const normalized = normalizeDualArmTrajectory(source);
normalized.points[0].left[0] = 99;
const point = trajectoryPointAtIndex(source, 1);
point.right[0] = 99;

console.log(JSON.stringify({
  sourceUnchanged: JSON.stringify(source) === before,
  arraysCopied: normalized.points[0].left !== source.points[0].left
    && normalized.points[0].right !== source.points[0].right,
  name: normalized.name,
  duration: trajectoryDurationSeconds(source),
  metadata: trajectoryMetadata(source),
  flattened: flattenTrajectoryPoint(source.points[1]),
  pointCopyPreserved: source.points[1].right[0] === -1,
  rejectsNull: rejects(() => normalizeDualArmTrajectory(null)),
  rejectsNoPoints: rejects(() => normalizeDualArmTrajectory({})),
  rejectsOnePoint: rejects(() => normalizeDualArmTrajectory({ points: [source.points[0]] })),
  rejectsNonZeroStart: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], time_from_start_s: 0.1 }, source.points[1],
  ] })),
  acceptsTinyZeroError: !rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], time_from_start_s: 1e-10 }, source.points[1],
  ] })),
  rejectsEqualTime: rejects(() => normalizeDualArmTrajectory({ points: [
    source.points[0], { ...source.points[1], time_from_start_s: 0 },
  ] })),
  rejectsDecreasingTime: rejects(() => normalizeDualArmTrajectory({ points: [
    source.points[0], source.points[1], { ...source.points[1], time_from_start_s: 1 },
  ] })),
  rejectsNegativeTime: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], time_from_start_s: -1 }, source.points[1],
  ] })),
  rejectsNaNTime: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], time_from_start_s: NaN }, source.points[1],
  ] })),
  rejectsInfinityTime: rejects(() => normalizeDualArmTrajectory({ points: [
    source.points[0], { ...source.points[1], time_from_start_s: Infinity },
  ] })),
  rejectsBooleanTime: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], time_from_start_s: false }, source.points[1],
  ] })),
  rejectsShortLeft: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], left: [0] }, source.points[1],
  ] })),
  rejectsLongRight: rejects(() => normalizeDualArmTrajectory({ points: [
    source.points[0], { ...source.points[1], right: [0,0,0,0,0,0,0] },
  ] })),
  rejectsBooleanJoint: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], left: [0,0,0,0,0,true] }, source.points[1],
  ] })),
  rejectsStringJoint: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], right: [0,0,0,0,0,"0"] }, source.points[1],
  ] })),
  rejectsNaNJoint: rejects(() => normalizeDualArmTrajectory({ points: [
    { ...source.points[0], left: [0,0,0,0,0,NaN] }, source.points[1],
  ] })),
  rejectsInfinityJoint: rejects(() => normalizeDualArmTrajectory({ points: [
    source.points[0], { ...source.points[1], right: [0,0,0,0,0,Infinity] },
  ] })),
  rejectsFractionalIndex: rejects(() => trajectoryPointAtIndex(source, 0.5)),
  rejectsOutOfRangeIndex: rejects(() => trajectoryPointAtIndex(source, 2)),
}));
'''
        result = _run_node({"trajectory.mjs": self.trajectory}, harness)
        self.assertTrue(result["sourceUnchanged"])
        self.assertTrue(result["arraysCopied"])
        self.assertTrue(result["pointCopyPreserved"])
        self.assertEqual(result["name"], "Copy Test")
        self.assertEqual(result["duration"], 2.5)
        self.assertEqual(result["metadata"]["pointCount"], 2)
        self.assertEqual(result["metadata"]["jointCountPerPoint"], 12)
        self.assertEqual(len(result["flattened"]), 12)
        for key, value in result.items():
            if key.startswith("rejects"):
                with self.subTest(assertion=key):
                    self.assertTrue(value)
        self.assertTrue(result["acceptsTinyZeroError"])

    def test_sampling_exact_interpolation_clamping_and_non_uniform_time(self) -> None:
        harness = r'''
import { sampleTrajectoryAtTime } from "./trajectory.mjs";
const trajectory = {
  name: "Non-uniform",
  points: [
    { time_from_start_s: 0, left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] },
    { time_from_start_s: 1, left: [2,4,6,8,10,12], right: [-2,-4,-6,-8,-10,-12] },
    { time_from_start_s: 3.5, left: [7,9,11,13,15,17], right: [-7,-9,-11,-13,-15,-17] },
  ],
};
const before = JSON.stringify(trajectory);
const first = sampleTrajectoryAtTime(trajectory, 0);
const intermediate = sampleTrajectoryAtTime(trajectory, 1);
const final = sampleTrajectoryAtTime(trajectory, 3.5);
const midpoint = sampleTrajectoryAtTime(trajectory, 0.5);
const nonUniform = sampleTrajectoryAtTime(trajectory, 2);
const beforeStart = sampleTrajectoryAtTime(trajectory, -10);
const afterEnd = sampleTrajectoryAtTime(trajectory, 99);
midpoint.left[0] = 999;
console.log(JSON.stringify({
  first, intermediate, final, midpoint, nonUniform, beforeStart, afterEnd,
  sourceUnchanged: JSON.stringify(trajectory) === before,
}));
'''
        result = _run_node({"trajectory.mjs": self.trajectory}, harness)
        self.assertEqual(result["first"]["left"], [0, 0, 0, 0, 0, 0])
        self.assertEqual(result["first"]["alpha"], 0)
        self.assertEqual(result["intermediate"]["left"], [2, 4, 6, 8, 10, 12])
        self.assertEqual(result["intermediate"]["right"], [-2, -4, -6, -8, -10, -12])
        self.assertEqual(result["final"]["left"], [7, 9, 11, 13, 15, 17])
        self.assertEqual(result["final"]["alpha"], 1)
        self.assertEqual(result["midpoint"]["alpha"], 0.5)
        self.assertEqual(result["midpoint"]["right"][1:], [-2, -3, -4, -5, -6])
        self.assertEqual(result["nonUniform"]["segmentIndex"], 1)
        self.assertEqual(result["nonUniform"]["alpha"], 0.4)
        self.assertEqual(result["nonUniform"]["left"][0], 4)
        self.assertEqual(result["nonUniform"]["right"][0], -4)
        self.assertEqual(result["beforeStart"]["time_from_start_s"], 0)
        self.assertEqual(result["afterEnd"]["time_from_start_s"], 3.5)
        self.assertTrue(result["sourceUnchanged"])

    def test_controller_state_playback_and_actual_planned_independence(self) -> None:
        start = self.digital_twin.index("const EXPECTED_JOINTS =")
        end = self.digital_twin.index("const publicApi =")
        controller = self.digital_twin[start:end]
        binding = "function bindTrajectoryPreviewControls" + self.digital_twin.split(
            "function bindTrajectoryPreviewControls", 1
        )[1].split("function countVisualMeshes", 1)[0]
        harness = f'''
import {{
  DEFAULT_STALE_TIMEOUT_MS,
  isNormalizedSnapshotStale,
  normalizeDualArmStatusSnapshot,
}} from "./adapter.mjs";
import {{
  DEFAULT_VISUAL_SMOOTHING_TAU_MS,
  copyDualArmPose,
  smoothDualArmPose,
}} from "./smoothing.mjs";
import {{ maxAbsoluteJointDelta, normalizePlannedDualArmPose }} from "./planned.mjs";
import {{
  normalizeDualArmTrajectory,
  sampleTrajectoryAtTime,
  trajectoryDurationSeconds,
  trajectoryPointAtIndex,
}} from "./trajectory.mjs";

class FakeElement {{
  constructor(value = "") {{
    this.value = value;
    this.max = "0";
    this.disabled = true;
    this.textContent = "";
    this.listeners = new Map();
    this.attributes = {{}};
  }}
  addEventListener(type, callback) {{ this.listeners.set(type, callback); }}
  setAttribute(name, value) {{ this.attributes[name] = value; }}
  dispatch(type, event = {{}}) {{
    const callback = this.listeners.get(type);
    if (callback) callback({{ target: this, preventDefault() {{}}, ...event }});
  }}
}}
const uiElements = new Map([
  ["digitalTwinTrajectoryTimeInput", new FakeElement("0.0")],
  ["digitalTwinTrajectoryScrubber", new FakeElement("0")],
  ["digitalTwinTrajectoryScrubberTime", new FakeElement()],
  ["digitalTwinTrajectoryPlaybackRate", new FakeElement("1")],
]);
globalThis.document = {{
  getElementById(id) {{ return uiElements.get(id) || null; }},
}};
let nowMs = 0;
Object.defineProperty(globalThis, "performance", {{
  configurable: true,
  value: {{ now() {{ return nowMs; }} }},
}});
let nextFrameId = 1;
const frames = new Map();
globalThis.requestAnimationFrame = (callback) => {{
  const id = nextFrameId++;
  frames.set(id, callback);
  return id;
}};
globalThis.cancelAnimationFrame = (id) => frames.delete(id);
function runFrame(timestamp) {{
  nowMs = timestamp;
  const pending = [...frames.entries()];
  frames.clear();
  pending.forEach(([_id, callback]) => callback(timestamp));
}}

{controller}
{binding}

function modelWithRecorder(recorder) {{
  const model = {{ joints: {{}}, visible: false, updateMatrixWorld() {{}} }};
  for (const name of EXPECTED_JOINTS) {{
    model.joints[name] = {{ setJointValue(value) {{ recorder.push([name, value]); }} }};
  }}
  return model;
}}
const actualCalls = [];
const plannedCalls = [];
robot = modelWithRecorder(actualCalls);
plannedRobot = modelWithRecorder(plannedCalls);
loadState.status = "READY";
plannedLoadState.status = "READY";
plannedPreviewState.status = "READY";
bindTrajectoryPreviewControls();
const timeInput = uiElements.get("digitalTwinTrajectoryTimeInput");
const scrubber = uiElements.get("digitalTwinTrajectoryScrubber");
const scrubberTime = uiElements.get("digitalTwinTrajectoryScrubberTime");
const trajectory = {{
  name: "Controller Test",
  points: [
    {{ time_from_start_s: 0, left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] }},
    {{ time_from_start_s: 1, left: [1,1,1,1,1,1], right: [-1,-1,-1,-1,-1,-1] }},
    {{ time_from_start_s: 2, left: [2,2,2,2,2,2], right: [-2,-2,-2,-2,-2,-2] }},
  ],
}};

const empty = getTrajectoryPreviewState();
const loaded = loadPlannedTrajectory(trajectory, "TEST TRAJECTORY");
const actualAfterLoad = actualCalls.length;
const plannedAfterLoad = plannedCalls.length;
const loadedUi = {{
  input: timeInput.value,
  inputMax: timeInput.max,
  inputDisabled: timeInput.disabled,
  slider: scrubber.value,
  sliderLabel: scrubberTime.textContent,
}};
timeInput.value = "999";
timeInput.dispatch("change");
const clampedInput = getTrajectoryPreviewState();
const clampedUi = {{ input: timeInput.value, slider: scrubber.value, label: scrubberTime.textContent }};
timeInput.value = "";
timeInput.dispatch("blur");
const invalidInput = getTrajectoryPreviewState();
const invalidUi = {{ input: timeInput.value, slider: scrubber.value, label: scrubberTime.textContent }};
timeInput.value = "1.2";
timeInput.dispatch("keydown", {{ key: "Enter" }});
const entered = getTrajectoryPreviewState();
const jumped = setTrajectoryPointIndex(1);
const scrubbed = setTrajectoryTime(0.5);
const play = playPlannedTrajectory();
const framesAfterPlay = frames.size;
const duplicatePlay = playPlannedTrajectory();
const framesAfterDuplicate = frames.size;
timeInput.value = "0.75";
timeInput.dispatch("change");
const typedWhilePlaying = getTrajectoryPreviewState();
runFrame(500);
const afterFrame = getTrajectoryPreviewState();
const playingUi = {{ input: timeInput.value, slider: scrubber.value, label: scrubberTime.textContent }};
const paused = pausePlannedTrajectory();
const framesAfterPause = frames.size;
const plannedBeforeActualFeedback = plannedCalls.length;
setJointValues({{
  left: [0.1,0.1,0.1,0.1,0.1,0.1],
  right: [-0.1,-0.1,-0.1,-0.1,-0.1,-0.1],
}});
const plannedAfterActualFeedback = plannedCalls.length;
const actualAfterPausedFeedback = actualCalls.length;
nowMs = 700;
const resumed = playPlannedTrajectory();
runFrame(2200);
const finished = getTrajectoryPreviewState();
const finishedUi = {{ input: timeInput.value, slider: scrubber.value, label: scrubberTime.textContent }};
const stopped = stopPlannedTrajectory();
const stoppedUi = {{ input: timeInput.value, slider: scrubber.value, label: scrubberTime.textContent }};
const rateBeforeInvalid = stopped.playbackRate;
const invalidRate = setTrajectoryPlaybackRate(Infinity);
const rateAfterInvalid = invalidRate.playbackRate;
const rateTwo = setTrajectoryPlaybackRate(2);
setTrajectoryTime(0.5);
playPlannedTrajectory();
hidePlannedModel();
const hiddenBeforeFrame = plannedRobot.visible;
runFrame(2300);
const hiddenAfterFrame = plannedRobot.visible;
pausePlannedTrajectory();
const trajectoryAfterHide = getTrajectoryPreviewState();
showPlannedModel();
const visibleAfterShow = plannedRobot.visible;
const beforeReset = getTrajectoryPreviewState();
resetToStaticPose();
const afterReset = getTrajectoryPreviewState();
const actualBeforeClear = actualCalls.length;
const cleared = clearPlannedTrajectory();
const actualAfterClear = actualCalls.length;

console.log(JSON.stringify({{
  empty, loaded, loadedUi, clampedInput, clampedUi, invalidInput, invalidUi, entered,
  actualAfterLoad, plannedAfterLoad, jumped, scrubbed,
  play, framesAfterPlay, duplicatePlay, framesAfterDuplicate, afterFrame,
  typedWhilePlaying, playingUi, finishedUi, stoppedUi,
  paused, framesAfterPause, resumed, finished, stopped,
  plannedBeforeActualFeedback, plannedAfterActualFeedback, actualAfterPausedFeedback,
  rateBeforeInvalid, rateAfterInvalid, invalidRate, rateTwo,
  hiddenBeforeFrame, hiddenAfterFrame, trajectoryAfterHide, visibleAfterShow,
  beforeReset, afterReset, cleared,
  actualBeforeClear, actualAfterClear,
  plannedVisibleAfterLoad: loaded.status === "READY" && plannedAfterLoad === 12,
}}));
'''
        result = _run_node(
            {
                "adapter.mjs": self.adapter,
                "smoothing.mjs": SMOOTHING_PATH.read_text(encoding="utf-8"),
                "planned.mjs": self.planned,
                "trajectory.mjs": self.trajectory,
            },
            harness,
        )
        self.assertEqual(result["empty"]["status"], "EMPTY")
        self.assertEqual(result["empty"]["playbackRate"], 1)
        self.assertEqual(result["loaded"]["status"], "READY")
        self.assertEqual(result["loaded"]["currentTimeS"], 0)
        self.assertEqual(result["loadedUi"], {
            "input": "0.0",
            "inputMax": "2",
            "inputDisabled": False,
            "slider": "0",
            "sliderLabel": "0.0 s",
        })
        self.assertEqual(result["clampedInput"]["currentTimeS"], 2)
        self.assertEqual(result["clampedUi"], {
            "input": "2.0", "slider": "2", "label": "2.0 s",
        })
        self.assertEqual(result["invalidInput"]["currentTimeS"], 2)
        self.assertEqual(result["invalidUi"], {
            "input": "2.0", "slider": "2", "label": "2.0 s",
        })
        self.assertEqual(result["entered"]["currentTimeS"], 1.2)
        self.assertTrue(result["plannedVisibleAfterLoad"])
        self.assertEqual(result["actualAfterLoad"], 0)
        self.assertEqual(result["plannedAfterLoad"], 12)
        self.assertEqual(result["jumped"]["currentTimeS"], 1)
        self.assertEqual(result["jumped"]["currentPointIndex"], 1)
        self.assertEqual(result["scrubbed"]["currentTimeS"], 0.5)
        self.assertEqual(result["play"]["status"], "PLAYING")
        self.assertTrue(result["play"]["playing"])
        self.assertEqual(result["framesAfterPlay"], 1)
        self.assertEqual(result["framesAfterDuplicate"], 1)
        self.assertTrue(result["typedWhilePlaying"]["playing"])
        self.assertEqual(result["typedWhilePlaying"]["currentTimeS"], 0.75)
        self.assertEqual(result["afterFrame"]["currentTimeS"], 1.25)
        self.assertEqual(result["playingUi"], {
            "input": "1.3", "slider": "1.25", "label": "1.3 s",
        })
        self.assertEqual(result["paused"]["status"], "PAUSED")
        self.assertFalse(result["paused"]["playing"])
        self.assertEqual(result["framesAfterPause"], 0)
        self.assertEqual(
            result["plannedBeforeActualFeedback"],
            result["plannedAfterActualFeedback"],
        )
        self.assertEqual(result["actualAfterPausedFeedback"], 12)
        self.assertEqual(result["resumed"]["status"], "PLAYING")
        self.assertEqual(result["finished"]["status"], "FINISHED")
        self.assertFalse(result["finished"]["playing"])
        self.assertEqual(result["finished"]["currentTimeS"], 2)
        self.assertEqual(result["finished"]["currentPointIndex"], 2)
        self.assertEqual(result["finishedUi"], {
            "input": "2.0", "slider": "2", "label": "2.0 s",
        })
        self.assertEqual(result["stopped"]["status"], "READY")
        self.assertEqual(result["stopped"]["currentTimeS"], 0)
        self.assertEqual(result["stoppedUi"], {
            "input": "0.0", "slider": "0", "label": "0.0 s",
        })
        self.assertEqual(result["rateBeforeInvalid"], result["rateAfterInvalid"])
        self.assertIn("finite number", result["invalidRate"]["validationError"])
        self.assertEqual(result["rateTwo"]["playbackRate"], 2)
        self.assertFalse(result["hiddenBeforeFrame"])
        self.assertFalse(result["hiddenAfterFrame"])
        self.assertIsNotNone(result["trajectoryAfterHide"]["trajectory"])
        self.assertTrue(result["visibleAfterShow"])
        self.assertEqual(result["beforeReset"]["trajectory"], result["afterReset"]["trajectory"])
        self.assertEqual(result["actualBeforeClear"], result["actualAfterClear"])
        self.assertEqual(result["cleared"]["status"], "EMPTY")
        self.assertIsNone(result["cleared"]["trajectory"])

    def test_public_api_ui_offline_contract_and_mock_data(self) -> None:
        api = self.digital_twin.split("const publicApi = {", 1)[1].split("};", 1)[0]
        for method in (
            "loadPlannedTrajectory",
            "clearPlannedTrajectory",
            "setTrajectoryTime",
            "setTrajectoryPointIndex",
            "playPlannedTrajectory",
            "pausePlannedTrajectory",
            "stopPlannedTrajectory",
            "setTrajectoryPlaybackRate",
            "getTrajectoryPreviewState",
        ):
            with self.subTest(method=method):
                self.assertIn(method, api)

        trajectory_controller = self.digital_twin.split(
            "function trajectoryStateSnapshot", 1
        )[1].split("function formatMirrorTimestamp", 1)[0]
        self.assertNotIn("setJointValues(", trajectory_controller)
        self.assertIn("setPlannedJointValues(", trajectory_controller)
        self.assertIn("requestAnimationFrame", trajectory_controller)
        self.assertIn("performance.now()", trajectory_controller)
        self.assertNotIn("setInterval", trajectory_controller)
        self.assertIn("elapsedSeconds * trajectoryPreviewState.playbackRate", trajectory_controller)
        self.assertIn("time_from_start_s: 4.2", self.digital_twin)

        panel = self.html.split(
            'class="digital-twin-trajectory-panel"', 1
        )[1].split('class="digital-twin-warning"', 1)[0]
        for label in (
            "Planned Joint Trajectory — OFFLINE ONLY",
            "Trajectory State",
            "Trajectory Name",
            "Point Count",
            "Current Segment / Point",
            "Current Time",
            "Duration",
            "Progress",
            "Playback Rate",
            "Trajectory Source",
            "Validation Error",
            "Joint Unit",
            "radians",
            "Time Unit",
            "seconds",
            "Load Mock Trajectory A",
            "Load Mock Trajectory B",
            "Previous Point",
            "Next Point",
            "Play",
            "Pause",
            "Stop",
            "Clear Trajectory",
            "0.25x",
            "0.5x",
            "1.0x",
            "2.0x",
            "PREVIEW DOES NOT EXECUTE ROBOT MOTION",
            "INTERPOLATION IS FOR VISUALIZATION ONLY",
        ):
            with self.subTest(label=label):
                self.assertIn(label, panel)
        self.assertIn('id="digitalTwinTrajectoryScrubber"', panel)
        self.assertIn('type="range"', panel)
        self.assertIn('id="digitalTwinTrajectoryTimeInput"', panel)
        self.assertIn('type="number"', panel)
        self.assertIn('step="0.1"', panel)
        self.assertIn('id="digitalTwinTrajectoryScrubberTime"', panel)
        self.assertNotIn(">Execute<", panel)
        self.assertNotIn("fetch(", trajectory_controller)
        self.assertNotIn("/api/", trajectory_controller)

        binding = self.digital_twin.split(
            "function bindTrajectoryPreviewControls", 1
        )[1].split("function countVisualMeshes", 1)[0]
        self.assertIn('document.getElementById("digitalTwinTrajectoryTimeInput")', binding)
        self.assertIn("setTrajectoryTime(timeSeconds)", binding)
        self.assertIn('event.key === "Enter"', binding)
        self.assertIn('addEventListener("change"', binding)
        self.assertIn('addEventListener("blur"', binding)
        self.assertIn("Number.isFinite(timeSeconds)", binding)

    def test_sticky_viewer_css_and_rendering_contract(self) -> None:
        wrapper = self.html.split(
            'class="digital-twin-sticky-viewer"', 1
        )[1].split('aria-label="Digital Twin viewer controls"', 1)[0]
        self.assertIn('id="digitalTwinViewer"', wrapper)
        self.assertIn('id="digitalTwinStatus"', wrapper)

        sticky_css = self.html.split(
            ".digital-twin-sticky-viewer {", 1
        )[1].split("}", 1)[0]
        self.assertIn("position: sticky", sticky_css)
        self.assertNotIn("position: fixed", sticky_css)
        self.assertIn("top:", sticky_css)
        self.assertIn("z-index:", sticky_css)
        self.assertIn("background: var(--panel)", sticky_css)

        viewer_css = self.html.split(
            ".digital-twin-viewer {", 1
        )[1].split("}", 1)[0]
        self.assertIn("height: clamp(340px, 50vh, 620px)", viewer_css)
        self.assertIn("min-height: 320px", viewer_css)
        self.assertIn("width: 100%", viewer_css)
        self.assertIn("new ResizeObserver(handleResize).observe(container)", self.digital_twin)


if __name__ == "__main__":
    unittest.main()
