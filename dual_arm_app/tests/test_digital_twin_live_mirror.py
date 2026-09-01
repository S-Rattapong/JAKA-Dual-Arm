"""Offline tests for the Phase 1C.1 status adapter and mirror controller."""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = REPOSITORY_ROOT / "dual_arm_app"
ADAPTER_PATH = APP_ROOT / "web/digital_twin_status_adapter.js"
SMOOTHING_PATH = APP_ROOT / "web/digital_twin_live_smoothing.js"
DIGITAL_TWIN_PATH = APP_ROOT / "web/digital_twin.js"
HTML_PATH = APP_ROOT / "web/index.html"
MOTION_ROUTE_FRAGMENTS = (
    "/api/jog",
    "/api/home",
    "/api/stop",
    "/api/direct",
    "/api/waypoint",
    "/api/program",
    "/api/fk",
    "/api/ik",
)


def _run_node_harness(adapter_source: str, harness_source: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1c1-") as temporary_directory:
        directory = Path(temporary_directory)
        (directory / "adapter.mjs").write_text(adapter_source, encoding="utf-8")
        (directory / "smoothing.mjs").write_text(
            SMOOTHING_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "harness.mjs").write_text(harness_source, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            check=True,
            capture_output=True,
            text=True,
        )
    return json.loads(result.stdout)


class DigitalTwinLiveMirrorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.adapter = ADAPTER_PATH.read_text(encoding="utf-8")
        cls.digital_twin = DIGITAL_TWIN_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_adapter_exists_and_has_no_runtime_or_backend_dependencies(self) -> None:
        self.assertTrue(ADAPTER_PATH.is_file())
        forbidden = ("fetch", "/api/status", "window", "document", "rclpy", "ROS")
        for text in forbidden + MOTION_ROUTE_FRAGMENTS:
            with self.subTest(text=text):
                self.assertNotIn(text, self.adapter)

    def test_adapter_normalization_rejection_copy_and_staleness(self) -> None:
        harness = r'''
import {
  DEFAULT_STALE_TIMEOUT_MS,
  isNormalizedSnapshotStale,
  normalizeDualArmStatusSnapshot,
} from "./adapter.mjs";

function rejects(callback) {
  try { callback(); return false; } catch (_error) { return true; }
}

const source = {
  left: { joint: [0, 1, 2, 3, 4, 5], tcp: [10, 20, 30] },
  right: { joint: [-1, -2, -3, -4, -5, -6], state: { ready: true } },
};
const sourceBefore = JSON.stringify(source);
const normalized = normalizeDualArmStatusSnapshot(source, 1000);
normalized.left[0] = 99;

const output = {
  timeout: DEFAULT_STALE_TIMEOUT_MS,
  sourceUnchanged: JSON.stringify(source) === sourceBefore,
  copiedArrays: normalized.left !== source.left.joint && normalized.right !== source.right.joint,
  preservedRadians: normalized.right.join(",") === "-1,-2,-3,-4,-5,-6",
  freshAtBoundary: !isNormalizedSnapshotStale({ receivedAtMs: 1000 }, 2500, 1500),
  staleAfterBoundary: isNormalizedSnapshotStale({ receivedAtMs: 1000 }, 2501, 1500),
  rejectsNull: rejects(() => normalizeDualArmStatusSnapshot(null)),
  rejectsUndefined: rejects(() => normalizeDualArmStatusSnapshot(undefined)),
  rejectsSnapshotString: rejects(() => normalizeDualArmStatusSnapshot("invalid")),
  rejectsMissingSide: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0,0,0,0] } })),
  rejectsShort: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0] }, right: { joint: [0,0,0,0,0,0] } })),
  rejectsLong: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0,0,0,0,0] }, right: { joint: [0,0,0,0,0,0] } })),
  rejectsString: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0,0,0,"0"] }, right: { joint: [0,0,0,0,0,0] } })),
  rejectsBoolean: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0,0,0,true] }, right: { joint: [0,0,0,0,0,0] } })),
  rejectsNaN: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0,0,0,NaN] }, right: { joint: [0,0,0,0,0,0] } })),
  rejectsInfinity: rejects(() => normalizeDualArmStatusSnapshot({ left: { joint: [0,0,0,0,0,Infinity] }, right: { joint: [0,0,0,0,0,0] } })),
};
console.log(JSON.stringify(output));
'''
        result = _run_node_harness(self.adapter, harness)
        self.assertEqual(result["timeout"], 1500)
        for key, value in result.items():
            if key != "timeout":
                with self.subTest(assertion=key):
                    self.assertTrue(value)

    def test_actual_mirror_controller_behavior_with_stub_robot(self) -> None:
        start = self.digital_twin.index("const EXPECTED_JOINTS =")
        end = self.digital_twin.index("const publicApi =")
        controller_source = self.digital_twin[start:end]
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
globalThis.document = {{ getElementById() {{ return null; }} }};
{controller_source}
const applied = [];
robot = {{
  joints: {{}},
  updateMatrixWorld() {{}},
}};
for (const name of EXPECTED_JOINTS) {{
  robot.joints[name] = {{ setJointValue(value) {{ applied.push([name, value]); }} }};
}}
loadState.status = "READY";

const initial = getMirrorState(1000);
const enabledWithoutSnapshot = setMirrorEnabled(true);
const now = Date.now();
const pose = MOCK_POSE_B;
const live = ingestStatusSnapshot(pose, now, "CUSTOM SOURCE");
const appliedAfterValid = applied.length;
const appliedValues = applied.slice();
const stale = getMirrorState(now + 1501);
const invalid = ingestStatusSnapshot({{ left: {{ joint: [0] }}, right: {{ joint: [0,0,0,0,0,0] }} }}, now + 2);
const appliedAfterInvalid = applied.length;
const reset = resetToStaticPose();
const resetValues = applied.slice(-12).map(entry => entry[1]);
const defaultSource = ingestStatusSnapshot(pose, now + 3);
const fallbackSource = ingestStatusSnapshot(pose, now + 4, "   ");

console.log(JSON.stringify({{
  initial,
  enabledWithoutSnapshot,
  live,
  stale,
  invalid,
  reset,
  defaultSource,
  fallbackSource,
  appliedAfterValid,
  appliedAfterInvalid,
  appliedValues,
  resetValues,
  mockA: [...MOCK_POSE_A.left.joint, ...MOCK_POSE_A.right.joint],
  mockB: [...MOCK_POSE_B.left.joint, ...MOCK_POSE_B.right.joint],
}}));
'''
        result = _run_node_harness(self.adapter, harness)
        self.assertEqual(result["initial"]["mode"], "STATIC")
        self.assertFalse(result["initial"]["enabled"])
        self.assertEqual(result["enabledWithoutSnapshot"]["mode"], "MIRROR_READY")
        self.assertEqual(result["live"]["mode"], "LIVE_MIRROR")
        self.assertEqual(result["live"]["updateSource"], "CUSTOM SOURCE")
        self.assertEqual(result["appliedAfterValid"], 12)
        self.assertEqual(result["stale"]["mode"], "STALE")
        self.assertEqual(result["invalid"]["mode"], "INVALID")
        self.assertEqual(result["appliedAfterInvalid"], 12)
        self.assertTrue(result["invalid"]["hasValidSnapshot"])
        self.assertEqual(result["invalid"]["updateSource"], "CUSTOM SOURCE")
        self.assertEqual(
            result["invalid"]["lastAcceptedSnapshotMs"],
            result["live"]["lastAcceptedSnapshotMs"],
        )
        self.assertEqual(result["reset"]["mode"], "STATIC")
        self.assertFalse(result["reset"]["enabled"])
        self.assertEqual(
            result["defaultSource"]["updateSource"],
            "LOCAL STATUS SNAPSHOT",
        )
        self.assertEqual(
            result["fallbackSource"]["updateSource"],
            "LOCAL STATUS SNAPSHOT",
        )
        self.assertEqual(result["resetValues"], [0] * 12)
        self.assertEqual(len(result["mockA"]), 12)
        self.assertEqual(len(result["mockB"]), 12)
        self.assertTrue(all(math.isfinite(value) for value in result["mockA"]))
        self.assertTrue(all(math.isfinite(value) for value in result["mockB"]))
        applied_radians = [entry[1] for entry in result["appliedValues"]]
        self.assertEqual(
            applied_radians,
            [0.20, -0.35, 0.25, 0.15, -0.20, 0.10,
             -0.20, 0.35, -0.25, -0.15, 0.20, -0.10],
        )

    def test_public_api_and_offline_only_source_contract(self) -> None:
        required_methods = (
            "resetCamera",
            "fitModel",
            "toggleGrid",
            "toggleAxes",
            "setJointValues",
            "getLoadState",
            "ingestStatusSnapshot",
            "setMirrorEnabled",
            "getMirrorState",
            "resetToStaticPose",
        )
        api_block = self.digital_twin.split("const publicApi = {", 1)[1].split(
            "};", 1
        )[0]
        for method in required_methods:
            with self.subTest(method=method):
                self.assertIn(method, api_block)
        self.assertNotIn("/api/status", self.digital_twin)
        self.assertNotIn("fetch(", self.digital_twin)
        for route in MOTION_ROUTE_FRAGMENTS:
            with self.subTest(route=route):
                self.assertNotIn(route, self.digital_twin)
        self.assertIn("enabled: false", self.digital_twin)
        self.assertIn("DEFAULT_STALE_TIMEOUT_MS", self.digital_twin)
        self.assertNotIn("radToDeg", self.digital_twin)

    def test_mock_poses_and_ui_are_offline_radians_only(self) -> None:
        for label in (
            "Mirror Mode",
            "Last Snapshot",
            "Joint Unit:",
            "radians",
            "Update Source",
            "Enable Mirror",
            "Disable Mirror",
            "Reset Static Pose",
            "Load Mock Pose A — OFFLINE MOCK",
            "Load Mock Pose B — OFFLINE MOCK",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html)
        self.assertIn("MOCK_POSE_A", self.digital_twin)
        self.assertIn("MOCK_POSE_B", self.digital_twin)
        self.assertIn("ingestStatusSnapshot(snapshot, Date.now(), label)", self.digital_twin)
        self.assertIn('"OFFLINE MOCK A"', self.digital_twin)
        self.assertIn('"OFFLINE MOCK B"', self.digital_twin)
        self.assertIn("[0.20, -0.35, 0.25, 0.15, -0.20, 0.10]", self.digital_twin)
        self.assertIn("[-0.20, 0.35, -0.25, -0.15, 0.20, -0.10]", self.digital_twin)

    def test_existing_controls_remain_and_no_execute_button_is_added(self) -> None:
        labels = (
            "Live Position / Direct Move",
            "Direct Joint Move",
            "Direct TCP Move",
            "Dual JAKA A12 Manual Jog",
            "STOP BOTH",
            "Home Both",
            "Refresh Status",
            "Waypoint Manager",
            "Program / Sequence",
            "System Status",
        )
        for label in labels:
            with self.subTest(label=label):
                self.assertIn(label, self.html)
        panel = self.html.split('aria-labelledby="digitalTwinTitle"', 1)[1].split(
            "</section>", 1
        )[0]
        self.assertNotIn(">Execute<", panel)

    def test_user_data_has_no_unstaged_changes(self) -> None:
        result = subprocess.run(
            [
                "git",
                "diff",
                "--quiet",
                "--",
                "dual_arm_app/tasks/waypoints.json",
                "dual_arm_app/programs",
                "dual_arm_app/objects",
            ],
            cwd=REPOSITORY_ROOT,
            check=False,
        )
        self.assertEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
