"""No-delete offline checks for the high-rate smooth Live Mirror path."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
LIVE_SOURCE = WEB / "digital_twin_live_source.js"
SMOOTHING = WEB / "digital_twin_live_smoothing.js"
CONTROLLER = WEB / "digital_twin.js"
DRIVER = ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"


class LiveMirrorSmoothingSafeTests(unittest.TestCase):
    def test_polling_is_10ms_read_only_cache_polling(self):
        source = LIVE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("const DEFAULT_POLL_INTERVAL_MS = 10;", source)
        self.assertIn('const JOINT_FEEDBACK_ENDPOINT = "/api/digital-twin/joints";', source)
        self.assertIn('method: "GET"', source)
        self.assertIn('cache: "no-store"', source)
        for forbidden in (
            "/api/jog", "/api/home", "/api/stop", "/api/direct",
            "/api/program/run", "/api/digital-twin/phase5/execute",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)

    def test_pure_smoothing_is_frame_rate_driven_and_wrap_safe(self):
        script = r'''
import {
  DEFAULT_VISUAL_SMOOTHING_TAU_MS,
  shortestAngularDeltaRad,
  smoothingAlpha,
  smoothDualArmPose,
} from "./dual_arm_app/web/digital_twin_live_smoothing.js";
const alpha60 = smoothingAlpha(1000 / 60);
const wrap = shortestAngularDeltaRad(Math.PI - 0.01, -Math.PI + 0.01);
const current = { left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] };
const target = { left: [1,1,1,1,1,1], right: [-1,-1,-1,-1,-1,-1] };
const smooth = smoothDualArmPose(current, target, 1000 / 144);
console.log(JSON.stringify({ alpha60, wrap, smooth, tau: DEFAULT_VISUAL_SMOOTHING_TAU_MS }));
'''
        result = subprocess.run(
            ["node", "--input-type=module", "-e", script],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(result.stdout)
        self.assertEqual(payload["tau"], 18)
        self.assertGreater(payload["alpha60"], 0)
        self.assertLess(payload["alpha60"], 1)
        self.assertAlmostEqual(payload["wrap"], 0.02, places=6)
        self.assertGreater(payload["smooth"]["left"][0], 0)
        self.assertLess(payload["smooth"]["left"][0], 1)
        self.assertLess(payload["smooth"]["right"][0], 0)
        self.assertGreater(payload["smooth"]["right"][0], -1)

    def test_main_model_uses_raf_smoothing_only_for_live_mirror(self):
        controller = CONTROLLER.read_text(encoding="utf-8")
        self.assertIn('from "./digital_twin_live_smoothing.js"', controller)
        self.assertIn("function applySmoothedMirrorVisualFrame", controller)
        self.assertIn("applySmoothedMirrorVisualFrame(frameTimeMs, Date.now())", controller)
        self.assertIn('mainModelState.source = "LIVE JOINT FEEDBACK"', controller)
        apply_latest = controller.split(
            "function applyLatestMirrorSnapshotIfPossible", 1
        )[1].split("function ingestStatusSnapshot", 1)[0]
        self.assertNotIn("setJointValues({", apply_latest)
        self.assertIn("seedMirrorVisualPoseFromLatest", apply_latest)
        self.assertIn("LIVE_MIRROR_VISUAL_UI_SYNC_MS = 50", controller)
        smooth_frame = controller.split(
            "function applySmoothedMirrorVisualFrame", 1
        )[1].split("function applyLatestMirrorSnapshotIfPossible", 1)[0]
        self.assertIn("applyMirrorPoseToMainModel", smooth_frame)
        # RAF smoothing may move only the rendered White MAIN model; it must not
        # overwrite the authoritative raw actual pose used by waypoint logic.
        apply_main = controller.split(
            "function applyMirrorPoseToMainModel", 1
        )[1].split("function seedMirrorVisualPoseFromLatest", 1)[0]
        self.assertNotIn("latestActualPose =", apply_main)
        ingest = controller.split("function ingestStatusSnapshot", 1)[1].split(
            "function setMirrorEnabled", 1
        )[0]
        self.assertIn("latestActualPose = {", ingest)
        self.assertNotIn("setExecutionWaypointTarget", smooth_frame)
        self.assertNotIn("hideExecutionWaypointTarget", smooth_frame)
        animate = controller.split("const animate =", 1)[1].split(
            "loadRobot();", 1
        )[0]
        self.assertIn("requestAnimationFrame(animate)", animate)
        self.assertIn("applySmoothedMirrorVisualFrame(frameTimeMs, Date.now())", animate)

    def test_driver_feedback_loop_is_unchanged_and_smoothing_is_visual_only(self):
        driver = DRIVER.read_text(encoding="utf-8")
        feedback_loop = driver.split("void get_conn_scoket_state()", 1)[1].split(
            "int main(", 1
        )[0]
        self.assertIn("rclcpp::sleep_for(chrono::milliseconds(50))", feedback_loop)
        smoothing = SMOOTHING.read_text(encoding="utf-8")
        for forbidden in ("fetch(", "rclpy", "joint_move", "servo_j", "/api/"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, smoothing)


if __name__ == "__main__":
    unittest.main()
