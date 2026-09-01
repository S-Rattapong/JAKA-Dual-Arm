"""Pure browser regressions for the Phase-5 next rigid waypoint Ghost."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HELPER = WEB / "digital_twin_phase5_waypoint_target.js"
CONTROLLER = WEB / "digital_twin_phase5_execution.js"
VIEWER = WEB / "digital_twin.js"


def run_helper(harness: str) -> dict:
    # Run directly from the workspace module; do not create/delete scratch files.
    script = harness.replace(
        'from "./target.mjs"',
        f'from {json.dumps(HELPER.as_uri())}',
    )
    result = subprocess.run(
        ["node", "--input-type=module", "-e", script],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(result.stderr)
    return json.loads(result.stdout)


FIXTURE = r'''
const point = (sample_index, time_from_start_s, value = sample_index) => {
  const left = Array(6).fill(value);
  const right = Array(6).fill(-value);
  return {sample_index, time_from_start_s, left, right, combined: [...left, ...right]};
};
const rigid = Array.from({length: 7}, (_, index) => point(index, index));
const objects = Array.from({length: 7}, (_, sample_index) => ({
  sample_index, time_from_start_s: sample_index,
}));
const basePlan = {
  ok: true, object_waypoint_count: 3,
  waypoints: ["W0", "W1", "W2"].map((identifier) => ({identifier})),
  global_path: rigid,
  object_samples: objects,
  approach_path: [],
  combined_path: rigid,
};
'''


class Phase5WaypointTargetHelperTests(unittest.TestCase):
    def test_no_approach_actual_w0_then_w1_w2_cursor_and_final_hide(self):
        result = run_helper(r'''
import {
  createRigidWaypointTargetCursor, updateRigidWaypointTargetCursor,
} from "./target.mjs";
''' + FIXTURE + r'''
const options = (actualPose) => ({
  executionIdentity: "run-a", actualPose, actualReceivedAtMs: 990, nowMs: 1000,
});
let cursor = createRigidWaypointTargetCursor(basePlan, "run-a");
const atW0 = updateRigidWaypointTargetCursor(basePlan, cursor, options(point(0, 0, 0)));
const atW1 = updateRigidWaypointTargetCursor(basePlan, atW0, options(point(0, 0, 3)));
const atW2 = updateRigidWaypointTargetCursor(basePlan, atW1, options(point(0, 0, 6)));
console.log(JSON.stringify({cursor, atW0, atW1, atW2}));
''')
        self.assertEqual(result["cursor"]["identifier"], "W1")
        self.assertEqual(result["atW0"]["identifier"], "W1")
        self.assertFalse(result["atW0"]["advanced"])
        self.assertEqual(result["atW1"]["identifier"], "W2")
        self.assertTrue(result["atW1"]["advanced"])
        self.assertFalse(result["atW2"]["visible"])
        self.assertEqual(result["atW2"]["reason"], "FINAL_RIGID_WAYPOINT_REACHED")

    def test_default_visual_switch_waits_until_actual_is_within_tighter_tolerance(self):
        result = run_helper(r'''
import {
  createRigidWaypointTargetCursor, updateRigidWaypointTargetCursor,
} from "./target.mjs";
''' + FIXTURE + r'''
const cursor = createRigidWaypointTargetCursor(basePlan, "run-tight");
const target = point(0, 0, 3);
const outside = {left: [...target.left], right: [...target.right]};
outside.left[0] += 0.016;
const inside = {left: [...target.left], right: [...target.right]};
inside.left[0] += 0.014;
const options = (actualPose) => ({
  executionIdentity: "run-tight", actualPose, actualReceivedAtMs: 990, nowMs: 1000,
});
const before = updateRigidWaypointTargetCursor(basePlan, cursor, options(outside));
const after = updateRigidWaypointTargetCursor(basePlan, before, options(inside));
console.log(JSON.stringify({before, after}));
''');
        self.assertEqual(result["before"]["identifier"], "W1")
        self.assertFalse(result["before"]["advanced"])
        self.assertEqual(result["after"]["identifier"], "W2")
        self.assertTrue(result["after"]["advanced"])

    def test_approach_targets_w0_until_actual_reaches_w0(self):
        result = run_helper(r'''
import {
  createRigidWaypointTargetCursor, updateRigidWaypointTargetCursor,
} from "./target.mjs";
''' + FIXTURE + r'''
const approach = [point(0, 0, -2), point(1, 1, -1), point(2, 2, 0)];
const combined = [
  point(0, 0, -2), point(1, 1, -1),
  ...rigid.map((sample, offset) => point(offset + 2, sample.time_from_start_s + 2, offset)),
];
const plan = {
  ...basePlan,
  approach: {enabled: true, duration_s: 2},
  approach_path: approach,
  combined_path: combined,
};
const options = (actualPose) => ({
  executionIdentity: "run-approach", actualPose,
  actualReceivedAtMs: 990, nowMs: 1000,
});
const cursor = createRigidWaypointTargetCursor(plan, "run-approach");
const approaching = updateRigidWaypointTargetCursor(
  plan, cursor, options(point(0, 0, -1)),
);
const reachedW0 = updateRigidWaypointTargetCursor(
  plan, approaching, options(point(0, 0, 0)),
);
console.log(JSON.stringify({cursor, approaching, reachedW0}));
''')
        self.assertEqual(result["cursor"]["identifier"], "W0")
        self.assertEqual(result["approaching"]["identifier"], "W0")
        self.assertEqual(result["reachedW0"]["identifier"], "W1")

    def test_stale_actual_does_not_advance_and_identity_mismatch_hides(self):
        result = run_helper(r'''
import {
  createRigidWaypointTargetCursor, updateRigidWaypointTargetCursor,
} from "./target.mjs";
''' + FIXTURE + r'''
const cursor = createRigidWaypointTargetCursor(basePlan, "run-current");
const stale = updateRigidWaypointTargetCursor(basePlan, cursor, {
  executionIdentity: "run-current", actualPose: point(0, 0, 3),
  actualReceivedAtMs: 700, nowMs: 1000, freshnessThresholdMs: 250,
});
const previous = updateRigidWaypointTargetCursor(basePlan, cursor, {
  executionIdentity: "run-previous", actualPose: point(0, 0, 3),
  actualReceivedAtMs: 990, nowMs: 1000,
});
console.log(JSON.stringify({stale, previous}));
''')
        self.assertEqual(result["stale"]["identifier"], "W1")
        self.assertFalse(result["stale"]["advanced"])
        self.assertFalse(result["stale"]["actualFresh"])
        self.assertFalse(result["previous"]["visible"])
        self.assertEqual(result["previous"]["reason"], "EXECUTION_IDENTITY_MISMATCH")

    def test_visual_tolerance_is_explicit_and_not_a_safety_gate(self):
        source = HELPER.read_text(encoding="utf-8")
        self.assertIn("EXECUTION_GHOST_JOINT_TOLERANCE_RAD = 0.015", source)
        self.assertIn("Visual-only waypoint indicator tolerance", source)
        for authority in ("safety", "authorize", "servo_j", "trajectory state"):
            with self.subTest(authority=authority):
                self.assertNotIn(f"function {authority}", source)


class Phase5WaypointTargetOwnershipTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = CONTROLLER.read_text(encoding="utf-8")
        cls.viewer = VIEWER.read_text(encoding="utf-8")

    def test_execution_uses_actual_proximity_cursor_not_elapsed_following(self):
        body = self.controller.split(
            "function updateExecutionWaypointTarget", 1
        )[1].split("function applyStatus", 1)[0]
        self.assertIn("updateRigidWaypointTargetCursor", body)
        self.assertIn("getLatestActualPose", body)
        self.assertIn("lastAcceptedSnapshotMs", body)
        self.assertIn("setExecutionWaypointTarget(target)", body)
        self.assertNotIn("elapsed_s", body)
        self.assertNotIn("setExecutionPreviewTime", self.controller)

    def test_execution_hides_target_when_actual_feedback_is_stale(self):
        body = self.controller.split(
            "function updateExecutionWaypointTarget", 1
        )[1].split("function applyStatus", 1)[0]
        self.assertIn("target.actualFresh !== true", body)
        self.assertIn("hideExecutionTargetGhost(digitalTwin)", body)
        self.assertNotIn("elapsed_s", body)

    def test_previous_artifact_fails_closed_and_cursor_is_monotonic(self):
        body = self.controller.split(
            "function updateExecutionWaypointTarget", 1
        )[1].split("function applyStatus", 1)[0]
        helper = HELPER.read_text(encoding="utf-8")
        self.assertIn("executionMatchesFrozenArtifact", body)
        self.assertIn("resetExecutionTargetCursor", body)
        self.assertIn("targetIndex + 1", helper)
        self.assertNotIn("targetIndex -", helper)

    def test_manual_frozen_preview_still_uses_continuous_raf_playback(self):
        frozen_play = self.controller.split(
            "function playFrozenArtifactPreview", 1
        )[1].split("function resetFrozenArtifactPreview", 1)[0]
        advance = self.viewer.split(
            "function advanceTrajectoryPlayback", 1
        )[1].split("function playPlannedTrajectory", 1)[0]
        scrub = self.viewer.split("function setTrajectoryTime", 1)[1].split(
            "function setTrajectoryPointIndex", 1
        )[0]
        self.assertIn("playPlannedTrajectory()", frozen_play)
        self.assertIn("elapsedSeconds * trajectoryPreviewState.playbackRate", advance)
        self.assertIn("scheduleTrajectoryFrame()", advance)
        self.assertIn("applyTrajectoryTime(timeSeconds, nextStatus)", scrub)

    def test_execution_target_helpers_are_planned_only(self):
        body = self.viewer.split("function setExecutionWaypointTarget", 1)[1].split(
            "function playIntegratedPlan", 1
        )[0]
        self.assertIn("setPlannedJointValues", body)
        self.assertIn("hidePlannedModel", body)
        self.assertNotIn("applyMirrorPoseToMainModel", body)
        self.assertNotIn("setJointValues", body)
        self.assertNotIn("mainModelState", body)


if __name__ == "__main__":
    unittest.main()
