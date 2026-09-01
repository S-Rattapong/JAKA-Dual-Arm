"""Offline browser regressions for Phase-5 Frozen Ghost execution authority."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from dual_arm_app.tests.test_phase5_operator_execution import (
    frozen_artifact,
    make_coordinator,
)


ROOT = Path(__file__).resolve().parents[2]
CONTROLLER = ROOT / "dual_arm_app/web/digital_twin_phase5_execution.js"
WAYPOINT_TARGET = ROOT / "dual_arm_app/web/digital_twin_phase5_waypoint_target.js"


def run_phase5_browser_harness(harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase5-preview-authority-") as temp:
        directory = Path(temp)
        controller = CONTROLLER.read_text(encoding="utf-8").replace(
            '"./digital_twin_phase5_waypoint_target.js"', '"./waypoint_target.mjs"'
        )
        (directory / "phase5.mjs").write_text(controller, encoding="utf-8")
        (directory / "waypoint_target.mjs").write_text(
            WAYPOINT_TARGET.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise AssertionError(result.stderr)
    return json.loads(result.stdout)


class Phase5ExecutionPreviewAuthorityTests(unittest.TestCase):
    def test_coordinator_preserves_terminal_artifact_a_diagnostics_after_b_freezes(self):
        for terminal_state in ("COMPLETED", "FAILED"):
            with self.subTest(terminal_state=terminal_state):
                coordinator, state, _clock, transport, _aborts = make_coordinator()
                state["artifact"].duration_s = 3.0
                coordinator.prepare()
                self.assertTrue(coordinator.execute(True)["ok"])

                coordinator._feedback_getter = lambda trajectory_id, terminal=terminal_state: {
                    "read_only": True,
                    "authoritative": True,
                    "feedback_status": "AUTHORITATIVE",
                    "combined_state": terminal,
                    "terminal": True,
                    "result": terminal,
                    "reason": f"ARTIFACT_A_{terminal}",
                    "trajectory_id": trajectory_id,
                    "drivers": {},
                    "common_timeline": {
                        "elapsed_s": 3.0,
                        "duration_s": 3.0,
                        "progress_0_to_1": 1.0,
                    },
                }
                closed_a = coordinator.inspection_state()["execution"]
                self.assertEqual(closed_a["state"], terminal_state)
                self.assertEqual(closed_a["artifact_fingerprint"], "artifact-1")
                self.assertEqual(closed_a["artifact_generation"], 7)

                state["artifact"] = frozen_artifact("artifact-2", "plan-2")
                state["artifact"].duration_s = 6.0
                state["generation"] = 8
                status_with_b = coordinator.inspection_state()

                self.assertEqual(
                    status_with_b["artifact"]["artifact_fingerprint"], "artifact-2"
                )
                self.assertEqual(status_with_b["artifact"]["generation"], 8)
                self.assertEqual(status_with_b["execution"]["state"], terminal_state)
                self.assertEqual(
                    status_with_b["execution"]["artifact_fingerprint"], "artifact-1"
                )
                self.assertEqual(
                    status_with_b["execution"]["artifact_generation"], 7
                )
                self.assertEqual(
                    status_with_b["driver_feedback"]["common_timeline"]["duration_s"],
                    3.0,
                )
                self.assertEqual(len(transport.calls), 1)

    def test_stale_terminal_execution_cannot_own_new_frozen_ghost(self):
        result = run_phase5_browser_harness(r'''
const elements = new Map();
function ui(id) {
  if (!elements.has(id)) {
    elements.set(id, {
      id, textContent: "", disabled: false, hidden: false, checked: false,
      value: 0, dataset: {}, listeners: {},
      addEventListener(type, callback) { this.listeners[type] = callback; },
      closest() { return {dataset: {}}; },
    });
  }
  return elements.get(id);
}

globalThis.window = globalThis;
globalThis.document = {
  readyState: "complete",
  getElementById: (id) => ui(id),
  addEventListener() {},
};
globalThis.addEventListener = () => {};
const intervalCallbacks = [];
globalThis.setInterval = (callback, milliseconds) => {
  intervalCallbacks.push({callback, milliseconds});
  return intervalCallbacks.length;
};
globalThis.confirm = () => { throw new Error("motion confirmation must not run"); };

let loaded = null;
let currentTimeS = null;
let playing = false;
let plannedVisible = false;
const executionTargets = [];
const hiddenTargets = [];
const makePoint = (index, time) => {
  const left = Array(6).fill(index);
  const right = Array(6).fill(-index);
  return {
    sample_index: index, time_from_start_s: time, left, right,
    combined: [...left, ...right],
  };
};
const integratedPlan = {
  ok: true, object_waypoint_count: 3,
  waypoints: ["W0", "W1", "W2"].map((identifier) => ({identifier})),
  global_path: [makePoint(0, 0), makePoint(1, 3), makePoint(2, 6)],
  object_samples: [0, 3, 6].map((time, sample_index) => ({
    sample_index, time_from_start_s: time,
  })),
  approach_path: [],
  combined_path: [makePoint(0, 0), makePoint(1, 3), makePoint(2, 6)],
};
let actualPose = makePoint(0, 0);
globalThis.dualArmDigitalTwin = {
  loadPlannedTrajectory(trajectory, source) {
    loaded = {
      trajectory, source, durationS: trajectory.points.at(-1).time_from_start_s,
      pointCount: trajectory.points.length,
    };
    currentTimeS = 0;
    plannedVisible = true;
    return this.getTrajectoryPreviewState();
  },
  getTrajectoryPreviewState() {
    return loaded && {
      status: "READY", source: loaded.source, durationS: loaded.durationS,
      pointCount: loaded.pointCount,
    };
  },
  getPlannedPreviewState() {
    return loaded && {
      visible: plannedVisible, source: loaded.source, loadState: {status: "READY"},
    };
  },
  clearPlannedTrajectory() { loaded = null; plannedVisible = false; },
  stopPlannedTrajectory() { currentTimeS = 0; playing = false; plannedVisible = true; },
  setTrajectoryPlaybackRate() {},
  playPlannedTrajectory() { playing = true; },
  getObjectGlobalPlanState() { return {status: "READY", plan: integratedPlan}; },
  getExecutionGateState() { return {currentPlanFingerprint: "plan-B"}; },
  getLatestActualPose() { return {left: actualPose.left, right: actualPose.right}; },
  getMirrorState() {
    return {
      enabled: true, hasValidSnapshot: true, mode: "LIVE_MIRROR",
      lastAcceptedSnapshotMs: Date.now(), staleTimeoutMs: 1500,
    };
  },
  setExecutionWaypointTarget(target) {
    executionTargets.push({
      waypointIndex: target.waypointIndex, identifier: target.identifier,
      timeS: target.timeS, pose: target.pose,
    });
    currentTimeS = target.timeS;
    playing = false;
    plannedVisible = true;
    return {applied: true};
  },
  hideExecutionWaypointTarget() {
    hiddenTargets.push(currentTimeS);
    plannedVisible = false;
  },
};

const artifactB = {
  available: true,
  artifact_fingerprint: "artifact-B",
  plan_fingerprint: "plan-B",
  generation: 8,
};
const status = {
  ok: true,
  artifact: artifactB,
  phase4_gate: {execution_ready: true, status: "READY"},
  transport: {robot_connection: {both_ready: true}},
  safe_state: {ok: true},
  start_match: {match: true},
  stop_generation: 11,
  ready_to_prepare: true,
  ready_to_replan_from_current: true,
  ready_to_move_to_initial: false,
  operator_authority: {pending: false, single_use: true, no_expiry: true},
  blocking_reasons: [],
  execution: {
    state: "COMPLETED",
    trajectory_id: "execution-A",
    artifact_fingerprint: "artifact-A",
    plan_fingerprint: "plan-A",
    artifact_generation: 7,
    duration_s: 3,
    reason: "BOTH_DRIVERS_COMPLETED",
  },
  driver_feedback: {
    authoritative: true,
    combined_state: "COMPLETED",
    reason: "BOTH_DRIVERS_COMPLETED",
    common_timeline: {elapsed_s: 3, duration_s: 3, progress_0_to_1: 1},
    drivers: {},
  },
};
const artifactPayload = {
  ok: true, available: true, status: "FROZEN", generation: 8,
  artifact: {
    artifact_fingerprint: "artifact-B",
    trajectory: {
      name: "six-second-B", duration_s: 6,
      common_timestamps_s: [0, 3, 6],
      left: {positions_rad: [[0,0,0,0,0,0], [1,1,1,1,1,1], [2,2,2,2,2,2]]},
      right: {positions_rad: [[0,0,0,0,0,0], [-1,-1,-1,-1,-1,-1], [-2,-2,-2,-2,-2,-2]]},
    },
  },
};
const requests = [];
globalThis.fetch = async (url, options = {}) => {
  requests.push({url, method: options.method || "GET"});
  const payload = url.endsWith("execution-artifact") ? artifactPayload : status;
  return {ok: true, status: 200, json: async () => payload};
};

await import("./phase5.mjs");
for (let index = 0; index < 20 && !globalThis.dualArmPhase5Execution.previewReady(); index += 1) {
  await new Promise((resolve) => setImmediate(resolve));
}

executionTargets.length = 0;
currentTimeS = 4.8; // operator scrubs the new six-second B Ghost away from 50%
await globalThis.dualArmPhase5Execution.refreshStatus();
const completedDisplay = ui("digitalTwinPhase5ExecutionState").textContent;
const scrubTimeAfterStaleCompletedPoll = currentTimeS;

ui("digitalTwinPhase5PreviewPlay").listeners.click();
const playingImmediatelyAfterClick = playing;
await globalThis.dualArmPhase5Execution.refreshStatus();
const playingAfterStaleCompletedPoll = playing;
const timeAfterStaleCompletedPoll = currentTimeS;

status.execution.state = "FAILED";
status.execution.reason = "OLD_ARTIFACT_FAILED";
status.driver_feedback.combined_state = "FAILED";
status.driver_feedback.reason = "OLD_ARTIFACT_FAILED";
await globalThis.dualArmPhase5Execution.refreshStatus();
const failedDisplay = ui("digitalTwinPhase5ExecutionState").textContent;
const failedResultDisplay = ui("digitalTwinPhase5CombinedResult").textContent;

const previewReadyForB = globalThis.dualArmPhase5Execution.previewReady();
const prepareEnabledForB = ui("digitalTwinPhase5Prepare").disabled === false;
const staleTargetCount = executionTargets.length;

status.execution = {
  state: "RUNNING", trajectory_id: "execution-B",
  artifact_fingerprint: "artifact-B", plan_fingerprint: "plan-B",
  artifact_generation: 8, duration_s: 6, reason: null,
};
status.driver_feedback = {
  authoritative: true, combined_state: "RUNNING", reason: null,
  common_timeline: {elapsed_s: 2.5, duration_s: 6, progress_0_to_1: 2.5 / 6},
  drivers: {},
};
await globalThis.dualArmPhase5Execution.refreshStatus();
const firstMatchingTarget = executionTargets.at(-1);
const firstMatchingTimeS = currentTimeS;

actualPose = makePoint(1, 3);
status.driver_feedback.common_timeline.elapsed_s = 3;
status.driver_feedback.common_timeline.progress_0_to_1 = 0.5;
await globalThis.dualArmPhase5Execution.refreshStatus();

actualPose = makePoint(2, 6);
status.driver_feedback.common_timeline.elapsed_s = 6;
status.driver_feedback.common_timeline.progress_0_to_1 = 1;
await globalThis.dualArmPhase5Execution.refreshStatus();
const hiddenAtFinalWaypoint = plannedVisible === false;

status.execution.state = "COMPLETED";
status.driver_feedback.combined_state = "COMPLETED";
status.driver_feedback.common_timeline.elapsed_s = 4;
await globalThis.dualArmPhase5Execution.refreshStatus();
const hiddenAtTerminal = plannedVisible === false;
const previewReadyWhileTerminalHidden = globalThis.dualArmPhase5Execution.previewReady();
ui("digitalTwinPhase5PreviewPlay").listeners.click();
const manualPlayAfterTerminal = playing && plannedVisible && currentTimeS === 0;
await globalThis.dualArmPhase5Execution.refreshStatus();
const manualPlaySurvivesTerminalPoll = playing && plannedVisible;

console.log(JSON.stringify({
  loadedDurationS: loaded.durationS,
  staleTargetCount,
  scrubTimeAfterStaleCompletedPoll,
  playingImmediatelyAfterClick,
  playingAfterStaleCompletedPoll,
  timeAfterStaleCompletedPoll,
  completedDisplay,
  failedDisplay,
  failedResultDisplay,
  previewReadyForB,
  prepareEnabledForB,
  firstMatchingTarget,
  firstMatchingTimeS,
  executionTargetSequence: executionTargets.map((target) => target.identifier),
  hiddenAtFinalWaypoint,
  hiddenAtTerminal,
  previewReadyWhileTerminalHidden,
  manualPlayAfterTerminal,
  manualPlaySurvivesTerminalPoll,
  matchingPlaying: playing,
  pollIntervalMs: intervalCallbacks[0].milliseconds,
  postRequests: requests.filter((request) => request.method === "POST"),
}));
''')

        self.assertEqual(result["loadedDurationS"], 6)
        self.assertEqual(result["staleTargetCount"], 0)
        self.assertEqual(result["scrubTimeAfterStaleCompletedPoll"], 4.8)
        self.assertTrue(result["playingImmediatelyAfterClick"])
        self.assertTrue(result["playingAfterStaleCompletedPoll"])
        self.assertEqual(result["timeAfterStaleCompletedPoll"], 0)
        self.assertIn("PREVIOUS ARTIFACT", result["completedDisplay"])
        self.assertIn("PREVIOUS ARTIFACT", result["failedDisplay"])
        self.assertIn("PREVIOUS ARTIFACT", result["failedResultDisplay"])
        self.assertTrue(result["previewReadyForB"])
        self.assertTrue(result["prepareEnabledForB"])
        self.assertEqual(result["firstMatchingTarget"]["identifier"], "W1")
        self.assertEqual(result["firstMatchingTarget"]["waypointIndex"], 1)
        self.assertEqual(result["firstMatchingTarget"]["timeS"], 3)
        self.assertEqual(result["firstMatchingTimeS"], 3)
        self.assertEqual(result["executionTargetSequence"], ["W1", "W2"])
        self.assertTrue(result["hiddenAtFinalWaypoint"])
        self.assertTrue(result["hiddenAtTerminal"])
        self.assertFalse(result["previewReadyWhileTerminalHidden"])
        self.assertTrue(result["manualPlayAfterTerminal"])
        self.assertTrue(result["manualPlaySurvivesTerminalPoll"])
        self.assertTrue(result["matchingPlaying"])
        self.assertEqual(result["pollIntervalMs"], 100)
        self.assertEqual(result["postRequests"], [])


if __name__ == "__main__":
    unittest.main()
