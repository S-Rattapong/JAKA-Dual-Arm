"""Pure offline tests for Phase-3B integrated Object/Planned preview."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
PLANNING = WEB / "digital_twin_phase3_planning.js"
TRAJECTORY = WEB / "digital_twin_object_trajectory_preview.js"
GRASP = WEB / "digital_twin_object_grasp_preview.js"
CONTROLLER = WEB / "digital_twin.js"


def run_node(harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase3b-preview-") as temp:
        directory = Path(temp)
        grasp = GRASP.read_text(encoding="utf-8")
        trajectory = TRAJECTORY.read_text(encoding="utf-8").replace(
            '"./digital_twin_object_grasp_preview.js"', '"./grasp.mjs"'
        )
        planning = PLANNING.read_text(encoding="utf-8").replace(
            '"./digital_twin_object_trajectory_preview.js"', '"./trajectory.mjs"'
        )
        (directory / "grasp.mjs").write_text(grasp, encoding="utf-8")
        (directory / "trajectory.mjs").write_text(trajectory, encoding="utf-8")
        (directory / "planning.mjs").write_text(planning, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise AssertionError(result.stderr)
    return json.loads(result.stdout)


PLAN_FIXTURE = r'''
const orientation = [0.1, -0.2, 0.3];
const makeObject = (index) => ({
  sample_index: index,
  time_from_start_s: index,
  object_pose: { translation_m: [index, index * 2, 0.8 + index * 0.1], rpy_rad: orientation },
});
const makePoint = (index) => {
  const left = Array.from({length: 6}, (_, joint) => index + joint * 0.1);
  const right = Array.from({length: 6}, (_, joint) => -index - joint * 0.1);
  return {
    sample_index: index, time_from_start_s: index, left, right,
    combined: [...left, ...right], graph_node_id: `L${index}:N0`, graph_node_index: 0,
    edge_cost_rad2: index, cumulative_cost_rad2: index,
  };
};
const fixture = {
  ok: true, plan_only: true, planner_status: "READY", trajectory_name: "fixture",
  candidate_attempts_per_arm: 3,
  candidate_exploration_profile: "WEB RUNTIME — BOUNDED IK SEED EXPLORATION",
  candidate_pruning_applied: false, planning_elapsed_s: 0.25,
  planner_source: "GLOBAL GRAPH SEARCH — GENERATED GRAPH OPTIMUM",
  optimality_scope: "Exact optimum over generated layered candidate graph",
  object_waypoint_count: 3, object_sample_count: 3, duration_s: 2,
  fixed_orientation_rpy_rad: orientation,
  grasp: {
    status: "GRASP_LOCKED", source: "LOCKED OPERATOR GRASP",
    content_revision: "sha256:test-grasp", lock_generation: 1,
    lock_revision: "sha256:test-lock", left: {matrix: [[1,0,0,0],[0,1,0,0.25],[0,0,1,0],[0,0,0,1]]},
    right: {matrix: [[1,0,0,0],[0,1,0,-0.25],[0,0,1,0],[0,0,0,1]]},
    expected_left_T_right: {matrix: [[1,0,0,0],[0,1,0,-0.5],[0,0,1,0],[0,0,0,1]]},
  },
  calibration: {
    revision: "sha256:test-calibration", model_revision: "sha256:test-calibration",
    revision_status: "MATCH",
  },
  object_samples: [0, 1, 2].map(makeObject),
  global_path: [0, 1, 2].map(makePoint),
  graph: {layer_count: 3, node_counts_per_layer: [2, 3, 2], selected_node_ids: ["L0:N0", "L1:N0", "L2:N0"]},
  global: {completed: true, total_cost_rad2: 2},
  greedy: {completed: true, total_cost_rad2: 3},
  comparison: {global_not_worse: true, cost_difference_rad2: 1},
  warnings: ["PLAN ONLY", "NO ROBOT EXECUTION"],
};
'''


class PureWaypointOperationsTests(unittest.TestCase):
    def test_add_delete_reorder_clear_and_demo_are_deterministic(self):
        result = run_node(r'''
import {
  PHASE3_DEMO_OBJECT_WAYPOINTS,
  addObjectPlanningWaypoint,
  deleteObjectPlanningWaypoint,
  moveObjectPlanningWaypoint,
  normalizeObjectPlanningWaypoints,
} from "./planning.mjs";
let values = [];
values = addObjectPlanningWaypoint(values, {identifier: "A", translation_m: [0, 1, 2]});
values = addObjectPlanningWaypoint(values, {identifier: "B", translation_m: [3, 4, 5]});
values = addObjectPlanningWaypoint(values, {identifier: "C", translation_m: [6, 7, 8]});
values = moveObjectPlanningWaypoint(values, 2, -1);
const moved = values.map((item) => item.identifier);
values = deleteObjectPlanningWaypoint(values, 1);
const deleted = values.map((item) => item.identifier);
const demo = normalizeObjectPlanningWaypoints(PHASE3_DEMO_OBJECT_WAYPOINTS, 2);
values = [];
console.log(JSON.stringify({moved, deleted, cleared: values.length, demo}));
''')
        self.assertEqual(result["moved"], ["A", "C", "B"])
        self.assertEqual(result["deleted"], ["A", "B"])
        self.assertEqual(result["cleared"], 0)
        self.assertEqual([item["identifier"] for item in result["demo"]], ["W0", "W1", "W2"])

    def test_duplicate_empty_and_fewer_than_two_are_rejected(self):
        result = run_node(r'''
import { buildObjectGlobalPlanRequest } from "./planning.mjs";
const base = {
  name: "Plan", fixedOrientationRpyRad: [0, 0, 0],
  segmentDurationS: 1, samplesPerSegment: 2,
};
const cases = [
  [{identifier: "", translation_m: [0,0,0]}, {identifier: "B", translation_m: [1,0,0]}],
  [{identifier: "A", translation_m: [0,0,0]}, {identifier: "a", translation_m: [1,0,0]}],
  [{identifier: "A", translation_m: [0,0,0]}],
];
const rejected = cases.map((waypoints) => {
  try { buildObjectGlobalPlanRequest({...base, waypoints}); return false; }
  catch (_error) { return true; }
});
console.log(JSON.stringify({rejected}));
''')
        self.assertEqual(result["rejected"], [True, True, True])

    def test_web_candidate_attempt_budget_defaults_to_3_and_validates_1_to_13(self):
        result = run_node(r'''
import { buildObjectGlobalPlanRequest } from "./planning.mjs";
const base = {
  name: "Plan",
  waypoints: [
    {identifier: "A", translation_m: [0,0,0]},
    {identifier: "B", translation_m: [1,0,0]},
  ],
  fixedOrientationRpyRad: [0,0,0], segmentDurationS: 1, samplesPerSegment: 2,
  expectedGraspContentRevision: "sha256:test-grasp", expectedLockGeneration: 1,
  expectedLockRevision: "sha256:test-lock",
  expectedCalibrationRevision: "sha256:test-calibration",
  expectedModelCalibrationRevision: "sha256:test-calibration",
};
const defaultAttempts = buildObjectGlobalPlanRequest(base).candidate_attempts_per_arm;
const selected = [1, 13].map((candidateAttemptsPerArm) => (
  buildObjectGlobalPlanRequest({...base, candidateAttemptsPerArm})
    .candidate_attempts_per_arm
));
const rejected = [0, 14, true, 3.5].map((candidateAttemptsPerArm) => {
  try { buildObjectGlobalPlanRequest({...base, candidateAttemptsPerArm}); return false; }
  catch (_error) { return true; }
});
console.log(JSON.stringify({defaultAttempts, selected, rejected}));
''')
        self.assertEqual(result["defaultAttempts"], 3)
        self.assertEqual(result["selected"], [1, 13])
        self.assertEqual(result["rejected"], [True, True, True, True])


class IntegratedSamplingTests(unittest.TestCase):
    def test_exact_samples_and_common_interpolation_alpha(self):
        result = run_node(r'''
import {
  normalizeObjectGlobalPlanResponse,
  sampleIntegratedObjectGlobalPlan,
  globalPlanToPlannedTrajectory,
} from "./planning.mjs";
''' + PLAN_FIXTURE + r'''
const plan = normalizeObjectGlobalPlanResponse(fixture);
const exact = sampleIntegratedObjectGlobalPlan(plan, 1);
const middle = sampleIntegratedObjectGlobalPlan(plan, 1.5);
const clamped = sampleIntegratedObjectGlobalPlan(plan, 99);
const joints = globalPlanToPlannedTrajectory(plan);
console.log(JSON.stringify({exact, middle, clamped, joints, orientation: plan.fixed_orientation_rpy_rad}));
''')
        self.assertAlmostEqual(result["exact"]["left"][0], 1)
        for actual, expected in zip(
            result["exact"]["objectPose"]["translationM"], [1, 2, 0.9]
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(result["middle"]["segmentIndex"], 1)
        self.assertAlmostEqual(result["middle"]["alpha"], 0.5)
        self.assertAlmostEqual(result["middle"]["left"][0], 1.5)
        for actual, expected in zip(
            result["middle"]["objectPose"]["translationM"], [1.5, 3, 0.95]
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            result["middle"]["objectPose"]["rpyRad"], [0.1, -0.2, 0.3]
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(result["clamped"]["time_from_start_s"], 2)
        self.assertEqual(len(result["joints"]["points"]), 3)

    def test_misaligned_times_are_rejected_but_variable_orientation_is_accepted(self):
        result = run_node(r'''
import { normalizeObjectGlobalPlanResponse } from "./planning.mjs";
''' + PLAN_FIXTURE + r'''
const badTime = JSON.parse(JSON.stringify(fixture));
badTime.global_path[1].time_from_start_s = 1.1;
let badTimeRejected = false;
try { normalizeObjectGlobalPlanResponse(badTime); } catch (_error) { badTimeRejected = true; }
const rotating = JSON.parse(JSON.stringify(fixture));
rotating.object_samples[1].object_pose.rpy_rad[2] = 0.4;
const rotatingAccepted = normalizeObjectGlobalPlanResponse(rotating).object_samples[1].object_pose.rpy_rad[2];
console.log(JSON.stringify({badTimeRejected, rotatingAccepted}));
''')
        self.assertTrue(result["badTimeRejected"])
        self.assertEqual(result["rotatingAccepted"], 0.4)

    def test_request_uses_only_plan_endpoint_and_normalizes_response(self):
        result = run_node(r'''
import { requestObjectGlobalPlan } from "./planning.mjs";
''' + PLAN_FIXTURE + r'''
let captured = null;
const fakeFetch = async (url, options) => {
  captured = {url, options};
  return {ok: true, status: 200, json: async () => fixture};
};
async function main() {
  const response = await requestObjectGlobalPlan({name: "request"}, fakeFetch);
  console.log(JSON.stringify({url: captured.url, method: captured.options.method, ok: response.ok}));
}
main();
''')
        self.assertEqual(result, {
            "url": "/api/digital-twin/plan-object-global",
            "method": "POST",
            "ok": True,
        })


class ControllerOwnershipTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = CONTROLLER.read_text(encoding="utf-8")

    def test_integrated_object_and_both_planned_arms_share_planned_clock(self):
        apply_body = self.source.split("function applyTrajectoryTime", 1)[1].split(
            "function loadPlannedTrajectory", 1
        )[0]
        self.assertIn("setPlannedJointValues", apply_body)
        self.assertIn("applyIntegratedPlanTime(sample.time_from_start_s)", apply_body)
        self.assertIn("sampleIntegratedObjectGlobalPlan", self.source)
        state = self.source.split("const integratedPlanPreviewState", 1)[1].split("};", 1)[0]
        self.assertNotIn("animationFrameId", state)

    def test_success_path_feeds_existing_planned_ghost_once(self):
        body = self.source.split("function loadObjectGlobalPlanPreview", 1)[1].split(
            "async function planObjectGlobal", 1
        )[0]
        self.assertIn("globalPlanToPlannedTrajectory(plan)", body)
        self.assertEqual(body.count("loadPlannedTrajectory("), 1)
        self.assertNotIn("new THREE", body)
        self.assertNotIn("actualRobot", body)

    def test_play_pause_resume_reset_speed_and_scrub_delegate_to_one_clock(self):
        for function_name, delegate in (
            ("setIntegratedPlanTime", "setTrajectoryTime"),
            ("playIntegratedPlan", "playPlannedTrajectory"),
            ("pauseIntegratedPlan", "pausePlannedTrajectory"),
            ("resetIntegratedPlan", "stopPlannedTrajectory"),
        ):
            body = self.source.split(f"function {function_name}", 1)[1].split("\n}", 1)[0]
            self.assertIn(delegate, body)
        self.assertIn("setTrajectoryPlaybackRate(Number(event.target.value))", self.source)
        self.assertIn("setTrajectoryTime(Number(event.target.value))", self.source)

    def test_manual_and_standalone_object_preview_ownership_remain_explicit(self):
        self.assertIn('integratedPlanPreviewState.owner = "MANUAL OBJECT PREVIEW"', self.source)
        self.assertIn('deactivateIntegratedPlanPreview("STANDALONE OBJECT TRAJECTORY")', self.source)
        self.assertIn("SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY", self.source)
        self.assertIn("loadPlannedTrajectory(\n      MOCK_TRAJECTORY_A", self.source)


if __name__ == "__main__":
    unittest.main()
