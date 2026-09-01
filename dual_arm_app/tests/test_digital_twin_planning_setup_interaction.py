"""Offline regression tests for planning setup and 3D waypoint interaction."""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path

from dual_arm_app.backend.object_global_planning import (
    ObjectGlobalPlanInputError,
    normalize_object_global_plan_request,
    plan_object_global,
)
from dual_arm_app.backend.planning_start_state_config import (
    PLANNING_START_JOINT_NAMES,
    PLANNING_START_STATE_RAD,
    PLANNING_START_STATE_SOURCE,
    PLANNING_START_STATE_UNIT,
    PlanningStartStateConfigError,
    normalize_planning_start_state_rad,
    planning_start_state_payload,
)
from dual_arm_app.backend.object_trajectory_ik import (
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
    DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG,
)
from dual_arm_app.backend.object_grasp_model import SYNTHETIC_OBJECT_GRASP_FIXTURE


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HTML_PATH = WEB / "index.html"
CONTROLLER_PATH = WEB / "digital_twin.js"
PLANNING_PATH = WEB / "digital_twin_phase3_planning.js"
TRAJECTORY_PATH = WEB / "digital_twin_object_trajectory_preview.js"
GRASP_PATH = WEB / "digital_twin_object_grasp_preview.js"
STATUS_ADAPTER_PATH = WEB / "digital_twin_status_adapter.js"
SMOOTHING_PATH = WEB / "digital_twin_live_smoothing.js"
PLANNER_PATH = ROOT / "dual_arm_app/backend/object_global_planning.py"
GRAPH_PATH = ROOT / "dual_arm_app/backend/object_trajectory_ik.py"
BACKEND_PATH = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
CONFIG_PATH = ROOT / "dual_arm_app/backend/planning_start_state_config.py"
START_STATE_SOURCE_PATH = WEB / "digital_twin_planning_start_state_source.js"


class IdParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids: list[str] = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for name, value in attrs if name == "id")


def run_planning_module(harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="planning-setup-") as temporary:
        directory = Path(temporary)
        (directory / "grasp.mjs").write_text(
            GRASP_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        trajectory = TRAJECTORY_PATH.read_text(encoding="utf-8").replace(
            '"./digital_twin_object_grasp_preview.js"', '"./grasp.mjs"'
        )
        planning = PLANNING_PATH.read_text(encoding="utf-8").replace(
            '"./digital_twin_object_trajectory_preview.js"', '"./trajectory.mjs"'
        )
        (directory / "trajectory.mjs").write_text(trajectory, encoding="utf-8")
        (directory / "planning.mjs").write_text(planning, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        completed = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode:
            raise AssertionError(completed.stderr)
        return json.loads(completed.stdout)


def run_main_model_ownership_harness(controller: str) -> dict:
    controller_slice = controller[
        controller.index("const EXPECTED_JOINTS ="):
        controller.index("const publicApi =")
    ]
    configured = {
        "left": list(PLANNING_START_STATE_RAD["left"]),
        "right": list(PLANNING_START_STATE_RAD["right"]),
    }
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
{controller_slice}
const writes = [];
robot = {{ joints: {{}}, updateMatrixWorld() {{}} }};
for (const name of EXPECTED_JOINTS) {{
  robot.joints[name] = {{ setJointValue(value) {{ writes.push([name, value]); }} }};
}}
loadState.status = "READY";
planningStartState.left = {json.dumps(configured["left"])};
planningStartState.right = {json.dumps(configured["right"])};
planningStartState.source = "{PLANNING_START_STATE_SOURCE}";
planningStartState.status = "READY";

const cacheBeforeOffline = mirrorState.latestValidSnapshot;
const offlineApplied = applyPlanningStartStateToMainWhenOffline(1000);
const offlinePose = JSON.parse(JSON.stringify(latestActualPose));
const offlineMirror = mirrorStateSnapshot();
const plannedAfterOffline = plannedPreviewStateSnapshot();
const writesAfterOffline = writes.length;

setMirrorEnabled(true);
const now = Date.now();
const liveInput = {{
  left: {{ joint: [0.11,0.12,0.13,0.14,0.15,0.16] }},
  right: {{ joint: [-0.21,-0.22,-0.23,-0.24,-0.25,-0.26] }},
}};
const liveMirror = ingestStatusSnapshot(liveInput, now, "LIVE TEST FEEDBACK");
const livePose = JSON.parse(JSON.stringify(latestActualPose));
const cachedAfterLive = JSON.parse(JSON.stringify(mirrorState.latestValidSnapshot));
const writesAfterLive = writes.length;
const configBlockedByLive = applyPlanningStartStateToMainWhenOffline(now + 1);
const poseAfterBlockedConfig = JSON.parse(JSON.stringify(latestActualPose));
const writesAfterBlockedConfig = writes.length;

const staleMirror = getMirrorState(now + DEFAULT_STALE_TIMEOUT_MS + 1);
const staleFallbackPose = JSON.parse(JSON.stringify(latestActualPose));
const cachedAfterStale = JSON.parse(JSON.stringify(mirrorState.latestValidSnapshot));

console.log(JSON.stringify({{
  offlineApplied,
  offlinePose,
  offlineMirror,
  cacheBeforeOffline,
  plannedAfterOffline,
  writesAfterOffline,
  liveMirror,
  livePose,
  cachedAfterLive,
  writesAfterLive,
  configBlockedByLive,
  poseAfterBlockedConfig,
  writesAfterBlockedConfig,
  staleMirror,
  staleFallbackPose,
  cachedAfterStale,
}}));
'''
    with tempfile.TemporaryDirectory(prefix="main-model-ownership-") as temporary:
        directory = Path(temporary)
        (directory / "adapter.mjs").write_text(
            STATUS_ADAPTER_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "smoothing.mjs").write_text(
            SMOOTHING_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        completed = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode:
            raise AssertionError(completed.stderr)
        return json.loads(completed.stdout)


def planning_payload(start=None):
    return {
        "name": "Planning_Setup_Test",
        "waypoints": [
            {"identifier": "W0", "translation_m": [0.0, 0.0, 0.8]},
            {"identifier": "W1", "translation_m": [0.1, 0.0, 0.9]},
        ],
        "fixed_orientation_rpy_rad": [0.0, 0.0, 0.0],
        "segment_duration_s": 1.0,
        "samples_per_segment": 2,
        "candidate_attempts_per_arm": 1,
        "initial_joint_state_rad": start or {
            "left": [0.0] * 6,
            "right": [0.0] * 6,
        },
        "planning_start_state_source": "MANUAL OFFLINE TEST",
        "expected_grasp_content_revision": "sha256:test-grasp",
        "expected_lock_generation": 1,
        "expected_lock_revision": "sha256:test-lock",
        "expected_calibration_revision": "sha256:test-calibration",
        "expected_model_calibration_revision": "sha256:test-calibration",
    }


TEST_PLANNING_AUTHORITY = {
    "grasp_status": "GRASP_LOCKED",
    "grasp_source": "LOCKED OPERATOR GRASP",
    "grasp_content_revision": "sha256:test-grasp",
    "lock_generation": 1,
    "lock_revision": "sha256:test-lock",
    "left": {"translation_m": [0, 0.25, 0], "rpy_rad": [0, 0, 0]},
    "right": {"translation_m": [0, -0.25, 0], "rpy_rad": [0, 0, 0]},
    "calibration_revision": "sha256:test-calibration",
    "model_calibration_revision": "sha256:test-calibration",
    "calibration_revision_status": "MATCH",
    "calibration_state": "MODEL_DEFAULT",
    "physical_calibration": "NOT_CALIBRATED",
    "physically_calibrated": False,
    "tcp_tool_contract_status": "UNVERIFIED",
}


def run_test_plan(payload, adapter, limits):
    return plan_object_global(
        payload,
        adapter,
        limits,
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=TEST_PLANNING_AUTHORITY,
    )


class FixedPlanningAdapter:
    def __init__(self):
        self.seeds: list[tuple[str, tuple[float, ...]]] = []

    def solve_arm_ik(
        self,
        *,
        group_name,
        ik_link_name,
        target_world_T_tip,
        seed_joint_positions_rad,
        timeout_s,
        avoid_collisions,
    ):
        del ik_link_name, target_world_T_tip, timeout_s, avoid_collisions
        seed = tuple(seed_joint_positions_rad)
        self.seeds.append((group_name, seed))
        values = (0.1,) * 6 if group_name == "left_arm" else (-0.2,) * 6
        return ArmIkSolution(True, values, "fixed offline solution")

    def check_combined_state(self, *, joint_positions_rad, group_name):
        del joint_positions_rad, group_name
        return CombinedStateValidity(True, "fixed offline valid")


class ObjectDragAndWaypointVisualizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")

    def test_center_6d_transform_controls_exist(self):
        self.assertIn("TransformControls", self.controller)
        initializer = self.controller.split(
            "function initializeObjectTransformControls", 1
        )[1].split("function applyObjectPreviewPose", 1)[0]
        self.assertIn('setMode("translate")', initializer)
        self.assertIn('setSpace("world")', initializer)
        self.assertIn("showX = true", initializer)
        self.assertIn("showY = true", initializer)
        self.assertIn("showZ = true", initializer)
        selection = self.controller.split(
            "function setRigidGraspSelectedFrame", 1
        )[1].split("function setRigidGraspGizmoMode", 1)[0]
        self.assertIn('setMode(rigidGraspState.gizmoMode)', selection)
        self.assertIn('? "world" : "local"', selection)
        self.assertIn('if (!["translate", "rotate"].includes(mode))', self.controller)
        self.assertNotIn('setMode("scale")', initializer)
        self.assertIn("Center orientation is editable after Lock Grasp", self.html)
        self.assertIn("digitalTwinCenterWaypointRotate", self.html)

    def test_drag_updates_canonical_6d_pose_and_rigid_grasp_frames(self):
        initializer = self.controller.split(
            "function initializeObjectTransformControls", 1
        )[1].split("function applyObjectPreviewPose", 1)[0]
        self.assertIn("applyObjectPreviewPose({", initializer)
        self.assertIn("rpyRad: [...centerPose.rpyRad]", initializer)
        canonical = self.controller.split("function applyObjectPreviewPose", 1)[1].split(
            "function objectPoseFromControls", 1
        )[0]
        self.assertIn(
            "computeWorldGraspFrameMatrices(normalizedPose, graspContent)",
            canonical,
        )
        self.assertIn("applyMatrixToFrame(objectFrame", canonical)
        self.assertIn("applyMatrixToFrame(leftGraspFrame", canonical)
        self.assertIn("applyMatrixToFrame(rightGraspFrame", canonical)
        self.assertIn("syncObjectGizmoToCanonicalPose(normalizedPose)", canonical)

    def test_drag_ownership_disables_camera_and_pauses_preview(self):
        initializer = self.controller.split(
            "function initializeObjectTransformControls", 1
        )[1].split("function applyObjectPreviewPose", 1)[0]
        self.assertIn('"dragging-changed"', initializer)
        self.assertIn("controls.enabled = !event.value", initializer)
        self.assertIn("pauseObjectTrajectoryForManualPreview()", initializer)
        self.assertIn('ownership = "MANUAL OBJECT DRAG"', initializer)

    def test_waypoint_markers_and_preplan_path_use_canonical_state(self):
        body = self.controller.split(
            "function syncObjectWaypointVisualization", 1
        )[1].split("function getObjectWaypointVisualizationState", 1)[0]
        self.assertIn("objectWaypointPlanningState.waypoints.map", body)
        self.assertIn("new THREE.SphereGeometry", body)
        self.assertIn("new THREE.Line", body)
        self.assertIn("CENTER WAYPOINT PATH — PRE-PLAN ONLY", self.html)
        self.assertNotIn("requestObjectGlobalPlan", body)
        self.assertNotIn("compute_ik", body)

    def test_every_waypoint_mutation_refreshes_the_same_visualization(self):
        updater = self.controller.split(
            "function updateObjectWaypointPlanningUi", 1
        )[1].split("function setObjectPlanningWaypoints", 1)[0]
        self.assertIn("syncObjectWaypointVisualization()", updater)
        setter = self.controller.split(
            "function setObjectPlanningWaypoints", 1
        )[1].split("function addObjectPlanningWaypointToState", 1)[0]
        self.assertIn("updateObjectWaypointPlanningUi()", setter)
        for operation in (
            "addObjectPlanningWaypointToState",
            "deleteObjectPlanningWaypointFromState",
            "moveObjectPlanningWaypointInState",
            "clearObjectPlanningWaypoints",
            "loadObjectPlanningDemoFixture",
        ):
            self.assertIn(operation, self.controller)

    def test_save_current_reads_canonical_pose_and_generates_unique_w_ids(self):
        body = self.controller.split(
            "function addCurrentObjectPlanningWaypoint", 1
        )[1].split("function deleteObjectPlanningWaypointFromState", 1)[0]
        self.assertIn("objectPreviewState.pose", body)
        self.assertIn("while (names.has(`w${sequence}`))", body)
        self.assertIn("translation_m: [...pose.translationM]", body)
        self.assertIn(">Save Current Center as Waypoint</button>", self.html)

    def test_public_inspection_state_is_defensive_and_html_ids_are_unique(self):
        for api in (
            "getObjectDragState",
            "setObjectDragEnabled",
            "getObjectWaypointVisualizationState",
        ):
            self.assertIn(api, self.controller)
        self.assertIn("translation_m: [...waypoint.translation_m]", self.controller)
        parser = IdParser()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))


class PlanningStartStateWebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.start_state_source = START_STATE_SOURCE_PATH.read_text(encoding="utf-8")
        cls.ownership = run_main_model_ownership_harness(cls.controller)

    def test_editable_start_state_panel_and_all_twelve_inputs_are_removed(self):
        self.assertNotIn("Planning Start State — OFFLINE", self.html)
        self.assertNotIn("digital-twin-planning-start-panel", self.html)
        for side in ("Left", "Right"):
            for index in range(1, 7):
                self.assertNotIn(f"digitalTwinPlanningStart{side}J{index}", self.html)

    def test_apply_profile_and_live_capture_controls_are_removed(self):
        for removed in (
            "digitalTwinPlanningStartApplyGhost",
            "digitalTwinPlanningStartProfileName",
            "digitalTwinPlanningStartProfileSelect",
            "digitalTwinPlanningStartProfileSave",
            "digitalTwinPlanningStartProfileLoad",
            "digitalTwinPlanningStartProfileDelete",
            "digitalTwinPlanningStartCaptureLive",
        ):
            self.assertNotIn(removed, self.html)
            self.assertNotIn(removed, self.controller)

    def test_start_profile_localstorage_state_and_apis_are_removed(self):
        for removed in (
            "PLANNING_START_PROFILE_STORAGE_KEY",
            "dualArmPlanningStartStateProfiles",
            "persistPlanningStartProfiles",
            "savePlanningStartProfile",
            "loadPlanningStartProfile",
            "deletePlanningStartProfile",
            "getPlanningStartProfileState",
        ):
            self.assertNotIn(removed, self.controller)

    def test_normal_web_global_request_omits_start_override(self):
        result = run_planning_module(r'''
import { buildObjectGlobalPlanRequest } from "./planning.mjs";
const request = buildObjectGlobalPlanRequest({
  name: "Start_Request",
  waypoints: [
    {identifier: "W0", translation_m: [0,0,0.8]},
    {identifier: "W1", translation_m: [0.1,0,0.8]},
  ],
  fixedOrientationRpyRad: [0,0,0], segmentDurationS: 1,
  samplesPerSegment: 2, candidateAttemptsPerArm: 3,
  expectedGraspContentRevision: "sha256:test-grasp",
  expectedLockGeneration: 1,
  expectedLockRevision: "sha256:test-lock",
  expectedCalibrationRevision: "sha256:test-calibration",
  expectedModelCalibrationRevision: "sha256:test-calibration",
});
console.log(JSON.stringify(request));
''')
        self.assertNotIn("initial_joint_state_rad", result)
        self.assertNotIn("planning_start_state_source", result)
        plan_body = self.controller.split("async function planObjectGlobal", 1)[1].split(
            "function bindObjectWaypointPlanningControls", 1
        )[0]
        self.assertIn("initialJointStateRad: nextPlanningStartOverride", plan_body)
        self.assertIn("planningStartStateSource: nextPlanningStartOverride", plan_body)
        self.assertIn("nextPlanningStartOverride = null;", plan_body)

    def test_read_only_loader_initializes_main_model_not_planned_ghost(self):
        body = self.controller.split("async function loadPlanningStartState", 1)[1].split(
            "function objectWaypointPlanningStateSnapshot", 1
        )[0]
        self.assertIn("requestPlanningStartState(fetchImpl)", body)
        self.assertIn(
            '"/api/digital-twin/planning-start-state"', self.start_state_source
        )
        self.assertIn('method: "GET"', self.start_state_source)
        self.assertNotIn('method: "POST"', self.start_state_source)
        self.assertIn("applyPlanningStartStateToMainWhenOffline", body)
        self.assertNotIn("setPlannedJointValues(", body)
        self.assertNotIn("ingestStatusSnapshot", body)
        self.assertIn("loadPlanningStartState();", self.controller)

    def test_offline_code_config_applies_exact_left_and_right_to_main(self):
        self.assertTrue(self.ownership["offlineApplied"])
        self.assertEqual(
            self.ownership["offlinePose"]["left"],
            list(PLANNING_START_STATE_RAD["left"]),
        )
        self.assertEqual(
            self.ownership["offlinePose"]["right"],
            list(PLANNING_START_STATE_RAD["right"]),
        )
        self.assertEqual(self.ownership["writesAfterOffline"], 12)
        self.assertEqual(
            self.ownership["offlineMirror"]["mainModelSource"],
            "OFFLINE CODE CONFIG",
        )

    def test_offline_config_does_not_touch_feedback_cache_or_planned_ghost(self):
        self.assertIsNone(self.ownership["cacheBeforeOffline"])
        planned = self.ownership["plannedAfterOffline"]
        self.assertIsNone(planned["latestPose"])
        self.assertFalse(planned["visible"])
        self.assertEqual(planned["source"], "NONE")

    def test_fresh_live_feedback_has_priority_and_blocks_code_config(self):
        expected_live = {
            "left": [0.11, 0.12, 0.13, 0.14, 0.15, 0.16],
            "right": [-0.21, -0.22, -0.23, -0.24, -0.25, -0.26],
        }
        self.assertEqual(self.ownership["liveMirror"]["mode"], "LIVE_MIRROR")
        self.assertEqual(
            self.ownership["liveMirror"]["mainModelSource"],
            "LIVE JOINT FEEDBACK",
        )
        self.assertEqual(self.ownership["livePose"], expected_live)
        self.assertEqual(
            {
                "left": self.ownership["cachedAfterLive"]["left"],
                "right": self.ownership["cachedAfterLive"]["right"],
            },
            expected_live,
        )
        self.assertFalse(self.ownership["configBlockedByLive"])
        self.assertEqual(self.ownership["poseAfterBlockedConfig"], expected_live)
        self.assertEqual(
            self.ownership["writesAfterBlockedConfig"],
            self.ownership["writesAfterLive"],
        )

    def test_stale_live_feedback_returns_main_ownership_to_code_config(self):
        self.assertEqual(self.ownership["staleMirror"]["mode"], "STALE")
        self.assertEqual(
            self.ownership["staleMirror"]["mainModelSource"],
            "OFFLINE CODE CONFIG",
        )
        self.assertEqual(
            self.ownership["staleFallbackPose"],
            {
                "left": list(PLANNING_START_STATE_RAD["left"]),
                "right": list(PLANNING_START_STATE_RAD["right"]),
            },
        )
        self.assertEqual(
            {
                "left": self.ownership["cachedAfterStale"]["left"],
                "right": self.ownership["cachedAfterStale"]["right"],
            },
            {
                "left": [0.11, 0.12, 0.13, 0.14, 0.15, 0.16],
                "right": [-0.21, -0.22, -0.23, -0.24, -0.25, -0.26],
            },
        )

    def test_global_plan_moves_only_planned_ghost_and_integrated_object(self):
        success = self.controller.split(
            "function loadObjectGlobalPlanPreview", 1
        )[1].split("async function planObjectGlobal", 1)[0]
        self.assertIn("globalPlanToPlannedTrajectory(plan)", success)
        self.assertIn("loadPlannedTrajectory(", success)
        self.assertNotIn("setJointValues(", success)
        preview = self.controller.split("function applyTrajectoryTime", 1)[1].split(
            "function loadPlannedTrajectory", 1
        )[0]
        self.assertIn("setPlannedJointValues(", preview)
        self.assertIn("applyIntegratedPlanTime(sample.time_from_start_s)", preview)
        self.assertNotIn("setJointValues(", preview)

    def test_backend_config_failure_isolated_and_read_only_state_is_public(self):
        body = self.controller.split("async function loadPlanningStartState", 1)[1].split(
            "function objectWaypointPlanningStateSnapshot", 1
        )[0]
        self.assertIn('planningStartState.status = "UNAVAILABLE"', body)
        self.assertIn("updateObjectGlobalPlanUi()", body)
        api = self.controller.split("const publicApi = {", 1)[1].split("};", 1)[0]
        self.assertIn("getPlanningStartState", api)
        for removed in ("setPlanningStartState", "applyPlanningStartStateToGhost"):
            self.assertNotIn(removed, api)

    def test_read_only_global_diagnostics_and_phase4_semantic_remain(self):
        self.assertIn("digitalTwinGlobalPlanPlanningStartSource", self.html)
        self.assertIn("digitalTwinGlobalPlanStartToFirstMax", self.html)
        self.assertIn("PHASE-4 START-TRANSITION VALIDATION NOT YET APPLIED", self.html)
        self.assertNotIn("SAFE TO EXECUTE", self.html)


class PlanningStartStateBackendTests(unittest.TestCase):
    limits = CanonicalJointPositionLimits((-6.28,) * 12, (6.28,) * 12)

    def test_code_config_has_exact_canonical_six_plus_six_radian_values(self):
        self.assertEqual(tuple(PLANNING_START_STATE_RAD), ("left", "right"))
        self.assertEqual(len(PLANNING_START_STATE_RAD["left"]), 6)
        self.assertEqual(len(PLANNING_START_STATE_RAD["right"]), 6)
        self.assertEqual(
            PLANNING_START_JOINT_NAMES,
            tuple(f"left_joint_{index}" for index in range(1, 7))
            + tuple(f"right_joint_{index}" for index in range(1, 7)),
        )
        self.assertEqual(PLANNING_START_STATE_UNIT, "radian")
        self.assertEqual(
            PLANNING_START_STATE_SOURCE,
            "CODE CONFIG — planning_start_state_config.py",
        )

    def test_config_validation_rejects_bool_nan_inf_and_bad_lengths(self):
        for mutation in ("left_short", "bool", "nan", "inf"):
            value = {"left": [0.0] * 6, "right": [0.0] * 6}
            if mutation == "left_short":
                value["left"] = [0.0] * 5
            elif mutation == "bool":
                value["right"][0] = True
            elif mutation == "nan":
                value["right"][0] = math.nan
            else:
                value["right"][0] = math.inf
            with self.subTest(mutation=mutation), self.assertRaises(
                PlanningStartStateConfigError
            ):
                normalize_planning_start_state_rad(value)

    def test_optional_override_and_canonical_limit_validation_remain_compatible(self):
        payload = planning_payload()
        payload["initial_joint_state_rad"]["left"][2] = 7.0
        with self.assertRaisesRegex(ObjectGlobalPlanInputError, "left_joint_3"):
            run_test_plan(payload, FixedPlanningAdapter(), self.limits)

    def test_default_code_config_is_layer_zero_seed_and_q0_stays_distinct(self):
        payload = planning_payload()
        payload.pop("initial_joint_state_rad")
        payload.pop("planning_start_state_source")
        adapter = FixedPlanningAdapter()
        result = run_test_plan(payload, adapter, self.limits)
        combined = normalize_planning_start_state_rad(PLANNING_START_STATE_RAD)
        self.assertEqual(adapter.seeds[0], ("left_arm", combined))
        self.assertEqual(adapter.seeds[1], ("right_arm", combined))
        self.assertEqual(result["planning_start_state_source"], PLANNING_START_STATE_SOURCE)
        self.assertNotEqual(
            result["first_selected_joint_state_rad"],
            result["planning_start_state_rad"],
        )
        self.assertEqual(len(result["start_to_first_raw_joint_delta_rad"]), 12)
        self.assertIn("joint_name", result["start_to_first_maximum_raw_joint_delta"])

    def test_read_only_payload_and_get_route_have_no_side_effects(self):
        payload = planning_start_state_payload(self.limits)
        self.assertTrue(payload["ok"])
        self.assertEqual(payload["joint_names"], list(PLANNING_START_JOINT_NAMES))
        self.assertEqual(len(payload["left"]), 6)
        self.assertEqual(len(payload["right"]), 6)
        backend = BACKEND_PATH.read_text(encoding="utf-8")
        route = backend.split(
            'def api_digital_twin_planning_start_state():', 1
        )[1].split("@app.post", 1)[0]
        self.assertIn("node.digital_twin_planning_start_state()", route)
        for forbidden in ("joint_move", "linear_move", "jog", "servo", "publish"):
            self.assertNotIn(forbidden, route)

    def test_graph_semantics_budgets_and_dp_core_remain_unchanged(self):
        graph_source = GRAPH_PATH.read_text(encoding="utf-8")
        builder = graph_source.split(
            "def build_object_trajectory_candidate_graph", 1
        )[1].split("def default_graph_edge_feasibility", 1)[0]
        self.assertIn("for node in layers[-1].nodes", builder)
        self.assertIn("for parent_node_id, base_seed in parent_seeds", builder)
        self.assertNotIn("top_k", builder.lower())
        self.assertEqual(DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.max_attempts_per_arm, 13)
        planner = PLANNER_PATH.read_text(encoding="utf-8")
        self.assertIn("max_attempts_per_arm=3", planner)
        self.assertIn("candidate_pruning_applied", planner)

    def test_new_planning_code_has_no_robot_motion_dependencies(self):
        sources = (
            PLANNER_PATH.read_text(encoding="utf-8"),
            PLANNING_PATH.read_text(encoding="utf-8"),
            CONFIG_PATH.read_text(encoding="utf-8"),
            START_STATE_SOURCE_PATH.read_text(encoding="utf-8"),
        )
        forbidden = (
            "/api/jog", "/api/home", "/api/direct/joint_move",
            "/api/direct/tcp_move", "/api/waypoint/run", "/api/program/run",
            "/api/sequence/run", "joint_move(", "linear_move(", "servo(",
        )
        for source in sources:
            for token in forbidden:
                with self.subTest(token=token):
                    self.assertNotIn(token, source)


if __name__ == "__main__":
    unittest.main()
