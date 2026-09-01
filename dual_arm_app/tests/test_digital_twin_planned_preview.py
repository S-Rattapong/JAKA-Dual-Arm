"""Offline tests for Phase 1D.1 Planned Ghost Pose Preview."""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = REPOSITORY_ROOT / "dual_arm_app"
PLANNED_HELPER_PATH = APP_ROOT / "web/digital_twin_planned_preview.js"
STATUS_ADAPTER_PATH = APP_ROOT / "web/digital_twin_status_adapter.js"
SMOOTHING_PATH = APP_ROOT / "web/digital_twin_live_smoothing.js"
DIGITAL_TWIN_PATH = APP_ROOT / "web/digital_twin.js"
HTML_PATH = APP_ROOT / "web/index.html"
URDF_PATH = APP_ROOT / "web/assets/dual_jaka_a12_web.urdf"


def _run_node_harness(files: dict[str, str], harness_source: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1d1-") as temporary_directory:
        directory = Path(temporary_directory)
        for name, source in files.items():
            (directory / name).write_text(source, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness_source, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            check=True,
            capture_output=True,
            text=True,
        )
    return json.loads(result.stdout)


class PlannedPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.helper = PLANNED_HELPER_PATH.read_text(encoding="utf-8")
        cls.adapter = STATUS_ADAPTER_PATH.read_text(encoding="utf-8")
        cls.digital_twin = DIGITAL_TWIN_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_pure_module_contract_validation_copy_and_delta(self) -> None:
        self.assertTrue(PLANNED_HELPER_PATH.is_file())
        for forbidden in ("fetch", "window", "document", "ROS", "/api/", "backend"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.helper)

        harness = r'''
import {
  flattenDualArmPose,
  maxAbsoluteJointDelta,
  normalizePlannedDualArmPose,
  plannedMinusActualJointDelta,
} from "./planned.mjs";

function rejects(callback) {
  try { callback(); return false; } catch (_error) { return true; }
}

const source = {
  left: [0, 1, 2, 3, 4, 5],
  right: [0, -1, -2, -3, -4, -5],
};
const before = JSON.stringify(source);
const normalized = normalizePlannedDualArmPose(source);
normalized.left[0] = 99;
const planned = { left: [1,2,3,4,5,6], right: [-1,-2,-3,-4,-5,-6] };
const actual = { left: [0,1,1,2,2,3], right: [0,-1,-1,-2,-2,-3] };
const delta = plannedMinusActualJointDelta(planned, actual);

console.log(JSON.stringify({
  sourceUnchanged: JSON.stringify(source) === before,
  arraysCopied: normalized.left !== source.left && normalized.right !== source.right,
  flattened: flattenDualArmPose(planned),
  delta,
  maximum: maxAbsoluteJointDelta(planned, actual),
  rejectsNull: rejects(() => normalizePlannedDualArmPose(null)),
  rejectsMissing: rejects(() => normalizePlannedDualArmPose({ left: [0,0,0,0,0,0] })),
  rejectsShort: rejects(() => normalizePlannedDualArmPose({ left: [0,0], right: [0,0,0,0,0,0] })),
  rejectsLong: rejects(() => normalizePlannedDualArmPose({ left: [0,0,0,0,0,0,0], right: [0,0,0,0,0,0] })),
  rejectsBoolean: rejects(() => normalizePlannedDualArmPose({ left: [0,0,0,0,0,true], right: [0,0,0,0,0,0] })),
  rejectsString: rejects(() => normalizePlannedDualArmPose({ left: [0,0,0,0,0,"0"], right: [0,0,0,0,0,0] })),
  rejectsNaN: rejects(() => normalizePlannedDualArmPose({ left: [0,0,0,0,0,NaN], right: [0,0,0,0,0,0] })),
  rejectsInfinity: rejects(() => normalizePlannedDualArmPose({ left: [0,0,0,0,0,Infinity], right: [0,0,0,0,0,0] })),
}));
'''
        result = _run_node_harness(
            {"planned.mjs": self.helper},
            harness,
        )
        self.assertTrue(result["sourceUnchanged"])
        self.assertTrue(result["arraysCopied"])
        self.assertEqual(len(result["flattened"]), 12)
        self.assertEqual(result["delta"]["left"], [1, 1, 2, 2, 3, 3])
        self.assertEqual(result["delta"]["right"], [-1, -1, -2, -2, -3, -3])
        self.assertEqual(result["maximum"], 3)
        for key, value in result.items():
            if key.startswith("rejects"):
                with self.subTest(assertion=key):
                    self.assertTrue(value)

    def test_actual_and_planned_controller_behavior_are_independent(self) -> None:
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
import {{
  maxAbsoluteJointDelta,
  normalizePlannedDualArmPose,
}} from "./planned.mjs";
globalThis.document = {{ getElementById() {{ return null; }} }};
{controller_source}

function modelWithRecorder(recorder) {{
  const model = {{ joints: {{}}, visible: true, updateMatrixWorld() {{}} }};
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

const distinctModels = robot !== plannedRobot;
const initial = getPlannedPreviewState();
const unavailableCapture = captureActualAsPlanned();
const actualZero = {{ left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] }};
setJointValues(actualZero);
const actualAfterSet = actualCalls.length;
const plannedPose = {{
  left: [0.30,-0.40,0.20,0.25,-0.15,0.12],
  right: [-0.30,0.40,-0.20,-0.25,0.15,-0.12],
}};
const visible = setPlannedJointValues(plannedPose, "TEST PLAN");
const plannedAfterSet = plannedCalls.length;
const actualAfterPlannedSet = actualCalls.length;
const invalid = setPlannedJointValues({{ left: [0], right: [0,0,0,0,0,0] }}, "BAD");
const plannedAfterInvalid = plannedCalls.length;
const hidden = hidePlannedModel();
const hiddenPose = hidden.latestPose;
const shown = showPlannedModel();
const actualBeforeClear = actualCalls.length;
const cleared = clearPlannedPreview();
const actualAfterClear = actualCalls.length;

setPlannedJointValues(plannedPose, "RESET INDEPENDENCE");
const plannedBeforeReset = getPlannedPreviewState();
resetToStaticPose();
const plannedAfterReset = getPlannedPreviewState();

const capturedActualInput = {{
  left: [0.1,0.2,0.3,0.4,0.5,0.6],
  right: [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6],
}};
setJointValues(capturedActualInput);
capturedActualInput.left[0] = 99;
const captured = captureActualAsPlanned();
captured.latestPose.left[0] = 88;
const capturedInternal = getPlannedPreviewState();

console.log(JSON.stringify({{
  distinctModels,
  initial,
  unavailableCapture,
  actualAfterSet,
  visible,
  plannedAfterSet,
  actualAfterPlannedSet,
  invalid,
  plannedAfterInvalid,
  hidden,
  hiddenPose,
  shown,
  actualBeforeClear,
  actualAfterClear,
  cleared,
  plannedBeforeReset,
  plannedAfterReset,
  captured,
  capturedInternal,
  finalActualCalls: actualCalls.length,
  finalPlannedCalls: plannedCalls.length,
}}));
'''
        result = _run_node_harness(
            {
                "adapter.mjs": self.adapter,
                "smoothing.mjs": SMOOTHING_PATH.read_text(encoding="utf-8"),
                "planned.mjs": self.helper,
            },
            harness,
        )
        self.assertTrue(result["distinctModels"])
        self.assertEqual(result["initial"]["status"], "READY")
        self.assertFalse(result["initial"]["visible"])
        self.assertEqual(result["unavailableCapture"]["status"], "INVALID")
        self.assertEqual(result["actualAfterSet"], 12)
        self.assertEqual(result["plannedAfterSet"], 12)
        self.assertEqual(result["actualAfterPlannedSet"], 12)
        self.assertEqual(result["visible"]["status"], "VISIBLE")
        self.assertTrue(result["visible"]["visible"])
        self.assertEqual(result["visible"]["source"], "TEST PLAN")
        self.assertTrue(math.isclose(result["visible"]["maxJointDeltaRad"], 0.4))
        self.assertEqual(result["invalid"]["status"], "INVALID")
        self.assertEqual(result["invalid"]["source"], "TEST PLAN")
        self.assertEqual(result["invalid"]["latestPose"], result["visible"]["latestPose"])
        self.assertEqual(result["plannedAfterInvalid"], 12)
        self.assertEqual(result["hidden"]["status"], "HIDDEN")
        self.assertFalse(result["hidden"]["visible"])
        self.assertEqual(result["hiddenPose"], result["visible"]["latestPose"])
        self.assertEqual(result["shown"]["status"], "VISIBLE")
        self.assertEqual(result["actualBeforeClear"], result["actualAfterClear"])
        self.assertIsNone(result["cleared"]["latestPose"])
        self.assertEqual(
            result["plannedAfterReset"]["latestPose"],
            result["plannedBeforeReset"]["latestPose"],
        )
        self.assertEqual(result["captured"]["source"], "ACTUAL POSE SNAPSHOT")
        self.assertEqual(result["captured"]["maxJointDeltaRad"], 0)
        self.assertEqual(result["capturedInternal"]["latestPose"]["left"][0], 0.1)

    def test_independent_loading_materials_and_failure_isolation(self) -> None:
        self.assertIn("let robot = null", self.digital_twin)
        self.assertIn("let plannedRobot = null", self.digital_twin)
        self.assertNotIn("plannedRobot = robot.clone", self.digital_twin)
        self.assertEqual(self.digital_twin.count("new URDFLoader(lifecycle.manager)"), 2)
        self.assertEqual(self.digital_twin.count("createUrdfLoadLifecycle({"), 3)
        self.assertGreaterEqual(self.digital_twin.count("loader.load(\n    MODEL_URL"), 2)
        self.assertEqual(self.digital_twin.count("scene.add(robot)"), 1)
        self.assertEqual(self.digital_twin.count("scene.add(plannedRobot)"), 1)
        self.assertIn("inspectUrdfModelReadiness(state.model, role)", self.digital_twin)
        self.assertIn('role: "Actual"', self.digital_twin)
        self.assertIn('role: "Planned"', self.digital_twin)
        self.assertIn("object.material.map(cloneMaterial)", self.digital_twin)
        self.assertIn("const ghostMaterial = material.clone()", self.digital_twin)
        self.assertIn("ghostMaterial.transparent = true", self.digital_twin)
        self.assertIn("ghostMaterial.opacity = PLANNED_GHOST_OPACITY", self.digital_twin)
        self.assertIn("ghostMaterial.depthWrite = false", self.digital_twin)
        self.assertIn("plannedRobot.visible = false", self.digital_twin)
        planned_loader = self.digital_twin.split(
            "function loadPlannedRobot()", 1
        )[1].split("function initialize()", 1)[0]
        planned_ready_callback = planned_loader.split(
            "onValidated: (_model, validation) => {", 1
        )[1].split("onFailure:", 1)[0]
        self.assertIn("makePlannedMaterialsTransparent(plannedRobot)", planned_ready_callback)
        self.assertLess(
            planned_ready_callback.index("makePlannedMaterialsTransparent(plannedRobot)"),
            planned_ready_callback.index("plannedLoadState.status = \"READY\""),
        )
        planned_urdf_callback = planned_loader.split(
            "(loadedRobot) => {", 1
        )[1].split("undefined,", 1)[0]
        self.assertNotIn("makePlannedMaterialsTransparent", planned_urdf_callback)
        self.assertIn("applyJointValuesToModel(robot, values)", self.digital_twin)
        self.assertIn(
            "applyJointValuesToModel(plannedRobot, plannedPreviewState.latestPose)",
            self.digital_twin,
        )
        fit_model = self.digital_twin.split("function fitModel", 1)[1].split(
            "function resetCamera", 1
        )[0]
        self.assertIn("setFromObject(robot)", fit_model)
        self.assertNotIn("plannedRobot", fit_model)
        planned_failure = self.digital_twin.split(
            "function failPlannedPreview", 1
        )[1].split("function loadRobot", 1)[0]
        self.assertNotIn("fail(", planned_failure)
        self.assertNotIn("loadState.status", planned_failure)

        lifecycle = self.digital_twin.split(
            "function createUrdfLoadLifecycle", 1
        )[1].split("function loadRobot()", 1)[0]
        for field in (
            "urdfParsed: false",
            "assetsLoaded: false",
            "finalized: false",
            "failedAssets: new Set()",
            "loadTimeout: null",
            "readinessTimer: null",
            "manager: new THREE.LoadingManager()",
            "model: null",
        ):
            with self.subTest(field=field):
                self.assertIn(field, lifecycle)

    def test_shared_lifecycle_gates_actual_and_planned_completion(self) -> None:
        lifecycle_source = "function createUrdfLoadLifecycle" + self.digital_twin.split(
            "function createUrdfLoadLifecycle", 1
        )[1].split("function loadRobot()", 1)[0]
        harness = r'''
const LOAD_TIMEOUT_MS = 20000;
const MODEL_READINESS_RETRY_MS = 40;
let nextTimerId = 1;
const timers = new Map();
const inspectionEvents = [];
const readyRoles = [];
const failures = [];
const THREE = {
  LoadingManager: class {},
};
const window = {
  setTimeout(callback, delay) {
    const id = nextTimerId++;
    timers.set(id, { callback, delay });
    return id;
  },
  clearTimeout(id) { timers.delete(id); },
};
console.info = () => {};
console.debug = () => {};
function inspectUrdfModelReadiness(model, role) {
  inspectionEvents.push(`${role}:${model.visuals}`);
  return {
    ready: model.role === role && model.visuals === 14 && !model.boundsEmpty,
    role,
    expectedJointCount: 12,
    foundJointCount: 12,
    missingJoints: [],
    expectedVisualCount: 14,
    foundVisualCount: model.visuals,
    boundsEmpty: model.boundsEmpty,
  };
}
function formatUrdfValidationError(diagnostic) {
  return `${diagnostic.role} URDF validation failed: expected 12 joints and 14 visual meshes, `
    + `found ${diagnostic.foundJointCount} joints and ${diagnostic.foundVisualCount} visual meshes; `
    + `missing joints: none; bounds empty: ${diagnostic.boundsEmpty}`;
}
function runTimers(delay) {
  const pending = [...timers.entries()].filter(([_id, timer]) => timer.delay === delay);
  pending.forEach(([id, timer]) => {
    timers.delete(id);
    timer.callback();
  });
}
function model(role, visuals = 0, boundsEmpty = true) {
  return { role, visuals, boundsEmpty, updateMatrixWorld() {} };
}
'''
        harness += lifecycle_source
        harness += r'''
function makeLifecycle(role) {
  return createUrdfLoadLifecycle({
    role,
    onValidated() { readyRoles.push(role); },
    onFailure(message) { failures.push({ role, message }); },
  });
}

const actual = makeLifecycle("Actual");
const planned = makeLifecycle("Planned");
const distinctManagers = actual.manager !== planned.manager;

actual.manager.onLoad();
planned.manager.onLoad();
const afterPreParse = { inspections: inspectionEvents.length, ready: readyRoles.length };

const actualModel = model("Actual");
const plannedModel = model("Planned");
actual.markParsed(actualModel);
planned.markParsed(plannedModel);
const afterParsed = { inspections: inspectionEvents.length, ready: readyRoles.length };

// Exercise the lost-event guard: invalidate Actual's first deferred completion
// with new manager activity, then accept the following stable completion.
actual.manager.onLoad();
actual.manager.onStart();
runTimers(0);
const afterInvalidatedActualCompletion = {
  inspections: inspectionEvents.length,
  ready: readyRoles.length,
};
actual.manager.onLoad();
planned.manager.onLoad();
const beforeDeferredCompletion = {
  inspections: inspectionEvents.length,
  ready: readyRoles.length,
};
runTimers(0);
const afterResourceCompletion = {
  inspections: [...inspectionEvents],
  ready: [...readyRoles],
  failures: [...failures],
  actualReady: actual.state.lastDiagnostic.ready,
  plannedReady: planned.state.lastDiagnostic.ready,
};

actualModel.visuals = 14;
actualModel.boundsEmpty = false;
plannedModel.visuals = 14;
plannedModel.boundsEmpty = false;
runTimers(MODEL_READINESS_RETRY_MS);

const mainInspectionEvents = [...inspectionEvents];
const mainReadyRoles = [...readyRoles];
const mainFailures = [...failures];
inspectionEvents.length = 0;
readyRoles.length = 0;
failures.length = 0;

const failedActual = makeLifecycle("Actual");
const successfulPlanned = makeLifecycle("Planned");
failedActual.markParsed(model("Actual"));
successfulPlanned.markParsed(model("Planned", 14, false));
failedActual.manager.onLoad();
successfulPlanned.manager.onLoad();
runTimers(0);
runTimers(LOAD_TIMEOUT_MS);
const failureIsolation = {
  inspections: [...inspectionEvents],
  ready: [...readyRoles],
  failures: [...failures],
};
inspectionEvents.length = 0;
readyRoles.length = 0;
failures.length = 0;

const successfulActual = makeLifecycle("Actual");
const failedPlanned = makeLifecycle("Planned");
successfulActual.markParsed(model("Actual", 14, false));
failedPlanned.markParsed(model("Planned"));
successfulActual.manager.onLoad();
failedPlanned.manager.onLoad();
runTimers(0);
runTimers(LOAD_TIMEOUT_MS);
const reverseFailureIsolation = {
  inspections: [...inspectionEvents],
  ready: [...readyRoles],
  failures: [...failures],
};

console.log(JSON.stringify({
  distinctManagers,
  afterPreParse,
  afterParsed,
  afterInvalidatedActualCompletion,
  beforeDeferredCompletion,
  afterResourceCompletion,
  inspectionEvents: mainInspectionEvents,
  readyRoles: mainReadyRoles,
  failures: mainFailures,
  failureIsolation,
  reverseFailureIsolation,
  actualState: actual.state,
  plannedState: planned.state,
}));
'''
        result = _run_node_harness({}, harness)
        self.assertTrue(result["distinctManagers"])
        for checkpoint in (
            "afterPreParse",
            "afterParsed",
            "afterInvalidatedActualCompletion",
            "beforeDeferredCompletion",
        ):
            with self.subTest(checkpoint=checkpoint):
                self.assertEqual(result[checkpoint], {"inspections": 0, "ready": 0})
        self.assertEqual(result["afterResourceCompletion"], {
            "inspections": ["Actual:0", "Planned:0"],
            "ready": [],
            "failures": [],
            "actualReady": False,
            "plannedReady": False,
        })
        self.assertEqual(
            result["inspectionEvents"],
            ["Actual:0", "Planned:0", "Actual:14", "Planned:14"],
        )
        self.assertEqual(result["readyRoles"], ["Actual", "Planned"])
        self.assertEqual(result["failures"], [])

        failure_isolation = result["failureIsolation"]
        self.assertEqual(failure_isolation["inspections"], ["Actual:0", "Planned:14"])
        self.assertEqual(failure_isolation["ready"], ["Planned"])
        self.assertEqual([item["role"] for item in failure_isolation["failures"]], ["Actual"])
        self.assertIn(
            "found 12 joints and 0 visual meshes",
            failure_isolation["failures"][0]["message"],
        )
        self.assertIn("bounds empty: true", failure_isolation["failures"][0]["message"])

        reverse_isolation = result["reverseFailureIsolation"]
        self.assertEqual(reverse_isolation["inspections"], ["Actual:14", "Planned:0"])
        self.assertEqual(reverse_isolation["ready"], ["Actual"])
        self.assertEqual([item["role"] for item in reverse_isolation["failures"]], ["Planned"])
        self.assertIn(
            "found 12 joints and 0 visual meshes",
            reverse_isolation["failures"][0]["message"],
        )
        self.assertIn("bounds empty: true", reverse_isolation["failures"][0]["message"])
        for state_name in ("actualState", "plannedState"):
            with self.subTest(state=state_name):
                state = result[state_name]
                self.assertTrue(state["urdfParsed"])
                self.assertTrue(state["assetsLoaded"])
                self.assertTrue(state["finalized"])

    def test_role_validator_counts_each_model_root_once_with_diagnostics(self) -> None:
        helper_source = self.digital_twin.split(
            "function countVisualMeshes", 1
        )[1].split("function makePlannedMaterialsTransparent", 1)[0]
        harness = f'''
const EXPECTED_VISUAL_COUNT = 14;
const EXPECTED_JOINTS = Array.from({{ length: 12 }}, (_value, index) => `joint_${{index + 1}}`);
const THREE = {{
  Box3: class {{
    setFromObject(model) {{ this.model = model; return this; }}
    isEmpty() {{ return Boolean(this.model.boundsEmpty); }}
  }},
}};
function countVisualMeshes{helper_source}

function makeRoot(meshCount, missingJoint = null, duplicateVisit = false) {{
  const meshes = Array.from({{ length: meshCount }}, () => ({{
    isMesh: true,
    geometry: {{ attributes: {{ position: {{ count: 3 }} }} }},
  }}));
  const joints = Object.fromEntries(EXPECTED_JOINTS.map(name => [name, {{}}]));
  if (missingJoint) delete joints[missingJoint];
  return {{
    joints,
    boundsEmpty: false,
    updateMatrixWorld() {{}},
    traverse(callback) {{
      meshes.forEach(callback);
      if (duplicateVisit && meshes[0]) callback(meshes[0]);
    }},
  }};
}}

const actualRoot = makeRoot(14, null, true);
const plannedRoot = makeRoot(14);
const transientRoot = makeRoot(0);
transientRoot.boundsEmpty = true;
const badRoot = makeRoot(15, "joint_12");
const actual = inspectUrdfModelReadiness(actualRoot, "Actual");
const planned = inspectUrdfModelReadiness(plannedRoot, "Planned");
const transient = inspectUrdfModelReadiness(transientRoot, "Planned");
const bad = inspectUrdfModelReadiness(badRoot, "Actual");
console.log(JSON.stringify({{
  actual,
  planned,
  transient,
  bad,
  badMessage: formatUrdfValidationError(bad),
  sceneWideCount: countVisualMeshes({{
    traverse(callback) {{ actualRoot.traverse(callback); plannedRoot.traverse(callback); }},
  }}),
}}));
'''
        result = _run_node_harness({}, harness)
        self.assertTrue(result["actual"]["ready"])
        self.assertEqual(result["actual"]["foundJointCount"], 12)
        self.assertEqual(result["actual"]["foundVisualCount"], 14)
        self.assertTrue(result["planned"]["ready"])
        self.assertEqual(result["planned"]["foundVisualCount"], 14)
        self.assertFalse(result["transient"]["ready"])
        self.assertEqual(result["transient"]["foundJointCount"], 12)
        self.assertEqual(result["transient"]["foundVisualCount"], 0)
        self.assertTrue(result["transient"]["boundsEmpty"])
        self.assertFalse(result["bad"]["ready"])
        self.assertEqual(result["bad"]["foundJointCount"], 11)
        self.assertEqual(result["bad"]["foundVisualCount"], 15)
        self.assertIn("expected 12 joints and 14 visual meshes", result["badMessage"])
        self.assertIn("found 11 joints and 15 visual meshes", result["badMessage"])
        self.assertEqual(result["sceneWideCount"], 28)

    def test_web_urdf_defines_expected_actual_and_planned_content(self) -> None:
        root = ET.parse(URDF_PATH).getroot()
        joint_names = {joint.attrib.get("name") for joint in root.findall("joint")}
        expected = {
            f"{side}_joint_{index}"
            for side in ("left", "right")
            for index in range(1, 7)
        }
        self.assertEqual(len(expected & joint_names), 12)
        self.assertEqual(len(root.findall(".//visual")), 14)

    def test_public_api_reset_independence_and_ui_contract(self) -> None:
        api_block = self.digital_twin.split("const publicApi = {", 1)[1].split(
            "};", 1
        )[0]
        for method in (
            "setPlannedJointValues",
            "showPlannedModel",
            "hidePlannedModel",
            "clearPlannedPreview",
            "getPlannedPreviewState",
            "captureActualAsPlanned",
        ):
            with self.subTest(method=method):
                self.assertIn(method, api_block)
        reset_block = self.digital_twin.split("function resetToStaticPose", 1)[1].split(
            "const publicApi", 1
        )[0]
        self.assertNotIn("clearPlannedPreview", reset_block)
        self.assertNotIn("plannedPreviewState.latestPose = null", reset_block)

        for label in (
            "Planned Pose Preview — OFFLINE ONLY",
            "Planned State",
            "Planned Visible",
            "Planned Source",
            "Max Joint Delta",
            "Unit:",
            "radians",
            "Planned Error",
            "Capture Actual as Planned",
            "Load Planned Mock A",
            "Load Planned Mock B",
            "Show Planned Ghost",
            "Hide Planned Ghost",
            "Clear Planned Preview",
            "PREVIEW DOES NOT EXECUTE ROBOT MOTION",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html)
        panel = self.html.split('aria-labelledby="digitalTwinTitle"', 1)[1].split(
            "</section>", 1
        )[0]
        self.assertNotIn(">Execute<", panel)
        for button_id in (
            "digitalTwinCaptureActualAsPlanned",
            "digitalTwinPlannedMockA",
            "digitalTwinPlannedMockB",
            "digitalTwinShowPlanned",
            "digitalTwinHidePlanned",
            "digitalTwinClearPlanned",
        ):
            self.assertNotIn(f'onclick=', panel.split(f'id="{button_id}"', 1)[1].split(">", 1)[0])

    def test_user_data_files_are_unchanged(self) -> None:
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
