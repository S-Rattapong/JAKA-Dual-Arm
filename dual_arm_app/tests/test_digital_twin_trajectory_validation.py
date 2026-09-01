"""Offline tests for Phase 1D.3A trajectory position-limit validation."""

from __future__ import annotations

import hashlib
import json
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from dual_arm_app.tools import build_dual_arm_validation_metadata as metadata_builder


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
WEB_ROOT = REPOSITORY_ROOT / "dual_arm_app" / "web"
METADATA_PATH = WEB_ROOT / "assets" / "dual_jaka_a12_joint_limits.json"
GENERATED_MODULE_PATH = WEB_ROOT / "digital_twin_joint_limit_metadata.js"
VALIDATOR_PATH = WEB_ROOT / "digital_twin_trajectory_validation.js"
TRAJECTORY_PATH = WEB_ROOT / "digital_twin_trajectory_preview.js"
PLANNED_PATH = WEB_ROOT / "digital_twin_planned_preview.js"
ADAPTER_PATH = WEB_ROOT / "digital_twin_status_adapter.js"
SMOOTHING_PATH = WEB_ROOT / "digital_twin_live_smoothing.js"
DIGITAL_TWIN_PATH = WEB_ROOT / "digital_twin.js"
HTML_PATH = WEB_ROOT / "index.html"
PROTECTED_UI_PROMPT = REPOSITORY_ROOT / "real_robot_configs" / "ui fix.md"
PROTECTED_UI_PROMPT_SHA256 = (
    "b1508c882d4f7bc8dab36fed1adac62a69086ee980f7702164b6b4d401cf18e1"
)
EXPECTED_JOINTS = [
    f"{side}_joint_{index}"
    for side in ("left", "right")
    for index in range(1, 7)
]


def _run_node(files: dict[str, str], harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1d3a-") as temporary_directory:
        directory = Path(temporary_directory)
        for name, source in files.items():
            (directory / name).write_text(source, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        try:
            result = subprocess.run(
                ["node", str(directory / "harness.mjs")],
                check=True,
                capture_output=True,
                text=True,
            )
        except subprocess.CalledProcessError as exc:
            raise AssertionError(exc.stderr or exc.stdout) from exc
    return json.loads(result.stdout)


class TrajectoryValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.metadata = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
        cls.generated_module = GENERATED_MODULE_PATH.read_text(encoding="utf-8")
        cls.validator = VALIDATOR_PATH.read_text(encoding="utf-8")
        cls.trajectory = TRAJECTORY_PATH.read_text(encoding="utf-8")
        cls.planned = PLANNED_PATH.read_text(encoding="utf-8")
        cls.adapter = ADAPTER_PATH.read_text(encoding="utf-8")
        cls.digital_twin = DIGITAL_TWIN_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_generated_metadata_matches_expanded_combined_robot_model(self) -> None:
        self.assertTrue(
            (REPOSITORY_ROOT / "dual_arm_app/tools/build_dual_arm_validation_metadata.py").is_file()
        )
        expanded = metadata_builder.expand_xacro(
            REPOSITORY_ROOT / metadata_builder.XACRO_RELATIVE_PATH
        )
        extracted = metadata_builder.extract_metadata(expanded)
        self.assertEqual(self.metadata, extracted)
        self.assertEqual(self.metadata["model"], "dual_jaka_a12")
        self.assertEqual(self.metadata["unit"], "radian")
        self.assertEqual(self.metadata["joint_order"], EXPECTED_JOINTS)
        self.assertEqual(len(self.metadata["joint_order"]), 12)
        self.assertEqual(self.metadata["model_structure"], {
            "link_count": 15,
            "total_joint_count": 14,
            "movable_joint_count": 12,
        })
        for joint_name in EXPECTED_JOINTS:
            limits = self.metadata["position_limits"][joint_name]
            with self.subTest(joint=joint_name):
                self.assertIsInstance(limits["min"], float)
                self.assertIsInstance(limits["max"], float)
                self.assertLess(limits["min"], limits["max"])
                self.assertEqual(limits, {"min": -6.28, "max": 6.28})

        velocity = self.metadata["urdf_velocity_metadata"]
        self.assertEqual(velocity["validation_status"], "NOT_EVALUATED")
        self.assertEqual(velocity["limits"]["left_joint_1"], 2.62)
        self.assertEqual(velocity["limits"]["right_joint_6"], 3.67)
        yaml_text = (
            REPOSITORY_ROOT
            / "src/jaka_ros2/src/jaka_a12_moveit_config/config/joint_limits.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("max_velocity: 1.57", yaml_text)
        self.assertIn("GENERATED FILE", self.generated_module)
        self.assertIn(self.metadata["source_model"], self.generated_module)
        controller_without_imports = self.digital_twin.split("const MODEL_URL", 1)[1]
        self.assertNotIn("-6.28", controller_without_imports)
        self.assertNotIn("6.28", controller_without_imports)

    def test_extractor_rejects_invalid_model_joint_and_limit_shapes(self) -> None:
        valid_xml = metadata_builder.expand_xacro(
            REPOSITORY_ROOT / metadata_builder.XACRO_RELATIVE_PATH
        )

        def changed_xml(mutator) -> str:
            root = ET.fromstring(valid_xml)
            mutator(root)
            return ET.tostring(root, encoding="unicode")

        cases = {
            "model": changed_xml(lambda root: root.set("name", "unexpected")),
            "missing": changed_xml(
                lambda root: root.remove(root.find("joint[@name='left_joint_1']"))
            ),
            "no-limit": changed_xml(
                lambda root: root.find("joint[@name='left_joint_1']").remove(
                    root.find("joint[@name='left_joint_1']/limit")
                )
            ),
            "non-finite": changed_xml(
                lambda root: root.find("joint[@name='left_joint_1']/limit").set(
                    "lower", "nan"
                )
            ),
            "reversed": changed_xml(
                lambda root: root.find("joint[@name='left_joint_1']/limit").set(
                    "lower", "7"
                )
            ),
            "duplicate": changed_xml(
                lambda root: root.find("joint[@name='right_joint_6']").set(
                    "name", "left_joint_1"
                )
            ),
        }
        for case_name, xml_text in cases.items():
            with self.subTest(case=case_name):
                with self.assertRaises(RuntimeError):
                    metadata_builder.extract_metadata(xml_text)

    def test_pure_validator_boundaries_diagnostics_and_copy_semantics(self) -> None:
        for forbidden in (
            "fetch",
            "window",
            "document",
            "Three",
            "ROS",
            "backend",
            "/api/",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.validator)

        harness = r'''
import {
  validateTrajectoryJointLimits,
  validateTrajectoryPointJointLimits,
} from "./validator.mjs";
const metadata = JSON.parse(globalThis.metadataJson);
const point = (time, left, right) => ({ time_from_start_s: time, left, right });
const fill = (value) => [value,value,value,value,value,value];
const inside = point(0, fill(0), fill(1));
const lower = point(0, fill(-6.28), fill(-6.28));
const upper = point(0, fill(6.28), fill(6.28));
const below = point(1.5, [0,0,-6.5,0,0,0], fill(0));
const above = point(2.5, fill(0), [0,0,0,0,0,6.5]);
const trajectory = { name: "Diagnostic", points: [inside, lower, below, above, upper] };
const before = JSON.stringify(trajectory);
const result = validateTrajectoryJointLimits(trajectory, metadata);
const firstBeforeMutation = { ...result.firstViolation };
result.violations[0].valueRad = 99;
const independentFirst = result.firstViolation.valueRad;
console.log(JSON.stringify({
  inside: validateTrajectoryPointJointLimits(inside, 0, metadata),
  lower: validateTrajectoryPointJointLimits(lower, 0, metadata),
  upper: validateTrajectoryPointJointLimits(upper, 0, metadata),
  below: validateTrajectoryPointJointLimits(below, 2, metadata),
  above: validateTrajectoryPointJointLimits(above, 3, metadata),
  result,
  firstBeforeMutation,
  independentFirst,
  sourceUnchanged: JSON.stringify(trajectory) === before,
}));
'''
        result = _run_node(
            {"validator.mjs": self.validator},
            f'globalThis.metadataJson = {json.dumps(json.dumps(self.metadata))};\n{harness}',
        )
        self.assertTrue(result["inside"]["valid"])
        self.assertTrue(result["lower"]["valid"])
        self.assertTrue(result["upper"]["valid"])
        self.assertFalse(result["below"]["valid"])
        self.assertFalse(result["above"]["valid"])
        below = result["below"]["firstViolation"]
        self.assertEqual(below["pointIndex"], 2)
        self.assertEqual(below["timeFromStartS"], 1.5)
        self.assertEqual(below["jointName"], "left_joint_3")
        self.assertAlmostEqual(below["amountOutsideRad"], 0.22)
        above = result["above"]["firstViolation"]
        self.assertEqual(above["jointName"], "right_joint_6")
        self.assertAlmostEqual(above["amountOutsideRad"], 0.22)
        self.assertFalse(result["result"]["valid"])
        self.assertEqual(result["result"]["checkedPointCount"], 5)
        self.assertEqual(result["result"]["checkedJointCount"], 60)
        self.assertEqual(len(result["result"]["violations"]), 2)
        self.assertEqual(result["firstBeforeMutation"]["valueRad"], -6.5)
        self.assertEqual(result["independentFirst"], -6.5)
        self.assertTrue(result["sourceUnchanged"])

    def test_controller_validation_lifecycle_and_model_independence(self) -> None:
        start = self.digital_twin.index("const EXPECTED_JOINTS =")
        end = self.digital_twin.index("const publicApi =")
        controller = self.digital_twin[start:end]
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
import {{ validateTrajectoryJointLimits }} from "./validator.mjs";
import {{ DUAL_JAKA_A12_JOINT_LIMIT_METADATA }} from "./metadata.mjs";
globalThis.document = {{ getElementById() {{ return null; }} }};
let nowMs = 0;
Object.defineProperty(globalThis, "performance", {{
  configurable: true, value: {{ now() {{ return nowMs; }} }},
}});
let nextFrameId = 1;
const frames = new Map();
globalThis.requestAnimationFrame = (callback) => {{
  const id = nextFrameId++; frames.set(id, callback); return id;
}};
globalThis.cancelAnimationFrame = (id) => frames.delete(id);
function runFrame(timestamp) {{
  nowMs = timestamp;
  const pending = [...frames.values()];
  frames.clear();
  pending.forEach((callback) => callback(timestamp));
}}
{controller}
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
const validTrajectory = {{
  name: "Valid",
  points: [
    {{ time_from_start_s: 0, left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] }},
    {{ time_from_start_s: 1, left: [1,1,1,1,1,1], right: [-1,-1,-1,-1,-1,-1] }},
  ],
}};
const noTrajectory = validateLoadedTrajectoryJointLimits();
loadPlannedTrajectory(validTrajectory, "VALID TEST");
const initial = getTrajectoryValidationState();
const trajectoryBefore = JSON.stringify(getTrajectoryPreviewState().trajectory);
const actualBeforeValidation = actualCalls.length;
const plannedBeforeValidation = plannedCalls.length;
const valid = validateLoadedTrajectoryJointLimits();
const trajectoryAfter = JSON.stringify(getTrajectoryPreviewState().trajectory);
const actualAfterValidation = actualCalls.length;
const plannedAfterValidation = plannedCalls.length;
playPlannedTrajectory();
runFrame(500);
const afterPlayback = getTrajectoryValidationState();
loadPlannedTrajectory(MOCK_INVALID_LIMIT_TRAJECTORY, "OFFLINE VALIDATION TEST");
const resetByNewTrajectory = getTrajectoryValidationState();
const invalid = validateLoadedTrajectoryJointLimits();
const clearedTrajectory = clearPlannedTrajectory();
const resetByClear = getTrajectoryValidationState();
console.log(JSON.stringify({{
  noTrajectory, initial, valid, afterPlayback, resetByNewTrajectory, invalid,
  clearedTrajectory, resetByClear, trajectoryBefore, trajectoryAfter,
  actualBeforeValidation, actualAfterValidation,
  plannedBeforeValidation, plannedAfterValidation,
}}));
'''
        result = _run_node(
            {
                "adapter.mjs": self.adapter,
                "smoothing.mjs": SMOOTHING_PATH.read_text(encoding="utf-8"),
                "planned.mjs": self.planned,
                "trajectory.mjs": self.trajectory,
                "validator.mjs": self.validator,
                "metadata.mjs": self.generated_module,
            },
            harness,
        )
        self.assertEqual(result["noTrajectory"]["status"], "ERROR")
        self.assertEqual(result["initial"]["status"], "NOT_VALIDATED")
        self.assertEqual(result["valid"]["status"], "VALID")
        self.assertEqual(result["valid"]["jointLimits"], "PASS")
        self.assertTrue(result["valid"]["valid"])
        self.assertEqual(result["valid"]["checkedPointCount"], 2)
        self.assertEqual(result["valid"]["checkedJointCount"], 24)
        self.assertEqual(result["valid"]["moveitStateValidity"], "NOT_RUN")
        self.assertEqual(result["valid"]["collision"], "NOT_RUN")
        self.assertEqual(result["valid"], result["afterPlayback"])
        self.assertEqual(result["trajectoryBefore"], result["trajectoryAfter"])
        self.assertEqual(result["actualBeforeValidation"], result["actualAfterValidation"])
        self.assertEqual(result["plannedBeforeValidation"], result["plannedAfterValidation"])
        self.assertEqual(result["resetByNewTrajectory"]["status"], "NOT_VALIDATED")
        self.assertEqual(result["invalid"]["status"], "INVALID")
        self.assertEqual(result["invalid"]["jointLimits"], "FAIL")
        self.assertEqual(result["invalid"]["violationCount"], 1)
        self.assertEqual(result["invalid"]["firstViolation"]["jointName"], "left_joint_3")
        self.assertEqual(result["invalid"]["firstViolation"]["pointIndex"], 1)
        self.assertEqual(result["invalid"]["firstViolation"]["timeFromStartS"], 1.5)
        self.assertEqual(result["resetByClear"]["status"], "NOT_VALIDATED")
        self.assertIsNone(result["clearedTrajectory"]["trajectory"])

    def test_public_api_ui_partial_semantics_scope_and_protected_file(self) -> None:
        api = self.digital_twin.split("const publicApi = {", 1)[1].split("};", 1)[0]
        for method in (
            "validateLoadedTrajectory",
            "validateLoadedTrajectoryJointLimits",
            "clearTrajectoryValidation",
            "getTrajectoryValidationState",
        ):
            with self.subTest(method=method):
                self.assertIn(method, api)

        lifecycle = self.digital_twin.split("function loadPlannedTrajectory", 1)[1].split(
            "function formatMirrorTimestamp", 1
        )[0]
        self.assertIn("clearTrajectoryValidation()", lifecycle)
        playback = lifecycle.split("function advanceTrajectoryPlayback", 1)[1].split(
            "function playPlannedTrajectory", 1
        )[0]
        self.assertNotIn("validateLoadedTrajectoryJointLimits", playback)

        panel = self.html.split(
            'class="digital-twin-validation-panel"', 1
        )[1].split('class="digital-twin-warning"', 1)[0]
        for label in (
            "Trajectory Validation — PLAN ONLY",
            "Overall Validation",
            "Joint Position Limits",
            "Stored-Point MoveIt Validity",
            "Stored-Point Collision Check",
            "Checked Points",
            "Violation Count",
            "First Failed Point",
            "Failed Joint",
            "Joint Value",
            "Allowed Range",
            "Stored Points Checked by MoveIt",
            "Stored Points Failed by MoveIt",
            "First Failed Stored Point",
            "First Stored-Point Collision Pair",
            "Stored-Point Collision Pair Count",
            "Maximum Stored-Point Penetration",
            "Validation Source",
            "Model Source",
            "Validation Error",
            "Validate Joint Limits",
            "Clear Validation",
            "Load Invalid Limit Test — OFFLINE VALIDATION TEST",
            "Load Known Collision Test — OFFLINE MOVEIT TEST",
            "NOT FOR ROBOT EXECUTION",
            "DISCRETE SAMPLED CHECK",
            "NOT A CONTINUOUS COLLISION GUARANTEE",
            "DYNAMICS NOT VALIDATED",
            "PHYSICAL EXECUTION NOT VALIDATED",
        ):
            with self.subTest(label=label):
                self.assertIn(label, panel)
        self.assertIn("PARTIAL PASS — JOINT LIMITS PASS — MOVEIT PENDING", self.digital_twin)
        self.assertNotIn(">Execute<", panel)
        validation_controller = self.digital_twin.split(
            "function trajectoryValidationStateSnapshot", 1
        )[1].split("function formatMirrorTimestamp", 1)[0]
        self.assertNotIn("fetch(", validation_controller)
        self.assertNotIn("/api/", validation_controller)
        self.assertNotIn("setJointValues(", validation_controller)
        self.assertNotIn("setPlannedJointValues(", validation_controller)

        if PROTECTED_UI_PROMPT.is_file():
            digest = hashlib.sha256(PROTECTED_UI_PROMPT.read_bytes()).hexdigest()
            self.assertEqual(digest, PROTECTED_UI_PROMPT_SHA256)

    def test_validation_ui_source_and_failed_point_formatters(self) -> None:
        helper_source = "function validationSourceLabel" + self.digital_twin.split(
            "function validationSourceLabel", 1
        )[1].split("function updateTrajectoryValidationUi", 1)[0]
        harness = f'''
{helper_source}
console.log(JSON.stringify({{
  logicalSource: validationSourceLabel(
    "GENERATED URDF POSITION LIMITS — src/config/dual_jaka_a12.urdf.xacro",
  ),
  emptySource: validationSourceLabel(""),
  modelPath: validationModelPath(
    "GENERATED URDF POSITION LIMITS — src/config/dual_jaka_a12.urdf.xacro",
  ),
  missingModelPath: validationModelPath("GENERATED URDF POSITION LIMITS"),
  unixBasename: validationPathBasename(
    "/workspace/config/dual_jaka_a12.urdf.xacro",
  ),
  relativeBasename: validationPathBasename(
    "src/config/dual_jaka_a12.urdf.xacro",
  ),
  windowsBasename: validationPathBasename(
    "src\\\\config\\\\dual_jaka_a12.urdf.xacro",
  ),
  missingBasename: validationPathBasename(null),
  secondOfThree: validationFailedPointLabel({{ pointIndex: 1 }}, 3),
  firstOfFive: validationFailedPointLabel({{ pointIndex: 0 }}, 5),
  noViolation: validationFailedPointLabel(null, 3),
  negativeIndex: validationFailedPointLabel({{ pointIndex: -1 }}, 3),
  fractionalIndex: validationFailedPointLabel({{ pointIndex: 1.5 }}, 3),
  outsideCount: validationFailedPointLabel({{ pointIndex: 3 }}, 3),
}}));
'''
        result = _run_node({}, harness)
        self.assertEqual(result["logicalSource"], "GENERATED URDF POSITION LIMITS")
        self.assertEqual(result["emptySource"], "NONE")
        self.assertEqual(result["modelPath"], "src/config/dual_jaka_a12.urdf.xacro")
        self.assertIsNone(result["missingModelPath"])
        self.assertEqual(result["unixBasename"], "dual_jaka_a12.urdf.xacro")
        self.assertEqual(result["relativeBasename"], "dual_jaka_a12.urdf.xacro")
        self.assertEqual(result["windowsBasename"], "dual_jaka_a12.urdf.xacro")
        self.assertEqual(result["missingBasename"], "UNAVAILABLE")
        self.assertEqual(result["secondOfThree"], "Point 2 of 3")
        self.assertEqual(result["firstOfFive"], "Point 1 of 5")
        self.assertEqual(result["noViolation"], "NONE")
        for key in ("negativeIndex", "fractionalIndex", "outsideCount"):
            with self.subTest(case=key):
                self.assertEqual(result[key], "UNAVAILABLE")

        update_ui = self.digital_twin.split(
            "function updateTrajectoryValidationUi", 1
        )[1].split("function clearTrajectoryValidation", 1)[0]
        self.assertIn("validationFailedPointLabel(", update_ui)
        self.assertIn("validationSourceLabel(", update_ui)
        self.assertIn("validationModelPath(", update_ui)
        self.assertIn("validationPathBasename(", update_ui)
        self.assertNotIn(
            "digitalTwinValidationSource: trajectoryValidationState.source",
            update_ui,
        )
        self.assertNotIn("dual_jaka_a12.urdf.xacro", update_ui)
        self.assertIn('id="digitalTwinValidationModelSource"', self.html)


if __name__ == "__main__":
    unittest.main()
