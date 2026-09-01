"""Offline regression tests for the completed Phase-1 Digital Twin UI."""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
URDF = WEB / "assets/dual_jaka_a12_web.urdf"
DIGITAL_TWIN = WEB / "digital_twin.js"
PHASE1_STATE = WEB / "digital_twin_phase1_state.js"
ROBOT_STATUS_SOURCE = WEB / "digital_twin_robot_status_source.js"
CONTROLLER_TCP_SOURCE = WEB / "digital_twin_tcp_source.js"
LIVE_SMOOTHING = WEB / "digital_twin_live_smoothing.js"
HTML = WEB / "index.html"


def run_node(files: dict[str, str], entrypoint: str = "harness.mjs") -> dict:
    with tempfile.TemporaryDirectory(prefix="phase1-completion-") as temp:
        directory = Path(temp)
        for name, source in files.items():
            (directory / name).write_text(source, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / entrypoint)],
            check=True,
            capture_output=True,
            text=True,
        )
    return json.loads(result.stdout)


class ModelTcpLinkAndUiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.urdf = ET.parse(URDF).getroot()
        cls.digital_twin = DIGITAL_TWIN.read_text(encoding="utf-8")
        cls.phase1 = PHASE1_STATE.read_text(encoding="utf-8")
        cls.status_source = ROBOT_STATUS_SOURCE.read_text(encoding="utf-8")
        cls.controller_source = CONTROLLER_TCP_SOURCE.read_text(encoding="utf-8")
        cls.html = HTML.read_text(encoding="utf-8")

    def test_selected_links_exist_and_are_distinct_terminal_links(self):
        links = {link.get("name") for link in self.urdf.findall("link")}
        joint_parents = {
            parent.get("link")
            for joint in self.urdf.findall("joint")
            if (parent := joint.find("parent")) is not None
        }
        self.assertIn("left_J6", links)
        self.assertIn("right_J6", links)
        self.assertNotEqual("left_J6", "right_J6")
        self.assertNotIn("left_J6", joint_parents)
        self.assertNotIn("right_J6", joint_parents)
        self.assertIn('left: "left_J6"', self.phase1)
        self.assertIn('right: "right_J6"', self.phase1)

    def test_frames_are_attached_to_model_links_and_publicly_inspectable(self):
        self.assertIn("new THREE.AxesHelper(MODEL_TCP_AXES_SIZE_M)", self.digital_twin)
        self.assertIn("terminalLink.add(frame)", self.digital_twin)
        self.assertIn("captureModelTcpWorldPoses();", self.digital_twin)
        self.assertIn("getModelTcpState", self.digital_twin)
        self.assertIn("selectedLinks", self.digital_twin)
        self.assertIn("frameVisible", self.digital_twin)
        self.assertIn("currentWorldPose", self.digital_twin)
        self.assertNotIn("/api/digital-twin/tcp", self.digital_twin)
        for forbidden in (
            "joint_move", "linear_move", "servo_move", "create_publisher",
            "create_action_client", "FollowJointTrajectory", "ExecuteTrajectory",
        ):
            self.assertNotIn(forbidden, self.digital_twin)

    def test_model_and_controller_tcp_semantics_are_visibly_separate(self):
        for expected in (
            "Model TCP/Flange — Web/URDF World",
            "Web/URDF world — translation m / orientation rad",
            "URDF MODEL FK FROM MIRRORED JOINT STATE",
            "Digital Twin model TCP/flange",
            "JAKA controller current user coordinate",
            "translation mm / orientation rad",
            "CONTROLLER FK DIAGNOSTIC — NOT USED FOR WEB-WORLD PLACEMENT",
            "not a claim about the active physical controller tool frame",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, self.html)
        self.assertNotIn("AxesHelper", self.controller_source)
        self.assertNotIn("matrixWorld", self.controller_source)

    def test_connection_alert_ui_and_operator_controls_remain(self):
        for expected in (
            "Left Connection (feedback-derived)",
            "Right Connection (feedback-derived)",
            "Left Robot State", "Right Robot State",
            "Left Phase-1 Fault/Alert", "Right Phase-1 Fault/Alert",
            "not complete JAKA fault codes",
            "Direct Joint Move", "Direct TCP Move", "STOP BOTH", "Home Both",
            "Waypoint Manager", "Program / Sequence",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, self.html)
        panel = self.html.split('aria-labelledby="digitalTwinTitle"', 1)[1].split(
            "</section>", 1
        )[0]
        self.assertNotIn(">Execute<", panel)


class PurePhase1StateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.phase1 = PHASE1_STATE.read_text(encoding="utf-8")

    def test_rpy_identity_axes_combined_pose_finite_and_separation(self):
        harness = r'''
import {
  matrix4ElementsToWorldPose,
  rotationMatrixToRpy,
} from "./phase1.mjs";

function matrixFromRpy(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [
    cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
    sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
    -sp, cp*sr, cp*cr,
  ];
}
function matrix4(rotation, translation) {
  return [
    rotation[0], rotation[3], rotation[6], 0,
    rotation[1], rotation[4], rotation[7], 0,
    rotation[2], rotation[5], rotation[8], 0,
    translation[0], translation[1], translation[2], 1,
  ];
}
function rejects(value) {
  try { matrix4ElementsToWorldPose(value); return false; }
  catch (_error) { return true; }
}
const identity = rotationMatrixToRpy(matrixFromRpy(0, 0, 0));
const roll = rotationMatrixToRpy(matrixFromRpy(Math.PI/2, 0, 0));
const pitch = rotationMatrixToRpy(matrixFromRpy(0, Math.PI/4, 0));
const yaw = rotationMatrixToRpy(matrixFromRpy(0, 0, -Math.PI/3));
const expectedCombined = [0.31, -0.42, 0.77];
const combined = rotationMatrixToRpy(matrixFromRpy(...expectedCombined));
const left = matrix4ElementsToWorldPose(
  matrix4(matrixFromRpy(0.1, 0.2, 0.3), [1, 2, 3]),
);
const right = matrix4ElementsToWorldPose(
  matrix4(matrixFromRpy(-0.2, 0.1, -0.4), [-1, -2, 0.5]),
);
console.log(JSON.stringify({
  identity, roll, pitch, yaw, expectedCombined, combined, left, right,
  allFinite: [...left.translationM, ...left.rpyRad,
              ...right.translationM, ...right.rpyRad].every(Number.isFinite),
  rejectsShort: rejects([1, 2, 3]),
  rejectsNaN: rejects([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,NaN,1]),
  rejectsBool: rejects([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,true,1]),
}));
'''
        result = run_node({"phase1.mjs": self.phase1, "harness.mjs": harness})

        def assert_vector(actual, expected):
            self.assertEqual(len(actual), len(expected))
            for found, wanted in zip(actual, expected):
                self.assertAlmostEqual(found, wanted, places=10)

        assert_vector(result["identity"], [0, 0, 0])
        assert_vector(result["roll"], [math.pi / 2, 0, 0])
        assert_vector(result["pitch"], [0, math.pi / 4, 0])
        assert_vector(result["yaw"], [0, 0, -math.pi / 3])
        assert_vector(result["combined"], result["expectedCombined"])
        self.assertEqual(result["left"]["translationM"], [1, 2, 3])
        self.assertEqual(result["right"]["translationM"], [-1, -2, 0.5])
        self.assertNotEqual(result["left"], result["right"])
        self.assertTrue(result["allFinite"])
        self.assertTrue(result["rejectsShort"])
        self.assertTrue(result["rejectsNaN"])
        self.assertTrue(result["rejectsBool"])

    def test_connection_and_phase1_alert_mapping(self):
        harness = r'''
import {
  deriveRobotStateAndAlert,
  feedbackConnectionStatus,
} from "./phase1.mjs";
const state = (overrides = {}) => ({
  power_state: 1, servo_state: 1, motion_state: 0, collision_state: 0,
  ...overrides,
});
const side = (feedbackStatus, overrides = {}) => ({
  feedbackStatus, state: state(overrides),
});
console.log(JSON.stringify({
  connections: ["LIVE", "STALE", "MISSING", "INVALID"].map(
    feedbackConnectionStatus,
  ),
  idle: deriveRobotStateAndAlert(side("LIVE")),
  moving: deriveRobotStateAndAlert(side("LIVE", { motion_state: 2 })),
  collision: deriveRobotStateAndAlert(side("LIVE", { collision_state: 1 })),
  power: deriveRobotStateAndAlert(side("LIVE", { power_state: 0 })),
  servo: deriveRobotStateAndAlert(side("LIVE", { servo_state: 0 })),
  stale: deriveRobotStateAndAlert(side("STALE")),
  missing: deriveRobotStateAndAlert({ feedbackStatus: "MISSING", state: null }),
  invalid: deriveRobotStateAndAlert({ feedbackStatus: "INVALID", state: null }),
}));
'''
        result = run_node({"phase1.mjs": self.phase1, "harness.mjs": harness})
        self.assertEqual(
            result["connections"], ["LIVE", "STALE", "MISSING", "INVALID"]
        )
        self.assertEqual(result["idle"], {"robotState": "IDLE", "faultAlert": "NONE"})
        self.assertEqual(result["moving"]["robotState"], "MOVING")
        self.assertEqual(result["collision"]["faultAlert"], "COLLISION")
        self.assertEqual(result["power"]["faultAlert"], "POWER OFF")
        self.assertEqual(result["servo"]["faultAlert"], "SERVO OFF")
        self.assertEqual(result["stale"]["faultAlert"], "FEEDBACK STALE")
        self.assertEqual(result["missing"]["faultAlert"], "FEEDBACK MISSING")
        self.assertEqual(result["invalid"]["faultAlert"], "INVALID FEEDBACK")


class ModelTcpControllerBehaviorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.digital_twin = DIGITAL_TWIN.read_text(encoding="utf-8")
        cls.phase1 = PHASE1_STATE.read_text(encoding="utf-8")

    def test_live_capture_stale_invalid_and_reset_behavior(self):
        start = self.digital_twin.index("const EXPECTED_JOINTS =")
        end = self.digital_twin.index("const publicApi =")
        controller = self.digital_twin[start:end]
        harness = f'''
import {{ matrix4ElementsToWorldPose }} from "./phase1.mjs";
import {{
  DEFAULT_VISUAL_SMOOTHING_TAU_MS,
  copyDualArmPose,
  smoothDualArmPose,
}} from "./smoothing.mjs";
const DEFAULT_STALE_TIMEOUT_MS = 1500;
const MODEL_TCP_AXES_SIZE_M = 0.14;
function normalizeDualArmStatusSnapshot(snapshot, receivedAtMs) {{
  if (!snapshot || !snapshot.left || !snapshot.right ||
      snapshot.left.joint.length !== 6 || snapshot.right.joint.length !== 6) {{
    throw new Error("invalid snapshot");
  }}
  return {{
    left: [...snapshot.left.joint], right: [...snapshot.right.joint], receivedAtMs,
  }};
}}
function isNormalizedSnapshotStale(snapshot, nowMs, timeoutMs) {{
  return nowMs - snapshot.receivedAtMs > timeoutMs;
}}
globalThis.document = {{ getElementById() {{ return null; }} }};
globalThis.THREE = {{
  AxesHelper: class AxesHelper {{
    constructor(size) {{ this.size = size; this.name = ""; this.userData = {{}}; this.visible = true; }}
  }},
}};
{controller}
function worldMatrix(x, y, z) {{
  return {{ elements: [1,0,0,0, 0,1,0,0, 0,0,1,0, x,y,z,1] }};
}}
const attached = {{ left: [], right: [] }};
const links = {{
  left_J6: {{ matrixWorld: worldMatrix(1, 2, 3), add(frame) {{ attached.left.push(frame); }} }},
  right_J6: {{ matrixWorld: worldMatrix(-1, -2, 0.5), add(frame) {{ attached.right.push(frame); }} }},
}};
const applied = [];
robot = {{ links, joints: {{}}, updateMatrixWorld() {{}} }};
for (const name of EXPECTED_JOINTS) {{
  robot.joints[name] = {{ setJointValue(value) {{ applied.push([name, value]); }} }};
}}
loadState.status = "READY";
const created = createModelTcpFrames();
setMirrorEnabled(true);
const now = Date.now();
ingestStatusSnapshot({{
  left: {{ joint: [0.1,0.2,0.3,0.4,0.5,0.6] }},
  right: {{ joint: [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6] }},
}}, now, "LIVE JOINT FEEDBACK");
const live = getModelTcpState();
getMirrorState(now + 1501);
const stale = getModelTcpState();
ingestStatusSnapshot({{ left: {{ joint: [0] }}, right: {{ joint: [0] }} }}, now + 2);
const invalid = getModelTcpState();
resetToStaticPose();
const reset = getModelTcpState();
console.log(JSON.stringify({{
  created, live, stale, invalid, reset,
  attachedDistinct: attached.left.length === 1 && attached.right.length === 1 &&
    attached.left[0] !== attached.right[0],
  appliedCount: applied.length,
}}));
'''
        result = run_node({
            "phase1.mjs": self.phase1,
            "smoothing.mjs": LIVE_SMOOTHING.read_text(encoding="utf-8"),
            "harness.mjs": harness,
        })
        self.assertTrue(result["created"])
        self.assertTrue(result["attachedDistinct"])
        self.assertEqual(result["live"]["selectedLinks"], {
            "left": "left_J6", "right": "right_J6",
        })
        self.assertEqual(result["live"]["feedbackStatus"], {
            "left": "LIVE", "right": "LIVE",
        })
        self.assertEqual(result["live"]["frameVisible"], {
            "left": True, "right": True,
        })
        self.assertEqual(
            result["live"]["currentWorldPose"]["left"]["translationM"],
            [1, 2, 3],
        )
        self.assertEqual(
            result["live"]["currentWorldPose"]["right"]["translationM"],
            [-1, -2, 0.5],
        )
        self.assertEqual(result["live"]["translationUnit"], "meter")
        self.assertEqual(result["live"]["orientationUnit"], "radian")
        self.assertEqual(result["stale"]["feedbackStatus"], {
            "left": "STALE", "right": "STALE",
        })
        self.assertEqual(result["stale"]["frameVisible"], {
            "left": False, "right": False,
        })
        self.assertEqual(
            result["stale"]["currentWorldPose"], result["live"]["currentWorldPose"]
        )
        self.assertEqual(result["invalid"]["feedbackStatus"], {
            "left": "INVALID", "right": "INVALID",
        })
        self.assertEqual(
            result["invalid"]["currentWorldPose"], result["live"]["currentWorldPose"]
        )
        self.assertIsNone(result["reset"]["currentWorldPose"]["left"])
        self.assertIsNone(result["reset"]["currentWorldPose"]["right"])
        self.assertGreaterEqual(result["appliedCount"], 12)


if __name__ == "__main__":
    unittest.main()
