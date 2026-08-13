"""Offline Phase-1 tests for RobotMsg freshness and read-only TCP sources."""

from __future__ import annotations

import ast
import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path

from dual_arm_app.backend.digital_twin_live_state import (
    ROBOT_STATE_FIELDS,
    build_digital_twin_robot_status,
    build_digital_twin_tcp_status,
    normalize_robot_state_message,
    normalize_tcp_pose,
)


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
HTML = ROOT / "dual_arm_app/web/index.html"
JOINT_SOURCE = ROOT / "dual_arm_app/web/digital_twin_live_source.js"
STATUS_SOURCE = ROOT / "dual_arm_app/web/digital_twin_robot_status_source.js"
TCP_SOURCE = ROOT / "dual_arm_app/web/digital_twin_tcp_source.js"


def valid_state_cache(received_at_ms=900):
    return {
        side: {
            "state": {
                "power_state": 1,
                "servo_state": 1,
                "motion_state": 0,
                "collision_state": 0,
            },
            "received_at_ms": received_at_ms,
            "valid": True,
            "status": "VALID",
            "error": None,
        }
        for side in ("left", "right")
    }


def valid_joint_cache(received_at_ms=950):
    return {
        side: {
            "joint": [0, 1, 2, 3, 4, 5],
            "received_at_ms": received_at_ms,
            "status": "VALID",
            "error": None,
        }
        for side in ("left", "right")
    }


class RobotStateNormalizationTests(unittest.TestCase):
    def test_normalizes_left_and_right_with_canonical_fields_only(self):
        source = {
            "power_state": 1,
            "servo_state": 1.0,
            "motion_state": 2,
            "collision_state": 0,
            "secret_internal": 99,
        }
        for side in ("left", "right"):
            with self.subTest(side=side):
                result = normalize_robot_state_message(source)
                self.assertTrue(result["valid"])
                self.assertEqual(tuple(result["state"]), ROBOT_STATE_FIELDS)
                self.assertNotIn("secret_internal", result["state"])

    def test_missing_invalid_non_finite_and_bool_are_rejected(self):
        self.assertFalse(normalize_robot_state_message(None)["valid"])
        base = {
            "power_state": 1,
            "servo_state": 1,
            "motion_state": 0,
            "collision_state": 0,
        }
        for bad in (True, False, math.nan, math.inf, -math.inf, 1.5, "1"):
            candidate = dict(base, power_state=bad)
            with self.subTest(value=bad):
                self.assertFalse(normalize_robot_state_message(candidate)["valid"])

    def test_response_is_copied_and_reports_live_stale_missing_invalid(self):
        states = valid_state_cache()
        joints = valid_joint_cache()
        live = build_digital_twin_robot_status(
            states, joints, server_time_ms=1000
        )
        self.assertTrue(live["ok"])
        self.assertEqual(live["left"]["feedback_status"], "LIVE")
        self.assertEqual(live["left"]["robot_state_age_ms"], 100)
        self.assertEqual(live["left"]["joint_feedback_age_ms"], 50)
        live["left"]["state"]["power_state"] = 999
        self.assertEqual(states["left"]["state"]["power_state"], 1)

        stale = build_digital_twin_robot_status(
            states, joints, server_time_ms=2501
        )
        self.assertEqual(stale["left"]["feedback_status"], "STALE")

        missing_states = valid_state_cache()
        missing_states["left"] = {
            "state": None,
            "received_at_ms": None,
            "valid": False,
            "status": "MISSING",
            "error": "No RobotMsg received",
        }
        missing = build_digital_twin_robot_status(
            missing_states, joints, server_time_ms=1000
        )
        self.assertEqual(missing["left"]["feedback_status"], "MISSING")

        invalid_states = valid_state_cache()
        invalid_states["right"] = {
            "state": None,
            "received_at_ms": None,
            "valid": False,
            "status": "INVALID",
            "error": "power_state must be a finite integer",
        }
        invalid = build_digital_twin_robot_status(
            invalid_states, joints, server_time_ms=1000
        )
        self.assertEqual(invalid["right"]["feedback_status"], "INVALID")

        invalid_joints = valid_joint_cache()
        invalid_joints["left"]["status"] = "INVALID"
        invalid_joints["left"]["error"] = "latest JointState was malformed"
        invalid_joint_status = build_digital_twin_robot_status(
            states, invalid_joints, server_time_ms=1000
        )
        self.assertEqual(
            invalid_joint_status["left"]["feedback_status"], "INVALID"
        )


class TcpNormalizationTests(unittest.TestCase):
    def test_valid_both_sides_preserves_raw_units_and_zeros(self):
        left = [0, 2, 3, 0, 0.5, -0.5]
        right = [10, 20, 30, 1, 2, 3]
        result = build_digital_twin_tcp_status(
            {"left": left, "right": right}, server_time_ms=1234
        )
        self.assertTrue(result["ok"])
        self.assertEqual(result["left"]["tcp"], [float(x) for x in left])
        self.assertEqual(
            result["units"],
            {"translation": "millimeter", "orientation": "radian"},
        )
        self.assertEqual(
            result["frame"], "jaka_controller_current_user_coordinate"
        )
        self.assertEqual(
            result["orientation_convention"], "JAKA CartesianPose RPY rx_ry_rz"
        )

    def test_one_or_both_unavailable_do_not_fabricate_zero_pose(self):
        one = build_digital_twin_tcp_status(
            {"left": [1, 2, 3, 4, 5, 6], "right": None},
            errors={"right": "service unavailable"},
            server_time_ms=1000,
        )
        self.assertTrue(one["left"]["valid"])
        self.assertFalse(one["right"]["valid"])
        self.assertIsNone(one["right"]["tcp"])
        self.assertEqual(one["right"]["error"], "service unavailable")

        both = build_digital_twin_tcp_status(
            {"left": None, "right": None}, server_time_ms=1000
        )
        self.assertFalse(both["ok"])
        self.assertIsNone(both["left"]["tcp"])
        self.assertIsNone(both["right"]["tcp"])

    def test_malformed_nan_infinity_and_bool_are_rejected(self):
        bad_poses = ([1, 2], [1, 2, 3, 4, 5, math.nan],
                     [1, 2, 3, 4, 5, math.inf],
                     [1, 2, 3, 4, 5, True], "123456")
        for pose in bad_poses:
            with self.subTest(pose=pose):
                normalized = normalize_tcp_pose(pose)
                self.assertFalse(normalized["valid"])
                self.assertIsNone(normalized["tcp"])


class BackendRouteSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = BACKEND.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)
        cls.node_class = next(
            item for item in cls.tree.body
            if isinstance(item, ast.ClassDef) and item.name == "DualJakaWebNode"
        )

    def method(self, name):
        return next(
            item for item in self.node_class.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        )

    def route(self, name):
        return next(
            item for item in self.tree.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        )

    def test_robot_status_route_is_get_only_cache_only(self):
        route = self.route("api_digital_twin_robot_status")
        self.assertEqual(
            [ast.unparse(item) for item in route.decorator_list],
            ["app.get('/api/digital-twin/robot-status')"],
        )
        route_source = ast.unparse(route)
        method_source = ast.unparse(self.method("digital_twin_robot_status"))
        for forbidden in (
            "node.status", "get_fk_pose", "call_async", "publish",
            "create_client", "create_publisher", "create_action_client",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, route_source + method_source)

    def test_tcp_route_is_get_only_and_method_calls_only_existing_fk_path(self):
        route = self.route("api_digital_twin_tcp")
        self.assertEqual(
            [ast.unparse(item) for item in route.decorator_list],
            ["app.get('/api/digital-twin/tcp')"],
        )
        source = ast.unparse(self.method("digital_twin_tcp_status"))
        self.assertIn("self.get_fk_pose(side)", source)
        for forbidden in (
            "get_ik", "joint_move", "linear_move", "servo",
            "publish", "create_action_client", "status()",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


class FrontendIsolationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.joint = JOINT_SOURCE.read_text(encoding="utf-8")
        cls.status = STATUS_SOURCE.read_text(encoding="utf-8")
        cls.tcp = TCP_SOURCE.read_text(encoding="utf-8")
        cls.html = HTML.read_text(encoding="utf-8")

    def test_source_separation_no_general_status_or_commands(self):
        self.assertIn('"/api/digital-twin/robot-status"', self.status)
        self.assertIn('"/api/digital-twin/tcp"', self.tcp)
        for source in (self.joint, self.status, self.tcp):
            self.assertNotIn("/api/status", source)
            self.assertNotIn('method: "POST"', source)
        for source in (self.status, self.tcp):
            for fragment in ("/api/jog", "/api/home", "/api/direct", "/api/program"):
                self.assertNotIn(fragment, source)

    def test_status_ui_semantics_tcp_blocker_and_existing_controls(self):
        for text in (
            "Left Feedback:", "Right Feedback:", "power_state",
            "servo_state", "motion_state", "collision_state",
            "Left TCP:", "Right TCP:",
            "translation mm / orientation rad",
            "feedback freshness, not a guaranteed robot-network connection",
            "TCP FRAME PLACEMENT BLOCKED PENDING VERIFIED BASE↔WORLD CONVERSION",
            "Direct Joint Move", "Direct TCP Move", "STOP BOTH", "Home Both",
            "Waypoint Manager", "Program / Sequence",
        ):
            with self.subTest(text=text):
                self.assertIn(text, self.html)
        panel = self.html.split('aria-labelledby="digitalTwinTitle"', 1)[1].split(
            "</section>", 1
        )[0]
        self.assertNotIn(">Execute<", panel)
        self.assertNotIn("left_J6", self.tcp)
        self.assertNotIn("right_J6", self.tcp)
        self.assertNotIn("AxesHelper", self.tcp)

    def test_independent_failures_do_not_touch_joint_mirror(self):
        harness = r'''
(async () => {
const texts = {};
globalThis.structuredClone = (value) => JSON.parse(JSON.stringify(value));
globalThis.AbortController = class {
  constructor() { this.signal = {}; }
  abort() {}
};
globalThis.document = { getElementById() { return {
  addEventListener() {},
  set textContent(value) { texts.value = value; },
}; } };
globalThis.window = {
  setTimeout() { return 1; }, clearTimeout() {},
  dualArmDigitalTwin: {
    setMirrorEnabled() {},
    ingestStatusSnapshot() { globalThis.ingestCount += 1; },
  },
};
globalThis.ingestCount = 0;
let mode = "joint";
globalThis.fetch = async () => {
  if (mode !== "joint") throw new Error(`${mode} failed`);
  return { ok: true, status: 200, json: async () => ({
    ok: true,
    left: { valid: true, joint: [0,0,0,0,0,0], received_at_ms: 100 },
    right: { valid: true, joint: [0,0,0,0,0,0], received_at_ms: 100 },
  }) };
};
const joint = await import("./joint.mjs");
const status = await import("./status.mjs");
const tcp = await import("./tcp.mjs");
await joint.pollLiveFeedbackOnce();
mode = "status";
await status.pollRobotStatusOnce();
const afterStatusFailure = globalThis.ingestCount;
mode = "tcp";
await tcp.pollTcpOnce();
const afterTcpFailure = globalThis.ingestCount;
mode = "joint";
await joint.pollLiveFeedbackOnce();
console.log(JSON.stringify({ afterStatusFailure, afterTcpFailure, final: globalThis.ingestCount }));
})().catch((error) => { console.error(error); process.exitCode = 1; });
'''
        with tempfile.TemporaryDirectory(prefix="phase1-live-") as temp:
            directory = Path(temp)
            (directory / "joint.mjs").write_text(self.joint, encoding="utf-8")
            (directory / "status.mjs").write_text(self.status, encoding="utf-8")
            (directory / "tcp.mjs").write_text(self.tcp, encoding="utf-8")
            (directory / "harness.mjs").write_text(harness, encoding="utf-8")
            try:
                result = subprocess.run(
                    ["node", str(directory / "harness.mjs")],
                    check=True, capture_output=True, text=True,
                )
            except subprocess.CalledProcessError as error:
                self.fail(error.stderr)
        output = json.loads(result.stdout)
        self.assertEqual(output, {
            "afterStatusFailure": 1,
            "afterTcpFailure": 1,
            "final": 2,
        })

    def test_stale_feedback_and_unavailable_tcp_are_rendered(self):
        harness = r'''
(async () => {
const ui = {};
globalThis.AbortController = class {
  constructor() { this.signal = {}; }
  abort() {}
};
globalThis.document = { getElementById(id) { return {
  addEventListener() {},
  set textContent(value) { ui[id] = value; },
  get textContent() { return ui[id]; },
}; } };
globalThis.window = { setTimeout() { return 1; }, clearTimeout() {} };
let payload;
globalThis.fetch = async () => ({
  ok: true, status: 200, json: async () => payload,
});
const status = await import("./status.mjs");
const tcp = await import("./tcp.mjs");
payload = {
  cache_only: true,
  left: {
    feedback_status: "STALE", robot_state_valid: true,
    joint_feedback_valid: true,
    state: { power_state: 1, servo_state: 1, motion_state: 0, collision_state: 0 },
  },
  right: {
    feedback_status: "MISSING", robot_state_valid: false,
    joint_feedback_valid: true, state: null,
  },
};
await status.pollRobotStatusOnce();
const renderedStatus = {
  left: ui.digitalTwinLeftFeedbackStatus,
  right: ui.digitalTwinRightFeedbackStatus,
  rightPower: ui.digitalTwinRightPowerState,
};
payload = {
  source: "jaka_get_fk_from_current_joint_feedback", cache_only: false,
  units: { translation: "millimeter", orientation: "radian" },
  frame: "jaka_controller_current_user_coordinate",
  orientation_convention: "JAKA CartesianPose RPY rx_ry_rz",
  left: { valid: true, tcp: [1,2,3,0.1,0.2,0.3], error: null },
  right: { valid: false, tcp: null, error: "FK unavailable" },
};
await tcp.pollTcpOnce();
console.log(JSON.stringify({
  renderedStatus,
  leftTcp: ui.digitalTwinLeftTcpStatus,
  rightTcp: ui.digitalTwinRightTcpStatus,
  placement: ui.digitalTwinTcpPlacementState,
}));
})().catch((error) => { console.error(error); process.exitCode = 1; });
'''
        with tempfile.TemporaryDirectory(prefix="phase1-render-") as temp:
            directory = Path(temp)
            (directory / "status.mjs").write_text(self.status, encoding="utf-8")
            (directory / "tcp.mjs").write_text(self.tcp, encoding="utf-8")
            (directory / "harness.mjs").write_text(harness, encoding="utf-8")
            result = subprocess.run(
                ["node", str(directory / "harness.mjs")],
                check=True, capture_output=True, text=True,
            )
        output = json.loads(result.stdout)
        self.assertEqual(output["renderedStatus"], {
            "left": "STALE",
            "right": "MISSING",
            "rightPower": "UNAVAILABLE",
        })
        self.assertEqual(output["leftTcp"], "LIVE")
        self.assertEqual(output["rightTcp"], "UNAVAILABLE")
        self.assertIn("BLOCKED PENDING VERIFIED BASE↔WORLD", output["placement"])


if __name__ == "__main__":
    unittest.main()
