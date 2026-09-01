"""Offline tests for Phase 1D.3B MoveIt state-validity integration."""

from __future__ import annotations

import ast
import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from dual_arm_app.backend.moveit_state_validation import (
    EXPECTED_JOINTS,
    MoveItStateValidationBridge,
    TrajectoryValidationInputError,
    aggregate_point_results,
    normalize_trajectory,
    parse_point_response,
)


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
BRIDGE = ROOT / "dual_arm_app/backend/moveit_state_validation.py"
WEB = ROOT / "dual_arm_app/web"
DIGITAL_TWIN = WEB / "digital_twin.js"
TRANSPORT = WEB / "digital_twin_moveit_validation_source.js"
HTML = WEB / "index.html"

VALID_LEFT = [3.14, 0.52124, -0.800072, 0.0, 0.798816, 0.0]
VALID_RIGHT = [0.0, 2.617504, 0.798816, 0.0, 2.339928, 0.0]
COLLISION_LEFT = [
    2.713157, -0.969667, -2.416695, -2.393133, 2.062073, 0.615523,
]
COLLISION_RIGHT = [
    1.822536, 1.363256, 0.214982, 2.807525, -0.720792, 0.308815,
]


def trajectory(*points):
    return {"name": "Test", "points": list(points)}


def point(timestamp=0.0, left=None, right=None):
    return {
        "time_from_start_s": timestamp,
        "left": list(VALID_LEFT if left is None else left),
        "right": list(VALID_RIGHT if right is None else right),
    }


class Message:
    pass


class FakeFuture:
    def __init__(self, response=None, *, done=True, error=None):
        self.response = response
        self.is_done = done
        self.error = error
        self.cancelled = False

    def done(self):
        return self.is_done

    def result(self):
        if self.error:
            raise self.error
        return self.response

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, futures, *, available=True):
        self.futures = list(futures)
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        self.availability_timeout = timeout_sec
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return self.futures.pop(0)


def bridge(client, timeout_s=0.02):
    return MoveItStateValidationBridge(
        client, Message, Message, Message, timeout_s=timeout_s
    )


def response(valid, contacts=()):
    return SimpleNamespace(valid=valid, contacts=list(contacts))


def contact(body_1, body_2, depth):
    return SimpleNamespace(
        contact_body_1=body_1, contact_body_2=body_2, depth=depth
    )


def run_node(harness):
    sources = {
        "adapter.mjs": (WEB / "digital_twin_status_adapter.js").read_text(),
        "smoothing.mjs": (WEB / "digital_twin_live_smoothing.js").read_text(),
        "planned.mjs": (WEB / "digital_twin_planned_preview.js").read_text(),
        "trajectory.mjs": (WEB / "digital_twin_trajectory_preview.js").read_text(),
        "validator.mjs": (WEB / "digital_twin_trajectory_validation.js").read_text(),
        "metadata.mjs": (WEB / "digital_twin_joint_limit_metadata.js").read_text(),
    }
    with tempfile.TemporaryDirectory(prefix="phase1d3b-") as temporary:
        directory = Path(temporary)
        for name, source in sources.items():
            (directory / name).write_text(source, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        try:
            completed = subprocess.run(
                ["node", str(directory / "harness.mjs")],
                check=True,
                capture_output=True,
                text=True,
            )
        except subprocess.CalledProcessError as error:
            raise AssertionError(error.stderr or error.stdout) from error
    return json.loads(completed.stdout)


class BackendValidationTests(unittest.TestCase):
    def test_exact_mapping_sequential_results_and_first_failure(self):
        client = FakeClient([
            FakeFuture(response(True)),
            FakeFuture(response(False, [contact("right_J2", "left_J6", 0.003128)])),
        ])
        source = trajectory(point(0), point(1, COLLISION_LEFT, COLLISION_RIGHT))
        before = json.dumps(source)
        result = bridge(client).validate_trajectory(source)

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["checked_point_count"], 2)
        self.assertEqual(result["failed_point_count"], 1)
        self.assertEqual(result["first_failed_point"]["point_index"], 1)
        self.assertTrue(result["first_failed_point"]["collision"])
        self.assertEqual(len(client.requests), 2)
        self.assertEqual(client.requests[0].group_name, "dual_arm")
        self.assertEqual(client.requests[0].robot_state.joint_state.name, list(EXPECTED_JOINTS))
        self.assertEqual(
            client.requests[1].robot_state.joint_state.position,
            COLLISION_LEFT + COLLISION_RIGHT,
        )
        self.assertEqual(json.dumps(source), before)

    def test_contact_semantics_deduplication_and_meter_depth(self):
        contacts = [
            contact("right_J1", "left_J4", 0.0017164069),
            contact("left_J4", "right_J1", 0.001),
            contact("right_J2", "left_J6", 0.0031280062),
        ]
        parsed = parse_point_response(response(False, contacts), 4, 1.5)
        summary = aggregate_point_results([parsed])
        self.assertTrue(parsed["collision"])
        self.assertEqual(parsed["contacts"][0]["body_1"], "right_J1")
        self.assertEqual(parsed["contacts"][0]["body_2"], "left_J4")
        self.assertAlmostEqual(parsed["contacts"][0]["depth_m"], 0.0017164069)
        self.assertEqual(summary["collision_pair_count"], 2)
        self.assertAlmostEqual(summary["max_penetration_depth_m"], 0.0031280062)

        no_contact = parse_point_response(response(False), 0, 0.0)
        self.assertFalse(no_contact["collision"])
        self.assertIn("NO COLLISION CONTACT RETURNED", no_contact["reason"])
        self.assertIsNone(aggregate_point_results([no_contact])["collision_free"])

    def test_unavailable_timeout_and_service_error_states(self):
        unavailable = bridge(FakeClient([], available=False)).validate_trajectory(
            trajectory(point())
        )
        self.assertEqual(unavailable["status"], "UNAVAILABLE")

        pending = FakeFuture(done=False)
        timed_out = bridge(FakeClient([pending]), timeout_s=0.001).validate_trajectory(
            trajectory(point())
        )
        self.assertEqual(timed_out["status"], "TIMEOUT")
        self.assertTrue(pending.cancelled)

        failed = bridge(FakeClient([FakeFuture(error=RuntimeError("service failed"))]))
        self.assertEqual(failed.validate_trajectory(trajectory(point()))["status"], "ERROR")

    def test_input_contract_rejects_malformed_unsafe_values(self):
        invalid = [
            None,
            {},
            {"points": []},
            {"points": [None]},
            trajectory(point(left=[0] * 5)),
            trajectory({
                "time_from_start_s": 0,
                "left": (0, 0, 0, 0, 0, 0),
                "right": list(VALID_RIGHT),
            }),
            trajectory(point(left=[0, 0, 0, 0, 0, True])),
            trajectory(point(right=[0, 0, 0, 0, 0, "0"])),
            trajectory(point(right=[0, 0, 0, 0, 0, math.nan])),
            trajectory(point(right=[0, 0, 0, 0, 0, math.inf])),
            trajectory(point(-0.1)),
            trajectory(point(0), point(0)),
        ]
        for candidate in invalid:
            with self.subTest(candidate=repr(candidate)):
                with self.assertRaises(TrajectoryValidationInputError):
                    normalize_trajectory(candidate)

    def test_endpoint_is_post_only_validation_and_bridge_has_no_motion_clients(self):
        backend_source = BACKEND.read_text(encoding="utf-8")
        tree = ast.parse(backend_source)
        endpoint = next(
            node for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "api_digital_twin_validate_trajectory"
        )
        decorators = [ast.unparse(item) for item in endpoint.decorator_list]
        self.assertEqual(decorators, ["app.post('/api/digital-twin/validate-trajectory')"])
        endpoint_source = ast.get_source_segment(backend_source, endpoint)
        self.assertIn("validate_digital_twin_trajectory", endpoint_source)
        for forbidden in ("joint_move", "servo", "jog", "execute", "start_motion"):
            self.assertNotIn(forbidden, endpoint_source.lower())

        bridge_source = BRIDGE.read_text(encoding="utf-8")
        self.assertIn("create_client(GetStateValidity", bridge_source)
        for forbidden in (
            "create_publisher", "create_action_client", "FollowJointTrajectory",
            "ExecuteTrajectory", "/execute_trajectory", "/move_action",
        ):
            self.assertNotIn(forbidden, bridge_source)

    def test_confirmed_regression_fixtures_are_encoded_exactly(self):
        source = DIGITAL_TWIN.read_text(encoding="utf-8")
        for value in VALID_LEFT + VALID_RIGHT + COLLISION_LEFT + COLLISION_RIGHT:
            self.assertIn(str(value).lower(), source.lower())
        self.assertIn("KNOWN_MOVEIT_COLLISION_TRAJECTORY", source)
        self.assertIn("time_from_start_s: 0", source)
        self.assertIn("time_from_start_s: 1", source)


class FrontendValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = DIGITAL_TWIN.read_text(encoding="utf-8")
        cls.html = HTML.read_text(encoding="utf-8")
        start = cls.source.index("const EXPECTED_JOINTS =")
        end = cls.source.index("const publicApi =")
        cls.controller = cls.source[start:end]

    def test_transport_is_one_post_validation_request(self):
        source = TRANSPORT.read_text(encoding="utf-8")
        self.assertIn('method: "POST"', source)
        self.assertIn("const requestBody = { trajectory }", source)
        self.assertIn("requestBody.sampled_path = sampledPath", source)
        self.assertIn("JSON.stringify(requestBody)", source)
        self.assertIn('"/api/digital-twin/validate-trajectory"', source)
        for forbidden in ("/joint_move", "/jog", "/run", "/execute"):
            self.assertNotIn(forbidden, source)

    def test_full_pipeline_states_duplicate_guard_and_joint_limit_gate(self):
        harness = f'''
import {{ DEFAULT_STALE_TIMEOUT_MS, isNormalizedSnapshotStale, normalizeDualArmStatusSnapshot }} from "./adapter.mjs";
import {{ DEFAULT_VISUAL_SMOOTHING_TAU_MS, copyDualArmPose, smoothDualArmPose }} from "./smoothing.mjs";
import {{ maxAbsoluteJointDelta, normalizePlannedDualArmPose }} from "./planned.mjs";
import {{ normalizeDualArmTrajectory, sampleTrajectoryAtTime, trajectoryDurationSeconds, trajectoryPointAtIndex }} from "./trajectory.mjs";
import {{ validateTrajectoryJointLimits }} from "./validator.mjs";
import {{ DUAL_JAKA_A12_JOINT_LIMIT_METADATA }} from "./metadata.mjs";
const elements = new Map();
globalThis.document = {{ getElementById(id) {{ if (!elements.has(id)) elements.set(id, {{ textContent: "", value: id === "digitalTwinSampledPathMaxJointStep" ? "0.05" : "", setAttribute() {{}} }}); return elements.get(id); }} }};
globalThis.performance = {{ now() {{ return 0; }} }};
globalThis.requestAnimationFrame = () => 1;
globalThis.cancelAnimationFrame = () => {{}};
let calls = 0;
let handler;
async function requestMoveItTrajectoryValidation(trajectory) {{ calls += 1; return handler(trajectory); }}
{self.controller}
(async () => {{
const valid = {{ name: "Valid", points: [{{ time_from_start_s: 0, left: {json.dumps(VALID_LEFT)}, right: {json.dumps(VALID_RIGHT)} }}, {{ time_from_start_s: 1, left: {json.dumps(VALID_LEFT)}, right: {json.dumps(VALID_RIGHT)} }}] }};
const invalidLimit = {{ name: "Bad", points: [{{ time_from_start_s: 0, left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] }}, {{ time_from_start_s: 1, left: [7,0,0,0,0,0], right: [0,0,0,0,0,0] }}] }};
loadPlannedTrajectory(invalidLimit, "TEST");
const gated = await validateLoadedTrajectory();
const callsAfterGate = calls;
loadPlannedTrajectory(valid, "TEST");
let release;
const deferred = new Promise((resolve) => {{ release = resolve; }});
handler = () => deferred;
const pending = validateLoadedTrajectory();
const checking = getTrajectoryValidationState();
const duplicate = await validateLoadedTrajectory();
release({{ validation: {{ status: "PASS", checked_point_count: 1, failed_point_count: 0, first_failed_point: null, first_collision_pair: null, collision_pair_count: 0, max_penetration_depth_m: null, error: null }} }});
const passed = await pending;
loadPlannedTrajectory(KNOWN_MOVEIT_COLLISION_TRAJECTORY, "TEST");
handler = async () => ({{ validation: {{ status: "FAIL", checked_point_count: 2, failed_point_count: 1, first_failed_point: {{ point_index: 1, contacts: [{{ body_1: "right_J2", body_2: "left_J6", depth_m: 0.0031280062 }}] }}, first_collision_pair: {{ body_1: "left_J6", body_2: "right_J2" }}, collision_pair_count: 1, max_penetration_depth_m: 0.0031280062, error: null }} }});
const collision = await validateLoadedTrajectory();
const collisionUi = {{ point: elements.get("digitalTwinValidationFirstMoveItPoint").textContent, pair: elements.get("digitalTwinValidationFirstCollisionPair").textContent, depth: elements.get("digitalTwinValidationMaxPenetration").textContent }};
loadPlannedTrajectory(valid, "TEST");
handler = async () => ({{ validation: {{ status: "UNAVAILABLE", checked_point_count: 0, failed_point_count: 0, collision_pair_count: 0, error: "offline" }} }});
const unavailable = await validateLoadedTrajectory();
loadPlannedTrajectory(valid, "TEST");
handler = async () => {{ const error = new Error("timeout"); error.code = "TIMEOUT"; throw error; }};
const timeout = await validateLoadedTrajectory();
console.log(JSON.stringify({{ gated, callsAfterGate, checking, duplicate, passed, collision, collisionUi, unavailable, timeout, calls }}));
}})().catch((error) => {{ console.error(error); process.exitCode = 1; }});
'''
        result = run_node(harness)
        self.assertEqual(result["gated"]["jointLimits"], "FAIL")
        self.assertEqual(result["gated"]["moveitStateValidity"], "NOT_RUN")
        self.assertEqual(result["callsAfterGate"], 0)
        self.assertEqual(result["checking"]["moveitStateValidity"], "CHECKING")
        self.assertEqual(result["duplicate"]["moveitStateValidity"], "CHECKING")
        self.assertEqual(result["passed"]["moveitStateValidity"], "PASS")
        self.assertEqual(result["passed"]["collision"], "PASS")
        self.assertEqual(result["collision"]["collision"], "FAIL")
        self.assertEqual(result["collisionUi"]["point"], "Point 2 of 2")
        self.assertIn("left_J6", result["collisionUi"]["pair"])
        self.assertEqual(result["collisionUi"]["depth"], "3.128 mm (0.003128 m)")
        self.assertEqual(result["unavailable"]["moveitStateValidity"], "UNAVAILABLE")
        self.assertEqual(result["unavailable"]["collision"], "NOT_RUN")
        self.assertEqual(result["timeout"]["moveitStateValidity"], "TIMEOUT")
        self.assertEqual(result["calls"], 4)

    def test_public_api_ui_warnings_and_no_automatic_validation(self):
        api = self.source.split("const publicApi = {", 1)[1].split("};", 1)[0]
        self.assertIn("validateLoadedTrajectory,", api)
        for text in (
            "Validate Full Sampled Path",
            "Load Known Collision Test — OFFLINE MOVEIT TEST",
            "Stored Points Checked by MoveIt", "Stored Points Failed by MoveIt",
            "First Failed Stored Point", "First Stored-Point Collision Pair",
            "Stored-Point Collision Pair Count", "Maximum Stored-Point Penetration",
            "NOT FOR ROBOT EXECUTION", "DISCRETE SAMPLED CHECK",
            "NOT A CONTINUOUS COLLISION GUARANTEE", "DYNAMICS NOT VALIDATED",
        ):
            self.assertIn(text, self.html)
        self.assertNotIn(">Execute<", self.html)
        lifecycle = self.source.split("function loadPlannedTrajectory", 1)[1].split(
            "function trajectoryValidationStateSnapshot", 1
        )[0]
        playback = lifecycle.split("function advanceTrajectoryPlayback", 1)[1]
        self.assertNotIn("requestMoveItTrajectoryValidation", playback)
        self.assertNotIn("validateLoadedTrajectory()", playback)
        self.assertIn("clearTrajectoryValidation()", lifecycle)


if __name__ == "__main__":
    unittest.main()
