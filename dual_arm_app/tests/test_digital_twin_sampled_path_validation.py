"""Offline tests for Phase 1D.3C between-point sampled validation."""

from __future__ import annotations

import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from dual_arm_app.backend.moveit_state_validation import MoveItStateValidationBridge
from dual_arm_app.backend.sampled_path_validation import (
    DEFAULT_MAX_JOINT_STEP_RAD,
    MAX_INTERIOR_SAMPLE_COUNT,
    SampledPathValidationInputError,
    generate_segment_interior_samples,
    generate_trajectory_interior_samples,
    max_joint_delta,
    segment_subdivision_count,
    validate_max_joint_step,
)


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
DIGITAL_TWIN = WEB / "digital_twin.js"
HTML = WEB / "index.html"
BRIDGE = ROOT / "dual_arm_app/backend/moveit_state_validation.py"
SAMPLER = ROOT / "dual_arm_app/backend/sampled_path_validation.py"


def point(timestamp, value):
    return {
        "time_from_start_s": timestamp,
        "left": [value, 0, 0, 0, 0, 0],
        "right": [0, 0, 0, 0, 0, 0],
    }


def trajectory(*points):
    return {"name": "Sampled", "points": list(points)}


class Message:
    pass


class FakeFuture:
    def __init__(self, response):
        self.response = response

    def done(self):
        return True

    def result(self):
        if isinstance(self.response, Exception):
            raise self.response
        return self.response


class PendingFuture:
    def __init__(self):
        self.cancelled = False

    def done(self):
        return False

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, responses, available=True):
        self.futures = [
            item if hasattr(item, "done") else FakeFuture(item)
            for item in responses
        ]
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return self.futures.pop(0)


def response(valid, contacts=()):
    return SimpleNamespace(valid=valid, contacts=list(contacts))


def contact(body_1, body_2, depth):
    return SimpleNamespace(
        contact_body_1=body_1,
        contact_body_2=body_2,
        depth=depth,
    )


def bridge(client, timeout_s=2.0):
    return MoveItStateValidationBridge(
        client, Message, Message, Message, timeout_s=timeout_s
    )


def run_node(harness):
    sources = {
        "adapter.mjs": (WEB / "digital_twin_status_adapter.js").read_text(),
        "planned.mjs": (WEB / "digital_twin_planned_preview.js").read_text(),
        "trajectory.mjs": (WEB / "digital_twin_trajectory_preview.js").read_text(),
        "validator.mjs": (WEB / "digital_twin_trajectory_validation.js").read_text(),
        "metadata.mjs": (WEB / "digital_twin_joint_limit_metadata.js").read_text(),
    }
    with tempfile.TemporaryDirectory(prefix="phase1d3c-") as temporary:
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


class PureSamplingTests(unittest.TestCase):
    def test_delta_subdivision_ceil_and_zero_motion(self):
        start = [0.0] * 12
        end = [0.0] * 12
        end[7] = -0.100001
        self.assertAlmostEqual(max_joint_delta(start, end), 0.100001)
        self.assertEqual(segment_subdivision_count(start, end, 0.05), 3)
        self.assertEqual(segment_subdivision_count(start, start, 0.05), 1)

        zero = generate_segment_interior_samples(point(0, 0), point(1, 0), 0, 0.05)
        self.assertEqual(zero["subdivision_count"], 1)
        self.assertEqual(zero["interior_sample_count"], 0)
        self.assertEqual(zero["samples"], [])

    def test_interior_alpha_joint_and_time_interpolation_excludes_endpoints(self):
        segment = generate_segment_interior_samples(
            point(2.0, 0.0), point(5.0, 0.12), 3, 0.05
        )
        self.assertEqual(segment["subdivision_count"], 3)
        self.assertEqual([item["sample_index"] for item in segment["samples"]], [1, 2])
        self.assertEqual([item["alpha"] for item in segment["samples"]], [1 / 3, 2 / 3])
        self.assertAlmostEqual(segment["samples"][0]["left"][0], 0.04)
        self.assertAlmostEqual(segment["samples"][1]["left"][0], 0.08)
        self.assertAlmostEqual(segment["samples"][0]["time_from_start_s"], 3.0)
        self.assertAlmostEqual(segment["samples"][1]["sample_time_from_start_s"], 4.0)
        self.assertTrue(all(item["left"][1:] == [0.0] * 5 for item in segment["samples"]))
        self.assertTrue(all(item["right"] == [0.0] * 6 for item in segment["samples"]))

    def test_trajectory_metadata_copy_semantics_and_default(self):
        source = trajectory(point(0, 0), point(1, 0.12), point(2, 0.12))
        before = json.dumps(source)
        result = generate_trajectory_interior_samples(source)
        self.assertEqual(result["max_joint_step_rad"], DEFAULT_MAX_JOINT_STEP_RAD)
        self.assertEqual(result["segment_count"], 2)
        self.assertEqual(result["generated_interior_sample_count"], 2)
        self.assertEqual(result["segments"][0]["interior_sample_count"], 2)
        self.assertEqual(result["segments"][1]["interior_sample_count"], 0)
        self.assertEqual(json.dumps(source), before)

    def test_invalid_steps_and_sample_cap_are_rejected_before_generation(self):
        for value in (0, -0.1, True, "0.05", math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(SampledPathValidationInputError):
                    validate_max_joint_step(value)
        source = trajectory(point(0, 0), point(1, 1))
        with self.assertRaisesRegex(SampledPathValidationInputError, "exceeding cap"):
            generate_trajectory_interior_samples(
                source,
                1 / (MAX_INTERIOR_SAMPLE_COUNT + 2),
            )


class BackendSampledValidationTests(unittest.TestCase):
    def test_stored_pass_then_interior_pass_sequential_and_canonical(self):
        client = FakeClient([response(True), response(True), response(True)])
        result = bridge(client).validate_trajectory(
            trajectory(point(0, 0), point(1, 0.1)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )
        sampled = result["sampled_path"]
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(sampled["status"], "PASS")
        self.assertEqual(sampled["segment_count"], 1)
        self.assertEqual(sampled["generated_interior_sample_count"], 1)
        self.assertEqual(sampled["checked_interior_sample_count"], 1)
        self.assertEqual(len(client.requests), 3)
        self.assertEqual(client.requests[0].robot_state.joint_state.position[0], 0.0)
        self.assertEqual(client.requests[1].robot_state.joint_state.position[0], 0.1)
        self.assertEqual(client.requests[2].robot_state.joint_state.position[0], 0.05)
        self.assertEqual(len(client.requests[2].robot_state.joint_state.name), 12)

    def test_stored_failure_and_unavailable_skip_samples(self):
        failed_client = FakeClient([response(True), response(False)])
        failed = bridge(failed_client).validate_trajectory(
            trajectory(point(0, 0), point(1, 0.1)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )
        self.assertEqual(failed["status"], "FAIL")
        self.assertEqual(failed["sampled_path"]["status"], "SKIPPED")
        self.assertEqual(len(failed_client.requests), 2)

        unavailable_client = FakeClient([], available=False)
        unavailable = bridge(unavailable_client).validate_trajectory(
            trajectory(point(0, 0), point(1, 0.1)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )
        self.assertEqual(unavailable["status"], "UNAVAILABLE")
        self.assertEqual(unavailable["sampled_path"]["status"], "SKIPPED")
        self.assertEqual(unavailable_client.requests, [])

    def test_interior_collision_metadata_pairs_depth_and_full_diagnostics(self):
        contacts = [
            contact("right_J2", "left_J6", 0.0031280062),
            contact("left_J6", "right_J2", 0.001),
        ]
        client = FakeClient([
            response(True), response(True),
            response(False, contacts), response(True),
        ])
        result = bridge(client).validate_trajectory(
            trajectory(point(0, 0), point(2, 0.12)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )["sampled_path"]
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["checked_interior_sample_count"], 2)
        self.assertEqual(result["failed_interior_sample_count"], 1)
        first = result["first_failed_sample"]
        self.assertEqual(first["segment_index"], 0)
        self.assertEqual(first["sample_index"], 1)
        self.assertEqual(first["subdivision_count"], 3)
        self.assertAlmostEqual(first["alpha"], 1 / 3)
        self.assertAlmostEqual(first["time_from_start_s"], 2 / 3)
        self.assertTrue(first["collision"])
        self.assertEqual(result["collision_pair_count"], 1)
        self.assertAlmostEqual(result["max_penetration_depth_m"], 0.0031280062)
        self.assertEqual(result["segments"][0]["checked_interior_sample_count"], 2)
        self.assertEqual(result["segments"][0]["failed_interior_sample_count"], 1)

    def test_invalid_without_contacts_is_not_collision(self):
        client = FakeClient([response(True), response(True), response(False)])
        sampled = bridge(client).validate_trajectory(
            trajectory(point(0, 0), point(1, 0.1)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )["sampled_path"]
        self.assertEqual(sampled["status"], "FAIL")
        self.assertFalse(sampled["first_failed_sample"]["collision"])
        self.assertIn("NO COLLISION CONTACT RETURNED", sampled["first_failed_sample"]["reason"])
        self.assertEqual(sampled["collision_pair_count"], 0)

    def test_sample_timeout_and_service_error_stop_without_concurrency(self):
        pending = PendingFuture()
        timeout_client = FakeClient([response(True), response(True), pending])
        timed_out = bridge(timeout_client, timeout_s=0.001).validate_trajectory(
            trajectory(point(0, 0), point(1, 0.1)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )["sampled_path"]
        self.assertEqual(timed_out["status"], "TIMEOUT")
        self.assertEqual(timed_out["checked_interior_sample_count"], 0)
        self.assertTrue(pending.cancelled)

        error_client = FakeClient([
            response(True), response(True), RuntimeError("sample service failed")
        ])
        errored = bridge(error_client).validate_trajectory(
            trajectory(point(0, 0), point(1, 0.1)),
            {"enabled": True, "max_joint_step_rad": 0.05},
        )["sampled_path"]
        self.assertEqual(errored["status"], "ERROR")
        self.assertIn("sample service failed", errored["error"])
        self.assertEqual(len(error_client.requests), 3)

    def test_disabled_is_backward_compatible_and_modules_have_no_motion_stack(self):
        result = bridge(FakeClient([response(True)])).validate_trajectory(
            trajectory(point(0, 0))
        )
        self.assertNotIn("sampled_path", result)
        combined = BRIDGE.read_text() + SAMPLER.read_text()
        for forbidden in (
            "create_publisher", "create_action_client", "FollowJointTrajectory",
            "ExecuteTrajectory", "/execute_trajectory", "/move_action",
            "/plan_kinematic_path", "/compute_ik",
        ):
            self.assertNotIn(forbidden, combined)


class FrontendSampledValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = DIGITAL_TWIN.read_text(encoding="utf-8")
        cls.html = HTML.read_text(encoding="utf-8")
        start = cls.source.index("const EXPECTED_JOINTS =")
        end = cls.source.index("const publicApi =")
        cls.controller = cls.source[start:end]

    def test_pipeline_input_checking_pass_fail_and_human_diagnostics(self):
        harness = f'''
import {{ DEFAULT_STALE_TIMEOUT_MS, isNormalizedSnapshotStale, normalizeDualArmStatusSnapshot }} from "./adapter.mjs";
import {{ maxAbsoluteJointDelta, normalizePlannedDualArmPose }} from "./planned.mjs";
import {{ normalizeDualArmTrajectory, sampleTrajectoryAtTime, trajectoryDurationSeconds, trajectoryPointAtIndex }} from "./trajectory.mjs";
import {{ validateTrajectoryJointLimits }} from "./validator.mjs";
import {{ DUAL_JAKA_A12_JOINT_LIMIT_METADATA }} from "./metadata.mjs";
const elements = new Map();
globalThis.document = {{ getElementById(id) {{ if (!elements.has(id)) elements.set(id, {{ textContent: "", value: "", setAttribute() {{}} }}); return elements.get(id); }} }};
globalThis.performance = {{ now() {{ return 0; }} }};
globalThis.requestAnimationFrame = () => 1;
globalThis.cancelAnimationFrame = () => {{}};
let calls = 0;
let optionsSeen = null;
let handler;
async function requestMoveItTrajectoryValidation(trajectory, options) {{ calls += 1; optionsSeen = options; return handler(trajectory); }}
{self.controller}
(async () => {{
const valid = {{ name: "Valid", points: [{{ time_from_start_s: 0, left: [0,0,0,0,0,0], right: [0,0,0,0,0,0] }}, {{ time_from_start_s: 1, left: [0.1,0,0,0,0,0], right: [0,0,0,0,0,0] }}] }};
loadPlannedTrajectory(valid, "TEST");
const invalidStep = await validateLoadedTrajectory(0);
const callsAfterInvalidStep = calls;
let release;
const deferred = new Promise((resolve) => {{ release = resolve; }});
handler = () => deferred;
const pending = validateLoadedTrajectory(0.05);
const checking = getTrajectoryValidationState();
release({{ validation: {{ status: "PASS", checked_point_count: 2, failed_point_count: 0, first_failed_point: null, first_collision_pair: null, collision_pair_count: 0, max_penetration_depth_m: null, error: null, sampled_path: {{ status: "PASS", max_joint_step_rad: 0.05, segment_count: 1, generated_interior_sample_count: 1, checked_interior_sample_count: 1, failed_interior_sample_count: 0, first_failed_sample: null, collision_pair_count: 0, first_collision_pair: null, max_penetration_depth_m: null, error: null }} }} }});
const passed = await pending;
loadPlannedTrajectory(valid, "TEST");
handler = async () => ({{ validation: {{ status: "PASS", checked_point_count: 2, failed_point_count: 0, first_failed_point: null, first_collision_pair: null, collision_pair_count: 0, max_penetration_depth_m: null, error: null, sampled_path: {{ status: "FAIL", max_joint_step_rad: 0.05, segment_count: 1, generated_interior_sample_count: 1, checked_interior_sample_count: 1, failed_interior_sample_count: 1, first_failed_sample: {{ segment_index: 0, sample_index: 1, subdivision_count: 2, alpha: 0.5, time_from_start_s: 0.5, contacts: [{{ body_1: "left_J6", body_2: "right_J2", depth_m: 0.003128 }}] }}, collision_pair_count: 1, first_collision_pair: {{ body_1: "left_J6", body_2: "right_J2" }}, max_penetration_depth_m: 0.003128, error: null }} }} }});
const failed = await validateLoadedTrajectory(0.05);
const ui = {{ overall: elements.get("digitalTwinValidationOverall").textContent, segment: elements.get("digitalTwinValidationSampledFirstSegment").textContent, sample: elements.get("digitalTwinValidationSampledFirstSample").textContent, alpha: elements.get("digitalTwinValidationSampledFailureAlpha").textContent, time: elements.get("digitalTwinValidationSampledFailureTime").textContent, depth: elements.get("digitalTwinValidationSampledMaxPenetration").textContent }};
console.log(JSON.stringify({{ invalidStep, callsAfterInvalidStep, checking, passed, failed, ui, calls, optionsSeen }}));
}})().catch((error) => {{ console.error(error); process.exitCode = 1; }});
'''
        result = run_node(harness)
        self.assertEqual(result["invalidStep"]["sampledPathStatus"], "ERROR")
        self.assertEqual(result["callsAfterInvalidStep"], 0)
        self.assertEqual(result["checking"]["moveitStateValidity"], "CHECKING")
        self.assertEqual(result["checking"]["sampledPathStatus"], "CHECKING")
        self.assertEqual(result["passed"]["sampledPathStatus"], "PASS")
        self.assertTrue(result["passed"]["valid"])
        self.assertEqual(result["failed"]["sampledPathStatus"], "FAIL")
        self.assertFalse(result["failed"]["valid"])
        self.assertEqual(result["ui"]["overall"], "INVALID — SAMPLED PATH COLLISION DETECTED")
        self.assertEqual(result["ui"]["segment"], "Segment 1 of 1")
        self.assertEqual(
            result["ui"]["sample"],
            "Interior Sample 1 of 1 (subdivision k=1)",
        )
        self.assertEqual(result["ui"]["alpha"], "0.500")
        self.assertEqual(result["ui"]["time"], "0.500 s")
        self.assertEqual(result["ui"]["depth"], "3.128 mm (0.003128 m)")
        self.assertEqual(result["calls"], 2)
        self.assertEqual(
            result["optionsSeen"]["sampledPath"],
            {"enabled": True, "max_joint_step_rad": 0.05},
        )

    def test_confirmed_runtime_fixture_load_only_and_validation_button_lifecycle(self):
        harness = f'''
import {{ DEFAULT_STALE_TIMEOUT_MS, isNormalizedSnapshotStale, normalizeDualArmStatusSnapshot }} from "./adapter.mjs";
import {{ maxAbsoluteJointDelta, normalizePlannedDualArmPose }} from "./planned.mjs";
import {{ normalizeDualArmTrajectory, sampleTrajectoryAtTime, trajectoryDurationSeconds, trajectoryPointAtIndex }} from "./trajectory.mjs";
import {{ validateTrajectoryJointLimits }} from "./validator.mjs";
import {{ DUAL_JAKA_A12_JOINT_LIMIT_METADATA }} from "./metadata.mjs";
const elements = new Map();
globalThis.document = {{ getElementById(id) {{ if (!elements.has(id)) elements.set(id, {{ textContent: "", value: id === "digitalTwinSampledPathMaxJointStep" ? "0.05" : "", disabled: false, setAttribute() {{}} }}); return elements.get(id); }} }};
globalThis.performance = {{ now() {{ return 0; }} }};
globalThis.requestAnimationFrame = () => 1;
globalThis.cancelAnimationFrame = () => {{}};
let validationCalls = 0;
async function requestMoveItTrajectoryValidation() {{ validationCalls += 1; throw new Error("must not run"); }}
{self.controller}
updateTrajectoryPreviewUi();
const initiallyDisabled = {{ full: elements.get("digitalTwinValidateStoredPoints").disabled, limits: elements.get("digitalTwinValidateJointLimits").disabled }};
loadPlannedTrajectory(CONFIRMED_SAMPLED_COLLISION_TRAJECTORY, "CONFIRMED MOVEIT RUNTIME FIXTURE — NOT FOR ROBOT EXECUTION");
const loaded = getTrajectoryPreviewState();
const validationAfterLoad = getTrajectoryValidationState();
const enabledAfterLoad = {{ full: !elements.get("digitalTwinValidateStoredPoints").disabled, limits: !elements.get("digitalTwinValidateJointLimits").disabled }};
clearPlannedTrajectory();
const disabledAfterClear = {{ full: elements.get("digitalTwinValidateStoredPoints").disabled, limits: elements.get("digitalTwinValidateJointLimits").disabled }};
console.log(JSON.stringify({{ initiallyDisabled, loaded, validationAfterLoad, enabledAfterLoad, disabledAfterClear, validationCalls }}));
'''
        result = run_node(harness)
        self.assertEqual(result["initiallyDisabled"], {"full": True, "limits": True})
        self.assertEqual(result["enabledAfterLoad"], {"full": True, "limits": True})
        self.assertEqual(result["disabledAfterClear"], {"full": True, "limits": True})
        self.assertEqual(result["validationCalls"], 0)
        self.assertEqual(result["validationAfterLoad"]["status"], "NOT_VALIDATED")
        self.assertEqual(result["loaded"]["source"], "CONFIRMED MOVEIT RUNTIME FIXTURE — NOT FOR ROBOT EXECUTION")
        self.assertEqual(result["loaded"]["trajectory"], {
            "name": "Confirmed Sampled Collision — MoveIt Runtime Fixture",
            "points": [
                {
                    "time_from_start_s": 0,
                    "left": [-1.757703, -0.355475, 0.115155, -1.264532, -2.483733, -2.463276],
                    "right": [2.017201, -1.186278, -0.911243, 1.576992, 1.103272, -0.331881],
                },
                {
                    "time_from_start_s": 2,
                    "left": [0.278808, 1.515081, -0.391248, -1.921538, 2.082136, 1.900224],
                    "right": [-0.736672, -2.398909, 0.073763, -0.023872, 1.615867, 0.576897],
                },
            ],
        })

    def test_ui_scope_mock_label_and_no_automatic_or_execute_path(self):
        for text in (
            "Between-point sampled validation configuration",
            "Sampled Path Status", "Max Joint Step", "Segment Count",
            "Generated Interior Samples", "Checked Interior Samples",
            "Failed Interior Samples", "First Failed Segment", "First Failed Sample",
            "Failure Alpha", "Failure Trajectory Time", "First Sampled Collision Pair",
            "Sampled Collision Pair Count", "Maximum Sampled Penetration",
            'value="0.05"', "0.05 rad ≈ 2.865°",
            "DISCRETE SAMPLED CHECK — NOT A CONTINUOUS COLLISION GUARANTEE",
            "DYNAMICS NOT VALIDATED", "PHYSICAL EXECUTION NOT VALIDATED",
            "SAMPLED-PATH MOCK TEST",
            "Stored-Point MoveIt Validity",
            "Stored-Point Collision Check",
            "Stored Points Checked by MoveIt",
            "Stored Points Failed by MoveIt",
            "First Failed Stored Point",
            "First Stored-Point Collision Pair",
            "Stored-Point Collision Pair Count",
            "Maximum Stored-Point Penetration",
            "Load Confirmed Sampled Collision Test — MOVEIT RUNTIME FIXTURE",
            "CONFIRMED MOVEIT RUNTIME FIXTURE — NOT FOR ROBOT EXECUTION",
            "LOAD ONLY; PRESS VALIDATE FULL SAMPLED PATH MANUALLY",
        ):
            self.assertIn(text, self.html)
        self.assertIn('id="digitalTwinValidateStoredPoints" type="button" disabled', self.html)
        self.assertIn('id="digitalTwinValidateJointLimits" type="button" disabled', self.html)
        fixture_binding = self.source.split("function bindTrajectoryValidationControls", 1)[1].split(
            "function countVisualMeshes", 1
        )[0]
        self.assertIn("CONFIRMED_SAMPLED_COLLISION_TRAJECTORY", fixture_binding)
        self.assertNotIn("validateLoadedTrajectory()", fixture_binding.split(
            "digitalTwinConfirmedSampledCollisionTest", 1
        )[1])
        self.assertNotIn(">Execute<", self.html)
        lifecycle = self.source.split("function loadPlannedTrajectory", 1)[1].split(
            "function trajectoryValidationStateSnapshot", 1
        )[0]
        self.assertIn("clearTrajectoryValidation()", lifecycle)
        playback = lifecycle.split("function advanceTrajectoryPlayback", 1)[1]
        self.assertNotIn("requestMoveItTrajectoryValidation", playback)
        self.assertNotIn("validateLoadedTrajectory()", playback)


if __name__ == "__main__":
    unittest.main()
