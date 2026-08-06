"""Offline tests for Phase 1C.2A joint-only feedback integration."""

from __future__ import annotations

import ast
import json
import math
import subprocess
import tempfile
import unittest
from pathlib import Path

from dual_arm_app.backend.joint_feedback import (
    build_digital_twin_joint_status,
    expected_joint_name_aliases,
    normalize_joint_state_message,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = REPOSITORY_ROOT / "dual_arm_app"
BACKEND_PATH = APP_ROOT / "backend/dual_jaka_web_backend.py"
LIVE_SOURCE_PATH = APP_ROOT / "web/digital_twin_live_source.js"
HTML_PATH = APP_ROOT / "web/index.html"


class JointFeedbackNormalizationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.left_aliases = expected_joint_name_aliases("left")

    def normalize(self, names, positions):
        return normalize_joint_state_message(names, positions, self.left_aliases)

    def test_name_mapping_and_shuffled_input(self) -> None:
        ordered = self.normalize(
            [f"left_joint_{index}" for index in range(1, 7)],
            [1, 2, 3, 4, 5, 6],
        )
        shuffled = self.normalize(
            ["joint6", "joint_2", "left_joint_4", "joint1", "joint_5", "joint3"],
            [6, 2, 4, 1, 5, 3],
        )
        self.assertTrue(ordered["valid"])
        self.assertEqual(ordered["mapping"], "name")
        self.assertEqual(ordered["joint"], [1, 2, 3, 4, 5, 6])
        self.assertTrue(shuffled["valid"])
        self.assertEqual(shuffled["joint"], [1, 2, 3, 4, 5, 6])

    def test_missing_duplicate_and_position_count_errors(self) -> None:
        missing = self.normalize(
            ["joint1", "joint2", "joint3", "joint4", "joint5"],
            [1, 2, 3, 4, 5],
        )
        duplicate = self.normalize(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint5"],
            [1, 2, 3, 4, 5, 6],
        )
        too_few_named = self.normalize(
            [f"joint{index}" for index in range(1, 7)],
            [1, 2, 3, 4, 5],
        )
        too_many_unnamed = self.normalize([], [1, 2, 3, 4, 5, 6, 7])
        for result in (missing, duplicate, too_few_named, too_many_unnamed):
            with self.subTest(error=result["error"]):
                self.assertFalse(result["valid"])
                self.assertIsNone(result["joint"])

    def test_empty_names_use_only_exact_six_position_fallback(self) -> None:
        result = self.normalize([], [0, 1, 2, 3, 4, 5])
        self.assertTrue(result["valid"])
        self.assertEqual(result["mapping"], "position_fallback")
        self.assertEqual(result["joint"], [0, 1, 2, 3, 4, 5])

    def test_non_finite_and_boolean_values_are_rejected(self) -> None:
        invalid_values = (math.nan, math.inf, -math.inf, True, False)
        for invalid in invalid_values:
            positions = [0, 1, 2, 3, 4, invalid]
            with self.subTest(value=invalid):
                self.assertFalse(self.normalize([], positions)["valid"])

    def test_inputs_are_not_mutated_and_results_are_copied(self) -> None:
        names = [f"joint{index}" for index in range(1, 7)]
        positions = [1, 2, 3, 4, 5, 6]
        original_names = list(names)
        original_positions = list(positions)
        result = self.normalize(names, positions)
        result["joint"][0] = 99
        result["names"][0] = "changed"
        self.assertEqual(names, original_names)
        self.assertEqual(positions, original_positions)


class JointFeedbackResponseTests(unittest.TestCase):
    @staticmethod
    def valid_cache():
        return {
            "left": {
                "joint": [1, 2, 3, 4, 5, 6],
                "received_at_ms": 900,
                "mapping": "name",
                "error": None,
                "names": [f"left_joint_{index}" for index in range(1, 7)],
            },
            "right": {
                "joint": [-1, -2, -3, -4, -5, -6],
                "received_at_ms": 950,
                "mapping": "position_fallback",
                "error": None,
                "names": [],
            },
        }

    def test_response_requires_both_sides_and_reports_non_negative_age(self) -> None:
        cache = self.valid_cache()
        response = build_digital_twin_joint_status(cache, server_time_ms=1000)
        self.assertTrue(response["ok"])
        self.assertEqual(response["source"], "ros_joint_state_cache")
        self.assertEqual(response["unit"], "radian")
        self.assertEqual(response["left"]["age_ms"], 100)
        self.assertEqual(response["right"]["age_ms"], 50)

        cache["right"]["joint"] = None
        missing_right = build_digital_twin_joint_status(cache, server_time_ms=1000)
        self.assertFalse(missing_right["ok"])
        self.assertFalse(missing_right["right"]["valid"])

        future_cache = self.valid_cache()
        future_cache["left"]["received_at_ms"] = 1100
        future = build_digital_twin_joint_status(future_cache, server_time_ms=1000)
        self.assertEqual(future["left"]["age_ms"], 0)

    def test_response_does_not_expose_mutable_cache_arrays(self) -> None:
        cache = self.valid_cache()
        response = build_digital_twin_joint_status(cache, server_time_ms=1000)
        response["left"]["joint"][0] = 999
        response["left"]["names"][0] = "changed"
        self.assertEqual(cache["left"]["joint"][0], 1)
        self.assertEqual(cache["left"]["names"][0], "left_joint_1")


class BackendJointFeedbackContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = BACKEND_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source)

    def class_method(self, name: str) -> ast.FunctionDef:
        node_class = next(
            item
            for item in self.tree.body
            if isinstance(item, ast.ClassDef) and item.name == "DualJakaWebNode"
        )
        return next(
            item
            for item in node_class.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        )

    def module_function(self, name: str) -> ast.FunctionDef:
        return next(
            item
            for item in self.tree.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        )

    def test_joint_only_method_has_no_fk_service_or_publish_calls(self) -> None:
        method_source = ast.unparse(self.class_method("digital_twin_joint_status"))
        for forbidden in ("get_fk_pose", "call_async", "publish", "create_request"):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, method_source)
        self.assertIn("build_digital_twin_joint_status", method_source)

    def test_endpoint_is_get_only_and_returns_only_joint_status(self) -> None:
        route = self.module_function("api_digital_twin_joints")
        decorators = [ast.unparse(decorator) for decorator in route.decorator_list]
        self.assertEqual(decorators, ["app.get('/api/digital-twin/joints')"])
        self.assertEqual(len(route.body), 1)
        self.assertEqual(
            ast.unparse(route.body[0]),
            "return node.digital_twin_joint_status()",
        )
        self.assertNotIn("node.status", ast.unparse(route))

    def test_existing_status_route_still_calls_node_status(self) -> None:
        route = self.module_function("api_status")
        self.assertEqual(
            [ast.unparse(decorator) for decorator in route.decorator_list],
            ["app.get('/api/status')"],
        )
        self.assertEqual(ast.unparse(route.body[0]), "return node.status()")


class FrontendLiveSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = LIVE_SOURCE_PATH.read_text(encoding="utf-8")
        cls.html = HTML_PATH.read_text(encoding="utf-8")

    def test_static_read_only_source_contract_and_ui(self) -> None:
        self.assertIn('const JOINT_FEEDBACK_ENDPOINT = "/api/digital-twin/joints"', self.source)
        self.assertNotIn("/api/status", self.source)
        self.assertNotIn('method: "POST"', self.source)
        for route in ("/api/jog", "/api/home", "/api/stop", "/api/direct", "/api/waypoint", "/api/program"):
            with self.subTest(route=route):
                self.assertNotIn(route, self.source)
        for label in (
            "Start Live Feedback — READ ONLY",
            "Stop Live Feedback",
            "Live Source State",
            "Poll Interval",
            "Last Fetch",
            "Last Fetch Error",
            "Load Mock Pose A — OFFLINE MOCK",
            "Load Mock Pose B — OFFLINE MOCK",
        ):
            with self.subTest(label=label):
                self.assertIn(label, self.html)
        panel = self.html.split('aria-labelledby="digitalTwinTitle"', 1)[1].split(
            "</section>", 1
        )[0]
        self.assertNotIn(">Execute<", panel)

    def test_polling_lifecycle_and_response_validation(self) -> None:
        harness = r'''
(async () => {
const timers = new Map();
const clearedTimers = [];
let nextTimerId = 1;
let fetchCalls = [];
let fetchImplementation;
let mirrorEnableCalls = [];
let ingested = [];

globalThis.AbortController = class AbortController {
  constructor() {
    const listeners = [];
    this.signal = {
      aborted: false,
      addEventListener(name, callback) {
        if (name === "abort") listeners.push(callback);
      },
    };
    this.abort = () => {
      if (this.signal.aborted) return;
      this.signal.aborted = true;
      listeners.forEach((callback) => callback());
    };
  }
};
globalThis.document = { getElementById() { return null; } };
globalThis.window = {
  setTimeout(callback, delay) {
    const id = nextTimerId++;
    timers.set(id, { callback, delay });
    return id;
  },
  clearTimeout(id) {
    clearedTimers.push(id);
    timers.delete(id);
  },
  dualArmDigitalTwin: {
    setMirrorEnabled(value) { mirrorEnableCalls.push(value); },
    ingestStatusSnapshot(snapshot, timestamp) { ingested.push({ snapshot, timestamp }); },
  },
};
globalThis.fetch = (url, options) => {
  fetchCalls.push({ url, method: options.method, cache: options.cache });
  return fetchImplementation(url, options);
};

const live = await import("./live_source.mjs");
const initial = live.getLiveFeedbackState();
const initialFetchCount = fetchCalls.length;
const validPayload = {
  ok: true,
  left: { valid: true, joint: [1,2,3,4,5,6], received_at_ms: 900 },
  right: { valid: true, joint: [-1,-2,-3,-4,-5,-6], received_at_ms: 950 },
};
fetchImplementation = async () => ({ ok: true, status: 200, json: async () => validPayload });
await live.pollLiveFeedbackOnce();
const validIngest = ingested[0];

fetchImplementation = async () => ({
  ok: true,
  status: 200,
  json: async () => ({ ...validPayload, right: { valid: false } }),
});
await live.pollLiveFeedbackOnce();
const ingestedAfterInvalid = ingested.length;

let resolveStartFetch;
fetchImplementation = (_url, _options) => new Promise((resolve) => { resolveStartFetch = resolve; });
const fetchCountBeforeStart = fetchCalls.length;
live.startLiveFeedback();
live.startLiveFeedback();
const fetchesFromDoubleStart = fetchCalls.length - fetchCountBeforeStart;
resolveStartFetch({ ok: true, status: 200, json: async () => validPayload });
await live.pollLiveFeedbackOnce();
await Promise.resolve();
const pollTimersAfterStart = [...timers.values()].filter(timer => timer.delay === 500).length;
live.stopLiveFeedback();
const timersAfterStop = timers.size;

let abortObserved = false;
fetchImplementation = (_url, options) => new Promise((_resolve, reject) => {
  options.signal.addEventListener("abort", () => {
    abortObserved = true;
    const error = new Error("aborted");
    error.name = "AbortError";
    reject(error);
  });
});
live.startLiveFeedback();
const inFlight = live.pollLiveFeedbackOnce();
live.stopLiveFeedback();
await inFlight;

console.log(JSON.stringify({
  initial,
  initialFetchCount,
  firstFetch: fetchCalls[0],
  validIngest,
  ingestedAfterInvalid,
  fetchesFromDoubleStart,
  mirrorEnableCalls,
  pollTimersAfterStart,
  timersAfterStop,
  abortObserved,
  finalState: live.getLiveFeedbackState(),
}));
})().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
'''
        with tempfile.TemporaryDirectory(prefix="phase1c2a-") as temporary_directory:
            directory = Path(temporary_directory)
            (directory / "live_source.mjs").write_text(self.source, encoding="utf-8")
            (directory / "harness.mjs").write_text(harness, encoding="utf-8")
            try:
                result = subprocess.run(
                    ["node", str(directory / "harness.mjs")],
                    check=True,
                    capture_output=True,
                    text=True,
                )
            except subprocess.CalledProcessError as error:
                self.fail(error.stderr)
        output = json.loads(result.stdout)
        self.assertFalse(output["initial"]["running"])
        self.assertEqual(output["initial"]["status"], "STOPPED")
        self.assertEqual(output["initial"]["pollIntervalMs"], 500)
        self.assertEqual(output["initialFetchCount"], 0)
        self.assertEqual(output["firstFetch"], {
            "url": "/api/digital-twin/joints",
            "method": "GET",
            "cache": "no-store",
        })
        self.assertEqual(output["validIngest"]["timestamp"], 900)
        self.assertEqual(len(output["validIngest"]["snapshot"]["left"]["joint"]), 6)
        self.assertEqual(output["ingestedAfterInvalid"], 1)
        self.assertEqual(output["fetchesFromDoubleStart"], 1)
        self.assertEqual(output["mirrorEnableCalls"], [True, True])
        self.assertEqual(output["pollTimersAfterStart"], 1)
        self.assertEqual(output["timersAfterStop"], 0)
        self.assertTrue(output["abortObserved"])
        self.assertFalse(output["finalState"]["running"])
        self.assertEqual(output["finalState"]["status"], "STOPPED")


if __name__ == "__main__":
    unittest.main()
