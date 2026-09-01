"""Offline Web and backend-boundary tests for Phase-4C."""

from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from html.parser import HTMLParser
from pathlib import Path

from dual_arm_app.backend.phase4_unified_validation import (
    REQUIRED_CHECKS as BACKEND_REQUIRED_CHECKS,
)
from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HTML = WEB / "index.html"
CONTROLLER = WEB / "digital_twin.js"
UNIFIED = WEB / "digital_twin_phase4_unified_validation.js"
PHASE4A = WEB / "digital_twin_phase4_validation.js"
PHASE4B = WEB / "digital_twin_phase4_collision.js"
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"


class _Ids(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for name, value in attrs if name == "id")


def run_unified_module(harness: str) -> dict:
    with tempfile.TemporaryDirectory(prefix="phase4c-web-") as temporary:
        directory = Path(temporary)
        (directory / "phase4a.mjs").write_text(
            PHASE4A.read_text(encoding="utf-8"), encoding="utf-8"
        )
        (directory / "phase4b.mjs").write_text(
            PHASE4B.read_text(encoding="utf-8"), encoding="utf-8"
        )
        unified = UNIFIED.read_text(encoding="utf-8").replace(
            '"./digital_twin_phase4_validation.js"', '"./phase4a.mjs"'
        ).replace(
            '"./digital_twin_phase4_collision.js"', '"./phase4b.mjs"'
        )
        (directory / "unified.mjs").write_text(unified, encoding="utf-8")
        (directory / "harness.mjs").write_text(harness, encoding="utf-8")
        result = subprocess.run(
            ["node", str(directory / "harness.mjs")],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode:
            raise AssertionError(result.stderr)
        return json.loads(result.stdout)


class Phase4CWebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML.read_text(encoding="utf-8")
        cls.controller = CONTROLLER.read_text(encoding="utf-8")
        cls.unified = UNIFIED.read_text(encoding="utf-8")

    def test_unified_ui_has_each_required_section_once(self):
        ids = (
            "digitalTwinPhase4COverall", "digitalTwinPhase4CExecutionReady",
            "digitalTwinPhase4CScopeStatus",
            "digitalTwinPhase4CPlanFingerprint", "digitalTwinPhase4CIk",
            "digitalTwinPhase4CJointLimits", "digitalTwinPhase4CSelfCollision",
            "digitalTwinPhase4CInterArmCollision", "digitalTwinPhase4CObjectCollision",
            "digitalTwinPhase4CEnvironmentCollision", "digitalTwinPhase4CSampledCollision",
            "digitalTwinPhase4CStartTransition", "digitalTwinPhase4CContinuity",
            "digitalTwinPhase4CJump", "digitalTwinPhase4CFk",
            "digitalTwinPhase4CRelativePose", "digitalTwinPhase4CFixedGrasp",
            "digitalTwinPhase4CTimeline", "digitalTwinPhase4CVelocity",
            "digitalTwinPhase4CAcceleration", "digitalTwinPhase4CFirstCheck",
            "digitalTwinPhase4CFailureReason", "digitalTwinPhase4CFailureLocation",
            "digitalTwinPhase4CCollision", "digitalTwinPhase4CJumpDetails",
            "digitalTwinPhase4CRelativeDetails", "digitalTwinPhase4CDynamicsDetails",
            "digitalTwinPhase4CBlockingReasons",
            "digitalTwinPhase4CDeferredWarnings",
        )
        for element_id in ids:
            with self.subTest(element_id=element_id):
                self.assertEqual(self.html.count(f'id="{element_id}"'), 1)
        for label in (
            "UNIFIED VALIDATION REPORT", "REQUIRED VALIDATION",
            "DEFERRED VALIDATION", "READY UNDER CURRENT PROJECT SCOPE",
            "OBJECT/ENVIRONMENT COLLISION AND ACCELERATION HARD-LIMIT VALIDATION",
            "ARE DEFERRED AND REMAIN UNVALIDATED", "NOT SAFETY CERTIFICATION",
        ):
            self.assertIn(label, self.html)
        self.assertIn(
            '`DEFERRED — ${unifiedCheckStatus(report, name)}`', self.controller
        )

    def test_frontend_required_checks_match_backend_membership_and_order(self):
        output = run_unified_module(r'''
import { PHASE4_REQUIRED_CHECKS } from "./unified.mjs";
console.log(JSON.stringify({requiredChecks: [...PHASE4_REQUIRED_CHECKS]}));
''')
        frontend = output["requiredChecks"]
        backend = list(BACKEND_REQUIRED_CHECKS)
        self.assertCountEqual(frontend, backend)
        self.assertEqual(frontend, backend)

    def test_web_gate_is_strict_for_missing_running_incomplete_fail_stale_and_match(self):
        output = run_unified_module(r'''
import { PHASE4_REQUIRED_CHECKS, PHASE4_UNIFIED_REPORT_VERSION,
  phase4ExecutionGateState } from "./unified.mjs";
const checks = Object.fromEntries(PHASE4_REQUIRED_CHECKS.map(name => [name, {status:"PASS"}]));
checks.object_collision = {status:"NOT_CONFIGURED"};
checks.environment_collision = {status:"NOT_CONFIGURED"};
checks.acceleration = {status:"LIMIT_UNAVAILABLE"};
const scope_policy = {required_checks:[...PHASE4_REQUIRED_CHECKS], deferred_checks:[
  {check:"object_collision",status:"NOT_CONFIGURED"},
  {check:"environment_collision",status:"NOT_CONFIGURED"},
  {check:"acceleration",status:"LIMIT_UNAVAILABLE"},
]};
const pass = {report_version:PHASE4_UNIFIED_REPORT_VERSION,
  plan_fingerprint:"abc", overall_status:"PASS", checks, scope_policy,
  execution_gate:{execution_ready:true, execution_ready_label:"YES"}};
const incomplete = JSON.parse(JSON.stringify(pass)); incomplete.overall_status = "INCOMPLETE";
incomplete.checks.start_transition.status = "NOT_EVALUATED";
incomplete.execution_gate = {execution_ready:false, execution_ready_label:"NO"};
const failed = JSON.parse(JSON.stringify(pass)); failed.overall_status = "FAIL";
failed.checks.self_collision.status = "FAIL";
failed.execution_gate = {execution_ready:false, execution_ready_label:"NO"};
const badScope = JSON.parse(JSON.stringify(pass)); badScope.scope_policy.required_checks.pop();
console.log(JSON.stringify({
  missing: phase4ExecutionGateState({currentPlanFingerprint:"abc"}),
  running: phase4ExecutionGateState({report:pass,currentPlanFingerprint:"abc",validationRunning:true}),
  incomplete: phase4ExecutionGateState({report:incomplete,currentPlanFingerprint:"abc"}),
  failed: phase4ExecutionGateState({report:failed,currentPlanFingerprint:"abc"}),
  stale: phase4ExecutionGateState({report:pass,currentPlanFingerprint:"different",stale:true}),
  deferred: phase4ExecutionGateState({report:pass,currentPlanFingerprint:"abc"}),
  badScope: phase4ExecutionGateState({report:badScope,currentPlanFingerprint:"abc"}),
  accepted: phase4ExecutionGateState({report:pass,currentPlanFingerprint:"abc"}),
}));
''')
        for name in ("missing", "running", "incomplete", "failed", "stale", "badScope"):
            self.assertFalse(output[name]["executionReady"], name)
        self.assertTrue(output["deferred"]["executionReady"])
        self.assertTrue(output["accepted"]["executionReady"])
        self.assertEqual(output["accepted"]["executionReadyLabel"], "YES")
        self.assertIn("DEFERRED CHECKS REMAIN UNVALIDATED", output["accepted"]["semantic"])

    def test_unified_transport_posts_one_combined_request(self):
        output = run_unified_module(r'''
import { buildPhase4UnifiedValidationRequest, requestPhase4UnifiedValidation,
  PHASE4_REQUIRED_CHECKS, PHASE4_UNIFIED_REPORT_VERSION } from "./unified.mjs";
const plan = {ok:true};
const request = buildPhase4UnifiedValidationRequest({
  plan, startState:{left:[0,0,0,0,0,0],right:[0,0,0,0,0,0],source:"TEST"},
  maxJointStepRad:0.04,
});
const checks = Object.fromEntries(PHASE4_REQUIRED_CHECKS.map(name => [name,{status:"PASS"}]));
const scope_policy = {required_checks:[...PHASE4_REQUIRED_CHECKS],deferred_checks:[]};
let call = null;
async function main() {
  const report = await requestPhase4UnifiedValidation(request, async (url, options) => {
    call = {url, method:options.method, body:JSON.parse(options.body)};
    return {ok:true, async json(){return {ok:true,report:{
      report_version:PHASE4_UNIFIED_REPORT_VERSION,plan_fingerprint:"abc",
      overall_status:"PASS",checks,scope_policy,
      execution_gate:{execution_ready:true,execution_ready_label:"YES"}}};}};
  });
  console.log(JSON.stringify({request,call,report}));
}
main();
''')
        self.assertEqual(output["call"]["url"], "/api/digital-twin/validate-phase4")
        self.assertEqual(output["call"]["method"], "POST")
        self.assertEqual(output["request"]["max_joint_step_rad"], 0.04)
        self.assertEqual(output["report"]["plan_fingerprint"], "abc")

    def test_plan_and_start_changes_invalidate_all_results_and_stale_response(self):
        for reason in (
            "OBJECT WAYPOINTS CHANGED", "OBJECT TRANSLATION CHANGED",
            "GLOBAL PLANNING CONFIGURATION CHANGED", "NEW GLOBAL PLAN LOADED",
            "PHASE-4 START STATE CHANGED", "PHASE-4 START STATE CLEARED",
        ):
            self.assertIn(reason, self.controller)
        self.assertIn("phase4UnifiedValidationState.generation", self.controller)
        self.assertIn("generation !== phase4UnifiedValidationState.generation", self.controller)
        invalidator = self.controller.split("function invalidateAllPhase4Validation", 1)[1].split(
            "function clearPhase4UnifiedValidation", 1
        )[0]
        self.assertIn('phase4TrajectoryValidationState.status = "NOT_VALIDATED"', invalidator)
        self.assertIn('phase4CollisionValidationState.status = "NOT_VALIDATED"', invalidator)

    def test_legacy_program_run_control_remains_independent_of_phase4_gate(self):
        self.assertIn(
            'id="runProgramButton" class="green" onclick="runSequence()">Run Program</button>',
            self.html,
        )
        handler = self.html.split("async function runSequence()", 1)[1].split(
            "function updateProgramLoopControls", 1
        )[0]
        self.assertNotIn("getExecutionGateState", handler)
        self.assertNotIn("executionGate.executionReady", handler)
        self.assertNotIn("plan_fingerprint", handler)
        update_ui = self.controller.split("function updatePhase4UnifiedValidationUi", 1)[1].split(
            "function invalidatePhase4UnifiedValidation", 1
        )[0]
        self.assertNotIn("runProgramButton", update_ui)

    def test_stop_controls_are_not_disabled_or_gated(self):
        self.assertGreaterEqual(self.html.count('class="stop" onclick="stopBoth()"'), 4)
        stop = self.html.split("async function stopBoth()", 1)[1].split(
            "function makeHoldButton", 1
        )[0]
        self.assertNotIn("getExecutionGateState", stop)
        self.assertNotIn("runProgramButton", stop)

    def test_public_inspection_api_and_unique_ids(self):
        api = self.controller.split("const publicApi = {", 1)[1].split("};", 1)[0]
        for name in (
            "getPhase4UnifiedValidationState",
            "validateCurrentGlobalPlanPhase4Unified",
            "clearPhase4UnifiedValidation",
            "getExecutionGateState",
        ):
            self.assertIn(name, api)
        parser = _Ids()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))


class Phase4CBackendGateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def route(self, path):
        return next(route.endpoint for route in self.backend.app.routes if route.path == path)

    def test_legacy_program_route_never_queries_phase4_gate(self):
        class FakeNode:
            def __init__(self):
                self.calls = 0

            def phase4_execution_gate_state(self, _fingerprint):
                raise AssertionError("legacy program route must not query Phase-4 gate")

            def run_program(self, *args):
                self.calls += 1
                self.args = args
                return {"ok": True, "fake_only": True}

        fake = FakeNode()
        original = self.backend.node
        self.backend.node = fake
        try:
            request = self.backend.ProgramRunRequest(
                steps=[{"name": "fake", "side": "both", "delay": 0}],
                loop_mode="count",
                loop_count=2,
                plan_fingerprint="legacy-ignored-for-compatibility",
            )
            result = self.route("/api/program/run")(request)
            self.assertTrue(result["fake_only"])
            self.assertEqual(fake.calls, 1)
            self.assertEqual(fake.args[-2:], ("count", 2))
        finally:
            self.backend.node = original

    def test_stop_route_never_queries_phase4_gate(self):
        class FakeNode:
            stop_calls = 0

            def phase4_execution_gate_state(self, _fingerprint):
                raise AssertionError("STOP must not query the execution gate")

            def request_stop_all(self, side):
                self.stop_calls += 1
                return {"ok": True, "side": side}

        fake = FakeNode()
        original = self.backend.node
        self.backend.node = fake
        try:
            result = self.route("/api/stop")(self.backend.StopRequest(side="both"))
            self.assertTrue(result["ok"])
            self.assertEqual(fake.stop_calls, 1)
        finally:
            self.backend.node = original

    def test_new_production_report_modules_have_no_robot_command_invocation(self):
        sources = (
            (ROOT / "dual_arm_app/backend/phase4_unified_validation.py").read_text(encoding="utf-8"),
            UNIFIED.read_text(encoding="utf-8"),
        )
        forbidden = (
            "joint_move(", "linear_move(", "start_jog(", "home(",
            "run_program(", "run_sequence(", "/api/direct/", "/api/jog",
            "/api/home", "/api/program/run", "/api/sequence/run",
        )
        for source in sources:
            for token in forbidden:
                with self.subTest(token=token):
                    self.assertNotIn(token, source)


if __name__ == "__main__":
    unittest.main()
