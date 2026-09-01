"""Offline acceptance coverage for Feature 7's simple execute confirmation."""

from __future__ import annotations

from html import unescape
from pathlib import Path
import re
import unittest

from dual_arm_app.tests.test_phase5_operator_execution import make_coordinator


ROOT = Path(__file__).resolve().parents[2]
HTML_PATH = ROOT / "dual_arm_app/web/index.html"
CONTROLLER_PATH = ROOT / "dual_arm_app/web/digital_twin_phase5_execution.js"
COORDINATOR_PATH = ROOT / "dual_arm_app/backend/phase5_execution_coordinator.py"
BACKEND_PATH = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"


class Feature7SimpleExecuteConfirmationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.coordinator_source = COORDINATOR_PATH.read_text(encoding="utf-8")
        cls.backend_source = BACKEND_PATH.read_text(encoding="utf-8")
        cls.phase5 = cls.html.split(
            'id="digitalTwinPhase5ExecutionSection"', 1
        )[1].split("</section>", 1)[0]

    def test_no_user_visible_token_expiry_or_countdown_contract(self) -> None:
        user_facing = self.html + self.controller
        for removed in (
            "Token Expiry",
            "confirmation_token",
            "expires_at_unix_ns",
            "countdown",
        ):
            self.assertNotIn(removed, user_facing)
        for removed in (
            "confirmation_token",
            "expires_at_unix_ns",
            "CONFIRMATION_TTL_S",
            "confirmation_ttl_s",
            "token_factory",
        ):
            self.assertNotIn(removed, self.coordinator_source)

    def test_step3_has_one_green_real_motion_execute_control(self) -> None:
        self.assertEqual(self.phase5.count('id="digitalTwinPhase5Prepare"'), 1)
        tag = re.search(
            r'<button id="digitalTwinPhase5Prepare"([^>]*)>(.*?)</button>',
            self.phase5,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(tag)
        self.assertIn("digital-twin-primary-action", tag.group(1))
        self.assertIn("Execute — REAL MOTION", " ".join(tag.group(2).split()))
        for removed_id in (
            "digitalTwinPhase5Execute",
            "digitalTwinPhase5OperatorConfirmed",
            "digitalTwinPhase5ConfirmationExpiry",
            "digitalTwinPhase5ConfirmationPanel",
        ):
            self.assertNotIn(removed_id, self.phase5)

    def test_click_path_prepares_confirms_then_executes_explicit_boolean(self) -> None:
        handler = self.controller.split(
            "async function executePhase5RealMotion", 1
        )[1].split("async function replanFromCurrent", 1)[0]
        prepare_index = handler.index("requestJson(PREPARE_ENDPOINT")
        confirm_index = handler.index("window.confirm")
        execute_index = handler.index("requestJson(EXECUTE_ENDPOINT")
        self.assertLess(prepare_index, confirm_index)
        self.assertLess(confirm_index, execute_index)
        self.assertIn("prepared.motion_dispatched !== false", handler)
        self.assertIn("if (!confirmed)", handler)
        self.assertIn("operator_confirmed: true", handler)
        self.assertIn(
            'prepareButton.addEventListener("click", executePhase5RealMotion)',
            self.controller,
        )
        automatic = self.controller.split(
            "// Automatic work is read-only GET status + Frozen Ghost preview only", 1
        )[1]
        self.assertNotIn('method: "POST"', automatic)

    def test_prepare_is_no_motion_and_execute_remains_motion_classified(self) -> None:
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import

        backend, cleanup = _mocked_backend_import()
        try:
            classify = backend._d33_is_motion_command
            self.assertFalse(classify("/api/digital-twin/phase5/prepare", "POST"))
            self.assertTrue(classify("/api/digital-twin/phase5/execute", "POST"))
            self.assertTrue(
                backend.DigitalTwinPhase5ExecuteRequest(
                    operator_confirmed=True
                ).operator_confirmed
            )
            with self.assertRaises(Exception):
                backend.DigitalTwinPhase5ExecuteRequest()
            for invalid in (1, "true", None):
                with self.subTest(invalid=invalid), self.assertRaises(Exception):
                    backend.DigitalTwinPhase5ExecuteRequest(
                        operator_confirmed=invalid
                    )
        finally:
            cleanup()
        self.assertIn("motion_dispatched", self.coordinator_source)
        self.assertIn("operator_confirmed: bool", self.backend_source)
        self.assertIn("operator_confirmed: bool = Field(..., strict=True)", self.backend_source)

    def test_no_expiry_false_confirm_and_single_use_fail_closed(self) -> None:
        coordinator, _state, clock, transport, _aborts = make_coordinator()
        prepared = coordinator.prepare()
        self.assertTrue(prepared["ok"])
        clock.now_ns += 10 * 365 * 24 * 60 * 60 * 1_000_000_000
        inspection = coordinator.inspection_state()["operator_authority"]
        self.assertTrue(inspection["pending"])
        self.assertTrue(inspection["single_use"])
        self.assertTrue(inspection["no_expiry"])
        self.assertNotIn("expires_at_unix_ns", inspection)
        self.assertEqual(
            coordinator.execute(False)["error"], "OPERATOR_CONFIRMATION_REQUIRED"
        )
        self.assertEqual(
            coordinator.execute(True)["error"],
            "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED",
        )
        self.assertEqual(transport.calls, [])

    def test_true_confirmation_dispatches_once_after_unbounded_wait(self) -> None:
        coordinator, _state, clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        clock.now_ns += 10 * 365 * 24 * 60 * 60 * 1_000_000_000
        self.assertTrue(coordinator.execute(True)["ok"])
        self.assertEqual(len(transport.calls), 1)
        self.assertEqual(
            coordinator.execute(True)["error"],
            "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED",
        )
        self.assertEqual(len(transport.calls), 1)

    def test_active_execution_consumes_authority_and_dispatches_nothing(self) -> None:
        coordinator, _state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        coordinator._execution["state"] = "RUNNING"
        rejected = coordinator.execute(True)
        self.assertEqual(rejected["error"], "PHASE5_EXECUTION_ALREADY_ACTIVE")
        self.assertEqual(transport.calls, [])
        self.assertEqual(
            coordinator.execute(True)["error"],
            "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED",
        )
        self.assertEqual(transport.calls, [])

    def test_generation_stop_and_invalidation_clear_authority(self) -> None:
        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["artifact"].plan_fingerprint = "plan-2"
        stale = coordinator.execute(True)
        self.assertFalse(stale["ok"])
        self.assertIn("plan_fingerprint", stale["binding_mismatches"])
        self.assertEqual(transport.calls, [])

        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["generation"] += 1
        stale = coordinator.execute(True)
        self.assertFalse(stale["ok"])
        self.assertIn("artifact_generation", stale["binding_mismatches"])
        self.assertEqual(transport.calls, [])

        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["stop"] += 1
        coordinator.on_stop(state["stop"])
        self.assertFalse(coordinator.inspection_state()["operator_authority"]["pending"])
        self.assertEqual(transport.calls, [])

        coordinator, _state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        coordinator.invalidate_authority("VALIDATION_CHANGED")
        self.assertFalse(coordinator.inspection_state()["operator_authority"]["pending"])
        self.assertEqual(transport.calls, [])

    def test_safety_authorities_and_driver_completion_remain_present(self) -> None:
        for preserved in (
            "_phase4_gate_getter",
            "_start_match_checker",
            "_safe_state_checker",
            "_legacy_conflict_getter",
            "artifact_fingerprint",
            "plan_fingerprint",
            "artifact_generation",
            "stop_generation",
            "submit_pair",
            "final_identity",
            "_abort_callback",
            "AUTHORITATIVE MATCHING LEFT + RIGHT DRIVER GetExecutionStatus",
            "WALL TIME NEVER INFERS COMPLETION",
        ):
            self.assertIn(preserved, self.coordinator_source)
        for preserved in (
            "Frozen Ghost / Interlock",
            "Phase 4 Gate",
            "Robot Safe State",
            "Start Match",
            "STOP BOTH — EXISTING STOP PATH",
            "STOP / E-stop",
        ):
            self.assertIn(preserved, self.phase5)

    def test_feature6_three_step_workflow_remains_intact(self) -> None:
        guide = re.search(
            r'<nav class="digital-twin-workflow-guide"[^>]*>(.*?)</nav>',
            self.html,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(guide)
        labels = re.findall(r"<span>(.*?)</span>", guide.group(1), re.DOTALL)
        self.assertEqual(
            [unescape(re.sub(r"<[^>]+>", "", label)).strip() for label in labels],
            ["1. Grasp Setup", "2. Center Waypoints", "3. Preview & Execute"],
        )


if __name__ == "__main__":
    unittest.main()
