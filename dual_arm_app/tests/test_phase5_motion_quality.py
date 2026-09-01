"""Offline tests for Phase-5 actual-start, replan, and YAML filter contracts."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import unittest

from dual_arm_app.backend.phase5_motion_quality import (
    REPLAN_FROM_CURRENT_SOURCE,
    evaluate_actual_start_match,
    fresh_actual_joint_state,
    normalize_phase5_motion_settings,
)


def artifact(initial=0.0):
    return SimpleNamespace(
        left_positions_rad=((initial,) * 6, (initial + 0.1,) * 6),
        right_positions_rad=((initial,) * 6, (initial - 0.1,) * 6),
    )


def actual(left=0.0, right=0.0, age_ms=10):
    return {
        "left": {"valid": True, "joint": [left] * 6, "age_ms": age_ms},
        "right": {"valid": True, "joint": [right] * 6, "age_ms": age_ms},
    }


class Phase5ActualStartTests(unittest.TestCase):
    def test_match_and_delta_threshold_are_inclusive(self):
        result = evaluate_actual_start_match(
            actual(left=0.01, right=-0.01), artifact(), threshold_rad=0.01
        )
        self.assertTrue(result["match"])
        self.assertEqual(result["state"], "READY")
        self.assertAlmostEqual(result["max_delta_rad"], 0.01)
        self.assertEqual(result["threshold_rad"], 0.01)

    def test_mismatch_invalid_and_stale_feedback_fail_closed(self):
        mismatch = evaluate_actual_start_match(
            actual(left=0.010001), artifact(), threshold_rad=0.01
        )
        self.assertFalse(mismatch["match"])
        self.assertEqual(mismatch["state"], "MISMATCH")
        self.assertIn("EXCEEDS_THRESHOLD", mismatch["reason"])

        invalid = actual()
        invalid["right"]["valid"] = False
        blocked = evaluate_actual_start_match(invalid, artifact())
        self.assertFalse(blocked["match"])
        self.assertIn("RIGHT_ACTUAL_FEEDBACK_INVALID", blocked["reason"])

        stale = evaluate_actual_start_match(
            actual(age_ms=501), artifact(), max_age_ms=500
        )
        self.assertFalse(stale["match"])
        self.assertIn("STALE", stale["reason"])

    def test_replan_snapshot_is_fresh_copy_with_exact_source(self):
        source = actual(left=0.2, right=-0.3)
        captured = fresh_actual_joint_state(source, max_age_ms=500)
        self.assertTrue(captured["ok"])
        self.assertEqual(captured["source"], REPLAN_FROM_CURRENT_SOURCE)
        source["left"]["joint"][0] = 9.0
        self.assertEqual(captured["initial_joint_state_rad"]["left"][0], 0.2)


class Phase5ConfigurationTests(unittest.TestCase):
    def test_defaults_are_conservative_and_restore_legacy_foresight(self):
        settings = normalize_phase5_motion_settings({})
        self.assertEqual(settings["start_match_threshold_rad"], 0.01)
        self.assertLessEqual(settings["recovery_joint_vel_rad_s"], 0.1)
        self.assertLessEqual(settings["recovery_joint_acc_rad_s2"], 0.2)
        self.assertEqual(settings["servo_step_num"], 1)
        self.assertEqual(settings["servo_filter"]["mode"], "LEGACY_FORESIGHT")
        self.assertEqual(settings["servo_filter"]["legacy_max_buf"], 15)
        self.assertEqual(settings["servo_filter"]["legacy_kp"], 0.03)

    def test_lpf_nlf_validate_and_invalid_tuning_fails_closed(self):
        lpf = normalize_phase5_motion_settings(
            {"phase5_servo_filter": {"mode": "LPF", "cutoff_hz": 8.0}}
        )
        self.assertEqual(lpf["servo_filter"]["lpf_cutoff_hz"], 8.0)
        nlf = normalize_phase5_motion_settings(
            {
                "phase5_servo_filter": {
                    "mode": "NLF",
                    "max_velocity_deg_s": 20.0,
                    "max_acceleration_deg_s2": 100.0,
                    "max_jerk_deg_s3": 500.0,
                }
            }
        )
        self.assertEqual(nlf["servo_filter"]["mode"], "NLF")
        with self.assertRaises(ValueError):
            normalize_phase5_motion_settings(
                {"phase5_servo_filter": {"mode": "LPF", "cutoff_hz": 0.0}}
            )

    def test_step_num_accepts_only_project_supported_vendor_values(self):
        for supported in (1, 2, 3, 4):
            with self.subTest(supported=supported):
                self.assertEqual(
                    normalize_phase5_motion_settings({"phase5_servo_step_num": supported})[
                        "servo_step_num"
                    ],
                    supported,
                )
        for invalid in (0, 5, True, 1.0):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                normalize_phase5_motion_settings(
                    {"phase5_servo_step_num": invalid}
                )


class Phase5RecoveryMemoryTests(unittest.TestCase):
    def test_last_executed_sample_zero_survives_new_frozen_artifact(self):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import

        backend, cleanup = _mocked_backend_import()
        try:
            node = backend.node
            first = SimpleNamespace(
                left_positions_rad=((1.0, 1.1, 1.2, 1.3, 1.4, 1.5),),
                right_positions_rad=((-1.0, -1.1, -1.2, -1.3, -1.4, -1.5),),
                artifact_fingerprint="fp-first",
                trajectory_name="first-run",
            )
            second = SimpleNamespace(
                left_positions_rad=((2.0,) * 6,),
                right_positions_rad=((-2.0,) * 6,),
                artifact_fingerprint="fp-second",
                trajectory_name="replanned-run",
            )
            node._phase5_artifact_snapshot = lambda: (first, 7, None)
            node.phase5_execution_coordinator.execute = lambda operator_confirmed: {
                "ok": True,
                "accepted": True,
                "execution": {
                    "artifact_fingerprint": "fp-first",
                    "artifact_generation": 7,
                },
            }
            node.execute_phase5_execution(True)
            self.assertEqual(node.phase5_recovery_initial["artifact_fingerprint"], "fp-first")

            node._phase5_artifact_snapshot = lambda: (second, 8, None)
            node.phase5_execution_coordinator.is_active = lambda: False
            node.safe_state_ok = lambda _side: (True, "safe")
            node._phase5_legacy_conflicts = lambda: []
            node.digital_twin_ros_joint_status = lambda: actual(left=0.2, right=-0.2)
            captured = {}
            node.move_both_joint = lambda **kwargs: captured.update(kwargs) or {
                "ok": True, "accepted": True
            }
            result = node.phase5_move_to_initial()
            self.assertTrue(result["ok"])
            self.assertEqual(result["initial_target_source"], "LAST_EXECUTED_FROZEN_ARTIFACT_SAMPLE_0")
            self.assertEqual(captured["left_target"], list(first.left_positions_rad[0]))
            self.assertEqual(captured["right_target"], list(first.right_positions_rad[0]))
            self.assertNotEqual(captured["left_target"], list(second.left_positions_rad[0]))
        finally:
            cleanup()


class Phase5RecoveryWebContractTests(unittest.TestCase):
    def test_replan_from_current_is_one_click_plan_validate_freeze_without_execute(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "dual_arm_app/web/digital_twin_phase5_execution.js").read_text()
        handler = source.split("async function replanFromCurrent()", 1)[1].split(
            "async function moveToInitial()", 1
        )[0]
        self.assertIn("setNextPlanningStartOverride", handler)
        self.assertIn("await digitalTwin.planObjectGlobal()", handler)
        self.assertIn("await digitalTwin.validateCurrentGlobalPlanPhase4Unified()", handler)
        self.assertIn("REPLAN FROM CURRENT COMPLETE — NO MOTION", handler)
        self.assertNotIn("executePhase5RealMotion", handler)
        self.assertNotIn("EXECUTE_ENDPOINT", handler)

        main = (root / "dual_arm_app/web/digital_twin.js").read_text()
        import_block = main.split('from "./digital_twin_phase3_planning.js";', 1)[0].rsplit("import {", 1)[1]
        self.assertIn("normalizePlanningStartState", import_block)
        self.assertIn("const normalized = normalizePlanningStartState(values);", main)

        html = (root / "dual_arm_app/web/index.html").read_text()
        phase5 = html.split('id="digitalTwinPhase5ExecutionSection"', 1)[1].split("</section>", 1)[0]
        primary = phase5.split('aria-label="Phase 5 primary operator controls"', 1)[1].split("</div>", 1)[0]
        secondary = phase5.split("More preview / status controls", 1)[1].split("</details>", 1)[0]
        for control in (
            "digitalTwinPhase5PreviewPlay",
            "digitalTwinPhase5ReplanFromCurrent",
            "digitalTwinPhase5MoveToInitial",
            "digitalTwinPhase5Stop",
            "digitalTwinPhase5Prepare",
        ):
            self.assertIn(control, primary)
        for control in (
            "digitalTwinPhase5PreviewReload",
            "digitalTwinPhase5PreviewReset",
            "digitalTwinPhase5Refresh",
        ):
            self.assertNotIn(control, primary)
            self.assertIn(control, secondary)

    def test_replan_refreshes_authority_before_initial_gate_and_surfaces_operator_action(self):
        root = Path(__file__).resolve().parents[2]
        digital_twin = (root / "dual_arm_app/web/digital_twin.js").read_text()
        phase5 = (root / "dual_arm_app/web/digital_twin_phase5_execution.js").read_text()
        html = (root / "dual_arm_app/web/index.html").read_text()

        planner = digital_twin.split("async function planObjectGlobal", 1)[1].split(
            "function bindObjectWaypointPlanningControls", 1
        )[0]
        refresh_index = planner.index("await loadWorldCalibrationState(fetchImpl)")
        grasp_index = planner.index("await requestGraspConfiguration(fetchImpl)")
        gate_index = planner.index("phase3PlanningAuthorityIdentity()")
        self.assertLess(refresh_index, gate_index)
        self.assertLess(grasp_index, gate_index)
        self.assertIn("getObjectWaypointPlanningState", phase5)
        self.assertIn("OBJECT PLANNING WAYPOINTS REQUIRED", phase5)
        self.assertIn("REPLAN FAILED —", phase5)
        self.assertIn('id="digitalTwinPhase5OperatorAction"', html)
        self.assertIn("Last Operator Action", html)

    def test_phase5_controls_self_bootstrap_idempotently_before_main_initialize(self):
        root = Path(__file__).resolve().parents[2]
        source = (root / "dual_arm_app/web/digital_twin_phase5_execution.js").read_text()
        self.assertIn("let controlsBound = false", source)
        self.assertIn("if (controlsBound) return", source)
        self.assertIn('if (!element("digitalTwinPhase5ExecutionSection")) return', source)
        self.assertIn('document.addEventListener("DOMContentLoaded", bindPhase5ExecutionControls', source)
        self.assertIn("bootstrapPhase5ExecutionControls();", source)
        bootstrap = source.split("function bootstrapPhase5ExecutionControls()", 1)[1]
        self.assertNotIn('method: "POST"', bootstrap)
        self.assertNotIn("executePhase5RealMotion()", bootstrap)


if __name__ == "__main__":
    unittest.main()
