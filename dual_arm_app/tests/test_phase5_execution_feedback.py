"""Safe offline acceptance tests for Phase 5 P5.11-P5.15 only."""

from __future__ import annotations

import re
import unittest
from pathlib import Path
from types import SimpleNamespace

from dual_arm_app.backend.phase5_execution_feedback import (
    build_phase5_execution_feedback,
    normalize_driver_execution_status,
)
from dual_arm_app.backend.phase5_execution_coordinator import (
    Phase5ExecutionCoordinator,
)


ROOT = Path(__file__).resolve().parents[2]


def driver_status(side, state, *, trajectory_id="p5-current", progress=0.5, reason=""):
    response = SimpleNamespace(
        valid=True,
        ret=1,
        message="memory snapshot",
        trajectory_id=trajectory_id,
        state=state,
        active=state in {"ARMED", "RUNNING"},
        start_time_unix_ns=1_900_000_000_000_000_000,
        terminal_time_unix_ns=(
            1_900_000_001_000_000_000
            if state in {"COMPLETED", "ABORTED", "FAILED"} else 0
        ),
        duration_s=2.0,
        elapsed_s=2.0 * progress,
        progress_0_to_1=progress,
        sample_index=10,
        sample_count=21,
        terminal_reason=reason,
    )
    return normalize_driver_execution_status(response, side=side)


def aggregate(trajectory_id, left, right, *, left_error=None, right_error=None):
    return build_phase5_execution_feedback(
        expected_trajectory_id=trajectory_id,
        left_response=left,
        right_response=right,
        left_error=left_error,
        right_error=right_error,
    )


class Phase5FeedbackAggregationTests(unittest.TestCase):
    def test_completed_requires_both_driver_confirmations_and_uses_min_progress(self):
        left = driver_status("left", "COMPLETED", progress=1.0, reason="COMPLETED")
        right = driver_status("right", "RUNNING", progress=0.8)
        running = aggregate("p5-current", left, right)
        self.assertEqual(running["combined_state"], "RUNNING")
        self.assertFalse(running["terminal"])
        self.assertEqual(running["common_timeline"]["progress_0_to_1"], 0.8)

        completed = aggregate(
            "p5-current",
            left,
            driver_status("right", "COMPLETED", progress=1.0, reason="COMPLETED"),
        )
        self.assertEqual(completed["combined_state"], "COMPLETED")
        self.assertTrue(completed["terminal"])

    def test_failed_has_priority_over_abort_and_details_are_side_specific(self):
        result = aggregate(
            "p5-current",
            driver_status("left", "ABORTED", reason="CANCELLED"),
            driver_status("right", "FAILED", reason="SERVO_STREAM_FAILED"),
        )
        self.assertEqual(result["combined_state"], "FAILED")
        self.assertIn("LEFT_ABORTED: CANCELLED", result["reason"])
        self.assertIn("RIGHT_FAILED: SERVO_STREAM_FAILED", result["reason"])

    def test_partial_abort_remains_active_until_both_drivers_are_terminal(self):
        result = aggregate(
            "p5-current",
            driver_status("left", "ABORTED", reason="CANCELLED_BEFORE_START"),
            driver_status("right", "RUNNING"),
        )
        self.assertEqual(result["combined_state"], "ABORT_REQUESTED")
        self.assertFalse(result["terminal"])
        self.assertIsNone(result["result"])

        closed = aggregate(
            "p5-current",
            driver_status("left", "ABORTED", reason="CANCELLED_BEFORE_START"),
            driver_status("right", "ABORTED", reason="CANCELLED"),
        )
        self.assertEqual(closed["combined_state"], "ABORTED")
        self.assertTrue(closed["terminal"])

    def test_mismatch_and_unavailable_are_explicit_and_nonterminal(self):
        mismatch = aggregate(
            "p5-current",
            driver_status("left", "COMPLETED", trajectory_id="old-id", progress=1.0),
            driver_status("right", "COMPLETED", progress=1.0),
        )
        self.assertEqual(mismatch["feedback_status"], "MISMATCH")
        self.assertFalse(mismatch["terminal"])

        unavailable = aggregate(
            "p5-current",
            None,
            driver_status("right", "COMPLETED", progress=1.0),
            left_error="service unavailable",
        )
        self.assertEqual(unavailable["feedback_status"], "FEEDBACK_UNAVAILABLE")
        self.assertFalse(unavailable["terminal"])

    def test_common_execution_contract_mismatch_is_fail_closed(self):
        left = driver_status("left", "RUNNING")
        cases = (
            ("COMMON_START_TIME_MISMATCH", "start_time_unix_ns", 1),
            ("DURATION_MISMATCH", "duration_s", 0.25),
            ("SAMPLE_COUNT_MISMATCH", "sample_count", 1),
        )
        for expected_reason, field, delta in cases:
            with self.subTest(field=field):
                right = dict(driver_status("right", "RUNNING"))
                right[field] = right[field] + delta
                result = aggregate("p5-current", left, right)
                self.assertEqual(result["feedback_status"], "MISMATCH")
                self.assertEqual(result["combined_state"], "MISMATCH")
                self.assertFalse(result["terminal"])
                self.assertIn(expected_reason, result["reason"])

    def test_servo_execution_settings_mismatch_is_fail_closed(self):
        def raw_status(side):
            return SimpleNamespace(
                valid=True, ret=1, message="memory snapshot",
                trajectory_id="p5-current", state="RUNNING", active=True,
                start_time_unix_ns=1_900_000_000_000_000_000,
                terminal_time_unix_ns=0, duration_s=2.0, elapsed_s=1.0,
                progress_0_to_1=0.5, sample_index=10, sample_count=21,
                terminal_reason="", servo_step_num=1, command_period_ms=8.0,
                telemetry_mode="PHASE5_PORT10004_SNAPSHOT",
                servo_filter_mode="LEGACY_FORESIGHT",
                servo_filter_legacy_max_buf=15, servo_filter_legacy_kp=0.03,
                servo_filter_lpf_cutoff_hz=0.0,
                servo_filter_nlf_max_velocity_deg_s=0.0,
                servo_filter_nlf_max_acceleration_deg_s2=0.0,
                servo_filter_nlf_max_jerk_deg_s3=0.0,
            )

        mutations = (
            ("SERVO_STEP_NUM_MISMATCH", lambda value: (setattr(value, "servo_step_num", 2), setattr(value, "command_period_ms", 16.0))),
            ("TELEMETRY_MODE_MISMATCH", lambda value: setattr(value, "telemetry_mode", "NORMAL")),
            ("SERVO_FILTER_MISMATCH", lambda value: (setattr(value, "servo_filter_mode", "NONE"), setattr(value, "servo_filter_legacy_max_buf", 0), setattr(value, "servo_filter_legacy_kp", 0.0))),
        )
        for expected_reason, mutate in mutations:
            with self.subTest(expected_reason=expected_reason):
                left = raw_status("left")
                right = raw_status("right")
                mutate(right)
                result = build_phase5_execution_feedback(
                    expected_trajectory_id="p5-current",
                    left_response=left,
                    right_response=right,
                )
                self.assertEqual(result["feedback_status"], "MISMATCH")
                self.assertEqual(result["combined_state"], "MISMATCH")
                self.assertFalse(result["terminal"])
                self.assertIn(expected_reason, result["reason"])

    def test_self_inconsistent_servo_period_is_rejected_before_aggregation(self):
        response = SimpleNamespace(
            valid=True, ret=1, message="memory snapshot", trajectory_id="p5-current",
            state="RUNNING", active=True, start_time_unix_ns=1, terminal_time_unix_ns=0,
            duration_s=2.0, elapsed_s=1.0, progress_0_to_1=0.5, sample_index=1,
            sample_count=3, terminal_reason="", servo_step_num=1, command_period_ms=16.0,
            telemetry_mode="PHASE5_PORT10004_SNAPSHOT",
        )
        normalized = normalize_driver_execution_status(response, side="left")
        self.assertFalse(normalized["valid"])
        self.assertIn("command_period_ms does not match servo_step_num", normalized["error"])

    def test_actual_joint_cache_is_copied_through_without_robot_access(self):
        joints = {
            "source": "ros_joint_state_cache",
            "left": {"valid": True, "joint": [0.1] * 6, "age_ms": 12},
            "right": {"valid": True, "joint": [-0.1] * 6, "age_ms": 15},
        }
        result = build_phase5_execution_feedback(
            expected_trajectory_id="p5-current",
            left_response=driver_status("left", "RUNNING"),
            right_response=driver_status("right", "RUNNING"),
            actual_joint_feedback=joints,
        )
        self.assertEqual(result["actual_joints"], joints)
        self.assertTrue(result["read_only"])

    def test_coordinator_terminal_state_closes_only_from_aggregated_feedback(self):
        feedback_state = {"right": "RUNNING"}

        def feedback_getter(trajectory_id):
            return aggregate(
                trajectory_id,
                driver_status("left", "COMPLETED", progress=1.0, reason="COMPLETED"),
                driver_status(
                    "right",
                    feedback_state["right"],
                    progress=1.0 if feedback_state["right"] == "COMPLETED" else 0.8,
                    reason="COMPLETED" if feedback_state["right"] == "COMPLETED" else "",
                ),
            )

        coordinator = Phase5ExecutionCoordinator(
            artifact_snapshot_getter=lambda: (None, 0, "test"),
            phase4_gate_getter=lambda _fingerprint: {},
            transport=SimpleNamespace(
                inspection_state=lambda: {"robot_connection": {"both_ready": True}}
            ),
            safe_state_checker=lambda _side: (True, "ok"),
            start_match_checker=lambda _artifact: {"match": False, "reason": "NO_ARTIFACT"},
            legacy_conflict_getter=lambda: [],
            stop_generation_getter=lambda: 0,
            abort_callback=lambda: None,
            feedback_getter=feedback_getter,
        )
        coordinator._execution = {
            "state": "RUNNING",
            "trajectory_id": "p5-current",
            "reason": None,
            "start_time_unix_ns": 1_900_000_000_000_000_000,
            "duration_s": 2.0,
        }
        self.assertEqual(
            coordinator.inspection_state()["execution"]["state"], "RUNNING"
        )
        feedback_state["right"] = "COMPLETED"
        closed = coordinator.inspection_state()["execution"]
        self.assertEqual(closed["state"], "COMPLETED")
        self.assertEqual(closed["reason"], "BOTH_DRIVERS_COMPLETED")

    def test_idle_coordinator_never_queries_driver_status(self):
        calls = []
        coordinator = Phase5ExecutionCoordinator(
            artifact_snapshot_getter=lambda: (None, 0, "test"),
            phase4_gate_getter=lambda _fingerprint: {},
            transport=SimpleNamespace(
                inspection_state=lambda: {"robot_connection": {"both_ready": True}}
            ),
            safe_state_checker=lambda _side: (True, "ok"),
            start_match_checker=lambda _artifact: {"match": False, "reason": "NO_ARTIFACT"},
            legacy_conflict_getter=lambda: [],
            stop_generation_getter=lambda: 0,
            abort_callback=lambda: None,
            feedback_getter=lambda trajectory_id: calls.append(trajectory_id) or {},
        )
        self.assertFalse(coordinator.is_active())
        coordinator.inspection_state()
        self.assertEqual(calls, [])

    def test_passive_snapshot_has_no_feedback_or_abort_side_effects(self):
        feedback_calls = []
        abort_calls = []
        coordinator = Phase5ExecutionCoordinator(
            artifact_snapshot_getter=lambda: (None, 0, "test"),
            phase4_gate_getter=lambda _fingerprint: {},
            transport=SimpleNamespace(
                inspection_state=lambda: {"robot_connection": {"both_ready": True}}
            ),
            safe_state_checker=lambda _side: (True, "ok"),
            start_match_checker=lambda _artifact: {"match": False, "reason": "NO_ARTIFACT"},
            legacy_conflict_getter=lambda: [],
            stop_generation_getter=lambda: 0,
            abort_callback=lambda: abort_calls.append("STOP_BOTH"),
            feedback_getter=lambda trajectory_id: feedback_calls.append(trajectory_id) or {},
        )
        coordinator._execution = {
            "state": "RUNNING", "trajectory_id": "p5-current", "reason": None,
            "start_time_unix_ns": 1_900_000_000_000_000_000, "duration_s": 2.0,
        }

        snapshot = coordinator.passive_execution_snapshot()
        snapshot["state"] = "MUTATED"

        self.assertEqual(coordinator._execution["state"], "RUNNING")
        self.assertEqual(feedback_calls, [])
        self.assertEqual(abort_calls, [])

    def test_partial_driver_failure_requests_peer_stop_once(self):
        abort_calls = []
        coordinator = Phase5ExecutionCoordinator(
            artifact_snapshot_getter=lambda: (None, 0, "test"),
            phase4_gate_getter=lambda _fingerprint: {},
            transport=SimpleNamespace(
                inspection_state=lambda: {"robot_connection": {"both_ready": True}}
            ),
            safe_state_checker=lambda _side: (True, "ok"),
            start_match_checker=lambda _artifact: {"match": False, "reason": "NO_ARTIFACT"},
            legacy_conflict_getter=lambda: [],
            stop_generation_getter=lambda: 0,
            abort_callback=lambda: abort_calls.append("STOP_BOTH"),
            feedback_getter=lambda trajectory_id: aggregate(
                trajectory_id,
                driver_status("left", "FAILED", reason="SERVO_STREAM_FAILED"),
                driver_status("right", "RUNNING", progress=0.4),
            ),
        )
        coordinator._execution = {
            "state": "RUNNING",
            "trajectory_id": "p5-current",
            "reason": None,
            "start_time_unix_ns": 1_900_000_000_000_000_000,
            "duration_s": 2.0,
        }
        first = coordinator.inspection_state()["execution"]
        self.assertEqual(first["state"], "ABORT_REQUESTED")
        self.assertEqual(abort_calls, ["STOP_BOTH"])
        second = coordinator.inspection_state()["execution"]
        self.assertEqual(second["state"], "ABORT_REQUESTED")
        self.assertEqual(abort_calls, ["STOP_BOTH"])



class Phase5DriverSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.driver = (
            ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"
        ).read_text()

    def test_status_service_schema_registration_and_callback_are_read_only(self):
        service = (
            ROOT / "src/jaka_ros2/src/jaka_msgs/srv/GetExecutionStatus.srv"
        ).read_text()
        for field in (
            "bool valid", "int64 ret", "string message", "string trajectory_id",
            "string state", "bool active", "int64 start_time_unix_ns",
            "int64 terminal_time_unix_ns", "float64 duration_s", "float64 elapsed_s",
            "float64 progress_0_to_1", "int64 sample_index", "uint64 sample_count",
            "bool commanded_sample_valid", "float64 commanded_time_from_start_s",
            "float64[] commanded_joints_rad", "string terminal_reason",
            "float64 max_abs_velocity_rad_s",
            "float64 max_abs_acceleration_rad_s2",
            "float64 max_abs_jerk_rad_s3",
            "uint64 dispatch_sample_count",
            "float64 mean_abs_lateness_ms",
            "float64 p95_abs_lateness_ms",
            "float64 p99_abs_lateness_ms",
            "float64 max_abs_lateness_ms",
            "float64 mean_abs_jitter_ms",
            "uint64 missed_cycle_count",
            "string servo_filter_mode",
        ):
            self.assertIn(field, service)
        cmake = (ROOT / "src/jaka_ros2/src/jaka_msgs/CMakeLists.txt").read_text()
        self.assertIn('"srv/GetExecutionStatus.srv"', cmake)
        callback = self.driver.split("bool get_execution_status_callback", 1)[1].split(
            "struct Phase5ServoFilterConfig", 1
        )[0]
        self.assertIn("phase5_trajectory_mutex", callback)
        self.assertNotIn("robot.", callback)
        self.assertNotIn("JAKAZuRobot", callback)

    def test_driver_contains_phase5_only_filter_and_dispatch_instrumentation(self):
        self.assertIn("robot.servo_move_use_none_filter()", self.driver)
        self.assertIn("robot.servo_move_use_joint_LPF", self.driver)
        self.assertIn("robot.servo_move_use_joint_NLF", self.driver)
        self.assertIn("store_phase5_dispatch_timing", self.driver)
        self.assertIn("missed_cycle_count", self.driver)
        legacy_servo = self.driver.split("bool servo_j_callback", 1)[1].split(
            "static int64_t host_wall_clock_now_ns", 1
        )[0]
        self.assertNotIn("filter", legacy_servo.lower())

    def test_terminal_mapping_and_legacy_motion_semantics_remain(self):
        finish = self.driver.split("static void finish_phase5_trajectory", 1)[1].split(
            "static void execute_phase5_joint_trajectory_worker", 1
        )[0]
        self.assertIn('reason.rfind("CANCELLED", 0)', finish)
        self.assertIn('phase5_execution_status.state = "ABORTED"', finish)
        self.assertIn('phase5_execution_status.state = "COMPLETED"', finish)
        self.assertIn('phase5_execution_status.state = "FAILED"', finish)
        self.assertLess(
            finish.index("terminal_time_unix_ns"),
            finish.index("phase5_trajectory_active = false"),
        )
        self.assertIn("robot.servo_j(&joint_pose, MoveMode::INCR)", self.driver)
        self.assertIn("robot.joint_move(&joint_pose, MoveMode::ABS, false", self.driver)
        stop = self.driver.split("bool stop_move_callback", 1)[1].split(
            "bool set_toolFrame_callback", 1
        )[0]
        self.assertLess(
            stop.index("phase5_trajectory_cancel_requested.store(true)"),
            stop.index("robot.motion_abort()"),
        )


class Phase5BackendCacheTests(unittest.TestCase):
    def test_phase5_status_passes_through_existing_joint_cache(self):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import

        backend, cleanup = _mocked_backend_import()
        try:
            backend.node.phase5_execution_coordinator = SimpleNamespace(
                inspection_state=lambda: {"ok": True, "execution": {"state": "IDLE"}}
            )
            with backend.node.digital_twin_joint_cache_lock:
                for index, side in enumerate(("left", "right")):
                    cache = backend.node.digital_twin_joint_cache[side]
                    cache.update({
                        "joint": [index + value / 10.0 for value in range(6)],
                        "received_at_ms": 1_900_000_000_000,
                        "mapping": "name",
                        "error": None,
                        "names": [f"{side}_joint_{joint}" for joint in range(1, 7)],
                    })
            result = backend.node.phase5_execution_state()
            self.assertTrue(result["actual_joints"]["left"]["valid"])
            self.assertTrue(result["actual_joints"]["right"]["valid"])
            self.assertEqual(result["actual_joints"]["left"]["joint"][1], 0.1)
            self.assertIn("CACHED ROS JOINT FEEDBACK ONLY", result["actual_joint_semantic"])
        finally:
            cleanup()


class Phase5WebSourceTests(unittest.TestCase):
    def test_phase5_panel_ids_are_unique_and_automatic_work_is_get_only(self):
        html = (ROOT / "dual_arm_app/web/index.html").read_text()
        controller = (
            ROOT / "dual_arm_app/web/digital_twin_phase5_execution.js"
        ).read_text()
        ids = re.findall(r'id="([^"]+)"', html)
        self.assertEqual(len(ids), len(set(ids)))
        for element_id in (
            "digitalTwinPhase5CommonProgress",
            "digitalTwinPhase5LeftDriver",
            "digitalTwinPhase5RightDriver",
            "digitalTwinPhase5ActualLeftTcp",
            "digitalTwinPhase5ActualRightTcp",
            "digitalTwinPhase5PreviewStatus",
            "digitalTwinPhase5PreviewTrajectory",
            "digitalTwinPhase5PreviewInterlock",
            "digitalTwinPhase5PreviewReload",
            "digitalTwinPhase5PreviewPlay",
            "digitalTwinPhase5PreviewReset",
            "digitalTwinPhase5Stop",
        ):
            self.assertIn(element_id, ids)
        phase5 = html.split('id="digitalTwinPhase5ExecutionSection"', 1)[1].split(
            "</section>", 1
        )[0]
        self.assertIn('id="digitalTwinPhase5Stop"', phase5)
        self.assertIn('onclick="stopBoth()"', phase5)
        automatic = controller.split("// Automatic work is read-only GET status + Frozen Ghost preview only", 1)[1]
        self.assertIn("refreshPhase5ExecutionStatus()", automatic)
        self.assertIn("STATUS_POLL_INTERVAL_MS", automatic)
        self.assertNotIn("executePhase5RealMotion()", automatic)
        self.assertNotIn('method: "POST"', automatic)
        self.assertNotIn("PREPARE_ENDPOINT", automatic)
        self.assertNotIn("EXECUTE_ENDPOINT", automatic)
        self.assertIn("ARTIFACT_ENDPOINT", controller)
        self.assertIn("ensureFrozenArtifactPreview", controller)
        self.assertIn("currentFrozenPreviewReady", controller)
        apply_status = controller.split("function applyStatus", 1)[1].split(
            "async function refreshPhase5ExecutionStatus", 1
        )[0]
        execute = controller.split("async function executePhase5RealMotion", 1)[1].split(
            "async function replanFromCurrent", 1
        )[0]
        self.assertIn("|| !frozenPreviewReady", apply_status)
        self.assertIn("if (!currentFrozenPreviewReady())", execute)
        self.assertLess(execute.index("PREPARE_ENDPOINT"), execute.index("window.confirm"))
        self.assertLess(execute.index("window.confirm"), execute.index("EXECUTE_ENDPOINT"))
        self.assertIn("operator_confirmed: true", execute)
        self.assertIn("preserveIntegratedPlan: true, syncIntegratedPlan: false", controller)
        self.assertIn("getTcpSourceState", controller)

    def test_phase5_operator_summary_keeps_raw_feedback_collapsed(self):
        html = (ROOT / "dual_arm_app/web/index.html").read_text()
        phase5 = html.split('id="digitalTwinPhase5ExecutionSection"', 1)[1].split(
            "</section>", 1
        )[0]
        diagnostics_id = 'id="digitalTwinPhase5Diagnostics"'
        details_start = phase5.rfind("<details", 0, phase5.index(diagnostics_id))
        details_end = phase5.index("</details>", details_start) + len("</details>")
        diagnostics = phase5[details_start:details_end]
        operator_summary = phase5[:details_start] + phase5[details_end:]
        details_tag = diagnostics.split(">", 1)[0]

        self.assertNotRegex(details_tag, r"\bopen(?:\s|=|$)")
        self.assertIn("digital-twin-operator-details", details_tag)
        for critical_id in (
            "digitalTwinPhase5ArtifactStatus",
            "digitalTwinPhase5PreviewStatus",
            "digitalTwinPhase5PreviewInterlock",
            "digitalTwinPhase5GateStatus",
            "digitalTwinPhase5SafeState",
            "digitalTwinPhase5ExecutionState",
            "digitalTwinPhase5CombinedResult",
            "digitalTwinPhase5CommonTimeline",
            "digitalTwinPhase5CommonProgress",
            "digitalTwinPhase5ProgressBar",
            "digitalTwinPhase5BlockingReasons",
            "digitalTwinPhase5PreviewPlay",
            "digitalTwinPhase5ReplanFromCurrent",
            "digitalTwinPhase5MoveToInitial",
            "digitalTwinPhase5Stop",
            "digitalTwinPhase5Prepare",
        ):
            self.assertIn(f'id="{critical_id}"', operator_summary)
            self.assertNotIn(f'id="{critical_id}"', diagnostics)

        secondary = phase5.split("More preview / status controls", 1)[1].split("</details>", 1)[0]
        for secondary_id in (
            "digitalTwinPhase5PreviewReload",
            "digitalTwinPhase5PreviewReset",
            "digitalTwinPhase5Refresh",
        ):
            self.assertIn(f'id="{secondary_id}"', secondary)
            self.assertNotIn(f'id="{secondary_id}"', diagnostics)

        for raw_id in (
            "digitalTwinPhase5ArtifactFingerprint",
            "digitalTwinPhase5ArtifactGeneration",
            "digitalTwinPhase5StopGeneration",
            "digitalTwinPhase5PreviewTrajectory",
            "digitalTwinPhase5TrajectoryId",
            "digitalTwinPhase5CommonStart",
            "digitalTwinPhase5TransportStatus",
            "digitalTwinPhase5CombinedReason",
            "digitalTwinPhase5Error",
            "digitalTwinPhase5LeftDriver",
            "digitalTwinPhase5LeftReason",
            "digitalTwinPhase5RightDriver",
            "digitalTwinPhase5RightReason",
            "digitalTwinPhase5ActualLeftJoints",
            "digitalTwinPhase5ActualLeftJointStatus",
            "digitalTwinPhase5ActualRightJoints",
            "digitalTwinPhase5ActualRightJointStatus",
            "digitalTwinPhase5ActualLeftTcp",
            "digitalTwinPhase5ActualRightTcp",
            "digitalTwinPhase5ActualTcpStatus",
            "digitalTwinPhase5TcpDisclaimer",
        ):
            self.assertIn(f'id="{raw_id}"', diagnostics)
            self.assertNotIn(f'id="{raw_id}"', operator_summary)

        self.assertNotIn('id="digitalTwinPhase5ConfirmationPanel"', phase5)
        self.assertNotIn("digital-twin-phase5-confirmation", phase5)

    def test_execution_ghost_helper_is_isolated_from_actual_live_mirror(self):
        source = (ROOT / "dual_arm_app/web/digital_twin.js").read_text()
        helper = source.split("function setExecutionWaypointTarget", 1)[1].split(
            "function playIntegratedPlan", 1
        )[0]
        self.assertIn("pausePlannedTrajectory()", helper)
        self.assertIn("trajectoryPreviewState.trajectory", helper)
        self.assertIn("setPlannedJointValues", helper)
        self.assertIn("hidePlannedModel", helper)
        self.assertNotIn("setJointValues(", helper)
        self.assertNotIn("applyMirrorPoseToMainModel", helper)
        self.assertNotIn("latestActualPose =", helper)
        self.assertNotIn("mainModelState", helper)
        self.assertIn("getLatestActualPose", source)
        self.assertEqual(source.count("let plannedRobot = null;"), 1)


if __name__ == "__main__":
    unittest.main()
