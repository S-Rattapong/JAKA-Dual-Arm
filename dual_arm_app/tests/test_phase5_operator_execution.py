"""Offline/mock/static acceptance tests for Phase 5 P5.8-P5.10."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import re
import unittest

from dual_arm_app.backend.phase5_execution_coordinator import (
    Phase5ExecutionCoordinator,
)
from dual_arm_app.backend.phase5_execution_transport import (
    ABSOLUTE_COMMON_START_MODE,
    Phase5AbsoluteStartTransport,
    Phase5ExecutionTransportError,
)


ROOT = Path(__file__).resolve().parents[2]


class FakeClock:
    def __init__(self, now_ns=1_800_000_000_000_000_000):
        self.now_ns = now_ns

    def __call__(self):
        return self.now_ns


class FakeReceipt:
    def __init__(self, start_time_unix_ns, trajectory_id):
        self.start_time_unix_ns = start_time_unix_ns
        self.trajectory_id = trajectory_id

    def to_dict(self):
        return {
            "start_time_unix_ns": self.start_time_unix_ns,
            "trajectory_id": self.trajectory_id,
        }


class FakeCoordinatorTransport:
    def __init__(self, *, ready=True, reject=False):
        self.ready = ready
        self.reject = reject
        self.calls = []

    def inspection_state(self):
        return {
            "robot_connection": {"both_ready": self.ready},
            "common_start": {"mode": ABSOLUTE_COMMON_START_MODE},
        }

    def submit_pair(self, **kwargs):
        self.calls.append(kwargs)
        if self.reject:
            raise Phase5ExecutionTransportError("RIGHT_TRAJECTORY_REJECTED")
        return FakeReceipt(kwargs["start_time_unix_ns"], kwargs["trajectory_id"])


def frozen_artifact(fingerprint="artifact-1", plan="plan-1"):
    return SimpleNamespace(
        artifact_fingerprint=fingerprint,
        plan_fingerprint=plan,
        trajectory_name="validated-object-path",
        common_timestamps_s=(0.0, 0.5, 1.0),
        left_positions_rad=((0.0,) * 6, (0.1,) * 6, (0.2,) * 6),
        right_positions_rad=((0.0,) * 6, (-0.1,) * 6, (-0.2,) * 6),
        duration_s=1.0,
        sample_count=3,
    )


def make_coordinator(*, transport=None, conflicts=None):
    clock = FakeClock()
    state = {
        "artifact": frozen_artifact(),
        "generation": 7,
        "stop": 11,
        "start_match": True,
    }
    aborts = []
    transport = transport or FakeCoordinatorTransport()
    coordinator = Phase5ExecutionCoordinator(
        artifact_snapshot_getter=lambda: (
            state["artifact"], state["generation"], None
        ),
        phase4_gate_getter=lambda fingerprint: {
            "execution_ready": fingerprint == state["artifact"].plan_fingerprint,
            "status": "READY",
        },
        transport=transport,
        safe_state_checker=lambda side: (side == "both", "ok"),
        start_match_checker=lambda _artifact: {
            "match": state["start_match"],
            "state": "READY" if state["start_match"] else "MISMATCH",
            "reason": "MATCH" if state["start_match"] else "DELTA_EXCEEDED",
            "max_delta_rad": 0.0 if state["start_match"] else 0.02,
            "threshold_rad": 0.01,
        },
        legacy_conflict_getter=lambda: list(conflicts or []),
        stop_generation_getter=lambda: state["stop"],
        abort_callback=lambda: aborts.append("existing-stop-path"),
        wall_clock_ns=clock,
        trajectory_id_factory=lambda: "trajectory-id-1",
        common_start_lead_s=2.0,
    )
    return coordinator, state, clock, transport, aborts


class Phase5OperatorAuthorityCoordinatorTests(unittest.TestCase):
    def test_prepare_is_no_motion_and_binds_all_authority_generations(self):
        coordinator, _state, _clock, transport, _aborts = make_coordinator()
        prepared = coordinator.prepare()
        self.assertTrue(prepared["ok"])
        self.assertFalse(prepared["motion_dispatched"])
        self.assertEqual(transport.calls, [])
        self.assertNotIn("confirmation_token", prepared)
        self.assertNotIn("expires_at_unix_ns", prepared)
        self.assertIn("created_at_unix_ns", prepared["authority"])
        self.assertEqual(prepared["authority"]["artifact_fingerprint"], "artifact-1")
        self.assertEqual(prepared["authority"]["plan_fingerprint"], "plan-1")
        self.assertEqual(prepared["authority"]["artifact_generation"], 7)
        self.assertEqual(prepared["authority"]["stop_generation"], 11)

    def test_operator_authority_is_single_use(self):
        coordinator, _state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        first = coordinator.execute(True)
        second = coordinator.execute(True)
        self.assertTrue(first["ok"])
        self.assertFalse(second["ok"])
        self.assertEqual(second["error"], "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED")
        self.assertEqual(len(transport.calls), 1)

    def test_operator_confirmation_must_be_exact_true_and_attempt_consumes_authority(self):
        for invalid in (False, None, 1, "true"):
            with self.subTest(invalid=invalid):
                coordinator, _state, _clock, transport, _aborts = make_coordinator()
                coordinator.prepare()
                rejected = coordinator.execute(invalid)
                self.assertEqual(rejected["error"], "OPERATOR_CONFIRMATION_REQUIRED")
                reused = coordinator.execute(True)
                self.assertEqual(
                    reused["error"], "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED"
                )
                self.assertEqual(transport.calls, [])

    def test_pending_operator_authority_has_no_timeout(self):
        coordinator, _state, clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        clock.now_ns += 365 * 24 * 60 * 60 * 1_000_000_000
        result = coordinator.execute(True)
        self.assertTrue(result["ok"])
        self.assertEqual(len(transport.calls), 1)

    def test_artifact_or_stop_generation_change_invalidates_authority(self):
        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["artifact"] = frozen_artifact("artifact-2", "plan-1")
        state["generation"] += 1
        stale = coordinator.execute(True)
        self.assertFalse(stale["ok"])
        self.assertIn("artifact_fingerprint", stale["binding_mismatches"])
        self.assertIn("artifact_generation", stale["binding_mismatches"])
        self.assertEqual(transport.calls, [])

        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["stop"] += 1
        stopped = coordinator.execute(True)
        self.assertFalse(stopped["ok"])
        self.assertIn("stop_generation", stopped["binding_mismatches"])
        self.assertEqual(transport.calls, [])

    def test_explicit_stop_hook_removes_pending_operator_authority(self):
        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["stop"] += 1
        coordinator.on_stop(state["stop"])
        result = coordinator.execute(True)
        self.assertEqual(result["error"], "PENDING_OPERATOR_AUTHORITY_MISSING_OR_REUSED")
        self.assertEqual(transport.calls, [])

    def test_stop_while_armed_marks_abort_requested_until_feedback_closes_it(self):
        coordinator, state, _clock, _transport, _aborts = make_coordinator()
        coordinator.prepare()
        self.assertTrue(coordinator.execute(True)["ok"])
        state["stop"] += 1
        coordinator.on_stop(state["stop"])
        execution = coordinator.inspection_state()["execution"]
        self.assertEqual(execution["state"], "ABORT_REQUESTED")
        self.assertEqual(execution["reason"], "STOP_GENERATION_CHANGED")

    def test_execute_uses_one_frozen_artifact_and_shared_future_start(self):
        coordinator, state, clock, transport, _aborts = make_coordinator()
        artifact = state["artifact"]
        coordinator.prepare()
        result = coordinator.execute(True)
        self.assertTrue(result["ok"])
        self.assertEqual(result["execution"]["state"], "ARMED")
        self.assertEqual(len(transport.calls), 1)
        call = transport.calls[0]
        self.assertIs(call["common_timestamps_s"], artifact.common_timestamps_s)
        self.assertIs(call["left_positions_rad"], artifact.left_positions_rad)
        self.assertIs(call["right_positions_rad"], artifact.right_positions_rad)
        self.assertEqual(call["start_time_unix_ns"], clock.now_ns + 2_000_000_000)
        self.assertEqual(result["common_start_mode"], ABSOLUTE_COMMON_START_MODE)

    def test_legacy_conflict_blocks_prepare_without_reusing_legacy_gate(self):
        coordinator, _state, _clock, transport, _aborts = make_coordinator(
            conflicts=["LEGACY_SEQUENCE_OR_PROGRAM_ACTIVE"]
        )
        result = coordinator.prepare()
        self.assertFalse(result["ok"])
        self.assertIn("LEGACY_SEQUENCE_OR_PROGRAM_ACTIVE", result["blocking_reasons"])
        self.assertEqual(transport.calls, [])

    def test_actual_start_mismatch_blocks_prepare_and_execute_recheck(self):
        coordinator, state, _clock, transport, _aborts = make_coordinator()
        state["start_match"] = False
        blocked = coordinator.prepare()
        self.assertFalse(blocked["ok"])
        self.assertTrue(any(
            "ACTUAL_START_INTERLOCK_BLOCKED" in reason
            for reason in blocked["blocking_reasons"]
        ))
        self.assertEqual(transport.calls, [])

        coordinator, state, _clock, transport, _aborts = make_coordinator()
        coordinator.prepare()
        state["start_match"] = False
        blocked = coordinator.execute(True)
        self.assertFalse(blocked["ok"])
        self.assertIn(
            "ACTUAL_START_INTERLOCK_BLOCKED: DELTA_EXCEEDED",
            blocked["blocking_reasons"],
        )
        self.assertEqual(transport.calls, [])

    def test_one_side_rejection_uses_existing_stop_path(self):
        rejecting = FakeCoordinatorTransport(reject=True)
        coordinator, _state, _clock, _transport, aborts = make_coordinator(
            transport=rejecting
        )
        coordinator.prepare()
        result = coordinator.execute(True)
        self.assertFalse(result["ok"])
        self.assertTrue(result["stop_requested"])
        self.assertEqual(aborts, ["existing-stop-path"])

    def test_authority_change_while_armed_uses_existing_stop_path(self):
        coordinator, _state, _clock, _transport, aborts = make_coordinator()
        coordinator.prepare()
        self.assertTrue(coordinator.execute(True)["ok"])
        coordinator.invalidate_authority("GRASP_OR_VALIDATION_CHANGED")
        self.assertEqual(aborts, ["existing-stop-path"])


class FakeClient:
    def __init__(self, side, call_order):
        self.side = side
        self.call_order = call_order
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.call_order.append(self.side)
        self.requests.append(request)
        return SimpleNamespace(
            accepted=True,
            ret=1,
            message=f"{self.side} accepted",
            trajectory_id=request["trajectory_id"],
        )


class Phase5AbsoluteTransportTests(unittest.TestCase):
    def test_exact_same_start_and_timeline_are_sent_to_both_before_wait(self):
        order = []
        left = FakeClient("left", order)
        right = FakeClient("right", order)

        def wait_future(future, _timeout):
            self.assertEqual(order, ["left", "right"])
            return future

        clock = FakeClock()
        transport = Phase5AbsoluteStartTransport(
            left_client=left,
            right_client=right,
            request_factory=lambda times, joints, start, trajectory_id, filter_config, step_num: {
                "times": times,
                "joints": joints,
                "start": start,
                "trajectory_id": trajectory_id,
                "filter": filter_config,
                "step_num": step_num,
            },
            wait_future_result=wait_future,
            stop_generation_getter=lambda: 4,
            driver_contract_checker=lambda: {"ok": True},
            wall_clock_ns=clock,
            left_service_name="/left_jaka_driver/execute_joint_trajectory",
            right_service_name="/right_jaka_driver/execute_joint_trajectory",
        )
        start_ns = clock.now_ns + 2_000_000_000
        receipt = transport.submit_pair(
            common_timestamps_s=(0.0, 1.0),
            left_positions_rad=((0.0,) * 6, (0.1,) * 6),
            right_positions_rad=((0.0,) * 6, (-0.1,) * 6),
            start_time_unix_ns=start_ns,
            trajectory_id="trajectory-1",
            expected_stop_generation=4,
        )
        self.assertEqual(left.requests[0]["start"], start_ns)
        self.assertEqual(right.requests[0]["start"], start_ns)
        self.assertEqual(left.requests[0]["times"], right.requests[0]["times"])
        self.assertEqual(len(left.requests[0]["joints"]), 12)
        self.assertEqual(len(right.requests[0]["joints"]), 12)
        self.assertEqual(receipt.start_time_unix_ns, start_ns)

    def test_driver_contract_mismatch_blocks_before_any_submission(self):
        order = []
        left = FakeClient("left", order)
        right = FakeClient("right", order)
        clock = FakeClock()
        transport = Phase5AbsoluteStartTransport(
            left_client=left,
            right_client=right,
            request_factory=lambda times, joints, start, trajectory_id, filter_config, step_num: {
                "times": times,
                "joints": joints,
                "start": start,
                "trajectory_id": trajectory_id,
                "filter": filter_config,
                "step_num": step_num,
            },
            wait_future_result=lambda future, _timeout: future,
            stop_generation_getter=lambda: 4,
            driver_contract_checker=lambda: {
                "ok": False,
                "expected_marker": "PHASE5_DRIVER_CONTRACT=LINEAR_JOINT_SPACE_V2",
                "sides": {"left": {"ok": True}, "right": {"ok": False}},
            },
            wall_clock_ns=clock,
            left_service_name="/left_jaka_driver/execute_joint_trajectory",
            right_service_name="/right_jaka_driver/execute_joint_trajectory",
        )
        with self.assertRaisesRegex(
            Phase5ExecutionTransportError, "PHASE5_DRIVER_CONTRACT_MISMATCH"
        ):
            transport.submit_pair(
                common_timestamps_s=(0.0, 1.0),
                left_positions_rad=((0.0,) * 6, (0.1,) * 6),
                right_positions_rad=((0.0,) * 6, (-0.1,) * 6),
                start_time_unix_ns=clock.now_ns + 2_000_000_000,
                trajectory_id="trajectory-contract-mismatch",
                expected_stop_generation=4,
            )
        self.assertEqual(order, [])
        self.assertEqual(left.requests, [])
        self.assertEqual(right.requests, [])


class Phase5SourceContractTests(unittest.TestCase):
    def test_service_schema_and_generation_contract(self):
        service = (ROOT / "src/jaka_ros2/src/jaka_msgs/srv/ExecuteJointTrajectory.srv").read_text()
        self.assertIn("float64[] time_from_start_s", service)
        self.assertIn("float64[] joint_positions_rad_flat", service)
        self.assertIn("int64 start_time_unix_ns", service)
        self.assertIn("uint8 servo_filter_mode", service)
        self.assertIn("uint8 SERVO_FILTER_LEGACY_FORESIGHT=3", service)
        self.assertIn("uint8 servo_step_num", service)
        self.assertIn("float64 servo_filter_nlf_max_jerk_deg_s3", service)
        self.assertIn("bool accepted", service)
        self.assertIn("int64 ret", service)
        cmake = (ROOT / "src/jaka_ros2/src/jaka_msgs/CMakeLists.txt").read_text()
        self.assertIn('"srv/ExecuteJointTrajectory.srv"', cmake)

    def test_backend_request_uses_flat_field_and_exact_authorities(self):
        from dual_arm_app.tests.test_selective_scope_reset import (
            _mocked_backend_import,
        )

        backend, cleanup = _mocked_backend_import()
        try:
            request = backend.node.make_execute_joint_trajectory_request(
                [0.0, 1.0], [0.0] * 12, 1_900_000_000_000_000_000, "p5-test"
            )
            self.assertEqual(request.time_from_start_s, [0.0, 1.0])
            self.assertEqual(request.joint_positions_rad_flat, [0.0] * 12)
            self.assertEqual(request.start_time_unix_ns, 1_900_000_000_000_000_000)
            self.assertEqual(request.trajectory_id, "p5-test")
            self.assertEqual(request.servo_filter_mode, 3)
            self.assertEqual(request.servo_filter_legacy_max_buf, 15)
            self.assertEqual(request.servo_filter_legacy_kp, 0.03)
            self.assertEqual(request.servo_step_num, 1)
            self.assertFalse(hasattr(request, "joint_positions_rad"))
        finally:
            cleanup()

    def test_driver_validation_absolute_servo_and_legacy_paths(self):
        driver = (ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp").read_text()
        helper = (ROOT / "src/jaka_ros2/src/jaka_driver/src/phase5_joint_trajectory.cpp").read_text()
        phase5_callback = driver.split("bool execute_joint_trajectory_callback", 1)[1].split(
            "bool stop_move_callback", 1
        )[0]
        legacy_servo = driver.split("bool servo_j_callback", 1)[1].split(
            "static int64_t host_wall_clock_now_ns", 1
        )[0]
        legacy_joint = driver.split("bool joint_move_callback", 1)[1].split(
            "bool jog_callback", 1
        )[0]
        self.assertIn("validate_trajectory", phase5_callback)
        self.assertIn("resample_linear", phase5_callback)
        self.assertNotIn("resample_quintic_hermite", phase5_callback)
        self.assertIn("PHASE5_DRIVER_CONTRACT=LINEAR_JOINT_SPACE_V2", driver)
        backend = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text()
        self.assertIn("phase5_driver_contract_preflight", backend)
        self.assertIn("driver_contract_checker=self.phase5_driver_contract_preflight", backend)
        self.assertIn("phase5_trajectory_active", phase5_callback)
        self.assertIn("thread(", phase5_callback)
        self.assertIn("robot.servo_move_enable(TRUE)", phase5_callback)
        self.assertLess(
            phase5_callback.index("robot.servo_move_enable(TRUE)"),
            phase5_callback.index("response->accepted = true"),
        )
        self.assertIn("MoveMode::ABS, static_cast<int>(servo_step_num)", driver)
        self.assertIn("robot.servo_j(&joint_pose, MoveMode::INCR)", legacy_servo)
        self.assertIn(
            "robot.joint_move(&joint_pose, MoveMode::ABS, false", legacy_joint
        )
        self.assertIn("kBaseServoPeriodS", helper)
        self.assertIn("build_quintic_hermite_trajectory", helper)
        self.assertIn("result.samples.front() = samples.front()", helper)
        self.assertIn("result.samples.back() = samples.back()", helper)

    def test_driver_reuses_one_global_session_and_stop_cancels_then_aborts(self):
        driver = (ROOT / "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp").read_text()
        callback = driver.split("bool execute_joint_trajectory_callback", 1)[1].split(
            "bool stop_move_callback", 1
        )[0]
        stop = driver.split("bool stop_move_callback", 1)[1].split(
            "bool set_toolFrame_callback", 1
        )[0]
        self.assertNotIn("login_in", callback)
        self.assertNotIn("login_out", callback)
        self.assertNotIn("JAKAZuRobot", callback)
        self.assertLess(stop.index("phase5_trajectory_cancel_requested.store(true)"), stop.index("robot.motion_abort()"))

    def test_backend_legacy_program_is_not_phase4_or_token_gated(self):
        backend = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text()
        route = backend.split('@app.post("/api/program/run")', 1)[1].split(
            '@app.post("/api/sequence/stop")', 1
        )[0]
        self.assertNotIn("phase4", route.lower())
        self.assertNotIn("confirmation_token", route)
        self.assertIn("return node.run_program", route)

    def test_stop_home_phase5_awareness_preserves_normal_paths(self):
        backend = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text()
        stop = backend.split("def request_stop_all", 1)[1].split("def home", 1)[0]
        home = backend.split("def home", 1)[1].split("def load_waypoints", 1)[0]
        self.assertIn("phase5_execution_coordinator.on_stop", stop)
        self.assertIn('self.stop_motion("both")', stop)
        self.assertIn("phase5_execution_coordinator.is_active", home)
        self.assertIn("return self.move_both_joint(", home)

    def test_ui_unique_ids_explicit_handlers_and_no_automatic_motion(self):
        html = (ROOT / "dual_arm_app/web/index.html").read_text()
        controller = (ROOT / "dual_arm_app/web/digital_twin_phase5_execution.js").read_text()
        ids = re.findall(r'id="([^"]+)"', html)
        self.assertEqual(len(ids), len(set(ids)))
        for element_id in (
            "digitalTwinPhase5ExecutionSection",
            "digitalTwinPhase5Prepare",
            "digitalTwinPhase5ReplanFromCurrent",
            "digitalTwinPhase5MoveToInitial",
            "digitalTwinPhase5StartMatch",
        ):
            self.assertIn(element_id, ids)
        self.assertIn('prepareButton.addEventListener("click"', controller)
        self.assertIn("executePhase5RealMotion", controller)
        self.assertIn("window.confirm", controller)
        self.assertIn("JSON.stringify({ operator_confirmed: true })", controller)
        self.assertIn('replanButton.addEventListener("click"', controller)
        self.assertIn('moveButton.addEventListener("click"', controller)
        initialization = controller.split(
            "// Automatic work is read-only GET status + Frozen Ghost preview only", 1
        )[1]
        self.assertIn("refreshPhase5ExecutionStatus()", initialization)
        self.assertIn("ensureFrozenArtifactPreview", controller)
        self.assertNotIn("executePhase5RealMotion()", initialization)
        self.assertNotIn("moveToInitial()", initialization)

    def test_recovery_endpoint_reuses_existing_joint_move_and_stop_contract(self):
        backend = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text()
        recovery = backend.split("def phase5_move_to_initial", 1)[1].split(
            "def phase5_execution_state", 1
        )[0]
        self.assertIn("self.move_both_joint(", recovery)
        self.assertIn("artifact.left_positions_rad[0]", recovery)
        self.assertIn("artifact.right_positions_rad[0]", recovery)
        self.assertIn("self.safe_state_ok(\"both\")", recovery)
        self.assertIn("self._phase5_legacy_conflicts()", recovery)
        self.assertNotIn("JAKAZuRobot", recovery)
        self.assertIn('onclick="stopBoth()"', (ROOT / "dual_arm_app/web/index.html").read_text())


if __name__ == "__main__":
    unittest.main()
