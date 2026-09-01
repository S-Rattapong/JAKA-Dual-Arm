"""Offline acceptance tests for Phase 5 P5.4-P5.7."""

from __future__ import annotations

from pathlib import Path
import threading
import unittest

from dual_arm_app.backend.phase5_execution_transport import (
    COMMON_START_MODE,
    Phase5ExecutionTransport,
    Phase5ExecutionTransportError,
)


class FakeClient:
    def __init__(self, *, ready=True):
        self.ready = ready
        self.calls = []
        self.lock = threading.Lock()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        with self.lock:
            self.calls.append(request)
        return {"future_for": request}
def make_request(joints, vel, acc):
    return {
        "joints": tuple(float(value) for value in joints),
        "vel": float(vel),
        "acc": float(acc),
    }


def make_transport(*, left=None, right=None, stop_generation=None):
    left = left or FakeClient()
    right = right or FakeClient()
    generation = stop_generation if stop_generation is not None else [7]
    return Phase5ExecutionTransport(
        left_move_client=left,
        right_move_client=right,
        request_factory=make_request,
        stop_generation_getter=lambda: generation[0],
        left_service_name="/left_jaka_driver/joint_move",
        right_service_name="/right_jaka_driver/joint_move",
    ), left, right, generation


class Phase5ExecutionTransportPureTests(unittest.TestCase):
    def test_p5_4_common_release_dispatches_both_from_one_barrier(self):
        transport, left, right, generation = make_transport()
        receipt, left_future, right_future = transport.dispatch_pair(
            left_joint_rad=[0.1] * 6,
            right_joint_rad=[-0.1] * 6,
            left_vel=0.2,
            right_vel=0.25,
            left_acc=0.3,
            right_acc=0.35,
            expected_stop_generation=generation[0],
        )
        self.assertEqual(len(left.calls), 1)
        self.assertEqual(len(right.calls), 1)
        self.assertEqual(receipt.stop_generation, 7)
        self.assertGreaterEqual(receipt.dispatch_skew_ns, 0)
        self.assertEqual(receipt.to_dict()["common_start_mode"], COMMON_START_MODE)
        self.assertEqual(left_future["future_for"], left.calls[0])
        self.assertEqual(right_future["future_for"], right.calls[0])
    def test_p5_4_rejects_if_either_existing_driver_service_is_not_ready(self):
        left = FakeClient(ready=False)
        transport, left, right, generation = make_transport(left=left)
        with self.assertRaisesRegex(
            Phase5ExecutionTransportError, "LEFT_JOINT_MOVE_SERVICE_NOT_READY"
        ):
            transport.dispatch_pair(
                left_joint_rad=[0.0] * 6,
                right_joint_rad=[0.0] * 6,
                left_vel=0.1,
                right_vel=0.1,
                left_acc=0.2,
                right_acc=0.2,
                expected_stop_generation=generation[0],
            )
        self.assertEqual(left.calls, [])
        self.assertEqual(right.calls, [])

    def test_p5_6_stop_generation_change_blocks_release_before_dispatch(self):
        transport, left, right, generation = make_transport()
        generation[0] += 1
        with self.assertRaisesRegex(
            Phase5ExecutionTransportError, "STOP_GENERATION_CHANGED_BEFORE_COMMON_RELEASE"
        ):
            transport.dispatch_pair(
                left_joint_rad=[0.0] * 6,
                right_joint_rad=[0.0] * 6,
                left_vel=0.1,
                right_vel=0.1,
                left_acc=0.2,
                right_acc=0.2,
                expected_stop_generation=7,
            )
        self.assertEqual(left.calls, [])
        self.assertEqual(right.calls, [])

    def test_p5_6_stop_race_at_barrier_aborts_before_either_dispatch(self):
        left = FakeClient()
        right = FakeClient()
        generations = iter((7, 8))
        transport = Phase5ExecutionTransport(
            left_move_client=left,
            right_move_client=right,
            request_factory=make_request,
            stop_generation_getter=lambda: next(generations),
            left_service_name="/left_jaka_driver/joint_move",
            right_service_name="/right_jaka_driver/joint_move",
        )
        with self.assertRaisesRegex(
            Phase5ExecutionTransportError, "STOP_GENERATION_CHANGED_AT_COMMON_RELEASE"
        ):
            transport.dispatch_pair(
                left_joint_rad=[0.0] * 6, right_joint_rad=[0.0] * 6,
                left_vel=0.1, right_vel=0.1, left_acc=0.2, right_acc=0.2,
                expected_stop_generation=7,
            )
        self.assertEqual(left.calls, [])
        self.assertEqual(right.calls, [])

    def test_p5_5_and_p5_7_contract_reuses_existing_driver_path(self):
        transport, _left, _right, _generation = make_transport()
        state = transport.inspection_state()
        connection = state["p5_5_robot_connection"]
        execution = state["p5_7_execution_path"]
        self.assertTrue(connection["uses_existing_driver_sessions"])
        self.assertFalse(connection["new_sdk_session"])
        self.assertTrue(connection["both_ready"])
        self.assertEqual(
            connection["left_joint_move_service"], "/left_jaka_driver/joint_move"
        )
        self.assertEqual(execution["request_factory"], "EXISTING make_joint_move_request")
        self.assertFalse(execution["direct_sdk_calls_from_backend"])
    def test_p5_4_contract_is_explicitly_not_hard_realtime(self):
        transport, _left, _right, _generation = make_transport()
        common_start = transport.inspection_state()["p5_4_common_start"]
        self.assertTrue(common_start["host_barrier_release"])
        self.assertTrue(common_start["dispatch_skew_measured"])
        self.assertFalse(common_start["hard_realtime_controller_sync"])
        self.assertIn("NOT GUARANTEED", common_start["semantic"])


class Phase5ExecutionTransportBackendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def test_readonly_transport_endpoint_does_not_dispatch_motion(self):
        node = self.backend.node
        route = next(
            item.endpoint for item in self.backend.app.routes
            if getattr(item, "path", None) == "/api/digital-twin/phase5/execution-transport"
        )
        payload = route()
        self.assertTrue(payload["ok"])
        self.assertTrue(payload["execution_endpoint_available"])
        self.assertFalse(payload["motion_dispatched"])
    def test_p5_10_transport_reuses_backend_clients_and_opens_no_sdk_session(self):
        backend_source = Path(
            "dual_arm_app/backend/dual_jaka_web_backend.py"
        ).read_text(encoding="utf-8")
        transport_source = Path(
            "dual_arm_app/backend/phase5_execution_transport.py"
        ).read_text(encoding="utf-8")
        self.assertIn("left_client=self.left_phase5_execute", backend_source)
        self.assertIn("right_client=self.right_phase5_execute", backend_source)
        for forbidden in (
            "create_client(", "login_in(", "login_out(", "JAKAZuRobot", "rclpy",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, transport_source)

    def test_p5_6_existing_stop_and_home_paths_are_preserved(self):
        source = Path(
            "dual_arm_app/backend/dual_jaka_web_backend.py"
        ).read_text(encoding="utf-8")
        stop = source.split("def request_stop_all", 1)[1].split("def home", 1)[0]
        home = source.split("def home", 1)[1].split("def load_waypoints", 1)[0]
        self.assertIn('self.stop_motion("both")', stop)
        self.assertIn("self.stop_generation", stop)
        self.assertIn("return self.move_both_joint(", home)
    def test_p5_7_existing_driver_joint_move_is_nonblocking(self):
        driver = Path(
            "src/jaka_ros2/src/jaka_driver/src/jaka_driver.cpp"
        ).read_text(encoding="utf-8")
        callback = driver.split("bool joint_move_callback", 1)[1].split(
            "bool jog_callback", 1
        )[0]
        self.assertIn(
            "robot.joint_move(&joint_pose, MoveMode::ABS, false, speed, accel",
            callback,
        )

    def test_phase5_motion_endpoints_and_isolated_web_ui_are_added(self):
        backend = Path(
            "dual_arm_app/backend/dual_jaka_web_backend.py"
        ).read_text(encoding="utf-8")
        html = Path("dual_arm_app/web/index.html").read_text(encoding="utf-8")
        controller = Path(
            "dual_arm_app/web/digital_twin_phase5_execution.js"
        ).read_text(encoding="utf-8")
        self.assertIn('@app.post("/api/digital-twin/phase5/prepare")', backend)
        self.assertIn('@app.post("/api/digital-twin/phase5/execute")', backend)
        self.assertIn('@app.get("/api/digital-twin/phase5/execution-transport")', backend)
        self.assertIn("Phase 5 Execution", html)
        self.assertIn("bindPhase5ExecutionControls", controller)

    def test_motion_classification_is_phase5_endpoint_specific(self):
        classify = self.backend._d33_is_motion_command
        self.assertFalse(classify("/api/digital-twin/phase5/execution", "GET"))
        self.assertFalse(classify("/api/digital-twin/phase5/prepare", "POST"))
        self.assertTrue(classify("/api/digital-twin/phase5/execute", "POST"))
        self.assertTrue(classify("/api/home", "POST"))


if __name__ == "__main__":
    unittest.main()
