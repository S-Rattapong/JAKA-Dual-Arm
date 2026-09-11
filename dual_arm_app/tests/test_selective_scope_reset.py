import hashlib
import fcntl
import importlib.util
import re
import sys
import threading
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
WEB_UI = ROOT / "dual_arm_app/web/index.html"

PRESERVED_ROUTES = {
    ("GET", "/"),
    ("GET", "/api/status"),
    ("GET", "/api/robot/activity"),
    ("POST", "/api/jog/start"),
    ("POST", "/api/jog/heartbeat"),
    ("POST", "/api/jog/stop"),
    ("POST", "/api/stop"),
    ("POST", "/api/home"),
    ("GET", "/api/waypoints"),
    ("POST", "/api/waypoint/save"),
    ("POST", "/api/waypoint/run"),
    ("POST", "/api/waypoint/delete"),
    ("POST", "/api/sequence/run"),
    ("POST", "/api/sequence/stop"),
    ("GET", "/api/program/list"),
    ("POST", "/api/program/save"),
    ("POST", "/api/program/load"),
    ("POST", "/api/program/delete"),
    ("POST", "/api/program/run"),
    ("POST", "/api/direct/joint_move"),
    ("POST", "/api/direct/tcp_move"),
}

REMOVED_ROUTES = {
    ("POST", "/api/servo/enable"),
    ("POST", "/api/servo/tiny_test"),
    ("POST", "/api/object/servo_stream"),
    ("POST", "/api/object/sync_current_from_live"),
    ("POST", "/api/object/set_current_pose"),
    ("POST", "/api/object/execute_interpolated"),
    ("POST", "/api/object/execute_move"),
    ("POST", "/api/object/validate_motion"),
    ("POST", "/api/object/validate_ik_jump"),
    ("POST", "/api/object/preview_targets"),
    ("POST", "/api/object/preview_targets_rigid_compare"),
    ("POST", "/api/object/calibration/capture_point"),
    ("POST", "/api/object/calibration/compute_frame"),
    ("POST", "/api/object/calibration/status"),
    ("POST", "/api/object/preview_calibrated_axes"),
    ("POST", "/api/object/preview_calibrated_rigid_compare"),
    ("POST", "/api/world_calibration/capture_point"),
    ("POST", "/api/world_calibration/compute_transform"),
    ("POST", "/api/world_calibration/status"),
    ("POST", "/api/world_calibration/preview_live"),
    ("POST", "/api/object/preview_calibrated_rigid_compare_world"),
    ("POST", "/api/object/validate_calibrated_world_ik"),
    ("POST", "/api/object/execute_calibrated_world_rigid"),
    ("POST", "/api/object/execute_calibrated_world_rigid_large"),
    ("POST", "/api/object/pending_calibrated_execute_commit_status"),
    ("POST", "/api/object/verify_pending_calibrated_execute_commit"),
    ("POST", "/api/object/commit_pending_calibrated_execute_pose"),
    ("POST", "/api/object/recover_pending_calibrated_execute_commit"),
    ("GET", "/api/object/list"),
    ("POST", "/api/object/capture_current"),
    ("POST", "/api/object/load"),
    ("POST", "/api/object/delete"),
}


class _FakeThread:
    instances = []

    def __init__(self, *args, **kwargs):
        self.target = kwargs.get("target")
        self.started = False
        self.__class__.instances.append(self)

    def start(self):
        # Record startup without invoking the jog loop or rclpy.spin.
        self.started = True


class _FakeNode:
    client_names = []
    subscription_names = []

    def __init__(self, _name):
        pass

    def create_subscription(self, _msg_type, topic, _callback, _qos):
        self.__class__.subscription_names.append(topic)
        return object()

    def create_client(self, _service_type, service_name):
        self.__class__.client_names.append(service_name)
        return object()


def _module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    return module


def _mocked_backend_import():
    original_thread = threading.Thread
    original_flock = fcntl.flock
    original_modules = {}
    _FakeThread.instances.clear()
    _FakeNode.client_names.clear()
    _FakeNode.subscription_names.clear()

    def forbidden(name):
        def fail(*_args, **_kwargs):
            raise AssertionError(f"{name} must not run in mocked import")
        return fail

    rclpy = _module(
        "rclpy",
        ok=lambda: True,
        init=forbidden("rclpy.init"),
        spin=forbidden("rclpy.spin"),
    )
    rclpy_node = _module("rclpy.node", Node=_FakeNode)
    rclpy.node = rclpy_node

    dummy = type("DummyRosType", (), {})
    execute_joint_trajectory = type(
        "ExecuteJointTrajectory",
        (),
        {"Request": type("ExecuteJointTrajectoryRequest", (), {})},
    )
    get_execution_status = type(
        "GetExecutionStatus",
        (),
        {"Request": type("GetExecutionStatusRequest", (), {})},
    )
    set_bool = type(
        "SetBool",
        (),
        {"Request": type("SetBoolRequest", (), {})},
    )
    fake_modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "std_srvs": _module("std_srvs"),
        "std_srvs.srv": _module("std_srvs.srv", Empty=dummy, SetBool=set_bool),
        "sensor_msgs": _module("sensor_msgs"),
        "sensor_msgs.msg": _module("sensor_msgs.msg", JointState=dummy),
        "std_msgs": _module("std_msgs"),
        "std_msgs.msg": _module(
            "std_msgs.msg", Float64MultiArray=dummy
        ),
        "jaka_msgs": _module("jaka_msgs"),
        "jaka_msgs.msg": _module("jaka_msgs.msg", RobotMsg=dummy),
        "jaka_msgs.srv": _module(
            "jaka_msgs.srv",
            Move=dummy,
            GetFK=dummy,
            GetIK=dummy,
            GetFrameState=dummy,
            ExecuteJointTrajectory=execute_joint_trajectory,
            GetExecutionStatus=get_execution_status,
        ),
    }
    fake_modules["std_srvs"].srv = fake_modules["std_srvs.srv"]
    fake_modules["sensor_msgs"].msg = fake_modules["sensor_msgs.msg"]
    fake_modules["std_msgs"].msg = fake_modules["std_msgs.msg"]
    fake_modules["jaka_msgs"].msg = fake_modules["jaka_msgs.msg"]
    fake_modules["jaka_msgs"].srv = fake_modules["jaka_msgs.srv"]

    def cleanup():
        threading.Thread = original_thread
        fcntl.flock = original_flock
        sys.modules.pop("dual_jaka_web_backend_selective_test", None)
        for name, old_module in original_modules.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module

    try:
        threading.Thread = _FakeThread
        fcntl.flock = lambda *_args, **_kwargs: None
        for name, module in fake_modules.items():
            original_modules[name] = sys.modules.get(name)
            sys.modules[name] = module

        spec = importlib.util.spec_from_file_location(
            "dual_jaka_web_backend_selective_test", BACKEND
        )
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module, cleanup
    except Exception:
        cleanup()
        raise


def _registered_routes(app):
    return {
        (method, route.path)
        for route in app.routes
        for method in (getattr(route, "methods", None) or set())
        if method in {"GET", "POST", "PUT", "DELETE"}
    }


def _tree_checksum(relative_dir):
    directory = ROOT / relative_dir
    digest_lines = []
    paths = (p for p in directory.rglob("*") if p.is_file())
    for path in sorted(paths, key=lambda p: str(p.relative_to(ROOT)).lower()):
        file_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        digest_lines.append(f"{file_hash}  {path.relative_to(ROOT)}\n")
    return len(digest_lines), hashlib.sha256("".join(digest_lines).encode()).hexdigest()


class SelectiveScopeResetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend, cls.cleanup_backend = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup_backend()

    def test_backend_import_is_fully_mocked(self):
        self.assertEqual(self.backend.app.title, "Dual JAKA A12 Web Backend")
        self.assertTrue(_FakeThread.instances)
        self.assertTrue(all(thread.started for thread in _FakeThread.instances))
        self.assertFalse(any("servo" in name for name in _FakeNode.client_names))
        # Other mocked backend imports in the same unittest process reset this
        # shared recorder. The backend's direct client contract must always be
        # present; MoveIt adapter clients are covered by their focused tests.
        self.assertTrue(
            {
                "/left_jaka_driver/jog",
                "/right_jaka_driver/jog",
                "/left_jaka_driver/joint_move",
                "/right_jaka_driver/joint_move",
                "/left_jaka_driver/linear_move",
                "/right_jaka_driver/linear_move",
                "/left_jaka_driver/get_fk",
                "/right_jaka_driver/get_fk",
                "/left_jaka_driver/get_ik",
                "/right_jaka_driver/get_ik",
                "/left_jaka_driver/get_frame_state",
                "/right_jaka_driver/get_frame_state",
                "/left_jaka_driver/stop_move",
                "/right_jaka_driver/stop_move",
                "/left_jaka_driver/execute_joint_trajectory",
                "/right_jaka_driver/execute_joint_trajectory",
                "/left_jaka_driver/get_execution_status",
                "/right_jaka_driver/get_execution_status",
            }.issubset(set(_FakeNode.client_names)),
        )

    def test_preserved_routes_are_registered(self):
        self.assertLessEqual(PRESERVED_ROUTES, _registered_routes(self.backend.app))

    def test_removed_routes_are_absent(self):
        registered = _registered_routes(self.backend.app)
        self.assertTrue(REMOVED_ROUTES.isdisjoint(registered))
        self.assertFalse(
            any(
                path.startswith("/api/object")
                or path.startswith("/api/world_calibration")
                for _method, path in registered
            )
        )

    def test_generic_direct_tcp_ik_remains(self):
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("def direct_tcp_move(", source)
        self.assertIn("GetIK.Request()", source)
        self.assertIn('"method": "get_ik_to_joint_move"', source)
        self.assertNotIn("def solve_ik_for_tcp_target(", source)

    def test_preserved_ui_labels_and_handlers_remain(self):
        html = WEB_UI.read_text(encoding="utf-8")
        labels = (
            "Live Position",
            "Direct Move",
            "Direct Joint Move",
            "Direct TCP Move",
            "Dual JAKA A12 Manual Jog",
            "Waypoint Manager",
            "Program / Sequence",
            "STOP BOTH",
            "Home Both",
            "Run Program",
            "Loop mode",
        )
        self.assertTrue(all(label in html for label in labels))
        handlers = (
            "sendDirectJointMove()",
            "sendDirectTcpMove()",
            "saveWaypoint()",
            "runSequence()",
            "stopBoth()",
            "homeBoth()",
        )
        self.assertTrue(all(handler in html for handler in handlers))

    def test_removed_ui_labels_are_absent(self):
        html = WEB_UI.read_text(encoding="utf-8")
        labels = (
            "D32.5C Servo Smooth Motion Test",
            "D32 Object Frame / Cooperative Setup",
            "D35 Object Frame Calibration",
            "D35.5A World/Base Calibration",
            "D35.5B Calibrated Rigid Preview With World Transform",
            "D35.5D IK / Joint Jump Validation",
            "D35.6A Verified Object Pose Commit",
            "D35.6A-R Recover Pending Commit",
            "D39 Operator Workflow",
            "Action Log",
            "Safety Checklist",
            "Demo Execute Gate",
        )
        self.assertTrue(all(label not in html for label in labels))

    def test_all_inline_button_handlers_still_exist(self):
        html = WEB_UI.read_text(encoding="utf-8")
        handler_names = set(
            re.findall(r'on(?:click|change)="\s*([A-Za-z_$][\w$]*)\s*\(', html)
        )
        function_names = set(
            re.findall(r'\b(?:async\s+)?function\s+([A-Za-z_$][\w$]*)\s*\(', html)
        )
        self.assertEqual(handler_names - function_names, set())

    def test_immutable_preserved_data_matches_pre_edit_snapshot(self):
        expected = {
            "dual_arm_app/programs": (
                3,
                "b72b8c3257e9e063a625391a9bd8dc499e9b94b2b3a16e81162f2e2fb9c89f6b",
            ),
            "dual_arm_app/objects": (
                2,
                "78bd25bb7e4a663a8221c7bc38ebf7f56d6f2d32d7ebf360dbb79bc5e1d17a4e",
            ),
        }
        for relative_dir, expected_value in expected.items():
            with self.subTest(relative_dir=relative_dir):
                self.assertEqual(_tree_checksum(relative_dir), expected_value)

    def test_waypoint_data_is_valid_and_preserves_baseline_entries(self):
        import json

        waypoint_path = ROOT / "dual_arm_app/tasks/waypoints.json"
        data = json.loads(waypoint_path.read_text(encoding="utf-8"))

        self.assertIsInstance(data, dict)
        self.assertGreaterEqual(len(data), 19)

        baseline_names = {
            "test_pose_01",
            "p3",
            "P1",
            "P2",
            "Pre_Grap",
            "Pre_Grap2",
            "Pre_Grap3",
            "Pre_Grap4",
            "Lift",
            "Move",
            "Place",
            "Leave1",
            "Leave2",
            "Leave3",
            "Pre_Place",
            "Pre_Lift",
            "Pre_Grap1",
            "Home_Pick_Place",
            "Home_Relate",
        }
        self.assertLessEqual(baseline_names, set(data))

        for name, waypoint in data.items():
            with self.subTest(waypoint=name):
                self.assertIsInstance(waypoint, dict)
                self.assertIn(waypoint.get("side"), {"left", "right", "both"})

                for side in ("left", "right"):
                    values = waypoint.get(side)
                    if values is None:
                        continue

                    self.assertIsInstance(values, list)
                    self.assertEqual(len(values), 6)
                    self.assertTrue(
                        all(
                            isinstance(value, (int, float))
                            and not isinstance(value, bool)
                            for value in values
                        )
                    )


if __name__ == "__main__":
    unittest.main()
