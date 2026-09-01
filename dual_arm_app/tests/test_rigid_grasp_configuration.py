"""Focused offline tests for PRE-P5.B customizable rigid grasp state."""

from __future__ import annotations

import ast
import math
import unittest
from pathlib import Path

from dual_arm_app.backend.object_grasp_model import RigidTransform, compose
from dual_arm_app.backend.rigid_grasp_configuration import (
    AuthoritativeRigidGraspState,
    GRASP_LOCKED,
    GRASP_UNLOCKED,
    PLANNER_INTEGRATION_READY,
    RigidGraspConfigurationError,
    RigidGraspContent,
    RigidGraspPose,
    SYNTHETIC_DEFAULT_DRAFT,
    compute_grasp_content_revision,
)
from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import


ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "dual_arm_app/backend/rigid_grasp_configuration.py"


def _default_payload():
    return AuthoritativeRigidGraspState().snapshot()["draft"]


class RigidGraspCoreTests(unittest.TestCase):
    def test_initial_draft_and_unlocked_semantics(self):
        state = AuthoritativeRigidGraspState().snapshot()
        self.assertEqual(state["state"], GRASP_UNLOCKED)
        self.assertEqual(state["draft_source"], SYNTHETIC_DEFAULT_DRAFT)
        self.assertEqual(state["draft"]["left"]["translation_m"], [0.0, 0.25, 0.0])
        self.assertEqual(state["draft"]["right"]["translation_m"], [0.0, -0.25, 0.0])
        self.assertEqual(state["draft"]["left"]["rpy_rad"], [0.0, 0.0, 0.0])
        self.assertEqual(state["draft"]["right"]["rpy_rad"], [0.0, 0.0, 0.0])
        self.assertFalse(state["physically_calibrated"])
        self.assertEqual(state["planner_integration"], PLANNER_INTEGRATION_READY)

    def test_left_and_right_translation_and_rpy_are_validated(self):
        payload = _default_payload()
        payload["left"] = {
            "translation_m": [0.1, 0.2, 0.3],
            "rpy_rad": [0.4, -0.5, 0.6],
        }
        payload["right"] = {
            "translation_m": [-0.1, -0.2, -0.3],
            "rpy_rad": [-0.4, 0.5, -0.6],
        }
        locked = AuthoritativeRigidGraspState().lock(payload)
        self.assertEqual(locked["state"], GRASP_LOCKED)
        self.assertEqual(locked["locked_snapshot"]["left"], payload["left"])
        self.assertEqual(locked["locked_snapshot"]["right"], payload["right"])

    def test_nan_inf_bool_invalid_shapes_and_ranges_are_rejected(self):
        invalid_payloads = []
        for value in (math.nan, math.inf, True):
            payload = _default_payload()
            payload["left"]["translation_m"][0] = value
            invalid_payloads.append(payload)
        for side, field, value in (
            ("left", "translation_m", [1, 2]),
            ("right", "rpy_rad", [1, 2, 3, 4]),
            ("left", "rpy_rad", [0, 0, 2 * math.pi + 0.01]),
            ("right", "translation_m", [0, -3.01, 0]),
        ):
            payload = _default_payload()
            payload[side][field] = value
            invalid_payloads.append(payload)
        for payload in invalid_payloads:
            with self.subTest(payload=payload), self.assertRaises(RigidGraspConfigurationError):
                AuthoritativeRigidGraspState().lock(payload)

    def test_revision_is_deterministic_and_sensitive_to_all_pose_groups(self):
        base = RigidGraspContent.from_payload(_default_payload())
        revision = compute_grasp_content_revision(base)
        self.assertEqual(revision, compute_grasp_content_revision(base))
        for side, field in (
            ("left", "translation_m"),
            ("right", "translation_m"),
            ("left", "rpy_rad"),
            ("right", "rpy_rad"),
        ):
            payload = _default_payload()
            payload[side][field][0] = 0.125
            changed = RigidGraspContent.from_payload(payload)
            with self.subTest(side=side, field=field):
                self.assertNotEqual(revision, compute_grasp_content_revision(changed))

    def test_locked_snapshot_is_immutable_and_payload_is_copy_safe(self):
        state = AuthoritativeRigidGraspState()
        payload = _default_payload()
        locked = state.lock(payload)
        payload["left"]["translation_m"][1] = 99
        locked["locked_snapshot"]["left"]["translation_m"][1] = 88
        fresh = state.snapshot()
        self.assertEqual(fresh["locked_snapshot"]["left"]["translation_m"][1], 0.25)
        snapshot = state._locked
        self.assertIsNotNone(snapshot)
        with self.assertRaises((AttributeError, TypeError)):
            snapshot.lock_generation = 99

    def test_unlock_preserves_values_and_relock_gets_new_identity(self):
        state = AuthoritativeRigidGraspState()
        first = state.lock(_default_payload())
        unlocked = state.unlock()
        self.assertEqual(unlocked["state"], GRASP_UNLOCKED)
        self.assertEqual(unlocked["draft"], {
            "left": first["locked_snapshot"]["left"],
            "right": first["locked_snapshot"]["right"],
        })
        relocked = state.lock(unlocked["draft"])
        self.assertGreater(relocked["lock_generation"], first["lock_generation"])
        self.assertNotEqual(
            relocked["authoritative_revision"], first["authoritative_revision"]
        )
        self.assertEqual(
            relocked["locked_snapshot"]["content_revision"],
            first["locked_snapshot"]["content_revision"],
        )

    def test_object_grasp_model_composition_remains_authoritative_math(self):
        content = RigidGraspContent(
            left=RigidGraspPose((0.1, 0.2, 0.3), (0.2, -0.1, 0.4)),
            right=RigidGraspPose((-0.1, -0.2, 0.1), (-0.3, 0.5, -0.2)),
        )
        world_t_center = RigidTransform.from_translation_rpy(
            (1.2, -0.7, 2.1), (0.4, 0.2, -0.6)
        )
        targets = content.model.compute_world_grasp_targets(world_t_center)
        self.assertTrue(targets.world_T_left.almost_equal(
            compose(world_t_center, content.left.transform)
        ))
        self.assertTrue(targets.world_T_right.almost_equal(
            compose(world_t_center, content.right.transform)
        ))

    def test_module_has_only_pure_standard_library_and_transform_core_dependencies(self):
        source = MODULE.read_text(encoding="utf-8")
        tree = ast.parse(source)
        imported = {
            node.module.split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        }
        imported.update(
            alias.name.split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        )
        self.assertLessEqual(
            imported,
            {"__future__", "hashlib", "json", "math", "threading", "dataclasses", "typing", "dual_arm_app", "object_grasp_model"},
        )
        for forbidden in ("rclpy", "jaka_msgs", "moveit_msgs"):
            self.assertNotIn(forbidden, source)


class RigidGraspBackendStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def route(self, path):
        return next(route.endpoint for route in self.backend.app.routes if route.path == path)

    def test_get_is_read_only_copy_safe_and_exposes_calibration_revision(self):
        first = self.route("/api/digital-twin/grasp-configuration")()
        first["grasp"]["draft"]["left"]["translation_m"][1] = 99
        second = self.route("/api/digital-twin/grasp-configuration")()
        self.assertTrue(second["ok"])
        self.assertEqual(second["grasp"]["state"], GRASP_UNLOCKED)
        self.assertEqual(second["grasp"]["draft"]["left"]["translation_m"][1], 0.25)
        self.assertTrue(second["grasp"]["world_calibration_revision"].startswith("sha256:"))

    def test_lock_unlock_and_relock_invalidate_phase4_readiness(self):
        node = self.backend.node
        payload = _default_payload()
        request = self.backend.DigitalTwinGraspLockRequest(**payload)
        node.phase4_unified_validation_report = {"old": True}
        locked = self.route("/api/digital-twin/grasp-configuration/lock")(request)
        self.assertEqual(locked["grasp"]["state"], GRASP_LOCKED)
        self.assertIsNone(node.phase4_unified_validation_report)
        first_revision = locked["grasp"]["authoritative_revision"]
        node.phase4_unified_validation_report = {"old": True}
        unlocked = self.route("/api/digital-twin/grasp-configuration/unlock")()
        self.assertEqual(unlocked["grasp"]["state"], GRASP_UNLOCKED)
        self.assertIsNone(node.phase4_unified_validation_report)
        node.phase4_unified_validation_report = {"old": True}
        relocked = self.route("/api/digital-twin/grasp-configuration/lock")(
            self.backend.DigitalTwinGraspLockRequest(**unlocked["grasp"]["draft"])
        )
        self.assertNotEqual(relocked["grasp"]["authoritative_revision"], first_revision)
        self.assertIsNone(node.phase4_unified_validation_report)
        gate = node.phase4_execution_gate_state("old")
        self.assertFalse(gate["execution_ready"])
        self.assertIn("AUTHORITATIVE_PLAN_IDENTITY_MISSING", gate["blocking_reasons"])

    def test_routes_are_software_state_only_and_stop_remains_ungated(self):
        source = (ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py").read_text(
            encoding="utf-8"
        )
        for route in (
            '/api/digital-twin/grasp-configuration")',
            '/api/digital-twin/grasp-configuration/lock")',
            '/api/digital-twin/grasp-configuration/unlock")',
        ):
            self.assertIn(route, source)
        core = MODULE.read_text(encoding="utf-8")
        for forbidden in (
            "joint_move(", "linear_move(", "servo(", "start_jog(", "home(",
            "run_sequence(", "/api/program/run",
        ):
            self.assertNotIn(forbidden, core)
        stop = source.split('@app.post("/api/stop")', 1)[1].split("@app.", 1)[0]
        self.assertNotIn("phase4_execution_gate_state", stop)


if __name__ == "__main__":
    unittest.main()
