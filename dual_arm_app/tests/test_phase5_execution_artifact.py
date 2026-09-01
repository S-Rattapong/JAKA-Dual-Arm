"""Offline acceptance tests for Phase 5 P5.1-P5.3."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import FrozenInstanceError
import unittest

from dual_arm_app.backend.phase4_unified_validation import (
    REPORT_VERSION,
    REQUIRED_CHECKS,
)
from dual_arm_app.backend.phase5_execution_artifact import (
    ARTIFACT_VERSION,
    TIMELINE_SEMANTIC,
    Phase5ExecutionArtifactError,
    freeze_validated_execution_artifact,
)


def valid_plan():
    times = [0.0, 0.5, 1.0]
    path = []
    objects = []
    for index, timestamp in enumerate(times):
        left = [index + joint / 10.0 for joint in range(6)]
        right = [index + 1.0 + joint / 10.0 for joint in range(6)]
        path.append({
            "sample_index": index,
            "time_from_start_s": timestamp,
            "left": left,
            "right": right,
            "combined": left + right,
        })
        objects.append({
            "sample_index": index,
            "time_from_start_s": timestamp,
            "phase": "APPROACH" if index == 0 else "RIGID",
        })
    return {
        "ok": True,
        "planner_status": "READY",
        "plan_only": True,
        "trajectory_name": "p5_fixture",
        "calibration": {
            "revision": "sha256:calibration",
            "model_revision": "sha256:calibration",
            "revision_status": "MATCH",
        },
        "grasp": {
            "content_revision": "grasp-content-1",
            "lock_generation": 4,
            "lock_revision": "grasp-lock-4",
        },
        "planning_start_state_rad": {
            "left": [0.0] * 6,
            "right": [0.0] * 6,
        },
        "combined_path": path,
        "combined_object_samples": objects,
        "combined_timestamps_s": times,
        "combined_duration_s": times[-1],
    }


def valid_report(fingerprint="plan-fingerprint-1"):
    checks = {name: {"status": "PASS"} for name in REQUIRED_CHECKS}
    return {
        "report_version": REPORT_VERSION,
        "plan_fingerprint": fingerprint,
        "overall_status": "PASS",
        "scope_policy": {"required_checks": list(REQUIRED_CHECKS)},
        "checks": checks,
        "execution_gate": {
            "execution_ready": True,
            "execution_ready_label": "YES",
            "status": "READY",
            "blocking_reasons": [],
        },
    }


def validation_start():
    return {
        "left": [0.01] * 6,
        "right": [-0.01] * 6,
    }


class Phase5ExecutionArtifactPureTests(unittest.TestCase):
    def freeze(self, plan=None, report=None):
        return freeze_validated_execution_artifact(
            plan or valid_plan(),
            report or valid_report(),
            validation_start_state=validation_start(),
        )

    def test_p5_1_freezes_immutable_validated_snapshot(self):
        plan = valid_plan()
        artifact = self.freeze(plan=plan)
        original = artifact.left_positions_rad[0][0]
        plan["combined_path"][0]["left"][0] = 999.0
        plan["combined_path"][0]["combined"][0] = 999.0
        self.assertEqual(artifact.left_positions_rad[0][0], original)
        with self.assertRaises(FrozenInstanceError):
            artifact.duration_s = 9.0

        payload = artifact.public_payload()
        payload["trajectory"]["left"]["positions_rad"][0][0] = 555.0
        self.assertEqual(artifact.left_positions_rad[0][0], original)
        self.assertEqual(payload["artifact_version"], ARTIFACT_VERSION)
        self.assertEqual(payload["artifact_status"], "FROZEN")

    def test_p5_1_fingerprint_is_deterministic_and_authority_bound(self):
        first = self.freeze()
        second = self.freeze()
        changed = self.freeze(report=valid_report("different-plan-fingerprint"))
        self.assertEqual(first.artifact_fingerprint, second.artifact_fingerprint)
        self.assertNotEqual(first.artifact_fingerprint, changed.artifact_fingerprint)
        self.assertEqual(first.plan_fingerprint, "plan-fingerprint-1")

    def test_p5_2_exactly_splits_each_canonical_12_joint_sample(self):
        artifact = self.freeze()
        self.assertEqual(artifact.sample_count, 3)
        for combined, left, right in zip(
            artifact.combined_positions_rad,
            artifact.left_positions_rad,
            artifact.right_positions_rad,
        ):
            self.assertEqual(len(left), 6)
            self.assertEqual(len(right), 6)
            self.assertEqual(combined, left + right)

    def test_p5_3_keeps_one_shared_timestamp_authority(self):
        artifact = self.freeze()
        payload = artifact.public_payload()
        trajectory = payload["trajectory"]
        self.assertEqual(artifact.common_timestamps_s, (0.0, 0.5, 1.0))
        self.assertEqual(trajectory["common_timestamps_s"], [0.0, 0.5, 1.0])
        self.assertEqual(trajectory["timestamp_authority"], TIMELINE_SEMANTIC)
        self.assertNotIn("timestamps_s", trajectory["left"])
        self.assertNotIn("timestamps_s", trajectory["right"])
        self.assertEqual(trajectory["sample_count"], len(trajectory["left"]["positions_rad"]))
        self.assertEqual(trajectory["sample_count"], len(trajectory["right"]["positions_rad"]))

    def test_p5_3_rejects_length_timestamp_and_duration_mismatch(self):
        cases = []
        missing_time = valid_plan()
        missing_time["combined_timestamps_s"].pop()
        cases.append(missing_time)
        wrong_time = valid_plan()
        wrong_time["combined_timestamps_s"][1] = 0.6
        cases.append(wrong_time)
        wrong_duration = valid_plan()
        wrong_duration["combined_duration_s"] = 1.1
        cases.append(wrong_duration)
        for plan in cases:
            with self.subTest(plan=plan), self.assertRaises(Phase5ExecutionArtifactError):
                self.freeze(plan=plan)

    def test_p5_2_rejects_joint_shape_order_and_split_mismatch(self):
        cases = []
        wrong_split = valid_plan()
        wrong_split["combined_path"][1]["combined"][0] += 0.1
        cases.append(wrong_split)
        wrong_count = valid_plan()
        wrong_count["combined_path"][0]["left"].pop()
        cases.append(wrong_count)
        wrong_index = valid_plan()
        wrong_index["combined_path"][2]["sample_index"] = 9
        cases.append(wrong_index)
        for plan in cases:
            with self.subTest(plan=plan), self.assertRaises(Phase5ExecutionArtifactError):
                self.freeze(plan=plan)

    def test_p5_1_rejects_unvalidated_or_blocked_phase4_report(self):
        failed = valid_report()
        failed["overall_status"] = "FAIL"
        with self.assertRaises(Phase5ExecutionArtifactError):
            self.freeze(report=failed)

        blocked = valid_report()
        blocked["execution_gate"]["execution_ready"] = False
        blocked["execution_gate"]["execution_ready_label"] = "NO"
        with self.assertRaises(Phase5ExecutionArtifactError):
            self.freeze(report=blocked)

        missing_check = valid_report()
        missing_check["checks"][REQUIRED_CHECKS[0]]["status"] = "NOT_EVALUATED"
        with self.assertRaises(Phase5ExecutionArtifactError):
            self.freeze(report=missing_check)


class Phase5BackendLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def test_phase4_pass_freezes_and_invalidation_revokes_current_artifact(self):
        node = self.backend.node
        originals = (
            node._phase4_authoritative_input_validation,
            node.validate_digital_twin_phase4_trajectory,
            node.validate_digital_twin_phase4_collision,
            self.backend.build_phase4_unified_report,
        )
        authority = {
            "status": "PASS",
            "grasp_content_revision": "grasp-content-1",
            "lock_generation": 4,
            "lock_revision": "grasp-lock-4",
            "calibration_revision": "sha256:calibration",
            "model_calibration_revision": "sha256:calibration",
        }
        try:
            node._phase4_authoritative_input_validation = lambda _plan: dict(authority)
            node.validate_digital_twin_phase4_trajectory = lambda _request: {}
            node.validate_digital_twin_phase4_collision = lambda _request: {}
            self.backend.build_phase4_unified_report = lambda *_args, **_kwargs: valid_report()
            report = node.validate_digital_twin_phase4_unified({
                "plan": valid_plan(),
                "start_state": validation_start(),
            })
            self.assertEqual(report["overall_status"], "PASS")
            state = node.phase5_execution_artifact_state()
            self.assertTrue(state["available"])
            self.assertEqual(state["status"], "FROZEN")
            self.assertEqual(state["artifact"]["trajectory"]["sample_count"], 3)

            node.invalidate_phase4_unified_validation("TEST INPUT CHANGED")
            invalidated = node.phase5_execution_artifact_state()
            self.assertFalse(invalidated["available"])
            self.assertEqual(invalidated["status"], "NOT_FROZEN")
            self.assertEqual(invalidated["reason"], "TEST INPUT CHANGED")
        finally:
            (
                node._phase4_authoritative_input_validation,
                node.validate_digital_twin_phase4_trajectory,
                node.validate_digital_twin_phase4_collision,
                self.backend.build_phase4_unified_report,
            ) = originals

    def test_readonly_endpoint_only_exposes_snapshot(self):
        route = next(
            item.endpoint for item in self.backend.app.routes
            if getattr(item, "path", None) == "/api/digital-twin/phase5/execution-artifact"
        )
        payload = route()
        self.assertTrue(payload["ok"])
        self.assertIn(payload["status"], {"FROZEN", "NOT_FROZEN"})

    def test_phase5_artifact_module_contains_no_motion_surface(self):
        from pathlib import Path
        source = Path(
            "dual_arm_app/backend/phase5_execution_artifact.py"
        ).read_text(encoding="utf-8")
        for forbidden in (
            "joint_move(", "linear_move(", "start_jog(", "/api/jog",
            "/api/program/run", "rclpy", "jaka_driver",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
