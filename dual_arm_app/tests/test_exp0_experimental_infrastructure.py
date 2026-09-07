"""Offline acceptance tests for EXP-0 recording and persistence."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import uuid
import unittest

TEST_ARTIFACT_ROOT = (
    Path(__file__).resolve().parents[2] / ".exp0_test_artifacts"
)


def preserved_test_root(label: str) -> Path:
    root = TEST_ARTIFACT_ROOT / f"{label}-{uuid.uuid4().hex}"
    root.mkdir(parents=True, exist_ok=False)
    return root


from dual_arm_app.backend.experimental_infrastructure import (
    ALIGNMENT_SEMANTIC,
    ExperimentError,
    ExperimentRecorder,
    ExperimentRunStore,
    SCHEMA_VERSION,
    TorqueCache,
    align_planned_to_actual,
    validate_plan_snapshot,
)


def artifact():
    left = ((0.0,) * 6, (1.0,) * 6, (2.0,) * 6)
    right = ((0.0,) * 6, (-1.0,) * 6, (-2.0,) * 6)
    return SimpleNamespace(
        artifact_fingerprint="artifact-exp0",
        plan_fingerprint="plan-exp0",
        validation_report_version="phase4-report-v1",
        trajectory_name="exp0-fixture",
        calibration_revision="calibration-1",
        model_calibration_revision="calibration-1",
        grasp_content_revision="grasp-content-1",
        grasp_lock_generation=3,
        grasp_lock_revision="grasp-lock-3",
        common_timestamps_s=(0.0, 1.0, 2.0),
        left_positions_rad=left,
        right_positions_rad=right,
        combined_positions_rad=tuple(a + b for a, b in zip(left, right)),
        duration_s=2.0,
    )


def report():
    return {
        "overall_status": "PASS",
        "plan_fingerprint": "plan-exp0",
        "report_version": "phase4-report-v1",
    }


def plan_snapshot():
    samples = []
    objects = []
    frozen = artifact()
    for index, timestamp in enumerate(frozen.common_timestamps_s):
        left = list(frozen.left_positions_rad[index])
        right = list(frozen.right_positions_rad[index])
        samples.append({
            "sample_index": index,
            "time_from_start_s": timestamp,
            "left": left,
            "right": right,
            "combined": left + right,
        })
        objects.append({
            "sample_index": index,
            "time_from_start_s": timestamp,
            "object_pose": {
                "translation_m": [timestamp, 0.0, 0.8],
                "rpy_rad": [0.0, 0.0, 0.0],
            },
            "left_target": {"translation_m": [timestamp, 0.2, 0.8]},
            "right_target": {"translation_m": [timestamp, -0.2, 0.8]},
        })
    return {
        "trajectory_name": "exp0-fixture",
        "combined_timestamps_s": [0.0, 1.0, 2.0],
        "combined_duration_s": 2.0,
        "combined_path": samples,
        "combined_object_samples": objects,
        "waypoints": [{"identifier": "P1", "speed_percent": 40, "acceleration_percent": 35}],
    }


def base_manifest(run_id):
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "label": "fixture",
        "state": "CLOSED",
        "created_at": "2026-09-04T00:00:00.000Z",
        "trajectory_name": "fixture",
        "result": {"sample_counts": {"planned": 1}},
    }


def base_raw(run_id):
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "planned": {
            "samples": [{
                "sample_index": 0,
                "time_from_start_s": 0.0,
                "left": {"joints_rad": [0.0] * 6},
                "right": {"joints_rad": [0.0] * 6},
                "cartesian": None,
            }],
        },
        "observations": [],
    }


class ExperimentStoreTests(unittest.TestCase):
    def setUp(self):
        self.test_root = preserved_test_root("store")
        self.store = ExperimentRunStore(self.test_root / "runs")

    def test_persistence_relabel_load_and_explicit_delete(self):
        run_id = self.store.generate_run_id(
            now=datetime(2026, 9, 4, tzinfo=timezone.utc)
        )
        self.store.create(base_manifest(run_id), base_raw(run_id))
        run_dir = self.store.root / run_id
        self.assertEqual(
            {path.name for path in run_dir.iterdir()},
            {"manifest.json", "raw.json", "aligned.json", "planned.csv", "observed.csv"},
        )
        self.assertEqual(self.store.load(run_id)["manifest"]["label"], "fixture")
        manifest = self.store.relabel(run_id, "renamed run")
        self.assertEqual(manifest["label"], "renamed run")
        self.assertEqual(self.store.list_runs()[0]["label"], "renamed run")
        self.store.delete(run_id)
        self.assertFalse(run_dir.exists())

    def test_path_containment_and_immutable_server_id_validation(self):
        for invalid in ("../escape", "/tmp/escape", "exp-not-valid", "", None):
            with self.subTest(invalid=invalid), self.assertRaises(ExperimentError):
                self.store.load(invalid)

    def test_corrupt_run_is_listed_as_warning_without_breaking_good_runs(self):
        good_id = self.store.generate_run_id()
        self.store.create(base_manifest(good_id), base_raw(good_id))
        corrupt_id = self.store.generate_run_id()
        corrupt_dir = self.store.root / corrupt_id
        corrupt_dir.mkdir()
        (corrupt_dir / "manifest.json").write_text("{broken", encoding="utf-8")
        rows = {item["run_id"]: item for item in self.store.list_runs()}
        self.assertEqual(rows[corrupt_id]["state"], "CORRUPT")
        self.assertIn("Corrupt experiment run", rows[corrupt_id]["warning"])
        self.assertEqual(rows[good_id]["state"], "CLOSED")

        valid_json_corrupt_id = self.store.generate_run_id()
        valid_json_corrupt_dir = self.store.root / valid_json_corrupt_id
        valid_json_corrupt_dir.mkdir()
        for name in ("manifest.json", "raw.json", "aligned.json"):
            (valid_json_corrupt_dir / name).write_text("[]", encoding="utf-8")
        rows = {item["run_id"]: item for item in self.store.list_runs()}
        self.assertEqual(rows[valid_json_corrupt_id]["state"], "CORRUPT")

    def test_delete_refuses_unexpected_entries(self):
        run_id = self.store.generate_run_id()
        self.store.create(base_manifest(run_id), base_raw(run_id))
        (self.store.root / run_id / "unexpected.txt").write_text("preserve", encoding="utf-8")
        with self.assertRaisesRegex(ExperimentError, "unexpected entries"):
            self.store.delete(run_id)
        self.assertTrue((self.store.root / run_id / "unexpected.txt").exists())


class PlanAndAlignmentTests(unittest.TestCase):
    def test_plan_snapshot_is_bound_to_frozen_samples_and_cartesian_data(self):
        checked = validate_plan_snapshot(plan_snapshot(), artifact())
        self.assertEqual(checked["trajectory_name"], "exp0-fixture")
        cases = []
        wrong_joint = plan_snapshot()
        wrong_joint["combined_path"][1]["left"][0] = 99.0
        cases.append(wrong_joint)
        wrong_time = plan_snapshot()
        wrong_time["combined_timestamps_s"][1] = 1.1
        cases.append(wrong_time)
        wrong_index = plan_snapshot()
        wrong_index["combined_path"][1]["sample_index"] = 8
        cases.append(wrong_index)
        wrong_cartesian = plan_snapshot()
        wrong_cartesian["combined_object_samples"][0]["object_pose"]["translation_m"] = [0.0, "bad", 0.8]
        cases.append(wrong_cartesian)
        for invalid in cases:
            with self.subTest(invalid=invalid), self.assertRaises(ExperimentError):
                validate_plan_snapshot(invalid, artifact())

    def test_alignment_is_analysis_only_interpolation_and_preserves_raw(self):
        raw = {
            "run_id": "exp-analysis",
            "execution": {"start_time_unix_ns": 1_000_000_000},
            "planned": {"samples": [
                {"time_from_start_s": 0.0, "left": {"joints_rad": [0.0] * 6}, "right": {"joints_rad": [0.0] * 6}},
                {"time_from_start_s": 2.0, "left": {"joints_rad": [2.0] * 6}, "right": {"joints_rad": [-2.0] * 6}},
            ]},
            "observations": [{
                "actual": {
                    "left": {"valid": True, "received_at_ms": 2000, "joints_rad": [0.9] * 6},
                    "right": {"valid": False, "received_at_ms": None, "joints_rad": None},
                },
            }, {
                "actual": {
                    "left": {"valid": True, "received_at_ms": 4000, "joints_rad": [2.0] * 6},
                    "right": {"valid": True, "received_at_ms": 4000, "joints_rad": [-2.0] * 6},
                },
            }],
        }
        before = deepcopy(raw)
        aligned = align_planned_to_actual(raw)
        self.assertEqual(raw, before)
        self.assertEqual(aligned["semantic"], ALIGNMENT_SEMANTIC)
        self.assertEqual(aligned["samples"][0]["status"], "INTERPOLATED")
        self.assertEqual(aligned["samples"][0]["planned_joints_rad"], [1.0] * 6)
        self.assertEqual(aligned["samples"][1]["status"], "MISSING_ACTUAL")
        self.assertEqual(aligned["samples"][2]["status"], "OUT_OF_RANGE")


class TorqueCacheTests(unittest.TestCase):
    def test_missing_valid_stale_and_invalid_are_explicit(self):
        cache = TorqueCache(stale_after_ms=50)
        missing = cache.snapshot(now_ms=100)
        self.assertEqual(missing["left"]["status"], "MISSING")
        cache.update("left", [1.0] * 6, received_at_ms=100)
        self.assertTrue(cache.snapshot(now_ms=120)["left"]["fresh"])
        stale = cache.snapshot(now_ms=151)["left"]
        self.assertEqual(stale["status"], "STALE")
        self.assertFalse(stale["fresh"])
        cache.update("right", [1.0] * 5, received_at_ms=120)
        self.assertEqual(cache.snapshot(now_ms=120)["right"]["status"], "INVALID")


class RecorderLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.test_root = preserved_test_root("recorder")
        self.clock_ms = 1000
        self.store = ExperimentRunStore(self.test_root / "runs")
        self.recorder = ExperimentRecorder(
            self.store,
            actual_stale_after_ms=100,
            wall_time_ms=lambda: self.clock_ms,
        )

    def arm(self):
        return self.recorder.arm(
            artifact=artifact(), artifact_generation=7,
            phase4_report=report(), phase4_generation=7,
            metadata={"label": "EXP-0", "path_type": "CURVED", "fixture_condition": "RIGID", "notes": "fixture"},
            plan_snapshot=plan_snapshot(), motion_configuration={"servo_step_num": 3},
            robot_configuration={"left": {"prefix": "/left_jaka_driver"}},
            grasp_snapshot={
                "left": {"translation_m": [0.0, 0.2, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
                "right": {"translation_m": [0.0, -0.2, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
            },
        )

    def test_arm_requires_matching_phase4_pass_and_generation(self):
        failed = report()
        failed["overall_status"] = "FAIL"
        with self.assertRaisesRegex(ExperimentError, "Phase4 PASS"):
            self.recorder.arm(
                artifact=artifact(), artifact_generation=7,
                phase4_report=failed, phase4_generation=7,
                metadata={},
            )
        with self.assertRaisesRegex(ExperimentError, "generations do not match"):
            self.recorder.arm(
                artifact=artifact(), artifact_generation=7,
                phase4_report=report(), phase4_generation=8,
                metadata={},
            )

    def test_command_actual_torque_observation_and_authoritative_terminal_close(self):
        armed = self.arm()
        run_id = armed["run_id"]
        self.recorder.bind_execution({
            "trajectory_id": "trajectory-exp0",
            "start_time_unix_ns": 1_000_000_000,
            "duration_s": 2.0,
            "artifact_fingerprint": "artifact-exp0",
            "plan_fingerprint": "plan-exp0",
            "artifact_generation": 7,
        })

        def phase_state(terminal=False):
            driver = lambda value: {
                "commanded_sample": {
                    "valid": True, "sample_index": 1,
                    "time_from_start_s": 1.0,
                    "joints_rad": [value] * 6,
                    "source": "driver memory",
                },
            }
            return {
                "execution": {"trajectory_id": "trajectory-exp0", "state": "RUNNING"},
                "driver_feedback": {
                    "trajectory_id": "trajectory-exp0",
                    "authoritative": True,
                    "terminal": terminal,
                    "combined_state": "COMPLETED" if terminal else "RUNNING",
                    "reason": "BOTH_DRIVERS_COMPLETED" if terminal else None,
                    "common_timeline": {"elapsed_s": 2.0 if terminal else 1.0, "duration_s": 2.0},
                    "drivers": {"left": driver(1.0), "right": driver(-1.0)},
                },
            }

        actual = {
            "source": "jaka_port10000_actual_feedback",
            "left": {"valid": True, "joint": [0.9] * 6, "received_at_ms": 990, "mapping": "port10000"},
            "right": {"valid": True, "joint": [-0.9] * 6, "received_at_ms": 980, "mapping": "port10000"},
        }
        torque = TorqueCache(stale_after_ms=100)
        torque.update("left", [2.0] * 6, received_at_ms=990)
        self.recorder.observe(phase_state(), actual, torque.snapshot(now_ms=1000))
        self.assertEqual(self.recorder.state()["state"], "RECORDING")
        self.clock_ms = 1100
        actual["left"]["received_at_ms"] = 1090
        self.recorder.observe(phase_state(terminal=True), actual, torque.snapshot(now_ms=1100))
        self.assertEqual(self.recorder.state()["state"], "CLOSED")
        loaded = self.store.load(run_id)
        self.assertEqual(loaded["manifest"]["result"]["status"], "COMPLETED")
        self.assertEqual(loaded["manifest"]["result"]["sample_counts"]["commanded_left"], 2)
        self.assertEqual(loaded["manifest"]["result"]["sample_counts"]["actual_right"], 2)
        self.assertEqual(loaded["manifest"]["result"]["sample_counts"]["torque_left"], 2)
        self.assertEqual(loaded["manifest"]["result"]["sample_counts"]["torque_right"], 0)
        self.assertEqual(loaded["aligned"]["semantic"], ALIGNMENT_SEMANTIC)
        self.assertEqual(loaded["exp1_analysis"]["status"], "PARTIAL")
        self.assertEqual(loaded["exp1_analysis"]["coverage"]["sides"]["left"]["usable_rows"], 1)
        self.assertEqual(loaded["exp1_analysis"]["coverage"]["sides"]["right"]["usable_rows"], 0)
        self.assertTrue((self.store.root / run_id / "exp1_analysis.json").is_file())
        self.assertIn("exp2_analysis", loaded)
        self.assertEqual(loaded["exp2_analysis"]["status"], "UNAVAILABLE")
        self.assertTrue((self.store.root / run_id / "exp2_analysis.json").is_file())
        self.assertTrue(self.recorder.state()["ok"])
        self.assertEqual(loaded["raw"]["observations"][0]["relative_pose"]["status"], "NOT_MEASURED")

    def test_disarm_only_applies_before_binding(self):
        self.arm()
        self.assertEqual(self.recorder.disarm()["state"], "DISARMED")
        with self.assertRaises(ExperimentError):
            self.recorder.disarm()

    def test_actual_missing_timestamp_is_error_not_fabricated_freshness(self):
        result = self.recorder._actual(
            {"valid": True, "joint": [0.1] * 6, "received_at_ms": None},
            "test-cache",
            1000,
        )
        self.assertFalse(result["valid"])
        self.assertFalse(result["fresh"])
        self.assertEqual(result["status"], "ERROR")
        self.assertIsNone(result["age_ms"])


class BackendWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def test_arm_binds_current_artifact_and_phase4_pass_without_motion(self):
        captured = {}
        frozen = artifact()
        recorder = SimpleNamespace(arm=lambda **kwargs: captured.update(kwargs) or {"state": "ARMED"})
        locked = SimpleNamespace(
            content_revision="grasp-content-1",
            lock_generation=3,
            lock_revision="grasp-lock-3",
            as_payload=lambda: {"left": {}, "right": {}},
        )
        fake = SimpleNamespace(
            _phase5_artifact_snapshot=lambda: (frozen, 7, None),
            phase4_execution_gate_state=lambda fingerprint: {"execution_ready": fingerprint == "plan-exp0"},
            phase4_unified_validation_lock=threading.Lock(),
            phase4_unified_validation_report=report(),
            phase4_unified_validation_generation=7,
            rigid_grasp_configuration=SimpleNamespace(locked_snapshot=lambda: locked),
            experiment_recorder=recorder,
            phase5_motion_settings={"servo_step_num": 3},
            phase5_motion_configuration_error=None,
            cfg={"left": {}, "right": {}},
        )
        result = self.backend.DualJakaWebNode.arm_experiment_recorder(
            fake, {"path_type": "CUSTOM", "fixture_condition": "NONE", "plan_snapshot": plan_snapshot()}
        )
        self.assertEqual(result["state"], "ARMED")
        self.assertIs(captured["artifact"], frozen)
        self.assertEqual(captured["phase4_generation"], 7)

    def test_experiment_api_prefix_is_d33_no_motion_for_all_methods(self):
        for method in ("GET", "POST", "PATCH", "DELETE"):
            with self.subTest(method=method):
                self.assertFalse(self.backend._d33_is_motion_command("/api/experiments/example/run", method))

    def test_recorder_failure_cannot_change_accepted_phase5_result(self):
        expected = {
            "ok": True,
            "accepted": True,
            "execution": {
                "trajectory_id": "accepted",
                "artifact_fingerprint": "artifact-exp0",
                "artifact_generation": 7,
            },
        }
        errors = []
        fake = SimpleNamespace(
            _phase5_artifact_snapshot=lambda: (artifact(), 7, None),
            phase5_execution_coordinator=SimpleNamespace(execute=lambda _confirmed: deepcopy(expected)),
            phase5_recovery_initial_lock=threading.Lock(),
            phase5_recovery_initial=None,
            experiment_recorder=SimpleNamespace(
                bind_execution=lambda _execution: (_ for _ in ()).throw(RuntimeError("disk unavailable")),
                record_error=lambda error: errors.append(str(error)),
            ),
        )
        result = self.backend.DualJakaWebNode.execute_phase5_execution(fake, True)
        self.assertEqual(result, expected)
        self.assertEqual(errors, ["disk unavailable"])


if __name__ == "__main__":
    unittest.main()
