from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from types import SimpleNamespace
import uuid
import unittest

from dual_arm_app.backend.experimental_infrastructure import (
    ALIGNMENT_SEMANTIC,
    ExperimentRunStore,
    SCHEMA_VERSION,
    EXP2_URDF_PATH,
)
from dual_arm_app.backend.experimental_rigid_grasp import (
    CENTER_SIGN,
    EXP2_SCHEMA_VERSION,
    EXP2_SEMANTIC,
    UrdfFkModel,
    analyze_rigid_grasp,
    exp2_input_provenance,
)

ROOT = Path(__file__).resolve().parents[2]
FIXTURE_URDF = ROOT / "dual_arm_app/tests/fixtures/exp2_simple_dual.urdf"
WEB_URDF = ROOT / "dual_arm_app/web/assets/dual_jaka_a12_web.urdf"
TEST_ARTIFACT_ROOT = ROOT / ".exp2_test_artifacts"


def preserved_test_root(label: str) -> Path:
    root = TEST_ARTIFACT_ROOT / f"{label}-{uuid.uuid4().hex}"
    root.mkdir(parents=True, exist_ok=False)
    return root


def locked_grasp():
    return {
        "left": {"translation_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
        "right": {"translation_m": [1.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    }


def evidence(*, right_translation=0.0, right_rotation=0.0, valid=True):
    start_ns = 1_000_000_000_000
    start_ms = start_ns / 1e6
    observations = []
    for index, offset_ms in enumerate((0, 100)):
        left = {
            "valid": valid,
            "fresh": valid,
            "received_at_ms": start_ms + offset_ms,
            "joints_rad": [0.0] * 6 if valid else None,
            "source": "fixture-left",
        }
        right_joints = [right_translation, right_rotation, 0.0, 0.0, 0.0, 0.0]
        right = {
            "valid": valid,
            "fresh": valid,
            "received_at_ms": start_ms + offset_ms,
            "joints_rad": right_joints if valid else None,
            "source": "fixture-right",
        }
        observations.append({"observation_index": index, "actual": {"left": left, "right": right}})
    raw = {
        "run_id": "exp-fixture",
        "execution": {"start_time_unix_ns": start_ns, "duration_s": 0.1},
        "observations": observations,
    }
    manifest = {
        "run_id": "exp-fixture",
        "locked_grasp": locked_grasp(),
        "actual_source_semantics": "fixture encoder evidence",
    }
    return raw, manifest


class Exp2MathTests(unittest.TestCase):
    def test_reference_identity_has_zero_error(self):
        raw, manifest = evidence()
        before = (deepcopy(raw), deepcopy(manifest))
        result = analyze_rigid_grasp(raw, manifest, FIXTURE_URDF)
        self.assertEqual((raw, manifest), before)
        self.assertEqual(result["status"], "AVAILABLE")
        self.assertEqual(result["semantic"], EXP2_SEMANTIC)
        self.assertEqual(result["center_disagreement_definition"], CENTER_SIGN)
        self.assertEqual(result["coverage"]["usable_synchronized_samples"], 2)
        self.assertAlmostEqual(result["relative_translation"]["magnitude_m"]["max"], 0.0, places=12)
        self.assertAlmostEqual(result["relative_orientation"]["angle"]["max"], 0.0, places=12)
        self.assertAlmostEqual(result["center_consistency"]["translation_magnitude_m"]["max"], 0.0, places=12)
        self.assertFalse(result["center_consistency"]["averaged_center"])
        self.assertFalse(result["actual_source_semantics"]["external_metrology"])

    def test_pure_translation_error_and_center_disagreement(self):
        result = analyze_rigid_grasp(*evidence(right_translation=0.1), FIXTURE_URDF)
        axis = result["relative_translation"]["axes_m"]["x"]
        self.assertAlmostEqual(axis["mean"], 0.1, places=12)
        self.assertAlmostEqual(axis["rmse"], 0.1, places=12)
        self.assertAlmostEqual(result["relative_translation"]["magnitude_m"]["max"], 0.1, places=12)
        center_x = result["center_consistency"]["translation_axes_m"]["x"]
        self.assertAlmostEqual(center_x["mean"], -0.1, places=12)
        self.assertAlmostEqual(result["center_consistency"]["translation_magnitude_m"]["rms"], 0.1, places=12)

    def test_pure_rotation_uses_rotation_angle_not_euler_subtraction(self):
        angle = 0.2
        result = analyze_rigid_grasp(*evidence(right_rotation=angle), FIXTURE_URDF)
        self.assertAlmostEqual(result["relative_orientation"]["angle"]["rms"], angle, places=10)
        self.assertAlmostEqual(result["relative_orientation"]["angle"]["max_deg"], math.degrees(angle), places=10)
        self.assertAlmostEqual(result["center_consistency"]["orientation_angle"]["rms"], angle, places=10)
        self.assertIn("no Euler subtraction", result["relative_orientation"]["error_definition"])

    def test_missing_actual_is_unavailable(self):
        result = analyze_rigid_grasp(*evidence(valid=False), FIXTURE_URDF)
        self.assertEqual(result["status"], "UNAVAILABLE")
        self.assertIn("at least two unique fresh Actual samples", result["error"])
        json.dumps(result, allow_nan=False)

    def test_calibrated_web_urdf_fk_has_expected_dual_chains(self):
        model = UrdfFkModel(WEB_URDF)
        left = model.fk("left_J6", {f"left_joint_{i}": 0.0 for i in range(1, 7)})
        right = model.fk("right_J6", {f"right_joint_{i}": 0.0 for i in range(1, 7)})
        self.assertEqual(len(left), 4)
        self.assertEqual(len(right), 4)
        self.assertTrue(all(math.isfinite(value) for matrix in (left, right) for row in matrix for value in row))
        self.assertNotEqual([left[i][3] for i in range(3)], [right[i][3] for i in range(3)])


class Exp2PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.root = preserved_test_root("persistence")
        self.store = ExperimentRunStore(self.root / "runs")

    def test_exp2_analysis_is_optional_persisted_and_delete_compatible(self):
        run_id = self.store.generate_run_id(now=datetime(2026, 9, 4, tzinfo=timezone.utc))
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "run_id": run_id,
            "state": "CLOSED",
            "label": "EXP2 fixture",
            "created_at": "2026-09-04T00:00:00.000Z",
            "locked_grasp": locked_grasp(),
            "result": {"sample_counts": {}},
        }
        raw = {"schema_version": SCHEMA_VERSION, "run_id": run_id, "semantic": "AUTHORITATIVE RECORDED EVIDENCE", "planned": {"samples": []}, "execution": {}, "observations": []}
        self.store.create(manifest, raw)
        self.assertNotIn("exp2_analysis", self.store.load(run_id))
        analysis = {
            "schema_version": EXP2_SCHEMA_VERSION,
            "experiment": "EXP-2 Rigid-Grasp Preservation",
            "run_id": run_id,
            "status": "UNAVAILABLE",
            "semantic": EXP2_SEMANTIC,
            "coverage": {},
            "samples": [],
        }
        analysis["input_provenance"] = exp2_input_provenance(raw, manifest, EXP2_URDF_PATH)
        self.store.write_exp2_analysis(run_id, analysis)
        loaded = self.store.load(run_id)
        self.assertEqual(loaded["exp2_analysis"], analysis)
        self.assertTrue((self.store.root / run_id / "exp2_analysis.json").is_file())
        self.store.delete(run_id)
        self.assertFalse((self.store.root / run_id).exists())


class Exp2BackendWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def test_exp2_analysis_method_and_route_are_no_motion(self):
        fake = SimpleNamespace(
            experiment_store=SimpleNamespace(
                analyze_exp2=lambda run_id: {"schema_version": EXP2_SCHEMA_VERSION, "run_id": run_id, "status": "AVAILABLE"}
            )
        )
        result = self.backend.DualJakaWebNode.analyze_experiment_run_exp2(fake, "exp-id")
        self.assertTrue(result["ok"])
        self.assertEqual(result["exp2_analysis"]["run_id"], "exp-id")
        self.assertFalse(self.backend._d33_is_motion_command("/api/experiments/exp-id/exp2-analysis", "POST"))
        route = next(route for route in self.backend.app.routes if getattr(route, "path", None) == "/api/experiments/{run_id}/exp2-analysis")
        self.assertEqual(route.methods, {"POST"})


if __name__ == "__main__":
    unittest.main()
