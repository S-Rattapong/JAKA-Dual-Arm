"""Offline acceptance tests for EXP-1 trajectory tracking analysis."""

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
)
from dual_arm_app.backend.experimental_trajectory_tracking import (
    ALIGNMENT_REFERENCE,
    ERROR_DEFINITION,
    EXP1_SCHEMA_VERSION,
    EXP1_SEMANTIC,
    analyze_trajectory_tracking,
)


TEST_ARTIFACT_ROOT = Path(__file__).resolve().parents[2] / ".exp1_test_artifacts"


def preserved_test_root(label: str) -> Path:
    root = TEST_ARTIFACT_ROOT / f"{label}-{uuid.uuid4().hex}"
    root.mkdir(parents=True, exist_ok=False)
    return root


def aligned_row(
    side: str,
    actual,
    planned,
    *,
    status: str = "INTERPOLATED",
    observation_index: int = 0,
    time_s: float = 0.0,
):
    return {
        "side": side,
        "status": status,
        "observation_index": observation_index,
        "time_from_start_s": time_s,
        "actual_joints_rad": actual,
        "planned_joints_rad": planned,
    }


def evidence(rows, run_id="exp-analysis"):
    raw = {
        "run_id": run_id,
        "observations": [{
            "actual": {
                "left": {"source": "recorded-left-encoder"},
                "right": {"source": "recorded-right-encoder"},
            },
            "commanded": {
                "left": {"joints_rad": [999.0] * 6},
                "right": {"joints_rad": [-999.0] * 6},
            },
        }],
    }
    aligned = {
        "run_id": run_id,
        "semantic": ALIGNMENT_SEMANTIC,
        "status": "ALIGNED",
        "samples": rows,
    }
    manifest = {
        "run_id": run_id,
        "actual_source_semantics": "Persisted encoder selection; no SDK call",
    }
    return raw, aligned, manifest


class PureMetricTests(unittest.TestCase):
    def test_signed_error_and_exact_joint_and_flattened_math(self):
        rows = [
            aligned_row("left", [1.0] + [0.0] * 5, [0.0] * 6, observation_index=4, time_s=0.25),
            aligned_row("left", [-3.0] + [0.0] * 5, [0.0] * 6, observation_index=9, time_s=0.75),
        ]
        raw, aligned, manifest = evidence(rows)
        before = (deepcopy(raw), deepcopy(aligned), deepcopy(manifest))
        result = analyze_trajectory_tracking(raw, aligned, manifest)

        self.assertEqual((raw, aligned, manifest), before)
        self.assertEqual(result["status"], "PARTIAL")
        self.assertEqual(result["error_definition"], ERROR_DEFINITION)
        self.assertEqual(result["alignment_reference"], ALIGNMENT_REFERENCE)
        self.assertEqual(result["semantic"], EXP1_SEMANTIC)
        joint = result["arms"]["left"]["joints"][0]
        self.assertEqual(joint["joint"], "Left J1")
        self.assertEqual(joint["sample_count"], 2)
        self.assertEqual(joint["mean_signed_error_rad"], -1.0)
        self.assertEqual(joint["mae_rad"], 2.0)
        self.assertAlmostEqual(joint["rmse_rad"], math.sqrt(5.0))
        self.assertEqual(joint["max_abs_error_rad"], 3.0)
        self.assertEqual(joint["max_abs_error_location"], {
            "time_from_start_s": 0.75,
            "observation_index": 9,
        })
        left = result["arms"]["left"]["summary"]
        self.assertEqual(left["sample_count"], 2)
        self.assertEqual(left["scalar_value_count"], 12)
        self.assertAlmostEqual(left["mae_rad"], 4.0 / 12.0)
        self.assertAlmostEqual(left["rmse_rad"], math.sqrt(10.0 / 12.0))
        self.assertEqual(left["max_abs_error_rad"], 3.0)
        self.assertEqual(left["worst_joint_by_max_error"], "Left J1")
        self.assertEqual(left["worst_joint_by_rmse"], "Left J1")
        self.assertEqual(result["error_samples"][0]["error_joints_rad"][0], 1.0)
        self.assertEqual(result["error_samples"][1]["error_joints_rad"][0], -3.0)

    def test_dual_arm_combined_identity_and_available_status(self):
        raw, aligned, manifest = evidence([
            aligned_row("left", [0.2] * 6, [0.0] * 6),
            aligned_row("right", [-0.4] * 6, [0.0] * 6),
        ])
        result = analyze_trajectory_tracking(raw, aligned, manifest)
        combined = result["combined"]
        self.assertEqual(result["status"], "AVAILABLE")
        self.assertEqual(combined["status"], "AVAILABLE")
        self.assertEqual(combined["sample_count"], 2)
        self.assertEqual(combined["scalar_value_count"], 12)
        self.assertAlmostEqual(combined["mae_rad"], 0.3)
        self.assertAlmostEqual(combined["rmse_rad"], math.sqrt(0.1))
        self.assertEqual(combined["max_abs_error_rad"], 0.4)
        self.assertEqual(combined["worst_joint_by_max_error"], "Right J1")
        self.assertEqual(combined["worst_joint_by_rmse"], "Right J1")
        self.assertIn("recorded-left-encoder", result["actual_source_semantics"]["observed_sources"])
        self.assertFalse(result["actual_source_semantics"]["external_metrology"])

    def test_partial_unavailable_and_skipped_status_coverage(self):
        raw, aligned, manifest = evidence([
            aligned_row("left", [0.1] * 6, [0.0] * 6),
            aligned_row("left", None, None, status="MISSING_ACTUAL", observation_index=1),
            aligned_row("right", [0.0] * 6, [0.0] * 6, status="OUT_OF_RANGE"),
        ])
        result = analyze_trajectory_tracking(raw, aligned, manifest)
        self.assertEqual(result["status"], "PARTIAL")
        self.assertEqual(result["arms"]["left"]["status"], "PARTIAL")
        self.assertEqual(result["arms"]["right"]["status"], "UNAVAILABLE")
        self.assertEqual(result["coverage"]["total_aligned_rows"], 3)
        self.assertEqual(result["coverage"]["usable_rows"], 1)
        self.assertEqual(result["coverage"]["skipped_status_counts"], {
            "MISSING_ACTUAL": 1,
            "OUT_OF_RANGE": 1,
        })

        unavailable = analyze_trajectory_tracking(*evidence([]))
        self.assertEqual(unavailable["status"], "UNAVAILABLE")
        self.assertIsNone(unavailable["combined"]["rmse_rad"])
        self.assertEqual(unavailable["combined"]["scalar_value_count"], 0)

    def test_non_finite_wrong_length_and_overflow_errors_are_rejected(self):
        rows = [
            aligned_row("left", [math.nan] * 6, [0.0] * 6),
            aligned_row("left", [math.inf] * 6, [0.0] * 6),
            aligned_row("right", [1.0] * 5, [0.0] * 6),
            aligned_row("right", [1e308] * 6, [-1e308] * 6),
        ]
        result = analyze_trajectory_tracking(*evidence(rows))
        self.assertEqual(result["status"], "UNAVAILABLE")
        self.assertEqual(result["coverage"]["usable_rows"], 0)
        self.assertEqual(
            result["coverage"]["skipped_status_counts"],
            {"INVALID_OR_NON_FINITE_VECTOR": 3, "NON_FINITE_ERROR": 1},
        )
        json.dumps(result, allow_nan=False)


def store_manifest(run_id):
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "label": "EXP-1 fixture",
        "state": "CLOSED",
        "created_at": "2026-09-04T00:00:00.000Z",
        "actual_source_semantics": "Persisted encoder evidence; not external metrology",
        "result": {"sample_counts": {}},
    }


def store_raw(run_id):
    return {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "semantic": "AUTHORITATIVE RECORDED EVIDENCE",
        "planned": {"samples": []},
        "observations": [],
    }


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        self.root = preserved_test_root("persistence")
        self.store = ExperimentRunStore(self.root / "runs")

    def test_optional_analysis_is_backward_compatible_and_reanalysis_preserves_evidence(self):
        run_id = self.store.generate_run_id(now=datetime(2026, 9, 4, tzinfo=timezone.utc))
        self.store.create(store_manifest(run_id), store_raw(run_id))
        loaded_old = self.store.load(run_id)
        self.assertNotIn("exp1_analysis", loaded_old)
        aligned = {
            "schema_version": SCHEMA_VERSION,
            "run_id": run_id,
            "semantic": ALIGNMENT_SEMANTIC,
            "status": "ALIGNED",
            "samples": [aligned_row("left", [0.1] * 6, [0.0] * 6)],
        }
        self.store.update(run_id, loaded_old["manifest"], loaded_old["raw"], aligned)
        run_dir = self.store.root / run_id
        raw_before = (run_dir / "raw.json").read_bytes()
        aligned_before = (run_dir / "aligned.json").read_bytes()

        analysis = self.store.analyze_exp1(run_id)
        self.assertEqual(analysis["schema_version"], EXP1_SCHEMA_VERSION)
        self.assertTrue((run_dir / "exp1_analysis.json").is_file())
        self.assertEqual((run_dir / "raw.json").read_bytes(), raw_before)
        self.assertEqual((run_dir / "aligned.json").read_bytes(), aligned_before)
        self.assertEqual(self.store.load(run_id)["exp1_analysis"], analysis)

        self.store.delete(run_id)
        self.assertFalse(run_dir.exists())


class BackendWiringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import
        cls.backend, cls.cleanup = _mocked_backend_import()

    @classmethod
    def tearDownClass(cls):
        cls.cleanup()

    def test_explicit_analysis_method_and_route_are_no_motion(self):
        expected = {"schema_version": EXP1_SCHEMA_VERSION, "status": "AVAILABLE"}
        fake = SimpleNamespace(
            experiment_store=SimpleNamespace(analyze_exp1=lambda run_id: {**expected, "run_id": run_id})
        )
        result = self.backend.DualJakaWebNode.analyze_experiment_run_exp1(fake, "exp-id")
        self.assertTrue(result["ok"])
        self.assertEqual(result["exp1_analysis"]["run_id"], "exp-id")
        self.assertFalse(self.backend._d33_is_motion_command(
            "/api/experiments/exp-id/exp1-analysis", "POST"
        ))
        route = next(
            route for route in self.backend.app.routes
            if getattr(route, "path", None) == "/api/experiments/{run_id}/exp1-analysis"
        )
        self.assertEqual(route.methods, {"POST"})


if __name__ == "__main__":
    unittest.main()
