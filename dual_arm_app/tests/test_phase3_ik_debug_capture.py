"""Offline diagnostics-only coverage for Phase-3 IK branch capture."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from dual_arm_app.backend.object_global_planning import (
    PHASE3_IK_DEBUG_CAPTURE_ENV,
    PHASE3_IK_DEBUG_CAPTURE_SCHEMA,
    _path_transition_payloads,
    _suspicious_alternative_payloads,
    plan_object_global,
)
from dual_arm_app.backend.object_grasp_model import SYNTHETIC_OBJECT_GRASP_FIXTURE
from dual_arm_app.backend.object_trajectory_ik import (
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
    SelectedTrajectoryGraphPoint,
    TrajectoryCandidateGraph,
    TrajectoryCandidateLayer,
    TrajectoryCandidateNode,
    TrajectoryCandidateProvenance,
    compare_greedy_and_global,
)


ROOT = Path(__file__).resolve().parents[2]
PLANNER_PATH = ROOT / "dual_arm_app/backend/object_global_planning.py"
GRAPH_PATH = ROOT / "dual_arm_app/backend/object_trajectory_ik.py"


AUTHORITY = {
    "grasp_status": "GRASP_LOCKED",
    "grasp_source": "LOCKED OPERATOR GRASP",
    "grasp_content_revision": "sha256:capture-grasp",
    "lock_generation": 7,
    "lock_revision": "sha256:capture-lock",
    "left": {
        "translation_m": [0.0, 0.25, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0],
    },
    "right": {
        "translation_m": [0.0, -0.25, 0.0],
        "rpy_rad": [0.0, 0.0, 0.0],
    },
    "calibration_revision": "sha256:capture-calibration",
    "model_calibration_revision": "sha256:capture-calibration",
    "calibration_revision_status": "MATCH",
    "calibration_state": "MODEL_DEFAULT",
    "physical_calibration": "NOT_CALIBRATED",
    "physically_calibrated": False,
    "tcp_tool_contract_status": "UNVERIFIED",
}


def request_payload() -> dict:
    return {
        "name": "Phase3_Debug_Capture_Test",
        "waypoints": [
            {"identifier": "W0", "translation_m": [0.0, 0.0, 0.8]},
            {"identifier": "W1", "translation_m": [0.2, 0.0, 0.9]},
        ],
        "fixed_orientation_rpy_rad": [0.1, -0.2, 0.3],
        "segment_duration_s": 1.0,
        "samples_per_segment": 2,
        "candidate_attempts_per_arm": 3,
        "initial_joint_state_rad": {
            "left": [0.0] * 6,
            "right": [0.0] * 6,
        },
        "planning_start_state_source": "OFFLINE CAPTURE TEST",
        "expected_grasp_content_revision": "sha256:capture-grasp",
        "expected_lock_generation": 7,
        "expected_lock_revision": "sha256:capture-lock",
        "expected_calibration_revision": "sha256:capture-calibration",
        "expected_model_calibration_revision": "sha256:capture-calibration",
    }


class DiagnosticAdapter:
    """Seed echo with deterministic IK failure and state-validity rejection."""

    def solve_arm_ik(self, *, group_name, seed_joint_positions_rad, **_kwargs):
        seed = tuple(seed_joint_positions_rad)
        positions = seed[:6] if group_name == "left_arm" else seed[6:]
        if group_name == "left_arm" and positions[0] < -0.1:
            return ArmIkSolution(False, diagnostic="synthetic missing Left IK")
        return ArmIkSolution(True, positions, diagnostic="synthetic seed echo")

    def check_combined_state(self, *, joint_positions_rad, group_name):
        assert group_name == "dual_arm"
        positions = tuple(joint_positions_rad)
        rejected = positions[0] > 0.1 and positions[6] > 0.1
        return CombinedStateValidity(
            not rejected,
            "synthetic inter-arm state rejection" if rejected else "synthetic valid",
        )


def run_plan() -> dict:
    return plan_object_global(
        request_payload(),
        DiagnosticAdapter(),
        CanonicalJointPositionLimits((-2.0,) * 12, (2.0,) * 12),
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=AUTHORITY,
    )


def decision_projection(result: dict) -> dict:
    return {
        key: result[key]
        for key in ("global_path", "global", "greedy", "comparison", "graph")
    }


class Phase3IkDebugCaptureTests(unittest.TestCase):
    def test_unset_environment_writes_nothing_and_preserves_decision(self):
        with tempfile.TemporaryDirectory(prefix="phase3-capture-unset-") as directory:
            target = Path(directory) / "capture.json"
            with mock.patch.dict(os.environ, {}, clear=False):
                os.environ.pop(PHASE3_IK_DEBUG_CAPTURE_ENV, None)
                first = run_plan()
                second = run_plan()
            self.assertFalse(target.exists())
            self.assertEqual(decision_projection(first), decision_projection(second))

    def test_enabled_capture_is_valid_complete_json_and_decision_is_unchanged(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(PHASE3_IK_DEBUG_CAPTURE_ENV, None)
            baseline = run_plan()
        with tempfile.TemporaryDirectory(prefix="phase3-capture-enabled-") as directory:
            target = Path(directory) / "capture.json"
            with mock.patch.dict(
                os.environ,
                {PHASE3_IK_DEBUG_CAPTURE_ENV: str(target)},
                clear=False,
            ):
                captured_result = run_plan()
            capture = json.loads(target.read_text(encoding="utf-8"))
            self.assertEqual([item.name for item in Path(directory).iterdir()], ["capture.json"])

        self.assertEqual(decision_projection(baseline), decision_projection(captured_result))
        self.assertEqual(capture["schema"], PHASE3_IK_DEBUG_CAPTURE_SCHEMA)
        authority = capture["input_authority"]
        self.assertEqual([item["identifier"] for item in authority["waypoints"]], ["W0", "W1"])
        self.assertEqual(authority["planning_start_state_source"], "OFFLINE CAPTURE TEST")
        self.assertEqual(authority["locked_grasp"]["content_revision"], "sha256:capture-grasp")
        self.assertEqual(authority["locked_grasp"]["lock_generation"], 7)
        self.assertEqual(authority["calibration_revision"], "sha256:capture-calibration")

        layers = capture["graph"]["layers"]
        self.assertEqual(len(layers), baseline["graph"]["layer_count"])
        self.assertTrue(all(layer["valid_nodes"] for layer in layers))
        self.assertTrue(all("center_target" in layer for layer in layers))
        provenance = layers[0]["valid_nodes"][0]["candidate_provenance"][0]
        self.assertEqual(provenance["left_candidate_index"], 0)
        self.assertEqual(len(provenance["left_source_seed_rad"]), 12)

        attempts = [item for layer in layers for item in layer["candidate_attempts"]]
        self.assertTrue(any(item["disposition"] == "IK_FAILURE" for item in attempts))
        rejected = [
            item for layer in layers for item in layer["rejected_candidate_pairs"]
        ]
        self.assertTrue(rejected)
        self.assertTrue(all(item["rejection_type"] == "STATE_VALIDITY_REJECTION" for item in rejected))
        self.assertTrue(all(item["diagnostic"] for item in rejected))

        global_search = capture["global_search"]
        greedy_search = capture["greedy_search"]
        self.assertEqual(global_search["selected_node_indices"], [
            point["node_index"] for point in global_search["selected_path"]
        ])
        self.assertEqual(greedy_search["selected_node_indices"], [
            point["node_index"] for point in greedy_search["selected_path"]
        ])
        self.assertEqual(global_search["total_cost_rad2"], baseline["global"]["total_cost_rad2"])
        self.assertEqual(greedy_search["total_cost_rad2"], baseline["greedy"]["total_cost_rad2"])
        self.assertTrue(global_search["node_dp"])
        self.assertTrue(capture["graph"]["evaluated_edges"])
        for transition in global_search["transitions"] + greedy_search["transitions"]:
            self.assertEqual(len(transition["raw_joint_deltas_rad"]), 12)
            self.assertEqual(len(transition["shortest_angular_joint_deltas_rad"]), 12)
            self.assertIn("raw_squared_l2_edge_cost_rad2", transition)

    def test_two_pi_alias_is_visible_without_changing_raw_cost(self):
        zero = (0.0,) * 12
        two_pi = (2.0 * math.pi,) + (0.0,) * 11
        path = (
            SelectedTrajectoryGraphPoint(0, 0.0, "L0:N0", 0, zero, zero[:6], zero[6:], 0.0, 0.0, ()),
            SelectedTrajectoryGraphPoint(1, 1.0, "L1:N0", 0, two_pi, two_pi[:6], two_pi[6:], (2.0 * math.pi) ** 2, (2.0 * math.pi) ** 2, ()),
        )
        transition = _path_transition_payloads(path)[0]
        self.assertAlmostEqual(transition["raw_joint_deltas_rad"][0], 2.0 * math.pi)
        self.assertAlmostEqual(transition["shortest_angular_joint_deltas_rad"][0], 0.0)
        self.assertAlmostEqual(transition["raw_squared_l2_edge_cost_rad2"], (2.0 * math.pi) ** 2)
        self.assertTrue(transition["suspicious"])

    def test_suspicious_summary_retains_alternatives_and_future_cost_evidence(self):
        provenance = (
            TrajectoryCandidateProvenance(
                None,
                0,
                0,
                "synthetic left",
                0,
                0,
                "synthetic right",
            ),
        )

        def node(layer_index, node_index, first_joint):
            positions = (first_joint,) + (0.0,) * 11
            return TrajectoryCandidateNode(
                sample_index=layer_index,
                node_index=node_index,
                combined_joint_positions_rad=positions,
                provenance=provenance,
                state_validity_provenance=("synthetic valid",),
            )

        graph = TrajectoryCandidateGraph(
            trajectory_name="future-cost-evidence",
            requested_layer_count=3,
            layers=(
                TrajectoryCandidateLayer(0, 0.0, (node(0, 0, 0.0),), ("INITIAL_SEED",)),
                TrajectoryCandidateLayer(
                    1,
                    1.0,
                    (node(1, 0, 2.0 * math.pi), node(1, 1, 0.5)),
                    ("L0:N0",),
                ),
                TrajectoryCandidateLayer(
                    2,
                    2.0,
                    (node(2, 0, 4.0 * math.pi),),
                    ("L1:N0", "L1:N1"),
                ),
            ),
            completed=True,
        )
        comparison = compare_greedy_and_global(
            graph,
            retain_debug_diagnostics=True,
        )
        global_result = comparison.global_search
        transitions = _path_transition_payloads(global_result.selected_path)
        summaries = _suspicious_alternative_payloads(
            graph,
            global_result,
            transitions,
        )

        self.assertEqual(
            [point.graph_node_index for point in global_result.selected_path],
            [0, 0, 0],
        )
        self.assertEqual(
            [point.graph_node_index for point in comparison.greedy.selected_path],
            [0, 1, 0],
        )
        self.assertLess(
            global_result.cumulative_raw_displacement_cost_rad2,
            comparison.greedy.cumulative_raw_displacement_cost_rad2,
        )
        first_summary = summaries[0]
        alternatives = first_summary["destination_layer_candidates"]
        self.assertEqual([item["node_index"] for item in alternatives], [0, 1])
        self.assertTrue(alternatives[0]["selected"])
        self.assertTrue(all(item["edge_from_selected_previous_node"]["feasible"] for item in alternatives))
        self.assertAlmostEqual(
            alternatives[0]["shortest_angular_joint_deltas_rad"][0],
            0.0,
        )
        self.assertLess(
            alternatives[1]["global_dp"]["cumulative_cost_rad2"],
            alternatives[0]["global_dp"]["cumulative_cost_rad2"],
        )
        final_dp = next(
            item
            for item in global_result.node_search_diagnostics
            if item.sample_index == 2 and item.node_index == 0
        )
        self.assertEqual(final_dp.selected_predecessor_node_index, 0)
        self.assertAlmostEqual(
            final_dp.cumulative_cost_rad2,
            2.0 * (2.0 * math.pi) ** 2,
        )

    def test_write_failure_logs_warning_without_mutating_selection(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop(PHASE3_IK_DEBUG_CAPTURE_ENV, None)
            baseline = run_plan()
        with tempfile.TemporaryDirectory(prefix="phase3-capture-failure-") as directory:
            target = Path(directory) / "missing" / "capture.json"
            with mock.patch.dict(
                os.environ,
                {PHASE3_IK_DEBUG_CAPTURE_ENV: str(target)},
                clear=False,
            ), self.assertLogs(
                "dual_arm_app.backend.object_global_planning", level="WARNING"
            ) as logs:
                result = run_plan()
            self.assertFalse(target.exists())
        self.assertEqual(decision_projection(baseline), decision_projection(result))
        self.assertIn("diagnostic capture failed", " ".join(logs.output))

    def test_instrumentation_adds_no_motion_or_frontend_surface(self):
        source = PLANNER_PATH.read_text(encoding="utf-8") + GRAPH_PATH.read_text(encoding="utf-8")
        for forbidden in ("joint_move(", "linear_move(", "/api/program/run"):
            self.assertNotIn(forbidden, source)
        self.assertNotIn(PHASE3_IK_DEBUG_CAPTURE_ENV, (ROOT / "dual_arm_app/web/digital_twin.js").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
