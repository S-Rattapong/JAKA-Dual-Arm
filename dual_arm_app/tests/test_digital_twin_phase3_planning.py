"""Offline Phase-3B Object Waypoint Web and planning-contract tests."""

from __future__ import annotations

import ast
import math
import unittest
from html.parser import HTMLParser
from pathlib import Path

from dual_arm_app.backend.object_global_planning import (
    ObjectGlobalPlanInputError,
    PHASE3_CONTINUITY_GATE_REASON,
    PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD,
    PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD,
    PLAN_ONLY_WARNINGS,
    WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG,
    WEB_GLOBAL_CANDIDATE_EXPLORATION_PROFILE,
    WEB_OPTIMALITY_SCOPE,
    normalize_object_global_plan_request,
    phase3_preview_continuity_edge_feasibility,
    plan_object_global,
    planning_unavailable_result,
)
from dual_arm_app.backend.object_trajectory_ik import (
    DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG,
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
    TrajectoryCandidateGraph,
    TrajectoryCandidateLayer,
    TrajectoryCandidateNode,
    TrajectoryCandidateProvenance,
    analyze_joint_transition,
    compare_greedy_and_global,
)
from dual_arm_app.backend.object_grasp_model import SYNTHETIC_OBJECT_GRASP_FIXTURE


ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / "dual_arm_app/web"
HTML_PATH = WEB / "index.html"
CONTROLLER_PATH = WEB / "digital_twin.js"
PLANNING_JS_PATH = WEB / "digital_twin_phase3_planning.js"
BACKEND_PATH = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
PLANNER_PATH = ROOT / "dual_arm_app/backend/object_global_planning.py"


class _IdParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids: list[str] = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for name, value in attrs if name == "id")


class EchoPlanningAdapter:
    def __init__(self, fail_group: str | None = None):
        self.fail_group = fail_group
        self.ik_calls = 0
        self.validity_calls = 0

    def solve_arm_ik(
        self,
        *,
        group_name,
        ik_link_name,
        target_world_T_tip,
        seed_joint_positions_rad,
        timeout_s,
        avoid_collisions,
    ):
        del ik_link_name, target_world_T_tip, timeout_s, avoid_collisions
        self.ik_calls += 1
        if group_name == self.fail_group:
            return ArmIkSolution(False, diagnostic="synthetic unavailable IK")
        del seed_joint_positions_rad
        # Every exploratory seed converges to the same deterministic IK result,
        # so candidate deduplication keeps this integration test intentionally small.
        return ArmIkSolution(True, (0.0,) * 6, "synthetic success")

    def check_combined_state(self, *, joint_positions_rad, group_name):
        self.validity_calls += 1
        self.last_group = group_name
        self.last_positions = tuple(joint_positions_rad)
        return CombinedStateValidity(True, "synthetic valid")


class BranchJumpPlanningAdapter(EchoPlanningAdapter):
    """Returns a genuine multi-joint branch jump at the final test sample."""

    def solve_arm_ik(
        self,
        *,
        group_name,
        ik_link_name,
        target_world_T_tip,
        seed_joint_positions_rad,
        timeout_s,
        avoid_collisions,
    ):
        del group_name, ik_link_name, seed_joint_positions_rad
        del timeout_s, avoid_collisions
        self.ik_calls += 1
        value = 3.0 if target_world_T_tip.translation_m[0] > 0.3 else 0.0
        return ArmIkSolution(True, (value,) * 6, "synthetic branch jump")


def graph_node(sample_index, node_index, positions):
    return TrajectoryCandidateNode(
        sample_index=sample_index,
        node_index=node_index,
        combined_joint_positions_rad=tuple(positions),
        provenance=(TrajectoryCandidateProvenance(
            parent_node_id=None if sample_index == 0 else "synthetic parent",
            left_candidate_index=0,
            left_source_seed_index=0,
            left_diagnostic="synthetic left",
            right_candidate_index=0,
            right_source_seed_index=0,
            right_diagnostic="synthetic right",
        ),),
        state_validity_provenance=("synthetic state valid",),
    )


def graph_from_joint_layers(*layers):
    graph_layers = tuple(
        TrajectoryCandidateLayer(
            sample_index=sample_index,
            time_from_start_s=float(sample_index),
            nodes=tuple(
                graph_node(sample_index, node_index, positions)
                for node_index, positions in enumerate(layer)
            ),
            source_parent_node_ids=(
                ("INITIAL_SEED",) if sample_index == 0
                else tuple(
                    f"L{sample_index - 1}:N{index}"
                    for index in range(len(layers[sample_index - 1]))
                )
            ),
        )
        for sample_index, layer in enumerate(layers)
    )
    return TrajectoryCandidateGraph(
        trajectory_name="phase3 continuity fixture",
        requested_layer_count=len(graph_layers),
        layers=graph_layers,
        completed=True,
    )


def request_payload():
    return {
        "name": "Phase3B_Test",
        "waypoints": [
            {"identifier": "W0", "translation_m": [0.0, 0.0, 0.8]},
            {"identifier": "W1", "translation_m": [0.2, 0.0, 0.9]},
            {"identifier": "W2", "translation_m": [0.4, 0.0, 1.0]},
        ],
        "fixed_orientation_rpy_rad": [0.1, -0.2, 0.3],
        "segment_duration_s": 1.0,
        "samples_per_segment": 2,
        "expected_grasp_content_revision": "sha256:test-grasp",
        "expected_lock_generation": 1,
        "expected_lock_revision": "sha256:test-lock",
        "expected_calibration_revision": "sha256:test-calibration",
        "expected_model_calibration_revision": "sha256:test-calibration",
    }


TEST_PLANNING_AUTHORITY = {
    "grasp_status": "GRASP_LOCKED",
    "grasp_source": "LOCKED OPERATOR GRASP",
    "grasp_content_revision": "sha256:test-grasp",
    "lock_generation": 1,
    "lock_revision": "sha256:test-lock",
    "left": {"translation_m": [0.0, 0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    "right": {"translation_m": [0.0, -0.25, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
    "calibration_revision": "sha256:test-calibration",
    "model_calibration_revision": "sha256:test-calibration",
    "calibration_revision_status": "MATCH",
    "calibration_state": "MODEL_DEFAULT",
    "physical_calibration": "NOT_CALIBRATED",
    "physically_calibrated": False,
    "tcp_tool_contract_status": "UNVERIFIED",
}


def run_test_plan(payload, adapter, limits):
    return plan_object_global(
        payload,
        adapter,
        limits,
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=TEST_PLANNING_AUTHORITY,
    )


class ObjectWaypointWebTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.controller = CONTROLLER_PATH.read_text(encoding="utf-8")
        cls.planning_js = PLANNING_JS_PATH.read_text(encoding="utf-8")

    def test_dedicated_panel_is_distinct_from_legacy_joint_waypoints(self):
        self.assertIn('id="digitalTwinOperatorWaypointStage"', self.html)
        self.assertIn("Center Waypoints", self.html)
        self.assertIn("Waypoint Manager", self.html)
        self.assertIn("Program / Sequence", self.html)

    def test_waypoint_controls_and_meter_fields_exist(self):
        for expected in (
            "digitalTwinObjectWaypointAdd",
            "digitalTwinObjectWaypointAddCurrent",
            "digitalTwinObjectWaypointDemo",
            "digitalTwinObjectWaypointClear",
            "digitalTwinObjectWaypointList",
            "X [m]", "Y [m]", "Z [m]",
            "Move Object Planning Waypoint",
        ):
            with self.subTest(expected=expected):
                if expected == "Move Object Planning Waypoint":
                    self.assertIn("moveObjectPlanningWaypointInState", self.controller)
                else:
                    self.assertIn(expected, self.html)

    def test_center_waypoints_are_visible_6d_poses_with_legacy_orientation_fallback(self):
        self.assertIn("every Center waypoint may have its own XYZ + Roll/Pitch/Yaw orientation", self.html)
        self.assertIn("digitalTwinCenterWaypointRotate", self.html)
        for field in ("Roll", "Pitch", "Yaw"):
            self.assertIn(f"digitalTwinObjectWaypointNew{field}", self.html)
        self.assertIn("waypoint.rpy_rad", self.controller)
        self.assertIn("fixedOrientationRpyRad", self.controller)

    def test_locked_center_has_dedicated_6d_gizmo_controls_and_numeric_rpy(self):
        for element_id in (
            "digitalTwinCenterWaypointMove",
            "digitalTwinCenterWaypointRotate",
            "digitalTwinCenterWaypointGizmoToggle",
            "digitalTwinObjectRoll",
            "digitalTwinObjectPitch",
            "digitalTwinObjectYaw",
        ):
            self.assertIn(element_id, self.html)
        self.assertIn('mode === "rotate"', self.controller)
        self.assertIn('rigidGraspState.state !== "GRASP_LOCKED"', self.controller)
        self.assertIn('rpyRad: [...centerPose.rpyRad]', self.controller)
        self.assertIn('value("digitalTwinObjectRoll")', self.controller)
        self.assertIn('field.disabled = !locked', self.controller)

    def test_diagnostics_costs_graph_failure_and_warnings_are_visible(self):
        for expected in (
            "Advanced Planning Settings / Diagnostics",
            "Nodes per Layer", "Selected Node Path / IDs",
            "Global Cost [rad²]", "Greedy Cost [rad²]", "Cost Difference",
            "Failure Sample / Layer", "Failure Reason",
            "Exact optimum over generated layered candidate graph",
            "Offline preview only — no robot motion is sent.",
            "IK Attempts / Arm",
            "WEB RUNTIME — BOUNDED IK SEED EXPLORATION",
            "This is not a robot safety setting",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, self.html)

    def test_phase3_has_no_execute_control_and_html_ids_are_unique(self):
        parser = _IdParser()
        parser.feed(self.html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))
        pre_execution_planning = self.html.split(
            'id="digitalTwinOperatorWaypointStage"', 1
        )[1].split(
            'id="digitalTwinAdvancedDiagnostics"', 1
        )[0]
        self.assertNotIn(">Execute", pre_execution_planning)
        self.assertNotIn("Execution Ready", pre_execution_planning)
        self.assertNotIn("Robot Ready", pre_execution_planning)

    def test_public_inspection_and_integrated_operations_are_exposed(self):
        for expected in (
            "getObjectWaypointState", "getObjectGlobalPlanState",
            "getIntegratedPlanPreviewState", "loadObjectGlobalPlanPreview",
            "setIntegratedPlanTime", "playIntegratedPlan",
            "pauseIntegratedPlan", "resetIntegratedPlan",
        ):
            self.assertIn(expected, self.controller)

    def test_failed_replacement_plan_clears_any_older_planned_trajectory(self):
        failure_branch = self.controller.split(
            "function loadObjectGlobalPlanPreview(response)", 1
        )[1].split("async function planObjectGlobal", 1)[0]
        failed_plan = failure_branch.split("if (!plan.ok)", 1)[1].split(
            "cancelObjectTrajectoryAnimation", 1
        )[0]
        self.assertIn("clearPlannedTrajectory();", failed_plan)


class BackendPlanningContractTests(unittest.TestCase):
    limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)

    def test_request_normalizes_phase3a_waypoints_and_rejects_bad_identifiers(self):
        normalized = normalize_object_global_plan_request(request_payload())
        self.assertEqual([item.identifier for item in normalized["waypoints"]], ["W0", "W1", "W2"])
        self.assertEqual(normalized["fixed_orientation_rpy_rad"], (0.1, -0.2, 0.3))
        self.assertEqual(normalized["candidate_attempts_per_arm"], 3)
        for mutation in ("empty", "duplicate", "too_few"):
            payload = request_payload()
            if mutation == "empty":
                payload["waypoints"][0]["identifier"] = ""
            elif mutation == "duplicate":
                payload["waypoints"][1]["identifier"] = "w0"
            else:
                payload["waypoints"] = payload["waypoints"][:1]
            with self.subTest(mutation=mutation), self.assertRaises(ObjectGlobalPlanInputError):
                normalize_object_global_plan_request(payload)

    def test_extended_default_remains_13_and_web_default_is_3(self):
        self.assertEqual(
            DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.max_attempts_per_arm,
            13,
        )
        self.assertEqual(
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.max_attempts_per_arm,
            3,
        )
        self.assertEqual(
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.perturbation_offset_rad,
            DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.perturbation_offset_rad,
        )
        self.assertEqual(
            WEB_GLOBAL_CANDIDATE_EXPLORATION_CONFIG.duplicate_tolerance_rad,
            DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.duplicate_tolerance_rad,
        )

    def test_request_accepts_1_through_13_and_rejects_out_of_range_or_bool(self):
        for attempts in (1, 3, 13):
            payload = request_payload()
            payload["candidate_attempts_per_arm"] = attempts
            with self.subTest(accepted=attempts):
                self.assertEqual(
                    normalize_object_global_plan_request(payload)[
                        "candidate_attempts_per_arm"
                    ],
                    attempts,
                )
        for attempts in (0, 14, True, False, 3.0, None):
            payload = request_payload()
            payload["candidate_attempts_per_arm"] = attempts
            with self.subTest(rejected=attempts), self.assertRaises(
                ObjectGlobalPlanInputError
            ):
                normalize_object_global_plan_request(payload)

    def test_valid_request_returns_aligned_object_and_global_paths(self):
        adapter = EchoPlanningAdapter()
        result = run_test_plan(request_payload(), adapter, self.limits)
        self.assertTrue(result["ok"])
        self.assertTrue(result["plan_only"])
        self.assertEqual(result["optimality_scope"], WEB_OPTIMALITY_SCOPE)
        self.assertEqual(result["warnings"], list(PLAN_ONLY_WARNINGS))
        self.assertEqual(result["candidate_attempts_per_arm"], 3)
        self.assertEqual(
            result["candidate_exploration_profile"],
            WEB_GLOBAL_CANDIDATE_EXPLORATION_PROFILE,
        )
        self.assertFalse(result["candidate_pruning_applied"])
        self.assertGreaterEqual(result["planning_elapsed_s"], 0.0)
        self.assertEqual(result["object_sample_count"], 3)
        self.assertEqual(len(result["object_samples"]), len(result["global_path"]))
        self.assertEqual(
            [item["time_from_start_s"] for item in result["object_samples"]],
            [item["time_from_start_s"] for item in result["global_path"]],
        )
        for point in result["global_path"]:
            self.assertEqual(len(point["left"]), 6)
            self.assertEqual(len(point["right"]), 6)
            self.assertTrue(all(math.isfinite(value) for value in point["combined"]))
        self.assertIn("global", result)
        self.assertIn("greedy", result)
        self.assertNotEqual(result["global"], result["greedy"])

    def test_preview_continuity_gate_accepts_an_ordinary_small_step(self):
        start = graph_node(0, 0, (0.0,) * 12)
        finish = graph_node(1, 0, (0.5,) + (0.0,) * 11)
        feasibility = phase3_preview_continuity_edge_feasibility(start, finish)
        self.assertTrue(feasibility.feasible)

    def test_preview_continuity_gate_rejects_raw_two_pi_preview_spin(self):
        start = graph_node(0, 0, (0.0,) * 12)
        finish = graph_node(1, 0, (2.0 * math.pi,) + (0.0,) * 11)
        transition = analyze_joint_transition(
            from_sample_index=0,
            to_sample_index=1,
            previous_joint_positions_rad=start.combined_joint_positions_rad,
            current_joint_positions_rad=finish.combined_joint_positions_rad,
        )
        feasibility = phase3_preview_continuity_edge_feasibility(start, finish)
        self.assertAlmostEqual(transition.max_shortest_abs_joint_step_rad, 0.0)
        self.assertFalse(feasibility.feasible)
        self.assertIn(PHASE3_CONTINUITY_GATE_REASON, feasibility.diagnostic)
        self.assertIn("raw maximum", feasibility.diagnostic)

    def test_captured_w2_branch_switch_is_rejected_as_a_genuine_jump(self):
        captured_before_w2 = (
            3.2130870876181814, 0.8905895744699024, -1.6515620203246566,
            -0.12428201788511478, 1.2751471491637525, 0.18248418125312388,
            -0.07365799982886656, 2.2422873045913114, 1.6503241243472622,
            0.12789542102516258, 1.865833631818895, -0.1860728578011528,
        )
        captured_w2_branch = (
            -0.02312945427401113, 2.0970713846734763, 2.1062013856207185,
            2.2281091549962744, -4.698898588795947, 0.8947227764910587,
            0.16914101265331458, 3.561434550859517, 4.3733438167392515,
            -3.290685722219912, 5.320367828065335, -3.1178153647701583,
        )
        start = graph_node(2, 0, captured_before_w2)
        finish = graph_node(3, 0, captured_w2_branch)
        transition = analyze_joint_transition(
            from_sample_index=2,
            to_sample_index=3,
            previous_joint_positions_rad=captured_before_w2,
            current_joint_positions_rad=captured_w2_branch,
        )
        feasibility = phase3_preview_continuity_edge_feasibility(start, finish)
        self.assertAlmostEqual(transition.max_abs_joint_step_rad, 5.974045737959699)
        self.assertAlmostEqual(
            transition.max_shortest_abs_joint_step_rad,
            3.046968765287793,
        )
        self.assertGreater(
            transition.max_shortest_abs_joint_step_rad,
            PHASE3_PREVIEW_MAX_SHORTEST_ANGULAR_STEP_RAD,
        )
        self.assertFalse(feasibility.feasible)

    def test_global_search_chooses_smooth_candidate_when_one_exists(self):
        zeros = (0.0,) * 12
        wrapped = (2.0 * math.pi,) + (0.0,) * 11
        smooth = (0.5,) + (0.0,) * 11
        finish = (0.6,) + (0.0,) * 11
        graph = graph_from_joint_layers(
            (zeros,),
            (wrapped, smooth),
            (finish,),
        )
        comparison = compare_greedy_and_global(
            graph,
            edge_feasibility=phase3_preview_continuity_edge_feasibility,
        )
        self.assertTrue(comparison.global_search.completed)
        self.assertEqual(
            [point.graph_node_index for point in comparison.global_search.selected_path],
            [0, 1, 0],
        )

    def test_plan_fails_closed_when_every_w2_edge_switches_ik_branch(self):
        result = run_test_plan(
            request_payload(), BranchJumpPlanningAdapter(), self.limits
        )
        self.assertFalse(result["ok"])
        self.assertEqual(result["planner_status"], "FAILED")
        self.assertEqual(result["global_path"], [])
        self.assertEqual(result["failure"]["reason"], PHASE3_CONTINUITY_GATE_REASON)
        self.assertEqual(result["failure"]["search_reason"], "NO_COMPLETE_GLOBAL_PATH")
        self.assertEqual(result["failure"]["failed_sample_index"], 2)
        self.assertGreater(result["failure"]["continuity_gate"]["rejected_edge_count"], 0)
        self.assertAlmostEqual(
            result["failure"]["continuity_gate"][
                "maximum_shortest_angular_joint_step_rad"
            ],
            3.0,
        )
        self.assertEqual(
            result["failure"]["continuity_gate"]["raw_step_limit_rad"],
            PHASE3_PREVIEW_MAX_RAW_JOINT_STEP_RAD,
        )
        self.assertIn("no discontinuous Planned Ghost", result["failure"]["message"])

    def test_explicit_web_attempt_budget_is_reported_without_pruning(self):
        payload = request_payload()
        payload["candidate_attempts_per_arm"] = 1
        result = run_test_plan(payload, EchoPlanningAdapter(), self.limits)
        self.assertTrue(result["ok"])
        self.assertEqual(result["candidate_attempts_per_arm"], 1)
        self.assertFalse(result["candidate_pruning_applied"])
        self.assertFalse(result["graph"]["candidate_pruning_applied"])

    def test_graph_generation_still_propagates_every_prior_layer_node(self):
        source = (
            ROOT / "dual_arm_app/backend/object_trajectory_ik.py"
        ).read_text(encoding="utf-8")
        builder = source.split(
            "def build_object_trajectory_candidate_graph", 1
        )[1].split("def default_graph_edge_feasibility", 1)[0]
        self.assertIn("for node in layers[-1].nodes", builder)
        self.assertIn("for parent_node_id, base_seed in parent_seeds", builder)
        self.assertIn("No candidate pruning is performed", builder)
        self.assertNotIn("greedy_result", builder)
        self.assertNotIn("top_k", builder.lower())

    def test_graph_failure_is_structured_and_not_a_partial_success(self):
        result = run_test_plan(
            request_payload(), EchoPlanningAdapter(fail_group="left_arm"), self.limits
        )
        self.assertFalse(result["ok"])
        self.assertEqual(result["planner_status"], "FAILED")
        self.assertEqual(result["global_path"], [])
        self.assertEqual(result["failure"]["reason"], "NO_LEFT_IK_CANDIDATE")
        self.assertEqual(result["failure"]["failed_sample_index"], 0)

    def test_moveit_unavailable_is_clean_plan_only_result(self):
        result = planning_unavailable_result("/compute_ik unavailable")
        self.assertFalse(result["ok"])
        self.assertTrue(result["plan_only"])
        self.assertEqual(result["planner_status"], "UNAVAILABLE")
        self.assertEqual(result["failure"]["reason"], "MOVEIT_PLANNING_UNAVAILABLE")

    def test_existing_node_route_and_no_motion_boundary(self):
        backend = BACKEND_PATH.read_text(encoding="utf-8")
        planner = PLANNER_PATH.read_text(encoding="utf-8")
        tree = ast.parse(backend)
        routes = [
            node for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "api_digital_twin_plan_object_global"
        ]
        self.assertEqual(len(routes), 1)
        self.assertIn('/api/digital-twin/plan-object-global', backend)
        self.assertIn("MoveItObjectTrajectoryPlanningAdapter.from_node(self)", backend)
        for forbidden in (
            "jakaAPI", "joint_move", "linear_move", "servo_move",
            "create_publisher", "create_action_client", "subprocess", "ros2 ",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, planner)


if __name__ == "__main__":
    unittest.main()
