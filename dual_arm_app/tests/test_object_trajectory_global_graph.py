"""Phase 3A deterministic tests for layered candidate graph and exact DP."""

from __future__ import annotations

import math
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

from dual_arm_app.backend.object_grasp_model import (
    SYNTHETIC_OBJECT_GRASP_FIXTURE,
    RigidTransform,
)
from dual_arm_app.backend.object_trajectory import (
    generate_translation_only_object_trajectory,
)
from dual_arm_app.backend.object_trajectory_ik import (
    EMPTY_GRAPH_LAYER,
    GLOBAL_GRAPH_OPTIMALITY_SCOPE,
    NO_COMPLETE_GLOBAL_PATH,
    NO_FEASIBLE_INCOMING_EDGE,
    NO_LEFT_IK_CANDIDATE,
    NO_RIGHT_IK_CANDIDATE,
    NO_VALID_DUAL_ARM_PAIR,
    RAW_DISPLACEMENT_COST_NOTICE,
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
    IkCandidateExplorationConfig,
    TrajectoryCandidateGraph,
    TrajectoryCandidateLayer,
    TrajectoryCandidateNode,
    TrajectoryCandidateProvenance,
    TrajectoryEdgeFeasibility,
    build_consecutive_graph_edges,
    build_object_trajectory_candidate_graph,
    compare_greedy_and_global,
    graph_edge_between,
    search_global_candidate_graph,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CORE_PATH = REPOSITORY_ROOT / "dual_arm_app/backend/object_trajectory_ik.py"


def provenance(parent: str | None = None) -> TrajectoryCandidateProvenance:
    return TrajectoryCandidateProvenance(
        parent_node_id=parent,
        left_candidate_index=0,
        left_source_seed_index=0,
        left_diagnostic="synthetic left",
        right_candidate_index=0,
        right_source_seed_index=0,
        right_diagnostic="synthetic right",
    )


def node(sample_index: int, node_index: int, values) -> TrajectoryCandidateNode:
    positions = tuple(values) if not isinstance(values, (int, float)) else (
        float(values),
    ) + (0.0,) * 11
    return TrajectoryCandidateNode(
        sample_index=sample_index,
        node_index=node_index,
        combined_joint_positions_rad=positions,
        provenance=(provenance(None if sample_index == 0 else "parent"),),
        state_validity_provenance=("synthetic state valid",),
    )


def graph_from_first_joint_layers(*layers: tuple[float, ...]) -> TrajectoryCandidateGraph:
    candidate_layers = tuple(
        TrajectoryCandidateLayer(
            sample_index=sample_index,
            time_from_start_s=float(sample_index),
            nodes=tuple(
                node(sample_index, node_index, value)
                for node_index, value in enumerate(values)
            ),
            source_parent_node_ids=(
                ("INITIAL_SEED",) if sample_index == 0
                else tuple(f"L{sample_index - 1}:N{i}" for i in range(len(layers[sample_index - 1])))
            ),
        )
        for sample_index, values in enumerate(layers)
    )
    return TrajectoryCandidateGraph(
        trajectory_name="synthetic graph",
        requested_layer_count=len(candidate_layers),
        layers=candidate_layers,
        completed=True,
    )


class SeedEchoAdapter:
    """Returns deterministic arm slices from every exploratory parent seed."""

    def __init__(self, *, fail_group: str | None = None, states_valid: bool = True):
        self.fail_group = fail_group
        self.states_valid = states_valid
        self.ik_parent_seeds: list[tuple[float, ...]] = []
        self.validity_calls: list[tuple[float, ...]] = []

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
        seed = tuple(seed_joint_positions_rad)
        self.ik_parent_seeds.append(seed)
        if group_name == self.fail_group:
            return ArmIkSolution(False, diagnostic=f"synthetic {group_name} failure")
        positions = seed[:6] if group_name == "left_arm" else seed[6:]
        return ArmIkSolution(True, positions, f"synthetic {group_name} success")

    def check_combined_state(self, *, joint_positions_rad, group_name):
        self.assert_group = group_name
        positions = tuple(joint_positions_rad)
        self.validity_calls.append(positions)
        return CombinedStateValidity(self.states_valid, "synthetic validity")


class GlobalCandidateGraphTests(unittest.TestCase):
    @staticmethod
    def two_sample_trajectory():
        return generate_translation_only_object_trajectory(
            "Graph Fixture",
            RigidTransform.identity(),
            (0.2, 0.0, 0.1),
            1.0,
            2,
            SYNTHETIC_OBJECT_GRASP_FIXTURE,
        )

    def test_node_requires_exactly_twelve_finite_joints_and_is_immutable(self) -> None:
        valid = node(0, 0, range(12))
        self.assertEqual(len(valid.combined_joint_positions_rad), 12)
        self.assertEqual(valid.left_joint_positions_rad, tuple(float(i) for i in range(6)))
        self.assertEqual(valid.right_joint_positions_rad, tuple(float(i) for i in range(6, 12)))
        self.assertEqual(valid.node_id, "L0:N0")
        self.assertTrue(valid.state_valid)
        with self.assertRaises(FrozenInstanceError):
            valid.node_index = 2  # type: ignore[misc]
        for positions in ((0,) * 11, (0,) * 13, (0,) * 5 + (math.nan,) + (0,) * 6):
            with self.subTest(positions=positions):
                with self.assertRaises((TypeError, ValueError)):
                    node(0, 0, positions)

    def test_layer_generation_uses_all_parents_deduplicates_and_is_deterministic(self) -> None:
        config = IkCandidateExplorationConfig(max_attempts_per_arm=2)
        limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
        first = build_object_trajectory_candidate_graph(
            self.two_sample_trajectory(),
            SeedEchoAdapter(),
            initial_seed_joint_positions_rad=(0.0,) * 12,
            candidate_exploration_config=config,
            joint_limits=limits,
        )
        second = build_object_trajectory_candidate_graph(
            self.two_sample_trajectory(),
            SeedEchoAdapter(),
            initial_seed_joint_positions_rad=(0.0,) * 12,
            candidate_exploration_config=config,
            joint_limits=limits,
        )
        self.assertTrue(first.completed)
        self.assertFalse(first.candidate_pruning_applied)
        self.assertEqual([len(layer.nodes) for layer in first.layers], [4, 9])
        self.assertEqual(
            first.layers[1].source_parent_node_ids,
            tuple(node.node_id for node in first.layers[0].nodes),
        )
        self.assertEqual(first.layers[1].candidate_pair_count, 16)
        self.assertEqual(first.layers[1].valid_candidate_pair_count, 16)
        self.assertTrue(any(len(item.provenance) > 1 for item in first.layers[1].nodes))
        self.assertEqual(
            tuple(node.combined_joint_positions_rad for layer in first.layers for node in layer.nodes),
            tuple(node.combined_joint_positions_rad for layer in second.layers for node in layer.nodes),
        )

    def test_edges_are_consecutive_complete_and_use_raw_rad_squared_diagnostics(self) -> None:
        graph = graph_from_first_joint_layers((0.0,), (1.0, 2.0), (3.0,))
        edges = build_consecutive_graph_edges(graph)
        self.assertEqual(len(edges), 4)
        self.assertTrue(all(edge.to_sample_index == edge.from_sample_index + 1 for edge in edges))
        edge = graph_edge_between(
            node(0, 0, (0.0,) * 12),
            node(1, 0, (1.0, -2.0) + (0.0,) * 9 + (0.5,)),
        )
        self.assertEqual(edge.raw_displacement_cost_rad2, 5.25)
        self.assertEqual(edge.max_raw_joint_step_rad, 2.0)
        self.assertEqual(edge.max_raw_joint_step_index, 1)
        self.assertEqual(edge.max_raw_joint_step_name, "left_joint_2")
        self.assertIn("NO PHASE-4 LIMIT", edge.feasibility_diagnostic)
        self.assertIn("NOT ENERGY", RAW_DISPLACEMENT_COST_NOTICE)
        self.assertIn("NOT", RAW_DISPLACEMENT_COST_NOTICE)

    def test_exact_dp_beats_greedy_and_backtracks_complete_ordered_path(self) -> None:
        # Greedy: 0 -> 1 -> 10 costs 1 + 81 = 82.
        # Global: 0 -> 2 -> 10 costs 4 + 64 = 68.
        graph = graph_from_first_joint_layers((0.0,), (1.0, 2.0), (10.0,))
        comparison = compare_greedy_and_global(graph)
        self.assertTrue(comparison.greedy.completed)
        self.assertTrue(comparison.global_search.completed)
        self.assertEqual(comparison.greedy.cumulative_raw_displacement_cost_rad2, 82.0)
        self.assertEqual(comparison.global_search.cumulative_raw_displacement_cost_rad2, 68.0)
        self.assertTrue(comparison.global_not_worse_on_equivalent_graph)
        self.assertEqual(
            [point.combined_joint_positions_rad[0] for point in comparison.greedy.selected_path],
            [0.0, 1.0, 10.0],
        )
        self.assertEqual(
            [point.combined_joint_positions_rad[0] for point in comparison.global_search.selected_path],
            [0.0, 2.0, 10.0],
        )
        self.assertEqual(
            [point.edge_cost_from_predecessor_rad2 for point in comparison.global_search.selected_path],
            [0.0, 4.0, 64.0],
        )
        self.assertEqual(
            [point.cumulative_cost_rad2 for point in comparison.global_search.selected_path],
            [0.0, 4.0, 68.0],
        )
        self.assertEqual(comparison.global_search.maximum_raw_single_joint_transition_rad, 8.0)
        self.assertEqual(comparison.global_search.optimality_scope, GLOBAL_GRAPH_OPTIMALITY_SCOPE)

    def test_tie_breaking_uses_lower_predecessor_then_final_node_index(self) -> None:
        graph = graph_from_first_joint_layers((0.0,), (1.0, -1.0), (0.0, 0.0))
        results = tuple(search_global_candidate_graph(graph) for _ in range(3))
        for result in results:
            self.assertTrue(result.completed)
            self.assertEqual([point.graph_node_index for point in result.selected_path], [0, 0, 0])
            self.assertEqual(result.cumulative_raw_displacement_cost_rad2, 2.0)

    def test_structured_generation_failures_cover_left_right_and_invalid_pair(self) -> None:
        config = IkCandidateExplorationConfig(max_attempts_per_arm=1)
        cases = (
            (SeedEchoAdapter(fail_group="left_arm"), NO_LEFT_IK_CANDIDATE),
            (SeedEchoAdapter(fail_group="right_arm"), NO_RIGHT_IK_CANDIDATE),
            (SeedEchoAdapter(states_valid=False), NO_VALID_DUAL_ARM_PAIR),
        )
        for adapter, expected_reason in cases:
            with self.subTest(expected_reason=expected_reason):
                graph = build_object_trajectory_candidate_graph(
                    self.two_sample_trajectory(),
                    adapter,
                    initial_seed_joint_positions_rad=(0.0,) * 12,
                    candidate_exploration_config=config,
                )
                self.assertFalse(graph.completed)
                self.assertEqual(graph.failure.reason, expected_reason)
                self.assertEqual(graph.failure.failed_sample_index, 0)
                self.assertEqual(graph.failure.candidate_pair_count, len(adapter.validity_calls))

    def test_empty_layer_no_incoming_edge_and_no_complete_path_diagnostics(self) -> None:
        empty_graph = graph_from_first_joint_layers((0.0,), ())
        empty = search_global_candidate_graph(empty_graph)
        self.assertFalse(empty.completed)
        self.assertEqual(empty.failure.reason, EMPTY_GRAPH_LAYER)
        self.assertEqual(empty.failed_sample_index, 1)

        graph = graph_from_first_joint_layers((0.0,), (1.0,), (2.0,))

        def block_middle(_from_node, to_node):
            return TrajectoryEdgeFeasibility(
                to_node.sample_index != 1,
                "synthetic middle feasibility",
            )

        no_incoming = search_global_candidate_graph(graph, edge_feasibility=block_middle)
        self.assertFalse(no_incoming.completed)
        self.assertEqual(no_incoming.failure.reason, NO_FEASIBLE_INCOMING_EDGE)
        self.assertEqual(no_incoming.failed_sample_index, 1)

        def block_final(_from_node, to_node):
            return TrajectoryEdgeFeasibility(
                to_node.sample_index != 2,
                "synthetic final feasibility",
            )

        incomplete = search_global_candidate_graph(graph, edge_feasibility=block_final)
        self.assertFalse(incomplete.completed)
        self.assertEqual(incomplete.failure.reason, NO_COMPLETE_GLOBAL_PATH)
        self.assertEqual(incomplete.failed_sample_index, 2)
        self.assertEqual(incomplete.selected_path, ())

    def test_graph_core_has_no_pruning_safety_threshold_or_runtime_command_path(self) -> None:
        source = CORE_PATH.read_text(encoding="utf-8")
        graph_source = source.split("class TrajectoryCandidateProvenance", 1)[1]
        feasibility_source = graph_source.split(
            "def default_graph_edge_feasibility", 1
        )[1].split("def graph_edge_between", 1)[0]
        self.assertIn("NO PHASE-4 LIMIT APPLIED", feasibility_source)
        for forbidden in (
            "max_joint_step_threshold",
            "velocity_limit",
            "acceleration_limit",
            "candidate_pruning",
            "rclpy",
            "moveit_msgs",
            "jaka_msgs",
            "joint_move",
            "linear_move",
            "servo_move",
            "execute_trajectory",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, feasibility_source)
        self.assertIn("No candidate pruning is performed", graph_source)


if __name__ == "__main__":
    unittest.main()
