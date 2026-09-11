"""Pure/offline tests for shared user-waypoint velocity shaping."""

from __future__ import annotations

import json
import math
from pathlib import Path
import subprocess

import pytest

from dual_arm_app.backend.object_grasp_model import (
    RigidTransform,
    SYNTHETIC_OBJECT_GRASP_FIXTURE,
)
from dual_arm_app.backend.object_global_planning import plan_object_global
from dual_arm_app.backend.object_trajectory import (
    ObjectWaypoint,
    generate_multi_waypoint_object_trajectory,
)
from dual_arm_app.backend.object_trajectory_ik import (
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
)
from dual_arm_app.backend.phase4_trajectory_validation import (
    COMMON_TIMELINE_SUCCESS,
    MoveItDynamicsLimits,
    validate_common_timeline,
    validate_phase4_trajectory,
)
from dual_arm_app.backend.phase4_unified_validation import (
    build_phase4_unified_report,
)
from dual_arm_app.backend.phase5_execution_artifact import (
    freeze_validated_execution_artifact,
)
from dual_arm_app.backend.waypoint_velocity_shaping import (
    DEFAULT_CADENCE_S,
    VELOCITY_SHAPING_PROFILE,
    VELOCITY_SHAPING_SEMANTIC,
    WaypointVelocityShapingError,
    build_segment_shape,
    evaluate_scalar_progress,
    retime_selected_waypoint_polyline,
)
from dual_arm_app.tests.test_digital_twin_phase3_planning import (
    TEST_PLANNING_AUTHORITY,
    request_payload,
)


ROOT = Path(__file__).resolve().parents[2]


def trajectory(
    *,
    durations=(1.0, 1.0),
    counts=(3, 3),
    waypoints=None,
):
    points = waypoints or (
        ObjectWaypoint("W0", (0.0, 0.0, 0.8), (0.0, 0.0, 0.0)),
        ObjectWaypoint("W1", (0.2, 0.0, 0.9), (0.0, 0.0, 0.2)),
        ObjectWaypoint("W2", (0.4, 0.0, 1.0), (0.0, 0.0, 0.4)),
    )
    return generate_multi_waypoint_object_trajectory(
        name="velocity-shaping-fixture",
        waypoints=points,
        fixed_object_orientation=RigidTransform.identity(),
        segment_durations_s=durations,
        segment_sample_counts=counts,
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
    )


def joint_vector(value):
    return tuple((joint + 1) * value * (-1.0 if joint % 3 == 1 else 1.0) for joint in range(12))


def reversal_result():
    coarse = (
        joint_vector(0.0),
        joint_vector(0.025),
        joint_vector(0.05),
        joint_vector(0.025),
        joint_vector(0.0),
    )
    return coarse, retime_selected_waypoint_polyline(
        coarse,
        trajectory(),
        SYNTHETIC_OBJECT_GRASP_FIXTURE,
    )


def test_profile_math_and_cadence_rounding_including_short_duration():
    exact = build_segment_shape(0.04800000000000001)
    short = build_segment_shape(0.001)
    assert exact.base_ticks == 2
    assert exact.ramp_ticks == 15
    assert exact.total_ticks == 30
    assert short.base_ticks == 1
    assert short.total_ticks == 30
    assert exact.effective_ramp_s >= 0.36 - 1e-12
    assert exact.effective_duration_s >= exact.requested_duration_s
    assert exact.cruise_scalar_speed <= 1.0 / exact.requested_duration_s + 1e-12

    for timestamp, expected_progress in (
        (0.0, 0.0),
        (exact.effective_duration_s, 1.0),
    ):
        state = evaluate_scalar_progress(exact, timestamp)
        assert state.progress == expected_progress
        assert state.velocity_per_s == 0.0
        assert state.acceleration_per_s2 == 0.0
        assert state.jerk_per_s3 == 0.0


def test_monotonic_no_dwell_exact_waypoint_and_cadence():
    coarse, result = reversal_result()
    times = [sample.time_from_start_s for sample in result.joint_samples]
    assert all(later > earlier for earlier, later in zip(times, times[1:]))
    assert all(
        math.isclose(time, index * DEFAULT_CADENCE_S, abs_tol=1e-15)
        for index, time in enumerate(times)
    )
    assert result.waypoint_ticks == (0, 58, 116)
    waypoint_tick = result.waypoint_ticks[1]
    waypoint_matches = [
        sample for sample in result.joint_samples
        if sample.combined_joint_positions_rad == coarse[2]
    ]
    assert len(waypoint_matches) == 1
    assert waypoint_matches[0].sample_index == waypoint_tick
    assert result.joint_samples[waypoint_tick - 1].combined_joint_positions_rad != coarse[2]
    assert result.joint_samples[waypoint_tick + 1].combined_joint_positions_rad != coarse[2]
    assert result.diagnostics["strict_timestamps_passed"] is True
    assert all(result.diagnostics["quality_check_pass_conditions"].values())


def test_full_12_joint_shared_polyline_interpolation_and_exact_endpoints():
    coarse, result = reversal_result()
    assert result.joint_samples[0].combined_joint_positions_rad == coarse[0]
    assert result.joint_samples[-1].combined_joint_positions_rad == coarse[-1]
    for sample in result.joint_samples:
        lower = coarse[sample.coarse_lower_sample_index]
        upper = coarse[sample.coarse_upper_sample_index]
        expected = tuple(
            (1.0 - sample.coarse_fraction) * lower[index]
            + sample.coarse_fraction * upper[index]
            for index in range(12)
        )
        assert sample.combined_joint_positions_rad == pytest.approx(expected)
    assert result.diagnostics["repeated_transition_on_moving_coarse_segment_count"] == 0


def test_right_angle_vertex_is_emitted_once_and_no_transition_cuts_corner():
    q0 = (0.0, 0.0) + (0.0,) * 10
    q1 = (1.0, 0.0) + (0.0,) * 10
    q2 = (1.0, 1.0) + (0.0,) * 10
    result = retime_selected_waypoint_polyline(
        (q0, q1, q2),
        trajectory(durations=(1.0,), counts=(3,), waypoints=(
            ObjectWaypoint("A", (0.0, 0.0, 0.0)),
            ObjectWaypoint("B", (1.0, 1.0, 0.0)),
        )),
        SYNTHETIC_OBJECT_GRASP_FIXTURE,
    )
    positions = [sample.combined_joint_positions_rad for sample in result.joint_samples]
    assert positions.count(q1) == 1
    for previous, current in zip(positions, positions[1:]):
        stays_on_first_edge = previous[1] == 0.0 and current[1] == 0.0
        stays_on_second_edge = previous[0] == 1.0 and current[0] == 1.0
        assert stays_on_first_edge or stays_on_second_edge
    assert result.diagnostics["coarse_polyline_preserved"] is True
    assert result.diagnostics["consecutive_samples_stay_on_one_coarse_edge"] is True


def test_every_multi_edge_coarse_vertex_is_exact_on_a_command_tick():
    coarse = tuple(
        tuple((joint + 1) * (index + (joint % 2) * 0.1) for joint in range(12))
        for index in range(5)
    )
    result = retime_selected_waypoint_polyline(
        coarse,
        trajectory(durations=(1.0,), counts=(5,), waypoints=(
            ObjectWaypoint("A", (0.0, 0.0, 0.0)),
            ObjectWaypoint("B", (1.0, 0.0, 0.0)),
        )),
        SYNTHETIC_OBJECT_GRASP_FIXTURE,
    )
    vertex_indices = result.diagnostics["coarse_vertex_dense_sample_indices"]
    assert len(vertex_indices) == len(coarse)
    assert len(set(vertex_indices)) == len(coarse)
    assert result.diagnostics["coarse_vertex_emission_counts"] == [1] * len(coarse)
    for vertex, sample_index in zip(coarse, vertex_indices):
        sample = result.joint_samples[sample_index]
        assert sample.combined_joint_positions_rad == vertex
        assert sample.time_from_start_s == sample_index * DEFAULT_CADENCE_S


def test_reversal_slowdown_peak_speed_nonincrease_and_finite_diagnostics():
    _coarse, result = reversal_result()
    diagnostics = result.diagnostics
    near = diagnostics["internal_waypoint_adjacent_speeds"][0]
    assert all(value <= 0.005 for value in near["joint_speed_before_rad_s"])
    assert all(value <= 0.005 for value in near["joint_speed_after_rad_s"])
    assert near["max_before_rad_s"] <= 0.005
    assert near["max_after_rad_s"] <= 0.005
    assert diagnostics["dense_peak_speed_not_above_coarse"] is True
    assert all(
        dense <= coarse + diagnostics["peak_speed_comparison_epsilon_rad_s"]
        for dense, coarse in zip(
            diagnostics["dense_peak_joint_speed_rad_s"],
            diagnostics["coarse_baseline_peak_joint_speed_rad_s"],
        )
    )
    assert diagnostics["discrete_velocity_finite"]
    assert diagnostics["discrete_acceleration_finite"]
    assert diagnostics["discrete_jerk_finite"]
    assert len(diagnostics["peak_discrete_acceleration_sample_indices"]) == 12
    assert len(diagnostics["peak_discrete_jerk_sample_indices"]) == 12
    assert diagnostics["manufacturer_or_safety_limit_claim"] is False
    signed_before = near["joint_velocity_before_rad_s"]
    signed_after = near["joint_velocity_after_rad_s"]
    assert signed_before[0] > 0.0
    assert signed_after[0] < 0.0
    assert near["joint_velocity_jump_rad_s"][0] == pytest.approx(
        abs(signed_after[0] - signed_before[0])
    )
    assert near["joint_velocity_jump_rad_s"][0] > 0.0
    assert diagnostics["waypoint_speed_gate_passed"] is True
    assert diagnostics["quality_check_status"] == "PASS"


def test_adaptive_ramp_enforces_waypoint_speed_gate_or_budget_fails_closed():
    coarse = (
        (0.0,) * 12,
        (10.0,) * 12,
        (20.0,) * 12,
    )
    coarse_trajectory = trajectory(counts=(2, 2))
    result = retime_selected_waypoint_polyline(
        coarse,
        coarse_trajectory,
        SYNTHETIC_OBJECT_GRASP_FIXTURE,
    )
    diagnostics = result.diagnostics
    assert diagnostics["requested_ramp_s"] == 0.36
    assert diagnostics["effective_ramp_s"] > 0.36
    assert diagnostics["adaptive_ramp_iteration_count"] > 1
    assert diagnostics["adaptive_ramp_increased"] is True
    assert diagnostics["waypoint_speed_gate_passed"] is True
    first_attempt = diagnostics["adaptive_ramp_attempts"][0]
    assert first_attempt["effective_ramp_s"] == 0.36
    assert first_attempt["waypoint_speed_gate_passed"] is False
    assert first_attempt["internal_waypoint_maxima_rad_s"][0][
        "max_before_rad_s"
    ] > 0.005
    adjacent = diagnostics["internal_waypoint_adjacent_speeds"][0]
    assert adjacent["max_before_rad_s"] <= 0.005
    assert adjacent["max_after_rad_s"] <= 0.005
    with pytest.raises(
        WaypointVelocityShapingError,
        match="sample budget exceeded while enforcing the USER-waypoint speed gate",
    ):
        retime_selected_waypoint_polyline(
            coarse,
            coarse_trajectory,
            SYNTHETIC_OBJECT_GRASP_FIXTURE,
            max_dense_samples=120,
        )

def test_object_endpoint_exactness_and_orientation_only_segment():
    points = (
        ObjectWaypoint("A", (0.1, -0.2, 0.8), (0.0, 0.0, 0.0)),
        ObjectWaypoint("B", (0.1, -0.2, 0.8), (0.0, 0.0, math.pi / 2.0)),
    )
    coarse_trajectory = trajectory(durations=(1.0,), counts=(3,), waypoints=points)
    coarse = (joint_vector(0.0), joint_vector(0.02), joint_vector(0.04))
    result = retime_selected_waypoint_polyline(
        coarse, coarse_trajectory, SYNTHETIC_OBJECT_GRASP_FIXTURE
    )
    assert result.object_samples[0].world_T_object == coarse_trajectory.samples[0].world_T_object
    assert result.object_samples[-1].world_T_object == coarse_trajectory.samples[-1].world_T_object
    assert result.object_samples[0].world_T_left == coarse_trajectory.samples[0].world_T_left
    assert result.object_samples[-1].world_T_right == coarse_trajectory.samples[-1].world_T_right
    assert all(
        sample.world_T_object.translation_m == points[0].translation_m
        for sample in result.object_samples
    )
    middle = result.object_samples[len(result.object_samples) // 2].world_T_object
    assert not middle.almost_equal(result.object_samples[0].world_T_object, 1e-6)


def test_dense_sample_budget_checked_before_allocation():
    coarse_trajectory = trajectory(durations=(120.0,), counts=(2,), waypoints=(
        ObjectWaypoint("A", (0.0, 0.0, 0.0)),
        ObjectWaypoint("B", (1.0, 0.0, 0.0)),
    ))
    with pytest.raises(WaypointVelocityShapingError, match="sample budget exceeded"):
        retime_selected_waypoint_polyline(
            (joint_vector(0.0), joint_vector(0.1)),
            coarse_trajectory,
            SYNTHETIC_OBJECT_GRASP_FIXTURE,
        )


class TargetPositionAdapter:
    """Deterministic pure IK stand-in producing a moving selected polyline."""

    def solve_arm_ik(self, *, target_world_T_tip, **_kwargs):
        x, _y, z = target_world_T_tip.translation_m
        return ArmIkSolution(True, (x, z, x + z, -x, -z, x - z), "fixture")

    def check_combined_state(self, **_kwargs):
        return CombinedStateValidity(True, "fixture")


def test_integration_profile_stretch_then_shaping_and_response_invariants():
    payload = request_payload()
    payload["waypoints"][0]["speed_percent"] = 50.0
    result = plan_object_global(
        payload,
        TargetPositionAdapter(),
        CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12),
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=TEST_PLANNING_AUTHORITY,
    )
    assert result["ok"]
    assert result["velocity_shaping"]["profile"] == VELOCITY_SHAPING_PROFILE
    assert result["velocity_shaping"]["semantic"] == VELOCITY_SHAPING_SEMANTIC
    assert result["velocity_shaping"]["enabled"] is True
    assert result["requested_segment_durations_s"] == [2.0, 1.0]
    assert result["segment_timing"][0]["requested_duration_s"] == 2.0
    assert result["segment_timing"][0]["duration_s"] > 2.0
    assert sum(result["segment_durations_s"]) == result["duration_s"]
    assert result["duration_s"] == result["global_path"][-1]["time_from_start_s"]
    assert len(result["object_samples"]) == len(result["global_path"])
    assert result["object_sample_count"] == len(result["global_path"])
    assert result["combined_path"] == result["global_path"]
    assert len(result["coarse_global_path"]) == result["coarse_object_sample_count"]
    assert result["graph"]["layer_count"] == len(result["coarse_global_path"])
    assert result["optimality_path_scope"] == "COARSE_CANDIDATE_GRAPH_ONLY"
    assert result["velocity_shaping"]["dense_ik_resolved"] is False
    assert result["velocity_shaping"]["independent_left_right_smoothing"] is False
    assert result["velocity_shaping"]["no_dwell"] is True
    assert validate_common_timeline(result)["status"] == COMMON_TIMELINE_SUCCESS
    parser = subprocess.run(
        [
            "node",
            "--input-type=module",
            "-e",
            """
import fs from "node:fs";
import {normalizeObjectGlobalPlanResponse} from "./dual_arm_app/web/digital_twin_phase3_planning.js";
const plan = normalizeObjectGlobalPlanResponse(JSON.parse(fs.readFileSync(0, "utf8")));
process.stdout.write(JSON.stringify({
  sampleCount: plan.global_path.length,
  duration: plan.duration_s,
  segmentSum: plan.segment_durations_s.reduce((sum, value) => sum + value, 0),
}));
""",
        ],
        cwd=ROOT,
        input=json.dumps(result),
        text=True,
        check=True,
        capture_output=True,
    )
    parsed = json.loads(parser.stdout)
    assert parsed == {
        "sampleCount": len(result["global_path"]),
        "duration": result["duration_s"],
        "segmentSum": result["duration_s"],
    }


class TranslationOnlyPlanningAdapter:
    """Encode target translation directly so offline FK can invert it exactly."""

    def solve_arm_ik(self, *, target_world_T_tip, **_kwargs):
        x, y, z = target_world_T_tip.translation_m
        return ArmIkSolution(True, (x, y, z, 0.0, 0.0, 0.0), "translation fixture")

    def check_combined_state(self, **_kwargs):
        return CombinedStateValidity(True, "translation fixture")


class TranslationOnlyFkAdapter:
    def compute_combined_fk(self, joint_positions_rad, timeout_s=2.0):
        assert timeout_s == 2.0
        return {
            "status": "PASS",
            "left": RigidTransform.from_translation_rpy(
                joint_positions_rad[:3], (0.0, 0.0, 0.0)
            ),
            "right": RigidTransform.from_translation_rpy(
                joint_positions_rad[6:9], (0.0, 0.0, 0.0)
            ),
        }


def _phase4b_collision_pass_fixture():
    collision_pass = {"status": "PASS", "first_failure": None, "collision_pairs": []}
    return {
        "status": "PASS",
        "robot_collision": {
            "sampled_path": {"status": "PASS", "samples": []},
            "self_collision": dict(collision_pass),
            "inter_arm_collision": dict(collision_pass),
        },
        "object_scene": {"status": "NOT_CONFIGURED"},
        "object_collision": {"status": "NOT_EVALUATED"},
        "environment_scene": {"status": "NOT_CONFIGURED"},
        "environment_collision": {"status": "NOT_EVALUATED"},
        "synchronized_sampling": {"max_joint_step_rad": 0.05},
        "collision_summary": {"first_failure": None},
    }


def test_real_phase4_validation_consumes_dense_authoritative_path_before_freeze():
    payload = request_payload()
    payload["fixed_orientation_rpy_rad"] = [0.0, 0.0, 0.0]
    payload["samples_per_segment"] = 3
    limits = CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12)
    plan = plan_object_global(
        payload,
        TranslationOnlyPlanningAdapter(),
        limits,
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=TEST_PLANNING_AUTHORITY,
    )
    assert plan["velocity_shaping"]["enabled"] is True
    assert len(plan["global_path"]) > len(plan["coarse_global_path"])
    assert plan["combined_path"] == plan["global_path"]
    validation_start = {
        "left": plan["global_path"][0]["left"],
        "right": plan["global_path"][0]["right"],
        "source": "DETERMINISTIC_TRANSLATION_ONLY_FIXTURE",
    }
    dynamics = MoveItDynamicsLimits(
        velocity_limits_rad_s=(100.0,) * 12,
        velocity_limit_source="OFFLINE_TEST",
        source_path="OFFLINE_TEST",
        default_velocity_scaling_factor=None,
        acceleration_limits_rad_s2=(1_000_000.0,) * 12,
        acceleration_limit_status="AVAILABLE",
        acceleration_limit_validation="PENDING",
        acceleration_limit_source="OFFLINE_TEST",
    )
    phase4a = validate_phase4_trajectory(
        plan,
        fk_adapter=TranslationOnlyFkAdapter(),
        joint_position_limits=limits,
        start_state=validation_start,
        dynamics_limits=dynamics,
    )
    assert phase4a["status"] == "PASS"
    assert phase4a["timeline"]["status"] == COMMON_TIMELINE_SUCCESS
    assert phase4a["fk"]["status"] == "PASS"
    assert phase4a["relative_pose"]["status"] == "PASS"
    assert phase4a["velocity"]["status"] == "PASS"
    assert len(phase4a["fk"]["samples"]) == len(plan["global_path"])

    # Collision is deliberately a unit seam here; the trajectory/timeline/FK/
    # dynamics evidence above comes from the real Phase-4A validator.
    report = build_phase4_unified_report(
        plan,
        phase4a,
        _phase4b_collision_pass_fixture(),
        joint_position_limits=limits,
        start_state=validation_start,
        authoritative_input_validation={"status": "PASS"},
    )
    assert report["overall_status"] == "PASS"
    artifact = freeze_validated_execution_artifact(
        plan,
        report,
        validation_start_state=validation_start,
    )
    assert artifact.sample_count == len(plan["combined_path"])
    assert artifact.common_timestamps_s == tuple(plan["combined_timestamps_s"])
    assert artifact.combined_positions_rad == tuple(
        tuple(point["combined"]) for point in plan["combined_path"]
    )


def test_unaligned_approach_explicitly_disables_shaping_without_changing_rigid_path():
    payload = request_payload()
    payload["initial_joint_state_rad"] = {"left": [0.0] * 6, "right": [0.0] * 6}
    payload["planning_start_state_source"] = "TEST"
    payload["approach_waypoints"] = [{
        "identifier": "A0",
        "left": {"translation_m": [0.0, 0.25, 0.8], "rpy_rad": [0.0, 0.0, 0.0]},
        "right": {"translation_m": [0.0, -0.25, 0.8], "rpy_rad": [0.0, 0.0, 0.0]},
    }]
    result = plan_object_global(
        payload,
        TargetPositionAdapter(),
        CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12),
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=TEST_PLANNING_AUTHORITY,
    )
    assert result["ok"]
    assert result["velocity_shaping"]["enabled"] is False
    assert "outside the focused rigid USER-waypoint" in result["velocity_shaping"]["disabled_reason"]
    assert result["velocity_shaping"]["cadence_shaped_execution"] is False
    assert result["velocity_shaping"]["adaptive_ramp_iteration_count"] == 0
    assert result["global_path"] == result["coarse_global_path"]
    assert result["segment_durations_s"] == result["requested_segment_durations_s"]
    assert result["approach_path"]


def test_cadence_aligned_approach_also_disables_focused_rigid_shaping():
    payload = request_payload()
    payload["segment_duration_s"] = 0.96
    payload["initial_joint_state_rad"] = {"left": [0.0] * 6, "right": [0.0] * 6}
    payload["planning_start_state_source"] = "TEST"
    payload["approach_waypoints"] = [{
        "identifier": "A0",
        "left": {"translation_m": [0.0, 0.25, 0.8], "rpy_rad": [0.0, 0.0, 0.0]},
        "right": {"translation_m": [0.0, -0.25, 0.8], "rpy_rad": [0.0, 0.0, 0.0]},
    }]
    result = plan_object_global(
        payload,
        TargetPositionAdapter(),
        CanonicalJointPositionLimits((-10.0,) * 12, (10.0,) * 12),
        grasp_model=SYNTHETIC_OBJECT_GRASP_FIXTURE,
        planning_authority=TEST_PLANNING_AUTHORITY,
    )
    assert result["velocity_shaping"]["enabled"] is False
    assert result["velocity_shaping"]["cadence_shaped_execution"] is False
    assert result["velocity_shaping"]["adaptive_ramp_iteration_count"] == 0
    assert "outside the focused rigid USER-waypoint" in result["velocity_shaping"]["disabled_reason"]
    assert result["global_path"] == result["coarse_global_path"]
    assert result["approach"]["duration_s"] == 0.96
    assert result["global_path"][0]["time_from_start_s"] == 0.0
    rigid_offset = len(result["approach_path"]) - 1
    assert result["combined_path"][rigid_offset]["time_from_start_s"] == 0.96
    assert result["combined_object_samples"][rigid_offset]["phase"] == "RIGID"
    assert result["combined_duration_s"] == result["combined_path"][-1]["time_from_start_s"]
