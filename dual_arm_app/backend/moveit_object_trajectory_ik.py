"""ROS2 MoveIt adapter for offline multi-candidate dual-arm IK selection.

MOVEIT PLANNING ONLY. NO JAKA DRIVER. NO ROBOT CONNECTION. NO MOTION EXECUTION.
This module creates only ``/compute_ik`` and ``/check_state_validity`` service
clients. It creates no publishers, controllers, trajectory actions, or hardware
driver clients.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any, Callable, Sequence

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory import (
    SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
)
from dual_arm_app.backend.object_trajectory_ik import (
    ABSOLUTE_STEP_REASON,
    CANDIDATE_EXPLORATION_WARNING,
    DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG,
    DEFAULT_IK_TIMEOUT_S,
    DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
    DUAL_ARM_GROUP_NAME,
    DUAL_ARM_JOINT_ORDER,
    LEFT_JOINT_ORDER,
    JOINT_POSITIONS_UNCHANGED_NOTICE,
    JUMP_HEURISTIC_WARNING,
    OFFLINE_MODEL_SEED_NOTICE,
    RAW_JOINT_DELTA_NOTICE,
    RAW_JOINT_DELTA_PRESERVED_NOTICE,
    RIGHT_JOINT_ORDER,
    RELATIVE_GROWTH_REASON,
    SHORTEST_ANGULAR_ANALYSIS_NOTICE,
    ArmIkSolution,
    CanonicalJointPositionLimits,
    CombinedStateValidity,
    IkCandidateExplorationConfig,
    JointJumpDetectionConfig,
    JointVector12,
    ObjectTrajectoryIkResult,
    TrajectoryJumpAnalysis,
    rotation_matrix_to_quaternion_xyzw,
    solve_sequential_object_trajectory_ik,
)


COMPUTE_IK_SERVICE = "/compute_ik"
CHECK_STATE_VALIDITY_SERVICE = "/check_state_validity"
WORLD_FRAME = "world"
JOINT_LIMIT_METADATA_PATH = (
    Path(__file__).resolve().parents[1]
    / "web/assets/dual_jaka_a12_joint_limits.json"
)

SAFETY_BANNER = """==================================================
OFFLINE OBJECT TRAJECTORY IK
MOVEIT PLANNING ONLY
NO JAKA DRIVER
NO ROBOT CONNECTION
NO MOTION EXECUTION
=================================================="""

JUMP_PROFILE_BANNER = f"""==================================================
OFFLINE JUMP HEURISTIC PROFILE
{JUMP_HEURISTIC_WARNING}
=================================================="""

CANDIDATE_PROFILE_BANNER = f"""==================================================
OFFLINE MULTI-CANDIDATE IK EXPLORATION
{CANDIDATE_EXPLORATION_WARNING}
=================================================="""


def load_canonical_joint_limits(
    metadata_path: Path = JOINT_LIMIT_METADATA_PATH,
) -> CanonicalJointPositionLimits:
    """Load existing Xacro-generated model limits in canonical joint order."""
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("unit") != "radian":
        raise RuntimeError("Joint-limit metadata must use radians")
    if tuple(metadata.get("joint_order", ())) != DUAL_ARM_JOINT_ORDER:
        raise RuntimeError("Joint-limit metadata order is not canonical")
    limits = metadata.get("position_limits")
    if not isinstance(limits, dict):
        raise RuntimeError("Joint-limit metadata has no position_limits mapping")
    return CanonicalJointPositionLimits(
        lower_rad=tuple(limits[name]["min"] for name in DUAL_ARM_JOINT_ORDER),
        upper_rad=tuple(limits[name]["max"] for name in DUAL_ARM_JOINT_ORDER),
    )


class MoveItObjectTrajectoryPlanningAdapter:
    """Reusable planning-only clients hosted by an existing ROS node.

    FastAPI uses ``from_node`` while its existing executor spins that node.
    The standalone CLI node supplies its synchronous spin waiter. Both paths
    share identical request construction and response parsing here.
    """

    def __init__(
        self,
        compute_ik_client: Any,
        state_validity_client: Any,
        clock: Any,
        call_and_wait: Callable[[Any, Any, float], Any],
    ) -> None:
        self.compute_ik_client = compute_ik_client
        self.state_validity_client = state_validity_client
        self.clock = clock
        self.call_and_wait = call_and_wait

    @classmethod
    def from_node(cls, node: Any) -> "MoveItObjectTrajectoryPlanningAdapter":
        compute_client = node.create_client(GetPositionIK, COMPUTE_IK_SERVICE)
        state_client = node.create_client(GetStateValidity, CHECK_STATE_VALIDITY_SERVICE)

        def wait_on_existing_executor(client: Any, request: Any, timeout_s: float) -> Any:
            future = client.call_async(request)
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.005)
            if not future.done():
                cancel = getattr(future, "cancel", None)
                if callable(cancel):
                    cancel()
                raise TimeoutError("MoveIt planning service response timed out")
            exception = future.exception()
            if exception is not None:
                raise RuntimeError(f"MoveIt planning service failed: {exception}")
            response = future.result()
            if response is None:
                raise RuntimeError("MoveIt planning service returned no response")
            return response

        return cls(compute_client, state_client, node.get_clock(), wait_on_existing_executor)

    @staticmethod
    def _fill_seed(robot_state: object, seed: Sequence[float]) -> None:
        robot_state.joint_state.name = list(DUAL_ARM_JOINT_ORDER)
        robot_state.joint_state.position = list(seed)
        robot_state.is_diff = False

    def unavailable_services(self) -> tuple[str, ...]:
        unavailable = []
        for service_name, client in (
            (COMPUTE_IK_SERVICE, self.compute_ik_client),
            (CHECK_STATE_VALIDITY_SERVICE, self.state_validity_client),
        ):
            ready = getattr(client, "service_is_ready", None)
            is_ready = bool(ready()) if callable(ready) else bool(
                client.wait_for_service(timeout_sec=0.0)
            )
            if not is_ready:
                unavailable.append(service_name)
        return tuple(unavailable)

    def solve_arm_ik(
        self,
        *,
        group_name: str,
        ik_link_name: str,
        target_world_T_tip: RigidTransform,
        seed_joint_positions_rad: JointVector12,
        timeout_s: float,
        avoid_collisions: bool,
    ) -> ArmIkSolution:
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.avoid_collisions = bool(avoid_collisions)
        timeout_seconds = int(timeout_s)
        request.ik_request.timeout.sec = timeout_seconds
        request.ik_request.timeout.nanosec = int(
            (timeout_s - timeout_seconds) * 1_000_000_000
        )
        self._fill_seed(request.ik_request.robot_state, seed_joint_positions_rad)

        pose = request.ik_request.pose_stamped
        pose.header.frame_id = WORLD_FRAME
        pose.header.stamp = self.clock.now().to_msg()
        matrix = target_world_T_tip.matrix
        pose.pose.position.x = matrix[0][3]
        pose.pose.position.y = matrix[1][3]
        pose.pose.position.z = matrix[2][3]
        quaternion = rotation_matrix_to_quaternion_xyzw(target_world_T_tip)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        response = self.call_and_wait(self.compute_ik_client, request, timeout_s + 1.0)
        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            return ArmIkSolution(False, diagnostic=f"MoveIt IK error_code={error_code}")
        positions_by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        expected_names = (
            LEFT_JOINT_ORDER if group_name == "left_arm" else RIGHT_JOINT_ORDER
        )
        missing = tuple(name for name in expected_names if name not in positions_by_name)
        if missing:
            return ArmIkSolution(
                False,
                diagnostic=f"MoveIt IK response missing joints: {', '.join(missing)}",
            )
        return ArmIkSolution(
            True,
            tuple(positions_by_name[name] for name in expected_names),
            diagnostic=f"MoveIt IK success for {group_name}",
        )

    def check_combined_state(
        self,
        *,
        joint_positions_rad: JointVector12,
        group_name: str,
    ) -> CombinedStateValidity:
        request = GetStateValidity.Request()
        request.group_name = group_name
        self._fill_seed(request.robot_state, joint_positions_rad)
        response = self.call_and_wait(self.state_validity_client, request, 2.0)
        contact_count = len(response.contacts)
        diagnostic = (
            "Combined state valid"
            if response.valid
            else f"Combined state invalid; contacts={contact_count}"
        )
        return CombinedStateValidity(bool(response.valid), diagnostic)


class MoveItObjectTrajectoryIkNode(Node):
    """Planning-only synchronous adapter over two MoveIt service clients."""

    def __init__(self, service_wait_timeout_s: float = 5.0) -> None:
        super().__init__("offline_object_trajectory_ik")
        self.compute_ik_client = self.create_client(
            GetPositionIK,
            COMPUTE_IK_SERVICE,
        )
        self.state_validity_client = self.create_client(
            GetStateValidity,
            CHECK_STATE_VALIDITY_SERVICE,
        )
        for service_name, client in (
            (COMPUTE_IK_SERVICE, self.compute_ik_client),
            (CHECK_STATE_VALIDITY_SERVICE, self.state_validity_client),
        ):
            if not client.wait_for_service(timeout_sec=service_wait_timeout_s):
                raise RuntimeError(
                    f"Offline MoveIt planning service unavailable: {service_name}"
                )
        self.planning_adapter = MoveItObjectTrajectoryPlanningAdapter(
            self.compute_ik_client,
            self.state_validity_client,
            self.get_clock(),
            self._call,
        )

    @staticmethod
    def _fill_seed(robot_state: object, seed: Sequence[float]) -> None:
        robot_state.joint_state.name = list(DUAL_ARM_JOINT_ORDER)
        robot_state.joint_state.position = list(seed)
        robot_state.is_diff = False

    def _call(self, client: object, request: object, timeout_s: float) -> object:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise TimeoutError("MoveIt planning service response timed out")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"MoveIt planning service failed: {exception}")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt planning service returned no response")
        return response

    def solve_arm_ik(
        self,
        *,
        group_name: str,
        ik_link_name: str,
        target_world_T_tip: RigidTransform,
        seed_joint_positions_rad: JointVector12,
        timeout_s: float,
        avoid_collisions: bool,
    ) -> ArmIkSolution:
        return self.planning_adapter.solve_arm_ik(
            group_name=group_name,
            ik_link_name=ik_link_name,
            target_world_T_tip=target_world_T_tip,
            seed_joint_positions_rad=seed_joint_positions_rad,
            timeout_s=timeout_s,
            avoid_collisions=avoid_collisions,
        )

    def check_combined_state(
        self,
        *,
        joint_positions_rad: JointVector12,
        group_name: str,
    ) -> CombinedStateValidity:
        return self.planning_adapter.check_combined_state(
            joint_positions_rad=joint_positions_rad,
            group_name=group_name,
        )


def print_jump_profile(config: JointJumpDetectionConfig) -> None:
    """Print explicit non-safety heuristic configuration before ROS startup."""
    print(JUMP_PROFILE_BANNER)
    absolute = (
        f"{config.absolute_step_threshold_rad:.6f} rad"
        if config.absolute_step_threshold_rad is not None
        else "DISABLED"
    )
    relative = (
        f"{config.relative_step_ratio_threshold:.6f}"
        if config.relative_step_ratio_threshold is not None
        else "DISABLED"
    )
    print(f"DETECTOR ENABLED = {'YES' if config.enabled else 'NO'}")
    print(f"ABSOLUTE THRESHOLD = {absolute}")
    print(f"RELATIVE RATIO THRESHOLD = {relative}")
    print(f"REFERENCE FLOOR = {config.relative_reference_floor_rad:.6f} rad")


def print_candidate_exploration_profile(
    config: IkCandidateExplorationConfig,
) -> None:
    print(CANDIDATE_PROFILE_BANNER)
    print(f"IK ATTEMPTS PER ARM = {config.max_attempts_per_arm}")
    print(f"SINGLE-JOINT SEED OFFSET = {config.perturbation_offset_rad:.6f} rad")
    print(f"RAW DUPLICATE TOLERANCE = {config.duplicate_tolerance_rad:.8f} rad")
    print(f"RANKING TIE TOLERANCE = {config.ranking_tie_tolerance:.3e}")
    print("EXPLORATORY SEED LIMIT POLICY = CLAMP")
    print(f"MODEL LIMIT METADATA = {JOINT_LIMIT_METADATA_PATH}")


def print_result_report(
    result: ObjectTrajectoryIkResult,
    jump_config: JointJumpDetectionConfig = DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG,
) -> None:
    print("Sample Time(s) Left IK Right IK Pair Valid Max |dq| (rad)")
    for sample in result.samples:
        maximum = (
            f"{sample.max_abs_joint_step_rad:.6f}"
            if sample.max_abs_joint_step_rad is not None
            else "N/A"
        )
        print(
            f"{sample.sample_index:>6} {sample.time_from_start_s:>7.2f} "
            f"{'PASS' if sample.left_ik_success else 'FAIL':>7} "
            f"{'PASS' if sample.right_ik_success else 'FAIL':>8} "
            f"{'PASS' if sample.combined_valid else 'FAIL':>10} {maximum:>14}"
        )
        if sample.failure_reason:
            print(f"  Failure: {sample.failure_reason}: {sample.diagnostic_message}")
        print(f"  Left IK attempts: {sample.left_ik_attempt_count}")
        print(f"  Left unique candidates: {sample.left_unique_candidate_count}")
        print(f"  Right IK attempts: {sample.right_ik_attempt_count}")
        print(f"  Right unique candidates: {sample.right_unique_candidate_count}")
        print(f"  Candidate pairs: {sample.candidate_pair_count}")
        print(f"  Valid dual-arm pairs: {sample.valid_candidate_pair_count}")
        if sample.selected_pair is not None:
            print(f"  Selected Left candidate: {sample.selected_left_candidate_index}")
            print(f"  Selected Right candidate: {sample.selected_right_candidate_index}")
            print(f"  Selected pair: {sample.selected_pair_index}")
            print(
                "  Raw continuity cost: "
                f"{sample.selected_raw_continuity_cost:.9f} rad^2"
            )
            print(
                "  Max raw joint step: "
                f"{sample.selected_max_raw_joint_step_rad:.6f} rad"
            )
            print(
                "  Max-step joint: "
                f"{sample.selected_max_raw_joint_step_name} "
                f"[{sample.selected_max_raw_joint_step_index}]"
            )
    print(f"Trajectory completed: {'YES' if result.completed else 'NO'}")
    print(
        f"Accepted samples: {result.accepted_sample_count} / "
        f"{result.requested_sample_count}"
    )
    failed = "NONE" if result.failed_sample_index is None else result.failed_sample_index
    print(f"Failed sample: {failed}")
    maximum = result.maximum_observed_joint_step_rad
    print(
        "Maximum observed joint step: "
        + (f"{maximum:.6f} rad" if maximum is not None else "N/A")
    )
    print_continuity_report(result, result.analyze_suspicious_jumps(jump_config))


def print_continuity_report(
    result: ObjectTrajectoryIkResult,
    jump_analysis: TrajectoryJumpAnalysis | None = None,
) -> None:
    """Print raw and shortest-angular metrics without modifying positions."""
    if jump_analysis is None:
        jump_analysis = result.analyze_suspicious_jumps()
    print()
    print(RAW_JOINT_DELTA_NOTICE)
    print(RAW_JOINT_DELTA_PRESERVED_NOTICE)
    print(SHORTEST_ANGULAR_ANALYSIS_NOTICE)
    print(JOINT_POSITIONS_UNCHANGED_NOTICE)
    for transition, jump_transition in zip(
        result.continuity_transitions,
        jump_analysis.transitions,
    ):
        print()
        print(
            f"Transition {transition.from_sample_index} -> "
            f"{transition.to_sample_index}"
        )
        print(
            "Joint                 q_prev(rad) q_curr(rad) "
            "dq_raw(rad) |dq_raw| dq_short(rad) |dq_short| Wrap?"
        )
        for record in transition.joint_deltas:
            print(
                f"{record.joint_name:<22} "
                f"{record.previous_position_rad:>11.6f} "
                f"{record.current_position_rad:>11.6f} "
                f"{record.delta_rad:>11.6f} "
                f"{record.abs_delta_rad:>8.6f} "
                f"{record.shortest_delta_rad:>13.6f} "
                f"{record.shortest_abs_delta_rad:>10.6f} "
                f"{'YES' if record.wraparound_adjusted else 'NO':>5}"
            )
        print(
            "Raw max: "
            f"{transition.max_joint_name} [{transition.max_joint_index}] = "
            f"{transition.max_abs_joint_step_rad:.6f} rad "
            f"(signed {transition.max_joint_delta_rad:.6f} rad)"
        )
        print(
            "Shortest-angular max: "
            f"{transition.max_shortest_joint_name} "
            f"[{transition.max_shortest_joint_index}] = "
            f"{transition.max_shortest_abs_joint_step_rad:.6f} rad "
            f"(signed {transition.max_shortest_joint_delta_rad:.6f} rad)"
        )
        print("Jump assessment:")
        if not jump_transition.has_suspicious_jump:
            print("Suspicious joints: NONE")
        else:
            for assessment in jump_transition.assessments:
                if not assessment.suspicious:
                    continue
                print(f"{assessment.joint_name}:")
                for reason in assessment.reasons:
                    print(f"  {reason}")
                print(
                    "  shortest |dq| = "
                    f"{assessment.shortest_abs_delta_rad:.6f} rad"
                )
                if ABSOLUTE_STEP_REASON in assessment.reasons:
                    print(
                        "  absolute threshold = "
                        f"{jump_analysis.config.absolute_step_threshold_rad:.6f} rad"
                    )
                if RELATIVE_GROWTH_REASON in assessment.reasons:
                    print(
                        "  previous |dq| = "
                        f"{assessment.previous_shortest_abs_delta_rad:.6f} rad"
                    )
                    print(f"  ratio = {assessment.relative_step_ratio:.6f}")
                    print(
                        "  ratio threshold = "
                        f"{jump_analysis.config.relative_step_ratio_threshold:.6f}"
                    )

    summary = result.continuity_summary
    print()
    print("Trajectory raw continuity maximum:")
    if summary.transition_count == 0:
        print("No accepted sample transitions; maximum: N/A")
    else:
        print(f"sample {summary.from_sample_index} -> {summary.to_sample_index}")
        print(f"joint: {summary.maximum_joint_name} [{summary.maximum_joint_index}]")
        print(f"signed delta: {summary.signed_delta_rad:.6f} rad")
        print(f"absolute delta: {summary.maximum_abs_joint_step_rad:.6f} rad")

    print()
    print("Trajectory shortest-angular continuity maximum:")
    if summary.transition_count == 0:
        print("No accepted sample transitions; maximum: N/A")
    else:
        print(
            f"sample {summary.shortest_from_sample_index} -> "
            f"{summary.shortest_to_sample_index}"
        )
        print(
            "joint: "
            f"{summary.maximum_shortest_joint_name} "
            f"[{summary.maximum_shortest_joint_index}]"
        )
        print(f"signed delta: {summary.signed_shortest_delta_rad:.6f} rad")
        print(
            "absolute delta: "
            f"{summary.maximum_shortest_abs_joint_step_rad:.6f} rad"
        )
    print(
        "Wraparound-adjusted transitions: "
        f"{summary.wraparound_adjusted_transition_count}"
    )
    print(
        "Wraparound-adjusted records: "
        f"{summary.wraparound_adjusted_record_count}"
    )
    print(
        "Suspicious jump transitions: "
        f"{jump_analysis.suspicious_transition_count}"
    )
    print(
        "Suspicious joint records: "
        f"{jump_analysis.suspicious_joint_record_count}"
    )
    print("Jump heuristic result:")
    if jump_analysis.suspicious_transition_count == 0:
        print("NO SUSPICIOUS IK JUMPS FLAGGED")
    else:
        print("SUSPICIOUS IK JUMPS FLAGGED FOR OFFLINE INVESTIGATION")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="OFFLINE MoveIt-only sequential object trajectory IK",
    )
    parser.add_argument("--ik-timeout", type=float, default=DEFAULT_IK_TIMEOUT_S)
    parser.add_argument("--service-wait-timeout", type=float, default=5.0)
    parser.add_argument(
        "--candidate-attempts-per-arm",
        type=int,
        default=DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.max_attempts_per_arm,
    )
    parser.add_argument(
        "--exploration-offset-rad",
        type=float,
        default=DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.perturbation_offset_rad,
    )
    parser.add_argument(
        "--duplicate-tolerance-rad",
        type=float,
        default=DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.duplicate_tolerance_rad,
    )
    parser.add_argument(
        "--ranking-tie-tolerance",
        type=float,
        default=DEFAULT_OFFLINE_CANDIDATE_EXPLORATION_CONFIG.ranking_tie_tolerance,
    )
    arguments = parser.parse_args()

    exploration_config = IkCandidateExplorationConfig(
        perturbation_offset_rad=arguments.exploration_offset_rad,
        max_attempts_per_arm=arguments.candidate_attempts_per_arm,
        duplicate_tolerance_rad=arguments.duplicate_tolerance_rad,
        ranking_tie_tolerance=arguments.ranking_tie_tolerance,
    )

    print(SAFETY_BANNER)
    print(OFFLINE_MODEL_SEED_NOTICE)
    print_jump_profile(DEFAULT_OFFLINE_JUMP_DETECTION_CONFIG)
    print_candidate_exploration_profile(exploration_config)
    joint_limits = load_canonical_joint_limits()
    rclpy.init()
    node: MoveItObjectTrajectoryIkNode | None = None
    try:
        node = MoveItObjectTrajectoryIkNode(arguments.service_wait_timeout)
        result = solve_sequential_object_trajectory_ik(
            SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
            node,
            ik_timeout_s=arguments.ik_timeout,
            candidate_exploration_config=exploration_config,
            joint_limits=joint_limits,
        )
        print_result_report(result)
        return 0 if result.completed else 1
    except Exception as error:
        print(f"OFFLINE planning prototype failed: {error}")
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
