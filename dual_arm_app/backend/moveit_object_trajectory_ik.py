"""ROS2 MoveIt adapter and Phase 1G.3A raw-continuity report.

MOVEIT PLANNING ONLY. NO JAKA DRIVER. NO ROBOT CONNECTION. NO MOTION EXECUTION.
This module creates only ``/compute_ik`` and ``/check_state_validity`` service
clients. It creates no publishers, controllers, trajectory actions, or hardware
driver clients.
"""

from __future__ import annotations

import argparse
from typing import Sequence

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node

from dual_arm_app.backend.object_grasp_model import RigidTransform
from dual_arm_app.backend.object_trajectory import (
    SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
)
from dual_arm_app.backend.object_trajectory_ik import (
    DEFAULT_IK_TIMEOUT_S,
    DUAL_ARM_GROUP_NAME,
    DUAL_ARM_JOINT_ORDER,
    LEFT_JOINT_ORDER,
    OFFLINE_MODEL_SEED_NOTICE,
    RAW_JOINT_DELTA_NOTICE,
    RIGHT_JOINT_ORDER,
    ArmIkSolution,
    CombinedStateValidity,
    JointVector12,
    ObjectTrajectoryIkResult,
    rotation_matrix_to_quaternion_xyzw,
    solve_sequential_object_trajectory_ik,
)


COMPUTE_IK_SERVICE = "/compute_ik"
CHECK_STATE_VALIDITY_SERVICE = "/check_state_validity"
WORLD_FRAME = "world"

SAFETY_BANNER = """==================================================
OFFLINE OBJECT TRAJECTORY IK
MOVEIT PLANNING ONLY
NO JAKA DRIVER
NO ROBOT CONNECTION
NO MOTION EXECUTION
=================================================="""


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
        pose.header.stamp = self.get_clock().now().to_msg()
        matrix = target_world_T_tip.matrix
        pose.pose.position.x = matrix[0][3]
        pose.pose.position.y = matrix[1][3]
        pose.pose.position.z = matrix[2][3]
        quaternion = rotation_matrix_to_quaternion_xyzw(target_world_T_tip)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        response = self._call(
            self.compute_ik_client,
            request,
            timeout_s + 1.0,
        )
        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            return ArmIkSolution(
                False,
                diagnostic=f"MoveIt IK error_code={error_code}",
            )

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
        response = self._call(self.state_validity_client, request, 2.0)
        contact_count = len(response.contacts)
        diagnostic = (
            "Combined state valid"
            if response.valid
            else f"Combined state invalid; contacts={contact_count}"
        )
        return CombinedStateValidity(bool(response.valid), diagnostic)


def print_result_report(result: ObjectTrajectoryIkResult) -> None:
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
    print_continuity_report(result)


def print_continuity_report(result: ObjectTrajectoryIkResult) -> None:
    """Print raw per-joint analysis; never apply wrapping or rejection."""
    print()
    print(RAW_JOINT_DELTA_NOTICE)
    for transition in result.continuity_transitions:
        print()
        print(
            f"Transition {transition.from_sample_index} -> "
            f"{transition.to_sample_index}"
        )
        print("Joint                 q_prev(rad) q_curr(rad)    dq(rad)  |dq|(rad)")
        for record in transition.joint_deltas:
            print(
                f"{record.joint_name:<22} "
                f"{record.previous_position_rad:>11.6f} "
                f"{record.current_position_rad:>11.6f} "
                f"{record.delta_rad:>10.6f} "
                f"{record.abs_delta_rad:>10.6f}"
            )
        print(
            "Max joint step: "
            f"{transition.max_joint_name} [{transition.max_joint_index}] = "
            f"{transition.max_abs_joint_step_rad:.6f} rad "
            f"(signed {transition.max_joint_delta_rad:.6f} rad)"
        )

    summary = result.continuity_summary
    print()
    print("Trajectory raw continuity maximum:")
    if summary.transition_count == 0:
        print("No accepted sample transitions; maximum: N/A")
        return
    print(f"sample {summary.from_sample_index} -> {summary.to_sample_index}")
    print(f"joint: {summary.maximum_joint_name} [{summary.maximum_joint_index}]")
    print(f"signed delta: {summary.signed_delta_rad:.6f} rad")
    print(f"absolute delta: {summary.maximum_abs_joint_step_rad:.6f} rad")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="OFFLINE MoveIt-only sequential object trajectory IK",
    )
    parser.add_argument("--ik-timeout", type=float, default=DEFAULT_IK_TIMEOUT_S)
    parser.add_argument("--service-wait-timeout", type=float, default=5.0)
    arguments = parser.parse_args()

    print(SAFETY_BANNER)
    print(OFFLINE_MODEL_SEED_NOTICE)
    rclpy.init()
    node: MoveItObjectTrajectoryIkNode | None = None
    try:
        node = MoveItObjectTrajectoryIkNode(arguments.service_wait_timeout)
        result = solve_sequential_object_trajectory_ik(
            SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY,
            node,
            ik_timeout_s=arguments.ik_timeout,
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
