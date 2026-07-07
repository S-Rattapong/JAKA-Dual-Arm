import ast

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from jaka_a12_dual_planner.markers import make_preview_markers
from jaka_a12_dual_planner.moveit_validation import (
    MoveItValidationClient,
    pose_stamped_from_transform,
)
from jaka_a12_dual_planner.object_frame import (
    compose_transform,
    invert_transform,
    transform_from_stamped,
    transform_from_translation_rpy,
    transform_to_stamped,
)
from jaka_a12_dual_planner.validation import validate_preview


def _list_param(node, name, expected_length):
    value = node.get_parameter(name).value
    if isinstance(value, str):
        value = ast.literal_eval(value)
    value = list(value)
    if len(value) != expected_length:
        raise ValueError(
            f"Parameter {name} must have {expected_length} values, got {value!r}"
        )
    return np.array([float(item) for item in value], dtype=float)


def _xyz_text(value):
    return f"[{value[0]:+.4f}, {value[1]:+.4f}, {value[2]:+.4f}]"


class DualObjectFrameMoveItValidationNode(Node):
    def __init__(self):
        super().__init__("dual_object_frame_moveit_validation_node")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("left_tcp_frame", "left_J6")
        self.declare_parameter("right_tcp_frame", "right_J6")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_dx", 0.0)
        self.declare_parameter("target_dy", 0.0)
        self.declare_parameter("target_dz", 0.05)
        self.declare_parameter("target_roll", 0.0)
        self.declare_parameter("target_pitch", 0.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("axis_length", 0.12)
        self.declare_parameter("workspace_min", [-2.0, -2.0, -0.1])
        self.declare_parameter("workspace_max", [2.0, 2.0, 2.0])
        self.declare_parameter("max_relative_distance_error", 0.005)
        self.declare_parameter("max_relative_orientation_error_rad", 0.03)
        self.declare_parameter("max_target_displacement", 0.30)
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("planning_time", 2.0)
        self.declare_parameter("plan_attempts", 3)
        self.declare_parameter("validate_once", False)
        self.declare_parameter("timer_period", 1.0)

        self.reference_frame = self.get_parameter("reference_frame").value
        self.left_tcp_frame = self.get_parameter("left_tcp_frame").value
        self.right_tcp_frame = self.get_parameter("right_tcp_frame").value
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.axis_length = float(self.get_parameter("axis_length").value)
        self.workspace_min = _list_param(self, "workspace_min", 3)
        self.workspace_max = _list_param(self, "workspace_max", 3)
        self.max_relative_distance_error = float(
            self.get_parameter("max_relative_distance_error").value
        )
        self.max_relative_orientation_error_rad = float(
            self.get_parameter("max_relative_orientation_error_rad").value
        )
        self.max_target_displacement = float(
            self.get_parameter("max_target_displacement").value
        )
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.plan_attempts = int(self.get_parameter("plan_attempts").value)
        self.validate_once = bool(self.get_parameter("validate_once").value)
        self.timer_period = max(0.2, float(self.get_parameter("timer_period").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/dual_object_frame_preview/markers",
            10,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10,
        )

        self.moveit = MoveItValidationClient(self)

        self.initialized = False
        self.validation_complete_once = False
        self.pending_future = None
        self.pending_stage = None
        self.preview_context = None
        self.latest_joint_state = None
        self.T_world_object_initial = None
        self.T_object_left_grasp = None
        self.T_object_right_grasp = None

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Dual object-frame MoveIt validation started. Validation only: "
            "no /joint_states publishing, no JointTrajectory publishing, "
            "no ExecuteTrajectory, and no FollowJointTrajectory action."
        )
        self.get_logger().info(
            "Expected MoveIt services: /compute_ik and /plan_kinematic_path. "
            "Start dual_moveit_rviz.launch.py first so move_group is available."
        )

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def lookup_tcp_transforms(self):
        left_tf = self.tf_buffer.lookup_transform(
            self.reference_frame,
            self.left_tcp_frame,
            Time(),
        )
        right_tf = self.tf_buffer.lookup_transform(
            self.reference_frame,
            self.right_tcp_frame,
            Time(),
        )
        return transform_from_stamped(left_tf), transform_from_stamped(right_tf)

    def initialize_from_current_tf(self, T_world_left, T_world_right):
        midpoint = 0.5 * (T_world_left[:3, 3] + T_world_right[:3, 3])
        self.T_world_object_initial = transform_from_translation_rpy(
            midpoint,
            0.0,
            0.0,
            0.0,
        )
        T_object_world = invert_transform(self.T_world_object_initial)
        self.T_object_left_grasp = compose_transform(T_object_world, T_world_left)
        self.T_object_right_grasp = compose_transform(T_object_world, T_world_right)
        self.initialized = True
        self.get_logger().info(
            "Initialized MoveIt validation object frame from current TCP midpoint: "
            f"object={_xyz_text(midpoint)}"
        )

    def target_delta_transform(self):
        return transform_from_translation_rpy(
            [
                float(self.get_parameter("target_dx").value),
                float(self.get_parameter("target_dy").value),
                float(self.get_parameter("target_dz").value),
            ],
            float(self.get_parameter("target_roll").value),
            float(self.get_parameter("target_pitch").value),
            float(self.get_parameter("target_yaw").value),
        )

    def compute_preview(self):
        T_world_object_target = compose_transform(
            self.T_world_object_initial,
            self.target_delta_transform(),
        )
        T_world_left_target = compose_transform(
            T_world_object_target,
            self.T_object_left_grasp,
        )
        T_world_right_target = compose_transform(
            T_world_object_target,
            self.T_object_right_grasp,
        )
        return T_world_object_target, T_world_left_target, T_world_right_target

    def publish_preview(
        self,
        T_world_object_target,
        T_world_left_target,
        T_world_right_target,
        T_world_actual_object,
    ):
        stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(
            [
                transform_to_stamped(
                    T_world_object_target,
                    self.reference_frame,
                    "target_object",
                    stamp,
                ),
                transform_to_stamped(
                    T_world_left_target,
                    self.reference_frame,
                    "target_left_grasp",
                    stamp,
                ),
                transform_to_stamped(
                    T_world_right_target,
                    self.reference_frame,
                    "target_right_grasp",
                    stamp,
                ),
                transform_to_stamped(
                    T_world_actual_object,
                    self.reference_frame,
                    "actual_object_midpoint",
                    stamp,
                ),
            ]
        )
        self.marker_pub.publish(
            make_preview_markers(
                self.reference_frame,
                stamp,
                T_world_object_target,
                T_world_left_target,
                T_world_right_target,
                self.axis_length,
                include_grasp_line=True,
                T_actual_object=T_world_actual_object,
            )
        )

    def local_final_status(self, validation):
        workspace_ok = validation["left_workspace_ok"] and validation[
            "right_workspace_ok"
        ]
        displacement_ok = validation["left_displacement_ok"] and validation[
            "right_displacement_ok"
        ]
        if not workspace_ok or not displacement_ok:
            return "WORKSPACE_FAILED"
        if (
            not validation["relative_distance_ok"]
            or not validation["relative_orientation_ok"]
        ):
            return "RELATIVE_CONSTRAINT_FAILED"
        return None

    def begin_validation(self):
        try:
            T_world_left, T_world_right = self.lookup_tcp_transforms()
        except TransformException as ex:
            self.get_logger().warn(f"Waiting for TCP TF frames: {ex}")
            return

        if not self.initialized:
            self.initialize_from_current_tf(T_world_left, T_world_right)

        T_world_object_target, T_world_left_target, T_world_right_target = (
            self.compute_preview()
        )
        actual_midpoint = 0.5 * (T_world_left[:3, 3] + T_world_right[:3, 3])
        T_world_actual_object = transform_from_translation_rpy(
            actual_midpoint,
            0.0,
            0.0,
            0.0,
        )
        validation = validate_preview(
            T_world_left,
            T_world_right,
            T_world_left_target,
            T_world_right_target,
            self.workspace_min,
            self.workspace_max,
            self.max_relative_distance_error,
            self.max_relative_orientation_error_rad,
            self.max_target_displacement,
        )
        self.publish_preview(
            T_world_object_target,
            T_world_left_target,
            T_world_right_target,
            T_world_actual_object,
        )

        final_status = self.local_final_status(validation)
        self.preview_context = {
            "validation": validation,
            "T_world_left_target": T_world_left_target,
            "T_world_right_target": T_world_right_target,
            "left_ik": None,
            "right_ik": None,
            "left_plan": None,
            "right_plan": None,
        }

        if final_status is not None:
            self.finish_validation(final_status)
            return

        if not self.moveit.wait_for_services_once(timeout_sec=0.0):
            self.get_logger().warn(
                "MoveIt validation waiting for /compute_ik and "
                "/plan_kinematic_path."
            )
            return

        stamp = self.get_clock().now().to_msg()
        left_pose = pose_stamped_from_transform(
            T_world_left_target,
            self.reference_frame,
            stamp,
        )
        self.pending_future = self.moveit.send_ik_request(
            "left_arm",
            self.left_tcp_frame,
            left_pose,
            self.latest_joint_state,
            self.ik_timeout,
        )
        self.pending_stage = "left_ik"

    def poll_pending_validation(self):
        if self.pending_future is None:
            return
        if not self.pending_future.done():
            return

        future = self.pending_future
        stage = self.pending_stage
        self.pending_future = None
        self.pending_stage = None

        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f"MoveIt service call failed during {stage}: {ex}")
            self.finish_validation("IK_FAILED" if "ik" in stage else "PLAN_FAILED")
            return

        if stage == "left_ik":
            result = self.moveit.parse_ik_response(response)
            self.preview_context["left_ik"] = result
            if not result.ok:
                self.finish_validation("IK_FAILED")
                return

            stamp = self.get_clock().now().to_msg()
            right_pose = pose_stamped_from_transform(
                self.preview_context["T_world_right_target"],
                self.reference_frame,
                stamp,
            )
            self.pending_future = self.moveit.send_ik_request(
                "right_arm",
                self.right_tcp_frame,
                right_pose,
                self.latest_joint_state,
                self.ik_timeout,
            )
            self.pending_stage = "right_ik"
            return

        if stage == "right_ik":
            result = self.moveit.parse_ik_response(response)
            self.preview_context["right_ik"] = result
            if not result.ok:
                self.finish_validation("IK_FAILED")
                return

            if self.latest_joint_state is None:
                self.get_logger().warn(
                    "No /joint_states received; IK succeeded but planning is "
                    "marked failed because a start state is unavailable."
                )
                self.finish_validation("PLAN_FAILED")
                return

            self.pending_future = self.moveit.send_motion_plan_request(
                "left_arm",
                self.latest_joint_state,
                self.preview_context["left_ik"].response.solution.joint_state,
                [f"left_joint_{index}" for index in range(1, 7)],
                self.planning_time,
                self.plan_attempts,
            )
            self.pending_stage = "left_plan"
            return

        if stage == "left_plan":
            result = self.moveit.parse_plan_response(response)
            self.preview_context["left_plan"] = result
            if not result.ok:
                self.finish_validation("PLAN_FAILED")
                return

            self.pending_future = self.moveit.send_motion_plan_request(
                "right_arm",
                self.latest_joint_state,
                self.preview_context["right_ik"].response.solution.joint_state,
                [f"right_joint_{index}" for index in range(1, 7)],
                self.planning_time,
                self.plan_attempts,
            )
            self.pending_stage = "right_plan"
            return

        if stage == "right_plan":
            result = self.moveit.parse_plan_response(response)
            self.preview_context["right_plan"] = result
            self.finish_validation("VALID_TARGET" if result.ok else "PLAN_FAILED")

    def result_text(self, key):
        result = self.preview_context.get(key)
        if result is None:
            return "not_run"
        return f"{result.ok}({result.message})"

    def finish_validation(self, final_status):
        validation = self.preview_context["validation"]
        self.get_logger().info(
            "MoveIt preview validation | "
            f"status={final_status} | "
            f"left={_xyz_text(validation['left_target_position'])} | "
            f"right={_xyz_text(validation['right_target_position'])} | "
            f"workspace_ok="
            f"{validation['left_workspace_ok'] and validation['right_workspace_ok']} | "
            f"rel_dist_err={validation['relative_distance_error']:+.6f} m | "
            f"rel_ori_err={validation['relative_orientation_error_rad']:.5f} rad "
            f"({validation['relative_orientation_error_deg']:.3f} deg) | "
            f"left_ik={self.result_text('left_ik')} | "
            f"right_ik={self.result_text('right_ik')} | "
            f"left_plan={self.result_text('left_plan')} | "
            f"right_plan={self.result_text('right_plan')}"
        )
        self.validation_complete_once = True
        self.preview_context = None

    def timer_callback(self):
        if self.pending_future is not None:
            self.poll_pending_validation()
            return
        if self.validate_once and self.validation_complete_once:
            return
        self.begin_validation()


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectFrameMoveItValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
