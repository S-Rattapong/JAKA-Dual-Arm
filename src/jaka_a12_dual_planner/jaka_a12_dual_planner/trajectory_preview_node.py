import ast

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from jaka_a12_dual_planner.markers import (
    make_axis_markers,
    make_line_marker,
    make_sphere_marker,
)
from jaka_a12_dual_planner.moveit_validation import (
    MoveItValidationClient,
    merge_joint_states,
    pose_stamped_from_transform,
)
from jaka_a12_dual_planner.object_frame import (
    compose_transform,
    invert_transform,
    quaternion_slerp,
    rotation_matrix_to_quaternion,
    transform_from_stamped,
    transform_from_translation_quaternion,
    transform_from_translation_rpy,
    transform_to_stamped,
)
from jaka_a12_dual_planner.validation import validate_preview


LEFT_JOINT_NAMES = [f"left_joint_{index}" for index in range(1, 7)]
RIGHT_JOINT_NAMES = [f"right_joint_{index}" for index in range(1, 7)]


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


def _joint_positions(joint_state, joint_names):
    if joint_state is None:
        return None
    positions = dict(zip(joint_state.name, joint_state.position))
    try:
        return np.array([float(positions[name]) for name in joint_names], dtype=float)
    except KeyError:
        return None


class DualObjectTrajectoryPreviewNode(Node):
    def __init__(self):
        super().__init__("dual_object_trajectory_preview_node")

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
        self.declare_parameter("num_waypoints", 10)
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("max_joint_jump", 0.35)
        self.declare_parameter("validate_once", True)
        self.declare_parameter("timer_period", 2.0)
        self.declare_parameter("workspace_min", [-2.0, -2.0, -0.1])
        self.declare_parameter("workspace_max", [2.0, 2.0, 2.0])
        self.declare_parameter("max_relative_distance_error", 0.005)
        self.declare_parameter("max_relative_orientation_error_rad", 0.03)
        self.declare_parameter("max_target_displacement", 0.30)

        self.reference_frame = self.get_parameter("reference_frame").value
        self.left_tcp_frame = self.get_parameter("left_tcp_frame").value
        self.right_tcp_frame = self.get_parameter("right_tcp_frame").value
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.num_waypoints = max(2, int(self.get_parameter("num_waypoints").value))
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.max_joint_jump = float(self.get_parameter("max_joint_jump").value)
        self.validate_once = bool(self.get_parameter("validate_once").value)
        self.timer_period = max(0.2, float(self.get_parameter("timer_period").value))
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/dual_object_trajectory_preview/markers",
            10,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_state_callback,
            10,
        )
        self.moveit = MoveItValidationClient(self)

        self.latest_joint_state = None
        self.initialized = False
        self.preview_complete_once = False
        self.state_validity_skipped_logged = False
        self.pending_future = None
        self.pending_stage = None
        self.current_index = 0
        self.waypoints = []
        self.T_world_object_initial = None
        self.T_object_left_grasp = None
        self.T_object_right_grasp = None
        self.T_world_left_current = None
        self.T_world_right_current = None
        self.previous_left_solution = None
        self.previous_right_solution = None
        self.max_left_joint_jump_seen = 0.0
        self.max_right_joint_jump_seen = 0.0
        self.markers_cleared = False

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Dual object trajectory preview started. Preview only: no "
            "/joint_states publishing, no controller JointTrajectory publishing, "
            "no ExecuteTrajectory, no FollowJointTrajectory action, and no JAKA "
            "driver service calls."
        )

    def clear_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(marker)
        self.marker_pub.publish(markers)
        self.markers_cleared = True

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
            "Initialized trajectory preview object frame from current TCP "
            f"midpoint: object={_xyz_text(midpoint)}"
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

    def interpolated_object_transform(self, target_delta, alpha):
        start_translation = self.T_world_object_initial[:3, 3]
        target_object = compose_transform(self.T_world_object_initial, target_delta)
        target_translation = target_object[:3, 3]
        translation = start_translation + alpha * (target_translation - start_translation)

        q0 = rotation_matrix_to_quaternion(self.T_world_object_initial[:3, :3])
        q1 = rotation_matrix_to_quaternion(target_object[:3, :3])
        q = quaternion_slerp(q0, q1, alpha)
        return transform_from_translation_quaternion(translation, q)

    def build_waypoints(self):
        target_delta = self.target_delta_transform()
        waypoints = []
        for index in range(self.num_waypoints):
            alpha = float(index + 1) / float(self.num_waypoints)
            T_object = self.interpolated_object_transform(target_delta, alpha)
            T_left = compose_transform(T_object, self.T_object_left_grasp)
            T_right = compose_transform(T_object, self.T_object_right_grasp)
            waypoints.append(
                {
                    "index": index,
                    "alpha": alpha,
                    "T_object": T_object,
                    "T_left": T_left,
                    "T_right": T_right,
                    "status": "PENDING",
                    "reason": "PENDING",
                    "validation": None,
                    "left_ik": None,
                    "right_ik": None,
                    "state_validity": None,
                    "left_joint_solution": None,
                    "right_joint_solution": None,
                    "left_joint_jump": 0.0,
                    "right_joint_jump": 0.0,
                    "max_joint_jump": 0.0,
                }
            )
        return waypoints

    def local_status(self, validation):
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

    def publish_final_target_tf(self):
        if not self.waypoints:
            return
        final = self.waypoints[-1]
        stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(
            [
                transform_to_stamped(
                    final["T_object"],
                    self.reference_frame,
                    "preview_target_object",
                    stamp,
                ),
                transform_to_stamped(
                    final["T_left"],
                    self.reference_frame,
                    "preview_target_left_grasp",
                    stamp,
                ),
                transform_to_stamped(
                    final["T_right"],
                    self.reference_frame,
                    "preview_target_right_grasp",
                    stamp,
                ),
            ]
        )

    def marker_color_for_waypoint(self, waypoint):
        if waypoint["status"] == "VALID":
            return [0.1, 0.85, 0.2, 0.85]
        if waypoint["status"] == "INVALID":
            return [1.0, 0.1, 0.1, 0.9]
        return [1.0, 0.85, 0.1, 0.75]

    def make_line_strip_marker(self, stamp, marker_id, namespace, points, width, color):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        for point in points:
            ros_point = Point()
            ros_point.x = float(point[0])
            ros_point.y = float(point[1])
            ros_point.z = float(point[2])
            marker.points.append(ros_point)
        return marker

    def make_text_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = stamp
        marker.ns = "preview_status_text"
        marker.id = 9000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.055
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95

        if self.waypoints:
            final_position = self.waypoints[-1]["T_object"][:3, 3]
            marker.pose.position.x = float(final_position[0])
            marker.pose.position.y = float(final_position[1])
            marker.pose.position.z = float(final_position[2] + 0.18)
        else:
            marker.pose.position.z = 0.5

        valid_count = sum(1 for waypoint in self.waypoints if waypoint["status"] == "VALID")
        first_invalid = next(
            (waypoint for waypoint in self.waypoints if waypoint["status"] == "INVALID"),
            None,
        )
        if first_invalid is None and self.preview_complete_once:
            status = "TRAJECTORY_PREVIEW_VALID"
            reason = "VALID"
        elif first_invalid is not None:
            status = "TRAJECTORY_PREVIEW_INVALID"
            reason = f"wp={first_invalid['index']} {first_invalid['reason']}"
        else:
            status = "TRAJECTORY_PREVIEW_PENDING"
            reason = "PENDING"
        marker.text = (
            f"{status}\nvalid={valid_count}/{len(self.waypoints)}\n"
            f"{reason}"
        )
        return marker

    def first_invalid_waypoint_index(self):
        for waypoint in self.waypoints:
            if waypoint["status"] == "INVALID":
                return waypoint["index"]
        return None

    def axis_waypoint_indices(self):
        if not self.waypoints:
            return set()
        indices = {0, len(self.waypoints) - 1}
        first_invalid = self.first_invalid_waypoint_index()
        if first_invalid is not None:
            indices.add(first_invalid)
        return indices

    def publish_markers(self):
        if not self.waypoints:
            return
        if not self.markers_cleared:
            self.clear_markers()
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        object_points = [waypoint["T_object"][:3, 3] for waypoint in self.waypoints]
        left_points = [waypoint["T_left"][:3, 3] for waypoint in self.waypoints]
        right_points = [waypoint["T_right"][:3, 3] for waypoint in self.waypoints]

        path_specs = [
            ("preview_object_path", object_points, [0.1, 0.8, 1.0, 0.9], 1, 0.010),
            ("preview_left_grasp_path", left_points, [1.0, 0.25, 0.15, 0.9], 2, 0.008),
            ("preview_right_grasp_path", right_points, [0.2, 0.45, 1.0, 0.9], 3, 0.008),
        ]
        for namespace, points, color, marker_id, width in path_specs:
            markers.markers.append(
                self.make_line_strip_marker(
                    stamp,
                    marker_id,
                    namespace,
                    points,
                    width,
                    color,
                )
            )

        axis_indices = self.axis_waypoint_indices()
        for waypoint in self.waypoints:
            index = waypoint["index"]
            color = self.marker_color_for_waypoint(waypoint)
            marker_scale = 0.030 if waypoint["status"] == "INVALID" else 0.020
            markers.markers.append(
                make_sphere_marker(
                    self.reference_frame,
                    stamp,
                    1000 + index,
                    "preview_object_waypoint",
                    waypoint["T_object"][:3, 3],
                    marker_scale,
                    color,
                )
            )
            markers.markers.append(
                make_line_marker(
                    self.reference_frame,
                    stamp,
                    2000 + index,
                    "preview_left_right_grasp_waypoint_line",
                    waypoint["T_left"][:3, 3],
                    waypoint["T_right"][:3, 3],
                    0.006,
                    color,
                )
            )
            if index in axis_indices:
                axis_namespace = f"preview_object_waypoint_{index:02d}_axis"
                for axis_marker in make_axis_markers(
                    self.reference_frame,
                    stamp,
                    axis_namespace,
                    3000 + index * 10,
                    waypoint["T_object"],
                    0.10,
                    0.008,
                ):
                    markers.markers.append(axis_marker)

        markers.markers.append(self.make_text_marker(stamp))
        self.marker_pub.publish(markers)
        self.publish_final_target_tf()

    def begin_preview(self):
        try:
            self.T_world_left_current, self.T_world_right_current = (
                self.lookup_tcp_transforms()
            )
        except TransformException as ex:
            self.get_logger().warn(f"Waiting for TCP TF frames: {ex}")
            return

        if not self.initialized:
            self.initialize_from_current_tf(
                self.T_world_left_current,
                self.T_world_right_current,
            )

        if not self.moveit.wait_for_ik_once(timeout_sec=0.0):
            self.get_logger().warn("Trajectory preview waiting for /compute_ik.")
            return

        self.clear_markers()
        if not self.moveit.state_validity_ready() and not self.state_validity_skipped_logged:
            self.state_validity_skipped_logged = True
            self.get_logger().warn(
                "/check_state_validity is not available; state/collision "
                "validity will be skipped."
            )

        self.waypoints = self.build_waypoints()
        self.current_index = 0
        self.preview_complete_once = False
        self.previous_left_solution = _joint_positions(
            self.latest_joint_state,
            LEFT_JOINT_NAMES,
        )
        self.previous_right_solution = _joint_positions(
            self.latest_joint_state,
            RIGHT_JOINT_NAMES,
        )
        self.max_left_joint_jump_seen = 0.0
        self.max_right_joint_jump_seen = 0.0
        self.get_logger().info(
            f"Starting trajectory preview: waypoints={len(self.waypoints)}, "
            f"target_dx={float(self.get_parameter('target_dx').value):+.3f}, "
            f"target_dy={float(self.get_parameter('target_dy').value):+.3f}, "
            f"target_dz={float(self.get_parameter('target_dz').value):+.3f}, "
            f"target_yaw={float(self.get_parameter('target_yaw').value):+.3f}, "
            f"max_joint_jump={self.max_joint_jump:.3f}"
        )
        self.advance_to_next_waypoint()

    def advance_to_next_waypoint(self):
        while self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            validation = validate_preview(
                self.T_world_left_current,
                self.T_world_right_current,
                waypoint["T_left"],
                waypoint["T_right"],
                self.workspace_min,
                self.workspace_max,
                self.max_relative_distance_error,
                self.max_relative_orientation_error_rad,
                self.max_target_displacement,
            )
            waypoint["validation"] = validation
            local_reason = self.local_status(validation)
            if local_reason is not None:
                waypoint["status"] = "INVALID"
                waypoint["reason"] = local_reason
                self.current_index += 1
                continue

            stamp = self.get_clock().now().to_msg()
            left_pose = pose_stamped_from_transform(
                waypoint["T_left"],
                self.reference_frame,
                stamp,
            )
            seed = self.seed_joint_state()
            self.pending_future = self.moveit.send_ik_request(
                "left_arm",
                self.left_tcp_frame,
                left_pose,
                seed,
                self.ik_timeout,
            )
            self.pending_stage = "left_ik"
            return

        self.finish_summary()

    def seed_joint_state(self):
        if self.previous_left_solution is None and self.previous_right_solution is None:
            return self.latest_joint_state

        seed = JointState()
        if self.latest_joint_state is not None:
            seed = merge_joint_states(self.latest_joint_state)
        if self.previous_left_solution is not None:
            left_state = JointState()
            left_state.name = LEFT_JOINT_NAMES
            left_state.position = [float(value) for value in self.previous_left_solution]
            seed = merge_joint_states(seed, left_state)
        if self.previous_right_solution is not None:
            right_state = JointState()
            right_state.name = RIGHT_JOINT_NAMES
            right_state.position = [float(value) for value in self.previous_right_solution]
            seed = merge_joint_states(seed, right_state)
        return seed

    def current_waypoint(self):
        if self.current_index >= len(self.waypoints):
            return None
        return self.waypoints[self.current_index]

    def poll_pending_preview(self):
        if self.pending_future is None or not self.pending_future.done():
            return

        future = self.pending_future
        stage = self.pending_stage
        self.pending_future = None
        self.pending_stage = None
        waypoint = self.current_waypoint()
        if waypoint is None:
            self.finish_summary()
            return

        try:
            response = future.result()
        except Exception as ex:
            waypoint["status"] = "INVALID"
            waypoint["reason"] = (
                "LEFT_IK_FAILED"
                if stage == "left_ik"
                else "RIGHT_IK_FAILED"
                if stage == "right_ik"
                else "STATE_VALIDITY_FAILED"
            )
            self.get_logger().error(
                f"MoveIt preview service call failed at waypoint {waypoint['index']} "
                f"during {stage}: {ex}"
            )
            self.current_index += 1
            self.advance_to_next_waypoint()
            return

        if stage == "left_ik":
            result = self.moveit.parse_ik_response(response)
            waypoint["left_ik"] = result
            if not result.ok:
                waypoint["status"] = "INVALID"
                waypoint["reason"] = "LEFT_IK_FAILED"
                self.current_index += 1
                self.advance_to_next_waypoint()
                return
            waypoint["left_joint_solution"] = _joint_positions(
                result.response.solution.joint_state,
                LEFT_JOINT_NAMES,
            )

            stamp = self.get_clock().now().to_msg()
            right_pose = pose_stamped_from_transform(
                waypoint["T_right"],
                self.reference_frame,
                stamp,
            )
            self.pending_future = self.moveit.send_ik_request(
                "right_arm",
                self.right_tcp_frame,
                right_pose,
                self.seed_joint_state(),
                self.ik_timeout,
            )
            self.pending_stage = "right_ik"
            return

        if stage == "right_ik":
            result = self.moveit.parse_ik_response(response)
            waypoint["right_ik"] = result
            if not result.ok:
                waypoint["status"] = "INVALID"
                waypoint["reason"] = "RIGHT_IK_FAILED"
                self.current_index += 1
                self.advance_to_next_waypoint()
                return
            waypoint["right_joint_solution"] = _joint_positions(
                result.response.solution.joint_state,
                RIGHT_JOINT_NAMES,
            )

            jump_reason = self.validate_joint_jump(waypoint)
            if jump_reason is not None:
                waypoint["status"] = "INVALID"
                waypoint["reason"] = jump_reason
                self.current_index += 1
                self.advance_to_next_waypoint()
                return

            if self.moveit.state_validity_ready():
                merged = merge_joint_states(
                    self.latest_joint_state,
                    result.response.solution.joint_state,
                    waypoint["left_ik"].response.solution.joint_state,
                )
                self.pending_future = self.moveit.send_state_validity_request(
                    merged,
                    "dual_arm",
                )
                self.pending_stage = "state_validity"
                return

            waypoint["state_validity"] = "skipped"
            self.accept_waypoint(waypoint)
            return

        if stage == "state_validity":
            result = self.moveit.parse_state_validity_response(response)
            waypoint["state_validity"] = result
            if not result.ok:
                waypoint["status"] = "INVALID"
                waypoint["reason"] = "STATE_VALIDITY_FAILED"
                self.current_index += 1
                self.advance_to_next_waypoint()
                return

            self.accept_waypoint(waypoint)

    def validate_joint_jump(self, waypoint):
        left = waypoint["left_joint_solution"]
        right = waypoint["right_joint_solution"]
        if left is None or right is None:
            waypoint["max_joint_jump"] = float("inf")
            return "JOINT_SOLUTION_UNAVAILABLE"

        left_jump = 0.0
        right_jump = 0.0
        if self.previous_left_solution is not None:
            left_jump = float(np.max(np.abs(left - self.previous_left_solution)))
        if self.previous_right_solution is not None:
            right_jump = float(np.max(np.abs(right - self.previous_right_solution)))

        waypoint["left_joint_jump"] = left_jump
        waypoint["right_joint_jump"] = right_jump
        waypoint["max_joint_jump"] = max(left_jump, right_jump)
        self.max_left_joint_jump_seen = max(self.max_left_joint_jump_seen, left_jump)
        self.max_right_joint_jump_seen = max(self.max_right_joint_jump_seen, right_jump)
        self.previous_left_solution = left
        self.previous_right_solution = right

        if waypoint["max_joint_jump"] > self.max_joint_jump:
            return "JOINT_JUMP_FAILED"
        return None

    def accept_waypoint(self, waypoint):
        waypoint["status"] = "VALID"
        waypoint["reason"] = "VALID"
        self.previous_left_solution = waypoint["left_joint_solution"]
        self.previous_right_solution = waypoint["right_joint_solution"]
        self.current_index += 1
        self.advance_to_next_waypoint()

    def result_text(self, result):
        if result is None:
            return "not_run"
        if result == "skipped":
            return "skipped"
        return f"{result.ok}({result.message})"

    def joint_text(self, solution):
        if solution is None:
            return "unavailable"
        return "[" + ", ".join(f"{value:+.3f}" for value in solution) + "]"

    def finish_summary(self):
        self.preview_complete_once = True
        self.publish_markers()
        valid_count = sum(1 for waypoint in self.waypoints if waypoint["status"] == "VALID")
        first_invalid = next(
            (waypoint for waypoint in self.waypoints if waypoint["status"] != "VALID"),
            None,
        )
        if first_invalid is None:
            final_status = "TRAJECTORY_PREVIEW_VALID"
            first_invalid_index = "none"
            reason = "VALID"
        else:
            final_status = "TRAJECTORY_PREVIEW_INVALID"
            first_invalid_index = str(first_invalid["index"])
            reason = first_invalid["reason"]

        final_validation = self.waypoints[-1]["validation"] if self.waypoints else None
        final_relative_distance_error = (
            final_validation["relative_distance_error"]
            if final_validation is not None
            else float("nan")
        )
        final_relative_orientation_error = (
            final_validation["relative_orientation_error_rad"]
            if final_validation is not None
            else float("nan")
        )

        for waypoint in self.waypoints:
            validation = waypoint["validation"]
            self.get_logger().info(
                f"Preview waypoint {waypoint['index']:02d} | "
                f"status={waypoint['status']} | reason={waypoint['reason']} | "
                f"object={_xyz_text(waypoint['T_object'][:3, 3])} | "
                f"left_joints={self.joint_text(waypoint['left_joint_solution'])} | "
                f"right_joints={self.joint_text(waypoint['right_joint_solution'])} | "
                f"left_jump={waypoint['left_joint_jump']:.5f} | "
                f"right_jump={waypoint['right_joint_jump']:.5f} | "
                f"state_validity={self.result_text(waypoint['state_validity'])} | "
                f"rel_dist_err={validation['relative_distance_error']:+.6f} m | "
                f"rel_ori_err={validation['relative_orientation_error_rad']:.5f} rad"
            )

        self.get_logger().info(
            f"{final_status} | total_waypoints={len(self.waypoints)} | "
            f"valid_waypoints={valid_count} | "
            f"first_invalid_waypoint={first_invalid_index} | reason={reason} | "
            f"maximum_left_joint_jump={self.max_left_joint_jump_seen:.5f} | "
            f"maximum_right_joint_jump={self.max_right_joint_jump_seen:.5f} | "
            f"final_relative_distance_error={final_relative_distance_error:+.6f} m | "
            f"final_relative_orientation_error={final_relative_orientation_error:.5f} rad"
        )
        self.pending_future = None
        self.pending_stage = None

    def timer_callback(self):
        self.publish_markers()
        if self.pending_future is not None:
            self.poll_pending_preview()
            return
        if self.validate_once and self.preview_complete_once:
            return
        if self.waypoints and not self.preview_complete_once:
            return
        self.begin_preview()


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectTrajectoryPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
