import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from jaka_a12_dual_planner.markers import (
    make_axis_markers,
    make_line_marker,
    make_sphere_marker,
)
from jaka_a12_dual_planner.moveit_validation import MoveItValidationClient
from jaka_a12_dual_planner.object_frame import transform_to_pose, transform_to_stamped
from jaka_a12_dual_planner.trajectory_display_node import (
    DualObjectTrajectoryDisplayNode,
)
from jaka_a12_dual_planner.trajectory_preview_node import _list_param


class DualObjectFinalDemoNode(DualObjectTrajectoryDisplayNode):
    def __init__(self):
        Node.__init__(self, "dual_object_final_demo_node")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("left_tcp_frame", "left_J6")
        self.declare_parameter("right_tcp_frame", "right_J6")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_dx", 0.0)
        self.declare_parameter("target_dy", 0.0)
        self.declare_parameter("target_dz", 0.05)
        self.declare_parameter("target_roll", 0.0)
        self.declare_parameter("target_pitch", 0.0)
        self.declare_parameter("target_yaw", 0.10)
        self.declare_parameter("num_waypoints", 10)
        self.declare_parameter("object_length", 0.70)
        self.declare_parameter("object_width", 0.12)
        self.declare_parameter("object_height", 0.04)
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("max_joint_jump", 0.35)
        self.declare_parameter("preview_dt", 0.5)
        self.declare_parameter("validate_once", True)
        self.declare_parameter("timer_period", 2.0)
        self.declare_parameter("display_topic", "/display_planned_path")
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
        self.object_length = float(self.get_parameter("object_length").value)
        self.object_width = float(self.get_parameter("object_width").value)
        self.object_height = float(self.get_parameter("object_height").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.max_joint_jump = float(self.get_parameter("max_joint_jump").value)
        self.preview_dt = float(self.get_parameter("preview_dt").value)
        self.validate_once = bool(self.get_parameter("validate_once").value)
        self.timer_period = max(0.2, float(self.get_parameter("timer_period").value))
        self.display_topic = self.get_parameter("display_topic").value
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

        from tf2_ros import Buffer, TransformBroadcaster, TransformListener

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/dual_object_final_demo/markers",
            10,
        )
        self.display_pub = self.create_publisher(
            DisplayTrajectory,
            self.display_topic,
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
        self.display_published_once = False

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Dual object final demo started. Visualization/validation only: "
            "no /joint_states publishing, no controller JointTrajectory "
            "publishing, no ExecuteTrajectory, no FollowJointTrajectory action, "
            "and no JAKA driver service calls."
        )

    def publish_final_target_tf(self):
        if not self.waypoints:
            return
        final = self.waypoints[-1]
        stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(
            [
                transform_to_stamped(
                    self.T_world_object_initial,
                    self.reference_frame,
                    "final_demo_start_object",
                    stamp,
                ),
                transform_to_stamped(
                    final["T_object"],
                    self.reference_frame,
                    "final_demo_target_object",
                    stamp,
                ),
                transform_to_stamped(
                    final["T_left"],
                    self.reference_frame,
                    "final_demo_target_left_grasp",
                    stamp,
                ),
                transform_to_stamped(
                    final["T_right"],
                    self.reference_frame,
                    "final_demo_target_right_grasp",
                    stamp,
                ),
            ]
        )

    def make_object_box_marker(self, stamp, marker_id, namespace, T, rgba):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = transform_to_pose(T)
        marker.scale.x = self.object_length
        marker.scale.y = self.object_width
        marker.scale.z = self.object_height
        marker.color.r = float(rgba[0])
        marker.color.g = float(rgba[1])
        marker.color.b = float(rgba[2])
        marker.color.a = float(rgba[3])
        return marker

    def make_text_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = stamp
        marker.ns = "final_demo_status_text"
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
            marker.pose.position.z = float(final_position[2] + 0.22)
        else:
            marker.pose.position.z = 0.5

        valid_count = sum(1 for waypoint in self.waypoints if waypoint["status"] == "VALID")
        first_invalid = next(
            (waypoint for waypoint in self.waypoints if waypoint["status"] == "INVALID"),
            None,
        )
        if first_invalid is None and self.preview_complete_once:
            status = "FINAL_DEMO_VALID"
            reason = "VALID"
        elif first_invalid is not None:
            status = "FINAL_DEMO_INVALID"
            reason = f"wp={first_invalid['index']} {first_invalid['reason']}"
        else:
            status = "FINAL_DEMO_PENDING"
            reason = "PENDING"
        marker.text = (
            f"{status}\nvalid={valid_count}/{len(self.waypoints)}\n"
            f"display={self.display_published_once}\n{reason}"
        )
        return marker

    def publish_markers(self):
        if not self.waypoints or self.T_world_object_initial is None:
            return
        if not self.markers_cleared:
            self.clear_markers()
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        final = self.waypoints[-1]

        markers.markers.append(
            self.make_object_box_marker(
                stamp,
                10,
                "object_start_box",
                self.T_world_object_initial,
                [0.75, 0.75, 0.75, 0.35],
            )
        )
        markers.markers.append(
            self.make_object_box_marker(
                stamp,
                11,
                "object_target_box",
                final["T_object"],
                [0.1, 0.8, 1.0, 0.50],
            )
        )

        object_points = [self.T_world_object_initial[:3, 3]] + [
            waypoint["T_object"][:3, 3] for waypoint in self.waypoints
        ]
        left_points = [self.T_world_left_current[:3, 3]] + [
            waypoint["T_left"][:3, 3] for waypoint in self.waypoints
        ]
        right_points = [self.T_world_right_current[:3, 3]] + [
            waypoint["T_right"][:3, 3] for waypoint in self.waypoints
        ]

        path_specs = [
            ("final_demo_object_path", object_points, [0.1, 0.8, 1.0, 0.95], 101, 0.012),
            ("final_demo_left_grasp_path", left_points, [1.0, 0.25, 0.15, 0.9], 102, 0.009),
            ("final_demo_right_grasp_path", right_points, [0.2, 0.45, 1.0, 0.9], 103, 0.009),
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

        first_invalid = self.first_invalid_waypoint_index()
        for waypoint in self.waypoints:
            index = waypoint["index"]
            color = self.marker_color_for_waypoint(waypoint)
            marker_scale = 0.032 if waypoint["status"] == "INVALID" else 0.020
            markers.markers.append(
                make_sphere_marker(
                    self.reference_frame,
                    stamp,
                    1000 + index,
                    "final_demo_object_waypoint",
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
                    "final_demo_grasp_pair_line",
                    waypoint["T_left"][:3, 3],
                    waypoint["T_right"][:3, 3],
                    0.006,
                    color,
                )
            )

        axis_specs = [
            ("start_object_axis", 3000, self.T_world_object_initial),
            ("target_object_axis", 3010, final["T_object"]),
            ("target_left_grasp_axis", 3020, final["T_left"]),
            ("target_right_grasp_axis", 3030, final["T_right"]),
        ]
        if first_invalid is not None:
            invalid = self.waypoints[first_invalid]
            axis_specs.append(
                (
                    f"first_invalid_object_axis_{first_invalid:02d}",
                    3040,
                    invalid["T_object"],
                )
            )
        for namespace, start_id, T in axis_specs:
            for marker in make_axis_markers(
                self.reference_frame,
                stamp,
                namespace,
                start_id,
                T,
                0.12,
                0.008,
            ):
                markers.markers.append(marker)

        markers.markers.append(self.make_text_marker(stamp))
        self.marker_pub.publish(markers)
        self.publish_final_target_tf()

    def finish_summary(self):
        self.preview_complete_once = True
        valid_count = sum(1 for waypoint in self.waypoints if waypoint["status"] == "VALID")
        first_invalid = next(
            (waypoint for waypoint in self.waypoints if waypoint["status"] != "VALID"),
            None,
        )

        if first_invalid is None and len(self.waypoints) > 0:
            self.publish_display_trajectory()
            final_status = "FINAL_DEMO_VALID"
            reason = "VALID"
        else:
            final_status = "FINAL_DEMO_INVALID"
            reason = first_invalid["reason"] if first_invalid is not None else "NO_WAYPOINTS"

        self.publish_markers()

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

        self.get_logger().info(
            f"{final_status} | total_waypoints={len(self.waypoints)} | "
            f"valid_waypoints={valid_count} | reason={reason} | "
            f"max_left_joint_jump={self.max_left_joint_jump_seen:.5f} | "
            f"max_right_joint_jump={self.max_right_joint_jump_seen:.5f} | "
            f"final_relative_distance_error={final_relative_distance_error:+.6f} m | "
            f"final_relative_orientation_error={final_relative_orientation_error:.5f} rad | "
            f"display_trajectory_published={self.display_published_once}"
        )
        self.pending_future = None
        self.pending_stage = None


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectFinalDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
