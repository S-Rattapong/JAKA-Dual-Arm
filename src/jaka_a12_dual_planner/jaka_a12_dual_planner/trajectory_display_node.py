import rclpy
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from jaka_a12_dual_planner.moveit_validation import (
    MoveItValidationClient,
    duration_msg,
    merge_joint_states,
)
from jaka_a12_dual_planner.trajectory_preview_node import (
    LEFT_JOINT_NAMES,
    RIGHT_JOINT_NAMES,
    DualObjectTrajectoryPreviewNode,
    _list_param,
)


DUAL_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES


class DualObjectTrajectoryDisplayNode(DualObjectTrajectoryPreviewNode):
    def __init__(self):
        Node.__init__(self, "dual_object_trajectory_display_node")

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
            "/dual_object_trajectory_display/markers",
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
            "Dual object trajectory display started. Visualization only: no "
            "/joint_states publishing, no controller JointTrajectory publishing, "
            "no ExecuteTrajectory, no FollowJointTrajectory action, and no JAKA "
            "driver service calls."
        )

    def make_text_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.reference_frame
        marker.header.stamp = stamp
        marker.ns = "display_status_text"
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
        if first_invalid is None and self.display_published_once:
            status = "DISPLAY_TRAJECTORY_PUBLISHED"
            reason = self.display_topic
        elif first_invalid is not None:
            status = "DISPLAY_TRAJECTORY_NOT_PUBLISHED"
            reason = f"wp={first_invalid['index']} {first_invalid['reason']}"
        else:
            status = "DISPLAY_TRAJECTORY_PENDING"
            reason = "PENDING"
        marker.text = (
            f"{status}\nvalid={valid_count}/{len(self.waypoints)}\n"
            f"{reason}"
        )
        return marker

    def build_display_trajectory(self):
        display = DisplayTrajectory()
        display.model_id = "dual_jaka_a12"
        display.trajectory_start = RobotState()
        display.trajectory_start.joint_state = merge_joint_states(self.latest_joint_state)

        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory.header.frame_id = self.reference_frame
        robot_trajectory.joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        robot_trajectory.joint_trajectory.joint_names = DUAL_JOINT_NAMES

        for index, waypoint in enumerate(self.waypoints):
            point = JointTrajectoryPoint()
            left = waypoint["left_joint_solution"]
            right = waypoint["right_joint_solution"]
            point.positions = [float(value) for value in left] + [
                float(value) for value in right
            ]
            point.time_from_start = duration_msg((index + 1) * self.preview_dt)
            robot_trajectory.joint_trajectory.points.append(point)

        display.trajectory.append(robot_trajectory)
        return display

    def publish_display_trajectory(self):
        display = self.build_display_trajectory()
        self.display_pub.publish(display)
        self.display_published_once = True

    def finish_summary(self):
        self.preview_complete_once = True
        valid_count = sum(1 for waypoint in self.waypoints if waypoint["status"] == "VALID")
        first_invalid = next(
            (waypoint for waypoint in self.waypoints if waypoint["status"] != "VALID"),
            None,
        )

        if first_invalid is None and len(self.waypoints) > 0:
            self.publish_display_trajectory()
            final_status = "DISPLAY_TRAJECTORY_PUBLISHED"
            first_invalid_index = "none"
            reason = "VALID"
        else:
            final_status = "DISPLAY_TRAJECTORY_NOT_PUBLISHED"
            first_invalid_index = (
                str(first_invalid["index"]) if first_invalid is not None else "none"
            )
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

        for waypoint in self.waypoints:
            validation = waypoint["validation"]
            self.get_logger().info(
                f"Display waypoint {waypoint['index']:02d} | "
                f"status={waypoint['status']} | reason={waypoint['reason']} | "
                f"left_jump={waypoint['left_joint_jump']:.5f} | "
                f"right_jump={waypoint['right_joint_jump']:.5f} | "
                f"state_validity={self.result_text(waypoint['state_validity'])} | "
                f"rel_dist_err={validation['relative_distance_error']:+.6f} m | "
                f"rel_ori_err={validation['relative_orientation_error_rad']:.5f} rad"
            )

        self.get_logger().info(
            f"{final_status} | total_waypoints={len(self.waypoints)} | "
            f"valid_waypoints={valid_count} | display_topic={self.display_topic} | "
            f"first_invalid_waypoint={first_invalid_index} | reason={reason} | "
            f"max_left_joint_jump={self.max_left_joint_jump_seen:.5f} | "
            f"max_right_joint_jump={self.max_right_joint_jump_seen:.5f} | "
            f"final_relative_distance_error={final_relative_distance_error:+.6f} m | "
            f"final_relative_orientation_error={final_relative_orientation_error:.5f} rad"
        )
        self.pending_future = None
        self.pending_stage = None


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectTrajectoryDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
