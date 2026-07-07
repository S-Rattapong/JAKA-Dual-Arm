import copy
import csv
import os
from datetime import datetime

import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from jaka_a12_dual_planner.moveit_validation import MoveItValidationClient
from jaka_a12_dual_planner.object_frame import transform_from_translation_rpy
from jaka_a12_dual_planner.trajectory_display_node import (
    DualObjectTrajectoryDisplayNode,
)
from jaka_a12_dual_planner.trajectory_preview_node import (
    LEFT_JOINT_NAMES,
    RIGHT_JOINT_NAMES,
    _joint_positions,
    _list_param,
)


DEFAULT_CASES = [
    {
        "case_name": "dz_005",
        "target_dx": 0.0,
        "target_dy": 0.0,
        "target_dz": 0.05,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.0,
    },
    {
        "case_name": "dx_005",
        "target_dx": 0.05,
        "target_dy": 0.0,
        "target_dz": 0.0,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.0,
    },
    {
        "case_name": "dy_005",
        "target_dx": 0.0,
        "target_dy": 0.05,
        "target_dz": 0.0,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.0,
    },
    {
        "case_name": "yaw_010",
        "target_dx": 0.0,
        "target_dy": 0.0,
        "target_dz": 0.0,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.10,
    },
    {
        "case_name": "yaw_020",
        "target_dx": 0.0,
        "target_dy": 0.0,
        "target_dz": 0.0,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.20,
    },
    {
        "case_name": "dx_dz_005",
        "target_dx": 0.05,
        "target_dy": 0.0,
        "target_dz": 0.05,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.0,
    },
    {
        "case_name": "stress_dx_030",
        "target_dx": 0.30,
        "target_dy": 0.0,
        "target_dz": 0.0,
        "target_roll": 0.0,
        "target_pitch": 0.0,
        "target_yaw": 0.0,
    },
]


CSV_COLUMNS = [
    "timestamp",
    "case_name",
    "target_dx",
    "target_dy",
    "target_dz",
    "target_roll",
    "target_pitch",
    "target_yaw",
    "num_waypoints",
    "trajectory_valid",
    "valid_waypoints",
    "total_waypoints",
    "first_invalid_waypoint",
    "reason",
    "max_left_joint_jump",
    "max_right_joint_jump",
    "final_relative_distance_error_m",
    "final_relative_orientation_error_rad",
    "final_relative_orientation_error_deg",
    "state_validity_available",
    "state_validity_failed_count",
    "left_ik_failed_count",
    "right_ik_failed_count",
    "workspace_failed_count",
    "relative_constraint_failed_count",
    "joint_jump_failed_count",
]


class DualObjectExperimentRunnerNode(DualObjectTrajectoryDisplayNode):
    def __init__(self):
        Node.__init__(self, "dual_object_experiment_runner_node")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("left_tcp_frame", "left_J6")
        self.declare_parameter("right_tcp_frame", "right_J6")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("num_waypoints", 10)
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("max_joint_jump", 0.35)
        self.declare_parameter("preview_dt", 0.5)
        self.declare_parameter("timer_period", 2.0)
        self.declare_parameter("output_dir", "~/jaka_ws/logs")
        self.declare_parameter("output_prefix", "dual_object_experiment")
        self.declare_parameter("run_once", True)
        self.declare_parameter("publish_last_case_display", True)
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
        self.timer_period = max(0.2, float(self.get_parameter("timer_period").value))
        self.output_dir = os.path.expanduser(
            self.get_parameter("output_dir").value
        )
        self.output_prefix = self.get_parameter("output_prefix").value
        self.run_once = bool(self.get_parameter("run_once").value)
        self.publish_last_case_display = bool(
            self.get_parameter("publish_last_case_display").value
        )
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
            "/dual_object_experiment_runner/markers",
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

        self.cases = [dict(item) for item in DEFAULT_CASES]
        self.case_index = -1
        self.current_case = None
        self.results = []
        self.completed = False
        self.last_valid_case_name = None
        self.last_valid_waypoints = None
        self.csv_path = None

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Dual object experiment runner started. Logging/validation/"
            "visualization only: no /joint_states publishing, no controller "
            "JointTrajectory publishing, no ExecuteTrajectory, no "
            "FollowJointTrajectory action, and no JAKA driver service calls."
        )

    def target_delta_transform(self):
        case = self.current_case or DEFAULT_CASES[0]
        return transform_from_translation_rpy(
            [
                case["target_dx"],
                case["target_dy"],
                case["target_dz"],
            ],
            case["target_roll"],
            case["target_pitch"],
            case["target_yaw"],
        )

    def begin_next_case(self):
        self.case_index += 1
        if self.case_index >= len(self.cases):
            self.finish_experiments()
            return

        try:
            self.T_world_left_current, self.T_world_right_current = (
                self.lookup_tcp_transforms()
            )
        except Exception as ex:
            self.case_index -= 1
            self.get_logger().warn(f"Waiting for TCP TF frames: {ex}")
            return

        if not self.initialized:
            self.initialize_from_current_tf(
                self.T_world_left_current,
                self.T_world_right_current,
            )

        if not self.moveit.wait_for_ik_once(timeout_sec=0.0):
            self.case_index -= 1
            self.get_logger().warn("Experiment runner waiting for /compute_ik.")
            return

        if not self.moveit.state_validity_ready() and not self.state_validity_skipped_logged:
            self.state_validity_skipped_logged = True
            self.get_logger().warn(
                "/check_state_validity is not available; state/collision "
                "validity will be skipped."
            )

        self.current_case = self.cases[self.case_index]
        self.clear_markers()
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
            f"Starting experiment case {self.current_case['case_name']} | "
            f"dx={self.current_case['target_dx']:+.3f} | "
            f"dy={self.current_case['target_dy']:+.3f} | "
            f"dz={self.current_case['target_dz']:+.3f} | "
            f"yaw={self.current_case['target_yaw']:+.3f}"
        )
        self.advance_to_next_waypoint()

    def summarize_current_case(self):
        valid_count = sum(1 for waypoint in self.waypoints if waypoint["status"] == "VALID")
        total_waypoints = len(self.waypoints)
        first_invalid = next(
            (waypoint for waypoint in self.waypoints if waypoint["status"] != "VALID"),
            None,
        )
        trajectory_valid = first_invalid is None and total_waypoints > 0
        first_invalid_waypoint = (
            "none" if first_invalid is None else str(first_invalid["index"])
        )
        reason = "VALID" if first_invalid is None else first_invalid["reason"]
        final_validation = self.waypoints[-1]["validation"] if self.waypoints else None

        counts = {
            "state_validity_failed_count": 0,
            "left_ik_failed_count": 0,
            "right_ik_failed_count": 0,
            "workspace_failed_count": 0,
            "relative_constraint_failed_count": 0,
            "joint_jump_failed_count": 0,
        }
        reason_to_count = {
            "STATE_VALIDITY_FAILED": "state_validity_failed_count",
            "LEFT_IK_FAILED": "left_ik_failed_count",
            "RIGHT_IK_FAILED": "right_ik_failed_count",
            "WORKSPACE_FAILED": "workspace_failed_count",
            "RELATIVE_CONSTRAINT_FAILED": "relative_constraint_failed_count",
            "JOINT_JUMP_FAILED": "joint_jump_failed_count",
        }
        for waypoint in self.waypoints:
            count_name = reason_to_count.get(waypoint["reason"])
            if count_name is not None:
                counts[count_name] += 1

        result = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "case_name": self.current_case["case_name"],
            "target_dx": self.current_case["target_dx"],
            "target_dy": self.current_case["target_dy"],
            "target_dz": self.current_case["target_dz"],
            "target_roll": self.current_case["target_roll"],
            "target_pitch": self.current_case["target_pitch"],
            "target_yaw": self.current_case["target_yaw"],
            "num_waypoints": self.num_waypoints,
            "trajectory_valid": trajectory_valid,
            "valid_waypoints": valid_count,
            "total_waypoints": total_waypoints,
            "first_invalid_waypoint": first_invalid_waypoint,
            "reason": reason,
            "max_left_joint_jump": self.max_left_joint_jump_seen,
            "max_right_joint_jump": self.max_right_joint_jump_seen,
            "final_relative_distance_error_m": (
                final_validation["relative_distance_error"]
                if final_validation is not None
                else float("nan")
            ),
            "final_relative_orientation_error_rad": (
                final_validation["relative_orientation_error_rad"]
                if final_validation is not None
                else float("nan")
            ),
            "final_relative_orientation_error_deg": (
                final_validation["relative_orientation_error_deg"]
                if final_validation is not None
                else float("nan")
            ),
            "state_validity_available": self.moveit.state_validity_ready(),
            **counts,
        }
        return result

    def finish_summary(self):
        self.preview_complete_once = True
        result = self.summarize_current_case()
        self.results.append(result)

        if result["trajectory_valid"]:
            self.last_valid_case_name = result["case_name"]
            self.last_valid_waypoints = copy.deepcopy(self.waypoints)

        self.publish_markers()
        self.get_logger().info(
            "EXPERIMENT_CASE_RESULT | "
            f"case={result['case_name']} | "
            f"valid={result['trajectory_valid']} | "
            f"valid_waypoints={result['valid_waypoints']}/"
            f"{result['total_waypoints']} | "
            f"first_invalid={result['first_invalid_waypoint']} | "
            f"reason={result['reason']} | "
            f"max_left_joint_jump={result['max_left_joint_jump']:.5f} | "
            f"max_right_joint_jump={result['max_right_joint_jump']:.5f} | "
            f"final_rel_dist_err="
            f"{result['final_relative_distance_error_m']:+.6f} | "
            f"final_rel_ori_err="
            f"{result['final_relative_orientation_error_rad']:.5f}"
        )

        self.pending_future = None
        self.pending_stage = None
        self.current_case = None
        self.waypoints = []

    def write_csv(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(
            self.output_dir,
            f"{self.output_prefix}_{timestamp}.csv",
        )
        with open(self.csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for result in self.results:
                writer.writerow(result)
        return self.csv_path

    def finish_experiments(self):
        if self.completed:
            return
        self.completed = True
        csv_path = self.write_csv()
        valid_cases = sum(1 for result in self.results if result["trajectory_valid"])

        if self.publish_last_case_display and self.last_valid_waypoints is not None:
            self.waypoints = copy.deepcopy(self.last_valid_waypoints)
            self.publish_display_trajectory()
            self.publish_markers()
            self.get_logger().info(
                "Published DisplayTrajectory for last valid experiment case: "
                f"{self.last_valid_case_name} -> {self.display_topic}"
            )

        self.get_logger().info(
            f"EXPERIMENT_SUMMARY | total_cases={len(self.results)} | "
            f"valid_cases={valid_cases} | csv_path={csv_path}"
        )

    def timer_callback(self):
        if self.pending_future is not None:
            self.poll_pending_preview()
            return
        if self.completed:
            return
        if self.current_case is not None:
            return
        self.begin_next_case()


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectExperimentRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
