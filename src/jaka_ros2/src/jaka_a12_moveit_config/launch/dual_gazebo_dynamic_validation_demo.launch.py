from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    command_backend = LaunchConfiguration("command_backend")
    left_gazebo_joint_trajectory_topic = LaunchConfiguration(
        "left_gazebo_joint_trajectory_topic"
    )
    right_gazebo_joint_trajectory_topic = LaunchConfiguration(
        "right_gazebo_joint_trajectory_topic"
    )
    dual_gazebo_trajectory_time_from_start = LaunchConfiguration(
        "dual_gazebo_trajectory_time_from_start"
    )
    dual_gazebo_trajectory_publish_period = LaunchConfiguration(
        "dual_gazebo_trajectory_publish_period"
    )
    dual_gazebo_command_position_smoothing = LaunchConfiguration(
        "dual_gazebo_command_position_smoothing"
    )
    dual_gazebo_command_alpha = LaunchConfiguration(
        "dual_gazebo_command_alpha"
    )
    enable_gazebo_feedback_validation = LaunchConfiguration(
        "enable_gazebo_feedback_validation"
    )
    gazebo_feedback_debug_log = LaunchConfiguration("gazebo_feedback_debug_log")
    gazebo_feedback_log_period = LaunchConfiguration("gazebo_feedback_log_period")
    gazebo_validation_csv_enabled = LaunchConfiguration(
        "gazebo_validation_csv_enabled"
    )
    gazebo_validation_csv_path = LaunchConfiguration("gazebo_validation_csv_path")
    enable_dynamic_object_test = LaunchConfiguration("enable_dynamic_object_test")
    dynamic_object_test_mode = LaunchConfiguration("dynamic_object_test_mode")
    dynamic_object_axis = LaunchConfiguration("dynamic_object_axis")
    dynamic_object_amplitude = LaunchConfiguration("dynamic_object_amplitude")
    dynamic_object_period = LaunchConfiguration("dynamic_object_period")
    dynamic_object_phase = LaunchConfiguration("dynamic_object_phase")
    dynamic_object_start_delay = LaunchConfiguration("dynamic_object_start_delay")
    dynamic_object_max_speed = LaunchConfiguration("dynamic_object_max_speed")
    dynamic_object_enable_yaw = LaunchConfiguration("dynamic_object_enable_yaw")
    dynamic_object_yaw_amplitude_deg = LaunchConfiguration(
        "dynamic_object_yaw_amplitude_deg"
    )
    dynamic_object_yaw_period = LaunchConfiguration("dynamic_object_yaw_period")
    dynamic_object_hold_initial_position = LaunchConfiguration(
        "dynamic_object_hold_initial_position"
    )
    enable_interactive_object_marker = LaunchConfiguration(
        "enable_interactive_object_marker"
    )
    interactive_marker_scale = LaunchConfiguration("interactive_marker_scale")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    object_min_x = LaunchConfiguration("object_min_x")
    object_max_x = LaunchConfiguration("object_max_x")
    object_min_y = LaunchConfiguration("object_min_y")
    object_max_y = LaunchConfiguration("object_max_y")
    object_min_z = LaunchConfiguration("object_min_z")
    object_max_z = LaunchConfiguration("object_max_z")
    max_object_target_speed = LaunchConfiguration("max_object_target_speed")
    enable_collision_avoidance = LaunchConfiguration("enable_collision_avoidance")
    collision_warning_distance = LaunchConfiguration("collision_warning_distance")
    collision_danger_distance = LaunchConfiguration("collision_danger_distance")
    collision_stop_distance = LaunchConfiguration("collision_stop_distance")
    collision_link_radius = LaunchConfiguration("collision_link_radius")
    collision_scale_min = LaunchConfiguration("collision_scale_min")
    collision_reject_on_danger = LaunchConfiguration("collision_reject_on_danger")
    collision_debug_log = LaunchConfiguration("collision_debug_log")
    collision_ignore_static_pairs = LaunchConfiguration(
        "collision_ignore_static_pairs"
    )
    collision_ignore_proximal_pairs = LaunchConfiguration(
        "collision_ignore_proximal_pairs"
    )
    collision_proximal_ignore_keywords = LaunchConfiguration(
        "collision_proximal_ignore_keywords"
    )
    collision_use_distal_pairs_for_safety = LaunchConfiguration(
        "collision_use_distal_pairs_for_safety"
    )
    enable_self_collision_avoidance = LaunchConfiguration(
        "enable_self_collision_avoidance"
    )
    self_collision_warning_distance = LaunchConfiguration(
        "self_collision_warning_distance"
    )
    self_collision_danger_distance = LaunchConfiguration(
        "self_collision_danger_distance"
    )
    self_collision_stop_distance = LaunchConfiguration("self_collision_stop_distance")
    self_collision_link_radius = LaunchConfiguration("self_collision_link_radius")
    self_collision_scale_min = LaunchConfiguration("self_collision_scale_min")
    self_collision_reject_on_danger = LaunchConfiguration(
        "self_collision_reject_on_danger"
    )
    self_collision_min_index_gap = LaunchConfiguration(
        "self_collision_min_index_gap"
    )
    self_collision_debug_log = LaunchConfiguration("self_collision_debug_log")
    collision_hard_stop_enabled = LaunchConfiguration("collision_hard_stop_enabled")
    reset_qdot_smoothing_on_collision_stop = LaunchConfiguration(
        "reset_qdot_smoothing_on_collision_stop"
    )
    collision_recovery_enabled = LaunchConfiguration("collision_recovery_enabled")
    collision_recovery_scale = LaunchConfiguration("collision_recovery_scale")
    collision_recovery_max_qdot = LaunchConfiguration("collision_recovery_max_qdot")
    collision_recovery_min_improvement = LaunchConfiguration(
        "collision_recovery_min_improvement"
    )
    collision_recovery_debug_log = LaunchConfiguration(
        "collision_recovery_debug_log"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "command_backend",
                default_value="dual_gazebo_trajectory",
            ),
            DeclareLaunchArgument(
                "left_gazebo_joint_trajectory_topic",
                default_value="/left_jaka_a12_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "right_gazebo_joint_trajectory_topic",
                default_value="/right_jaka_a12_controller/joint_trajectory",
            ),
            DeclareLaunchArgument(
                "dual_gazebo_trajectory_time_from_start",
                default_value="0.50",
            ),
            DeclareLaunchArgument(
                "dual_gazebo_trajectory_publish_period",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "dual_gazebo_command_position_smoothing",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dual_gazebo_command_alpha",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "enable_gazebo_feedback_validation",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "gazebo_feedback_debug_log",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "gazebo_feedback_log_period",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "gazebo_validation_csv_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "gazebo_validation_csv_path",
                default_value="/tmp/dual_gazebo_dynamic_validation.csv",
            ),
            DeclareLaunchArgument(
                "enable_dynamic_object_test",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dynamic_object_test_mode",
                default_value="sinusoidal",
            ),
            DeclareLaunchArgument(
                "dynamic_object_axis",
                default_value="x",
            ),
            DeclareLaunchArgument(
                "dynamic_object_amplitude",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "dynamic_object_period",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "dynamic_object_phase",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "dynamic_object_start_delay",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "dynamic_object_max_speed",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "dynamic_object_enable_yaw",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "dynamic_object_yaw_amplitude_deg",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "dynamic_object_yaw_period",
                default_value="12.0",
            ),
            DeclareLaunchArgument(
                "dynamic_object_hold_initial_position",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_interactive_object_marker",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "interactive_marker_scale",
                default_value="0.20",
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/robot_description",
            ),
            DeclareLaunchArgument("object_min_x", default_value="-0.5"),
            DeclareLaunchArgument("object_max_x", default_value="2.0"),
            DeclareLaunchArgument("object_min_y", default_value="-1.0"),
            DeclareLaunchArgument("object_max_y", default_value="1.0"),
            DeclareLaunchArgument("object_min_z", default_value="0.05"),
            DeclareLaunchArgument("object_max_z", default_value="2.0"),
            DeclareLaunchArgument("max_object_target_speed", default_value="0.50"),
            DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
            DeclareLaunchArgument("collision_warning_distance", default_value="0.25"),
            DeclareLaunchArgument("collision_danger_distance", default_value="0.15"),
            DeclareLaunchArgument("collision_stop_distance", default_value="0.08"),
            DeclareLaunchArgument("collision_link_radius", default_value="0.04"),
            DeclareLaunchArgument("collision_scale_min", default_value="0.10"),
            DeclareLaunchArgument("collision_reject_on_danger", default_value="true"),
            DeclareLaunchArgument("collision_debug_log", default_value="true"),
            DeclareLaunchArgument(
                "collision_ignore_static_pairs",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "collision_ignore_proximal_pairs",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "collision_proximal_ignore_keywords",
                default_value="base_link,J1",
            ),
            DeclareLaunchArgument(
                "collision_use_distal_pairs_for_safety",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_self_collision_avoidance",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "self_collision_warning_distance",
                default_value="0.20",
            ),
            DeclareLaunchArgument(
                "self_collision_danger_distance",
                default_value="0.12",
            ),
            DeclareLaunchArgument(
                "self_collision_stop_distance",
                default_value="0.07",
            ),
            DeclareLaunchArgument(
                "self_collision_link_radius",
                default_value="0.035",
            ),
            DeclareLaunchArgument(
                "self_collision_scale_min",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "self_collision_reject_on_danger",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "self_collision_min_index_gap",
                default_value="2",
            ),
            DeclareLaunchArgument("self_collision_debug_log", default_value="true"),
            DeclareLaunchArgument(
                "collision_hard_stop_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "reset_qdot_smoothing_on_collision_stop",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "collision_recovery_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument("collision_recovery_scale", default_value="0.15"),
            DeclareLaunchArgument("collision_recovery_max_qdot", default_value="0.03"),
            DeclareLaunchArgument(
                "collision_recovery_min_improvement",
                default_value="0.002",
            ),
            DeclareLaunchArgument(
                "collision_recovery_debug_log",
                default_value="true",
            ),
            Node(
                package="jaka_coop_monitor",
                executable="dual_safety_posture_demo",
                name="dual_safety_posture_demo",
                output="screen",
                parameters=[
                    {
                        "robot_description_topic": robot_description_topic,
                        "command_backend": ParameterValue(
                            command_backend,
                            value_type=str,
                        ),
                        "left_gazebo_joint_trajectory_topic": ParameterValue(
                            left_gazebo_joint_trajectory_topic,
                            value_type=str,
                        ),
                        "right_gazebo_joint_trajectory_topic": ParameterValue(
                            right_gazebo_joint_trajectory_topic,
                            value_type=str,
                        ),
                        "dual_gazebo_trajectory_time_from_start": ParameterValue(
                            dual_gazebo_trajectory_time_from_start,
                            value_type=float,
                        ),
                        "dual_gazebo_trajectory_publish_period": ParameterValue(
                            dual_gazebo_trajectory_publish_period,
                            value_type=float,
                        ),
                        "dual_gazebo_command_position_smoothing": ParameterValue(
                            dual_gazebo_command_position_smoothing,
                            value_type=bool,
                        ),
                        "dual_gazebo_command_alpha": ParameterValue(
                            dual_gazebo_command_alpha,
                            value_type=float,
                        ),
                        "enable_gazebo_feedback_validation": ParameterValue(
                            enable_gazebo_feedback_validation,
                            value_type=bool,
                        ),
                        "gazebo_feedback_debug_log": ParameterValue(
                            gazebo_feedback_debug_log,
                            value_type=bool,
                        ),
                        "gazebo_feedback_log_period": ParameterValue(
                            gazebo_feedback_log_period,
                            value_type=float,
                        ),
                        "gazebo_validation_csv_enabled": ParameterValue(
                            gazebo_validation_csv_enabled,
                            value_type=bool,
                        ),
                        "gazebo_validation_csv_path": ParameterValue(
                            gazebo_validation_csv_path,
                            value_type=str,
                        ),
                        "enable_dynamic_object_test": ParameterValue(
                            enable_dynamic_object_test,
                            value_type=bool,
                        ),
                        "dynamic_object_test_mode": ParameterValue(
                            dynamic_object_test_mode,
                            value_type=str,
                        ),
                        "dynamic_object_axis": ParameterValue(
                            dynamic_object_axis,
                            value_type=str,
                        ),
                        "dynamic_object_amplitude": ParameterValue(
                            dynamic_object_amplitude,
                            value_type=float,
                        ),
                        "dynamic_object_period": ParameterValue(
                            dynamic_object_period,
                            value_type=float,
                        ),
                        "dynamic_object_phase": ParameterValue(
                            dynamic_object_phase,
                            value_type=float,
                        ),
                        "dynamic_object_start_delay": ParameterValue(
                            dynamic_object_start_delay,
                            value_type=float,
                        ),
                        "dynamic_object_max_speed": ParameterValue(
                            dynamic_object_max_speed,
                            value_type=float,
                        ),
                        "dynamic_object_enable_yaw": ParameterValue(
                            dynamic_object_enable_yaw,
                            value_type=bool,
                        ),
                        "dynamic_object_yaw_amplitude_deg": ParameterValue(
                            dynamic_object_yaw_amplitude_deg,
                            value_type=float,
                        ),
                        "dynamic_object_yaw_period": ParameterValue(
                            dynamic_object_yaw_period,
                            value_type=float,
                        ),
                        "dynamic_object_hold_initial_position": ParameterValue(
                            dynamic_object_hold_initial_position,
                            value_type=bool,
                        ),
                        "enable_interactive_object_marker": ParameterValue(
                            enable_interactive_object_marker,
                            value_type=bool,
                        ),
                        "interactive_marker_scale": ParameterValue(
                            interactive_marker_scale,
                            value_type=float,
                        ),
                        "object_min_x": ParameterValue(
                            object_min_x,
                            value_type=float,
                        ),
                        "object_max_x": ParameterValue(
                            object_max_x,
                            value_type=float,
                        ),
                        "object_min_y": ParameterValue(
                            object_min_y,
                            value_type=float,
                        ),
                        "object_max_y": ParameterValue(
                            object_max_y,
                            value_type=float,
                        ),
                        "object_min_z": ParameterValue(
                            object_min_z,
                            value_type=float,
                        ),
                        "object_max_z": ParameterValue(
                            object_max_z,
                            value_type=float,
                        ),
                        "max_object_target_speed": ParameterValue(
                            max_object_target_speed,
                            value_type=float,
                        ),
                        "enable_collision_avoidance": ParameterValue(
                            enable_collision_avoidance,
                            value_type=bool,
                        ),
                        "collision_warning_distance": ParameterValue(
                            collision_warning_distance,
                            value_type=float,
                        ),
                        "collision_danger_distance": ParameterValue(
                            collision_danger_distance,
                            value_type=float,
                        ),
                        "collision_stop_distance": ParameterValue(
                            collision_stop_distance,
                            value_type=float,
                        ),
                        "collision_link_radius": ParameterValue(
                            collision_link_radius,
                            value_type=float,
                        ),
                        "collision_scale_min": ParameterValue(
                            collision_scale_min,
                            value_type=float,
                        ),
                        "collision_reject_on_danger": ParameterValue(
                            collision_reject_on_danger,
                            value_type=bool,
                        ),
                        "collision_debug_log": ParameterValue(
                            collision_debug_log,
                            value_type=bool,
                        ),
                        "collision_ignore_static_pairs": ParameterValue(
                            collision_ignore_static_pairs,
                            value_type=bool,
                        ),
                        "collision_ignore_proximal_pairs": ParameterValue(
                            collision_ignore_proximal_pairs,
                            value_type=bool,
                        ),
                        "collision_proximal_ignore_keywords": ParameterValue(
                            collision_proximal_ignore_keywords,
                            value_type=str,
                        ),
                        "collision_use_distal_pairs_for_safety": ParameterValue(
                            collision_use_distal_pairs_for_safety,
                            value_type=bool,
                        ),
                        "enable_self_collision_avoidance": ParameterValue(
                            enable_self_collision_avoidance,
                            value_type=bool,
                        ),
                        "self_collision_warning_distance": ParameterValue(
                            self_collision_warning_distance,
                            value_type=float,
                        ),
                        "self_collision_danger_distance": ParameterValue(
                            self_collision_danger_distance,
                            value_type=float,
                        ),
                        "self_collision_stop_distance": ParameterValue(
                            self_collision_stop_distance,
                            value_type=float,
                        ),
                        "self_collision_link_radius": ParameterValue(
                            self_collision_link_radius,
                            value_type=float,
                        ),
                        "self_collision_scale_min": ParameterValue(
                            self_collision_scale_min,
                            value_type=float,
                        ),
                        "self_collision_reject_on_danger": ParameterValue(
                            self_collision_reject_on_danger,
                            value_type=bool,
                        ),
                        "self_collision_min_index_gap": ParameterValue(
                            self_collision_min_index_gap,
                            value_type=int,
                        ),
                        "self_collision_debug_log": ParameterValue(
                            self_collision_debug_log,
                            value_type=bool,
                        ),
                        "collision_hard_stop_enabled": ParameterValue(
                            collision_hard_stop_enabled,
                            value_type=bool,
                        ),
                        "reset_qdot_smoothing_on_collision_stop": ParameterValue(
                            reset_qdot_smoothing_on_collision_stop,
                            value_type=bool,
                        ),
                        "collision_recovery_enabled": ParameterValue(
                            collision_recovery_enabled,
                            value_type=bool,
                        ),
                        "collision_recovery_scale": ParameterValue(
                            collision_recovery_scale,
                            value_type=float,
                        ),
                        "collision_recovery_max_qdot": ParameterValue(
                            collision_recovery_max_qdot,
                            value_type=float,
                        ),
                        "collision_recovery_min_improvement": ParameterValue(
                            collision_recovery_min_improvement,
                            value_type=float,
                        ),
                        "collision_recovery_debug_log": ParameterValue(
                            collision_recovery_debug_log,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
