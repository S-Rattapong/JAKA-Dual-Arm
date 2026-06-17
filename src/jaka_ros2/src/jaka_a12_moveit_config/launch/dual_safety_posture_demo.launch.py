from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    object_max_z = LaunchConfiguration("object_max_z")
    enable_joint_limit_avoidance = LaunchConfiguration("enable_joint_limit_avoidance")
    enable_joint_limit_test_pose = LaunchConfiguration("enable_joint_limit_test_pose")
    joint_limit_warning_margin = LaunchConfiguration("joint_limit_warning_margin")
    joint_limit_danger_margin = LaunchConfiguration("joint_limit_danger_margin")
    joint_limit_avoidance_gain = LaunchConfiguration("joint_limit_avoidance_gain")
    max_avoidance_velocity = LaunchConfiguration("max_avoidance_velocity")
    joint_limit_hard_stop_margin = LaunchConfiguration("joint_limit_hard_stop_margin")
    debug_joint_limit_raw_avoidance = LaunchConfiguration(
        "debug_joint_limit_raw_avoidance"
    )
    enable_collision_avoidance = LaunchConfiguration("enable_collision_avoidance")
    collision_warning_distance = LaunchConfiguration("collision_warning_distance")
    collision_danger_distance = LaunchConfiguration("collision_danger_distance")
    collision_stop_distance = LaunchConfiguration("collision_stop_distance")
    collision_link_radius = LaunchConfiguration("collision_link_radius")
    collision_scale_min = LaunchConfiguration("collision_scale_min")
    collision_reject_on_danger = LaunchConfiguration("collision_reject_on_danger")
    collision_debug_log = LaunchConfiguration("collision_debug_log")
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
    collision_hard_stop_enabled = LaunchConfiguration(
        "collision_hard_stop_enabled"
    )
    reset_qdot_smoothing_on_collision_stop = LaunchConfiguration(
        "reset_qdot_smoothing_on_collision_stop"
    )
    collision_recovery_enabled = LaunchConfiguration("collision_recovery_enabled")
    collision_recovery_scale = LaunchConfiguration("collision_recovery_scale")
    collision_recovery_max_qdot = LaunchConfiguration(
        "collision_recovery_max_qdot"
    )
    collision_recovery_min_improvement = LaunchConfiguration(
        "collision_recovery_min_improvement"
    )
    collision_recovery_debug_log = LaunchConfiguration(
        "collision_recovery_debug_log"
    )
    enable_object_yaw = LaunchConfiguration("enable_object_yaw")
    object_yaw_deg = LaunchConfiguration("object_yaw_deg")
    enable_object_yaw_sine = LaunchConfiguration("enable_object_yaw_sine")
    object_yaw_amplitude_deg = LaunchConfiguration("object_yaw_amplitude_deg")
    object_yaw_frequency_hz = LaunchConfiguration("object_yaw_frequency_hz")
    object_yaw_debug_log = LaunchConfiguration("object_yaw_debug_log")
    enable_interactive_object_yaw = LaunchConfiguration("enable_interactive_object_yaw")
    interactive_yaw_debug_log = LaunchConfiguration("interactive_yaw_debug_log")
    max_object_yaw_rate_deg_s = LaunchConfiguration("max_object_yaw_rate_deg_s")
    enable_object_yaw_rate_limit = LaunchConfiguration("enable_object_yaw_rate_limit")
    enable_ee_yaw_orientation_control = LaunchConfiguration(
        "enable_ee_yaw_orientation_control"
    )
    ee_yaw_orientation_gain = LaunchConfiguration("ee_yaw_orientation_gain")
    max_ee_yaw_angular_speed = LaunchConfiguration("max_ee_yaw_angular_speed")
    ee_yaw_task_damping = LaunchConfiguration("ee_yaw_task_damping")
    ee_yaw_debug_log = LaunchConfiguration("ee_yaw_debug_log")
    enable_full_ee_orientation_control = LaunchConfiguration(
        "enable_full_ee_orientation_control"
    )
    full_ee_orientation_gain = LaunchConfiguration("full_ee_orientation_gain")
    max_ee_angular_speed = LaunchConfiguration("max_ee_angular_speed")
    full_ee_orientation_task_damping = LaunchConfiguration(
        "full_ee_orientation_task_damping"
    )
    full_ee_orientation_debug_log = LaunchConfiguration(
        "full_ee_orientation_debug_log"
    )
    prefer_full_orientation_over_yaw_only = LaunchConfiguration(
        "prefer_full_orientation_over_yaw_only"
    )

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("jaka_a12_moveit_config"), "config", "dual_jaka_a12.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("jaka_a12_moveit_config"), "config", "dual_interactive_object_demo.rviz"]
    )

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("object_max_z", default_value="2.0"),
            DeclareLaunchArgument("enable_joint_limit_avoidance", default_value="true"),
            DeclareLaunchArgument("enable_joint_limit_test_pose", default_value="false"),
            DeclareLaunchArgument("joint_limit_warning_margin", default_value="0.30"),
            DeclareLaunchArgument("joint_limit_danger_margin", default_value="0.12"),
            DeclareLaunchArgument("joint_limit_avoidance_gain", default_value="0.08"),
            DeclareLaunchArgument("max_avoidance_velocity", default_value="0.05"),
            DeclareLaunchArgument("joint_limit_hard_stop_margin", default_value="0.05"),
            DeclareLaunchArgument(
                "debug_joint_limit_raw_avoidance",
                default_value="false",
            ),
            DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
            DeclareLaunchArgument("collision_warning_distance", default_value="0.25"),
            DeclareLaunchArgument("collision_danger_distance", default_value="0.15"),
            DeclareLaunchArgument("collision_stop_distance", default_value="0.08"),
            DeclareLaunchArgument("collision_link_radius", default_value="0.04"),
            DeclareLaunchArgument("collision_scale_min", default_value="0.10"),
            DeclareLaunchArgument("collision_reject_on_danger", default_value="true"),
            DeclareLaunchArgument("collision_debug_log", default_value="true"),
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
            DeclareLaunchArgument("self_collision_link_radius", default_value="0.035"),
            DeclareLaunchArgument("self_collision_scale_min", default_value="0.10"),
            DeclareLaunchArgument(
                "self_collision_reject_on_danger",
                default_value="true",
            ),
            DeclareLaunchArgument("self_collision_min_index_gap", default_value="2"),
            DeclareLaunchArgument("self_collision_debug_log", default_value="true"),
            DeclareLaunchArgument("collision_hard_stop_enabled", default_value="true"),
            DeclareLaunchArgument(
                "reset_qdot_smoothing_on_collision_stop",
                default_value="true",
            ),
            DeclareLaunchArgument("collision_recovery_enabled", default_value="true"),
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
            DeclareLaunchArgument("enable_object_yaw", default_value="false"),
            DeclareLaunchArgument("object_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("enable_object_yaw_sine", default_value="false"),
            DeclareLaunchArgument("object_yaw_amplitude_deg", default_value="20.0"),
            DeclareLaunchArgument("object_yaw_frequency_hz", default_value="0.05"),
            DeclareLaunchArgument("object_yaw_debug_log", default_value="true"),
            DeclareLaunchArgument(
                "enable_interactive_object_yaw",
                default_value="true",
            ),
            DeclareLaunchArgument("interactive_yaw_debug_log", default_value="true"),
            DeclareLaunchArgument("max_object_yaw_rate_deg_s", default_value="30.0"),
            DeclareLaunchArgument("enable_object_yaw_rate_limit", default_value="true"),
            DeclareLaunchArgument(
                "enable_ee_yaw_orientation_control",
                default_value="false",
            ),
            DeclareLaunchArgument("ee_yaw_orientation_gain", default_value="0.6"),
            DeclareLaunchArgument("max_ee_yaw_angular_speed", default_value="0.25"),
            DeclareLaunchArgument("ee_yaw_task_damping", default_value="0.05"),
            DeclareLaunchArgument("ee_yaw_debug_log", default_value="true"),
            DeclareLaunchArgument(
                "enable_full_ee_orientation_control",
                default_value="false",
            ),
            DeclareLaunchArgument("full_ee_orientation_gain", default_value="0.35"),
            DeclareLaunchArgument("max_ee_angular_speed", default_value="0.15"),
            DeclareLaunchArgument(
                "full_ee_orientation_task_damping",
                default_value="0.08",
            ),
            DeclareLaunchArgument(
                "full_ee_orientation_debug_log",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "prefer_full_orientation_over_yaw_only",
                default_value="true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="jaka_coop_monitor",
                executable="dual_safety_posture_demo",
                name="dual_safety_posture_demo",
                output="screen",
                parameters=[
                    {
                        "robot_description_topic": "/robot_description",
                        "joint_states_topic": "/joint_states",
                        "object_max_z": ParameterValue(
                            object_max_z,
                            value_type=float,
                        ),
                        "enable_joint_limit_avoidance": ParameterValue(
                            enable_joint_limit_avoidance,
                            value_type=bool,
                        ),
                        "enable_joint_limit_test_pose": ParameterValue(
                            enable_joint_limit_test_pose,
                            value_type=bool,
                        ),
                        "joint_limit_warning_margin": ParameterValue(
                            joint_limit_warning_margin,
                            value_type=float,
                        ),
                        "joint_limit_danger_margin": ParameterValue(
                            joint_limit_danger_margin,
                            value_type=float,
                        ),
                        "joint_limit_avoidance_gain": ParameterValue(
                            joint_limit_avoidance_gain,
                            value_type=float,
                        ),
                        "max_avoidance_velocity": ParameterValue(
                            max_avoidance_velocity,
                            value_type=float,
                        ),
                        "joint_limit_hard_stop_margin": ParameterValue(
                            joint_limit_hard_stop_margin,
                            value_type=float,
                        ),
                        "debug_joint_limit_raw_avoidance": ParameterValue(
                            debug_joint_limit_raw_avoidance,
                            value_type=bool,
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
                        "enable_object_yaw": ParameterValue(
                            enable_object_yaw,
                            value_type=bool,
                        ),
                        "object_yaw_deg": ParameterValue(
                            object_yaw_deg,
                            value_type=float,
                        ),
                        "enable_object_yaw_sine": ParameterValue(
                            enable_object_yaw_sine,
                            value_type=bool,
                        ),
                        "object_yaw_amplitude_deg": ParameterValue(
                            object_yaw_amplitude_deg,
                            value_type=float,
                        ),
                        "object_yaw_frequency_hz": ParameterValue(
                            object_yaw_frequency_hz,
                            value_type=float,
                        ),
                        "object_yaw_debug_log": ParameterValue(
                            object_yaw_debug_log,
                            value_type=bool,
                        ),
                        "enable_interactive_object_yaw": ParameterValue(
                            enable_interactive_object_yaw,
                            value_type=bool,
                        ),
                        "interactive_yaw_debug_log": ParameterValue(
                            interactive_yaw_debug_log,
                            value_type=bool,
                        ),
                        "max_object_yaw_rate_deg_s": ParameterValue(
                            max_object_yaw_rate_deg_s,
                            value_type=float,
                        ),
                        "enable_object_yaw_rate_limit": ParameterValue(
                            enable_object_yaw_rate_limit,
                            value_type=bool,
                        ),
                        "enable_ee_yaw_orientation_control": ParameterValue(
                            enable_ee_yaw_orientation_control,
                            value_type=bool,
                        ),
                        "ee_yaw_orientation_gain": ParameterValue(
                            ee_yaw_orientation_gain,
                            value_type=float,
                        ),
                        "max_ee_yaw_angular_speed": ParameterValue(
                            max_ee_yaw_angular_speed,
                            value_type=float,
                        ),
                        "ee_yaw_task_damping": ParameterValue(
                            ee_yaw_task_damping,
                            value_type=float,
                        ),
                        "ee_yaw_debug_log": ParameterValue(
                            ee_yaw_debug_log,
                            value_type=bool,
                        ),
                        "enable_full_ee_orientation_control": ParameterValue(
                            enable_full_ee_orientation_control,
                            value_type=bool,
                        ),
                        "full_ee_orientation_gain": ParameterValue(
                            full_ee_orientation_gain,
                            value_type=float,
                        ),
                        "max_ee_angular_speed": ParameterValue(
                            max_ee_angular_speed,
                            value_type=float,
                        ),
                        "full_ee_orientation_task_damping": ParameterValue(
                            full_ee_orientation_task_damping,
                            value_type=float,
                        ),
                        "full_ee_orientation_debug_log": ParameterValue(
                            full_ee_orientation_debug_log,
                            value_type=bool,
                        ),
                        "prefer_full_orientation_over_yaw_only": ParameterValue(
                            prefer_full_orientation_over_yaw_only,
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
