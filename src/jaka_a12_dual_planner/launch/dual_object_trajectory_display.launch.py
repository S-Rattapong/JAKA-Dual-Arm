from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_dx = LaunchConfiguration("target_dx")
    target_dy = LaunchConfiguration("target_dy")
    target_dz = LaunchConfiguration("target_dz")
    target_roll = LaunchConfiguration("target_roll")
    target_pitch = LaunchConfiguration("target_pitch")
    target_yaw = LaunchConfiguration("target_yaw")
    num_waypoints = LaunchConfiguration("num_waypoints")
    ik_timeout = LaunchConfiguration("ik_timeout")
    max_joint_jump = LaunchConfiguration("max_joint_jump")
    preview_dt = LaunchConfiguration("preview_dt")
    validate_once = LaunchConfiguration("validate_once")
    timer_period = LaunchConfiguration("timer_period")
    display_topic = LaunchConfiguration("display_topic")
    max_relative_distance_error = LaunchConfiguration("max_relative_distance_error")
    max_relative_orientation_error_rad = LaunchConfiguration(
        "max_relative_orientation_error_rad"
    )
    max_target_displacement = LaunchConfiguration("max_target_displacement")

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_dx", default_value="0.0"),
            DeclareLaunchArgument("target_dy", default_value="0.0"),
            DeclareLaunchArgument("target_dz", default_value="0.05"),
            DeclareLaunchArgument("target_roll", default_value="0.0"),
            DeclareLaunchArgument("target_pitch", default_value="0.0"),
            DeclareLaunchArgument("target_yaw", default_value="0.0"),
            DeclareLaunchArgument("num_waypoints", default_value="10"),
            DeclareLaunchArgument("ik_timeout", default_value="0.5"),
            DeclareLaunchArgument("max_joint_jump", default_value="0.35"),
            DeclareLaunchArgument("preview_dt", default_value="0.5"),
            DeclareLaunchArgument("validate_once", default_value="true"),
            DeclareLaunchArgument("timer_period", default_value="2.0"),
            DeclareLaunchArgument("display_topic", default_value="/display_planned_path"),
            DeclareLaunchArgument("workspace_min", default_value="[-2.0, -2.0, -0.1]"),
            DeclareLaunchArgument("workspace_max", default_value="[2.0, 2.0, 2.0]"),
            DeclareLaunchArgument("max_relative_distance_error", default_value="0.005"),
            DeclareLaunchArgument(
                "max_relative_orientation_error_rad",
                default_value="0.03",
            ),
            DeclareLaunchArgument("max_target_displacement", default_value="0.30"),
            Node(
                package="jaka_a12_dual_planner",
                executable="dual_object_trajectory_display",
                name="dual_object_trajectory_display_node",
                output="screen",
                parameters=[
                    {
                        "target_dx": ParameterValue(target_dx, value_type=float),
                        "target_dy": ParameterValue(target_dy, value_type=float),
                        "target_dz": ParameterValue(target_dz, value_type=float),
                        "target_roll": ParameterValue(target_roll, value_type=float),
                        "target_pitch": ParameterValue(target_pitch, value_type=float),
                        "target_yaw": ParameterValue(target_yaw, value_type=float),
                        "num_waypoints": ParameterValue(num_waypoints, value_type=int),
                        "ik_timeout": ParameterValue(ik_timeout, value_type=float),
                        "max_joint_jump": ParameterValue(
                            max_joint_jump,
                            value_type=float,
                        ),
                        "preview_dt": ParameterValue(preview_dt, value_type=float),
                        "validate_once": ParameterValue(
                            validate_once,
                            value_type=bool,
                        ),
                        "timer_period": ParameterValue(
                            timer_period,
                            value_type=float,
                        ),
                        "display_topic": ParameterValue(
                            display_topic,
                            value_type=str,
                        ),
                        "workspace_min": LaunchConfiguration("workspace_min"),
                        "workspace_max": LaunchConfiguration("workspace_max"),
                        "max_relative_distance_error": ParameterValue(
                            max_relative_distance_error,
                            value_type=float,
                        ),
                        "max_relative_orientation_error_rad": ParameterValue(
                            max_relative_orientation_error_rad,
                            value_type=float,
                        ),
                        "max_target_displacement": ParameterValue(
                            max_target_displacement,
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
