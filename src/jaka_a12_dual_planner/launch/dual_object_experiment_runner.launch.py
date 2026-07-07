from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    num_waypoints = LaunchConfiguration("num_waypoints")
    ik_timeout = LaunchConfiguration("ik_timeout")
    max_joint_jump = LaunchConfiguration("max_joint_jump")
    preview_dt = LaunchConfiguration("preview_dt")
    timer_period = LaunchConfiguration("timer_period")
    output_dir = LaunchConfiguration("output_dir")
    output_prefix = LaunchConfiguration("output_prefix")
    run_once = LaunchConfiguration("run_once")
    publish_last_case_display = LaunchConfiguration("publish_last_case_display")
    display_topic = LaunchConfiguration("display_topic")
    max_relative_distance_error = LaunchConfiguration("max_relative_distance_error")
    max_relative_orientation_error_rad = LaunchConfiguration(
        "max_relative_orientation_error_rad"
    )
    max_target_displacement = LaunchConfiguration("max_target_displacement")

    return LaunchDescription(
        [
            DeclareLaunchArgument("num_waypoints", default_value="10"),
            DeclareLaunchArgument("ik_timeout", default_value="0.5"),
            DeclareLaunchArgument("max_joint_jump", default_value="0.35"),
            DeclareLaunchArgument("preview_dt", default_value="0.5"),
            DeclareLaunchArgument("timer_period", default_value="2.0"),
            DeclareLaunchArgument("output_dir", default_value="~/jaka_ws/logs"),
            DeclareLaunchArgument(
                "output_prefix",
                default_value="dual_object_experiment",
            ),
            DeclareLaunchArgument("run_once", default_value="true"),
            DeclareLaunchArgument("publish_last_case_display", default_value="true"),
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
                executable="dual_object_experiment_runner",
                name="dual_object_experiment_runner_node",
                output="screen",
                parameters=[
                    {
                        "num_waypoints": ParameterValue(num_waypoints, value_type=int),
                        "ik_timeout": ParameterValue(ik_timeout, value_type=float),
                        "max_joint_jump": ParameterValue(
                            max_joint_jump,
                            value_type=float,
                        ),
                        "preview_dt": ParameterValue(preview_dt, value_type=float),
                        "timer_period": ParameterValue(
                            timer_period,
                            value_type=float,
                        ),
                        "output_dir": ParameterValue(output_dir, value_type=str),
                        "output_prefix": ParameterValue(
                            output_prefix,
                            value_type=str,
                        ),
                        "run_once": ParameterValue(run_once, value_type=bool),
                        "publish_last_case_display": ParameterValue(
                            publish_last_case_display,
                            value_type=bool,
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
