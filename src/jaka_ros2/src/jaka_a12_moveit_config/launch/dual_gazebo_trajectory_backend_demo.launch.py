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
    dual_gazebo_backend_debug_log = LaunchConfiguration(
        "dual_gazebo_backend_debug_log"
    )
    robot_description_topic = LaunchConfiguration("robot_description_topic")

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
                default_value="1.00",
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
                default_value="0.15",
            ),
            DeclareLaunchArgument(
                "dual_gazebo_backend_debug_log",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/robot_description",
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
                        "dual_gazebo_backend_debug_log": ParameterValue(
                            dual_gazebo_backend_debug_log,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
