from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    command_backend = LaunchConfiguration("command_backend")
    gazebo_joint_trajectory_topic = LaunchConfiguration(
        "gazebo_joint_trajectory_topic"
    )
    gazebo_single_arm_source = LaunchConfiguration("gazebo_single_arm_source")
    gazebo_trajectory_time_from_start = LaunchConfiguration(
        "gazebo_trajectory_time_from_start"
    )
    gazebo_trajectory_publish_period = LaunchConfiguration(
        "gazebo_trajectory_publish_period"
    )
    gazebo_command_position_smoothing = LaunchConfiguration(
        "gazebo_command_position_smoothing"
    )
    gazebo_command_alpha = LaunchConfiguration("gazebo_command_alpha")
    gazebo_backend_debug_log = LaunchConfiguration("gazebo_backend_debug_log")
    gazebo_wait_for_robot_description = LaunchConfiguration(
        "gazebo_wait_for_robot_description"
    )
    robot_description_topic = LaunchConfiguration("robot_description_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("command_backend", default_value="gazebo_trajectory"),
            DeclareLaunchArgument(
                "gazebo_joint_trajectory_topic",
                default_value="/jaka_a12_controller/joint_trajectory",
            ),
            DeclareLaunchArgument("gazebo_single_arm_source", default_value="left"),
            DeclareLaunchArgument(
                "gazebo_trajectory_time_from_start",
                default_value="0.20",
            ),
            DeclareLaunchArgument(
                "gazebo_trajectory_publish_period",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "gazebo_command_position_smoothing",
                default_value="true",
            ),
            DeclareLaunchArgument("gazebo_command_alpha", default_value="0.35"),
            DeclareLaunchArgument("gazebo_backend_debug_log", default_value="true"),
            DeclareLaunchArgument(
                "gazebo_wait_for_robot_description",
                default_value="false",
            ),
            DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"),
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
                        "gazebo_joint_trajectory_topic": ParameterValue(
                            gazebo_joint_trajectory_topic,
                            value_type=str,
                        ),
                        "gazebo_single_arm_source": ParameterValue(
                            gazebo_single_arm_source,
                            value_type=str,
                        ),
                        "gazebo_trajectory_time_from_start": ParameterValue(
                            gazebo_trajectory_time_from_start,
                            value_type=float,
                        ),
                        "gazebo_trajectory_publish_period": ParameterValue(
                            gazebo_trajectory_publish_period,
                            value_type=float,
                        ),
                        "gazebo_command_position_smoothing": ParameterValue(
                            gazebo_command_position_smoothing,
                            value_type=bool,
                        ),
                        "gazebo_command_alpha": ParameterValue(
                            gazebo_command_alpha,
                            value_type=float,
                        ),
                        "gazebo_backend_debug_log": ParameterValue(
                            gazebo_backend_debug_log,
                            value_type=bool,
                        ),
                        "gazebo_wait_for_robot_description": ParameterValue(
                            gazebo_wait_for_robot_description,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
