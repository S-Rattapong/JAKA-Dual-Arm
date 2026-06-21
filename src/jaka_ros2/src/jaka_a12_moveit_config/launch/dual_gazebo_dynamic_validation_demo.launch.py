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
                    }
                ],
            ),
        ]
    )
