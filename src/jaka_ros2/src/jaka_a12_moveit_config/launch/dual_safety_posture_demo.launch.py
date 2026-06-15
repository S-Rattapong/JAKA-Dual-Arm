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
