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
    startup_hold_seconds = LaunchConfiguration("startup_hold_seconds")
    initial_pose_only = LaunchConfiguration("initial_pose_only")
    initial_left_positions = LaunchConfiguration("initial_left_positions")
    initial_right_positions = LaunchConfiguration("initial_right_positions")
    initial_left_joints = [
        LaunchConfiguration(f"initial_left_joint_{i}") for i in range(1, 7)
    ]
    initial_right_joints = [
        LaunchConfiguration(f"initial_right_joint_{i}") for i in range(1, 7)
    ]

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("jaka_a12_moveit_config"), "config", "dual_jaka_a12.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("jaka_a12_moveit_config"), "config", "dual_jaka_a12_tf_demo.rviz"]
    )

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("startup_hold_seconds", default_value="2.0"),
            DeclareLaunchArgument("initial_pose_only", default_value="false"),
            DeclareLaunchArgument(
                "initial_left_positions",
                default_value="[3.14, 0.52124, -0.800072, 0.0, 0.798816, 0.0]",
            ),
            DeclareLaunchArgument(
                "initial_right_positions",
                default_value="[0.0, 2.617504, 0.798816, 0.0, 2.339928, 0.0]",
            ),
            *[
                DeclareLaunchArgument(f"initial_left_joint_{i}", default_value="nan")
                for i in range(1, 7)
            ],
            *[
                DeclareLaunchArgument(f"initial_right_joint_{i}", default_value="nan")
                for i in range(1, 7)
            ],
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="jaka_coop_monitor",
                executable="dual_relative_motion_demo",
                name="dual_relative_motion_demo",
                output="screen",
                parameters=[
                    {
                        "robot_description_topic": "/robot_description",
                        "joint_states_topic": "/joint_states",
                        "startup_hold_seconds": ParameterValue(
                            startup_hold_seconds,
                            value_type=float,
                        ),
                        "initial_pose_only": ParameterValue(
                            initial_pose_only,
                            value_type=bool,
                        ),
                        "initial_left_positions": initial_left_positions,
                        "initial_right_positions": initial_right_positions,
                        **{
                            f"initial_left_joint_{i}": ParameterValue(
                                initial_left_joints[i - 1],
                                value_type=float,
                            )
                            for i in range(1, 7)
                        },
                        **{
                            f"initial_right_joint_{i}": ParameterValue(
                                initial_right_joints[i - 1],
                                value_type=float,
                            )
                            for i in range(1, 7)
                        },
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
