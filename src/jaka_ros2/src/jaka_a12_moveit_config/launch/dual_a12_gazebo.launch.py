import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_x = LaunchConfiguration("left_x")
    left_y = LaunchConfiguration("left_y")
    left_z = LaunchConfiguration("left_z")
    left_roll = LaunchConfiguration("left_roll")
    left_pitch = LaunchConfiguration("left_pitch")
    left_yaw = LaunchConfiguration("left_yaw")
    right_x = LaunchConfiguration("right_x")
    right_y = LaunchConfiguration("right_y")
    right_z = LaunchConfiguration("right_z")
    right_roll = LaunchConfiguration("right_roll")
    right_pitch = LaunchConfiguration("right_pitch")
    right_yaw = LaunchConfiguration("right_yaw")
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("jaka_a12_moveit_config"),
            "config",
            "dual_a12_gazebo.urdf.xacro",
        ]
    )
    initial_positions_file = PathJoinSubstitution(
        [
            FindPackageShare("jaka_a12_moveit_config"),
            "config",
            "initial_positions.yaml",
        ]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " initial_positions_file:=",
                initial_positions_file,
                " left_x:=",
                left_x,
                " left_y:=",
                left_y,
                " left_z:=",
                left_z,
                " left_roll:=",
                left_roll,
                " left_pitch:=",
                left_pitch,
                " left_yaw:=",
                left_yaw,
                " right_x:=",
                right_x,
                " right_y:=",
                right_y,
                " right_z:=",
                right_z,
                " right_roll:=",
                right_roll,
                " right_pitch:=",
                right_pitch,
                " right_yaw:=",
                right_yaw,
            ]
        ),
        value_type=str,
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="dual_a12_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_dual_a12 = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dual_jaka_a12",
        output="screen",
        arguments=[
            "-name",
            "dual_jaka_a12",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "false",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "left_jaka_a12_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "right_jaka_a12_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("jaka_a12_moveit_config"),
        "config",
        "dual_jaka_a12_tf_demo.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    spawn_controllers_after_model = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_dual_a12,
            on_exit=[
                joint_state_broadcaster_spawner,
                left_controller_spawner,
                right_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "left_x",
                default_value="0.0",
                description="Left JAKA A12 base x position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "left_y",
                default_value="0.075",
                description="Left JAKA A12 base y position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "left_z",
                default_value="1.2",
                description="Left JAKA A12 base z position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "left_roll",
                default_value="1.570796326795",
                description="Left JAKA A12 base roll in radians.",
            ),
            DeclareLaunchArgument(
                "left_pitch",
                default_value="0.0",
                description="Left JAKA A12 base pitch in radians.",
            ),
            DeclareLaunchArgument(
                "left_yaw",
                default_value="3.14159265359",
                description="Left JAKA A12 base yaw in radians.",
            ),
            DeclareLaunchArgument(
                "right_x",
                default_value="0.0",
                description="Right JAKA A12 base x position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "right_y",
                default_value="-0.075",
                description="Right JAKA A12 base y position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "right_z",
                default_value="1.2",
                description="Right JAKA A12 base z position in Gazebo.",
            ),
            DeclareLaunchArgument(
                "right_roll",
                default_value="-1.570796326795",
                description="Right JAKA A12 base roll in radians.",
            ),
            DeclareLaunchArgument(
                "right_pitch",
                default_value="0.0",
                description="Right JAKA A12 base pitch in radians.",
            ),
            DeclareLaunchArgument(
                "right_yaw",
                default_value="3.14159265359",
                description="Right JAKA A12 base yaw in radians.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz for visualization only.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="empty.sdf",
                description="Gazebo world SDF to load.",
            ),
            gazebo_launch,
            robot_state_publisher,
            spawn_dual_a12,
            spawn_controllers_after_model,
            rviz,
        ]
    )
