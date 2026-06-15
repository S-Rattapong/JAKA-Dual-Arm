from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
pi = 3.14159265359

def generate_launch_description():
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

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
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {'robot_description': robot_description},
                    {
                        'zeros': {
                            'left_joint_1': pi,
                            'left_joint_2': pi/6,
                            'left_joint_3': -0.8,
                            'left_joint_4': 0.0,
                            'left_joint_5': 0.8,
                            'left_joint_6': 0.0,

                            'right_joint_1': 0.0, 
                            'right_joint_2': (pi/6)*5,
                            'right_joint_3': 0.8,
                            'right_joint_4': 0.0,
                            'right_joint_5': 2.34,
                            'right_joint_6': 0.0,
                        }
                    }
                ],
            ),  
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[robot_description],
                condition=UnlessCondition(use_gui),
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
#pi = 3.14159265359
#pi/2 = 1.570796326795

