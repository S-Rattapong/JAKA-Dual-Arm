import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    package_name = "jaka_a12_moveit_config"
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "dual_jaka_a12.urdf.xacro"]
    )
    srdf_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "dual_jaka_a12.srdf"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "moveit.rviz"]
    )

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file]),
    }
    robot_description_semantic = {
        "robot_description_semantic": Command(["cat ", srdf_file]),
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(package_name, "config/dual_kinematics.yaml"),
    }
    moveit_controllers = load_yaml(package_name, "config/dual_moveit_controllers.yaml")
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        planning_pipelines,
        moveit_controllers,
        planning_scene_monitor_parameters,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_joint_state_publisher_gui", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="dual_joint_state_publisher_gui",
                output="screen",
                parameters=[
                    robot_description,
                    {
                        "zeros": {
                            "left_joint_1": 3.14,
                            "left_joint_2": 0.52124,
                            "left_joint_3": -0.800072,
                            "left_joint_4": 0.0,
                            "left_joint_5": 0.798816,
                            "left_joint_6": 0.0,
                            "right_joint_1": 0.0,
                            "right_joint_2": 2.617504,
                            "right_joint_3": 0.798816,
                            "right_joint_4": 0.0,
                            "right_joint_5": 2.339928,
                            "right_joint_6": 0.0,
                        },
                    },
                ],
                condition=IfCondition(use_joint_state_publisher_gui),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
