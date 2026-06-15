from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "jaka_a12", package_name="jaka_a12_moveit_config"
    ).to_moveit_configs()

    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    # Publish /joint_states first. The demo launch starts ros2_control_node with
    # fake hardware, then these spawners activate the broadcaster and trajectory
    # controller. If the broadcaster is not active, RViz/robot_state_publisher
    # can see /joint_states but it has no publisher.
    controllers = ["joint_state_broadcaster"] + controller_names

    return LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen",
            )
            for controller in controllers
        ]
    )
