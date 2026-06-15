from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # The MoveIt demo launch also starts ros2_control controller spawners. Without
    # the fake hardware plugin in the robot_description, ros2_control_node exits
    # before /controller_manager is available, so the spawners wait forever and
    # /joint_states has no publisher. Keep this demo local-only by enabling the
    # mock_components/GenericSystem path in jaka_a12.ros2_control.xacro.
    moveit_config = (
        MoveItConfigsBuilder("jaka_a12", package_name="jaka_a12_moveit_config")
        .robot_description(mappings={"use_rviz_sim": "true"})
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
