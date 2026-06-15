import numpy as np

import rclpy

from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from jaka_coop_monitor.dual_object_trajectory_demo import DualObjectTrajectoryDemo


class DualInteractiveObjectDemo(DualObjectTrajectoryDemo):
    def __init__(self):
        if not hasattr(self, '_demo_node_name'):
            self._demo_node_name = 'dual_interactive_object_demo'
        if not hasattr(self, '_demo_title'):
            self._demo_title = 'Dual Interactive Object Demo'
        if not hasattr(self, '_log_trajectory_params'):
            self._log_trajectory_params = False
        super().__init__()
        self.declare_parameter('marker_scale', 0.20)

        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.marker_server = InteractiveMarkerServer(self, 'desired_object_marker')
        self.marker_created = False
        self.target_object_position = None

        self.get_logger().info(
            'Drag the desired_object interactive marker in RViz to move both arms.'
        )
        self.get_logger().info(
            'Interactive marker update topic: /desired_object_marker/update'
        )

    def initialize_state(self):
        super().initialize_state()
        self.target_object_position = self.initial_object_position.copy()
        self.latest_desired_object_position = self.target_object_position.copy()
        self.create_object_marker()

    def desired_object_position(self, elapsed):
        del elapsed

        if self.target_object_position is None:
            return self.initial_object_position.copy()

        return self.target_object_position.copy()

    def create_object_marker(self):
        if self.marker_created:
            return

        marker = InteractiveMarker()
        marker.header.frame_id = 'world'
        marker.name = 'desired_object'
        marker.description = 'desired_object'
        marker.scale = max(self.marker_scale * 2.5, 0.05)
        marker.pose = self.pose_from_position(self.target_object_position)

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True

        object_marker = Marker()
        object_marker.type = Marker.SPHERE
        object_marker.scale.x = self.marker_scale
        object_marker.scale.y = self.marker_scale
        object_marker.scale.z = self.marker_scale
        object_marker.color.r = 0.1
        object_marker.color.g = 0.8
        object_marker.color.b = 1.0
        object_marker.color.a = 0.75

        visual_control.markers.append(object_marker)
        marker.controls.append(visual_control)

        marker.controls.append(self.make_move_axis_control('move_x', 1.0, 0.0, 0.0))
        marker.controls.append(self.make_move_axis_control('move_y', 0.0, 1.0, 0.0))
        marker.controls.append(self.make_move_axis_control('move_z', 0.0, 0.0, 1.0))

        self.marker_server.insert(marker, feedback_callback=self.process_marker_feedback)
        self.marker_server.applyChanges()
        self.marker_created = True

    def make_move_axis_control(self, name, x, y, z):
        control = InteractiveMarkerControl()
        control.name = name
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1.0
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        return control

    def process_marker_feedback(self, feedback):
        self.target_object_position = np.array(
            [
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z,
            ],
            dtype=float,
        )

    def pose_from_position(self, position):
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = DualInteractiveObjectDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
