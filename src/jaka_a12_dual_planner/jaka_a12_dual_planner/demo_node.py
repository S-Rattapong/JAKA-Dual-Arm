import ast
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from jaka_a12_dual_planner.markers import make_preview_markers
from jaka_a12_dual_planner.object_frame import (
    compose_transform,
    invert_transform,
    transform_from_stamped,
    transform_from_translation_rpy,
    transform_to_stamped,
)
from jaka_a12_dual_planner.validation import validate_preview


def _list_param(node, name, expected_length):
    value = node.get_parameter(name).value
    if isinstance(value, str):
        value = ast.literal_eval(value)
    value = list(value)
    if len(value) != expected_length:
        raise ValueError(
            f"Parameter {name} must have {expected_length} values, got {value!r}"
        )
    return np.array([float(item) for item in value], dtype=float)


def _xyz_text(value):
    return f"[{value[0]:+.4f}, {value[1]:+.4f}, {value[2]:+.4f}]"


class DualObjectFramePreviewNode(Node):
    def __init__(self):
        super().__init__("dual_object_frame_preview_node")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("left_tcp_frame", "left_J6")
        self.declare_parameter("right_tcp_frame", "right_J6")
        self.declare_parameter("target_dx", 0.0)
        self.declare_parameter("target_dy", 0.0)
        self.declare_parameter("target_dz", 0.05)
        self.declare_parameter("target_roll", 0.0)
        self.declare_parameter("target_pitch", 0.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("axis_length", 0.12)
        self.declare_parameter("workspace_min", [-2.0, -2.0, -0.1])
        self.declare_parameter("workspace_max", [2.0, 2.0, 2.0])
        self.declare_parameter("max_relative_distance_error", 0.005)
        self.declare_parameter("max_relative_orientation_error_rad", 0.03)
        self.declare_parameter("max_target_displacement", 0.30)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("log_period", 1.0)

        self.reference_frame = self.get_parameter("reference_frame").value
        self.left_tcp_frame = self.get_parameter("left_tcp_frame").value
        self.right_tcp_frame = self.get_parameter("right_tcp_frame").value
        self.axis_length = float(self.get_parameter("axis_length").value)
        self.workspace_min = _list_param(self, "workspace_min", 3)
        self.workspace_max = _list_param(self, "workspace_max", 3)
        self.max_relative_distance_error = float(
            self.get_parameter("max_relative_distance_error").value
        )
        self.max_relative_orientation_error_rad = float(
            self.get_parameter("max_relative_orientation_error_rad").value
        )
        self.max_target_displacement = float(
            self.get_parameter("max_target_displacement").value
        )
        self.publish_rate = max(0.5, float(self.get_parameter("publish_rate").value))
        self.log_period = max(0.1, float(self.get_parameter("log_period").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/dual_object_frame_preview/markers",
            10,
        )

        self.initialized = False
        self.last_log_time = None
        self.T_world_object_initial = None
        self.T_object_left_grasp = None
        self.T_object_right_grasp = None
        self.initial_left_right_distance = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            "Dual object-frame preview started. This node publishes only TF, "
            "markers, and logs; it does not publish /joint_states or trajectories."
        )
        self.get_logger().info(
            f"Using temporary TCP frames: {self.left_tcp_frame}, "
            f"{self.right_tcp_frame} in {self.reference_frame}."
        )

    def lookup_tcp_transforms(self):
        left_tf = self.tf_buffer.lookup_transform(
            self.reference_frame,
            self.left_tcp_frame,
            Time(),
        )
        right_tf = self.tf_buffer.lookup_transform(
            self.reference_frame,
            self.right_tcp_frame,
            Time(),
        )
        return transform_from_stamped(left_tf), transform_from_stamped(right_tf)

    def initialize_from_current_tf(self, T_world_left, T_world_right):
        midpoint = 0.5 * (T_world_left[:3, 3] + T_world_right[:3, 3])
        self.T_world_object_initial = transform_from_translation_rpy(
            midpoint,
            0.0,
            0.0,
            0.0,
        )
        T_object_world = invert_transform(self.T_world_object_initial)
        self.T_object_left_grasp = compose_transform(T_object_world, T_world_left)
        self.T_object_right_grasp = compose_transform(T_object_world, T_world_right)
        self.initial_left_right_distance = float(
            np.linalg.norm(T_world_right[:3, 3] - T_world_left[:3, 3])
        )
        self.initialized = True
        self.get_logger().info(
            "Initialized object frame from current TCP midpoint: "
            f"object={_xyz_text(midpoint)}, "
            f"left={_xyz_text(T_world_left[:3, 3])}, "
            f"right={_xyz_text(T_world_right[:3, 3])}, "
            f"relative_distance={self.initial_left_right_distance:.5f} m"
        )

    def target_delta_transform(self):
        return transform_from_translation_rpy(
            [
                float(self.get_parameter("target_dx").value),
                float(self.get_parameter("target_dy").value),
                float(self.get_parameter("target_dz").value),
            ],
            float(self.get_parameter("target_roll").value),
            float(self.get_parameter("target_pitch").value),
            float(self.get_parameter("target_yaw").value),
        )

    def compute_preview(self):
        T_world_object_target = compose_transform(
            self.T_world_object_initial,
            self.target_delta_transform(),
        )
        T_world_left_target = compose_transform(
            T_world_object_target,
            self.T_object_left_grasp,
        )
        T_world_right_target = compose_transform(
            T_world_object_target,
            self.T_object_right_grasp,
        )
        return T_world_object_target, T_world_left_target, T_world_right_target

    def publish_preview(
        self,
        T_world_object_target,
        T_world_left_target,
        T_world_right_target,
        T_world_actual_object,
    ):
        stamp = self.get_clock().now().to_msg()
        transforms = [
            transform_to_stamped(
                T_world_object_target,
                self.reference_frame,
                "target_object",
                stamp,
            ),
            transform_to_stamped(
                T_world_left_target,
                self.reference_frame,
                "target_left_grasp",
                stamp,
            ),
            transform_to_stamped(
                T_world_right_target,
                self.reference_frame,
                "target_right_grasp",
                stamp,
            ),
            transform_to_stamped(
                T_world_actual_object,
                self.reference_frame,
                "actual_object_midpoint",
                stamp,
            ),
        ]
        self.tf_broadcaster.sendTransform(transforms)
        markers = make_preview_markers(
            self.reference_frame,
            stamp,
            T_world_object_target,
            T_world_left_target,
            T_world_right_target,
            self.axis_length,
            include_grasp_line=True,
            T_actual_object=T_world_actual_object,
        )
        self.marker_pub.publish(markers)

    def log_validation_if_needed(self, validation):
        now = self.get_clock().now()
        if self.last_log_time is not None:
            elapsed = (now - self.last_log_time).nanoseconds * 1e-9
            if elapsed < self.log_period:
                return
        self.last_log_time = now

        self.get_logger().info(
            "Preview validation | "
            f"ok={validation['ok']} | "
            f"left={_xyz_text(validation['left_target_position'])} | "
            f"right={_xyz_text(validation['right_target_position'])} | "
            f"rel_dist={validation['target_relative_distance']:.5f} m | "
            f"rel_dist_err={validation['relative_distance_error']:+.6f} m | "
            f"rel_ori_err={validation['relative_orientation_error_rad']:.5f} rad "
            f"({validation['relative_orientation_error_deg']:.3f} deg) | "
            f"workspace_ok="
            f"{validation['left_workspace_ok'] and validation['right_workspace_ok']} | "
            f"displacement_ok="
            f"{validation['left_displacement_ok'] and validation['right_displacement_ok']} | "
            f"left_disp={validation['left_displacement']:.4f} m | "
            f"right_disp={validation['right_displacement']:.4f} m"
        )

        if abs(float(self.get_parameter("target_pitch").value)) > 1.45:
            self.get_logger().warn(
                "target_pitch is near +/-90 deg; RPY visualization may be "
                "hard to interpret."
            )

    def timer_callback(self):
        try:
            T_world_left, T_world_right = self.lookup_tcp_transforms()
        except TransformException as ex:
            self.get_logger().warn(f"Waiting for TCP TF frames: {ex}")
            return

        if not self.initialized:
            self.initialize_from_current_tf(T_world_left, T_world_right)

        T_world_object_target, T_world_left_target, T_world_right_target = (
            self.compute_preview()
        )
        actual_midpoint = 0.5 * (T_world_left[:3, 3] + T_world_right[:3, 3])
        T_world_actual_object = transform_from_translation_rpy(
            actual_midpoint,
            0.0,
            0.0,
            0.0,
        )

        validation = validate_preview(
            T_world_left,
            T_world_right,
            T_world_left_target,
            T_world_right_target,
            self.workspace_min,
            self.workspace_max,
            self.max_relative_distance_error,
            self.max_relative_orientation_error_rad,
            self.max_target_displacement,
        )
        self.publish_preview(
            T_world_object_target,
            T_world_left_target,
            T_world_right_target,
            T_world_actual_object,
        )
        self.log_validation_if_needed(validation)


def main(args=None):
    rclpy.init(args=args)
    node = DualObjectFramePreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
