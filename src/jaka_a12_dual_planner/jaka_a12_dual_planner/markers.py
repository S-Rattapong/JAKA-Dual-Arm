from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _point_from_xyz(xyz):
    point = Point()
    point.x = float(xyz[0])
    point.y = float(xyz[1])
    point.z = float(xyz[2])
    return point


def _set_color(marker, rgba):
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3])


def make_sphere_marker(
    frame_id,
    stamp,
    marker_id,
    namespace,
    position,
    scale,
    rgba,
):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = _point_from_xyz(position)
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(scale)
    marker.scale.y = float(scale)
    marker.scale.z = float(scale)
    _set_color(marker, rgba)
    return marker


def make_line_marker(
    frame_id,
    stamp,
    marker_id,
    namespace,
    start,
    end,
    width,
    rgba,
):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width)
    _set_color(marker, rgba)
    marker.points.append(_point_from_xyz(start))
    marker.points.append(_point_from_xyz(end))
    return marker


def make_axis_markers(frame_id, stamp, namespace, start_id, T, axis_length, width):
    origin = T[:3, 3]
    axes = [
        ("x", [1.0, 0.1, 0.1, 0.9], T[:3, 0]),
        ("y", [0.1, 0.8, 0.1, 0.9], T[:3, 1]),
        ("z", [0.1, 0.3, 1.0, 0.9], T[:3, 2]),
    ]
    markers = []
    for index, (name, color, direction) in enumerate(axes):
        markers.append(
            make_line_marker(
                frame_id,
                stamp,
                start_id + index,
                f"{namespace}_{name}",
                origin,
                origin + direction * axis_length,
                width,
                color,
            )
        )
    return markers


def make_preview_markers(
    frame_id,
    stamp,
    T_object,
    T_left,
    T_right,
    axis_length,
    include_grasp_line=True,
    T_actual_object=None,
):
    markers = MarkerArray()
    object_position = T_object[:3, 3]
    left_position = T_left[:3, 3]
    right_position = T_right[:3, 3]

    markers.markers.append(
        make_sphere_marker(
            frame_id,
            stamp,
            1,
            "target_object",
            object_position,
            0.06,
            [0.1, 0.8, 1.0, 0.75],
        )
    )
    markers.markers.append(
        make_sphere_marker(
            frame_id,
            stamp,
            2,
            "target_left_grasp",
            left_position,
            0.045,
            [1.0, 0.25, 0.15, 0.8],
        )
    )
    markers.markers.append(
        make_sphere_marker(
            frame_id,
            stamp,
            3,
            "target_right_grasp",
            right_position,
            0.045,
            [0.2, 0.45, 1.0, 0.8],
        )
    )

    marker_id = 10
    for T, namespace in (
        (T_object, "target_object_axis"),
        (T_left, "target_left_grasp_axis"),
        (T_right, "target_right_grasp_axis"),
    ):
        for marker in make_axis_markers(
            frame_id,
            stamp,
            namespace,
            marker_id,
            T,
            axis_length,
            0.01,
        ):
            markers.markers.append(marker)
        marker_id += 10

    if include_grasp_line:
        markers.markers.append(
            make_line_marker(
                frame_id,
                stamp,
                50,
                "target_grasp_line",
                left_position,
                right_position,
                0.012,
                [1.0, 0.85, 0.1, 0.85],
            )
        )

    if T_actual_object is not None:
        markers.markers.append(
            make_sphere_marker(
                frame_id,
                stamp,
                60,
                "actual_object_midpoint",
                T_actual_object[:3, 3],
                0.04,
                [0.9, 0.9, 0.9, 0.65],
            )
        )

    return markers
