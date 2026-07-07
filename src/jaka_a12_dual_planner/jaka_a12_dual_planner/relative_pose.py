import math

import numpy as np

from jaka_a12_dual_planner.object_frame import (
    invert_transform,
    normalize_quaternion,
    rotation_matrix_to_quaternion,
)


def quaternion_inverse(q):
    q = normalize_quaternion(q)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = normalize_quaternion(q1)
    x2, y2, z2, w2 = normalize_quaternion(q2)
    return normalize_quaternion(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def relative_orientation_quaternion(left_quaternion, right_quaternion):
    return quaternion_multiply(quaternion_inverse(left_quaternion), right_quaternion)


def quaternion_angle_error(current, desired):
    current = normalize_quaternion(current)
    desired = normalize_quaternion(desired)
    dot = float(np.dot(current, desired))
    dot = abs(max(min(dot, 1.0), -1.0))
    return 2.0 * math.acos(dot)


def transform_relative_quaternion(T_left, T_right):
    T_relative = invert_transform(T_left) @ T_right
    return rotation_matrix_to_quaternion(T_relative[:3, :3])


def relative_position_vector(T_left, T_right):
    return np.array(T_right[:3, 3] - T_left[:3, 3], dtype=float)


def relative_distance(T_left, T_right):
    return float(np.linalg.norm(relative_position_vector(T_left, T_right)))


def transform_quaternion(T):
    return rotation_matrix_to_quaternion(T[:3, :3])


def relative_pose_metrics(T_left, T_right, desired=None):
    vector = relative_position_vector(T_left, T_right)
    distance = float(np.linalg.norm(vector))
    rel_q = relative_orientation_quaternion(
        transform_quaternion(T_left),
        transform_quaternion(T_right),
    )

    metrics = {
        "relative_position": vector,
        "relative_distance": distance,
        "relative_quaternion": rel_q,
    }

    if desired is not None:
        distance_error = distance - float(desired["relative_distance"])
        orientation_error = quaternion_angle_error(
            rel_q,
            desired["relative_quaternion"],
        )
        metrics.update(
            {
                "relative_distance_error": distance_error,
                "relative_distance_error_abs": abs(distance_error),
                "relative_orientation_error_rad": orientation_error,
                "relative_orientation_error_deg": math.degrees(orientation_error),
            }
        )

    return metrics
