import math

import numpy as np
from geometry_msgs.msg import Pose, TransformStamped


def normalize_quaternion(q):
    q = np.array(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def quaternion_to_rotation_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion(R):
    R = np.array(R, dtype=float)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return normalize_quaternion([x, y, z, w])


def rpy_to_rotation_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    R_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    R_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return R_z @ R_y @ R_x


def rpy_to_quaternion(roll, pitch, yaw):
    return rotation_matrix_to_quaternion(rpy_to_rotation_matrix(roll, pitch, yaw))


def quaternion_slerp(q0, q1, t):
    q0 = normalize_quaternion(q0)
    q1 = normalize_quaternion(q1)
    t = max(0.0, min(1.0, float(t)))
    dot = float(np.dot(q0, q1))

    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        return normalize_quaternion(q0 + t * (q1 - q0))

    theta_0 = math.acos(max(min(dot, 1.0), -1.0))
    sin_theta_0 = math.sin(theta_0)
    if abs(sin_theta_0) < 1e-12:
        return q0.copy()

    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return normalize_quaternion((s0 * q0) + (s1 * q1))


def pose_to_transform(pose):
    T = np.eye(4, dtype=float)
    T[:3, :3] = quaternion_to_rotation_matrix(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    T[:3, 3] = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ]
    return T


def transform_to_pose(T):
    T = np.array(T, dtype=float)
    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    q = rotation_matrix_to_quaternion(T[:3, :3])
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose


def transform_from_translation_quaternion(translation, quaternion):
    T = np.eye(4, dtype=float)
    T[:3, 3] = np.array(translation, dtype=float)
    T[:3, :3] = quaternion_to_rotation_matrix(quaternion)
    return T


def transform_from_translation_rpy(translation, roll, pitch, yaw):
    return transform_from_translation_quaternion(
        translation,
        rpy_to_quaternion(roll, pitch, yaw),
    )


def transform_from_stamped(msg):
    t = msg.transform.translation
    q = msg.transform.rotation
    return transform_from_translation_quaternion(
        [t.x, t.y, t.z],
        [q.x, q.y, q.z, q.w],
    )


def invert_transform(T):
    T = np.array(T, dtype=float)
    inv = np.eye(4, dtype=float)
    R = T[:3, :3]
    inv[:3, :3] = R.T
    inv[:3, 3] = -(R.T @ T[:3, 3])
    return inv


def compose_transform(A, B):
    return np.array(A, dtype=float) @ np.array(B, dtype=float)


def apply_transform(T, point):
    point_h = np.ones(4, dtype=float)
    point_h[:3] = np.array(point, dtype=float)
    return (np.array(T, dtype=float) @ point_h)[:3]


def transform_to_stamped(T, parent_frame, child_frame, stamp):
    pose = transform_to_pose(T)
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation.x = pose.position.x
    msg.transform.translation.y = pose.position.y
    msg.transform.translation.z = pose.position.z
    msg.transform.rotation.x = pose.orientation.x
    msg.transform.rotation.y = pose.orientation.y
    msg.transform.rotation.z = pose.orientation.z
    msg.transform.rotation.w = pose.orientation.w
    return msg
