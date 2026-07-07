import numpy as np

from jaka_a12_dual_planner.relative_pose import relative_pose_metrics


def check_workspace(position, workspace_min, workspace_max):
    position = np.array(position, dtype=float)
    workspace_min = np.array(workspace_min, dtype=float)
    workspace_max = np.array(workspace_max, dtype=float)
    ok = bool(np.all(position >= workspace_min) and np.all(position <= workspace_max))
    return {
        "ok": ok,
        "position": position,
        "workspace_min": workspace_min,
        "workspace_max": workspace_max,
    }


def check_target_displacement(current_T, target_T, max_target_displacement):
    displacement = float(
        np.linalg.norm(np.array(target_T[:3, 3]) - np.array(current_T[:3, 3]))
    )
    return {
        "ok": displacement <= max_target_displacement,
        "displacement": displacement,
        "max_target_displacement": float(max_target_displacement),
    }


def validate_preview(
    T_left_current,
    T_right_current,
    T_left_target,
    T_right_target,
    workspace_min,
    workspace_max,
    max_relative_distance_error,
    max_relative_orientation_error_rad,
    max_target_displacement,
):
    desired_relative = relative_pose_metrics(T_left_current, T_right_current)
    target_relative = relative_pose_metrics(
        T_left_target,
        T_right_target,
        desired=desired_relative,
    )

    left_workspace = check_workspace(
        T_left_target[:3, 3],
        workspace_min,
        workspace_max,
    )
    right_workspace = check_workspace(
        T_right_target[:3, 3],
        workspace_min,
        workspace_max,
    )
    left_displacement = check_target_displacement(
        T_left_current,
        T_left_target,
        max_target_displacement,
    )
    right_displacement = check_target_displacement(
        T_right_current,
        T_right_target,
        max_target_displacement,
    )

    relative_distance_ok = (
        target_relative["relative_distance_error_abs"]
        <= max_relative_distance_error
    )
    relative_orientation_ok = (
        target_relative["relative_orientation_error_rad"]
        <= max_relative_orientation_error_rad
    )

    ok = bool(
        left_workspace["ok"]
        and right_workspace["ok"]
        and left_displacement["ok"]
        and right_displacement["ok"]
        and relative_distance_ok
        and relative_orientation_ok
    )

    return {
        "ok": ok,
        "left_workspace_ok": left_workspace["ok"],
        "right_workspace_ok": right_workspace["ok"],
        "left_displacement_ok": left_displacement["ok"],
        "right_displacement_ok": right_displacement["ok"],
        "relative_distance_ok": relative_distance_ok,
        "relative_orientation_ok": relative_orientation_ok,
        "left_target_position": np.array(T_left_target[:3, 3], dtype=float),
        "right_target_position": np.array(T_right_target[:3, 3], dtype=float),
        "target_relative_distance": target_relative["relative_distance"],
        "relative_distance_error": target_relative["relative_distance_error"],
        "relative_distance_error_abs": target_relative[
            "relative_distance_error_abs"
        ],
        "relative_orientation_error_rad": target_relative[
            "relative_orientation_error_rad"
        ],
        "relative_orientation_error_deg": target_relative[
            "relative_orientation_error_deg"
        ],
        "left_displacement": left_displacement["displacement"],
        "right_displacement": right_displacement["displacement"],
        "workspace_min": np.array(workspace_min, dtype=float),
        "workspace_max": np.array(workspace_max, dtype=float),
        "max_relative_distance_error": float(max_relative_distance_error),
        "max_relative_orientation_error_rad": float(
            max_relative_orientation_error_rad
        ),
        "max_target_displacement": float(max_target_displacement),
    }
