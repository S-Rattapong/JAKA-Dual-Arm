"""Launch helper for the canonical dual-arm World/Base calibration."""

from __future__ import annotations

import sys
from pathlib import Path

from launch.substitutions import Command, FindExecutable


def _find_repository_root() -> Path | None:
    candidates = (Path(__file__).resolve(), Path.cwd().resolve())
    relative_artifact = Path("dual_arm_app/config/world_frame_calibration.json")
    for origin in candidates:
        for candidate in (origin, *origin.parents):
            if (candidate / relative_artifact).is_file():
                return candidate
    return None


def _load_xacro_mappings():
    repository_root = _find_repository_root()
    if repository_root is not None:
        root_text = str(repository_root)
        if root_text not in sys.path:
            sys.path.insert(0, root_text)
        from dual_arm_app.backend.world_frame_calibration import (  # noqa: PLC0415
            load_world_calibration,
        )

        calibration_path = (
            repository_root / "dual_arm_app/config/world_frame_calibration.json"
        )
        return load_world_calibration(calibration_path).xacro_mappings()

    installed_share = Path(__file__).resolve().parent.parent
    installed_artifact = installed_share / "config/world_frame_calibration.json"
    helper_directory = str(Path(__file__).resolve().parent)
    if helper_directory not in sys.path:
        sys.path.insert(0, helper_directory)
    try:
        from world_frame_calibration import load_world_calibration  # noqa: PLC0415
    except ImportError as exc:
        raise RuntimeError(
            "Cannot load the canonical world calibration validator from the "
            "workspace or installed MoveIt share directory"
        ) from exc
    return load_world_calibration(installed_artifact).xacro_mappings()


def calibrated_robot_description(xacro_file):
    """Create the one robot_description Command with validated Xacro inputs."""
    mappings = _load_xacro_mappings()
    command = [FindExecutable(name="xacro"), " ", xacro_file]
    for name in (
        "left_base_xyz",
        "left_base_rpy",
        "right_base_xyz",
        "right_base_rpy",
    ):
        command.append(f' {name}:="{mappings[name]}"')
    return {"robot_description": Command(command)}
