#!/usr/bin/env python3
"""Generate deterministic Web joint-limit metadata from the dual-arm Xacro."""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


XACRO_RELATIVE_PATH = Path(
    "src/jaka_ros2/src/jaka_a12_moveit_config/config/dual_jaka_a12.urdf.xacro"
)
JSON_OUTPUT_RELATIVE_PATH = Path(
    "dual_arm_app/web/assets/dual_jaka_a12_joint_limits.json"
)
MODULE_OUTPUT_RELATIVE_PATH = Path(
    "dual_arm_app/web/digital_twin_joint_limit_metadata.js"
)
EXPECTED_MODEL_NAME = "dual_jaka_a12"
EXPECTED_JOINTS = tuple(
    f"{side}_joint_{index}"
    for side in ("left", "right")
    for index in range(1, 7)
)
MOVABLE_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})


def find_repository_root() -> Path:
    """Find the checkout by walking upward from this script."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / XACRO_RELATIVE_PATH).is_file() and (
            candidate / "dual_arm_app"
        ).is_dir():
            return candidate
    raise RuntimeError(
        f"Could not locate repository root containing {XACRO_RELATIVE_PATH}"
    )


def expand_xacro(xacro_path: Path) -> str:
    """Expand the combined model without launching ROS nodes."""
    xacro_command = shutil.which("xacro")
    if xacro_command is None:
        raise RuntimeError("xacro executable not found")
    try:
        result = subprocess.run(
            [xacro_command, str(xacro_path)],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or "no diagnostic output").strip()
        raise RuntimeError(f"xacro expansion failed: {detail}") from exc
    return result.stdout


def finite_attribute(element: ET.Element, attribute: str, joint_name: str) -> float:
    """Read one required finite numeric limit attribute."""
    raw_value = element.get(attribute)
    if raw_value is None:
        raise RuntimeError(f"Joint {joint_name} has no {attribute} position limit")
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise RuntimeError(
            f"Joint {joint_name} has non-numeric {attribute} limit: {raw_value}"
        ) from exc
    if not math.isfinite(value):
        raise RuntimeError(f"Joint {joint_name} has non-finite {attribute} limit")
    return value


def extract_metadata(expanded_urdf: str) -> dict[str, object]:
    """Validate an expanded URDF and extract deterministic joint metadata."""
    try:
        root = ET.fromstring(expanded_urdf)
    except ET.ParseError as exc:
        raise RuntimeError(f"Expanded robot model is not valid XML: {exc}") from exc
    if root.tag != "robot" or root.get("name") != EXPECTED_MODEL_NAME:
        raise RuntimeError(
            f"Expanded model must be robot '{EXPECTED_MODEL_NAME}'"
        )

    all_joints = root.findall("joint")
    movable_joints = [
        joint for joint in all_joints if joint.get("type") in MOVABLE_JOINT_TYPES
    ]
    movable_names = [joint.get("name") for joint in movable_joints]
    duplicate_names = sorted(
        name for name in set(movable_names) if movable_names.count(name) > 1
    )
    duplicate_expected = [name for name in duplicate_names if name in EXPECTED_JOINTS]
    if duplicate_expected:
        raise RuntimeError(
            f"Duplicate expected movable joints: {', '.join(duplicate_expected)}"
        )
    if len(movable_joints) != len(EXPECTED_JOINTS):
        raise RuntimeError(
            f"Expected exactly {len(EXPECTED_JOINTS)} movable joints, "
            f"found {len(movable_joints)}"
        )
    missing = [name for name in EXPECTED_JOINTS if name not in movable_names]
    if missing:
        raise RuntimeError(f"Missing expected movable joints: {', '.join(missing)}")
    unexpected = [name for name in movable_names if name not in EXPECTED_JOINTS]
    if unexpected:
        raise RuntimeError(f"Unexpected movable joints: {', '.join(unexpected)}")
    if movable_names != list(EXPECTED_JOINTS):
        raise RuntimeError("Movable joint order does not match the expected dual-arm order")

    position_limits: dict[str, dict[str, float]] = {}
    velocity_limits: dict[str, float] = {}
    for joint in movable_joints:
        joint_name = joint.get("name")
        limit = joint.find("limit")
        if limit is None:
            raise RuntimeError(f"Joint {joint_name} has no position limit element")
        lower = finite_attribute(limit, "lower", joint_name)
        upper = finite_attribute(limit, "upper", joint_name)
        if lower >= upper:
            raise RuntimeError(
                f"Joint {joint_name} must have lower position limit < upper limit"
            )
        position_limits[joint_name] = {"min": lower, "max": upper}

        raw_velocity = limit.get("velocity")
        if raw_velocity is not None:
            try:
                velocity = float(raw_velocity)
            except ValueError as exc:
                raise RuntimeError(
                    f"Joint {joint_name} has non-numeric URDF velocity metadata"
                ) from exc
            if not math.isfinite(velocity) or velocity <= 0:
                raise RuntimeError(
                    f"Joint {joint_name} has invalid URDF velocity metadata"
                )
            velocity_limits[joint_name] = velocity

    return {
        "schema_version": 1,
        "model": EXPECTED_MODEL_NAME,
        "unit": "radian",
        "source_model": XACRO_RELATIVE_PATH.as_posix(),
        "model_structure": {
            "link_count": len(root.findall("link")),
            "total_joint_count": len(all_joints),
            "movable_joint_count": len(movable_joints),
        },
        "joint_order": list(EXPECTED_JOINTS),
        "position_limits": position_limits,
        "urdf_velocity_metadata": {
            "unit": "radian_per_second",
            "validation_status": "NOT_EVALUATED",
            "limits": velocity_limits,
        },
    }


def write_atomically(path: Path, content: str) -> None:
    """Replace one generated output only after a complete same-directory write."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_file.write(content)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
            temporary_path = Path(temporary_file.name)
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def generated_module(metadata: dict[str, object]) -> str:
    """Return an ES module generated from the same object as the audit JSON."""
    serialized = json.dumps(metadata, indent=2, ensure_ascii=True)
    return (
        "// GENERATED FILE — DO NOT EDIT.\n"
        "// Source: dual_jaka_a12.urdf.xacro via "
        "build_dual_arm_validation_metadata.py.\n"
        "export const DUAL_JAKA_A12_JOINT_LIMIT_METADATA = "
        f"Object.freeze({serialized});\n"
    )


def main() -> int:
    repository_root = find_repository_root()
    xacro_path = repository_root / XACRO_RELATIVE_PATH
    metadata = extract_metadata(expand_xacro(xacro_path))
    json_content = json.dumps(metadata, indent=2, ensure_ascii=True) + "\n"
    json_path = repository_root / JSON_OUTPUT_RELATIVE_PATH
    module_path = repository_root / MODULE_OUTPUT_RELATIVE_PATH
    write_atomically(json_path, json_content)
    write_atomically(module_path, generated_module(metadata))
    print(f"Generated joint-limit JSON: {json_path}")
    print(f"Generated joint-limit module: {module_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
