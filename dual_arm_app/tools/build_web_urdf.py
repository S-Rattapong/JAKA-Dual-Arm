#!/usr/bin/env python3
"""Build the browser-safe dual JAKA A12 URDF from the verified ROS Xacro."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


XACRO_RELATIVE_PATH = Path(
    "src/jaka_ros2/src/jaka_a12_moveit_config/config/dual_jaka_a12.urdf.xacro"
)
OUTPUT_RELATIVE_PATH = Path(
    "dual_arm_app/web/assets/dual_jaka_a12_web.urdf"
)
ROS_MESH_PREFIX = "package://jaka_description/meshes/jaka_a12_meshes/"
# The URDF is served from /digital-twin/assets/. URDFLoader resolves geometry
# against that working directory, so ../meshes/ maps to /digital-twin/meshes/.
WEB_MESH_PREFIX = "../meshes/"
EXPECTED_JOINTS = tuple(
    f"{side}_joint_{index}"
    for side in ("left", "right")
    for index in range(1, 7)
)


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


def generate_urdf(xacro_path: Path) -> str:
    """Run xacro and return a Web-compatible, validated URDF string."""
    xacro_command = shutil.which("xacro")
    if xacro_command is None:
        raise RuntimeError(
            "xacro executable not found; source the ROS 2 environment or install xacro"
        )

    try:
        result = subprocess.run(
            [xacro_command, str(xacro_path)],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        details = (exc.stderr or exc.stdout or "no diagnostic output").strip()
        raise RuntimeError(f"xacro generation failed: {details}") from exc

    urdf_text = result.stdout.replace(ROS_MESH_PREFIX, WEB_MESH_PREFIX)
    validate_urdf(urdf_text)
    return urdf_text


def validate_urdf(urdf_text: str) -> None:
    """Validate the generated model before replacing the checked-in asset."""
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError as exc:
        raise RuntimeError(f"Generated URDF is not valid XML: {exc}") from exc

    if root.tag != "robot" or root.get("name") != "dual_jaka_a12":
        raise RuntimeError("Generated URDF root must be robot 'dual_jaka_a12'")

    link_names = {element.get("name") for element in root.findall("link")}
    required_links = {"world", "left_base_link", "right_base_link"}
    missing_links = sorted(required_links - link_names)
    if missing_links:
        raise RuntimeError(f"Generated URDF is missing links: {missing_links}")

    joint_names = {element.get("name") for element in root.findall("joint")}
    missing_joints = sorted(set(EXPECTED_JOINTS) - joint_names)
    if missing_joints:
        raise RuntimeError(f"Generated URDF is missing joints: {missing_joints}")

    if "package://" in urdf_text or ROS_MESH_PREFIX in urdf_text:
        raise RuntimeError("Generated URDF still contains ROS package URIs")
    if WEB_MESH_PREFIX not in urdf_text:
        raise RuntimeError("Generated URDF contains no Web mesh references")


def write_atomically(output_path: Path, content: str) -> None:
    """Replace the output only after a complete write in the same directory."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_file.write(content)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
            temporary_path = Path(temporary_file.name)
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def main() -> int:
    repository_root = find_repository_root()
    xacro_path = repository_root / XACRO_RELATIVE_PATH
    output_path = repository_root / OUTPUT_RELATIVE_PATH
    if not xacro_path.is_file():
        raise RuntimeError(f"Verified dual-arm Xacro does not exist: {xacro_path}")

    write_atomically(output_path, generate_urdf(xacro_path))
    print(f"Generated Web URDF: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
