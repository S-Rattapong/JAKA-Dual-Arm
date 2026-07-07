import os
from glob import glob

from setuptools import find_packages, setup


package_name = "jaka_a12_dual_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="newtonrattapong",
    maintainer_email="newtonrattapong@todo.todo",
    description="No-motion dual-arm object-frame planner preview tools for JAKA A12.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            (
                "dual_object_frame_preview_node = "
                "jaka_a12_dual_planner.demo_node:main"
            ),
            (
                "dual_object_frame_moveit_validation = "
                "jaka_a12_dual_planner.moveit_validation_node:main"
            ),
            (
                "dual_object_trajectory_validation = "
                "jaka_a12_dual_planner.trajectory_validation_node:main"
            ),
            (
                "dual_object_trajectory_preview = "
                "jaka_a12_dual_planner.trajectory_preview_node:main"
            ),
            (
                "dual_object_trajectory_display = "
                "jaka_a12_dual_planner.trajectory_display_node:main"
            ),
            (
                "dual_object_experiment_runner = "
                "jaka_a12_dual_planner.experiment_runner_node:main"
            ),
            (
                "dual_object_final_demo = "
                "jaka_a12_dual_planner.final_demo_node:main"
            ),
        ],
    },
)
