from setuptools import find_packages, setup

package_name = 'jaka_coop_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newtonrattapong',
    maintainer_email='newtonrattapong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'ee_pose_monitor = jaka_coop_monitor.ee_pose_monitor:main',
        'pose_error_monitor = jaka_coop_monitor.pose_error_monitor:main',
        'pose_error_logger = jaka_coop_monitor.pose_error_logger:main',
        'dual_relative_pose_monitor = jaka_coop_monitor.dual_relative_pose_monitor:main',
        'dual_relative_pose_logger = jaka_coop_monitor.dual_relative_pose_logger:main',
        'virtual_object_frame_publisher = jaka_coop_monitor.virtual_object_frame_publisher:main',
        'object_grasp_frame_publisher = jaka_coop_monitor.object_grasp_frame_publisher:main',
        'object_grasp_error_monitor = jaka_coop_monitor.object_grasp_error_monitor:main',
        'object_grasp_error_logger = jaka_coop_monitor.object_grasp_error_logger:main',
        'joint_limit_monitor = jaka_coop_monitor.joint_limit_monitor:main',
        'joint_limit_logger = jaka_coop_monitor.joint_limit_logger:main',
        'dual_jacobian_monitor = jaka_coop_monitor.dual_jacobian_monitor:main',
        'dual_jacobian_velocity_monitor = jaka_coop_monitor.dual_jacobian_velocity_monitor:main',
        'dual_jacobian_velocity_logger = jaka_coop_monitor.dual_jacobian_velocity_logger:main',
        'dual_relative_jacobian_monitor = jaka_coop_monitor.dual_relative_jacobian_monitor:main',
        'dual_relative_jacobian_logger = jaka_coop_monitor.dual_relative_jacobian_logger:main',
        'dual_relative_motion_demo = jaka_coop_monitor.dual_relative_motion_demo:main',
        'dual_object_trajectory_demo = jaka_coop_monitor.dual_object_trajectory_demo:main',
        'dual_interactive_object_demo = jaka_coop_monitor.dual_interactive_object_demo:main',
        'dual_task_priority_object_demo = jaka_coop_monitor.dual_task_priority_object_demo:main',
        'dual_task_priority_joint_limit_demo = jaka_coop_monitor.dual_task_priority_joint_limit_demo:main',
        'dual_safety_posture_demo = jaka_coop_monitor.dual_safety_posture_demo:main',
    ],
},
)
