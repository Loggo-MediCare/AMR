"""
display.launch.py

Milestone 1: bring up the minimal AMR TF tree.

Starts:
  - robot_state_publisher: reads amr.urdf and broadcasts the static links
    (camera_link, imu_link) to /tf_static, and the moving links
    (left_wheel_link, right_wheel_link) to /tf using whatever joint state
    joint_state_publisher is currently publishing.
  - joint_state_publisher: publishes default (zero-position) joint states
    for the two continuous wheel joints, since amr.urdf has no real
    encoders/hardware interface yet. This is what makes /tf non-empty for
    the wheel joints in this early milestone.

No Gazebo, no Nav2, no AMCL, no SLAM here by design.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'amr.urdf')

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
