import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("lidar_dart_cpp"), "config", "params.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="lidar_dart_cpp",
                executable="lidar_dart_node",
                name="lidar_dart_node",
                output="screen",
                parameters=[params],
            )
        ]
    )
