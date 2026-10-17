"""Collect the reference field map used by the LiDAR-only calibration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    params = os.path.join(
        get_package_share_directory("lidar_calibrate"), "config", "lidar_calibrate.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value=os.path.join(os.path.expanduser("~"), "dart_calibration"),
                description="Directory that receives the reference PCD.",
            ),
            Node(
                package="lidar_calibrate",
                executable="map_builder_node",
                name="map_builder_node",
                output="screen",
                parameters=[params, {"output_pcd": [output_dir, "/field_map.pcd"]}],
            ),
        ]
    )
