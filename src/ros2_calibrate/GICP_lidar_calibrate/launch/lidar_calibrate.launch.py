"""Run the LiDAR-only calibration against the reference field map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("lidar_calibrate")
    params = os.path.join(share, "config", "lidar_calibrate.yaml")
    default_map = os.path.join(share, "config", "RM2026.pcd")

    map_pcd = LaunchConfiguration("map_pcd")
    output_yaml = LaunchConfiguration("output_yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_pcd",
                default_value=default_map,
                description="Reference field cloud. Defaults to the bundled RM2026 map.",
            ),
            DeclareLaunchArgument(
                "output_yaml",
                default_value=os.path.join(
                    os.path.expanduser("~"), "dart_calibration", "lidar_extrinsics.yaml"
                ),
                description="Where the calibration result is written.",
            ),
            Node(
                package="lidar_calibrate",
                executable="lidar_calibrate_node",
                name="lidar_calibrate_node",
                output="screen",
                parameters=[
                    params,
                    {"map_pcd": map_pcd, "output_yaml": output_yaml},
                ],
            ),
        ]
    )
