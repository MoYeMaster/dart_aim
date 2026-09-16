import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    detector_params = os.path.join(
        get_package_share_directory("rm_vision_bringup"),
        "config",
        "detector_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            PushRosNamespace("dart"),
            Node(
                package="detector",
                executable="detector_node",
                name="detector",
                output="screen",
                parameters=[detector_params],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    ["detector:=", LaunchConfiguration("log_level")],
                ],
            ),
        ]
    )
