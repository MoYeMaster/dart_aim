import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory("hik_camera_ros2_driver"),
        "config",
        "camera_params.yaml",
    )
    detector_params = os.path.join(
        get_package_share_directory("rm_vision_bringup"),
        "config",
        "detector_params.yaml",
    )

    camera_node = Node(
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        name="camera_node",
        output="screen",
        parameters=[camera_params],
        arguments=[
            "--ros-args",
            "--log-level",
            ["camera_node:=", LaunchConfiguration("camera_log_level")],
        ],
    )
    detector_node = Node(
        package="detector",
        executable="detector_node",
        name="detector",
        output="screen",
        parameters=[detector_params],
        arguments=[
            "--ros-args",
            "--log-level",
            ["detector:=", LaunchConfiguration("detector_log_level")],
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_log_level", default_value="info"),
            DeclareLaunchArgument("detector_log_level", default_value="info"),
            PushRosNamespace("dart"),
            camera_node,
            TimerAction(period=1.0, actions=[detector_node]),
        ]
    )
