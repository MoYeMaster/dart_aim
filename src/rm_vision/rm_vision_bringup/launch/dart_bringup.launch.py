import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory("hik_camera_ros2_driver"),
        "config",
        "camera_params.yaml",
    )
    lidar_params = os.path.join(
        get_package_share_directory("lidar_dart_cpp"), "config", "params.yaml"
    )
    detector_params = os.path.join(
        get_package_share_directory("rm_vision_bringup"),
        "config",
        "detector_params.yaml",
    )
    gimbal_description = os.path.join(
        get_package_share_directory("rm_gimbal_description"),
        "urdf",
        "rm_gimbal.urdf.xacro",
    )
    dart_detector_params = os.path.join(
        get_package_share_directory("rm_vision_bringup"),
        "config",
        "dart_detector_params.yaml",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", gimbal_description]),
                    value_type=str,
                )
            }
        ],
    )

    camera_node = Node(
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        name="camera_node",
        output="screen",
        parameters=[camera_params, {"frame_id": "camera_optical_frame"}],
        arguments=[
            "--ros-args",
            "--log-level",
            ["camera_node:=", LaunchConfiguration("camera_log_level")],
        ],
    )
    lidar_node = Node(
                package="lidar_dart_cpp",
                executable="lidar_dart_node",
                name="lidar_dart_node",
                output="screen",
                parameters=[lidar_params],
    )

    detector_node = Node(
        package="detector",
        executable="detector_node",
        name="detector",
        output="screen",
        parameters=[detector_params, {"dart_states_topic": "detector/vision_dart_states"}],
        arguments=[
            "--ros-args",
            "--log-level",
            ["detector:=", LaunchConfiguration("detector_log_level")],
        ],
    )
    dart_detector_node = Node(
        package="dart_detector",
        executable="dart_detector_node",
        name="dart_detector",
        output="screen",
        parameters=[dart_detector_params],
    )
    tracker_node = Node(
        package="tracker",
        executable="tracker_node",
        name="tracker",
        output="screen",
        parameters=[{"dart_states_topic": "detector/dart_states"}],
        arguments=[
            "--ros-args",
            "--log-level",
            ["tracker_node:=", LaunchConfiguration("tracker_log_level")],
        ],
    )
    serial_driver_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        name="rm_serial_driver",
        output="screen",
        parameters=[
            {
                "device_name": LaunchConfiguration("serial_device"),
                "baud_rate": LaunchConfiguration("serial_baud_rate"),
                "flow_control": LaunchConfiguration("serial_flow_control"),
                "parity": LaunchConfiguration("serial_parity"),
                "dart_states_topic": "detector/dart_states",
                "stop_bits": ParameterValue(
                    LaunchConfiguration("serial_stop_bits"), value_type=str
                ),
            }
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            ["rm_serial_driver:=", LaunchConfiguration("serial_log_level")],
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_log_level", default_value="info"),
            DeclareLaunchArgument("detector_log_level", default_value="info"),
            DeclareLaunchArgument("tracker_log_level", default_value="info"),
            DeclareLaunchArgument("serial_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("serial_flow_control", default_value="none"),
            DeclareLaunchArgument("serial_parity", default_value="none"),
            DeclareLaunchArgument("serial_stop_bits", default_value="1"),
            DeclareLaunchArgument("serial_log_level", default_value="info"),
            PushRosNamespace("dart"),
            robot_state_publisher_node,
            camera_node,
            lidar_node,
            serial_driver_node,
            tracker_node,
            TimerAction(period=1.0, actions=[detector_node, dart_detector_node]),
        ]
    )
