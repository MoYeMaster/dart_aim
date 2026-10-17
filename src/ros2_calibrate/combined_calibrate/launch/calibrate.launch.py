"""One launch file for the whole camera + LiDAR joint calibration.

    ros2 launch combined_calibrate calibrate.launch.py
    ros2 launch combined_calibrate calibrate.launch.py stage:=capture
    ros2 launch combined_calibrate calibrate.launch.py stage:=solve
    ros2 launch combined_calibrate calibrate.launch.py start_drivers:=false
    ros2 launch combined_calibrate calibrate.launch.py use_rviz:=false

stage:
  all      drivers + guided calibration (capture, pick corners, solve, verify)
  capture  record shots only, pick the corners later with the CLI tools
  solve    solve from the shots already on disk and publish the overlay
  verify   only publish the overlay of an existing extrinsics.yaml

The LiDAR must publish PointCloud2 (xfer_format 0), the camera image_raw plus
image_raw/camera_info; both drivers are started from here unless you pass
start_drivers:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _package_available(package):
    """True when ament can find the package in the current environment."""
    try:
        get_package_share_directory(package)
        return True
    except Exception:
        return False


def _optional_share_file(package, relative):
    """Path inside an installed package, or '' when the package is absent."""
    try:
        return os.path.join(get_package_share_directory(package), relative)
    except Exception:
        return ""


def _calibration_nodes(context, *args, **kwargs):
    stage = context.launch_configurations.get("stage", "all")
    mode = {
        "all": "guided",
        "capture": "capture",
        "solve": "solve",
        "verify": "verify",
    }.get(stage, "guided")

    parameters = [LaunchConfiguration("params_file"), {
        "mode": mode,
        "shots_dir": LaunchConfiguration("shots_dir"),
        "extrinsics_path": LaunchConfiguration("extrinsics"),
        "lidar_topic": LaunchConfiguration("lidar_topic"),
        "image_topic": LaunchConfiguration("image_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "lidar_frame": LaunchConfiguration("lidar_frame"),
        "accumulate_frames": LaunchConfiguration("accumulate_frames"),
        "corner_count": LaunchConfiguration("corner_count"),
        "corner_labels": LaunchConfiguration("corner_labels"),
        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
    }]

    interactive = context.launch_configurations.get("interactive", "")
    if interactive:
        # bool, not the "true"/"false" string the launch argument carries
        parameters[1]["interactive"] = ParameterValue(LaunchConfiguration("interactive"),
                                                      value_type=bool)

    return [
        Node(
            package="combined_calibrate",
            executable="calibrate_node",
            name="calibrate",
            output="screen",
            emulate_tty=True,
            parameters=parameters,
        )
    ]


def generate_launch_description():
    camera_params = _optional_share_file("hik_camera_ros2_driver", "config/camera_params.yaml")
    lidar_config = _optional_share_file("livox_ros2_driver", "config/livox_lidar_config.json")
    package_share = get_package_share_directory("combined_calibrate")

    declared_arguments = [
        DeclareLaunchArgument("stage", default_value="all",
                              description="all | capture | solve | verify"),
        DeclareLaunchArgument("start_drivers", default_value="true",
                              description="Start the Livox and Hikvision drivers"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Start rviz2 for cloud / image / overlay"),
        DeclareLaunchArgument(
            "shots_dir",
            default_value=os.path.join(os.path.expanduser("~"), "dart_calibration", "shots"),
            description="Directory holding the shot_* folders"),
        DeclareLaunchArgument("extrinsics", default_value="",
                              description="Result YAML, empty = <shots_dir>/extrinsics.yaml"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(package_share, "config",
                                                         "combined_calibrate.yaml"),
                              description="combined_calibrate parameter file"),
        DeclareLaunchArgument("camera_params", default_value=camera_params,
                              description="Parameter file of hik_camera_ros2_driver"),
        DeclareLaunchArgument("lidar_config", default_value=lidar_config,
                              description="Livox user_config_path JSON"),
        DeclareLaunchArgument("lidar_publish_freq", default_value="10.0",
                              description="LiDAR publish rate in Hz"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame",
                              description="Frame id of the LiDAR cloud"),
        DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/image_raw/camera_info"),
        DeclareLaunchArgument("accumulate_frames", default_value="20"),
        DeclareLaunchArgument("corner_count", default_value="4"),
        DeclareLaunchArgument("corner_labels", default_value="左上,右上,右下,左下"),
        DeclareLaunchArgument("interactive", default_value="",
                              description="true/false, empty = auto detect a display"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0",
                              description="overlay publish rate after solving"),
    ]

    driver_condition = IfCondition(LaunchConfiguration("start_drivers"))

    livox_driver = Node(
        package="livox_ros2_driver",
        executable="livox_ros2_driver_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=driver_condition,
        parameters=[{
            "xfer_format": 0,          # 0 = sensor_msgs/PointCloud2
            "multi_topic": 0,          # every LiDAR on the same topic
            "data_src": 0,
            "publish_freq": LaunchConfiguration("lidar_publish_freq"),
            "output_data_type": 0,
            "frame_id": LaunchConfiguration("lidar_frame"),
            "user_config_path": LaunchConfiguration("lidar_config"),
            "lvx_file_path": "/tmp/livox_calibration.lvx",
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    hik_driver = Node(
        package="hik_camera_ros2_driver",
        executable="hik_camera_ros2_driver_node",
        name="hik_camera_ros2_driver",
        output="screen",
        emulate_tty=True,
        condition=driver_condition,
        parameters=[LaunchConfiguration("camera_params")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(package_share, "config", "combined_calibrate.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    def driver_actions():
        """Start a driver when its package is here, otherwise log why it is skipped.

        The drivers usually live in their own workspace (source it before this
        launch); a missing one must not take the whole calibration down.
        """
        for package, node, what in (
            ("livox_ros2_driver", livox_driver, "LiDAR"),
            ("hik_camera_ros2_driver", hik_driver, "camera"),
        ):
            if _package_available(package):
                yield node
            else:
                yield LogInfo(
                    condition=driver_condition,
                    msg=[f"combined_calibrate: {package} is not installed in this "
                         f"environment, so the {what} driver is not started. Source the "
                         f"workspace that provides it, or pass start_drivers:=false and "
                         f"run the {what} driver yourself."],
                )

    return LaunchDescription(
        declared_arguments
        + [
            LogInfo(msg=["combined_calibrate: stage=", LaunchConfiguration("stage"),
                         "  shots=", LaunchConfiguration("shots_dir")]),
            *driver_actions(),
            OpaqueFunction(function=_calibration_nodes),
            rviz,
        ]
    )
