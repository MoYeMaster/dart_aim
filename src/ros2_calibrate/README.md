# ros2_calibrate

> **dart_aim 集成说明（ROS 2 Jazzy）**：本目录已合并到当前工作区。
> 当前依赖、编译及本地环境加载方式见根目录 `docs/radar_build.md`。
> 下文保留原独立 Humble 工程说明，其中旧工作区路径不适用于当前集成构建。

镖架标定工作空间，两套彼此独立的方案，都跑在 ROS 2 Humble 上：

| 目录 | 方案 | 传感器 | 标定对象 |
| --- | --- | --- | --- |
| `combined_calibrate` | 相机 + 激光雷达联合标定（C++ / ROS 2） | Mid-70 + 海康相机 | 雷达到相机的 6 自由度外参 |
| `lidar_calibrate` | 仅激光雷达标定 | 仅 Mid-70 | 雷达坐标系到场地坐标系的位姿 |

两套方案解的是不同的问题，可以同时留着：

- `combined_calibrate` 给出 `T_lidar_to_camera`，用于把点云投到图像上做验靶、给视觉提供深度、或者做传感器融合；
  它自己会把 `livox_ros2_driver`（PointCloud2 模式）和 `hik_camera_ros2_driver` 一起拉起来。
- `lidar_calibrate` 不碰相机，靠 GICP 把当前点云配准到参考场地点云，直接输出镖架在场地里的位姿，对应东北大学 `T-DT_Radar` 的雷达定位思路。

## 编译

```bash
cd ~/Dart_ROS2_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select combined_calibrate lidar_calibrate
source install/setup.bash
```

相机/雷达驱动（`src/livox_ros2_driver`、`src/hik_camera_ros2_driver`）如果也要在本工作空间编译，
Livox 驱动会去 GitHub 拉 Livox-SDK，需要网络；也可以继续用已经编好的
`~/ws_livox/install`，两者 source 起来都能被 launch 找到。

根目录 `~/Dart/Dart_ROS2_WS` 里还放着固件工程（根 `CMakeLists.txt`、`Src/`、`Lib/`），
直接 `colcon build` 会把那些目录一起当成包扫进来，所以在这个工作空间里始终用
`--packages-select`。

`src/livox_camera_lidar_calibration` 是 Livox 官方的 **ROS 1 (catkin)** 版联合标定，
ROS 2 的 colcon 编译不了它；它已经被 C++/ROS 2 的 `combined_calibrate` 取代，所以
目录里放了 `COLCON_IGNORE`。要回到 ROS 1 用它，删掉那个文件即可。

## 运行

```bash
# 方案一：联合标定（C++ 版，一条命令跑完采集 -> 点角点 -> 求解 -> 验证）
ros2 launch combined_calibrate calibrate.launch.py

# 只采集 / 只求解 / 只验证 / 驱动已经跑着
ros2 launch combined_calibrate calibrate.launch.py stage:=capture
ros2 launch combined_calibrate calibrate.launch.py stage:=solve start_drivers:=false
ros2 launch combined_calibrate calibrate.launch.py stage:=verify start_drivers:=false
ros2 launch combined_calibrate calibrate.launch.py start_drivers:=false use_rviz:=false interactive:=false

# 事后补角点、单独求解的 CLI 工具
ros2 run combined_calibrate pick_corners ~/dart_calibration/shots/shot_000
ros2 run combined_calibrate pick_corners ~/dart_calibration/shots/shot_000 --lidar
ros2 run combined_calibrate solve_extrinsic ~/dart_calibration/shots
ros2 run combined_calibrate project_cloud

# 方案二：仅雷达标定（自带 RM2026.pcd 场地点云，直接跑）
ros2 launch lidar_calibrate lidar_calibrate.launch.py

# 换自己的场地时才需要先建图
ros2 launch lidar_calibrate build_map.launch.py output_dir:=$HOME/dart_calibration
```

联合标定现在是 C++ 实现，流程和话题细节见 `combined_calibrate/README.md`；
旧的 Python 实现在 `combined_calibrate_python_backup/`（带 `COLCON_IGNORE`，不参与编译）。

细节见各自目录下的 `README.md`。
