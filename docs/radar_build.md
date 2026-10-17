# 雷达相关包的合并与编译

## 工作区与依赖

本次验证环境：Ubuntu 24.04、ROS 2 Jazzy、GCC 13、系统 OpenCV 4.6 / PCL 1.14。
请勿混用此前独立标定工程 README 中的 Humble 环境。

| 包 | 用途 | 主要依赖 |
| --- | --- | --- |
| `livox_interfaces` | Livox 自定义消息 | ROSIDL、std_msgs |
| `livox_sdk_vendor` | 编译并安装 Livox SDK | ament_cmake、本仓库 Livox-SDK |
| `livox_ros2_driver` | Livox 驱动 / 组件 | livox_sdk_vendor、livox_interfaces、PCL、APR |
| `lidar_dart_cpp` | 雷达感知节点 | rclcpp、pcl_conversions、PCL、Eigen |
| `combined_calibrate` | 相机与雷达联合标定 | OpenCV、PCL、cv_bridge；运行驱动、RViz |
| `lidar_calibrate` | 基于参考地图的 GICP 标定 | PCL、pcl_conversions、tf2_ros |
| `multisensor_calibration_interface` | 多传感器标定接口 | ROSIDL、sensor_msgs、geometry_msgs |
| `small_gicp_vendor` | small_gicp 外部库 | ament_cmake_vendor_package、Eigen；首次下载需网络 |
| `multisensor_calibration` | 多传感器 GUI 标定工具 | small_gicp、pcl_ros、OpenCV、Qt5、RViz、TF2 |

`src/livox_driver/Livox-SDK/COLCON_IGNORE` 只防止 SDK 被 colcon 单独发现。
SDK 仍由 `livox_sdk_vendor` 以 ExternalProject 编译、安装，不会跳过雷达驱动。
优先使用现有 SDK 源码；源码不存在时才下载原 vendor 指定的 SDK 版本。

## 推荐的系统依赖安装

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
rosdep check --from-paths src --ignore-src --rosdistro jazzy
```

本机原先缺少以下四项（它们的系统安装需要 sudo）：

```bash
sudo apt-get install libapr1-dev libaprutil1-dev \
  ros-jazzy-pcl-ros ros-jazzy-ament-cmake-vendor-package
```

`combined_calibrate` 的 PCL rosdep key 已由无定义的 `libpcl-dev` 改为
`libpcl-all-dev`。系统实际安装的 Debian 包仍是 `libpcl-dev`。

## 全工作区编译

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --symlink-install --executor sequential \
  --event-handlers console_cohesion+
source install/setup.bash
```

使用包级串行、包内最多两个编译任务，避免 PCL / Eigen 模板编译耗尽内存。
所有新增包（包括 GUI 标定工具）均参与构建；不需要忽略标定包来通过编译。

## 本机无 sudo 的临时依赖前缀

本次会话将缺少的官方 apt 包解包到 `build/radar_deps/root`，没有写入 `/usr`、
`/opt/ros` 或系统 dpkg 数据库。在 Bash 中使用本次构建结果：

```bash
source /opt/ros/jazzy/setup.bash
source build/radar_deps/setup.bash
source install/setup.bash
```

当前终端若为 Zsh，则对应为：

```zsh
source /opt/ros/jazzy/setup.zsh
source build/radar_deps/setup.zsh
source install/setup.zsh
```

重新编译时也应先 source 上述本地依赖脚本，再执行 colcon 命令。
`build/radar_deps/setup.bash` 是本地生成文件，不纳入版本管理；删除 `build/`
会同时删除这套临时依赖。新机器建议按上文使用 rosdep 正式安装。
由于 dpkg 未记录安装，当前 `rosdep check` 仍会报告这四项缺失；不能把
“本地前缀足以编译”当作“系统已安装全部依赖”。

本地下载的 deb 与下载记录保留在 `build/radar_deps/debs/`。
完整构建输出保留在 `build/radar_build.log` 及 colcon 的 `log/` 目录。

## 编译修复范围

- 联合标定的 cv_bridge 头文件改为 Jazzy 使用的 `.hpp`；补装原先已构建但未安装的
  `extrinsic_camera_camera_calibration` 可执行程序。
- 补齐雷达 / 标定 launch 及多传感器工具实际使用的依赖声明。
- `lidar_dart_cpp` 导出库目标及其公共依赖，供后续包链接使用。
- SDK 优先复用本地源码，且只对新 GCC 的 `c++20-compat` 警告取消 `-Werror`。
- `small_gicp` 固定为本次验证的提交 `fa0cfc983417c044b6036f71dfa448038bb9c820`，
  避免跟随 master 变化导致后续构建漂移。使用同一提交的源码归档代替 Git 克隆，
  减少下载量并规避本机 GitHub Git 传输失败。支持通过
  `-DSMALL_GICP_ARCHIVE_URL=file:///绝对路径/small_gicp.tar.gz` 使用离线归档。
  本次缓存的归档位于 `build/radar_deps/small_gicp-fa0cfc9.tar.gz`。
- APR 改为配置阶段必须找到，并用 `PkgConfig::APR` 保留头文件、编译选项及库路径，
  支持非系统前缀安装，避免缺失依赖拖到编译或链接阶段。
- 标定包安装时排除源码目录中遗留的 `build/install/log`，不搬运旧构建产物。
- 全量重编发现原 `detector` 的 `state_sub_` 被声明为订阅对象而不是共享指针，
  已补上 `::SharedPtr`；未修改导引灯识别 / 追踪算法。

## 本次验证结果（2026-10-17）

- 最终全工作区复核：`Summary: 17 packages finished [5.38s]`，退出码 0。
  这是修复后的增量复核耗时，不是从零编译耗时。
- 9 个新增雷达相关 ROS 包与原有 8 个包均完成构建；SDK 由 vendor 包统一构建。
- 20 个雷达相关可执行程序 / 共享库通过 `ldd` 检查，无 `not found`。
- Livox 与多传感器标定 Python 接口及原生 type support 导入成功。
- Livox 驱动及两个多传感器可视化组件可通过 `ros2 component types` 发现。
- 雷达感知、GICP 标定、联合标定、Livox 驱动四个 launch 的 `--show-args` 检查通过，
  这仅验证 launch 描述加载，不启动设备。
- 独立 CMake 消费者能链接 `lidar_dart_cpp::lidar_dart_cpp`，并通过简单坐标平移检查。
- 本次未运行 `colcon test`，未验证实机雷达、相机或标定精度。

## 验证边界

编译、链接、安装与动态库解析不等于传感器实机验证。
雷达 IP / 广播码、相机设备、标定板、外参以及 `/livox/lidar` 的实际数据仍需实机检查。
`lidar_dart_cpp` 的地图路径默认为空、外参仍是配置初值，不能直接视为有效的融合结果。
原有视觉/导引灯算法与 bringup 启动链路未在此次依赖修复中调整。
