# combined_calibrate

相机 + 激光雷达联合标定（C++ / ROS 2），求 `T_lidar_to_camera`
（把雷达点变换到相机系的 4x4 刚体变换）。一条 `ros2 launch` 就能跑完
采集 → 点角点 → 求解 → 投影验证。

```bash
ros2 launch combined_calibrate calibrate.launch.py
```

## 和旧版（Python）的关系

旧的 Python 版把流程拆成 `capture / pick_corners / solve_extrinsic / verify` 四条命令，
本版用 C++ 重写并合并成一个引导式节点 `calibrate_node`，同时把相机、雷达驱动一起拉起来，
所以正常情况下只需要上面那一条命令。求解的最小二乘问题和 Livox 官方 `getExt1` 一致：
最小化雷达靶标角点在图像上的重投影误差（`cv::solvePnP` 多初值 + `solvePnPRefineLM`）。

## 前置条件

- ROS 2 Humble，工作空间里有 `livox_ros2_driver`（PointCloud2 模式）和
  `hik_camera_ros2_driver`。两个驱动都在 `~/Dart/Dart_ROS2_WS/src` 下。
- 相机发 `/image_raw` + `/image_raw/camera_info`，雷达发 `/livox/lidar`
  （`xfer_format: 0`，即 `sensor_msgs/PointCloud2`）。驱动已在本 launch 里带默认参数启动，
  用 `start_drivers:=false` 可以关掉，改成自己手动起。
- 靶标放在两个传感器公共视场内，采集时保持静止（Mid-70 非重复扫描，每帧很稀疏，
  所以每个位姿累积 20 帧 ≈ 2 秒）。

驱动不必编在本工作空间里：把提供驱动的那个工作空间（例如 `~/ws_livox`）的
`install/setup.bash` 一起 source 即可，launch 按包名找驱动。如果哪个驱动包当前环境里
找不到，launch 不会整个失败，而是打一条日志跳过它，这时自己手起驱动或者
`start_drivers:=false` 都行。

## 一条命令的流程

```bash
ros2 launch combined_calibrate calibrate.launch.py
```

1. 节点等 `/livox/lidar`、`/image_raw`、`/image_raw/camera_info` 都有数据，
   并弹出一个实时预览窗口（`interactive:=false` 时不开窗）。
2. 摆好靶标，在**运行 launch 的那个终端**按回车：开始累积点云 → 存 `shot_000/`
   （`cloud.pcd` + `image.png` + `intrinsics.yaml`）。
3. 弹出图像窗口，按标题里的顺序点 4 个角点：左键添加、右键/`u` 撤销、`r` 重置、
   回车确认、ESC 放弃这个位姿。确认后写 `corners_img.txt` 和 `corners_preview.png`。
4. 在 rviz 里用工具栏的 **Publish Point** 按**同样顺序**点这 4 个角点，
   终端会实时打印每个点的坐标和已有个数；点满后写 `corners_lidar.txt`，
   并打印点间距方便和靶标实际尺寸核对。输入 `u` 撤销上一个点，`s` 跳过该位姿。
5. 重复 2–4，建议至少 5 个不同位姿（靶标要倾斜、换距离，不要所有位姿都正对且同距离）。
6. 终端输入 `f` 结束采集：解算、打印每个 shot 的重投影误差、写
   `<shots_dir>/extrinsics.yaml`，然后自动进入验证状态，在 rviz 里持续发布
   点云投影叠加（`/calibrate/overlay`、`/calibrate/points_camera`）。

中途 `q` 退出。重投影误差应在 1 px 量级；明显偏大的 shot 会被自动当外点剔除一次。

## 只跑其中一个阶段

```bash
# 只采集，角点后面用命令行工具补
ros2 launch combined_calibrate calibrate.launch.py stage:=capture

# 已经有角点，只求解 + 验证（不重新采集）
ros2 launch combined_calibrate calibrate.launch.py stage:=solve start_drivers:=false

# 只做投影验证
ros2 launch combined_calibrate calibrate.launch.py stage:=verify start_drivers:=false

# 驱动已经在别处跑着 / 不需要 rviz / 无显示器
ros2 launch combined_calibrate calibrate.launch.py start_drivers:=false use_rviz:=false interactive:=false
```

常用参数：`shots_dir:=`（默认 `~/dart_calibration/shots`）、`extrinsics:=`（默认
`<shots_dir>/extrinsics.yaml`）、`accumulate_frames:=`、`corner_count:=`、
`corner_labels:=`、`lidar_config:=`、`camera_params:=`、`lidar_publish_freq:=`。
全部参数见 `ros2 launch combined_calibrate calibrate.launch.py --show-args`。

## 无显示器 / 事后补角点

`calibrate_node` 在 `interactive:=false` 时只负责采集，角点用两个 CLI 工具补：

```bash
# 图像角点（OpenCV 窗口，需要显示器）
ros2 run combined_calibrate pick_corners ~/dart_calibration/shots/shot_000

# 雷达角点（rviz 里 Publish Point，顺序与图像一致）
ros2 run combined_calibrate pick_corners ~/dart_calibration/shots/shot_000 --lidar

# 也可以直接手写 corners_lidar.txt / corners_img.txt，每行一个点

# 求解（不需要 ROS 运行环境）
ros2 run combined_calibrate solve_extrinsic ~/dart_calibration/shots

# 验证：把点云投到图像上，发布 ~/overlay 和 ~/points_camera
ros2 run combined_calibrate project_cloud --ros-args \
    -p extrinsics:=$HOME/dart_calibration/shots/extrinsics.yaml
```

`solve_extrinsic` 的选项：`--output`、`--intrinsics`（用别的内参文件，比如驱动的
`camera_info.yaml`）、`--no-outlier-rejection`、`--no-fix-roi`。

## 角点顺序

图像角点和雷达角点必须一一对应，顺序就是唯一的对应依据，标错必错。默认顺序
`左上 -> 右上 -> 右下 -> 左下`（`corner_labels` 可改）。`corners_preview.png`
是核对图像角点用的。

## 数据格式

```text
shot_000/
├── cloud.pcd         累积点云（默认二进制 PCD，rviz/CloudCompare/PCL 都能读）
├── image.png         与该点云同时刻的相机图像
├── intrinsics.yaml   相机内参（已按实际图像尺寸修正，见下）
├── corners_img.txt   每行 "<u> <v>"，顺序即角点顺序
├── corners_lidar.txt 每行 "<x> <y> <z>"，顺序必须与上面一致
└── corners_preview.png  图像角点标注图
```

`extrinsics.yaml` 沿用旧版字段，可以直接喂给镖架工程的外参链：

```yaml
T_lidar_to_camera: {rows: 4, cols: 4, data: [...]}   # p_camera = R * p_lidar + t
T_camera_to_lidar: {rows: 4, cols: 4, data: [...]}
translation_m: [...]
rotation_rpy_deg: [...]                              # ZYX
mean_reprojection_error_px: ...
shots: [...]
per_shot_error_px: [...]
outlier_shots: [...]
```

接到镖架里依旧按链复合：

```text
T_camera_to_world = T_muzzle_to_world @ T_gimbal_to_muzzle
                  @ T_lidar_to_gimbal @ T_camera_to_lidar
```

## 关于内参（本机相机的一个坑）

`hik_camera_ros2_driver` 的 `camera_info.yaml` 是按 **3072x2048 全传感器** 标的
（`cx≈1641`），而 `camera_params.yaml` 里 `width: 1536, height: 960, offset_x: 768,
offset_y: 544` 是居中 ROI。直接把这份 `camera_info` 当内参用会错到天上。

节点会自动识别这种情况：图像尺寸和 `camera_info` 声明的尺寸不一致时，按“居中 ROI”
移动主点（`cx -= (info_w - img_w)/2`），并打一条 WARN 说明新旧数值，
写进 `shot_*/intrinsics.yaml` 的就是修正后的内参。想关掉用 `fix_roi_intrinsics:=false`；
如果相机实际出图确实不是 3072x2048，建议直接按实际分辨率重新标一次内参，
再用 `--intrinsics` 传给 `solve_extrinsic`。

## 自检（不需要硬件）

```bash
python3 test/synthetic_check.py        # 合成 5 个位姿 -> 求解 -> 和真值比对
python3 test/ros_pipeline_check.py     # 合成话题 -> capture -> solve -> 投影
python3 test/launch_check.py <shots>   # 单条 launch 能否拉起验证节点
```

三个脚本都自己找 `combined_calibrate` 的可执行文件（`PATH` → `COMBINED_CALIBRATE_PREFIX`
→ `AMENT_PREFIX_PATH` → `ros2 pkg prefix`），只要先 `source install/setup.bash`，
装在哪都行；也可以用 `COMBINED_CALIBRATE_PREFIX=/path/to/install` 显式指定。

三个脚本都用本机 apt 的 OpenCV/numpy 1.x。**不要 `pip install --user -U numpy`**：
user site 的 numpy 2.x 会让所有 cv2 调用报 “src is not a numpy array”，
需要升级就用 `pip install --user "numpy<2"`。

## 编译

```bash
cd ~/Dart_ROS2_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select combined_calibrate
source install/setup.bash
```

CMake 里显式把 `OpenCV_DIR` / `PCL_DIR` 指到 `/usr/lib/x86_64-linux-gnu/cmake/...`：
本机 `/usr/local` 还有一套 OpenCV 4.12 和 PCL 1.15，而 `cv_bridge` 和 ROS 都是按
apt 的 OpenCV 4.5 / PCL 1.12 编的，混用会在运行时炸。

## 依赖

`rclcpp`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`cv_bridge`、OpenCV(calib3d/highgui)、
PCL(io/common)，运行验证还需要 `rviz2`。驱动：`livox_ros2_driver`、`hik_camera_ros2_driver`。

## 常见问题

- **一直显示“等待传感器数据”**：`ros2 topic hz /livox/lidar`、`ros2 topic hz /image_raw`
  看驱动是否在发；雷达必须用 `xfer_format: 0`，`livox_lidar_msg_launch.py` 发的是
  `CustomMsg`，节点收不到。启动顺序上先有驱动再有本节点也没问题（订阅是懒启动的）。
- **图像窗口是黑的/没有窗口**：确认 `DISPLAY` 有值；无显示器时用
  `interactive:=false` 并在别的机器上用 `pick_corners` 补角点。
- **终端按回车没反应**：`ros2 launch` 用了 `emulate_tty`，如果在某些终端里仍然无效，
  直接 `ros2 run combined_calibrate calibrate_node --ros-args -p mode:=guided` 单跑节点。
- **求解报“no usable shot”**：`shot_*` 目录必须同时有 `corners_img.txt` 和
  `corners_lidar.txt`，两个文件行数必须一致。
- **重投影误差 > 2 px**：多半是某个角点点偏了（尤其雷达侧），或者靶标在累积期间动了。
  `extrinsics.yaml` 里的 `per_shot_error_px` 可以看出是哪一组。
