# lidar_calibrate

只给激光雷达标定，不涉及相机。做法仿照东北大学
[T-DT_Radar](https://github.com/T-DT-Algorithm-2026/T-DT_Radar) 的雷达定位节点：
把当前点云配准到一张参考场地点云上，直接得到镖架在场地坐标系里的位姿。

T-DT 的定位逻辑与本包一致，逐条对应如下：

| T-DT `localization.cpp` | 本包 |
| --- | --- |
| 累积 20 帧点云 | `accumulate_frames` |
| 0.1°×0.1° 网格取每格最远点 | `lidar_calibrate::gridFarthestPoint` |
| 裁剪 x(5,30) y(-10,8) z<7 | `roi.*` |
| VoxelGrid 0.1 m 下采样 | `leaf_size` |
| GICP 配准 `RM2026.pcd`，fitness<0.2 | `map_pcd` + `fitness_threshold` |
| 发布 `rm_frame -> livox_frame` 的 TF | 同名 TF，可用 `map_frame`/`lidar_frame` 改 |

差别只在参数化：阈值、ROI、话题名、输出文件都是 ROS 2 参数，配在
`config/lidar_calibrate.yaml` 里。另外本包把 GICP 结果同时写进 YAML，
方便给下位机或弹道解算用。

## 为什么不需要相机

相机标定那套（内参 + 手眼）只是为了把雷达点投到图像上。如果标定目标只是
"镖架在场地里的位姿"，相机完全不参与，也就没有相机和雷达之间的外参误差传播。
T-DT 正是靠这一点把两个传感器彻底解耦。

## 流程

### 1. 准备参考场地点云

包里已经自带东北大学的 `RM2026.pcd`（786487 点，`binary_compressed`，坐标系 `rm_frame`，
范围 x[-0.57, 28.58] y[-0.55, 15.50] z[-0.20, 2.00]），
安装到 `share/lidar_calibrate/config/RM2026.pcd`，`lidar_calibrate.launch.py`
默认就用它，不用额外操作。

只有想换自己的场地时，才需要重新建图：

```bash
ros2 launch lidar_calibrate build_map.launch.py output_dir:=$HOME/dart_calibration
```

节点累积 20 帧、做网格最远点滤波和 ROI 裁剪，把结果写到
`$HOME/dart_calibration/field_map.pcd` 然后自动退出。**这张图就是"场地"的参考，
它所在的坐标系就是最终输出的坐标系**，所以建图时镖架的位姿必须和比赛时一致。

### 2. 标定

```bash
ros2 launch lidar_calibrate lidar_calibrate.launch.py
```

累积 20 帧后做 GICP，实测输出：

```text
[lidar_calibrate_node]: Loaded reference map with 175033 points from .../config/RM2026.pcd
[lidar_calibrate_node]: Initial guess translation [m] 0.000 0.000 0.000
[lidar_calibrate_node]: GICP fitness score: 0.0017 (threshold 0.20)
[lidar_calibrate_node]: Wrote extrinsics to /home/anyi/dart_calibration/lidar_extrinsics.yaml
[lidar_calibrate_node]: translation [m] 0.2290 -0.1504 0.0436, rpy [deg] -0.048 0.010 2.690
[lidar_calibrate_node]: Calibration done, shutting down
```

换场地点云或换输出路径：

```bash
ros2 launch lidar_calibrate lidar_calibrate.launch.py \
    map_pcd:=$HOME/dart_calibration/field_map.pcd \
    output_yaml:=$HOME/dart_calibration/lidar_extrinsics.yaml
```


## 参数

`config/lidar_calibrate.yaml`：

- `roi.*`：场地在雷达系里的范围。**这个和安装位置强相关**，必须按实际安装调整，
  否则网格滤波和配准都会失准。
- `accumulate_frames`：累积帧数。Mid-70 单帧稀疏，建议不低于 20。
- `cell_degrees`：网格角分辨率，0.1° 对应 T-DT 的设置。
- `leaf_size`：体素下采样尺寸，0.1 m。
- `fitness_threshold`：GICP 收敛判定。沿用 T-DT 的 0.2。注意 PCL 的
  `getFitnessScore()` 返回的是对应点距离的**均方值**，所以 0.2 相当于约 0.45 m
  的 RMS，这个阈值很松，配歪了也可能过。**不要只信这个数，一定要在 rviz 里看
  `filtered_cloud` 和 `field_map.pcd` 是否贴合。**
- `initial_guess.*`：GICP 的初值（雷达在场地系里的粗位姿）。GICP 是局部方法，
  初值必须落在正确解的吸引域内，否则会收敛到错误局部极小值。把它设成建图之后
  镖架的标称位姿即可，默认全零（等于假设已经对齐）。
- `stop_after_calibration`：`true` 标定完就退出；`false` 则持续发布 TF，
  适合边跑边看。

## 验证状态

用 `RM2026.pcd` 做的自配准验证：把场地点云按已知位姿变换后当成实时点云发出去，
看能不能解回来。60000 点源点云、`RM2026.pcd` 做目标、初值全零：

| 真值 | 解出 | 误差 |
| --- | --- | --- |
| 平移 `[0.25, -0.18, 0.04]`，yaw 2.865° | `[0.229, -0.150, 0.044]`，yaw 2.690° | 约 3.7 cm，0.18° |
| 平移 `[1.00, -0.80, 0.15]`，yaw 10.0° | `[0.966, -0.758, 0.152]`，yaw 9.714° | 约 4.5 cm，0.29° |

两次 fitness 分别是 0.0017 和 0.0020，单次 GICP 约 9 s。

**注意这是自配准**：源点云是从目标点云里切出来的，真值代价理论上为 0，
所以它验证的是"配准算法在这套场地结构下能不能收敛到正确解"，不等于实车
就是这个精度——实车还有地面外杂点、运动畸变、ROI 没对准等问题。
上线前必须用真实点云再跑一遍。

初值给到 10° / 1.3 m 的偏差仍能收敛，吸引域够用。**但初值全零要求镖架大致
停在 `rm_frame` 原点附近、朝向 +x**，这也正是默认 `roi` 的假设。实际安装位姿
差得远时，调 `roi` 或把 `initial_guess.*` 设成标称位姿。

## 局限

- GICP 是局部方法。偏差过大（例如转了大角度）会收敛到错误局部极小值，
  而 fitness 只是收敛点的值，**配歪了也不会报警**。PCL 的 `getFitnessScore()`
  返回对应点距离的均方值，阈值 0.2 相当于约 0.45 m RMS，很松。
  不要只看这个数，务必在 rviz 里把 `filtered_cloud` 和场地点云叠一起看。
- `roi` 必须按实际安装调，它同时作用于 `map_builder_node` 和 `lidar_calibrate_node`。
- 纯地面主导的 scan 横向约束很弱。场地里如果有立柱、基地、围挡这类竖直结构，
  尽量让它们落在 ROI 内。
- 单次 GICP 约 9 s（175k 目标点）。要更快可以调大 `leaf_size`，或换更小的参考点云。
