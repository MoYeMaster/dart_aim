# 飞行飞镖三维检测

`dart_detector_node` 订阅 `image_raw` 和 `camera_info`，通过现有轮廓及颜色筛选获取旋转矩形，复用 detector 包的 `rm_auto_aim::PnpSolver` 通用平面点对求解接口，再按图像时间戳通过 TF2 将相机光学坐标转换到 `target_frame`（默认 `muzzle_link`）。发布 `dart_detector/detections`，`position` 单位为米，消息帧为目标坐标系，像素字段仍对应原图。

启动配置位于 `rm_vision_bringup/config/dart_detector_params.yaml`，已加入 `dart_bringup.launch.py`。必须将 `dart_length_m` 和 `dart_width_m` 设置为被检测彩色区域对应的实际长边、短边尺寸（米），不能使用整支飞镖的尺寸替代局部彩色区域尺寸。默认零值表示尚未配置，此时发布空检测列表。

四个物点采用平面矩形模型，与 `minAreaRect` 四角对应；这些角点是轮廓包围盒而非真实物体关键点，因此三维位置是近似估计，姿态、透视缩短、运动模糊和目标形变都会影响精度。需要用实拍数据校验后再用于控制。

图像与 CameraInfo 必须使用相同光学帧和分辨率，并提供有效内参及 OpenCV 支持的畸变模型。启动文件将相机帧统一为 URDF 中的 `camera_optical_frame`。TF 必须提供图像时刻从该光学帧到枪口的变换；URDF 中的相机和枪口安装位置需要按实测外参配置。缺少标定、尺寸或 TF 时发布空列表；PnP 失败的候选不发布。

本改动仅新增三维检测输出。现有飞镖 tracker 的像素滤波不会自动变成三维跟踪。

当前相机配置文件的采集尺寸为 1536×960，标定文件尺寸为 3072×2048，两者不匹配。使用该采集配置前，需要提供匹配实际输出图像的标定文件；若使用传感器 ROI 裁剪，主点还需要减去裁剪偏移，不能仅按比例缩放内参。
