## 1. 构建逻辑

代码分成三层：

1. `Detector`
   - 输入：`cv::Mat` 类型的 `CV_8UC1` 灰度图。
   - 输出：候选引导灯的几何特征 `std::vector<BaseLight>`。
   - 只负责 OpenCV 图像处理，不依赖 ROS，便于单元测试和离线调参。

2. `DetectorNode`
   - 订阅 ROS2 图像和相机内参。
   - 调用 `Detector`。
   - 用 55 mm 的物理直径和 25 m 的标称距离选择最符合尺寸的候选。
   - 发布候选调试消息、选中目标消息和叠加结果图像。

3. 组件注册和自动入口
   - `detector_node.cpp` 末尾的注册宏导出 `rm_auto_aim::DetectorNode`。
   - `CMakeLists.txt` 中的 `rclcpp_components_register_node` 会自动生成普通 ROS2 可执行程序
     `detector_node`，不再需要手写 `main.cpp`。
   - 因此当前既可以用 `ros2 run detector detector_node` 启动，也可以在组件容器中加载
     `rm_auto_aim::DetectorNode`。

完整的每帧处理顺序是：

```text
sensor_msgs/msg/Image
        |
        v
cv_bridge 转换为 mono8
        |
        v
高斯滤波 -> 灰度阈值 -> 开运算 -> 闭运算
        |
        v
提取外部轮廓
        |
        v
面积、圆度、椭圆填充率、长宽比、像素直径筛选
        |
        v
候选按 score 降序排列
        |
        v
根据 55 mm / 25 m 的期望像素直径选择基地灯
        |
        +--> detector/debug_lights
        +--> detector/lights
        `--> detector/result_image
```

## 2. 几何和距离计算

相机内参来自 `sensor_msgs/msg/CameraInfo.k`：

```text
fx = k[0]    fy = k[4]
cx = k[2]    cy = k[5]
```

已知引导灯实际直径 `D`、图像中的像素直径 `d` 和水平焦距 `fx` 时，使用针孔模型估算
相机到灯的深度：

```text
Z = D * fx / d
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

其中：

- `D` 是 `physical_diameter_m`，当前为 `0.055 m`。
- `d` 是候选轮廓的最小包围圆直径 `diameter_px`。
- `(u, v)` 是候选中心像素坐标 `center`。
- `(X, Y, Z)` 写入 `Light.pose.position`，坐标轴方向遵循相机坐标系约定。

期望像素直径为：

```text
d_expected = fx * D / expected_distance_m
```

选择目标时，候选自身的形状分数为：

```text
score = circularity * fill_ratio
```

然后根据候选直径与 `d_expected` 的对数误差进行惩罚。这样滑台左右随机移动时，
算法不依赖固定的水平像素位置；如果画面里出现其他圆形亮点，已知尺寸可以提供额外判据。

注意：如果没有收到有效 `CameraInfo`，节点仍会发布候选和选中目标，但距离退化为
`expected_distance_m`，横向和纵向位置不会根据相机内参计算。比赛使用前必须确认相机
标定文件有效。

## 3. detector 参数

默认参数文件由启动包统一管理，路径为：

```text
src/rm_vision/rm_vision_bringup/config/detector_params.yaml
```

参数文件使用 `/**`，因此可以同时适用于独立启动和 `/dart` 命名空间启动。ROS 参数名、
默认值和作用如下。

### 3.1 话题参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `image_topic` | `image_raw` | 输入图像话题。相对话题名会自动附加节点命名空间。 |
| `camera_info_topic` | `image_raw/camera_info` | 相机内参话题。Hik 驱动的 `camera_topic` 设置为 `image_raw` 后，内参话题为该层级。 |
| `debug_lights_topic` | `detector/debug_lights` | 所有候选灯的调试消息话题。 |
| `lights_topic` | `detector/lights` | 选中的基地灯话题；没有选中目标时发布空数组。 |
| `result_image_topic` | `detector/result_image` | 叠加轮廓、中心和 `BASE` 标记的 BGR 图像话题。 |

在总启动文件中使用 `/dart` 命名空间时，实际默认话题为：

```text
/dart/image_raw
/dart/image_raw/camera_info
/dart/detector/debug_lights
/dart/detector/lights
/dart/detector/result_image
```

### 3.2 目标物理参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `physical_diameter_m` | `0.055` | 引导灯实际直径，55 mm。 |
| `expected_distance_m` | `25.0` | 目标选择的标称距离，单位 m。不是每帧强制距离。 |
| `expected_diameter_tolerance` | `0.60` | 期望像素直径惩罚的容忍系数，越大越不强调尺寸匹配。代码内部最小按 `0.05` 处理。 |

`expected_distance_m` 用于目标选择和没有标定信息时的距离回退。
收到有效内参后，最终发布的 `Light.distance` 会优先使用当前帧的像素直径估计。

### 3.3 灰度预处理参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `threshold` | `180` | 二值化阈值。灰度值大于该值的区域成为白色候选区域。 |
| `blur_kernel_size` | `3` | 高斯滤波核边长。小于等于 1 表示不滤波；偶数会自动加 1。 |
| `open_kernel_size` | `3` | 形态学开运算核边长，用于去除小噪声。 |
| `close_kernel_size` | `3` | 形态学闭运算核边长，用于填补小孔和连接近邻区域。 |

调参时通常先调整 `threshold`，再决定是否增大形态学核。核过大可能把小灯抹掉，
也可能把相邻亮点连接成一个不规则大轮廓。

### 3.4 候选轮廓参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `min_area` | `4.0` | 轮廓最小面积，单位像素平方。 |
| `max_area` | `5000.0` | 轮廓最大面积，单位像素平方。 |
| `min_diameter_px` | `3.0` | 最小包围圆直径，单位像素。 |
| `max_diameter_px` | `200.0` | 最大包围圆直径，单位像素。 |
| `min_circularity` | `0.50` | 最小圆度，计算式为 `4*pi*area/perimeter^2`，圆越接近 1。 |
| `min_fill_ratio` | `0.40` | 轮廓面积与拟合椭圆面积之比，用于排除空心或破碎亮斑。 |
| `max_aspect_ratio` | `2.0` | 最小外接矩形长边与短边的最大比值，用于排除长条形目标。 |

真实图像调参推荐顺序：

1. 先保证灯能通过 `threshold` 形成完整白色区域。
2. 观察 `result_image` 中是否出现橙色候选椭圆。
3. 用灯的像素直径设置 `min_diameter_px` 和 `max_diameter_px`。
4. 再提高 `min_circularity`、`min_fill_ratio` 或降低 `max_aspect_ratio`，减少误检。

### 3.5 ROI 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `roi_x_min` | `0.0` | ROI 左边界，占图像宽度的比例。 |
| `roi_x_max` | `1.0` | ROI 右边界，占图像宽度的比例。 |
| `roi_y_min` | `0.0` | ROI 上边界，占图像高度的比例。 |
| `roi_y_max` | `1.0` | ROI 下边界，占图像高度的比例。 |

例如只检测图像上半部分，可以设置 `roi_y_max: 0.5`。ROI 目前只检查候选中心，
不是裁剪图像，因此边界附近的轮廓仍可能有一部分落在 ROI 外。

## 4. C++ 结构体和变量

### `BaseLight`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `center` | `cv::Point2f` | 候选轮廓质心，单位像素。 |
| `ellipse` | `cv::RotatedRect` | 对候选轮廓拟合出的旋转椭圆，用于调试绘制和填充率计算。 |
| `diameter_px` | `float` | 最小包围圆直径，单位像素。 |
| `area_px` | `float` | 原始轮廓面积，单位像素平方。 |
| `circularity` | `float` | 轮廓圆度。 |
| `fill_ratio` | `float` | 轮廓面积占拟合椭圆面积的比例。 |
| `score` | `float` | `circularity * fill_ratio`，用于候选排序。 |

### `Detector::Params`

`Params` 是纯 OpenCV 检测器的配置快照。节点在构造时读取 ROS 参数并一次性填入，
后续每帧检测使用同一份配置。当前没有动态参数回调，修改 YAML 后需要重启节点。

### `DetectorNode` 成员变量

| 变量 | 含义 |
| --- | --- |
| `detector_` | OpenCV 检测核心，负责输出候选列表。 |
| `physical_diameter_m_` | 引导灯实际直径。 |
| `expected_distance_m_` | 标称目标距离和无内参时的回退距离。 |
| `expected_diameter_tolerance_` | 尺寸匹配惩罚的容忍系数。 |
| `fx_`, `fy_` | 相机水平、垂直焦距，单位像素。 |
| `cx_`, `cy_` | 相机主点坐标，单位像素。 |
| `camera_info_received_` | 是否已经收到有效的 `CameraInfo`。 |
| `image_sub_` | 输入 `sensor_msgs/msg/Image` 订阅器。 |
| `camera_info_sub_` | 输入 `sensor_msgs/msg/CameraInfo` 订阅器。 |
| `debug_lights_pub_` | 候选调试消息发布器。 |
| `lights_pub_` | 选中基地灯消息发布器。 |
| `result_image_pub_` | 叠加图像发布器。 |

关键回调和函数：

- `cameraInfoCallback`：检查 `fx`、`fy` 是否为正数，并缓存 `fx/fy/cx/cy`。
- `imageCallback`：完成图像转换、检测、目标选择和三个输出发布。
- `declareDetectorParams`：集中声明所有 OpenCV 检测参数。
- `selectBaseLight`：结合形状分数和物理尺寸选择一个候选。
- `publishLightMessages`：填充 `DebugLights` 和 `Lights`。
- `publishResultImage`：把候选画成橙色，把选中目标画成绿色并写入 `BASE`。
- `kNoSelection`：`size_t` 最大值，表示当前帧没有选中目标。

## 5. 相机 mono8 链路

`hik_camera_ros2_driver` 现在按以下规则工作：

- 相机像素格式参数默认设置为 `Mono8`。
- SDK 转换目标设置为 `PixelType_Gvsp_Mono8`。
- 每帧先根据实际宽高调整 `image_msg_.data`，再把其指针传给 SDK。
- ROS 图像字段固定为：
  - `encoding = "mono8"`
  - `step = width`
  - `data.size() = width * height`

这三项必须同时成立。只修改 `encoding` 而不修改缓冲区长度，会造成 SDK 写入越界或转换失败；
只修改相机像素格式而不修改 ROS 消息，会导致 `cv_bridge` 收到错误编码。

相机配置文件为 `hik_camera_ros2_driver/config/camera_params.yaml`，主要参数如下：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `camera_info_url` | `package://hik_camera_ros2_driver/config/camera_info.yaml` | 相机标定文件地址，决定 `CameraInfo.k` 的有效性。 |
| `pixel_format` | `Mono8` | 请求 Hik SDK 输出的相机像素格式。 |
| `adc_bit_depth` | `Bits_8`（代码默认值） | 相机 ADC 位深；Mono8 相机使用 8 bit。 |
| `use_sensor_data_qos` | `false` | 是否使用 ROS 传感器数据 QoS 发布图像。 |
| `camera_name` | `camera` | 相机实例名，参与默认 frame 名称生成。 |
| `frame_id` | `camera_optical_frame` | 图像和相机内参的坐标系名称。 |
| `camera_topic` | `image_raw` | image_transport 相机图像根话题。 |
| `acquisition_frame_rate` | `249.1` | 相机采集帧率，单位 Hz，最终受硬件范围限制。 |
| `exposure_time` | `3000` | 曝光时间，单位 us。 |
| `gain` | `6.0` | 模拟增益，单位 dB，过大可能放大背景噪声。 |

`camera_name`、`frame_id` 和 `camera_topic` 都显式配置，是为了避免驱动内部默认拼接
出意外话题名。当前总启动的实际输入链路为：

```text
/dart/camera_node
  -- image_transport camera_topic: /dart/image_raw
  -- camera info:             /dart/image_raw/camera_info
                         |
                         v
/dart/detector
```

## 6. 启动方式

只启动检测节点：

```bash
ros2 launch rm_vision_bringup detector.launch.py
```

启动相机和检测节点：

```bash
ros2 launch rm_vision_bringup dart_bringup.launch.py
```

直接运行检测节点：

```bash
ros2 run detector detector_node --ros-args --params-file \
  $(ros2 pkg prefix rm_vision_bringup)/share/rm_vision_bringup/config/detector_params.yaml
```

当前 `rm_vision_bringup` 已删除 tracker、串口、自动录包和旧组件容器配置。
这些功能与本阶段基地灯识别没有直接关系，并且原启动文件引用了当前工作区不存在的包。

## 7. 消息接口说明

消息定义位于 `auto_aim_interfaces/msg`。可以按数据流分成三组。

### 7.1 `Light` 和 `Lights`：给下游使用的检测结果

`Light.msg` 表示一个已经选中的有效灯目标：

```text
geometry_msgs/Pose pose
float32 distance
```

- `pose.position.x/y/z`：目标在相机坐标系中的近似三维位置，单位 m。
- `pose.orientation`：当前单个圆形灯没有可观测的可靠朝向，代码只设置单位四元数。
- `distance`：根据已知直径、焦距和像素直径估算的距离，单位 m。

`Lights.msg` 是多个 `Light` 的帧级容器：

```text
std_msgs/Header header
Light[] lights
```

- `header.stamp`：当前图像结果的时间戳。
- `header.frame_id`：当前结果所属坐标系，当前来自相机图像消息。
- `lights`：有效目标数组。

本阶段约定 `detector/lights` 中最多发布一个元素：

```text
lights.size() == 0：当前没有有效基地引导灯
lights.size() == 1：当前选中的元素就是基地引导灯
```

### 7.2 `DebugLight` 和 `DebugLights`：给开发调试使用的候选结果

`DebugLight.msg` 描述一个通过轮廓筛选的候选亮斑：

```text
geometry_msgs/Pose2D[] corners
uint32 contour_area
uint16 contour_width
uint16 contour_height
uint16 ellipse_width
uint16 ellipse_height
uint32 ellipse_area
geometry_msgs/Pose2D ellipse_center
float32 filled
```

- `corners`：当前代码填入椭圆外接矩形的左上角和右下角两个点，并不是完整四角数组。
- `contour_area`：轮廓面积，单位像素平方。
- `contour_width/height`：外接矩形尺寸，单位像素。
- `ellipse_width/height`：拟合椭圆尺寸，单位像素。
- `ellipse_area`：当前代码使用椭圆宽度乘高度的近似量，不是严格的
  `pi * width * height / 4`。
- `ellipse_center`：候选中心，单位像素。
- `filled`：轮廓面积与拟合椭圆面积之比。

`DebugLights.msg` 是候选数组：

```text
DebugLight[] data
```

`DebugLight` 和 `Light` 的核心区别是：

| 项目 | `DebugLight` | `Light` |
| --- | --- | --- |
| 表示对象 | 一个候选亮斑 | 一个最终选中的目标 |
| 数据维度 | 二维图像几何信息 | 三维近似位置和距离 |
| 主要用途 | 调阈值、分析误检、可视化 | 瞄准、控制、后续跟踪 |
| 是否一定是基地灯 | 不一定 | 当前约定是 |
| 是否包含距离 | 否 | 是 |

当前 `DebugLights` 没有 `Header`、`selected` 和 `confidence` 字段，所以不能单独从
消息中判断某个候选是否被选中。开发时应结合 `detector/result_image`：绿色椭圆和
`BASE` 文本表示选中目标，橙色椭圆表示普通候选。

### 7.3 `Target`：跟踪器输出

```text
std_msgs/Header header
bool tracking
string id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 dt
```

用于表达经过多帧跟踪后的目标状态：

- `tracking`：当前是否跟踪有效。
- `id`：目标标识字符串。
- `position`：目标位置。
- `velocity`：目标速度。
- `dt`：状态更新间隔。

第一阶段没有跟踪器节点，因此当前未使用。

### 7.4 `TrackerInfo`：跟踪器内部调试信息

```text
float64 position_diff
float64 yaw_diff
geometry_msgs/Point position
float64 yaw
```

用于分析滤波预测和观测之间的差异：

- `position_diff`：位置预测误差。
- `yaw_diff`：角度预测误差。
- `position` 和 `yaw`：未滤波观测值。

第一阶段当前未使用。

### 7.5 `TimeInfo`：额外时间信息

```text
std_msgs/Header header
int64 time
```

可以用于相机、串口和控制模块之间的时间传递，但 `time` 的单位目前没有在接口中定义。
后续必须统一为微秒、毫秒或纳秒中的一种，避免时间补偿错误。

## 8. 当前限制和后续接口缺口

- `mono8` 图像本身无法验证颜色是绿色。如果滤光片不能排除其他颜色的高亮目标，需要增加
  彩色图像、双通道滤光片响应或其他场景先验。
- `Lights.msg` 没有 `type`、`confidence` 和像素中心字段，目前通过 `lights` 数组中唯一元素
  表示基地灯。
- `DebugLights.msg` 没有 header、时间戳和 selected 标志，调试消息无法独立判断哪一个候选
  被选中，只能结合结果图或数组排序观察。
- 当前只发布相机坐标系下的几何位置，还没有 TF 外参转换到云台、底盘或世界坐标系。
- 当前未实现激光雷达重定位、时间同步、滑台状态融合和多帧跟踪。
- 真实比赛图像仍需要重新测量 25 m 处的像素直径并调整阈值、ROI 和候选过滤参数。
