#include "detector/detector_node.hpp"

#include "detector/dart_state.hpp"
#include "detector/image_utils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace rm_auto_aim
{
namespace
{
constexpr std::size_t kNoSelection = std::numeric_limits<std::size_t>::max();

void fillDebugLight(const BaseLight & detection, auto_aim_interfaces::msg::DebugLight & debug_light)
{
  const cv::Rect bounding_box = detection.ellipse.boundingRect();
  geometry_msgs::msg::Pose2D point;
  point.x = bounding_box.x;
  point.y = bounding_box.y;
  debug_light.corners.push_back(point);
  point.x = bounding_box.x + bounding_box.width;
  point.y = bounding_box.y + bounding_box.height;
  debug_light.corners.push_back(point);
  debug_light.contour_area = static_cast<uint32_t>(std::max(0.0F, detection.area_px));
  debug_light.contour_width = static_cast<uint16_t>(std::max(0, bounding_box.width));
  debug_light.contour_height = static_cast<uint16_t>(std::max(0, bounding_box.height));
  debug_light.ellipse_width =
    static_cast<uint16_t>(std::max(0.0F, detection.ellipse.size.width));
  debug_light.ellipse_height =
    static_cast<uint16_t>(std::max(0.0F, detection.ellipse.size.height));
  debug_light.ellipse_area =
    static_cast<uint32_t>(std::max(
      0.0F, detection.ellipse.size.width * detection.ellipse.size.height));
  debug_light.ellipse_center.x = detection.center.x;
  debug_light.ellipse_center.y = detection.center.y;
  debug_light.filled = detection.fill_ratio;
}

std::string distanceLabel(const PnpResult & result)
{
  if (!result.valid || !std::isfinite(result.distance_m) || result.distance_m <= 0.0) {
    return "Distance: N/A";
  }

  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "Distance: %.2f m", result.distance_m);
  return buffer;
}

void drawDistanceLabel(cv::Mat & image, const BaseLight & detection, const PnpResult & result)
{
  const std::string label = distanceLabel(result);
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double kFontScale = 0.5;
  constexpr int kThickness = 1;
  int baseline = 0;
  const cv::Size text_size =
    cv::getTextSize(label, font, kFontScale, kThickness, &baseline);
  const cv::Rect box = detection.ellipse.boundingRect();
  const int x = std::clamp(box.x, 0, std::max(0, image.cols - text_size.width));
  const int y = std::clamp(
    box.y + box.height + text_size.height + baseline + 4,
    text_size.height + baseline, image.rows - 2);

  cv::putText(
    image, label, cv::Point(x, y), font, kFontScale, cv::Scalar(255, 255, 0),
    kThickness, cv::LINE_AA);
}
}  // namespace

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("detector", options),
  detector_(declareDetectorParams()),
  pnp_solver_(0.055),
  ballistic_calculator_(declareBallisticParams())
{
  physical_diameter_m_ = declare_parameter("physical_diameter_m", 0.055);
  expected_distance_m_ = declare_parameter("expected_distance_m", 25.0);
  expected_diameter_tolerance_ = declare_parameter("expected_diameter_tolerance", 0.60);
  camera_type_ = declare_parameter("camera_type", 0);
  middle_error_ratio_ = declare_parameter("middle_error_ratio", 0.03);
  pnp_solver_.setTargetDiameter(physical_diameter_m_);

  const auto image_topic = declare_parameter("image_topic", "image_raw");
  const auto camera_info_topic = declare_parameter("camera_info_topic", "camera_info");
  const auto debug_lights_topic =
    declare_parameter("debug_lights_topic", "detector/debug_lights");
  const auto lights_topic = declare_parameter("lights_topic", "detector/lights");
  const auto dart_states_topic = declare_parameter("dart_states_topic", "detector/dart_states");
  const auto result_image_topic =
    declare_parameter("result_image_topic", "detector/result_image");

  debug_lights_pub_ =
    create_publisher<auto_aim_interfaces::msg::DebugLights>(debug_lights_topic, 10);
  lights_pub_ =
    create_publisher<auto_aim_interfaces::msg::Lights>(lights_topic, rclcpp::SensorDataQoS());
  dart_states_pub_ =
    create_publisher<auto_aim_interfaces::msg::DartStates>(dart_states_topic, 10);
  result_image_pub_ = image_transport::create_publisher(this, result_image_topic);

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::cameraInfoCallback, this, std::placeholders::_1));
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Detector started: camera_type=%d, physical diameter %.3f m, expected distance %.1f m",
    camera_type_, physical_diameter_m_, expected_distance_m_);
}

Detector::Params DetectorNode::declareDetectorParams()
{
  Detector::Params params;
  params.threshold = declare_parameter("threshold", params.threshold);
  params.blur_kernel_size = declare_parameter("blur_kernel_size", params.blur_kernel_size);
  params.open_kernel_size = declare_parameter("open_kernel_size", params.open_kernel_size);
  params.close_kernel_size = declare_parameter("close_kernel_size", params.close_kernel_size);
  params.min_area = declare_parameter("min_area", params.min_area);
  params.max_area = declare_parameter("max_area", params.max_area);
  params.min_diameter_px = declare_parameter("min_diameter_px", params.min_diameter_px);
  params.max_diameter_px = declare_parameter("max_diameter_px", params.max_diameter_px);
  params.min_circularity = declare_parameter("min_circularity", params.min_circularity);
  params.min_fill_ratio = declare_parameter("min_fill_ratio", params.min_fill_ratio);
  params.max_aspect_ratio = declare_parameter("max_aspect_ratio", params.max_aspect_ratio);
  params.roi_x_min = declare_parameter("roi_x_min", params.roi_x_min);
  params.roi_x_max = declare_parameter("roi_x_max", params.roi_x_max);
  params.roi_y_min = declare_parameter("roi_y_min", params.roi_y_min);
  params.roi_y_max = declare_parameter("roi_y_max", params.roi_y_max);
  return params;
}

BallisticParams DetectorNode::declareBallisticParams()
{
  BallisticParams params;
  params.dart_mass_kg = declare_parameter("dart_mass_kg", params.dart_mass_kg);
  params.cross_section_area_m2 =
    declare_parameter("dart_cross_section_area_m2", params.cross_section_area_m2);
  params.drag_coefficient = declare_parameter("drag_coefficient", params.drag_coefficient);
  params.air_density_kg_m3 =
    declare_parameter("air_density_kg_m3", params.air_density_kg_m3);
  params.target_height_difference_m =
    declare_parameter("target_height_difference_m", params.target_height_difference_m);
  params.launch_angle_deg = declare_parameter("camera_pitch_deg", params.launch_angle_deg);
  params.spring_effective_stroke_m =
    declare_parameter("spring_effective_stroke_m", params.spring_effective_stroke_m);
  params.launch_efficiency =
    declare_parameter("launch_efficiency", params.launch_efficiency);
  params.gravity_m_s2 = declare_parameter("gravity_m_s2", params.gravity_m_s2);
  params.integration_time_step_s =
    declare_parameter("ballistic_integration_time_step_s", params.integration_time_step_s);
  params.max_initial_speed_mps =
    declare_parameter("ballistic_max_initial_speed_mps", params.max_initial_speed_mps);
  return params;
}

void DetectorNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (camera_info_msg->k[0] <= 0.0 || camera_info_msg->k[4] <= 0.0 ||
    camera_info_msg->k[8] <= 0.0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid camera intrinsics");
    return;
  }

  fx_ = camera_info_msg->k[0];
  fy_ = camera_info_msg->k[4];
  cx_ = camera_info_msg->k[2];
  cy_ = camera_info_msg->k[5];
  camera_info_received_ = true;

  std::array<double, 9> camera_matrix{};
  std::copy(camera_info_msg->k.begin(), camera_info_msg->k.end(), camera_matrix.begin());
  pnp_solver_.setCameraInfo(camera_matrix, camera_info_msg->d);
}

std::size_t DetectorNode::selectBaseLight(const std::vector<BaseLight> & detections) const
{
  if (detections.empty()) {
    return kNoSelection;
  }

  double expected_diameter_px = 0.0;
  if (camera_info_received_ && expected_distance_m_ > 0.0) {
    expected_diameter_px = fx_ * physical_diameter_m_ / expected_distance_m_;
  }

  std::size_t best_index = 0;
  double best_score = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto & detection = detections[i];
    double score = detection.score;
    if (expected_diameter_px > 0.0 && detection.diameter_px > 0.0) {
      const double diameter_ratio = detection.diameter_px / expected_diameter_px;
      const double tolerance = std::max(0.05, expected_diameter_tolerance_);
      const double size_error = std::abs(std::log(diameter_ratio));
      score -= size_error / tolerance;
    }
    if (score > best_score) {
      best_score = score;
      best_index = i;
    }
  }
  return best_index;
}

bool DetectorNode::estimateLight(const BaseLight & detection, PnpResult & result) const
{
  result = PnpResult{};
  if (!camera_info_received_) {
    return false;
  }

  if (pnp_solver_.solve(detection, result)) {
    return true;
  }

  const double distance_m = pnp_solver_.estimateDistance(detection.diameter_px);
  if (distance_m <= 0.0 || fx_ <= 0.0 || fy_ <= 0.0) {
    return false;
  }

  result.valid = true;
  result.distance_m = distance_m;
  result.position_z_m = distance_m;
  result.position_x_m = (detection.center.x - cx_) * distance_m / fx_;
  result.position_y_m = (detection.center.y - cy_) * distance_m / fy_;
  return true;
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  cv::Mat img;
  try {
    img = makeDetectorInputImage(*image_msg, camera_type_);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to convert image for detector: %s", ex.what());
    return;
  }

  const auto detections = detector_.detect(img);
  const std::size_t selected_index = selectBaseLight(detections);

  publishLightMessages(image_msg->header, detections, selected_index);
  publishDartStates(img, detections, selected_index);
  publishResultImage(img, image_msg->header, detections, selected_index);
}

void DetectorNode::publishLightMessages(
  const std_msgs::msg::Header & header, const std::vector<BaseLight> & detections,
  std::size_t selected_index)
{
  auto_aim_interfaces::msg::DebugLights debug_msg;
  for (const auto & detection : detections) {
    auto_aim_interfaces::msg::DebugLight debug_light;
    fillDebugLight(detection, debug_light);
    debug_msg.data.push_back(debug_light);
  }
  debug_lights_pub_->publish(debug_msg);

  auto_aim_interfaces::msg::Lights lights_msg;
  lights_msg.header = header;
  if (selected_index != kNoSelection && selected_index < detections.size()) {
    const auto & detection = detections[selected_index];
    PnpResult result;
    if (estimateLight(detection, result)) {
      auto_aim_interfaces::msg::Light light_msg;
      light_msg.pose.position.x = result.position_x_m;
      light_msg.pose.position.y = result.position_y_m;
      light_msg.pose.position.z = result.position_z_m;
      light_msg.pose.orientation.w = 1.0;
      light_msg.distance = static_cast<float>(result.distance_m);
      lights_msg.lights.push_back(light_msg);
    }
  }
  lights_pub_->publish(lights_msg);
}

void DetectorNode::publishDartStates(
  const cv::Mat & detector_image, const std::vector<BaseLight> & detections,
  std::size_t selected_index)
{
  auto_aim_interfaces::msg::DartStates dart_states_pub;
  if (selected_index != kNoSelection && selected_index < detections.size()) {
    const auto & detection = detections[selected_index];
    const auto state = classifyDartState(
      detection.center.x, static_cast<double>(detector_image.cols), middle_error_ratio_);
    dart_states_pub.states = state.states;
    dart_states_pub.is_middle = state.is_middle;
    dart_states_pub.dyaw = 0.0;

    PnpResult result;
    if (estimateLight(detection, result)) {
      dart_states_pub.force =
        ballistic_calculator_.calculateRequiredForce(result.distance_m);
    }
  }
  dart_states_pub_->publish(dart_states_pub);
}

void DetectorNode::publishResultImage(
  const cv::Mat & detector_image, const std_msgs::msg::Header & header,
  const std::vector<BaseLight> & detections, std::size_t selected_index)
{
  cv_bridge::CvImage result_image;
  result_image.header = header;
  result_image.encoding = "bgr8";
  cv::cvtColor(detector_image, result_image.image, cv::COLOR_GRAY2BGR);

  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto & detection = detections[i];
    const bool selected = i == selected_index;
    const cv::Scalar color = selected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
    PnpResult result;
    estimateLight(detection, result);

    cv::ellipse(result_image.image, detection.ellipse, color, selected ? 2 : 1);
    cv::circle(result_image.image, detection.center, 2, color, cv::FILLED);
    drawDistanceLabel(result_image.image, detection, result);
    if (selected) {
      cv::putText(
        result_image.image, "BASE", detection.center + cv::Point2f(6.0F, -6.0F),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
  }

  result_image_pub_.publish(*result_image.toImageMsg());
}
}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::DetectorNode)
