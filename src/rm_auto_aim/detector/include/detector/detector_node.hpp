#ifndef DETECTOR__DETECTOR_NODE_HPP_
#define DETECTOR__DETECTOR_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/dart_states.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"
#include "auto_aim_interfaces/msg/lights.hpp"
#include "detector/ballistics.hpp"
#include "detector/detector.hpp"
#include "detector/pnp.hpp"

namespace rm_auto_aim
{

class DetectorNode : public rclcpp::Node
{
public:
  explicit DetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  Detector::Params declareDetectorParams();
  BallisticParams declareBallisticParams();
  std::size_t selectBaseLight(const std::vector<BaseLight> & detections) const;
  bool estimateLight(const BaseLight & detection, PnpResult & result) const;
  void publishLightMessages(
    const std_msgs::msg::Header & header, const std::vector<BaseLight> & detections,
    std::size_t selected_index);
  void publishDartStates(
    const cv::Mat & detector_image, const std::vector<BaseLight> & detections,
    std::size_t selected_index);
  void publishResultImage(
    const cv::Mat & detector_image, const std_msgs::msg::Header & header,
    const std::vector<BaseLight> & detections, std::size_t selected_index);

  Detector detector_;
  PnpSolver pnp_solver_;
  BallisticCalculator ballistic_calculator_;

  double physical_diameter_m_ = 0.055;
  double expected_distance_m_ = 25.0;
  double expected_diameter_tolerance_ = 0.60;
  int camera_type_ = 0;
  double middle_error_ratio_ = 0.03;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  bool camera_info_received_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr debug_lights_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Lights>::SharedPtr lights_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DartStates>::SharedPtr dart_states_pub_;
  image_transport::Publisher result_image_pub_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__DETECTOR_NODE_HPP_
