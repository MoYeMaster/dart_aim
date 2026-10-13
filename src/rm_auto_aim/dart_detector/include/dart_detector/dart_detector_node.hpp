#ifndef DART_DETECTOR__DART_DETECTOR_NODE_HPP_
#define DART_DETECTOR__DART_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/dart_detections.hpp"
#include "dart_detector/dart_detector.hpp"
#include "detector/pnp.hpp"

namespace rm_auto_aim
{

class DartDetectorNode : public rclcpp::Node
{
public:
  explicit DartDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

  DartDetector::Params declareDetectorParams();

  DartDetector detector_;
  PnpSolver pnp_solver_;
  double dart_length_m_;
  double dart_width_m_;
  std::string target_frame_;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DartDetections>::SharedPtr dart_detections_pub_;
};

}  // namespace rm_auto_aim

#endif  // DART_DETECTOR__DART_DETECTOR_NODE_HPP_
