#ifndef DART_DETECTOR__DART_DETECTOR_NODE_HPP_
#define DART_DETECTOR__DART_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>

#include "auto_aim_interfaces/msg/dart_detections.hpp"
#include "dart_detector/dart_detector.hpp"

namespace rm_auto_aim
{

class DartDetectorNode : public rclcpp::Node
{
public:
  explicit DartDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

  DartDetector::Params declareDetectorParams();

  DartDetector detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DartDetections>::SharedPtr dart_detections_pub_;
};

}  // namespace rm_auto_aim

#endif  // DART_DETECTOR__DART_DETECTOR_NODE_HPP_
