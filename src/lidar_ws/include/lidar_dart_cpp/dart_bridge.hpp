#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lidar_dart {

class DartBridge {
 public:
  explicit DartBridge(rclcpp::Node* node);

  void publish(const TargetOutput& output);

 private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slant_range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fire_pub_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace lidar_dart
