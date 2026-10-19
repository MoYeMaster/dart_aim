#include "lidar_dart_cpp/dart_bridge.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace lidar_dart {

DartBridge::DartBridge(rclcpp::Node* node) : clock_(node->get_clock()) {
  target_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "~/target_position", 10);
  slant_range_pub_ = node->create_publisher<std_msgs::msg::Float32>("~/slant_range", 10);
  yaw_pub_ = node->create_publisher<std_msgs::msg::Float32>("~/muzzle_yaw", 10);
  pitch_pub_ = node->create_publisher<std_msgs::msg::Float32>("~/muzzle_pitch", 10);
  confidence_pub_ = node->create_publisher<std_msgs::msg::Float32>("~/confidence", 10);
  fire_pub_ = node->create_publisher<std_msgs::msg::Bool>("~/fire_allowed", 10);
}

void DartBridge::publish(const TargetOutput& output) {
  geometry_msgs::msg::PointStamped point;
  point.header.stamp = clock_->now();
  // point.header.frame_id = "world";
  point.header.frame_id = "muzzle_link";
  point.point.x = output.target_world.x();
  point.point.y = output.target_world.y();
  point.point.z = output.target_world.z();
  target_pub_->publish(point);

  std_msgs::msg::Float32 slant;
  slant.data = static_cast<float>(output.slant_range);
  slant_range_pub_->publish(slant);

  std_msgs::msg::Float32 yaw;
  yaw.data = static_cast<float>(output.muzzle_yaw);
  yaw_pub_->publish(yaw);

  std_msgs::msg::Float32 pitch;
  pitch.data = static_cast<float>(output.muzzle_pitch);
  pitch_pub_->publish(pitch);

  std_msgs::msg::Float32 confidence;
  confidence.data = static_cast<float>(output.confidence);
  confidence_pub_->publish(confidence);

  std_msgs::msg::Bool fire;
  fire.data = output.fire_allowed;
  fire_pub_->publish(fire);
}

}  // namespace lidar_dart
