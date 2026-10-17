// Small ROS <-> OpenCV helpers shared by the nodes.
#pragma once

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace combined_calibrate {

/// Any sensor_msgs image (mono8, bgr8, bayer_*) as a BGR cv::Mat.
inline bool imageToBgrMat(const sensor_msgs::msg::Image &msg, cv::Mat &bgr, std::string *error) {
  try {
    bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception &exception) {
    if (error) *error = std::string("cv_bridge: ") + exception.what();
    return false;
  }
  if (bgr.empty()) {
    if (error) *error = "empty camera frame";
    return false;
  }
  return true;
}

/// XYZ points of a PointCloud2, skipping NaN and the all-zero padding that
/// Livox uses for invalid returns.
inline bool pointsFromCloudMsg(const sensor_msgs::msg::PointCloud2 &msg,
                               std::vector<cv::Point3f> &points, std::string *error) {
  points.clear();
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    points.reserve(size_t(msg.width) * size_t(msg.height));
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (x == 0.0f && y == 0.0f && z == 0.0f) continue;
      points.emplace_back(x, y, z);
    }
  } catch (const std::runtime_error &exception) {
    if (error) *error = std::string("cannot read the cloud: ") + exception.what();
    return false;
  }
  return true;
}

inline void publishBgrImage(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher,
                            const std_msgs::msg::Header &header, const cv::Mat &image) {
  if (image.empty() || !publisher) return;
  cv_bridge::CvImage out;
  out.header = header;
  out.encoding = "bgr8";
  out.image = image;
  publisher->publish(*out.toImageMsg());
}

/// PointCloud2 in the camera frame, coloured by depth so rviz shows the same
/// colours as the overlay image.
inline void publishXyzRgbCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
    const std_msgs::msg::Header &header, const std::vector<cv::Point3f> &points, double near_m,
    double far_m) {
  if (!publisher || points.empty()) return;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");
  const double range = std::max(1e-3, far_m - near_m);
  for (const auto &point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    double ratio = (double(point.z) - near_m) / range;
    ratio = std::min(1.0, std::max(0.0, ratio));
    const int index = static_cast<int>(std::lround(ratio * 255.0));
    *iter_r = static_cast<uint8_t>(index);
    *iter_g = static_cast<uint8_t>(index < 128 ? 2 * index : 2 * (255 - index));
    *iter_b = static_cast<uint8_t>(255 - index);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
  publisher->publish(msg);
}

}  // namespace combined_calibrate
