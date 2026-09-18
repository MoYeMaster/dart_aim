#ifndef DETECTOR__IMAGE_UTILS_HPP_
#define DETECTOR__IMAGE_UTILS_HPP_

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rm_auto_aim
{

inline cv::Mat makeDetectorInputImage(
  const sensor_msgs::msg::Image & image_msg, int camera_type)
{
  if (camera_type == 1) {
    const auto color_image = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat gray_image;
    cv::cvtColor(color_image->image, gray_image, cv::COLOR_BGR2GRAY);
    return gray_image;
  }

  return cv_bridge::toCvCopy(image_msg, "mono8")->image;
}

}  // namespace rm_auto_aim

#endif  // DETECTOR__IMAGE_UTILS_HPP_
