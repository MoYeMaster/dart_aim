#include "dart_detector/dart_detector_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <functional>
#include <vector>

namespace rm_auto_aim
{

DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions & options)
: Node("dart_detector", options),
  detector_(declareDetectorParams())
{
  const auto image_topic = declare_parameter("image_topic", "image_raw");
  const auto dart_detections_topic =
    declare_parameter("dart_detections_topic", "dart_detector/detections");

  dart_detections_pub_ = create_publisher<auto_aim_interfaces::msg::DartDetections>(
    dart_detections_topic, 10);
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&DartDetectorNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Dart detector started: image topic '%s'", image_topic.c_str());
}

DartDetector::Params DartDetectorNode::declareDetectorParams()
{
  DartDetector::Params params;
  params.threshold = declare_parameter("threshold", params.threshold);
  params.blur_kernel_size = declare_parameter("blur_kernel_size", params.blur_kernel_size);
  params.open_kernel_size = declare_parameter("open_kernel_size", params.open_kernel_size);
  params.min_area = declare_parameter("min_area", params.min_area);
  params.max_area = declare_parameter("max_area", params.max_area);
  params.min_short_side_px = declare_parameter("min_short_side_px", params.min_short_side_px);
  params.max_short_side_px = declare_parameter("max_short_side_px", params.max_short_side_px);
  params.min_aspect_ratio = declare_parameter("min_aspect_ratio", params.min_aspect_ratio);
  params.max_aspect_ratio = declare_parameter("max_aspect_ratio", params.max_aspect_ratio);
  params.min_fill_ratio = declare_parameter("min_fill_ratio", params.min_fill_ratio);
  params.roi_x_min = declare_parameter("roi_x_min", params.roi_x_min);
  params.roi_x_max = declare_parameter("roi_x_max", params.roi_x_max);
  params.roi_y_min = declare_parameter("roi_y_min", params.roi_y_min);
  params.roi_y_max = declare_parameter("roi_y_max", params.roi_y_max);
  return params;
}

void DartDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  cv::Mat img_gray;
  cv::Mat img_color;
  try {
      img_color = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      std::vector<cv::Mat> channels;
      cv::split(img_color, channels);
      cv::max(channels[0], channels[1], img_gray);
      cv::max(img_gray, channels[2], img_gray);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to convert image for dart detector: %s", ex.what());
    return;
  }

  auto_aim_interfaces::msg::DartDetections detections_msg;
  detections_msg.header = image_msg->header;
  detections_msg.detections = detector_.detect(img_color,img_gray);
  dart_detections_pub_->publish(detections_msg);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::DartDetectorNode)
