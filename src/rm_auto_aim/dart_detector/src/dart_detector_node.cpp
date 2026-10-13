#include "dart_detector/dart_detector_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <functional>
#include <vector>

namespace rm_auto_aim
{

  DartDetectorNode::DartDetectorNode(const rclcpp::NodeOptions &options)
      : Node("dart_detector", options),
        detector_(declareDetectorParams())
  {
    const auto image_topic = declare_parameter("image_topic", "image_raw");
    const auto dart_detections_topic =
        declare_parameter("dart_detections_topic", "dart_detector/detections");
    const auto camera_info_topic = declare_parameter("camera_info_topic", "camera_info");
    dart_length_m_ = declare_parameter("dart_length_m", 0.0);
    dart_width_m_ = declare_parameter("dart_width_m", 0.0);
    target_frame_ = declare_parameter("target_frame", "muzzle_link");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        std::bind(&DartDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

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

  void DartDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
  {
    fx_ = info->k[0];
    fy_ = info->k[4];
    cx_ = info->k[2];
    cy_ = info->k[5];
    camera_info_ = info;
    pnp_solver_.setCameraInfo(info->k, info->d);
  }

  void DartDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
  {
    cv::Mat img_gray;
    cv::Mat img_color;
    auto_aim_interfaces::msg::DartDetections detections_msg;
    detections_msg.header = image_msg->header;
    detections_msg.header.frame_id = target_frame_;
    if (!std::isfinite(dart_length_m_) || !std::isfinite(dart_width_m_) ||
        dart_length_m_ <= 0.0 || dart_width_m_ <= 0.0 || dart_length_m_ < dart_width_m_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Configure dart_length_m >= dart_width_m > 0 for PnP");
      dart_detections_pub_->publish(detections_msg);
      return;
    }
    if (!camera_info_ || !pnp_solver_.hasCameraInfo() || image_msg->header.frame_id.empty() ||
        camera_info_->header.frame_id != image_msg->header.frame_id ||
        camera_info_->width != image_msg->width || camera_info_->height != image_msg->height ||
        (camera_info_->distortion_model != "plumb_bob" &&
         camera_info_->distortion_model != "rational_polynomial" &&
         !camera_info_->distortion_model.empty()))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Waiting for compatible camera calibration and image frame");
      dart_detections_pub_->publish(detections_msg);
      return;
    }
    try
    {
      img_color = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      std::vector<cv::Mat> channels;
      cv::split(img_color, channels);
      cv::max(channels[0], channels[1], img_gray);
      cv::max(img_gray, channels[2], img_gray);
    }
    catch (const cv_bridge::Exception &ex)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to convert image for dart detector: %s", ex.what());
      return;
    }

    std::vector<cv::RotatedRect> boxes;
    auto detections = detector_.detect(img_color, img_gray, &boxes);
    if (detections.empty())
    {
      dart_detections_pub_->publish(detections_msg);
      return;
    }
    geometry_msgs::msg::TransformStamped camera_to_target;
    try
    {
      camera_to_target = tf_buffer_->lookupTransform(
          target_frame_, image_msg->header.frame_id, rclcpp::Time(image_msg->header.stamp));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Dart TF unavailable: %s", ex.what());
      dart_detections_pub_->publish(detections_msg);
      return;
    }
    for (std::size_t index = 0; index < detections.size(); ++index)
    {
      const auto &box = boxes[index];
      const float half_width = static_cast<float>(
          (box.size.width >= box.size.height ? dart_length_m_ : dart_width_m_) * 0.5);
      const float half_height = static_cast<float>(
          (box.size.width >= box.size.height ? dart_width_m_ : dart_length_m_) * 0.5);
      const std::vector<cv::Point3f> object_points{
          {-half_width, half_height, 0.0F}, {-half_width, -half_height, 0.0F}, {half_width, -half_height, 0.0F}, {half_width, half_height, 0.0F}};
      cv::Point2f corners[4];
      box.points(corners);
      const std::vector<cv::Point2f> image_points(corners, corners + 4);
      PnpResult result;
      if (!pnp_solver_.solve(object_points, image_points, result))
      {
        continue;
      }
      geometry_msgs::msg::PointStamped camera_point;
      camera_point.header = image_msg->header;
      camera_point.point.x = result.position_x_m;
      camera_point.point.y = result.position_y_m;
      camera_point.point.z = result.position_z_m;
      geometry_msgs::msg::PointStamped target_point;
      tf2::doTransform(camera_point, target_point, camera_to_target);
      detections[index].position = target_point.point;
      detections_msg.detections.push_back(detections[index]);
    }
    dart_detections_pub_->publish(detections_msg);
  }

} // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::DartDetectorNode)
