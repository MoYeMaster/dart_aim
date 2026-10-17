/// Project the LiDAR cloud onto the camera image to verify an extrinsic.
///
///   ros2 run combined_calibrate project_cloud --ros-args -p extrinsics:=$HOME/dart_calibration/shots/extrinsics.yaml
///
/// Publishes the overlay as an image and the cloud in the camera frame, so the
/// point cloud can be compared with the edges it came from.
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "combined_calibrate/calib_io.hpp"
#include "combined_calibrate/calib_math.hpp"
#include "combined_calibrate/overlay.hpp"
#include "combined_calibrate/ros_util.hpp"

namespace cc = combined_calibrate;

class ProjectCloud : public rclcpp::Node {
 public:
  ProjectCloud() : rclcpp::Node("project_cloud") {
    const std::string default_extrinsics =
        std::string(std::getenv("HOME") != nullptr ? std::getenv("HOME") : ".") +
        "/dart_calibration/shots/extrinsics.yaml";
    extrinsics_path_ = declare_parameter("extrinsics", default_extrinsics);
    const std::string lidar_topic = declare_parameter("lidar_topic", std::string("/livox/lidar"));
    const std::string image_topic = declare_parameter("image_topic", std::string("/image_raw"));
    const std::string info_topic =
        declare_parameter("camera_info_topic", std::string("/image_raw/camera_info"));
    accumulate_frames_ = declare_parameter("accumulate_frames", 20);
    max_depth_m_ = declare_parameter("max_depth_m", 30.0);
    min_depth_m_ = declare_parameter("min_depth_m", 0.3);
    point_size_ = declare_parameter("point_size", 2);
    const double rate = std::max(0.1, declare_parameter("publish_rate_hz", 2.0));
    save_path_ = declare_parameter("save_path", std::string(""));
    show_window_ = declare_parameter("show_window", false);
    fix_roi_intrinsics_ = declare_parameter("fix_roi_intrinsics", true);

    std::string error;
    if (!cc::readExtrinsics(extrinsics_path_, extrinsics_, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      throw std::runtime_error(error);
    }
    RCLCPP_INFO(get_logger(), "extrinsics: %s", extrinsics_path_.c_str());

    const auto qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, qos, std::bind(&ProjectCloud::onCloud, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos, std::bind(&ProjectCloud::onImage, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, qos, std::bind(&ProjectCloud::onCameraInfo, this, std::placeholders::_1));

    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("~/overlay", 1);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points_camera", 1);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               std::bind(&ProjectCloud::publishOverlay, this));
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    std::vector<cv::Point3f> points;
    if (!cc::pointsFromCloudMsg(*msg, points, nullptr)) return;
    buffer_.insert(buffer_.end(), points.begin(), points.end());
    if (++frames_ >= accumulate_frames_) {
      cloud_ = buffer_;
      buffer_.clear();
      frames_ = 0;
      RCLCPP_INFO_ONCE(get_logger(), "cloud ready: %zu points", cloud_.size());
    }
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) { image_msg_ = msg; }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    cc::Intrinsics info;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) info.K(row, col) = msg->k[row * 3 + col];
    }
    info.dist.assign(msg->d.begin(), msg->d.end());
    info.width = int(msg->width);
    info.height = int(msg->height);
    if (info.valid()) intrinsics_ = info;
  }

  void publishOverlay() {
    if (!image_msg_ || cloud_.empty() || !intrinsics_.valid()) return;
    cv::Mat image;
    std::string error;
    if (!cc::imageToBgrMat(*image_msg_, image, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
      return;
    }
    cc::Intrinsics info = intrinsics_;
    if (fix_roi_intrinsics_) cc::adjustIntrinsicsForImage(info, image.cols, image.rows, nullptr);

    cc::OverlayOptions options;
    options.max_depth_m = max_depth_m_;
    options.min_depth_m = min_depth_m_;
    options.point_size = point_size_;
    const cv::Mat overlay = cc::renderOverlay(image, cloud_, info, extrinsics_, options);

    cc::publishBgrImage(overlay_pub_, image_msg_->header, overlay);
    std_msgs::msg::Header header = image_msg_->header;
    header.frame_id = "camera";
    cc::publishXyzRgbCloud(cloud_pub_, header, cc::transformCloud(cloud_, extrinsics_), min_depth_m_,
                           max_depth_m_);

    if (!save_path_.empty() && !saved_) {
      if (cc::writeImage(save_path_, overlay)) {
        saved_ = true;
        RCLCPP_INFO(get_logger(), "saved %s", save_path_.c_str());
      }
    }
    if (show_window_) {
      try {
        cv::imshow("combined_calibrate | overlay", overlay);
        cv::waitKey(1);
      } catch (const cv::Exception &exception) {
        show_window_ = false;
        RCLCPP_WARN(get_logger(), "cannot show the overlay window: %s", exception.what());
      }
    }
  }

  std::string extrinsics_path_;
  std::string save_path_;
  double max_depth_m_ = 30.0;
  double min_depth_m_ = 0.3;
  int point_size_ = 2;
  int accumulate_frames_ = 20;
  bool show_window_ = false;
  bool fix_roi_intrinsics_ = true;
  bool saved_ = false;

  cv::Matx44d extrinsics_ = cv::Matx44d::eye();
  cc::Intrinsics intrinsics_;
  sensor_msgs::msg::Image::ConstSharedPtr image_msg_;
  std::vector<cv::Point3f> buffer_;
  std::vector<cv::Point3f> cloud_;
  int frames_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ProjectCloud>());
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
