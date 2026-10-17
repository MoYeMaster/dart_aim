#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "lidar_calibrate/accumulator.hpp"

namespace lidar_calibrate {
namespace {

Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw) {
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

Eigen::Vector3d rotationToRpyDeg(const Eigen::Matrix3d& rot) {
  const double sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  Eigen::Vector3d rpy;
  if (sy > 1e-9) {
    rpy(0) = std::atan2(rot(2, 1), rot(2, 2));
    rpy(1) = std::atan2(-rot(2, 0), sy);
    rpy(2) = std::atan2(rot(1, 0), rot(0, 0));
  } else {
    rpy(0) = std::atan2(-rot(1, 2), rot(1, 1));
    rpy(1) = std::atan2(-rot(2, 0), sy);
    rpy(2) = 0.0;
  }
  return rpy * 180.0 / M_PI;
}

}  // namespace

class LidarCalibrate : public rclcpp::Node {
 public:
  LidarCalibrate() : Node("lidar_calibrate_node") {
    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    const std::string map_pcd =
        declare_parameter<std::string>("map_pcd", "field_map.pcd");
    map_frame_ = declare_parameter<std::string>("map_frame", "rm_frame");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    output_yaml_ =
        declare_parameter<std::string>("output_yaml", "lidar_extrinsics.yaml");
    accumulate_frames_ = declare_parameter<int>("accumulate_frames", 20);
    cell_degrees_ = declare_parameter<double>("cell_degrees", 0.1);
    leaf_size_ = declare_parameter<double>("leaf_size", 0.1);
    fitness_threshold_ = declare_parameter<double>("fitness_threshold", 0.2);
    max_iterations_ = declare_parameter<int>("max_iterations", 100);
    stop_after_calibration_ = declare_parameter<bool>("stop_after_calibration", true);
    initial_guess_ = Eigen::Matrix4d::Identity();
    initial_guess_.block<3, 3>(0, 0) = rpyToRotation(
        declare_parameter<double>("initial_guess.roll", 0.0),
        declare_parameter<double>("initial_guess.pitch", 0.0),
        declare_parameter<double>("initial_guess.yaw", 0.0));
    initial_guess_(0, 3) = declare_parameter<double>("initial_guess.x", 0.0);
    initial_guess_(1, 3) = declare_parameter<double>("initial_guess.y", 0.0);
    initial_guess_(2, 3) = declare_parameter<double>("initial_guess.z", 0.0);
    roi_.min_x = declare_parameter<double>("roi.min_x", 5.0);
    roi_.max_x = declare_parameter<double>("roi.max_x", 30.0);
    roi_.min_y = declare_parameter<double>("roi.min_y", -10.0);
    roi_.max_y = declare_parameter<double>("roi.max_y", 8.0);
    roi_.min_z = declare_parameter<double>("roi.min_z", -1.0);
    roi_.max_z = declare_parameter<double>("roi.max_z", 7.0);

    map_ = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(map_pcd, *map_) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load reference map %s", map_pcd.c_str());
      ready_ = false;
      return;
    }
    map_ = voxelDownsample(map_, leaf_size_);
    RCLCPP_INFO(get_logger(), "Loaded reference map with %zu points from %s",
                map_->size(), map_pcd.c_str());
    RCLCPP_INFO(get_logger(), "Initial guess translation [m] %.3f %.3f %.3f",
                initial_guess_(0, 3), initial_guess_(1, 3), initial_guess_(2, 3));

    filtered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "filtered_cloud", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 10,
        std::bind(&LidarCalibrate::onCloud, this, std::placeholders::_1));
  }

  /// False when the reference map could not be loaded, the caller must not spin.
  bool ready() const { return ready_; }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (calibrated_) {
      publishTransform();
      return;
    }

    auto frame = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *frame);
    frames_.push_back(frame);
    if (static_cast<int>(frames_.size()) < accumulate_frames_) {
      return;
    }

    const CloudPtr scan = accumulate(frames_, cell_degrees_, roi_, leaf_size_);
    frames_.clear();
    if (scan->size() < 10) {
      RCLCPP_WARN(get_logger(),
                  "Only %zu points survived accumulation, check topic and ROI",
                  scan->size());
      return;
    }

    sensor_msgs::msg::PointCloud2 filtered_msg;
    pcl::toROSMsg(*scan, filtered_msg);
    filtered_msg.header.frame_id = lidar_frame_;
    filtered_msg.header.stamp = msg->header.stamp;
    filtered_publisher_->publish(filtered_msg);

    // GICP is a local method: seed it with the initial guess, otherwise a
    // metre-scale offset lands every correspondence on the wrong surface and
    // the solver settles into a wrong minimum. Same pattern as map_icp.cpp.
    auto scan_seeded = std::make_shared<Cloud>();
    pcl::transformPointCloud(*scan, *scan_seeded, initial_guess_.cast<float>());

    auto aligned = std::make_shared<Cloud>();
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputTarget(map_);
    gicp.setInputSource(scan_seeded);
    gicp.setMaximumIterations(max_iterations_);
    gicp.align(*aligned);

    const double fitness = gicp.getFitnessScore();
    RCLCPP_INFO(get_logger(), "GICP fitness score: %.4f (threshold %.2f)", fitness,
                fitness_threshold_);

    if (!gicp.hasConverged() || fitness > fitness_threshold_) {
      RCLCPP_WARN(get_logger(), "Registration did not converge well, retrying");
      return;
    }

    transform_ =
        initial_guess_ * gicp.getFinalTransformation().cast<double>();
    calibrated_ = true;
    publishTransform();
    writeResult(fitness);
    if (stop_after_calibration_) {
      RCLCPP_INFO(get_logger(), "Calibration done, shutting down");
      rclcpp::shutdown();
    }
  }

  void publishTransform() {
    const Eigen::Matrix3d rot = transform_.block<3, 3>(0, 0);
    const Eigen::Quaterniond q(rot);

    geometry_msgs::msg::TransformStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = map_frame_;
    stamped.child_frame_id = lidar_frame_;
    stamped.transform.translation.x = transform_(0, 3);
    stamped.transform.translation.y = transform_(1, 3);
    stamped.transform.translation.z = transform_(2, 3);
    stamped.transform.rotation.x = q.x();
    stamped.transform.rotation.y = q.y();
    stamped.transform.rotation.z = q.z();
    stamped.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(stamped);
  }

  void writeResult(double fitness) {
    const Eigen::Matrix3d rot = transform_.block<3, 3>(0, 0);
    const Eigen::Vector3d rpy = rotationToRpyDeg(rot);

    std::ofstream out(output_yaml_);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s for writing",
                   output_yaml_.c_str());
      return;
    }
    out << "# LiDAR-only calibration, produced by lidar_calibrate_node.\n";
    out << "# p_map = R * p_lidar + t, column-major rotation, row-major matrix.\n";
    out << "map_frame: " << map_frame_ << "\n";
    out << "lidar_frame: " << lidar_frame_ << "\n";
    out << "fitness_score: " << fitness << "\n";
    out << "rotation_rpy_deg: [" << rpy(0) << ", " << rpy(1) << ", " << rpy(2)
        << "]\n";
    out << "translation_m: [" << transform_(0, 3) << ", " << transform_(1, 3) << ", "
        << transform_(2, 3) << "]\n";
    out << "matrix4x4:\n";
    for (int row = 0; row < 4; ++row) {
      out << "  [";
      for (int col = 0; col < 4; ++col) {
        out << transform_(row, col) << (col == 3 ? "" : ", ");
      }
      out << "]\n";
    }
    out.close();
    RCLCPP_INFO(get_logger(), "Wrote extrinsics to %s", output_yaml_.c_str());
    RCLCPP_INFO(get_logger(), "translation [m] %.4f %.4f %.4f, rpy [deg] %.3f %.3f %.3f",
                transform_(0, 3), transform_(1, 3), transform_(2, 3), rpy(0), rpy(1),
                rpy(2));
  }

  std::string map_frame_;
  std::string lidar_frame_;
  std::string output_yaml_;
  int accumulate_frames_ = 20;
  int max_iterations_ = 100;
  double cell_degrees_ = 0.1;
  double leaf_size_ = 0.1;
  double fitness_threshold_ = 0.2;
  bool stop_after_calibration_ = true;
  bool calibrated_ = false;
  bool ready_ = true;
  Roi roi_;
  CloudPtr map_;
  Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d initial_guess_ = Eigen::Matrix4d::Identity();
  std::vector<CloudConstPtr> frames_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace lidar_calibrate

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_calibrate::LidarCalibrate>();
  if (node->ready()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
