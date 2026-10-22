#include "lidar_dart_cpp/coordinate_transform.hpp"
#include "lidar_dart_cpp/dart_bridge.hpp"
#include "lidar_dart_cpp/ekf_fusion.hpp"
#include "lidar_dart_cpp/map_icp.hpp"
#include "lidar_dart_cpp/preprocess.hpp"
#include "lidar_dart_cpp/target_solver.hpp"
#include "lidar_dart_cpp/types.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <tf2_ros/buffer.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace lidar_dart {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

class LidarDartNode : public rclcpp::Node {
 public:
  LidarDartNode() : Node("lidar_dart_node") {
    declareParameters();

    t_lidar_to_gimbal_ = loadPoseParam("extrinsics.lidar_to_gimbal");
    t_gimbal_to_muzzle_ = loadPoseParam("extrinsics.gimbal_to_muzzle");
    muzzle_world_ = loadPoseParam("muzzle_world");
    gimbal_pose_ = Pose::Identity();
    last_gimbal_pose_ = Pose::Identity();

    const std::string map_path = get_parameter("map_path").as_string();
    if (!map_path.empty()) {
      map_ = loadPCD(map_path);
      if (map_) {
        RCLCPP_INFO(get_logger(), "Loaded static map from %s", map_path.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to load static map from %s", map_path.c_str());
      }
    }

    const Eigen::Vector3d initial_target(
        get_parameter("ekf.initial_x").as_double(),
        get_parameter("ekf.initial_y").as_double(),
        get_parameter("ekf.initial_z").as_double());
    ekf_ = std::make_unique<TargetEKF>(
        initial_target,
        get_parameter("ekf.initial_covariance").as_double(),
        get_parameter("ekf.process_noise_pos").as_double(),
        get_parameter("ekf.process_noise_vel").as_double(),
        muzzle_world_);

    bridge_ = std::make_unique<DartBridge>(this);

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        get_parameter("lidar_topic").as_string(),
        rclcpp::QoS(get_parameter("qos_depth").as_int()),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onLidar(msg); });
    gimbal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/gimbal_pose", 10,
        //后面通信弄完记得改这个话题，串口包那边要发一个位姿的话题
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onGimbalPose(msg); });
    camera_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "~/camera_bearing", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { onCameraBearing(msg); });
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

 private:
  void declareParameters() {
    declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    declare_parameter<std::string>("map_path", "");
    declare_parameter<double>("scan_duration", 0.1);
    declare_parameter<int>("qos_depth", 10);

    declarePoseParams("muzzle_world", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    declarePoseParams("extrinsics.lidar_to_gimbal", 0.0, 0.0, 0.18, 0.0, 0.0, 0.0);
    declarePoseParams("extrinsics.gimbal_to_muzzle", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    declare_parameter<double>("preprocess.min_range", 0.2);
    declare_parameter<double>("preprocess.max_range", 25.0);
    declare_parameter<double>("preprocess.roi_yaw_min_deg", -45.0);
    declare_parameter<double>("preprocess.roi_yaw_max_deg", 45.0);
    declare_parameter<double>("preprocess.roi_pitch_min_deg", -30.0);
    declare_parameter<double>("preprocess.roi_pitch_max_deg", 30.0);
    declare_parameter<int>("preprocess.sor_k", 20);
    declare_parameter<double>("preprocess.sor_std_ratio", 2.0);

    declare_parameter<int>("icp.max_iterations", 50);
    declare_parameter<double>("icp.transformation_epsilon", 1e-6);
    declare_parameter<double>("icp.max_correspondence_dist", 1.0);

    declare_parameter<double>("target.distance_threshold", 0.05);
    declare_parameter<int>("target.min_inliers", 10);
    declare_parameter<int>("target.quality_inliers", 50);

    declare_parameter<double>("ekf.initial_x", 0.0);
    declare_parameter<double>("ekf.initial_y", 10.0);
    declare_parameter<double>("ekf.initial_z", 1.0);
    declare_parameter<double>("ekf.initial_covariance", 1.0);
    declare_parameter<double>("ekf.process_noise_pos", 0.01);
    declare_parameter<double>("ekf.process_noise_vel", 0.05);
    declare_parameter<double>("ekf.lidar_noise", 0.05);
    declare_parameter<double>("ekf.camera_noise_az", 0.005);
    declare_parameter<double>("ekf.camera_noise_el", 0.005);

    declare_parameter<double>("fire.min_confidence", 0.6);
    declare_parameter<int>("fire.min_inliers", 10);
  }

  void declarePoseParams(const std::string& prefix, double x, double y, double z,
                         double roll, double pitch, double yaw) {
    declare_parameter<double>(prefix + ".x", x);
    declare_parameter<double>(prefix + ".y", y);
    declare_parameter<double>(prefix + ".z", z);
    declare_parameter<double>(prefix + ".roll", roll);
    declare_parameter<double>(prefix + ".pitch", pitch);
    declare_parameter<double>(prefix + ".yaw", yaw);
  }

  Pose loadPoseParam(const std::string& prefix) {
    return poseFromXYZRPY(
        get_parameter(prefix + ".x").as_double(),
        get_parameter(prefix + ".y").as_double(),
        get_parameter(prefix + ".z").as_double(),
        get_parameter(prefix + ".roll").as_double(),
        get_parameter(prefix + ".pitch").as_double(),
        get_parameter(prefix + ".yaw").as_double());
  }

  bool tf2Transform(
    const TargetSolution &solution,
    const rclcpp::Time &stamp,
    geometry_msgs::msg::PointStamped &tf_target_from_lidar)
  {
    if (!tf_buffer_ || !tf_listener_)
    {
      return false;
    }

    geometry_msgs::msg::PointStamped point_lidar;
    point_lidar.header.frame_id = "livox_frame";
    point_lidar.header.stamp = stamp;
    point_lidar.point.x = solution.center.x();
    point_lidar.point.y = solution.center.y();
    point_lidar.point.z = solution.center.z();

    try
    {
      tf_target_from_lidar = tf_buffer_->transform(point_lidar, "muzzle_link");
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Failed to transform point to muzzle_link: %s", ex.what());
      return false;
    }

    return true;
  }

  void onGimbalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_gimbal_pose_ = gimbal_pose_;
    gimbal_pose_ = Pose::Identity();
    gimbal_pose_.translation() =
        Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    gimbal_pose_.linear() = Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z).toRotationMatrix();
  }

  void onCameraBearing(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) {
      return;
    }
    ekf_->updateCamera(
        Eigen::Vector2d(msg->data[0], msg->data[1]),
        get_parameter("ekf.camera_noise_az").as_double(),
        get_parameter("ekf.camera_noise_el").as_double());
  }

  void onLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudPtr raw = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *raw);
    if (raw->empty()) {
      return;
    }

    LidarFrame frame;
    frame.cloud = raw;
    frame.stamp_sec =
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    const double scan_duration = get_parameter("scan_duration").as_double();
    frame.timestamps.resize(raw->size());
    for (std::size_t i = 0; i < raw->size(); ++i) {
      frame.timestamps[i] =
          raw->size() <= 1 ? 0.0 : scan_duration * static_cast<double>(i) /
                                      static_cast<double>(raw->size() - 1);
    }

    CloudPtr clean = runPreprocess(
        frame,
        last_gimbal_pose_,
        gimbal_pose_,
        get_parameter("preprocess.min_range").as_double(),
        get_parameter("preprocess.max_range").as_double(),
        get_parameter("preprocess.roi_yaw_min_deg").as_double(),
        get_parameter("preprocess.roi_yaw_max_deg").as_double(),
        get_parameter("preprocess.roi_pitch_min_deg").as_double(),
        get_parameter("preprocess.roi_pitch_max_deg").as_double(),
        get_parameter("preprocess.sor_k").as_int(),
        get_parameter("preprocess.sor_std_ratio").as_double());

    // Pose pose_world = muzzle_world_;
    // if (map_ && map_->size() >= 3) {
    //   IcpResult icp_result;
    //   pose_world = localize(
    //       clean, map_, muzzle_world_,
    //       get_parameter("icp.max_iterations").as_int(),
    //       get_parameter("icp.transformation_epsilon").as_double(),
    //       get_parameter("icp.max_correspondence_dist").as_double(),
    //       icp_result);
    //   RCLCPP_DEBUG(get_logger(), "ICP fitness=%.2f rmse=%.3f",
    //                icp_result.fitness, icp_result.inlier_rmse);
    // }


    // const CloudPtr points_world = lidarToWorld(
    //     clean, t_lidar_to_gimbal_, gimbal_pose_, t_gimbal_to_muzzle_, pose_world);
    // const Eigen::Vector3d muzzle_pos = pose_world.translation();
    auto solution = solveTarget(
        // pose_world,
        // muzzle_world,
        clean,
        // Eigen::Vector3d::Zero(),
        get_parameter("target.distance_threshold").as_double(),
        get_parameter("target.min_inliers").as_int(),
        get_parameter("target.quality_inliers").as_int());

    const double now_sec = frame.stamp_sec;
    if (last_stamp_sec_ >= 0.0) {
      const double dt = now_sec - last_stamp_sec_;
      if (dt > 0.0 && dt < 1.0) {
        ekf_->predict(dt);
      }
    }
    last_stamp_sec_ = now_sec;

    double confidence = 0.0;
    int inlier_count = 0;
    if (solution) {
      confidence = solution->confidence;
      inlier_count = solution->inlier_count;
      geometry_msgs::msg::PointStamped target_in_muzzle;
      if (!tf2Transform(*solution, rclcpp::Time(msg->header.stamp), target_in_muzzle)) {
        return;
      }
      ekf_->updateLidar(Eigen::Vector3d(target_in_muzzle.point.x,
                        target_in_muzzle.point.y,
                        target_in_muzzle.point.z),
                        get_parameter("ekf.lidar_noise").as_double());
    }

    const Eigen::Vector3d target_muzzle = ekf_->position();
    // Eigen::Vector3d target_in_muzzle = pose_world.inverse() * target_world;
    const double yaw =
        std::atan2(target_muzzle.y(), target_muzzle.x()) * 180.0 / kPi;
    const double pitch =
        std::atan2(target_muzzle.z(),
                   std::hypot(target_muzzle.x(), target_muzzle.y())) *
        180.0 / kPi;

    TargetOutput output;
    output.target_world = target_muzzle;
    // output.slant_range = (target_world - muzzle_pos).norm();
    output.slant_range  = target_muzzle.norm();
    output.muzzle_yaw = yaw;
    output.muzzle_pitch = pitch;
    output.confidence = confidence;
    output.inlier_count = inlier_count;
    output.frame_stamp_sec = now_sec;
    output.fire_allowed =
        confidence >= get_parameter("fire.min_confidence").as_double() &&
        inlier_count >= get_parameter("fire.min_inliers").as_int();
    bridge_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr camera_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Pose t_lidar_to_gimbal_;
  Pose t_gimbal_to_muzzle_;
  Pose muzzle_world_;
  Pose gimbal_pose_;
  Pose last_gimbal_pose_;
  CloudPtr map_;
  std::unique_ptr<TargetEKF> ekf_;
  std::unique_ptr<DartBridge> bridge_;
  double last_stamp_sec_{-1.0};
};

}  // namespace lidar_dart

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_dart::LidarDartNode>());
  rclcpp::shutdown();
  return 0;
}
