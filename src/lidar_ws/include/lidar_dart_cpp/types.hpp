#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace lidar_dart {

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;
using Pose = Eigen::Isometry3d;

struct LidarFrame {
  CloudPtr cloud;
  std::vector<double> timestamps;
  double stamp_sec{0.0};
};

struct Line2D {
  double a{0.0};
  double b{0.0};
  double c{0.0};
};

struct IcpResult {
  bool converged{false};
  double fitness{0.0};
  double inlier_rmse{0.0};
  int iterations{0};
};

struct TargetSolution {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double slant_range{0.0};
  double confidence{0.0};
  int inlier_count{0};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
};

struct TargetOutput {
  Eigen::Vector3d target_world{Eigen::Vector3d::Zero()};
  double slant_range{0.0};
  double muzzle_yaw{0.0};
  double muzzle_pitch{0.0};
  double confidence{0.0};
  bool fire_allowed{false};
  int inlier_count{0};
  double frame_stamp_sec{0.0};
};

}  // namespace lidar_dart
