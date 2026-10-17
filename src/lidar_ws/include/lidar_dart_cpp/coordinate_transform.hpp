#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lidar_dart {

Pose poseFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw);
Eigen::Vector3d rpyFromMatrix(const Eigen::Matrix3d& rot);
double poseYaw(const Pose& pose);
Eigen::Vector2d poseXY(const Pose& pose);
Pose compose(const Pose& first, const Pose& second);

Eigen::Vector3d transformPoint(const Pose& pose, const Eigen::Vector3d& point);
CloudPtr transformCloud(const CloudConstPtr& cloud, const Pose& pose);

CloudPtr lidarToWorld(const CloudConstPtr& points_lidar,
                      const Pose& t_lidar_to_gimbal,
                      const Pose& gimbal_pose,
                      const Pose& t_gimbal_to_muzzle,
                      const Pose& muzzle_world);

}  // namespace lidar_dart
