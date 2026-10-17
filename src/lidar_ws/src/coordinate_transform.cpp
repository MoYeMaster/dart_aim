#include "lidar_dart_cpp/coordinate_transform.hpp"

#include <cmath>
#include <memory>

namespace lidar_dart {

Pose poseFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw) {
  Pose pose = Pose::Identity();
  pose.translate(Eigen::Vector3d(x, y, z));
  pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  return pose;
}

Eigen::Vector3d rpyFromMatrix(const Eigen::Matrix3d& rot) {
  const double sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  if (sy > 1e-9) {
    return Eigen::Vector3d(std::atan2(rot(2, 1), rot(2, 2)),
                           std::atan2(-rot(2, 0), sy),
                           std::atan2(rot(1, 0), rot(0, 0)));
  }
  return Eigen::Vector3d(std::atan2(-rot(1, 2), rot(1, 1)),
                         std::atan2(-rot(2, 0), sy),
                         0.0);
}

double poseYaw(const Pose& pose) { return rpyFromMatrix(pose.rotation())(2); }

Eigen::Vector2d poseXY(const Pose& pose) {
  return Eigen::Vector2d(pose.translation().x(), pose.translation().y());
}

Pose compose(const Pose& first, const Pose& second) { return second * first; }

Eigen::Vector3d transformPoint(const Pose& pose, const Eigen::Vector3d& point) {
  return pose * point;
}

CloudPtr transformCloud(const CloudConstPtr& cloud, const Pose& pose) {
  auto out = std::make_shared<Cloud>();
  out->resize(cloud->size());
  out->header = cloud->header;
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& p = cloud->points[i];
    const Eigen::Vector3d v = pose * Eigen::Vector3d(p.x, p.y, p.z);
    out->points[i] = PointT(static_cast<float>(v.x()), static_cast<float>(v.y()),
                            static_cast<float>(v.z()));
  }
  return out;
}

CloudPtr lidarToWorld(const CloudConstPtr& points_lidar,
                      const Pose& t_lidar_to_gimbal,
                      const Pose& gimbal_pose,
                      const Pose& t_gimbal_to_muzzle,
                      const Pose& muzzle_world) {
  const Pose lidar_to_muzzle = compose(compose(t_lidar_to_gimbal, gimbal_pose),
                                       t_gimbal_to_muzzle);
  const Pose lidar_to_world = compose(lidar_to_muzzle, muzzle_world);
  return transformCloud(points_lidar, lidar_to_world);
}

}  // namespace lidar_dart
