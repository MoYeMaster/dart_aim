#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <optional>
#include <vector>

namespace lidar_dart {
struct Target{
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  float area,distance;
};

std::optional<TargetSolution> solveTarget(const CloudConstPtr& points,
                                        //   const Eigen::Vector3d& origin,
                                          double distance_threshold,
                                          int min_inliers,
                                          int quality_inliers);

}  // namespace lidar_dart
