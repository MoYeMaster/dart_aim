#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <optional>

namespace lidar_dart {

std::optional<TargetSolution> solveTarget(const CloudConstPtr& points,
                                        //   const Eigen::Vector3d& origin,
                                          double distance_threshold,
                                          int min_inliers,
                                          int quality_inliers);

}  // namespace lidar_dart
