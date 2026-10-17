#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <string>

namespace lidar_dart {

Pose localize(const CloudConstPtr& scan,
              const CloudConstPtr& map,
              const Pose& init_pose,
              int max_iterations,
              double transformation_epsilon,
              double max_correspondence_dist,
              IcpResult& result);

CloudPtr loadPCD(const std::string& path);

}  // namespace lidar_dart
