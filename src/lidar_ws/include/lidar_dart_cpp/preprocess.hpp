#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <vector>

namespace lidar_dart {

CloudPtr filterRange(const CloudConstPtr& cloud, double min_range, double max_range);

CloudPtr cropROI(const CloudConstPtr& cloud,
                 double min_range,
                 double max_range,
                 double yaw_min_deg,
                 double yaw_max_deg,
                 double pitch_min_deg,
                 double pitch_max_deg);

CloudPtr statisticalOutlierRemoval(const CloudConstPtr& cloud,
                                   int k,
                                   double std_ratio);

CloudPtr undistortMotion(const CloudConstPtr& cloud,
                         const std::vector<double>& timestamps,
                         const Pose& t_start,
                         const Pose& t_end);

CloudPtr runPreprocess(const LidarFrame& frame,
                       const Pose& t_start,
                       const Pose& t_end,
                       double min_range,
                       double max_range,
                       double roi_yaw_min_deg,
                       double roi_yaw_max_deg,
                       double roi_pitch_min_deg,
                       double roi_pitch_max_deg,
                       int sor_k,
                       double sor_std_ratio);

}  // namespace lidar_dart
