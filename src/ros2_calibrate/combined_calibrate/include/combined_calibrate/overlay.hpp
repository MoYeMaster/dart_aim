// Project a LiDAR cloud into the camera image, colour coded by depth.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "combined_calibrate/calib_types.hpp"

namespace combined_calibrate {

struct OverlayOptions {
  double max_depth_m = 30.0;
  double min_depth_m = 0.3;
  int point_size = 2;
  bool color_by_depth = true;
};

/// Draw `cloud` (LiDAR frame) on top of `bgr` using T_lidar_to_camera.
cv::Mat renderOverlay(const cv::Mat &bgr, const std::vector<cv::Point3f> &cloud,
                      const Intrinsics &info, const cv::Matx44d &T_lidar_to_camera,
                      const OverlayOptions &options = {});

/// Transform the cloud into the camera frame (for the rviz PointCloud2 output).
std::vector<cv::Point3f> transformCloud(const std::vector<cv::Point3f> &cloud,
                                        const cv::Matx44d &T_lidar_to_camera);

}  // namespace combined_calibrate
