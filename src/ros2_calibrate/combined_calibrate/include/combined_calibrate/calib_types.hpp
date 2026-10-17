// Shared value types for the camera + LiDAR joint calibration tools.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace combined_calibrate {

/// Camera intrinsics, together with the image size they were calibrated for.
struct Intrinsics {
  cv::Matx33d K = cv::Matx33d::eye();
  std::vector<double> dist;  ///< plumb_bob: k1 k2 p1 p2 [k3 ...]
  int width = 0;
  int height = 0;

  bool valid() const { return K(0, 0) > 0.0 && K(1, 1) > 0.0; }
};

/// One calibration pose: the same physical corners seen by both sensors.
struct Shot {
  std::string name;
  std::string dir;
  std::vector<cv::Point3d> lidar;
  std::vector<cv::Point2d> image;

  bool complete() const {
    return !lidar.empty() && lidar.size() == image.size();
  }
};

struct SolveResult {
  bool ok = false;
  cv::Matx33d R = cv::Matx33d::eye();
  cv::Vec3d t;
  double mean_error_px = 0.0;
  double worst_error_px = 0.0;
  std::vector<double> per_shot_error_px;
  std::vector<std::string> used_shots;
  std::vector<std::string> rejected_shots;
};

/// 4x4 homogeneous transform from a rotation matrix and a translation.
cv::Matx44d composePose(const cv::Matx33d &R, const cv::Vec3d &t);

/// Inverse of a rigid 4x4 transform.
cv::Matx44d invertPose(const cv::Matx44d &T);

/// Roll/pitch/yaw in degrees, ZYX convention (matching the previous tool).
cv::Vec3d rotationToRpyDegrees(const cv::Matx33d &R);

}  // namespace combined_calibrate
