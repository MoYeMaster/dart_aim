#include "lidar_dart_cpp/preprocess.hpp"

#include "lidar_dart_cpp/coordinate_transform.hpp"

#include <pcl/filters/statistical_outlier_removal.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace lidar_dart {

namespace {

constexpr double kPi = 3.14159265358979323846;

double degrees(double rad) { return rad * 180.0 / kPi; }

Pose interpolatePose(const Pose& start, const Pose& end, double alpha) {
  const Eigen::Vector3d rpy_start = rpyFromMatrix(start.rotation());
  const Eigen::Vector3d rpy_end = rpyFromMatrix(end.rotation());
  const Eigen::Vector3d t = start.translation() + alpha * (end.translation() - start.translation());
  const Eigen::Vector3d rpy = rpy_start + alpha * (rpy_end - rpy_start);
  return poseFromXYZRPY(t.x(), t.y(), t.z(), rpy.x(), rpy.y(), rpy.z());
}

}  // namespace

CloudPtr filterRange(const CloudConstPtr& cloud, double min_range, double max_range) {
  auto out = std::make_shared<Cloud>();
  out->header = cloud->header;
  out->reserve(cloud->size());
  for (const auto& p : cloud->points) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (r >= min_range && r <= max_range) {
      out->push_back(p);
    }
  }
  return out;
}

CloudPtr cropROI(const CloudConstPtr& cloud,
                 double min_range,
                 double max_range,
                 double yaw_min_deg,
                 double yaw_max_deg,
                 double pitch_min_deg,
                 double pitch_max_deg) {
  auto out = std::make_shared<Cloud>();
  out->header = cloud->header;
  out->reserve(cloud->size());
  for (const auto& p : cloud->points) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    const double yaw = degrees(std::atan2(p.y, p.x));
    const double pitch = degrees(std::atan2(p.z, std::hypot(p.x, p.y)));
    if (r >= min_range && r <= max_range && yaw >= yaw_min_deg &&
        yaw <= yaw_max_deg && pitch >= pitch_min_deg && pitch <= pitch_max_deg) {
      out->push_back(p);
    }
  }
  return out;
}

CloudPtr statisticalOutlierRemoval(const CloudConstPtr& cloud, int k, double std_ratio) {
  auto out = std::make_shared<Cloud>();
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(static_cast<int>(std::max(2, k)));
  sor.setStddevMulThresh(std_ratio);
  sor.filter(*out);
  out->header = cloud->header;
  return out;
}

CloudPtr undistortMotion(const CloudConstPtr& cloud,
                         const std::vector<double>& timestamps,
                         const Pose& t_start,
                         const Pose& t_end) {
  auto out = std::make_shared<Cloud>();
  out->header = cloud->header;
  out->reserve(cloud->size());
  if (cloud->empty() || timestamps.size() != cloud->size()) {
    *out = *cloud;
    return out;
  }

  const double span = timestamps.back() - timestamps.front();
  const Pose start_inv = t_start.inverse();
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const double alpha = span > 1e-9
                             ? std::clamp((timestamps[i] - timestamps.front()) / span, 0.0, 1.0)
                             : 0.0;
    const Pose t_i = interpolatePose(t_start, t_end, alpha);
    const Pose t_i_to_start = start_inv * t_i;
    const auto& p = cloud->points[i];
    const Eigen::Vector3d v = t_i_to_start * Eigen::Vector3d(p.x, p.y, p.z);
    out->push_back(PointT(static_cast<float>(v.x()), static_cast<float>(v.y()),
                          static_cast<float>(v.z())));
  }
  return out;
}

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
                       double sor_std_ratio) {
  CloudPtr clean = filterRange(frame.cloud, min_range, max_range);
  clean = cropROI(clean, min_range, max_range, roi_yaw_min_deg, roi_yaw_max_deg,
                  roi_pitch_min_deg, roi_pitch_max_deg);
  clean = statisticalOutlierRemoval(clean, sor_k, sor_std_ratio);
  return undistortMotion(clean, frame.timestamps, t_start, t_end);
}

}  // namespace lidar_dart
