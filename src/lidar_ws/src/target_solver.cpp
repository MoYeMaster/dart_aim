#include "lidar_dart_cpp/target_solver.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>

namespace lidar_dart {

std::optional<TargetSolution> solveTarget(const CloudConstPtr& points,
                                          // const Eigen::Vector3d& origin,
                                          double distance_threshold,
                                          int min_inliers,
                                          int quality_inliers) {
  if (!points || points->size() < 3) {
    return std::nullopt;
  }

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(points);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  seg.segment(inliers, coefficients);

  const int inlier_count = static_cast<int>(inliers.indices.size());
  if (coefficients.values.size() < 4 || inlier_count < min_inliers) {
    return std::nullopt;
  }

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (int idx : inliers.indices) {
    const auto& p = points->points[static_cast<std::size_t>(idx)];
    center += Eigen::Vector3d(p.x, p.y, p.z);
  }
  center /= static_cast<double>(inlier_count);

  TargetSolution solution;
  solution.center = center;
  solution.normal = Eigen::Vector3d(coefficients.values[0], coefficients.values[1],
                                    coefficients.values[2]).normalized();
  solution.slant_range = center.norm();
  solution.inlier_count = inlier_count;
  solution.confidence = std::clamp(
      static_cast<double>(inlier_count) / static_cast<double>(quality_inliers), 0.0, 1.0);
  return solution;
}

}  // namespace lidar_dart
