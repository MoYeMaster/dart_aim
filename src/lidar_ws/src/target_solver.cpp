#include "lidar_dart_cpp/target_solver.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_dart {
namespace {
constexpr double kPi = 3.14159265358979323846;
// 当前几何先验；距离相对输入点云原点，倾角要求输入坐标系 Z 轴竖直。
constexpr double kExpectedRange = 25.0;
constexpr double kMinRange = 20.0;
constexpr double kMaxRange = 30.0;
constexpr double kExpectedTilt = 27.5 * kPi / 180.0;
constexpr double kTiltTolerance = 5.0 * kPi / 180.0;
constexpr double kMinArea = 0.01;
constexpr double kMaxArea = 0.1;
}  // namespace

std::optional<TargetSolution> solveTarget(const CloudConstPtr& points,
                                          double distance_threshold,
                                          int min_inliers,
                                          int quality_inliers) {
  if (!points || points->size() < 3 || !std::isfinite(distance_threshold) ||
      distance_threshold <= 0.0 || min_inliers < 3 || quality_inliers <= 0) {
    return std::nullopt;
  }

  CloudPtr remaining(new Cloud);
  remaining->header = points->header;
  remaining->reserve(points->size());
  for (const auto& p : points->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      remaining->push_back(p);
    }
  }

  std::optional<TargetSolution> best;
  double best_score = -std::numeric_limits<double>::infinity();
  pcl::ExtractIndices<PointT> extract;
  pcl::SACSegmentation<PointT> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setMaxIterations(1000);

  while (remaining->size() >= static_cast<std::size_t>(min_inliers)) {
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(remaining);
    seg.segment(*inliers, coefficients);

    const int inlier_count = static_cast<int>(inliers->indices.size());
    if (coefficients.values.size() < 4 || inlier_count < min_inliers) {
      break;
    }

    const CloudConstPtr plane_cloud = remaining;
    CloudPtr rest(new Cloud);
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*rest);
    remaining = rest;

    Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1],
                           coefficients.values[2]);
    const double normal_norm = normal.norm();
    if (!normal.allFinite() || !std::isfinite(coefficients.values[3]) ||
        normal_norm < 1e-9) {
      continue;
    }
    normal /= normal_norm;
    const double plane_offset = coefficients.values[3] / normal_norm;
    const double tilt = std::acos(std::clamp(std::abs(normal.z()), 0.0, 1.0));
    const double tilt_error = std::abs(tilt - kExpectedTilt);
    if (tilt_error > kTiltTolerance) {
      continue;
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int idx : inliers->indices) {
      center += plane_cloud->points[static_cast<std::size_t>(idx)]
                    .getVector3fMap().cast<double>();
    }
    center /= static_cast<double>(inlier_count);
    const double distance = center.norm();
    if (!center.allFinite() || distance < kMinRange || distance > kMaxRange) {
      continue;
    }

    const Eigen::Vector3d axis = std::abs(normal.x()) < 0.9
                                    ? Eigen::Vector3d::UnitX()
                                    : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d u = normal.cross(axis).normalized();
    const Eigen::Vector3d v = normal.cross(u);
    double u_min = std::numeric_limits<double>::infinity();
    double u_max = -std::numeric_limits<double>::infinity();
    double v_min = std::numeric_limits<double>::infinity();
    double v_max = -std::numeric_limits<double>::infinity();
    double squared_error = 0.0;
    for (int idx : inliers->indices) {
      const Eigen::Vector3d p = plane_cloud->points[static_cast<std::size_t>(idx)]
                                   .getVector3fMap().cast<double>();
      const Eigen::Vector3d q = p - center;
      const double pu = q.dot(u);
      const double pv = q.dot(v);
      u_min = std::min(u_min, pu);
      u_max = std::max(u_max, pu);
      v_min = std::min(v_min, pv);
      v_max = std::max(v_max, pv);
      const double residual = normal.dot(p) + plane_offset;
      squared_error += residual * residual;
    }
    const double area = (u_max - u_min) * (v_max - v_min);
    if (!std::isfinite(area) || area < kMinArea || area > kMaxArea) {
      continue;
    }

    const double rmse = std::sqrt(squared_error / static_cast<double>(inlier_count));
    if (!std::isfinite(rmse)) {
      continue;
    }
    TargetSolution candidate;
    candidate.center = center;
    candidate.normal = normal.z() < 0.0 ? -normal : normal;
    candidate.slant_range = distance;
    candidate.inlier_count = inlier_count;
    candidate.confidence = std::clamp(
        static_cast<double>(inlier_count) / static_cast<double>(quality_inliers),
        0.0, 1.0);

    const double tilt_score = 1.0 - tilt_error / kTiltTolerance;
    const double fit_score = 1.0 - std::clamp(rmse / distance_threshold, 0.0, 1.0);
    const double range_score = 1.0 - std::clamp(
        std::abs(distance - kExpectedRange) / (kMaxRange - kExpectedRange), 0.0, 1.0);
    const double score = 0.35 * candidate.confidence + 0.30 * tilt_score +
                         0.20 * fit_score + 0.15 * range_score;
    if (!best || score > best_score ||
        (score == best_score && inlier_count > best->inlier_count)) {
      best = candidate;
      best_score = score;
    }
  }
  return best;
}

}  // namespace lidar_dart
