#pragma once

#include "lidar_dart_cpp/types.hpp"

#include <Eigen/Core>

#include <vector>

namespace lidar_dart {

CloudPtr removeGround(const CloudConstPtr& cloud, double ground_z, double ground_tolerance);

std::vector<Line2D> fitLines2D(const CloudConstPtr& cloud,
                               int n_lines,
                               double distance_threshold,
                               int max_iterations,
                               int min_inliers);

bool lineIntersection(const Line2D& line1,
                      const Line2D& line2,
                      Eigen::Vector2d& point);

struct FeatureSet {
  std::vector<Line2D> lines;
  std::vector<Eigen::Vector2d> corners;
  CloudPtr non_ground;
};

FeatureSet extractFeatures(const CloudConstPtr& cloud,
                           double ground_z,
                           double ground_tolerance,
                           int n_lines,
                           double distance_threshold,
                           int max_iterations);

}  // namespace lidar_dart
