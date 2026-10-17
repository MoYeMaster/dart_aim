#include "lidar_dart_cpp/feature_extract.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>

namespace lidar_dart {

namespace {

Line2D lineFromPoints(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  const double dx = p2.y() - p1.y();
  const double dy = p1.x() - p2.x();
  const double norm = std::hypot(dx, dy);
  Line2D line;
  if (norm < 1e-9) {
    return line;
  }
  line.a = dx / norm;
  line.b = dy / norm;
  line.c = -(line.a * p1.x() + line.b * p1.y());
  return line;
}

double lineDistance(const Line2D& line, const Eigen::Vector2d& p) {
  return std::abs(line.a * p.x() + line.b * p.y() + line.c);
}

}  // namespace

CloudPtr removeGround(const CloudConstPtr& cloud, double ground_z, double ground_tolerance) {
  auto out = std::make_shared<Cloud>();
  out->header = cloud->header;
  out->reserve(cloud->size());
  for (const auto& p : cloud->points) {
    if (std::abs(p.z - ground_z) > ground_tolerance) {
      out->push_back(p);
    }
  }
  return out;
}

std::vector<Line2D> fitLines2D(const CloudConstPtr& cloud,
                               int n_lines,
                               double distance_threshold,
                               int max_iterations,
                               int min_inliers) {
  std::vector<Line2D> lines;
  const int n = static_cast<int>(cloud->size());
  if (n < 2) {
    return lines;
  }

  std::vector<Eigen::Vector2d> points;
  points.reserve(n);
  for (const auto& p : cloud->points) {
    points.emplace_back(p.x, p.y);
  }

  std::vector<bool> remaining(n, true);
  std::mt19937 rng(12345);

  for (int line_index = 0; line_index < n_lines; ++line_index) {
    const int remaining_count =
        static_cast<int>(std::count(remaining.begin(), remaining.end(), true));
    if (remaining_count < min_inliers) {
      break;
    }

    Line2D best_line;
    std::vector<int> best_inliers;
    for (int iter = 0; iter < max_iterations; ++iter) {
      int i = 0;
      int j = 0;
      int tries = 0;
      do {
        i = static_cast<int>(rng() % n);
        j = static_cast<int>(rng() % n);
        ++tries;
      } while ((i == j || !remaining[i] || !remaining[j]) && tries < 1000);
      if (!remaining[i] || !remaining[j] || i == j) {
        continue;
      }

      const Line2D candidate = lineFromPoints(points[i], points[j]);
      if (candidate.a == 0.0 && candidate.b == 0.0) {
        continue;
      }

      std::vector<int> inliers;
      inliers.reserve(n);
      for (int k = 0; k < n; ++k) {
        if (remaining[k] && lineDistance(candidate, points[k]) < distance_threshold) {
          inliers.push_back(k);
        }
      }
      if (static_cast<int>(inliers.size()) >= std::max(min_inliers,
                                                       static_cast<int>(best_inliers.size()))) {
        best_line = candidate;
        best_inliers = std::move(inliers);
      }
    }

    if (best_inliers.empty()) {
      break;
    }
    lines.push_back(best_line);
    for (int idx : best_inliers) {
      remaining[idx] = false;
    }
  }
  return lines;
}

bool lineIntersection(const Line2D& line1, const Line2D& line2, Eigen::Vector2d& point) {
  const double det = line1.a * line2.b - line2.a * line1.b;
  if (std::abs(det) < 1e-9) {
    return false;
  }
  const double x = (line2.b * line1.c - line1.b * line2.c) / det;
  const double y = (line1.a * line2.c - line2.a * line1.c) / det;
  point = Eigen::Vector2d(x, y);
  return true;
}

FeatureSet extractFeatures(const CloudConstPtr& cloud,
                           double ground_z,
                           double ground_tolerance,
                           int n_lines,
                           double distance_threshold,
                           int max_iterations) {
  FeatureSet result;
  result.non_ground = removeGround(cloud, ground_z, ground_tolerance);
  if (result.non_ground->size() < static_cast<std::size_t>(n_lines * 2)) {
    return result;
  }

  result.lines = fitLines2D(result.non_ground, n_lines, distance_threshold,
                            max_iterations, 8);
  result.corners.reserve(4);
  for (std::size_t i = 0; i < result.lines.size(); ++i) {
    for (std::size_t j = i + 1; j < result.lines.size(); ++j) {
      Eigen::Vector2d corner;
      if (lineIntersection(result.lines[i], result.lines[j], corner) &&
          result.corners.size() < 4) {
        result.corners.push_back(corner);
      }
    }
  }
  return result;
}

}  // namespace lidar_dart
