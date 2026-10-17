#include "lidar_calibrate/accumulator.hpp"

#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <map>
#include <utility>

namespace lidar_calibrate {
namespace {

struct GridCell {
  PointT farthest;
  double max_distance = -1.0;
};

bool isFinite(const PointT& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

}  // namespace

CloudPtr gridFarthestPoint(const std::vector<CloudConstPtr>& frames,
                           double cell_degrees) {
  const double cell = cell_degrees * M_PI / 180.0;
  std::map<std::pair<int, int>, GridCell> grid;

  for (const auto& frame : frames) {
    if (!frame) {
      continue;
    }
    for (const auto& point : frame->points) {
      if (!isFinite(point)) {
        continue;
      }
      const double azimuth = std::atan2(point.y, point.x);
      const double elevation =
          std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
      const int azimuth_index = static_cast<int>(std::floor(azimuth / cell));
      const int elevation_index = static_cast<int>(std::floor(elevation / cell));
      const double distance = std::sqrt(point.x * point.x + point.y * point.y +
                                        point.z * point.z);

      auto& entry = grid[std::make_pair(azimuth_index, elevation_index)];
      if (distance > entry.max_distance) {
        entry.farthest = point;
        entry.max_distance = distance;
      }
    }
  }

  auto result = std::make_shared<Cloud>();
  result->reserve(grid.size());
  for (const auto& entry : grid) {
    if (entry.second.max_distance > 0.0) {
      result->push_back(entry.second.farthest);
    }
  }
  return result;
}

CloudPtr cropRoi(const CloudConstPtr& cloud, const Roi& roi) {
  auto result = std::make_shared<Cloud>();
  if (!cloud) {
    return result;
  }
  result->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (point.x < roi.min_x || point.x > roi.max_x) continue;
    if (point.y < roi.min_y || point.y > roi.max_y) continue;
    if (point.z < roi.min_z || point.z > roi.max_z) continue;
    result->push_back(point);
  }
  return result;
}

CloudPtr voxelDownsample(const CloudConstPtr& cloud, double leaf_size) {
  auto result = std::make_shared<Cloud>();
  if (!cloud || leaf_size <= 0.0) {
    if (cloud) {
      *result = *cloud;
    }
    return result;
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size),
                    static_cast<float>(leaf_size));
  voxel.setInputCloud(cloud);
  voxel.filter(*result);
  return result;
}

CloudPtr accumulate(const std::vector<CloudConstPtr>& frames, double cell_degrees,
                    const Roi& roi, double leaf_size) {
  const CloudPtr filtered = gridFarthestPoint(frames, cell_degrees);
  const CloudPtr cropped = cropRoi(filtered, roi);
  return voxelDownsample(cropped, leaf_size);
}

}  // namespace lidar_calibrate
