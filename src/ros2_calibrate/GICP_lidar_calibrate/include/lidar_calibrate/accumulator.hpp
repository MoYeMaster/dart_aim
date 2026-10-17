#ifndef LIDAR_CALIBRATE_ACCUMULATOR_HPP
#define LIDAR_CALIBRATE_ACCUMULATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace lidar_calibrate {

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

/// Axis-aligned box applied to a scan before registration.
struct Roi {
  double min_x = -50.0;
  double max_x = 50.0;
  double min_y = -50.0;
  double max_y = 50.0;
  double min_z = -10.0;
  double max_z = 10.0;
};

/// Keep the farthest point of every (azimuth, elevation) cell of the
/// accumulated scan. Near-field clutter such as the launcher frame or the
/// ground directly under the sensor never wins the "farthest point" test in
/// its own cell, so it drops out without any manual masking.
CloudPtr gridFarthestPoint(const std::vector<CloudConstPtr>& frames,
                           double cell_degrees);

/// Keep only points inside the ROI box.
CloudPtr cropRoi(const CloudConstPtr& cloud, const Roi& roi);

/// Voxel downsample with the given leaf size.
CloudPtr voxelDownsample(const CloudConstPtr& cloud, double leaf_size);

/// Run the full chain: grid filter, ROI crop, voxel downsample.
CloudPtr accumulate(const std::vector<CloudConstPtr>& frames,
                    double cell_degrees,
                    const Roi& roi,
                    double leaf_size);

}  // namespace lidar_calibrate

#endif  // LIDAR_CALIBRATE_ACCUMULATOR_HPP
