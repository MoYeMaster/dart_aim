#include "lidar_dart_cpp/map_icp.hpp"

#include "lidar_dart_cpp/coordinate_transform.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <memory>

namespace lidar_dart {

Pose localize(const CloudConstPtr& scan,
              const CloudConstPtr& map,
              const Pose& init_pose,
              int max_iterations,
              double transformation_epsilon,
              double max_correspondence_dist,
              IcpResult& result) {
  result = IcpResult{};
  if (!scan || !map || scan->size() < 3 || map->size() < 3) {
    return init_pose;
  }

  const CloudPtr source_aligned = transformCloud(scan, init_pose);
  auto aligned = std::make_shared<Cloud>();

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(source_aligned);
  icp.setInputTarget(map);
  icp.setMaximumIterations(max_iterations);
  icp.setTransformationEpsilon(transformation_epsilon);
  icp.setMaxCorrespondenceDistance(max_correspondence_dist);
  icp.align(*aligned);

  result.converged = icp.hasConverged();
  result.fitness = icp.getFitnessScore(max_correspondence_dist);
  result.inlier_rmse = std::sqrt(result.fitness);

  Pose icp_pose = Pose::Identity();
  icp_pose.matrix() = icp.getFinalTransformation().cast<double>();
  return init_pose * icp_pose;
}

CloudPtr loadPCD(const std::string& path) {
  auto cloud = std::make_shared<Cloud>();
  if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) {
    return nullptr;
  }
  return cloud;
}

}  // namespace lidar_dart
