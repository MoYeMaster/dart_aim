// The calibration solver: same least-squares problem as the Livox official
// getExt1 node (minimise the reprojection error of the LiDAR target corners),
// solved with OpenCV's solvePnP + LM refinement.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "combined_calibrate/calib_types.hpp"

namespace combined_calibrate {

/// Camera drivers usually publish `camera_info` for the full sensor while the
/// image is a centred ROI crop.  Move the principal point so the intrinsics
/// match the image that is actually delivered.
/// Returns true when anything was changed; `message` gets a human readable
/// description of the adjustment.
bool adjustIntrinsicsForImage(Intrinsics &info, int image_width, int image_height,
                              std::string *message = nullptr);

/// Solve T_lidar_to_camera from shots that have both corner files.
/// `info` overrides the per-shot intrinsics when valid.
bool solveExtrinsic(const std::vector<Shot> &shots, const Intrinsics &info, SolveResult &result,
                    std::string *error = nullptr, bool reject_outliers = true);

/// Load every complete shot below `root`.
bool loadShots(const std::string &root, std::vector<Shot> &shots, std::vector<std::string> &skipped,
               std::string *error = nullptr);

/// Read the intrinsics stored next to the shots (first one that has them).
bool loadShotIntrinsics(const std::string &root, Intrinsics &info, std::string *source = nullptr);

/// Write the result in the same YAML layout the previous tool used, so the
/// file can still be fed to the dart rig parameter chain.
bool writeExtrinsicsYaml(const std::string &path, const cv::Matx44d &lidar_to_camera,
                         const SolveResult &result, int points_per_shot);

/// Mean reprojection error per shot in pixels.
std::vector<double> reprojectionErrors(const std::vector<Shot> &shots, const Intrinsics &info,
                                       const cv::Matx33d &R, const cv::Vec3d &t);

}  // namespace combined_calibrate
