/// Offline solver: read the shots from disk and write extrinsics.yaml.
///
///   ros2 run combined_calibrate solve_extrinsic ~/dart_calibration/shots
///
/// The result points from the LiDAR frame to the camera frame:
/// p_camera = R * p_lidar + t.
#include <opencv2/imgcodecs.hpp>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "combined_calibrate/calib_io.hpp"
#include "combined_calibrate/calib_math.hpp"

namespace cc = combined_calibrate;

namespace {

void usage() {
  std::cout << "usage: solve_extrinsic <shots_dir> [options]\n"
               "  --output <file>            result YAML (default <shots_dir>/extrinsics.yaml)\n"
               "  --intrinsics <file>        use this camera_matrix/distortion instead of the shots\n"
               "  --no-outlier-rejection     keep every shot, even the obvious outliers\n"
               "  --no-fix-roi               do not shift the principal point for a ROI crop\n"
            << std::endl;
}

}  // namespace

int main(int argc, char **argv) {
  std::string shots_dir;
  std::string output;
  std::string intrinsics_file;
  bool reject_outliers = true;
  bool fix_roi = true;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--output" && i + 1 < argc) {
      output = argv[++i];
    } else if (arg == "--intrinsics" && i + 1 < argc) {
      intrinsics_file = argv[++i];
    } else if (arg == "--no-outlier-rejection") {
      reject_outliers = false;
    } else if (arg == "--no-fix-roi") {
      fix_roi = false;
    } else if (arg == "-h" || arg == "--help") {
      usage();
      return 0;
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "unknown option " << arg << std::endl;
      usage();
      return 1;
    } else {
      shots_dir = arg;
    }
  }
  if (shots_dir.empty()) {
    usage();
    return 1;
  }

  std::vector<cc::Shot> shots;
  std::vector<std::string> skipped;
  std::string error;
  if (!cc::loadShots(shots_dir, shots, skipped, &error)) {
    std::cerr << error << std::endl;
    return 1;
  }
  for (const std::string &entry : skipped) std::cerr << "skipped " << entry << std::endl;

  cc::Intrinsics info;
  if (!intrinsics_file.empty()) {
    if (!cc::readIntrinsics(intrinsics_file, info, &error)) {
      std::cerr << error << std::endl;
      return 1;
    }
  } else if (!cc::loadShotIntrinsics(shots_dir, info)) {
    std::cerr << "no intrinsics.yaml under " << shots_dir
              << ", pass --intrinsics with a camera_info style file" << std::endl;
    return 1;
  }

  if (fix_roi) {
    cv::Mat probe;
    if (cc::readImage(cc::joinPath(shots.front().dir, "image.png"), probe)) {
      std::string message;
      if (cc::adjustIntrinsicsForImage(info, probe.cols, probe.rows, &message) && !message.empty()) {
        std::cout << "note: " << message << std::endl;
      }
    }
  }

  cc::SolveResult result;
  if (!cc::solveExtrinsic(shots, info, result, &error, reject_outliers)) {
    std::cerr << "solve failed: " << error << std::endl;
    return 1;
  }

  std::cout << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < result.used_shots.size(); ++i) {
    std::cout << result.used_shots[i] << ": mean reprojection error " << result.per_shot_error_px[i]
              << " px" << std::endl;
  }
  for (const std::string &name : result.rejected_shots) {
    std::cout << name << ": rejected as an outlier" << std::endl;
  }
  std::cout << "overall: mean " << result.mean_error_px << " px, worst shot " << result.worst_error_px
            << " px over " << result.used_shots.size() << " shots" << std::endl;
  if (result.mean_error_px > 1.0) {
    std::cout << "warning: a healthy result is around or below 1 px; re-check the corner picks"
              << std::endl;
  }

  int points_per_shot = 0;
  for (const auto &shot : shots) points_per_shot = std::max(points_per_shot, int(shot.lidar.size()));
  const cv::Matx44d T = cc::composePose(result.R, result.t);
  const std::string path = output.empty() ? cc::joinPath(shots_dir, "extrinsics.yaml") : output;
  if (!cc::writeExtrinsicsYaml(path, T, result, points_per_shot)) {
    std::cerr << "cannot write " << path << std::endl;
    return 1;
  }
  const cv::Vec3d rpy = cc::rotationToRpyDegrees(result.R);
  std::cout << "wrote " << path << std::endl;
  std::cout << "T_lidar_to_camera = [" << std::endl;
  for (int row = 0; row < 4; ++row) {
    std::cout << "  ";
    for (int col = 0; col < 4; ++col) std::cout << std::setw(12) << T(row, col) << " ";
    std::cout << std::endl;
  }
  std::cout << "]" << std::endl;
  std::cout << "translation " << cv::norm(result.t) << " m, rpy " << rpy(0) << " " << rpy(1) << " "
            << rpy(2) << " deg" << std::endl;

  return 0;
}
