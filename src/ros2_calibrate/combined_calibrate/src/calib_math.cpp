#include "combined_calibrate/calib_math.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>

#include "combined_calibrate/calib_io.hpp"

namespace fs = std::filesystem;

namespace combined_calibrate {
namespace {

cv::Mat distortionMat(const Intrinsics &info) {
  if (info.dist.empty()) return {};
  cv::Mat mat(1, static_cast<int>(info.dist.size()), CV_64F);
  for (size_t i = 0; i < info.dist.size(); ++i) mat.at<double>(0, static_cast<int>(i)) = info.dist[i];
  return mat;
}

double meanError(const std::vector<cv::Point3d> &object, const std::vector<cv::Point2d> &image,
                 const Intrinsics &info, const cv::Matx33d &R, const cv::Vec3d &t) {
  if (object.empty()) return 0.0;
  std::vector<cv::Point3f> object_f;
  object_f.reserve(object.size());
  for (const auto &point : object) object_f.emplace_back(float(point.x), float(point.y), float(point.z));
  cv::Vec3d rvec;
  cv::Rodrigues(R, rvec);
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_f, rvec, t, cv::Mat(info.K), distortionMat(info), projected);
  double sum = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const double dx = double(projected[i].x) - image[i].x;
    const double dy = double(projected[i].y) - image[i].y;
    sum += std::sqrt(dx * dx + dy * dy);
  }
  return sum / double(projected.size());
}

bool solveOnce(const std::vector<Shot> &shots, const Intrinsics &info, cv::Matx33d &R, cv::Vec3d &t,
               std::string *error) {
  std::vector<cv::Point3f> object;
  std::vector<cv::Point2f> image;
  for (const auto &shot : shots) {
    for (size_t i = 0; i < shot.lidar.size(); ++i) {
      object.emplace_back(float(shot.lidar[i].x), float(shot.lidar[i].y), float(shot.lidar[i].z));
      image.emplace_back(float(shot.image[i].x), float(shot.image[i].y));
    }
  }
  if (object.size() < 4) {
    if (error) *error = "need at least 4 corner pairs, got " + std::to_string(object.size());
    return false;
  }

  const cv::Mat dist = distortionMat(info);
  const cv::Mat camera = cv::Mat(info.K);

  // A single planar pose has more than one pose that fits the image points, so
  // try every initialisation OpenCV offers, refine each one and keep the
  // candidate with the smallest reprojection error.
  const int flags[] = {cv::SOLVEPNP_SQPNP, cv::SOLVEPNP_EPNP, cv::SOLVEPNP_IPPE,
                       cv::SOLVEPNP_ITERATIVE};
  bool found = false;
  double best_error = 0.0;
  cv::Matx33d best_R = cv::Matx33d::eye();
  cv::Vec3d best_t;

  for (const int flag : flags) {
    cv::Mat rvec;
    cv::Mat tvec;
    bool ok = false;
    try {
      ok = cv::solvePnP(object, image, camera, dist, rvec, tvec, false, flag);
    } catch (const cv::Exception &exception) {
      ok = false;
    }
    if (!ok || rvec.empty() || tvec.empty()) continue;

    try {
      cv::solvePnPRefineLM(object, image, camera, dist, rvec, tvec);
    } catch (const cv::Exception &exception) {
      // keep the unrefined solution, the error below decides if it wins
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object, rvec, tvec, camera, dist, projected);
    if (projected.size() != image.size()) continue;
    double sum = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
      const double dx = double(projected[i].x) - image[i].x;
      const double dy = double(projected[i].y) - image[i].y;
      sum += std::sqrt(dx * dx + dy * dy);
    }
    const double mean = sum / double(projected.size());
    if (!found || mean < best_error) {
      cv::Mat rotation;
      cv::Rodrigues(rvec, rotation);
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) best_R(row, col) = rotation.at<double>(row, col);
      }
      best_t = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
      best_error = mean;
      found = true;
    }
  }

  if (!found) {
    if (error) *error = "solvePnP failed, check the corner pairing";
    return false;
  }
  R = best_R;
  t = best_t;
  return true;
}

}  // namespace

cv::Matx44d composePose(const cv::Matx33d &R, const cv::Vec3d &t) {
  cv::Matx44d T = cv::Matx44d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) T(row, col) = R(row, col);
    T(row, 3) = t(row);
  }
  return T;
}

cv::Matx44d invertPose(const cv::Matx44d &T) {
  cv::Matx44d inverse = cv::Matx44d::eye();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) inverse(row, col) = T(col, row);
  }
  for (int row = 0; row < 3; ++row) {
    double value = 0.0;
    for (int col = 0; col < 3; ++col) value -= T(col, row) * T(col, 3);
    inverse(row, 3) = value;
  }
  return inverse;
}

cv::Vec3d rotationToRpyDegrees(const cv::Matx33d &R) {
  const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  if (sy > 1e-9) {
    roll = std::atan2(R(2, 1), R(2, 2));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = std::atan2(-R(1, 2), R(1, 1));
    pitch = std::atan2(-R(2, 0), sy);
    yaw = 0.0;
  }
  constexpr double kRadToDeg = 180.0 / M_PI;
  return cv::Vec3d(roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg);
}

bool adjustIntrinsicsForImage(Intrinsics &info, int image_width, int image_height,
                              std::string *message) {
  if (image_width <= 0 || image_height <= 0) return false;
  const bool same_size = info.width == image_width && info.height == image_height;
  if (same_size) return false;
  if (info.width <= 0 || info.height <= 0) {
    info.width = image_width;
    info.height = image_height;
    if (message) {
      *message = "camera_info did not declare an image size, using the frame size for the record";
    }
    return false;
  }

  const double shift_x = 0.5 * double(info.width - image_width);
  const double shift_y = 0.5 * double(info.height - image_height);
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2);
  stream << "camera_info is calibrated for " << info.width << "x" << info.height << " but the frame is "
         << image_width << "x" << image_height << "; assuming a centred ROI crop, principal point ("
         << info.K(0, 2) << ", " << info.K(1, 2) << ") -> (" << (info.K(0, 2) - shift_x) << ", "
         << (info.K(1, 2) - shift_y) << ")";
  info.K(0, 2) -= shift_x;
  info.K(1, 2) -= shift_y;
  info.width = image_width;
  info.height = image_height;
  if (message) *message = stream.str();
  return true;
}

std::vector<double> reprojectionErrors(const std::vector<Shot> &shots, const Intrinsics &info,
                                       const cv::Matx33d &R, const cv::Vec3d &t) {
  std::vector<double> errors;
  errors.reserve(shots.size());
  for (const auto &shot : shots) errors.push_back(meanError(shot.lidar, shot.image, info, R, t));
  return errors;
}

bool solveExtrinsic(const std::vector<Shot> &shots, const Intrinsics &info, SolveResult &result,
                    std::string *error, bool reject_outliers) {
  result = SolveResult();
  if (!info.valid()) {
    if (error) *error = "camera intrinsics are missing (need camera_matrix)";
    return false;
  }
  if (shots.empty()) {
    if (error) *error = "no shot with matching corner files";
    return false;
  }

  std::vector<Shot> current = shots;
  cv::Matx33d R = cv::Matx33d::eye();
  cv::Vec3d t;
  for (int pass = 0; pass < 2; ++pass) {
    if (!solveOnce(current, info, R, t, error)) return false;
    if (!reject_outliers || pass == 1) break;

    std::vector<double> errors = reprojectionErrors(current, info, R, t);
    std::vector<double> sorted = errors;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted[sorted.size() / 2];
    const double limit = std::max(3.0, 3.0 * median);
    if (errors.size() < 3) break;

    std::vector<Shot> kept;
    std::vector<std::string> dropped;
    for (size_t i = 0; i < current.size(); ++i) {
      if (errors[i] > limit && current.size() - dropped.size() > 2) {
        dropped.push_back(current[i].name);
      } else {
        kept.push_back(current[i]);
      }
    }
    if (dropped.empty()) break;
    current = kept;
    result.rejected_shots.insert(result.rejected_shots.end(), dropped.begin(), dropped.end());
  }

  result.R = R;
  result.t = t;
  result.per_shot_error_px = reprojectionErrors(current, info, R, t);
  result.used_shots.clear();
  for (const auto &shot : current) result.used_shots.push_back(shot.name);
  result.mean_error_px = 0.0;
  result.worst_error_px = 0.0;
  for (const double value : result.per_shot_error_px) {
    result.mean_error_px += value;
    result.worst_error_px = std::max(result.worst_error_px, value);
  }
  if (!result.per_shot_error_px.empty()) {
    result.mean_error_px /= double(result.per_shot_error_px.size());
  }
  result.ok = true;
  return true;
}

bool loadShots(const std::string &root, std::vector<Shot> &shots, std::vector<std::string> &skipped,
               std::string *error) {
  shots.clear();
  skipped.clear();
  const std::vector<std::string> dirs = shotDirectories(root);
  if (dirs.empty()) {
    if (error) *error = "no shot_* directory under " + root;
    return false;
  }
  for (const std::string &dir : dirs) {
    const std::string name = fs::path(dir).filename().string();
    const std::string image_file = joinPath(dir, "corners_img.txt");
    const std::string lidar_file = joinPath(dir, "corners_lidar.txt");
    if (!fs::exists(image_file) || !fs::exists(lidar_file)) {
      skipped.push_back(name + " (missing corner file)");
      continue;
    }
    Shot shot;
    shot.name = name;
    shot.dir = dir;
    if (!readPoints2(image_file, shot.image) || !readPoints3(lidar_file, shot.lidar)) {
      skipped.push_back(name + " (corner file unreadable)");
      continue;
    }
    if (shot.image.size() != shot.lidar.size()) {
      skipped.push_back(name + " (image/lidar corner count differ)");
      continue;
    }
    if (shot.image.size() < 3) {
      skipped.push_back(name + " (fewer than 3 corners)");
      continue;
    }
    shots.push_back(shot);
  }
  if (shots.empty()) {
    if (error) *error = "no usable shot under " + root + " (pick corners first)";
    return false;
  }
  return true;
}

bool loadShotIntrinsics(const std::string &root, Intrinsics &info, std::string *source) {
  for (const std::string &dir : shotDirectories(root)) {
    const std::string path = joinPath(dir, "intrinsics.yaml");
    if (!fs::exists(path)) continue;
    if (readIntrinsics(path, info, nullptr)) {
      if (source) *source = path;
      return true;
    }
  }
  return false;
}

bool writeExtrinsicsYaml(const std::string &path, const cv::Matx44d &lidar_to_camera,
                         const SolveResult &result, int points_per_shot) {
  const cv::Matx44d camera_to_lidar = invertPose(lidar_to_camera);
  const cv::Vec3d rpy = rotationToRpyDegrees(result.R);

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9);
  stream << "# p_camera = R * p_lidar + t, produced by combined_calibrate (C++)\n";
  stream << "comment: \"p_camera = R * p_lidar + t, produced by combined_calibrate\"\n";
  auto matrix = [&stream](const char *key, const cv::Matx44d &matrix) {
    stream << key << ":\n  rows: 4\n  cols: 4\n  data: [";
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        if (row != 0 || col != 0) stream << ", ";
        stream << matrix(row, col);
      }
    }
    stream << "]\n";
  };
  matrix("T_lidar_to_camera", lidar_to_camera);
  matrix("T_camera_to_lidar", camera_to_lidar);

  stream << std::setprecision(6);
  stream << "translation_m: [" << result.t(0) << ", " << result.t(1) << ", " << result.t(2) << "]\n";
  stream << "rotation_rpy_deg: [" << rpy(0) << ", " << rpy(1) << ", " << rpy(2) << "]\n";
  stream << "translation_norm_m: " << cv::norm(result.t) << "\n";

  stream << std::setprecision(4);
  stream << "mean_reprojection_error_px: " << result.mean_error_px << "\n";
  stream << "worst_shot_reprojection_error_px: " << result.worst_error_px << "\n";
  stream << "points_per_shot: " << points_per_shot << "\n";
  stream << "shot_count: " << result.used_shots.size() << "\n";

  stream << "shots: [";
  for (size_t i = 0; i < result.used_shots.size(); ++i) {
    if (i != 0) stream << ", ";
    stream << result.used_shots[i];
  }
  stream << "]\n";
  stream << "per_shot_error_px: [";
  for (size_t i = 0; i < result.per_shot_error_px.size(); ++i) {
    if (i != 0) stream << ", ";
    stream << result.per_shot_error_px[i];
  }
  stream << "]\n";
  if (!result.rejected_shots.empty()) {
    stream << "outlier_shots: [";
    for (size_t i = 0; i < result.rejected_shots.size(); ++i) {
      if (i != 0) stream << ", ";
      stream << result.rejected_shots[i];
    }
    stream << "]\n";
  }

  if (!ensureDirectory(fs::path(path).parent_path().string())) return false;
  std::ofstream file(path);
  if (!file) return false;
  file << stream.str();
  return true;
}

}  // namespace combined_calibrate
