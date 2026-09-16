#include "detector/pnp.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>

namespace rm_auto_aim
{

PnpSolver::PnpSolver(double target_diameter_m)
: target_diameter_m_(target_diameter_m)
{
}

PnpSolver::PnpSolver(
  const std::array<double, 9> & camera_matrix,
  const std::vector<double> & distortion_coefficients,
  double target_diameter_m)
: target_diameter_m_(target_diameter_m)
{
  setCameraInfo(camera_matrix, distortion_coefficients);
}

void PnpSolver::setTargetDiameter(double target_diameter_m)
{
  target_diameter_m_ = target_diameter_m;
}

void PnpSolver::setCameraInfo(
  const std::array<double, 9> & camera_matrix,
  const std::vector<double> & distortion_coefficients)
{
  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
    camera_matrix[0], camera_matrix[1], camera_matrix[2],
    camera_matrix[3], camera_matrix[4], camera_matrix[5],
    camera_matrix[6], camera_matrix[7], camera_matrix[8]);

  if (distortion_coefficients.empty()) {
    distortion_coefficients_ = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    distortion_coefficients_ = cv::Mat(distortion_coefficients, true).reshape(1, 1);
  }
}

bool PnpSolver::hasCameraInfo() const
{
  return hasValidCameraModel();
}

double PnpSolver::estimateDistance(double diameter_px) const
{
  if (!hasValidCameraModel() || target_diameter_m_ <= 0.0 || diameter_px <= 0.0) {
    return 0.0;
  }

  const double focal_length_px = 0.5 * (camera_matrix_.at<double>(0, 0) +
    camera_matrix_.at<double>(1, 1));
  return focal_length_px * target_diameter_m_ / diameter_px;
}

bool PnpSolver::solve(const BaseLight & light, PnpResult & result) const
{
  result = PnpResult{};
  if (!hasValidCameraModel() || target_diameter_m_ <= 0.0 ||
    light.ellipse.size.width <= 0.0F || light.ellipse.size.height <= 0.0F)
  {
    return false;
  }

  const double radius_m = target_diameter_m_ * 0.5;
  const std::vector<cv::Point3f> object_points{
    cv::Point3f(-static_cast<float>(radius_m), 0.0F, 0.0F),
    cv::Point3f(static_cast<float>(radius_m), 0.0F, 0.0F),
    cv::Point3f(0.0F, -static_cast<float>(radius_m), 0.0F),
    cv::Point3f(0.0F, static_cast<float>(radius_m), 0.0F)};

  const double angle_rad = light.ellipse.angle * CV_PI / 180.0;
  const cv::Point2f horizontal_axis(
    static_cast<float>(std::cos(angle_rad) * light.ellipse.size.width * 0.5),
    static_cast<float>(std::sin(angle_rad) * light.ellipse.size.width * 0.5));
  const cv::Point2f vertical_axis(
    static_cast<float>(-std::sin(angle_rad) * light.ellipse.size.height * 0.5),
    static_cast<float>(std::cos(angle_rad) * light.ellipse.size.height * 0.5));
  const std::vector<cv::Point2f> image_points{
    light.center - horizontal_axis,
    light.center + horizontal_axis,
    light.center - vertical_axis,
    light.center + vertical_axis};

  cv::Mat rvec;
  cv::Mat tvec;
  try {
    if (!cv::solvePnP(
        object_points, image_points, camera_matrix_, distortion_coefficients_, rvec, tvec, false,
        cv::SOLVEPNP_IPPE))
    {
      return false;
    }
  } catch (const cv::Exception &) {
    return false;
  }

  if (tvec.rows != 3 || tvec.cols != 1) {
    return false;
  }

  const double x = tvec.at<double>(0, 0);
  const double y = tvec.at<double>(1, 0);
  const double z = tvec.at<double>(2, 0);
  const double distance = std::sqrt(x * x + y * y + z * z);
  if (!std::isfinite(distance) || z <= 0.0) {
    return false;
  }

  result.valid = true;
  result.distance_m = distance;
  result.position_x_m = x;
  result.position_y_m = y;
  result.position_z_m = z;
  result.rvec = rvec;
  result.tvec = tvec;
  return true;
}

bool PnpSolver::hasValidCameraModel() const
{
  return camera_matrix_.rows == 3 && camera_matrix_.cols == 3 &&
         camera_matrix_.type() == CV_64F &&
         camera_matrix_.at<double>(0, 0) > 0.0 &&
         camera_matrix_.at<double>(1, 1) > 0.0 &&
         camera_matrix_.at<double>(2, 2) > 0.0;
}

}  // namespace rm_auto_aim
