#ifndef DETECTOR__PNP_HPP_
#define DETECTOR__PNP_HPP_

#include <opencv2/core.hpp>

#include <array>
#include <vector>

#include "detector/detector.hpp"

namespace rm_auto_aim
{

struct PnpResult
{
  bool valid = false;
  double distance_m = 0.0;
  double position_x_m = 0.0;
  double position_y_m = 0.0;
  double position_z_m = 0.0;
  cv::Mat rvec;
  cv::Mat tvec;
};

class PnpSolver
{
public:
  explicit PnpSolver(double target_diameter_m = 0.055);

  PnpSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients,
    double target_diameter_m = 0.055);

  void setTargetDiameter(double target_diameter_m);

  void setCameraInfo(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  bool hasCameraInfo() const;

  double estimateDistance(double diameter_px) const;

  bool solve(const BaseLight & light, PnpResult & result) const;

private:
  bool hasValidCameraModel() const;

  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  double target_diameter_m_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__PNP_HPP_
