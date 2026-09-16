#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <cmath>
#include <vector>

#include "detector/pnp.hpp"

namespace rm_auto_aim
{
namespace
{
BaseLight makeSyntheticLight()
{
  BaseLight light;
  light.center = cv::Point2f(640.0F, 360.0F);
  light.ellipse = cv::RotatedRect(light.center, cv::Size2f(22.0F, 22.0F), 0.0F);
  light.diameter_px = 22.0F;
  light.area_px = 380.0F;
  light.circularity = 0.95F;
  light.fill_ratio = 0.95F;
  light.score = 0.90F;
  return light;
}
}  // namespace

TEST(PnpSolverTest, EstimatesDistanceFromKnownCircularTarget)
{
  const std::array<double, 9> camera_matrix{
    800.0, 0.0, 640.0,
    0.0, 800.0, 360.0,
    0.0, 0.0, 1.0};
  const std::vector<double> distortion_coefficients(5, 0.0);
  PnpSolver solver(camera_matrix, distortion_coefficients, 0.055);

  PnpResult result;
  ASSERT_TRUE(solver.solve(makeSyntheticLight(), result));
  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.distance_m, 2.0, 0.25);
  EXPECT_GT(result.position_z_m, 0.0);
}

TEST(PnpSolverTest, RejectsInvalidCameraModel)
{
  PnpSolver solver;
  PnpResult result;
  EXPECT_FALSE(solver.solve(makeSyntheticLight(), result));
  EXPECT_FALSE(result.valid);
}

TEST(PnpSolverTest, UsesPixelDiameterFallbackWhenPnpCannotBeSolved)
{
  PnpSolver solver;
  const double distance_m = solver.estimateDistance(22.0);

  EXPECT_NEAR(distance_m, 0.0, 1e-9);
}
}  // namespace rm_auto_aim
