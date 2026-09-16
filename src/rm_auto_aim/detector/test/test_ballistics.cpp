#include <gtest/gtest.h>

#include <cmath>

#include "detector/ballistics.hpp"

namespace rm_auto_aim
{
namespace
{
BallisticParams makeTestParams()
{
  BallisticParams params;
  params.dart_mass_kg = 0.05;
  params.cross_section_area_m2 = 0.0001;
  params.drag_coefficient = 0.47;
  params.target_height_difference_m = 0.0;
  params.launch_angle_deg = 45.0;
  params.spring_effective_stroke_m = 0.10;
  params.launch_efficiency = 0.80;
  return params;
}
}  // namespace

TEST(BallisticCalculatorTest, CalculatesForceFromRequiredSpeedWithoutDrag)
{
  auto params = makeTestParams();
  params.air_density_kg_m3 = 0.0;
  BallisticCalculator calculator(params);

  const double speed_mps = calculator.calculateRequiredSpeed(10.0);
  const double force_n = calculator.calculateRequiredForce(10.0);

  EXPECT_NEAR(speed_mps, std::sqrt(9.81 * 10.0), 0.25);
  EXPECT_NEAR(
    force_n, 0.5 * params.dart_mass_kg * speed_mps * speed_mps /
    (params.spring_effective_stroke_m * params.launch_efficiency), 0.01);
}

TEST(BallisticCalculatorTest, AirResistanceIncreasesRequiredForce)
{
  auto no_drag_params = makeTestParams();
  no_drag_params.air_density_kg_m3 = 0.0;
  BallisticCalculator no_drag_calculator(no_drag_params);

  auto drag_params = makeTestParams();
  BallisticCalculator drag_calculator(drag_params);

  EXPECT_GT(
    drag_calculator.calculateRequiredSpeed(10.0),
    no_drag_calculator.calculateRequiredSpeed(10.0));
  EXPECT_GT(
    drag_calculator.calculateRequiredForce(10.0),
    no_drag_calculator.calculateRequiredForce(10.0));
}

TEST(BallisticCalculatorTest, InvalidParametersReturnZero)
{
  auto params = makeTestParams();
  params.dart_mass_kg = 0.0;
  BallisticCalculator calculator(params);

  EXPECT_DOUBLE_EQ(calculator.calculateRequiredSpeed(10.0), 0.0);
  EXPECT_DOUBLE_EQ(calculator.calculateRequiredForce(10.0), 0.0);
}
}  // namespace rm_auto_aim
