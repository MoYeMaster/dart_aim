#include "detector/ballistics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_auto_aim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kBinarySearchIterations = 40;
constexpr double kHeightToleranceM = 0.01;
}

BallisticCalculator::BallisticCalculator(const BallisticParams & params)
: params_(params)
{
}

double BallisticCalculator::calculateRequiredSpeed(double distance_m) const
{
  if (!isValid() || distance_m <= 0.0) {
    return 0.0;
  }

  const double angle_rad = params_.launch_angle_deg * kPi / 180.0;
  const double cos_angle = std::cos(angle_rad);
  const double tan_angle = std::tan(angle_rad);
  const double no_drag_denominator =
    2.0 * cos_angle * cos_angle *
    (distance_m * tan_angle - params_.target_height_difference_m);

  if (params_.air_density_kg_m3 <= 0.0 ||
    params_.drag_coefficient <= 0.0 || params_.cross_section_area_m2 <= 0.0)
  {
    if (no_drag_denominator <= 0.0) {
      return 0.0;
    }
    return std::sqrt(
      params_.gravity_m_s2 * distance_m * distance_m /
      no_drag_denominator);
  }

  double previous_speed = 0.0;
  double previous_error = 0.0;
  bool has_previous_sample = false;
  double best_speed = 0.0;
  double best_error = std::numeric_limits<double>::infinity();

  const double speed_step = std::max(0.5, params_.max_initial_speed_mps / 400.0);
  for (double speed = speed_step; speed <= params_.max_initial_speed_mps;
    speed += speed_step)
  {
    double height_m = 0.0;
    if (!simulateHeight(speed, distance_m, height_m)) {
      continue;
    }

    const double error = height_m - params_.target_height_difference_m;
    if (std::abs(error) < best_error) {
      best_error = std::abs(error);
      best_speed = speed;
    }

    if (has_previous_sample && previous_error * error <= 0.0) {
      double lower_speed = previous_speed;
      double upper_speed = speed;
      double lower_error = previous_error;

      for (int i = 0; i < kBinarySearchIterations; ++i) {
        const double middle_speed = 0.5 * (lower_speed + upper_speed);
        double middle_height_m = 0.0;
        if (!simulateHeight(middle_speed, distance_m, middle_height_m)) {
          lower_speed = middle_speed;
          continue;
        }

        const double middle_error =
          middle_height_m - params_.target_height_difference_m;
        if (std::abs(middle_error) < kHeightToleranceM) {
          return middle_speed;
        }

        if (lower_error * middle_error <= 0.0) {
          upper_speed = middle_speed;
        } else {
          lower_speed = middle_speed;
          lower_error = middle_error;
        }
      }
      return 0.5 * (lower_speed + upper_speed);
    }

    previous_speed = speed;
    previous_error = error;
    has_previous_sample = true;
  }

  if (best_speed > 0.0 && best_error <= kHeightToleranceM) {
    return best_speed;
  }
  return 0.0;
}

double BallisticCalculator::calculateRequiredForce(double distance_m) const
{
  if (!isValid()) {
    return 0.0;
  }

  const double required_speed_mps = calculateRequiredSpeed(distance_m);
  if (required_speed_mps <= 0.0) {
    return 0.0;
  }

  return 0.5 * params_.dart_mass_kg * required_speed_mps * required_speed_mps /
         (params_.spring_effective_stroke_m * params_.launch_efficiency);
}

bool BallisticCalculator::isValid() const
{
  const double angle_rad = params_.launch_angle_deg * kPi / 180.0;
  return std::isfinite(params_.dart_mass_kg) &&
         std::isfinite(params_.cross_section_area_m2) &&
         std::isfinite(params_.drag_coefficient) &&
         std::isfinite(params_.air_density_kg_m3) &&
         std::isfinite(params_.target_height_difference_m) &&
         std::isfinite(params_.launch_angle_deg) &&
         std::isfinite(params_.spring_effective_stroke_m) &&
         std::isfinite(params_.launch_efficiency) &&
         std::isfinite(params_.gravity_m_s2) &&
         std::isfinite(params_.integration_time_step_s) &&
         std::isfinite(params_.max_initial_speed_mps) &&
         params_.dart_mass_kg > 0.0 &&
         params_.cross_section_area_m2 >= 0.0 &&
         params_.drag_coefficient >= 0.0 &&
         params_.air_density_kg_m3 >= 0.0 &&
         params_.gravity_m_s2 > 0.0 &&
         params_.integration_time_step_s > 0.0 &&
         params_.max_initial_speed_mps > 0.0 &&
         params_.spring_effective_stroke_m > 0.0 &&
         params_.launch_efficiency > 0.0 &&
         params_.launch_efficiency <= 1.0 &&
         params_.launch_angle_deg > 0.0 &&
         params_.launch_angle_deg < 90.0 &&
         std::abs(std::cos(angle_rad)) > 1e-9;
}

bool BallisticCalculator::simulateHeight(
  double initial_speed_mps, double distance_m, double & height_m) const
{
  if (initial_speed_mps <= 0.0 || distance_m <= 0.0) {
    return false;
  }

  const double angle_rad = params_.launch_angle_deg * kPi / 180.0;
  const double drag_factor = 0.5 * params_.air_density_kg_m3 *
    params_.drag_coefficient * params_.cross_section_area_m2 /
    params_.dart_mass_kg;
  const double dt = params_.integration_time_step_s;
  const double max_time_s = 20.0;

  double x_m = 0.0;
  double y_m = 0.0;
  double vx_mps = initial_speed_mps * std::cos(angle_rad);
  double vy_mps = initial_speed_mps * std::sin(angle_rad);

  for (double time_s = 0.0; time_s < max_time_s; time_s += dt) {
    const double previous_x_m = x_m;
    const double previous_y_m = y_m;
    const double speed_mps = std::hypot(vx_mps, vy_mps);
    const double acceleration_x_mps2 = -drag_factor * speed_mps * vx_mps;
    const double acceleration_y_mps2 =
      -params_.gravity_m_s2 - drag_factor * speed_mps * vy_mps;

    vx_mps += acceleration_x_mps2 * dt;
    vy_mps += acceleration_y_mps2 * dt;
    x_m += vx_mps * dt;
    y_m += vy_mps * dt;

    if (x_m >= distance_m) {
      const double x_delta_m = x_m - previous_x_m;
      if (x_delta_m <= 1e-12) {
        height_m = y_m;
      } else {
        const double interpolation =
          (distance_m - previous_x_m) / x_delta_m;
        height_m = previous_y_m + interpolation * (y_m - previous_y_m);
      }
      return std::isfinite(height_m);
    }
  }

  return false;
}

}  // namespace rm_auto_aim
