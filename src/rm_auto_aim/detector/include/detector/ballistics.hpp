#ifndef DETECTOR__BALLISTICS_HPP_
#define DETECTOR__BALLISTICS_HPP_

namespace rm_auto_aim
{

struct BallisticParams
{
  double dart_mass_kg = 0.05;
  double cross_section_area_m2 = 0.0001;
  double drag_coefficient = 0.47;
  double air_density_kg_m3 = 1.225;
  double target_height_difference_m = 0.0;
  double launch_angle_deg = 33.0;
  double spring_effective_stroke_m = 0.10;
  double launch_efficiency = 0.80;
  double gravity_m_s2 = 9.81;
  double integration_time_step_s = 0.001;
  double max_initial_speed_mps = 200.0;
};

class BallisticCalculator
{
public:
  explicit BallisticCalculator(const BallisticParams & params);

  double calculateRequiredSpeed(double distance_m) const;

  double calculateRequiredForce(double distance_m) const;

private:
  bool isValid() const;

  bool simulateHeight(
    double initial_speed_mps, double distance_m, double & height_m) const;

  BallisticParams params_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__BALLISTICS_HPP_
