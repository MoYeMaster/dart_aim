#ifndef DETECTOR__DART_STATE_HPP_
#define DETECTOR__DART_STATE_HPP_

#include <cmath>

namespace rm_auto_aim
{

struct DartStateResult
{
  int states = 0;
  bool is_middle = false;
};

inline DartStateResult classifyDartState(
  double target_x_px, double image_width_px, double middle_error_ratio)
{
  DartStateResult result;
  if (!std::isfinite(target_x_px) || !std::isfinite(image_width_px) ||
    !std::isfinite(middle_error_ratio) || image_width_px <= 0.0 ||
    middle_error_ratio < 0.0)
  {
    return result;
  }

  const double normalized_error =
    (target_x_px - image_width_px * 0.5) / image_width_px;
  if (std::abs(normalized_error) <= middle_error_ratio) {
    result.is_middle = true;
    return result;
  }

  result.states = normalized_error < 0.0 ? -1 : 1;
  return result;
}

}  // namespace rm_auto_aim

#endif  // DETECTOR__DART_STATE_HPP_
