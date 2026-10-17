// Interactive corner picking on an image with OpenCV highgui.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace combined_calibrate {

struct PickOutcome {
  bool accepted = false;
  bool aborted = false;
  std::vector<cv::Point2d> points;  ///< full resolution image coordinates
};

/// Show `bgr` and let the user click `count` corners in a fixed order.
/// Left click adds, right click removes the last one, 'u' undoes, 'r' resets,
/// ENTER accepts, ESC aborts.  The window is scaled down to `max_width`.
PickOutcome pickCornersOnImage(const cv::Mat &bgr, int count,
                               const std::vector<std::string> &labels,
                               const std::string &window_title, int max_width = 1600);

/// The same annotations the picker draws, so the preview PNG looks familiar.
cv::Mat drawCornerAnnotations(const cv::Mat &bgr, const std::vector<cv::Point2d> &points,
                              const std::vector<std::string> &labels);

}  // namespace combined_calibrate
