#ifndef DART_DETECTOR__DART_DETECTOR_HPP_
#define DART_DETECTOR__DART_DETECTOR_HPP_

#include <opencv2/core.hpp>

#include <vector>

#include "auto_aim_interfaces/msg/dart_detection.hpp"

namespace rm_auto_aim
{

enum DartType{
  RED,
  BLUE,
  GREEN,
  OTHER,
};

class DartDetector
{
public:
  struct Params
  {
    int threshold = 180;
    int blur_kernel_size = 3;
    int open_kernel_size = 3;

    double min_area = 4.0;
    double max_area = 5000.0;
    double min_short_side_px = 3.0;
    double max_short_side_px = 200.0;
    double min_aspect_ratio = 1.5;
    double max_aspect_ratio = 8.0;
    double min_fill_ratio = 0.30;

    double roi_x_min = 0.0;
    double roi_x_max = 1.0;
    double roi_y_min = 0.0;
    double roi_y_max = 1.0;
  };

  DartDetector();

  explicit DartDetector(const Params & params);

  std::vector<auto_aim_interfaces::msg::DartDetection> detect(const cv::Mat & img_color,const cv::Mat & img_gray) const;

  cv::Mat preprocess(const cv::Mat & img) const;

private:
  bool inRoi(const cv::Point2f & point, const cv::Size & image_size) const;
  DartType detectDartType(const cv::Mat &color_frame, const cv::RotatedRect box) const;

  
  Params params_;
};

}  // namespace rm_auto_aim

#endif  // DART_DETECTOR__DART_DETECTOR_HPP_
