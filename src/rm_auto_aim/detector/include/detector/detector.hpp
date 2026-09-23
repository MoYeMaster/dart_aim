#ifndef DETECTOR__DETECTOR_HPP_
#define DETECTOR__DETECTOR_HPP_

#include <opencv2/core.hpp>

#include <vector>

namespace rm_auto_aim
{

struct BaseLight
{
  cv::Point2f center;
  cv::RotatedRect ellipse;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float diameter_px = 0.0F;
  float area_px = 0.0F;
  float circularity = 0.0F;
  float fill_ratio = 0.0F;
  float score = 0.0F;// 没有神经网络，所以这个是几何上的分数
};

class Detector
{
public:
  struct Params
  {
    int threshold = 180;
    int blur_kernel_size = 3;
    int open_kernel_size = 3;
    int close_kernel_size = 3;

    double min_area = 4.0;
    double max_area = 5000.0;
    double min_diameter_px = 3.0;
    double max_diameter_px = 200.0;
    double min_circularity = 0.50;
    double min_fill_ratio = 0.40;
    double max_aspect_ratio = 2.0;

    double roi_x_min = 0.0;
    double roi_x_max = 1.0;
    double roi_y_min = 0.0;
    double roi_y_max = 1.0;
  };

  Detector();

  explicit Detector(const Params & params);

  std::vector<BaseLight> detect(const cv::Mat & img) const;

  cv::Mat preprocess(const cv::Mat & img) const;

private:
  bool inRoi(const cv::Point2f & point, const cv::Size & image_size) const;

  Params params_;
};

}  // namespace rm_auto_aim

#endif  // DETECTOR__DETECTOR_HPP_
