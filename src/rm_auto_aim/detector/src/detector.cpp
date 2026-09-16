#include "detector/detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{
namespace
{
int normalizeKernelSize(int size)
{
  if (size <= 1) {
    return 0;
  }
  return size % 2 == 0 ? size + 1 : size;
}

void applyMorphology(cv::Mat & image, int kernel_size, int operation)
{
  const int normalized_size = normalizeKernelSize(kernel_size);
  if (normalized_size == 0) {
    return;
  }

  const auto kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(normalized_size, normalized_size));
  cv::morphologyEx(image, image, operation, kernel);
}
}  // namespace

Detector::Detector()
: params_{}
{
}

Detector::Detector(const Params & params)
: params_(params)
{
}

cv::Mat Detector::preprocess(const cv::Mat & img) const
{
  CV_Assert(img.type() == CV_8UC1);

  cv::Mat filtered = img;
  const int blur_size = normalizeKernelSize(params_.blur_kernel_size);
  if (blur_size > 1) {
    cv::GaussianBlur(
      img, filtered, cv::Size(blur_size, blur_size), 0.0, 0.0, cv::BORDER_REPLICATE);
  }

  cv::Mat binary;
  cv::threshold(filtered, binary, params_.threshold, 255, cv::THRESH_BINARY);
  applyMorphology(binary, params_.open_kernel_size, cv::MORPH_OPEN);
  applyMorphology(binary, params_.close_kernel_size, cv::MORPH_CLOSE);
  return binary;
}

std::vector<BaseLight> Detector::detect(const cv::Mat & img) const
{
  if (img.empty()) {
    return {};
  }
  CV_Assert(img.type() == CV_8UC1);

  const cv::Mat binary = preprocess(img);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::drawContours(img, contours, -1, cv::Scalar(0, 0, 255), 0.5, cv::LINE_8);

  std::vector<BaseLight> detections;
  detections.reserve(contours.size());

  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < params_.min_area || area > params_.max_area || contour.size() < 5U) {
      continue;
    }

    const cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) < 1e-9) {
      continue;
    }
    const cv::Point2f center(
      static_cast<float>(moments.m10 / moments.m00),
      static_cast<float>(moments.m01 / moments.m00));
    if (!inRoi(center, img.size())) {
      continue;
    }

    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 1e-6) {
      continue;
    }

    const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    if (circularity < params_.min_circularity) {
      continue;
    }

    const cv::RotatedRect ellipse = cv::fitEllipse(contour);
    const double ellipse_area =
      CV_PI * ellipse.size.width * ellipse.size.height / 4.0;
    if (ellipse_area <= 1e-6) {
      continue;
    }

    const double fill_ratio = area / ellipse_area;
    if (fill_ratio < params_.min_fill_ratio) {
      continue;
    }

    const cv::RotatedRect min_rect = cv::minAreaRect(contour);
    const double short_side = std::min(min_rect.size.width, min_rect.size.height);
    const double long_side = std::max(min_rect.size.width, min_rect.size.height);
    if (short_side <= 1e-6 || long_side / short_side > params_.max_aspect_ratio) {
      continue;
    }

    float radius = 0.0F;
    cv::Point2f enclosing_center;
    cv::minEnclosingCircle(contour, enclosing_center, radius);
    const double diameter = 2.0 * radius;
    if (diameter < params_.min_diameter_px || diameter > params_.max_diameter_px) {
      continue;
    }

    BaseLight detection;
    detection.center = center;
    detection.ellipse = ellipse;
    detection.diameter_px = static_cast<float>(diameter);
    detection.area_px = static_cast<float>(area);
    detection.circularity = static_cast<float>(circularity);
    detection.fill_ratio = static_cast<float>(fill_ratio);
    detection.score = static_cast<float>(circularity * fill_ratio);
    detections.push_back(detection);
  }

  std::sort(
    detections.begin(), detections.end(),
    [](const BaseLight & lhs, const BaseLight & rhs) {
      return lhs.score > rhs.score;
    });
  return detections;
}

bool Detector::inRoi(const cv::Point2f & point, const cv::Size & image_size) const
{
  const double x_min = params_.roi_x_min * image_size.width;
  const double x_max = params_.roi_x_max * image_size.width;
  const double y_min = params_.roi_y_min * image_size.height;
  const double y_max = params_.roi_y_max * image_size.height;
  return point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max;
}

}  // namespace rm_auto_aim
