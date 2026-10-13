#include "dart_detector/dart_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace rm_auto_aim
{
  namespace
  {
    int normalizeKernelSize(int size)
    {
      if (size <= 1)
      {
        return 0;
      }
      return size % 2 == 0 ? size + 1 : size;
    }

    void applyMorphology(cv::Mat &image, int kernel_size, int operation)
    {
      const int normalized_size = normalizeKernelSize(kernel_size);
      if (normalized_size == 0)
      {
        return;
      }

      const auto kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE, cv::Size(normalized_size, normalized_size));
      cv::morphologyEx(image, image, operation, kernel);
    }
  } // namespace

  DartDetector::DartDetector()
      : params_{}
  {
  }

  DartDetector::DartDetector(const Params &params)
      : params_(params)
  {
  }

  cv::Mat DartDetector::preprocess(const cv::Mat &img_gray) const
  {
    CV_Assert(img_gray.type() == CV_8UC1);

    cv::Mat filtered = img_gray;
    const int blur_size = normalizeKernelSize(params_.blur_kernel_size);
    if (blur_size > 1)
    {
      cv::GaussianBlur(
          img_gray, filtered, cv::Size(blur_size, blur_size), 0.0, 0.0, cv::BORDER_REPLICATE);
    }

    cv::Mat binary;
    cv::threshold(filtered, binary, params_.threshold, 255, cv::THRESH_BINARY);
    applyMorphology(binary, params_.open_kernel_size, cv::MORPH_OPEN);
    return binary;
  }

  std::vector<auto_aim_interfaces::msg::DartDetection> DartDetector::detect(
      const cv::Mat &img_color, const cv::Mat &img_gray,
      std::vector<cv::RotatedRect> * boxes) const
  {
    std::vector<std::pair<auto_aim_interfaces::msg::DartDetection, cv::RotatedRect>> candidates;

    const cv::Mat binary = preprocess(img_gray);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    candidates.reserve(contours.size());
    for (const auto &contour : contours)
    {
      const double area = cv::contourArea(contour);
      if (area < params_.min_area || area > params_.max_area)
      {
        continue;
      }

      const cv::RotatedRect box = cv::minAreaRect(contour);

      const double short_side = std::min(box.size.width, box.size.height);
      const double long_side = std::max(box.size.width, box.size.height);
      if (short_side < params_.min_short_side_px || short_side > params_.max_short_side_px)
      {
        continue;
      }

      const double aspect_ratio = long_side / std::max(1e-6, short_side);
      if (aspect_ratio < params_.min_aspect_ratio || aspect_ratio > params_.max_aspect_ratio)
      {
        continue;
      }

      const double fill_ratio = area / std::max(1e-6, static_cast<double>(box.size.width) * box.size.height);
      if (fill_ratio < params_.min_fill_ratio)
      {
        continue;
      }

      if (!inRoi(box.center, img_gray.size()))
      {
        continue;
      }

      auto type = detectDartType(img_color,box);
      if ( type != DartType::RED && type != DartType::BLUE)
      {
        continue;
      }
      
      auto_aim_interfaces::msg::DartDetection detection;
      detection.pixel_x = box.center.x;
      detection.pixel_y = box.center.y;
      detection.pixel_width = static_cast<float>(box.size.width);
      detection.pixel_height = static_cast<float>(box.size.height);
      detection.confidence = static_cast<float>(fill_ratio);
      candidates.emplace_back(detection, box);
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [](const auto &lhs, const auto &rhs)
        {
          return lhs.first.confidence > rhs.first.confidence;
        });
    std::vector<auto_aim_interfaces::msg::DartDetection> detections;
    detections.reserve(candidates.size());
    if (boxes) {
      boxes->clear();
      boxes->reserve(candidates.size());
    }
    for (const auto & candidate : candidates) {
      detections.push_back(candidate.first);
      if (boxes) {
        boxes->push_back(candidate.second);
      }
    }
    return detections;
  }

  DartType DartDetector::detectDartType(const cv::Mat &color_frame, const cv::RotatedRect box) const
  {
    if (color_frame.empty() || color_frame.type() != CV_8UC3)
    {
      return DartType::OTHER;
    }

    const cv::Rect image_rect(0, 0, color_frame.cols, color_frame.rows);
    const cv::Rect roi = box.boundingRect() & image_rect;
    if (roi.width <= 0 || roi.height <= 0)
    {
      return DartType::OTHER;
    }

    // 把轮廓的旋转矩形按 ROI 原点平移后填成掩膜，只在掩膜内取色
    cv::Point2f corners[4];
    box.points(corners);
    std::vector<cv::Point> polygon;
    polygon.reserve(4);
    for (const auto &corner : corners)
    {
      polygon.emplace_back(cvRound(corner.x) - roi.x, cvRound(corner.y) - roi.y);
    }

    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::fillConvexPoly(mask, polygon, cv::Scalar(255));

    cv::Mat roi_hsv;
    cv::cvtColor(color_frame(roi), roi_hsv, cv::COLOR_BGR2HSV);

    const int s_color_thresh = 60;
    const int v_dart_min = 120;
    const int v_green_min = 80;

    auto countColorPixels = [&](int h_low, int h_high, int v_low)
    {
      cv::Mat hits;
      cv::inRange(
          roi_hsv, cv::Scalar(h_low, s_color_thresh, v_low), cv::Scalar(h_high, 255, 255), hits);
      cv::bitwise_and(hits, mask, hits);
      return cv::countNonZero(hits);
    };

    const int red_pixels = countColorPixels(0, 10, v_dart_min) + countColorPixels(160, 180, v_dart_min);
    const int blue_pixels = countColorPixels(100, 130, v_dart_min);
    const int green_pixels = countColorPixels(35, 85, v_green_min);

    if (red_pixels == 0 && blue_pixels == 0 && green_pixels == 0)
    {
      return DartType::OTHER;
    }
    if (red_pixels >= blue_pixels && red_pixels >= green_pixels)
    {
      return DartType::RED;
    }
    if (blue_pixels >= green_pixels)
    {
      return DartType::BLUE;
    }
    return DartType::GREEN;
  }

  bool DartDetector::inRoi(const cv::Point2f &point, const cv::Size &image_size) const
  {
    const double x_min = params_.roi_x_min * image_size.width;
    const double x_max = params_.roi_x_max * image_size.width;
    const double y_min = params_.roi_y_min * image_size.height;
    const double y_max = params_.roi_y_max * image_size.height;
    return point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max;
  }

} // namespace rm_auto_aim
