#include "combined_calibrate/overlay.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace combined_calibrate {
namespace {

void drawDot(cv::Mat &canvas, int x, int y, const cv::Vec3b &color, int radius) {
  if (x < 0 || y < 0 || x >= canvas.cols || y >= canvas.rows) return;
  if (radius <= 0) {
    canvas.at<cv::Vec3b>(y, x) = color;
    return;
  }
  const int x0 = std::max(0, x - radius);
  const int x1 = std::min(canvas.cols - 1, x + radius);
  const int y0 = std::max(0, y - radius);
  const int y1 = std::min(canvas.rows - 1, y + radius);
  for (int row = y0; row <= y1; ++row) {
    cv::Vec3b *pixels = canvas.ptr<cv::Vec3b>(row);
    for (int col = x0; col <= x1; ++col) pixels[col] = color;
  }
}

const cv::Mat &depthLut() {
  static const cv::Mat lut = []() {
    cv::Mat ramp(1, 256, CV_8UC1);
    for (int i = 0; i < 256; ++i) ramp.at<uchar>(0, i) = static_cast<uchar>(i);
    cv::Mat colored;
    cv::applyColorMap(ramp, colored, cv::COLORMAP_JET);
    return colored;
  }();
  return lut;
}

void drawDepthLegend(cv::Mat &canvas, double near_m, double far_m) {
  const int bar_height = std::min(180, std::max(60, canvas.rows / 4));
  const int bar_width = 16;
  const int x = 16;
  const int y = canvas.rows - bar_height - 40;
  if (y < 0 || x + bar_width >= canvas.cols) return;

  for (int i = 0; i < bar_height; ++i) {
    const double ratio = 1.0 - double(i) / double(bar_height - 1);
    const char depth = static_cast<char>(std::lround(ratio * 255.0));
    const cv::Vec3b color = depthLut().at<cv::Vec3b>(0, static_cast<int>(static_cast<uchar>(depth)));
    cv::rectangle(canvas, cv::Rect(x, y + i, bar_width, 1), cv::Scalar(color[0], color[1], color[2]),
                  cv::FILLED);
  }
  cv::rectangle(canvas, cv::Rect(x, y, bar_width, bar_height), cv::Scalar(255, 255, 255), 1);

  std::ostringstream near_text;
  near_text << std::fixed << std::setprecision(1) << near_m << " m";
  std::ostringstream far_text;
  far_text << std::fixed << std::setprecision(1) << far_m << " m";
  cv::putText(canvas, near_text.str(), cv::Point(x + bar_width + 6, y + bar_height - 4),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3, cv::LINE_AA, false);
  cv::putText(canvas, near_text.str(), cv::Point(x + bar_width + 6, y + bar_height - 4),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, false);
  cv::putText(canvas, far_text.str(), cv::Point(x + bar_width + 6, y + 14),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3, cv::LINE_AA, false);
  cv::putText(canvas, far_text.str(), cv::Point(x + bar_width + 6, y + 14),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, false);
}

}  // namespace

std::vector<cv::Point3f> transformCloud(const std::vector<cv::Point3f> &cloud,
                                        const cv::Matx44d &T_lidar_to_camera) {
  std::vector<cv::Point3f> transformed;
  transformed.reserve(cloud.size());
  for (const auto &point : cloud) {
    const double x = T_lidar_to_camera(0, 0) * point.x + T_lidar_to_camera(0, 1) * point.y +
                     T_lidar_to_camera(0, 2) * point.z + T_lidar_to_camera(0, 3);
    const double y = T_lidar_to_camera(1, 0) * point.x + T_lidar_to_camera(1, 1) * point.y +
                     T_lidar_to_camera(1, 2) * point.z + T_lidar_to_camera(1, 3);
    const double z = T_lidar_to_camera(2, 0) * point.x + T_lidar_to_camera(2, 1) * point.y +
                     T_lidar_to_camera(2, 2) * point.z + T_lidar_to_camera(2, 3);
    transformed.emplace_back(float(x), float(y), float(z));
  }
  return transformed;
}

cv::Mat renderOverlay(const cv::Mat &bgr, const std::vector<cv::Point3f> &cloud,
                      const Intrinsics &info, const cv::Matx44d &T_lidar_to_camera,
                      const OverlayOptions &options) {
  if (bgr.empty()) return cv::Mat();
  cv::Mat canvas = bgr.clone();
  if (cloud.empty() || !info.valid()) return canvas;

  cv::Mat dist;
  if (!info.dist.empty()) {
    dist = cv::Mat(1, static_cast<int>(info.dist.size()), CV_64F);
    for (size_t i = 0; i < info.dist.size(); ++i) {
      dist.at<double>(0, static_cast<int>(i)) = info.dist[i];
    }
  }

  const double near_m = std::max(0.05, options.min_depth_m);
  const double far_m = std::max(near_m + 1e-3, options.max_depth_m);
  const double range = far_m - near_m;

  std::vector<cv::Point3f> visible;
  visible.reserve(cloud.size());
  for (const auto &point : cloud) {
    const double x = T_lidar_to_camera(0, 0) * point.x + T_lidar_to_camera(0, 1) * point.y +
                     T_lidar_to_camera(0, 2) * point.z + T_lidar_to_camera(0, 3);
    const double y = T_lidar_to_camera(1, 0) * point.x + T_lidar_to_camera(1, 1) * point.y +
                     T_lidar_to_camera(1, 2) * point.z + T_lidar_to_camera(1, 3);
    const double z = T_lidar_to_camera(2, 0) * point.x + T_lidar_to_camera(2, 1) * point.y +
                     T_lidar_to_camera(2, 2) * point.z + T_lidar_to_camera(2, 3);
    if (z < near_m || z > far_m) continue;
    visible.emplace_back(float(x), float(y), float(z));
  }
  if (visible.empty()) {
    drawDepthLegend(canvas, near_m, far_m);
    return canvas;
  }

  std::vector<cv::Point2f> projected;
  cv::projectPoints(visible, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Mat(info.K), dist,
                    projected);
  if (projected.size() != visible.size()) return canvas;

  for (size_t i = 0; i < projected.size(); ++i) {
    if (!std::isfinite(projected[i].x) || !std::isfinite(projected[i].y)) continue;
    const int x = cvRound(projected[i].x);
    const int y = cvRound(projected[i].y);
    cv::Vec3b color(0, 255, 0);
    if (options.color_by_depth) {
      double ratio = (double(visible[i].z) - near_m) / range;
      ratio = std::min(1.0, std::max(0.0, ratio));
      color = depthLut().at<cv::Vec3b>(0, static_cast<int>(std::lround(ratio * 255.0)));
    }
    drawDot(canvas, x, y, color, options.point_size);
  }
  drawDepthLegend(canvas, near_m, far_m);
  return canvas;
}

}  // namespace combined_calibrate
