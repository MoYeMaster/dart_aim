// File helpers shared by the calibration tools: corner lists, PCD clouds,
// images and the intrinsics/extrinsics YAML files.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "combined_calibrate/calib_types.hpp"

namespace combined_calibrate {

std::string joinPath(const std::string &dir, const std::string &name);
bool ensureDirectory(const std::string &path);

/// Sorted list of "shot_*" directories below `root`.
std::vector<std::string> shotDirectories(const std::string &root);

bool readPoints2(const std::string &path, std::vector<cv::Point2d> &points);
bool readPoints3(const std::string &path, std::vector<cv::Point3d> &points);
bool writePoints2(const std::string &path, const std::vector<cv::Point2d> &points);
bool writePoints3(const std::string &path, const std::vector<cv::Point3d> &points);

/// PCD input/output through PCL, so every PCD flavour (ascii, binary,
/// compressed) that CloudCompare or PCL can write is readable.
bool readCloud(const std::string &path, std::vector<cv::Point3f> &points);
bool writeCloud(const std::string &path, const std::vector<cv::Point3f> &points, bool binary);

bool readImage(const std::string &path, cv::Mat &bgr);
bool writeImage(const std::string &path, const cv::Mat &bgr);

bool readIntrinsics(const std::string &path, Intrinsics &info, std::string *error = nullptr);
bool writeIntrinsics(const std::string &path, const Intrinsics &info);
bool readExtrinsics(const std::string &path, cv::Matx44d &T, std::string *error = nullptr);

}  // namespace combined_calibrate
