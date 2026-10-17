#include "combined_calibrate/calib_io.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/imgcodecs.hpp>

#include <cstdlib>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "combined_calibrate/mini_yaml.hpp"

namespace fs = std::filesystem;

namespace combined_calibrate {
namespace {

std::string strip(const std::string &text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::vector<double> splitNumbers(const std::string &line, std::string *tail = nullptr) {
  std::vector<double> values;
  std::string text = line;
  std::replace(text.begin(), text.end(), ',', ' ');
  std::istringstream stream(text);
  std::string token;
  while (stream >> token) {
    char *end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
      if (tail != nullptr) *tail = token;
      break;
    }
    values.push_back(value);
  }
  return values;
}

}  // namespace

std::string joinPath(const std::string &dir, const std::string &name) {
  if (dir.empty()) return name;
  if (dir.back() == '/') return dir + name;
  return dir + "/" + name;
}

bool ensureDirectory(const std::string &path) {
  std::error_code code;
  fs::create_directories(path, code);
  return !code;
}

std::vector<std::string> shotDirectories(const std::string &root) {
  std::vector<std::string> dirs;
  std::error_code code;
  if (!fs::is_directory(root, code)) return dirs;
  for (const auto &entry : fs::directory_iterator(root, code)) {
    if (!entry.is_directory()) continue;
    const std::string name = entry.path().filename().string();
    if (name.rfind("shot_", 0) == 0) dirs.push_back(entry.path().string());
  }
  std::sort(dirs.begin(), dirs.end());
  return dirs;
}

bool readPoints2(const std::string &path, std::vector<cv::Point2d> &points) {
  points.clear();
  std::ifstream file(path);
  if (!file) return false;
  std::string line;
  while (std::getline(file, line)) {
    const std::string text = strip(line);
    if (text.empty() || text[0] == '#') continue;
    const std::vector<double> values = splitNumbers(line);
    if (values.size() < 2) return false;
    points.emplace_back(values[0], values[1]);
  }
  return !points.empty();
}

bool readPoints3(const std::string &path, std::vector<cv::Point3d> &points) {
  points.clear();
  std::ifstream file(path);
  if (!file) return false;
  std::string line;
  while (std::getline(file, line)) {
    const std::string text = strip(line);
    if (text.empty() || text[0] == '#') continue;
    const std::vector<double> values = splitNumbers(line);
    if (values.size() < 3) return false;
    points.emplace_back(values[0], values[1], values[2]);
  }
  return !points.empty();
}

bool writePoints2(const std::string &path, const std::vector<cv::Point2d> &points) {
  if (!ensureDirectory(fs::path(path).parent_path().string())) return false;
  std::ofstream file(path);
  if (!file) return false;
  file << std::fixed << std::setprecision(4);
  for (const auto &point : points) file << point.x << ' ' << point.y << '\n';
  return true;
}

bool writePoints3(const std::string &path, const std::vector<cv::Point3d> &points) {
  if (!ensureDirectory(fs::path(path).parent_path().string())) return false;
  std::ofstream file(path);
  if (!file) return false;
  file << std::fixed << std::setprecision(6);
  for (const auto &point : points) file << point.x << ' ' << point.y << ' ' << point.z << '\n';
  return true;
}

bool readCloud(const std::string &path, std::vector<cv::Point3f> &points) {
  points.clear();
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path, blob) < 0) return false;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(blob, cloud);
  points.reserve(cloud.size());
  for (const auto &point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    points.emplace_back(point.x, point.y, point.z);
  }
  return !points.empty();
}

bool writeCloud(const std::string &path, const std::vector<cv::Point3f> &points, bool binary) {
  if (!ensureDirectory(fs::path(path).parent_path().string())) return false;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto &point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    cloud.emplace_back(point.x, point.y, point.z);
  }
  if (cloud.empty()) return false;
  const int result =
      binary ? pcl::io::savePCDFileBinary(path, cloud) : pcl::io::savePCDFileASCII(path, cloud);
  return result == 0;
}

bool readImage(const std::string &path, cv::Mat &bgr) {
  bgr = cv::imread(path, cv::IMREAD_COLOR);
  return !bgr.empty();
}

bool writeImage(const std::string &path, const cv::Mat &bgr) {
  if (bgr.empty()) return false;
  if (!ensureDirectory(fs::path(path).parent_path().string())) return false;
  const std::vector<int> params{cv::IMWRITE_PNG_COMPRESSION, 3};
  return cv::imwrite(path, bgr, params);
}

bool readIntrinsics(const std::string &path, Intrinsics &info, std::string *error) {
  MiniYaml doc;
  if (!MiniYaml::loadFile(path, doc, error)) return false;

  std::vector<double> matrix = doc.getNumberList("camera_matrix");
  if (matrix.size() != 9) {
    const std::vector<double> k = doc.getNumberList("k");
    if (k.size() == 9) matrix = k;
  }
  if (matrix.size() != 9) {
    if (error) *error = path + ": camera_matrix must hold 9 numbers";
    return false;
  }
  info.K = cv::Matx33d(matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6],
                       matrix[7], matrix[8]);

  info.dist = doc.getNumberList("distortion");
  if (info.dist.empty()) info.dist = doc.getNumberList("distortion_coefficients");
  if (info.dist.empty()) info.dist = doc.getNumberList("d");

  info.width = 0;
  info.height = 0;
  doc.getInt("image_width", info.width);
  doc.getInt("image_height", info.height);
  if (info.width == 0 || info.height == 0) {
    doc.getInt("width", info.width);
    doc.getInt("height", info.height);
  }
  if (!info.valid()) {
    if (error) *error = path + ": focal length is zero";
    return false;
  }
  return true;
}

bool writeIntrinsics(const std::string &path, const Intrinsics &info) {
  if (!ensureDirectory(fs::path(path).parent_path().string())) return false;
  std::ofstream file(path);
  if (!file) return false;
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << "# written by combined_calibrate (C++)\n";
  stream << "image_width: " << info.width << "\n";
  stream << "image_height: " << info.height << "\n";
  stream << "camera_matrix: [";
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (row != 0 || col != 0) stream << ", ";
      stream << info.K(row, col);
    }
  }
  stream << "]\n";
  stream << "distortion: [";
  for (size_t i = 0; i < info.dist.size(); ++i) {
    if (i != 0) stream << ", ";
    stream << info.dist[i];
  }
  stream << "]\n";
  file << stream.str();
  return true;
}

bool readExtrinsics(const std::string &path, cv::Matx44d &T, std::string *error) {
  MiniYaml doc;
  if (!MiniYaml::loadFile(path, doc, error)) return false;
  std::vector<double> values = doc.getNumberList("T_lidar_to_camera");
  if (values.size() != 16) values = doc.getNumberList("data");
  if (values.size() != 16) {
    if (error) *error = path + ": T_lidar_to_camera must hold 16 numbers";
    return false;
  }
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) T(row, col) = values[static_cast<size_t>(row) * 4 + col];
  }
  return true;
}

}  // namespace combined_calibrate
