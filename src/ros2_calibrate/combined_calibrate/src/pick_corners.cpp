/// Re-pick the corners of a single shot.
///
///   ros2 run combined_calibrate pick_corners <shot_dir>            # image clicks
///   ros2 run combined_calibrate pick_corners <shot_dir> --lidar    # rviz Publish Point
///
/// The image tool opens the stored frame; the LiDAR tool listens to
/// /clicked_point, so run rviz with the shot_*/cloud.pcd loaded and use the
/// Publish Point tool in the same order as the image corners.
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "combined_calibrate/calib_io.hpp"
#include "combined_calibrate/gui_pick.hpp"

namespace cc = combined_calibrate;

namespace {

void usage() {
  std::cout << "usage: pick_corners <shot_dir> [--lidar] [--count N] [--labels a,b,c,d]\n"
               "  --lidar          record corners from rviz /clicked_point instead of the image\n"
               "  --count N        how many corners to pick (default 4)\n"
               "  --labels a,b,c   names shown while picking, in the picking order\n"
            << std::endl;
}

std::vector<std::string> splitLabels(const std::string &text) {
  std::vector<std::string> labels;
  std::string current;
  for (char c : text) {
    if (c == ',') {
      labels.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  labels.push_back(current);
  return labels;
}

int runImagePicking(const std::string &shot_dir, int count, const std::vector<std::string> &labels) {
  cv::Mat image;
  if (!cc::readImage(cc::joinPath(shot_dir, "image.png"), image)) {
    std::cerr << "cannot read " << cc::joinPath(shot_dir, "image.png") << std::endl;
    return 1;
  }
  std::cout << "点击顺序: ";
  for (int i = 0; i < count; ++i) {
    if (i != 0) std::cout << " -> ";
    std::cout << (i < int(labels.size()) && !labels[i].empty() ? labels[i] : "P" + std::to_string(i + 1));
  }
  std::cout << "\n左键添加，右键/u 撤销，r 重置，ENTER 确认，ESC 取消" << std::endl;

  const cc::PickOutcome outcome =
      cc::pickCornersOnImage(image, count, labels, "combined_calibrate | pick corners");
  if (!outcome.accepted || outcome.aborted) {
    std::cerr << "nothing written" << std::endl;
    return 1;
  }
  if (!cc::writePoints2(cc::joinPath(shot_dir, "corners_img.txt"), outcome.points)) {
    std::cerr << "cannot write corners_img.txt" << std::endl;
    return 1;
  }
  cc::writeImage(cc::joinPath(shot_dir, "corners_preview.png"),
                 cc::drawCornerAnnotations(image, outcome.points, labels));
  std::cout << "wrote " << cc::joinPath(shot_dir, "corners_img.txt") << " (" << outcome.points.size()
            << " corners)" << std::endl;
  return 0;
}

int runLidarPicking(const std::string &shot_dir, int count, const std::vector<std::string> &labels) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("pick_corners_lidar");
  const std::string topic = node->declare_parameter("clicked_point_topic", std::string("/clicked_point"));
  std::vector<cv::Point3d> points;

  auto subscription = node->create_subscription<geometry_msgs::msg::PointStamped>(
      topic, 10, [&](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        if (int(points.size()) >= count) return;
        points.emplace_back(msg->point.x, msg->point.y, msg->point.z);
        const std::string label =
            (int(points.size()) <= int(labels.size()) && !labels[points.size() - 1].empty())
                ? labels[points.size() - 1]
                : "P" + std::to_string(points.size());
        std::cout << label << " = (" << std::fixed << std::setprecision(4) << msg->point.x << ", "
                  << msg->point.y << ", " << msg->point.z << ")  [" << points.size() << "/" << count
                  << "]" << std::endl;
      });

  std::cout << "在 rviz 里用 Publish Point 依次点 " << count << " 个角点（订阅 " << topic << "）"
            << std::endl;
  while (rclcpp::ok() && int(points.size()) < count) {
    rclcpp::spin_some(node);
  }

  if (int(points.size()) < count) {
    std::cerr << "interrupted after " << points.size() << " points" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  if (!cc::writePoints3(cc::joinPath(shot_dir, "corners_lidar.txt"), points)) {
    std::cerr << "cannot write corners_lidar.txt" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "wrote " << cc::joinPath(shot_dir, "corners_lidar.txt") << std::endl;
  std::cout << "点间距(核对靶标尺寸):";
  for (size_t i = 1; i < points.size(); ++i) {
    const cv::Point3d delta = points[i] - points[0];
    std::cout << " P1-P" << (i + 1) << "=" << std::sqrt(delta.dot(delta)) << "m";
  }
  std::cout << std::endl;
  rclcpp::shutdown();
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  std::string shot_dir;
  std::string label_text;
  int count = 4;
  bool lidar = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--lidar") {
      lidar = true;
    } else if (arg == "--count" && i + 1 < argc) {
      count = std::stoi(argv[++i]);
    } else if (arg == "--labels" && i + 1 < argc) {
      label_text = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      usage();
      return 0;
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "unknown option " << arg << std::endl;
      usage();
      return 1;
    } else {
      shot_dir = arg;
    }
  }
  if (shot_dir.empty()) {
    usage();
    return 1;
  }
  const std::vector<std::string> labels = label_text.empty() ? std::vector<std::string>()
                                                             : splitLabels(label_text);
  return lidar ? runLidarPicking(shot_dir, count, labels) : runImagePicking(shot_dir, count, labels);
}
