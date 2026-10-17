/// Guided camera + LiDAR joint calibration in one node.
///
///   capture shots -> pick the target corners in the image -> pick the same
///   corners on the cloud in rviz -> solve -> project to verify
///
/// Everything is driven from the terminal of this node, so a single launch
/// file is enough to calibrate.
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "combined_calibrate/calib_io.hpp"
#include "combined_calibrate/calib_math.hpp"
#include "combined_calibrate/gui_pick.hpp"
#include "combined_calibrate/overlay.hpp"
#include "combined_calibrate/ros_util.hpp"

namespace fs = std::filesystem;

namespace combined_calibrate {
namespace {

std::vector<std::string> splitList(const std::string &text) {
  std::vector<std::string> items;
  std::string current;
  for (char c : text) {
    if (c == ',' || c == ';') {
      items.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  items.push_back(current);
  for (auto &item : items) {
    const auto first = item.find_first_not_of(" \t");
    const auto last = item.find_last_not_of(" \t");
    if (first == std::string::npos) {
      item.clear();
    } else {
      item = item.substr(first, last - first + 1);
    }
  }
  return items;
}

std::string formatPoint(const cv::Point3d &point) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(4);
  stream << "(" << point.x << ", " << point.y << ", " << point.z << ")";
  return stream.str();
}

/// Non blocking terminal reader: one command per line.
class TerminalReader {
 public:
  void start() {
    thread_ = std::thread([this]() {
      std::string line;
      while (std::getline(std::cin, line)) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(line);
      }
    });
    thread_.detach();
  }

  std::vector<std::string> drain() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> lines(queue_.begin(), queue_.end());
    queue_.clear();
    return lines;
  }

 private:
  std::thread thread_;
  std::mutex mutex_;
  std::vector<std::string> queue_;
};

}  // namespace

class CalibrateNode : public rclcpp::Node {
 public:
  CalibrateNode() : rclcpp::Node("calibrate") {
    shots_dir_ = declare_parameter("shots_dir", defaultShotsDir());
    mode_ = declare_parameter("mode", std::string("guided"));
    lidar_topic_ = declare_parameter("lidar_topic", std::string("/livox/lidar"));
    image_topic_ = declare_parameter("image_topic", std::string("/image_raw"));
    camera_info_topic_ = declare_parameter("camera_info_topic", std::string("/image_raw/camera_info"));
    clicked_point_topic_ = declare_parameter("clicked_point_topic", std::string("/clicked_point"));
    lidar_frame_ = declare_parameter("lidar_frame", std::string("livox_frame"));
    accumulate_frames_ = declare_parameter("accumulate_frames", 20);
    max_shots_ = declare_parameter("max_shots", 8);
    min_shots_ = declare_parameter("min_shots", 3);
    corner_count_ = declare_parameter("corner_count", 4);
    corner_labels_ = splitList(declare_parameter("corner_labels", std::string("左上,右上,右下,左下")));
    interactive_ = declare_parameter("interactive", displayAvailable());
    show_preview_ = declare_parameter("show_preview", true);
    preview_hz_ = declare_parameter("preview_hz", 2.0);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 2.0);
    max_depth_m_ = declare_parameter("max_depth_m", 30.0);
    point_size_ = declare_parameter("point_size", 2);
    fix_roi_intrinsics_ = declare_parameter("fix_roi_intrinsics", true);
    pcd_binary_ = declare_parameter("pcd_binary", true);
    extrinsics_path_ = declare_parameter("extrinsics_path", std::string(""));

    const auto sensor_qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, sensor_qos, std::bind(&CalibrateNode::onCloud, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, sensor_qos, std::bind(&CalibrateNode::onImage, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, sensor_qos,
        std::bind(&CalibrateNode::onCameraInfo, this, std::placeholders::_1));
    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic_, 10,
        std::bind(&CalibrateNode::onClickedPoint, this, std::placeholders::_1));

    preview_pub_ = create_publisher<sensor_msgs::msg::Image>("~/preview", 1);
    overlay_pub_ = create_publisher<sensor_msgs::msg::Image>("~/overlay", 1);
    camera_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points_camera", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 1);

    ensureDirectory(shots_dir_);
    if (extrinsics_path_.empty()) extrinsics_path_ = joinPath(shots_dir_, "extrinsics.yaml");

    RCLCPP_INFO(get_logger(), "shots       : %s", shots_dir_.c_str());
    RCLCPP_INFO(get_logger(), "mode        : %s", mode_.c_str());
    RCLCPP_INFO(get_logger(), "subscribing : %s | %s | %s", lidar_topic_.c_str(), image_topic_.c_str(),
                camera_info_topic_.c_str());
    if (!interactive_) {
      std::cout << "[info] interactive mode is off (no display or interactive:=false): the nodes\n"
                   "       record shots only, pick corners later with\n"
                   "       ros2 run combined_calibrate pick_corners <shot_dir>\n"
                << std::endl;
    }
    terminal_.start();
    switchMode();
  }

  ~CalibrateNode() override {
    try {
      cv::destroyAllWindows();
    } catch (const cv::Exception &exception) {
      // nothing to do
    }
  }

  void run() {
    while (rclcpp::ok() && !finished_) {
      rclcpp::spin_some(this->get_node_base_interface());
      tick();
      handleTerminal();
      publishStatus();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

 private:
  enum class State {
    kWaitReady,
    kWaitStart,
    kAccumulate,
    kPickImage,
    kPickLidar,
    kAskNext,
    kSolve,
    kVerify,
    kFinished,
  };

  static bool displayAvailable() {
    return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
  }

  static std::string defaultShotsDir() {
    const char *home = std::getenv("HOME");
    return std::string(home != nullptr ? home : ".") + "/dart_calibration/shots";
  }

  // ---------------------------------------------------------------- callbacks

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    latest_image_ = msg;
    have_image_ = true;
    const auto now = this->now();
    const double period = preview_hz_ > 0.0 ? 1.0 / preview_hz_ : 0.0;
    if (period <= 0.0) return;
    if (last_preview_.nanoseconds() != 0 &&
        (now - last_preview_).seconds() < period) {
      return;
    }
    last_preview_ = now;
    cv::Mat preview;
    if (!toBgr(msg, preview)) return;
    cv::Mat small;
    const int max_width = 960;
    if (preview.cols > max_width) {
      const double scale = double(max_width) / double(preview.cols);
      cv::resize(preview, small, cv::Size(), scale, scale, cv::INTER_AREA);
    } else {
      small = preview;
    }
    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = "bgr8";
    out.image = small;
    preview_pub_->publish(*out.toImageMsg());
    if (show_preview_ && interactive_) safeImshow(kPreviewWindow, small);
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    Intrinsics info;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) info.K(row, col) = msg->k[row * 3 + col];
    }
    info.dist.assign(msg->d.begin(), msg->d.end());
    info.width = int(msg->width);
    info.height = int(msg->height);
    if (info.valid()) {
      raw_intrinsics_ = info;
      have_intrinsics_ = true;
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    have_cloud_ = true;
    if (state_ != State::kAccumulate && state_ != State::kVerify) return;

    std::vector<cv::Point3f> points;
    if (!parseCloud(*msg, points) || points.empty()) return;

    if (state_ == State::kAccumulate) {
      cloud_buffer_.insert(cloud_buffer_.end(), points.begin(), points.end());
      if (++cloud_frames_ >= accumulate_frames_) finishShot();
      return;
    }

    verify_buffer_.insert(verify_buffer_.end(), points.begin(), points.end());
    if (++verify_frames_ >= accumulate_frames_) {
      verify_cloud_ = verify_buffer_;
      verify_buffer_.clear();
      verify_frames_ = 0;
    }
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg) {
    if (state_ != State::kPickLidar) {
      if (!warned_click_) {
        warned_click_ = true;
        std::cout << "[hint] a point arrived but the node is not waiting for LiDAR corners yet; "
                     "ignore it"
                  << std::endl;
      }
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != lidar_frame_) {
      RCLCPP_WARN_ONCE(get_logger(), "clicked point is in frame '%s' but the cloud is '%s'",
                       msg->header.frame_id.c_str(), lidar_frame_.c_str());
    }
    const cv::Point3d point(msg->point.x, msg->point.y, msg->point.z);
    lidar_points_.push_back(point);
    std::cout << "  " << labelFor(lidar_points_.size() - 1) << " = " << formatPoint(point) << "  ["
              << lidar_points_.size() << "/" << expectedLidarCorners() << "]" << std::endl;
    if (int(lidar_points_.size()) >= expectedLidarCorners()) storeLidarCorners();
  }

  // ------------------------------------------------------------ state machine

  void switchMode() {
    if (mode_ == "verify") {
      if (!loadExtrinsics()) {
        RCLCPP_ERROR(get_logger(), "verify mode needs %s", extrinsics_path_.c_str());
        finished_ = true;
        return;
      }
      setState(State::kVerify);
    } else if (mode_ == "solve") {
      setState(State::kSolve);
    } else if (mode_ == "capture") {
      setState(State::kWaitReady);
    } else {
      setState(State::kWaitReady);
    }
  }

  void setState(State state) {
    state_ = state;
    switch (state) {
      case State::kWaitReady:
        waiting_reported_ = false;
        break;
      case State::kWaitStart:
        std::cout << "\n=== ready: target in view of both sensors ===\n"
                  << "ENTER  开始第 " << (shot_index_ + 1) << " 个位姿的采集\n"
                  << "f      结束采集并求解\n"
                  << "q      退出" << std::endl;
        break;
      case State::kAccumulate:
        std::cout << "采集中 (" << accumulate_frames_ << " 帧点云，约 "
                  << int(std::ceil(accumulate_frames_ / 10.0)) << " 秒)，保持靶标静止..." << std::endl;
        break;
      case State::kPickLidar:
        std::cout << "\n在 rviz 里用 Publish Point 依次点 " << expectedLidarCorners()
                  << " 个角点（顺序和图像一致：" << orderText() << "）\n"
                  << "u 撤销上一个点   s 跳过这个位姿的雷达角点   f 结束并求解" << std::endl;
        break;
      case State::kAskNext:
        if (shot_index_ >= max_shots_) {
          std::cout << "\n已采集 " << shot_index_ << " 个位姿（max_shots），开始求解" << std::endl;
          setState(State::kSolve);
          break;
        }
        std::cout << "\nENTER  把靶标挪到下一个位姿后开始采集\n"
                  << "f      结束采集并求解\n"
                  << "q      退出" << std::endl;
        break;
      case State::kSolve:
        break;
      case State::kVerify:
        break;
      case State::kFinished:
        break;
      case State::kPickImage:
        break;
    }
  }

  void tick() {
    switch (state_) {
      case State::kWaitReady:
        if (!have_image_ || !have_cloud_) {
          if (!waiting_reported_) {
            waiting_reported_ = true;
            std::cout << "等待传感器数据: 图像 " << (have_image_ ? "ok" : "...") << "  点云 "
                      << (have_cloud_ ? "ok" : "...") << std::endl;
          }
          break;
        }
        std::cout << "图像和点云都有数据了" << std::endl;
        setState(State::kWaitStart);
        break;
      case State::kAccumulate:
        break;
      case State::kPickImage:
        runImagePicking();
        break;
      case State::kPickLidar:
        break;
      case State::kWaitStart:
      case State::kAskNext:
        break;
      case State::kSolve:
        runSolve();
        break;
      case State::kVerify:
        publishVerify();
        break;
      case State::kFinished:
        break;
    }
  }

  void handleTerminal() {
    for (const std::string &raw : terminal_.drain()) {
      std::string command = raw;
      const auto first = command.find_first_not_of(" \t");
      command = first == std::string::npos ? std::string() : command.substr(first);
      const auto last = command.find_last_not_of(" \t");
      if (last != std::string::npos) command = command.substr(0, last + 1);

      if (command == "q" || command == "Q" || command == "quit") {
        finished_ = true;
        continue;
      }
      if (command == "f" || command == "F" || command == "finish") {
        if (countCompleteShots() == 0) {
          std::cout << "还没有任何一个 shot 同时有图像和雷达角点，先采集并点角点" << std::endl;
          continue;
        }
        setState(State::kSolve);
        continue;
      }
      if (state_ == State::kPickLidar) {
        if (command == "u" || command == "U") {
          if (!lidar_points_.empty()) {
            lidar_points_.pop_back();
            std::cout << "撤销，剩 " << lidar_points_.size() << " 个点" << std::endl;
          }
          continue;
        }
        if (command == "s" || command == "S") {
          std::cout << "跳过这个位姿的雷达角点（该 shot 不会参与求解）" << std::endl;
          lidar_points_.clear();
          setState(State::kAskNext);
          continue;
        }
        if (command.empty()) {
          if (lidar_points_.empty()) {
            std::cout << "还没有收到雷达角点" << std::endl;
            continue;
          }
          storeLidarCorners();
          continue;
        }
        continue;
      }
      if ((state_ == State::kWaitStart || state_ == State::kAskNext) && command.empty()) {
        startShot();
        continue;
      }
      if (!command.empty()) {
        std::cout << "未知命令: " << command << "（可用: ENTER / f / q）" << std::endl;
      }
    }
  }

  // ---------------------------------------------------------------- workflow

  void startShot() {
    cloud_buffer_.clear();
    cloud_frames_ = 0;
    shot_image_.release();
    setState(State::kAccumulate);
  }

  void finishShot() {
    if (!latest_image_) {
      RCLCPP_WARN(get_logger(), "no camera frame yet, restarting the shot");
      setState(State::kAccumulate);
      cloud_buffer_.clear();
      cloud_frames_ = 0;
      return;
    }

    cv::Mat image;
    if (!toBgr(latest_image_, image)) {
      RCLCPP_ERROR(get_logger(), "cannot convert the camera frame to bgr8");
      setState(State::kWaitStart);
      return;
    }
    shot_image_ = image;

    Intrinsics info = raw_intrinsics_;
    if (fix_roi_intrinsics_) {
      std::string message;
      if (adjustIntrinsicsForImage(info, image.cols, image.rows, &message) && !message.empty()) {
        RCLCPP_WARN_STREAM(get_logger(), message);
      }
    }

    std::ostringstream name;
    name << "shot_" << std::setw(3) << std::setfill('0') << shot_index_;
    shot_name_ = name.str();
    shot_dir_ = joinPath(shots_dir_, shot_name_);

    const bool cloud_ok = writeCloud(joinPath(shot_dir_, "cloud.pcd"), cloud_buffer_, pcd_binary_);
    const bool image_ok = writeImage(joinPath(shot_dir_, "image.png"), image);
    if (have_intrinsics_) writeIntrinsics(joinPath(shot_dir_, "intrinsics.yaml"), info);

    if (!cloud_ok || !image_ok) {
      RCLCPP_ERROR(get_logger(), "failed to write %s", shot_dir_.c_str());
      setState(State::kWaitStart);
      return;
    }
    RCLCPP_INFO(get_logger(), "saved %s with %zu points", shot_name_.c_str(), cloud_buffer_.size());
    std::cout << "saved " << shot_name_ << " (" << cloud_buffer_.size() << " 点)" << std::endl;
    ++shot_index_;

    if (interactive_ && mode_ != "capture") {
      setState(State::kPickImage);
    } else {
      setState(State::kAskNext);
    }
  }

  void runImagePicking() {
    if (shot_image_.empty()) {
      setState(State::kAskNext);
      return;
    }
    std::cout << "\n在弹出的窗口里按顺序点击 " << corner_count_ << " 个靶标角点: " << orderText()
              << "\n左键添加，右键/u 撤销，r 重置，ENTER 确认，ESC 放弃该位姿" << std::endl;
    const std::string title = "combined_calibrate | pick corners | " + shot_name_;
    const PickOutcome outcome =
        pickCornersOnImage(shot_image_, corner_count_, corner_labels_, title);
    if (outcome.aborted || !outcome.accepted) {
      std::cout << "已放弃该位姿的图像角点（cloud.pcd 和 image.png 仍在磁盘上）" << std::endl;
      setState(State::kAskNext);
      return;
    }

    image_points_ = outcome.points;
    if (!writePoints2(joinPath(shot_dir_, "corners_img.txt"), image_points_)) {
      RCLCPP_ERROR(get_logger(), "cannot write corners_img.txt");
      setState(State::kAskNext);
      return;
    }
    writeImage(joinPath(shot_dir_, "corners_preview.png"),
               drawCornerAnnotations(shot_image_, image_points_, corner_labels_));
    std::cout << "corner 已保存 (" << image_points_.size() << " 个)" << std::endl;

    lidar_points_.clear();
    warned_click_ = false;
    setState(State::kPickLidar);
  }

  void storeLidarCorners() {
    if (lidar_points_.empty()) return;
    if (!writePoints3(joinPath(shot_dir_, "corners_lidar.txt"), lidar_points_)) {
      RCLCPP_ERROR(get_logger(), "cannot write corners_lidar.txt");
      setState(State::kAskNext);
      return;
    }
    if (!image_points_.empty() && image_points_.size() != lidar_points_.size()) {
      RCLCPP_WARN(get_logger(), "%s: %zu image corners but %zu LiDAR corners", shot_name_.c_str(),
                  image_points_.size(), lidar_points_.size());
    }
    printPairDistances();
    std::cout << "corners_lidar.txt 已保存" << std::endl;
    setState(State::kAskNext);
  }

  void runSolve() {
    std::vector<Shot> shots;
    std::vector<std::string> skipped;
    std::string error;
    if (!loadShots(shots_dir_, shots, skipped, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      setState(State::kAskNext);
      return;
    }
    for (const std::string &entry : skipped) {
      RCLCPP_WARN(get_logger(), "skipped %s", entry.c_str());
    }
    if (int(shots.size()) < min_shots_) {
      RCLCPP_WARN(get_logger(), "only %zu usable shots (recommended: at least %d)", shots.size(),
                  min_shots_);
    }

    Intrinsics info = raw_intrinsics_;
    if (!info.valid()) {
      if (!loadShotIntrinsics(shots_dir_, info)) {
        RCLCPP_ERROR(get_logger(), "no camera intrinsics found in the shots or on the topic");
        setState(State::kAskNext);
        return;
      }
    }
    if (fix_roi_intrinsics_ && !shot_image_.empty()) {
      adjustIntrinsicsForImage(info, shot_image_.cols, shot_image_.rows, nullptr);
    }

    SolveResult result;
    if (!solveExtrinsic(shots, info, result, &error)) {
      RCLCPP_ERROR(get_logger(), "solve failed: %s", error.c_str());
      setState(State::kAskNext);
      return;
    }

    int points_per_shot = 0;
    for (const auto &shot : shots) points_per_shot = std::max(points_per_shot, int(shot.lidar.size()));
    for (size_t i = 0; i < result.used_shots.size(); ++i) {
      std::cout << "  " << result.used_shots[i] << ": " << std::fixed << std::setprecision(3)
                << result.per_shot_error_px[i] << " px" << std::endl;
    }
    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(3);
    summary << "重投影误差: 平均 " << result.mean_error_px << " px, 最差 " << result.worst_error_px
            << " px, 使用 " << result.used_shots.size() << " 个 shot";
    std::cout << summary.str() << std::endl;
    if (!result.rejected_shots.empty()) {
      std::cout << "被当作外点剔除: ";
      for (const auto &name : result.rejected_shots) std::cout << name << " ";
      std::cout << std::endl;
    }
    if (result.mean_error_px > 2.0) {
      RCLCPP_WARN(get_logger(),
                  "reprojection error is large (%.2f px): check the corner picks and that the "
                  "target stayed still",
                  result.mean_error_px);
    }

    const cv::Matx44d T = composePose(result.R, result.t);
    if (!writeExtrinsicsYaml(extrinsics_path_, T, result, points_per_shot)) {
      RCLCPP_ERROR(get_logger(), "cannot write %s", extrinsics_path_.c_str());
      setState(State::kAskNext);
      return;
    }
    const cv::Vec3d rpy = rotationToRpyDegrees(result.R);
    std::cout << "T_lidar_to_camera 已写入 " << extrinsics_path_ << "\n"
              << "  平移 " << cv::norm(result.t) << " m, RPY " << rpy(0) << " " << rpy(1) << " "
              << rpy(2) << " deg" << std::endl;
    extrinsics_ = T;
    have_extrinsics_ = true;
    verify_buffer_.clear();
    verify_frames_ = 0;
    setState(State::kVerify);
  }

  void publishVerify() {
    if (!have_extrinsics_) return;
    const auto now = this->now();
    const double period = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.5;
    if (last_overlay_.nanoseconds() != 0 && (now - last_overlay_).seconds() < period) return;
    last_overlay_ = now;
    if (!latest_image_) return;

    cv::Mat image;
    if (!toBgr(latest_image_, image)) return;

    if (verify_cloud_.empty()) return;

    Intrinsics info = raw_intrinsics_;
    adjustIntrinsicsForImage(info, image.cols, image.rows, nullptr);

    OverlayOptions options;
    options.max_depth_m = max_depth_m_;
    options.point_size = point_size_;
    const cv::Mat overlay = renderOverlay(image, verify_cloud_, info, extrinsics_, options);
    publishImage(overlay_pub_, latest_image_->header, overlay);

    std_msgs::msg::Header header = latest_image_->header;
    header.frame_id = "camera";
    const std::vector<cv::Point3f> camera_cloud = transformCloud(verify_cloud_, extrinsics_);
    publishPointCloud(camera_cloud, header);
    if (interactive_ && show_preview_) safeImshow(kOverlayWindow, overlay);
  }

  // ------------------------------------------------------------------ helpers

  bool loadExtrinsics() {
    std::string error;
    if (!readExtrinsics(extrinsics_path_, extrinsics_, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    have_extrinsics_ = true;
    RCLCPP_INFO(get_logger(), "loaded %s", extrinsics_path_.c_str());
    return true;
  }

  bool toBgr(const sensor_msgs::msg::Image::ConstSharedPtr &msg, cv::Mat &bgr) const {
    std::string error;
    if (!imageToBgrMat(*msg, bgr, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    return true;
  }

  bool parseCloud(const sensor_msgs::msg::PointCloud2 &msg,
                  std::vector<cv::Point3f> &points) const {
    std::string error;
    if (!pointsFromCloudMsg(msg, points, &error)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }
    return true;
  }

  void publishImage(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher,
                    const std_msgs::msg::Header &header, const cv::Mat &image) {
    publishBgrImage(publisher, header, image);
  }

  void publishPointCloud(const std::vector<cv::Point3f> &camera_cloud,
                         const std_msgs::msg::Header &header) {
    publishXyzRgbCloud(camera_cloud_pub_, header, camera_cloud, 0.3, max_depth_m_);
  }

  void publishStatus() {
    const auto now = this->now();
    if (status_pub_->get_subscription_count() == 0) return;
    if (last_status_.nanoseconds() != 0 && (now - last_status_).seconds() < 1.0) return;
    last_status_ = now;
    std_msgs::msg::String msg;
    msg.data = statusText();
    status_pub_->publish(msg);
  }

  std::string statusText() const {
    switch (state_) {
      case State::kWaitReady:
        return "waiting for the camera image and the LiDAR cloud";
      case State::kWaitStart:
        return "ready for " + std::to_string(shot_index_ + 1) + ". pose (press ENTER in the terminal)";
      case State::kAccumulate:
        return "capturing pose " + std::to_string(shot_index_ + 1) + ": " +
               std::to_string(cloud_frames_) + "/" + std::to_string(accumulate_frames_) + " scans";
      case State::kPickImage:
        return "click the target corners in the image window";
      case State::kPickLidar:
        return "publish points on the cloud in rviz (" + std::to_string(lidar_points_.size()) + "/" +
               std::to_string(expectedLidarCorners()) + ")";
      case State::kAskNext:
        return "shot " + shot_name_ + " stored - move the target and press ENTER (f = solve)";
      case State::kSolve:
        return "solving";
      case State::kVerify:
        return "publishing the projection overlay";
      case State::kFinished:
        return "done";
    }
    return {};
  }

  int countCompleteShots() const {
    int count = 0;
    for (const std::string &dir : shotDirectories(shots_dir_)) {
      if (fs::exists(joinPath(dir, "corners_img.txt")) &&
          fs::exists(joinPath(dir, "corners_lidar.txt"))) {
        ++count;
      }
    }
    return count;
  }

  std::string labelFor(size_t index) const {
    if (index < corner_labels_.size() && !corner_labels_[index].empty()) return corner_labels_[index];
    return "P" + std::to_string(index + 1);
  }

  std::string orderText() const {
    std::string text;
    const int count = std::max(corner_count_, int(image_points_.size()));
    for (int i = 0; i < count; ++i) {
      if (i != 0) text += " -> ";
      text += labelFor(size_t(i));
    }
    return text;
  }

  int expectedLidarCorners() const {
    if (!image_points_.empty()) return int(image_points_.size());
    return corner_count_;
  }

  void printPairDistances() const {
    if (lidar_points_.size() < 2) return;
    std::cout << "  点间距(用于核对靶标尺寸):";
    for (size_t i = 1; i < lidar_points_.size(); ++i) {
      const cv::Point3d delta = lidar_points_[i] - lidar_points_[0];
      std::cout << " P1-P" << (i + 1) << "=" << std::fixed << std::setprecision(3)
                << std::sqrt(delta.dot(delta)) << "m";
    }
    std::cout << std::endl;
  }

  bool safeImshow(const std::string &window, const cv::Mat &image) {
    if (image.empty()) return false;
    try {
      cv::imshow(window, image);
      cv::waitKey(1);
      return true;
    } catch (const cv::Exception &exception) {
      if (!preview_failed_) {
        preview_failed_ = true;
        RCLCPP_WARN(get_logger(), "cannot show '%s' (%s); continuing headless", window.c_str(),
                    exception.what());
      }
      return false;
    }
  }

  // ------------------------------------------------------------------ members

  static constexpr const char *kPreviewWindow = "combined_calibrate | live";
  static constexpr const char *kOverlayWindow = "combined_calibrate | overlay";

  std::string shots_dir_;
  std::string mode_;
  std::string lidar_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string clicked_point_topic_;
  std::string lidar_frame_;
  std::string extrinsics_path_;
  int accumulate_frames_ = 20;
  int max_shots_ = 8;
  int min_shots_ = 3;
  int corner_count_ = 4;
  std::vector<std::string> corner_labels_;
  bool interactive_ = false;
  bool show_preview_ = true;
  double preview_hz_ = 2.0;
  double publish_rate_hz_ = 2.0;
  double max_depth_m_ = 30.0;
  int point_size_ = 2;
  bool fix_roi_intrinsics_ = true;
  bool pcd_binary_ = true;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camera_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  State state_ = State::kWaitReady;
  TerminalReader terminal_;
  bool finished_ = false;
  bool waiting_reported_ = false;
  bool warned_click_ = false;
  bool preview_failed_ = false;

  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  bool have_image_ = false;
  bool have_cloud_ = false;
  bool have_intrinsics_ = false;
  Intrinsics raw_intrinsics_;

  std::vector<cv::Point3f> cloud_buffer_;
  int cloud_frames_ = 0;
  std::vector<cv::Point3f> verify_buffer_;
  std::vector<cv::Point3f> verify_cloud_;
  int verify_frames_ = 0;

  std::string shot_name_;
  std::string shot_dir_;
  cv::Mat shot_image_;
  int shot_index_ = 0;
  std::vector<cv::Point2d> image_points_;
  std::vector<cv::Point3d> lidar_points_;

  cv::Matx44d extrinsics_ = cv::Matx44d::eye();
  bool have_extrinsics_ = false;

  rclcpp::Time last_preview_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_overlay_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_{0, 0, RCL_ROS_TIME};
};

}  // namespace combined_calibrate

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<combined_calibrate::CalibrateNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
