#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

#include "lidar_calibrate/accumulator.hpp"

namespace lidar_calibrate {

class MapBuilder : public rclcpp::Node {
 public:
  MapBuilder() : Node("map_builder_node") {
    const std::string lidar_topic =
        declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    output_pcd_ = declare_parameter<std::string>("output_pcd", "field_map.pcd");
    accumulate_frames_ = declare_parameter<int>("accumulate_frames", 20);
    cell_degrees_ = declare_parameter<double>("cell_degrees", 0.1);
    leaf_size_ = declare_parameter<double>("leaf_size", 0.1);
    roi_.min_x = declare_parameter<double>("roi.min_x", 5.0);
    roi_.max_x = declare_parameter<double>("roi.max_x", 30.0);
    roi_.min_y = declare_parameter<double>("roi.min_y", -10.0);
    roi_.max_y = declare_parameter<double>("roi.max_y", 8.0);
    roi_.min_z = declare_parameter<double>("roi.min_z", -1.0);
    roi_.max_z = declare_parameter<double>("roi.max_z", 7.0);

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, 10,
        std::bind(&MapBuilder::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Collecting %d scans from %s", accumulate_frames_,
                lidar_topic.c_str());
    RCLCPP_INFO(get_logger(), "Reference map will be written to %s",
                output_pcd_.c_str());
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto frame = std::make_shared<Cloud>();
    pcl::fromROSMsg(*msg, *frame);
    frames_.push_back(frame);

    if (static_cast<int>(frames_.size()) < accumulate_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "collected %zu/%d",
                           frames_.size(), accumulate_frames_);
      return;
    }

    const CloudPtr map = accumulate(frames_, cell_degrees_, roi_, leaf_size_);
    if (map->empty()) {
      RCLCPP_ERROR(get_logger(),
                   "Reference map is empty, check the ROI and the LiDAR topic");
      rclcpp::shutdown();
      return;
    }

    const CloudConstPtr const_map = map;
    if (pcl::io::savePCDFileBinary(output_pcd_, *const_map) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to write %s", output_pcd_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Wrote %zu points to %s", const_map->size(),
                  output_pcd_.c_str());
    }
    rclcpp::shutdown();
  }

  std::string output_pcd_;
  int accumulate_frames_ = 20;
  double cell_degrees_ = 0.1;
  double leaf_size_ = 0.1;
  Roi roi_;
  std::vector<CloudConstPtr> frames_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace lidar_calibrate

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_calibrate::MapBuilder>());
  rclcpp::shutdown();
  return 0;
}
