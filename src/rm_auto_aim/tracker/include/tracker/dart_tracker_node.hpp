#ifndef DART_TRACKER_NODE_HPP
#define DART_TRACKER_NODE_HPP

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "auto_aim_interfaces/msg/dart_detections.hpp"
#include "kalman_filter.hpp"

namespace rm_auto_aim
{
using namespace std::chrono_literals;

class DartTrackerNode : public rclcpp::Node
{
public:
    explicit DartTrackerNode(const rclcpp::NodeOptions & options);

private:
    void timer_callback();
    void dart_detections_callback(const auto_aim_interfaces::msg::DartDetections::SharedPtr msg);

    rclcpp::Subscription<auto_aim_interfaces::msg::DartDetections>::SharedPtr dart_detections_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DartDetections>::SharedPtr dart_track_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    KalmanFilter kf_;
    rclcpp::Time last_time_;
    rclcpp::Time current_time_;
    auto_aim_interfaces::msg::DartDetection filtered_dart_;
    bool has_measurement_{false};
};
}

#endif  // DART_TRACKER_NODE_HPP
