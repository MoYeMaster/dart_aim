#ifndef TRACKER_NODE_HPP_
#define TRACKER_NODE_HPP_

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/lights.hpp"
#include "kalman_filter.hpp"

namespace rm_auto_aim
{
using namespace std::chrono_literals;   

class TrackerNode : public rclcpp::Node
{
public:
    explicit TrackerNode(const rclcpp::NodeOptions & options);

private:
    void timer_callback();
    void lights_callback(const auto_aim_interfaces::msg::Lights::SharedPtr msg);

    rclcpp::Subscription<auto_aim_interfaces::msg::Lights>::SharedPtr lights_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Lights>::SharedPtr lights_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    KalmanFilter kf_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    rclcpp::Time last_time_;
    rclcpp::Time current_time_;
};
} // namespace rm_auto_aim

#endif  // TRACKER_NODE_HPP_