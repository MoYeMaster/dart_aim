#include "tracker/dart_tracker_node.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{
    DartTrackerNode::DartTrackerNode(const rclcpp::NodeOptions & options)
    : Node("dart_tracker_node", options)
    {
        const auto dart_detections_topic =
            this->declare_parameter("dart_detections_topic", "dart_detector/detections");
        const auto dart_track_topic =
            this->declare_parameter("dart_track_topic", "dart_tracker/filtered_detections");
        const double qp = this->declare_parameter("qp", 0.1);
        const double qv = this->declare_parameter("qv", 0.1);
        const double r = this->declare_parameter("r", 0.1);

        Eigen::VectorXd x0(4);
        x0 << 0, 0, 0, 0;
        Eigen::MatrixXd cov0 = Eigen::MatrixXd::Identity(4, 4) * 100;
        kf_.init(x0, cov0);
        kf_.setProcessNoise(qp, qv);
        kf_.setMeasurementNoise(r);
        last_time_ = this->now();

        dart_detections_sub_ = this->create_subscription<auto_aim_interfaces::msg::DartDetections>(
            dart_detections_topic, 10,
            std::bind(&DartTrackerNode::dart_detections_callback, this, std::placeholders::_1));
        dart_track_pub_ = this->create_publisher<auto_aim_interfaces::msg::DartDetections>(
            dart_track_topic, 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&DartTrackerNode::timer_callback, this));
    }

    void DartTrackerNode::dart_detections_callback(
        const auto_aim_interfaces::msg::DartDetections::SharedPtr msg)
    {
        if (msg->detections.empty())
        {
            return;
        }

        const auto best = std::max_element(
            msg->detections.begin(), msg->detections.end(),
            [](const auto & lhs, const auto & rhs) {
                return lhs.confidence < rhs.confidence;
            });

        kf_.update(best->pixel_x, best->pixel_y);
        filtered_dart_ = *best;
        has_measurement_ = true;
    }

    void DartTrackerNode::timer_callback()
    {
        current_time_ = this->now();
        const double dt = (current_time_ - last_time_).seconds();
        last_time_ = current_time_;
        if (dt > 0.0)
        {
            kf_.predict(dt);
        }

        if (!has_measurement_)
        {
            return;
        }

        const Eigen::VectorXd state = kf_.getState();
        filtered_dart_.pixel_x = static_cast<float>(state(0));
        filtered_dart_.pixel_y = static_cast<float>(state(1));

        auto dart_track_msg = std::make_unique<auto_aim_interfaces::msg::DartDetections>();
        dart_track_msg->header.stamp = current_time_;
        dart_track_msg->detections.push_back(filtered_dart_);
        dart_track_pub_->publish(std::move(dart_track_msg));
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::DartTrackerNode);
