#include "tracker/tracker_node.hpp"

#include <cmath>

namespace rm_auto_aim
{
    TrackerNode::TrackerNode(const rclcpp::NodeOptions & options) : Node("tracker_node", options)
    {
        const auto lights_topic = this->declare_parameter("lights_topic", "detector/lights");
        const auto filtered_lights_topic =
            this->declare_parameter("filtered_lights_topic", "tracker/filtered_lights");
        const auto dart_states_topic =
            this->declare_parameter("dart_states_topic", "detector/dart_states");
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

        lights_sub_ = this->create_subscription<auto_aim_interfaces::msg::Lights>(
            lights_topic, 10, std::bind(&TrackerNode::lights_callback, this, std::placeholders::_1));
        lights_pub_ = this->create_publisher<auto_aim_interfaces::msg::Lights>(filtered_lights_topic, 10);
        dart_states_pub_ =
            this->create_publisher<auto_aim_interfaces::msg::DartStates>(dart_states_topic, 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&TrackerNode::timer_callback, this));
    }

    void TrackerNode::lights_callback(const auto_aim_interfaces::msg::Lights::SharedPtr msg)
    {
        if (msg->lights.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No lights detected.");
            return;
        }

        const auto & light = msg->lights[0];
        double zx = light.pose.position.x;
        double zz = light.pose.position.z;
        if (zz <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring light with invalid depth: %.3f", zz);
            return;
        }

        kf_.update(zx, zz);
        filtered_light_ = light;
        has_measurement_ = true;
    }

    void TrackerNode::timer_callback()
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

        Eigen::VectorXd state = kf_.getState();
        filtered_light_.pose.position.x = state(0);
        filtered_light_.pose.position.z = state(1);

        auto filtered_lights_msg = std::make_unique<auto_aim_interfaces::msg::Lights>();
        filtered_lights_msg->header.stamp = current_time_;
        // const auto timestamp = current_time_.nanoseconds();
        // filtered_lights_msg->header.stamp.sec =
        // static_cast<int32_t>(timestamp / 1000000000);
        // filtered_lights_msg->header.stamp.nanosec =
        // static_cast<uint32_t>(timestamp % 1000000000);

        filtered_lights_msg->lights.push_back(filtered_light_);
        lights_pub_->publish(std::move(filtered_lights_msg));

        auto dart_states_msg = std::make_unique<auto_aim_interfaces::msg::DartStates>();
        dart_states_msg->dyaw = static_cast<float>(std::atan2(state(0), state(1)));
        dart_states_pub_->publish(std::move(dart_states_msg));
    }


}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TrackerNode);
