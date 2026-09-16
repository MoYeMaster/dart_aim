#include "tracker/tracker_node.hpp"

namespace rm_auto_aim
{
    TrackerNode::TrackerNode(const rclcpp::NodeOptions & options) : Node("tracker_node")
    {
        this->declare_parameter("qp",0.1);
        this->declare_parameter("qv",0.1);
        double qp = this->get_parameter("qp").as_double();
        double qv = this->get_parameter("qv").as_double();
        this->declare_parameter("r",0.1);
        double r = this->get_parameter("r").as_double();

        Eigen::VectorXd x0(4);
        x0 << 0, 0, 0, 0;
        Eigen::MatrixXd cov0 = Eigen::MatrixXd::Identity(4, 4) * 100;
        kf_.init(x0, cov0);
        kf_.setProcessNoise(qp, qv);
        kf_.setMeasurementNoise(r);

        lights_sub_ = this->create_subscription<auto_aim_interfaces::msg::Lights>(
            "lights", 10, std::bind(&TrackerNode::lights_callback, this, std::placeholders::_1));
        lights_pub_ = this->create_publisher<auto_aim_interfaces::msg::Lights>("filtered_lights", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&TrackerNode::timer_callback, this));
    }

    void TrackerNode::timer_callback()
    {
        rclcpp::Time now = this->now();
        double dt = (current_time_ - last_time_).seconds();
        last_time_ = current_time_;
        kf_.predict(dt);

        auto filtered_lights_msg = std::make_shared<auto_aim_interfaces::msg::Lights>();
        Eigen::VectorXd state = kf_.getState();
        // filtered_lights_msg.x = state(0);
        // filtered_lights_msg.y = state(1);
        lights_pub_->publish(*filtered_lights_msg);
    }


}

