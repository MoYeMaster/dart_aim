#pragma once

#include "rm_serial_driver/event_handler/basic_event_handler.hpp"

#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rm_serial_driver::handler
{
class RollHandler : public BasicEventHandler
{
public:
  explicit RollHandler(rclcpp::Node * _node, const std::string & _name);
  void handleEvent(const ReceivePacket & _packet) override;

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
};
}
