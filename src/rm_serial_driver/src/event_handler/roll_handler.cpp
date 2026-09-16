#include "rm_serial_driver/event_handler/roll_handler.hpp"

namespace rm_serial_driver::handler
{
RollHandler::RollHandler(rclcpp::Node * _node, const std::string & _name)
: BasicEventHandler(_node, _name)
{
  roll_pub_ = node_->create_publisher<std_msgs::msg::Float32>("roll", 0);
}
void RollHandler::handleEvent(const ReceivePacket & _packet)
{
  std_msgs::msg::Float32 roll;
  roll.data = _packet.roll;
  roll_pub_->publish(roll);
}
}

REGISTER_EVENT_HANDLER(rm_serial_driver::handler::RollHandler);
