#include "rm_serial_driver/serial_data.hpp"

// Add concrete handler implementations here and register them below.
namespace rm_serial_driver::handler
{
RollHandler::RollHandler(rclcpp::Node * _node, const std::string & _name)
: BasicEventHandler(_node, _name)
{
  roll_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "/serial_driver/roll", 0);
}
void RollHandler::handleEvent(const ReceivePacket & _packet)
{
  std_msgs::msg::Float32 roll;
  roll.data = readFloat<0>(_packet);
  roll_pub_->publish(roll);
}
}  // namespace rm_serial_driver::handler

REGISTER_EVENT_HANDLER(rm_serial_driver::handler::RollHandler);
