#include "rm_serial_driver/serial_data.hpp"
// Add concrete handler implementations here and register them below.
namespace rm_serial_driver::handler
{
YawHandler::YawHandler(rclcpp::Node * _node, const std::string & _name)
: BasicEventHandler(_node, _name)
{
  yaw_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
    "serial_driver/pose", 0);
}
void YawHandler::handleEvent(const ReceivePacket & _packet)
{
  std_msgs::msg::Float32 yaw;
  yaw.data = readFloat<0>(_packet);
  yaw_pub_->publish(yaw);
}
}  // namespace rm_serial_driver::handler

REGISTER_EVENT_HANDLER(rm_serial_driver::handler::YawHandler);
