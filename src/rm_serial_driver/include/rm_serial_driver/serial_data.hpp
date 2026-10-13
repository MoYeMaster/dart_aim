#pragma once

#include <string>

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float32.hpp>

#include "auto_aim_interfaces/msg/dart_states.hpp"
#include "rm_serial_driver/event_handler/basic_event_handler.hpp"
#include "rm_serial_driver/serial_writer/basic_serial_writer.hpp"

// Declare concrete handlers and writers here; keep registration in their .cpp files.
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
}  // namespace rm_serial_driver::handler

namespace rm_serial_driver::writer
{
class DyawWriter : public BasicSerialWriter
{
public:
  DyawWriter(rclcpp::Node * _node, const std::string & _name, WriteSerialFn _fn);

private:
  using DartStates = auto_aim_interfaces::msg::DartStates;

  void dyawSubCallback(const DartStates::ConstSharedPtr & _message);

  rclcpp::Subscription<DartStates>::SharedPtr dyaw_sub_;
};
}  // namespace rm_serial_driver::writer
