#pragma once

#include <string>

#include <rclcpp/subscription.hpp>

#include "auto_aim_interfaces/msg/dart_states.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/serial_writer/basic_serial_writer.hpp"

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
