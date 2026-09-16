#include <utility>

#include "rm_serial_driver/serial_writer/dyaw_writer.hpp"

namespace rm_serial_driver::writer
{

DyawWriter::DyawWriter(
  rclcpp::Node * _node, const std::string & _name, WriteSerialFn _fn)
: BasicSerialWriter(_node, _name, std::move(_fn))
{
  const auto topic = node_->declare_parameter<std::string>(
    "dart_states_topic", "detector/dart_states");

  dyaw_sub_ = node_->create_subscription<DartStates>(
    topic, rclcpp::QoS(10),
    [this](const DartStates::ConstSharedPtr _message)
    {
      this->dyawSubCallback(_message);
    });
}

void DyawWriter::dyawSubCallback(
  const DartStates::ConstSharedPtr & _message)
{
  if (_message == nullptr) {
    return;
  }

  writeSerial_(rm_serial_driver::makeDyawPacket(_message->dyaw));
}

}  // namespace rm_serial_driver::writer

REGISTER_SERIAL_WRITER(rm_serial_driver::writer::DyawWriter);
