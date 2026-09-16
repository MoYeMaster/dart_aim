#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

namespace rm_serial_driver::writer
{

using WriteSerialFn = std::function<void (const std::vector<uint8_t> &)>;

class BasicSerialWriter
{
protected:
  BasicSerialWriter(rclcpp::Node * _node, const std::string & _name, WriteSerialFn _fn)
  : node_(_node), name_(_name), writeSerial_(_fn)
  {
  }

  rclcpp::Node * node_;
  std::string name_;
  WriteSerialFn writeSerial_;

public:
  using UniquePtr = std::unique_ptr<BasicSerialWriter>;
  virtual ~BasicSerialWriter() = default;
};

namespace registry
{

struct WriterFactory
{
  std::string name_;
  std::unique_ptr<BasicSerialWriter>(* create_)(rclcpp::Node *, WriteSerialFn);
};

inline std::vector<WriterFactory> & getWriterFactory()
{
  static std::vector<WriterFactory> factories;
  return factories;
}

} // namespace registry

} // namespace rm_serial_driver::writer

#define RM_SERIAL_DRIVER_WRITER_CONCAT_IMPL(lhs, rhs) lhs ## rhs
#define RM_SERIAL_DRIVER_WRITER_CONCAT(lhs, rhs) RM_SERIAL_DRIVER_WRITER_CONCAT_IMPL(lhs, rhs)

// clang-format off

/// @brief Use this macro to register your writer after all of your class defination
// NOLINTBEGIN(readability-identifier-naming)
#define REGISTER_SERIAL_WRITER(WriterClass) \
  namespace \
  { \
  using WriteSerialFn = rm_serial_driver::writer::WriteSerialFn; \
  using BasicSerialWriter = rm_serial_driver::writer::BasicSerialWriter; \
  const bool RM_SERIAL_DRIVER_WRITER_CONCAT(serial_writer_registered_, __COUNTER__) = []() -> bool \
    { \
      rm_serial_driver::writer::registry::getWriterFactory().push_back( \
      { \
        #WriterClass, \
        [](rclcpp::Node * _node, WriteSerialFn _fn) -> std::unique_ptr<BasicSerialWriter> \
        { \
          return std::make_unique<WriterClass>(_node, #WriterClass, _fn); \
        }}); \
      return true; \
    }(); \
  }
// NOLINTEND(readability-identifier-naming)
// clang-format on
