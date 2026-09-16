#pragma once

#include <atomic>
#include <mutex>
#include <vector>
#include <thread>

#include <rclcpp/node.hpp>
#include <serial_driver/serial_driver.hpp>

#include "rm_serial_driver/event_handler/basic_event_handler.hpp"
#include "rm_serial_driver/serial_writer/basic_serial_writer.hpp"

namespace rm_serial_driver
{

class SerialDriverNode : public rclcpp::Node
{
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions & _options);

  ~SerialDriverNode() override;

  void writeSerial(const std::vector<uint8_t> & _data);

private:
  void receiveData();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::thread receive_thread_;
  std::atomic_bool shutting_down_{false};
  std::mutex serial_mutex_;
  void reOpenPort();

  std::vector<handler::BasicEventHandler::UniquePtr> handlers_;
  std::vector<writer::BasicSerialWriter::UniquePtr> writers_;
  void declareSerialParameters();

}; // SerialDriverNode


} // namespace rm_serial_driver
