#include <array>
#include <cstdint>
// #include <mutex>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver_node.hpp"

namespace rm_serial_driver
{

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions & _options)
: Node("rm_serial_driver", _options), owned_ctx_(std::make_unique<IoContext>(2)),
  serial_driver_(std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_))
{
  this->declareSerialParameters();
  // Init serial port
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error creating serial port: %s - %s",
      device_name_.c_str(), ex.what());
    throw ex;
  }


  // Init handlers
  for (const auto & factory : handler::registry::getHandlerFactories()) {
    try {
      handlers_.emplace_back(factory.create_(this));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Error while initializing %s : %s", factory.name_.c_str(),
        ex.what());
      throw ex;
    }
  }

  for (const auto & factory : writer::registry::getWriterFactory()) {
    try {
      writers_.emplace_back(
        factory.create_(
          this,
          [this](const std::vector<uint8_t> & _data)
          {
            this->writeSerial(_data);
          }));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Error while initializing %s : %s",
        factory.name_.c_str(), ex.what());
      throw;
    }
  }

  receive_thread_ = std::thread(&SerialDriverNode::receiveData, this);


}

SerialDriverNode::~SerialDriverNode()
{
  shutting_down_.store(true);
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
  } catch (const std::exception & ex) {
    RCLCPP_DEBUG(get_logger(), "Error while closing serial port: %s", ex.what());
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}


void SerialDriverNode::writeSerial(const std::vector<uint8_t> & _data)
{
  try {
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_driver_->port()->send(_data);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    this->reOpenPort();
  }
}


void SerialDriverNode::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data(sizeof(ReceivePacket) - 1);

  // clang-format off
  while (rclcpp::ok() && !shutting_down_.load()) {
    try {
      serial_driver_->port()->receive(header);

      if (0x5A != header[0]) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
        continue;
      }

      serial_driver_->port()->receive(data);

      ReceivePacket packet = fromVectorWithoutHeader(data);

      bool crc_ok =
        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      if (!crc_ok) {
        RCLCPP_ERROR(this->get_logger(), "CRC Error!");
        continue;
      }

      for (const auto & handler : handlers_) {
        handler->handle(packet);
      }

    } catch (const std::exception & ex) {
      if (shutting_down_.load()) {
        break;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(), 20, "Error while receiving data: %s", ex.what());
      this->reOpenPort();
    }
  }
  // clang-format on
}

void SerialDriverNode::reOpenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  while (rclcpp::ok() && !shutting_down_.load()) {
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
      }
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Successfully reopened port");
      return;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
      if (rclcpp::ok() && !shutting_down_.load()) {
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
  }
}


void SerialDriverNode::declareSerialParameters()
try
{
  this->device_name_ = declare_parameter<std::string>("device_name", "");

  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  baud_rate = declare_parameter<int>("baud_rate", 0);

  const auto fc_string = declare_parameter<std::string>("flow_control", "");

  if (fc_string == "none") {
    fc = FlowControl::NONE;
  } else if (fc_string == "hardware") {
    fc = FlowControl::HARDWARE;
  } else if (fc_string == "software") {
    fc = FlowControl::SOFTWARE;
  } else {
    throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or hardware."};
  }

  const auto pt_string = declare_parameter<std::string>("parity", "");

  if (pt_string == "none") {
    pt = Parity::NONE;
  } else if (pt_string == "odd") {
    pt = Parity::ODD;
  } else if (pt_string == "even") {
    pt = Parity::EVEN;
  } else {
    throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
  }

  const auto sb_string = declare_parameter<std::string>("stop_bits", "");

  if (sb_string == "1" || sb_string == "1.0") {
    sb = StopBits::ONE;
  } else if (sb_string == "1.5") {
    sb = StopBits::ONE_POINT_FIVE;
  } else if (sb_string == "2" || sb_string == "2.0") {
    sb = StopBits::TWO;
  } else {
    throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

} catch (const rclcpp::ParameterTypeException & ex) {
  RCLCPP_ERROR(this->get_logger(), "Parameters declare error: %s", ex.what());
  throw ex;
} catch (const std::exception & ex) {
  RCLCPP_ERROR(this->get_logger(), "Unknown Error: %s", ex.what());
  throw ex;
}

} // namespace rm_serial_driver


#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriverNode);
