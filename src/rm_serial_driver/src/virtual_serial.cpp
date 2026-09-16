#include <algorithm>
#include <functional>
#include <rclcpp/node.hpp>
#include <string>

#include "rm_serial_driver/event_handler/basic_event_handler.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{

using namespace std::chrono_literals;

class VirtualSerialNode : public rclcpp::Node
{
public:
  VirtualSerialNode(const rclcpp::NodeOptions _options);
  ~VirtualSerialNode() override = default;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<handler::BasicEventHandler::UniquePtr> handlers_;

  ReceivePacket getVirtualPacket();
  uint8_t getTaskModeParameter();
  uint8_t getDetectColorParameter();
  void receiveDataTimerCallback();
};

VirtualSerialNode::VirtualSerialNode(const rclcpp::NodeOptions _options)
: Node("virtual_serial", _options)
{
  // this->declare_parameter<std::string>("task_mode", "aim");
  // this->declare_parameter<bool>("reset_tracker", false);
  // this->declare_parameter<bool>("is_play", false);
  // this->declare_parameter<bool>("change_target", false);
  this->declare_parameter<double>("virtual_roll", 0.0);
  // this->declare_parameter<double>("virtual_pitch", 0.0);
  // this->declare_parameter<double>("virtual_yaw", 0.0);
  // this->declare_parameter<double>("bullet_speed", 22.0);


  // Init handlers
  for (const auto & factory : handler::registry::getHandlerFactories()) {
    try {
      handlers_.emplace_back(factory.create_(this));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Error while initializing %s : %s", factory.name_.c_str(), ex.what());
      throw ex;
    }
  }

  timer_ = this->create_wall_timer(1ms, [this]() {this->receiveDataTimerCallback();});
}

ReceivePacket VirtualSerialNode::getVirtualPacket()
{
  ReceivePacket virtual_packet {
    .header = 0x5A,
    // .detect_color = getDetectColorParameter(),
    // .task_mode = getTaskModeParameter(),
    // .reset_tracker = this->get_parameter("reset_tracker").as_bool(),
    // .is_play = static_cast<uint8_t>(this->get_parameter("is_play").as_bool()),
    // .change_target = this->get_parameter("change_target").as_bool(),
    .roll = static_cast<float>(this->get_parameter("virtual_roll").as_double()),
    // .pitch = static_cast<float>(this->get_parameter("virtual_pitch").as_double()),
    // .yaw = static_cast<float>(this->get_parameter("virtual_yaw").as_double()),
    // .bullet_speed = static_cast<float>(this->get_parameter("bullet_speed").as_double()),
    .checksum = 0,
  };

  return virtual_packet;
}

uint8_t VirtualSerialNode::getTaskModeParameter()
{
  const std::string task_mode = this->get_parameter("task_mode").as_string();

  if (task_mode == "aim" || task_mode == "auto") {
    return 0;
  }
  if (task_mode == "small_buff") {
    return 1;
  }
  if (task_mode == "large_buff") {
    return 2;
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "VirtualSerialNode: invalid task_mode %s, fallback to aim",
    task_mode.c_str());
  return 0;
}

uint8_t VirtualSerialNode::getDetectColorParameter()
{
  const int detect_color = this->get_parameter("detect_color").as_int();
  if (detect_color == 0 || detect_color == 1) {
    return static_cast<uint8_t>(detect_color);
  }

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "VirtualSerialNode: invalid detect_color %d, clamp to 0 or 1",
    detect_color);
  return static_cast<uint8_t>(std::clamp(detect_color, 0, 1));
}

void VirtualSerialNode::receiveDataTimerCallback()
{
  const ReceivePacket virtual_packet = getVirtualPacket();

  for (const auto & handler : handlers_) {
    handler->handle(virtual_packet);
  }
}

} // namespace rm_serial_driver

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::VirtualSerialNode);
