#pragma once

#include "rm_serial_driver/packet.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

namespace rm_serial_driver::handler
{

/**
 * @brief 串口数据处理器基类，最好一个topic对应一个处理器
 *
 * #### 使用方法：
 *
 * 定义一个新的处理类并公有继承这个类。
 *
 * 构造函数接受`(rclcpp::Node * _node, std::string _name)`参数并用初始化列表初始化基类
 *
 * 不需要考虑构造参数从哪里来（构造宏会做好一切）
 *
 * 正常在构造函数中创建发布者并用成员变量存储其shared_ptr
 *
 * 创建发布者使用`node_->create_publisher(args);`即可
 *
 * 实现基类的`void handleEvent(const ReceivePacket & packet)`函数
 *
 * 最后在源文件最后一行使用`REGISTER_EVENT_HANDLER`宏并把类传入即可
 *
 * 不需要改上层rm_serial_driver的东西，包括其构造函数
 *
 * @note 每个新的事件处理类所在头文件至少要被一个会被编译的.cpp文件include，否则无法自注册
 */
class BasicEventHandler
{
protected:
  BasicEventHandler(rclcpp::Node * _node, const std::string & _name)
  : node_(_node), name_(_name)
  {
    if (node_ == nullptr) {
      throw std::invalid_argument("Node pointer cannot be null");
    }
  }

  virtual void handleEvent(const ReceivePacket & _packet) = 0;

  rclcpp::Node * node_ = nullptr;
  std::string name_ = "BasicEventHandler";

public:
  using UniquePtr = std::unique_ptr<BasicEventHandler>;

  // clang-format off
  void handle(const ReceivePacket & _packet) try
  {
    handleEvent(_packet);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in %s : %s", name_.c_str(), ex.what());
    throw ex;
  }
  // clang-format on

  virtual ~BasicEventHandler() = default;
};


namespace registry
{
// using HandlerFactory = std::unique_ptr<BasicEventHandler> (*)(rclcpp::Node *);

struct HandlerFactory
{
  std::string name_;
  std::unique_ptr<BasicEventHandler>(* create_)(rclcpp::Node *);
};

inline std::vector<HandlerFactory> & getHandlerFactories()
{
  static std::vector<HandlerFactory> factories;
  return factories;
}
} // namespace registry

} // namespace rm_serial_driver::handler

#define RM_SERIAL_DRIVER_HANDLER_CONCAT_IMPL(lhs, rhs) lhs ## rhs
#define RM_SERIAL_DRIVER_HANDLER_CONCAT(lhs, rhs) RM_SERIAL_DRIVER_HANDLER_CONCAT_IMPL(lhs, rhs)

// clang-format of

/// @brief Use this macro to register your handler after all of your class defination
// NOLINTBEGIN(readability-identifier-naming)
#define REGISTER_EVENT_HANDLER(HandlerClass) \
  namespace \
  { \
  using BasicEventHandler = rm_serial_driver::handler::BasicEventHandler; \
  const bool RM_SERIAL_DRIVER_HANDLER_CONCAT(event_handler_registered_, __COUNTER__) = []() -> bool \
    { \
      rm_serial_driver::handler::registry::getHandlerFactories().push_back( \
      { \
        #HandlerClass, \
        [](rclcpp::Node * _node_ptr) -> std::unique_ptr<BasicEventHandler> \
        {return std::make_unique<HandlerClass>(_node_ptr, #HandlerClass);}}); \
      return true; \
    }(); \
  }
// NOLINTEND(readability-identifier-naming)
// clang-format on
