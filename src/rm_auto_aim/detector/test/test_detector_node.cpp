#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "detector/detector_node.hpp"

TEST(DetectorNodeTest, StartsWithDefaultConfiguration)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto node = std::make_shared<rm_auto_aim::DetectorNode>(rclcpp::NodeOptions());

  EXPECT_STREQ(node->get_name(), "detector");

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}
