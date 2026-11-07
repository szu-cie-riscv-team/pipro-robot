#ifndef K1MUSE_COMMUNICATION__STM32_COMMUNICATION_NODE_HPP_
#define K1MUSE_COMMUNICATION__STM32_COMMUNICATION_NODE_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace k1muse_communication
{

std::shared_ptr<rclcpp::Node> make_stm32_communication_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace k1muse_communication

#endif  // K1MUSE_COMMUNICATION__STM32_COMMUNICATION_NODE_HPP_
