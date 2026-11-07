#ifndef K1MUSE_MANAGER__TASK_MANAGER_NODE_HPP_
#define K1MUSE_MANAGER__TASK_MANAGER_NODE_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace k1muse_manager
{

std::shared_ptr<rclcpp::Node> make_task_manager_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace k1muse_manager

#endif  // K1MUSE_MANAGER__TASK_MANAGER_NODE_HPP_
