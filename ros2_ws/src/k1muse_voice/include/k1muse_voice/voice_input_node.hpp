#ifndef K1MUSE_VOICE__VOICE_INPUT_NODE_HPP_
#define K1MUSE_VOICE__VOICE_INPUT_NODE_HPP_

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace k1muse_voice
{

std::shared_ptr<rclcpp::Node> make_voice_input_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace k1muse_voice

#endif  // K1MUSE_VOICE__VOICE_INPUT_NODE_HPP_
