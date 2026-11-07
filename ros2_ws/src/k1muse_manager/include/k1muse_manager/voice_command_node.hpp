#ifndef K1MUSE_MANAGER__VOICE_COMMAND_NODE_HPP_
#define K1MUSE_MANAGER__VOICE_COMMAND_NODE_HPP_

#include <memory>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include "k1muse_manager/voice_intent.hpp"
#include "k1muse_manager/drug_catalog.hpp"

namespace k1muse_manager
{

// Cancel retains the existing voice-owned navigation cancellation semantics.
enum class VoiceCommandType {None, Navigate, Cancel, Fetch, CancelTask, Survey};

struct VoiceCommand
{
  VoiceCommandType type{VoiceCommandType::None};
  std::string task_name;
  std::string drug_name{};
  int quantity{1};
};

// Returns 0 for an explicit unsupported/ambiguous quantity, otherwise 1..3.
int resolve_fetch_quantity(const std::string & text);

VoiceCommand parse_voice_command(
  const std::string & text, const std::string & address_word,
  const std::string & navigate_word, const std::string & cancel_navigation_word,
  const std::vector<VoiceDrug> & drugs = {});

std::shared_ptr<rclcpp::Node> make_voice_command_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  CloudTransport transport = {},
  std::function<void(const std::string &, const nlohmann::json &)> restart = {},
  std::string prepare_work = {}, nlohmann::json catalog = {});

}  // namespace k1muse_manager

#endif  // K1MUSE_MANAGER__VOICE_COMMAND_NODE_HPP_
