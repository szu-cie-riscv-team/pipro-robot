#include <exception>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_manager/voice_command_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::executors::SingleThreadedExecutor executor;
    std::string prepare_work;
    nlohmann::json catalog;
    while (rclcpp::ok()) {
      bool restart = false;
      auto handoff = std::exchange(prepare_work, {});
      auto node = k1muse_manager::make_voice_command_node(rclcpp::NodeOptions(), {},
        [&](const std::string & work,const nlohmann::json & snapshot) {
          prepare_work = work;
          catalog = snapshot;
          restart = true;
          executor.cancel();
        }, std::move(handoff),catalog);
      executor.add_node(node);
      executor.spin();
      executor.remove_node(node);
      node.reset();  // Joins the intent worker and frees its model before reloading configuration.
      if (!restart) {break;}
    }
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("voice_command"), "C15 failed: %s", error.what());
      rclcpp::shutdown();
      return 1;
    }
    return 0;
  }
  rclcpp::shutdown();
  return 0;
}
