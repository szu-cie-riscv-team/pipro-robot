#include <exception>
#include <memory>

#include "k1muse_perception/target_transform_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<k1muse_perception::TargetTransformNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "target_transform"), "C11 startup/runtime failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
