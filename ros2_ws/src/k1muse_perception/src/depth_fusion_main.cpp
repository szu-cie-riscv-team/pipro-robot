#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "k1muse_perception/depth_fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<k1muse_perception::DepthFusionNode>();
  // Sequence startup directly; fast configure must not depend on DDS event discovery.
  using State = lifecycle_msgs::msg::State;
  if (node->configure().id() != State::PRIMARY_STATE_INACTIVE ||
    node->activate().id() != State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "C10 startup failed");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  RCLCPP_INFO(node->get_logger(), "C10 Active");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
