#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "k1muse_manager/task_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(k1muse_manager::make_task_manager_node());
  rclcpp::shutdown();
  return 0;
}
