#include <rclcpp/rclcpp.hpp>

#include "k1muse_communication/stm32_communication_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(k1muse_communication::make_stm32_communication_node());
  rclcpp::shutdown();
  return 0;
}
