#include <rclcpp/rclcpp.hpp>

#include <k1muse_interfaces/msg/chassis_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "k1muse_communication/chassis_odometry.hpp"

namespace k1muse_communication
{

class ChassisOdometryNode : public rclcpp::Node
{
public:
  ChassisOdometryNode()
  : Node("chassis_odometry")
  {
    const bool publish_tf = declare_parameter<bool>("publish_tf", true);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    twist_variance_ = declare_parameter<std::vector<double>>(
      "twist_variance", {0.0004, 0.0004, 0.01});
    if (odom_frame_id_.empty()) {
      throw std::invalid_argument("odom_frame_id must be non-empty");
    }
    if (twist_variance_.size() != 3) {
      throw std::invalid_argument("twist_variance requires [vx, vy, omega]");
    }
    for (const double variance : twist_variance_) {
      if (!std::isfinite(variance) || variance <= 0.0) {
        throw std::invalid_argument("twist_variance requires finite positive values");
      }
    }
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    status_subscription_ = create_subscription<k1muse_interfaces::msg::ChassisStatus>(
      "/k1muse/chassis/status", 10,
      std::bind(&ChassisOdometryNode::handle_status, this, std::placeholders::_1));
    if (publish_tf) {
      transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }
  }

private:
  void handle_status(const k1muse_interfaces::msg::ChassisStatus::SharedPtr status)
  {
    if (!has_finite_motion(*status)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite chassis motion in odometry");
      return;
    }

    if (has_last_tick_ && is_tick_regression(last_tick_, status->tick_ms)) {
      RCLCPP_WARN(get_logger(), "STM32 tick regressed; starting a new odometry sequence");
      has_last_tick_ = false;
    }

    auto output = make_chassis_odometry(*status);
    output.odometry.header.frame_id = odom_frame_id_;
    output.odometry.twist.covariance[0] = twist_variance_[0];
    output.odometry.twist.covariance[7] = twist_variance_[1];
    output.odometry.twist.covariance[35] = twist_variance_[2];
    odometry_publisher_->publish(output.odometry);
    if (transform_broadcaster_) {
      output.transform.header = output.odometry.header;
      transform_broadcaster_->sendTransform(output.transform);
    }
    last_tick_ = status->tick_ms;
    has_last_tick_ = true;
  }

  uint32_t last_tick_{};
  bool has_last_tick_{};
  std::string odom_frame_id_;
  std::vector<double> twist_variance_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<k1muse_interfaces::msg::ChassisStatus>::SharedPtr status_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
};

}  // namespace k1muse_communication

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<k1muse_communication::ChassisOdometryNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("chassis_odometry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
