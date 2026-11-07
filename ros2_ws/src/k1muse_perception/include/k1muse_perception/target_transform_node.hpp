#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace k1muse_perception
{

// Transform positions only: C10 orientation, extents and covariance are unestimated.
bool transform_targets(
  const vision_msgs::msg::Detection3DArray & input,
  const geometry_msgs::msg::TransformStamped & transform,
  vision_msgs::msg::Detection3DArray & output);

class TargetTransformNode : public rclcpp::Node
{
public:
  explicit TargetTransformNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void targets_callback(vision_msgs::msg::Detection3DArray::ConstSharedPtr message);

  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr subscription_;
};

}  // namespace k1muse_perception
