#include "k1muse_communication/chassis_odometry.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace k1muse_communication
{

bool has_finite_motion(const k1muse_interfaces::msg::ChassisStatus & status)
{
  return std::isfinite(status.wcs_x) && std::isfinite(status.wcs_y) &&
         std::isfinite(status.wcs_direction) && std::isfinite(status.wcs_vx) &&
         std::isfinite(status.wcs_vy) && std::isfinite(status.omega);
}

ChassisOdometryOutput make_chassis_odometry(
  const k1muse_interfaces::msg::ChassisStatus & status)
{
  ChassisOdometryOutput output;
  const double yaw = status.wcs_direction;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  const auto quaternion = tf2::toMsg(orientation);

  output.odometry.header.stamp = status.header.stamp;
  output.odometry.header.frame_id = "odom";
  output.odometry.child_frame_id = "base_link";
  output.odometry.pose.pose.position.x = status.wcs_x;
  output.odometry.pose.pose.position.y = status.wcs_y;
  output.odometry.pose.pose.orientation = quaternion;
  output.odometry.twist.twist.linear.x =
    std::cos(yaw) * status.wcs_vx + std::sin(yaw) * status.wcs_vy;
  output.odometry.twist.twist.linear.y =
    -std::sin(yaw) * status.wcs_vx + std::cos(yaw) * status.wcs_vy;
  output.odometry.twist.twist.angular.z = status.omega;

  output.transform.header = output.odometry.header;
  output.transform.child_frame_id = output.odometry.child_frame_id;
  output.transform.transform.translation.x = output.odometry.pose.pose.position.x;
  output.transform.transform.translation.y = output.odometry.pose.pose.position.y;
  output.transform.transform.rotation = quaternion;
  return output;
}

bool is_tick_regression(uint32_t previous_tick, uint32_t current_tick)
{
  return current_tick < previous_tick;
}

}  // namespace k1muse_communication
