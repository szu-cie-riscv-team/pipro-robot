#ifndef K1MUSE_COMMUNICATION__CHASSIS_ODOMETRY_HPP_
#define K1MUSE_COMMUNICATION__CHASSIS_ODOMETRY_HPP_

#include <cstdint>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <k1muse_interfaces/msg/chassis_status.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace k1muse_communication
{

struct ChassisOdometryOutput
{
  nav_msgs::msg::Odometry odometry;
  geometry_msgs::msg::TransformStamped transform;
};

ChassisOdometryOutput make_chassis_odometry(
  const k1muse_interfaces::msg::ChassisStatus & status);
bool has_finite_motion(const k1muse_interfaces::msg::ChassisStatus & status);
bool is_tick_regression(uint32_t previous_tick, uint32_t current_tick);

}  // namespace k1muse_communication

#endif  // K1MUSE_COMMUNICATION__CHASSIS_ODOMETRY_HPP_
