#pragma once

#include <cstdint>
#include <string>
#include <sensor_msgs/msg/image.hpp>

namespace k1muse_perception
{

struct DepthImageView
{
  const sensor_msgs::msg::Image * image{nullptr};
  bool floating{false};
  bool swap_bytes{false};
  // The layout is validated once by make_depth_view before sampling.
  double at(uint32_t u, uint32_t v) const;
};

bool make_depth_view(
  const sensor_msgs::msg::Image & image, DepthImageView & view, std::string & error);

}  // namespace k1muse_perception
