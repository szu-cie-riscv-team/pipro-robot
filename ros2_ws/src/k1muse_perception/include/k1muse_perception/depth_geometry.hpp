#pragma once

#include "k1muse_perception/depth_image_view.hpp"

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace k1muse_perception
{

struct DepthRoi
{
  uint32_t x1{0}, y1{0}, x2{0}, y2{0};
  double center_u{0.0}, center_v{0.0}, size_x{0.0}, size_y{0.0};
};

bool make_depth_roi(
  const vision_msgs::msg::BoundingBox2D & box, uint32_t width, uint32_t height,
  DepthRoi & roi);
bool valid_depth_calibration(const sensor_msgs::msg::CameraInfo & info, std::string & error);
bool project_depth(
  const sensor_msgs::msg::CameraInfo & info, double u, double v, double z,
  geometry_msgs::msg::Point & point);

}  // namespace k1muse_perception
