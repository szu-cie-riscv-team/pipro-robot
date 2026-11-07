#include "k1muse_perception/depth_geometry.hpp"
#include "k1muse_perception/depth_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace k1muse_perception
{

bool make_depth_roi(
  const vision_msgs::msg::BoundingBox2D & box, uint32_t width, uint32_t height,
  DepthRoi & roi)
{
  const double u = box.center.position.x, v = box.center.position.y;
  if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(box.size_x) ||
    !std::isfinite(box.size_y) || box.size_x <= 0 || box.size_y <= 0 ||
    box.center.theta != 0.0 || !width || !height)
  {
    return false;
  }
  roi = {
    static_cast<uint32_t>(std::clamp(std::floor(u - box.size_x / 2), 0.0, double(width))),
    static_cast<uint32_t>(std::clamp(std::floor(v - box.size_y / 2), 0.0, double(height))),
    static_cast<uint32_t>(std::clamp(std::ceil(u + box.size_x / 2), 0.0, double(width))),
    static_cast<uint32_t>(std::clamp(std::ceil(v + box.size_y / 2), 0.0, double(height))),
    u, v, box.size_x, box.size_y};
  return roi.x2 > roi.x1 && roi.y2 > roi.y1;
}

bool valid_depth_config(const DepthConfig & config, std::string & error)
{
  if (config.center_window_size <= 0 || config.center_window_size % 2 == 0 ||
    !config.min_valid_count || !std::isfinite(config.min_depth_m) ||
    !std::isfinite(config.max_depth_m) || config.min_depth_m <= 0 ||
    config.max_depth_m <= config.min_depth_m)
  {
    error = "invalid depth range, sample count, or odd window size";
    return false;
  }
  return true;
}

bool valid_depth_calibration(const sensor_msgs::msg::CameraInfo & info, std::string & error)
{
  if (!info.width || !info.height || info.header.frame_id.empty() ||
    !std::all_of(info.k.begin(), info.k.end(), [](double x) {return std::isfinite(x);}) ||
    !std::all_of(info.d.begin(), info.d.end(), [](double x) {return std::isfinite(x);}) ||
    info.k[0] <= 0 || info.k[4] <= 0 || info.k[8] != 1 || info.k[1] != 0 ||
    info.k[3] != 0 || info.k[6] != 0 || info.k[7] != 0 ||
    info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset || info.roi.y_offset ||
    (info.roi.width && info.roi.width != info.width) ||
    (info.roi.height && info.roi.height != info.height))
  {
    error = "invalid_calibration";
    return false;
  }
  if ((info.distortion_model != "plumb_bob" &&
    !(info.distortion_model.empty() && info.d.empty())) ||
    (!info.d.empty() && info.d.size() != 4 && info.d.size() != 5))
  {
    error = "unsupported_distortion";
    return false;
  }
  return true;
}

bool project_depth(
  const sensor_msgs::msg::CameraInfo & info, double u, double v, double z,
  geometry_msgs::msg::Point & point)
{
  double x = (u - info.k[2]) / info.k[0];
  double y = (v - info.k[5]) / info.k[4];
  if (std::any_of(info.d.begin(), info.d.end(), [](double d) {return d != 0.0;})) {
    std::vector<cv::Point2d> pixel{{u, v}}, normalized;
    cv::Mat camera(3, 3, CV_64F, const_cast<double *>(info.k.data()));
    cv::undistortPoints(pixel, normalized, camera, info.d);
    x = normalized[0].x;
    y = normalized[0].y;
  }
  point.x = x * z;
  point.y = y * z;
  point.z = z;
  return z > 0 && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(z);
}

}  // namespace k1muse_perception
