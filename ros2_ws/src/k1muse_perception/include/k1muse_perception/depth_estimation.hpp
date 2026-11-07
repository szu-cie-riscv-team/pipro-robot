#pragma once

#include <cstddef>
#include <vector>

#include "k1muse_perception/depth_geometry.hpp"

namespace k1muse_perception
{

struct DepthConfig
{
  int center_window_size{5};
  size_t min_valid_count{3};
  double min_depth_m{0.1};
  double max_depth_m{2.0};
};

struct DepthEstimate
{
  bool valid{false};
  const char * reason{"insufficient_samples"};
  double u{0.0}, v{0.0}, depth_m{0.0};
  size_t sampled_count{0}, valid_count{0};
};

struct WeightedDepthSample
{
  double depth, weight, u, v;
};

struct DepthScratch
{
  std::vector<WeightedDepthSample> samples;
  std::vector<WeightedDepthSample> selected;
  std::vector<double> values;
};

using DepthAlgorithm = DepthEstimate (*)(
  const DepthImageView &, const DepthRoi &, const DepthConfig &, DepthScratch &);

bool valid_depth_config(const DepthConfig & config, std::string & error);
DepthEstimate depth_center_mean(
  const DepthImageView & image, const DepthRoi & roi, const DepthConfig & config,
  DepthScratch & scratch);
DepthEstimate depth_roi_median(
  const DepthImageView & image, const DepthRoi & roi, const DepthConfig & config,
  DepthScratch & scratch);
DepthEstimate depth_yolo_weighted(
  const DepthImageView & image, const DepthRoi & roi, const DepthConfig & config,
  DepthScratch & scratch);

}  // namespace k1muse_perception
