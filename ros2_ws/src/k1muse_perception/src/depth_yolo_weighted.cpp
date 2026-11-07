// Copyright (C) 2023 Miguel Ángel González Santamarta
// Copyright (C) 2026 K1Muse contributors (C++ port and representative pixel)
// SPDX-License-Identifier: GPL-3.0-or-later
// Port of _compute_spatial_weights and the depth-centre statistics in
// mgonzs13/yolo_ros, revision eb11e81f7dbdb81c23a61d743ce0bac3cf9b07eb.
// This program is free software: you can redistribute it and/or modify it
// under the GNU General Public License, version 3 or any later version.
// It is distributed WITHOUT ANY WARRANTY; see LICENSES/GPL-3.0.txt.

#include "k1muse_perception/depth_estimation.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace k1muse_perception
{
namespace
{

double percentile(const std::vector<double> & sorted, double fraction)
{
  const double position = fraction * (sorted.size() - 1);
  const size_t low = static_cast<size_t>(position);
  const size_t high = std::min(low + 1, sorted.size() - 1);
  return sorted[low] + (sorted[high] - sorted[low]) * (position - low);
}

size_t weighted_index(const std::vector<WeightedDepthSample> & samples, double fraction)
{
  double total = 0.0;
  for (const auto & sample : samples) {
    total += sample.weight;
  }
  double cumulative = 0.0;
  for (size_t i = 0; i < samples.size(); ++i) {
    cumulative += samples[i].weight;
    if (cumulative / total >= fraction) {return i;}
  }
  return samples.size() - 1;
}

void sort_depth(std::vector<WeightedDepthSample> & samples)
{
  std::sort(
    samples.begin(), samples.end(), [](const auto & a, const auto & b) {
      if (a.depth != b.depth) {return a.depth < b.depth;}
      if (a.v != b.v) {return a.v < b.v;}
      return a.u < b.u;
    });
}

}  // namespace

DepthEstimate depth_yolo_weighted(
  const DepthImageView & image, const DepthRoi & roi, const DepthConfig & config,
  DepthScratch & scratch)
{
  DepthEstimate result;
  auto & samples = scratch.samples;
  auto & selected = scratch.selected;
  auto & values = scratch.values;
  samples.clear();
  selected.clear();
  values.clear();
  for (uint32_t v = roi.y1; v < roi.y2; ++v) {
    for (uint32_t u = roi.x1; u < roi.x2; ++u) {
      ++result.sampled_count;
      const double depth = image.at(u, v);
      if (!std::isfinite(depth) || depth < config.min_depth_m || depth > config.max_depth_m) {
        continue;
      }
      const double dx = (u - roi.center_u) / (roi.size_x / 2 + 1e-6);
      const double dy = (v - roi.center_v) / (roi.size_y / 2 + 1e-6);
      const double weight = std::max(0.3, std::exp(-0.5 * (dx * dx + dy * dy) / (0.8 * 0.8)));
      samples.push_back({depth, weight, double(u), double(v)});
      values.push_back(depth);
    }
  }
  result.valid_count = samples.size();
  if (samples.size() < config.min_valid_count) {return result;}
  if (samples.size() < 4) {
    sort_depth(samples);
    const auto & left = samples[(samples.size() - 1) / 2];
    const auto & right = samples[samples.size() / 2];
    result.depth_m = (left.depth + right.depth) / 2;
    result.u = (left.u + right.u) / 2;
    result.v = (left.v + right.v) / 2;
    result.valid = true;
    result.reason = "ok";
    return result;
  }
  std::sort(values.begin(), values.end());
  const double minimum = values.front(), maximum = values.back();
  const double range = maximum - minimum;
  const size_t bins = range <= 0 ? 30 :
    static_cast<size_t>(std::clamp(range / 0.01, 20.0, 60.0));
  // NumPy histogram expands a constant range by 0.5 at each end.
  const double lower = range > 0 ? minimum : minimum - 0.5;
  const double upper = range > 0 ? maximum : maximum + 0.5;
  const double bin_width = (upper - lower) / bins;
  const double iqr = percentile(values, 0.75) - percentile(values, 0.25);
  std::array<double, 60> histogram{};
  for (const auto & sample : samples) {
    const size_t bin = std::min(bins - 1, static_cast<size_t>((sample.depth - lower) / bin_width));
    histogram[bin] += sample.weight;
  }
  const int kernel = std::min<size_t>(5, bins / 4);
  size_t peak = 0;
  double peak_weight = -1.0;
  for (size_t i = 0; i < bins; ++i) {
    double smooth = 0.0;
    for (int j = 0; j < kernel; ++j) {
      const int index = static_cast<int>(i) + (kernel - 1) / 2 - j;
      if (index >= 0 && index < static_cast<int>(bins)) {smooth += histogram[index] / kernel;}
    }
    if (smooth > peak_weight) {peak = i; peak_weight = smooth;}
  }
  const double mode = lower + (peak + 0.5) * bin_width;
  values.clear();
  for (const auto & sample : samples) {
    values.push_back(std::abs(sample.depth - mode));
  }
  std::sort(values.begin(), values.end());
  const double mad = percentile(values, 0.5);
  const double threshold = iqr < 0.03 ? std::clamp(3.5 * mad, 0.08, 0.30) :
    (iqr < 0.10 ? std::clamp(4.0 * mad, 0.12, 0.40) : std::clamp(5.0 * mad, 0.15, 0.60));
  for (const auto & sample : samples) {
    if (std::abs(sample.depth - mode) <= threshold) {selected.push_back(sample);}
  }
  const size_t min_points = std::max<size_t>(6, static_cast<size_t>(samples.size() * 0.15));
  if (selected.size() < min_points) {
    sort_depth(samples);
    const double low = samples[weighted_index(samples, 0.02)].depth;
    const double high = samples[weighted_index(samples, 0.85)].depth;
    selected.clear();
    for (const auto & sample : samples) {
      if (sample.depth >= low && sample.depth <= high) {selected.push_back(sample);}
    }
  }
  if (selected.empty()) {selected = samples;}
  sort_depth(selected);
  size_t start = weighted_index(selected, 0.02);
  size_t end = weighted_index(selected, 0.98);
  if (end <= start) {start = 0; end = selected.size();}
  double weight_sum = 0.0;
  for (size_t i = start; i < end; ++i) {
    const auto & sample = selected[i];
    weight_sum += sample.weight;
    result.depth_m += sample.depth * sample.weight;
    result.u += sample.u * sample.weight;
    result.v += sample.v * sample.weight;
  }
  result.depth_m /= weight_sum;
  result.u /= weight_sum;
  result.v /= weight_sum;
  result.valid = true;
  result.reason = "ok";
  return result;
}

}  // namespace k1muse_perception
