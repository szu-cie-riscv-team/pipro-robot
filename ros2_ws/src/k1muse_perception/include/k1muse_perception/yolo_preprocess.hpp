#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace k1muse_perception
{

struct LetterboxMetadata
{
  int source_width{0};
  int source_height{0};
  int input_width{0};
  int input_height{0};
  int resized_width{0};
  int resized_height{0};
  int left{0};
  int top{0};
  float scale{0.0F};
};

struct PreprocessConfig
{
  int input_width{320};
  int input_height{320};
  uint8_t padding_value{114};
  float normalize_scale{1.0F / 255.0F};
};

struct PreprocessedImage
{
  cv::Mat resized;
  cv::Mat letterboxed;
  std::vector<float> tensor;
  LetterboxMetadata letterbox;
};

// Converts a non-empty CV_8UC3 RGB image to a normalized NCHW tensor.
bool preprocess_rgb8(
  const cv::Mat & source, const PreprocessConfig & config,
  PreprocessedImage & output, std::string & error);

}  // namespace k1muse_perception
