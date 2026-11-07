#include "k1muse_perception/yolo_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace k1muse_perception
{

bool preprocess_rgb8(
  const cv::Mat & source, const PreprocessConfig & config,
  PreprocessedImage & output, std::string & error)
{
  if (source.empty() || source.type() != CV_8UC3) {
    error = "expected a non-empty CV_8UC3 RGB image";
    return false;
  }
  if (config.input_width <= 0 || config.input_height <= 0 ||
    !std::isfinite(config.normalize_scale) || config.normalize_scale <= 0.0F)
  {
    error = "invalid preprocessing configuration";
    return false;
  }

  const float width_scale = static_cast<float>(config.input_width) /
    static_cast<float>(source.cols);
  const float height_scale = static_cast<float>(config.input_height) /
    static_cast<float>(source.rows);
  const float scale = std::min(width_scale, height_scale);
  const int resized_width = static_cast<int>(std::lround(source.cols * scale));
  const int resized_height = static_cast<int>(std::lround(source.rows * scale));
  if (resized_width <= 0 || resized_height <= 0 ||
    resized_width > config.input_width || resized_height > config.input_height)
  {
    error = "letterbox resize dimensions are invalid";
    return false;
  }

  const int left = (config.input_width - resized_width) / 2;
  const int top = (config.input_height - resized_height) / 2;
  cv::resize(
    source, output.resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
  output.letterboxed.create(config.input_height, config.input_width, CV_8UC3);
  output.letterboxed.setTo(
    cv::Scalar(config.padding_value, config.padding_value, config.padding_value));
  output.resized.copyTo(output.letterboxed(cv::Rect(left, top, resized_width, resized_height)));

  const size_t plane_size = static_cast<size_t>(config.input_width) *
    static_cast<size_t>(config.input_height);
  output.tensor.resize(plane_size * 3U);
  for (int y = 0; y < config.input_height; ++y) {
    const auto * row = output.letterboxed.ptr<cv::Vec3b>(y);
    for (int x = 0; x < config.input_width; ++x) {
      const size_t offset = static_cast<size_t>(y) * static_cast<size_t>(config.input_width) +
        static_cast<size_t>(x);
      output.tensor[offset] = static_cast<float>(row[x][0]) * config.normalize_scale;
      output.tensor[plane_size + offset] = static_cast<float>(row[x][1]) * config.normalize_scale;
      output.tensor[2U * plane_size + offset] =
        static_cast<float>(row[x][2]) * config.normalize_scale;
    }
  }

  output.letterbox = LetterboxMetadata{
    source.cols, source.rows, config.input_width, config.input_height,
    resized_width, resized_height, left, top, scale};
  return true;
}

}  // namespace k1muse_perception
