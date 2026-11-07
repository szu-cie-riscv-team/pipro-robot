#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace k1muse_perception
{
struct OcrConfig
{
  std::string det_model_path, rec_model_path, dictionary_path;
  int det_width{640}, det_height{640}, rec_width{320}, rec_height{48};
  float det_threshold{0.3F}, box_threshold{0.6F}, unclip_ratio{2.0F};
  float nms_threshold{0.5F};
  std::size_t max_candidates{256}, max_text_lines{32};
  bool rec_output_logits{false};
};
struct OcrText
{
  std::string raw_text;
  std::array<cv::Point2f, 4> polygon;
  float score{0.0F};
};
struct OcrFrame
{
  std::vector<OcrText> texts;
  std::size_t candidates{0}, recognized{0};
  bool truncated{false}, cancelled{false};
};

// One worker owns the engine, sessions and all reusable buffers.
class OcrEngine
{
public:
  explicit OcrEngine(const OcrConfig & config);
  ~OcrEngine();
  OcrEngine(const OcrEngine &) = delete;
  OcrEngine & operator=(const OcrEngine &) = delete;
  OcrFrame infer(const cv::Mat & rgb8, const std::atomic_bool & stop);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace k1muse_perception
