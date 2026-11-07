#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <vision_msgs/msg/detection2_d.hpp>

#include "k1muse_perception/yolo_preprocess.hpp"

namespace k1muse_perception
{

struct DetectionCandidate
{
  float center_x{0.0F};
  float center_y{0.0F};
  float width{0.0F};
  float height{0.0F};
  int class_index{-1};
  float score{0.0F};
};

struct PostprocessConfig
{
  float confidence_threshold{0.25F};
  float nms_iou_threshold{0.45F};
  int max_detections{100};
  std::vector<std::string> labels;
};

// Decodes the explicitly selected single-output float [1, 84, N] contract.
bool decode_yolov8_raw_1x84xn(
  const float * data, const std::vector<int64_t> & shape,
  float confidence_threshold, std::vector<DetectionCandidate> & candidates,
  std::string & error);

struct YoloDflOutput
{
  const float * data{nullptr};
  std::vector<int64_t> shape;
};

// The K1 320 model: three (DFL logits, class probabilities, score sum) branches.
// Branch strides are 8/16/32; each distance has 16 bins. The class-channel
// count is validated against the configured label file by YoloEngine.
bool decode_yolov8_dfl_9outputs(
  const std::array<YoloDflOutput, 9> & outputs, int input_width, int input_height,
  float confidence_threshold, std::vector<DetectionCandidate> & candidates,
  std::string & error);

// Applies class-aware NMS, restores source-image coordinates, and creates Detection2D entries.
std::vector<vision_msgs::msg::Detection2D> make_detections(
  const std::vector<DetectionCandidate> & candidates, const LetterboxMetadata & letterbox,
  const PostprocessConfig & config);

}  // namespace k1muse_perception
