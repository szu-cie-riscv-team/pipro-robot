#include "k1muse_perception/yolo_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace k1muse_perception
{
namespace
{

struct SourceBox
{
  float x1;
  float y1;
  float x2;
  float y2;
};

float intersection_over_union(const SourceBox & lhs, const SourceBox & rhs)
{
  const float x1 = std::max(lhs.x1, rhs.x1);
  const float y1 = std::max(lhs.y1, rhs.y1);
  const float x2 = std::min(lhs.x2, rhs.x2);
  const float y2 = std::min(lhs.y2, rhs.y2);
  const float intersection_width = std::max(0.0F, x2 - x1);
  const float intersection_height = std::max(0.0F, y2 - y1);
  const float intersection = intersection_width * intersection_height;
  const float lhs_area = std::max(0.0F, lhs.x2 - lhs.x1) * std::max(0.0F, lhs.y2 - lhs.y1);
  const float rhs_area = std::max(0.0F, rhs.x2 - rhs.x1) * std::max(0.0F, rhs.y2 - rhs.y1);
  const float denominator = lhs_area + rhs_area - intersection;
  return denominator > 0.0F ? intersection / denominator : 0.0F;
}

bool source_box(
  const DetectionCandidate & candidate, const LetterboxMetadata & letterbox,
  SourceBox & box)
{
  if (letterbox.scale <= 0.0F || letterbox.source_width <= 0 || letterbox.source_height <= 0 ||
    !std::isfinite(candidate.center_x) || !std::isfinite(candidate.center_y) ||
    !std::isfinite(candidate.width) || !std::isfinite(candidate.height) ||
    candidate.width <= 0.0F || candidate.height <= 0.0F)
  {
    return false;
  }

  const float x1 = (candidate.center_x - candidate.width * 0.5F -
    static_cast<float>(letterbox.left)) / letterbox.scale;
  const float y1 = (candidate.center_y - candidate.height * 0.5F -
    static_cast<float>(letterbox.top)) / letterbox.scale;
  const float x2 = (candidate.center_x + candidate.width * 0.5F -
    static_cast<float>(letterbox.left)) / letterbox.scale;
  const float y2 = (candidate.center_y + candidate.height * 0.5F -
    static_cast<float>(letterbox.top)) / letterbox.scale;
  box = SourceBox{
    std::clamp(x1, 0.0F, static_cast<float>(letterbox.source_width)),
    std::clamp(y1, 0.0F, static_cast<float>(letterbox.source_height)),
    std::clamp(x2, 0.0F, static_cast<float>(letterbox.source_width)),
    std::clamp(y2, 0.0F, static_cast<float>(letterbox.source_height))};
  return box.x2 > box.x1 && box.y2 > box.y1;
}

}  // namespace

bool decode_yolov8_raw_1x84xn(
  const float * data, const std::vector<int64_t> & shape,
  float confidence_threshold, std::vector<DetectionCandidate> & candidates,
  std::string & error)
{
  candidates.clear();
  if (data == nullptr || shape.size() != 3U || shape[0] != 1 || shape[1] != 84 || shape[2] <= 0) {
    error = "expected YOLOv8 float output shape [1, 84, N]";
    return false;
  }
  if (!std::isfinite(confidence_threshold) || confidence_threshold < 0.0F ||
    confidence_threshold > 1.0F)
  {
    error = "confidence threshold must be in [0, 1]";
    return false;
  }

  const size_t anchors = static_cast<size_t>(shape[2]);
  for (size_t anchor = 0; anchor < anchors; ++anchor) {
    float best_score = -1.0F;
    int best_class = -1;
    for (int class_index = 0; class_index < 80; ++class_index) {
      const float score = data[static_cast<size_t>(4 + class_index) * anchors + anchor];
      if (std::isfinite(score) && score > best_score) {
        best_score = score;
        best_class = class_index;
      }
    }
    if (best_score < confidence_threshold) {
      continue;
    }
    const float width = data[2U * anchors + anchor];
    const float height = data[3U * anchors + anchor];
    if (!std::isfinite(data[anchor]) || !std::isfinite(data[anchors + anchor]) ||
      !std::isfinite(width) || !std::isfinite(height) || width <= 0.0F || height <= 0.0F)
    {
      continue;
    }
    candidates.push_back(
      DetectionCandidate{
        data[anchor], data[anchors + anchor], width, height, best_class, best_score});
  }
  return true;
}

bool decode_yolov8_dfl_9outputs(
  const std::array<YoloDflOutput, 9> & outputs, int input_width, int input_height,
  float confidence_threshold, std::vector<DetectionCandidate> & candidates,
  std::string & error)
{
  candidates.clear();
  if (input_width <= 0 || input_height <= 0 || input_width % 32 != 0 ||
    input_height % 32 != 0 || !std::isfinite(confidence_threshold) ||
    confidence_threshold < 0.0F || confidence_threshold > 1.0F)
  {
    error = "invalid YOLOv8 DFL input dimensions or confidence threshold";
    return false;
  }
  // Validate all branches before reading any tensor or returning partial results.
  const int64_t class_count = outputs[1].shape.size() == 4U ? outputs[1].shape[1] : 0;
  if (class_count <= 0) {
    error = "YOLOv8 DFL class output must have at least one channel";
    return false;
  }
  for (size_t branch = 0; branch < 3U; ++branch) {
    const int stride = 8 << branch;
    for (size_t tensor = 0; tensor < 3U; ++tensor) {
      const int64_t channels = tensor == 0U ? 64 : (tensor == 1U ? class_count : 1);
      const auto & output = outputs[branch * 3U + tensor];
      const std::vector<int64_t> expected{
        1, channels, input_height / stride, input_width / stride};
      if (output.data == nullptr || output.shape != expected) {
        error = "expected nine YOLOv8 DFL NCHW outputs: 64/classes/1 channels at strides 8/16/32";
        return false;
      }
    }
  }

  for (size_t branch = 0; branch < 3U; ++branch) {
    const int stride = 8 << branch;
    const size_t grid_width = static_cast<size_t>(input_width / stride);
    const size_t anchors = grid_width * static_cast<size_t>(input_height / stride);
    const float * boxes = outputs[branch * 3U].data;
    const float * scores = outputs[branch * 3U + 1U].data;
    const float * sums = outputs[branch * 3U + 2U].data;
    for (size_t anchor = 0; anchor < anchors; ++anchor) {
      // The sum is a cheap rejection bound, not an objectness multiplier.
      if (!std::isfinite(sums[anchor]) || sums[anchor] < confidence_threshold) {
        continue;
      }
      float best_score = -1.0F;
      int best_class = -1;
      for (int64_t class_index = 0; class_index < class_count; ++class_index) {
        const float score = scores[static_cast<size_t>(class_index) * anchors + anchor];
        if (std::isfinite(score) && score >= 0.0F && score <= 1.0F && score > best_score) {
          best_score = score;
          best_class = static_cast<int>(class_index);
        }
      }
      if (best_class < 0 || best_score < confidence_threshold) {
        continue;
      }
      std::array<float, 4> distances{};
      bool valid = true;
      for (size_t side = 0; side < 4U && valid; ++side) {
        std::array<float, 16> logits{};
        for (size_t bin = 0; bin < 16U; ++bin) {
          logits[bin] = boxes[(side * 16U + bin) * anchors + anchor];
          valid = valid && std::isfinite(logits[bin]);
        }
        if (!valid) {
          break;
        }
        const float maximum = *std::max_element(logits.begin(), logits.end());
        float sum = 0.0F;
        float weighted_sum = 0.0F;
        for (size_t bin = 0; bin < 16U; ++bin) {
          // Stable softmax prevents overflow without altering the DFL expectation.
          const float probability = std::exp(logits[bin] - maximum);
          sum += probability;
          weighted_sum += probability * static_cast<float>(bin);
        }
        distances[side] = weighted_sum / sum;
      }
      if (!valid) {
        continue;
      }
      const float anchor_x = static_cast<float>(anchor % grid_width) + 0.5F;
      const float anchor_y = static_cast<float>(anchor / grid_width) + 0.5F;
      const float x1 = (anchor_x - distances[0]) * stride;
      const float y1 = (anchor_y - distances[1]) * stride;
      const float x2 = (anchor_x + distances[2]) * stride;
      const float y2 = (anchor_y + distances[3]) * stride;
      candidates.push_back(DetectionCandidate{
        (x1 + x2) * 0.5F, (y1 + y2) * 0.5F, x2 - x1, y2 - y1,
        best_class, best_score});
    }
  }
  return true;
}

std::vector<vision_msgs::msg::Detection2D> make_detections(
  const std::vector<DetectionCandidate> & candidates, const LetterboxMetadata & letterbox,
  const PostprocessConfig & config)
{
  std::vector<std::pair<DetectionCandidate, SourceBox>> usable;
  usable.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    SourceBox box{};
    if (candidate.score >= config.confidence_threshold && candidate.class_index >= 0 &&
      candidate.class_index < static_cast<int>(config.labels.size()) &&
      source_box(candidate, letterbox, box))
    {
      usable.emplace_back(candidate, box);
    }
  }
  std::sort(
    usable.begin(), usable.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first.score > rhs.first.score;
    });

  std::vector<bool> suppressed(usable.size(), false);
  std::vector<vision_msgs::msg::Detection2D> detections;
  detections.reserve(
    std::min(
      static_cast<size_t>(std::max(config.max_detections, 0)),
      usable.size()));
  for (size_t index = 0; index < usable.size() &&
    static_cast<int>(detections.size()) < config.max_detections; ++index)
  {
    if (suppressed[index]) {
      continue;
    }
    const auto & candidate = usable[index].first;
    const auto & box = usable[index].second;
    vision_msgs::msg::Detection2D detection;
    detection.bbox.center.position.x = (box.x1 + box.x2) * 0.5F;
    detection.bbox.center.position.y = (box.y1 + box.y2) * 0.5F;
    detection.bbox.center.theta = 0.0;
    detection.bbox.size_x = box.x2 - box.x1;
    detection.bbox.size_y = box.y2 - box.y1;
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = config.labels[static_cast<size_t>(candidate.class_index)];
    hypothesis.hypothesis.score = candidate.score;
    // Detection2D cannot omit this field. Its identity quaternion is only a valid message default,
    // not a claim that this 2D detector knows an object pose.
    hypothesis.pose.pose.orientation.w = 1.0;
    detection.results.push_back(std::move(hypothesis));
    detections.push_back(std::move(detection));

    for (size_t later = index + 1; later < usable.size(); ++later) {
      if (!suppressed[later] && usable[later].first.class_index == candidate.class_index &&
        intersection_over_union(box, usable[later].second) > config.nms_iou_threshold)
      {
        suppressed[later] = true;
      }
    }
  }
  return detections;
}

}  // namespace k1muse_perception
