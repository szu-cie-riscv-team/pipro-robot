#include "k1muse_voice/voice_input_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace k1muse_voice
{

UtteranceBuffer::UtteranceBuffer(
  const std::size_t frame_samples, const std::size_t pre_roll_frames,
  const std::size_t max_utterance_samples)
: frame_samples_(frame_samples),
  pre_roll_samples_(frame_samples * pre_roll_frames),
  max_utterance_samples_(max_utterance_samples)
{
  if (frame_samples_ == 0U || pre_roll_frames == 0U ||
    max_utterance_samples_ < frame_samples_)
  {
    throw std::invalid_argument("invalid utterance buffer limits");
  }
  utterance_.reserve(max_utterance_samples_);
}

void UtteranceBuffer::retain_pre_roll(const std::vector<int16_t> & frame)
{
  pre_roll_.insert(pre_roll_.end(), frame.begin(), frame.end());
  while (pre_roll_.size() > pre_roll_samples_) {
    pre_roll_.pop_front();
  }
}

SegmentUpdate UtteranceBuffer::push(
  const std::vector<int16_t> & frame, const bool speech_start,
  const bool speech_active, const bool speech_end)
{
  if (frame.size() != frame_samples_) {
    throw std::invalid_argument("PCM frame size does not match the VAD window");
  }

  retain_pre_roll(frame);
  const bool starts_now = !collecting_ && (speech_start || speech_active);
  if (starts_now) {
    collecting_ = true;
    utterance_.assign(pre_roll_.begin(), pre_roll_.end());
  } else if (collecting_) {
    utterance_.insert(utterance_.end(), frame.begin(), frame.end());
  }

  if (!collecting_) {
    return {};
  }
  if (utterance_.size() > max_utterance_samples_) {
    collecting_ = false;
    pre_roll_.clear();
    utterance_.clear();
    return {SegmentEvent::kDroppedTooLong, {}};
  }
  if (!speech_end) {
    return {};
  }

  collecting_ = false;
  pre_roll_.clear();
  SegmentUpdate update;
  update.event = SegmentEvent::kReady;
  update.samples = std::move(utterance_);
  utterance_.clear();
  utterance_.reserve(max_utterance_samples_);
  return update;
}

void UtteranceBuffer::reset()
{
  collecting_ = false;
  pre_roll_.clear();
  utterance_.clear();
}

std::string clean_sensevoice_text(const std::string & text)
{
  std::string without_tags;
  without_tags.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ) {
    if (i + 1U < text.size() && text[i] == '<' && text[i + 1U] == '|') {
      const auto end = text.find("|>", i + 2U);
      if (end != std::string::npos) {
        i = end + 2U;
        continue;
      }
    }
    without_tags.push_back(text[i]);
    ++i;
  }

  std::string cleaned;
  cleaned.reserve(without_tags.size());
  bool in_space = true;
  for (const unsigned char value : without_tags) {
    if (std::isspace(value) != 0) {
      if (!in_space && !cleaned.empty()) {
        cleaned.push_back(' ');
      }
      in_space = true;
    } else {
      cleaned.push_back(static_cast<char>(value));
      in_space = false;
    }
  }
  if (!cleaned.empty() && cleaned.back() == ' ') {
    cleaned.pop_back();
  }
  return cleaned;
}

bool valid_vad_thresholds(const double trigger_threshold, const double stop_threshold)
{
  return std::isfinite(trigger_threshold) && std::isfinite(stop_threshold) &&
         stop_threshold > 0.0 && stop_threshold < trigger_threshold &&
         trigger_threshold <= 1.0;
}

}  // namespace k1muse_voice
