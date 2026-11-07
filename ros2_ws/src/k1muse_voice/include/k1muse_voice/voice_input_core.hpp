#ifndef K1MUSE_VOICE__VOICE_INPUT_CORE_HPP_
#define K1MUSE_VOICE__VOICE_INPUT_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace k1muse_voice
{

enum class SegmentEvent
{
  kNone,
  kReady,
  kDroppedTooLong,
};

struct SegmentUpdate
{
  SegmentEvent event{SegmentEvent::kNone};
  std::vector<int16_t> samples;
};

class UtteranceBuffer
{
public:
  UtteranceBuffer(
    std::size_t frame_samples, std::size_t pre_roll_frames,
    std::size_t max_utterance_samples);

  SegmentUpdate push(
    const std::vector<int16_t> & frame, bool speech_start,
    bool speech_active, bool speech_end);
  void reset();

private:
  void retain_pre_roll(const std::vector<int16_t> & frame);

  std::size_t frame_samples_;
  std::size_t pre_roll_samples_;
  std::size_t max_utterance_samples_;
  bool collecting_{false};
  std::deque<int16_t> pre_roll_;
  std::vector<int16_t> utterance_;
};

std::string clean_sensevoice_text(const std::string & text);
bool valid_vad_thresholds(double trigger_threshold, double stop_threshold);

}  // namespace k1muse_voice

#endif  // K1MUSE_VOICE__VOICE_INPUT_CORE_HPP_
