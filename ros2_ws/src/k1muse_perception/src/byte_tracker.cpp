#include "k1muse_perception/byte_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include "k1muse_perception/bytetrack/BYTETracker.hpp"

namespace k1muse_perception
{

class ByteTracker::Impl
{
public:
  struct ClassTracks
  {
    std::unique_ptr<bytetrack::BYTETracker> tracker;
    std::vector<bytetrack::Object> objects;
  };
  ByteTrackerConfig config;
  bool configured{false}, have_frame{false};
  uint32_t width{0}, height{0};
  int frame_index{0};
  int64_t stamp{0};
  std::string frame_id;
  std::map<std::string, ClassTracks> classes;
  std::vector<int> matched_ids;

  size_t count() const
  {
    size_t size = 0;
    for (const auto & entry : classes) {size += entry.second.tracker->track_count();}
    return size;
  }

  void limit_lost_tracks()
  {
    // Current tracks <= current detections <= capacity. Only unmatched Lost
    // tracks need eviction; never displace a matched current measurement.
    size_t size = count();
    while (size > config.capacity) {
      std::vector<bytetrack::STrack> * oldest = nullptr;
      size_t index = 0;
      int age = -1, id = 0;
      for (auto & entry : classes) {
        auto & core = *entry.second.tracker;
        auto & lost = core.lost_tracks();
        for (size_t i = 0; i < lost.size(); ++i) {
          const int candidate_age = core.current_frame() - lost[i].end_frame();
          if (candidate_age > age || (candidate_age == age && lost[i].track_id < id)) {
            oldest = &lost; index = i; age = candidate_age; id = lost[i].track_id;
          }
        }
      }
      if (!oldest) {throw std::logic_error("ByteTrack capacity invariant violated");}
      oldest->erase(oldest->begin() + index);
      --size;
    }
  }
};

ByteTracker::ByteTracker() : impl_(std::make_unique<Impl>()) {}
ByteTracker::~ByteTracker() = default;

bool ByteTracker::configure(const ByteTrackerConfig & config, std::string & error)
{
  const auto probability = [](float value) {
      return std::isfinite(value) && value > 0 && value <= 1;
    };
  if (!probability(config.track_threshold) || !probability(config.new_track_threshold) ||
    config.new_track_threshold < config.track_threshold || !probability(config.match_threshold) ||
    config.lost_buffer_frames < 10 || config.capacity == 0)
  {
    error = "ByteTrack requires ordered score thresholds in (0,1], a match threshold in (0,1], "
      "at least 10 lost-buffer frames and positive capacity";
    return false;
  }
  clear();
  impl_->config = config;
  impl_->configured = true;
  return true;
}

void ByteTracker::clear()
{
  impl_->classes.clear();
  std::vector<int>().swap(impl_->matched_ids);
  impl_->frame_index = 0;
  impl_->have_frame = false;
}

size_t ByteTracker::track_count() const {return impl_->count();}

void ByteTracker::update(
  vision_msgs::msg::Detection2DArray & message, uint32_t width, uint32_t height)
{
  auto & state = *impl_;
  if (!state.configured || message.detections.size() > state.config.capacity) {
    throw std::logic_error("ByteTrack is unconfigured or input exceeds C08 max_detections");
  }
  const int64_t ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
    message.header.stamp.nanosec;
  if (state.have_frame && (state.frame_id != message.header.frame_id ||
    state.width != width || state.height != height))
  {
    clear();
  }
  if (state.have_frame && ns <= state.stamp) {
    message.detections.clear();  // No re-prediction or new IDs for repeated/late source images.
    return;
  }
  ++state.frame_index;
  for (auto & entry : state.classes) {entry.second.objects.clear();}
  state.matched_ids.assign(message.detections.size(), 0);
  for (size_t i = 0; i < message.detections.size(); ++i) {
    const auto & detection = message.detections[i];
    const auto & hypothesis = detection.results.front().hypothesis;
    auto & group = state.classes[hypothesis.class_id];
    if (!group.tracker) {
      const auto & c = state.config;
      group.tracker = std::make_unique<bytetrack::BYTETracker>(
        c.lost_buffer_frames, c.track_threshold, c.new_track_threshold,
        c.match_threshold, state.frame_index - 1);
    }
    const auto & box = detection.bbox;
    bytetrack::Object object;
    object.rect = cv::Rect_<float>(
      box.center.position.x - box.size_x * 0.5, box.center.position.y - box.size_y * 0.5,
      box.size_x, box.size_y);
    object.label = 0;  // One upstream tracker per label prevents cross-class association.
    object.prob = hypothesis.score;
    object.detection_index = static_cast<int>(i);
    group.objects.push_back(object);
  }
  for (auto & entry : state.classes) {
    for (const auto & track : entry.second.tracker->update(entry.second.objects)) {
      if (track.frame_id == state.frame_index && track.detection_index >= 0) {
        state.matched_ids.at(track.detection_index) = track.track_id;
      }
    }
  }
  state.limit_lost_tracks();
  for (auto it = state.classes.begin(); it != state.classes.end();) {
    if (it->second.tracker->track_count() == 0) {it = state.classes.erase(it);}
    else {++it;}
  }
  size_t kept = 0;
  for (size_t i = 0; i < message.detections.size(); ++i) {
    if (state.matched_ids[i] == 0) {continue;}
    auto & detection = message.detections[i];
    detection.id = std::to_string(state.matched_ids[i]);
    if (kept != i) {message.detections[kept] = std::move(detection);}
    ++kept;
  }
  message.detections.resize(kept);
  state.width = width; state.height = height;
  state.frame_id = message.header.frame_id; state.stamp = ns; state.have_frame = true;
}

}  // namespace k1muse_perception
