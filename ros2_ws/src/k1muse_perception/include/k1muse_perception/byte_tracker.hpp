#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <vision_msgs/msg/detection2_d_array.hpp>

namespace k1muse_perception
{

struct ByteTrackerConfig
{
  float track_threshold{0.4F};
  float new_track_threshold{0.4F};
  float match_threshold{0.8F};
  int lost_buffer_frames{30};
  size_t capacity{100};
};

// Thin ROS adapter around the pinned upstream C++ ByteTrack implementation.
// Output contains only confirmed tracks with a measurement in this source frame.
class ByteTracker
{
public:
  ByteTracker();
  ~ByteTracker();
  ByteTracker(const ByteTracker &) = delete;
  ByteTracker & operator=(const ByteTracker &) = delete;

  bool configure(const ByteTrackerConfig & config, std::string & error);
  void clear();
  void update(
    vision_msgs::msg::Detection2DArray & detections, uint32_t width, uint32_t height);
  size_t track_count() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace k1muse_perception
