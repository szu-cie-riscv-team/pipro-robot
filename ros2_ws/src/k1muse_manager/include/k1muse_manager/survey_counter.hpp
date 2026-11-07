#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace k1muse_manager
{
// Measured box crossings in a fixed, one-dimensional shelf reference.
// No ROS, marker trajectories, image storage or second tracker.
class ShelfCounter
{
public:
  struct Box {std::string id; double distance{0};};
  struct Result {bool complete{false}; int quantity{0}; std::string reason;};
  void reset(unsigned confirmations, double deadband, bool left_to_right=true);
  void set_boundary(bool left, double position);
  void observe_boxes(int64_t ns, double camera_position, const std::vector<Box> & boxes);
  int provisional_count() const {return provisional_;}
  Result finalize() const;

private:
  struct Sample {int64_t ns{0}; double distance{0}, position{0};};
  struct Track
  {
    bool initialized{false}, eligible{false}, counted{false}, pending{false};
    uint64_t last_seq{0};
    unsigned confirmations{0};
    Sample positive;
    double crossing{0};
  };
  void observe(Track &, Sample, bool allow_new);
  bool inside(double) const;
  void recount();
  unsigned confirmations_{2};
  double deadband_{2}, direction_{1};
  uint64_t seq_{0};
  int64_t last_frame_{0};
  std::optional<double> left_, right_, frontier_;
  std::unordered_map<std::string, Track> tracks_;
  std::vector<std::string> pending_ids_;
  std::vector<double> lost_crossings_;
  int provisional_{0};
};
}  // namespace k1muse_manager
