#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace k1muse_web {
// Steady clock seconds supplied by the node; also usable without ROS in tests.
struct ManualInput {
  bool enabled{false};
  uint32_t sequence{0};
  double received{0}, x{0}, y{0}, yaw{0};
  void disable() { enabled = false; x = y = yaw = 0; }
  void enable(double now) { disable(); enabled = true; received = now; }
  bool accept(uint32_t seq, double vx, double vy, double wz, double now) {
    if (!enabled || seq <= sequence ||
        !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) return false;
    sequence = seq; received = now;
    const double length = std::hypot(vx, vy);
    const double scale = length > .20 ? .20 / length : 1;
    x = vx * scale; y = vy * scale; yaw = std::clamp(wz, -.60, .60);
    return true;
  }
  bool expired(double now) const { return enabled && now - received >= .200; }
};
}  // namespace k1muse_web
