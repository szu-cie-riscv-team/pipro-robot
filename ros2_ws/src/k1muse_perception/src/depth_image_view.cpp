#include "k1muse_perception/depth_image_view.hpp"

#include <cstring>
#include <limits>

namespace k1muse_perception
{

bool make_depth_view(
  const sensor_msgs::msg::Image & image, DepthImageView & view, std::string & error)
{
  view = {};
  const bool floating = image.encoding == "32FC1";
  if (!floating && image.encoding != "16UC1") {
    error = "invalid_encoding";
    return false;
  }
  const size_t bytes = floating ? 4 : 2;
  const uint64_t row_bytes = static_cast<uint64_t>(image.width) * bytes;
  const uint64_t required = static_cast<uint64_t>(image.step) * image.height;
  if (!image.width || !image.height || image.width > std::numeric_limits<int>::max() ||
    image.height > std::numeric_limits<int>::max() || image.step < row_bytes ||
    required > image.data.size())
  {
    error = "invalid_layout";
    return false;
  }
  const uint16_t endian_probe = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  view = {&image, floating, static_cast<bool>(image.is_bigendian) != host_big_endian};
  return true;
}

double DepthImageView::at(uint32_t u, uint32_t v) const
{
  const size_t bytes = floating ? 4 : 2;
  const auto * source = image->data.data() + static_cast<size_t>(v) * image->step + u * bytes;
  uint8_t storage[4];
  if (swap_bytes) {
    for (size_t i = 0; i < bytes; ++i) {
      storage[i] = source[bytes - 1 - i];
    }
    source = storage;
  }
  if (floating) {
    float value;
    std::memcpy(&value, source, sizeof(value));
    return value;
  }
  uint16_t value;
  std::memcpy(&value, source, sizeof(value));
  return value * 0.001;
}

}  // namespace k1muse_perception
