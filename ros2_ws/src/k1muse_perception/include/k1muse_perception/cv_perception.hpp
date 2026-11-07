#pragma once

#include <string>
#include <vector>

#include "k1muse_perception/qr_reader.hpp"

namespace k1muse_perception
{

struct CvFrameObservation
{
  std::string error;
  std::vector<QrCode> markers;
};

class CvPerception
{
public:
  CvFrameObservation process_rgb8(const cv::Mat & rgb);

private:
  QrReader reader_;
};

}  // namespace k1muse_perception
