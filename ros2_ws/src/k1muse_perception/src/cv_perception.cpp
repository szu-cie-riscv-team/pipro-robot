#include "k1muse_perception/cv_perception.hpp"
#include <exception>

namespace k1muse_perception
{
CvFrameObservation CvPerception::process_rgb8(const cv::Mat & rgb)
{
  CvFrameObservation result;
  try {result.markers = reader_.read_rgb8(rgb);}
  catch (const std::exception &) {result.error = "image_processing_error";}
  return result;
}
}  // namespace k1muse_perception
