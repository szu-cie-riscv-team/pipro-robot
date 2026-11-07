#include "k1muse_perception/qr_reader.hpp"
#include <algorithm>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace k1muse_perception
{
QrReader::QrReader()
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
: detector_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50))
#else
: dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)),
  parameters_(cv::aruco::DetectorParameters::create())
#endif
{}
std::vector<QrCode> QrReader::read_rgb8(const cv::Mat & rgb)
{
  if (rgb.empty() || rgb.type() != CV_8UC3) {
    throw std::invalid_argument("marker reader requires a nonempty RGB8 image");
  }
  cv::cvtColor(rgb, gray_, cv::COLOR_RGB2GRAY);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  detector_.detectMarkers(gray_, corners, ids);
#else
  cv::aruco::detectMarkers(gray_, dictionary_, corners, ids, parameters_);
#endif
  if (ids.size() != corners.size()) {throw std::runtime_error("invalid marker result");}
  std::vector<QrCode> result;
  result.reserve(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (corners[i].size() != 4) {throw std::runtime_error("invalid marker corners");}
    QrCode marker; marker.id = ids[i];
    std::copy(corners[i].begin(), corners[i].end(), marker.corners.begin());
    result.push_back(marker);
  }
  return result;
}
}  // namespace k1muse_perception
