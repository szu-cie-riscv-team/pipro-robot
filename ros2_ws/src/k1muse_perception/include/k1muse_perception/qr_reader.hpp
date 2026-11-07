#pragma once

#include <array>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#include <opencv2/aruco.hpp>
#endif

namespace k1muse_perception
{
struct QrCode
{
  int id{-1};
  std::array<cv::Point2f, 4> corners{};
};

class QrReader
{
public:
  QrReader();
  std::vector<QrCode> read_rgb8(const cv::Mat & rgb);
private:
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::ArucoDetector detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
#endif
  cv::Mat gray_;
};
}  // namespace k1muse_perception
